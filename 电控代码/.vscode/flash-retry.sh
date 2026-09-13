#!/usr/bin/env bash
# 无线 DAPLink 烧录重试脚本
#
# 背景：这台无线 DAPLink（Horco CMSIS-DAP v2 / FW v0.2）链路不稳定，
# 单次烧录用会间歇性失败，典型报错：
#   Error: CMSIS-DAP command CMD_DAP_SWJ_CLOCK failed.
#   Error: Error connecting DP: cannot read IDR
#   Error: Failed to write memory at 0x........   (地址随机，跨 SRAM/外设/PPB)
# 失败点随机 = 链路层丢包，不是配置或固件问题。链路有好窗口和坏窗口，
# 盯着重试通常几次就能抓到好窗口。
#
# 关键经验（2026-09-12 实测）：
#   * 失败率只跟「事务数量」相关，跟 SWCLK 频率无关 —— 读 64KB 在
#     50/100/200/500/2000 kHz 下 0/15 全失败，而短操作稳定成功。
#     所以调 speed 是在浪费生命，别试。
#   * 也排除过供电问题：64KB 读几乎不耗电，照样全挂。
#   * 因此这里用 `halt` 而**不是** `reset halt` —— reset 本身约 50% 失败
#     （Error connecting DP: cannot read IDR），不带 reset 的序列成功率高得多。
#
# 用法：
#   bash .vscode/flash-retry.sh                 用默认 ELF
#   bash .vscode/flash-retry.sh path/to/xx.elf  指定 ELF
#   ATTEMPTS=50 bash .vscode/flash-retry.sh     改重试次数（默认 30）

set -u

# OpenOCD 位置不写死：默认从 LOCALAPPDATA 推导，换机器 / 升级 xpack 版本时用环境变量覆盖
#   OPENOCD_ROOT="C:/path/to/xpack-openocd-x.y.z" bash .vscode/flash-retry.sh
LA_DIR="$(printf '%s' "${LOCALAPPDATA:-$HOME/AppData/Local}" | sed -E 's#\\#/#g')"
OPENOCD_ROOT="${OPENOCD_ROOT:-$LA_DIR/xpack-openocd/xpack-openocd-0.12.0-7}"
OPENOCD="$OPENOCD_ROOT/bin/openocd.exe"
SCRIPTS="$OPENOCD_ROOT/openocd/scripts"
ATTEMPTS="${ATTEMPTS:-30}"

if [ ! -f "$OPENOCD" ]; then
    echo "找不到 OpenOCD: $OPENOCD" >&2
    echo "用 OPENOCD_ROOT 指定实际安装目录后重试，例如：" >&2
    echo "  OPENOCD_ROOT=\"\$LOCALAPPDATA/xpack-openocd/xpack-openocd-0.12.0-7\" bash .vscode/flash-retry.sh" >&2
    exit 2
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELF="${1:-$HERE/../build/Debug/STM32F407_CAN_CMD.elf}"

if [ ! -f "$ELF" ]; then
    echo "找不到 ELF: $ELF" >&2
    exit 2
fi

# OpenOCD 是原生 Windows 程序，喂给它 Windows 风格路径（d:/... 而不是 /d/...）
if command -v cygpath >/dev/null 2>&1; then
    ELF="$(cygpath -m "$ELF")"
else
    ELF="$(printf '%s' "$ELF" | sed -E 's#^/([a-zA-Z])/#\1:/#')"
fi

echo "ELF     : $ELF"
echo "重试上限: $ATTEMPTS"
echo

for i in $(seq 1 "$ATTEMPTS"); do
    printf '第 %2d 次: ' "$i"
    OUT=$("$OPENOCD" -s "$SCRIPTS" \
        -f interface/cmsis-dap.cfg \
        -f target/stm32f4x.cfg \
        -c "transport select swd" \
        -c "init" \
        -c "halt" \
        -c "flash write_image erase $ELF" \
        -c "verify_image $ELF" \
        -c "reset run" \
        -c "exit" 2>&1)
    # 用明确标志判定，不要用"有没有 Error 行"（2026-09-12 踩过这个坑）
    if echo "$OUT" | grep -qE "verified [0-9]+ bytes"; then
        echo "$(echo "$OUT" | grep -E 'wrote|verified' | tr '\n' ' ')"
        echo ">>> 烧录成功（第 $i 次）"
        exit 0
    fi
    echo "$(echo "$OUT" | grep -E 'Error' | head -1)"
    sleep 3
done

echo ">>> $ATTEMPTS 次全部失败"
echo "    把电脑端 dongle 拔插一次 + 单片机板子断电重启，然后再跑一次"
exit 1
