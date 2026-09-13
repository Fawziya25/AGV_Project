# AGV_Project — 项目长期约定

## 工程概况
- STM32F407VET6 / Cortex-M4 / GCC 工具链，CMake + Ninja 构建，STM32CubeMX 生成。
- 构建目录 `build/Debug`，产物 `STM32F407_CAN_CMD.elf`。
- 调试：无线 DAPLink（Horco CMSIS-DAP v2）+ OpenOCD；链路不稳，用 `.vscode/flash-retry.sh` 重试烧录。
- `.settings/` 是 STM32Cube 工程元信息（器件/bundle 版本），**故意提交**，当作工具链版本锁。

## 铁律：仓库里不写机器绝对路径
本项目曾因在别人账号（`Dawn`）下开发、配置里留 `C:/Users/Dawn/...`，
换机后 cortex-debug 直接报 `GDB executable was not found`。约定：

- 工具链一律走变量，绝不写 `C:/Users/xxx`：
  - `${env:CUBE_BUNDLE_PATH}/<bundle>/<ver>/bin` —— bundle 根（cmake/ninja/gnu-tools/gnu-gdb…）
  - `${env:LOCALAPPDATA}/stm32cube/packs/...` —— SVD 等 pack 内容（**不在** bundles 下）
  - `${env:LOCALAPPDATA}/xpack-openocd/...` —— openocd
  - `${workspaceFolder}/...` —— 工程内文件
- 机器私有东西（个人权限白名单等）走 `*.local.json` + `.gitignore`。
- `githooks/pre-commit` 会在 commit 阶段拦截硬编码用户主目录路径；
  新 clone 后需执行一次：`git config core.hooksPath githooks`。

## 本机工具链位置（仅供参考，配置里不要写死）
- bundles 根 = `C:\Users\Fawziya\AppData\Local\stm32cube\bundles`（= `CUBE_BUNDLE_PATH`）
- gcc/objdump: `.../gnu-tools-for-stm32/14.3.1+st.2/bin`
- gdb:        `.../gnu-gdb-for-stm32/14.3.1+st.2/bin`
- cmake:      `.../cmake/4.3.1+st.1/bin`；ninja: `.../ninja/1.13.2+st.1/bin`
- openocd:    `C:\Users\Fawziya\AppData\Local\xpack-openocd\xpack-openocd-0.12.0-7\bin`
