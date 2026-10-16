# `_obsolete/` —— 改名搬迁后留下的旧副本

这里放的是**不参与编译**的历史文件，只是从 include 路径里挪出来，**没有删除**。

## 为什么挪

`UserCode/Inc/pick.h` 与 `HARDWARE/Inc/pick.h` **重名，且 include guard 都是 `__PICK_H`**：

| 文件 | 内容 |
|---|---|
| `HARDWARE/Inc/pick.h`（现行） | `Pick_JumpTo()` 函数 + `POS_*` 括号宏 + `Pick_TurnTable_Set` + `Pick_Gripper_Observe`；`HARDWARE/Src/pick.c` 实现它，CMakeLists 编的也是它 |
| `UserCode/Inc/pick.h`（本目录，旧） | `Pick_MoveTo()` + 另一套 `POS_A/B/C` 宏 |

C 的 `#include "xxx.h"` 会**先搜包含它的那个文件所在目录**，而 `task.h`、`pick_task.h` 都在
`UserCode/Inc/` —— 所以它们拿到的是旧副本，导致 `Pick_JumpTo` / `Pick_Color1` 等全部"未声明"，
全量重建失败。`-I` 的顺序在这个场景下不起作用（已实测：`build/Debug` 全量重建报
`error: implicit declaration of function 'Pick_JumpTo'`）。

把旧文件移出 include 路径后，`pick.h` 只剩 `HARDWARE/Inc` 一份，编译立即恢复。
详见 `改动记录.md` 第 49 节。

## 这里现在有什么

- `pick.h` —— 旧版机械臂头文件（`Pick_MoveTo` 一代）

## 已清掉的旧副本（2026-10-16）

下面这些同样是改名/搬迁后遗留的旧副本，**都不在 `CMakeLists.txt` 里、不参与编译**，
留着会误导 grep 与"按文件名找实现"。2026-10-16 已**直接删除**（没搬进本目录）：

- `UserCode/Inc/pick_ctrl.h`、`UserCode/Src/pick_ctrl.c`（`pick_ctrl` → `pick` 那一代）
- `UserCode/Src/openmv.c`、`UserCode/Inc/openmv.h`（旧 ASCII 协议时代，已被 `camera.*` 取代；
  注意 `openmv.c` 里还在调 `HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ...)`，grep 时会撞车）

要取回就在删除前的那个 commit 上 `git show <commit>:<路径>`。

`UserCode/Src/pick.c`（旧实现，现行实现在 `HARDWARE/Src/pick.c`）**还留着没清** ——
它的头文件 `UserCode/Inc/pick.h` 已经搬走了，所以它单独放着连编译都过不了；
好在没有任何编译单元 include 它，暂时无害。

## ⚠️ `pick_task.*` **不是**旧副本，别跟着一起清

`UserCode/Src/pick_task.c` / `UserCode/Inc/pick_task.h` 是**现行代码**：
`task.h`、`task.c`、`test.c` 都 include 它，`Pick_RAW` / `Pick_ROUGH` / `Place_ROUGH` /
`Place_TEMP` 只在它里面声明，`CMakeLists.txt` 也编它 —— 删了会连带任务调度整块编译不过。

2026-10-16 搬目录（`D:\Projects\stm32` → `D:\Projects\github`）时它俩被误删过一次，
`build/Debug` 配置直接卡在 `Cannot find source file: UserCode/Src/pick_task.c`。
以后按上面那份清单清理废弃副本时，认准清单，别按"看着像旧名字"下手。
