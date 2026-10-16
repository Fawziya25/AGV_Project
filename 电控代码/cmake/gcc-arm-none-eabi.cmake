set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- 默认按"在 PATH 上"找；PATH 上找不到就去 STM32Cube 的 bundle 里
# 捞一个绝对路径（见下面 TOOLCHAIN_PREFIX 的三种取法）。
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

# 找工具链目录：PATH → $CUBE_BUNDLE_PATH → %LOCALAPPDATA%\stm32cube\bundles。
# 为什么要费这个劲：裸名字 arm-none-eabi-gcc 要求 PATH 上有工具链，而**只有
# STM32 扩展**会临时把它塞进 PATH。别的 CMake 前端（比如 VS Code 的 CMake Tools
# 用 VS BuildTools 自带的 cmake）来 configure 时 PATH 上没有，就会以
# "arm-none-eabi-gcc is not a full path and was not found in the PATH" 直接失败，
# 顺便把 STM32 扩展正在用的 CMakeCache.txt 覆盖掉。2026-10-16 踩过一次。
# 取到绝对路径后，谁跑 configure 都不挑 PATH 了。
find_program(ARM_NONE_EABI_GCC NAMES arm-none-eabi-gcc)

if(NOT ARM_NONE_EABI_GCC)
    file(GLOB _arm_gcc_candidates
         "$ENV{CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc.exe"
         "$ENV{LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc.exe")
    if(_arm_gcc_candidates)
        list(GET _arm_gcc_candidates 0 ARM_NONE_EABI_GCC)
    endif()
endif()

set(TOOLCHAIN_SUFFIX                "")   # 可执行后缀：Windows 上从找到的文件名里取，见下

if(ARM_NONE_EABI_GCC AND IS_ABSOLUTE "${ARM_NONE_EABI_GCC}")
    get_filename_component(TOOLCHAIN_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)
    get_filename_component(_gcc_name         "${ARM_NONE_EABI_GCC}" NAME)
    # 把 arm-none-eabi-gcc.exe 里的 "arm-none-eabi-gcc" 摘掉，剩下的就是 .exe（Linux 上是空串）。
    # 必须有这个后缀：CMake 拿到**绝对路径**时不会像 find_program 那样自动补 .exe，
    # 写成 .../arm-none-eabi-gcc 会报 "is not a full path to an existing compiler tool"。
    string(REGEX REPLACE "^arm-none-eabi-gcc" "" TOOLCHAIN_SUFFIX "${_gcc_name}")
    set(TOOLCHAIN_PREFIX "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-")
endif()
# 还是没找到就保持裸名字 + 空后缀 —— 行为和改之前一样，让 CMake 自己报找不到。

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX})
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
