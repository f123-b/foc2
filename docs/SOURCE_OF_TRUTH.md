# Source of Truth

本文件记录的是当前工作区快照实际包含的运行边界，而不是目标设计。

## 运行时归属

| 区域 | 状态 | 责任 |
|---|---|---|
| `odrive-baseline/Firmware` | STM32 实际固件 | ODrive v0.5.1、板级 HAL、FreeRTOS、USB CDC、Axis、Encoder、Controller、Motor 与 PWM；`tools/build-firmware.ps1` 以此目录构建。 |
| `firmware/` | portable snapshot | 小型 CMake 可测核心及一份 ODrive vendor 快照；它不是完整可烧录固件，也不会被 `build-firmware.ps1` 链接进 ELF。 |
| `host/foc-studio/` | Windows host | Electron/Web Serial UI、协议解析、模拟设备和主机协议测试。 |
| `protocol/` | 接口契约 | ASCII 命令与遥测格式；固件实现位于 `odrive-baseline/Firmware/communication/ascii_protocol.cpp`。 |
| `tools/` | 构建/操作脚本 | 本地 toolchain、Tup、测试、桌面构建和 DFU 脚本。 |
| `odrive-baseline/Firmware/Tests` | 固件侧单元测试 | doctest 测试；目前未由根目录 `test-project.ps1` 构建或执行。 |

## 配置来源与漂移

首次启动、NVM 无效或擦除配置后，`odrive-baseline/Firmware/MotorControl/main.cpp::load_foc_studio_defaults()` 是实际 STM32 默认值来源。有效运行配置随后可由 NVM 和 ASCII/Fibre property 写入覆盖。

`firmware/include/foc_config.hpp` 复制了极对数、电阻、电感、KV、力矩常数、带宽和 CPR，其中 `max_current_a = 17 A` 又不同于实际 bring-up 默认值 `2 A`。因为该文件不参与烧录固件，此类重复不会改变板上行为，却会误导测试、文档和以后移植，属于配置漂移风险。

建议的低风险统一方案：

1. 以 `main.cpp` 的产品默认值和 NVM 为唯一运行时事实来源；
2. 将 portable core 的常量改为从一个仅含产品标称值的共享、生成或导出文件读取，不能再另行声明可运行上限；
3. 在构建时增加比较测试，验证极对数、电阻、电感、KV/力矩常数、当前 bring-up limit、带宽及 SPI/ABZ CPR 一致；
4. 不在本阶段把运行时 ABZ 调参直接塞入 Fibre `Config_t`，避免接口生成和 NVM 兼容性的大范围变更。

## 版本与可追溯性限制

当前提供的工作区没有 `.git` 目录。因此无法确认目标 baseline `2f979e72bc5dd4ab196c91ddfa02306a167307a2`、无法比较 `49bea5b..2f979e7`、不能创建要求的分支，也不能生成本阶段 commit SHA。后续工作必须在包含完整 Git 历史的克隆中重新执行基线步骤后才可合并或回退。
