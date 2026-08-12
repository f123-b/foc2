# 已知问题

优先级基于源码审查；“已证明”仅代表代码路径可证，不代表已在所有硬件上复现。

## P0

本快照未发现可由静态代码直接证明的 P0（立即造成不可控 PWM 的缺陷）。由于完整 ARM 构建和 HIL 均不可执行，此结论不是安全认证。

## P1

| 问题 | 文件 / 函数 | 原因与风险 | 建议 |
|---|---|---|---|
| 遥测续命 watchdog | `communication/ascii_protocol.cpp` `g`/`j` branches | 已证明：两条只读遥测路径调用 `Axis::watchdog_feed()`。持续 telemetry 可维持最后一个运动命令，运动命令生命周期不再等同于控制更新。 | 单独 commit 分离 control watchdog 与 telemetry；以 `u` 和显式控制命令兼容迁移，并加 timeout HIL test。 |
| 无法复现 ARM 基线 | `tools/test-project.ps1`, `tools/build-firmware.ps1` | 已证明：ARM/Tup 工具链不在工作区。根测试脚本原先还依赖未构建的 exe，本阶段已改为自行 CMake 构建。没有 ELF/HEX/BIN 和 hash，无法判断当前快照是否等于指定基线。 | 恢复完整 Git clone/toolchain，并在 CI 构建 ARM。 |
| 运行参数有双来源 | `main.cpp::load_foc_studio_defaults`, `firmware/include/foc_config.hpp` | 已证明：portable 常量与烧录默认值重复且电流上限不一致。 | 采用 `SOURCE_OF_TRUTH.md` 的单一运行时来源和一致性测试。 |

## P2

| 问题 | 文件 / 函数 | 原因与风险 | 建议 |
|---|---|---|---|
| ABZ magic numbers分散 | `controller.cpp`, `low_speed_compensator.hpp`, `ascii_protocol.cpp` | 已证明：blend、filter、gain、I clamp、breakaway 和 scan 常数散落；不能独立 A/B 或说明有效参数。 | 仅重构为私有 `AbzLowSpeedConfig`，保持初始数值等价。 |
| 观测不足 | `ascii_protocol.cpp` telemetry | 已证明：缺 filtered/P/final torque/saturation/delta/blend/bandwidth/no-progress/effective gains。 | 在控制周期写轻量诊断成员，USB 低频读取。 |
| 边界仅值连续 | `controller.cpp::update` | 已证明：多段 linear clamp 的函数值连续，但 0.50/0.75/1.00/1.50/1.75/2.00 均有导数或状态切换。 | 用 deterministic test 与 HIL 测最终 torque；不先调 PI。 |
| Position mode estimator区域 | `Controller::update` | 已证明路径、未验证后果：`vel_des` 由位置误差推导，仍按 command speed 选 estimator，并可启用最低速度补偿。 | 独立 position small-error HIL 与 mode-switch tests。 |
| 补偿与积分/比例功能重叠 | `LowSpeedCompensator::update`, `Controller::update` | 推测：三者可同时推动转子；虽然 I hold 与 clamp 降低储能，未有闭环仿真或 HIL 证明不会出现 release limit cycle。 | 先记录分项 torque、状态和进度，再以数据决定改动。 |

## P3

| 问题 | 文件 / 函数 | 原因与风险 | 建议 |
|---|---|---|---|
| 文档与实际快照不一致 | `README.md` 旧链接 | 已证明：原 README 指向不存在的 docs 并使用固定 `D:\\foc2` 路径。 | 本阶段已改为现有文档和 `<repository-root>`。 |
| 固件 doctest 未纳入根测试 | `Firmware/Tests/test_runner.cpp` | 已证明：已有 filter/compensator tests，但根脚本不运行它们。 | 下一测试 commit 接入标准 runner/CI。 |
