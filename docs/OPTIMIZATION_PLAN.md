# 优化计划

> 注：本文档早期的“window/PLL blend、velocity filter、gain scheduling”计划已被
> 更早的实现取代；随后又经 [ABZ 机械测速架构](ABZ_VELOCITY_ESTIMATION.md) 收敛为
> 单一 observer 反馈 + 50/100 ms 双窗口诊断 + M/T 诊断，blend 与
> VelocityFeedbackFilter 已删除。以下第 5 步的仿真目标已由
> `Firmware/Tests/test_velocity_estimators.cpp` 覆盖。

本阶段没有修改 Kp、Ki、积分限幅、补偿扭矩、滤波带宽或 estimator blend。下列步骤必须在完整 Git clone 和可复现 ARM toolchain 中逐个完成；每步通过测试和固件构建后才进入下一步。

1. `docs: document FOC runtime architecture and source of truth`：完成本阶段文档、修复 README 链接并记录不可用的基线条件。
2. `test: establish FOC control regression baseline`：让统一脚本先配置并运行 portable CMake tests；将 `Firmware/Tests/test_runner.cpp` 纳入 host runner；增加 encoder window、overflow/reversal、filter NaN/bandwidth、compensator timeout/mode exit、controller continuity 与 protocol safety tests。
3. `refactor: isolate ABZ low-speed control configuration`：新增运行时私有 `AbzLowSpeedConfig`，集中并带单位表达 0.50/0.75/1.00/1.50/1.75/2.00 turn/s、8/15 Hz、0.003/0.008 Nm、补偿扭矩、25/50 ms 与计数阈值。初始值逐字节等价，默认 feature flags 全开，且仅 Idle 可变。
4. `fix: separate control watchdog from telemetry activity`：保留 `u` heartbeat 与 `p/q/v/c/t/b` 控制命令喂狗；令 `g/j/f/r` 只读。先提供 temporary compatibility flag、发布迁移说明和 timeout HIL test，避免静默破坏旧主机。
5. `test: add deterministic ABZ low-speed control simulation`：用真实 observer、PI clamp 和 compensator 搭建固定步长机械/4000 CPR encoder 模型。它只筛查 limit cycle、windup 和不连续性，不可称为硬件验证。
6. 仅在 HIL 数据证明需要时，单变量优化控制参数。每一次只改一个机制，并运行完整回归矩阵。

Feature flags 的范围应只覆盖 ABZ 速度/位置级联：observer 反馈、50/100 ms 诊断窗口、M/T 诊断、ABZ gain scheduling、I clamp、low-speed compensator、anti-cogging injection。禁止影响 torque control、current PI、Clarke/Park、SVPWM 或 sensorless。
