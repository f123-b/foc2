# HIL 测试计划

软件仿真和单元测试不能证明电机稳定性。本计划用于在限流 2 A、力矩限幅 0.2 Nm、12 V 系统和可随时急停的前提下获得可比较的实机证据；不得通过提高电流、力矩或积分来掩盖问题。

## 记录内容

以同一时间基准记录 command velocity、control velocity（observer，即闭环反馈）、raw PLL velocity、50/100 ms window velocity、M/T velocity、observer bandwidth、estimator disagreement、Iq setpoint、Iq measured、P torque、I torque、low-speed torque、total/pre-limit torque、final limited torque/saturation、encoder count/delta、glitch count、state、position error、low-speed state。FOC Studio 的 `g` fast telemetry（56 字段）与 `j` aggregate 已提供这些量；缺失字段先作为下一阶段观测性工作，不可从 UI 显示值推断。

## 速度场景

每档稳态至少 10 s，正负方向均做：±2、±1.5、±1.0、±0.5、±0.2 turn/s。另做 0 -> 0.2、0.2 -> 0、0.2 -> -0.2、1 -> 2、2 -> 1、堵转与受控释放、以及可重复负载扰动。位置模式另做小于/等于/大于 2 与 4 encoder counts 的误差跨越。

每段记录：平均速度误差、峰峰纹波、最大 Iq、最大总 torque、饱和占比、no-progress/compensator 状态时长、越界或 fault。2 turn/s 不得劣于基线；任何速度出现超速、持续周期性大幅振荡、或急停失效即停止试验。

## 安全验证

在每个可动模式验证 `x 0`：PWM 立即关闭、请求 Idle、再次 arm 不复用前次 trajectory/I/compensation/current-PI 状态。验证停止发送控制命令但持续发送 `g/j` 的当前行为；在 watchdog 改动后同一场景必须于 timeout 后停止。只读遥测测试应与显式 `u 0` heartbeat 测试分开。

原始 CSV、固件 ELF SHA-256、NVM 备份、接线/负载说明和测试脚本版本必须随每组结果归档。结论中只能写“该 HIL 配置下通过”，不能泛化为所有机械系统。
