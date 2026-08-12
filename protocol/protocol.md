# FOC Studio USB CDC 通信协议

固件通过 USB CDC 接口传输以换行结束的 ASCII 命令，波特率为 115200。
基础 ASCII 命令仍可使用，包括 `r`、`w`、`p`、`v`、`c`、`u`、
`ss`、`se` 和 `sr`。FOC Studio 额外增加以下命令：

| 命令 | 响应 | 作用 |
|---|---|---|
| `m axis mode` | `ok mode n` 或 `err ...` | 切换反馈模式，只允许在 `Idle` 状态执行 |
| `g axis` | `! ...` | 读取轻量快帧遥测，并续运行看门狗 |
| `j axis` | `@ ...` | 读取一条聚合遥测数据 |
| `x axis` | `ok stopped` | 立即关闭 PWM，清空控制状态并请求进入 `Idle` |
| `k axis` | `ok clear` | 清除故障，只允许在 `Idle` 状态执行 |
| `a axis` | `ok calibrating` | 按当前模式执行电机校准或完整校准 |
| `b axis` | `ok cogging-calibrating` | 仅 ABZ：以 ±2 turn/s 正反各扫描 6 圈并生成齿槽补偿图 |
| `t axis position` | 无 | 使用 v2 固定梯形轨迹移动到绝对位置 |

## 反馈模式编号

| 数值 | 模式 |
|---:|---|
| 0 | AS5047P SPI 绝对值，16384 CPR |
| 1 | ABZ 增量，4000 CPR |
| 2 | 无感 FOC |
| 3 | 无感 FOC，同时监视 SPI 编码器 |
| 4 | 无感 FOC，同时监视 ABZ 编码器 |

## 遥测格式

快帧用于示波器和控制量，格式为：

```text
! state velocity current position bus_voltage phase_a_voltage phase_b_voltage phase_c_voltage id_measured iq_setpoint id_setpoint velocity_setpoint raw_velocity window_velocity velocity_integrator_torque low_speed_torque position_setpoint position_error low_speed_state
```

上位机按 20 ms（目标 50 Hz）请求快帧。

`@` 后前十个字段保持兼容，随后附加子模块诊断和控制许可状态：

```text
@ state fault velocity current position bus_voltage fet_temperature observer_locked angle_error mode motor_error encoder_error controller_error sensorless_error armed_state encoder_ready motor_calibrated direction fet_thermistor_error motor_thermistor_error control_mode phase_a_voltage phase_b_voltage phase_c_voltage id_measured iq_setpoint id_setpoint anticogging_valid anticogging_active anticogging_index
```

- `velocity`：机械转速，单位 turn/s；
- `current`：PWM 使能时测得的 q 轴电流，单位 A；`Idle` 或 PWM 关闭时为 0；
- `position`：机械位置，单位 turn；
- `bus_voltage`：母线电压，单位 V；
- `fet_temperature`：板载 MOS 热敏温度，单位 °C；
- `angle_error`：无感角度与编码器角度之差，单位 turn。
- `armed_state`：0 为 PWM 已关闭，3 为 PWM 已使能；
- `encoder_ready`、`motor_calibrated`：控制前置条件；
- `direction`：电机方向，必须为 1 或 -1 才能进入控制；
- 五类 `*_error` 用于解析轴聚合故障对应的具体原因。
- `control_mode`：1 为扭矩、2 为速度、3 为位置控制。
- `phase_a_voltage`、`phase_b_voltage`、`phase_c_voltage`：由电流环最终的
  `v_alpha/v_beta` 反 Clarke 变换得到的 PWM 指令相电压，单位 V。V3.6 控制板
  没有三相电压 ADC，因此这些字段不是示波器探头实测电压；`Idle` 或 PWM 关闭时为 0。
- `id_measured`、`iq_setpoint`、`id_setpoint`：d/q 轴电流诊断量，单位 A；
  `Idle` 或 PWM 关闭时为 0。
- `velocity_setpoint`：级联位置/速度控制器使用的速度给定，单位 turn/s；
- `raw_velocity`、`window_velocity`：编码器 PLL 原始速度和 ABZ 10 ms 计数窗口速度，
  单位 turn/s；非 ABZ 模式下窗口值仅用于诊断；
- `velocity_integrator_torque`、`low_speed_torque`：速度积分和低速起动补偿各自贡献的
  转矩，单位 Nm；
- `position_setpoint`、`position_error`：轨迹位置给定和当前级联位置误差，单位 turn；
- `low_speed_state`：0/1/2/3 分别表示空闲、运行、起动和恢复状态；
- `anticogging_index`：双向扫描的总进度，`0～1800` 为正转 6 圈，`1800～3600`
  为反转 6 圈；完成后 `anticogging_valid=1`。

`b axis` 使用 `0.75 turn/s²` 平滑升速至 `+2 turn/s`，正转采集 6 圈，再平滑换向至
`-2 turn/s` 反转采集 6 圈，最后减速至实测速度低于 `0.05 turn/s` 后进入 `Idle`。
每个位置分别计算正反方向的平均维持转矩，再将两者平均以抵消摩擦和控制相位偏差；
缺少任一方向样本的位置写为零补偿。正常耗时约 17 秒。

FOC Studio 的位置/速度/扭矩路径在切换控制模式时会清除未使用的前馈并清空速度积分。
位置按钮发送 `t axis absolute_position`，固件每次把轨迹限制固定为 0.30 turn/s 和
0.60 turn/s² 加减速，避免旧 NVM 参数产生过快移动；扭矩命令仍走基础
`c axis torque` 和 `PASSTHROUGH` 输入。

速度模式临时使用 `VEL_RAMP`、速度增益 0.0025、速度积分 0.01、速度斜坡
0.3 turn/s²。ABZ 速度和位置级联环使用 10 ms 固定计数窗口，1.5～1.75 turn/s 与
PLL 平滑混合；混合结果再经 8～15 Hz 速度相关低通后同时进入 P/I。低速起动恢复要求
累计正向移动 3 个计数，原地计数回摆不会误判起转；ABZ 积分转矩始终限制在
0.003～0.008 Nm。窗口速度只进入级联外环和速度遥测，
不改写编码器原始估算，因此扭矩模式、电流限速和 FOC 相位推进保持原路径。离开 ABZ
速度/位置级联模式、急停、切换反馈模式或故障回到 Idle 后，固件立即清除低速状态。
`ss` 只允许在所有轴均为 Idle 时执行，并会先恢复速度模式的临时参数。

上位机必须先读取设备参数才能批量写入，并且只写实际修改的可编辑项。电机方向、
极对数、编码器 CPR、电阻、电感和预校准标志不参与批量写入，防止默认值覆盖校准结果。

纯无感模式不能从 `0.1 turn/s` 闭环启动。上位机要求先用 ±5～±10 turn/s 的速度
目标完成开环爬升；固件使用 1.5 A、100 electrical-rad/s² 爬升到约 6.37 turn/s。
只有无感锁定后才允许切换扭矩模式。

当前轻量快帧目标周期为 20 ms（50 Hz），完整状态帧周期为 200 ms（5 Hz）。三相指令电压在
高电角速度下会发生欠采样，不能替代高速采样示波器或差分电压探头。

`g` 和 `j` 遥测命令都会续看门狗，上位机所有串口写入通过同一顺序队列发送。运行状态
连续 1.0 秒没有任何有效遥测/控制通信时，固件看门狗会退出控制状态；`Idle` 状态下
看门狗自动保持，不会锁存超时故障。
