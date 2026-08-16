# ABZ 机械测速架构（ABZ Velocity Estimation）

本文档描述 ABZ（增量）编码器模式下，FOC 控制器的机械速度估算架构：每一个
估算器的输入、用途、进入哪个闭环、如何从 FOC Studio 判读。本文档是
`Controller::update()`、`Encoder::update()` 与 `communication/ascii_protocol.cpp`
实际调用链的权威说明；如与旧注释冲突，以代码为准。

## 1. 背景：为什么旧测速链“速度不可信”

FOC Studio 中 ABZ 速度模式的主速度曲线来自 `control_observer_velocity_`
（40 Hz 固定带宽 observer），而该 observer 的输入是
`shadow_count_ / CPR`。这条链有几个真实问题：

1. **int32 计数溢出风险**：`shadow_count_` 是 int32。4000 CPR 下
   2 turn/s = 8000 count/s，约 3.1 天溢出；10 turn/s 约 15 小时溢出。
   溢出后 observer 位置输入直接跳变，速度曲线出现无法解释的台阶。
2. **float32 精度恶化**：`shadow_count_ / CPR` 转成 float 后，超过约
   4194 圈（2^24 个 count）就丢失单 count 分辨率；长时间运行 observer
   的 innovation 精度下降。
3. **固定 40 Hz 带宽**：低速时 40 Hz 带宽把单 count 量化噪声放大成
   速度纹波；高速时又不够快。一个带宽无法同时满足 0.05 与 4 turn/s。
4. **每 8 kHz tick 重算增益**：`configure(bandwidth)` 每周期执行
   `2*pi`、乘法、平方，纯浪费。
5. **多套估算器并存、职责不清**：滚动窗口、M/T、PLL、observer、
   VelocityFeedbackFilter 并存，且旧注释声称“count-time estimate is single
   feedback source”，与真实调用链不符（真实反馈是 observer）。
6. **上位机 PI 参数与固件不一致**：`apply_foc_velocity_tuning()` 修改
   generic `vel_gain`/`vel_integrator_gain`，而 ABZ 闭环实际使用
   `abz_vel_gain`/`abz_vel_integrator_gain`，UI 调了参数但闭环没用上。

## 2. 架构目标（修改后）

```
                  ┌── encoder PLL (vel_estimate_)
ABZ encoder ──────┤      用途：commutation、电角度插值、encoder 内部预测、安全
（timer count）    │
                  ├── control velocity observer (AbzVelocityObserver)
                  │      用途：速度 PI 反馈（唯一来源）
                  │
                  ├── 50 ms rolling window
                  │      用途：快速机械诊断 / observer 初始化种子
                  │
                  ├── 100 ms rolling window
                  │      用途：稳态机械参考
                  │
                  └── M/T (count-time) estimator
                         用途：低速边沿诊断、异常 count 分析
```

约束（本次严格遵守）：

- 50/100 ms 窗口**永不进入**电角度路径；
- 电角度/commutation 仍由 encoder PLL 体系负责，未改动；
- 无动态内存、无阻塞、无每周期大数组清零；
- 不因调速故意加重的低通滤波器（LPF）被移除；
- 不修改 Clarke/Park/SVPWM、电流环、anticogging map、friction 主逻辑。

## 3. 数据流（真实调用链）

```
TIM encoder counter (16-bit)
  -> Encoder::sample_now()   (low_level pwm_adc_cb)
  -> Encoder::update()       (Axis::do_updates(), 每 8 kHz tick)
       delta_enc = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_
       shadow_count_ (int32) += delta_enc            [commutation 快路径]
       mechanical_count_ (int64) += delta_enc        [诊断/observer/位置跟踪]
       mechanical_velocity_window_.push(delta_enc)
            -> velocity_window_50ms_  = sum50 / (CPR * 50ms)
            -> velocity_window_100ms_ = sum100 / (CPR * 100ms)
       mt_velocity_estimate_ = mt_velocity_estimator_.update(delta_enc, T, CPR)
       PLL: pos_estimate_ / vel_estimate_ / phase_ / interpolation_（不动）
  -> Controller::update()    (run_closed_loop_control_loop, 每 8 kHz tick)
       cascaded_abz_mode == (VELOCITY|POSITION) && encoder=MODE_INCREMENTAL
       observer 输入: delta_position = last_delta_count_ / CPR   （增量驱动）
       observer 带宽: bandwidth_for(|vel_des|) 自适应，gain 只在带宽变化>0.5Hz 时重算
       velocity_feedback = control_observer_velocity_           （唯一反馈）
       v_err = vel_des - velocity_feedback
       torque = Kp*v_err + integrator + feedforward(anticogging/friction)
       -> abz velocity torque limit -> motor global torque limit -> Motor::update()
```

## 4. 各估算器详解

### 4.1 ABZ encoder raw count（`delta_enc` / `shadow_count_` / `mechanical_count_`）

- `delta_enc`：每 control tick 的 16 位计数器差（带符号扩展），wrapping 安全。
- `shadow_count_`（int32）：commutation/电角度快路径，保留原 ODrive 行为。
- `mechanical_count_`（int64）：新增，每 tick `+= delta_enc`。int64 在
  4000 CPR、任意实际转速下不会在设备寿命内溢出；供诊断、observer 源与
  位置跟踪使用。**observer 不直接吃绝对位置**（见 4.3），所以大整数
  不会恶化 observer 精度。

### 4.2 encoder PLL（`vel_estimate_`，count/s 后除以 CPR 得 turn/s）

- 输入：`count_in_cpr_`（wrap 后 0..CPR）相位检测。
- 用途：**commutation、电角度插值（interpolation_）、encoder 内部预测、
  FOC Studio 的“PLL 原始速度”**。
- 本项目不改它的计算与作用。

### 4.3 control velocity observer（`AbzVelocityObserver` → `control_observer_velocity_`）

- **唯一进入 ABZ 速度 PI 的反馈**：`Controller::velocity_feedback_for_control()`
  在 `cascaded_abz_control() && control_observer_valid_` 时返回
  `control_observer_velocity_`。
- 输入：每 tick 增量位置 `last_delta_count_ / CPR`。observer 内部维护本地
  帧 `pos_meas_`（+= delta），当 `|pos_meas_| > 8 turn` 时 rebase 回 0
  （`pos_meas_`/`pos_hat_` 同时平移，innovation 不变）。因此：
  - 不受 int32 shadow_count 溢出影响；
  - 本地帧始终很小，float32 永不丢 count 级精度；
  - 长时间运行可靠。
- 结构：临界阻尼位置 PLL：
  `omega_n = 2*pi*bandwidth; kp = 2*omega_n; ki = 0.25*kp^2`。
- 自适应带宽（平滑插值，无突变）：

  | \|指令速度\| (turn/s) | observer 带宽 (Hz) |
  |---|---|
  | 0 | min（默认 15） |
  | 0.3 | ~20 |
  | 1.0 | ~30 |
  | 2.0 | ~40 |
  | ≥4.0 | max（默认 50） |

  带宽夹在 `abz_velocity_observer_min_bandwidth` 与
  `abz_velocity_observer_max_bandwidth` 之间。gain 只在带宽变化超过
  0.5 Hz 时重算（`set_bandwidth` 内部 epsilon 判断），稳态指令下每 tick
  只做一次带宽插值（几次乘法），无除法、无重算。
- 生命周期：`Controller::reset()` 置 `control_observer_valid_ = false`；
  下一个 ABZ 控制 tick 用 `reset(position, initial_velocity)` 初始化，
  `initial_velocity` 优先级：50 ms 窗口 valid → M/T valid → 0。这样
  **旋转中切入速度模式不会从 0 启动造成 torque impulse**。

### 4.4 50 ms rolling window（`velocity_window_50ms_`）

- 真滑动窗口，每个 control tick 更新；`velocity = sum(delta)/ (CPR * 50ms)`。
- 用途：快速机械诊断参考、observer 初始化种子、anticogging 采样 sanity 对照。
- 4000 CPR 下量化分辨率 ≈ 1/(4000*0.05) = 0.005 turn/s。
- **不进电角度、不进速度 PI**。

### 4.5 100 ms rolling window（`velocity_window_100ms_`）

- 真滑动窗口，每个 control tick 更新；`velocity = sum(delta)/ (CPR * 100ms)`。
- 用途：稳态机械速度参考（最稳）。
- 4000 CPR 下量化分辨率 ≈ 1/(4000*0.1) = 0.0025 turn/s。
- **不进电角度、不进速度 PI**（100 ms 太慢，不适合闭环反馈）。

实现上两个窗口共享**单一 800-slot ring buffer**（8 kHz 下 100 ms = 800 tick、
50 ms = 400 tick；样本数由 `current_meas_hz` 在编译期推导，并有
`static_assert` 校验 100 ms == 2×50 ms）。维护 `sum50`/`sum100` 两个 int64，
每 tick：加最新 delta、减去各自窗口滑出的 delta。RAM 约 3.2 KiB，无动态分配。

### 4.6 M/T (count-time) estimator（`mt_velocity_estimate_`）

- 定位：**低速边沿诊断**，不进入速度 PI、不进电角度。
- 发布规则：count 阈值（低速）或 max_publish_time（高速/停止）触发；
  输出 slew 限制（`max_velocity_slew`）抑制单 count 毛刺。
- 零速行为（本次修正）：不再因 `max_publish_time=10ms` 到期且
  `count_accum==0` 就周期性发布 0（会造成 “0 → 非0 → 0” 闪烁）：
  - 边沿仍被预期（距上次边沿 < 4×理论边沿间隔，下限 30 ms）时，
    零 count 的发布**保持**上一个估计值；
  - 超过边沿超时（转子确已停止/远慢于估计）后，估计值按 τ=0.1 s
    指数衰减到 0（一阶近似 `v *= (1 - dt/τ)`，无 exp 调用），不硬跳。
- 用途：FOC Studio “M/T 速度”通道；与 observer/窗口对比可判断低速
  边沿行为；检测异常 count。

### 4.7 已删除：VelocityFeedbackFilter（velocity_filter.hpp）

旧 `VelocityFeedbackFilter abz_velocity_feedback_filter_` 只被
`clear()`/`reset()`，从未 `update()`——已确认是 dead code，与
`velocity_filter.hpp` 一并删除。`raw_overspeed_lead_count_` 同样删除。

## 5. 哪个进入速度 PI？哪个进 telemetry？

| 量 | 进入 ABZ 速度 PI？ | FOC Studio 通道 |
|---|---|---|
| `control_observer_velocity_`（observer） | **是（唯一）** | `velocity` / `controlVelocity` / `observerVelocity` |
| `vel_estimate_`（encoder PLL） | 否（commutation/安全） | `encoderPllVelocity`（rawVelocity 别名） |
| `velocity_window_50ms_` | 否（诊断） | `velocityWindow50ms` |
| `velocity_window_100ms_` | 否（诊断） | `velocityWindow100ms` |
| `mt_velocity_estimate_`（M/T） | 否（诊断） | `mtVelocity` |
| `velocity_estimator_disagreement_` | 否（诊断） | `velocityEstimatorDisagreement` |
| `observer_bandwidth_` | 否（诊断） | `observerBandwidth` |
| `abz_count_glitch_count_` | 否（诊断） | `abzCountGlitchCount` |

FOC Studio 主速度曲线（`velocity`）== 速度 PI 实际反馈
（`control_observer_velocity_`），不再显示任意某个 estimator。

## 6. 为什么不能直接用单周期 delta_count 测速

- 4000 CPR、8 kHz 下，1 turn/s 只有 0.5 count/tick：单 tick 测速在
  “0、1、0、1”之间跳（0 ↔ 0.25 turn/s），无法直接闭环；
- 低速（0.1 turn/s）时平均每 2.5 ms 才 1 个 count，单 tick 几乎总是 0；
- 窗口/observer 的意义是把计数到达时间的随机性（量化噪声）平滑掉：
  窗口用足够长的真实 count 求和，observer 用有带宽的动力学滤波。
- 量化分辨率对比（4000 CPR）：
  - 50 ms 窗口：0.005 turn/s；
  - 100 ms 窗口：0.0025 turn/s；
  - 单 tick（0.125 ms）：0.2 turn/s（不可用）。

## 7. observer 带宽策略

见 4.3。原则：低速降带宽（滤 count 量化噪声，避免 torque hunting），
高速升带宽（跟踪真实机械速度）。带宽来自**指令速度**（不是测量速度），
避免测量噪声反馈进带宽选择造成 estimator chatter；连续插值、无硬切换，
闭环连续。

## 8. FOC Studio 如何判读

在“示波器”页使用两个新预设：**“速度估算诊断”**（Velocity Setpoint、
Control Velocity、50 ms、100 ms、Raw PLL、M/T）和**“速度环诊断”**
（Velocity Setpoint、Control Velocity、Velocity Error、Iq Setpoint、
P Torque、I Torque、Friction Torque、Final Torque）。

| 现象 | 判读 |
|---|---|
| A. controlVelocity 波动大，而 window50/window100 平 | estimator/observer 问题：降 observer 最低带宽、检查 observer 参数 |
| B. controlVelocity、window50、window100 一起波动 | 真实机械速度波动（机械偏心/负载/失步） |
| C. window100 很稳、window50 有周期 ripple | 高频机械 ripple / cogging（真实齿槽），不是测速问题 |
| D. 所有速度估计都波，且 finalTorque 大幅反向修正 | speed-loop hunting：先查 observer 带宽与 PI 增益 |
| E. deltaCount 出现异常反向或大 spike（abzCountGlitchCount 增长） | ABZ 硬件/信号完整性问题（接线、电平、屏蔽、电源纹波） |
| F. velocityEstimatorDisagreement 持续很大 | 观察器与窗口不一致：检查是否发生计数毛刺或机械异常 |

## 9. 调试流程（刷固件后）

见 README 中“ABZ 测速验证流程”一节。要点：

1. ABZ 模式、完整校准、保存；
2. 依次速度指令 0.2 / 0.5 / 1.0 / 1.5 / 2.0 / 3.0 turn/s，每档稳定 ≥ 3 s；
3. 同时观察 velocitySetpoint、controlVelocity、window50、window100、
   rawPLL、M/T、Iq、P torque、I torque、final torque；
4. 按第 8 节规则判读；若 A/D 出现，先调 observer 带宽，再轻调
   abz_vel_gain（先 Kp 后 Ki），Ki 不要回到 0.01 量级。

## 10. 参数一览（默认值）

| 参数 | 默认 | 说明 |
|---|---|---|
| `abz_vel_gain` | 0.002 Nm/(turn/s) | ABZ 速度 PI Kp（唯一有效 Kp） |
| `abz_vel_integrator_gain` | 0.002 Nm/(turn/s·s) | ABZ 速度 PI Ki |
| `abz_velocity_observer_min_bandwidth` | 15 Hz | observer 最低带宽 |
| `abz_velocity_observer_max_bandwidth` | 50 Hz | observer 最高带宽 |
| `abz_velocity_torque_limit` | 0.015 Nm | ABZ 速度环转矩限幅 |
| `abz_velocity_integrator_limit` | 0.015 Nm | 速度积分器限幅 |
| `abz_coulomb_friction_torque` | 0.0015 Nm | 库仑摩擦前馈 |
| `abz_breakaway_torque` | 0.0055 Nm | 破槽转矩 |
| window 样本数 | current_meas_hz/20、/10 | 编译期由 8 kHz 推导 |

## 11. 兼容与迁移说明

- 配置布局变化：`control_velocity_observer_bandwidth` 被两个新字段替代，
  `nvm_config.hpp` 的 `config_version` 升到 0x0003；旧 NVM 配置失效，需
  重新校准并保存（开发阶段可接受，README 已注明）。
- FOC Studio 旧字段保留 alias（`velocity`/`rawVelocity`/`windowVelocity`/
  `mTVelocity`/`controlObserverVelocity`），新 UI 使用规范名。
- fast telemetry `g` 追加 4 个字段（100 ms 窗口、有效带宽、估算器分歧、
  计数毛刺计数），`respond()` buffer 扩到 512；协议测试验证 56 字段对齐。
