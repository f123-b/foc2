# FOC Studio 项目知识库（AI 可检索版）

---
kb_id: foc-studio
document_type: engineering_project_knowledge_base
language: zh-CN
snapshot_date: 2026-08-22
project_status: active_worktree
primary_runtime: firmware-target/Firmware
host_runtime: host/foc-studio
protocol_contract: protocol/protocol.md
verification_level: automated_build_and_algorithm_tests_passed_hil_required
intended_consumers: [AI assistant, developer, interviewer, maintainer]
---

> 文档定位：这是项目知识库，不是单纯的面试稿。AI 应先读取第 0 节的知识库规则、事实状态和组件关系，再按第 22 节索引定位详细内容。第 18 节的问答是由技术事实派生的表达模板，不能反向覆盖代码事实。
>
> 当前快照：以 firmware-target/Firmware 的实际构建代码、host/foc-studio 的实际实现、protocol/protocol.md 的协议契约和本次构建/测试结果为准。自动化测试通过不等于任意机械负载下的 HIL 稳定性已证明。

## 0. 知识库使用规则

### 0.1 事实状态标签

AI 处理本知识库中的结论时，使用以下状态区分可信度：

| 标签 | 含义 | 回答时的措辞 |
|---|---|---|
| IMPLEMENTED | 当前代码中存在明确实现 | “当前代码实现为……” |
| VERIFIED_AUTOMATED | 已由协议测试、单元测试、ARM smoke 或全量构建验证 | “自动化验证已通过……” |
| VERIFIED_HIL_REQUIRED | 需要真实电机、负载、供电和编码器数据 | “软件侧已具备，实机结论仍需 HIL……” |
| DESIGN_INTENT | 设计目标、约束或推荐方案，不代表全都已完成 | “设计目标是……” |
| KNOWN_RISK | 已识别但尚未完全解决的风险 | “当前已知风险是……，建议……” |
| INTERVIEW_DERIVED | 为面试表达整理的摘要或回答模板 | “面试时可以这样表述……” |

如果一句话同时包含多个状态，必须拆开回答，不能把 DESIGN_INTENT 或 INTERVIEW_DERIVED 说成 VERIFIED_AUTOMATED。

### 0.2 来源优先级

当不同文件、旧文档或用户口述出现冲突时，按以下顺序判断：

1. 当前实际参与固件构建的源码和配置；
2. 当前 host parser、协议实现和自动化测试；
3. 固件构建脚本、编译产物和测试脚本；
4. docs/ 中的架构、回归和 HIL 文档；
5. 本文的面试表达模板；
6. 历史说明、旧参数、旧截图或推测。

重点事实来源：

| 事实域 | 首选来源 | 关键入口 |
|---|---|---|
| 实际烧录固件 | 构建脚本和 target source | tools/build-firmware.ps1、firmware-target/Firmware |
| 产品默认参数 | 固件启动默认值 | firmware-target/Firmware/MotorControl/main.cpp |
| 控制链路 | 控制器和电机实现 | MotorControl/controller.cpp、motor.cpp、axis.cpp |
| ABZ 速度架构 | observer/window/encoder/controller | MotorControl/abz_velocity_observer.hpp、abz_velocity_window.hpp、encoder.cpp、controller.cpp |
| 通信格式 | 固件 parser + host parser | communication/ascii_protocol.cpp、host/foc-studio/protocol.js |
| 自动化证据 | 测试脚本和输出 | tools/test-project.ps1、Firmware/Tests、firmware/build-host |
| 实机结论 | HIL 原始数据和记录 | docs/HIL_TEST_PLAN.md 及测试归档 |

### 0.3 AI 回答规范

回答项目问题时，优先按以下顺序组织：

1. 结论：直接回答问题；
2. 机制：解释数据流、状态机、公式或边界；
3. 证据：给出代码路径、测试类型或构建结果；
4. 限制：说明是否仍依赖 HIL、硬件条件或未完成事项；
5. 下一步：只有在用户询问改进时再给方案。

禁止 AI 自动补全以下未被本知识库证明的信息：

- 未记录的 PCB 层数、线宽、器件型号、采样电阻、栅极电阻、BOM、成本或产量；
- 未记录的实机速度误差、温升、效率、扭矩精度、振动或稳定性数字；
- 未记录的个人代码提交量、团队人数、项目周期和具体职责；
- 把软件模拟设备说成电机仿真或 HIL；
- 把自动化测试通过说成“所有硬件均验证通过”；
- 把参考成熟项目的架构说成直接使用其成品硬件或官方固件。

### 0.4 项目别名和术语归一化

| 术语/别名 | 统一含义 |
|---|---|
| FOC Studio | 本项目整体，包括自研控制板固件、USB 协议和 Electron 上位机 |
| target firmware | firmware-target/Firmware，实际编译和烧录的 STM32 固件 |
| portable core | firmware/，平台无关、可在 host 测试的精简核心，不是烧录固件 |
| host | host/foc-studio，Electron/浏览器上位机 |
| ABZ | 增量编码器反馈模式，目标 CPR 为 4000 |
| SPI | AS5047P 绝对值编码器反馈模式，目标 CPR 为 16384 |
| control observer | ABZ 速度/位置级联环唯一速度 PI feedback |
| raw PLL | Encoder 原始 PLL 速度，负责电角度插值/commutation 等路径，不是 ABZ 速度 PI feedback |
| fast frame | 固件以 ! 开头的高频诊断帧，host 目标 50 Hz |
| aggregate frame | 固件以 @ 开头的聚合状态帧，host 目标 5 Hz |
| torque limit | 可能包括 ABZ velocity torque limit 与 motor global torque limit，需明确层级 |
| HIL | 真实控制板、电机、编码器、供电和负载组成的实机测试，不等同于 host 单元测试 |

### 0.5 知识图谱主关系

~~~text
FOC Studio
  ├─ contains -> self-designed control board
  ├─ contains -> target firmware
  ├─ contains -> USB CDC ASCII protocol
  ├─ contains -> Electron/Web Serial host
  ├─ controls -> 3505-KV650 PMSM
  ├─ supports_feedback -> AS5047P SPI / ABZ incremental / sensorless
  └─ diagnoses -> velocity / current / torque / faults / estimator disagreement

Host UI
  -> USB CDC
  -> ASCII parser
  -> Axis state machine
  -> Controller
  -> Motor current control
  -> Clarke/Park/current PI/inverse Park/SVPWM
  -> DRV8301/MOSFET/motor
  -> Encoder or sensorless feedback
  -> Controller

ABZ encoder
  -> delta_count
  -> raw PLL for electrical phase and commutation
  -> 50 ms window for diagnosis/seed/gate
  -> 100 ms window for steady diagnosis
  -> M/T for edge diagnosis
  -> control observer for the only ABZ velocity PI feedback
~~~

### 0.6 文档导航

| AI 任务 | 首先读取 | 再读取 |
|---|---|---|
| 快速介绍项目 | 第 1 节 | 第 2、4 节 |
| 解释完整架构 | 第 4 节 | 第 5、6、11、12 节 |
| 解释 ABZ 低速问题 | 第 7 节 | 第 8、9、14、15 节 |
| 解释安全和急停 | 第 10、16 节 | 第 11.1、17 节 |
| 解释上位机 | 第 12 节 | 第 11、13 节 |
| 解释测试可信度 | 第 14 节 | 第 10.3、17 节 |
| 生成面试回答 | 第 1、2 节 | 第 18、19、21 节 |
| 排查速度振荡 | 第 7、8 节 | 第 14.3、18 的 Q9/Q27/Q28 |
| 判断某结论是否真实 | 第 0.1～0.3 节 | 对应代码/测试来源 |

### 0.7 组件注册表

| Entity ID | 实体 | 责任 | 主要输入 | 主要输出 | 权威代码 |
|---|---|---|---|---|---|
| SYS | FOC Studio system | 连接控制板、运行 FOC、提供诊断 | host command、sensor feedback | motor torque、telemetry、fault | README.md、本文第 4 节 |
| BOARD | self-designed control board | 电源、三相功率级、ADC/PWM、USB CDC、保护 | 12V bus、encoder、电机三相线 | phase PWM、current samples、USB stream | firmware-target/Firmware/Board、MotorControl/low_level.cpp |
| FW | target firmware | 在 MCU 上执行实时控制与安全策略 | command/config、ADC、encoder | PWM timings、state、errors | firmware-target/Firmware |
| AXIS | Axis state machine | 控制状态切换、健康检查、watchdog、控制循环封装 | requested state、component errors | Idle/Calibration/ClosedLoop/Sensorless、disarm | MotorControl/axis.cpp/.hpp |
| ENC | Encoder | SPI/ABZ 采样、PLL、电角度、机械速度诊断 | timer count、SPI sample、CPR | phase、position、raw PLL、delta/window/M/T | MotorControl/encoder.cpp/.hpp |
| OBS | AbzVelocityObserver | ABZ 唯一速度 PI feedback | delta_count/CPR、dt、command speed | control observer velocity、adaptive bandwidth | MotorControl/abz_velocity_observer.hpp |
| CTRL | Controller | position/velocity/torque 外环、FF、limit、anti-windup | setpoint、position、velocity estimates | torque setpoint、diagnostics | MotorControl/controller.cpp/.hpp |
| FRICTION | FrictionCompensator | 低速 Coulomb/breakaway 补偿 | command、velocity error、encoder progress | friction torque、state、timers | MotorControl/friction_compensator.hpp |
| COGGING | anticogging pipeline | 双向扫描、map、质量 gate、前馈 | position、velocity、torque/state | valid map、anticogging torque | MotorControl/controller.cpp/.hpp |
| MOTOR | Motor | torque→Iq、电流 PI、调制和 PWM timing | torque、phase、phase velocity、ADC current | Id/Iq、Valpha/Vbeta、PWM | MotorControl/motor.cpp/.hpp |
| COMM | ASCII protocol | 命令解析、property、telemetry framing | USB/UART bytes | state/config/control、!/@ response | communication/ascii_protocol.cpp |
| HOST | FOC Studio host | UI、串口队列、parser、scope、config safety | !/@ frames、user input | m/v/p/c/t/x/g/j/u/r/w commands | host/foc-studio/app.js、protocol.js |
| TEST | verification system | 验证协议、算法、状态机、ARM build、target build | source、test vectors、toolchain | pass/fail、artifacts、hash | tools/test-project.ps1、Firmware/Tests |

### 0.8 核心不可违反约束

这些约束是 AI 生成设计解释、代码建议或面试回答时必须保留的边界：

1. 实际可烧录固件是 firmware-target/Firmware；firmware/portable core 不能替代它。
2. USB/host 只能改变 setpoint、配置或状态请求，不能承担 8 kHz 实时闭环。
3. ABZ velocity/position cascade 的唯一 velocity PI feedback 是 control observer。
4. Encoder raw PLL 负责电角度/commutation/interpolation 等路径；50/100 ms window、M/T 不能被描述成 ABZ PI feedback。
5. ABZ 低速优化不能无理由修改 Clarke、Park、current PI、SVPWM、commutation 或 sensorless 主路径。
6. 所有 torque 输出要区分 raw/P+I+FF、ABZ limit 后和 motor global limit 后的层级。
7. 急停必须清 Controller transient state、current-control state 并 disarm PWM；不能只把 UI 按钮置灰。
8. 单点编码器 spike 不等于持续超速；ABZ 超速需要正常层/应急层和连续周期限定。
9. 抗齿槽 map 只有通过 coverage、幅值、跳变和 wrap 等质量 gate 才能标记 valid。
10. 自动化测试、固件构建和 HIL 分别证明不同问题，不能互相替代。

### 0.9 核心事实记录

| Fact ID | 原子事实 | 状态 | 证据/来源 |
|---|---|---|---|
| F-001 | 产品默认使用单轴、3505-KV650、10 pole pairs | IMPLEMENTED | MotorControl/main.cpp |
| F-002 | 目标母线是 12V；控制板耐压设计为 56V 器件级 | IMPLEMENTED | main.cpp、项目安全说明 |
| F-003 | SPI 反馈目标为 AS5047P/16384 CPR | IMPLEMENTED | main.cpp、encoder config |
| F-004 | ABZ 反馈目标为 4000 CPR | IMPLEMENTED | host UI、协议和 encoder workflow |
| F-005 | nominal current/control loop 为 8 kHz | IMPLEMENTED | Board timing、encoder window derivation |
| F-006 | target firmware 输出 ELF/HEX/BIN | VERIFIED_AUTOMATED | tools/build-firmware.ps1、本次 build |
| F-007 | host protocol test 通过 | VERIFIED_AUTOMATED | npm.cmd test |
| F-008 | ABZ estimator test 为 135 checks/0 failures | VERIFIED_AUTOMATED | test_velocity_estimators.cpp |
| F-009 | portable core CTest 为 1/1 passed | VERIFIED_AUTOMATED | firmware/build-host |
| F-010 | ABZ observer 用 delta_count/CPR，并在 local frame 过大时 rebase | IMPLEMENTED | abz_velocity_observer.hpp、controller.cpp |
| F-011 | observer 是 ABZ velocity PI 唯一 feedback | IMPLEMENTED | Controller::velocity_feedback_for_control |
| F-012 | 50/100 ms window 和 M/T 只做诊断、seed 或 gate | IMPLEMENTED | encoder.cpp、controller.cpp |
| F-013 | ABZ 使用独立 Kp/Ki、低速 gain schedule 和 anti-windup | IMPLEMENTED | controller.cpp/.hpp |
| F-014 | 摩擦补偿有 IDLE/RUNNING/BREAKAWAY/RECOVERING | IMPLEMENTED | friction_compensator.hpp |
| F-015 | 抗齿槽使用双向扫描和 map quality gate | IMPLEMENTED | controller.cpp |
| F-016 | g/j 当前都会 feed watchdog | KNOWN_RISK | ascii_protocol.cpp |
| F-017 | 当前软件测试不能证明所有机械负载的低速稳定性 | VERIFIED_HIL_REQUIRED | HIL_TEST_PLAN.md |
| F-018 | 当前 ELF 统计为 text 271924/data 1620/bss 136568 | VERIFIED_AUTOMATED | build-firmware.ps1 输出 |

## 1. 项目摘要与对外表述

### 1.1 一句话版本

FOC Studio 是一个面向自研三相电机控制板和 3505-KV650 电机的单轴 FOC 电机控制与诊断系统，包含 STM32 实时固件、USB CDC ASCII 通信协议和 Windows Electron 上位机，重点解决 ABZ 增量编码器在低速运行时的测速量化噪声、摩擦卡滞、齿槽转矩和调试可观测性问题。

### 1.2 3 分钟版本

项目运行在自研的 56V 器件耐压级三相控制板上，产品实际使用单轴。固件在参考成熟 FOC 控制架构后自主完成 Axis、Encoder、Controller、Motor、板级驱动和通信集成，加入了 FOC Studio 产品默认参数、反馈模式切换、模式化校准、低速 ABZ 速度环、双向抗齿槽标定和面向示波器的扩展遥测。

系统闭环链路是：上位机通过 USB CDC 发送 ASCII 命令，固件 ASCII parser 将命令转换成 Axis 状态、Controller 输入或配置写入；Encoder 在控制周期中采样 SPI/ABZ/无感反馈；Controller 计算位置/速度/扭矩输出；Motor 将扭矩转换为 Iq，执行 Clarke/Park 变换、电流 PI、反 Park 和 SVPWM，最终驱动 DRV8301、三相 MOSFET 和电机。

ABZ 低速是项目最有技术含量的部分。之前如果直接用单周期编码器计数测速，会因为 4000 CPR 在 8 kHz 下低速时大量周期为 0、偶尔为 1，导致速度曲线阶梯化并引起速度环 hunting。当前方案用每周期 delta_count / CPR 驱动一个带本地 frame rebase 的自适应带宽机械速度 observer，observer 是 ABZ 速度/位置级联环唯一的速度 PI 反馈；50 ms/100 ms 滑动窗口和 M/T 估算器只用于诊断、初始化和采样质量判断，不进入电角度和速度 PI。控制器再叠加平滑的低速增益调度、带状态机的摩擦/脱困补偿、可选抗齿槽前馈，并通过 ABZ torque limit、全局 torque limit 和积分 anti-windup 限制风险。

上位机侧采用 Electron + Web Serial，主进程只负责窗口和串口权限选择，渲染进程负责界面、协议编解码、命令顺序队列、50 Hz 快帧轮询、5 Hz 聚合状态轮询和实时示波器。配置写入要求先读取设备现值，只写发生变化的可编辑项，从而避免页面默认值覆盖电机方向、编码器 CPR 和校准结果。

### 1.3 面试口径（INTERVIEW_DERIVED）

> 我主要负责电机控制系统中 ABZ 低速速度估算与控制链路、固件和上位机之间的诊断协议，以及调试工具的可观测性建设。我的工作重点不是修改 FOC 电流环，而是在不触碰 Clarke/Park、SVPWM 和 sensorless 主路径的前提下，重构 ABZ 机械测速的职责边界，增加增量 observer、双窗口诊断、低速摩擦补偿和抗齿槽标定，并把 P/I/补偿/饱和/编码器状态全部暴露到上位机，最后用 host 单元测试、协议测试、ARM smoke 和固件全量构建验证。

## 2. 项目定位、范围和关键指标

### 2.1 项目目标

| 目标 | 具体内容 |
|---|---|
| 可运行 | 编译出 STM32 固件，支持 USB CDC、Axis 状态机和完整 FOC 控制链路 |
| 可控制 | 支持扭矩、速度、位置三种控制模式；位置模式使用固定安全梯形轨迹 |
| 多反馈 | 支持 AS5047P SPI 绝对值编码器、ABZ 增量编码器、纯无感 FOC，以及无感+编码器监视 |
| 低速可用 | 降低 ABZ 低速计数噪声，避免单周期测速阶梯导致的速度环 hunting |
| 可调试 | 以 50 Hz 快帧和 5 Hz 聚合帧暴露速度、转矩、电流、状态、限幅和估算器差异 |
| 可维护 | 将编译、测试、桌面打包、工具链加载和 DFU 操作固化到 PowerShell 脚本 |
| 可安全恢复 | 急停、状态限制、PWM disarm、watchdog、过流/过压/超速/驱动故障路径闭合 |

### 2.2 运行范围

| 项目 | 当前实现 |
|---|---|
| 硬件 | 自研三相电机控制板，功率级按 56V 器件耐压设计；产品 profile 使用单轴 |
| MCU | STM32F405 系列，Tup 编译参数为 Cortex-M4/STM32F405xx |
| 电机 | 3505-KV650，10 极对，目标母线 12 V |
| SPI 编码器 | AS5047P，16384 CPR，CS 使用 GPIO4/PA3 |
| ABZ 编码器 | 4000 CPR，增量计数器采样 |
| 控制频率 | 名义电流/控制循环 8 kHz；50 ms/100 ms 窗口按该频率编译期推导 |
| 通信 | USB CDC，按行结束的 ASCII 协议；上位机串口打开参数为 115200 |
| 上位机 | Electron Windows portable；同时保留 Edge/Chrome Web Serial 浏览器备用版 |
| 轴数 | 固件框架支持多轴扩展，但 FOC Studio 产品默认只配置单轴 |

### 2.3 产品默认参数

实际板上默认值来自 firmware-target/Firmware/MotorControl/main.cpp 的 load_foc_studio_defaults()。校准成功并保存后，运行时配置以 NVM 为准。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| 极对数 | 10 | 3505-KV650 电机 |
| 相电阻 | 0.1 Ω | 可由电机校准重新测量 |
| 相电感 | 42.3 μH | 可由电机校准重新测量 |
| 扭矩常数 | 8.27 / 650 ≈ 0.012723 Nm/A | PMSM 扭矩到 q 轴电流换算 |
| 校准电流 | 1 A | 电机/编码器校准阶段使用 |
| bring-up 电流限制 | 2 A | 首次调试的安全上限 |
| 电流限制 margin | 1 A | 电流保护余量 |
| 扭矩限制 | 0.2 Nm | 电机全局扭矩限幅 |
| 电流环带宽 | 500 rad/s | 产品 profile，不是电机额定电压 |
| SPI CPR | 16384 | AS5047P |
| Encoder PLL 带宽 | 100 | 原始编码器 PLL 参数 |
| 速度限制 | 20 turn/s | 超速正常层基于 vel_limit * tolerance |
| ABZ Kp/Ki | 0.002 / 0.002 | 单位为 Nm/(turn/s)、Nm/(turn/s·s) |
| ABZ observer 带宽 | 15～50 Hz | 随指令速度平滑调度 |
| ABZ 速度环 torque limit | 0.015 Nm | 在全局电机 torque limit 前单独限幅 |
| ABZ 积分限幅 | 0.015 Nm | 防止积分储能释放过大 |
| Coulomb / breakaway | 0.0015 / 0.0055 Nm | 低速连续摩擦和脱困前馈 |
| watchdog | 1.0 s | Idle 状态自动维持；运行状态超时退出控制 |

> 注意：控制板能承受 56 V，不等于 3505-KV650 电机可以接 56 V。面试中应主动区分“板级耐压”和“电机/系统额定电压”。

## 3. 仓库结构与真实运行边界

~~~text
foc-master/
├─ firmware-target/Firmware/       # 实际烧录的 STM32 固件 source of truth
│  ├─ MotorControl/                 # Axis / Encoder / Controller / Motor / main
│  ├─ communication/               # USB/UART、ASCII、CAN、Fibre 接口
│  ├─ Board/v3/                     # HAL、ADC/PWM、FreeRTOS、STM32 工程
│  ├─ fibre/                        # 属性接口生成与运行时接口
│  ├─ Drivers/ / Middlewares/       # DRV8301、FreeRTOS、USB 等依赖
│  └─ Tests/                        # 固件侧 doctest 与 host 可编译算法测试
├─ firmware/                        # portable core，不是可烧录固件
│  ├─ include/ / src/               # 平台无关状态机和反馈模式逻辑
│  └─ tests/                        # CMake/CTest host 测试
├─ host/foc-studio/                 # Electron/浏览器上位机
│  ├─ app.js                        # UI 状态、串口读写、轮询、示波器
│  ├─ protocol.js                   # 命令构造、故障码、遥测解析
│  └─ desktop/                      # Electron main/preload
├─ protocol/protocol.md             # USB CDC ASCII 协议契约
├─ docs/                            # 架构、HIL、回归、风险和面试文档
└─ tools/                           # 工具链、测试、固件和桌面构建脚本
~~~

### 3.1 Source of truth 原则

1. 要回答“实际烧录哪个固件”，看 tools/build-firmware.ps1：它进入 firmware-target/Firmware，用 Tup 生成构建脚本，输出 firmware-target/Firmware/build/FOCStudioFirmware.{elf,hex,bin}。
2. firmware/ 是 portable snapshot，适合在 Windows 上测试安全状态机和反馈模式，不能替代完整 STM32 固件，也不会被固件构建链接进 ELF。
3. 产品默认值看 firmware-target/Firmware/MotorControl/main.cpp，持久化配置看 NVM/Fibre property；上位机 safe profile 只是调参辅助，不能当成实际设备配置。
4. ABZ 反馈和诊断格式分别以 firmware-target/Firmware/communication/ascii_protocol.cpp、host/foc-studio/protocol.js 和 protocol/protocol.md 的兼容字段顺序为准。
5. 如果文档、UI 默认值和固件运行时值不一致，优先相信“实际构建和实际烧录代码”，并记录配置漂移风险。

### 3.2 与 ODrive 的关系和差异

面试中的推荐表述是：这是自研控制板和自主维护的 FOC 固件，设计阶段参考了 ODrive 等成熟开源电机控制项目的控制思想、状态机组织方式和工程实践，但没有把项目描述成直接使用 ODrive 成品板或简单修改官方代码。自研部分重点体现在板级硬件适配、产品化参数、安全边界、ABZ 低速算法、通信协议、上位机和测试工具链。

| 对比维度 | 参考项目的通用做法 | FOC Studio 的实现差异 |
|---|---|---|
| 硬件 | 通用多轴电机控制板，板级资源和引脚由平台固定 | 自研单轴控制板，围绕 3505-KV650、12V 母线、DRV8301、ADC 电流采样、PWM 和 USB CDC 重新确定器件、接口和安全参数 |
| 固件定位 | 通用电机控制框架，支持较多电机、编码器和接口组合 | 面向单轴产品 profile 做裁剪和重构，默认参数、校准流程、限流、限扭矩和无感启动策略均按目标电机重新定义 |
| ABZ 速度反馈 | 通用 Encoder PLL/速度估算路径 | ABZ 速度/位置级联环使用 delta_count 驱动的自适应 observer 作为唯一速度 PI 反馈；50/100 ms window 和 M/T 只做诊断与 seed |
| 低速控制 | 通用速度 PI 和基础限幅 | 增加 ABZ 专用 Kp/Ki、smoothstep 低速增益、积分 anti-windup、Coulomb/breakaway 状态机和反向清理 |
| 抗齿槽 | 通用 map/标定能力 | 自主实现双向恒速扫描、样本质量 gate、分周期 map 后处理、平滑、统计和失败保护 |
| 通信 | 通用 property/native 接口为主 | 设计面向调试和生产 bring-up 的 USB CDC ASCII 命令，增加 g 快帧、j 聚合帧、故障分层和 append-only 遥测契约 |
| 上位机 | 依赖通用命令行或通用工具 | 自主实现 Electron/Web Serial FOC Studio，包含校准向导、配置防覆盖、故障历史、实时示波器和模拟设备 |
| 安全策略 | 通用 watchdog、限流和状态机 | 增加产品化急停清理、ABZ 双层超速限定、glitch 诊断、固定位置轨迹限速和上位机控制队列 |
| 构建与测试 | 平台通用构建方式 | 自主维护 ARM/Tup/CMake/Node 构建脚本，增加协议测试、ABZ 算法测试、ARM smoke、固件全量构建和 HIL 回归矩阵 |

面试中可以用一句话总结差异：

> 我参考了成熟的 FOC 工程架构，但真正交付的是自研板卡上的产品化控制系统；相比通用实现，我重点重写了 ABZ 低速反馈、摩擦和抗齿槽补偿、诊断协议、上位机和安全测试边界。

## 4. 总体架构

### 4.1 系统组件图

~~~mermaid
flowchart LR
    UI["FOC Studio Electron / Browser UI"] --> SERIAL["Web Serial / USB CDC"]
    SERIAL --> ASCII["ASCII protocol parser"]
    ASCII --> AXIS["Axis state machine"]
    AXIS --> CTRL["Controller: position / velocity / torque"]
    CTRL --> MOTOR["Motor: torque to Iq, current PI"]
    MOTOR --> FOC["Clarke / Park / inverse Park / SVPWM"]
    FOC --> POWER["DRV8301 + MOSFET + PMSM"]
    POWER --> FEEDBACK["AS5047P / ABZ / sensorless feedback"]
    FEEDBACK --> ENC["Encoder / SensorlessEstimator"]
    ENC --> CTRL
    CTRL --> TELEMETRY["g fast frame / j aggregate frame"]
    TELEMETRY --> UI
~~~

### 4.2 运行时数据流

~~~mermaid
sequenceDiagram
    participant Host as FOC Studio
    participant USB as USB CDC
    participant Axis as Axis task
    participant Enc as Encoder
    participant Ctrl as Controller
    participant Motor as Motor / PWM ISR

    Host->>USB: v 0 target 或 t 0 position
    USB->>Axis: ASCII parser 写入 input/config
    loop 每个电流控制周期
        Axis->>Enc: sample/update encoder or sensorless state
        Enc-->>Ctrl: phase, velocity, delta_count, diagnostics
        Ctrl->>Ctrl: setpoint -> position loop -> velocity PI -> torque
        Ctrl->>Motor: torque setpoint, phase, phase_vel
        Motor->>Motor: torque -> Iq -> current PI -> SVM timings
        Motor-->>Axis: deadline / current / fault status
    end
    Host->>USB: g 0 / j 0
    USB-->>Host: ! fast frame / @ aggregate frame
~~~

### 4.3 线程和实时性分层

| 层 | 典型频率/时机 | 责任 | 实时要求 |
|---|---:|---|---|
| PWM/ADC ISR | 电流控制周期 | 采样电流、触发 ADC、更新 PWM 时序、deadline 保护 | 最高；连续失约必须 disarm |
| Axis 控制循环 | 与电流测量同步，名义 8 kHz | 健康检查、Encoder 更新、Controller 更新、Motor 更新 | 不能阻塞、不能遍历大表 |
| USB/UART 线程 | 异步 | 收命令、解析 ASCII、发送响应 | 不能承担高频控制职责 |
| Host polling | g 目标 50 Hz，j 目标 5 Hz | 诊断、状态渲染、示波器缓存 | 串口命令按单一队列顺序写入 |
| 校准/后处理 | 控制循环中的非阻塞状态机 | 电阻/电感、编码器 offset、齿槽扫描和 map 后处理 | 每周期分摊工作，避免错过 PWM deadline |

核心原则是：USB 命令只能改变目标值、配置或状态请求；真正的闭环控制必须在 Axis 的实时控制循环中完成，不能把控制环放到 Electron 定时器或串口线程。

## 5. FOC 控制链路

### 5.1 FOC 的基本流程

对三相 PMSM，固件将三相采样电流转换到静止坐标系，再转换到转子坐标系：

~~~text
Ia/Ib/Ic
   │
   ├─ Clarke: Ialpha = -Ib - Ic
   │           Ibeta  = (Ib - Ic) / sqrt(3)
   │
   ├─ Park:   Id = cos(theta) * Ialpha + sin(theta) * Ibeta
   │           Iq = cos(theta) * Ibeta  - sin(theta) * Ialpha
   │
   ├─ d/q current PI: Id/Iq error -> Vd/Vq
   │
   ├─ inverse Park: Vd/Vq -> Valpha/Vbeta
   │
   └─ SVPWM: Valpha/Vbeta -> 三相 PWM compare timings
~~~

Motor::update() 负责把上层 torque setpoint 转成 Iq 给定。对 PMSM 近似为：

~~~text
Iq_setpoint = torque_setpoint / torque_constant
~~~

然后根据电机方向、有效电流限制和电机类型做符号处理与限幅。Motor::FOC_current() 读取 ADC 电流，计算 Id/Iq，运行 d/q 电流 PI，处理调制向量饱和，并调用 enqueue_modulation_timings() 写入 PWM 时序。

### 5.2 电流 PI 的抗饱和

当 Vd/Vq 归一化后的调制向量超过可用调制范围时，代码会缩放调制量，并让电流 PI 积分项按 0.99 衰减；未饱和时才正常累加 d/q 积分。这样可以降低母线电压不足、目标电流过大时的积分堆积。

电流环的保护包括：

- ADC 电流采样超范围或 current sense saturation 时设置 motor error；
- 实际电流超过 effective_current_lim + current_lim_margin 时退出；
- 三相/母线/DRV8301/温度错误通过 Axis 健康检查汇聚；
- 控制循环未按时产生 PWM timings 时触发 deadline 保护并关 PWM；
- Motor::arm() 会清 Controller 状态和电流环 d/q 积分，先排队一个零电压 SVM 周期，再使能 PWM。

### 5.3 为什么低速问题不应该优先改电流环

ABZ 低速问题的根因首先在机械速度估算和外环反馈：4000 CPR、8 kHz、1 turn/s 时每周期平均 0.5 count，0.1 turn/s 平均 2.5 ms 才来一个 count。若把这个量化噪声直接送入速度 PI，外环会看到 0 与非 0 交替的速度信号，可能产生 torque hunting。

因此本项目把优化边界限制在：

- ABZ 机械速度 observer；
- 50/100 ms 窗口和 M/T 诊断；
- ABZ 外环的 Kp/Ki、积分限幅、摩擦/齿槽补偿；
- 遥测和测试。

没有为了调低速而修改 Clarke/Park、SVPWM、电流 PI、换相和 sensorless 主路径。这样做的好处是故障定位范围小、回归风险低，也能保证 SPI、扭矩模式和无感路径不被 ABZ 调参污染。

## 6. 反馈模式与控制模式

### 6.1 反馈模式

| 编号 | 模式 | 角度/速度来源 | 适用场景 |
|---:|---|---|---|
| 0 | SPI | AS5047P 绝对角度，16384 CPR | 绝对位置、常规闭环 |
| 1 | ABZ | 增量编码器，4000 CPR | 低速机械速度、位置/速度闭环 |
| 2 | Sensorless | 无感 observer/PLL | 不接编码器，必须先开环爬升 |
| 3 | Sensorless + SPI monitor | 控制仍无感，SPI 只监视角度误差 | 无感启动和编码器一致性诊断 |
| 4 | Sensorless + ABZ monitor | 控制仍无感，ABZ 只监视角度误差 | 无感与增量编码器对照 |

模式切换只允许在 Idle 状态执行。上位机在 UI 层禁用非 Idle 切换，固件 m axis mode 也会再次校验，不能只依赖前端。

### 6.2 控制模式

| 模式 | 输入 | 输出关系 |
|---|---|---|
| 扭矩 | torque setpoint | 直接进入 torque→Iq→电流环，速度限制可作为额外保护 |
| 速度 | velocity setpoint | v_err = vel_des - velocity_feedback，经 P/I/FF 得 torque |
| 位置 | position setpoint | vel_des = vel_setpoint + position_gain * position_error，随后进入速度环 |

输入模式还包括 PASSTHROUGH、VEL_RAMP、TORQUE_RAMP、位置二阶滤波和梯形轨迹。FOC Studio 的位置按钮使用 t axis position，固件固定使用 0.30 turn/s 速度和 0.60 turn/s² 加减速，避免旧 NVM 的轨迹参数导致调试时突然高速移动。

### 6.3 纯无感模式为什么不能从 0.1 turn/s 直接启动

无感估算依赖反电动势/磁链观测，静止或极低速时信号太弱。当前上位机将无感速度目标种到约 6 turn/s，固件通过 lock-in/open-loop ramp 以 1.5 A、100 electrical-rad/s² 的参数爬升，锁定后才进入 sensorless closed loop；无感模式不允许位置控制。

## 7. ABZ 低速测速架构

### 7.1 为什么需要多套估算器，但只能有一个闭环反馈

不同估算器的时间尺度和职责不同：

| 估算量 | 输入 | 主要职责 | 是否进入 ABZ 速度 PI |
|---|---|---|---|
| Encoder PLL vel_estimate | count_in_cpr 相位误差 | 电角度插值、commutation、原始速度和应急监视 | 否 |
| 50 ms window | 最近 400 个 8 kHz delta_count | 快速机械诊断、observer 初始化、标定门限 | 否 |
| 100 ms window | 最近 800 个 8 kHz delta_count | 稳态机械速度参考 | 否 |
| M/T | 编码器边沿和边沿间隔 | 低速边沿诊断、稀疏计数分析 | 否 |
| ABZ control observer | 每周期 delta_count / CPR | 平滑速度、ABZ 速度 PI 唯一反馈 | 是 |

如果让多个估算器动态切换成闭环反馈，切换点会带来反馈阶跃、相位变化和积分冲击。因此当前设计采用“单一控制反馈 + 多路诊断”的职责分离。

### 7.2 增量 observer 的算法

控制循环每周期从 Encoder 得到 last_delta_count_，以 delta_position_turn = last_delta_count / CPR 作为 observer 输入。observer 内部维护一个有限的本地位置 frame：

~~~text
pos_meas += delta_position
pos_hat  += dt * vel_hat
innovation = pos_meas - pos_hat
pos_hat   += dt * kp * innovation
vel_hat   += dt * ki * innovation
~~~

observer 使用临界阻尼位置 PLL 形式：

~~~text
omega_n = 2π * bandwidth
kp      = 2 * omega_n
ki      = 0.25 * kp²
~~~

当本地位置绝对值超过 8 turn 时，同时平移 pos_meas 和 pos_hat，保持 innovation 不变。这一步解决两个长期运行问题：

1. 不再让 observer 依赖 int32 shadow_count_ 的持续增长；
2. 不让绝对 float 位置变得过大后丢失单个编码器 count 的分辨率。

注意边界：Encoder 原始 PLL 和部分绝对位置路径仍是基础控制框架中的历史路径，本文不声称整个 Encoder 子系统的长期计数技术债都已经消失；这里只保证 ABZ control observer 这条闭环路径使用增量输入和 bounded local frame。

### 7.3 自适应带宽

带宽由指令速度的绝对值决定，而不是由含噪测量速度决定，避免 estimator bandwidth 被噪声反复抖动。

| abs(command_velocity) turn/s | observer 带宽 |
|---:|---:|
| 0 | 15 Hz |
| 0.3 | 约 20 Hz |
| 1.0 | 约 30 Hz |
| 2.0 | 约 40 Hz |
| 4.0 及以上 | 50 Hz |

中间使用分段线性插值，并夹在运行时配置的 min/max 之间。只有带宽变化超过 0.5 Hz 时才重算 2π、乘法和平方增益，稳态 8 kHz 热路径只做调度和 observer update。

设计权衡：低速需要低带宽抑制 count quantization，但会增加响应延迟；高速提高带宽以跟踪真实机械变化。面试中应说明这是“噪声—响应速度”的可解释折中，不是越低越稳或越高越好。

### 7.4 observer 初始化避免切模式 torque impulse

进入 ABZ 速度/位置级联控制时，observer 不从 0 速度盲启动，而使用同一个 seed helper 初始化：

~~~text
50 ms window valid
    -> M/T valid
    -> encoder PLL valid
    -> 0
~~~

速度模式切入时，vel_setpoint_ 也会用同优先级的当前速度初始化。因此旋转中的 Idle→速度模式切换不会因为“设定值已经是当前速度、observer 却从 0 起步”而瞬间产生巨大的速度误差和 P torque。

### 7.5 双滑动窗口的 O(1) 实现

在名义 8 kHz 下：

- 50 ms = 400 samples；
- 100 ms = 800 samples；
- 两个窗口共用一个 800-slot ring buffer，约 3.2 KiB（int32_t）；
- 使用 int64_t sum50/sum100 保存窗口和；
- 每周期只做“加新样本、减离开窗口样本”，时间复杂度 O(1)；
- 不在控制热路径动态分配内存，也不每周期清空大数组。

窗口速度公式：

~~~text
velocity = sum(delta_count) / (CPR * sample_count * current_meas_period)
~~~

4000 CPR 下，50 ms 分辨率约 0.005 turn/s，100 ms 分辨率约 0.0025 turn/s；它们适合诊断和稳定参考，但 100 ms 延迟过大，不适合直接闭速度环。

### 7.6 M/T 估算器

M/T 通过编码器边沿之间的时间间隔估计低速速度。当前代码对它做了两点工程化处理：

- 输出有 slew limit，避免单个 stray count 变成巨大速度 spike；
- 零速时，如果仍在预期边沿间隔内，保持上一个估计；超过超时后再平滑衰减到 0，避免 0 → 非 0 → 0 闪烁。

M/T 不进入电角度、不进入 ABZ 速度 PI，只用于示波器对照和低速边沿质量分析。

### 7.7 ABZ 速度 PI、低速增益和 anti-windup

在 ABZ 速度/位置级联模式下，控制器计算：

~~~text
v_err = vel_des - control_observer_velocity
P     = effective_Kp * v_err
I     = vel_integrator_torque
FF    = anticogging_torque + friction_torque + torque_feed_forward
T_raw = torque_setpoint + P + I + FF
~~~

ABZ 速度模式使用独立的 abz_vel_gain 和 abz_vel_integrator_gain，不会误用通用 velocity 参数。低速区间 [0.5, 1.2] turn/s 使用 smoothstep 在 base gains 与 low-speed gains 之间连续混合，避免硬切换造成 torque step。

积分 anti-windup 采用条件积分：

- 如果 torque 已在上限且速度误差仍推动输出往上，暂停积分；
- 如果误差反向，立即允许积分释放；
- 积分值再限制在 ABZ torque limit 和 motor global torque limit 的最小值内；
- 退出速度环或切换扭矩模式时清零积分。

### 7.8 双层超速限定

超速检测和速度 PI 是两个独立职责：

| 层 | 判据 | 目的 |
|---|---|---|
| 正常层 | control observer 超过 vel_limit * vel_limit_tolerance | 检测控制器真实使用的反馈速度 |
| 应急层 | raw PLL 或 50 ms window 超过正常限的 2 倍 | 防止低带宽 observer 还未跟上时真实 runaway 被漏掉 |

两层共享一个 16-cycle consecutive qualifier。单个 raw PLL spike 不会马上停 PWM；连续 16 个控制周期才锁存 ERROR_OVERSPEED。名义 8 kHz 下约为 2 ms qualification window。非 ABZ 模式保持原有 raw PLL 对正常限的判断逻辑。

### 7.9 ABZ 计数 glitch 诊断

每周期根据 max(command speed, observer speed, window50 speed) 推导物理可接受的 delta_count 上界。超过约 3 倍上界时增加 abz_count_glitch_count_，但不直接 fault：

- 真实快速拖动、减速或超调可能让瞬时速度高于命令，因此用 observer/window 扩大 envelope；
- raw PLL 不作为 envelope，因为它正是可能产生 spike 的信号；
- glitch counter 是定位线缆、AB 相序、计数器采样和信号完整性问题的诊断指标，不是安全停机判据。

## 8. 低速摩擦/脱困补偿

### 8.1 为什么不直接把 Kp 调大

如果低速转不动，直接提高 Kp 会同时放大测速量化噪声，导致“卡住—积累—突然突破—反向修正”的 limit cycle。摩擦前馈更适合表达与速度误差无关的机械阻力，Kp/Ki 仍保持可控。

### 8.2 FrictionCompensator 状态机

~~~text
IDLE
  │ 非零 ABZ velocity command
  ▼
RUNNING  ──持续误差 + 无进展达到确认时间──>  BREAKAWAY
  ▲                                              │
  │ 速度达到目标比例并持续确认                   │ 足够 encoder progress + speed ratio
  │                                              ▼
  └────────────── RECOVERING <──────────────────┘
                       │ 再次 stall
                       └──────────────> BREAKAWAY
~~~

主要逻辑：

- RUNNING 阶段提供 Coulomb friction 和平滑的低速 holding assist；
- 速度命令超过阈值、速度误差持续为正且若干时间无有效 forward progress 时进入 BREAKAWAY；
- BREAKAWAY 输出逐步上升到 breakaway torque，不做单周期阶跃；
- 退出必须同时满足方向一致的 encoder 进展、速度比例和持续确认时间；不能只凭第一个 count 退出；
- RECOVERING 先逐步释放附加 torque，若又 stall 则重新进入 BREAKAWAY；
- 反向命令会清理方向相关状态，不能复用正向补偿；
- 命令归零、模式退出、急停或 fault 时，正常禁用路径是 ramp-to-zero，安全清理路径是 hard clear。

补偿只在“ABZ 增量 + 速度控制”生效，位置、SPI、sensorless 和扭矩模式不应被这套低速补偿改变。抗齿槽速度扫描期间会主动关闭 friction assist，避免把补偿状态变化采进位置同步 map。

## 9. 抗齿槽标定与前馈

### 9.1 为什么做双向扫描

单向扫描采到的 torque 同时包含齿槽转矩、库仑摩擦、方向偏置和速度环瞬态。正反向各扫描，再对同一位置 bin 的结果求平均，可以显著抵消方向相关摩擦，保留位置相关的 cogging 分量。

### 9.2 当前标定状态机

命令 b axis 只允许 ABZ 模式、Idle、无 fault、motor calibrated 且 encoder ready 时执行。当前实现使用：

~~~text
RAMP_FORWARD
  -> FORWARD
  -> RAMP_REVERSE
  -> REVERSE
  -> FINALIZE
  -> SMOOTH
  -> STATS
  -> VALIDATE
  -> COMPLETE / FAILED
~~~

默认参数是 2 turn/s、每个方向 10 turns、0.25 s steady-speed dwell、15 s motion phase timeout。代码会按配置值执行，不能把历史文档中的旧圈数当成当前固件事实。

### 9.3 采样质量门限

一个位置 bin 只有在以下条件满足时才采样：

- 当前方向的 50 ms mechanical window 速度接近扫描目标；
- observer 与 50 ms window 不存在 gross disagreement；
- 非扫描路径不能处于 reverse 或非 RUNNING 摩擦状态；
- ABZ torque 或 motor torque 没有饱和；
- 每个 unwrapped position bin 只采样一次，避免同一 bin 在多个 tick 重复计权。

正向样本和反向样本分别保存，后处理阶段求平均，缺少任一方向的 bin 置零。3600-bin map 的 finalize、平滑和统计每周期只处理配置数量的 bins，避免一次遍历 3600 项拖过 8 kHz 控制截止时间。

### 9.4 Map quality gate

最终 map 会计算 coverage、mean、RMS、peak-to-peak、最大相邻跳变、首尾 wrap jump 和最大绝对值，只有满足覆盖率、幅值和连续性门限才设置 anticogging.pre_calibrated = true。失败会进入 FAILED 并回 Idle，避免半成品 map 被启用。

运行时应用 map 时：

- 3600-bin 位置使用相邻 bin 线性插值；
- 减去 map mean，避免引入恒定偏置；
- 按 anticogging_torque_limit 限制补偿幅值；
- ABZ 低速扫描产生的 map 只在 ABZ velocity control 应用；
- 低速使用 polarity/boost/fade schedule，且从 0 速度平滑 blend in，避免第一拍 torque step。

## 10. Axis 状态机与安全设计

### 10.1 典型状态流

~~~text
StartupSequence
    ├─ MotorCalibration
    ├─ EncoderIndexSearch（如启用）
    ├─ EncoderOffsetCalibration
    └─ Idle

Idle
    ├─ ClosedLoopControl
    ├─ SensorlessControl
    ├─ FullCalibrationSequence
    └─ 仍保持 PWM 关闭

ClosedLoopControl / SensorlessControl
    ├─ 每周期健康检查 + estimator update + controller + motor
    ├─ watchdog 超时 / fault / deadline miss -> disarm -> Idle
    └─ x axis -> 立即清状态、关 PWM、请求 Idle
~~~

Axis 的 run_control_loop() 每周期做：健康检查、Encoder/无感 estimator 更新、watchdog check、Controller update、Motor update，然后等待下一次 current measurement interrupt 并检查 PWM deadline。

如果 handler 返回失败、健康检查失败、current measurement timeout 或 PWM deadline 失败，最终都进入安全停机路径：关闭 PWM、清 brake current、回 Idle 并锁存相应故障。

### 10.2 急停路径

x axis 的处理顺序是：

1. 恢复 FOC Studio 临时速度/位置/扭矩安全参数；
2. 清零 input torque、input velocity，并把 input position 归到当前位置；
3. Controller::reset()，清速度积分、observer 状态、摩擦补偿和诊断状态；
4. Motor::reset_current_control()，清 d/q 电流 PI 积分、Iq/Id setpoint 和报告值；
5. safety_critical_disarm_motor_pwm()；
6. 请求 AXIS_STATE_IDLE。

这样做是为了防止“急停后重新 arm，旧轨迹、旧积分或旧 breakaway 状态被复用”。Motor::arm() 也会再次重置 Controller 和 current control，形成二次防线。

### 10.3 Watchdog 的真实行为和已知风险

默认运行 watchdog 为 1 s；Idle 状态不会因为 watchdog 自动制造故障。FOC Studio 每 20 ms 调度一次轮询，每 250 ms 发送 u 0 heartbeat，同时发送 g 0，每 200 ms 发送一次 j 0。

当前固件的 g 和 j 分支也会调用 Axis::watchdog_feed()。这意味着在“只读遥测仍持续、显式控制 heartbeat 已停止”的异常场景下，telemetry 可能维持控制状态。它是当前代码已知的 P1 级工程风险，后续更理想的设计是：

- 只有明确的控制命令或 u heartbeat 能喂 control watchdog；
- g/j 只读，不改变控制生存期；
- 增加兼容迁移期和 timeout HIL 测试。

面试中不要把“watchdog 存在”直接说成“所有通信都能安全超时”；应主动说明当前实现和改进方案。

## 11. USB CDC ASCII 协议

### 11.1 命令分类

| 命令 | 示例 | 作用/限制 |
|---|---|---|
| m | m 0 1 | 切 ABZ；只允许 Idle |
| g | g 0 | 快帧遥测；目标 50 Hz；当前会 feed watchdog |
| j | j 0 | 聚合状态/故障遥测；目标 5 Hz；当前会 feed watchdog |
| x | x 0 | 急停：清状态、关 PWM、请求 Idle |
| k | k 0 | 清故障；只允许 Idle |
| a | a 0 | 按当前反馈模式执行校准；只允许 Idle |
| b | b 0 | ABZ 双向抗齿槽标定；只允许 ready 且 Idle |
| p | p 0 1.2 | 位置控制，可带速度/扭矩前馈 |
| q | q 0 1.2 0.3 0.02 | 位置控制并临时设置速度/扭矩限幅 |
| v | v 0 0.5 0 | 速度控制，可带 torque feed-forward |
| c | c 0 0.01 | 扭矩控制 |
| t | t 0 1.2 | 固定安全梯形轨迹移动到绝对位置 |
| f | f 0 | 读取位置/速度 |
| r | r axis0.error | 读取 Fibre property |
| w | w axis0.controller.config.abz_vel_gain 0.002 | 写 Fibre property |
| u | u 0 | 显式喂 watchdog |
| ss | ss | 保存配置；要求所有轴 Idle，且恢复速度临时参数 |

### 11.2 帧格式和兼容策略

@ 是聚合状态帧，前 10 个字段保持旧兼容顺序，后面追加 motor/encoder/controller fault、armed state、校准状态、三相指令电压、Id/Iq 和抗齿槽统计。

! 是快帧，字段无名字、按位置解析；当前代码是 append-only 设计，新字段追加在末尾，旧字段位置不变。主要诊断字段包括：

- 控制反馈速度、raw PLL、50 ms/100 ms window、M/T、observer velocity；
- velocitySetpoint、velocityError、P torque、I torque、low-speed torque、final torque；
- ABZ torque limit 前后值和饱和标志；
- last_delta_count、shadow_count、glitch count、observer bandwidth、estimator disagreement；
- friction state、target、continuous/breakaway torque、recovery timer；
- anticogging phase/progress/scan velocity/quality state。

固件 respond() 使用 1024 字节缓冲，host protocol test 对 worst-case fast frame 长度做精确边界验证；如果 snprintf 不满足长度，固件丢弃整帧而不是发送截断帧，避免 host parser 解析半条记录。

### 11.3 协议设计中的工程取舍

- 选择 ASCII 而不是自定义二进制：开发阶段可用串口终端直接排查，协议可读性强，也便于兼容既有 property 命令；
- 代价是带宽和解析效率较差，因此只把低频遥测放在 USB，8 kHz 控制完全在 MCU 内部；
- 快帧不携带字段名以降低长度，但依赖字段顺序稳定，所以采用 append-only 兼容规则并在 host 测试中锁定映射；
- 上位机所有串口写入进入同一个 Promise queue，避免 heartbeat、控制命令、配置写入交叉导致设备端命令顺序异常。

## 12. Electron 上位机设计

### 12.1 进程边界

| 进程 | 实现 | 责任 |
|---|---|---|
| Electron main | desktop/main.cjs | 创建窗口、申请 serial 权限、弹出 USB CDC 设备选择 |
| preload | desktop/preload.cjs | 只暴露 window.focDesktop.isDesktop，不暴露 Node 能力 |
| renderer | app.js | Web Serial 读写、命令队列、遥测解析、UI、示波器、模拟设备 |
| protocol module | protocol.js | 常量、命令 builder、fault decoder、fast/aggregate parser、LineParser |

Electron 配置为 contextIsolation: true、nodeIntegration: false，主进程不把任意 Node API 注入渲染页；Web Serial 的具体读写仍走浏览器标准 API。

### 12.2 连接和轮询

1. 用户选择串口后以 115200 打开；
2. 获取 readable/writable stream reader/writer；
3. readSerialLoop() 将字节流交给 LineParser，处理分包和多行；
4. 写入通过 serialWriteQueue 串行化；
5. 每 20 ms 的调度器先按需发送 u，再发送 g；
6. 每 200 ms 发送一次 j；
7. 快帧进入 history，用于实时示波器、光标、窗口缩放、自动量程和 min/max/average/peak-to-peak 统计；
8. 连接断开时 cancel reader、release locks、关闭 port，并提示 watchdog 将停止电机。

### 12.3 配置写入防误操作

配置页不是简单把 HTML 默认值全部写到设备：

- 必须先逐项 r 读取设备值；
- 保存前只筛选 value != deviceValue 的可编辑项；
- 只允许 Idle 写配置；
- 方向、极对数、CPR、电阻、电感和 pre-calibrated 等校准敏感项不参与批量写入；
- 前端还对电流环带宽、速度增益、制动电阻和回馈电流做范围校验；
- 写入成功后提示“还需要保存到 Flash”，区分 RAM 配置和 NVM 配置。

### 12.4 模拟设备

上位机保留模拟设备模式，可在没有控制板时演示连接、校准向导、故障状态、配置页面和示波器交互。它用于 UI/协议联调，不应被描述为电机控制仿真或 HIL。

## 13. 构建、烧录和发布

### 13.1 固件构建

~~~powershell
cd <repository-root>
powershell -ExecutionPolicy Bypass -File tools\bootstrap-toolchain.ps1
powershell -ExecutionPolicy Bypass -File tools\build-firmware.ps1
~~~

构建脚本会：

1. 加载 Arm GNU 和 Tup 环境；
2. 在确认路径为 firmware-target/Firmware/build 后清理旧 build；
3. 动态注入 Python 路径，生成 Fibre 接口文件；
4. 用 tup generate 生成离线 build-firmware.bat；
5. 在 Tup 注入环境之外运行 ARM 编译器，规避 Windows DLL 注入导致的编译器崩溃；
6. 检查 ELF/HEX/BIN 均存在，并输出 arm-none-eabi-size 和 objdump 信息。

Tup 配置关键项：

~~~text
CONFIG_BOARD_VERSION=v3.6-56V
CONFIG_USB_PROTOCOL=native
CONFIG_UART_PROTOCOL=ascii
CONFIG_DEBUG=false
CONFIG_DOCTEST=false
~~~

其中 v3.6-56V 是现有板级适配 profile 的配置标识，不作为面试中的硬件来源描述；硬件口径统一表述为自研控制板。

### 13.2 桌面版构建

~~~powershell
cd <repository-root>
powershell -ExecutionPolicy Bypass -File tools\build-desktop.ps1
~~~

脚本在 host/foc-studio 执行 npm ci 和 npm run desktop:dist，Electron Builder 生成 Windows portable 包，产物位于 host/foc-studio/dist。

### 13.3 烧录和 DFU 的安全原则

项目脚本只负责编译和准备产物，不在构建后自动给电机上电，也不应把“编译成功”当成“硬件安全可运行”。烧录/上电前要确认：

- 电机固定、三相线和编码器线无误；
- 母线为 12 V 调试条件；
- 初始电流限制 2 A、扭矩限制 0.2 Nm；
- 编码器模式和 CPR 与实际硬件一致；
- 手边有急停；
- 先 Idle、再校准、再小速度测试；
- 首次 HIL 不用提高 current/torque/integrator 来掩盖问题。

## 14. 测试体系和当前证据

### 14.1 自动化测试分层

| 层级 | 命令/位置 | 覆盖内容 |
|---|---|---|
| Host protocol | npm.cmd test / host/foc-studio/test_protocol.mjs | 命令编码、故障码、@/! 字段解析、行解析、快帧长度边界 |
| ABZ estimator | firmware-target/Firmware/Tests/test_velocity_estimators.cpp | observer、window、M/T、低速稀疏计数、反转、停止、slew、积分 anti-windup |
| Portable core | cmake -S firmware -B firmware/build-host + CTest | 反馈模式状态、安全故障状态机 |
| ARM smoke | tools/test-project.ps1 | portable core 以 Cortex-M4 freestanding 方式编译 |
| 固件全量 | tools/build-firmware.ps1 | 真实 target、HAL、FreeRTOS、USB、Fibre、MotorControl 全链路编译/链接 |
| HIL | docs/HIL_TEST_PLAN.md | 实际电机速度、Iq、torque、超速、急停、堵转、释放和通信故障 |

### 14.2 本次实际验证结果

本次在当前工作区执行：

~~~powershell
powershell -ExecutionPolicy Bypass -File tools\test-project.ps1
powershell -ExecutionPolicy Bypass -File tools\build-firmware.ps1
~~~

结果：

- Host ASCII protocol：通过；
- ABZ velocity estimator：135 checks，0 failures；
- portable core CTest：1/1 passed；
- ARM feedback/safety smoke compile：通过；
- 固件 Tup 全量构建：通过；
- ELF size：text 271,924 B，data 1,620 B，bss 136,568 B，total 410,112 B；
- 固件入口地址：0x0800C29D；
- ELF SHA-256：325891F12AC158D63AA9FBB163387B52670339F1A4956B4F6937FDF0472977D7；
- HEX SHA-256：A3A553B0E20507A71E0A695CCAC0D32FD2A59A3DBD3531C994CB8AFD9E78F896；
- BIN SHA-256：3720BCD0B17DF6680F7DA7B48562C5AC85BFCB07192648FD2F603FA2A7B952A2。

这些结果证明当前代码能通过自动化构建和算法级测试，但不证明“任意电机、任意负载、任意参数下低速稳定”。HIL 仍需要采集原始 CSV、固件 hash、NVM 备份、负载和接线说明。

### 14.3 HIL 重点指标

每个速度点同时记录：

- command velocity；
- control observer velocity；
- raw PLL、50 ms、100 ms、M/T；
- observer bandwidth 和 estimator disagreement；
- Iq setpoint / Iq measured；
- P torque、I torque、friction torque、anticogging torque、final torque；
- ABZ torque limit 前后值和 saturation；
- encoder delta/shadow count、glitch count；
- state、position error、friction state、anticogging state。

建议速度场景：正负 0.2/0.5/1.0/1.5/2.0 turn/s，以及 0→0.2、0.2→0、0.2→-0.2、1→2、2→1、堵转/释放、位置误差跨越 2/4 counts、急停和 watchdog 场景。

## 15. 性能、内存和实时性分析

### 15.1 热路径约束

不能在 8 kHz 控制循环中做：

- 动态内存分配；
- 阻塞式 USB/串口 IO；
- 一次性遍历 3600-bin map；
- 每 tick 重算不变的三角函数/observer 增益；
- 大规模清零或不可预测的复杂度操作。

项目中的对应设计：

- window 使用固定 ring buffer 和 O(1) 更新；
- observer gain 使用 bandwidth cache，变化超过阈值才重算；
- 抗齿槽 finalize/smooth/stats 分摊到多个 control cycle；
- telemetry 只在通信线程读取轻量成员，不在 ISR 中做高频日志；
- 上位机 50 Hz 采样不能替代高速示波器，也不能从相电压遥测推断高速真实相电压。

### 15.2 当前固件资源结果

本次实际 ELF 的静态统计：

~~~text
text = 271,924 B
data =   1,620 B
bss  = 136,568 B
dec  = 410,112 B
~~~

bss 中包含控制对象、FreeRTOS、基础运行时结构和抗齿槽数组等运行时状态，面试中如果被问到内存，应先说明这里是整个固件的 ELF 统计，不要把所有 bss 归因于 ABZ window。

### 15.3 复杂度回答模板

> ABZ window 是每周期 O(1)，只更新两个 int64 sum 和固定 ring slot；observer 是固定数量浮点运算；friction 状态机也是 O(1)；抗齿槽采样是 O(1)，3600-bin 后处理按每周期 1/2/4/8 bins 分摊。因此热路径没有动态分配和一次性大循环，实时风险主要通过控制 deadline、单元测试和实际 size/build 结果验证。

## 16. 安全性、可靠性和错误处理

### 16.1 错误分层

~~~text
Encoder / Motor / Controller / Sensorless / Thermistor / DRV
                         │
                         ▼
                      Axis error
                         │
                         ▼
                 disarm / Idle / host @ frame
~~~

上位机不会只显示一个总 fault，而是解析 axis、motor、encoder、controller、sensorless 等多个 bitmask，给出故障摘要和建议。例如 controller overspeed、encoder no response、DRV fault、current sense saturation 和 watchdog timeout 会分别显示。

### 16.2 输入校验

- mode、axis、float 参数都在 parser 层校验；
- 模式切换、清故障、校准、保存配置有 Idle/ready 限制；
- property 写入由 Fibre type info 做类型转换，非法属性会返回 error；
- 上位机对配置范围做二次校验，但固件不能依赖前端校验；
- 无效估算器、NaN/Inf、非法输入模式会设置 controller error 并退出控制循环。

### 16.3 失败模式与处理

| 失败模式 | 处理 |
|---|---|
| USB 断开 | host 释放串口；watchdog 期望使电机回 Idle；实际当前 g/j feed 风险需 HIL 验证 |
| 单个编码器 spike | 不立即停机；计入 glitch/由 16-cycle 超速 qualifier 过滤 |
| 持续超速 | 锁存 ERROR_OVERSPEED，退出闭环并 disarm |
| 电流采样超限 | Motor error，关闭 PWM |
| PWM deadline miss | 安全关 PWM，避免持续错误调制 |
| 抗齿槽样本不足/质量失败 | map 不标记 pre-calibrated，不启用半成品 |
| 模式退出/急停 | 清 observer、I、friction、trajectory 和 current PI transient state |

## 17. 已知问题和诚实边界

面试中技术可信度很大程度来自于能否主动说清楚边界。当前项目至少要说明：

1. watchdog 与 telemetry feed 耦合：g/j 会 feed watchdog；上位机另有 u heartbeat。理想方案应分离 control watchdog 和 telemetry activity。
2. 配置来源有重复：main.cpp 的实际固件默认值与 portable firmware/include/foc_config.hpp 存在重复，曾有电流上限不一致风险。后续应使用共享生成配置或构建时一致性检查。
3. 完整 Encoder 原始路径的长期计数技术债：ABZ control observer 已改为 delta + local rebase，但基础 shadow_count_、绝对 float position 和 Encoder PLL 路径仍需独立评估，不能宣称全链路长期溢出问题已解决。
4. 固件 doctest 尚未完全纳入根测试脚本：当前根脚本覆盖 host protocol、ABZ estimator、portable core 和 ARM smoke；Firmware/Tests 的完整 doctest runner 仍应进一步接入 CI。
5. HIL 仍是必要证据：自动化测试只能证明算法边界、协议和构建；摩擦、负载、供电、编码器安装和线缆信号影响必须通过实机数据判断。
6. 三相电压遥测不是 ADC 实测电压：phase_a/b/c_voltage 是由最终 v_alpha/v_beta 反 Clarke 得到的 PWM 指令相电压，V3.6 没有三相电压 ADC，高速下不能替代差分探头。

## 18. 常见问题回答卡片（INTERVIEW_DERIVED）

### Q1：请介绍这个项目的整体架构。

答：这是一个三层系统：STM32 固件负责实时 FOC 和安全状态机；USB CDC ASCII 协议负责可调试的命令和遥测；Electron 上位机负责连接、配置、控制和示波器。控制链路从 Axis 状态机进入 Controller，再进入 Motor 电流环和 SVPWM，Encoder/无感 estimator 作为反馈闭环回 Controller。上位机不能替代实时控制循环。

### Q2：你为什么选择 FOC？

答：FOC 把三相交流电流转换到转子 d/q 坐标系，使磁链和转矩近似解耦，可以分别控制 Id/Iq。相比六步换相，低速平滑性、动态响应和扭矩控制能力更好，适合需要速度/位置闭环和低速诊断的系统。

### Q3：FOC 中 Clarke、Park、SVPWM 分别做什么？

答：Clarke 把三相量投影到两相静止坐标系，Park 再按电角度旋转到 d/q 坐标系；电流 PI 在 d/q 中运行；逆 Park 回到静止坐标系，SVPWM 把电压矢量转成三相 PWM compare。项目中 Motor::FOC_current() 负责这条链路。

### Q4：为什么 ABZ 低速测速难？

答：低速时编码器计数稀疏，而控制周期很快。4000 CPR、8 kHz、1 turn/s 时每周期平均 0.5 count，直接用单周期 delta 会在 0 和非 0 之间跳；0.1 turn/s 更明显。这个噪声如果进入速度 PI 会造成 torque hunting。

### Q5：为什么不用 50 ms window 直接做速度 PI 反馈？

答：50 ms 量化和延迟比单周期好，但仍然是窗口输出；100 ms 更稳但延迟更大。速度环需要连续的动态反馈，所以项目用 delta-driven observer 做唯一 PI feedback，窗口只作为诊断、observer seed 和标定 gate。这样避免多 estimator handover。

### Q6：observer 如何解决 int32 溢出和 float 精度问题？

答：它不再把持续增长的 shadow_count / CPR 作为位置输入，而是每周期输入 delta_count / CPR，内部累加到小范围 local frame；超过 8 turn 时同时 rebase measured/estimated position，保持 innovation 不变。这样 ABZ 控制 observer 不依赖 shadow_count 长期增长，也不让 float 绝对位置变大到丢失单 count 精度。

### Q7：observer 带宽为什么随指令速度变化？

答：低速主要问题是量化噪声，应降低带宽；高速主要问题是跟踪延迟，应提高带宽。带宽按指令速度平滑插值到约 15～50 Hz，使用指令而非测量值避免噪声导致 bandwidth chatter，且 gain 只在变化超过 0.5 Hz 时重算。

### Q8：为什么 observer 的初始速度要从 window/M/T/PLL seed？

答：如果电机已经在转，模式切换时 setpoint 是当前速度，但 observer 从 0 开始，第一拍会出现很大速度误差和 P torque。使用同一个 seed 优先级初始化 observer 和 setpoint，可以避免切换瞬态。

### Q9：如何区分“测速器有问题”和“机械真的在抖”？

答：同时看 control observer、50 ms、100 ms、raw PLL、M/T。只有 observer 抖而 window100 稳，倾向 estimator/observer 问题；几路都抖，倾向机械/负载；window100 稳、window50 有周期 ripple，可能是高频机械 ripple/cogging；所有速度抖且 final torque 反向大幅变化，怀疑 speed-loop hunting；glitch count 增长则检查 ABZ 信号完整性。

### Q10：为什么要有双层超速检测？

答：正常层看 observer，和控制器实际使用的速度反馈一致；应急层看 raw PLL/50 ms window 的更高阈值，防止低带宽 observer 还没跟上时真实 runaway 被漏掉。两层都需要连续 16 个周期确认，过滤单点 spike，持续违规则 fault。

### Q11：为什么低速不直接提高 Kp？

答：Kp 同时放大速度误差和测速噪声，可能把量化噪声转成转矩振荡。项目用独立 ABZ Kp/Ki、smoothstep 低速 gain schedule、摩擦前馈和 breakaway 状态机，把机械阻力和控制误差分开处理。

### Q12：摩擦补偿如何避免突然加一大坨 torque？

答：补偿有 IDLE/RUNNING/BREAKAWAY/RECOVERING 状态，进入 breakaway 需要持续命令、正向误差和无进展确认；输出通过 rise/release slew 限制。退出还要求 encoder progress、速度比例和持续时间，不能只凭一个 count。命令归零时 ramp-to-zero，急停时 hard clear。

### Q13：抗齿槽为什么必须正反向扫描？

答：单向样本混有库仑摩擦和速度环瞬态。正向/反向分别采样后按位置 bin 平均，方向相关摩擦会互相抵消，位置同步的 cogging torque 更容易保留下来。当前还要求速度稳定、无 estimator gross disagreement、无 torque saturation，并做 coverage/连续性质量门限。

### Q14：如何保证抗齿槽后处理不拖过实时 deadline？

答：不在一个控制周期遍历 3600 bins。finalize、smooth、stats 每周期只处理 1/2/4/8 个 bins，状态机推进；运动扫描也是非阻塞的。这样每周期工作量有固定上界。

### Q15：速度环的 anti-windup 怎么做？

答：先计算 P+I+FF，再经过 ABZ torque limit 和 motor global torque limit。若输出在上限且误差仍推动同方向，就暂停积分；误差反向时允许积分释放；积分值还限制在最紧的有效限幅内。退出速度环时清零积分。

### Q16：为什么要有 ABZ 专用 abz_vel_gain，不用 generic vel_gain？

答：4000 CPR ABZ 的反馈噪声和通用 velocity 参数尺度不同。若 UI 修改通用参数而 ABZ loop 实际读取专用参数，会出现“页面调了但闭环没变”的假配置问题。当前代码把 ABZ loop 使用的 Kp/Ki 明确隔离，并在 UI safe profile 里写正确字段。

### Q17：急停是怎么保证没有旧状态残留的？

答：x 会清输入、Controller、速度积分、observer、摩擦补偿、trajectory 和 current PI 状态，调用安全 disarm 并请求 Idle；重新 arm 时 Motor 又会重置一次控制状态并先排队零电压 SVM 周期。这样不会复用旧轨迹或积分。

### Q18：watchdog 能保证 USB 断开就停机吗？

答：设计目标是能，当前上位机每 250 ms 发 u，运行超时默认 1 s；但当前 g/j 遥测分支也会 feed watchdog，所以只读遥测持续时可能维持控制，这是已知风险。改进是把 control heartbeat 与 telemetry feed 分离，并增加“停止控制命令但保留遥测”的 HIL 测试。

### Q19：为什么协议不用 JSON 或 protobuf？

答：项目参考成熟电机控制系统的 ASCII/property 思路，调试阶段需要串口终端可读和快速排障，所以采用换行 ASCII。高频闭环仍在 MCU，USB 只做 50/5 Hz 低频命令和遥测，性能足够。代价是字段位置契约脆弱，因此使用 append-only 和自动化长度/解析测试。

### Q20：如何避免串口命令乱序？

答：上位机所有 write 都进入同一个 Promise queue；heartbeat、fast telemetry、slow telemetry、控制命令和配置写入共享顺序通道，避免多个 writer 并发写入导致设备收到交错命令。

### Q21：为什么快帧按位置解析，而不带字段名？

答：示波器需要更高频率和更短帧，字段名会显著增加长度。为了兼容，前面的字段位置固定，新字段只能追加；固件和 host protocol test 同时锁定字段顺序和 worst-case 长度，超过 1024 时丢弃整帧。

### Q22：上位机如何防止默认值覆盖校准参数？

答：配置页先逐项读设备并记录 deviceValue；只有可编辑且值确实变化的字段才写入；方向、极对数、CPR、电阻、电感和预校准标志排除在批量写入外；还要求 Idle 和经过范围校验。

### Q23：为什么 Electron 需要 main/preload/renderer 分层？

答：main 处理窗口和串口权限，renderer 处理 Web Serial/UI，preload 只暴露最小桥接信息；启用 context isolation、关闭 nodeIntegration 可以减少页面脚本直接拿到 Node 文件系统和进程能力的风险。

### Q24：这个项目如何测试？

答：协议层测试命令和帧解析；算法层测试 observer/window/M/T/friction/anti-windup；portable core 做状态机测试；ARM smoke 检查目标编译器兼容；Tup 做真实固件构建；最后必须用 HIL 验证真实电机和供电/负载。当前自动化结果是协议通过、135 个估算器 checks 通过、portable CTest 1/1、ARM smoke 和固件全量构建通过。

### Q25：单元测试通过能证明低速稳定吗？

答：不能。单元测试能证明确定性算法边界、overflow/reversal/anti-windup、协议和构建；低速稳定性还受到电机摩擦、负载惯量、齿槽、供电、编码器安装和线缆信号影响，必须在限流、限扭矩和可急停条件下做 HIL 并保存原始数据。

### Q26：目前最大的技术风险是什么？

答：优先是 watchdog 与 telemetry feed 耦合，其次是配置多来源漂移和完整 Encoder 原始计数路径的长期技术债；另外 HIL 覆盖还需要持续扩充。回答风险时要同时给出验证方式和改进方案，而不是只说“以后优化”。

### Q27：如果速度曲线仍然振荡，你怎么排查？

答：先不直接调 Kp。先确认 mode/state/encoder CPR/方向/校准；同时看 observer、50/100 ms、M/T、raw PLL、estimator disagreement、delta count/glitch count；再看 P/I/friction/anticogging/final torque 和 saturation。如果只有 observer 抖，先查带宽和计数路径；如果几路都抖，查机械/信号；如果 torque 饱和，查限幅和负载；一次只改一个变量并回归全部速度点。

### Q28：如果电机不转但 torque 已经很大，怎么办？

答：看 friction state、no-progress time、breakaway extra torque、position error、Iq measured/setpoint 和 torque saturation。确认方向和编码器计数是否前进；若是静摩擦，允许 breakaway 状态机按限幅释放；若是反向计数或 glitch，先停机检查 ABZ 相线、CPR、编码器供电和机械卡滞，不能盲目提高电流。

### Q29：如果只在重启后抗齿槽失效，查什么？

答：查 anticogging.pre_calibrated 是否真的写入 NVM、Encoder setup 是否恢复 anticogging_valid、map 是否完整、quality gate 是否通过，以及上位机读到的 anticoggingValid/coverage。当前代码专门处理了 ABZ map 在重启后的恢复路径。

### Q30：如果固件能编译但上位机收不到数据，怎么排查？

答：按链路分层：USB device enumeration→Electron serial permission→串口是否 115200→reader 是否拿到字节→LineParser 是否遇到换行→@/! 前缀和字段是否全是 finite number→firmware respond() 是否因 buffer overflow 丢帧→host polling 是否被 write queue/断开异常阻塞。先用终端手动发送 j 0，再看协议 parser，不直接怀疑 FOC 算法。

## 19. 面试中的项目复盘与后续计划

### 19.1 当前最重要的工程结论

1. 低速问题的首要矛盾是反馈估算和外环，而不是电流环；
2. 多估算器可以并存，但闭环反馈必须有唯一责任人；
3. 诊断字段必须足够细，才能把“速度不稳”拆成 estimator、机械、补偿、饱和或信号问题；
4. 安全路径必须在固件和上位机各做一层，不能只靠 UI 禁用按钮；
5. 实时系统的性能优化首先是固定上界、O(1)、不阻塞和分摊工作，而不是盲目微优化；
6. 自动化测试、ARM build 和 HIL 分别证明不同命题，不能用一个命题替代另一个。

### 19.2 推荐后续迭代顺序

1. 把 g/j telemetry feed 与 control watchdog 分离，增加超时 HIL；
2. 将 Firmware/Tests/test_runner.cpp 纳入根测试脚本和 CI；
3. 用共享/生成配置消除 main.cpp 与 portable snapshot 的参数重复；
4. 增加完整 controller continuity、mode exit、re-arm、position small-error 和 encoder overflow 测试；
5. 增加 deterministic ABZ mechanical simulation，用于筛查 limit cycle/windup，不代替 HIL；
6. 以 HIL 数据为依据，一次只修改一个 estimator、gain、compensation 或 limit 参数；
7. 继续评估 Encoder 原始 PLL/绝对位置路径的长期计数和 float precision 技术债。

## 20. 关联资料和代码入口

| 主题 | 文件 |
|---|---|
| 项目概览和运行命令 | README.md |
| 实际运行边界 | docs/SOURCE_OF_TRUTH.md |
| 控制链路 | docs/CONTROL_ARCHITECTURE.md |
| ABZ 估算器 | docs/ABZ_VELOCITY_ESTIMATION.md |
| USB CDC 协议 | protocol/protocol.md |
| HIL 测试 | docs/HIL_TEST_PLAN.md |
| 回归矩阵 | docs/REGRESSION_MATRIX.md |
| 已知问题 | docs/KNOWN_ISSUES.md |
| 固件默认值 | firmware-target/Firmware/MotorControl/main.cpp |
| Axis 控制/看门狗 | firmware-target/Firmware/MotorControl/axis.cpp/.hpp |
| Encoder/PLL/window | firmware-target/Firmware/MotorControl/encoder.cpp/.hpp |
| ABZ observer | firmware-target/Firmware/MotorControl/abz_velocity_observer.hpp |
| 摩擦补偿 | firmware-target/Firmware/MotorControl/friction_compensator.hpp |
| Controller 外环 | firmware-target/Firmware/MotorControl/controller.cpp/.hpp |
| Motor 电流环/FOC | firmware-target/Firmware/MotorControl/motor.cpp/.hpp |
| ASCII parser/遥测 | firmware-target/Firmware/communication/ascii_protocol.cpp |
| Host parser/命令 | host/foc-studio/protocol.js |
| Host UI/轮询/示波器 | host/foc-studio/app.js |
| Electron 权限边界 | host/foc-studio/desktop/main.cjs、preload.cjs |

## 21. 面试前与 AI 生成前的检查清单

面试前至少能脱稿回答；AI 生成项目介绍、技术方案或排障建议时，也应先检查这些关键事实：

- 项目解决了什么问题，自己负责哪一段；
- 完整数据流从 USB 命令到 PWM、再从编码器反馈回 Controller 的路径；
- FOC 的 Clarke/Park/current PI/SVPWM 各自职责；
- ABZ 低速为什么不能用单周期 delta，observer 为什么用增量和 local rebase；
- 50 ms、100 ms、M/T、PLL 和 observer 谁进闭环、谁只做诊断；
- 摩擦补偿状态机如何进出 BREAKAWAY，为什么不能直接提高 Kp；
- anti-windup、torque limit、超速 qualifier、watchdog、急停如何共同保证安全；
- 协议为什么是 ASCII、如何避免乱序、如何保证快帧兼容和不截断；
- 配置为什么必须先读后写；
- 自动化测试能证明什么，HIL 还要证明什么；
- 当前已知风险是什么、怎样设计下一步验证。

最重要的表达方式是：先说结论，再说代码路径、数据证据和边界。例如不要说“低速已经完全解决”，而要说“ABZ control observer 的增量输入、低速控制和诊断链路已经自动化验证，当前固件也能全量构建；不同机械负载下的低速稳定性仍以 HIL 数据为准”。

## 22. 知识库维护协议

### 22.1 何时必须更新本知识库

出现以下任一变化时，必须同步更新第 0 节的事实记录和对应详细章节：

- 控制反馈来源变化，例如 observer、PLL、window 或 M/T 的职责变化；
- 新增/删除控制模式、反馈模式、协议命令或遥测字段；
- 修改默认电流、扭矩、速度、observer、friction 或 anticogging 参数；
- 修改急停、watchdog、超速、current limit 或 PWM disarm 路径；
- target firmware 的实际 source of truth、构建 profile 或产物格式变化；
- 自动化测试结果、固件 hash 或 HIL 结论变化；
- 自研板卡的 MCU、功率级、ADC、PWM、编码器接口或母线条件变化。

### 22.2 每条新知识的记录格式

新增事实建议使用以下格式，便于 AI 进行检索和冲突判断：

~~~text
Fact ID:
Topic:
Statement:
Status: IMPLEMENTED | VERIFIED_AUTOMATED | VERIFIED_HIL_REQUIRED | DESIGN_INTENT | KNOWN_RISK | INTERVIEW_DERIVED
Evidence:
Affected components:
Valid from:
Invalidated by:
Notes:
~~~

### 22.3 变更后的最低验证要求

| 变更内容 | 至少执行 |
|---|---|
| host protocol/parser | npm.cmd test |
| observer/window/friction/anti-windup | ABZ estimator tests + portable core tests |
| target C++/board/fibre | tools/test-project.ps1 + tools/build-firmware.ps1 |
| safety/watchdog/急停 | 自动化测试 + 对应 HIL negative test |
| 参数或控制性能 | 自动化回归 + 受控 HIL + 保存原始数据 |
| telemetry 字段 | 固件/host parser 测试 + worst-case frame length test |

### 22.4 AI 输出前的事实检查

AI 在输出项目介绍、代码方案、面试回答或故障诊断前，应检查：

1. 是否明确区分 target firmware 和 portable core；
2. 是否明确区分 control observer、raw PLL、window 和 M/T；
3. 是否把实现、自动化验证、HIL 待验证和面试口径分开；
4. 是否把当前已知的 watchdog/配置/长期计数风险隐藏掉；
5. 是否给出了可定位的代码文件、测试脚本或文档证据；
6. 是否引入了本知识库没有记录的硬件、性能或个人经历细节。
