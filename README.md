# FOC Studio

FOC Studio 是一个面向 V3.6 硬件平台和 3505-KV650 电机的单轴 FOC 项目。
当前使用 M0 轴，支持以下反馈方式：

- AS5047P SPI 绝对值编码器，16384 CPR；
- ABZ 增量编码器，4000 CPR；
- 无感 FOC；
- 无感 FOC，同时用 SPI 或 ABZ 监视角度误差。

Windows 上位机通过控制器 USB CDC 接口通信，默认作为独立 Electron 软件运行，
也保留 Microsoft Edge/Chrome 浏览器备用方式。第三方组件和许可证见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 从这里开始

第一次使用请按以下顺序阅读：

1. [项目介绍与代码分析](docs/项目介绍与代码分析.md)：项目结构和真实运行边界；
2. [Source of Truth](docs/SOURCE_OF_TRUTH.md)：实际烧录固件、配置来源和漂移风险；
3. [控制架构](docs/CONTROL_ARCHITECTURE.md)：编码器、控制器、电流环和 PWM 链路；
4. [优化计划](docs/OPTIMIZATION_PLAN.md)：后续小步、可回归的改进顺序；
5. [HIL 测试计划](docs/HIL_TEST_PLAN.md)：实机采数与安全操作要求；
6. [回归矩阵](docs/REGRESSION_MATRIX.md)：当前基线及待补测试；
7. [已知问题](docs/KNOWN_ISSUES.md)：按优先级分类的工程风险；
8. [通信协议](protocol/protocol.md)：USB CDC ASCII 命令说明。

## 运行 Windows 桌面版

首次运行需要联网下载 Electron：

```powershell
cd <repository-root>
powershell -ExecutionPolicy Bypass -File tools\start-desktop.ps1
```

生成可独立启动的 Windows 便携版：

```powershell
powershell -ExecutionPolicy Bypass -File tools\build-desktop.ps1
```

输出位于 `host/foc-studio/dist/FOC-Studio-0.1.0.exe`。桌面版使用 Electron
内置 Web Serial 访问控制器 USB CDC，不需要浏览器页面。

## 浏览器备用版

在 PowerShell 中执行：

```powershell
cd <repository-root>
powershell -ExecutionPolicy Bypass -File tools\serve-host.ps1
```

然后用 Edge 或 Chrome 打开 [http://127.0.0.1:4173](http://127.0.0.1:4173)，
点击“模拟设备”。此步骤不需要连接控制器。

## 测试与编译

```powershell
cd <repository-root>
powershell -ExecutionPolicy Bypass -File tools\test-project.ps1
powershell -ExecutionPolicy Bypass -File tools\build-firmware.ps1
```

生成的固件位于：

- `firmware-target/Firmware/build/FOCStudioFirmware.elf`
- `firmware-target/Firmware/build/FOCStudioFirmware.hex`
- `firmware-target/Firmware/build/FOCStudioFirmware.bin`

目标处理器架构为 ARMv7E-M，STM32 Flash 起始地址为 `0x08000000`。

## 目录结构

- `firmware-target/Firmware`：可重复编译的 STM32 固件工程，
  已加入产品默认参数和 FOC Studio ASCII 命令；
- `firmware`：提取后的便携配置、安全状态机和保留的基础控制源码；
- `host/foc-studio`：Electron Windows 上位机、浏览器备用版和模拟设备；
- `tools`：Arm GNU、Tup、测试、编译、启动上位机和受保护的 DFU 脚本；
- `docs`：使用、接线、校准和实现状态文档；
- `protocol`：上位机与固件的通信协议。

完整固件工程仍保留 HAL、FreeRTOS、USB、Fibre 接口生成和必要的板级支持，
因为这些内容是生成可验证 STM32 固件所必需的。无关的上游 GUI、主机工具和
仓库其他内容没有放入精简的 `firmware` 目录。

## 默认安全参数

- 首次调试电流限制：2 A；
- 校准电流：1 A；
- 扭矩限制：0.2 Nm；
- 母线过压保护：16 V；
- 通信看门狗：位置/扭矩模式 1.0 秒，速度模式临时 2.0 秒；
- 电机和编码器 `pre_calibrated`：默认均为 `false`。

3505 电机额定电压是 12 V。控制板可以承受 56 V，不代表这台电机可以使用
56 V。烧录或接通电机电源之前，必须遵循[HIL 测试计划](docs/HIL_TEST_PLAN.md)
中的安全前置条件，并核对实际硬件接线。
项目脚本不会自动烧录，也不会自动使电机上电。
