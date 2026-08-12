# 便携固件层

本目录保存精简后的、与 STM32 平台无关的 FOC Studio 逻辑：

- 已校验的 3505-KV650 和 AS5047P/ABZ 参数；
- 仅允许在 `Idle` 状态执行的反馈模式切换；
- 故障锁存安全状态机；
- Windows 单元测试和 Cortex-M4 编译测试；
- `vendor/upstream-v0.5.1` 中保留的基础电机、编码器、无感估算、控制器、
  底层驱动和 DRV8301 源码。

完整 STM32 固件从 `../firmware-target/Firmware` 编译。即使只使用一个电机轴，启动
代码、HAL、FreeRTOS、链接脚本、USB CDC 和 Fibre 类型信息仍是必要依赖。

请使用：

```powershell
powershell -ExecutionPolicy Bypass -File ..\tools\build-firmware.ps1
```

本目录中的便携快照不能单独编译成可烧录固件。
