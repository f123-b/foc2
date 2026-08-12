# 本地编译工具

工具安装在 `D:\foc2\tools`：

- Arm GNU Toolchain `12.2.MPACBTI-Rel1`，GCC `12.2.1`；
- Tup `v0.8`，由官方 `gittup/tup` 仓库的 `v0.8` 标签编译。

Arm 安装包来自 Arm 官方开发者下载地址，SHA-256 校验值为：

```text
C5215E37E70A2FA3233CD1F348AB74896281C40D1C531FC719CA6BA11EB99290
```

在 PowerShell 中加载环境：

```powershell
. .\tools\env.ps1
```

在普通命令提示符中加载环境：

```bat
tools\env.cmd
```

通常不需要手动加载环境，以下脚本会自动完成设置：

```powershell
powershell -ExecutionPolicy Bypass -File tools\test-project.ps1
powershell -ExecutionPolicy Bypass -File tools\build-firmware.ps1
```

ODrive 固件使用 `odrive-baseline/Firmware/tup.config` 中的 V3.6 56 V 板级配置。
由于 Tup 在 Windows 上的依赖注入 DLL 会导致 Arm 编译器崩溃，项目使用
`tup generate` 生成离线批处理脚本，再在 Tup 注入环境之外运行编译器。
