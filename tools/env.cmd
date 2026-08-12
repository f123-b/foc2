@echo off
set "PROJECT_ROOT=%~dp0.."
set "ARM_NONE_EABI_ROOT=%PROJECT_ROOT%\tools\arm-gnu-toolchain"
set "TUP_ROOT=%PROJECT_ROOT%\tools\tup"
set "PATH=%ARM_NONE_EABI_ROOT%\bin;%TUP_ROOT%\bin;%PATH%"
echo FOC2 toolchain environment loaded
arm-none-eabi-gcc.exe --version
tup.exe --version
