$projectRoot = Split-Path -Parent $PSScriptRoot
$armRoot = Join-Path $projectRoot 'tools\arm-gnu-toolchain'
$tupRoot = Join-Path $projectRoot 'tools\tup'

$env:ARM_NONE_EABI_ROOT = $armRoot
$env:TUP_ROOT = $tupRoot
$env:PATH = "$($armRoot)\bin;$($tupRoot)\bin;$env:PATH"

Write-Host "FOC2 toolchain environment loaded"
& (Join-Path $armRoot 'bin\arm-none-eabi-gcc.exe') --version | Select-Object -First 1
& (Join-Path $tupRoot 'bin\tup.exe') --version
