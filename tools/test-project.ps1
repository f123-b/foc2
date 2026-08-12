[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot

Push-Location (Join-Path $projectRoot 'host\foc-studio')
try {
    & node test_protocol.mjs
    if ($LASTEXITCODE -ne 0) { throw 'Host protocol tests failed.' }
} finally {
    Pop-Location
}

$portableFirmwareRoot = Join-Path $projectRoot 'firmware'
$portableBuildRoot = Join-Path $portableFirmwareRoot 'build-host'
& cmake.exe -S $portableFirmwareRoot -B $portableBuildRoot
if ($LASTEXITCODE -ne 0) { throw 'Portable firmware test configuration failed.' }
& cmake.exe --build $portableBuildRoot --config Release
if ($LASTEXITCODE -ne 0) { throw 'Portable firmware test build failed.' }
& ctest.exe --test-dir $portableBuildRoot --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Portable firmware tests failed.' }

. (Join-Path $PSScriptRoot 'env.ps1')
$smokeRoot = Join-Path $projectRoot 'firmware\arm-smoke'
New-Item -ItemType Directory -Force -Path $smokeRoot | Out-Null
& arm-none-eabi-g++.exe -std=c++17 -mcpu=cortex-m4 -mthumb -ffreestanding -I (Join-Path $projectRoot 'firmware\include') -c (Join-Path $projectRoot 'firmware\src\feedback_mode.cpp') -o (Join-Path $smokeRoot 'feedback_mode.o')
if ($LASTEXITCODE -ne 0) { throw 'ARM feedback-mode smoke compile failed.' }
& arm-none-eabi-g++.exe -std=c++17 -mcpu=cortex-m4 -mthumb -ffreestanding -I (Join-Path $projectRoot 'firmware\include') -c (Join-Path $projectRoot 'firmware\src\safety_state.cpp') -o (Join-Path $smokeRoot 'safety_state.o')
if ($LASTEXITCODE -ne 0) { throw 'ARM safety-state smoke compile failed.' }

Write-Host 'All project tests passed.' -ForegroundColor Green
