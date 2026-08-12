[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$firmwareRoot = Join-Path $projectRoot 'odrive-baseline\Firmware'
$buildRoot = Join-Path $firmwareRoot 'build'
$expectedBuildRoot = [System.IO.Path]::GetFullPath((Join-Path $projectRoot 'odrive-baseline\Firmware\build'))

. (Join-Path $PSScriptRoot 'env.ps1')

if (Test-Path -LiteralPath $buildRoot) {
    $resolvedBuildRoot = [System.IO.Path]::GetFullPath($buildRoot)
    if ($resolvedBuildRoot -ne $expectedBuildRoot) {
        throw "Refusing to clean unexpected path: $resolvedBuildRoot"
    }
    Remove-Item -LiteralPath $resolvedBuildRoot -Recurse -Force
}

$generatedFiles = @(
    'autogen\interfaces.hpp',
    'autogen\function_stubs.hpp',
    'autogen\endpoints.hpp',
    'autogen\type_info.hpp',
    'autogen\version.c'
)

function Find-Python3 {
    $candidates = @()
    if ($env:PYTHON) { $candidates += $env:PYTHON }
    $candidates += (Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe')
    $pythonCommand = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($pythonCommand) { $candidates += $pythonCommand.Source }

    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (!(Test-Path -LiteralPath $candidate)) { continue }
        try {
            $version = & $candidate --version 2>&1
            if ($LASTEXITCODE -eq 0 -and "$version" -match '^Python 3') { return $candidate }
        } catch {
            continue
        }
    }
    throw 'Python 3 was not found. Set the PYTHON environment variable to python.exe.'
}

$pythonExe = Find-Python3
$sourceConfig = Join-Path $firmwareRoot 'tup.config'
$effectiveConfigName = "tup.foc2-$PID.config"
$effectiveConfig = Join-Path $firmwareRoot $effectiveConfigName
$configLines = Get-Content -LiteralPath $sourceConfig | Where-Object { $_ -notmatch '^CONFIG_PYTHON=' }
$pythonForTup = $pythonExe.Replace('\', '/')
@($configLines) + "CONFIG_PYTHON=$pythonForTup -B" | Set-Content -LiteralPath $effectiveConfig -Encoding Ascii

foreach ($relativePath in $generatedFiles) {
    $generatedPath = Join-Path $firmwareRoot $relativePath
    if (Test-Path -LiteralPath $generatedPath) {
        Remove-Item -LiteralPath $generatedPath -Force
    }
}

Push-Location $firmwareRoot
try {
    & (Join-Path $env:TUP_ROOT 'bin\tup.exe') generate --config $effectiveConfigName build-firmware.bat
    if ($LASTEXITCODE -ne 0) { throw 'Tup offline script generation failed.' }

    & "$env:SystemRoot\System32\cmd.exe" /d /c build-firmware.bat
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }

    $elf = Join-Path $buildRoot 'ODriveFirmware.elf'
    $hex = Join-Path $buildRoot 'ODriveFirmware.hex'
    $bin = Join-Path $buildRoot 'ODriveFirmware.bin'
    foreach ($artifact in @($elf, $hex, $bin)) {
        if (!(Test-Path -LiteralPath $artifact)) { throw "Missing artifact: $artifact" }
    }

    & arm-none-eabi-size.exe $elf
    & arm-none-eabi-objdump.exe -f $elf
    Write-Host "Firmware ready: $hex" -ForegroundColor Green
} finally {
    Pop-Location
    if (Test-Path -LiteralPath $effectiveConfig) { Remove-Item -LiteralPath $effectiveConfig -Force }
}
