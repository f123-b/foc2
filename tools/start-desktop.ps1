[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$hostRoot = Join-Path $projectRoot 'host\foc-studio'
$npm = (Get-Command npm.cmd -ErrorAction Stop).Source
$electron = Join-Path $hostRoot 'node_modules\electron\dist\electron.exe'

Push-Location $hostRoot
try {
    $env:ELECTRON_MIRROR = 'https://npmmirror.com/mirrors/electron/'
    $env:npm_config_registry = 'https://registry.npmmirror.com'
    if (!(Test-Path -LiteralPath $electron)) {
        Write-Host '首次运行：正在安装 Electron 桌面版依赖。' -ForegroundColor Cyan
        & $npm ci
        if ($LASTEXITCODE -ne 0) { throw 'npm 依赖安装失败，请检查网络后重试。' }
    }
    & $npm run desktop
    if ($LASTEXITCODE -ne 0) { throw 'FOC Studio 桌面版启动失败。' }
} finally {
    Remove-Item Env:ELECTRON_MIRROR -ErrorAction SilentlyContinue
    Remove-Item Env:npm_config_registry -ErrorAction SilentlyContinue
    Pop-Location
}
