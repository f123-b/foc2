[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$hostRoot = Join-Path $projectRoot 'host\foc-studio'
$npm = (Get-Command npm.cmd -ErrorAction Stop).Source
$electron = Join-Path $hostRoot 'node_modules\electron\dist\electron.exe'
$env:ELECTRON_MIRROR = 'https://npmmirror.com/mirrors/electron/'
$env:ELECTRON_BUILDER_BINARIES_MIRROR = 'https://npmmirror.com/mirrors/electron-builder-binaries/'

Push-Location $hostRoot
try {
    if (!(Test-Path -LiteralPath $electron)) {
        & $npm install --registry=https://registry.npmmirror.com
        if ($LASTEXITCODE -ne 0) { throw 'npm dependency installation failed.' }
    }
    & $npm run desktop:dist
    if ($LASTEXITCODE -ne 0) { throw 'Windows portable build failed.' }
    Write-Host "Windows desktop build ready: $hostRoot\dist" -ForegroundColor Green
} finally {
    Pop-Location
}
