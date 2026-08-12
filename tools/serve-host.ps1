[CmdletBinding()]
param([int]$Port = 4173)

$projectRoot = Split-Path -Parent $PSScriptRoot
$hostRoot = Join-Path $projectRoot 'host\foc-studio'
& python -m http.server $Port --directory $hostRoot
