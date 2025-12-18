<#
用法示例（默认 Release，GUI 模式）：

# 首次或切换回 GUI 配置：
#   .\scripts\build_utf8.ps1 -Reconfigure

# 仅编译已配置的 GUI 目标：
#   .\scripts\build_utf8.ps1

# 指定配置：
#   .\scripts\build_utf8.ps1 -Configuration Debug
#   .\scripts\build_utf8.ps1 -Configuration Release

# 传递额外 build 参数（放在 -- 后）：
#   .\scripts\build_utf8.ps1 -Configuration Release -- -- /m:4

# 说明：
# - -Reconfigure 会强制写入 BUILD_GUI=ON、BUILD_HEADLESS=OFF，并使用指定生成器/架构/Qt 前缀。
# - 默认仅编译 GUI 可执行 `linger_perceiver.exe`（Release）。
# - 合法配置名：Debug、Release、RelWithDebInfo、MinSizeRel。
#
#>
param(
    [string]$Configuration = 'Release',
    [switch]$Reconfigure,
    [string]$QtPath = "",
    [string]$Generator = "Visual Studio 17 2022",
    [string]$Arch = "x64",
    [Parameter(ValueFromRemainingArguments=$true)]
    [string[]]$Args
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$OutputEncoding = [System.Text.Encoding]::UTF8
chcp 65001 | Out-Null

$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildDir = Join-Path $RepoRoot "build"

if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
    $Reconfigure = $true
}

if ($Reconfigure) {
    Write-Host "Configuring (GUI)..."
    $cmakeArgs = @(
        "-S", $RepoRoot,
        "-B", $BuildDir,
        "-G", $Generator,
        "-A", $Arch,
        "-DBUILD_GUI=ON",
        "-DBUILD_HEADLESS=OFF"
    )
    if ($QtPath) {
        $cmakeArgs += "-DCMAKE_PREFIX_PATH=$QtPath"
    }
    & cmake @cmakeArgs
}

# 验证配置名
switch -Regex ($Configuration) {
    '^(Debug|Release|RelWithDebInfo|MinSizeRel)$' { }
    default {
        Write-Host "Unknown configuration '$Configuration'. Valid: Debug, Release, RelWithDebInfo, MinSizeRel" -ForegroundColor Yellow
        exit 1
    }
}

Write-Host "Building GUI target (Configuration=$Configuration)..." -ForegroundColor Cyan
& cmake --build $BuildDir --config $Configuration --target linger_perceiver @Args
