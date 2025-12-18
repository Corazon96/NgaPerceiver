<#
用法示例（默认 Release）：

# 仅编译已配置的无头目标（默认 Release）：
#   .\scripts\build_headless.ps1

# 重新配置（关闭 GUI，开启 HEADLESS）并编译 Release：
#   .\scripts\build_headless.ps1 -Reconfigure -Configuration Release

# 指定生成器/架构（如需）：
#   .\scripts\build_headless.ps1 -Reconfigure -Generator "Visual Studio 17 2022" -Arch x64

# 可选 Qt 前缀（便于后续 GUI 共用，不影响 headless）：
#   .\scripts\build_headless.ps1 -Reconfigure -QtPath "D:/Qt/Qt5.12.7/5.12.7/msvc2017_64"

# 说明：
# - 默认只构建 headless 可执行文件 `linger_perceiver_service.exe`（Release）。
# - 第一次或修改 CMake 选项时需加 -Reconfigure；否则沿用现有 build 目录配置。
# - BUILD_GUI 会被禁用，BUILD_HEADLESS 会被启用。
#
#>
param(
    [string]$Configuration = "Release",
    [switch]$Reconfigure,
    [string]$QtPath = "",
    [string]$Generator = "Visual Studio 17 2022",
    [string]$Arch = "x64"
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
    Write-Host "Configuring (headless)..."
    $cmakeArgs = @(
        "-S", $RepoRoot,
        "-B", $BuildDir,
        "-G", $Generator,
        "-A", $Arch,
        "-DBUILD_GUI=OFF",
        "-DBUILD_HEADLESS=ON"
    )
    if ($QtPath) {
        # optional: allows reuse of same prefix path if you later build GUI
        $cmakeArgs += "-DCMAKE_PREFIX_PATH=$QtPath"
    }
    & cmake @cmakeArgs
}

Write-Host "Building headless target (Configuration=$Configuration)..."
& cmake --build $BuildDir --config $Configuration --target linger_perceiver_service

if ($LASTEXITCODE -eq 0) {
    $outDir = Join-Path $BuildDir $Configuration
    Write-Host "Done. Binary: $outDir\linger_perceiver_service.exe"
} else {
    throw "Build failed."
}
