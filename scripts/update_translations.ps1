# 翻译文件更新脚本
# 当 UI 文件或源代码中新增了可翻译字符串后，运行此脚本更新 .ts 文件

param(
    [string]$QtDir = "D:/Qt/Qt5.12.7/5.12.7/msvc2017_64"
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

# 查找 lupdate 工具
$lupdate = Join-Path $QtDir "bin/lupdate.exe"
if (-not (Test-Path $lupdate)) {
    Write-Error "lupdate.exe not found at: $lupdate"
    Write-Host "Please specify Qt path: .\scripts\update_translations.ps1 -QtDir 'C:\Qt\5.x.x\msvc20xx_64'"
    exit 1
}

Write-Host "Using lupdate: $lupdate" -ForegroundColor Cyan

# 定义要扫描的源文件和 UI 文件
$sources = @(
    "$ProjectRoot/src",
    "$ProjectRoot/include",
    "$ProjectRoot/ui"
)

# 定义翻译文件
$tsFiles = @(
    "$ProjectRoot/translations/linger_zh_CN.ts",
    "$ProjectRoot/translations/linger_en.ts"
)

# 构建 lupdate 参数
$sourceArgs = @()
foreach ($src in $sources) {
    if (Test-Path $src) {
        $sourceArgs += $src
    }
}

# 运行 lupdate
foreach ($tsFile in $tsFiles) {
    Write-Host "`nUpdating: $tsFile" -ForegroundColor Green
    
    $args = $sourceArgs + @("-ts", $tsFile, "-no-obsolete")
    
    Write-Host "Running: lupdate $($args -join ' ')" -ForegroundColor DarkGray
    & $lupdate @args
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Successfully updated: $tsFile" -ForegroundColor Green
    } else {
        Write-Warning "lupdate returned non-zero exit code for: $tsFile"
    }
}

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "Translation files updated!" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Open .ts files and add/edit translations"
Write-Host "2. Run build script to compile .qm files: .\scripts\build_utf8.ps1"
Write-Host "========================================"
