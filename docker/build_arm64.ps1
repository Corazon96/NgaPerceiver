# LingerPerceiver ARM64 交叉编译脚本 (Windows PowerShell)
# 用法: .\docker\build_arm64.ps1

param(
    [string]$Configuration = "Release",
    [switch]$RebuildImage,
    [switch]$CleanBuild
)

$ErrorActionPreference = "Continue"  # 允许继续执行，不因 stderr 输出而中断
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ImageName = "linger-arm64-build"
$DockerfilePath = "$ProjectRoot\docker\Dockerfile.arm64"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "LingerPerceiver ARM64 Cross-Compile" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# Step 1: 确保 QEMU 模拟器已注册（仅需运行一次）
Write-Host "`n[1/4] Setting up QEMU for ARM64 emulation..." -ForegroundColor Yellow
# 重定向 stderr 到 stdout，避免 PowerShell 将 docker 的进度信息误判为错误
$null = docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 2>&1
Write-Host "QEMU setup completed." -ForegroundColor Green

# Step 2: 构建 Docker 镜像
$ImageExists = (docker images -q $ImageName 2>$null) -ne ""
if ($RebuildImage -or (-not $ImageExists)) {
    Write-Host "`n[2/4] Building Docker image (this may take a while)..." -ForegroundColor Yellow
    docker build --platform linux/arm64 -t $ImageName -f $DockerfilePath $ProjectRoot\docker
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error: Docker image build failed!" -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "`n[2/4] Docker image already exists, skipping build..." -ForegroundColor Green
}

# Step 3: 编译项目
Write-Host "`n[3/4] Compiling LingerPerceiver for ARM64..." -ForegroundColor Yellow

# 将构建脚本的换行符转换为 Unix 格式
$BuildScriptPath = "$ProjectRoot\docker\build_arm64.sh"
$BuildScriptContent = Get-Content $BuildScriptPath -Raw
$BuildScriptContent = $BuildScriptContent -replace "`r`n", "`n" -replace "`r", "`n"
[System.IO.File]::WriteAllText($BuildScriptPath, $BuildScriptContent, [System.Text.UTF8Encoding]::new($false))

docker run --rm `
    --platform linux/arm64 `
    -v "${ProjectRoot}:/workspace" `
    -e "CLEAN_BUILD=$CleanBuild" `
    $ImageName `
    bash /workspace/docker/build_arm64.sh

if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Compilation failed!" -ForegroundColor Red
    exit 1
}

# Step 4: 显示部署信息
Write-Host "`n[4/4] Build completed successfully!" -ForegroundColor Green
Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "Deployment Instructions:" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host @"

部署包已创建于: build_arm64/deploy/

1. 拷贝整个 deploy 目录到树莓派:
   scp -r build_arm64/deploy linger@<树莓派IP>:~/linger_perceiver

2. 在树莓派上运行（无需安装任何依赖！）:
   cd ~/linger_perceiver
   ./run.sh

3. 后台运行:
   nohup ./run.sh > linger.log 2>&1 &

所有依赖库已自动打包，拷贝即可运行！

"@ -ForegroundColor White
