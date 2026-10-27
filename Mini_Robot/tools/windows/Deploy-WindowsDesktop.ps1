param(
    [string]$BuildDir = "build\windows-msvc",
    [string]$QtPrefix,
    [string]$OpenCvBin
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($QtPrefix)) {
    throw "QtPrefix is required (for example C:\Qt\6.8.3\msvc2022_64)."
}
if ([string]::IsNullOrWhiteSpace($OpenCvBin)) {
    throw "OpenCvBin is required (for example C:\tools\opencv\build\x64\vc16\bin)."
}

$desktopExe = Join-Path $BuildDir "src\micro_robot_desktop.exe"
if (-not (Test-Path $desktopExe)) {
    throw "Desktop executable not found: $desktopExe. Build with --preset windows-msvc first."
}

$deployDir = Join-Path $BuildDir "deploy\micro_robot_desktop"
New-Item -ItemType Directory -Path $deployDir -Force | Out-Null
Copy-Item -Path $desktopExe -Destination $deployDir -Force

$windeployqt = Join-Path $QtPrefix "bin\windeployqt.exe"
if (-not (Test-Path $windeployqt)) {
    throw "windeployqt not found under QtPrefix: $windeployqt"
}

& $windeployqt --release --no-translations --no-opengl-sw --dir $deployDir (Join-Path $deployDir "micro_robot_desktop.exe")
if (-not $?) {
    throw "windeployqt failed."
}

if (-not (Test-Path $OpenCvBin)) {
    throw "OpenCV runtime directory not found: $OpenCvBin"
}

Get-ChildItem -Path $OpenCvBin -Filter "opencv*.dll" -File | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination $deployDir -Force
}

Write-Host "Deployment completed: $deployDir"
