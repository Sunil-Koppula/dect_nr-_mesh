param(
    [Parameter(Mandatory=$true)]
    [string]$port,

    [string]$bin = "build/hello_dect/zephyr/zephyr.signed.bin"
)

# OTA Upload Script
# Uploads firmware to MCUboot secondary slot via MCUmgr SMP over UART.
# Requires: pip install smpclient[serial]
#
# Usage: ./ota_upload.ps1 -port COM36
#        ./ota_upload.ps1 -port COM36 -bin path/to/zephyr.signed.bin

$ErrorActionPreference = "Stop"

if (-not (Test-Path $bin)) {
    Write-Host "OTA binary not found: $bin" -ForegroundColor Red
    Write-Host "Run ./build.ps1 first" -ForegroundColor Yellow
    exit 1
}

$binAbsolute = (Resolve-Path $bin).Path
$imageSize = (Get-Item $binAbsolute).Length
Write-Host "Image: $bin ($imageSize bytes)" -ForegroundColor Cyan

$pyScript = @"
import asyncio
import sys

async def main():
    from smpclient import SMPClient
    from smpclient.transport.serial import SMPSerialTransport

    port = sys.argv[1]
    bin_path = sys.argv[2]

    with open(bin_path, 'rb') as f:
        image = f.read()

    size = len(image)
    print(f'Connecting to {port}...')

    async with SMPClient(SMPSerialTransport(), port) as client:
        print(f'Uploading {size} bytes to secondary slot...')
        async for offset in client.upload(image, slot=0, first_timeout_s=60.0):
            pct = int(offset * 100 / size)
            print(f'\r  Progress: {pct}% ({offset}/{size})', end='', flush=True)

    print(f'\n\nUpload complete!')

asyncio.run(main())
"@

$tempPy = [System.IO.Path]::GetTempPath() + "smp_upload.py"
[System.IO.File]::WriteAllText($tempPy, $pyScript)

# Use system Python (not NCS venv) which has smpclient installed
$systemPython = "$env:LOCALAPPDATA\Programs\Python\Python313\python.exe"
if (-not (Test-Path $systemPython)) {
    # Fallback: search for python with smpclient
    $systemPython = "python"
}

Write-Host "Starting SMP upload..." -ForegroundColor Yellow
& $systemPython $tempPy $port $binAbsolute
$exitCode = $LASTEXITCODE

Remove-Item $tempPy -ErrorAction SilentlyContinue

if ($exitCode -eq 0) {
    Write-Host "Image written to MCUboot secondary slot." -ForegroundColor Green
    Write-Host "Reset the device to apply the update." -ForegroundColor Cyan
} else {
    Write-Host "Upload failed." -ForegroundColor Red
    exit 1
}
