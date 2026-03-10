param(
    [Parameter(Mandatory=$true)]
    [string]$sn,

    [Parameter(Mandatory=$true)]
    [ValidateSet("gateway", "anchor", "sensor")]
    [string]$type
)

# Segger Serial Number
$sn1 = "1052004739"
$sn2 = "1052050495"
$sn3 = "1052071448"

$modem_firmware = "c:\ncs\mfw-nr+-phy_nrf91x1_2.0.0.zip"
$CurrentDirectory = (Get-Location).Path.Replace('\', '/')
$app_hex = "$CurrentDirectory/build_$type/merged.hex"

# Check that the build exists
if (-not (Test-Path $app_hex)) {
    Write-Host "Build not found: $app_hex" -ForegroundColor Red
    Write-Host "Run ./build_$type.ps1 first" -ForegroundColor Yellow
    exit 1
}

Write-Host "`n=== Flashing $($type.ToUpper()) to device $sn ===" -ForegroundColor Cyan

# # Upgrade Modem Firmware (uncomment to use)
# Write-Host "Programming Modem Firmware..." -ForegroundColor Yellow
# nrfutil 91 modem-firmware-upgrade --firmware $modem_firmware --serial-number $sn

Write-Host "Flashing $app_hex..." -ForegroundColor Green
nrfjprog --program $app_hex --chiperase --verify -s $sn
if ($LASTEXITCODE -ne 0) {
    Write-Host "FAILED to flash device $sn" -ForegroundColor Red
    exit 1
}

Write-Host "Resetting board..." -ForegroundColor Green
nrfjprog --reset -s $sn

Write-Host "`nDone! Connect serial terminal at 115200 baud to see output." -ForegroundColor Cyan
