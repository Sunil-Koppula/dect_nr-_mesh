param(
    [Parameter(Mandatory=$true, Position=0)]
    [string]$board
)

# Board serial number mapping
$boards = @{
    "1" = "1052004739"
    "2" = "1052050495"
    "3" = "1052071448"
    "4" = "1052009857"
}

$modem_firmware = "c:\ncs\mfw-nr+-phy_nrf91x1_2.0.0.zip"
$CurrentDirectory = (Get-Location).Path.Replace('\', '/')
$app_hex = "$CurrentDirectory/build/merged.hex"

# Check that the build exists
if (-not (Test-Path $app_hex)) {
    Write-Host "Build not found: $app_hex" -ForegroundColor Red
    Write-Host "Run ./build.ps1 first" -ForegroundColor Yellow
    exit 1
}

# Determine which boards to flash
if ($board -eq "all") {
    $targets = $boards.Keys | Sort-Object
} elseif ($boards.ContainsKey($board)) {
    $targets = @($board)
} else {
    Write-Host "Invalid board: $board" -ForegroundColor Red
    Write-Host "Usage: ./program.ps1 <1|2|3|4|all>" -ForegroundColor Yellow
    Write-Host "  1 = SN $($boards['1'])" -ForegroundColor Gray
    Write-Host "  2 = SN $($boards['2'])" -ForegroundColor Gray
    Write-Host "  3 = SN $($boards['3'])" -ForegroundColor Gray
    Write-Host "  4 = SN $($boards['4'])" -ForegroundColor Gray
    exit 1
}

$failed = @()

foreach ($t in $targets) {
    $sn = $boards[$t]
    Write-Host "`n=== Flashing firmware to Board $t (SN: $sn) ===" -ForegroundColor Cyan

    # # Upgrade Modem Firmware (uncomment to use)
    # Write-Host "Programming Modem Firmware..." -ForegroundColor Yellow
    # nrfutil 91 modem-firmware-upgrade --firmware $modem_firmware --serial-number $sn

    Write-Host "Flashing $app_hex..." -ForegroundColor Green
    nrfjprog --program $app_hex --chiperase --verify -s $sn
    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAILED to flash Board $t (SN: $sn)" -ForegroundColor Red
        $failed += $t
        continue
    }

    Write-Host "Resetting board..." -ForegroundColor Green
    nrfjprog --reset -s $sn
    Write-Host "Board $t done." -ForegroundColor Green
}

Write-Host "`n=== Summary ===" -ForegroundColor Cyan
Write-Host "Total: $($targets.Count) | Succeeded: $($targets.Count - $failed.Count) | Failed: $($failed.Count)"
if ($failed.Count -gt 0) {
    Write-Host "Failed boards: $($failed -join ', ')" -ForegroundColor Red
    exit 1
}
Write-Host "`nDevice type will be determined by GPIO pins P0.21/P0.22 at boot." -ForegroundColor Cyan
