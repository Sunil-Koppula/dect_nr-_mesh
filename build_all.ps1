#
# Build all three device types: gateway, anchor, sensor
#

$ErrorActionPreference = "Stop"

$Board = "nrf9151dk/nrf9151/ns"
$ProjectDir = $PSScriptRoot

Write-Host "========================================="
Write-Host " Building GATEWAY"
Write-Host "========================================="
west build -b $Board $ProjectDir -d "$ProjectDir\build_gateway" `
    -- -DEXTRA_CONF_FILE="$ProjectDir\gateway.conf"

Write-Host ""
Write-Host "========================================="
Write-Host " Building ANCHOR"
Write-Host "========================================="
west build -b $Board $ProjectDir -d "$ProjectDir\build_anchor" `
    -- -DEXTRA_CONF_FILE="$ProjectDir\anchor.conf"

Write-Host ""
Write-Host "========================================="
Write-Host " Building SENSOR"
Write-Host "========================================="
west build -b $Board $ProjectDir -d "$ProjectDir\build_sensor" `
    -- -DEXTRA_CONF_FILE="$ProjectDir\sensor.conf"

Write-Host ""
Write-Host "========================================="
Write-Host " All builds complete!"
Write-Host "========================================="
Write-Host " Gateway: build_gateway\hello_dect\zephyr\zephyr.hex"
Write-Host " Anchor:  build_anchor\hello_dect\zephyr\zephyr.hex"
Write-Host " Sensor:  build_sensor\hello_dect\zephyr\zephyr.hex"
