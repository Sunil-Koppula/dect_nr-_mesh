$venvPath = "c:/ncs/.venv3.2.3"
$activateScript = "$venvPath/Scripts/Activate.ps1"
$CurrentDirectory = (Get-Location).Path.Replace('\', '/')

# Activate virtual environment if not already active
if (-not $env:VIRTUAL_ENV) {
    Write-Output "Activating virtual environment..."
    & $activateScript
} else {
    Write-Output "Virtual environment is already activated."
}

# Set environment variables
Set-Location -Path "c:/ncs/v3.2.3/"
$env:NCS_TOOLCHAIN_VERSION = "v3.2.3"
$env:PATH = "c:/ncs/toolchains/d2843cfba2/opt/bin;" + $env:PATH

# Clear CONF_FILE/BOARD_ROOT so they don't leak into MCUboot child build
Remove-Item Env:\CONF_FILE -ErrorAction SilentlyContinue
Remove-Item Env:\BOARD_ROOT -ErrorAction SilentlyContinue

# Build unified firmware (device type is determined at runtime via GPIO pins P0.21/P0.22)
Write-Output "Building firmware (device type selected by pins at runtime)..."
west build --build-dir "$CurrentDirectory/build" `
           --board nrf9151dk/nrf9151/ns `
           --sysbuild `
           $CurrentDirectory

# Return to original directory
Set-Location -Path $CurrentDirectory
