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
$env:BOARD_ROOT = "$CurrentDirectory"
$env:CONF_FILE = "$CurrentDirectory/prj.conf"
$env:NCS_TOOLCHAIN_VERSION = "v3.2.3"
$env:PATH = "c:/ncs/toolchains/d2843cfba2/opt/bin;" + $env:PATH

# Build ANCHOR
Write-Output "Building ANCHOR (DEVICE_TYPE=2)..."
west build --build-dir "$CurrentDirectory/build_anchor" `
           --board nrf9151dk/nrf9151/ns `
           --pristine `
           --sysbuild `
           $CurrentDirectory `
           -- "-DEXTRA_CONF_FILE=anchor.conf"

# Return to original directory
Set-Location -Path $CurrentDirectory
