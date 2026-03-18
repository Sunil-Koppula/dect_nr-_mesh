param(
    [string[]]$skip = @()
)

# Normalize skip values to lowercase for safety
$skip = $skip | ForEach-Object { $_.ToLower() }

$ErrorActionPreference = "Stop"

Write-Output "Cleaning old build folders..."

# Map components to folders
$components = @{
    "sensor"  = "build_sensor"
    "anchor"  = "build_anchor"
    "gateway" = "build_gateway"
}

foreach ($key in $components.Keys) {
    $folder = $components[$key]

    if ($skip -contains $key) {
        Write-Output "Skipping clean for $key"
        continue
    }

    if (Test-Path $folder) {
        Remove-Item $folder -Recurse -Force
        Write-Output "Removed $folder"
    }
}

Write-Output "Starting builds..."

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Run builds conditionally
if ($skip -notcontains "sensor") {
    Write-Output "Building sensor..."
    & "$scriptDir\build_sensor.ps1"
} else {
    Write-Output "Skipped sensor build"
}

if ($skip -notcontains "anchor") {
    Write-Output "Building anchor..."
    & "$scriptDir\build_anchor.ps1"
} else {
    Write-Output "Skipped anchor build"
}

if ($skip -notcontains "gateway") {
    Write-Output "Building gateway..."
    & "$scriptDir\build_gateway.ps1"
} else {
    Write-Output "Skipped gateway build"
}

Write-Output "Build process completed."