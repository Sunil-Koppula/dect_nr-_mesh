$venvPath = "c:/ncs/.venv3.2.3"
$pythonPath = "$venvPath/Scripts/python.exe"

# Create Python virtual environment if it does not exist
if (-not (Test-Path "$venvPath")) {
    Write-Output "Creating Python virtual environment at $venvPath..."
    python -m venv "$venvPath"
} else {
    Write-Output "Virtual environment already exists."
}

# Activate the virtual environment for the current session
Write-Output "Activating virtual environment..."
& "$venvPath/Scripts/Activate.ps1"

# Upgrade pip and install required Python packages
Write-Output "Upgrading pip and installing Python dependencies..."
& $pythonPath -m pip install --upgrade pip setuptools wheel

# Install west and Zephyr/Nordic dependencies
Write-Output "Installing west..."
& $pythonPath -m pip install west

Write-Output "Installing Zephyr dependencies..."
& $pythonPath -m pip install -r "c:/ncs/v3.2.3/zephyr/scripts/requirements.txt"

Write-Output "Installing Nordic SDK dependencies..."
& $pythonPath -m pip install -r "c:/ncs/v3.2.3/nrf/scripts/requirements.txt"

Write-Output "Environment setup complete."
