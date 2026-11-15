# Adds the toolboxes the original jskoun/MPC_HydrofoilControl code needs to the
# existing MATLAB R2024b install.  Your license (1061615) already covers them;
# they are just not installed.
#
# RUN IN AN *ADMINISTRATOR* POWERSHELL (it writes to C:\Program Files):
#   powershell -ExecutionPolicy Bypass -File C:\Users\ragha\hydrofoil-mpc\setup\matlab\install_toolboxes.ps1
#
# Download size is several GB; C: needs roughly 10 GB free.
$ErrorActionPreference = "Stop"
$mpm = Join-Path $PSScriptRoot "mpm.exe"
if (-not (Test-Path $mpm)) {
    Invoke-WebRequest -Uri "https://www.mathworks.com/mpm/win64/mpm" -OutFile $mpm -UseBasicParsing
}
& $mpm install --release=R2024b --destination="C:\Program Files\MATLAB\R2024b" --products `
    Simulink `
    Control_System_Toolbox `
    Model_Predictive_Control_Toolbox `
    Simulink_Control_Design `
    Signal_Processing_Toolbox `
    Optimization_Toolbox
Write-Host "Done. Verify with:  matlab -batch ""ver"""
