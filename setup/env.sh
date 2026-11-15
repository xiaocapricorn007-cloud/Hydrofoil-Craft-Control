# Source this in every WSL shell that runs the simulator:
#   source <repo>/setup/env.sh
source /opt/ros/lyrical/setup.bash
[ -f "$HOME/hydrofoil_build/install/setup.bash" ] && source "$HOME/hydrofoil_build/install/setup.bash"
# WSLg GPU rendering: use Mesa's D3D12 driver on the NVIDIA adapter
# (without these, OpenGL falls back to llvmpipe software rendering)
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
# Python side of the project (validated model + controllers)
export HYDROFOIL_PY=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$HYDROFOIL_PY${PYTHONPATH:+:$PYTHONPATH}"
