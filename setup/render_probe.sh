#!/usr/bin/env bash
# Render check: GUI + chase camera with a steep sea, screenshots of both.
#   render_probe.sh <tag> [world] [hf sea arguments]
# GUI picture -> figures/render/<tag>_gui.png, chase camera -> figures/render/<tag>_cam.png
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TAG=$1; WORLD=${2:-foiling_cam.sdf}
SEA=${3:-"regular 0.25 2.5 30"}
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
OUT=$S/figures/render; mkdir -p $OUT/${TAG}_gui; rm -f $OUT/${TAG}_gui/*.png
ros2 launch hydrofoil_bringup foiling.launch.py world:=$WORLD > /tmp/render_probe.log 2>&1 &
sleep 8
ros2 run hydrofoil_control env_node > /tmp/render_env.log 2>&1 &
sleep 2
ros2 run hydrofoil_control control_node --ros-args -p duration:=60.0 > /tmp/render_ctl.log 2>&1 &
sleep 3
ros2 run hydrofoil_control hf sea $SEA > /dev/null 2>&1
[ -n "$SPEED" ] && ros2 run hydrofoil_control hf speed $SPEED > /dev/null 2>&1
sleep ${WAIT:-14}
timeout 10 gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
   --timeout 5000 --req "data: \"$OUT/${TAG}_gui\"" > /dev/null
timeout 20 python3 $S/scripts/save_camera_frame.py $OUT/${TAG}_cam.png > /dev/null
grep -E "new sea|rror" /tmp/render_env.log | cut -c1-160 | head -3
grep -iE "error|exception" /tmp/render_probe.log | cut -c1-200 | head -5
bash $S/setup/stop_sim.sh
for f in $OUT/${TAG}_gui/*.png; do mv "$f" $OUT/${TAG}_gui.png; done
rmdir $OUT/${TAG}_gui 2>/dev/null
ls $OUT | grep "^$TAG"
