#!/usr/bin/env bash
# What the installed Gazebo offers for rendering (read-only).
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
echo "--- gz share dirs"; ls -d /usr/share/gz* /opt/ros/lyrical/share/gz* 2>/dev/null
echo "--- sky / cubemap textures"; find / -xdev \( -iname "*skybox*" -o -iname "*.dds" \) 2>/dev/null | grep -v proc | head -20
echo "--- particle / smoke textures"; find / -xdev \( -iname "*smoke*" -o -iname "*particle*" \) 2>/dev/null | grep -iE "png|jpg|sdf|material" | head -20
echo "--- example worlds with sky / particles / shader / waves"
for d in /usr/share/gz /opt/ros/lyrical/share /opt/ros/lyrical/opt; do
  [ -d $d ] && grep -rlE "<sky|particle_emitter|ShaderParam|waves" $d --include=*.sdf 2>/dev/null | head -20
done
echo "--- gui configs"; find / -xdev -name "gui.config" 2>/dev/null | head
echo "--- system plugins"; find / -xdev -name "libgz-sim*-system*.so" 2>/dev/null | grep -iE "particle|shader|visual" | head
