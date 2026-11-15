"""Conversion between Gazebo/ROS conventions and the model's NED / FRD.

Gazebo world: z up; link frame FLU (x fwd, y left, z up).
Model:        NED world (z down); body FRD.  Related by a 180 deg rotation
about x: vectors flip y and z; roll is unchanged, pitch and yaw change sign.
"""
import math
import os
import pathlib
import sys

# the validated Python model lives outside the ROS workspace
# repository root (holds the `hydrofoil` Python package); setup/env.sh exports it.
# Fallback: follow the symlinked source file back to the repository.
_HP = os.environ.get("HYDROFOIL_PY") or str(pathlib.Path(os.path.realpath(__file__)).parents[4])
if _HP not in sys.path:
    sys.path.insert(0, _HP)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def odom_to_state(msg):
    """nav_msgs/Odometry (world pose, body-FLU twist) ->
    dict(t, xN, yE, z, phi, theta, psi, u, v, w, p, q, r) in NED/FRD."""
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    x, y, z, w = o.x, o.y, o.z, o.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    v = msg.twist.twist.linear
    a = msg.twist.twist.angular
    return dict(t=stamp_to_sec(msg.header.stamp), xN=p.x, yE=-p.y, z=-p.z,
                phi=roll, theta=-pitch, psi=-yaw,
                u=v.x, v=-v.y, w=-v.z, p=a.x, q=-a.y, r=-a.z)


# indices into the /hydrofoil/debug vector published by the FoilDynamics plugin
DBG_RELEASE = 0
DBG_ACT = slice(1, 4)
DBG_H = slice(4, 7)
DBG_ALPHA = slice(7, 10)
DBG_LIFT = slice(10, 13)
DBG_THRUST = 13
DBG_RPM = 14
DBG_CMD = slice(15, 18)
DBG_T = 18
DBG_PHYS = 27        # physics mask (phase 4)
DBG_UPWASH = 28      # wake upwash at the aft foil [m/s]
