"""Phase 2 control node: Supervisor (speed + height MPC, guidance, operator
overrides, watchdog) driving the Gazebo craft.

Operator topics (all under /cmd):
  /cmd/speed          std_msgs/Float64   speed setpoint [m/s]         (default 7)
  /cmd/accel_limit    std_msgs/Float64MultiArray  [accel] or [accel, decel] in m/s^2
  /cmd/ride_height    std_msgs/Float64   CG height above water [m]    (default 0.3)
  /cmd/pitch_trim     std_msgs/Float64   [deg]
  /cmd/heel_trim      std_msgs/Float64   [deg]
  /cmd/mode           std_msgs/String    AUTO | ASSIST | MANUAL
  /cmd/foil_override  std_msgs/Float64MultiArray  [FS, FP, A] in deg, NaN = release
Outputs:
  /hydrofoil/cmd/foil_fs|foil_fp|foil_a|rpm   std_msgs/Float64 (to the plugin)
  /hydrofoil/status   std_msgs/String (JSON, 10 Hz)

Parameters:
  scenario_json  optional scripted operator actions, e.g.
                 '[[1.0,"speed",11],[1.0,"accel",[0.8,0.5]],[8,"mode","ASSIST"]]'
                 times are seconds after the craft is released
  duration       stop after this many seconds (0 = run forever)
  log            path of an .npz log written at the end ('' = none)
"""
import json
from dataclasses import asdict

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, String
from ros_gz_interfaces.msg import Float32Array

from .gz_state import odom_to_state, DBG_RELEASE, DBG_ACT, DBG_H
from hydrofoil.design import build_models, NOMINAL_IDX
from hydrofoil.supervisor import Supervisor


def apply_action(sup, name, value):
    if name == "speed":
        sup.set_speed(value)
    elif name == "accel":
        v = list(value) if isinstance(value, (list, tuple)) else [value]
        sup.set_accel(*v[:2])
    elif name == "height":
        sup.set_height(value)
    elif name == "pitch":
        sup.set_trim(pitch_deg=value)
    elif name == "heel":
        sup.set_trim(heel_deg=value)
    elif name == "mode":
        return sup.set_mode(value)
    elif name == "foil":
        sup.set_override(value)
    else:
        return False
    return True


class ControlNode(Node):
    def __init__(self):
        super().__init__("hydrofoil_control")
        self.declare_parameter("scenario_json", "")
        self.declare_parameter("duration", 0.0)
        self.declare_parameter("log", "")
        sj = self.get_parameter("scenario_json").value
        if sj.endswith(".json"):              # a file path is also accepted
            with open(sj) as f:
                sj = f.read()
        self.events = sorted(json.loads(sj), key=lambda e: e[0]) if sj else []
        self.T = float(self.get_parameter("duration").value)
        self.log_path = self.get_parameter("log").value

        self.get_logger().info("building linear model array (21 speeds) and MPC ...")
        models = build_models()
        self.nom = models[NOMINAL_IDX]
        self.sup = Supervisor(models)
        self.cmd = np.r_[self.nom["delta"], self.nom["rpm"]]
        self.release = -1.0
        self.act = self.nom["delta"].copy()
        self.h = None
        self.k = 0
        self.rows = []
        self.done = False
        self.last_fs = ""

        self.pubs = [self.create_publisher(Float64, f"/hydrofoil/cmd/{n}", 10)
                     for n in ("foil_fs", "foil_fp", "foil_a", "rpm")]
        self.status_pub = self.create_publisher(String, "/hydrofoil/status", 10)
        self.env_pub = self.create_publisher(String, "/env/set", 10)
        self.sonar = self.zeta = float("nan")
        self.create_subscription(Float32Array, "/hydrofoil/debug", self.on_debug, 50)
        self.create_subscription(Odometry, "/hydrofoil/odom", self.on_odom, 50)
        f64 = lambda topic, name: self.create_subscription(
            Float64, topic, lambda m, n=name: self.operator(n, m.data), 10)
        f64("/cmd/speed", "speed"); f64("/cmd/ride_height", "height")
        f64("/cmd/pitch_trim", "pitch"); f64("/cmd/heel_trim", "heel")
        self.create_subscription(Float64MultiArray, "/cmd/accel_limit",
                                 lambda m: self.operator("accel", list(m.data)), 10)
        self.create_subscription(Float64MultiArray, "/cmd/foil_override",
                                 lambda m: self.operator("foil", list(m.data)), 10)
        self.create_subscription(String, "/cmd/mode", lambda m: self.operator("mode", m.data), 10)
        self.get_logger().info("ready: design speed 7 m/s, ride height 0.3 m, mode AUTO")

    def operator(self, name, value):
        if name == "env":                     # scripted environment change -> env node
            self.env_pub.publish(String(data=json.dumps(value)))
            self.get_logger().info(f"operator: env = {value}")
            return
        ok = apply_action(self.sup, name, value)
        self.get_logger().info(f"operator: {name} = {value}" + ("" if ok else "  (REJECTED)"))

    def on_debug(self, msg):
        d = msg.data
        if len(d) >= 7:
            self.release = float(d[DBG_RELEASE])
            self.act = np.array(d[DBG_ACT], float)
            self.h = np.array(d[DBG_H], float)
        if len(d) >= 21:
            self.sonar, self.zeta = float(d[19]), float(d[20])

    def send(self):
        for pub, v in zip(self.pubs, self.cmd):
            pub.publish(Float64(data=float(v)))

    def on_odom(self, msg):
        if self.done:
            return
        s = odom_to_state(msg)
        if self.release < 0:
            self.send()                       # trim command releases the craft from the rail
            return
        t = s["t"] - self.release
        if t >= self.k * 0.1 - 5e-4:          # 10 Hz
            while self.events and self.events[0][0] <= t + 1e-6:
                _, name, value = self.events.pop(0)
                self.operator(name, value)
            y = np.array([s["z"], s["phi"], s["theta"], s["u"], s["w"], s["p"], s["q"]])
            if self.k == 0:
                self.sup.mpc.reset(y0=y, u0=self.cmd)
            self.cmd = self.sup.step(t, y, self.h)
            self.send()
            self.k += 1
            st = self.sup.status
            self.status_pub.publish(String(data=json.dumps(asdict(st))))
            if st.failsafe and st.failsafe != self.last_fs:
                self.get_logger().warning("FAILSAFE: " + st.failsafe)
            self.last_fs = st.failsafe
        if self.log_path:
            g = self.sup.guid
            hmin = float(np.min(self.h)) if self.h is not None else np.nan
            self.rows.append([t, s["z"], s["phi"], s["theta"], s["u"], s["w"], s["p"], s["q"],
                              self.cmd[3], -g.h_ref, g.u_ref, hmin, *self.act, *self.cmd[:3],
                              {"AUTO": 0, "ASSIST": 1, "MANUAL": 2}[self.sup.mode],
                              self.sonar, self.zeta])
        if self.T > 0 and t >= self.T:
            self.finish()

    def finish(self):
        if self.log_path and self.rows:
            a = np.array(self.rows)
            np.savez(self.log_path, t=a[:, 0], z=a[:, 1], phi=a[:, 2], theta=a[:, 3], u=a[:, 4],
                     w=a[:, 5], p=a[:, 6], q=a[:, 7], rpm=a[:, 8], zref=a[:, 9], uref=a[:, 10],
                     hmin=a[:, 11], act=a[:, 12:15], cmd=a[:, 15:18], mode=a[:, 18],
                     sonar=a[:, 19], zeta=a[:, 20])
            self.get_logger().info(f"saved {len(a)} samples to {self.log_path}")
        self.done = True


def main():
    rclpy.init()
    node = ControlNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.finish()
    except Exception:
        if rclpy.ok():            # a real error; shutdown races are not
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
