"""Environment node (phase 3): the operator's sea state, wind, current and
water settings, turned into what the Gazebo plugin needs.

  in : /env/set     std_msgs/String  JSON with any of the keys
         {"sea": {"hs": 0.4, "tp": 3.0, "dir": 0, "spread": false, "ramp": 5}}
         {"sea": {"amp": 0.1, "omega": 3.0, "dir": 0}}      regular wave
         {"sea": {"kind": "calm"}}
         {"wind": {"speed": 10, "dir": 90, "gust": 0.2}}
         {"current": {"speed": 0.5, "dir": 180}}
         {"water": {"temp": 15, "salinity": 35}}  or  {"water": {"rho": 1000}}
         {"physics": "v2"}   plant model fidelity (phase 4): "ref", "v2", a mask,
                             or switch names "downwash,ventilation"
       directions: sea and wind = where they come FROM, degrees clockwise from the
       bow (0 = head-on); current = where it flows TO (0 = with the craft)
  out: /hydrofoil/env/params  [rho, aero, rho_air, wind_n, wind_e, cur_n, cur_e, physics mask (-1 = keep)]  10 Hz
       /hydrofoil/env/waves   [id, ramp, (amp, omega, psi, phase) x N]  on change + 1 Hz
       /env/status            std_msgs/String JSON, 2 Hz

The time base is the plugin's wave clock (seconds since the craft was released),
read from /hydrofoil/debug, so that a scripted scenario is repeatable.
"""
import json

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import String
from ros_gz_interfaces.msg import Float32Array

from . import gz_state  # noqa: F401  (adds the hydrofoil python package to sys.path)
from hydrofoil.environment import Environment

DBG_WAVE_TIME = 22


class EnvNode(Node):
    def __init__(self):
        super().__init__("hydrofoil_env")
        self.declare_parameter("seed", 0)
        self.declare_parameter("aero", True)
        self.declare_parameter("initial", "")
        self.declare_parameter("physics", "")
        self.env = Environment(seed=int(self.get_parameter("seed").value),
                               aero=bool(self.get_parameter("aero").value))
        self.tw = 0.0
        self.k = -1
        self.pending = []
        init = self.get_parameter("initial").value
        if init:
            self.pending.append(json.loads(init))
        phys = str(self.get_parameter("physics").value)
        if phys:
            self.pending.append({"physics": phys})
        self.par_pub = self.create_publisher(Float32Array, "/hydrofoil/env/params", 10)
        self.wav_pub = self.create_publisher(Float32Array, "/hydrofoil/env/waves", 10)
        self.st_pub = self.create_publisher(String, "/env/status", 10)
        self.create_subscription(String, "/env/set", self.on_set, 20)
        self.create_subscription(Float32Array, "/hydrofoil/debug", self.on_debug, 50)
        self.get_logger().info(f"ready: calm sea, no wind, rho {self.env.water['rho']:.1f}, "
                               f"air drag {'on' if self.env.aero else 'off'}")

    def on_set(self, msg):
        try:
            upd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"bad /env/set JSON: {e}")
            return
        self.pending.append(upd)

    def send_waves(self):
        c = self.env.wave_components()
        ramp = float(self.env.sea.get("ramp", 5.0))
        data = [float(self.env.wave_id), ramp] + [float(x) for x in c.ravel()]
        self.wav_pub.publish(Float32Array(data=data))

    def on_debug(self, msg):
        if len(msg.data) <= DBG_WAVE_TIME:
            return
        self.tw = float(msg.data[DBG_WAVE_TIME])
        k = int(np.floor(self.tw / 0.1 + 1e-6))
        if k == self.k and not self.pending:
            return
        for upd in self.pending:
            try:
                changed = self.env.apply(upd, self.tw)
            except (ValueError, TypeError, KeyError) as e:
                self.get_logger().error(f"rejected {json.dumps(upd)}: {e}")
                continue
            self.get_logger().info(f"set {json.dumps(upd)}" + ("  -> new sea" if changed else ""))
            if changed:
                self.send_waves()
        self.pending = []
        if k != self.k:
            self.k = k
            wind = self.env.step(self.tw)
            cur = self.env.current_vec()
            e = self.env
            self.par_pub.publish(Float32Array(data=[
                float(e.water["rho"]), 1.0 if e.aero else 0.0, 1.225,
                float(wind[0]), float(wind[1]), float(cur[0]), float(cur[1]),
                float(-1 if e.physics is None else e.physics.mask())]))
            if k % 10 == 0 and self.env.wave_id > 0:
                self.send_waves()                 # re-send: the plugin ignores a known id
            if k % 5 == 0:
                st = self.env.status()
                st["t"] = round(self.tw, 2)
                self.st_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = EnvNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
