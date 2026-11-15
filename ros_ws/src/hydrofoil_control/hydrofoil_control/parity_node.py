"""Phase 1 parity run: drive the Gazebo craft with exactly the controller,
reference and timing used by the Python simulator (hydrofoil/sim.py), log the
trajectory and save it for comparison (scripts/compare_gz.py).

  ros2 run hydrofoil_control parity_node --ros-args -p controller:=MPC-IDP
       -p scenario:=step -p duration:=4.0 -p out:=/tmp/gz_step.npz
"""
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import Float32Array

from .gz_state import odom_to_state, DBG_RELEASE, DBG_ACT, DBG_H
from hydrofoil.design import build_models, build_controllers, NOMINAL_IDX
from hydrofoil.params import Params
from hydrofoil.sim import make_reference


class ParityNode(Node):
    def __init__(self):
        super().__init__("hydrofoil_parity")
        self.declare_parameter("controller", "MPC-IDP")
        self.declare_parameter("scenario", "step")
        self.declare_parameter("duration", 4.0)
        self.declare_parameter("out", "/tmp/gz_parity.npz")
        name = self.get_parameter("controller").value
        scen = self.get_parameter("scenario").value
        self.T = float(self.get_parameter("duration").value)
        self.out = self.get_parameter("out").value

        self.get_logger().info(f"building models + controller {name} ...")
        models = build_models()
        self.nom = models[NOMINAL_IDX]
        self.ctrl = build_controllers(models, which=(name,))[name]
        self.ctrl.reset()
        self.ref = make_reference("step", dz=-0.2, t0=0.5) if scen == "step" else make_reference("none")
        self.P = Params()
        self.xi = (self.nom["rpm"] - self.P.rpm_trim_ff) / self.P.pi_I
        self.u_cmd = self.nom["delta"].copy()
        self.rpm = float(self.nom["rpm"])
        self.release = -1.0
        self.act = self.nom["delta"].copy()
        self.h = np.zeros(3)
        self.k_ctrl = 0
        self.rows = []
        self.done = False

        self.pubs = [self.create_publisher(Float64, f"/hydrofoil/cmd/{n}", 10)
                     for n in ("foil_fs", "foil_fp", "foil_a", "rpm")]
        self.create_subscription(Float32Array, "/hydrofoil/debug", self.on_debug, 50)
        self.create_subscription(Odometry, "/hydrofoil/odom", self.on_odom, 50)
        self.get_logger().info("ready, waiting for /hydrofoil/odom")

    def on_debug(self, msg):
        d = msg.data
        if len(d) >= 7:
            self.release = float(d[DBG_RELEASE])
            self.act = np.array(d[DBG_ACT], float)
            self.h = np.array(d[DBG_H], float)

    def send(self):
        for pub, v in zip(self.pubs, [*self.u_cmd, self.rpm]):
            pub.publish(Float64(data=float(v)))

    def on_odom(self, msg):
        if self.done:
            return
        s = odom_to_state(msg)
        if self.release < 0:
            self.send()            # the first command releases the craft from the rail
            return
        t = s["t"] - self.release
        y = np.array([s["z"], s["phi"], s["theta"], s["w"], s["p"], s["q"]])
        if t >= self.k_ctrl * 0.1 - 5e-4:           # 10 Hz foil controller, ZOH
            tk = self.k_ctrl * 0.1
            rf = [self.ref(tk + i * 0.1) for i in range(10)]
            self.u_cmd = np.asarray(self.ctrl(y, self.ref(tk), rf, speed=s["u"]), float)
            self.k_ctrl += 1
        e = 7.0 - s["u"]                             # 100 Hz speed PI (one odom msg = 10 ms)
        self.rpm = self.P.rpm_trim_ff + self.P.pi_P * e + self.P.pi_I * self.xi
        self.xi += e * self.P.pi_Ts
        self.send()
        self.rows.append([t, s["z"], s["phi"], s["theta"], s["u"], s["w"], s["p"], s["q"],
                          self.rpm, self.ref(t)[0], *self.act, *self.u_cmd, *self.h,
                          s["v"], s["r"], s["psi"]])
        if t >= self.T:
            a = np.array(self.rows)
            np.savez(self.out, t=a[:, 0], z=a[:, 1], phi=a[:, 2], theta=a[:, 3], u=a[:, 4], w=a[:, 5],
                     p=a[:, 6], q=a[:, 7], rpm=a[:, 8], zref=a[:, 9], act=a[:, 10:13], cmd=a[:, 13:16],
                     h=a[:, 16:19], v=a[:, 19], r=a[:, 20], psi=a[:, 21])
            self.get_logger().info(f"saved {len(a)} samples to {self.out}")
            self.done = True


def main():
    rclpy.init()
    node = ParityNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():            # a real error; shutdown races are not
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
