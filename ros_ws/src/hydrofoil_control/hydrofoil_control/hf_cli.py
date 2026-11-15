"""hf: operator command line for the hydrofoil simulator.

VESSEL
  hf speed 10            speed setpoint [m/s]            (5 .. 15)
  hf accel 0.8 [0.5]     acceleration [and deceleration] limit [m/s^2]
  hf height 0.4          ride height, CG above water [m] (0.15 .. 0.55)
  hf pitch 1.0           pitch trim [deg]
  hf heel -2             heel trim [deg]
  hf mode auto|assist|manual
  hf foil fs 5           fix one foil (fs | fp | a) at an angle [deg]
  hf foil all 3.3 3.3 3.0   fix all three
  hf foil fs off         release one foil      /  hf foil off   release all
  hf status              one status snapshot   /  hf watch      continuous
Foil overrides only act in ASSIST or MANUAL mode.

ENVIRONMENT   (directions: sea and wind = where they come FROM, degrees clockwise
               from the bow, 0 = head-on; current = where it flows TO, 0 = with you)
  hf sea 0.4 3.0 [dir] [spread]   irregular sea: Hs [m], peak period Tp [s]
  hf sea regular 0.1 3.0 [dir]    regular wave: amplitude [m], frequency [rad/s]
  hf sea calm
  hf wind 10 [dir] [gust]         speed [m/s], direction, gust intensity 0..0.6
  hf current 0.5 [dir]            speed [m/s], direction
  hf water 15 [35]                temperature [C], salinity [psu] -> density
  hf rho 1000                     water density directly [kg/m^3]
  hf env                          environment status

SIMULATOR
  hf physics v2                   plant model fidelity: ref (validated reference model),
                                  v2 (all phase 4 upgrades), or switch names, e.g.
                                  hf physics downwash,ventilation   (see GAZEBO.md)
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String

NAN = float("nan")


def _publish(node, msg_type, topic, msg):
    pub = node.create_publisher(msg_type, topic, 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print(f"no subscriber on {topic}: is the simulator running?")
        return False
    # Discovery is asynchronous: the subscriber may not have matched this
    # publisher yet even though we already see it.  Wait briefly, then send.
    t1 = time.time()
    while time.time() - t1 < 0.4:
        rclpy.spin_once(node, timeout_sec=0.05)
    pub.publish(msg)
    for _ in range(8):
        rclpy.spin_once(node, timeout_sec=0.02)
    return True


def _once(node, topic, timeout=3.0):
    box = []
    sub = node.create_subscription(String, topic, lambda m: box.append(m.data), 10)
    t0 = time.time()
    while not box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return json.loads(box[-1]) if box else None


def _fmt(s):
    ov = ["--" if v is None else f"{v:.1f}" for v in s["foil_override_deg"]]
    fs = f"  !! {s['failsafe']}" if s["failsafe"] else (f"  ! {s['warning']}" if s.get("warning") else "")
    return (f"t={s['t']:7.1f}s  {s['mode']:6s} speed {s['speed']:5.2f} (ref {s['speed_ref']:5.2f}, set {s['speed_set']:5.2f}) m/s"
            f" | height {s['height']:.3f} (set {s['height_set']:.2f}) m | roll {s['roll_deg']:+5.2f} pitch {s['pitch_deg']:+5.2f} deg"
            f" | foils {s['foil_cmd_deg'][0]:5.2f} {s['foil_cmd_deg'][1]:5.2f} {s['foil_cmd_deg'][2]:5.2f} deg"
            f" (override {ov[0]} {ov[1]} {ov[2]}) | rpm {s['rpm_cmd']:6.0f} | min sub {s['min_submergence']:.2f} m{fs}")


def _fmt_env(e):
    s, w, c, wa = e["sea"], e["wind"], e["current"], e["water"]
    if s["kind"] == "irregular":
        sea = (f"irregular Hs {s['hs']:.2f} m (realised {s['hs_realised']:.2f}), Tp {s['tp']:.1f} s, "
               f"from {s['dir']:.0f} deg" + (", spread" if s.get("spread") else ""))
    elif s["kind"] == "regular":
        sea = f"regular amp {s['amp']:.2f} m, omega {s['omega']:.2f} rad/s, from {s['dir']:.0f} deg"
    else:
        sea = "calm"
    ts = "" if wa["temp"] is None else f" ({wa['temp']:.0f} C, {wa['salinity']:.0f} psu)"
    return (f"t={e.get('t', 0):7.1f}s  sea: {sea} | wind {w['speed']:.1f} m/s from {w['dir']:.0f} deg, gust {w['gust']:.2f}"
            f" (now N {w['now'][0]:+.1f} E {w['now'][1]:+.1f}) | current {c['speed']:.2f} m/s to {c['dir']:.0f} deg"
            f" | water {wa['rho']:.1f} kg/m3{ts} | air drag {'on' if e['aero'] else 'off'}"
            f" | physics {e.get('physics') or 'as launched'}")


def _env(node, upd):
    return _publish(node, String, "/env/set", String(data=json.dumps(upd)))


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help", "help"):
        print(__doc__)
        return
    rclpy.init()
    node = Node("hf_cli")
    f = lambda i, d=None: float(a[i]) if len(a) > i else d
    try:
        c = a[0].lower()
        if c == "speed":
            _publish(node, Float64, "/cmd/speed", Float64(data=f(1)))
        elif c == "height":
            _publish(node, Float64, "/cmd/ride_height", Float64(data=f(1)))
        elif c == "pitch":
            _publish(node, Float64, "/cmd/pitch_trim", Float64(data=f(1)))
        elif c == "heel":
            _publish(node, Float64, "/cmd/heel_trim", Float64(data=f(1)))
        elif c == "accel":
            _publish(node, Float64MultiArray, "/cmd/accel_limit",
                     Float64MultiArray(data=[float(x) for x in a[1:3]]))
        elif c == "mode":
            _publish(node, String, "/cmd/mode", String(data=a[1].upper()))
        elif c == "foil":
            idx = {"fs": 0, "fp": 1, "a": 2}
            if a[1].lower() == "off":
                v = [NAN, NAN, NAN]
            elif a[1].lower() == "all":
                v = [float(x) for x in a[2:5]]
            else:
                # single foil: keep the others as they are (read current overrides)
                cur = _once(node, "/hydrofoil/status") or {"foil_override_deg": [None, None, None]}
                v = [NAN if x is None else float(x) for x in cur["foil_override_deg"]]
                v[idx[a[1].lower()]] = NAN if a[2].lower() == "off" else float(a[2])
            _publish(node, Float64MultiArray, "/cmd/foil_override", Float64MultiArray(data=v))
        elif c == "sea":
            if a[1].lower() == "calm":
                _env(node, {"sea": {"kind": "calm"}})
            elif a[1].lower() == "regular":
                _env(node, {"sea": {"kind": "regular", "amp": f(2), "omega": f(3), "dir": f(4, 0.0)}})
            else:
                _env(node, {"sea": {"kind": "irregular", "hs": f(1), "tp": f(2), "dir": f(3, 0.0),
                                    "spread": len(a) > 4 and a[4].lower() in ("spread", "1", "true", "yes")}})
        elif c == "wind":
            _env(node, {"wind": {"speed": f(1), "dir": f(2, 0.0), "gust": f(3, 0.0)}})
        elif c == "current":
            _env(node, {"current": {"speed": f(1), "dir": f(2, 0.0)}})
        elif c == "water":
            _env(node, {"water": {"temp": f(1), "salinity": f(2, 35.0)}})
        elif c == "rho":
            _env(node, {"water": {"rho": f(1)}})
        elif c == "physics":
            _env(node, {"physics": a[1] if len(a) > 1 else "ref"})
        elif c == "env":
            e = _once(node, "/env/status")
            print(_fmt_env(e) if e else "no /env/status received")
        elif c == "status":
            s = _once(node, "/hydrofoil/status")
            print(_fmt(s) if s else "no /hydrofoil/status received")
        elif c == "watch":
            node.create_subscription(String, "/hydrofoil/status",
                                     lambda m: print(_fmt(json.loads(m.data)), flush=True), 10)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
        else:
            print(__doc__)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
