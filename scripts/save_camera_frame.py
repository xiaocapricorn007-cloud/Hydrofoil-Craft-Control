"""Save one frame of the Gazebo camera (/hydrofoil/camera, bridged to ROS) as a
PNG, with no image libraries (pure zlib writer).
   python3 save_camera_frame.py out.png"""
import struct
import sys
import zlib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    out = sys.argv[1]
    rclpy.init()
    node = Node("save_camera_frame")
    box = []
    node.create_subscription(Image, "/hydrofoil/camera", lambda m: box.append(m), 2)
    while rclpy.ok() and len(box) < 2:          # skip the first frame
        rclpy.spin_once(node, timeout_sec=0.5)
    m = box[-1]
    data = bytes(m.data)
    if m.encoding == "bgr8":
        data = b"".join(data[i + 2:i + 3] + data[i + 1:i + 2] + data[i:i + 1] for i in range(0, len(data), 3))
    if m.step != m.width * 3:
        data = b"".join(data[y * m.step:y * m.step + m.width * 3] for y in range(m.height))
    write_png(out, m.width, m.height, data)
    print(f"saved {m.width}x{m.height} {m.encoding} frame to {out}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
