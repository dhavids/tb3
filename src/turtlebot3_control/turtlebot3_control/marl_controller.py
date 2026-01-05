#!/usr/bin/env python3
import os
import sys
import math
import threading
import argparse
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82


def constrain(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_two_floats(line: str) -> Optional[Tuple[float, float]]:
    s = line.strip()
    if not s:
        return None
    s = s.replace(",", " ")
    parts = [p for p in s.split() if p]
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


class MarlController(Node):
    def __init__(self, scan_lines: int, print_hz: float, degrees_yaw: bool) -> None:
        super().__init__("marl_controller")

        self._scan_lines = scan_lines
        self._print_hz = print_hz
        self._degrees_yaw = degrees_yaw

        self._model = os.environ.get("TURTLEBOT3_MODEL", "burger").lower().strip()
        if self._model not in ("burger", "waffle", "waffle_pi"):
            self.get_logger().warn(
                f"Unknown TURTLEBOT3_MODEL='{self._model}', defaulting to 'burger' limits."
            )
            self._model = "burger"

        self._lock = threading.Lock()
        self._target_lin = 0.0
        self._target_ang = 0.0
        self._have_cmd = False

        self._yaw_rad: Optional[float] = None
        self._scan_ranges: Optional[List[float]] = None

        self._cmd_pub = self.create_publisher(
            Twist, "cmd_vel", QoSProfile(depth=10)
        )

        self.create_subscription(Odometry, "odom", self._on_odom, QoSProfile(depth=10))
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)

        self._publish_hz = 10.0
        self.create_timer(1.0 / self._publish_hz, self._publish_cmd)
        self.create_timer(1.0 / self._print_hz, self._print_status)

        self._stop_event = threading.Event()
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        yaw_mode = "degrees" if self._degrees_yaw else "radians"
        self.get_logger().info(
            f"marl_controller started | scan_lines={self._scan_lines}, "
            f"print_hz={self._print_hz}, yaw={yaw_mode}"
        )

    def _lin_limit(self) -> float:
        return BURGER_MAX_LIN_VEL if self._model == "burger" else WAFFLE_MAX_LIN_VEL

    def _ang_limit(self) -> float:
        return BURGER_MAX_ANG_VEL if self._model == "burger" else WAFFLE_MAX_ANG_VEL

    def _clamp_cmd(self, lin: float, ang: float) -> Tuple[float, float]:
        return (
            constrain(lin, -self._lin_limit(), self._lin_limit()),
            constrain(ang, -self._ang_limit(), self._ang_limit()),
        )

    def _stdin_loop(self) -> None:
        try:
            for line in sys.stdin:
                if self._stop_event.is_set():
                    break
                parsed = parse_two_floats(line)
                if parsed is None:
                    continue
                lin, ang = self._clamp_cmd(*parsed)
                with self._lock:
                    self._target_lin = lin
                    self._target_ang = ang
                    self._have_cmd = True
        finally:
            self._stop_event.set()

    def _publish_cmd(self) -> None:
        with self._lock:
            lin = self._target_lin
            ang = self._target_ang

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self._cmd_pub.publish(twist)

        if self._stop_event.is_set():
            rclpy.shutdown()

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._yaw_rad = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            self._scan_ranges = None
            return

        step = n / float(self._scan_lines)

        forward_idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        forward_idx = max(0, min(n - 1, forward_idx))

        sampled = []
        for i in range(self._scan_lines):
            idx = int(round(forward_idx + i * step)) % n
            r = ranges[idx]
            sampled.append(r if math.isfinite(r) else float("inf"))

        self._scan_ranges = sampled

    def _print_status(self) -> None:
        with self._lock:
            lin = self._target_lin
            ang = self._target_ang
            src = "stdin" if self._have_cmd else "default"

        if self._yaw_rad is None:
            yaw_str = "yaw: N/A"
        else:
            if self._degrees_yaw:
                yaw_val = self._yaw_rad * 180.0 / math.pi
                yaw_str = f"yaw: {yaw_val:+.1f} deg"
            else:
                yaw_str = f"yaw: {self._yaw_rad:+.3f} rad"

        scan_str = (
            "scan: N/A"
            if self._scan_ranges is None
            else f"scan[{len(self._scan_ranges)}]: {self._scan_ranges}"
        )

        print(
            f"[marl_controller] cmd({src}): "
            f"lin {lin:+.3f}, ang {ang:+.3f} | {yaw_str} | {scan_str}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()

    # Positional arguments
    parser.add_argument("scan_lines", nargs="?", type=int, default=36)
    parser.add_argument("print_hz", nargs="?", type=float, default=2.0)

    # Flag arguments
    parser.add_argument("--scan_lines", dest="scan_lines_flag", type=int)
    parser.add_argument("--print_hz", dest="print_hz_flag", type=float)
    parser.add_argument("--degrees_yaw", action="store_true")

    args = parser.parse_args()

    scan_lines = args.scan_lines_flag if args.scan_lines_flag is not None else args.scan_lines
    print_hz = args.print_hz_flag if args.print_hz_flag is not None else args.print_hz

    if scan_lines <= 0:
        print("[marl_controller] ERROR: scan_lines must be > 0")
        sys.exit(1)

    if print_hz <= 0.0:
        print("[marl_controller] ERROR: print_hz must be > 0")
        sys.exit(1)

    rclpy.init()
    node = MarlController(
        scan_lines=scan_lines,
        print_hz=print_hz,
        degrees_yaw=args.degrees_yaw,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
