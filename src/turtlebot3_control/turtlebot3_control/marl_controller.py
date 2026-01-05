#!/usr/bin/env python3
import os
import sys
import math
import threading
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
    # yaw (z-axis rotation), ROS standard ENU
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_two_floats(line: str) -> Optional[Tuple[float, float]]:
    """
    Accepts:
      "0.1 0.2"
      "0.1,0.2"
      "0.1\t0.2"
    Ignores surrounding whitespace.
    """
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
    def __init__(self) -> None:
        super().__init__("marl_controller")

        self._model = os.environ.get("TURTLEBOT3_MODEL", "burger").lower().strip()
        if self._model not in ("burger", "waffle", "waffle_pi"):
            self.get_logger().warn(
                f"Unknown TURTLEBOT3_MODEL='{self._model}', defaulting to 'burger' limits."
            )
            self._model = "burger"

        # Velocity command state (set by stdin thread, published by timer)
        self._lock = threading.Lock()
        self._target_lin = 0.0
        self._target_ang = 0.0
        self._have_cmd = False  # becomes True after first valid stdin input

        # Latest feedback state (set by callbacks, printed by timer)
        self._yaw_rad: Optional[float] = None
        self._scan_ranges: Optional[List[float]] = None
        self._scan_range_min: Optional[float] = None
        self._scan_range_max: Optional[float] = None

        # Publisher: cmd_vel
        pub_qos = QoSProfile(depth=10)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", pub_qos)

        # Subscriptions: odom and scan (typical TurtleBot3 topics)
        self.create_subscription(Odometry, "odom", self._on_odom, QoSProfile(depth=10))
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)

        # Timers
        self._publish_hz = 10.0
        self._print_hz = 2.0

        self._pub_timer = self.create_timer(1.0 / self._publish_hz, self._publish_cmd)
        self._print_timer = self.create_timer(1.0 / self._print_hz, self._print_status)

        # Stdin reader thread
        self._stop_event = threading.Event()
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        self.get_logger().info(
            "marl_controller started. Enter: '<linear_x> <angular_z>' per line. "
            "Example: '0.1 0.0' or '0.0 0.5'. Ctrl-D or Ctrl-C to stop."
        )
        self.get_logger().info(
            f"Using TurtleBot3 model '{self._model}' for velocity limits."
        )

    def _lin_limit(self) -> float:
        return BURGER_MAX_LIN_VEL if self._model == "burger" else WAFFLE_MAX_LIN_VEL

    def _ang_limit(self) -> float:
        return BURGER_MAX_ANG_VEL if self._model == "burger" else WAFFLE_MAX_ANG_VEL

    def _clamp_cmd(self, lin: float, ang: float) -> Tuple[float, float]:
        lin = constrain(lin, -self._lin_limit(), self._lin_limit())
        ang = constrain(ang, -self._ang_limit(), self._ang_limit())
        return lin, ang

    def _stdin_loop(self) -> None:
        """
        Blocking read from stdin. Updates target velocity upon valid input.
        """
        try:
            for line in sys.stdin:
                if self._stop_event.is_set():
                    break
                parsed = parse_two_floats(line)
                if parsed is None:
                    # Keep it quiet; stdin might contain blank lines.
                    continue
                lin, ang = self._clamp_cmd(parsed[0], parsed[1])
                with self._lock:
                    self._target_lin = lin
                    self._target_ang = ang
                    self._have_cmd = True
        except Exception as exc:
            # Avoid raising out of thread; log once.
            try:
                self.get_logger().error(f"stdin loop error: {exc}")
            except Exception:
                pass
        finally:
            # If stdin closes (Ctrl-D), request stop.
            self._stop_event.set()

    def _publish_cmd(self) -> None:
        """
        Publish the most recent command. If stdin has never provided a command yet,
        we still publish zero by default (safe).
        """
        with self._lock:
            lin = self._target_lin
            ang = self._target_ang

        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self._cmd_pub.publish(twist)

        # If stdin closed, initiate shutdown after commanding stop once.
        if self._stop_event.is_set():
            # publish zero next tick in shutdown, but also stop now
            self._publish_zero_and_shutdown()

    def _publish_zero_and_shutdown(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._cmd_pub.publish(twist)
        self.get_logger().info("Stopping and shutting down (stdin closed or stop requested).")
        rclpy.shutdown()

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._yaw_rad = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _on_scan(self, msg: LaserScan) -> None:
        # Store ranges; also compute basic stats ignoring NaN/inf
        ranges = list(msg.ranges)
        finite = [r for r in ranges if math.isfinite(r)]
        self._scan_ranges = ranges
        if finite:
            self._scan_range_min = min(finite)
            self._scan_range_max = max(finite)
        else:
            self._scan_range_min = None
            self._scan_range_max = None

    def _print_status(self) -> None:
        with self._lock:
            lin = self._target_lin
            ang = self._target_ang
            have_cmd = self._have_cmd

        # Orientation
        if self._yaw_rad is None:
            yaw_str = "yaw: N/A (no /odom yet)"
        else:
            yaw_deg = (self._yaw_rad * 180.0) / math.pi
            yaw_str = f"yaw: {yaw_deg:+.1f} deg"

        # Rangefinder
        if self._scan_ranges is None:
            scan_str = "scan: N/A (no /scan yet)"
        else:
            if self._scan_range_min is None or self._scan_range_max is None:
                scan_str = "scan: no finite ranges"
            else:
                scan_str = f"scan: min {self._scan_range_min:.2f} m, max {self._scan_range_max:.2f} m"

        cmd_src = "stdin" if have_cmd else "default(0,0)"
        print(
            f"[marl_controller] cmd({cmd_src}): lin {lin:+.3f} m/s, ang {ang:+.3f} rad/s | {yaw_str} | {scan_str}",
            flush=True,
        )

    def destroy_node(self) -> bool:
        # Stop stdin thread if we are shutting down
        self._stop_event.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MarlController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort stop command on exit
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            node._cmd_pub.publish(twist)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
