"""Gazebo -> real SO101 follower. Reverse of real_to_gazebo.py.

DRY RUN (default): prints what it WOULD send, moves nothing.
LIVE: LIVE=1 python3 sim_to_real.py

Never run together with real_to_gazebo.py (feedback loop).
"""
import math, os, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = os.environ.get("PORT", "/dev/ttyACM0")
ROBOT_ID = os.environ.get("ROBOT_ID", "my_follower")  # must match your follower calibration id
LIVE = os.environ.get("LIVE") == "1"

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SIGN = {j: 1.0 for j in ARM}        # keep identical to real_to_gazebo.py
OFFSET_RAD = {j: 0.0 for j in ARM}  # keep identical to real_to_gazebo.py
GRIP_MIN, GRIP_MAX = -0.1745, 1.7453

# URDF joint limits (rad)
LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (GRIP_MIN, GRIP_MAX),
}

RATE_HZ = 20
MAX_STEP_DEG = 3.0     # max change per tick per joint (= 60 deg/s)
SYNC_TOL_DEG = 20.0    # refuse to start live if real and sim differ more than this
MAX_STEP_GRIP = 5.0    # gripper units (0-100) per tick


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SimToReal(Node):
    def __init__(self, robot):
        super().__init__("sim_to_real")
        self.robot = robot
        self.sim = {}
        self.cmd = None  # last commanded real values
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(f"mode: {'LIVE' if LIVE else 'DRY RUN (no motion)'}")

    def on_js(self, msg):
        self.sim = dict(zip(msg.name, msg.position))

    def target_from_sim(self):
        t = {}
        for j in ARM:
            rad = clamp(self.sim[j], *LIMITS[j])
            t[f"{j}.pos"] = math.degrees(SIGN[j] * (rad - OFFSET_RAD[j]))
        g = clamp(self.sim["gripper"], *LIMITS["gripper"])
        t["gripper.pos"] = clamp((g - GRIP_MIN) / (GRIP_MAX - GRIP_MIN) * 100.0, 0.0, 100.0)
        return t

    def tick(self):
        if not all(k in self.sim for k in ARM + ["gripper"]):
            return  # wait for sim joint states
        target = self.target_from_sim()
        real = self.robot.get_observation()
        real = {k: v for k, v in real.items() if k.endswith(".pos")}

        if self.cmd is None:  # first tick: sync check
            worst = max(abs(target[k] - real[k]) for k in target if k != "gripper.pos")
            self.get_logger().info(f"start gap real vs sim: {worst:.1f} deg")
            if LIVE and worst > SYNC_TOL_DEG:
                self.get_logger().error(
                    f"gap > {SYNC_TOL_DEG} deg. Move the real arm near the sim pose first. Exiting.")
                raise SystemExit(1)
            self.cmd = dict(real)

        out = {}
        for k, tv in target.items():
            step = MAX_STEP_GRIP if k == "gripper.pos" else MAX_STEP_DEG
            out[k] = self.cmd[k] + clamp(tv - self.cmd[k], -step, step)

        if not LIVE:
            print("target:", {k: round(v, 1) for k, v in target.items()})
            print("real  :", {k: round(v, 1) for k, v in real.items()})
            return

        self.robot.send_action(out)
        self.cmd = out


def main():
    cfg = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,           # joint values in true degrees
        max_relative_target=10.0,   # hardware-level cap per command
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)  # never starts calibration
    rclpy.init()
    node = SimToReal(robot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        robot.disconnect()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
