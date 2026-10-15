import math, os, time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from lerobot.teleoperators.so_leader import SO101Leader as Leader, SO101LeaderConfig as Cfg
except ImportError:
    from lerobot.teleoperators.so_leader import SOLeader as Leader, SOLeaderTeleopConfig as Cfg

PORT = "/dev/ttyACM0"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SIGN = {j: 1.0 for j in ARM}        # change to -1.0 if a joint moves the wrong way
OFFSET_RAD = {j: 0.0 for j in ARM}  # zero correction if needed
GRIP_MIN, GRIP_MAX = -0.1745, 1.7453  # gripper rad range, assumed from SO101 URDF
PRINT_ONLY = os.environ.get("PRINT_ONLY") == "1"

class Bridge(Node):
    def __init__(self, dev):
        super().__init__("real_to_gazebo")
        self.dev = dev
        self.pub_real = self.create_publisher(JointState, "/real_so101/joint_states", 10)
        self.pub_arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.create_timer(0.05, self.tick)  # 20 Hz

    def tick(self):
        a = self.dev.get_action()  # keys like "shoulder_pan.pos"
        arm = [SIGN[j] * math.radians(a[f"{j}.pos"]) + OFFSET_RAD[j] for j in ARM]
        grip = GRIP_MIN + (a["gripper.pos"] / 100.0) * (GRIP_MAX - GRIP_MIN)
        if PRINT_ONLY:
            print({k: round(v, 2) for k, v in a.items()}, "| rad:", [round(x, 2) for x in arm], round(grip, 2))
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM + ["gripper"]
        js.position = arm + [grip]
        self.pub_real.publish(js)
        for pub, names, pos in ((self.pub_arm, ARM, arm), (self.pub_grip, ["gripper"], [grip])):
            m = JointTrajectory(); m.joint_names = names
            p = JointTrajectoryPoint(); p.positions = pos
            p.time_from_start = Duration(sec=0, nanosec=100_000_000)
            m.points = [p]; pub.publish(m)

def main():
    dev = Leader(Cfg(port=PORT, id="my_leader"))
    dev.connect(calibrate=False)  # never starts calibration
    rclpy.init()
    node = Bridge(dev)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        dev.disconnect()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
