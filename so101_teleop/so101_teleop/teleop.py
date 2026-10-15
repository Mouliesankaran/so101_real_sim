import select
import sys
import termios
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIPPER_JOINT = 'gripper'
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]

# Same limits as the URDF
LIMITS = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}

# key -> (joint index, direction)
KEYMAP = {
    'q': (0, +1), 'a': (0, -1),
    'w': (1, +1), 's': (1, -1),
    'e': (2, +1), 'd': (2, -1),
    'r': (3, +1), 'f': (3, -1),
    't': (4, +1), 'g': (4, -1),
    'y': (5, +1), 'h': (5, -1),
}

HELP = """
====================================
       SO-101 KEYBOARD TELEOP
====================================
Shoulder Pan : q / a      Shoulder Lift: w / s
Elbow Flex   : e / d      Wrist Flex   : r / f
Wrist Roll   : t / g      Gripper      : y / h
Reset: z      Quit: x      Step: 0.05 rad
(After 1 s idle, targets re-sync to /joint_states,
 so MoveIt moves and keyboard moves never fight.)
====================================
"""


class SO101Teleop(Node):

    def __init__(self):
        super().__init__('so101_teleop')
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)

        self.positions = [0.0] * 6
        self.current = None
        self.step = 0.05
        self.move_time = 0.3   # seconds per command
        self.last_key = 0.0
        self.resync_after = 1.0

    def on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values for j in ALL_JOINTS):
            self.current = [values[j] for j in ALL_JOINTS]

    def resync(self):
        if self.current is not None:
            self.positions = list(self.current)

    def send(self, joints, positions, pub):
        traj = JointTrajectory()
        traj.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        secs = int(self.move_time)
        point.time_from_start = Duration(
            sec=secs, nanosec=int((self.move_time - secs) * 1e9))
        traj.points.append(point)
        pub.publish(traj)

    def publish(self, arm=True, gripper=True):
        if arm:
            self.send(ARM_JOINTS, self.positions[:5], self.arm_pub)
        if gripper:
            self.send([GRIPPER_JOINT], self.positions[5:], self.gripper_pub)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                if not select.select([sys.stdin], [], [], 0.02)[0]:
                    if time.monotonic() - self.last_key > self.resync_after:
                        self.resync()
                    continue

                key = sys.stdin.read(1)
                if key == 'x':
                    break

                if time.monotonic() - self.last_key > self.resync_after:
                    self.resync()
                self.last_key = time.monotonic()

                if key in KEYMAP:
                    idx, sign = KEYMAP[key]
                    lo, hi = LIMITS[ALL_JOINTS[idx]]
                    self.positions[idx] = max(
                        lo, min(hi, self.positions[idx] + sign * self.step))
                    self.publish(arm=idx < 5, gripper=idx == 5)
                elif key == 'z':
                    self.positions = [0.0] * 6
                    self.publish()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = SO101Teleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
