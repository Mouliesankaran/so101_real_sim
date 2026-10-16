# SO-101 Sim-to-Real (ROS 2 Gazebo + RViz + MoveIt + Real Arm)

> **Goal:** the SO-101 arm opens in Gazebo and RViz, keyboard teleop or MoveIt moves it in Gazebo, and the real SO-101 follower arm copies the motion.
>
> **Setup:** ROS 2 Humble, Gazebo (`gazebo_ros`), MoveIt 2, LeRobot (virtual environment `~/lerobot/.venv`), workspace `~/so101_real_sim`, real arm on `/dev/ttyACM0`.
>
> **Done:** Steps 1 to 7 below.

## How it works

```
Teleop or MoveIt  →  Gazebo arm  →  sim_to_real.py  →  Real SO-101 follower arm
```

Commands go to Gazebo first. `sim_to_real.py` reads Gazebo's `/joint_states`, converts radians to degrees, and sends them to the real arm. `real_to_gazebo.py` does the opposite and is used to sync the poses before going live.

## Repository structure (what to put on GitHub)

Only the `src/` folder and the `bridge/` folder are needed. `build/`, `install/` and `log/` are created again by `colcon build`.

```
so101-sim-to-real-ros2/
├── README.md
├── src/
│   ├── so101_description/                 (copied from ~/so101/src)
│   │   └── meshes/                        (d435, so_arm101)
│   ├── s0101_gazebo/                      (copied from ~/so101/src, package name so101_gazebo)
│   │   ├── launch/
│   │   │   ├── so101_gazebo.launch.py          Gazebo + spawn + controllers
│   │   │   ├── so101_rviz.launch.py            RViz
│   │   │   └── so101_classic_moveit.launch.py  Gazebo + MoveIt + RViz
│   │   ├── config/
│   │   │   ├── controllers.yaml                arm_controller, gripper_controller
│   │   │   └── follower_gazebo.rviz            RViz config
│   │   ├── worlds/empty_world.world
│   │   └── so101_description/
│   │       ├── urdf/so101.urdf                 MODIFIED (Step 1)
│   │       └── meshes/                         (*.stl)
│   ├── so101_moveit_config/               (copied from ~/so101/src)
│   │   └── config/so101/so101.srdf             groups: arm, gripper, arm_with_gripper
│   └── so101_teleop/                      (copied from ~/so101/src, Step 6)
│       └── so101_teleop/teleop.py
└── bridge/                                (saved in ~/test_so101/bridge)
    ├── real_to_gazebo.py                       existing script (Step 4)
    └── sim_to_real.py                          NEW (Step 5)
```

Files are listed only where they were seen or used in this project; other files inside the packages are not listed. The full content of the new file (`sim_to_real.py`) is inside Step 5.

## Setup from this repository

```bash
git clone https://github.com/<your-username>/so101-sim-to-real-ros2.git
mkdir -p ~/so101_real_sim
cp -r so101-sim-to-real-ros2/src ~/so101_real_sim/
mkdir -p ~/test_so101/bridge
cp so101-sim-to-real-ros2/bridge/*.py ~/test_so101/bridge/
```

- Used for: puts the packages in the workspace `~/so101_real_sim` and the bridge scripts in `~/test_so101/bridge`.
- How to use: run once. Replace `<your-username>` with your GitHub name.

```bash
cd ~/so101_real_sim
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

- Used for: builds the packages and loads the workspace.
- How to use: run inside the workspace root. Run `source install/setup.bash` in every new terminal.

**Note (absolute paths):** the URDF contains absolute `/home/moulie/...` paths (mesh folder and the `controllers.yaml` path). On another computer or user name, find them and change them:

```bash
grep -rn "/home/moulie" ~/so101_real_sim/src --include=*.urdf
```

- Used for: lists every absolute path in the URDF files.
- How to use: replace each path with the matching path on your machine, then rebuild.

---

## Step 1: Fix the arm not showing in RViz

**Problem:** the arm opened in Gazebo but RViz showed `Could not load mesh resource` errors. The URDF mesh paths were bare `/home/...` paths, and RViz needs `file://` or `package://`.

```bash
ros2 topic list | grep description
```

- Used for: checks that the robot description is published.
- How to use: run while Gazebo is running. It should print `/robot_description`.

```bash
ros2 pkg prefix so101_description
```

- Used for: checks that the workspace is loaded in the terminal.
- How to use: run with the package name. `Package not found` means `source install/setup.bash` is missing.

```bash
find ~/so101 ~/test -name "base_so101_v2.stl" 2>/dev/null
```

- Used for: finds where the mesh files are.
- How to use: run once. The copy in `s0101_gazebo/so101_description/meshes/` is the one the URDF points to.

```bash
cd ~/so101/src
grep -rl "/home/moulie/so101/src/s0101_gazebo/so101_description/meshes" .
```

- Used for: finds which file has the bare mesh paths.
- How to use: run inside `src`. It printed `./s0101_gazebo/so101_description/urdf/so101.urdf`.

```bash
sed -i 's#filename="/home/moulie/so101/src/s0101_gazebo/so101_description/meshes/#filename="file:///home/moulie/so101/src/s0101_gazebo/so101_description/meshes/#g' ~/so101/src/s0101_gazebo/so101_description/urdf/so101.urdf
```

- Used for: adds `file://` in front of every mesh path.
- How to use: run once. Only the folder part changes; the STL file names stay the same.

```bash
grep -m3 "mesh filename" ~/so101/src/s0101_gazebo/so101_description/urdf/so101.urdf
grep -m1 "<robot name" ~/so101/src/s0101_gazebo/so101_description/urdf/so101.urdf
```

- Used for: checks the new paths and the robot name (`so101_new_calib`).
- How to use: each mesh line should start with `file:///home/...`.

```bash
cd ~/so101
colcon build --packages-select s0101_gazebo
source install/setup.bash
ros2 topic echo /robot_description --once --full-length | grep -m3 filename
```

- Used for: rebuilds, loads the workspace, and checks what is really published.
- How to use: restart Gazebo and RViz from the same terminal after this.

**Result:** the arm shows in RViz and Gazebo.

---

## Step 2: Create a new workspace for the real-robot work

```bash
mkdir -p ~/so101_real_sim/src
```

- Used for: creates the new workspace and its `src` folder.
- How to use: run once. The working `~/so101` stays untouched.

```bash
cp -r ~/so101/src/so101_description ~/so101/src/s0101_gazebo ~/so101/src/so101_moveit_config ~/so101_real_sim/src/
```

- Used for: copies the three packages into the new workspace.
- How to use: run once from any folder.

```bash
cd ~/so101_real_sim
source /opt/ros/humble/setup.bash
colcon build
```

- Used for: builds the copied packages.
- How to use: run inside the workspace root, and again after any change.

**Result:** workspace built.

---

## Step 3: Start Gazebo and RViz in the new workspace

Run each command in its own terminal. In every new terminal first run:

```bash
cd ~/so101_real_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- Used for: loads ROS 2 and this workspace into the terminal.
- How to use: run in every new terminal.

```bash
ros2 launch so101_gazebo so101_gazebo.launch.py
```

- Used for: starts Gazebo, spawns the arm and starts the controllers.
- How to use: run in the first terminal and keep it open.

```bash
ros2 launch so101_gazebo so101_rviz.launch.py
```

- Used for: opens RViz with the arm model.
- How to use: run in a second terminal.

```bash
ros2 control list_controllers
```

- Used for: checks that the controllers are running.
- How to use: run while Gazebo is running.

**Result:** `joint_state_broadcaster`, `arm_controller` and `gripper_controller` are all `active`.

---

## Step 4: Real arm to Gazebo (existing script)

```bash
source ~/lerobot/.venv/bin/activate
python3 ~/test_so101/bridge/real_to_gazebo.py
```

- Used for: makes the Gazebo arm copy the real arm. Also used to sync the poses before going live.
- How to use: run for about 5 seconds, then press Ctrl+C.

**Result:** the Gazebo arm folds like the real arm.

---

## Step 5: Gazebo to real arm (`sim_to_real.py`)

`sim_to_real.py` reads `/joint_states` from Gazebo, converts radians to degrees and sends them to the real follower arm. It is a dry run by default.

```bash
mkdir -p ~/test_so101/bridge
cp ~/Downloads/sim_to_real.py ~/test_so101/bridge/
ls ~/test_so101/bridge
```

- Used for: saves the script next to `real_to_gazebo.py`.
- How to use: run once. The `ls` output should show both scripts.

Full content of `~/test_so101/bridge/sim_to_real.py` (NEW):

```python
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
```

```bash
find ~/.cache/huggingface/lerobot/calibration -name "*.json"
```

- Used for: finds the existing LeRobot calibration files.
- How to use: run once. Only `teleoperators/so_leader/my_leader.json` existed.

```bash
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
cp ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json
```

- Used for: gives the follower arm the calibration id `my_follower`.
- How to use: run once. This fixed the error `has no calibration registered`. A new follower calibration with `lerobot-calibrate` was not run.

```bash
cd ~/so101_real_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/lerobot/.venv/bin/activate
python3 ~/test_so101/bridge/sim_to_real.py
```

- Used for: dry run. Prints what would be sent and moves nothing.
- How to use: Gazebo must be running. The first run showed a gap of 84 degrees (Gazebo at zero, real arm folded).

**Problem we had:** `No module named 'lerobot'`. The LeRobot virtual environment was not active in that terminal. Fixed with `source ~/lerobot/.venv/bin/activate`.

```bash
ros2 topic pub -r 1 --times 5 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.3, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

- Used for: moves Gazebo's shoulder_pan by 0.3 rad to test the sign of the conversion.
- How to use: run in a second terminal while the dry run is running. All 5 arm joints must be in the command.

**Result:** `target: shoulder_pan.pos` changed to 17.2 degrees, so the sign is correct.

```bash
python3 ~/test_so101/bridge/real_to_gazebo.py
```

- Used for: syncs Gazebo to the real arm pose before going live.
- How to use: run for about 5 seconds, then Ctrl+C.

**Problem we had:** the live script exited with `gap > 20.0 deg` (gap 24.3). This is the built-in safety check. Fixed by syncing again.

```bash
LIVE=1 python3 ~/test_so101/bridge/sim_to_real.py
```

- Used for: sends Gazebo's joint positions to the real arm.
- How to use: run in a terminal with the LeRobot environment active. Without `LIVE=1` it is a dry run.

```bash
ros2 topic pub -r 1 --times 3 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.0, -1.45, 1.47, 1.3, -0.37], time_from_start: {sec: 3}}]}"
```

- Used for: first small live move from the folded pose. Only shoulder_pan changes.
- How to use: run in a second terminal while the live script is running.

**Result:** the real arm moved with Gazebo.

---

## Step 6: Keyboard teleop

```bash
grep -n "create_publisher" ~/so101/src/so101_teleop/so101_teleop/teleop.py
sed -n '55,60p' ~/so101/src/so101_teleop/so101_teleop/teleop.py
```

- Used for: checks which topics teleop publishes to.
- How to use: run once. Both topics matched the Gazebo controllers: `/arm_controller/joint_trajectory` and `/gripper_controller/joint_trajectory`.

```bash
cp -r ~/so101/src/so101_teleop ~/so101_real_sim/src/
cd ~/so101_real_sim && colcon build --packages-select so101_teleop
ros2 pkg executables so101_teleop
```

- Used for: copies the teleop package, builds it, and lists its node.
- How to use: run once. The last command printed `so101_teleop teleop`.

```bash
ros2 run so101_teleop teleop
```

- Used for: moves the arm with keyboard keys.
- How to use: run in a third terminal while Gazebo and the live script are running.

**Result:** key presses moved Gazebo and the real arm together.

---

## Step 7: MoveIt plan and execute

```bash
ros2 launch so101_gazebo so101_classic_moveit.launch.py
```

- Used for: starts Gazebo, MoveIt and RViz together.
- How to use: start the live script in another terminal first. In RViz drag a small goal, click Plan, then Execute.

**Result:** Gazebo moved first and the real arm followed.

---

## Run order (daily use)

1. Terminal 1: `ros2 launch so101_gazebo so101_gazebo.launch.py` (or the MoveIt launch from Step 7).
2. Terminal 2 (LeRobot environment active): `python3 ~/test_so101/bridge/real_to_gazebo.py` for about 5 seconds, then Ctrl+C.
3. Terminal 2: `python3 ~/test_so101/bridge/sim_to_real.py` (dry run), then `LIVE=1 python3 ~/test_so101/bridge/sim_to_real.py`.
4. Terminal 3: `ros2 run so101_teleop teleop`, or use MoveIt in RViz.
5. Stop: press Ctrl+C in the live script terminal first.

## Safety

- Never run `real_to_gazebo.py` and `sim_to_real.py` at the same time.
- `sim_to_real.py` is a dry run unless `LIVE=1` is set.
- It exits if the real and Gazebo arms differ by more than 20 degrees at start.
- It moves the real arm at most 3 degrees per cycle (20 cycles per second).
- Keep the arm area clear and your hand near the power switch.
