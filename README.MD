# Franka Research 3 Keyboard Teleoperation Data Collection

> [!NOTE]
> This repository uses Polymetis for end-effector Cartesian impedance control and LeRobot for keyboard teleoperation and dataset collection.

## 1. Environment Setup

### 1.1 Create the Conda Environment

```bash
conda create -n franka_data python=3.10 -y
conda activate franka_data
```

### 1.2 Install [libfranka](https://github.com/frankarobotics/libfranka)

Install the official Debian package matching the robot system image. Tested on
Ubuntu 22.04 with libfranka 0.19.0.

```bash
cd /tmp
FRANKA_VERSION="0.19.0"
FRANKA_UBUNTU_CODENAME="$(lsb_release -cs)"
FRANKA_DEB="libfranka_${FRANKA_VERSION}_${FRANKA_UBUNTU_CODENAME}_amd64.deb"
wget "https://github.com/frankarobotics/libfranka/releases/download/${FRANKA_VERSION}/${FRANKA_DEB}"
wget "https://github.com/frankarobotics/libfranka/releases/download/${FRANKA_VERSION}/${FRANKA_DEB}.sha256"
sha256sum -c "${FRANKA_DEB}.sha256"
sudo dpkg -i "${FRANKA_DEB}"
```

### 1.3 Install LeRobot

```bash
git clone https://github.com/huggingface/lerobot.git ~/lerobot
cd ~/lerobot
git checkout 306429a85b9cc79d8a86278d8b5bc024182c3cde
python -m pip install -e .
```

### 1.4 Install Polymetis

Polymetis uses its own Python 3.8 environment and links to the libfranka DEB.

```bash
git clone https://github.com/Shenzhaolong1330/fairo-franka.git ~/fairo-franka
cd ~/fairo-franka
git checkout f64e925efcc566bd1827603888df2d5b09a76fe1

conda env create -f polymetis/polymetis/environment.yml
conda activate polymetis-local
python -m pip install zerorpc

cmake -S polymetis/polymetis -B polymetis/polymetis/build \
  -DBUILD_FRANKA=ON \
  -DFranka_DIR=/usr/lib/cmake/Franka \
  -DBUILD_TESTS=OFF \
  -DBUILD_DOCS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY="$CONDA_PREFIX/lib" \
  -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$CONDA_PREFIX/lib" \
  -DCMAKE_RUNTIME_OUTPUT_DIRECTORY="$CONDA_PREFIX/bin"
cmake --build polymetis/polymetis/build --parallel 8
python -m pip install -e polymetis/polymetis
```

Two Conda environments are required:

- `franka_data`: LeRobot, keyboard control, cameras, and dataset tools.
- `polymetis-local`: Polymetis and the Franka control services.

Activate `franka_data` for normal use. The launcher commands enter
`polymetis-local` automatically.

### 1.5 Install This Repository

```bash
mkdir -p ~/franka_data_collection
git clone https://github.com/Shenzhaolong1330/lerobot_franka_teleop.git \
  ~/franka_data_collection/lerobot_franka_teleop
conda activate franka_data
cd ~/franka_data_collection/lerobot_franka_teleop
python -m pip install -e .
```

### 1.6 View Available Commands

```bash
franka-help
```

The main commands are:

```text
franka-start-arm       Start the Polymetis arm service
franka-start-gripper   Start the Franka Hand service; PGI needs no service
franka-start-server    Start the local ZeroRPC bridge
franka-record          Record a keyboard teleoperation dataset
franka-replay          Replay a recorded dataset
franka-visualize       Visualize a recorded dataset
tools-check-rs         List connected RealSense cameras
tools-check-info       Clean local dataset information
tools-check-dataset    Check dataset integrity
tools-prune-dataset    Remove selected episodes into a new dataset
tools-robot-state      Read Franka state through ZeroRPC
test-franka-gripper    Test the configured gripper
map_gripper.sh         Map the PGI serial device
```

## 2. Obtain and Configure Parameters

- `record.cameras.enabled: false`: use dummy images without opening cameras.
- `record.robot.use_gripper: false`: disable gripper hardware and actions; the
  state gripper value remains `0`.

### 2.1 Configure RealSense Cameras (Optional)

```bash
tools-check-rs
```

Set `record.cameras.wrist_cam_serial` and
`record.cameras.exterior_cam_serial` in `scripts/config/cfg.yaml`. When
`record.cameras.enabled` is `false`, fixed dummy images are used and real
cameras are not opened.

### 2.2 Configure PGI (Optional)

Connect only one PGI USB serial device, then run:

```bash
map_gripper.sh franka_pgi_gripper
```

The default PGI path in `cfg.yaml` is `/dev/franka_pgi_gripper`. PGI is driven
directly over serial and does not use `franka-start-gripper`.
The mapped device uses the `dialout` group. If needed, add the current user and
sign in again:

```bash
sudo usermod -aG dialout "$USER"
```

### 2.3 Configure the Robot and Keyboard

Edit `scripts/config/cfg.yaml` before recording. Important fields are:

- `services.franka_ip`: physical robot controller IP. Polymetis and ZeroRPC
  hosts remain `127.0.0.1` when all services run on the same computer.
- `services.robot_model.tcp_xyz_rpy`: TCP pose relative to Franka link8 as
  `[x, y, z, roll, pitch, yaw]` in metres and radians.
- `record.robot.use_gripper`: enable or disable gripper observations and actions.
- `record.robot.gripper.type`: `pgi` or `franka_hand`.
- `record.robot.reference_frame`: `base` or `tcp`. Keyboard deltas use this
  frame. With `tcp`, `tcp_pose` is stored relative to the episode reference
  pose; with `base`, it is stored as an absolute base-frame pose.
- `record.robot.init_pose`: reset pose `[x, y, z, roll, pitch, yaw]` in metres
  and radians. Rotation uses XYZ Euler angles.
- `record.robot.init_pose_range`: random reset range
  `[x, y, z, roll_deg, pitch_deg, yaw_deg]`.
- `record.robot.reset_time_to_go`: automatic minimum-jerk reset duration.
- `record.robot.cartesian_impedance.select_vector`: enabled keyboard axes in
  `[X, Y, Z, RX, RY, RZ]` order.
- `stiffness` and `damping`: Cartesian impedance gains for the six axes.
- `error_clip`: positive per-axis Cartesian controller error limits; `null`
  disables clipping for an axis.
- `target_filter_alpha`: target low-pass coefficient. Values closer to `1`
  apply less filtering.
- `translation_axis_deadband` and `rotation_axis_deadband`: reject small
  transform and state-read residuals.
- `nullspace_stiffness` and `joint1_nullspace_stiffness`: joint posture gains.
- `record.teleop.step_size` and `rot_step_size`: normal translation and
  rotation steps.
- `record.teleop.fine_step_size` and `fine_rot_step_size`: steps selected by
  `/`.
- `record.teleop.control_frame_euler_deg`: control-frame XYZ
  Euler orientation relative to the robot base, in degrees.

The recorded state has 19 values: TCP pose, TCP speed, TCP force/torque, and
gripper position.

## 3. Dataset Recording

### 3.1 Upload to Hugging Face (Optional)

Set `record.storage.push_to_hub: true` in `cfg.yaml`, then log in:

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN}
huggingface-cli whoami
```

### 3.2 Start Recording

Before `franka-start-arm`, click **Activate FCI** in Franka Desk. Polymetis
cannot connect without FCI.

Activate `franka_data` in each terminal below, then run one service per terminal.

Terminal 1:

```bash
franka-start-arm      # Start arm control
```

For Franka Hand only, terminal 2:

```bash
franka-start-gripper  # Start Franka Hand service
```

Terminal 3:

```bash
franka-start-server   # Start ZeroRPC server
```

Terminal 4:

```bash
franka-record         # Start recording
```

`franka-record` resets the robot to `record.robot.init_pose` before the first
episode. Stop the arm and ZeroRPC services before switching to Teach mode.

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: Recording</b>
</p>

## 4. Dataset Replay

Keep the arm, optional Franka Hand, and ZeroRPC services running. Move the robot
to the recorded initial pose, select the dataset and episode under `replay`, and
run:

```bash
franka-replay
```

## 5. Dataset Visualization

Set `visualize.dataset_name` and `visualize.episode_index` in `cfg.yaml`, then
run:

```bash
franka-visualize
```

<p align="center">
  <img src="assets/episode_data.png" alt="Episode Data" width="600">
  <br>
  <b>Figure 2: Episode Data Window</b>
</p>

<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>Figure 3: Visualization</b>
</p>

The episode data window shows the complete action for every frame and lets you
select one observation group, such as `tcp_pose`, `tcp_speed`, `tcp_force`, or
`gripper_raw_position`. The Rerun viewer shows camera images and time-series data.

## 6. Dataset Appending and Resume

To append episodes to an existing dataset, set:

```yaml
record:
  task:
    resume: true
    resume_dataset: "<existing_repo_id>"
```

Then run:

```bash
franka-record
```

The existing dataset must use the same FPS and feature schema.

## 7. Merge Datasets

Datasets recorded in separate stages can be merged with LeRobot:

```bash
lerobot-edit-dataset \
  --repo_id <merged_repo_id> \
  --operation.type merge \
  --operation.repo_ids "['<repo_id_1>', '<repo_id_2>']"
```

## 8. Dataset Check and Pruning

Set `check_dataset.dataset_name` in `cfg.yaml`, then check every episode and
video frame:

```bash
tools-check-dataset
```

To remove damaged or unwanted episodes while keeping the source dataset, set
`prune_episodes.old_dataset_name` and `prune_episodes.new_dataset_name`, then
run:

```bash
tools-prune-dataset "[2,5]"
```

If dataset folders were deleted manually, clean and re-index
`dataset_info.txt` with:

```bash
tools-check-info
```

## 9. Recording Controls

### 9.1 Episode Control Keys

1. **Right Arrow**
   1. End and save the current episode.
   2. When another episode remains, press Enter to enter manual pre-reset
      control.
   3. Move the robot to a safe pose and press Right Arrow again.
   4. The robot automatically resets to `init_pose`.
   5. Press Enter to start the next episode.
2. **Left Arrow**: discard the current episode buffer and record it again.
3. **Esc**: save the current episode and exit the recording session.
4. **Ctrl+C or exception**: stop recording and ask whether the incomplete
   dataset folder should be moved to trash.

### 9.2 Keyboard Teleoperation Keys

| Key | Action |
|:---:|:-------|
| `w` / `s` | Move X negative / positive |
| `a` / `d` | Move Y negative / positive |
| `q` / `e` | Move Z positive / negative |
| `r` / `t` | Rotate RX positive / negative |
| `g` / `f` | Rotate RY positive / negative |
| `b` / `v` | Rotate RZ positive / negative |
| `o` | Latch gripper open |
| `l` | Latch gripper closed |
| `/` | Toggle normal and fine steps |

## 10. Dataset Naming and Contents

### 10.1 Dataset Naming

Datasets are stored under `~/.cache/huggingface/lerobot`. The directory also
contains:

- `dataset_info.txt`: dataset name, task, date, version, user notes, and run
  duration information.
- `dataset_info_backup`: backups created by `tools-check-info`.
- Dataset folders containing episode data and videos.

Generated dataset names use `<dataset>_<YYYYMMDD>_vXX`. The dataset identifier comes from
`record.repo_id=<user>/<dataset>`, and the version increments automatically.
It is independent of the task instruction.
For example:

```yaml
record:
  repo_id: "scylearning/franka_dataset"
  task:
    description: "Load the centrifuge tube rack."
```

<p align="center">
  <img src="assets/dataset.png" alt="Dataset" width="600">
  <br>
  <b>Figure 4: Dataset</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="Dataset Info" width="600">
  <br>
  <b>Figure 5: Dataset information</b>
</p>

### 10.2 Recorded Features

- `observation.state`: 19 values: six TCP pose values, six TCP speed values,
  six TCP force/torque values, and one normalized gripper position.
- `action`: six Cartesian deltas and, when enabled, one gripper command.
- `wrist_image` and `exterior_image`: RealSense video frames or dummy frames.
- `task`: task instruction for each frame.

TCP rotations and rotation actions use XYZ Euler angles in radians. At the start
of every episode, the current end-effector pose is saved as the episode
reference used by `reference_frame: tcp` observations.

### 10.3 Recording Checklist

1. Configure the robot, gripper, cameras, task description, and `repo_id`.
2. Check the initial pose and keep the robot workspace clear.
3. Start the arm, optional Franka Hand, and ZeroRPC services.
4. Run `franka-record`; the robot resets before the first episode.
5. Use `tools-check-dataset` after collection and `tools-check-info` after
   manually deleting dataset folders.
