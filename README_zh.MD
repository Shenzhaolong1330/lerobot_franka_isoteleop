# Franka Research 3 键盘遥操作数据采集

> [!NOTE]
> 本仓库使用 Polymetis 实现末端笛卡尔阻抗控制，并基于 LeRobot 完成键盘遥操作与数据采集。

## 1. 环境配置

### 1.1 创建 Conda 环境

```bash
conda create -n franka_data python=3.10 -y
conda activate franka_data
```

### 1.2 安装 [libfranka](https://github.com/frankarobotics/libfranka)

安装与机器人 system image 匹配的官方 Debian 包。验证环境为 Ubuntu 22.04 和
libfranka 0.19.0。

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

### 1.3 安装 LeRobot

```bash
git clone https://github.com/huggingface/lerobot.git ~/lerobot
cd ~/lerobot
git checkout 306429a85b9cc79d8a86278d8b5bc024182c3cde
python -m pip install -e .
```

### 1.4 安装 Polymetis

Polymetis 使用独立的 Python 3.8 环境，并链接 DEB 安装的 libfranka。

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

本项目使用两个 Conda 环境：

- `franka_data`：LeRobot、键盘控制、相机和数据集工具。
- `polymetis-local`：Polymetis 和 Franka 控制服务。

正常使用时激活 `franka_data`；启动命令会自动进入 `polymetis-local`。

### 1.5 安装本仓库

```bash
mkdir -p ~/franka_data_collection
git clone https://github.com/Shenzhaolong1330/lerobot_franka_teleop.git \
  ~/franka_data_collection/lerobot_franka_teleop
conda activate franka_data
cd ~/franka_data_collection/lerobot_franka_teleop
python -m pip install -e .
```

### 1.6 查看可用命令

```bash
franka-help
```

主要命令如下：

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

## 2. 获取和配置参数

- `record.cameras.enabled: false`：不打开相机，使用固定假画面。
- `record.robot.use_gripper: false`：禁用夹爪硬件与 action，state 中夹爪值保持为 `0`。

### 2.1 配置 RealSense 相机（可选）

```bash
tools-check-rs
```

将序列号填写到 `scripts/config/cfg.yaml` 的
`record.cameras.wrist_cam_serial` 和
`record.cameras.exterior_cam_serial`。当 `record.cameras.enabled` 为 `false`
时，程序使用固定假画面，不会打开真实相机。

### 2.2 配置 PGI（可选）

只连接一个 PGI USB 串口设备，然后运行：

```bash
map_gripper.sh franka_pgi_gripper
```

`cfg.yaml` 中的默认路径为 `/dev/franka_pgi_gripper`。PGI 由录制程序直接通过
串口控制，不需要运行 `franka-start-gripper`。
映射设备使用 `dialout` 用户组。如当前用户不在该组，执行以下命令并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

### 2.3 配置机械臂与键盘

录制前编辑 `scripts/config/cfg.yaml`。主要字段如下：

- `services.franka_ip`：真实机器人控制器 IP。所有服务在同一台电脑运行时，
  Polymetis 和 ZeroRPC 地址保持 `127.0.0.1`。
- `services.robot_model.tcp_xyz_rpy`：TCP 相对 Franka link8 的位姿，格式为
  `[x, y, z, roll, pitch, yaw]`，单位为米和弧度。
- `record.robot.use_gripper`：启用或关闭夹爪 observation 和 action。
- `record.robot.gripper.type`：`pgi` 或 `franka_hand`。
- `record.robot.reference_frame`：`base` 或 `tcp`。键盘增量使用该坐标系。
  选择 `tcp` 时，`tcp_pose` 保存为相对 episode 参考位姿；选择 `base` 时，
  保存为 base 坐标系下的绝对位姿。
- `record.robot.init_pose`：复位位姿 `[x, y, z, roll, pitch, yaw]`，单位为米和
  弧度，旋转使用 XYZ 欧拉角。
- `record.robot.init_pose_range`：随机复位范围
  `[x, y, z, roll_deg, pitch_deg, yaw_deg]`。
- `record.robot.reset_time_to_go`：自动最小加加速度复位时间。
- `record.robot.cartesian_impedance.select_vector`：按照
  `[X, Y, Z, RX, RY, RZ]` 排列的键盘启用轴。
- `stiffness` 和 `damping`：六个方向的笛卡尔阻抗刚度和阻尼。
- `error_clip`：各方向正值控制误差限制；`null` 表示该方向不限制。
- `target_filter_alpha`：目标低通滤波系数，越接近 `1`，滤波越少。
- `translation_axis_deadband` 和 `rotation_axis_deadband`：过滤坐标转换和状态读取
  产生的微小残差。
- `nullspace_stiffness` 和 `joint1_nullspace_stiffness`：关节构型保持增益。
- `record.teleop.step_size` 和 `rot_step_size`：普通平移与旋转步长。
- `record.teleop.fine_step_size` 和 `fine_rot_step_size`：按 `/` 后使用的步长。
- `record.teleop.control_frame_euler_deg`：控制坐标系相对机器人 base
  的 XYZ 欧拉角，单位为度。

采集的 state 固定为 19 维：TCP 位姿、TCP 速度、TCP 力/力矩和夹爪位置。

## 3. 数据集录制

### 3.1 上传到 Hugging Face（可选）

在 `cfg.yaml` 中设置 `record.storage.push_to_hub: true`，然后登录：

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN}
huggingface-cli whoami
```

### 3.2 开始录制

运行 `franka-start-arm` 前，在 Franka Desk 中点击 **Activate FCI**；未激活 FCI
时 Polymetis 无法连接。

在以下每个终端中先激活 `franka_data`，再分别启动一个服务。

终端 1：

```bash
franka-start-arm      # 启动机械臂控制
```

仅使用 Franka Hand 时，在终端 2 运行：

```bash
franka-start-gripper  # 启动 Franka Hand 服务
```

终端 3：

```bash
franka-start-server   # 启动 ZeroRPC 服务
```

终端 4：

```bash
franka-record         # 开始录制
```

首个 episode 前，`franka-record` 会复位到 `record.robot.init_pose`。切换到 Teach
模式前，先停止机械臂和 ZeroRPC 服务。

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: 数据录制</b>
</p>

## 4. 数据集回放

保持机械臂、可选的 Franka Hand 和 ZeroRPC 服务运行。将机械臂移动到录制时的
初始位姿，在 `replay` 中设置数据集和 episode，然后运行：

```bash
franka-replay
```

## 5. 数据集可视化

在 `cfg.yaml` 中设置 `visualize.dataset_name` 和 `visualize.episode_index`，然后运行：

```bash
franka-visualize
```

<p align="center">
  <img src="assets/episode_data.png" alt="Episode Data" width="600">
  <br>
  <b>Figure 2: Episode 数据窗口</b>
</p>

<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>Figure 3: 数据可视化</b>
</p>

Episode 数据窗口显示每一帧的完整 action，并支持选择一个 observation 组查看，
例如 `tcp_pose`、`tcp_speed`、`tcp_force` 或 `gripper_raw_position`。Rerun 窗口显示相机图像
和时序数据。

## 6. 数据集追加与恢复

向现有数据集追加 episode 时设置：

```yaml
record:
  task:
    resume: true
    resume_dataset: "<existing_repo_id>"
```

然后运行：

```bash
franka-record
```

现有数据集必须使用相同的 FPS 和特征结构。

## 7. 合并数据集

分阶段录制的数据集可以使用 LeRobot 合并：

```bash
lerobot-edit-dataset \
  --repo_id <merged_repo_id> \
  --operation.type merge \
  --operation.repo_ids "['<repo_id_1>', '<repo_id_2>']"
```

## 8. 数据集检查与裁剪

在 `cfg.yaml` 中设置 `check_dataset.dataset_name`，然后逐 episode 检查数据和视频帧：

```bash
tools-check-dataset
```

删除损坏或不需要的 episode，同时保留原数据集时，设置
`prune_episodes.old_dataset_name` 和 `prune_episodes.new_dataset_name`，然后运行：

```bash
tools-prune-dataset "[2,5]"
```

如果手动删除过数据集文件夹，可清理并重新编号 `dataset_info.txt`：

```bash
tools-check-info
```

## 9. 录制控制

### 9.1 Episode 控制按键

1. **右方向键**
   1. 结束并保存当前 episode。
   2. 如果还有下一个 episode，按 Enter 进入人工预复位控制。
   3. 用键盘将机械臂移动到安全位置，再次按右方向键。
   4. 机械臂自动复位到 `init_pose`。
   5. 按 Enter 开始下一个 episode。
2. **左方向键**：丢弃当前 episode 缓存并重新录制。
3. **Esc**：保存当前 episode 并退出整个录制任务。
4. **Ctrl+C 或异常**：停止录制，并询问是否将未完成的数据集文件夹移动到回收站。

### 9.2 键盘遥操作按键

| 按键 | 动作 |
|:---:|:-----|
| `w` / `s` | X 负方向 / 正方向移动 |
| `a` / `d` | Y 负方向 / 正方向移动 |
| `q` / `e` | Z 正方向 / 负方向移动 |
| `r` / `t` | RX 正方向 / 负方向旋转 |
| `g` / `f` | RY 正方向 / 负方向旋转 |
| `b` / `v` | RZ 正方向 / 负方向旋转 |
| `o` | 夹爪保持打开 |
| `l` | 夹爪保持闭合 |
| `/` | 切换普通与精细步长 |

## 10. 数据集命名与内容

### 10.1 数据集命名

数据集保存在 `~/.cache/huggingface/lerobot`。该目录还包含：

- `dataset_info.txt`：记录数据集名称、任务、日期、版本、用户注释和运行时间。
- `dataset_info_backup`：`tools-check-info` 创建的旧记录备份。
- 保存 episode 数据和视频的数据集文件夹。

实际生成的数据集名称格式为 `<dataset>_<YYYYMMDD>_vXX`。数据集标识来自
`record.repo_id=<user>/<dataset>`，与任务指令相互独立，版本号自动递增。例如：

```yaml
record:
  repo_id: "scylearning/franka_dataset"
  task:
    description: "Load the centrifuge tube rack."
```

<p align="center">
  <img src="assets/dataset.png" alt="Dataset" width="600">
  <br>
  <b>Figure 4: 数据集</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="Dataset Info" width="600">
  <br>
  <b>Figure 5: 数据集信息</b>
</p>

### 10.2 采集特征

- `observation.state`：19 维，包括六维 TCP 位姿、六维 TCP 速度、六维 TCP
  力/力矩和一维归一化夹爪位置。
- `action`：六维笛卡尔增量；启用夹爪时再增加一维夹爪指令。
- `wrist_image` 和 `exterior_image`：RealSense 视频帧或假画面。
- `task`：每一帧对应的任务指令。

TCP 旋转和旋转 action 使用 XYZ 欧拉角，单位为弧度。每个 episode 开始时都会记录
当前末端位姿，作为 `reference_frame: tcp` observation 使用的 episode 参考位姿。

### 10.3 录制检查清单

1. 配置机械臂、夹爪、相机、任务描述和 `repo_id`。
2. 检查初始位姿并清空机械臂工作空间。
3. 启动机械臂、可选的 Franka Hand 和 ZeroRPC 服务。
4. 运行 `franka-record`；首个 episode 前机械臂会自动复位。
5. 采集后运行 `tools-check-dataset`；手动删除数据集文件夹后运行
   `tools-check-info`。
