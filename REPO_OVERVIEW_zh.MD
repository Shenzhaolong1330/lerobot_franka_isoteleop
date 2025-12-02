# LeRobot Franka Isometric Teleoperation Repository Description

## 项目概述

这是一个用于Franka机器人异构遥操作的代码库，支持通过Dynamixel电机控制器进行远程操作。项目包含机器人控制模块、遥操作模块、数据记录和可视化工具等组件。

## 文件结构

```
├── lerobot_robot_franka/         # Franka机器人控制模块
│   ├── lerobot_robot_franka/
│   │   ├── __init__.py
│   │   ├── config/
│   │   │   └── cfg.yaml
│   │   ├── config_franka.py
│   │   └── franka.py
│   └── setup.py
├── lerobot_teleoperator_franka/  # 遥操作控制模块
│   ├── lerobot_teleoperator_franka/
│   │   ├── __init__.py
│   │   ├── config/
│   │   │   └── cfg.yaml
│   │   ├── config_teleop.py
│   │   ├── dynamixel/
│   │   │   ├── __init__.py
│   │   │   ├── driver.py
│   │   │   ├── dynamixel_robot.py
│   │   │   └── robot.py
│   │   └── teleop.py
│   └── setup.py
├── scripts/                      # 各类脚本工具
│   ├── config/
│   │   └── cfg.yaml
│   ├── core/
│   │   ├── run_record.py
│   │   ├── run_replay.py
│   │   └── run_visualize.py
│   ├── help/
│   │   └── help_info.py
│   ├── test/
│   │   └── gripper_ctrl.py
│   ├── tools/
│   │   ├── check_dataset_info.py
│   │   ├── check_master_port.sh
│   │   ├── map_gripper.sh
│   │   └── rs_devices.py
│   ├── utils/
│   │   ├── dataset_utils.py
│   │   └── teleop_joint_offsets.py
│   └── __init__.py
├── assets/                       # 资源文件(图片)
│   ├── dataset.png
│   ├── dataset_info.png
│   ├── record.png
│   ├── robot.png
│   └── visualize.png
├── README.MD
├── README_zh.MD
└── setup.py
```

## 文件清单

### 核心模块文件

#### lerobot_robot_franka 模块
- `lerobot_robot_franka/franka.py`: Franka机器人控制核心实现
- `lerobot_robot_franka/config_franka.py`: Franka机器人配置类定义
- `lerobot_robot_franka/config/cfg.yaml`: Franka机器人默认配置文件
- `lerobot_robot_franka/__init__.py`: 包初始化文件

#### lerobot_teleoperator_franka 模块
- `lerobot_teleoperator_franka/teleop.py`: 遥操作控制核心实现
- `lerobot_teleoperator_franka/config_teleop.py`: 遥操作配置类定义
- `lerobot_teleoperator_franka/config/cfg.yaml`: 遥操作默认配置文件
- `lerobot_teleoperator_franka/__init__.py`: 包初始化文件

#### Dynamixel驱动模块
- `lerobot_teleoperator_franka/dynamixel/driver.py`: Dynamixel驱动协议和实现
- `lerobot_teleoperator_franka/dynamixel/dynamixel_robot.py`: Dynamixel机器人控制类
- `lerobot_teleoperator_franka/dynamixel/robot.py`: Dynamixel机器人基础类
- `lerobot_teleoperator_franka/dynamixel/__init__.py`: 包初始化文件

### 脚本工具文件

#### 核心功能脚本
- `scripts/core/run_record.py`: 数据记录脚本
- `scripts/core/run_replay.py`: 数据回放脚本
- `scripts/core/run_visualize.py`: 数据可视化脚本

#### 实用工具脚本
- `scripts/utils/dataset_utils.py`: 数据集处理工具
- `scripts/utils/teleop_joint_offsets.py`: 关节偏移量计算工具

#### 测试脚本
- `scripts/test/gripper_ctrl.py`: 夹爪控制测试脚本

#### 配置检查脚本
- `scripts/tools/check_dataset_info.py`: 数据集信息检查工具
- `scripts/tools/rs_devices.py`: 设备连接检查工具
- `scripts/tools/map_gripper.sh`: 夹爪映射脚本
- `scripts/tools/check_master_port.sh`: 端口检查脚本

#### 帮助信息脚本
- `scripts/help/help_info.py`: 帮助信息显示脚本

#### 配置文件
- `scripts/config/cfg.yaml`: 脚本默认配置文件

### 文档和资源文件
- `README.MD`: 英文说明文档
- `README_zh.MD`: 中文说明文档
- `assets/*.png`: 项目相关图片资源

## 核心类和函数说明

### Franka机器人控制 (lerobot_robot_franka/franka.py)

#### Franka类
- `__init__(self, config: FrankaConfig)`: 初始化Franka机器人实例
- `connect(self)`: 连接Franka机器人
- `disconnect(self)`: 断开与Franka机器人的连接
- `_read_gripper_state(self)`: 读取夹爪状态
- `send_action(self, action)`: 发送控制指令给机器人
- `get_observation(self)`: 获取机器人当前观测状态

#### FrankaConfig类
- 配置Franka机器人的各项参数，如IP地址、端口、夹爪阈值等

### 遥操作控制 (lerobot_teleoperator_franka/teleop.py)

#### FrankaTeleop类
- `__init__(self, config: TeleopConfig)`: 初始化遥操作控制器
- `get_action(self)`: 获取遥操作指令
- `apply_offsets(self, joints)`: 应用关节偏移量

#### TeleopConfig类
- 配置遥操作的各项参数

### Dynamixel驱动 (lerobot_teleoperator_franka/dynamixel/driver.py)

#### DynamixelDriverProtocol类
- 定义Dynamixel驱动的接口规范

#### DynamixelDriver类
- `set_joints(self, joints)`: 设置关节角度
- `get_joints(self)`: 获取当前关节角度
- `torque_enabled(self, enabled)`: 启用/禁用扭矩
- `verify_operating_mode(self)`: 验证操作模式

### 数据记录和回放 (scripts/core/)

#### run_record.py
- `main()`: 数据记录主函数

#### run_replay.py
- `run_replay()`: 数据回放函数

#### run_visualize.py
- `to_hwc_uint8_numpy()`: 图像格式转换函数

### 工具函数 (scripts/utils/)

#### dataset_utils.py
- 数据集处理相关工具函数

#### teleop_joint_offsets.py
- `get_start_joints()`: 获取初始关节角度

### 测试函数 (scripts/test/)

#### gripper_ctrl.py
- 夹爪控制测试相关函数