# MuJoCo Zenoh 集成 - 简化版

## 文件说明

- **simulator.py** - 无界面仿真器，发布状态，接收动作
- **state_viewer.py** - 有界面状态查看器，接收状态并显示可视化
- **action_publisher.py** - 动作发布器，发送控制命令

## 特性

✅ **动态 XML 解析** - 自动解析模型文件获取关节、执行器信息和约束范围  
✅ **统一前缀** - 可配置的 Zenoh topic 前缀  
✅ **安全控制** - 自动限制控制值在合理范围内  
✅ **实时显示** - 状态查看器显示详细的关节和执行器信息  

## 快速开始

### 1. 启动仿真器（终端1）
```bash
# 默认配置
python simulator.py

# 自定义配置
python simulator.py --model franka_emika_panda/scene.xml --prefix robot1 --rate 100
```

### 2. 启动状态查看器（终端2）
```bash
# 默认配置（会显示可视化界面）
python state_viewer.py

# 自定义配置
python state_viewer.py --model franka_emika_panda/scene.xml --prefix robot1
```

### 3. 发送动作命令（终端3）

#### 交互模式
```bash
python action_publisher.py

# 可用命令:
> info                    # 显示模型信息
> ctrl 0.1 0.0 -0.2      # 发送控制输入
> pos 0.0 0.0 0.0 -1.57  # 发送位置目标
> zero                   # 发送零控制
> home                   # 回到home位置
> demo_sine 30 0.5       # 30秒正弦波，幅值0.5
> quit                   # 退出
```

#### 自动演示模式
```bash
# 正弦波控制
python action_publisher.py --mode sine --duration 30 --amplitude 0.3

# 阶跃位置控制
python action_publisher.py --mode step --duration 20
```

## Zenoh Topics

- **Action Topic**: `{prefix}/action`
- **State Topic**: `{prefix}/state`

默认前缀: `mujoco`

## 动作格式

```json
{
  "type": "ctrl|qpos|qvel",
  "values": [1.0, 2.0, 3.0, ...],
  "timestamp": 1234567890.123
}
```

## 状态格式

```json
{
  "timestamp": 1234567890.123,
  "sim_time": 1.234,
  "joints": {
    "names": ["joint1", "joint2", ...],
    "positions": [0.1, 0.2, ...],
    "velocities": [0.0, 0.0, ...],
    "ranges": {"joint1": [-3.14, 3.14], ...}
  },
  "actuators": {
    "names": ["actuator1", "actuator2", ...],
    "controls": [0.1, 0.0, ...],
    "ranges": {"actuator1": [-1.0, 1.0], ...}
  },
  "bodies": {
    "names": ["body1", "body2", ...],
    "positions": [0.0, 0.0, 0.5, ...],
    "orientations": [1.0, 0.0, 0.0, 0.0, ...]
  }
}
```

## 使用示例

### 示例1: 本地测试
```bash
# 终端1
python simulator.py

# 终端2  
python state_viewer.py

# 终端3
python action_publisher.py
> demo_sine 10 0.2
```

### 示例2: 自定义前缀
```bash
# 所有终端都使用相同前缀
python simulator.py --prefix robot_arm
python state_viewer.py --prefix robot_arm  
python action_publisher.py --prefix robot_arm
```

### 示例3: 不同模型
```bash
python simulator.py --model my_robot.xml
python state_viewer.py --model my_robot.xml
python action_publisher.py --model my_robot.xml
```

## 模型信息

程序会自动解析 XML 文件获取：
- 关节名称和运动范围
- 执行器名称和控制范围  
- 身体名称和层次结构
- 默认位置和参数

在 action_publisher 中输入 `info` 命令可以查看完整的模型信息。
