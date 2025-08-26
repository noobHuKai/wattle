# MuJoCo Zenoh 集成 - 简化版

三个核心文件实现完整的分布式 MuJoCo 仿真系统。

## 🚀 快速开始

```bash
# 安装依赖
pip install mujoco eclipse-zenoh numpy

# 终端1：启动无界面仿真器
python simulator.py

# 终端2：启动可视化查看器
python state_viewer.py

# 终端3：发送控制命令
python action_publisher.py
```

## 📁 文件说明

- **simulator.py** - 无界面仿真器，发布状态 `{prefix}/state`，接收动作 `{prefix}/action`
- **state_viewer.py** - 有界面查看器，接收状态并实时渲染
- **action_publisher.py** - 动作发布器，交互式控制或演示模式

## ⚡ 特性

- ✅ **动态XML解析** - 自动获取关节、执行器信息
- ✅ **统一前缀** - 可配置的topic前缀避免冲突  
- ✅ **实时通信** - 使用Zenoh实现低延迟分布式通信
- ✅ **简洁易用** - 最小化配置，开箱即用

## 🎯 Topics

- `{prefix}/action` - 动作命令（JSON格式）
- `{prefix}/state` - 状态信息（JSON格式，60Hz）

默认prefix: `mujoco`

## 🔧 使用示例

### 基本控制
```bash
python action_publisher.py
> ctrl 0.1 0.0 0.0 0.0 0.0 0.0 0.0  # 发送控制输入
> zero                               # 零控制输入  
> demo_sine 30                       # 30秒正弦波演示
> quit                               # 退出
```

### 自定义配置
```bash
# 使用不同模型和前缀
python simulator.py --model my_robot.xml --prefix robot1
python state_viewer.py --model my_robot.xml --prefix robot1  
python action_publisher.py --model my_robot.xml --prefix robot1
```

### 快速演示
```bash
python demo_simple.py  # 运行30秒自动演示
```

## 📋 数据格式

### 动作 (action)
```json
{
  "joints": {
    "positions": [0.0, 0.0, ...],
    "velocities": [0.0, 0.0, ...]
  },
  "actuators": {
    "controls": [0.1, 0.0, ...]  
  }
}
```

### 状态 (state)  
```json
{
  "timestamp": 1234567890.123,
  "simulation_time": 1.234,
  "joints": {
    "names": ["joint1", "joint2", ...],
    "positions": [0.1, 0.2, ...],
    "velocities": [0.0, 0.0, ...] 
  },
  "actuators": {
    "names": ["actuator1", "actuator2", ...],
    "controls": [0.1, 0.0, ...]
  }
}
```

## 🔥 参数

### 通用参数
- `--model` - MuJoCo模型文件路径 (默认: `franka_emika_panda/scene.xml`)
- `--prefix` - Zenoh topic前缀 (默认: `mujoco`)

### simulator.py
- `--duration` - 运行时长（秒）
- `--rate` - 状态发布频率（Hz，默认60）

### action_publisher.py  
- `--mode` - 运行模式 (`interactive`/`sine`)
- `--duration` - 演示时长（秒）

简单、高效、即用！🎉
