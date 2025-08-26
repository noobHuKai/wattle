# MuJoCo Zenoh 集成

这个项目在 MuJoCo 仿真器中集成了 Zenoh 通信，实现了分布式的机器人仿真和控制系统。

## 功能特性

1. **统一的 topic 前缀**：所有 Zenoh topic 都有可定制的统一前缀
2. **动作监听**：使用单个 topic 监听所有动作（action）
3. **态势推送**：新增 topic 用于推送仿真状态（state）
4. **状态渲染器**：独立的 Python 程序接收状态并可选地实时渲染

## 文件结构

- `main.py` - 主要的 MuJoCo 仿真器，集成了 Zenoh 通信
- `state_viewer.py` - 状态接收器和可视化渲染器
- `action_publisher.py` - 动作发布器示例和测试工具
- `franka_emika_panda/` - Franka Emika Panda 机器人模型文件

## Zenoh Topics

### Action Topic: `{prefix}/action`
接收动作消息，支持以下类型：
- `ctrl` - 控制输入
- `qpos` - 关节位置
- `qvel` - 关节速度

消息格式：
```json
{
    "type": "ctrl|qpos|qvel",
    "values": [1.0, 2.0, 3.0, ...],
    "timestamp": 1234567890.123
}
```

### State Topic: `{prefix}/state`
发布仿真状态信息，包含：
- `timestamp` - 时间戳
- `time` - 仿真时间
- `qpos` - 关节位置
- `qvel` - 关节速度
- `qacc` - 关节加速度
- `ctrl` - 控制输入
- `xpos` - 身体位置
- `xquat` - 身体方向（四元数）
- `xmat` - 身体旋转矩阵
- `sensor_data` - 传感器数据

## 安装依赖

```bash
pip install mujoco zenoh numpy
```

## 使用方法

### 1. 启动主仿真器

启动带可视化的仿真器：
```bash
python main.py
```

仿真器将：
- 加载 Franka Emika Panda 机器人模型
- 监听动作消息 (`panda_robot/action`)
- 发布状态消息 (`panda_robot/state`)
- 显示 MuJoCo 可视化窗口

### 2. 启动状态查看器

在另一个终端中启动状态查看器：

带可视化：
```bash
python state_viewer.py
```

仅控制台模式：
```bash
python state_viewer.py --no-viewer
```

自定义参数：
```bash
python state_viewer.py --prefix my_robot --model my_model.xml
```

### 3. 发送动作命令

#### 交互模式
```bash
python action_publisher.py --mode interactive
```

然后可以输入命令：
- `ctrl 0.1 0.2 0.0 0.0 0.0 0.0 0.0` - 发送控制输入
- `pos 0.0 0.0 0.0 -1.57 0.0 1.57 0.78` - 发送位置目标
- `demo_sine 30` - 运行30秒正弦波控制
- `demo_step 20` - 运行20秒阶跃位置控制
- `quit` - 退出

#### 自动演示模式
正弦波控制：
```bash
python action_publisher.py --mode sine --duration 30
```

阶跃位置控制：
```bash
python action_publisher.py --mode step --duration 20
```

## 命令行参数

### main.py
主仿真器目前使用硬编码的参数，你可以修改代码中的：
- `model_path`: MuJoCo 模型文件路径
- `topic_prefix`: Zenoh topic 前缀

### state_viewer.py
- `--model`, `-m`: MuJoCo 模型文件路径（默认: `franka_emika_panda/scene.xml`）
- `--prefix`, `-p`: Zenoh topic 前缀（默认: `panda_robot`）
- `--no-viewer`, `-n`: 禁用可视化查看器，仅使用控制台模式

### action_publisher.py
- `--prefix`, `-p`: Zenoh topic 前缀（默认: `panda_robot`）
- `--mode`, `-m`: 操作模式 (`interactive`, `sine`, `step`)
- `--duration`, `-d`: 演示持续时间（秒）

## 典型使用场景

### 场景1：本地仿真和控制
1. 启动主仿真器：`python main.py`
2. 在交互模式下发送动作：`python action_publisher.py`

### 场景2：分布式系统
1. 在计算机A上启动仿真器：`python main.py`
2. 在计算机B上启动状态查看器：`python state_viewer.py --prefix panda_robot`
3. 在计算机C上运行控制器：`python action_publisher.py --mode sine`

### 场景3：无头仿真
修改 `main.py` 中的 `simulator.run_with_viewer()` 为 `simulator.run_headless()`，然后：
1. 启动无头仿真器：`python main.py`
2. 启动状态查看器：`python state_viewer.py`
3. 发送控制命令

## 自定义和扩展

### 修改 topic 前缀
在所有程序中使用相同的 `--prefix` 参数，例如：
```bash
# 终端1
python main.py  # 修改代码中的 topic_prefix="my_robot"
# 终端2  
python state_viewer.py --prefix my_robot
# 终端3
python action_publisher.py --prefix my_robot
```

### 添加新的动作类型
在 `main.py` 的 `action_callback` 方法中添加新的动作类型处理：

```python
elif action_data.get('type') == 'custom_action':
    # 处理自定义动作
    custom_data = action_data.get('values', [])
    # 你的自定义逻辑
```

### 扩展状态信息
在 `main.py` 的 `get_state` 方法中添加更多状态信息：

```python
state = {
    # 现有状态...
    'custom_data': your_custom_data,
    'additional_sensors': additional_sensor_readings,
}
```

## 故障排除

### Zenoh 连接问题
确保所有程序使用相同的 Zenoh 配置。默认情况下使用本地组播发现。

### 模型文件路径问题
确保 `franka_emika_panda/scene.xml` 文件存在，或使用 `--model` 参数指定正确路径。

### 性能问题
- 调整状态发布频率（`state_publish_rate`）
- 使用无头模式减少可视化开销
- 检查网络延迟（分布式部署时）

## 注意事项

1. 确保所有节点使用相同的 topic 前缀
2. 状态发布频率默认为60Hz，可根据需要调整
3. 动作消息会立即应用到仿真中
4. 使用 Ctrl+C 优雅地关闭所有程序
