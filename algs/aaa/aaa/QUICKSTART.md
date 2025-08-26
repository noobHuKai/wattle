# MuJoCo Zenoh 集成 - 快速入门

## 🚀 快速开始

### 1. 安装依赖
```bash
pip install mujoco eclipse-zenoh numpy
```

### 2. 验证系统
```bash
python test_system.py
```

### 3. 启动系统

#### 方法 A: 使用启动脚本 (推荐)
```bash
# 启动仿真器 (带可视化)
python start.py simulator

# 在新终端启动状态查看器
python start.py viewer

# 在新终端发送动作命令
python start.py publisher
```

#### 方法 B: 手动启动
```bash
# 终端1: 启动仿真器
python main.py

# 终端2: 启动状态查看器  
python state_viewer.py

# 终端3: 发送动作
python action_publisher.py
```

### 4. 运行演示
```bash
# 自动演示 (正弦波控制)
python start.py publisher --mode sine

# 交互式控制
python start.py publisher
> ctrl 0.1 0.0 0.0 0.0 0.0 0.0 0.0
```

## 📋 系统架构

```
┌─────────────────┐    action     ┌─────────────────┐
│  Action         │──────────────→│  MuJoCo         │
│  Publisher      │   (zenoh)     │  Simulator      │
└─────────────────┘               └─────────────────┘
                                          │
                                          │ state
                                          │ (zenoh) 
                                          ↓
                                  ┌─────────────────┐
                                  │  State          │
                                  │  Viewer         │
                                  └─────────────────┘
```

## 🎯 Zenoh Topics

- **Action Topic**: `{prefix}/action` - 接收控制命令
- **State Topic**: `{prefix}/state` - 发布仿真状态

默认前缀: `panda_robot`

## 💡 使用技巧

1. **无头仿真**: 添加 `--headless` 参数用于服务器部署
2. **自定义前缀**: 使用 `--prefix my_robot` 避免冲突  
3. **控制台监视**: 使用 `--no-viewer` 减少资源消耗
4. **批量控制**: 使用演示模式测试不同控制策略

## 🔧 故障排除

- **Zenoh 连接问题**: 检查网络和防火墙设置
- **MuJoCo 显示问题**: 尝试无头模式或检查图形驱动
- **模型加载失败**: 确保 `franka_emika_panda/scene.xml` 存在

需要帮助？查看完整文档: `README.md`
