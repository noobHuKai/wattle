#!/usr/bin/env python3
"""
快速测试脚本 - 验证 MuJoCo Zenoh 集成的基本功能
"""

import zenoh
import json
import time
import mujoco


def test_zenoh_connectivity():
    """测试 Zenoh 连接"""
    print("测试 Zenoh 连接...")
    try:
        session = zenoh.open(zenoh.Config())
        print("✓ Zenoh 会话创建成功")
        
        # 测试发布器
        publisher = session.declare_publisher("test/topic")
        print("✓ 发布器创建成功")
        
        # 测试订阅器  
        def callback(sample):
            print(f"✓ 收到消息: {sample.payload.decode('utf-8')}")
        
        subscriber = session.declare_subscriber("test/topic", callback)
        print("✓ 订阅器创建成功")
        
        # 发送测试消息
        test_message = {"test": "message", "timestamp": time.time()}
        publisher.put(json.dumps(test_message))
        print("✓ 测试消息发送成功")
        
        # 等待接收
        time.sleep(1)
        
        session.close()
        print("✓ Zenoh 连接测试完成")
        return True
        
    except Exception as e:
        print(f"✗ Zenoh 连接测试失败: {e}")
        return False


def test_mujoco_model():
    """测试 MuJoCo 模型加载"""
    print("\n测试 MuJoCo 模型加载...")
    try:
        model = mujoco.MjModel.from_xml_path("franka_emika_panda/scene.xml")
        data = mujoco.MjData(model)
        
        print(f"✓ 模型加载成功")
        print(f"  - 关节数量: {model.nq}")
        print(f"  - 控制输入数量: {model.nu}")  
        print(f"  - 身体数量: {model.nbody}")
        print(f"  - 几何体数量: {model.ngeom}")
        
        # 测试仿真步骤
        mujoco.mj_step(model, data)
        print("✓ 仿真步骤执行成功")
        
        return True
        
    except Exception as e:
        print(f"✗ MuJoCo 模型测试失败: {e}")
        return False


def test_integration():
    """测试完整集成"""
    print("\n测试完整集成...")
    
    try:
        # 导入我们的模块
        from engine import MuJoCoZenohSimulator
        
        # 创建仿真器（不启动循环）
        simulator = MuJoCoZenohSimulator(
            model_path="franka_emika_panda/scene.xml",
            topic_prefix="test_robot"
        )
        
        print("✓ 仿真器创建成功")
        
        # 测试状态获取
        state = simulator.get_state()
        print("✓ 状态获取成功")
        print(f"  - 当前时间: {state['time']}")
        print(f"  - 关节位置数量: {len(state['qpos'])}")
        print(f"  - 关节速度数量: {len(state['qvel'])}")
        
        # 测试动作回调
        test_action = {
            "type": "ctrl",
            "values": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "timestamp": time.time()
        }
        
        # 模拟 Zenoh 样本
        class MockSample:
            def __init__(self, data):
                self.payload = MockPayload(json.dumps(data))
        
        class MockPayload:
            def __init__(self, data):
                self._data = data.encode('utf-8')
            
            def decode(self, encoding='utf-8'):
                return self._data.decode(encoding)
        
        # 测试动作回调
        mock_sample = MockSample(test_action)
        simulator.action_callback(mock_sample)
        print("✓ 动作回调测试成功")
        
        # 清理
        simulator.close()
        print("✓ 集成测试完成")
        return True
        
    except Exception as e:
        print(f"✗ 集成测试失败: {e}")
        return False


def main():
    print("=== MuJoCo Zenoh 集成测试 ===")
    
    # 运行所有测试
    tests = [
        test_zenoh_connectivity,
        test_mujoco_model, 
        test_integration
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    # 结果总结
    print("=== 测试结果 ===")
    print(f"通过: {passed}/{total}")
    
    if passed == total:
        print("🎉 所有测试通过！系统已准备就绪。")
        print("\n下一步:")
        print("1. 运行主仿真器: python main.py")
        print("2. 运行状态查看器: python state_viewer.py")
        print("3. 发送动作命令: python action_publisher.py")
    else:
        print("⚠️  部分测试失败，请检查配置。")
    
    return passed == total


if __name__ == "__main__":
    main()
