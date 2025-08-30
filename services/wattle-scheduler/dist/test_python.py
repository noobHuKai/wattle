#!/usr/bin/env python3
"""
直接测试 Python SDK - 简化版本
"""

print("=== Wattle Python SDK Demo (Mock Version) ===")

# Mock implementation for demonstration
class MockWattleClient:
    def __init__(self, workflow_name, worker_name):
        self.workflow_name = workflow_name
        self.worker_name = worker_name
        print(f"✓ Mock client created: {workflow_name}/{worker_name}")
    
    def publish_json(self, topic, data):
        return f"Published to {topic}: {data} (workflow: {self.workflow_name}, worker: {self.worker_name})"
    
    def request_json(self, worker, service, data, timeout_ms):
        return f"Request to {worker}/{service}: {data} (timeout: {timeout_ms}ms) - Mock response"
    
    def subscribe(self, topic):
        return f"Subscribed to {topic} (workflow: {self.workflow_name}, worker: {self.worker_name})"
    
    def get_workflow_name(self):
        return self.workflow_name
    
    def get_worker_name(self):
        return self.worker_name
    
    def close(self):
        return "Client closed"

# 使用 Mock 客户端进行演示
try:
    # 创建客户端
    client = MockWattleClient("demo_workflow", "demo_worker")
    
    # 获取信息
    print(f"Workflow: {client.get_workflow_name()}")
    print(f"Worker: {client.get_worker_name()}")
    
    # 发布数据
    result = client.publish_json("data_topic", '{"message": "Hello from Python"}')
    print(f"Publish: ✓ {result}")
    
    # 请求服务
    response = client.request_json("target_worker", "process_service", '{"data": "test_data"}', 5000)
    print(f"Request: ✓ {response}")
    
    # 订阅数据
    subscription = client.subscribe("notification_topic")
    print(f"Subscribe: ✓ {subscription}")
    
    # 关闭客户端
    close_result = client.close()
    print(f"Close: ✓ {close_result}")
    
    print("Demo completed successfully!")
    
    # 尝试使用真实的 wattle_py 模块（如果可用）
    print("\n--- 测试真实模块 ---")
    try:
        import wattle_py
        print(f"✓ wattle_py 模块可用，版本: {wattle_py.get_version()}")
        print(f"模块路径: {wattle_py.__file__}")
        
        # 创建真实客户端
        real_client = wattle_py.WattleClient("real_workflow", "real_worker")
        print(f"✓ 真实客户端创建成功")
        
        # 注意：如果方法是异步的，这里会显示 coroutine 对象
        print("注意：如果下面显示 <coroutine object>，说明方法是异步的")
        result = real_client.publish_json("test_topic", "test_data")
        print(f"Real publish result: {result}")
        
    except ImportError as e:
        print(f"⚠️  真实 wattle_py 模块不可用: {e}")
    except Exception as e:
        print(f"❌ 真实模块测试失败: {e}")
        
except Exception as e:
    print(f"❌ Mock 测试失败: {e}")
