#!/usr/bin/env python3
"""
Wattle Python SDK 示例
"""

def test_wattle_python():
    try:
        import wattle_py
        print("=== Wattle Python SDK Demo ===")
        
        # 创建客户端
        client = wattle_py.WattleClient("demo_workflow", "demo_worker")
        print(f"✓ 客户端创建成功")
        
        # 获取信息 (简化版本没有这些方法，直接显示)
        print(f"Workflow: demo_workflow")
        print(f"Worker: demo_worker")
        
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
        
    except ImportError as e:
        print(f"❌ 导入 wattle_py 失败: {e}")
        print("请先安装: pip install wattle_py-0.1.0-cp312-cp312-manylinux_2_34_x86_64.whl")
    except Exception as e:
        print(f"❌ 运行错误: {e}")

if __name__ == "__main__":
    test_wattle_python()
