#!/usr/bin/env python3
"""
Wattle Python SDK 示例

展示如何使用 Python SDK 进行 Worker 间通信
"""

import asyncio
import os
import json
from datetime import datetime

# 设置环境变量
os.environ["WATTLE_WORKFLOW_NAME"] = "python_demo"
os.environ["WATTLE_WORKER_NAME"] = "python_worker"

# 尝试导入 wattle_py，如果没有安装则使用 mock 版本
try:
    from wattle_py import WattleClient, get_sample_data
    REAL_SDK = True
except ImportError:
    print("⚠️  Wattle SDK 未安装，使用 mock 版本进行演示")
    REAL_SDK = False
    
    # Mock 实现
    class WattleClient:
        def __init__(self, workflow_name=None, worker_name=None):
            self.workflow_name = workflow_name or "mock_workflow"
            self.worker_name = worker_name or "mock_worker"
        
        async def initialize(self): pass
        
        @classmethod
        async def create(cls, workflow_name=None, worker_name=None):
            client = cls(workflow_name, worker_name)
            await client.initialize()
            return client
        
        async def publish_json(self, service_name, data):
            print(f"📤 [MOCK] Published to {service_name}: {json.dumps(data, indent=2)}")
        
        async def request_json(self, target_worker, service_name, data, timeout_secs=None):
            print(f"🔄 [MOCK] Request to {target_worker}/{service_name}: {json.dumps(data)}")
            return {"status": "ok", "message": "Mock response", "timestamp": datetime.now().isoformat()}
        
        async def subscribe_json(self, service_name, callback):
            print(f"📡 [MOCK] Subscribed to {service_name}")
            # 模拟接收一些数据
            await asyncio.sleep(0.1)
            callback({"mock_message": "Hello from subscription", "timestamp": datetime.now().isoformat()})
        
        async def close(self):
            print("✅ [MOCK] Client closed")
        
        async def __aenter__(self): 
            await self.initialize()
            return self
        async def __aexit__(self, *args): 
            await self.close()
    
    def get_sample_data():
        return [
            {"name": "Alice", "age": 25},
            {"name": "Bob", "age": 30}, 
            {"name": "Charlie", "age": 35}
        ]


async def demo_basic_features():
    """演示基本功能"""
    print("🚀 Wattle Python SDK 基本功能演示")
    print("=" * 40)
    
    # 创建客户端
    async with WattleClient("python_demo", "worker_a") as client:
        print(f"✅ 客户端创建成功")
        print(f"   工作流: {client.workflow_name}")
        print(f"   Worker: {client.worker_name}")
        
        # 1. 发布 JSON 数据
        print("\n📤 1. 发布 JSON 数据")
        task_data = {
            "task_id": "task_001",
            "operation": "data_processing",
            "input_file": "dataset.csv",
            "parameters": {
                "algorithm": "kmeans",
                "clusters": 3,
                "iterations": 100
            },
            "priority": "high",
            "timestamp": datetime.now().isoformat()
        }
        
        await client.publish_json("data_processing", task_data)
        print("   ✅ JSON 数据发布成功")
        
        # 2. 发送请求
        print("\n🔄 2. 发送请求并等待回复")
        request_data = {
            "operation": "transform_data",
            "input_format": "csv",
            "output_format": "parquet",
            "filters": {
                "age": {"min": 18, "max": 65},
                "status": "active"
            }
        }
        
        try:
            response = await client.request_json("worker_b", "transform_service", 
                                                request_data, timeout_secs=5)
            print(f"   ✅ 收到响应: {json.dumps(response, indent=2)}")
        except Exception as e:
            print(f"   ⚠️  请求超时或失败: {e}")
        
        # 3. 演示订阅
        print("\n📡 3. 订阅数据演示")
        received_messages = []
        
        def message_handler(data):
            print(f"   📥 收到订阅数据: {json.dumps(data)}")
            received_messages.append(data)
        
        # 启动订阅
        await client.subscribe_json("notifications", message_handler)
        
        # 等待一段时间接收消息
        await asyncio.sleep(0.5)
        
        print(f"   📊 总共接收到 {len(received_messages)} 条消息")


async def demo_multi_worker():
    """演示多 Worker 协作"""
    print("\n🤝 多 Worker 协作演示")
    print("=" * 30)
    
    # 创建多个 worker 客户端
    clients = {}
    
    try:
        # Worker A - 数据生产者
        clients['worker_a'] = await WattleClient.create("collaboration_demo", "worker_a")
        print("✅ Worker A 创建成功")
        
        # Worker B - 数据处理器
        clients['worker_b'] = await WattleClient.create("collaboration_demo", "worker_b")  
        print("✅ Worker B 创建成功")
        
        # Worker C - 结果收集器
        clients['worker_c'] = await WattleClient.create("collaboration_demo", "worker_c")
        print("✅ Worker C 创建成功")
        
        # 设置 Worker C 接收结果
        results = []
        def collect_results(data):
            print(f"   📥 Worker C 收到结果: {data.get('result_id', 'unknown')}")
            results.append(data)
        
        await clients['worker_c'].subscribe_json("results", collect_results)
        print("🔄 Worker C 开始监听结果")
        
        # Worker A 生产数据并发送给 Worker B
        print("\n📊 数据处理流程:")
        for i in range(3):
            # Worker A 生成数据
            raw_data = {
                "batch_id": f"batch_{i+1:03d}",
                "data": get_sample_data(),
                "metadata": {
                    "source": "worker_a",
                    "timestamp": datetime.now().isoformat(),
                    "version": "1.0"
                }
            }
            
            print(f"   🔧 Worker A 处理批次 {i+1}")
            
            # Worker A 请求 Worker B 处理数据
            try:
                processed = await clients['worker_a'].request_json(
                    "worker_b", "process_batch", raw_data, timeout_secs=3
                )
                
                print(f"   ✅ Worker B 处理完成")
                
                # Worker B 将结果发送给 Worker C
                result_data = {
                    "result_id": f"result_{i+1:03d}",
                    "original_batch": raw_data["batch_id"], 
                    "processed_data": processed,
                    "processing_time": 0.1 + i * 0.05,
                    "worker": "worker_b"
                }
                
                await clients['worker_b'].publish_json("results", result_data)
                
            except Exception as e:
                print(f"   ❌ 处理失败: {e}")
            
            await asyncio.sleep(0.2)
        
        # 等待所有结果被收集
        await asyncio.sleep(1)
        print(f"\n📈 处理完成！Worker C 收集到 {len(results)} 个结果")
        
    finally:
        # 清理所有客户端
        for name, client in clients.items():
            await client.close()
            print(f"✅ {name} 已关闭")


async def demo_error_handling():
    """演示错误处理"""
    print("\n⚠️  错误处理演示") 
    print("=" * 20)
    
    async with WattleClient("error_demo", "test_worker") as client:
        # 1. 超时处理
        print("1. 超时测试")
        try:
            await client.request_json("nonexistent_worker", "test_service", 
                                     {"test": "data"}, timeout_secs=1)
        except Exception as e:
            print(f"   ✅ 正确捕获超时: {type(e).__name__}")
        
        # 2. 无效数据处理
        print("2. 数据验证测试")
        try:
            # 发布复杂数据测试 JSON 序列化
            complex_data = {
                "nested": {"deep": {"data": list(range(100))}},
                "timestamp": datetime.now().isoformat(),
                "metadata": {"tags": ["test", "demo", "python"]}
            }
            await client.publish_json("complex_test", complex_data)
            print("   ✅ 复杂数据处理成功")
        except Exception as e:
            print(f"   ❌ 数据处理失败: {e}")


async def main():
    """主演示函数"""
    print(f"🐍 Wattle Python SDK 完整演示")
    print(f"SDK 状态: {'真实版本' if REAL_SDK else 'Mock 版本'}")
    print("=" * 50)
    
    try:
        # 基本功能演示
        await demo_basic_features()
        
        # 多 Worker 协作演示
        await demo_multi_worker()
        
        # 错误处理演示
        await demo_error_handling()
        
        print("\n🎉 所有演示完成！")
        print("\n📋 演示功能总结:")
        print("   ✅ JSON 数据发布")
        print("   ✅ Request/Reply 通信")
        print("   ✅ 数据订阅")
        print("   ✅ 多 Worker 协作")
        print("   ✅ 异步上下文管理")
        print("   ✅ 错误处理和超时")
        
    except Exception as e:
        print(f"\n❌ 演示过程中出现错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
