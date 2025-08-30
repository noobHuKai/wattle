"""
Wattle Python SDK

Multi-language SDK for Worker-to-Worker communication using Zenoh + Apache Arrow.
"""

import os
import json
import asyncio
from typing import Optional, Callable, Any, Dict

try:
    from ._internal import PyWattleClient, create_sample_data
except ImportError:
    # 开发模式下的占位符
    class PyWattleClient:
        def __init__(self):
            self.workflow_name = ""
            self.worker_name = ""
        
        async def initialize(self): pass
        async def publish_json(self, service_name: str, data: str): pass
        async def request_json(self, target_worker: str, service_name: str, data: str, timeout_secs: Optional[int] = None) -> str: return "{}"
        async def subscribe_json(self, service_name: str, callback): pass
        async def close(self): pass
    
    def create_sample_data() -> str:
        return json.dumps([
            {"name": "Alice", "age": 25},
            {"name": "Bob", "age": 30},
            {"name": "Charlie", "age": 35}
        ])

__version__ = "0.1.0"
__all__ = ["WattleClient", "create_sample_data"]


class WattleClient:
    """
    Wattle SDK 主客户端类
    
    提供 Worker 间通信功能，支持 JSON 和 Arrow 数据格式。
    """
    
    def __init__(self, workflow_name: Optional[str] = None, worker_name: Optional[str] = None):
        """
        初始化 Wattle 客户端
        
        Args:
            workflow_name: 工作流名称，如果为 None 则从环境变量 WATTLE_WORKFLOW_NAME 读取
            worker_name: Worker 名称，如果为 None 则从环境变量 WATTLE_WORKER_NAME 读取
        """
        # 设置环境变量
        if workflow_name:
            os.environ["WATTLE_WORKFLOW_NAME"] = workflow_name
        if worker_name:
            os.environ["WATTLE_WORKER_NAME"] = worker_name
            
        self._inner = PyWattleClient()
        self._initialized = False
    
    async def initialize(self):
        """异步初始化客户端"""
        if not self._initialized:
            await self._inner.initialize()
            self._initialized = True
    
    @classmethod
    async def create(cls, workflow_name: Optional[str] = None, worker_name: Optional[str] = None) -> 'WattleClient':
        """
        创建并初始化客户端的便捷方法
        
        Args:
            workflow_name: 工作流名称
            worker_name: Worker 名称
            
        Returns:
            初始化完成的 WattleClient 实例
        """
        client = cls(workflow_name, worker_name)
        await client.initialize()
        return client
    
    async def publish_json(self, service_name: str, data: Dict[str, Any]):
        """
        发布 JSON 数据
        
        Args:
            service_name: 服务名称
            data: 要发布的数据（字典格式）
        """
        await self._ensure_initialized()
        json_str = json.dumps(data)
        await self._inner.publish_json(service_name, json_str)
    
    async def request_json(self, target_worker: str, service_name: str, data: Dict[str, Any], 
                          timeout_secs: Optional[int] = None) -> Dict[str, Any]:
        """
        发送 JSON 请求并等待回复
        
        Args:
            target_worker: 目标 Worker 名称
            service_name: 服务名称  
            data: 请求数据（字典格式）
            timeout_secs: 超时时间（秒）
            
        Returns:
            响应数据（字典格式）
        """
        await self._ensure_initialized()
        json_str = json.dumps(data)
        response_str = await self._inner.request_json(target_worker, service_name, json_str, timeout_secs)
        return json.loads(response_str)
    
    async def subscribe_json(self, service_name: str, callback: Callable[[Dict[str, Any]], None]):
        """
        订阅 JSON 数据
        
        Args:
            service_name: 服务名称
            callback: 接收数据的回调函数
        """
        await self._ensure_initialized()
        
        def wrapper(json_str: str):
            try:
                data = json.loads(json_str)
                callback(data)
            except json.JSONDecodeError as e:
                print(f"Failed to parse JSON: {e}")
        
        await self._inner.subscribe_json(service_name, wrapper)
    
    async def close(self):
        """关闭客户端"""
        if self._initialized:
            await self._inner.close()
            self._initialized = False
    
    @property
    def workflow_name(self) -> str:
        """获取工作流名称"""
        return self._inner.workflow_name
    
    @property 
    def worker_name(self) -> str:
        """获取 Worker 名称"""
        return self._inner.worker_name
    
    async def _ensure_initialized(self):
        """确保客户端已初始化"""
        if not self._initialized:
            await self.initialize()
    
    async def __aenter__(self):
        """异步上下文管理器入口"""
        await self.initialize()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """异步上下文管理器出口"""
        await self.close()


# 便捷函数
def get_sample_data() -> list:
    """
    获取示例数据
    
    Returns:
        示例数据列表
    """
    json_str = create_sample_data()
    return json.loads(json_str)


# 同步 API 包装器（可选）
class WattleClientSync:
    """
    Wattle SDK 同步客户端包装器
    
    将异步 API 包装为同步调用，适合在同步代码中使用。
    """
    
    def __init__(self, workflow_name: Optional[str] = None, worker_name: Optional[str] = None):
        self._async_client = WattleClient(workflow_name, worker_name)
        self._loop = None
    
    def _ensure_loop(self):
        """确保事件循环存在"""
        if self._loop is None:
            try:
                self._loop = asyncio.get_event_loop()
            except RuntimeError:
                self._loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._loop)
    
    def initialize(self):
        """同步初始化客户端"""
        self._ensure_loop()
        return self._loop.run_until_complete(self._async_client.initialize())
    
    def publish_json(self, service_name: str, data: Dict[str, Any]):
        """同步发布 JSON 数据"""
        self._ensure_loop()
        return self._loop.run_until_complete(
            self._async_client.publish_json(service_name, data)
        )
    
    def request_json(self, target_worker: str, service_name: str, data: Dict[str, Any], 
                    timeout_secs: Optional[int] = None) -> Dict[str, Any]:
        """同步发送 JSON 请求"""
        self._ensure_loop()
        return self._loop.run_until_complete(
            self._async_client.request_json(target_worker, service_name, data, timeout_secs)
        )
    
    def close(self):
        """同步关闭客户端"""
        if self._loop:
            self._loop.run_until_complete(self._async_client.close())
    
    @property
    def workflow_name(self) -> str:
        return self._async_client.workflow_name
    
    @property
    def worker_name(self) -> str:
        return self._async_client.worker_name
