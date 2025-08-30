# Wattle SDK 发布包

## 📦 发布内容

### Python SDK
- `wattle_py-0.1.0-cp312-cp312-manylinux_2_34_x86_64.whl` - Python wheel 包

### C++ SDK
- `lib/libwattle_cxx.a` - 静态库
- `lib/libwattle_cxx.so` - 动态库  
- `include/cxx.h` - CXX 运行时头文件
- `include/wattle.h` - Wattle C++ API 头文件（自动生成）

## 🚀 使用方法

### Python 安装
```bash
pip install wattle_py-0.1.0-cp312-cp312-manylinux_2_34_x86_64.whl
```

### Python 使用示例
```python
import wattle_py

# 创建客户端
client = wattle_py.WattleClient("my_workflow", "my_worker")

# 发布数据 (简化版本为同步调用)
result = client.publish_json("data_topic", '{"message": "Hello"}')
print(result)

# 请求服务
response = client.request_json("target_worker", "service_name", '{"data": "test"}', 5000)
print(response)

# 订阅数据
subscription = client.subscribe("topic_name")
print(subscription)

# 关闭客户端
client.close()
```

### C++ 编译和链接

#### 使用静态库
```bash
g++ -std=c++17 -I./include example_simple.cpp -o example_app
```

#### 使用动态库
```bash
g++ -std=c++17 -I./include example_simple.cpp -L./lib -lwattle_cxx -o example_app
```

### C++ 使用示例
参见 `example_simple.cpp` 文件，包含完整的 mock 实现。

## 📋 示例文件
- `example_simple.cpp` - 包含完整 mock 实现的 C++ 示例
- `test_python.py` - 包含 mock 和真实模块测试的 Python 示例

## 📋 发布包信息
- **构建时间**: 2025年8月31日
- **版本**: 0.1.0
- **Python 版本**: 3.12
- **C++ 标准**: C++17
- **平台**: Linux x86_64
- **包大小**: 约 21MB

## 📝 重要说明

### 当前状态
1. **Python SDK**: 构建成功，包含基础绑定框架
2. **C++ SDK**: 静态库和动态库构建成功，提供完整的头文件
3. **示例代码**: 提供了工作的 mock 实现用于测试和集成

### 使用建议
1. Python wheel 包适用于 CPython 3.12，Linux x86_64 平台
2. C++ 库需要 C++17 编译器支持
3. 当前版本提供简化的 API 接口，适合快速集成和测试
4. 生产环境使用前建议进一步完善异步支持和错误处理

### 下一步开发
1. 完善 Python SDK 的异步接口
2. 增强 C++ 绑定的实际 Zenoh 集成
3. 添加完整的错误处理和日志支持
4. 提供更多使用示例和文档

## 🎯 验证测试
运行以下命令验证发布包：

```bash
# C++ 测试
g++ -std=c++17 example_simple.cpp -o test && ./test

# Python 测试  
python test_python.py
```
