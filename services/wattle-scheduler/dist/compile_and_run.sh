#!/bin/bash

echo "🔨 编译 Wattle C++ 示例..."

# 使用静态库编译
g++ -std=c++17 -I./include example.cpp ./lib/libwattle_cxx.a -pthread -ldl -o example_static

if [ $? -eq 0 ]; then
    echo "✅ 静态库编译成功 - example_static"
else
    echo "❌ 静态库编译失败"
fi

# 使用动态库编译
g++ -std=c++17 -I./include example.cpp -L./lib -lwattle_cxx -Wl,-rpath,./lib -pthread -ldl -o example_dynamic

if [ $? -eq 0 ]; then
    echo "✅ 动态库编译成功 - example_dynamic"
else
    echo "❌ 动态库编译失败"
fi

echo "🚀 运行示例..."
echo "--- 静态库版本 ---"
./example_static

echo ""
echo "--- 动态库版本 ---"  
./example_dynamic
