#!/bin/bash

# Wattle C++ SDK Build and Run Script

set -e

echo "🚀 Building Wattle C++ SDK..."
echo "=============================="

# Check if we're in the right directory
if [ ! -f "Cargo.toml" ]; then
    echo "❌ Error: Please run this script from the apis/cxx directory"
    exit 1
fi

# Check environment variables
if [ -z "$WATTLE_WORKFLOW_NAME" ]; then
    echo "⚠️  WATTLE_WORKFLOW_NAME not set, using default: cpp_demo_workflow"
    export WATTLE_WORKFLOW_NAME="cpp_demo_workflow"
fi

if [ -z "$WATTLE_WORKER_NAME" ]; then
    echo "⚠️  WATTLE_WORKER_NAME not set, using default: cpp_demo_worker"
    export WATTLE_WORKER_NAME="cpp_demo_worker"
fi

echo "📋 Configuration:"
echo "   Workflow: $WATTLE_WORKFLOW_NAME"
echo "   Worker:   $WATTLE_WORKER_NAME"
echo ""

# Build Rust library
echo "🦀 Building Rust library..."
cargo build --release
if [ $? -ne 0 ]; then
    echo "❌ Rust build failed"
    exit 1
fi
echo "✅ Rust library built successfully"

# Build C++ demo directly with g++
echo "🔨 Building C++ demo with g++..."

# Get the cxx generated include path
CXX_INCLUDE_DIR="$(find ../../target/release/build -name 'cxxbridge' -type d 2>/dev/null | head -1)/include"
CXX_SOURCE_DIR="$(find ../../target/release/build -name 'cxxbridge' -type d 2>/dev/null | head -1)/sources"
CXX_CPP_FILE="$(find ../../target/release/build -name '*.cc' -path '*/cxxbridge/sources/*' 2>/dev/null | head -1)"

echo "📁 CXX include dir: $CXX_INCLUDE_DIR"
echo "📁 CXX source dir: $CXX_SOURCE_DIR"
echo "📁 CXX generated cpp: $CXX_CPP_FILE"

# Compile C++ demo with generated cxx bridge code
g++ -std=c++17 \
    -I"$CXX_INCLUDE_DIR" \
    -L../../target/release \
    -lwattle_cxx \
    -pthread \
    examples/simple_demo.cpp \
    "$CXX_CPP_FILE" \
    -o simple_demo

if [ $? -ne 0 ]; then
    echo "❌ C++ build failed"
    exit 1
fi
echo "✅ C++ demo built successfully"

# Run demo
echo ""
echo "🎯 Running C++ demo..."
echo "====================="

# Set library path
export LD_LIBRARY_PATH="../../target/release:$LD_LIBRARY_PATH"

./simple_demo

echo ""
echo "🎉 Demo completed!"
echo ""
echo "🏗️  Build artifacts:"
echo "   - Rust library: ../../target/release/libwattle_cxx.so (Linux)"
echo "   - C++ executable: ./simple_demo"
echo ""
echo "📚 To use in your project:"
echo "   1. Include generated headers from cxxbridge"
echo "   2. Link: -lwattle_cxx"
echo "   3. Library path: -L/path/to/wattle-cxx/target/release"
