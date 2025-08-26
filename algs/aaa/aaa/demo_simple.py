#!/usr/bin/env python3
"""
简单演示脚本 - 快速测试三个组件
"""

import subprocess
import time
import sys
import signal

def run_demo():
    print("=== MuJoCo Zenoh 集成演示 ===")
    print("这将启动一个简单的测试演示：")
    print("1. 仿真器（无界面）")  
    print("2. 动作发布器（正弦波演示）")
    print()
    
    processes = []
    
    def cleanup(signum, frame):
        print("\n正在清理进程...")
        for p in processes:
            if p.poll() is None:
                p.terminate()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, cleanup)
    
    try:
        python_exe = "/home/hukai/main/wattle/algs/aaa/.venv/bin/python"
        
        # 启动仿真器
        print("启动仿真器...")
        sim_proc = subprocess.Popen([
            python_exe, "simulator.py", 
            "--duration", "30",
            "--prefix", "demo"
        ])
        processes.append(sim_proc)
        
        time.sleep(3)  # 等待仿真器启动
        
        # 启动动作发布器
        print("启动动作发布器（正弦波演示）...")
        action_proc = subprocess.Popen([
            python_exe, "action_publisher.py",
            "--mode", "sine",
            "--duration", "25", 
            "--prefix", "demo"
        ])
        processes.append(action_proc)
        
        print("\n演示运行中...")
        print("提示：你可以在另一个终端运行以下命令查看可视化：")
        print(f"  {python_exe} state_viewer.py --prefix demo")
        print("\n按 Ctrl+C 停止演示")
        
        # 等待进程完成
        for proc in processes:
            proc.wait()
            
        print("演示完成！")
        
    except Exception as e:
        print(f"演示出错: {e}")
        cleanup(None, None)

if __name__ == "__main__":
    run_demo()
