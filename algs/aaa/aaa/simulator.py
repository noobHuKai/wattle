#!/usr/bin/env python3
"""
MuJoCo 仿真器 - 无界面版本
运行 MuJoCo 仿真，监听 Zenoh 动作消息，发布状态消息
动态解析 XML 模型文件获取关节和执行器信息
"""

import mujoco
import zenoh
import json
import numpy as np
import time
import threading
import argparse
import xml.etree.ElementTree as ET
from typing import Dict, Any, List


class MuJoCoSimulator:
    def __init__(self, model_path: str, topic_prefix: str = "mujoco"):
        """
        初始化 MuJoCo 仿真器
        
        Args:
            model_path: MuJoCo 模型文件路径
            topic_prefix: Zenoh topic 统一前缀
        """
        self.model_path = model_path
        self.topic_prefix = topic_prefix
        
        # 加载 MuJoCo 模型
        print(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 解析 XML 获取模型信息
        self.model_info = self._parse_model_xml()
        
        # 初始化 Zenoh
        print("Initializing Zenoh...")
        self.session = zenoh.open(zenoh.Config())
        
        # 定义 topic 路径
        self.action_topic = f"{topic_prefix}/action"
        self.state_topic = f"{topic_prefix}/state"
        
        # 创建发布器和订阅器
        self.state_publisher = self.session.declare_publisher(self.state_topic)
        self.action_subscriber = self.session.declare_subscriber(
            self.action_topic, 
            self._action_callback
        )
        
        # 状态发布控制
        self.running = True
        self.state_publish_rate = 100  # Hz
        
        print("MuJoCo Simulator initialized")
        print(f"  Model: {model_path}")
        print(f"  DOF: {self.model.nq} (position), {self.model.nv} (velocity)")
        print(f"  Actuators: {self.model.nu}")
        print(f"  Bodies: {self.model.nbody}")
        print(f"  Action topic: {self.action_topic}")
        print(f"  State topic: {self.state_topic}")
        print(f"  Joint names: {self.model_info['joint_names']}")
        print(f"  Actuator names: {self.model_info['actuator_names']}")
    
    def _parse_model_xml(self) -> Dict[str, Any]:
        """解析 XML 文件获取模型信息"""
        try:
            tree = ET.parse(self.model_path)
            root = tree.getroot()
            
            info = {
                'joint_names': [],
                'actuator_names': [],
                'body_names': [],
                'sensor_names': [],
                'joint_ranges': {},
                'actuator_ranges': {}
            }
            
            # 解析关节信息
            for joint in root.findall('.//joint'):
                name = joint.get('name')
                if name:
                    info['joint_names'].append(name)
                    # 获取关节范围
                    range_attr = joint.get('range')
                    if range_attr:
                        range_vals = [float(x) for x in range_attr.split()]
                        info['joint_ranges'][name] = range_vals
            
            # 解析执行器信息
            for actuator in root.findall('.//actuator/*'):  # motor, position, velocity 等
                name = actuator.get('name')
                if name:
                    info['actuator_names'].append(name)
                    # 获取执行器范围
                    ctrlrange = actuator.get('ctrlrange')
                    if ctrlrange:
                        range_vals = [float(x) for x in ctrlrange.split()]
                        info['actuator_ranges'][name] = range_vals
            
            # 解析身体信息
            for body in root.findall('.//body'):
                name = body.get('name')
                if name:
                    info['body_names'].append(name)
            
            # 解析传感器信息
            for sensor in root.findall('.//sensor/*'):
                name = sensor.get('name')
                if name:
                    info['sensor_names'].append(name)
            
            return info
            
        except Exception as e:
            print(f"Warning: Could not parse XML file: {e}")
            return {
                'joint_names': [f"joint_{i}" for i in range(self.model.nq)],
                'actuator_names': [f"actuator_{i}" for i in range(self.model.nu)],
                'body_names': [f"body_{i}" for i in range(self.model.nbody)],
                'sensor_names': [],
                'joint_ranges': {},
                'actuator_ranges': {}
            }
    
    def _action_callback(self, sample):
        """处理接收到的动作消息"""
        try:
            payload_str = str(sample.payload)
            action_data = json.loads(payload_str)
            
            action_type = action_data.get('type')
            values = np.array(action_data.get('values', []))
            
            if action_type == 'ctrl':
                # 控制输入
                if len(values) <= self.model.nu:
                    self.data.ctrl[:len(values)] = values
                    print(f"Applied control: {values[:min(3, len(values))]}...")
                    
            elif action_type == 'qpos':
                # 关节位置
                if len(values) <= self.model.nq:
                    self.data.qpos[:len(values)] = values
                    print(f"Set joint positions: {values[:min(3, len(values))]}...")
                    
            elif action_type == 'qvel':
                # 关节速度
                if len(values) <= self.model.nv:
                    self.data.qvel[:len(values)] = values
                    print(f"Set joint velocities: {values[:min(3, len(values))]}...")
                    
            else:
                print(f"Unknown action type: {action_type}")
                
        except Exception as e:
            print(f"Error processing action: {e}")
    
    def _get_state(self) -> Dict[str, Any]:
        """获取当前仿真状态"""
        state = {
            'timestamp': time.time(),
            'sim_time': self.data.time,
            
            # 关节信息
            'joints': {
                'names': self.model_info['joint_names'][:self.model.nq],
                'positions': self.data.qpos.tolist(),
                'velocities': self.data.qvel.tolist(),
                'accelerations': self.data.qacc.tolist(),
                'ranges': self.model_info['joint_ranges']
            },
            
            # 执行器信息
            'actuators': {
                'names': self.model_info['actuator_names'][:self.model.nu],
                'controls': self.data.ctrl.tolist(),
                'ranges': self.model_info['actuator_ranges']
            },
            
            # 身体信息
            'bodies': {
                'names': self.model_info['body_names'][:self.model.nbody],
                'positions': self.data.xpos.tolist(),
                'orientations': self.data.xquat.tolist(),
                'rotation_matrices': self.data.xmat.tolist()
            },
            
            # 传感器信息
            'sensors': {
                'names': self.model_info['sensor_names'],
                'data': self.data.sensordata.tolist() if self.data.sensordata is not None else []
            },
            
            # 模型元信息
            'model_info': {
                'nq': self.model.nq,
                'nv': self.model.nv,
                'nu': self.model.nu,
                'nbody': self.model.nbody,
                'timestep': self.model.opt.timestep
            }
        }
        return state
    
    def _state_publish_loop(self):
        """状态发布循环"""
        dt = 1.0 / self.state_publish_rate
        
        while self.running:
            try:
                state = self._get_state()
                state_json = json.dumps(state)
                self.state_publisher.put(state_json)
                time.sleep(dt)
            except Exception as e:
                print(f"Error publishing state: {e}")
                time.sleep(dt)
    
    def run(self, duration: float = None):
        """运行仿真（无界面模式）"""
        print("Starting headless simulation...")
        print("Press Ctrl+C to stop")
        
        # 启动状态发布线程
        state_thread = threading.Thread(target=self._state_publish_loop, daemon=True)
        state_thread.start()
        
        start_time = time.time()
        step_count = 0
        
        try:
            while self.running:
                # 执行仿真步骤
                mujoco.mj_step(self.model, self.data)
                step_count += 1
                
                # 检查运行时长
                if duration and (time.time() - start_time) > duration:
                    print(f"Simulation completed after {duration} seconds")
                    break
                
                # 定期打印统计信息
                if step_count % 10000 == 0:  # 每10000步打印一次
                    elapsed = time.time() - start_time
                    rate = step_count / elapsed if elapsed > 0 else 0
                    print(f"Steps: {step_count}, Time: {elapsed:.1f}s, Rate: {rate:.0f} Hz")
                
                # 控制仿真频率（实时）
                time.sleep(max(0, self.model.opt.timestep - 0.0001))  # 稍微补偿计算时间
                
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        finally:
            self.running = False
    
    def close(self):
        """关闭仿真器"""
        print("Shutting down simulator...")
        self.running = False
        if hasattr(self, 'session'):
            self.session.close()


def main():
    parser = argparse.ArgumentParser(description='MuJoCo Headless Simulator with Zenoh')
    parser.add_argument('--model', '-m',
                       default='franka_emika_panda/scene.xml',
                       help='MuJoCo model file path')
    parser.add_argument('--prefix', '-p',
                       default='mujoco',
                       help='Zenoh topic prefix')
    parser.add_argument('--duration', '-d',
                       type=float,
                       help='Simulation duration in seconds')
    parser.add_argument('--rate', '-r',
                       type=int,
                       default=100,
                       help='State publish rate in Hz')
    
    args = parser.parse_args()
    
    # 创建仿真器
    simulator = MuJoCoSimulator(
        model_path=args.model,
        topic_prefix=args.prefix
    )
    simulator.state_publish_rate = args.rate
    
    try:
        simulator.run(duration=args.duration)
    finally:
        simulator.close()


if __name__ == "__main__":
    main()
