#!/usr/bin/env python3
"""
MuJoCo 动作发布器
向 MuJoCo 仿真器发送动作命令，动态解析模型信息
"""

import zenoh
import json
import time
import numpy as np
import argparse
import xml.etree.ElementTree as ET


class ActionPublisher:
    def __init__(self, model_path: str, topic_prefix: str = "mujoco"):
        self.model_path = model_path
        self.topic_prefix = topic_prefix
        
        print(f"Parsing MuJoCo model: {model_path}")
        self.model_info = self._parse_model_xml()
        
        print("Initializing Zenoh...")
        self.session = zenoh.open(zenoh.Config())
        
        self.action_topic = f"{topic_prefix}/action"
        self.action_publisher = self.session.declare_publisher(self.action_topic)
        
        print("Action Publisher initialized")
        print(f"  Publishing to: {self.action_topic}")
        print(f"  Joints: {len(self.model_info['joint_names'])}")
        print(f"  Actuators: {len(self.model_info['actuator_names'])}")
    
    def _parse_model_xml(self):
        try:
            tree = ET.parse(self.model_path)
            root = tree.getroot()
            
            info = {
                'joint_names': [],
                'actuator_names': [],
                'joint_ranges': {},
                'actuator_ranges': {}
            }
            
            for joint in root.findall('.//joint'):
                name = joint.get('name')
                if name:
                    info['joint_names'].append(name)
                    range_attr = joint.get('range')
                    if range_attr:
                        range_vals = [float(x) for x in range_attr.split()]
                        info['joint_ranges'][name] = range_vals
            
            for actuator in root.findall('.//actuator/*'):
                name = actuator.get('name')
                if name:
                    info['actuator_names'].append(name)
                    ctrlrange = actuator.get('ctrlrange')
                    if ctrlrange:
                        range_vals = [float(x) for x in ctrlrange.split()]
                        info['actuator_ranges'][name] = range_vals
            
            return info
            
        except Exception as e:
            print(f"Warning: Could not parse XML file: {e}")
            return {
                'joint_names': [f"joint_{i}" for i in range(7)],
                'actuator_names': [f"actuator_{i}" for i in range(7)],
                'joint_ranges': {},
                'actuator_ranges': {}
            }
    
    def send_control_action(self, ctrl_values):
        action = {
            "type": "ctrl",
            "values": ctrl_values,
            "timestamp": time.time()
        }
        
        action_json = json.dumps(action)
        self.action_publisher.put(action_json)
        print(f"Sent control: {[f'{v:.3f}' for v in ctrl_values[:5]]}")
    
    def send_position_action(self, qpos_values):
        action = {
            "type": "qpos", 
            "values": qpos_values,
            "timestamp": time.time()
        }
        
        action_json = json.dumps(action)
        self.action_publisher.put(action_json)
        print(f"Sent position: {[f'{v:.3f}' for v in qpos_values[:5]]}")
    
    def run_demo_sine_wave(self, duration: float = 30.0):
        print(f"Running sine wave demo for {duration} seconds...")
        
        start_time = time.time()
        num_actuators = max(len(self.model_info['actuator_names']), 7)
        
        try:
            while (time.time() - start_time) < duration:
                t = time.time() - start_time
                
                ctrl = []
                for i in range(num_actuators):
                    frequency = 0.5 + i * 0.1
                    value = 0.3 * np.sin(frequency * t)
                    ctrl.append(value)
                
                self.send_control_action(ctrl)
                time.sleep(0.05)  # 20 Hz
                
        except KeyboardInterrupt:
            print("Demo stopped by user.")
    
    def run_interactive(self):
        print("Interactive Action Publisher")
        print("Commands:")
        print("  ctrl <val1> <val2> ... - Send control inputs")
        print("  pos <val1> <val2> ...  - Send position targets")
        print("  demo_sine [duration]   - Run sine wave demo")
        print("  zero                   - Send zero controls")
        print("  info                   - Show model info")
        print("  quit                   - Exit")
        
        try:
            while True:
                command = input(f"\n[{self.topic_prefix}]> ").strip()
                
                if command == "quit":
                    break
                elif command == "zero":
                    num_actuators = max(len(self.model_info['actuator_names']), 7)
                    self.send_control_action([0.0] * num_actuators)
                    
                elif command == "info":
                    print("\n=== Model Information ===")
                    print(f"Joints: {self.model_info['joint_names']}")
                    print(f"Actuators: {self.model_info['actuator_names']}")
                    print(f"Joint ranges: {self.model_info['joint_ranges']}")
                    print(f"Actuator ranges: {self.model_info['actuator_ranges']}")
                        
                elif command.startswith("ctrl"):
                    try:
                        values = [float(x) for x in command.split()[1:]]
                        self.send_control_action(values)
                    except (ValueError, IndexError):
                        print("Usage: ctrl <val1> <val2> ...")
                        
                elif command.startswith("pos"):
                    try:
                        values = [float(x) for x in command.split()[1:]]
                        self.send_position_action(values)
                    except (ValueError, IndexError):
                        print("Usage: pos <val1> <val2> ...")
                        
                elif command.startswith("demo_sine"):
                    try:
                        duration = float(command.split()[1]) if len(command.split()) > 1 else 30.0
                        self.run_demo_sine_wave(duration)
                    except ValueError:
                        print("Usage: demo_sine [duration]")
                        
                else:
                    print("Unknown command. Type 'quit' to exit.")
                    
        except KeyboardInterrupt:
            print("\nExiting...")
    
    def close(self):
        if hasattr(self, 'session'):
            self.session.close()


def main():
    parser = argparse.ArgumentParser(description='MuJoCo Action Publisher via Zenoh')
    parser.add_argument('--model', '-m',
                       default='franka_emika_panda/scene.xml',
                       help='MuJoCo model file path')
    parser.add_argument('--prefix', '-p',
                       default='mujoco',
                       help='Zenoh topic prefix')
    parser.add_argument('--mode',
                       choices=['interactive', 'sine'],
                       default='interactive',
                       help='Operation mode')
    parser.add_argument('--duration', '-d',
                       type=float,
                       default=30.0,
                       help='Demo duration in seconds')
    
    args = parser.parse_args()
    
    publisher = ActionPublisher(
        model_path=args.model,
        topic_prefix=args.prefix
    )
    
    try:
        if args.mode == 'interactive':
            publisher.run_interactive()
        elif args.mode == 'sine':
            publisher.run_demo_sine_wave(args.duration)
            
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
