#!/usr/bin/env python3
"""
MuJoCo 状态查看器 - 有界面版本
接收 Zenoh 状态消息并实时显示可视化，动态解析模型信息
"""

import mujoco
from mujoco import viewer
import zenoh
import json
import numpy as np
import time
import threading
import argparse
import xml.etree.ElementTree as ET


class MuJoCoStateViewer:
    def __init__(self, model_path: str, topic_prefix: str = "mujoco"):
        self.model_path = model_path
        self.topic_prefix = topic_prefix
        
        print(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        self.model_info = self._parse_model_xml()
        
        print("Initializing Zenoh...")
        self.session = zenoh.open(zenoh.Config())
        
        self.state_topic = f"{topic_prefix}/state"
        
        self.latest_state = None
        self.state_lock = threading.Lock()
        self.running = True
        self.message_count = 0
        self.last_message_time = time.time()
        
        self.state_subscriber = self.session.declare_subscriber(
            self.state_topic,
            self._state_callback
        )
        
        print("State Viewer initialized")
        print(f"  Listening to: {self.state_topic}")
    
    def _parse_model_xml(self):
        try:
            tree = ET.parse(self.model_path)
            root = tree.getroot()
            
            info = {
                'joint_names': [],
                'actuator_names': [],
                'body_names': []
            }
            
            for joint in root.findall('.//joint'):
                name = joint.get('name')
                if name:
                    info['joint_names'].append(name)
            
            for actuator in root.findall('.//actuator/*'):
                name = actuator.get('name')
                if name:
                    info['actuator_names'].append(name)
            
            for body in root.findall('.//body'):
                name = body.get('name')
                if name:
                    info['body_names'].append(name)
            
            return info
            
        except Exception as e:
            print(f"Warning: Could not parse XML file: {e}")
            return {
                'joint_names': [f"joint_{i}" for i in range(self.model.nq)],
                'actuator_names': [f"actuator_{i}" for i in range(self.model.nu)],
                'body_names': [f"body_{i}" for i in range(self.model.nbody)]
            }
    
    def _state_callback(self, sample):
        try:
            payload_str = str(sample.payload)
            state_data = json.loads(payload_str)
            
            with self.state_lock:
                self.latest_state = state_data
                self.message_count += 1
                self.last_message_time = time.time()
                
                if 'joints' in state_data:
                    joints = state_data['joints']
                    if 'positions' in joints:
                        qpos = np.array(joints['positions'])
                        if len(qpos) == len(self.data.qpos):
                            self.data.qpos[:] = qpos
                            
                    if 'velocities' in joints:
                        qvel = np.array(joints['velocities'])
                        if len(qvel) == len(self.data.qvel):
                            self.data.qvel[:] = qvel
                
                if 'actuators' in state_data:
                    actuators = state_data['actuators']
                    if 'controls' in actuators:
                        ctrl = np.array(actuators['controls'])
                        if len(ctrl) == len(self.data.ctrl):
                            self.data.ctrl[:] = ctrl
                
                mujoco.mj_forward(self.model, self.data)
                
        except Exception as e:
            print(f"Error processing state: {e}")
    
    def run(self):
        print("Starting MuJoCo state viewer...")
        print("Press Ctrl+C to exit")
        
        try:
            with viewer.launch_passive(self.model, self.data) as v:
                while v.is_running() and self.running:
                    v.sync()
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nShutting down viewer...")
        finally:
            self.running = False
    
    def close(self):
        self.running = False
        if hasattr(self, 'session'):
            self.session.close()


def main():
    parser = argparse.ArgumentParser(description='MuJoCo State Viewer via Zenoh')
    parser.add_argument('--model', '-m',
                       default='franka_emika_panda/scene.xml',
                       help='MuJoCo model file path')
    parser.add_argument('--prefix', '-p',
                       default='mujoco',
                       help='Zenoh topic prefix')
    
    args = parser.parse_args()
    
    state_viewer = MuJoCoStateViewer(
        model_path=args.model,
        topic_prefix=args.prefix
    )
    
    try:
        state_viewer.run()
    finally:
        state_viewer.close()


if __name__ == "__main__":
    main()
