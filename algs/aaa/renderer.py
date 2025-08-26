import itertools
import time
import threading
import mujoco
import mujoco.viewer
import zenoh
import pickle

MODEL_PATH = "franka_emika_panda/scene.xml"


def main():
    with zenoh.open(zenoh.Config()) as session:
        model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        data = mujoco.MjData(model)

        # 使用锁来保护数据访问
        data_lock = threading.Lock()
        
        # 状态回调：更新 qpos/qvel
        def on_state(sample):
            state = pickle.loads(sample.payload.to_bytes())
            with data_lock:
                data.qpos[:] = state["qpos"]
                data.qvel[:] = state["qvel"]

        sub = session.declare_subscriber("mujoco/state", on_state)

        def key_callback(keycode):
            print("Key pressed:", keycode)

        with mujoco.viewer.launch_passive(
            model, data, key_callback=key_callback
        ) as viewer:
            
            while viewer.is_running():
                # 在viewer同步之前获取数据锁并更新物理状态
                with data_lock:
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                # 添加小延迟以减少CPU占用
                time.sleep(0.01)  # 1ms delay


if __name__ == "__main__":
    main()
