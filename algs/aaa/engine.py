import time
import mujoco
import zenoh
import pickle
import numpy as np

MODEL_PATH = "franka_emika_panda/scene.xml"

def main():
    with zenoh.open(zenoh.Config()) as session:
        pub = session.declare_publisher("mujoco/state")
        sub = session.declare_subscriber("mujoco/action", lambda sample: on_action(sample))

        model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        data = mujoco.MjData(model)

        # 2. 获取执行器数量
        num_actuators = model.nu
        print(f"Number of actuators: {num_actuators}")

        # 3. 创建初始控制信号数组
        ctrl = np.zeros(num_actuators)


        action = None
        def on_action(sample):
            nonlocal action
            action = pickle.loads(sample.payload.to_bytes())

        while True:
            if action is not None:
                print(f"Received action: {action}")
                data.ctrl[:] = action
            mujoco.mj_step(model, data)
            state = {"qpos": data.qpos.copy(), "qvel": data.qvel.copy()}
            pub.put(pickle.dumps(state))
            time.sleep(0.01)  # 100Hz

if __name__ == "__main__":
    main()
