import zenoh
import pickle
import numpy as np
import time
def main():
    with zenoh.open(zenoh.Config()) as session:
        pub = session.declare_publisher("mujoco/action")

        def on_state(sample):
            state = pickle.loads(sample.payload.to_bytes())
            qpos, qvel = state["qpos"], state["qvel"]
            # 简单的 PD 控制器
            target = np.zeros(8)
            action = -0.1 * (qpos[:8] - target) - 0.01 * qvel[:8]
            pub.put(pickle.dumps(action))

        sub = session.declare_subscriber("mujoco/state", on_state)

        time.sleep(10)
if __name__ == "__main__":
    main()
