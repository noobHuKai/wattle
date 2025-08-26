import zenoh
import pickle
import numpy as np

def main():
    with zenoh.open(zenoh.Config()) as session:
        pub = session.declare_publisher("mujoco/action")

        def on_state(sample):
            state = pickle.loads(sample.payload.to_bytes())
            qpos, qvel = state["qpos"], state["qvel"]
            # 简单的 PD 控制器
            target = np.zeros_like(qpos)
            action = -0.1 * (qpos - target) - 0.01 * qvel
            pub.put(pickle.dumps(action))

        sub = session.declare_subscriber("mujoco/state", on_state)
        input("Controller running... Press Enter to quit\n")

if __name__ == "__main__":
    main()
