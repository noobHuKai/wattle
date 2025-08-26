# MuJoCo Zenoh Integration - AI Coding Agent Instructions

## Project Architecture

This is a **distributed robotics simulation system** that integrates MuJoCo physics simulation with Zenoh communication middleware. The system uses a **3-component pub/sub architecture**:

- `simulator.py`: Headless MuJoCo simulator that publishes state @ 100Hz and subscribes to actions
- `state_viewer.py`: Visual interface that subscribes to state and renders real-time MuJoCo visualization  
- `action_publisher.py`: Control interface that publishes actions (interactive CLI or demo modes)

**Key principle**: All components dynamically parse the same MuJoCo XML model file to extract joint names, actuator ranges, and body hierarchies - never hardcode these.

## Critical Communication Patterns

### Zenoh Topics (configurable prefix, default: `mujoco`)
- `{prefix}/action`: JSON action commands → simulator
- `{prefix}/state`: JSON state data from simulator → viewers (100Hz)

### Message Formats
**Actions** support both named and indexed control:
```python
# Named control (preferred)
{"joints": {"positions": {"joint1": 0.5, "joint2": -1.0}}}
{"actuators": {"controls": {"actuator1": 0.1}}}

# Indexed control (fallback) 
{"joints": {"positions": [0.5, -1.0, 0.0, ...]}}
```

**State** includes full simulation context:
```python
{
    "timestamp": float,
    "sim_time": float, 
    "joints": {"names": [...], "positions": [...], "velocities": [...], "ranges": {...}},
    "actuators": {"names": [...], "controls": [...], "ranges": {...}},
    "bodies": {"names": [...], "positions": [...], "orientations": [...]}
}
```

## XML Dynamic Parsing Pattern

**Critical**: Each component parses the XML model file (`franka_emika_panda/scene.xml` by default) to extract:
- Joint names from `<joint name="...">` elements
- Actuator names/ranges from `<actuator><*/name="..." ctrlrange="...">` elements  
- Body hierarchy from `<body name="...">` elements

Example pattern in each component:
```python
def _parse_model_xml(self) -> Dict[str, Any]:
    tree = ET.parse(self.model_path)
    root = tree.getroot()
    # Extract joint names: root.findall('.//joint')
    # Extract actuator info: root.findall('.//actuator/*') 
    # Fall back to indexed names if parsing fails
```

## Essential Development Workflows

### Testing/Running the System
```bash
# Start components in separate terminals (order matters)
python simulator.py --prefix test --rate 60
python state_viewer.py --prefix test  
python action_publisher.py --prefix test --mode interactive

# Quick validation
python -c "from simulator import MuJoCoSimulator; s=MuJoCoSimulator('franka_emika_panda/scene.xml'); print('OK'); s.close()"
```

### Interactive Control Commands (action_publisher.py)
```
ctrl 0.1 0.0 -0.2    # Send actuator controls
pos 0.0 0.0 -1.57    # Send joint positions  
info                 # Show parsed model info
zero                 # Zero all controls
demo_sine 30         # Run 30s sine wave demo
```

### Model File Structure
- Main models: `franka_emika_panda/{scene.xml,panda.xml,mjx_*.xml}`
- Assets: `franka_emika_panda/assets/` (meshes referenced by XML)
- Different variants: `panda.xml` (with gripper), `panda_nohand.xml` (arm only), `mjx_*.xml` (MJX-optimized)

## Project-Specific Conventions

### Error Handling
- XML parsing failures fall back to indexed naming (`joint_0`, `actuator_0`, etc.)
- Missing model files cause immediate startup failure (by design)
- Zenoh connection uses default config - no custom discovery

### State Management  
- Simulator runs headless with configurable publish rate (default 100Hz)
- State viewer synchronizes MuJoCo visualization using `mujoco.mj_forward()`
- Thread-safe state updates use locks in state_viewer.py

### Configuration Pattern
All components accept:
- `--model/-m`: XML model file path
- `--prefix/-p`: Zenoh topic namespace prefix  
- Component-specific: `--duration`, `--rate`, `--mode`

## Integration Points

- **MuJoCo**: Physics simulation core, loads XML models via `mujoco.MjModel.from_xml_path()`
- **Zenoh**: Pub/sub communication, initialize with `zenoh.open(zenoh.Config())`
- **Threading**: State publishing runs in daemon threads, UI components handle Ctrl+C gracefully

## Common Pitfalls

1. **Topic prefix mismatch**: All components must use identical `--prefix` values
2. **Model file paths**: Relative paths resolved from working directory, not script location
3. **XML parsing**: Joint/actuator counts from MuJoCo model may differ from XML parsing results
4. **Message ordering**: Actions applied immediately, no queuing/buffering system

When extending: Follow the dynamic XML parsing pattern, use structured JSON messages, and maintain the pub/sub separation of concerns.
