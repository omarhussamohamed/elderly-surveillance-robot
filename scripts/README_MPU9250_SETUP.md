# MPU9250 Node Setup Instructions

## Fix "Cannot locate node" Error

If you see:
```
ERROR: cannot launch node of type [elderly_bot/mpu9250_node.py]: Cannot locate node
```

Follow these steps **on the Jetson Nano**:

### Step 1: Make Script Executable

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x scripts/mpu9250_node.py
```

### Step 2: Rebuild the Package

```bash
cd ~/catkin_ws
catkin_make
# OR if using catkin-tools:
# catkin build
```

### Step 3: Source the Workspace

```bash
source ~/catkin_ws/devel/setup.bash
```

### Step 4: Verify Installation

```bash
rosrun elderly_bot mpu9250_node.py
```

You should see:
```
[INFO] [timestamp]: Initialized I2C bus X
```

### Step 5: Launch the Node

```bash
roslaunch elderly_bot mpu9250_jetson.launch
```

## Alternative: Run Script Directly

If the above doesn't work, you can run the script directly:

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
python mpu9250_node.py
```

But this won't work with the launch file - you need to install it properly.

## Troubleshooting

- **File not found**: Check that `src/elderly_bot/scripts/mpu9250_node.py` exists
- **Permission denied**: Run `chmod +x scripts/mpu9250_node.py` again
- **Still not found**: Make sure you ran `catkin_make` and `source devel/setup.bash`
- **Wrong Python**: Ensure shebang line is `#!/usr/bin/env python` (not python3)

