# LiveKit Integration Setup for Ubuntu 18.04 ARM64

This guide provides step-by-step instructions to set up LiveKit video streaming integration on Ubuntu 18.04 ARM64 (Jetson Nano).

## Prerequisites
- Ubuntu 18.04 ARM64
- ROS Melodic installed
- Python 2.7 (default) and Python 3.8 (side-by-side)
- USB camera publishing to `/camera/image_raw`

## Installation Steps

1. **Install Dependencies**
   ```bash
   cd ~/catkin_ws/src/elderly_bot/scripts
   sudo bash install_livekit_18.04.sh
   ```

2. **Activate Python 3.8 Virtual Environment**
   ```bash
   source /opt/livekit_venv/bin/activate
   ```

3. **Start ROS Bridge Server**
   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

4. **Start Camera Node**
   ```bash
   roslaunch elderly_bot kvs_stream.launch
   ```

5. **Run LiveKit Streamer**
   ```bash
   roslaunch elderly_bot test_livekit.launch
   ```

6. **Verify Stream**
   Open your browser and navigate to:
   ```
   http://localhost:7881
   ```

## Dependencies

Ensure the following Python packages are installed:
- `paho-mqtt`: MQTT client library for Python. Install using:
  ```bash
  sudo pip install paho-mqtt
  ```

## Troubleshooting

### Python 2.7/3.8 Coexistence Issues
- Ensure the virtual environment is activated before running Python 3.8 scripts.

### ROS Bridge Connection Failures
- Verify the ROS bridge server is running on the correct port (default: 9090).

### GStreamer Plugin Missing
- Ensure `gstreamer1.0-plugins-*` packages are installed.

### LiveKit Server Certificate Issues
- Use `ws://` for local testing instead of `wss://`.

### Memory Exhaustion on Jetson Nano
- Reduce video resolution or bitrate in `livekit_config.yaml`.

## Performance Benchmarks
- **CPU Usage**: <30%
- **Memory Usage**: <300MB

## Notes
- For production, update `server_url` in `livekit_config.yaml` to your LiveKit server.
- Use hardware encoding (`nvh264enc`) for better performance on Jetson Nano.