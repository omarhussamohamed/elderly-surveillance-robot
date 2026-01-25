import cv2
import numpy as np
import roslibpy
from livekit import LiveKitRoom
import yaml
import os
import signal
import sys

def load_config(config_path):
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

def signal_handler(sig, frame):
    print("Shutting down LiveKit streamer...")
    sys.exit(0)

def main():
    # Load configuration
    config_path = os.path.join(os.path.dirname(__file__), '../config/livekit_config.yaml')
    config = load_config(config_path)

    # ROS Bridge connection
    ros_client = roslibpy.Ros(host=config['livekit']['ros_bridge']['host'],
                              port=config['livekit']['ros_bridge']['port'])
    ros_client.run()
    print("Connected to ROS bridge.")

    # Subscribe to /camera/image_raw
    image_topic = roslibpy.Topic(ros_client, config['livekit']['ros_bridge']['topic'], 'sensor_msgs/Image')

    # LiveKit connection
    room = LiveKitRoom(
        url=config['livekit']['server_url'],
        api_key=config['livekit']['api_key'],
        api_secret=config['livekit']['api_secret']
    )
    room.connect()
    print("Connected to LiveKit server.")

    def on_image_message(message):
        # Decode ROS image (BGR8)
        height = message['height']
        width = message['width']
        data = np.frombuffer(message['data'], dtype=np.uint8).reshape((height, width, 3))

        # Resize image
        resized = cv2.resize(data, (config['livekit']['video']['width'], config['livekit']['video']['height']))

        # Encode to H.264
        _, encoded = cv2.imencode('.h264', resized)

        # Stream to LiveKit
        room.publish_video(encoded.tobytes())

    image_topic.subscribe(on_image_message)

    # Graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    print("LiveKit streamer running. Press Ctrl+C to stop.")
    signal.pause()

if __name__ == '__main__':
    main()