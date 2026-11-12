"""
Shows the EKF output in utils/dynamic_MIST.py, next to the raw UWB poses of the "dynamic" output controller.
Subscribes to the EKF's nav_msgs/Odometry and forwards each position to the display's UDP socket as one more "tag"
(DISPLAY_ID), in the same <ifff format as src/outputs/dynamic.py.

Needs a ROS 2 install (source /opt/ros/jazzy/setup.bash). Usage: python3 testing/ekf_to_dynamic.py
"""
import socket
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry

EKF_TOPIC = "/odometry/filtered"  # Output of the ekf_node started with testing/ekf_uwb_test.yaml
UDP_HOST = "127.0.0.1"            # Same destination as src/outputs/dynamic.py
UDP_PORT = 5005
DISPLAY_ID = 110                  # ID shown in the display's legend for the EKF track. Must differ from TAG_ID


def main():
    rclpy.init()
    node = rclpy.create_node("ekf_to_dynamic")
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def forward(msg: Odometry):
        position = msg.pose.pose.position
        sender.sendto(struct.pack("<ifff", DISPLAY_ID, position.x, position.y, position.z), (UDP_HOST, UDP_PORT))

    node.create_subscription(Odometry, EKF_TOPIC, forward, 10)
    print(f"[INFO] Forwarding {EKF_TOPIC} to {UDP_HOST}:{UDP_PORT} as tag {DISPLAY_ID}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        sender.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
