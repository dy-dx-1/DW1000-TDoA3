"""
ROS 2 output: publishes every solved pose as a geometry_msgs/PoseWithCovarianceStamped (e.g. for robot_localization's EKF).
Only imported when "ros2" is in OUTPUTS. rclpy comes from the ROS 2 install (source /opt/ros/jazzy/setup.bash),
not from requirements.txt.
"""
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped

from config import ROS2
from pose import CycleUpdate, TDOA3_Pose
from outputs.base import OutputController

MIN_VARIANCE_M2 = 0.05**2  # Floor on the x/y(/z) variance, so the EKF never trusts UWB more than ~5 cm std, even when the
                           # solver reports less (e.g. per anchor biases are not part of the solver's covariance)
UNUSED_VARIANCE = 1e3      # Variance of the axes we don't measure (roll, pitch, yaw, and z in 2D). Large = unknown,
                           # 0 would mean perfectly known
QUEUE_DEPTH = 10           # Publisher history depth (ROS default)


def build_covariance_array(pose:TDOA3_Pose)->list[float]:
    """
    Builds the 36 float row-major covariance of a PoseWithCovarianceStamped (order x, y, z, rot_x, rot_y, rot_z) in m².
    - Solved block (x,y in 2D, x,y,z in 3D): the solver's covariance, with its diagonal floored at MIN_VARIANCE_M2.
      Raising the diagonal keeps the matrix positive definite, so no other correction is needed.
    - Other axes: UNUSED_VARIANCE on the diagonal, no cross terms.
    """
    n = 2 if pose.mode_2d else 3
    covariance = [0.0] * 36
    for i in range(n):
        for j in range(n):
            covariance[6*i + j] = float(pose.covariance[i, j])
        covariance[7*i] = max(covariance[7*i], MIN_VARIANCE_M2)
    for i in range(n, 6):
        covariance[7*i] = UNUSED_VARIANCE
    return covariance


class Ros2OutputController(OutputController):
    """
    Publishes every solved pose on ROS2["topic"], in ROS2["frame_id"], stamped with the cycle's solve time.
    Positions are in meters (ROS convention), regardless of UNITS and DECIMALS. Orientation is not measured.
    Cycles without a pose publish nothing; a message is printed when publishing stops and when it resumes.
    """
    def __init__(self, tag_id):
        self.tag_id = tag_id
        # Without rclpy's own signal handlers, Ctrl-C stays a plain KeyboardInterrupt handled by the tag's context manager
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node(ROS2["node_name"])
        self._publisher = self._node.create_publisher(PoseWithCovarianceStamped, ROS2["topic"], QUEUE_DEPTH)
        self._paused = False   # Whether the last cycle(s) had no pose to publish
        self._skipped = 0      # Number of cycles without a pose since publishing paused
        print(f"[INFO] ROS 2: publishing PoseWithCovarianceStamped on {ROS2['topic']} (frame '{ROS2['frame_id']}')")

    def publish(self, update: CycleUpdate) -> None:
        pose = update.pose
        if pose is None:
            if not self._paused:
                print("[WARNING] ROS 2: no pose to publish, publishing paused until the next solved pose")
                self._paused = True
                self._skipped = 0
            self._skipped += 1
            return
        if self._paused:
            print(f"[INFO] ROS 2: publishing resumed after {self._skipped} cycle(s) without a pose")
            self._paused = False

        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = update.stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = update.stamp_ns % 1_000_000_000
        msg.header.frame_id = ROS2["frame_id"]
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = pose.z
        msg.pose.pose.orientation.w = 1.0 # Identity quaternion, orientation must still be a valid quaternion
        msg.pose.covariance = build_covariance_array(pose)
        self._publisher.publish(msg)

    def close(self) -> None:
        try:
            self._node.destroy_node()
        finally:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
