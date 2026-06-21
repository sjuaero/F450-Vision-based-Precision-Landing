#!/usr/bin/env python3
"""
ArUco Marker Detector Node.

Detects ArUco markers from drone's downward-facing camera and publishes
the marker pose for precision landing. Uses OpenCV's ArUco module.

Camera frame convention: x=right, y=down, z=forward (optical frame)
Published pose is in camera_optical_frame.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, Int32
import tf2_ros

try:
    import cv2
    from cv_bridge import CvBridge
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_id', 42)
        self.declare_parameter('marker_size_m', 0.5)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('camera_topic', '/drone/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/drone/camera/camera_info')
        self.declare_parameter('publish_debug_image', True)
        # f450 SDF 하드코딩 intrinsics (camera_info 토픽 없을 때 fallback)
        self.declare_parameter('fallback_fx', 662.62)
        self.declare_parameter('fallback_fy', 618.45)
        self.declare_parameter('fallback_cx', 409.89)
        self.declare_parameter('fallback_cy', 220.37)
        self.declare_parameter('fallback_frame_id', 'camera_optical_frame')

        self.target_marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size_m').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # Camera intrinsics — fallback to f450 SDF values if camera_info not received
        fx = self.get_parameter('fallback_fx').value
        fy = self.get_parameter('fallback_fy').value
        cx = self.get_parameter('fallback_cx').value
        cy = self.get_parameter('fallback_cy').value
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros(5, dtype=np.float64)
        self.camera_frame_id = self.get_parameter('fallback_frame_id').value
        self.camera_info_received = True  # ready immediately with fallback

        if CV2_AVAILABLE:
            self.bridge = CvBridge()
            # Setup ArUco detector
            aruco_dict_id = getattr(cv2.aruco, aruco_dict_name, cv2.aruco.DICT_4X4_50)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            self.aruco_params = cv2.aruco.DetectorParameters()
            # Tune detection parameters for reliable detection
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.minMarkerPerimeterRate = 0.03
            self.aruco_params.maxMarkerPerimeterRate = 4.0
            self.aruco_params.polygonalApproxAccuracyRate = 0.05
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info(
                f'ArUco detector initialized: dict={aruco_dict_name}, '
                f'target_id={self.target_marker_id}, size={self.marker_size}m'
            )
        else:
            self.get_logger().error('opencv-contrib-python not installed! ArUco detection disabled.')

        # TF broadcaster for marker transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.detected_pub = self.create_publisher(Bool, '/aruco/detection_status', 10)
        self.marker_id_pub = self.create_publisher(Int32, '/aruco/marker_id', 10)
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(Image, '/aruco/detection_image', 10)

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1)
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # Detection state
        self.consecutive_detections = 0
        self.consecutive_losses = 0
        self.is_detected = False

        self.get_logger().info('ArUco Detector Node started.')

    def camera_info_callback(self, msg: CameraInfo):
        K = np.array(msg.k).reshape(3, 3)
        if K[0, 0] > 0:  # valid intrinsics
            self.camera_matrix = K
            self.dist_coeffs = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            if msg.header.frame_id:
                self.camera_frame_id = msg.header.frame_id
            self.get_logger().info(
                f'Camera info updated from topic. Frame: {self.camera_frame_id}, '
                f'fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}',
                once=True,
            )

    def image_callback(self, msg: Image):
        if not CV2_AVAILABLE or not self.camera_info_received:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        detected = Bool()
        if ids is not None:
            # Look for our target marker
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.target_marker_id:
                    # Estimate pose using solvePnP
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_size,
                        self.camera_matrix, self.dist_coeffs
                    )
                    rvec = rvec[0][0]
                    tvec = tvec[0][0]

                    # Build PoseStamped in camera frame
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.header.frame_id = self.camera_frame_id
                    pose_msg.pose.position.x = float(tvec[0])
                    pose_msg.pose.position.y = float(tvec[1])
                    pose_msg.pose.position.z = float(tvec[2])

                    # Convert rotation vector to quaternion
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    q = self._rotation_matrix_to_quaternion(rot_mat)
                    pose_msg.pose.orientation.x = q[0]
                    pose_msg.pose.orientation.y = q[1]
                    pose_msg.pose.orientation.z = q[2]
                    pose_msg.pose.orientation.w = q[3]

                    self.pose_pub.publish(pose_msg)

                    # Publish marker ID
                    id_msg = Int32()
                    id_msg.data = int(marker_id)
                    self.marker_id_pub.publish(id_msg)

                    # Broadcast TF
                    self._broadcast_marker_tf(msg.header, tvec, rot_mat)

                    detected.data = True
                    self.consecutive_detections += 1
                    self.consecutive_losses = 0

                    if not self.is_detected and self.consecutive_detections >= 3:
                        self.is_detected = True
                        self.get_logger().info(
                            f'ArUco marker {self.target_marker_id} acquired! '
                            f'Distance: {tvec[2]:.2f}m, '
                            f'Offset: ({tvec[0]:.2f}, {tvec[1]:.2f})m'
                        )
                    break
        else:
            detected.data = False
            self.consecutive_losses += 1
            self.consecutive_detections = 0
            if self.is_detected and self.consecutive_losses >= 10:
                self.is_detected = False
                self.get_logger().warn('ArUco marker lost!')

        self.detected_pub.publish(detected)

        # Publish debug visualization
        if self.publish_debug and hasattr(self, 'debug_image_pub'):
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])

    def _broadcast_marker_tf(self, header, tvec, rot_mat):
        t = TransformStamped()
        t.header = header
        t.child_frame_id = 'aruco_marker'
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        q = self._rotation_matrix_to_quaternion(rot_mat)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
