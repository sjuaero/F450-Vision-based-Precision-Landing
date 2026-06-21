#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
from enum import Enum

# ROS 2 라이브러리 추가
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # 오프셋 및 고도 전송용
from std_msgs.msg import String      # 상태(STATE) 전송용
from cv_bridge import CvBridge       # ROS2 이미지 <-> OpenCV 이미지 변환

# ── 상태 정의 ──────────────────────────────────────────
class State(Enum):
    RED_TRACK        = 1
    CHECKER_APPROACH = 2
    ARUCO_LAND       = 3

# ── 실제 크기 및 카메라 파라미터 (기존과 동일) ──────────────────
RED_PAD_W     = 1.0     
RED_PAD_H     = 1.0     
CHECKER_SIZE  = 0.6     
MARKER_LENGTH = 0.12    

camera_matrix = np.array([
    [662.62062508,   0.,         409.89163239],
    [  0.,         618.45342238, 220.36685329],
    [  0.,           0.,           1.        ]
], dtype=float)
dist_coeffs = np.array([[0.1555602, -0.84105673, -0.00829141, 0.08769156, 1.07340778]])

state = State.RED_TRACK

def estimate_dist(pixel_w, pixel_h, real_w, real_h, fx=662.62):
    vals = []
    if pixel_w > 0: vals.append((real_w * fx) / pixel_w)
    if pixel_h > 0: vals.append((real_h * fx) / pixel_h)
    return float(np.mean(vals)) if vals else None

# ── 탐지 알고리즘 함수들 (기존 함수 그대로 사용) ──────────────────
def detect_red(frame, hsv):
    mask1 = cv2.inRange(hsv, (0,   60, 60), (20,  255, 255))
    mask2 = cv2.inRange(hsv, (165, 60, 60), (180, 255, 255))
    mask  = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((7, 7), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None, None, None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 800: return None, None, None
    x, y, w, h = cv2.boundingRect(largest)
    cx, cy = x + w // 2, y + h // 2
    dist   = estimate_dist(w, h, RED_PAD_W, RED_PAD_H)
    return (cx, cy), dist, (x, y, w, h)

def detect_checker(frame, gray, red_rect):
    if red_rect is None: return None, None
    ox, oy, ow, oh = red_rect
    margin = 5
    rx1 = max(ox + margin, 0); ry1 = max(oy + margin, 0)
    rx2 = min(ox + ow - margin, gray.shape[1]); ry2 = min(oy + oh - margin, gray.shape[0])
    roi = gray[ry1:ry2, rx1:rx2]
    if roi.size == 0: return None, None
    _, thresh = cv2.threshold(roi, 80, 255, cv2.THRESH_BINARY_INV)
    k = np.ones((9, 9), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE,  k)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, k)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None, None
    best, best_score = None, -1
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 300: continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0
        score  = area * aspect
        if score > best_score: best, best_score = (x, y, w, h), score
    if best is None: return None, None
    bx, by, bw, bh = best
    ax, ay = bx + rx1, by + ry1
    cx, cy = ax + bw // 2, ay + bh // 2
    cv2.rectangle(frame, (ax, ay), (ax+bw, ay+bh), (0, 255, 255), 2)
    dist = estimate_dist(bw, bh, CHECKER_SIZE, CHECKER_SIZE)
    return (cx, cy), dist

def detect_aruco(frame, gray):
    detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_50))
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None: return None, None, 0
    aruco.drawDetectedMarkers(frame, corners, ids)
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix, dist_coeffs)
    centers, dists = [], []
    for i, (tvec, corner) in enumerate(zip(tvecs, corners)):
        pts = corner.reshape(-1, 2)
        cx  = int(np.mean(pts[:, 0])); cy = int(np.mean(pts[:, 1]))
        centers.append((cx, cy))
        dists.append(float(tvec[0][2]))
        cv2.putText(frame, f"ID:{ids[i][0]} {tvec[0][2]:.2f}m", (cx, cy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    avg_cx   = int(np.mean([c[0] for c in centers]))
    avg_cy   = int(np.mean([c[1] for c in centers]))
    avg_dist = float(np.mean(dists))
    return (avg_cx, avg_cy), avg_dist, len(ids)

# ────────────────────────────────────────────────────────
# ROS 2 노드 클래스 정의
# ────────────────────────────────────────────────────────
class LandingPadDetectorNode(Node):
    def __init__(self):
        super().__init__('landing_pad_detector')
        self.bridge = CvBridge()
        
        # 구독자(Subscriber): Gazebo 카메라 이미지 수신
        self.image_sub = self.create_subscription(
            Image,
            '/f450/camera/image_raw',
            self.image_callback,
            10
        )
        
        # 발행자(Publisher): 결과 이미지 및 드론 제어용 오프셋 데이터 송신
        self.debug_img_pub = self.create_publisher(Image, '/landing/debug_image', 10)
        self.offset_pub    = self.create_publisher(Point, '/landing/offset', 10)
        self.state_pub     = self.create_publisher(String, '/landing/state', 10)
        
        self.get_logger().info("ROS2 3단계 정밀착륙 탐지 노드가 시작되었습니다.")

    def image_callback(self, msg):
        global state
        # ROS 2 이미지 메시지를 OpenCV 이미지(BGR)로 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        # 왜곡 보정 및 해상도 중심값 계산
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        result = frame.copy()
        H, W = frame.shape[:2]
        cx0, cy0 = W // 2, H // 2

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 각 단계별 객체 탐지
        red_center,     red_dist,     red_rect     = detect_red(result, hsv)
        aruco_center,   aruco_dist,   aruco_count  = detect_aruco(result, gray)
        checker_center, checker_dist               = None, None
        if red_rect is not None:
            checker_center, checker_dist = detect_checker(result, gray, red_rect)

        # 상태 제어 로직
        if aruco_count >= 4:
            state = State.ARUCO_LAND
        elif aruco_count >= 1 or checker_center is not None:
            state = State.CHECKER_APPROACH
        elif red_center is not None:
            state = State.RED_TRACK

        COLOR = {
            State.RED_TRACK:        (0,   0, 255),
            State.CHECKER_APPROACH: (0, 255, 255),
            State.ARUCO_LAND:       (255,  0,   0),
        }
        c = COLOR[state]

        # 제어 명령으로 보낼 변수 초기화
        target_offset_x = 0.0
        target_offset_y = 0.0
        estimated_altitude = 0.0

        def put(txt, row):
            cv2.putText(result, txt, (10, 30 + row*30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)

        # 화면 오버레이 그리기 및 오프셋 계산
        if state == State.RED_TRACK and red_center:
            cx, cy = red_center
            x, y, bw, bh = red_rect
            cv2.rectangle(result, (x, y), (x+bw, y+bh), c, 2)
            cv2.circle(result, (cx, cy), 10, c, -1)
            put(f"[1] RED  dist:{red_dist:.2f}m" if red_dist else "[1] RED", 0)
            put(f"    center:({cx},{cy})  offset:({cx-cx0:+d},{cy-cy0:+d})", 1)
            target_offset_x = float(cx - cx0)
            target_offset_y = float(cy - cy0)
            estimated_altitude = float(red_dist) if red_dist else 0.0

        elif state == State.CHECKER_APPROACH:
            ref_c = checker_center or aruco_center
            ref_d = checker_dist   or aruco_dist
            label = "CHECKER" if checker_center else f"ARUCO {aruco_count}/4"
            if ref_c:
                cx, cy = ref_c
                cv2.circle(result, (cx, cy), 10, c, -1)
                put(f"[2] {label}  dist:{ref_d:.2f}m" if ref_d else f"[2] {label}", 0)
                put(f"    center:({cx},{cy})  offset:({cx-cx0:+d},{cy-cy0:+d})", 1)
                target_offset_x = float(cx - cx0)
                target_offset_y = float(cy - cy0)
                estimated_altitude = float(ref_d) if ref_d else 0.0

        elif state == State.ARUCO_LAND and aruco_center:
            cx, cy = aruco_center
            cv2.circle(result, (cx, cy), 15, c, -1)
            put(f"[3] ARUCO PRECISE  {aruco_count}/4", 0)
            put(f"    altitude:{aruco_dist:.3f}m", 1)
            put(f"    offset:({cx-cx0:+d},{cy-cy0:+d})px", 2)
            target_offset_x = float(cx - cx0)
            target_offset_y = float(cy - cy0)
            estimated_altitude = float(aruco_dist)

        # 십자선 가이드라인 그리기
        cv2.putText(result, f"STATE: {state.name}", (10, H-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        cv2.line(result, (cx0-20, cy0), (cx0+20, cy0), (255,255,255), 1)
        cv2.line(result, (cx0, cy0-20), (cx0, cy0+20), (255,255,255), 1)

        # ── ROS 2 토픽 발행 ─────────────────────────────────
        # 1. 제어 오프셋 발행 (Point 메시지: x=X오프셋, y=Y오프셋, z=추정고도)
        control_msg = Point()
        control_msg.x = target_offset_x
        control_msg.y = target_offset_y
        control_msg.z = estimated_altitude
        self.offset_pub.publish(control_msg)

        # 2. 상태 문자열 발행
        state_msg = String()
        state_msg.data = state.name
        self.state_pub.publish(state_msg)

        # 3. 디버그 그래픽이 그려진 영상을 /landing/debug_image 토픽으로 전송
        try:
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"디버그 이미지 발행 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LandingPadDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
