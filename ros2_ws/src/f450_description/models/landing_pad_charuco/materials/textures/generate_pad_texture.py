#!/usr/bin/env python3
"""
착륙 패드 텍스처 생성

실측 규격 (detect.py 기준):
  - 빨간 패드 전체   : 1.0m x 1.0m
  - 중앙 charuco 영역: 0.6m x 0.6m  (3x3 체커보드, 셀 0.2m)
  - ArUco 마커 한 변 : 0.12m        (셀의 0.6 비율)

마커 배치('+' 대칭, ID 0~3):
    [black ][marker0][black ]
    [marker1][black ][marker2]
    [black ][marker3][black ]
"""
import cv2
import cv2.aruco as aruco
import numpy as np

PAD_PX        = 2048                       # 출력 텍스처 한 변 (px) = 1.0m
PAD_M         = 1.0
CHECKER_M     = 0.6
MARKER_M      = 0.12

CHECKER_PX    = int(PAD_PX * CHECKER_M / PAD_M)      # 0.6m -> 1229px
CELL_PX       = CHECKER_PX // 3                       # 0.2m -> ~409px
MARKER_PX     = int(CELL_PX * (MARKER_M / (CHECKER_M / 3)))  # 0.12/0.2 = 0.6 비율

ORIGIN        = (PAD_PX - CHECKER_PX) // 2            # 체커보드 좌상단 오프셋(px)

RED_BGR       = (0, 0, 200)
BLACK         = (20, 20, 20)
WHITE         = (255, 255, 255)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

img = np.full((PAD_PX, PAD_PX, 3), RED_BGR, dtype=np.uint8)

pattern = [
    ['black',   'marker0', 'black'],
    ['marker1', 'black',   'marker2'],
    ['black',   'marker3', 'black'],
]
marker_ids = {'marker0': 0, 'marker1': 1, 'marker2': 2, 'marker3': 3}

for row in range(3):
    for col in range(3):
        cell = pattern[row][col]
        cx0 = ORIGIN + col * CELL_PX
        cy0 = ORIGIN + row * CELL_PX

        if cell == 'black':
            cv2.rectangle(img, (cx0, cy0), (cx0 + CELL_PX, cy0 + CELL_PX), BLACK, -1)
        else:
            # 셀 배경은 흰색, 그 위에 ArUco 마커를 중앙 배치
            cv2.rectangle(img, (cx0, cy0), (cx0 + CELL_PX, cy0 + CELL_PX), WHITE, -1)
            mid = marker_ids[cell]
            marker_img = aruco.generateImageMarker(aruco_dict, mid, MARKER_PX)
            marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
            mx0 = cx0 + (CELL_PX - MARKER_PX) // 2
            my0 = cy0 + (CELL_PX - MARKER_PX) // 2
            img[my0:my0 + MARKER_PX, mx0:mx0 + MARKER_PX] = marker_bgr

out_path = '/home/user/ros2_ws/src/f450_description/models/landing_pad_charuco/materials/textures/landing_pad.png'
cv2.imwrite(out_path, img)
print(f'saved: {out_path}')
print(f'PAD_PX={PAD_PX} CHECKER_PX={CHECKER_PX} CELL_PX={CELL_PX} MARKER_PX={MARKER_PX} ORIGIN={ORIGIN}')
