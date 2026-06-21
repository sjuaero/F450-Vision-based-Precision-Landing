# agri-mission-ws

F450 드론 + R1/HBX 로버 농업 미션 ROS2 워크스페이스. PX4 SITL + Gazebo + ArUco 기반 정밀 착륙/방역 미션.

## 구조

```
ros2_ws/src/
  f450_description/   # F450 드론 URDF, 메시, launch, 착륙 월드/모델
  agri_mission/        # 미션 패키지 모음
    agri_aruco_detector/    # ArUco 인식 + 착륙패드 감지(landing_pad_detector_node)
    agri_drone_controller/  # 드론 제어(MAVROS), 방역 패턴 실행
    agri_rover_controller/  # 로버 제어(Nav2)
    agri_mission_manager/   # 미션 상태 관리
    agri_llm_interface/     # LLM 기반 미션 명령 해석
    agri_bringup/            # 통합 launch, rviz, config
    agri_gazebo/              # Gazebo 월드/모델
    agri_interfaces/          # msg/srv/action 정의

scripts/                # mavsdk 기반 미션 스크립트 (drone_mission.py 등)

px4_overlay/            # PX4-Autopilot(업스트림) 위에 추가/수정해야 하는 파일들
  run_*.sh, multi_sitl.sh        # PX4-Autopilot 루트에 복사
  airframes/                      # ROMFS/px4fmu_common/init.d-posix/airframes/ 에 복사
  patches/airframes_and_mavlink.patch  # airframes/CMakeLists.txt, px4-rc.mavlink 에 적용
  gz_models_worlds/              # Tools/simulation/gz (PX4-gazebo-models 서브모듈)의 models/, worlds/ 에 복사
                                  # r1_rover_overlay/ 는 기존 r1_rover 모델에 추가/교체할 파일
```

## 환경 요구사항

- Ubuntu 24.04 (WSL2)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 빌드 완료 (`~/PX4-Autopilot`)
- ROS2 Jazzy, Gazebo Harmonic
- Python 3 + mavsdk, opencv-contrib-python(aruco)

## 설치

1. `px4_overlay/` 내용을 안내대로 `~/PX4-Autopilot` 트리에 복사하고 `patches/airframes_and_mavlink.patch` 적용
2. `ros2_ws/src/` 를 자신의 ROS2 워크스페이스 `src/` 에 복사 후 `colcon build`
3. `scripts/` 를 `~/scripts` 로 복사

## 실행 (터미널 4개)

```bash
# 1. PX4 SITL + Gazebo (착륙 미션용)
~/PX4-Autopilot/run_drone_landing.sh

# 2. uXRCE-DDS 브리지
MicroXRCEAgent udp4 -p 8888

# 3. 카메라 브리지 + 착륙패드 인식
ros2 launch f450_description f450_px4_bridge.launch.py
# 자동으로 뜨는 rqt_image_view 창에서 토픽을 /landing/debug_image 로 선택

# 4. 착륙 미션 제어
python3 ~/scripts/drone_mission.py
```

지그재그 비행 경로 설계 시 드론 반경의 1.5배를 여유로 둘 것.
