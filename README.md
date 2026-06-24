# 🚗 ACCA_2025: Autonomous Driving Competition Project

**자율주행자동차 경진대회 ACCA팀**의 ERP42 기반 자율주행 시스템입니다.  
LiDAR, GPS, IMU, 카메라를 융합하여 다양한 미션(콘 주행, 주차, 장애물 회피, 신호등 정지선 등)을 수행합니다.

---

## 🏗️ System Architecture

본 프로젝트는 **센서 → 인식 → 측위 → 경로계획 → 제어** 의 자율주행 전체 파이프라인을 ROS2(Humble)로 구현합니다.

### 전체 데이터 흐름

```
[Hardware]
  ERP42 차량 + Velodyne LiDAR + Ublox GPS + Xsens IMU + 카메라(3대)
       │
[Sensor Driver Layer]
  erp42_communication / ublox / ntrip_ros2 / Xsens_MTi_ROS_Driver
       │
[Perception Layer]
  plane_fit_ground_filter → adaptive_clustering → fusion(Camera-LiDAR)
  crop(CropBox) ──────────────────────────────────────────────────────
       │
[Localization Layer]
  localization(GPS+IMU+Encoder) + lidar_localization(NDT/GICP)
  robot_localization(EKF) + tf(map↔odom TF)
       │
[Planning Layer]
  create_db / path_making (경로 녹화)
  path_plan_cone (콘 기반 경로생성)
  costmap_has (주차: Costmap + Hybrid A*)
  obstacle_avoidance (장애물 우회)
  no_gps_pth (GPS 없는 환경: RRT*)
       │
[Control Layer]
  erp42_control (미션별 Stanley/MPC/PurePursuit 컨트롤러)
```

---

## 📦 패키지 목록

### 🔌 센서 드라이버 (`src/sensor/`)

| 패키지 | 역할 |
|---|---|
| `erp42_communication` | ERP42 차량과 시리얼 통신, 제어 명령 송수신 및 피드백 수신 |
| `ublox` | Ublox GPS 수신기 드라이버 (`sensor_msgs/NavSatFix` 발행) |
| `ntrip_ros2` | NTRIP 프로토콜 기반 RTK 보정 데이터 수신 (고정밀 GPS) |
| `Xsens_MTi_ROS_Driver_and_Ntrip_Client-ros2` | Xsens MTi 시리즈 IMU 드라이버 (가속도, 각속도, 방향 발행) |

---

### 📐 커스텀 메시지 정의

#### `erp42_msgs`
ERP42 차량 제어에 필요한 커스텀 메시지 타입 정의

| 메시지 | 설명 |
|---|---|
| `ControlMessage` | 속도(KPH), 조향(DEG), 기어, E-Stop 명령 |
| `SerialFeedBack` | 차량 피드백 (속도 m/s, 조향 rad) |
| `AckermannDriveStamped` | Ackermann 조향 명령 |
| `StanleyError` | Stanley 컨트롤러 오차 정보 |

#### `adaptive_clustering_msgs`
| 메시지 | 설명 |
|---|---|
| `ClusterArray` | LiDAR 클러스터 배열 (헤더 + Cluster 리스트) |

---

### 🧭 측위 (Localization)

#### `localization`
GPS, IMU, 엔코더를 융합하여 차량의 위치와 방향을 추정합니다.

| 파일 | 역할 |
|---|---|
| `gps_imu_encoder_odometry.py` | IMU 방향 + 엔코더 속도를 결합한 주 오도메트리 |
| `wheel_odometry.py` | 엔코더 기반 휠 오도메트리 |
| `imu_odometry.py` | IMU 단독 오도메트리 |
| `kf_localization.py` / `kalman_localization.py` | 칼만 필터 기반 위치 추정 |
| `gps_odom.py` | GPS 신호를 Odometry 메시지로 변환 |
| `map_server.py` / `real_time_map_server.py` | 사전 제작된 경로 맵 서버 |

#### `lidar_localization_ros2-humble`
3D LiDAR 포인트클라우드와 사전 제작된 PCD 맵을 매칭하여 cm급 정밀 측위를 수행합니다.
- **NDT (Normal Distributions Transform)** / **GICP** / **NDT_OMP** 알고리즘 지원
- 입력: `/cloud`, `/map`, `/odom`, `/imu` → 출력: `/pcl_pose`

#### `ndt_omp_ros2-humble`
NDT 알고리즘의 OpenMP 병렬 가속화 라이브러리 (`lidar_localization`의 의존성)

#### `robot_localization`
EKF(Extended Kalman Filter) / UKF를 이용해 여러 오도메트리 소스를 융합하는 표준 ROS2 패키지

#### `tf`
좌표계 변환(TF) 관리 노드 모음

| 파일 | 역할 |
|---|---|
| `map_odom_tf_publisher_static.cpp` | GPS 기반 map↔odom 정적 TF 발행 |
| `map_odom_tf_publisher_MLB.cpp` | MLB 방식 map↔odom TF 발행 |
| `odometry_map_frame.cpp` | Odometry를 map 프레임으로 변환 |
| `localization_path_tf.cpp` | 경로 TF 변환 |

---

### 👁️ 인식 (Perception)

#### `plane_fit_ground_filter`
LiDAR 포인트클라우드에서 **평면 피팅(Plane Fitting)** 방식으로 지면을 제거합니다.
- 입력: `/cropbox_filtered` → 출력: `/points_no_ground`, `/points_ground`
- 파라미터: `sensor_height`, `th_seeds`, `th_dist`, `num_iter` 등

#### `crop` / `fusion` (CropBox)
Velodyne LiDAR의 관심 영역(ROI)만 통과시키는 **CropBox 필터**입니다.
- 차량 주변 특정 범위 내 포인트만 추출
- 다양한 실험 환경 버전(학교, KCity) 및 용도별(라인검출, 콘검출) 버전 포함

#### `adaptive_clustering`
지면 제거 후 포인트클라우드에서 **Euclidean 클러스터링**으로 개별 객체(콘, 장애물)를 탐지합니다.
- VLP-16/32 등 센서 모델에 따라 클러스터링 파라미터 자동 조정
- 출력: 각 클러스터의 중심 좌표 (`PoseArray`) + 바운딩박스 마커

#### `fusion`
**카메라-LiDAR 퓨전**으로 콘의 색상(노랑/파랑)을 판별합니다.
- YOLO(`darknet_ros`)의 바운딩박스 + LiDAR 콘 위치를 Projection 행렬로 매칭
- 3대 카메라(좌/중/우) 지원
- 출력: `point/yellow` (노란 콘), `point/blue` (파란 콘)

#### `opencv_lane`
카메라 이미지를 **그레이스케일**로 변환하여 차선 검출 전처리를 수행하는 노드

#### `pointcloud_visualizer`
저장된 `.pcd` 파일을 RViz2에서 시각화하는 노드 (VoxelGrid 다운샘플링 적용)

---

### 🗺️ 경로 관리 (Path Management)

#### `create_db`
주행 중 수집한 경로 포인트를 **SQLite 데이터베이스**로 저장·관리합니다.

| 파일 | 역할 |
|---|---|
| `DB.py` | SQLite 기반 경로 DB (Node/Path 테이블) |
| `path_making.py` / `path_making_bs.py` | 주행하며 경로 포인트 실시간 녹화 |
| `path_collector.py` | 경로 포인트 수집기 |
| `path_separate.py` | 녹화된 경로를 미션 구간별로 분리 |

#### `path_making`
경로 녹화 유틸리티 패키지 (`create_db`의 보조 도구)

---

### 🛣️ 경로 계획 (Path Planning)

#### `path_plan_cone`
노란/파란 콘 위치로부터 **Delaunay 삼각분할**을 이용해 콘 중앙 경로를 실시간 생성합니다.
- 콘 간 중점을 연결하여 주행 경로 생성
- Cubic Spline 보간으로 부드러운 경로 출력

#### `costmap_has`
**주차 미션**을 위한 Costmap 기반 경로 계획 패키지

| 파일 | 역할 |
|---|---|
| `costmap.py` | 콘 위치 기반 OccupancyGrid Costmap 생성 (decay 적용) |
| `path_planner.py` | **Hybrid A\*** 알고리즘으로 주차 경로 계획 |
| `pure_pursuit.py` | Pure Pursuit 기반 경로 추종 |
| `PathPlanning/HybridAStar/` | Hybrid A* + Reeds-Shepp 경로 구현 |

#### `obstacle_avoidance`
전방 장애물 감지 시 **좌/우 우회 경로**를 자동 선택하여 발행합니다.
- 미리 저장된 좌/우 우회 경로(txt) + 전역 경로(txt) 관리
- GPS 좌표 변환(EPSG:4326 → EPSG:2097) 포함

#### `no_gps_pth`
GPS 신호 없는 실내/음영 구역에서 **RRT\* 알고리즘**으로 경로를 계획합니다.
- Costmap을 구독하여 장애물 회피 경로 탐색
- Stanley 컨트롤러로 경로 추종

---

### 🔄 좌표 변환 (TF Conversion)

#### `tf_cone`
카메라-LiDAR 퓨전 결과(velodyne 프레임)의 콘 위치를 **odom 프레임**으로 변환합니다.
- 노란/파란 콘 각각의 TF 변환 처리
- 콘 위치 누적 카운팅 버전 포함

#### `parking_tf`
콘 클러스터 위치(velodyne 프레임)를 **map 프레임**으로 변환합니다. (주차 미션용)
- `cone_pose_transform.cpp`: `/cone_poses` → `/cone_poses_map` 변환

---

### 🎮 차량 제어 (Vehicle Control)

#### `erp42_control`
각 경진대회 미션에 특화된 Stanley/PID/MPC 컨트롤러 모음입니다.  
모든 컨트롤러는 `erp42_msgs/ControlMessage`를 발행하여 차량을 구동합니다.

| 파일 | 미션 | 설명 |
|---|---|---|
| `controller_cone.py` | 🟡 콘 슬라롬 | 콘 사이 Stanley 추종, 속도 적응 조절 |
| `controller_cone_mpc.py` | 🟡 콘 슬라롬 (MPC) | Iterative Linear MPC 기반 콘 주행 |
| `controller_cone_mpc_curve.py` | 🟡 콘 슬라롬 (곡선 MPC) | 곡선 구간 MPC 최적화 |
| `controller_traffic_light.py` | 🚦 신호등 | 신호등 인식 후 정지/출발 제어 |
| `controller_stop_line.py` | ⛔ 정지선 | 정지선 감지 및 일시 정지 |
| `controller_parking.py` | 🅿️ 주차 | Hybrid A* 경로 추종 주차 |
| `controller_obstacle.py` | 🚧 장애물 회피 | 장애물 감지 시 우회 경로 전환 |
| `controller_delivery.py` | 📦 배달 | 배달 지점 이동 및 복귀 (전진/후진) |
| `controller_uturn.py` | 🔄 U-턴 | DB 경로 기반 U-턴 수행 |
| `controller_pickup.py` | 🤝 픽업 | 픽업 지점 정차 및 출발 |
| `iterative_linear_mpc.py` | 🧠 MPC 공통 | cvxpy 기반 Iterative Linear MPC 구현 |
| `DB.py` | 💾 DB 공통 | SQLite 경로 DB 읽기/쓰기 |

---

## 🖥️ Environment & Equipment

### Software
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble
- **Language**: Python 3.10, C++17
- **Database**: SQLite3

### Hardware
- **차량**: ERP42 (전기차 기반 자율주행 플랫폼)
- **LiDAR**: Velodyne VLP-16 / VLP-32
- **GPS**: Ublox (RTK 보정: NTRIP)
- **IMU**: Xsens MTi 시리즈
- **카메라**: 3대 (좌/중/우) + YOLO(darknet_ros) 객체 검출

---

## 📦 Dependencies

```bash
# ROS2 Humble 기본 패키지 외 추가 의존성
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install ros-humble-tf-transformations

# Python 의존성
pip install pyproj scipy shapely cvxpy

# 외부 ROS2 패키지 (별도 클론 필요)
# darknet_ros_msgs (YOLO 바운딩박스 메시지)
# nmea_msgs, mavros_msgs
```

---

## 🔨 Build

```bash
# 저장소 클론
git clone https://github.com/beomseok3/ACCA_2025
cd ACCA_2025

# 커스텀 메시지 먼저 빌드 (필수!)
colcon build --symlink-install --packages-select adaptive_clustering_msgs erp42_msgs
source install/setup.bash

# 전체 빌드
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Execution Guide

### 1. 센서 구동
```bash
# GPS (Ublox)
ros2 launch ublox_gps ublox_gps_node-launch.py

# IMU (Xsens)
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py

# LiDAR (Velodyne)
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py

# ERP42 차량 통신
ros2 run erp42_communication erp42_serial_node
```

### 2. 측위 구동
```bash
# GPS+IMU+엔코더 오도메트리
ros2 run localization gps_imu_encoder_odometry

# map↔odom TF 발행
ros2 run tf map_odom_tf_static

# EKF 퓨전 (robot_localization)
ros2 launch robot_localization ekf.launch.py
```

### 3. 인식 파이프라인 구동
```bash
# CropBox 필터 → 지면 제거 → 클러스터링 → 카메라-LiDAR 퓨전
ros2 launch costmap_has all_launch.py
```

### 4. 미션별 경로 계획 구동 예시

```bash
# 콘 슬라롬: 콘 좌표 변환 후 Delaunay 경로 생성
ros2 run tf_cone cone_in_odom_frame
ros2 run path_plan_cone path_cone

# 주차: Costmap 생성 후 Hybrid A* 계획
ros2 run parking_tf cone_pose_transform
ros2 run costmap_has costmap
ros2 run costmap_has path_planner

# 장애물 회피
ros2 run obstacle_avoidance obstacle_avoid
```

### 5. 차량 제어 노드 구동

```bash
# 예시: 콘 슬라롬 미션
ros2 run erp42_control controller_cone

# 예시: 주차 미션
ros2 run erp42_control controller_parking

# 예시: 신호등 미션
ros2 run erp42_control controller_traffic_light
```

---

## ⚠️ 주요 파라미터

### Stanley 컨트롤러 공통 파라미터
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `/stanley_controller/p_gain_*` | 2.07 | 미션별 P 게인 |
| `/stanley_controller/i_gain_*` | 0.85 | 미션별 I 게인 |
| `/speed_supporter/he_gain_*` | 40.0~50.0 | 헤딩 오차 속도 보정 게인 |
| `/speed_supporter/ce_gain_*` | 20.0~30.0 | 곡률 오차 속도 보정 게인 |

### Adaptive Clustering 파라미터
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `z_axis_min` | -0.8 | 클러스터링 최소 z 높이 |
| `z_axis_max` | 2.0 | 클러스터링 최대 z 높이 |
| `x_threshold` / `y_threshold` | 0.5m | 콘 크기 필터 임계값 |
| `cluster_size_min/max` | 5 / 100 | 유효 클러스터 포인트 수 범위 |

---

## 🔧 Trouble Shooting

### 1. TF 변환 실패
**증상** > 콘 좌표 변환 시 `Transform failed` 경고 발생

**원인** > map↔odom TF가 아직 발행되지 않은 상태에서 변환 시도

**해결** > 측위 노드와 TF 발행 노드가 정상 구동 중인지 확인 후 재시도

---

### 2. GPS RTK 정확도 저하
**증상** > 측위 오차가 커져 경로 추종 성능 저하

**원인** > NTRIP 서버 연결 불안정 또는 GPS 음영 구역

**해결** > `ntrip_ros2` 노드 재시작, GPS 신호가 양호한 위치로 이동 후 재초기화

---

### 3. Hybrid A* 경로 생성 실패
**증상** > 주차 경로가 생성되지 않음

**원인** > Costmap에 콘 위치가 업데이트되지 않거나, 시작/목표 지점 설정 오류

**해결** > `costmap` 토픽 확인(`ros2 topic echo /costmap`), `parking_x/y` 파라미터 조정

---

## 📁 패키지 의존 관계 요약

```
erp42_msgs ←─── erp42_control
                     ↑
adaptive_clustering_msgs ←── adaptive_clustering
                                    ↑
plane_fit_ground_filter → [points_no_ground] → adaptive_clustering
crop/fusion(cropbox) ──→ [cropbox_filtered] → plane_fit_ground_filter
fusion(camera-lidar) ──→ [yellow/blue point] → tf_cone → path_plan_cone
                                            → parking_tf → costmap_has
localization ──→ [odometry] ──→ erp42_control
lidar_localization ──→ [pcl_pose] ──→ robot_localization(EKF)
create_db / path_making ──→ [.db 파일] ──→ erp42_control
```
