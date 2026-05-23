# Capstone ROS2 Package 점검 문서

점검 기준: 현재 워크스페이스의 `/home/twkim/ros2_ws/src/capstone` 코드.

빌드 확인: `source /opt/ros/humble/setup.bash && colcon build --packages-up-to capstone --cmake-args -DBUILD_FULL_CAPSTONE=ON` 성공. 단, 기존 install 공간에 `custom_msgs`가 이미 있어 colcon override 경고가 출력되었다.

## 1. 전체 요약

`capstone` 저장소는 ROS2 인터페이스 패키지인 `custom_msgs`와 실제 노드/알고리즘 패키지인 `capstone`으로 나뉜다.

```text
capstone/
├── custom_msgs/      # ROS2 custom message package
├── src/              # ROS2 ament_cmake package: C++ nodes + Python scripts
├── models/           # 모델/자산 보관용 디렉터리
├── README.md
└── system_timing_analysis.txt
```

주 동작 흐름은 아래와 같다.

```text
Camera
  -> vision.py
  -> /vision_picam 또는 /vision_webcam
  -> ctrl_node
  -> /control
  -> bridge_node
  -> Serial/UART
  -> MCU / motor controller
  -> Serial/UART feedback
  -> bridge_node
  -> /joint
  -> ctrl_node
```

핵심 역할은 다음과 같다.

| 구성요소 | 역할 |
| --- | --- |
| `vision.py` | 카메라 프레임을 읽고 YOLO로 타겟을 검출한 뒤 `VisionMsg`를 publish |
| `ctrl_node` | PiCam/Webcam vision과 joint feedback을 받아 추정기와 제어기를 실행하고 `ControlMsg` publish |
| `bridge_node` | ROS `ControlMsg`를 시리얼 프레임으로 변환하고, MCU feedback을 `JointMsg`로 publish |
| `test_node` | 제어 없이 Vision/Kalman 추정만 테스트하고 `/test/debug` publish |
| `track_test.py`, `vision_test.py`, `test_kalman_evo.py` | 시각화, 합성 입력, Kalman 성능 평가 |
| `prediction/` | 향후 타겟 위치 예측 모델 학습/평가용 오프라인 PyTorch 코드 |

## 2. ROS 패키지 구조

### 2.1 `custom_msgs`

`custom_msgs`는 `rosidl_generate_interfaces()`로 아래 메시지를 생성한다.

| 메시지 | 용도 |
| --- | --- |
| `VisionMsg` | YOLO/vision 결과, 이미지 좌표계 상태 |
| `JointMsg` | MCU/모터 feedback으로부터 받은 yaw/pitch 상태 |
| `ControlMsg` | 제어 노드가 MCU에 보낼 yaw/pitch 명령 |
| `TestDebug` | 추정기/제어기 상태를 시각화 및 로그용으로 노출 |

### 2.2 `capstone`

`capstone/src`는 C++과 Python이 섞인 `ament_cmake` 패키지다.

```text
src/
├── CMakeLists.txt
├── package.xml
├── config/params.yaml
├── launch/
│   ├── ctrl_launch.py
│   ├── test_launch.py
│   └── vision_test_launch.py
├── include/
│   ├── bridge/
│   ├── controller/
│   ├── data/config/
│   ├── data/state/
│   ├── estimator/
│   ├── ros/
│   └── utils/
├── src/
│   ├── bridge/
│   ├── controller/
│   ├── data/config/
│   ├── estimator/
│   ├── ros/
│   ├── bridge.cpp
│   ├── ctrl.cpp
│   └── test.cpp
└── scripts/
    ├── vision.py
    ├── vision_test.py
    ├── track_test.py
    ├── test_visualizer.py
    ├── test_kalman_evo.py
    ├── camera_calibration.py
    └── prediction/
```

빌드 옵션 주의:

| 항목 | 현재 설정 |
| --- | --- |
| 기본 빌드 | `test_node`와 `capstone_library`만 빌드 |
| `ctrl_node`, `bridge_node` | `BUILD_FULL_CAPSTONE=ON`일 때만 빌드 |
| Python 설치 | `vision.py`, `vision_test.py`, `track_test.py`, `test_visualizer.py`, `test_kalman_evo.py`, 일부 prediction script 설치 |

즉 `ctrl_launch.py`로 실제 제어를 실행하려면 `colcon build` 때 CMake 옵션을 켜야 한다.

```bash
colcon build --packages-up-to capstone --cmake-args -DBUILD_FULL_CAPSTONE=ON
```

## 3. Launch 구성

### 3.1 `ctrl_launch.py`

실제 운용에 가까운 launch 파일이다.

| 노드 | 실행 파일 | 조건 | 주요 파라미터 |
| --- | --- | --- | --- |
| `bridge_node` | `bridge_node` | 항상 | `config_path` |
| `ctrl_node` | `ctrl_node` | 항상 | `config_path`, `webcam_pause_hold_sec` |
| `vision_webcam_node` | `vision.py` | `enable_webcam` | `camera=webcam`, `/vision_webcam`, `/vision_webcam/image` |
| `vision_picam_node` | `vision.py` | `enable_picam` | `camera=picam`, `/vision_picam`, `/vision_picam/image` |
| `track_picam_node` | `track_test.py` | `enable_debug` | PiCam image/vision visualization |
| `track_webcam_node` | `track_test.py` | `enable_debug` | Webcam image/vision visualization |

기본 launch argument는 `enable_debug=true`, `publish_image=true`, `enable_webcam=true`, `enable_picam=true`다.

### 3.2 `test_launch.py`

Kalman 추정기 테스트용이다.

| 노드 | 실행 파일 | 역할 |
| --- | --- | --- |
| `test_node` | `test_node` | `/vision` 구독, Kalman 추정, `/test/debug` publish |
| `python_node` | `test_kalman_evo.py` | 합성 vision 입력 publish 및 evo metric 시각화 |

현재 `test_node`는 `xterm -e gdb -ex run --args` prefix가 설정되어 있다.

### 3.3 `vision_test_launch.py`

실카메라 vision과 Kalman debug를 같이 보는 테스트용이다.

| 노드 | 실행 파일 | 역할 |
| --- | --- | --- |
| `vision_node` | `vision.py` | 기본 `/vision` publish |
| `test_node` | `test_node` | `/vision` 기반 추정 |
| `python_node` | `vision_test.py` | `/test/debug`와 image 시각화 |

## 4. ROS 토픽 및 메시지 흐름

### 4.1 실제 제어 흐름

```text
vision_picam_node
  pub /vision_picam       custom_msgs/VisionMsg
  pub /vision_picam/image sensor_msgs/Image, publish_image=true일 때

vision_webcam_node
  sub /vision_webcam/enabled std_msgs/Bool
  pub /vision_webcam         custom_msgs/VisionMsg
  pub /vision_webcam/image   sensor_msgs/Image, publish_image=true일 때

bridge_node
  sub /control custom_msgs/ControlMsg
  pub /joint   custom_msgs/JointMsg

ctrl_node
  sub /vision         custom_msgs/VisionMsg
  sub /vision_picam   custom_msgs/VisionMsg
  sub /vision_webcam  custom_msgs/VisionMsg
  sub /joint          custom_msgs/JointMsg
  pub /control              custom_msgs/ControlMsg
  pub /test/debug           custom_msgs/TestDebug
  pub /vision_webcam/enabled std_msgs/Bool
```

`ctrl_node`는 `/vision`, `/vision_picam`, `/vision_webcam` 세 토픽을 모두 구독한다. `VisionMsg.camera == "picam"`이면 PiCam으로 처리하고, 그 외에는 Webcam으로 처리한다. `VisionMsg.camera`가 비어 있으면 내부 `RobotState` 기본값이 `"webcam"`이라서 Webcam 계열로 들어간다.

### 4.2 메시지별 변수

#### `VisionMsg`

Vision 노드가 제어/추정 노드로 보내는 이미지 좌표계 타겟 상태다.

| 필드 | 타입 | 의미 | 현재 사용 방식 |
| --- | --- | --- | --- |
| `header.stamp` | `std_msgs/Header` | 샘플 시간 | `RobotState` 생성 시 읽지만, `ctrl_node`에서는 수신 시간으로 다시 덮어씀 |
| `detected` | `bool` | YOLO 검출 여부 | Kalman measurement 여부 판단 |
| `tracked` | `bool` | tracker 기반 추적 여부 | 현재 `vision.py`는 항상 `False` |
| `p` | `float32[]` | 타겟 중심 픽셀 `[x, y]` | 제어/추정의 position |
| `v` | `float32[]` | 픽셀 속도 `[vx, vy]` | 있으면 Kalman velocity update에 사용 |
| `a` | `float32[]` | 픽셀 가속도 `[ax, ay]` | 있으면 Kalman acceleration update에 사용 |
| `bbox` | `float32[]` | bbox 크기 `[w, h]` | 시각화에서 bbox 그릴 때 사용 |
| `img_center` | `float32[]` | 이미지 중심 `[cx, cy]` | PiCam pixel 제어 기준점 |
| `confidence` | `float32` | 검출 confidence | 메시지에는 있지만 현재 `vision.py`는 명시 설정하지 않음 |
| `covariance` | `float32[]` | 측정 공분산 확장용 | 현재 빈 배열 |
| `camera` | `string` | `"picam"` 또는 `"webcam"` | `ctrl_node`의 카메라 분기 기준 |

#### `JointMsg`

Bridge가 MCU feedback을 ROS로 올리는 메시지다.

| 필드 | 타입 | 의미 |
| --- | --- | --- |
| `header.stamp` | `std_msgs/Header` | bridge 수신/parse 시간 |
| `joint` | `float32[]` | `[yaw, pitch]` |
| `joint_vel` | `float32[]` | `[yaw_velocity, pitch_velocity]` |
| `joint_torque` | `float32[]` | `[yaw_torque, pitch_torque]` |

`ctrl_node`는 `joint`와 `joint_vel`만 캐싱한다.

#### `ControlMsg`

제어 노드가 Bridge/MCU로 보내는 명령이다.

| 필드 | 타입 | 의미 | 현재 설정 |
| --- | --- | --- | --- |
| `header.stamp` | `std_msgs/Header` | 제어 publish 시간 | `ctrl_node` 현재 시간 |
| `u_yaw` | `float32` | yaw 명령 | rad 단위 값으로 사용 |
| `u_pitch` | `float32` | pitch 명령 | rad 단위 값으로 사용 |
| `fire` | `bool` | 발사 명령 | 현재 컨트롤러는 항상 `false` |
| `reload` | `bool` | reload 명령 | 현재 컨트롤러는 항상 `false` |
| `manual` | `bool` | 수동 명령 여부 | `ctrl_node`는 항상 `false` |
| `ispixel` | `bool` | 픽셀 기반 상대 제어 여부 | PiCam tracking/prediction이면 `true`, 각도 명령이면 `false` |

#### `TestDebug`

디버깅/시각화용 통합 상태 메시지다.

| 필드 그룹 | 필드 | 의미 |
| --- | --- | --- |
| 시간 | `sample_time`, `dt` | 원본 샘플 시간 또는 추정 상태 시간, Kalman dt |
| 원본 vision | `raw_p`, `raw_v`, `raw_a`, `has_raw` | YOLO/vision 입력 상태 |
| 추정 상태 | `estimated_p`, `estimated_v`, `estimated_a`, `estimator_initialized`, `predicted_only` | Kalman 상태 |
| detection | `detected`, `tracked` | 원본 또는 현재 상태의 detection flag |
| control | `has_control`, `u_yaw`, `u_pitch`, `fire`, `reload`, `is_pixel` | 제어 출력 |
| mode | `tracking_mode`, `control_source` | `ctrl_node`의 결정 결과 |
| camera 선택 | `webcam_enabled`, `picam_age`, `webcam_age`, `source_age` | 카메라 handoff/age 디버그 |

## 5. 노드별 내부 동작

### 5.1 `vision.py`

`VisionNode`는 ROS timer를 쓰지 않고 `run()` 내부 while loop에서 계속 동작한다.

```text
read_img()
  -> detect_bell()
  -> publish()
  -> publishImage()
  -> executor.spin_once(timeout_sec=0.0)
```

카메라 입력:

| 카메라 | 코드 경로 | 설정 |
| --- | --- | --- |
| PiCam/CSI | `LatestFrameReader` + GStreamer | 1280x720, 60 fps, `cv2.rotate(..., ROTATE_180)` |
| Webcam | `WebCAMLatestFrameReader` + V4L2 | `/dev/video1`, 1920x1080, MJPG, 30 fps |

YOLO detection:

| 항목 | 값 |
| --- | --- |
| 모델 | `yolo_model_path_picam` 또는 `yolo_model_path_webcam` |
| inference size | `imgsz=416` |
| confidence threshold | `conf=0.65` hard-coded |
| class filter | `classes=[1]` hard-coded |
| 선택 bbox | 첫 번째 bbox `boxes.xyxy[0]` |

motion estimate:

| 변수 | 계산 |
| --- | --- |
| `position` | bbox 중심 `[x, y]` |
| `velocity` | `(position - prev_position) / dt`에 LPF 적용 |
| `acceleration` | `(velocity - prev_velocity) / dt`에 LPF 적용 |
| LPF 계수 | `a_ = 0.8` |
| reset 조건 | detection 실패 또는 `dt > max_motion_dt` |

Webcam enable:

| 토픽 | 동작 |
| --- | --- |
| `/vision_webcam/enabled` | `False`를 받으면 Webcam 노드는 detection 상태와 motion estimate를 reset하고 publish loop를 건너뜀 |

### 5.2 `ctrl_node`

`CtrlNode`는 vision, joint, timer callback으로 이루어진 메인 제어 노드다.

입력 cache:

| 입력 | 내부 변수 |
| --- | --- |
| `/vision_picam` 또는 `camera=="picam"` | `latest_picam_state_`, `latest_picam_receive_time_` |
| `/vision_webcam`, `/vision`, 또는 `camera!="picam"` | `latest_webcam_state_`, `latest_webcam_receive_time_` |
| `/joint` | `joint_`, `joint_vel_` |

timer 주기:

| 파라미터 | 현재 값 |
| --- | --- |
| `controller.frequency` | `50` Hz |
| timer period | 약 `20 ms` |

제어 모드 선택 우선순위:

```text
1. 새 PiCam detection이 있으면 PICAM_TRACK
2. PiCam lock이 있고 KF prediction 가능 시간이면 PICAM_PREDICT
3. PiCam lock이 있고 hold 시간이 남아 있으면 PICAM_HOLD
4. fresh Webcam detection이 있으면 WEBCAM_SEARCH
5. 그 외 IDLE
```

구체 조건:

| 조건 | 파라미터 |
| --- | --- |
| PiCam prediction 유지 | `picam_prediction_max_sec = 0.25 s` |
| PiCam hold 유지 | `picam_track_hold_sec = 1.0 s` |
| Webcam detection freshness | `webcam_measurement_max_age = 0.3 s` |

모드별 출력:

| 모드 | source | webcam enabled | 제어 함수 | `ControlMsg.ispixel` |
| --- | --- | --- | --- | --- |
| `PICAM_TRACK` | `PICAM_DETECTION` | `false` | `computePicamPixelTrack()` | `true` |
| `PICAM_PREDICT` | `PICAM_PREDICTION` | `false` | `computePicamPixelTrack()` | `true` |
| `PICAM_HOLD` | `JOINT_HOLD` | `false` | `computeJointHold()` | `false` |
| `WEBCAM_SEARCH` | `WEBCAM_DETECTION` | `true` | `computeWebcamAngleSearch()` | `false` |
| `IDLE` | `JOINT_HOLD` | `true` | `computeJointHold()` | `false` |

PiCam pixel 제어:

```text
img_center_target = img_center + image_offset
u = Kp * (img_center_target - p) - Kd * v
u_rad = deg2rad(u)
```

현재 `params.yaml` 값:

```text
image_offset = [30.0, 100.0]
Kp = diag([0.01, -0.005])
Kd = diag([0.0, 0.0])
```

Webcam angle search:

1. Webcam pixel을 카메라 calibration으로 normalized ray로 변환한다.
2. distortion coefficient를 반복 보정한다.
3. ray와 joint 각도를 이용해 타겟 yaw/pitch angle을 계산한다.
4. Webcam의 경우 joint를 `[0, 0]`으로 놓고 각도를 계산한다.

현재 fire/reload:

`ControlState::pixel()`과 `ControlState::angle()` 모두 `fire=false`, `reload=false`를 반환한다. 따라서 현재 메인 제어 루프에는 실제 발사/reload 조건이 연결되어 있지 않다.

### 5.3 `Estimator` / `KalmanFilter`

현재 빌드되는 Kalman filter는 이미지 평면의 constant acceleration 모델이다.

상태 벡터:

```text
x = [px, py, vx, vy, ax, ay]^T
```

prediction model:

```text
px' = px + vx*dt + 0.5*ax*dt^2
py' = py + vy*dt + 0.5*ay*dt^2
vx' = vx + ax*dt
vy' = vy + ay*dt
ax' = ax
ay' = ay
```

업데이트 흐름:

| 입력 | 동작 |
| --- | --- |
| `init(RobotState)` | detection/tracking이 있을 때 `p`, optional `v`로 초기화 |
| `update(RobotState)` | vision measurement update |
| `update(double t)` | measurement 없이 process prediction |
| `getState(false)` | correction 이후 `x_`를 `RobotState`로 복사 |
| `getState(true)` | process prediction `x_pred_`를 `RobotState`로 복사 |

measurement covariance:

| 상황 | 파라미터 |
| --- | --- |
| `detected=true` position | `r_detected` |
| `tracked=true` position | `r_tracked` |
| velocity hint | `r_temp_vel` |
| acceleration hint | `r_temp_acc` |

현재 `params.yaml` 값:

```text
q_acc = 1e+5
r_detected = 10.0
r_tracked = 10.0
r_temp_vel = 900
r_temp_acc = 54000
p0_pos = 10.0
p0_vel = 900
p0_acc = 54000
max_time_gap = 1.0
```

### 5.4 `bridge_node`

`BridgeNode`는 ROS 제어 메시지와 MCU 시리얼 프로토콜 사이를 변환한다.

ROS interface:

| 방향 | 토픽 | 메시지 |
| --- | --- | --- |
| subscribe | `/control` | `custom_msgs/ControlMsg` |
| publish | `/joint` | `custom_msgs/JointMsg` |

주기:

| 동작 | 파라미터 | 현재 값 |
| --- | --- | --- |
| serial read watchdog | `bridge.serial.watchdog_frequency` | `200 Hz` |
| serial write | `bridge.serial.write_frequency` | `50 Hz` |

시리얼 설정:

| 파라미터 | 현재 값 |
| --- | --- |
| `name` | `"usb"` |
| `read_port` | `/dev/ttyTHS1` |
| `write_port` | `/dev/ttyACM0` |
| `baud` | `115200` |
| `data_bits` | `8` |
| `stop_bits` | `1` |
| `parity` | `0` |

현재 구현에서 `BridgeNode`는 `Serial(port_config, false)`로 생성된다. `Serial::openSerialPort(false)`는 `write_port`를 연다. 따라서 현재 코드는 `/dev/ttyACM0` 하나의 fd로 read/write를 모두 수행하고, `read_port`는 실제로 사용되지 않는다.

제어 write frame:

```text
SOF          2 bytes  0xAA 0x55
prev_status 1 byte   이전 read frame parse 결과, OK=0x01, FAILED=0x00
len         1 byte   9
seq         1 byte   write sequence
u_yaw       4 bytes  float32
u_pitch     4 bytes  float32
flags       1 byte
crc16       2 bytes  CCITT, little-endian 저장
```

`flags` bit:

| bit | 조건 | 의미 |
| --- | --- | --- |
| `0x01` | `!msg.ispixel` | angle 명령 |
| `0x02` | `msg.fire` | shoot |
| `0x04` | `msg.reload` | reload 또는 stop 확장 |

feedback read frame:

```text
SOF       2 bytes  0xAA 0x55
status    1 byte
len       1 byte   25 expected
seq       1 byte
payload  25 bytes
crc16     2 bytes
```

현재 decode는 payload 앞 24 byte를 `float[6]`으로 읽는다.

| float index | ROS 출력 |
| --- | --- |
| `values[0]` | `joint[0]`, yaw |
| `values[1]` | `joint_vel[0]`, yaw velocity |
| `values[2]` | `joint_torque[0]`, yaw torque |
| `values[3]` | `joint[1]`, pitch |
| `values[4]` | `joint_vel[1]`, pitch velocity |
| `values[5]` | `joint_torque[1]`, pitch torque |

remote status가 실패이고 `last_write_frame_`이 있으면 마지막 write frame을 재전송한다.

### 5.5 `test_node`

`TestNode`는 제어를 하지 않고 estimator만 검증한다.

| 방향 | 토픽 | 메시지 |
| --- | --- | --- |
| subscribe | `/vision` 기본값 | `VisionMsg` |
| publish | `/test/debug` | `TestDebug` |

동작:

1. `/vision`이 들어오면 `RobotState`로 변환한다.
2. `detected || tracked`인 경우 Kalman filter를 init/update한다.
3. vision callback 직후 correction 상태를 `/test/debug`로 publish한다.
4. timer에서는 measurement 없이 prediction을 수행하고 `predicted_only=true`로 debug publish한다.

timer 주기는 `controller.frequency`를 공유한다. 현재 값은 `50 Hz`다.

## 6. 내부 데이터 구조

### 6.1 `RobotState`

`VisionMsg`와 joint feedback을 내부 알고리즘에서 쓰기 편하게 모은 구조체다.

| 필드 | 의미 |
| --- | --- |
| `t`, `dt` | 샘플 시간, 직전 measurement 대비 시간 간격 |
| `p`, `v`, `a` | 이미지 평면 position/velocity/acceleration |
| `img_center` | 이미지 중심 |
| `camera` | `"picam"` 또는 `"webcam"` |
| `joint`, `joint_vel` | yaw/pitch 실제 joint feedback |
| `angle`, `omega` | angle 기반 controller 설계 흔적, 현재 main controller에서는 거의 사용하지 않음 |
| `detected`, `tracked`, `process` | measurement/process 상태 flag |

### 6.2 `ControlState`

제어기 내부 출력이다. 최종적으로 `ControlMsg`로 변환된다.

| 필드 | 의미 |
| --- | --- |
| `u_yaw`, `u_pitch` | yaw/pitch 명령 |
| `fire`, `reload` | 발사/reload flag |
| `isPixel` | pixel 상대 제어인지 angle 명령인지 구분 |

### 6.3 Config 구조

| 파일 | 클래스 | 읽는 YAML |
| --- | --- | --- |
| `control_config.*` | `ControlConfig`, `CameraCalibration` | `calibration`, `controller` |
| `estimator_config.*` | `EstimatorConfig` | `estimator`, `controller.frequency` |
| `port_config.*` | `PortConfig` | `bridge.serial` |

## 7. `params.yaml` 주요 값

### 7.1 Vision

```yaml
vision:
  target_class: 1
  confidence: 0.2
```

현재 `vision.py`에서는 이 값을 실제로 읽지 않고, class `1`, confidence `0.65`를 코드에 hard-code한다.

### 7.2 Camera Calibration

`calibration.webcam`과 `calibration.picam`에 각각 아래 값을 둔다.

| 값 | 용도 |
| --- | --- |
| `image_width`, `image_height` | 카메라 이미지 크기 |
| `camera_matrix` | row-major 3x3 intrinsic matrix |
| `distortion_coefficients` | OpenCV distortion coefficients |

`Controller::computeWebcamAngleSearch()`에서 pixel ray 보정에 사용된다.

### 7.3 Controller

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `image_offset` | `[30.0, 100.0]` | PiCam 조준 기준점을 이미지 중심에서 이동 |
| `angle_offset` | `[0.0, 0.0]` | 현재 main controller에서는 실질적으로 사용 안 함 |
| `Kp` | `[0.01, -0.005]` | PiCam pixel PD gain |
| `Kd` | `[0.0, 0.0]` | PiCam pixel velocity damping |
| `frequency` | `50` | `ctrl_node`, `test_node` timer 기준 |
| `time_delay` | `0.1` | 현재 main controller에서는 fire 예측에 연결 안 됨 |
| `picam_prediction_max_sec` | `0.25` | PiCam detection 유실 후 KF prediction 유지 |
| `picam_track_hold_sec` | `1.0` | prediction 후 joint hold 유지 |
| `webcam_measurement_max_age` | `0.3` | Webcam detection 유효 시간 |
| `err_p_track`, `err_v_track`, `err_p_fire` | `1e-3` | FSM/발사 조건 설계용, 현재 main controller 미사용 |

### 7.4 Bridge

| 파라미터 | 현재 값 |
| --- | --- |
| `watchdog_frequency` | `200` |
| `write_frequency` | `50` |
| `baud` | `115200` |

## 8. 테스트/시각화 스크립트

| 파일 | 역할 |
| --- | --- |
| `track_test.py` | image, vision, debug를 OpenCV window에 overlay |
| `vision_test.py` | `/test/debug`를 offline matplotlib 또는 realtime image overlay로 시각화 |
| `test_visualizer.py` | 합성 `VisionMsg` publish 및 Kalman/debug 시각화 |
| `test_kalman_evo.py` | 합성 trajectory를 publish하고 APE/RPE metric 계산 |
| `camera_calibration.py` | checkerboard 기반 camera calibration |
| `prediction/generate_synthetic_data.py` | prediction 학습용 synthetic CSV 생성 |
| `prediction/sample_training_data.py` | ROS `test_node`와 함께 Kalman output을 CSV로 수집 |
| `prediction/gru`, `prediction/tcn` | 미래 target 위치 예측 모델 학습/평가 |

## 9. 현재 코드 기준 주의점

1. `ctrl_node`와 `bridge_node`는 기본 빌드에 포함되지 않는다.
   - `src/CMakeLists.txt`의 `BUILD_FULL_CAPSTONE` 기본값이 `OFF`다.
   - 실제 제어 launch를 쓰려면 `-DBUILD_FULL_CAPSTONE=ON`이 필요하다.

2. `bridge_node`는 현재 `write_port`만 연다.
   - `params.yaml`에는 `read_port=/dev/ttyTHS1`, `write_port=/dev/ttyACM0`로 분리되어 있다.
   - 하지만 `BridgeNode`가 `Serial(port_config, false)`로 생성되어 `/dev/ttyACM0` 하나에서 read/write를 모두 수행한다.
   - 진짜 read/write 포트가 분리된 하드웨어라면 구현 수정이 필요하다.

3. read frame payload 길이는 25 byte인데 실제 decode는 24 byte만 사용한다.
   - `kReadPayloadSize = 25`
   - `float[6] = 24 byte`만 `memcpy`한다.
   - 남은 1 byte의 의미가 프로토콜 문서와 맞는지 확인이 필요하다.

4. `VisionMsg`에는 `source_mode` 필드가 없다.
   - 그런데 `test_visualizer.py`, `test_kalman_evo.py`, `prediction/sample_training_data.py`는 `msg.source_mode = ...`를 설정한다.
   - 현재 메시지 정의 기준으로는 Python runtime에서 attribute error가 날 가능성이 높다.

5. `vision.py`는 `params.yaml`의 `vision.target_class`, `vision.confidence`를 사용하지 않는다.
   - 실제 inference는 `classes=[1]`, `conf=0.65`로 고정되어 있다.

6. `FSM`, `controller_angle.cpp`, `kalman_filter_angle.cpp`, `kalman_filter_cv.cpp`는 현재 main build/flow에서 쓰이지 않는 구버전 또는 실험 코드로 보인다.
   - `controller_angle.cpp`는 현재 `Controller` 클래스 정의와 맞지 않는 멤버를 참조한다.
   - `kalman_filter_angle.cpp`도 현재 빌드되는 `kalman_filter.hpp`와 내용이 섞여 있어 그대로 빌드 대상에 넣으면 문제가 날 수 있다.

7. 현재 fire/reload 로직은 실제 상태 머신에 연결되어 있지 않다.
   - `ControlState::pixel()`과 `ControlState::angle()`이 항상 `fire=false`, `reload=false`를 반환한다.
   - `ControlMsg`와 serial flags에는 fire/reload 전달 경로가 있지만 상위 제어 판단이 아직 비어 있다.

8. `ctrl_node`는 `VisionMsg.header.stamp` 대신 수신 시간을 `RobotState.t`로 사용한다.
   - 카메라 capture 시간과 ROS 수신 시간 차이를 구분하려면 별도 필드 또는 stamp 유지가 필요하다.

9. `JointMsg` 배열 길이 검증이 약하다.
   - `ctrl_node::jointCallback()`은 `joint`, `joint_vel`를 바로 Eigen vector로 변환한다.
   - Bridge가 내는 정상 메시지는 길이 2지만, 외부에서 malformed message를 publish하면 문제가 될 수 있다.

## 10. 추천 실행 명령

기본 estimator 테스트:

```bash
colcon build --packages-up-to capstone
source ~/ros2_ws/install/setup.bash
ros2 launch capstone test_launch.py
```

실제 제어 노드 포함 빌드:

```bash
colcon build --packages-up-to capstone --cmake-args -DBUILD_FULL_CAPSTONE=ON
source ~/ros2_ws/install/setup.bash
ros2 launch capstone ctrl_launch.py
```

토픽 확인:

```bash
ros2 topic list
ros2 topic echo /vision_picam
ros2 topic echo /control
ros2 topic echo /joint
ros2 topic echo /test/debug
```
