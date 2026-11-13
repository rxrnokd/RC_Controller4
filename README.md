# Raspberry Pi RC 제어 · ROS 2 Jazzy · SLAM 초기 버전

이 폴더의 코드를 **기존 `~/rc_lidar_docker/controller`에 추가**하여 사용합니다. 기존 Dockerfile, `ros2_ws/src/sllidar_ros2`, `config/slam.yaml`, `maps`는 교체하지 않습니다. Docker/ROS/SLAM을 재설치하는 스크립트는 없습니다.

현재 제공물은 실제 Linux GPIO 제어 코드, 웹 UI, ROS 연결 코드와 안전 테스트입니다. 실제 차량·Pi·Docker에는 접속하지 않았습니다. **실차 주행 검증 완료 제품이 아닙니다.** 첨부 자료만으로 확정할 수 없는 전원·기구 설정은 `null`이며, 기본 설정으로는 모터·서보를 구동하지 않습니다. 미측정값을 생성하는 시뮬레이터는 없습니다.

## 1. 첨부 자료 분석과 구현 범위

| 첨부 | 확인한 내용 | 구현에 반영한 내용 |
|---|---|---|
| `도커 ROS SLAM 폴더 구조.txt` | Dockerfile, config/slam.yaml, maps, sllidar_ros2 소스/launch/SDK 구조 | 기존 경로를 상위 프로젝트로 사용. 실제 Dockerfile 내용·이미지명·컨테이너명·마운트·slam.yaml 내용은 자료에 없음 |
| `GPIO_Pin.txt` | BCM 핀 번호, 모터 AB 엔코더, MT6701 ABZ, US-100 4개 | 아래 핀 표와 config.json |
| `스크린샷 2026-11-08 081911.png` | JGB37-520R30-12, 정격 12V, 1:30, 정격 280rpm, 무부하 320rpm, 정격전류 0.36A, 정지전류 3.2A, AB 11 line, 엔코더 전원 3.3~5V | 정격 전압을 넘어서는 설정 거부. 320rpm을 안전한 주행속도나 최대 PWM 보증으로 사용하지 않음. 실제 감속비/카운트 보정 필요 |
| `스크린샷 2026-11-08 081923.png` | LD-1501MG, 6~8.4V, PWM 500~2500µs, 축 회전 0~180°, 정지전류 2.4~3A | 절대 펄스 경계 검증. 실제 조향 링크의 좌/우 끝점·중립은 별도 설정. 축 회전각과 차량 바퀴 조향각은 다름 |
| `MDD3A_Datasheet.pdf` (표 1~3) | 입력 4~16V, 채널당 연속 3A/피크 5A(<5초), PWM 최대 20kHz, 로직 High 1.7~12V, 보조 5V 출력 최대 200mA | A/B 입력 제어. Low/Low는 브레이크. 1kHz PWM은 최대치가 아닌 구동 설정이며 실제 pigpio 선택 주파수도 검증. 5VO를 Pi/서보 전원으로 사용하지 않음 |
| `SLAMTEC_rplidar_datasheet_C1_v1.2_en.pdf` | C1M1, 460800bps 8N1, 5V(4.8~5.2V), 기동 전류 typical 800mA, 5k samples/s, typical 10Hz, 백색 물체 0.05~12m, 흑색 0.05~6m | 공식 C1 launch/USB 드라이버 재사용. raw SDK 직접 재구현 없음. 드라이버가 ROS 좌표계 변환 담당 |
| `SLAMTEC_rplidarkit_usermanual_C1_v1.2_en.pdf` | USB Type-C/CP2102 어댑터, 전원만으로 스캔 시작하지 않음, 스캔/상태 확인, 약 2분 예열 권장 | USB 장치 전달·스캔 점검·전원 문제 해결 절차 |
| `LR002_SLAMTEC_rplidar_sdk_v2.0_en.pdf` | SDK include/src, serial/TCP/UDP 채널, scan/health API, 플랫폼별 빌드 | 첨부된 기존 sllidar_ros2 내 SDK를 사용. 이 문서의 구형 A2/A3 baud 예를 C1에 적용하지 않음 |

MDD3A의 3A 연속 정격보다 모터 정지전류 3.2A가 큽니다. 현재 전류센서가 없으므로 **PWM 제한만으로 전류 제한을 보장할 수 없습니다.** 걸림/스톨을 지속시키지 않고 전원 차단 장치를 사용해야 합니다. 배터리 최대 실측 전압과 부하 조건 없이 “PWM 100%가 안전하다”고 설정하지 않았습니다.

### 구현 상태

| 기능 | 코드 상태 | 실제 차량에서 필요한 확인 |
|---|---|---|
| MANUAL 정·역 PWM, 조향 | 구현 | 전원·극성·펄스 중립·링크 한계 |
| 비상정지, 만료, 단일 조종권, 모드별 차단 | 구현/단위 테스트 | Wi-Fi 끊기·프로세스 종료·실제 제동 거리 |
| 모터 AB 엔코더 / 트레일러 축 MT6701 ABZ | 독립 센서로 구현 | 각 센서의 전압, x4 카운트, 방향, Z 위상 |
| US-100 순차 측정 | 구현 | 거리 허용 범위, 펄스 모드, 반사/간섭 |
| RPM/회전/거리/PI | 조건부 구현 | 카운트 보정, PI 이득, 저속 펄스 간격 |
| `/odom`, `odom → base_link` TF | 조건부 구현 | 모터가 좌우 후륜을 각각 측정하는지, 윤거 |
| C1 `/scan`, SLAM launch, 지도/위치 표시 | 구현, 실행 미검증 | 기존 컨테이너/설정, 실제 TF/지도/위치 정확도 |
| AUTO_PARK 공통 프로토콜/클라이언트 | 구현, 기본 잠금 | 향후 노트북 알고리즘·위치 품질 검증·실차 승인 |
| MPC·경로계획·자동주차 알고리즘 | **구현하지 않음** | 노트북에서 향후 구현 |

## 2. 구조와 파일

```text
Windows / macOS 브라우저          향후 노트북 Python 알고리즘
        └──────── HTTP + WebSocket /ws ────────┘
                       Raspberry Pi OS
                 server.py + core.py (명령 중재)
                      hardware.py → pigpiod → GPIO
                           │
              runtime/ros.sock (공유 폴더 Unix socket)
                           │
                  기존 Ubuntu / ROS Jazzy Docker
              ros_bridge.py → /odom + /tf + /tf_static
              기존 sllidar_ros2 → /scan
              기존 SLAM Toolbox → /map + map→odom TF
```

외부에서 접근하는 서버는 8080 HTTP/WebSocket 하나입니다. ROS 데이터와 명령은 같은 WebSocket을 사용하며, 컨테이너와 호스트 사이에는 외부 포트가 없는 Unix 소켓 하나를 사용합니다. rosbridge/MQTT/추가 API 서버는 필요 없습니다. Docker 컨테이너에는 GPIO를 전달하지 않습니다.

* `config.json`: 하드웨어/네트워크/안전 설정의 단일 원본.
* `core.py`: Command Arbiter, 출력 제한, PI, Odometry.
* `hardware.py`: 실제 pigpio 입출력, ABZ, 초음파, 독립 watchdog 프로세스.
* `server.py`, `ui.html`: 웹 서버와 브라우저 UI.
* `ros_bridge.py`, `rc.launch.py`, `run_ros.sh`: 기존 ROS 환경 연동.
* `laptop_client.py`: 향후 알고리즘용 비동기 Python API와 읽기 전용 모니터.
* `rc-controller.service`, `requirements.txt`, `tests/`: 시작 설정·의존성·테스트.

## 3. 먼저 기존 환경 확인 — Raspberry Pi OS 터미널

```bash
cd ~/rc_lidar_docker
pwd
cat Dockerfile
cat config/slam.yaml
ls ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
ls maps
docker ps -a
docker images
```

기존 컨테이너 이름을 확인해 다음 `기존컨테이너`를 실제 이름으로 바꿉니다.

```bash
docker inspect 기존컨테이너 --format '{{.Config.Image}}'
docker inspect 기존컨테이너 --format '{{json .Mounts}}'
docker inspect 기존컨테이너 --format '{{json .HostConfig.Devices}}'
docker exec -it 기존컨테이너 bash
```

**이제 Docker 내부 터미널:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix sllidar_ros2
python3 -c 'import rclpy, tf2_ros; from nav_msgs.msg import Odometry, OccupancyGrid'
```

`sllidar_ros2`만 검색되지 않으면 기존 workspace의 `install/setup.bash`를 source합니다. install이 없는 경우에만 **기존 소스**를 빌드합니다. `/project`는 아래 Docker 마운트 예시의 컨테이너 경로이며, 기존 마운트가 다르면 실제 경로를 사용합니다.

```bash
cd /project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2
source install/setup.bash
ros2 pkg prefix sllidar_ros2
```

소스 새 clone, Dockerfile 교체, ROS 배포판 재설치는 하지 않습니다.

## 4. 파일 복사 — Windows 개발 PowerShell

아래 사용자명·IP는 실제 Pi 값으로 바꿉니다. 브라우저 조종만 하는 노트북에는 Python이 필요 없습니다. SSH/SCP는 초기 배포할 때만 사용합니다.

```powershell
cd 'C:\Users\goote\OneDrive\바탕 화면\RC_Controller4'
$PiTarget = '사용자명@192.168.0.50'
ssh $PiTarget 'mkdir -p ~/rc_lidar_docker/controller/tests'
scp config.json core.py hardware.py server.py ui.html ros_bridge.py rc.launch.py run_ros.sh laptop_client.py requirements.txt rc-controller.service README.md "${PiTarget}:~/rc_lidar_docker/controller/"
scp tests/test_safety.py tests/test_web.py "${PiTarget}:~/rc_lidar_docker/controller/tests/"
python -m unittest discover -s tests -v
```

macOS에서도 같은 파일을 `scp`로 복사할 수 있습니다. 첨부 원본은 Pi 실행에 필요하지 않으며 기존 Dockerfile/maps는 덮어쓰지 않습니다.

## 5. 호스트 의존성과 최초 UI 실행 — Raspberry Pi OS

```bash
cd ~/rc_lidar_docker/controller
sudo apt update
sudo apt install python3-venv python3-pip pigpiod python3-pigpio
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
sudo systemctl enable --now pigpiod
systemctl status pigpiod --no-pager
```

OS 저장소에 `pigpiod` 패키지가 없으면 해당 Raspberry Pi OS 릴리스에서 Pi 4 지원 pigpio를 먼저 확보해야 합니다. 다른 GPIO 라이브러리로 조용히 대체하지 않습니다. 이 코드는 pigpio가 지원하는 Pi 4용이며 Pi 5용이 아닙니다.

pigpiod 8888 포트는 네트워크에 노출하지 않습니다. 배포된 서비스 옵션을 `systemctl cat pigpiod`로 확인하고 로컬 접속만 허용하는 `-l` 옵션을 사용합니다. 옵션이 없다면 다음 override를 적용합니다.

```bash
sudo systemctl edit pigpiod
```

편집기에 다음 입력 후 저장:

```ini
[Service]
ExecStart=
ExecStart=/usr/bin/pigpiod -l
```

```bash
sudo systemctl daemon-reload
sudo systemctl restart pigpiod
```

`config.json`의 `hardware_enabled:false` 상태에서는 GPIO 연결 없이 UI를 실행할 수 있습니다. 이 모드는 센서값 모두 `null`인 설정 점검용이며 시뮬레이션이 아닙니다.

### Wi-Fi 설정

```bash
hostname -I
ip -4 address show wlan0
nano config.json
```

예를 들어 Pi IP가 `192.168.0.50`, 신뢰하는 Wi-Fi 서브넷이 `192.168.0.0/24`인 경우에만 다음과 같이 수정합니다.

```json
"network": {
  "bind": "192.168.0.50",
  "port": 8080,
  "allowed_subnets": ["192.168.0.0/24", "127.0.0.0/8"],
  "allowed_hosts": ["192.168.0.50:8080", "127.0.0.1:8080", "localhost:8080"]
}
```

공유기에서 DHCP 예약을 하면 IP 변경을 줄일 수 있습니다. 접속 IP·Host 헤더·브라우저 Origin을 검사하고 cross-origin WebSocket을 거부합니다. 로그인·비밀번호·토큰·키 발급 시스템은 없습니다. 이는 같은 LAN의 악의적인 클라이언트를 인증하는 기능이 아닙니다. 신뢰하는 Wi-Fi에만 두고 인터넷 포트포워딩을 하지 않습니다. `allowed_subnets`에 모든 네트워크를 넣지 않습니다.

```bash
.venv/bin/python server.py
```

노트북 브라우저에서 `http://192.168.0.50:8080` 접속. 설정이 부족하면 UI에 부족한 항목이 표시되고 구동 준비가 차단됩니다. 프로그램 중지: `Ctrl+C`.

## 6. 핀 배선과 하드웨어 설정

**모든 번호는 BCM이며 물리 헤더 번호가 아닙니다.**

| 기능 | GPIO |
|---|---|
| MDD3A M1A / M1B | 16 / 20 |
| 모터 1 엔코더 A / B | 4 / 14 |
| MDD3A M2A / M2B | 21 / 26 |
| 모터 2 엔코더 A / B | 15 / 18 |
| 서보 PWM | 13 |
| 트레일러 축 MT6701 A / B / Z | 19 / 10 / 9 |
| US-100 1 Trig / Echo | 17 / 27 |
| US-100 2 Trig / Echo | 22 / 23 |
| US-100 3 Trig / Echo | 24 / 25 |
| US-100 4 Trig / Echo | 5 / 6 |

Pi GPIO는 3.3V 입력입니다. 5V 엔코더/초음파 Echo를 직접 연결하지 말고 실제 출력 레벨을 확인해 레벨 변환합니다. 공통 GND, 모터·서보 별도 적정 전원, 전원 차단 가능한 스위치를 준비합니다. 모터/서보를 Pi GPIO나 MDD3A의 200mA 보조 출력으로 구동하지 않습니다. MDD3A 입력에 외부 풀다운을 두면 부팅 중 신호 부유를 줄일 수 있습니다.

GPIO14/15는 UART와 충돌합니다. `sudo raspi-config`에서 serial login shell을 끄고 이 프로젝트에서는 해당 UART를 사용하지 않습니다. GPIO9/10은 SPI0와 충돌하므로 SPI0를 사용하지 않습니다. GPIO18/19 등과 겹치는 I2S, PWM/audio overlay, 기타 프로세스도 끕니다. 라이다는 USB 어댑터를 사용합니다. `wiring_verified`는 이 확인을 마친 뒤에만 `true`로 바꿉니다.

### 최소 수동 운전 설정

전원을 끄고 바퀴를 띄운 상태에서 다음 값을 확인합니다. 임의 기본 최대치를 적용하는 자동 보정은 없습니다.

1. `drive.supply_max_v`: 완충 상태를 포함한 모터 전원 최대 측정값. 이 초기 버전은 **4~12V**만 허용합니다. MDD3A의 16V 허용과 12V 모터 정격을 혼동하지 않습니다.
2. `drive.pwm_limit`: 부하·전원 조건을 확인한 운용 상한, 0~1 비율. `null`은 구동 금지. UI 슬라이더는 이 상한을 올릴 수 없습니다. 이 값은 전류 제한기를 대신하지 않습니다.
3. `servo.supply_max_v`: 실제 최대 전압, 6~8.4V 범위.
4. `servo.min_us / center_us / max_us`: 링크가 걸리지 않는 실제 펄스 끝점과 중립. 500~2500µs 안에 있어야 하며 `min < center < max`. **자료의 500/2500을 조향 링크 끝점으로 그대로 복사하지 않습니다.** 링크를 분리하고 서보 테스터 등으로 중립/회전 방향을 확인한 뒤 바퀴 조향의 안전한 끝점을 측정합니다.
5. `servo.left_rad / right_rad`: 차량 바퀴가 중립에서 좌/우로 돌아갈 수 있는 양의 라디안 값. 실제 바퀴 각도를 각도기로 측정합니다. 서보축 각도와 다릅니다. 좌회전 명령이 반대면 `servo.direction:-1`.
6. `hardware_enabled:true`, `wiring_verified:true`. 서버를 재시작합니다. 센서만 먼저 확인하려면 하드웨어 활성화 후 모터/서보 한계값은 `null`로 유지하면 됩니다. PWM은 0이고 서보 펄스는 시작되지 않습니다.

모터 방향은 사용자의 요청대로 배선을 바꾸거나 `drive.motors[i].direction`을 `1/-1`로 바꾸면 됩니다. 소프트웨어 기준 양수는 차량 전진입니다. 양의 전진 때 엔코더 카운트도 증가하도록 **별도로** `encoder_direction`을 맞춥니다. 두 모터를 물리적으로 반대로 장착했다면 두 채널의 부호가 다를 수 있습니다.

MDD3A는 DIR/ENABLE 방식이 아닙니다. 정방향은 A에 PWM/B=0, 역방향은 A=0/B에 PWM, 정지는 A=B=0(브레이크)입니다. 방향 전환 시 출력 0까지 감속한 뒤 기본 150ms 대기합니다. 실제 브레이크 감속 시간·전류는 실차에서 확인해야 합니다.

### 트레일러 축 MT6701 ABZ와 별도의 모터 엔코더

사용자 확인: **MT6701은 DC 모터와 무관한 트레일러 축 각도 센서입니다.** DC 모터별 AB 엔코더와 별도로 읽고, 모터 RPM·주행거리·차량 Odometry·서보 측정각으로 사용하지 않습니다. 향후 노트북의 트레일러 자동주차 알고리즘에 독립적인 각도 입력으로 전달합니다.

MT6701은 첨부에 **A/B/Z**로 연결되어 있으므로 I2C/SSI 절대각 센서처럼 읽지 않습니다. A/B 양 에지의 x4 카운트로 부호 있는 회전을 계산하고, Z 상승 에지에서 인덱스 기준을 기록합니다. `config.json`의 `mt6701` 설정은 이 트레일러 축에만 적용됩니다. 센서 축과 트레일러의 실제 기계적 기준을 보정하기 전에는 상대각을 차량-트레일러 사이의 절대 꺾임각으로 해석하지 않습니다.

* `counts_per_rev:null`: raw count만 유효하며 각도는 `null`.
* `angle_rad`: 부팅 당시 상대 위치를 0으로 하는 `[0,2π)` 상대각.
* `index_angle_rad`: Z를 실제 한 번 감지한 뒤에만 유효한 Z 기준각. 부팅 직후 절대각은 없음.
* `index_offset_rad`: 물리적 Z와 원하는 기준의 실측 오프셋.
* 상태 메시지 키는 `sensors.mt6701`을 유지하며 `role:trailer_axis_angle`로 용도를 명시합니다. 이 객체에 모터용 RPM·주행거리 필드를 포함하지 않습니다.

모터 `counts_per_wheel_rev`는 바퀴/출력축을 정확히 여러 바퀴 손으로 돌리고, raw count 증가량을 회전수로 나누어 설정합니다. 자료의 11 line이 모터축 A채널 PPR이고 실제 감속비가 정확히 30이며 바퀴 직결이라면 `11×4×30=1320`이 후보입니다. 이 해석과 실제 기어비가 확인되지 않아 기본값으로 넣지 않았습니다. MT6701도 실제 설정 분해능을 확인하거나 정확히 한 바퀴의 x4 카운트를 측정합니다.

```text
wheel_turns = signed_count / counts_per_wheel_rev
distance_m = wheel_turns × π × 0.075
travel_m = total_absolute_edges / counts_per_wheel_rev × π × 0.075
RPM = Δcount × 60 / (counts_per_wheel_rev × Δt)
```

`distance_m`는 부호 있는 순변위, `travel_m`는 전·후진을 합친 누적 회전 거리입니다. 미끄러짐을 보정한 지상 이동거리가 아닙니다. 값은 서버 시작 이후 누적이며 재부팅 시 초기화합니다. RPM은 최소 100ms 창으로 계산하고 에지가 기본 400ms 이상 없으면 `null`로 표시합니다. 정지와 단선을 무펄스만으로 구별할 수 없어 0RPM으로 정상 판정하지 않습니다. 잘못된 quadrature 전이는 오류 카운트로 남고 해당 세션의 피드백 사용을 차단합니다.

### US-100

첨부에는 US-100의 거리/전압 데이터시트가 없습니다. `min_cm`, `max_cm`는 해당 모듈 사양과 검증한 유효 범위로 설정한 다음 `enabled:true`로 바꿉니다. Trig/Echo 펄스 모드로 설정하고 UART 모드 점퍼 상태를 확인합니다. 4개를 동시에 쏘지 않고 순차 측정하며 시작 간격은 최소 60ms, 기본 Echo timeout은 30ms입니다. 거리 계산에 343m/s를 사용하므로 온도 오차는 존재합니다. `timeout`, `echo_stuck_high`, `out_of_range`, `stale`은 값 `null`과 함께 전송합니다. 600ms 이상 갱신되지 않은 값도 정상 거리로 유지하지 않습니다. 센서가 전·후·좌·우 어느 방향인지는 첨부에 없어 UI에 번호로 표시합니다.

## 7. 수동 조종과 안전 동작

1. `조종권 획득` → `정지 잠금 해제` → `구동 준비`를 누릅니다. 전원 재투입/재시작 때 항상 MANUAL, 비상정지 잠금, 출력 0으로 시작합니다.
2. PWM 상한과 키 입력 크기를 조절합니다. WASD/방향키를 누르고 있을 때 전·후진/조향합니다. 왼쪽은 양의 조향(rad), 전진은 양의 속도/PWM입니다.
3. 키를 놓으면 목표 출력·조향이 0으로 돌아가며 설정한 기울기로 변합니다. 창 이탈/탭 숨김은 즉시 정지·구동 해제합니다. 버튼은 클릭 후 자동으로 포커스를 해제합니다. 슬라이더/선택상자에 포커스가 있을 때는 운전 키가 들어가지 않으므로 바깥을 클릭합니다.
4. `정지`는 즉시 모터 브레이크와 구동 해제, `비상정지`/Space는 추가로 잠금입니다. 비상정지는 관찰자도 보낼 수 있습니다. 재시작/잠금 해제만으로 움직이지 않습니다.
5. 소유자가 연결을 끊거나 상태 송신을 따라가지 못하면 정지합니다. 다른 클라이언트는 관찰만 가능하며, 소유자가 반납하거나 연결이 끊겨야 조종권을 얻습니다. 새 소유자가 모드를 직접 확인합니다.

서보는 정상 정지·비상정지 때 마지막 명령 위치를 유지하여 갑자기 중앙으로 꺾이지 않습니다. 프로그램 종료/독립 watchdog 발동 때는 펄스를 끕니다. 이것은 실제 조향각 확인이나 기계적 고정의 보장이 아닙니다. UI의 `measured_rad`는 센서가 없으므로 항상 `null`입니다.

기본 명령 만료 350ms, 제어 주기 약 20ms, 상태 전송 100ms, 독립 출력 watchdog 500ms입니다. 이 시간은 하드웨어 데이터시트 최대값이 아니라 변경 가능한 통신/운용 정책입니다. 느린 Wi-Fi에서 주행 안전을 위해 제한을 무작정 늘리지 않습니다. 명령의 증가 sequence와 마지막 수신 `server_time` 에코를 검사해 오래 대기한 명령을 거부합니다. 브라우저 재접속 시 자동 claim/arm은 하지 않습니다.

`hardware.py --guard`는 별도 프로세스로 pigpio에 접속하여 제어 루프 heartbeat 손실/부모 종료 때 모터를 0으로 만들고 서보를 끕니다. 따라서 앱 `kill -9` 때 pigpiod에 PWM이 그대로 남는 위험을 줄입니다. **커널 정지·pigpiod 자체 정지·전원/GPIO 고장까지 소프트웨어로 보장하지는 않습니다.** 실제 비상전원 차단을 마련하고, motor drive 전원을 끈 상태에서 watchdog 동작을 먼저 확인합니다.

### 속도 명령과 PI

수동 PWM은 엔코더 없이 작동합니다. 목표 속도(m/s) 방식은 다음 중 하나가 준비되어야 합니다.

* `pi_enabled:false`: 실측한 `open_loop_m_s_at_full_pwm`와 `max_speed_m_s` 필요. `PWM ≈ 목표 속도 / 보정 스케일`이며 **실제 속도를 보장하지 않는 개방루프 명령**입니다. 무부하 정격 RPM으로 이 스케일을 임의 계산하지 않습니다.
* `pi_enabled:true`: 두 모터의 유효 RPM과 실차 튜닝 `kp`, `ki`, `max_speed_m_s` 필요. 목표 m/s를 바퀴 RPM으로 환산하고 채널별 PI/anti-windup을 적용합니다. 이득 단위는 PWM/RPM, PWM/(RPM·s)입니다.

PI는 피드백이 없으면 초기 기동도 거부합니다. 최초 검증은 수동 PWM으로 유효 에지를 확보한 뒤 전환해야 합니다. 정지 상태에서 자동으로 PWM을 올려 피드백을 “찾는” 기동은 구현하지 않았습니다. 저속에서 에지 간격이 timeout보다 길다면 피드백은 사용 불가입니다. 피드백 손실은 비상정지 잠금과 PI 적분 초기화이며, 수동 PWM으로 자동 전환하지 않습니다. 사용자가 원인을 해결하고 모드/명령 방식을 선택한 후 잠금 해제·재무장해야 합니다.

속도 제어로 **회전 주행**하려면 `drive.wheelbase_m`(앞뒤 축간거리), `odom.track_m`, 실제 좌우 모터 인덱스와 `rear_wheel_geometry_verified`도 필요합니다. 차량 중심 목표 속도를 조향 명령에 따른 Ackermann 근사로 좌/우 후륜 목표에 배분합니다(`vL/R = v × (1 ∓ track·tan(steering)/(2·wheelbase))`). 이 조향각은 명령 기반 근사이며 측정각이 아닙니다. 이 기하 설정이 없으면 속도 방식의 조향 주행 명령을 거부하며 수동 PWM은 계속 사용할 수 있습니다.

## 8. Odometry 준비와 SLAM 연결

`odom.enabled`는 기본 false입니다. MT6701 상대각 하나나 모터 PWM 추정값으로 가짜 `/odom`을 발행하지 않습니다. 아래 구현은 **모터 엔코더 2개가 각각 좌우 후륜의 실제 회전을 측정하는 경우**입니다. 다른 구동 구조라면 활성화하지 않습니다.

1. 두 모터의 `counts_per_wheel_rev`와 전진 부호 보정.
2. 좌우 후륜 접지점 중심 간 거리(윤거)를 측정하여 `odom.track_m` 설정. 앞뒤 축간거리와 다릅니다.
3. `left_motor/right_motor`에 인덱스 0/1 지정, `rear_wheel_geometry_verified:true`, `enabled:true`.
4. `base_link` 원점은 후륜 축 중앙, +x 전방/+y 좌측/+z 위. 라이다 위치와 방향을 측정해 `ros.laser_xyz_yaw:[x,y,z,yaw]`를 m/rad로 설정. 모두 0으로 가정하지 않습니다.

`ds=(dl+dr)/2`, `dθ=(dr-dl)/track`로 실제 후륜 두 바퀴 회전을 적분합니다. 조향 명령각을 측정각으로 사용하지 않습니다. 처음에는 두 센서의 실제 에지가 있어야 위치 적분을 시작합니다. 구동을 해제한 채 차체를 짧게 앞으로 밀어 양쪽 에지를 확인할 수 있습니다.

정지 명령이고 모터 출력도 0인 경우, 이미 실제 에지가 확인된 엔코더의 **변화 없는 카운트**로 정지 위치를 유지합니다. 이때 `position_quality:unchanged_at_commanded_rest`이며 무펄스만으로 단선/정지를 구별할 수 없습니다. 움직임을 요구하는데 피드백이 사라지면 Odometry를 중단하고 위치 연속성 오류를 잠급니다. 센서 복구만으로 이전 위치에 이어 붙이지 않습니다. 서버와 SLAM을 정지·재시작하고 기존 지도 재위치 추정 또는 새 매핑을 수행합니다. 이 초기 Odometry에는 IMU 융합/슬립 보정이 없고 covariance는 보수적인 초기 상수이며 실측 튜닝 대상입니다.

### 기존 slam.yaml 확인 — Raspberry Pi OS에서 파일 편집

기존 파일을 먼저 백업하고 기존 알고리즘 파라미터를 보존합니다.

```bash
cd ~/rc_lidar_docker
cp -n config/slam.yaml config/slam.yaml.before-controller
nano config/slam.yaml
```

기존 `slam_toolbox.ros__parameters` 아래 항목만 다음과 맞춥니다. 전체 파일을 이 조각으로 덮어쓰지 않습니다.

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    use_map_saver: true
    transform_publish_period: 0.02
```

동일 TF를 발행하는 기존 static `odom→base_link` 또는 별도 Odometry publisher가 있다면 동시에 실행하지 않습니다. `map→odom`은 SLAM Toolbox, `odom→base_link`는 이 bridge, `base_link→laser`는 측정된 static TF가 각각 담당합니다.

### Docker 실행 방법 A — 기존 컨테이너에 프로젝트 마운트가 있는 경우

Pi OS에서 기존 컨테이너를 시작한 뒤 기존 마운트의 controller 경로를 사용합니다. 라이다를 사용하는 기존 launch가 이미 실행 중이면 중복 실행하지 말고 먼저 종료합니다.

```bash
docker start 기존컨테이너
docker exec -it 기존컨테이너 bash /project/controller/run_ros.sh
```

컨테이너를 다른 사용자로 실행한다면 호스트 controller 사용자와 UID가 같아야 합니다. 기본 root 컨테이너는 공유 Unix 소켓에 접근할 수 있습니다. socket은 0600이며 `chmod 777`로 해결하지 않습니다.

### Docker 실행 방법 B — 공유 마운트/USB가 없는 경우

기존 컨테이너에는 생성 후 bind mount를 추가할 수 없습니다. **기존 이미지를 재사용하는 실행용 컨테이너만** 추가합니다. 기존 컨테이너 파일시스템 안에만 설치한 ROS 패키지가 있다면 먼저 정지된 컨테이너를 `docker commit 기존컨테이너 rc-ros-existing:local`로 보존하여 그 이미지를 사용할 수 있습니다. 이 경우에도 재설치는 하지 않습니다. bind mount의 내용은 commit에 포함되지 않으므로 기존 프로젝트 경로를 유지합니다.

**Pi OS 터미널:**

```bash
ls -l /dev/serial/by-id/
docker inspect 기존컨테이너 --format '{{.Config.Image}}'
# 아래 이미지 이름은 위 명령에서 확인한 값 또는 보존한 이미지로 교체
ROS_IMAGE='기존이미지:태그'
PROJECT_DIR="$HOME/rc_lidar_docker"
# 확인된 라이다 tty 장치로 바꿀 것
LIDAR_DEVICE='/dev/ttyUSB0'
docker run -d --name rc-ros-runtime --restart unless-stopped \
  --device "$LIDAR_DEVICE:/dev/ttyUSB0" \
  --mount "type=bind,source=$PROJECT_DIR,target=/project" \
  --env ROS_DOMAIN_ID=42 \
  "$ROS_IMAGE" bash /project/controller/run_ros.sh
docker logs -f rc-ros-runtime
```

기존 이미지 ENTRYPOINT가 ROS 진입을 막는 사용자 정의 프로그램이면 확인 후 `--entrypoint /bin/bash`로 바꾸고 이미지 뒤에는 `/project/controller/run_ros.sh`만 전달합니다. `/dev/ttyUSB0` 번호가 바뀌는 환경에서는 `/dev/serial/by-id/`의 해당 장치가 가리키는 실제 장치를 확인해 전달하고 컨테이너를 다시 생성해야 합니다. USB 재연결 후 기존 컨테이너 device 매핑이 자동으로 복구된다고 가정하지 않습니다.

이 실행 예시는 Docker 외부에 ROS 포트를 publish하지 않습니다. 모든 ROS 노드는 동일 컨테이너 안에서 통신합니다. ROS DDS를 노트북에 설치할 필요 없습니다. `config.json` 변경은 서버와 ROS launch를 모두 재시작해야 반영됩니다.

Odometry 설정이나 라이다 static TF 설정이 없으면 launch는 **C1 드라이버와 bridge만** 실행하고 SLAM Toolbox는 시작하지 않습니다. UI는 “미준비”를 표시합니다. SLAM이 실행되어도 실제 scan/odom/map/TF의 최신성이 없으면 정상으로 표시하지 않습니다. 상태의 `tracking:true`는 최신 ROS 데이터가 도착한다는 의미이며 scan matching 정확도를 보증하는 품질 점수는 아닙니다.

## 9. 지도 저장과 재사용 — Docker 내부 터미널

```bash
source /opt/ros/jazzy/setup.bash
source /project/ros2_ws/install/setup.bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /map --once --field info
ros2 service list | grep slam_toolbox
```

저장 이름은 기존 지도와 겹치지 않게 정합니다. PGM/YAML 지도와 pose graph는 목적이 다릅니다.

```bash
# 표시·지도 서버용 occupancy map (PGM/YAML)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/project/maps/garage_01'}}"
# 이후 계속 매핑/SLAM 재위치 추정용 pose graph + data
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/project/maps/garage_01_graph'}"
ls -lh /project/maps/
```

`save_map` 결과 `result:0`와 실제 생성 파일을 확인합니다. 기존 설치에서 map saver 실행 파일이 빠져 있다면 pose graph 저장을 우선 확인하고 해당 설치의 map saver 의존성을 점검합니다. 재설치 스크립트로 전체 ROS를 교체하지 않습니다.

**계속 매핑:** 정상 Odometry/scan을 확보한 매핑 노드에서 알려진 실제 초기 위치로 graph를 불러옵니다. 아래 0,0,0은 원점에 실제 놓았을 때만 사용합니다.

```bash
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/project/maps/garage_01_graph', match_type: 2, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

**저장 지도에서 위치 추정 전용:** 기존 `config/slam.yaml`을 별도 파일로 복사하여 `mode:localization`, `map_file_name:/project/maps/garage_01_graph`, 실제 `map_start_pose:[x,y,yaw]`를 설정합니다. `run_ros.sh` 실행 전에 `RC_SLAM_CONFIG=/project/config/localization.yaml`을 환경에 넣고 `rc.launch.py`의 SLAM launch를 `localization_launch.py`로 선택할 수 있도록 아래 `RC_SLAM_LOCALIZATION` 설정을 사용합니다. 두 SLAM 노드를 동시에 실행하지 않습니다. 기존 버전의 파라미터는 `ros2 param list /slam_toolbox`와 설치된 launch/config로 확인합니다. PGM/YAML만으로 SLAM pose graph를 복구할 수는 없습니다.

```bash
# Docker 내부에서, 기존 매핑 launch를 종료한 후
RC_SLAM_LOCALIZATION=1 RC_SLAM_CONFIG=/project/config/localization.yaml \
  bash /project/controller/run_ros.sh
```

자동 시작 컨테이너에 적용할 때는 같은 두 환경변수를 Docker `--env`로 전달합니다.

## 10. 공통 WebSocket 프로토콜 v1

`ws://PI_IP:8080/ws`, UTF-8 JSON. 상태/지도와 명령은 같은 연결로 왕복합니다. 계정/접속키 없음. JSON 숫자에 NaN/Infinity/boolean을 허용하지 않습니다. 제어 명령은 4KiB 이하입니다.

| type | 주요 필드 / 동작 |
|---|---|
| `claim` | `source:MANUAL` 또는 `AUTO_PARK`. 연결 단위 독점 조종권. 재claim으로 역할을 바꿀 수 없음 |
| `release` | 즉시 정지·구동 해제·조종권 반납 |
| `estop` | 모든 연결에서 요청 가능. 즉시 정지·잠금 |
| `reset_estop` | 소유자만 잠금 해제. 구동은 여전히 해제 |
| `arm` | 설정·모드·정지 잠금·AUTO 준비 확인 후 구동 준비 |
| `stop` | 정지·명령 초기화·구동 해제 |
| `mode` | `mode:MANUAL/AUTO_PARK`. 전환 시 정지·명령 초기화. 활성 source만 주행 가능 |
| `limit` | `pwm:0..설정상한`, 즉시 상한 축소. 설정 상한 초과 요청 거부 |
| `algorithm_status` | AUTO 소유자 `ready:true/false`. true는 500ms 미만 간격 갱신 필요 |
| `command` | 아래 형식. mode/source/seq/시간 검증. 잘못된 활성 제어 명령은 정지 |
| `state` | 서버→클라이언트 약 10Hz. 센서·명령·출력·모드·위치·준비 상태 |
| `map` | 신규 접속 또는 내용/metadata hash 변경 시에만 전송 |
| `ack`, `error` | 요청 처리 결과. error.message는 원인 문자열 |

수동 PWM도 동일 `command`와 중재기를 거치지만 AUTO에서는 `control:pwm`을 거부합니다.

```json
{"v":1,"type":"command","source":"MANUAL","seq":1,"server_time":1234.5,"control":"pwm","throttle":0.1,"steering_rad":0.0}
```

공통 차량 속도/조향 명령:

```json
{"v":1,"type":"command","source":"AUTO_PARK","seq":2,"server_time":1234.6,"control":"speed","speed_m_s":0.1,"steering_rad":0.0}
```

예제의 `server_time`을 그대로 보내지 않습니다. 직전 state에서 받은 실제 값을 에코합니다. 이는 비밀키/토큰이 아니며 PC/Pi 시계 동기화 없이 지연을 제한하기 위한 Pi monotonic timestamp입니다. `seq`는 해당 claim 동안 증가시킵니다. `speed_m_s`는 설정 속도 범위로 제한하며 조향 범위 초과는 오류입니다. 앞으로의 AUTO 알고리즘은 PWM 필드를 사용하지 않습니다.

state의 주요 데이터는 `sensors.motors[]`의 `pwm/target_pwm/rpm/count/turns/distance_m/travel_m/valid`, `sensors.mt6701`, `sensors.ultrasonic[]`의 `cm/status/age_s`, `servo.command_rad/target_rad/measured_rad/pulse_us`, `odom`, `ros.pose`, `ros.tracking`, `reason`, `configuration_errors`입니다. 정상 출력값 0과 미측정 `null`을 구분합니다. GPIO에 적용한 PWM과 목표 PWM은 모터축 실제 출력/토크 측정값이 아닙니다.

map은 `width`, `height`, `resolution`, `origin:[x,y,yaw]`, `frame:map`, `revision`, `encoding:base64_uint8_occupancy_plus_one`, `data`를 포함합니다. data를 base64 decode한 바이트에서 1을 빼면 ROS row-major occupancy입니다(`-1` unknown, `0..100` 확률). 화면의 y축 반전과 지도 원점 회전은 UI에서 처리합니다. 기본 최대 200만 셀입니다. 지도는 변경 시만 전송하고, 오래된 지도는 흐리게 남기되 현재 위치 화살표는 숨깁니다.

### 향후 노트북 알고리즘 연결

**Windows/macOS 알고리즘 개발 환경에서만:**

```bash
python -m pip install 'aiohttp>=3.10,<4'
python laptop_client.py http://192.168.0.50:8080
```

이 명령은 읽기 전용 상태 모니터입니다. 자동주차를 시작하지 않습니다. 향후 알고리즘은 `VehicleClient`의 `claim()`, `algorithm_ready(True)`, `activate()`, `drive(speed_m_s, steering_rad)`, `stop()`을 직렬로 사용합니다. 10Hz 이상 상태와 최신 지도/pose를 확인하고, 준비 heartbeat와 drive를 350ms 명령 만료 이내로 반복합니다. `activate`는 알고리즘 사용자가 실행을 요청한 시점에만 호출하며 연결 자체로 자동 실행하지 않습니다. 예외/연결 종료는 차량을 정지시킵니다.

AUTO 활성화에는 `algorithm_installed:true`, `commissioned:true`, PI 사용, 알고리즘 ready heartbeat, 최신 양측 RPM, 유효 Odometry, 최신 scan/TF/지도, 4개 초음파 정상과 `obstacle_stop_cm` 초과가 모두 필요합니다. 현재 기본값은 false/null입니다. 모든 초음파 방향을 보수적으로 검사하여 어느 방향 장애물이든 AUTO를 차단합니다. MANUAL에는 장애물 자동 회피를 추가하지 않았습니다.

잘못된 위치를 정상 TF로 발행하는 경우까지 freshness만으로 검출할 수 없습니다. 향후 노트북 알고리즘은 scan matching 품질, 위치 점프, 경로 오차를 검사해 `ready:false`와 `stop`을 보내야 합니다. 이 검증 없이 `commissioned:true`로 바꾸지 않습니다. 무펄스 정지에서 PI 자동 기동을 하지 않으므로 AUTO의 독립 정지 출발은 후속 실차 피드백 검증/기동 정책 개발 대상입니다. 현재는 연결 인터페이스만 제공하는 단계입니다.

## 11. 전원 재공급 후 실행 단순화

먼저 수동 실행과 전원 차단 검증을 마친 뒤 호스트 서비스를 설치합니다. Pi OS 사용자 홈이 `/home/사용자명`인 일반적인 경우:

```bash
cd ~/rc_lidar_docker/controller
sed "s/RC_USER/$(id -un)/g" rc-controller.service > /tmp/rc-controller.service
sudo install -m 644 /tmp/rc-controller.service /etc/systemd/system/rc-controller.service
sudo systemctl daemon-reload
sudo systemctl enable --now rc-controller
systemctl status rc-controller --no-pager
journalctl -u rc-controller -f
```

홈 경로가 다르면 설치 전 `WorkingDirectory`와 `ExecStart`를 실제 경로로 수정합니다. systemd `KillMode=mixed`는 주 프로세스에 먼저 종료 신호를 보내고 독립 guard가 정지할 시간을 줍니다. Python 스크립트와 guard를 별도로 강제 종료하지 않습니다.

위 Docker 방법 B의 컨테이너는 `--restart unless-stopped`로 재부팅 후 시작합니다. Docker 서비스도 기존 설정에서 활성화되어 있는지 확인합니다. 방법 A를 쓴 경우 기존 컨테이너 재시작 정책/시작 command를 유지하면서 ROS launch를 자동 실행하는 기존 운용 설정을 사용하거나 방법 B로 옮깁니다. 단순 `docker exec`로 띄운 프로세스는 컨테이너 재시작 시 자동 재실행되지 않습니다.

```bash
sudo systemctl enable docker
# 설정 변경 후
sudo systemctl restart rc-controller
docker restart rc-ros-runtime
```

평소에는 전원 켜기 → Pi 부팅 대기 → 브라우저 접속 → 상태 확인 → 조종권/잠금 해제/구동 준비 순서입니다. 재부팅 뒤 자동 주행하지 않습니다.

## 12. 테스트와 단계별 실제 검증

**개발 Windows:**

```powershell
python -m unittest discover -s tests -v
python -m compileall -q core.py hardware.py server.py ros_bridge.py rc.launch.py laptop_client.py tests
```

현재 작업 환경에서 정책/계산 테스트 **20개 통과**, Linux+aiohttp 웹 통합 테스트 **2개는 환경 부족으로 건너뜀**. Python 구문과 브라우저 JavaScript 구문도 검사했습니다. GPIO/PWM 파형, Pi 성능, 실제 센서 및 Docker/ROS 실행 검증은 수행하지 않았습니다. 이 테스트들은 고정 입력의 단위 테스트이며 센서 시뮬레이터가 아닙니다.

**Pi OS:**

```bash
cd ~/rc_lidar_docker/controller
.venv/bin/python -m unittest discover -s tests -v
```

웹 통합 테스트는 하드웨어를 비활성화한 서비스로 실제 WebSocket의 독점 조종권/비상정지/연결 해제/미측정값을 검증합니다. GPIO를 구동하지 않습니다.

실차 검증 순서:

1. 모터 전원 분리: 핀 충돌, 3.3V 신호, 공통 GND, 실제 전원과 PWM/서보 파형 확인.
2. 바퀴 띄움: 출력 상한을 검증한 값으로 설정, 조향 링크 걸림 확인, 각 모터 전진 부호 확인.
3. 비상정지/키 해제/창 전환/브라우저 종료/Wi-Fi 단절 때 모터가 정지하고 재무장이 필요한지 확인. 제동 시간은 실제 측정.
4. 낮은 검증 출력에서 server 프로세스 종료 및 `kill -9` 후 guard가 출력 0을 만드는지 확인. 모터 전원 차단 스위치를 즉시 조작할 수 있게 준비. pigpiod/OS 자체 정지는 소프트웨어 보장 대상 아님.
5. 모터/MT6701 카운트 보정. 정·역 왕복, Z 통과 전후, 피드백 분리, 0RPM/미측정 구분 확인.
6. 초음파 4개 각각 측정/미반사/연결 해제/간섭 검증.
7. 수동 PWM 저속 운전 중 센서/지도 갱신 확인. UI 여러 창에서 동시에 조종 불가 확인.
8. Odometry 직선 1m·후진·회전 실측 비교. 위치 단절 때 SLAM 정상 표시가 꺼지는지 확인.
9. C1 scan 주기, TF 트리, SLAM 지도 생성·저장·재사용 확인.
10. PI는 마지막으로 튜닝. AUTO는 미래 알고리즘이 준비되기 전 활성화하지 않음.

## 13. 오류 해결

| 증상 | 확인/조치 |
|---|---|
| 브라우저 연결 안 됨 | IP/8080, Pi 서비스 로그, 같은 Wi-Fi, AP client isolation, network.bind가 현재 IP인지 확인 |
| HTTP 403 | allowed_subnets/allowed_hosts와 브라우저 주소 일치 확인. HTTPS 외부 페이지에서 연결하지 않음 |
| 구동 준비 불가 | UI configuration_errors, hardware_enabled/wiring_verified, 모터/서보 실측 한계, estop, source/mode 확인 |
| GPIO 연결 실패 | pigpiod 상태, 로컬 8888, Pi 4 지원 버전. root로 웹 서버를 실행하여 우회하지 않음 |
| 두 번째 서버가 시작 안 됨 | 중복 GPIO 소유 방지 flock. 기존 systemd 서비스와 수동 실행 중 하나만 사용 |
| 모터 반대로 회전 | 전원 끈 뒤 배선 변경 또는 motor.direction 변경. encoder_direction도 전진 count 증가 확인 |
| RPM/각도가 — | counts_per_rev, 실제 에지, signal 전압, UART/SPI 충돌. 무펄스는 정상 RPM으로 가정하지 않음 |
| PI 켜면 즉시 정지 | 유효 최신 양측 RPM 필요. 수동 PWM으로 피드백 검증. 정지 출발 자동 bootstrap 없음 |
| 초음파 timeout | US-100 펄스 모드/점퍼, Echo 레벨, 공통 GND, 거리 범위, 반사면/센서 간섭 |
| ROS bridge 연결 없음 | 공유 폴더에 runtime/ros.sock 존재, mount 경로, 호스트 UID 또는 컨테이너 root, 호스트 server 실행 |
| C1 scan 없음 | USB 장치 매핑, 460800, 안정된 5V/기동 전류, 단일 드라이버 사용. SDK 데모와 동시에 포트를 열지 않음 |
| SLAM NOT STARTED | odom.enabled와 실제 laser_xyz_yaw 설정. 가짜 static odom TF로 우회하지 않음 |
| map은 있지만 위치 없음 | scan/odom/TF 최신성, base_link/laser 프레임, use_sim_time:false, odom continuity fault 확인 |
| Odometry continuity lost | 차량 정지, 센서 원인 해결, host/SLAM 재시작, 재위치 추정. 임의로 이전 위치에 이어 붙이지 않음 |
| 맵이 너무 커서 연결 종료 | max_map_cells 제한, SLAM map resolution/영역 확인. 서버를 추가하지 않고 지도 크기/갱신량 조절 |
| 재부팅 후 Docker만 살아 있고 ROS는 없음 | docker exec 프로세스는 자동 재실행 안 됨. 컨테이너 main command를 run_ros.sh로 구성 |
| run_ros.sh 실행 오류 | Windows CRLF이면 `sed -i 's/\r$//' run_ros.sh`. `bash run_ros.sh`로 실행, 기존 workspace source 확인 |

## 참고한 공식 소스

첨부 사양을 우선 적용하고 변경 가능한 ROS/Python API는 다음 공식 소스를 확인했습니다.

* [SLAMTEC C1 ROS 2 launch](https://github.com/Slamtec/sllidar_ros2/blob/main/launch/sllidar_c1_launch.py)
* [SLAM Toolbox Jazzy async launch](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/launch/online_async_launch.py)
* [SLAM Toolbox 저장/복원 및 위치 추정 설명](https://github.com/SteveMacenski/slam_toolbox)
* [DeserializePoseGraph 서비스](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/srv/DeserializePoseGraph.srv)
* [SaveMap 서비스](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/srv/SaveMap.srv)
* [pigpio 공식 Python 구현/API](https://github.com/joan2937/pigpio/blob/master/pigpio.py)
