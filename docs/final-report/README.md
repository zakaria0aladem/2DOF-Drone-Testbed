# 2-DOF Drone Testbed – MTR 520 Final Project

Amr Saleh (b00091495), Zakaria Aladem (b00113342)
MTR 520 Embedded Systems for Mechatronics, American University of Sharjah, Fall 2026

The testbed is a quad-X frame on two joints (pitch and yaw). An STM32F407 (Hiwonder ROS Robot Controller v1.2) reads the IMU and the two joint encoders and runs the pitch and yaw controllers at 500 Hz. A Raspberry Pi 5 is the gateway: it talks to the STM32 over USB serial, does colour tracking with the camera, and connects the rig to an MQTT broker. The dashboards (Node-RED and a web page) show live telemetry and send references, gains and disarm commands.

```
Dashboards  <--MQTT-->  Broker (laptop)  <--MQTT/Wi-Fi-->  Raspberry Pi  <--USB serial 115200-->  STM32  -->  ESC + 4 motors
                                                           (camera)                                ^
                                                                                          IMU + 2 encoders
```

## 1. Folder contents

| Folder | Contents |
|---|---|
| `01_Final_Report` | Final report (PDF and Word) |
| `02_STM32` | STM32CubeIDE project `DRONE` (open `DRONE.ioc` / import the folder). Our code is in `BSP/` and `Core/Src/main.c` |
| `03_Raspberry_Pi` | Pi scripts (`mqtt_bridge.py`, `link.py`, `protocol.py`, `camera_track.py`, `datalog.py`, ...), web dashboard, systemd service, `requirements.txt`, and `ros2_ws/` (ROS 2 bonus: Dockerfile, `ros.sh`, packages `testbed_ros` and `testbed_interfaces`) |
| `04_IoT` | Node-RED flow, Mosquitto config, dashboard files, dashboard screenshots |
| `05_Testing` | Result figures taken from the demo recording: dashboard, pitch steps, yaw steps, camera tracking and digital twin, ROS 2 (`fig_*.png`) |
| `06_Demonstration` | Backup demo videos: `IoT_Demo.mp4` (dashboards, camera tracking, digital twin) and `ROS_Demo.mp4` (ROS 2 + Foxglove) |

## 2. Hardware and wiring

| Item | Connection |
|---|---|
| 3S LiPo battery (12.6 V full) | Board screw terminal and 4-in-1 ESC (XT60) through the Y-cable, common ground |
| 4-in-1 ESC signals | M3 port: PB6 (ESC1), PB7 (ESC2). M4 port: PB4 (ESC3), PB5 (ESC4). Signal + GND only |
| Yaw encoder E8T-720 | M1 port: A = PA0, B = PA1 (TIM5), 5 V + GND from M1 |
| Pitch encoder E8T-720 | M2 port: A = PA15, B = PB3 (TIM2) |
| IMU QMI8658 | On the board, I2C2 (PB10/PB11) |
| Raspberry Pi 5 | USB cable from the Pi to the board's middle USB-C port (USART3). Shows up as `/dev/ttyACM0` |
| Camera Module 3 | Pi CSI port, fixed on the rig with the 3D-printed mount |
| ST-LINK | Board SWD pins (PA13/PA14), for flashing |

Never connect anything to pins 1 and 6 of the M ports (brushed motor outputs at battery voltage).

## 3. One-time setup

### 3.1 STM32 firmware

1. Install STM32CubeIDE. File > Import > Existing Projects into Workspace > select `02_STM32/DRONE`.
2. Build (hammer icon). Float printing must stay enabled (Project Properties > C/C++ Build > Settings > MCU Settings > "Use float with printf").
3. Connect the ST-LINK to the SWD pins, power the board, then Run > Run (or Debug) to flash.

Alternative without ST-LINK: flash `Debug/DRONE.bin` over USB port 1 with `stm32loader -p COMx -f F4 -R -B -e -w -v -g 0x08000000 DRONE.bin` (see `docs/FLASHING.md` in the repo). Do not use FlyMcu, it turns on read protection.

### 3.2 Laptop: broker and Node-RED (Windows, PowerShell)

1. Install Mosquitto (64-bit) from mosquitto.org. As administrator:
   `Stop-Service mosquitto; Set-Service mosquitto -StartupType Manual`
   `New-NetFirewallRule -DisplayName "MQTT testbed" -Direction Inbound -Protocol TCP -LocalPort 1883,9001 -Action Allow`
2. Install Node.js LTS, then `npm.cmd install -g --unsafe-perm node-red`.
3. Start `node-red.cmd`, open http://localhost:1880, Manage palette > Install `@flowfuse/node-red-dashboard`.
4. Menu > Import > `04_IoT/nodered/testbed_flow.json` > Deploy (import only once).

### 3.3 Raspberry Pi

1. Copy the Pi folder: `scp -r 03_Raspberry_Pi amrsaleh@<pi IP>:~/drone/pi`
2. Install packages: `sudo apt install -y python3-serial python3-paho-mqtt python3-opencv python3-picamera2`
   (or `pip install -r requirements.txt`, see the file for details).
3. The broker runs on the laptop, so stop the Pi's own: `sudo systemctl disable --now mosquitto`
4. Auto-start of the bridge: put the laptop IP after `--broker` in `systemd/drone-bridge.service`, then
   `sudo cp ~/drone/pi/systemd/drone-bridge.service /etc/systemd/system/`
   `sudo systemctl daemon-reload && sudo systemctl enable --now drone-bridge`
5. Check: `journalctl -u drone-bridge -f` should show the board port and `bridge started`.

## 4. Startup (every time)

1. Laptop, window 1 (broker): `& "C:\Program Files\mosquitto\mosquitto.exe" -v -c "<path>\04_IoT\mosquitto\testbed.conf"`
   It should print listen sockets on ports 1883 and 9001.
2. Laptop, window 2: `node-red.cmd`
3. Check both IP addresses, they change when a device reconnects to the Wi-Fi: laptop `ipconfig` (Wi-Fi adapter, IPv4), Pi `hostname -I` (first address). If the laptop IP changed, point the bridge at it on the Pi:
   `sudo sed -i -E 's/--broker [0-9.]+/--broker <laptop IP>/' /etc/systemd/system/drone-bridge.service`
   `sudo systemctl daemon-reload && sudo systemctl restart drone-bridge`
4. Place the rig at rest and keep it still. Plug in the battery (board and ESC) and power the Pi.
   The STM32 calibrates the gyro in the first second, so do not touch the rig during power-up.
5. The bridge starts by itself on the Pi. The Mosquitto window prints `New client connected` within about 30 s.
   To run it by hand instead: `cd ~/drone/pi && python3 mqtt_bridge.py --broker <laptop IP>`
6. Open a dashboard:
   - Node-RED: http://localhost:1880/dashboard/testbed
   - Web dashboard: `http://<pi IP>:8080/?broker=<laptop IP>`
   - Saved logs: `http://<pi IP>:8080/logs/`

## 5. Operating the rig

| Action | How |
|---|---|
| Arm motors | Press **K1** on the board. 6 s countdown (one beep per second), long beep when armed. There is no remote arm |
| Pitch reference | Slider in the dashboard (clamped to ±20° on the Pi, ±35° on the board) |
| Yaw reference | Yaw box or presets in the dashboard |
| Camera tracking | Mode buttons: Manual, Track pitch, Track yaw, Track both. Use a blue ball (HSV range in `camera_track.py`) |
| Change gains | Controller gains form > Apply. The board only accepts values inside the allowed ranges and confirms with `$GAIN` |
| Log data | Start log / Stop log. CSV files go to `~/drone/pi/logs/` on the Pi |
| Disarm | **DISARM MOTORS** button in either dashboard. Hard stop: unplug the battery |
| After a cutoff or IMU fault | Motors stop and disarm. Press K1 to clear the fault and arm again |

Shut down: disarm, unplug the battery, `sudo shutdown now` on the Pi, close the Mosquitto and Node-RED windows.

## 6. Communication summary

### 6.1 STM32 <-> Raspberry Pi (UART over USB, 115200 8N1)

Each message is one ASCII line `$<fields>*<CS>\r\n`, where CS is the XOR of all characters between `$` and `*` (2 hex digits). Example: `$SET,10.0*71`.

| Direction | Message | Rate | Content |
|---|---|---|---|
| Pi -> STM32 | `$SET,<deg>` | 10 Hz | Pitch reference, also the heartbeat |
| Pi -> STM32 | `$YAW,<deg>` | 10 Hz | Yaw reference (after one has been set) |
| Pi -> STM32 | `$DIS` | on demand | Disarm motors |
| Pi -> STM32 | `$GP,kp,ki,kd` / `$GY,kp,ki,kd` | on demand | New pitch / yaw gains |
| STM32 -> Pi | `$TEL,...` (18 fields) | 50 Hz | Angles, references, controller outputs, 4 motor pulses, state |
| STM32 -> Pi | `$GAIN,...` (6 gains) | 1 Hz and after a change | Gains in use |

Error handling: bad checksum, unknown command or invalid number -> line discarded and counted. Targets are clamped. No valid command for 500 ms -> `link_lost = 1` and the pitch reference returns to 0° (level).

### 6.2 MQTT topics (broker on the laptop, port 1883, WebSocket 9001)

| Topic | Direction | Content |
|---|---|---|
| `testbed/tel` | Pi -> dashboards | Each `$TEL` line as JSON, 50 Hz |
| `testbed/status` | Pi -> dashboards | Board connected, state, mode, logging, link counters, camera (2 Hz, retained) |
| `testbed/gains` | Pi -> dashboards | Gains in use (retained) |
| `testbed/event` | Pi -> dashboards | Text events (command accepted/rejected, link lost) |
| `testbed/cmd/pitch`, `testbed/cmd/yaw` | dashboard -> Pi | Reference in degrees |
| `testbed/cmd/mode` | dashboard -> Pi | `manual`, `camera`, `camera_yaw`, `camera_both` |
| `testbed/cmd/gains` | dashboard -> Pi | `{"axis": "pitch", "kp": 6, "ki": 2.5, "kd": 0.85}` |
| `testbed/cmd/log` | dashboard -> Pi | `start` / `stop` |
| `testbed/cmd/disarm` | dashboard -> Pi | Any payload |

Only one program may open the serial port. While `mqtt_bridge.py` runs, do not start `console.py`, `step_test.py` or `camera_track.py` (stop the service first: `sudo systemctl stop drone-bridge`).

## 7. ROS 2 interface (bonus)

The rig can also run as a ROS 2 robot. ROS 2 Jazzy runs in a Docker container on the Pi (Jazzy needs Ubuntu 24.04, the Pi runs Raspberry Pi OS). The ROS node uses the same `link.py` and `protocol.py` as the MQTT bridge, so the STM32 firmware and the serial protocol are unchanged.

```
ROS tools / other nodes  <--topics, service, params-->  angle_gate node  <--link.py, USB serial-->  STM32
                                                         |
                                     /joint_states --> robot_state_publisher --> /tf --> Foxglove 3D view (laptop)
```

Only one program may own the serial port: run **either** the MQTT bridge **or** ROS, never both.

### 7.1 One-time setup (Pi)

1. Install Docker: `curl -fsSL https://get.docker.com | sh`, then `sudo usermod -aG docker $USER` and log in again.
2. Build the image from `~/ros2_ws` (the folder with the `Dockerfile`): `docker build -t rig-ros .`
   The image is `ros:jazzy-ros-base` plus `python3-serial`, `ros-jazzy-foxglove-bridge`, `ros-jazzy-turtlesim` and `ros-jazzy-rqt-graph`.
3. `ros.sh` starts the container with the serial port, the workspace (`/ros2_ws`) and the Pi scripts (`/pi`) mounted, host networking and `ROS_DOMAIN_ID=42`.
4. Laptop: install the Foxglove app (foxglove.dev/download). No ROS is needed on the laptop.

### 7.2 Start (every time)

1. Pi: `sudo systemctl stop drone-bridge` (frees the serial port).
2. Pi: `hostname -I` (the Pi IP changes after a reboot).
3. Terminal 1: `cd ~/ros2_ws && ./ros.sh`, then inside the container
   `colcon build && source install/setup.bash`
   `ros2 launch /ros2_ws/src/testbed_ros/launch/rig.launch.py`
   This starts three nodes: `angle_gate` (our node, with its PID gains), `robot_state_publisher` (the URDF model) and `foxglove_bridge` (port 8765).
4. Terminal 2: `docker exec -it rig bash`, then `source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash`.
5. Laptop: Foxglove > Open connection > Foxglove WebSocket > `ws://<pi IP>:8765`. In the 3D panel set Display frame to `base_link` and turn on `/robot_description`.
6. Back to the dashboards afterwards: `docker stop rig`, then `sudo systemctl start drone-bridge`.

### 7.3 ROS interface of the `angle_gate` node

| Kind | Name | Type | Use |
|---|---|---|---|
| Publisher | `/testbed/pitch` | `std_msgs/Float64` | IMU pitch (°), 50 Hz |
| Publisher | `/testbed/yaw` | `std_msgs/Float64` | Encoder yaw (°), 50 Hz |
| Publisher | `/joint_states` | `sensor_msgs/JointState` | `yaw_joint`, `pitch_joint` in rad, time-stamped, for the 3D model |
| Subscriber | `/testbed/pitch_cmd` | `std_msgs/Float64` | Pitch reference (°) |
| Subscriber | `/testbed/yaw_cmd` | `std_msgs/Float64` | Yaw reference (°) |
| Service | `/testbed/disarm` | `std_srvs/Trigger` | Disarms the motors and replies with the board's confirmed state |
| Parameters | `pitch_kp`, `pitch_ki`, `pitch_kd`, `yaw_kp`, `yaw_ki`, `yaw_kd` | double | PID gains; a parameter callback sends `$GP` / `$GY` to the board when one changes |

`testbed_interfaces` defines the `MoveTo` action (goal: target and tolerance in degrees, timeout; feedback: current angle, error, elapsed time; result: success, final error, time). The action server is future work.

### 7.4 Examples (terminal 2)

| Action | Command |
|---|---|
| List nodes and topics | `ros2 node list`, `ros2 topic list` |
| Watch pitch | `ros2 topic echo /testbed/pitch` |
| Pitch reference 10° | `ros2 topic pub --once /testbed/pitch_cmd std_msgs/msg/Float64 "{data: 10.0}"` |
| Yaw reference 30° | `ros2 topic pub --once /testbed/yaw_cmd std_msgs/msg/Float64 "{data: 30.0}"` |
| Change a gain live | `ros2 param set /angle_gate pitch_kp 7.0` (always with a decimal point) |
| Disarm | `ros2 service call /testbed/disarm std_srvs/srv/Trigger` |

Arming stays on key K1, as with the dashboards. The 3D model (`urdf/rig.urdf`) is for display only: its sizes come from our 3D wiring model, and it moves with the real encoder and IMU angles.

## 8. Testing without the rig

- Fake board (Linux/Pi): `python3 fake_board.py` prints a port such as `/dev/pts/3`, then `python3 mqtt_bridge.py --port /dev/pts/3 --no-camera --broker <laptop IP>`.
- Digital twin (laptop): the simulator in `sim/` of the repo runs the same `DRONE.elf`. Start it with `python -m testbed_sim --gui <path>\DRONE.elf`, then `python mqtt_bridge.py --port socket://localhost:5333 --prefix twin --broker localhost --http 0 --no-camera`. The dashboards draw it as a second line.

## 9. Troubleshooting

| Problem | Fix |
|---|---|
| Dashboard shows "connecting" | Mosquitto window is closed: start it again (step 4.1) |
| Dashboard connected but no data | Laptop IP changed: check `ipconfig`, then run the `sed` and restart commands in step 4.3 |
| Serial port busy on the Pi | Two bridges running: `pkill -f mqtt_bridge.py`, then `sudo systemctl restart drone-bridge` |
| "Board not found" | USB cable must be in the board's middle USB-C port |
| IMU FAULT shown at start | Power-cycle the board with the rig still |
| Motors do not start after K1 | Wait for the long beep. Check the battery charge and that pitch is inside ±60° |
| Yaw tracking turns away from the ball | Flip `YAW_SIGN` in `camera_track.py`, restart the bridge |
| ROS node: `SerialException ... multiple access` | The bridge still owns the port: `sudo systemctl stop drone-bridge`, stop any old node, launch again |
| Foxglove: connection failed | Pi IP changed (`hostname -I`) or the launch file is not running |

## 10. Demonstration results (from the videos)

The values below were read from the live dashboard charts in `06_Demonstration/IoT_Demo.mp4` and `ROS_Demo.mp4`. Figures are in `05_Testing`; details are in Section 10 of the report.

| Test | What the video shows |
|---|---|
| Status | Board connected, motors ARMED, mode MANUAL, Pi -> board link ok |
| Pitch steps (gains 6 / 2.5 / 0.85) | 0 -> +20 -> -19 -> 0°. Rise under 1 s, held within 1° (-18.6° for -19°), settles in about 5-6 s |
| Yaw steps, gains 3 / 0 / 0.5 | -45 -> +90°: reaches 90° in about 3 s, peaks near 115°, then stays 10-20° short (no integral term) |
| Live gain change | Yaw gains changed from the dashboard to 2 / 0.5 / 2, confirmed by `$GAIN` |
| Yaw step, gains 2 / 0.5 / 2 | 90 -> 0°: undershoots to -17°, back within 2.2° after about 20 s |
| Camera, Track pitch | Ball found: Pi reference +12.8°, board +13.2°, rig +16.8°. Reference goes back to 0° when the ball is lost |
| Digital twin | Dragging the yaw dot sends a yaw reference; the real rig turns toward it |
| ROS 2 | Node and topic lists, `topic echo /testbed/pitch`, `pitch_cmd` ±10° (echo 10.5°), `yaw_cmd` 30° and back to 0°, Foxglove 3D model follows the real rig |

Not shown in the videos: the DISARM button, CSV logging, Track yaw / Track both results, the ROS parameter change and the ROS disarm service. These are implemented as described above.
