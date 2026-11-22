# Testbed IoT: One-Time Setup

Do this once per machine. After that, use [EVERY_TIME.md](EVERY_TIME.md).

**Who runs what**

| Machine | Runs | Address |
|---|---|---|
| STM32 board | firmware (bsp_comm: $TEL out, commands in) | USB-C port 2 into the Pi |
| Raspberry Pi | mqtt_bridge.py (serial ↔ MQTT, camera, logs, web dashboard :8080) | 10.25.155.231, user amrsaleh |
| Laptop | Mosquitto broker (1883, 9001) + Node-RED (1880) + optional simulator twin | 10.25.153.181 (check with ipconfig) |

All commands are one line each. Laptop commands go in **PowerShell** (prompt starts with PS). Steps marked *admin* need **Run as administrator**.

## 1. Laptop: get the code

1. GitHub Desktop: open 2DOF-Drone-Testbed, **Fetch origin**, then **Pull**.
2. Repo folder: `C:\Users\zakar\OneDrive - Balqa Applied University\Desktop\Z\2DOF-Drone-Testbed` (the single one, not the nested old copy).

## 2. Laptop: Mosquitto broker

1. Install Mosquitto (64-bit) from https://mosquitto.org/download/ if it is not installed yet. Check: `Test-Path "C:\Program Files\mosquitto\mosquitto.exe"` prints True.
2. *admin*: `Stop-Service mosquitto; Set-Service mosquitto -StartupType Manual` (we start it with our own settings instead).
3. *admin*: `New-NetFirewallRule -DisplayName "MQTT testbed" -Direction Inbound -Protocol TCP -LocalPort 1883,9001 -Action Allow`

## 3. Laptop: Node-RED

1. Install Node.js LTS from https://nodejs.org (defaults), then open a new PowerShell.
2. `npm.cmd install -g --unsafe-perm node-red` (use the .cmd names: Windows blocks the .ps1 versions).
3. Start it once: `node-red.cmd`, open http://localhost:1880.
4. ☰ → **Manage palette** → **Install** → search `@flowfuse/node-red-dashboard` → Install.
5. ☰ → **Import** → select a file → `firmware\pi\nodered\testbed_flow.json` → Import → **Deploy**. Import it only once; when updating later choose **Replace**. If boxes appear twice, stop Node-RED, run `Rename-Item $HOME\.node-red\flows.json flows_old.json`, start it and import once.
6. If the Pi's IP ever changes: double-click the **Camera picture** node and edit the address at the top.

## 4. Laptop: digital twin (optional)

1. `python -m pip install paho-mqtt pyserial`
2. `cd sim` then `python -m pip install -r requirements.txt`
3. Build the DRONE project once in CubeIDE so `firmware\DRONE\Debug\DRONE.elf` exists.

## 5. Pi: copy the code

1. Laptop PowerShell: `scp -r "C:\Users\zakar\OneDrive - Balqa Applied University\Desktop\Z\2DOF-Drone-Testbed\firmware\pi" amrsaleh@10.25.155.231:~/drone`
2. Use the IP, not dronepi.local (Windows can't resolve that name). The Pi and laptop must be on the same Wi-Fi.

## 6. Pi: packages and startup service

In the Pi SSH window (`ssh amrsaleh@10.25.155.231`):

1. `sudo apt install -y python3-paho-mqtt python3-serial python3-opencv`
2. The Pi does not run its own broker: `sudo systemctl disable --now mosquitto`
3. `sudo cp ~/drone/pi/systemd/drone-bridge.service /etc/systemd/system/`
4. `sudo systemctl daemon-reload`
5. `sudo systemctl enable --now drone-bridge`
6. Check: `journalctl -u drone-bridge -f` should show `Board on /dev/ttyACM0` and `bridge started` (Ctrl+C to leave).

From now on the bridge starts by itself every time the Pi boots.

## 7. If the laptop IP changes

1. Laptop: `ipconfig` → Wi-Fi **IPv4 Address**.
2. Pi: `sudo nano /etc/systemd/system/drone-bridge.service`, change `--broker`, save (Ctrl+O, Enter, Ctrl+X).
3. Pi: `sudo systemctl daemon-reload` then `sudo systemctl restart drone-bridge`
4. Web dashboard: click **Broker** (top right) and enter the new IP.

## 8. Updating after code changes

1. GitHub Desktop: Pull.
2. Repeat step 5 (scp).
3. Pi: `sudo systemctl restart drone-bridge`
4. If the Node-RED flow changed: Import → **Replace** → Deploy.

## Settings you may need to change

| File | Setting | What it does |
|---|---|---|
| `firmware/pi/camera_track.py` | `YAW_SIGN` | flip (1.0 / -1.0) if yaw tracking turns away from the ball |
| same | `PITCH_AXIS` | `"y"` so Track both uses up/down for pitch |
| same | `HSV_LOW`, `HSV_HIGH` | ball colour (blue now) |
| `firmware/pi/mqtt_bridge.py` | `PITCH_LIMIT`, `YAW_LIMIT` | largest references the dashboards may send (20°, 150°) |
