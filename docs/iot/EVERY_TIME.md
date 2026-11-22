# Testbed IoT: Every-Time Start

Do these in order each session. The one-time setup must be done first ([SETUP.md](SETUP.md)).

## 1. Laptop: broker (window 1, keep open)

`& "C:\Program Files\mosquitto\mosquitto.exe" -v -c "C:\Users\zakar\OneDrive - Balqa Applied University\Desktop\Z\2DOF-Drone-Testbed\firmware\pi\mosquitto\testbed.conf"`

It should print `Opening ipv4 listen socket on port 1883` and `Opening websockets listen socket on port 9001`.

## 2. Laptop: Node-RED (window 2, keep open)

`node-red.cmd`

## 3. Check the laptop IP

`ipconfig` → Wi-Fi **IPv4 Address**. If it is not 10.25.153.181, follow step 7 of SETUP.md.

## 4. Rig and Pi

1. Props off for the first test of the day.
2. Board USB-C port 2 into the Pi, power the Pi and the rig (PSU 12 V).
3. The bridge starts by itself. The Mosquitto window prints `New client connected from 10.25.155.231` within about 30 s of the Pi booting.
4. If it doesn't: Pi SSH `journalctl -u drone-bridge -f` to see why, or `sudo systemctl restart drone-bridge`.

## 5. Digital twin (optional, windows 3 and 4)

1. From the repo folder: `cd sim; python -m testbed_sim --gui ..\firmware\DRONE\Debug\DRONE.elf` and press **K1** in its window to arm the twin.
2. From the repo folder: `cd firmware\pi; python mqtt_bridge.py --port socket://localhost:5333 --prefix twin --broker localhost --http 0 --no-camera`

## 6. Open the dashboards

| Dashboard | Address |
|---|---|
| Node-RED | http://localhost:1880/dashboard/testbed |
| Node-RED editor (show the flow) | http://localhost:1880 |
| Web dashboard | http://10.25.155.231:8080/?broker=10.25.153.181 |
| Saved logs | http://10.25.155.231:8080/logs/ |

## 7. Run

1. Arm with **K1** on the board (6 s beep countdown). There is no remote arm.
2. Pitch: slider, or drag the dot in the **Digital twin** box. Yaw: yaw box, presets, or drag the yaw dot.
3. Camera modes: **Track pitch**, **Track yaw**, **Track both**; **Manual** to stop tracking.
4. Gains: Controller gains forms → Apply.
5. Logging: **Start log** / **Stop log**. Files go to `~/drone/pi/logs/` on the Pi. Copy all: `scp -r amrsaleh@10.25.155.231:~/drone/pi/logs .`
6. **DISARM MOTORS** stops the motors from either dashboard. Cutting the PSU is the hard stop.

## 8. Shut down

1. DISARM, then PSU off.
2. Pi: `sudo shutdown now` (wait for the green LED to stop) before pulling its power.
3. Close the Mosquitto and Node-RED windows (Ctrl+C).

## Quick fixes

| Symptom | Fix |
|---|---|
| Node-RED nodes say "connecting" | Mosquitto window closed: run step 1 again |
| Dashboard connected but no data | Bridge not reaching the broker: check laptop IP, `sudo systemctl restart drone-bridge` |
| Boxes appear twice in Node-RED | flow imported twice: rename `flows.json`, import once (SETUP.md step 3.5) |
| "Address already in use" / serial busy on the Pi | a second bridge is running: `pkill -f mqtt_bridge.py`, then `sudo systemctl restart drone-bridge` |
| PowerShell "scripts disabled" | use `npm.cmd` / `node-red.cmd` |
| Yaw tracking turns away from the ball | flip `YAW_SIGN` in `camera_track.py`, copy to Pi, restart bridge |
