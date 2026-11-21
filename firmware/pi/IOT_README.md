# IoT dashboard (Pi side)

```
 RIG                     PI (gateway)                 LAPTOP (server)              ANY BROWSER
 STM32 board --USB serial--> mqtt_bridge.py --MQTT 1883--> mosquitto broker --ws 9001--> web dashboard
 ($TEL 50 Hz /             (only program on               |                          (served by the Pi, :8080)
  $SET $YAW $DIS $GP $GY)   the serial port)              +--MQTT 1883--> Node-RED --> Node-RED dashboard
                                                                                       (laptop :1880/dashboard/testbed)
```

`mqtt_bridge.py` is the only program that opens the serial port. Everything else
talks to it through the broker on the laptop, so several dashboards can watch and
command at once. The ROS 2 node (`source:=mqtt`) is one more client of the same broker.

## Files

| File | Runs on | Job |
|---|---|---|
| `firmware/DRONE/BSP/bsp_comm.c/.h` | STM32 | serial protocol: sends `$TEL`/`$GAIN`, checks and applies `$SET $YAW $DIS $GP $GY`, link-loss timer |
| `mqtt_bridge.py` | Pi | MQTT client; serial <-> MQTT; command limits; status; CSV logging; camera mode; web server :8080 |
| `link.py` | Pi | `BoardLink`: opens the serial port, reads lines in a thread, sends commands 10x/s |
| `protocol.py` | Pi | line format, checksum, field names, builds and parses every message |
| `datalog.py` | Pi | writes telemetry to `logs/*.csv` |
| `camera_track.py` | Pi | colour tracking, used by the bridge in camera mode |
| `mosquitto/testbed.conf` | laptop | broker settings: port 1883 (programs) and 9001 (browsers) |
| `nodered/testbed_flow.json` | laptop | Node-RED flow = the Node-RED dashboard |
| `dashboard/index.html` + `vendor/` | browser | the web dashboard (an MQTT client itself) |
| `fake_board.py` | Pi/Linux | pretend STM32 for testing without the rig |

## MQTT topics

| Topic | Direction | Content |
|---|---|---|
| `testbed/tel` | Pi -> dashboards | every `$TEL` line as JSON (50 Hz), keys = `protocol.TEL_FIELDS` |
| `testbed/status` | Pi -> dashboards | JSON, 2 Hz, retained: board connected, motor state, mode, logging, link counters, camera |
| `testbed/event` | Pi -> dashboards | `{"t": time, "text": "..."}` (commands accepted/rejected, link lost...) |
| `testbed/cmd/pitch` | dashboard -> Pi | `"10"` pitch reference, deg, clamped to +/-20, ignored in camera mode |
| `testbed/cmd/yaw` | dashboard -> Pi | `"-45"` yaw reference, deg, clamped to +/-150 |
| `testbed/cmd/mode` | dashboard -> Pi | `"manual"` or `"camera"` |
| `testbed/cmd/log` | dashboard -> Pi | `"start"` or `"stop"` (CSV in `logs/`) |
| `testbed/cmd/disarm` | dashboard -> Pi | anything: disarms the motors (arming only with K1) |

## One-time setup

Laptop (Windows), PowerShell as administrator, one line each:
1. Install Mosquitto (64-bit installer from https://mosquitto.org/download/) with default options.
2. `Stop-Service mosquitto; Set-Service mosquitto -StartupType Manual` (we start it with our own settings instead)
3. `New-NetFirewallRule -DisplayName "MQTT testbed" -Direction Inbound -Protocol TCP -LocalPort 1883,9001 -Action Allow`
4. Install Node.js LTS from https://nodejs.org, then `npm install -g --unsafe-perm node-red`
5. `node-red` (leave it open), open http://localhost:1880, menu -> Manage palette -> Install -> `@flowfuse/node-red-dashboard`
6. Menu -> Import -> select `firmware/pi/nodered/testbed_flow.json` -> Import -> Deploy

Pi: `sudo apt install -y python3-paho-mqtt python3-serial` (and copy `firmware/pi` to `~/drone/pi`).
The Pi does not need its own mosquitto any more: `sudo systemctl disable --now mosquitto`.

## Every time

1. Laptop: `ipconfig` -> note the IPv4 address (example `10.25.155.40`). It can change between days.
2. Laptop, window 1: `& "C:\Program Files\mosquitto\mosquitto.exe" -v -c "<repo>\firmware\pi\mosquitto\testbed.conf"`
3. Laptop, window 2: `node-red`
4. Board USB-C port 2 into the Pi, power the rig.
5. Pi: `cd ~/drone/pi && python3 mqtt_bridge.py --broker <laptop IP>`
6. Node-RED dashboard: http://localhost:1880/dashboard/testbed
7. Web dashboard: `http://<pi IP>:8080`, click **Broker** (top right), enter the laptop IP once.

The window from step 2 prints every client that connects: that is the broker at work.

## Testing without the rig

```
python3 fake_board.py                 # prints a port such as /dev/pts/3
python3 mqtt_bridge.py --port /dev/pts/3 --no-camera --broker <laptop IP>
```

## Credits (third-party files in `dashboard/vendor/`)

- Layout and styles: [Tabler](https://github.com/tabler/tabler) 1.4.0, MIT licence
- Font: IBM Plex Sans (via Fontsource 5.1.1), SIL Open Font Licence 1.1
- Charts: [Chart.js](https://www.chartjs.org/) 4.4.7, MIT licence
- MQTT in the browser: [MQTT.js](https://github.com/mqttjs/MQTT.js) 5.10.1, MIT licence
- Node-RED dashboard: [FlowFuse Dashboard 2.0](https://dashboard.flowfuse.com/), Apache 2.0
