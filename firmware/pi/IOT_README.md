# IoT dashboard (Pi side)

```
 STM32 board --USART3 ($TEL / $SET $YAW $DIS)--> mqtt_bridge.py --MQTT--> mosquitto broker
                                                   |                          |
                                                   |                 +--------+---------+
                                                   |                 |                  |
                                        web dashboard (port 8080)   Node-RED dashboard   ROS 2 node
                                                                    (port 1880)          (source:=mqtt)
```

`mqtt_bridge.py` is the only program that opens the serial port. Everything else
talks to it through MQTT, so several dashboards can watch and command at once.

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

## One-time setup on the Pi

Type each command on one line:

1. `sudo apt install -y mosquitto mosquitto-clients python3-paho-mqtt python3-serial`
2. `sudo cp ~/2DOF-Drone-Testbed/firmware/pi/mosquitto/testbed.conf /etc/mosquitto/conf.d/`
3. `sudo systemctl restart mosquitto`

## Every time

1. Plug the board's USB-C port 2 into the Pi, power the rig.
2. `cd ~/2DOF-Drone-Testbed/firmware/pi`
3. `python3 mqtt_bridge.py`
4. On the laptop/phone (same Wi-Fi): open `http://<pi-name>.local:8080`

Check the broker by hand: `mosquitto_sub -t "testbed/#" -v`

## Node-RED dashboard (second dashboard)

One-time:
1. `bash <(curl -sL https://github.com/node-red/linux-installers/releases/latest/download/update-nodejs-and-nodered-deb)`
2. `cd ~/.node-red && npm install @flowfuse/node-red-dashboard`
3. `sudo systemctl enable --now nodered`
4. Open `http://<pi-name>.local:1880`, menu -> Import -> select `nodered/testbed_flow.json` -> Import -> Deploy.

Then (with `mqtt_bridge.py` running) open `http://<pi-name>.local:1880/dashboard/testbed`.

## Testing without the rig

```
python3 fake_board.py                 # prints a port such as /dev/pts/3
python3 mqtt_bridge.py --port /dev/pts/3 --no-camera
```

## Credits (third-party files in `dashboard/vendor/`)

- Layout and styles: [Tabler](https://github.com/tabler/tabler) 1.4.0, MIT licence
- Font: IBM Plex Sans (via Fontsource 5.1.1), SIL Open Font Licence 1.1
- Charts: [Chart.js](https://www.chartjs.org/) 4.4.7, MIT licence
- MQTT in the browser: [MQTT.js](https://github.com/mqttjs/MQTT.js) 5.10.1, MIT licence
- Node-RED dashboard: [FlowFuse Dashboard 2.0](https://dashboard.flowfuse.com/), Apache 2.0
