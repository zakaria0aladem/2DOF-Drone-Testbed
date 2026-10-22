# PC tools

Copies of the telemetry tools from the project (`telemetry/host`), with two
small changes so they also talk to the simulator:

* `tbbridge.py` opens ports with `serial.serial_for_url`, so `--port` takes
  `COM9` (real board) or `socket://localhost:5333` (simulator), and reads a
  socket in chunks instead of byte by byte.
* `--baud` defaults to 1000000, the Hiwonder firmware's USART3 speed.
* The handshake repeats PING every 0.25 s until the HELLO comes. The
  firmware's `tx_frame()` drops a frame when the previous one is still being
  sent, so at 500 Hz about one PING in three got no answer (on the real board
  too; the simulator showed it).

`tbproto.py` (the frame format) is unchanged.

`examples/controller_template.py` changed: the old `HoldAngle` used the motor
pattern (1, 1, -1, -1), which tilts the frame sideways (roll), a direction
the rig can't move in, so the yaw joint never turned. It now uses the yaw
pattern (1, -1, -1, 1) and has an integral term, because joint friction
stopped the PD short of the target. In the simulator it turns yaw to 0.5 rad
in about a second:

    python tbbridge.py run --port socket://localhost:5333 --arm --controller examples/controller_template.py:HoldAngle --duration 10

`python tbbridge.py log --help` and `python tbbridge.py run --help` list all
the options.
