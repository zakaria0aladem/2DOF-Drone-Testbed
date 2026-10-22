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

`tbproto.py` (the frame format) and `examples/controller_template.py` are
unchanged. See the telemetry README for all the `tbbridge.py` options.
