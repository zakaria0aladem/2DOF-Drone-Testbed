# Board datasheets

Board: **Hiwonder ROS Robot Controller v1.2** (STM32F407VET6 @ 168 MHz, 16 MHz crystal, QMI8658 IMU).

| File | What it is |
|---|---|
| RRC Hardware Introduction.pdf | Board overview: ports, buttons, LEDs, power |
| RRC Schematic Explanation.pdf | Walk-through of each part of the schematic |
| RRC V1.1 Schematic.pdf | Full schematic |
| referenceManual.pdf |reference manual for MCU|
| F407datasheet.pdf | datasheet for the MCU |
| QMI8658A_Datasheet_Rev_A | IMU datasheet |
**Careful:** Hiwonder only publishes the **V1.1** schematic. It shows the old IMU (MPU6050) and an 8 MHz crystal. Our board is v1.2 (QMI8658, 16 MHz). Use the schematic for wiring only, never for clock or IMU settings.


