# Board datasheets

Board: **Hiwonder ROS Robot Controller v1.2** (STM32F407VET6 @ 168 MHz, 16 MHz crystal, QMI8658 IMU).

| File | What it is |
|---|---|
| RRC Hardware Introduction.pdf | Board overview: ports, buttons, LEDs, power |
| RRC Schematic Explanation.pdf | Walk-through of each part of the schematic |
| RRC V1.1 Schematic.pdf | Full schematic |

**Careful:** Hiwonder only publishes the **V1.1** schematic. It shows the old IMU (MPU6050) and an 8 MHz crystal. Our board is v1.2 (QMI8658, 16 MHz). Use the schematic for wiring only, never for clock or IMU settings.

Still missing (not in the Hiwonder files): the STM32F407 datasheet, the RM0090 reference manual and the QMI8658 datasheet. Download them from st.com and qstcorp.com and add them here.
