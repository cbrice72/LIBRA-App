# Serial Device Identifiers

- [2D LIDAR](#2d-lidar)
- [WaterArduino](#waterarduino)
- [FlowArduino](#flowarduino)
- [ManipArduino](#maniparduino)

## 2D LIDAR

| Field | Value |
| --- | --- |
| Port | `ttyUSBx` |
| Description | USB-Serial Controller |
| Manufacturer | Prolific Technology Inc. |

## WaterArduino

Controls the counterweight water levels.

| Field | Value |
| --- | --- |
| Port | `ttyACMx` |
| Description | Arduino Nano Every |
| Manufacturer | Arduino LLC |
| Device ID | `"water"` |

## FlowArduino

Reads data from the flow sensors.

| Field | Value |
| --- | --- |
| Port | `ttyACMx` |
| Description | Arduino Nano Every |
| Manufacturer | Arduino LLC |
| Device ID | `"flow"` |
  
## ManipArduino

> ***NOTE:*** Currently UNUSED since the servos (Tower Pro MG-996R) were too weak to move the sensor suite, much less hold it upright.

Controls the three tip-mounted servos (pitch-yaw-pitch).

| Field | Value |
| --- | --- |
| Port | `ttyACMx` |
| Description | Seeed XIAO M0 |
| Manufacturer | Seeed |
| Device ID | N/A |
