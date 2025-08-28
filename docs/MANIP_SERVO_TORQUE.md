# Torque on Manipulator Servo No. 1

**This document refers specifically to the first version of the LIBRA-I manipulator, which uses the weaker TowerPro "MG996R" servos.**

After replacement of the USB webcam with the RealSense D456 and addition of the 2D LIDAR, the base tilt servo on the manipulator would rarely operate as expected. This document attempts to provide an explanation.

## Specifications

### *Servo*

- **Brand:** TowerPro
- **Series:** MG996R
- **Input Voltage:** 5.0 V
- **Operation Speed (@ 5V):** 0.18 s/deg
- **Stall Torque (@ 5V):** 0.95 Nm

### *Conditions*

- **Payload:** 0.365 kg
- **Payload Dist. (L x H):** 0.130 * 0.040 m

## Calculations

Absolute distance of payload from servo axis:
$r = \sqrt{(0.130)^2 + (0.040)^2} = 0.136~m$

Force exerted by combined mass of all components forward of servo no. 1:
$F = m * g = 0.365 * 9.81 = 3.580~N$

Torque exerted on servo:
$\tau = r * F = 0.136 * 3.580 = \bold{0.487~Nm}$

## Discussion

Although resulting torque on servo no. 1 is *around half* of its rated stall torque, in practice it is rarely able to move under these conditions. It seemingly spends most of its operational time in a "failed" state - this can be seen even when switching out the servo for a brand-new one. Occasionally, it attempts to recover: it may start twitching towards its previously commanded position or make clicking sounds (these can be resolved by lightly holding the rest of the manipulator in place).
