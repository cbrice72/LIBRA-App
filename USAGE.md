# LIBRA-II Usage

A collection of detailed instructions and lessons learned from operating the LIBRA robot.

1. [Extending the Arm in Kikura Lab (North Lab Bldg. 1)](#extending-the-arm-in-kikura-lab-north-lab-bldg-1)
2. [Troubleshooting](#troubleshooting)

## Extending the Arm in Kikura Lab (North Lab Bldg. 1)

1. Plug in LIBRA power cables
2. Plug in LIBRA USB cable
3. Start LIBRA_App
    - This activates the HEBI motors so they can hold their position
4. Connect to serial ports/COMs via (see `notes/Arduino COM IDs.txt`)
5. Disable water system (click `DISABLE`)
6. Remove tape supports
    - Around the furthest two links
    - Around the mast and counterweight links
7. Slowly unfold LIBRA (see `notes/LIBRA-I Unfolding Angles.xlsx`)

<br>

# Troubleshooting

## Network / Connecting to Actuators

To connect to HEBI actuators without a DHCP server (i.e., router), a static IP must be assigned to each actuator as well as the computer running the control app.

### *HEBI Actuators*

A static IP can be assigned through the HEBI Scope software by right-clicking an actuator in the device list and selecting "set address".
If the actuator does not show up or is inaccessible, follow the instructions [here](https://docs.hebi.us/core_concepts.html#connectionless-static-ip-reset) to reset it to the default address of `10.11.12.13`.

### *Windows*

Before setting a static IP, ensure that the actuator is powered on and connected via Ethernet.
Then, in Settings, navigate to "Network & Internet" -> "Ethernet" and select the actuator's network (may show up as "Unidentified Network" or something similar).
Under "IP settings", click "Edit" and manually assign a static IP according to the table below.

| Setting | Value | Notes |
|---|---|---|
| IP assignment | Manual |  |
| IP address | `10.11.12.___` | `___` can be anything (e.g., `2`) as long as it doesn't clash with the default actuator address (`10.11.12.13`) or other user-assigned actuator addresses |
| Subnet prefix length | 24 | i.e., `255.255.255.0` |
| Gateway | `10.11.12.1` | Doesn't matter, but required |

## Resetting Misaligned Actuators

1. Open the HEBI Scope app
2. Select the misaligned actuator
3. Go to Monitoring and set "Position" to 0
4. Manually straighten the arm
    - Go to the ground floor and align it (rotate the motors) by hand
5. Go to Dashboard -> Advanced
    1. Set "Position Offset" to the opposite value
    2. Set "Effort" to 0
