# LIBRA-I Usage

A collection of detailed instructions and lessons learned from operating the LIBRA robot.

- [Extending the Arm in Kikura Lab (North Lab Bldg. 1)](#extending-the-arm-in-kikura-lab-north-lab-bldg-1)
- [Networking / Connecting to Actuators](#networking--connecting-to-actuators)

## Extending the Arm in Kikura Lab (North Lab Bldg. 1)

1. Plug in LIBRA power cables
2. Plug in LIBRA USB cable
3. Start LIBRA_App
    - This activates the HEBI motors so they can hold their position
4. Connect to serial ports/COMs via terminal (see `notes/Arduino COM IDs.txt`)
5. Disable water system (click `DISABLE`)
6. Remove tape supports
    - Around the furthest two links (1st floor)
    - Around the mast and counterweight links (2nd floor)
7. Slowly unfold LIBRA (see `notes/LIBRA-I Unfolding Angles.xlsx`)

<br>

# Troubleshooting

## Networking / Connecting to Actuators

To connect to HEBI actuators without a DHCP server (i.e., router), a static IP must be assigned to each actuator as well as the computer running the control app. See [notes/HEBI.md](notes/HEBI.md) for more information.
