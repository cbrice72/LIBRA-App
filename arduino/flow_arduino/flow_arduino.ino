/******************************************************************************
 * @file   flow_arduino.ino
 * @brief  Sketch for analog-to-digital conversion of flow sensor data.
 *
 * @author brice.c.aa
 ******************************************************************************/

/* MATH:
 * - Flow sensor (WFK2-005BABAA, CKD Corp.) outputs an analog signal of 4-20 mA.
 * - Arduino Nano Every's analog input pins are rated for 0-5 V.
 * - Necessary resistance calculated via Ohm's Law: R = V/I = 5/0.02 = 250 Ohms.
 */

/**
 * @brief Initializes variables, pin modes, libraries, etc.
 *
 * @note Required Arduino function (called once, at startup).
 */
void setup() {
    // TODO
}

/**
 * @brief Main Arduino control function; begins running once `setup()` finishes.
 *
 * @note Required Arduino function (called repeatedly).
 *
 * @see setup
 */
void loop() {
    // TODO
}
