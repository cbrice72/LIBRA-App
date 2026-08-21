/******************************************************************************
 * @file   water_arduino.ino
 * @brief  Sketch for controlling up to two combined water inflow/outflow systems.
 *
 * @author Yuto Goto (original)
 * @author Christian Brice (modifications)
 ******************************************************************************/

// NOLINTBEGIN: don't lint Arduino C++ code

/* --- TABLE OF CONTENTS ---
 * !Main Functions
 * !Local Helpers
 */

// Settings
const long BAUD_RATE = 115200;
const char DEVICE_ID[] = "water";

const int HANDSHAKE_DELAY_MS = 2000;
bool handshake_done = false;  // resets on reconnection

const unsigned long STALE_INPUT_TIMEOUT_MS =
    5000;  // safety feature: turn off if no commands received for this long

// Pin Assignments
const int OUT_PINS[4] = {2, 4, 7, 8};  // two pins for up to two systems
const int LED_PIN = 13;                // on-board LED

//------------------------------------------------------------------------------
// !Main Functions
//------------------------------------------------------------------------------

/**
 * @brief Initializes variables, pin modes, libraries, etc.
 *
 * @note Required Arduino function (called once, at startup).
 */
void setup() {
    for (int i = 0; i < 4; ++i) {
        pinMode(OUT_PINS[i], OUTPUT);
        digitalWrite(OUT_PINS[i], LOW);
    }
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // start with LED off

    Serial.begin(BAUD_RATE);
}

/**
 * @brief Main Arduino control function; begins running once `setup()` finishes.
 *
 * @note Required Arduino function (called repeatedly).
 *
 * @see setup
 */
void loop() {
    static unsigned long time = 0;  // timestamp

    // Handshake with LIBRA App before accepting any commands
    if (!handshake_done) {
        if (Serial.available() && Serial.read() == '?') {
            // Reply to device ID query
            Serial.println(DEVICE_ID);
            delay(HANDSHAKE_DELAY_MS);
            handshake_done = true;

            // Empty the serial buffer before continuing
            while (Serial.available()) {
                Serial.read();
            }

            time = millis();  // avoid stale timestamp disconnect
        }
        return;  // don't do work before successful handshake
    }

    // Accept incoming commands
    if (Serial.available()) {
        byte data = Serial.read();

        // Process command
        for (int i = 0; i < 4; ++i) {
            // Set state of i-th pin to state of i-th bit
            digitalWrite(OUT_PINS[i], bool(data & (1 << (3 - i))));
        }
        digitalWrite(LED_PIN, HIGH);

        time = millis();

    } else if ((millis() - time) > STALE_INPUT_TIMEOUT_MS) {
        // Set all pins low if it's been a while since the last command
        for (int i = 0; i < 4; ++i) {
            digitalWrite(OUT_PINS[i], LOW);
        }
        digitalWrite(LED_PIN, LOW);
    }
}

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

// (none)

// NOLINTEND
