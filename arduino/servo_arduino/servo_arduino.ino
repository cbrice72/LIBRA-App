// NOLINTBEGIN: don't lint Arduino C++ code

#include <Servo.h>

Servo sv_pitch;
Servo sv_pan;
Servo sv_tilt;

const int kDefaultPitch = 180;
const int kDefaultPan = 90;
const int kDefaultTilt = 0;

/**
 * @brief Initializes variables, pin modes, libraries, etc.
 *
 * @note Required Arduino function (called once, at startup).
 */
void setup() {
    Serial.begin(115200);

    // Attach each servo object to a specific pin
    sv_pitch.attach(0);
    sv_pan.attach(1);
    sv_tilt.attach(2);

    // Set default positions for each servo
    sv_pitch.write(kDefaultPitch);
    sv_pan.write(kDefaultPan);
    sv_tilt.write(kDefaultTilt);
}

/**
 * @brief Main Arduino control function; begins running once `setup()` finishes.
 *
 * @note Required Arduino function (called repeatedly).
 *
 * @see setup()
 */
void loop() {
    // Check for incoming commands from LIBRA App
    if (Serial.available()) {
        // Parse command data
        String cmd = Serial.readStringUntil('\n');
        String cmds[3] = {"\0"};
        if (split(cmd, ' ', cmds, 3) == -1) {
            return;
        }

        // Write new positions to each servo
        sv_pitch.writeMicroseconds(
            (int)mapfloat(float(kDefaultPitch) - cmds[0].toFloat(), 0, 180, 400,
                          2470));
        sv_pan.writeMicroseconds(
            (int)mapfloat(float(kDefaultPan) - cmds[1].toFloat(), 0, 180, 530,
                          2530));
        sv_tilt.writeMicroseconds(
            (int)mapfloat(float(kDefaultTilt) - cmds[2].toFloat(), 0, 180, 420,
                          2470));
    }
}

/**
 * @brief Parses an input string into one or more substrings based on delimiter.
 *
 * @param data Input data
 * @param delimiter Field separator in input data
 * @param dst Pointer to array in which to save output data
 * @param arraySize Size of output array
 * @return Number of elements parsed (-1 if error)
 */
int split(String data, char delimiter, String* dst, int arraySize) {
    int index = 0;
    int datalength = data.length();

    // Iterate over every character in input data
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        // If delimiter found...
        if (tmp == delimiter) {
            // ... move onto next index
            index++;
            if (index > (arraySize - 1)) {
                return -1;
            }
        } else {
            // ... else add char to current array index
            dst[index] += tmp;
        }
    }

    return (index + 1);
}

/**
 * @brief Re-maps a float value from one range to another.
 *
 * @param x Value to map
 * @param in_min Lower bound of `x`'s current range
 * @param in_max Upper bound of `x`'s current range
 * @param out_min Lower bound of target range
 * @param out_max Upper bound of target range
 * @return Mapped value
 */
float mapfloat(float x, float in_min, float in_max, float out_min,
               float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// NOLINTEND
