/******************************************************************************
 * @file   flow_arduino.ino
 * @brief  Sketch for analog-to-digital conversion of flow sensor data.
 *
 * @author Christian Brice
 ******************************************************************************/

// NOLINTBEGIN: don't lint Arduino C++ code

/* --- TABLE OF CONTENTS ---
 * !Main Functions
 * !Local Helpers
 */

/* MATH:
 * - Flow sensor (WFK2-005BABAA, CKD Corp.) outputs an analog signal of 4-20 mA.
 * - Arduino Nano Every's analog input pins are rated for 0-5 V.
 * - Necessary resistance calculated via Ohm's Law: R = V/I = 5/0.02 = 250 Ohms.
 */

// Settings
const long BAUD_RATE = 115200;
const char DEVICE_ID[] = "flow";

const float LOOP_PERIOD_MS = 500;  // 2 Hertz
const float FAULT_THRESHOLD_A =
    0.0038;  // Amps, slightly less than SENSOR_MIN_SIGNAL_A

// Pin Assignments
const int INFLOW_PIN = A0;
const int OUTFLOW_PIN = A1;
const int LED_PIN = 13;  // on-board LED

// Hardware Values
const float SENSOR_MIN_SIGNAL_A = 0.004;  // Amps
const float SENSOR_MAX_SIGNAL_A = 0.020;
const float SENSOR_MIN_FLOW_LPS = 0.4 / 60.0;  // Liters/second
const float SENSOR_MAX_FLOW_LPS = 5.0 / 60.0;

const float RESISTOR_OHMS = 250.0;  // see calculation in "MATH" comment above

const float ADC_REF_VOLTAGE = 4.3;   // Arduino Nano Every stable logic level
const float ADC_MAX_VALUE = 1023.0;  // 10-bit ADC

//------------------------------------------------------------------------------
// !Main Functions
//------------------------------------------------------------------------------

/**
 * @brief Initializes variables, pin modes, libraries, etc.
 *
 * @note Required Arduino function (called once, at startup).
 */
void setup() {
    pinMode(INFLOW_PIN, INPUT);
    pinMode(OUTFLOW_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // start with LED off

    analogReference(INTERNAL4V3);  // use stable internal 4.3V reference

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
    // Intercept device ID query
    if (Serial.available()) {
        byte data = Serial.read();
        if (data == '?') {
            Serial.println(DEVICE_ID);
        }
    }

    // Read analog values
    int raw_inflow = analogRead(INFLOW_PIN);
    int raw_outflow = analogRead(OUTFLOW_PIN);

    // Calculate and send digital values
    float in_v = (raw_inflow / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float in_i = in_v / RESISTOR_OHMS;
    float inflow = CalculateFlow(in_i);

    float out_v = (raw_outflow / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float out_i = out_v / RESISTOR_OHMS;
    float outflow = CalculateFlow(out_i);

    SendData(inflow, outflow);

    // Also show sensor connection status via on-board LED
    if (in_i >= FAULT_THRESHOLD_A && out_i >= FAULT_THRESHOLD_A) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    delay(LOOP_PERIOD_MS);
}

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

/**
 * @brief Maps the flow sensor's current to a physical flow rate.
 *
 * @param current Sensor current (A)
 * @return Flow rate (L/s), or -1.0 for fault
 *
 * @note Although the native Arduino `map()` function does exactly this, it uses
 *       integer math, thus truncating any precision we'd get from the sensor.
 */
float CalculateFlow(float current) {
    if (current < FAULT_THRESHOLD_A) {
        return -1.0;
    }

    // Linear interpolation: (X - X_min) * (Y_max - Y_min)
    //                       ----------------------------- + Y_min
    //                               X_max - X_min
    float flow = ((current - SENSOR_MIN_SIGNAL_A)
                  * (SENSOR_MAX_FLOW_LPS - SENSOR_MIN_FLOW_LPS)
                  / (SENSOR_MAX_SIGNAL_A - SENSOR_MIN_SIGNAL_A))
                 + SENSOR_MIN_FLOW_LPS;

    // Reject values outside of the sensor's stated range
    if (flow < SENSOR_MIN_FLOW_LPS) {
        flow = 0.0;  // disturbances may register as tiny flow rates; ignore them
    } else if (flow > SENSOR_MAX_FLOW_LPS) {
        flow = SENSOR_MAX_FLOW_LPS;
    }

    return flow;
}

/**
 * @brief Sends flow rates over serial connection (format: "inflow,outflow\n").
 *
 * @param inflow Calculated inflow rate (L/s)
 * @param outflow Calculated outflow rate (L/s)
 */
void SendData(float inflow, float outflow) {
    /* TODO: use this with LIBRA App
    // Send compact packet
    Serial.print(inflow);
    Serial.print(',');
    Serial.println(outflow); // incl. packet termination
    */

    // Print to terminal
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] ");

    Serial.print("INFLOW: ");
    if (inflow < 0.0) {
        Serial.print("FAULT");
    } else {
        Serial.print(inflow * 1000.0);  // convert to mL/s
        Serial.print(" (mL/s)");
    }

    Serial.print(" | OUTFLOW: ");
    if (outflow < 0.0) {
        Serial.print("FAULT");
    } else {
        Serial.print(outflow * 1000.0);  // convert to mL/s
        Serial.print(" (mL/s)");
    }

    Serial.println();
}

// NOLINTEND
