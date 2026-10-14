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
    0.0020;  // Amps; low flow cut is 5% of FS (full scale) = 0.0025 A,
             // plus slightly lower to account for variability

// Pin Assignments
const int INFLOW_PIN = A0;
const int OUTFLOW_PIN = A1;
const int LED_PIN = 13;  // on-board LED

// Hardware Values
const float SENSOR_MIN_SIGNAL_A = 0.004;  // Amps
const float SENSOR_MAX_SIGNAL_A = 0.020;
/* NOTE:
 * [[    THE FOLLOWING VALUES MUST MATCH BOTH SENSORS' "Original Range"    ]]
 * [[  CONFIGURATION, OTHERWISE CALCULATED FLOW RATES WILL BE INCORRECT!!  ]]
 * The sensor is rated for 0.4-5.0 L/min, but will still output a signal outside
 * those bounds. From 5.0-5.5 L/min (110% * FS), the sensor outputs a warning but
 * continues to function. Above 5.5 L/min, the sensor errors. To ensure accurate
 * readings, the "Original Range" setting on each sensor is set to 0.4-5.0 L/min.
 */
const float SENSOR_RANGE_MIN_LPM = 0.4;  // liters/min
const float SENSOR_RANGE_MAX_LPM = 5.0;

const float SENSOR_MIN_FLOW_MLPS = (SENSOR_RANGE_MIN_LPM / 60.0)
                                   * 1000.0;  // milliliters/second
const float SENSOR_MAX_FLOW_MLPS = (SENSOR_RANGE_MAX_LPM / 60.0) * 1000.0;

const float RESISTOR_OHMS = 250.0;  // see calculation in "MATH" comment above

const float ADC_REF_VOLTAGE = 4.3;    // Arduino Nano Every stable logic level
const float ADC_RESOLUTION = 1024.0;  // 10-bit ADC

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

    // Use stable internal reference voltage (4.3V) for analog readings
    // (instead of the default Vcc (5.0V), which may fluctuate)
    /* NOTE:
     * - [ADC Ceiling]
     *     4.3V reference / 250 Ohm resistor = 17.2 mA max.
     *     This caps detectable flow rate at ~4.19 L/min.
     *     ---
     *     4.3V reference / 200 Ohm resistor = 21.5 mA max.
     *     This caps detectable flow rate at ~5.38 L/min,
     *     safely above the 5.00 L/min upper limit.
     * - [Inflow Rate]
     *     Manually controlled by valve at output end of sensor.
     *     Set to the 2.5 tick mark (50% closed) = ~3.95 L/min max.
     * - [Outflow Rate]
     *     Manually controlled by valve at output end of sensor.
     *     Set to the ?.? tick mark (??% closed) = ~?.?? L/min max;
     *     necessary since pump (Koshin MG-25-AAA-4) can suck more than twice
     *     the sensor's rated max. flow.
     */
    analogReference(INTERNAL4V3);

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
    // Accept incoming commands
    if (Serial.available()) {
        if (Serial.read() == '?') {
            // Reply to device ID query
            Serial.print('?');  // prepend magic to indicate query response
            Serial.println(DEVICE_ID);
        }
    }

    // Read analog values
    int raw_inflow = analogRead(INFLOW_PIN);
    int raw_outflow = analogRead(OUTFLOW_PIN);

    // Calculate and send digital values
    float in_v = (raw_inflow / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    float in_i = in_v / RESISTOR_OHMS;
    float inflow = CalculateFlow(in_i);

    float out_v = (raw_outflow / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    float out_i = out_v / RESISTOR_OHMS;
    float outflow = CalculateFlow(out_i);

    // Send compact packet
    Serial.print(inflow);
    Serial.print(',');
    Serial.println(outflow);  // incl. packet termination

    // Also show sensor connection status via on-board LED
    if (in_i >= FAULT_THRESHOLD_A && out_i >= FAULT_THRESHOLD_A) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    /*
    PrintDebugLine(raw_inflow, in_v, in_i, inflow,
                   raw_outflow, out_v, out_i, outflow);
    */

    delay(LOOP_PERIOD_MS);
}

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

/**
 * @brief Maps the flow sensor's current to a physical flow rate.
 *
 * @param current Sensor current (A)
 * @return Flow rate (mL/s), or -1.0 for fault
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
                  * (SENSOR_MAX_FLOW_MLPS - SENSOR_MIN_FLOW_MLPS)
                  / (SENSOR_MAX_SIGNAL_A - SENSOR_MIN_SIGNAL_A))
                 + SENSOR_MIN_FLOW_MLPS;

    // Reject values outside of the sensor's stated range
    if (flow < SENSOR_MIN_FLOW_MLPS) {
        flow = 0.0;  // disturbances may register as tiny flow rates; ignore them
    } else if (flow > SENSOR_MAX_FLOW_MLPS) {
        flow = SENSOR_MAX_FLOW_MLPS;
    }

    return flow;
}

/**
 * @brief Prints debug information for the flow sensors.
 *
 * @param raw_inflow Raw analog reading (ADC value)from inflow sensor
 * @param in_v Voltage reading from inflow sensor
 * @param in_i Current reading from inflow sensor
 * @param inflow Calculated flow rate from inflow sensor
 * @param raw_outflow Raw analog reading (ADC value) from outflow sensor
 * @param out_v Voltage reading from outflow sensor
 * @param out_i Current reading from outflow sensor
 * @param outflow Calculated flow rate from outflow sensor
 */
void PrintDebugLine(int raw_inflow, float in_v, float in_i, float inflow,
                    int raw_outflow, float out_v, float out_i, float outflow) {
    // Timestamp
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] ");

    // Inflow sensor metrics
    Serial.print("IN ADC: ");
    Serial.print(raw_inflow);
    Serial.print(" | V: ");
    Serial.print(in_v, 3);
    Serial.print(" | mA: ");
    Serial.print(in_i * 1000.0, 2);
    if (inflow == -1.0) {
        Serial.print(" | INFLOW: FAULT ");
    } else {
        Serial.print(" | INFLOW: ");
        Serial.print(inflow / 16.6667, 2);
        Serial.print(" L/min (");
        Serial.print(inflow, 2);
        Serial.print(" mL/s)");
    }

    // Separator
    Serial.print(" || ");

    // Outflow sensor metrics
    Serial.print("OUT ADC: ");
    Serial.print(raw_outflow);
    Serial.print(" | V: ");
    Serial.print(out_v, 3);
    Serial.print(" | mA: ");
    Serial.print(out_i * 1000.0, 2);
    if (outflow == -1.0) {
        Serial.println(" | OUTFLOW: FAULT");
    } else {
        Serial.print(" | OUTFLOW: ");
        Serial.print(outflow / 16.6667, 2);
        Serial.print(" L/min (");
        Serial.print(outflow, 2);
        Serial.println(" mL/s)");
    }
}

// NOLINTEND
