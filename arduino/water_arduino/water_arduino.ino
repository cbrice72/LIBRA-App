// NOLINTBEGIN: don't lint Arduino C++ code

const int pin[4] = {2, 4, 7, 8};

/**
 * @brief Initializes variables, pin modes, libraries, etc.
 *
 * @note Required Arduino function (called once, at startup).
 */
void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(pin[i], OUTPUT);
        digitalWrite(pin[i], LOW);
    }
    pinMode(13, OUTPUT);
    Serial.begin(115200);
}

/**
 * @brief Main Arduino control function; begins running once `setup()` finishes.
 *
 * @note Required Arduino function (called repeatedly).
 *
 * @see setup
 */
void loop() {
    // 基準時刻の変数 - Time reference variable
    static unsigned long timestamp = 0;

    // 受信データがある場合 - If there is incoming data...
    if (Serial.available()) {
        byte data = Serial.read();  // ... 1バイト読み込む - read 1 byte
        for (int i = 0; i < 4; i++) {
            // i番目のピンの状態を、読み込んだデータの上からi番目のビットの状態にする
            // Set state of i-th pin to state of i-th bit (from beginning of data)
            digitalWrite(pin[i], bool(data & (1 << (3 - i))));
        }
        timestamp = millis();  // 現在を基準時刻とする - Update reference time
        digitalWrite(13, HIGH);  // ボード上のLEDを点灯する - Turn on board LED
    } else if ((millis() - timestamp)
               > 5000) {  // 受信データがない状態が5000ms続いた場合
                          // If no data has been received for 5000 ms...
        for (int i = 0; i < 4; i++) {
            digitalWrite(pin[i],
                         LOW);  // ... 全ピンをLOWにする - set all pins LOW
        }
        digitalWrite(13, LOW);  // ボード上のLEDを消灯する - Turn off board LED
    }
}

// NOLINTEND
