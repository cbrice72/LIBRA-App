const int pin[4] = {2, 4, 7, 8};

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(pin[i], OUTPUT);
    digitalWrite(pin[i], LOW);
  }
  pinMode(13, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  static unsigned long timestamp = 0;           // 基準時刻の変数

  if ( Serial.available() ) {                   // 受信データがある場合
    byte data = Serial.read();                  // 1バイト読み込む
    for (int i = 0; i < 4; i++) {        
      digitalWrite(pin[i], bool(data & (1 << (3-i))));    // i番目のピンの状態を、読み込んだデータの上からi番目のビットの状態にする
    }
    timestamp = millis();                       // 現在を基準時刻とする
    digitalWrite(13, HIGH);                     // ボード上のLEDを点灯する
  }
  else if ((millis() - timestamp) > 5000) {     // 受信データがない状態が5000ms続いた場合
    for (int i = 0; i < 4; i++) {
      digitalWrite(pin[i], LOW);                // 全ピンをLOWにする
    }
    digitalWrite(13, LOW);                      // ボード上のLEDを消灯する
  }
}
