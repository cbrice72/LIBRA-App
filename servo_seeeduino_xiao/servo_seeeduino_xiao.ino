#include <Servo.h>
Servo sv_pitch;
Servo sv_pan;
Servo sv_tilt;

float floatmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {
  Serial.begin(115200);
  sv_pitch.attach(0);
  sv_pan.attach(1);
  sv_tilt.attach(2);
  sv_pitch.write(180);
  sv_pan.write(90);
  sv_tilt.write(0);
}

void loop() {
  if ( Serial.available() ) {
    String cmd = Serial.readStringUntil('\n');
    String cmds[3] = {"\0"};
    if (split(cmd, ' ', cmds,3) == -1)return;
   
    sv_pitch.writeMicroseconds((int)floatmap(180.0-cmds[0].toFloat(),0,180,400,2470));
    sv_pan.writeMicroseconds((int)floatmap(90.0-cmds[1].toFloat(),0,180,530,2530));
    sv_tilt.writeMicroseconds((int)floatmap(-cmds[2].toFloat(),0,180,420,2470));
  }
}

int split(String data, char delimiter, String *dst, int arraySize) {
  int index = 0;
  int datalength = data.length();
  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if ( tmp == delimiter ) {
      index++;
      if ( index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}
