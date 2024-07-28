#include <Servo.h>

Servo servo1;
Servo servo2;

int armPin = 0;
int armLed = 1;
bool armed;

void setup() {

  armed = false;
  servo1.attach(6);
  servo2.attach(7);
  servo1.write(0);
  servo2.write(0);

  pinMode(armPin, INPUT);
  pinMode(armLed, OUTPUT);
}

void loop() {
  if (digitalRead(armPin) == LOW)
  {
    armed = true;
  }
  if (armed == true && millis() % 1000 > 500)
  {
    digitalWrite(armLed, HIGH);
  }
  else
  {
    digitalWrite(armLed, LOW);
  }
}

void parachuteRelease()
{
  servo1.write(180);
  servo2.write(180);
}



