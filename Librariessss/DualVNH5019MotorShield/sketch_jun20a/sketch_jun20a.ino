#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;


int speed;

void setup()
{
Serial.begin(115200);
Serial.println("hello people");
md.init();
}

void loop()
{
md.setM1Speed(400);
delay(5000);
md.setM1Brake(100);
//delay(5000);

}
