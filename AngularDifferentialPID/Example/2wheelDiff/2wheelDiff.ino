#include <AngularDifferentialPID.h>
motor m1(19,13,10,9,11),m2(18,5,7,6,4);
diffPID d(&m1,&m2);
void setup() {
  // put your setup code here, to run once:
  m1.forward();
  m1.setDirection_angle(0);
  m1.setAngular_offset(0);
  m2.forward();
  m2.setDirection_angle(3.14*2/6);
  m2.setAngular_offset(0);
  Serial.begin(115200);
  d.enable = true;
  d.setTunings(1,1,1);
  d.setPWM(100);
}
double prev = 0;
void loop() {
  d.compute();
}
