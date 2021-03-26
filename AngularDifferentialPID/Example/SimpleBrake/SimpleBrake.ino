/*m1....m2
  .      .
  .      .
  .      .
  .      .
  m3....m4 */
#include <AngularDifferentialPID.h>
int curr_pwm = 150;
int flag=0;
motor m1(19,13,10,9,11),m2(18,5,7,6,4),m3(20,26,23,22,2),m4(21,25,31,33,3);
diffPID pidf1(&m4,&m3);//(m3.in1,m3.in2,m4.in1,m4.in2,m3.pwm,m4.pwm);
diffPID pidf2(&m1,&m2);//(m1.in1,m1.in2,m2.in1,m2.in2,m1.pwm,m2.pwm);
diffPID pidl1(&m3,&m1);
diffPID pidl2(&m2,&m4);

void brake()
{
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enable = true;
  pidf2.enable = true;
  pidl1.enable = true;
  pidl2.enable = true;
  pidf1.enableBrake();
  pidf2.enableBrake();
  pidl1.enableBrake();
  pidl2.enableBrake(); 
}

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  pidf1.setTunings(1,1,1);
  pidf2.setTunings(1,1,1);
  pidl1.setTunings(1,1,1);
  pidl2.setTunings(1,1,1);
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ; 
  brake();
}

int data = -1;
void loop() 
{
  //Serial.println("Hello");
  pidf1.compute();
  pidf2.compute();
  
  pidl1.compute();
  pidl2.compute();

}