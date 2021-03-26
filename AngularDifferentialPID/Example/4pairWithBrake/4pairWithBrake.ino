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
void forward()
{
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enable = true;
  pidf2.enable = true;
  pidl1.enable = false;
  pidl2.enable = false;
  m1.forward();
  m2.forward();
  m3.forward();
  m4.forward();
  pidf1.enableBrake(false);
  pidf2.enableBrake(false);
  pidl1.enableBrake(false);
  pidl2.enableBrake(false);
  
}

void backward()
{
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enableBrake(false);
  pidf2.enableBrake(false);
  pidl1.enableBrake(false);
  pidl2.enableBrake(false);
  pidf1.enable = true;
  pidf2.enable = true;
  pidl1.enable = false;
  pidl2.enable = false;
  m3.backward();
  m4.backward();
  m1.backward();
  m2.backward();
}
void left()
{
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enableBrake(false);
  pidf2.enableBrake(false);
  pidl1.enableBrake(false);
  pidl2.enableBrake(false);

  pidl1.enable = true;
  pidl2.enable = true;
  pidf1.enable = false;
  pidf2.enable = false;
  
  m3.backward();
  m4.forward();
  m1.forward();
  m2.backward();
}
void right()
{
  pidf1.enableBrake(false);
  pidf2.enableBrake(false);
  pidl1.enableBrake(false);
  pidl2.enableBrake(false);
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidl1.enable = true;
  pidl2.enable = true;
  pidf1.enable = false;
  pidf2.enable = false;
  
  m3.forward();
  m4.backward();
  m1.backward();
  m2.forward();
}
void clockwise()
{
  pidf1.enableBrake(false);
  pidf2.enableBrake(false);
  pidl1.enableBrake(false);
  pidl2.enableBrake(false);
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enable = true;
  pidf2.enable = true;
  pidl1.enable = false;
  pidl2.enable = false;
  m1.forward();
  m3.forward();
  m2.backward();
  m4.backward();
}

void anticlockwise()
{
  pidf1.enableBrake(false);
  pidf2.enableBrake(false);
  pidl1.enableBrake(false);
  pidl2.enableBrake(false);  
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enable = true;
  pidf2.enable = true;
  pidl1.enable = false;
  pidl2.enable = false;
  m4.forward();
  m2.forward();
  m1.backward();
  m3.backward();
}

void Stop()
{
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidf1.enable = false;
  pidf2.enable = false;
  pidl1.enable = false;
  pidl2.enable = false;
  m1.stop();
  m2.stop();
  m3.stop();
  m4.stop();
}

void setup() {
 // pid1 = new diffPID(&m3,&m4);
 // pid2 = new diffPID(&m1,&m2);
  
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
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(31,OUTPUT);
  pinMode(33,OUTPUT);

  // m1.setAngular_offset(-45);
  // m2.setAngular_offset(+45);
  // m3.setAngular_offset(+45);
  // m4.setAngular_offset(-45);
  
}
int data = -1;
void loop() 
{
  //Serial.println("Hello");
  if(Serial2.available()>0)
  {
    data=Serial2.read();
    //if(millis()-st>100 || data==255)
    {
      if(data==1)
        forward();
      else if(data==5)
        backward();
      else if(data==3)
        right();
      else if(data==7)
        left();
      else if(data==255)
        Stop();
      else if(data==9)
        clockwise();
      else if(data==10)
        anticlockwise();
      else  if(data==11)
      {
        Serial.println("Increasing Speed");
        curr_pwm+=25;
        
        curr_pwm = curr_pwm>200 ? 200:curr_pwm;
        
        curr_pwm = curr_pwm<0 ? 0:curr_pwm;
        pidf1.setPWM(curr_pwm);
        pidf2.setPWM(curr_pwm);
        pidl1.setPWM(curr_pwm);
        pidl2.setPWM(curr_pwm);
      
      }
      else if(data == 12)
      {
        Serial.println("Decreasing Speed");
        curr_pwm -=25;
        
        curr_pwm = curr_pwm>200 ? 200:curr_pwm;
        
        curr_pwm = curr_pwm<0 ? 0:curr_pwm;
        pidf1.setPWM(curr_pwm);
        pidf2.setPWM(curr_pwm);
        pidl1.setPWM(curr_pwm);
        pidl2.setPWM(curr_pwm);
      }
    }
  }
  pidf1.compute();
  pidf2.compute();
  
  pidl1.compute();
  pidl2.compute();

}
