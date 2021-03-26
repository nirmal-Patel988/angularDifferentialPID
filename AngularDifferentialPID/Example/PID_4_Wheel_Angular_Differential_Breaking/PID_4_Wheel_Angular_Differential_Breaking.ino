#include <AngularDifferentialPID.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(52,50);
int curr_pwm = 255; 
int flag=0;
bool ln = false;
bool fl = true;
double kp = 15;
double ki = 0;
double kd = 0.07;
double prevm1 = 0;
double prevm2 = 0;
double prevm3 = 0;
double prevm4 = 10;
motor m1(19,13,9,10,11),m2(18,5,7,6,4),m3(20,26,23,22,2),m4(21,25,31,33,3);
diffPID pidf1(&m4,&m3);//(m3.in1,m3.in2,m4.in1,m4.in2,m3.pwm,m4.pwm);
diffPID pidf2(&m1,&m2);//(m1.in1,m1.in2,m2.in1,m2.in2,m1.pwm,m2.pwm);
diffPID pidl1(&m3,&m1);
diffPID pidl2(&m2,&m4);
bool isStop = false;
//IdiffPID base();
void forward()
{
  Serial.println("Forward");
  pidf1.disableBrake(true);
  pidf2.disableBrake(true);
  pidl1.disableBrake(true);
  pidl2.disableBrake(true);
  isStop = false;
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ;
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
    
}

void backward()
{
  Serial.println("Back");
  pidf1.disableBrake(true);
  pidf2.disableBrake(true);
  pidl1.disableBrake(true);
  pidl2.disableBrake(true);
  isStop = false;
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ;
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
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
  Serial.println("Left");
  pidf1.disableBrake(true);
  pidf2.disableBrake(true);
  pidl1.disableBrake(true);
  pidl2.disableBrake(true);
  isStop = false;
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ;
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
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
  Serial.println("Right");
  pidf1.disableBrake(true);
  pidf2.disableBrake(true);
  pidl1.disableBrake(true);
  pidl2.disableBrake(true);
  isStop = false;
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ;
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  
  m3.forward();
  m4.backward();
  m1.backward();
  m2.forward();
}
void clockwise()
{
  Serial.println("clockwise");
  pidf1.disableBrake(true);
  pidf2.disableBrake(true);
  pidl1.disableBrake(true);
  pidl2.disableBrake(true);
  isStop = false;
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ;
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
  
  Serial.println("anticlockwise");
  pidf1.disableBrake(true);
  pidf2.disableBrake(true);
  pidl1.disableBrake(true);
  pidl2.disableBrake(true);
  isStop = false;
  pidf1.setPWM(curr_pwm) ;
  pidf2.setPWM(curr_pwm) ;
  pidl1.setPWM(curr_pwm) ;
  pidl2.setPWM(curr_pwm) ;
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

void Stop(){
  Serial.println("Breaking");
  isStop = true;
  pidf1.enableBrake(true);
  pidf2.enableBrake(true);
  pidl1.enableBrake(true);
  pidl2.enableBrake(true);
  
  pidf1.setPWM(0) ;
  pidf2.setPWM(0) ;
  pidl1.setPWM(0) ;
  pidl2.setPWM(0) ;
  pidf1.reset();
  pidf2.reset();
  pidl1.reset();
  pidl2.reset();
  pidl1.enable = true;
  pidl2.enable = true;
  pidf1.enable = true;
  pidf2.enable = true;
  
//  m1.brake();
//  m2.brake();
//  m3.brake();
//  m4.brake();

}

void setup() {
 // pid1 = new diffPID(&m3,&m4);
 // pid2 = new diffPID(&m1,&m2);
  
  Serial.begin(9600);
    mySerial.begin(9600);
    m1.directional_offset = -1;
    m3.directional_offset = -1;
  pidf1.setTunings(kp,ki,kd);
  pidf2.setTunings(kp,ki,kd);
  pidl1.setTunings(kp,ki,kd);
  pidl2.setTunings(kp,ki,kd);
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
  Serial.println("System Started Waiting for command");
  //change
  forward();
  
}
int data = -1;
void loop() 
{
  //Serial.println("Hello");
  if(mySerial.available()>0)
  {
    data=mySerial.read();
    Serial.print("Data Recieved");
    Serial.println(data);
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
        
        curr_pwm = curr_pwm>255 ? 255:curr_pwm;
        
        curr_pwm = curr_pwm<0 ? 0:curr_pwm;
        if(!isStop){
          
          pidf1.setPWM(curr_pwm);
          pidf2.setPWM(curr_pwm);
          pidl1.setPWM(curr_pwm);
          pidl2.setPWM(curr_pwm);
        }
      }
      else if(data == 12)
      {
        Serial.println("Decreasing Speed");
        curr_pwm -=25;
        
        curr_pwm = curr_pwm>255 ? 255:curr_pwm;
        
        curr_pwm = curr_pwm<0 ? 0:curr_pwm;
        
      if(!isStop){
          
          pidf1.setPWM(curr_pwm);
          pidf2.setPWM(curr_pwm);
          pidl1.setPWM(curr_pwm);
          pidl2.setPWM(curr_pwm);
        }
      }
    }
  }
//
//  if(m1.getReadings()!=prevm1 || ln){
//    ln = true;
//    Serial.print(m1.getReadings());
//    Serial.print("<- m1,");
//    prevm1 = m1.getReadings();  
//  }
//  
//  if(m2.getReadings()!=prevm2 || ln){
//    ln = true;
//    Serial.print(m2.getReadings());
//    Serial.print("<- m2,");
//    prevm2 = m2.getReadings();  
//  }
//  if(m3.getReadings()!=prevm3 || ln){
//    ln = true;
//    Serial.print(m3.getReadings());
//    Serial.print("<- m3,");
//    prevm3 = m3.getReadings();  
//  }
//  if(m4.getReadings()!=prevm4 || ln){
//    ln = true;
//    Serial.print(m4.getReadings());
//    Serial.print("<- m4,");
//    prevm4 = m4.getReadings();  
//  }
//  if(ln){
//      Serial.println();
//      ln = false;  
//  }
  pidf1.compute();
  pidf2.compute();
  
  pidl1.compute();
  pidl2.compute();
//  Serial.print(m1.getReadings());
//  Serial.print(",");
//  Serial.print(m2.getReadings());
//  Serial.print(",");
//  Serial.print(m3.getReadings());
//  Serial.print(",");
//  Serial.println(m4.getReadings());
  //Serial.println(m2.getDirection());
  //Serial.println(m3.getDirection());
  //Serial.println(m4.getDirection());

}
