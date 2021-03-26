#include "EncoderMotor/EncoderMotor.h"
#include <PID_v1.h>
class diffPID
{
public:
    bool stable = false;
    bool forward = false;
    bool brake = false;
    bool enable = false;
    motor *m1,*m2;
    double Kp=0, Ki=0, Kd = 0;
    double Setpoint = 0,Output = 0,Input = 0;
    int PWM = 0;
    PID *myPID =  new PID(&Input,&Output,&Setpoint,0,0,0,DIRECT);
    diffPID()
    {
        m1 = new motor();
        m2 = new motor();
    }
    double getReadings()
    {
        return (abs(m1->getReadings())+abs(m2->getReadings()))/2;
    }
    void setPWM(int p)
    {
        PWM = p;
    }
    void setTunings(double kp,double ki,double kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        myPID->SetTunings(kp,ki,kd);
    }
    void reset()
    {
        m1->reset();
        m2->reset();
    }
    diffPID(motor *mt1,motor *mt2)
    {
        m1 = mt1;
        m2 = mt2;
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(1);
        myPID->SetOutputLimits(-255,255);
    }
    void enableBrake(bool b=true)
    {
        brake = b;
    }
    void disableBrake(bool b=true)
    {   
        brake = !b;
    }
    void compute()
    {
        // Serial.println("computing");
        if(enable)
        {
            // Serial.println("enable");

            long value1, value2;
            // Serial.println();
            value1 = m1->getReadings();
            value2 = (brake) ? 0 : (m2->getReadings());
            Input = abs(abs(value1)-abs(value2))*(-1);
            // Serial.println(Input);
            if(brake)
            {
                //brake = m1->brake();
                //Serial.println(value1);    
               if(Input<-170){
                // myPID->SetTunings(0,0,0);
                stable = false;
                //Serial.println("HIGH DIFF");
                }
                else{
                  //  Serial.println("LOW DIFF");
                    stable = true;
                }
                if(value1 > 0)
                {
                    m1->backward();
                }
                else if(value1 < 0)
               {
                    m1->forward();
                }
            }
            else
            {        
                // Serial.println("Setting tuning");
                // Serial.println(Kp);
                        
                myPID->SetTunings(Kp,Ki,Kd);
            }
            
            myPID->Compute();
                        // Serial.print(',');

            // Serial.print(Output);
            if(abs(value1)>=abs(value2) && !brake)    
            {
                // Serial.println("1 is faster");                
                if(brake)
                {    
                    if(!stable)
                        m1->setPWM((PWM-Output)<0 ? 0:(PWM-Output));
                    else
                        m1->setPWM(1);
                }
                else if(!brake)
                {
                    m1->setPWM((PWM-Output)<0 ? 0:(PWM-Output));
                    m2->setPWM((PWM+Output)>255?255:(PWM+Output));
                    // Serial.print(Input);//POSITIVE ERROR
                    // Serial.print(',');
                    // Serial.print((PWM-Output)<0 ? 0:(PWM-Output));//LEFT OUTPUT
                    // Serial.print(',');
                    // Serial.print(PWM + ((Output<=(255-PWM))?Output:(255-PWM)));//RIGHT OUTPUT                
                }
            }
            else
            {
                if(brake)
                {    
                    if(!stable)    
                        m1->setPWM((PWM+Output)>255 ? 255:(PWM+Output));
                    else
                        m1->setPWM(1);
                }
                else
                    m1->setPWM((PWM+Output)>255 ? 255:(PWM+Output));
                if(!brake)
                    m2->setPWM((PWM-Output)<0 ? 0:(PWM-Output));
                // Serial.print(Input*(-1));//NEGATIVE ERROR
                // Serial.print(',');
                // Serial.print((PWM-Output)<0 ? 0:(PWM-Output));//LEFT OUTPUT
                // Serial.print(',');
                // Serial.print(PWM - ((Output<=PWM)?Output:PWM));//RIGHT OUTPUT
             }
            //Serial.println();
            if(value1 > 100000 || value2>100000)
            {
                m1->reset();
                m2->reset();
            }
        }
    }

};