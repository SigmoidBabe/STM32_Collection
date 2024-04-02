/**
 * Drive a robot forwards or backwards by using a PID controller to vary
 * the PWM signal to H-bridges connected to the motors to attempt to maintain
 * a constant velocity.
 */
#include "mbed.h"
#include "Motor.h"
#include "QEI.h"
#include "PID.h"

Timer t;
Timer t2;

Serial pc(USBTX, USBRX); // tx, rx


//Initialize pin for dc motor
//(Forward Pin, Backward Pin, PWM Pin)
Motor DepanKanan(PA_9, PB_10 , PA_8); //DepanKanan

//Initialize Pin for motor encoder
//(Pin A, Pin B, NC, pulse per revolution)
QEI DepanKananQEI(PC_11, PD_2, NC, 624); //Depan Kanan

//Kc, Ti, Td, Sampling Period
//Tune these value for better PID control
PID DepanKananPID(0.96, 0.1, 0.000001, 0.02);


int main() {
    pc.baud(9600);
    //PWM Period
    DepanKanan.period(0.01f);    

    DepanKananPID.setInputLimits(0, 3000);//Input  units: counts per second.
    DepanKananPID.setOutputLimits(0.0, 0.9);//Output units: PwmOut duty cycle as %.
    DepanKananPID.setMode(AUTO_MODE);

    int DepanKananPulses      = 0; //How far the right wheel has travelled.(in pulse)
    int DepanKananPrevPulses  = 0; //The previous reading of how far the left wheel
    //has travelled.
    float DepanKananVelocity  = 0.0; //The velocity of the left wheel in pulses per
    //second.
  

    wait(3); //Wait a few seconds before we start moving.

    
    DepanKananPID.setSetPoint(1000);

    t.start();
    while (t.read_ms() <= 3000) {
        
        t2.start();
        //sampling start
        DepanKananPulses = DepanKananQEI.getPulses();
        DepanKananVelocity = ((double)DepanKananPulses - (double)DepanKananPrevPulses) / 0.02;
        DepanKananPrevPulses = DepanKananPulses;

        //assign velocity value to PID input
        DepanKananPID.setProcessValue(fabs(DepanKananVelocity));

        //----------//
        //set motor's pwm with PID output
        DepanKanan.speed(DepanKananPID.compute());

        
        pc.printf("\n %f " ,DepanKananVelocity );
    
        
        while (t2.read_ms() <= 20) {
        
        }
        //sampling reset
        t2.reset();

    }
    t.reset();
    DepanKanan.brake();

}
