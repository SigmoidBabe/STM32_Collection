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


//Motor DepanKanan(PA_0, PB_0, PA_4); //Depan Kiri
Motor DepanKanan(PA_9, PB_10 , PA_8); //DepanKanan
//Motor DepanKanan(PC_8, PC_5, PA_12); //Belakang Kanan
//Motor DepanKanan(PA_1, PC_0, PC_1); //Belakang Kiri

//QEI DepanKananQEI(PC_10, PC_12, NC, 986); //Depan Kiri
QEI DepanKananQEI(PC_11, PD_2, NC, 624); //Depan Kanan
//QEI DepanKananQEI(PB_1, PB_15, NC, 986); //Belakang Kanan
//QEI DepanKananQEI(PA_15, PB_7, NC, 986);// Belakang Kiri

//25  Mei 2022
PID DepanKananPID(0.96, 0.1, 0.000001, 0.02);
//PID DepanKiriPID(0.85, 0.08, 0.000000000001, 0.02);
//PID BelakangKiriPID(0.496, 0.0565, 0.000000000001, 0.02); Belakang Kiri
//PID BelakangKananPID(0.5, 0.022, 0.00000000001, 0.02);

//2022
//PID DepanKananPID(0.65, 0.1, 0.0000000001, 0.02);  //Kc, Ti, Td belakang kiri



//PID DepanKananPID(0.8, 0.1, 0.0000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 1
//PID DepanKananPID(0.7, 0.1, 0.0000000000005, 0.02);  //Kc, Ti, Td MOTOR baru nomor 2
//PID DepanKananPID(0.8, 0.1, 0.00000002, 0.02);  //Kc, Ti, Td MOTOR baru nomor 3



//PID DepanKananPID(0.495, 0.1, 0.00000035, 0.09); //Kc, Ti, Td, RATE


int main() {
    //int DepanKananPulses  = 0; 
    pc.baud(9600);
    DepanKanan.period(0.01f);    

    DepanKananPID.setInputLimits(0, 3000);//Input  units: counts per second.
    DepanKananPID.setOutputLimits(0.0, 0.9);//Output units: PwmOut duty cycle as %.
    DepanKananPID.setMode(AUTO_MODE);

    int DepanKananPulses      = 0; //How far the left wheel has travelled.
    int DepanKananPrevPulses  = 0; //The previous reading of how far the left wheel
    //has travelled.
    float DepanKananVelocity  = 0.0; //The velocity of the left wheel in pulses per
    //second.
  

    wait(3); //Wait a few seconds before we start moving.

    
    DepanKananPID.setSetPoint(1000);

    t.start();
    while (t.read_ms() <= 3000) {
        
        /*DepanKanan.speed(0.5f);
        
        DepanKananPulses = DepanKananQEI.getPulses();
        
        pc.printf("\n\r %d " , DepanKananPulses);*/
        t2.start();
        

        DepanKananPulses = DepanKananQEI.getPulses();
        DepanKananVelocity = ((double)DepanKananPulses - (double)DepanKananPrevPulses) / 0.02;
        DepanKananPrevPulses = DepanKananPulses;
        //DepanKananVelocity = (DepanKananVelocity/210.0f)*60.0f;
        
        DepanKananPID.setProcessValue(fabs(DepanKananVelocity));

        //----------//
        DepanKanan.speed(DepanKananPID.compute());

        
        pc.printf("\n %f " ,DepanKananVelocity );
    
        
        while (t2.read_ms() <= 20) {
        
        }
        
        t2.reset();

    }
    t.reset();
    DepanKanan.brake();

}
