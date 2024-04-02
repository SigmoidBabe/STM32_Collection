#include "mbed.h"

#define i2c_address 0x60

int BEARING_Register;
char bits[2];
int _byteHigh;
int _byteLow;
int bearing;

I2C CMPS(PB_4, PA_8);
Serial pc(USBTX, USBRX);

int main()
{
    //bearing register in CMPS14 is 0x02(according to the datasheet)
    BEARING_Register = 0x02;
    pc.baud(9600);
    while(1)
    {
        CMPS.unlock();
        CMPS.start();
    // to indicate an i2c read, shift the 7 bit address up 1 bit and set bit 0 to a 1
        CMPS.write(i2c_address << 1); 
        int writeResult = CMPS.write(BEARING_Register);
        if(writeResult != 1)
        {
            pc.printf("%d\n", writeResult);
            return 0;
        }
        else
        {
            CMPS.stop();
            //read bearing value 
            CMPS.read(i2c_address <<1, bits, 2);
            _byteHigh = bits[0];
            _byteLow = bits[1];
            //combine 2bytes into single 16-bit integer
            bearing = (_byteHigh<<8) | _byteLow;
            pc.printf("Bearing : %d\n", bearing);
        }
    }
}

 
