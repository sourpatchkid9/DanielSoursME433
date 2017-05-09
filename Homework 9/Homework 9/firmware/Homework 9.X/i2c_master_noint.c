#include "i2c_master_noint.h"
#include <xc.h>
#define SLAVE_ADDR 0b1101011


void i2c_master_setup(void) {
I2C2BRG = 390; // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
// look up PGD for your PIC32
I2C2CONbits.ON = 1; // turn on the I2C2 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
I2C2CONbits.SEN = 1; // send the start bit
while(I2C2CONbits.SEN) { ; } // wait for the start bit to be sent
}

void i2c_master_restart(void) { 
I2C2CONbits.RSEN = 1; // send a restart 
while(I2C2CONbits.RSEN) { ; } // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
I2C2TRN = byte; // if an address, bit 0 = 0 for write, 1 for read
while(I2C2STATbits.TRSTAT) { ; } // wait for the transmission to finish
if(I2C2STATbits.ACKSTAT) { // if this is high, slave has not acknowledged
// ("I2C2 Master: failed to receive ACK\r\n");
}
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
I2C2CONbits.RCEN = 1; // start receiving data
while(!I2C2STATbits.RBF) { ; } // wait to receive the data
return I2C2RCV; // read and return the data
}

void i2c_master_ack(int val) { // sends ACK = 0 (slave should send another byte)
// or NACK = 1 (no more bytes requested from slave)
I2C2CONbits.ACKDT = val; // store ACK/NACK in ACKDT
I2C2CONbits.ACKEN = 1; // send ACKDT
while(I2C2CONbits.ACKEN) { ; } // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) { // send a STOP:
I2C2CONbits.PEN = 1; // comm is complete and master relinquishes bus
while(I2C2CONbits.PEN) { ; } // wait for STOP to complete
}

void init_IMU(){
ANSELBbits.ANSB2 = 0;
ANSELBbits.ANSB3 = 0;

i2c_master_setup(); // init I2C2, which we use as a master
__builtin_enable_interrupts(); 
char CTRL1_XL = 0b10000010;
char CTRL2_G = 0b1000100;
char CTRL3_C = 0b00000100;
//accelerometer
i2c_master_start();
i2c_master_send(SLAVE_ADDR<<1);
i2c_master_send(0x10);
i2c_master_send(CTRL1_XL);
i2c_master_stop();
//gyro
i2c_master_start();
i2c_master_send(SLAVE_ADDR<<1);
i2c_master_send(0x11);
i2c_master_send(CTRL2_G);
i2c_master_stop();
//multi read
i2c_master_start();
i2c_master_send(SLAVE_ADDR<<1);
i2c_master_send(0x12);
i2c_master_send(CTRL3_C);
i2c_master_stop();
} 
void setExpander(char pin, char level){

i2c_master_start();
i2c_master_send(SLAVE_ADDR << 1);
i2c_master_send(0x9);
char num = 0 | (level << pin);
i2c_master_send(num);
i2c_master_stop();
}

char getExpander(){
i2c_master_start();
i2c_master_send(SLAVE_ADDR << 1);
i2c_master_send(0x9);
i2c_master_restart();
i2c_master_send((SLAVE_ADDR << 1) | 1);
char temp = i2c_master_recv();
i2c_master_ack(1);
i2c_master_stop();
return temp;
}


void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length){
i2c_master_start();
int i = 0;
i2c_master_send(address << 1);
i2c_master_send(reg + i); // starts at temp low, goes however long
i2c_master_restart();
i2c_master_send((address << 1) | 1);
for (i; i < length; ++i){
char temp = i2c_master_recv();
if ( i < length - 1){
        i2c_master_ack(0);
}
data[i] = temp;
}
i2c_master_ack(1);
i2c_master_stop();
}

short combineData(char * data, int index){
    short value = data[index + 1] << 8;
    value = data[index];
    return value;
}
