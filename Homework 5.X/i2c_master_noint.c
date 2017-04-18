#include "i2c_master_noint.h"
#include <xc.h>
#define SLAVE_ADDR 0x32


void i2c_master_setup(void) {
  I2C2BRG = 0x1000;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

void i2c_initialize_expander(void){
    ANSELbits.ANSB2 = 0;
    ANSELbits.ANSB3 = 0;
    char buf[100] = {};                       // buffer for sending messages to the user
  unsigned char master_write0 = 0xCD;       // first byte that master writes
  unsigned char master_write1 = 0x91;       // second byte that master writes
  unsigned char master_read0  = 0x00;       // first received byte
  unsigned char master_read1  = 0x00;       // second received byte
}

void i2c_write(unsigned char address, unsigned char value, unsigned char reg){
    i2c_master_start(); // make the start bit

    i2c_master_send(address<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing

    i2c_master_send(reg); // the register to write to

    i2c_master_send(value); // the value to put in the register

    i2c_master_stop(); // make the stop bit
}

void i2c_read(unsigned char address, unsigned char reg){
    i2c_master_start(); // make the start bit

    i2c_master_send(address<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing

    i2c_master_send(reg); // the register to read from

    i2c_master_restart(); // make the restart bit

    i2c_master_send(address<1|1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading

    char r = i2c_master_recv(); // save the value returned

    i2c_master_ack(1); // make the ack so the slave knows we got it

    i2c_master_stop(); // make the stop bit
}
