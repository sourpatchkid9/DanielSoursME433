/* 
 * File:   i2c_master_noint.h
 * Author: Daniel
 *
 * Created on April 17, 2017, 9:23 PM
 */

#ifndef I2C_MASTER_NOINT_H
#define	I2C_MASTER_NOINT_H

void i2c_master_setup(void);

void i2c_master_start(void);

void i2c_master_restart(void);

void i2c_master_send(unsigned char byte);

unsigned char i2c_master_recv(void);

void i2c_master_ack(int val);

void i2c_master_stop(void);

void initExpander();
    
void setExpander(char pin, char level);

char getExpander();
 



#endif	/* I2C_MASTER_NOINT_H */


/* 
 * File:   main5.c
 * Author: Daniel
 *
 * Created on April 17, 2017, 8:47 PM
 */
/*
#include <stdio.h>
#include <stdlib.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include "i2c_master_noint.h"
#define SLAVE_ADDR 0b0100000
#define CS LATBbits.LATB7

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1// use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x1001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


void initSPI1(){
    TRISBbits.TRISB7 = 0;
    RPB8Rbits.RPB8R = 0b0011;
   // RPB7Rbits.RPB7R = 0b0011;

    CS = 1;
            
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void write_voltage(unsigned int channel, unsigned int voltage){
    unsigned char bigdata = 0b01110000;
    bigdata |= (channel << 7);
    bigdata |= (voltage >> 4);
    unsigned char littledata = 0b0;
    littledata  |= ((voltage && 0b1111)<<4);
    CS = 0;
    spi_io(bigdata);
    spi_io(littledata);
    CS = 1;
}
int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    
 
    __builtin_enable_interrupts();

  
    
    initSPI1();
    i2c_master_setup();
    
    char num = 0b11110000;
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR<<1);
    i2c_master_send(0x0);
    i2c_master_send(num);
    i2c_master_stop();
    
    
    
    initExpander();
    setExpander(0, 0);
    setExpander(7, 1);
    

    while (1){
        char temp = getExpander();
        char bit7 = temp >> 7;
        if (bit7){
            setExpander(0, 1);
        }
        else{
            setExpander(0, 0);
        }
        
    }
}
*/