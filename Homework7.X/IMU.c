/* 
* File: IMU.c
* Author: Daniel
*
* Created on April 22, 2017, 4:04 PM
*/
#include "i2c_master_noint.h"
#include <stdio.h>
#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro
#include<math.h>
#include "ILI9163C.h"
#include <stdio.h>
#define SLAVE_ADDR 0b1101011
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
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
#pragma config USERID = 0x0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON// allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

int main() { // buffer for sending messages to the user
init_IMU();
SPI1_init();
LCD_init();
LCD_clearScreen(BLACK);


int length = 14;
char msg[100];
unsigned char data[length];

while(1){
    _CP0_SET_COUNT(0);
    I2C_read_multiple(SLAVE_ADDR, 0x20, data, length);
    short temp = combineData(data,0);
    short gyroX = combineData(data,2);
    short gyroY = combineData(data,4);
    short gyroZ = combineData(data,6);
    short accelX = combineData(data,8);
    short accelY = combineData(data,10);
    short accelZ = combineData(data,12);
    sprintf(msg,"AccelX: %d",accelX);
    write_string(msg,28,32,CYAN);
    sprintf(msg,"AccelY: %d",accelY);
    write_string(msg,28,40,CYAN);
    sprintf(msg,"GyroX: %d",gyroX);
    write_string(msg,28,48,CYAN);
    sprintf(msg,"GyroY: %d",gyroY);
    write_string(msg,28,56,CYAN);
        while (_CP0_GET_COUNT()<4000000){    
}
    write_string("`````",63,32,BLACK);
    write_string("`````",63,40,BLACK);
    write_string("`````",63,48,BLACK);
    write_string("`````",63,56,BLACK);
}
}