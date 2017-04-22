/* 
 * File:   LCD.c
 * Author: Daniel
 *
 * Created on April 18, 2017, 2:10 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "ILI9163C.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>

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

void draw_character(char c, char x,char y, short color1){
    char d = c-0x20;
    int i,j;
    for (i = 0;i <= 4; ++i){
        //if(x*i<128)
            for (j = 0;j <= 7; ++j){
                //if(y*j<128)
                    if (((ASCII[d][i]) >> j) & 1){
                        LCD_drawPixel(x+i,y+j,color1);
                    }
            }
    }
}

void write_string(char str[], char x, char y, short color){
    int i = 0;
    while(str[i]){
        draw_character(str[i],x,y,color);
        x +=5;
        i++;
    }
}    



void draw_bar(char x,char y,short color1, short color2,char len, char w){
    int i,j;
    char str[100];
    for (i = 0;i<=len;++i){
        sprintf(str,"Hello World %d",i);
        write_string("```",88,32,BLACK);
        write_string(str,28,32,WHITE);
      
        for (j = 0;j<=w;++j){
            _CP0_SET_COUNT(0);
            if (i < len/2){
                LCD_drawPixel(x+i,y+j,color1);
                while(_CP0_GET_COUNT() < 4000){
                    
                }
            }
            else{
                LCD_drawPixel(x+i,y+j,color2);
                while(_CP0_GET_COUNT() < 4000){
                    
                }
            }
        }
    
}
}
void clear_bar(char x,char y,short color1, short color2,char len, char w){
    int i,j;
    
    for (i = 0;i<=len;++i){
        for (j = 0;j<=w;++j){
            _CP0_SET_COUNT(0);
            if (i < len/2){
                LCD_drawPixel(x+i,y+j,color1);
            }
            else{
                LCD_drawPixel(x+i,y+j,color2);
            }
        }
    
}
}
int main() {
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    while(1){
   draw_bar(13,60,BLUE,YELLOW,100,5);
   clear_bar(13,60,BLACK,BLACK,100,5);
    }
    
}