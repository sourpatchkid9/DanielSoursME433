/* 
 * File:   i2c_master_noint.h
 * Author: Daniel
 *
 * Created on April 17, 2017, 9:23 PM
 */

#ifndef I2C_MASTER_NOINT_H
#define	I2C_MASTER_NOINT_H
#define SLAVE_ADDR 0b1101011

void i2c_master_setup(void);

void i2c_master_start(void);

void i2c_master_restart(void);

void i2c_master_send(unsigned char byte);

unsigned char i2c_master_recv(void);

void i2c_master_ack(int val);

void i2c_master_stop(void);

void init_IMU();
    
void setExpander(char pin, char level);

char getExpander();
 
void I2C_read_multiple(unsigned char address, unsigned char register, unsigned char * data, int length);

short combineData(char * data, int index);


#endif	/* I2C_MASTER_NOINT_H */
