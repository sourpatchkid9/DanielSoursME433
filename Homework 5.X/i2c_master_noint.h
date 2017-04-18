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

void i2c_initialize_expander(void);

void i2c_read(unsigned char address, unsigned char reg);

void i2c_write(unsigned char address, unsigned char value, unsigned char reg);


#endif	/* I2C_MASTER_NOINT_H */



