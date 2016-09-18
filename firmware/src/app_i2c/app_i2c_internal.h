/* 
 * File:   app_i2c_internal.h
 * Author: Luke
 *
 * Created on 9. Juni 2016, 23:59
 */

#ifndef APP_I2C_INTERNAL_H
#define	APP_I2C_INTERNAL_H

#include "app.h"

extern APP_DATA appData;

#define I2C_DIGIPOT_ADDRESS 0b01011100
#define I2C_DIGIPOT_WRITE_COMMAND 0b00000000
//Includes an extra zero bit. Address is only the first 7 bits
#define I2C_WRITE_BIT 0b0
#define I2C_READ_BIT 0b1


void app_i2c_internal_init(void);
bool app_i2c_internal_write(uint8_t address, uint8_t *data, uint8_t length);
void app_i2c_internal_writeDigipot(uint8_t value);

//void app_i2c_internal_run(void);
//void app_i2c_internal_blocking(void);
//void app_i2c_internal_writeDigipot(uint8_t value);


#endif	/* APP_I2C_INTERNAL_H */

