/* 
 * File:   app_i2c_external.h
 * Author: Luke
 *
 * Created on 30. June 2016, 22:31
 */

/*******************************************************************************
 * Format of I2C data transmitted by the Ultrasonic Anemometer                 *
 * Byte 0 is the first byte transmitted, byte 7 is the last                    *
 * Bytes [0:1]                                                                 *
 *     Wind speed North to South                                               *
 *     Format: int16_t, Unit: mm/s                                             *
 * Bytes [2:3]                                                                 *
 *     Wind speed East to West                                                 *
 *     Format: int16_t, Unit: mm/s                                             *
 * Bytes [4:5]                                                                 *
 *     Sonic temperature                                                       *
 *     Format: int16_t, Unit: 0.01 degrees centigrade                          *
 * Bytes [6:7]                                                                 *
 *     Status information                                                      *
 *     TBD                                                                     *
 ******************************************************************************/

#ifndef APP_I2C_EXTERNAL_H
#define	APP_I2C_EXTERNAL_H

#include "app.h"

extern APP_DATA appData;

#define I2C_SLAVE_ADDRESS 0x51

void app_i2c_external_init(void);

#endif	/* APP_I2C_EXTERNAL_H */

