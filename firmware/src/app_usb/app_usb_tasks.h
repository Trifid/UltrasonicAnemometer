/* 
 * File:   app_usb_communication.h
 * Author: Luke
 *
 * Created on 31. Mai 2016, 23:00
 */

#ifndef APP_USB_COMMUNICATION_H
#define	APP_USB_COMMUNICATION_H

#include "../app.h"

extern APP_DATA appData;
extern volatile uint16_t timeslot; //bit0=AXIS, bit1=DIRECTION
//extern volatile uint32_t zcd_capture[NUMBER_OF_ZCD_CAPTURES];
//extern volatile int16_t amplitude[NUMBER_OF_ZCD_CAPTURES-1];

//extern volatile SINGLE_MEASUREMENT *singleMeasurementProcessPtr;

void app_usb_tasks(void);

#endif	/* APP_USB_COMMUNICATION_H */

