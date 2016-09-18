/* 
 * File:   app_usb.h
 * Author: Luke
 *
 * Created on 25. Mai 2016, 00:00
 */

#ifndef APP_USB_H
#define	APP_USB_H

#include "../app.h"

#define APP_USB_BUFFER_SIZE 64
 
/* Size of circular buffers (must be a power of 2) */
#define APP_USB_RECEIVE_CIRCULAR_BUFFER_SIZE 1
#define APP_USB_TRANSMIT_CIRCULAR_BUFFER_SIZE 16
 
/* Derived definitions */
#define APP_USB_RECEIVE_BUFFER_EMPTY 0
#define APP_USB_RECEIVE_BUFFER_FULL APP_USB_RECEIVE_CIRCULAR_BUFFER_SIZE
#define APP_USB_RECEIVE_BUFFER_MASK (APP_USB_RECEIVE_CIRCULAR_BUFFER_SIZE-1)
#define APP_USB_TRANSMIT_BUFFER_EMPTY 0
#define APP_USB_TRANSMIT_BUFFER_FULL APP_USB_TRANSMIT_CIRCULAR_BUFFER_SIZE
#define APP_USB_TRANSMIT_BUFFER_MASK (APP_USB_TRANSMIT_CIRCULAR_BUFFER_SIZE-1)

inline bool app_usb_isReady(void);
inline uint8_t* app_usb_getReceiveBuffer(void);
inline uint8_t app_usb_getReceiveBufferCount(void);
inline uint8_t* app_usb_getTransmitBuffer(void);
inline uint8_t* app_usb_getZeroedTransmitBuffer(void);
inline uint8_t app_usb_getTransmitBufferCount(void);
void app_usb_freeReceiveBuffer(void);
void app_usb_sendTransmitBuffer();

//Initialization of all application USB data structures
void app_usb_init(void);


#endif	/* APP_USB_H */


