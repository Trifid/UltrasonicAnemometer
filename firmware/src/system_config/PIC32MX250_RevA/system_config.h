/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.
    
    Created with MPLAB Harmony Version 1.07
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Common System Service Configuration Options
*/
#define SYS_VERSION_STR           "1.07"
#define SYS_VERSION               10700

// *****************************************************************************
/* Clock System Service Configuration Options
*/
#define SYS_CLK_FREQ                        48000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            48000000ul
#define SYS_CLK_UPLL_BEFORE_DIV2_FREQ       96000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         8000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       0ul
   
/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

    
/*** Ports System Service Configuration ***/
    
// Pin 2: ADC+    
#define PIN_ADCPLUS_CHANNEL PORT_CHANNEL_A
#define PIN_ADCPLUS_BIT PORTS_BIT_POS_0
#define PIN_ADCPLUS_ANALOG_PIN PORTS_ANALOG_PIN_0 
// Pin 2: ADC-    
#define PIN_ADCMINUS_CHANNEL PORT_CHANNEL_A
#define PIN_ADCMINUS_BIT PORTS_BIT_POS_1
#define PIN_ADCMINUS_ANALOG_PIN PORTS_ANALOG_PIN_1 
// Pin 4: PGD/READY    
#define PIN_PGDREADY_CHANNEL PORT_CHANNEL_B
#define PIN_PGDREADY_BIT PORTS_BIT_POS_0
#define PIN_PGDREADY_ANALOG_PIN PORTS_ANALOG_PIN_2
// Pin 5: PGC/SS   
#define PIN_PGCSS_CHANNEL PORT_CHANNEL_B
#define PIN_PGCSS_BIT PORTS_BIT_POS_1
#define PIN_PGCSS_ANALOG_PIN PORTS_ANALOG_PIN_3 
// Pin 6: SDA2 
#define PIN_SDA2_CHANNEL PORT_CHANNEL_B
#define PIN_SDA2_BIT PORTS_BIT_POS_2
#define PIN_SDA2_ANALOG_PIN PORTS_ANALOG_PIN_4
// Pin 7: SCL2
#define PIN_SCL2_CHANNEL PORT_CHANNEL_B
#define PIN_SCL2_BIT PORTS_BIT_POS_3
#define PIN_SCL2_ANALOG_PIN PORTS_ANALOG_PIN_5
// Pin 11: AXIS   
#define PIN_AXIS_CHANNEL PORT_CHANNEL_A
#define PIN_AXIS_BIT PORTS_BIT_POS_4
// Pin 12: DIRECTION 
#define PIN_DIRECTION_CHANNEL PORT_CHANNEL_B
#define PIN_DIRECTION_BIT PORTS_BIT_POS_4
//Pin 14: SIGNAL  
#define PIN_SIGNAL_CHANNEL PORT_CHANNEL_B
#define PIN_SIGNAL_BIT PORTS_BIT_POS_5
// Pin 16: ZCD 
#define PIN_ZCD_CHANNEL PORT_CHANNEL_B
#define PIN_ZCD_BIT PORTS_BIT_POS_7
// Pin 17: SCL1    
#define PIN_SCL1_CHANNEL PORT_CHANNEL_B
#define PIN_SCL1_BIT PORTS_BIT_POS_8
// Pin 18: SDA1   
#define PIN_SDA1_CHANNEL PORT_CHANNEL_B
#define PIN_SDA1_BIT PORTS_BIT_POS_9
// Pin 24: MISO  
#define PIN_MISO_CHANNEL PORT_CHANNEL_B
#define PIN_MISO_BIT PORTS_BIT_POS_13
#define PIN_MISO_ANALOG_PIN PORTS_ANALOG_PIN_11
// Pin 25: MOSI   
#define PIN_MOSI_CHANNEL PORT_CHANNEL_B
#define PIN_MOSI_BIT PORTS_BIT_POS_14
#define PIN_MOSI_ANALOG_PIN PORTS_ANALOG_PIN_10
// Pin 26: SCLK 
#define PIN_SCLK_CHANNEL PORT_CHANNEL_B
#define PIN_SCLK_BIT PORTS_BIT_POS_15
#define PIN_SCLK_ANALOG_PIN PORTS_ANALOG_PIN_9
    
    
//Assign I2C modules
#define I2C_EXTERNAL I2C_ID_1
#define I2C_INTERNAL I2C_ID_2


    
// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************
/*** USB Driver Configuration ***/


/* Enables Device Support */
#define DRV_USBFS_DEVICE_SUPPORT      true

/* Disable Device Support */
#define DRV_USBFS_HOST_SUPPORT      false

#define DRV_USBFS_PERIPHERAL_ID       USB_ID_1
#define DRV_USBFS_INTERRUPT_SOURCE    INT_SOURCE_USB_1
#define DRV_USBFS_INDEX               0


/* Interrupt mode enabled */
#define DRV_USBFS_INTERRUPT_MODE      true


/* Number of Endpoints used */
#define DRV_USBFS_ENDPOINTS_NUMBER    2




/*** USB Device Stack Configuration ***/










/* The USB Device Layer will not initialize the USB Driver */
#define USB_DEVICE_DRIVER_INITIALIZE_EXPLICIT

/* Maximum device layer instances */
#define USB_DEVICE_INSTANCES_NUMBER     1

/* EP0 size in bytes */
#define USB_DEVICE_EP0_BUFFER_SIZE      64









/* Maximum instances of HID function driver */
#define USB_DEVICE_HID_INSTANCES_NUMBER     1










/* HID Transfer Queue Size for both read and
   write. Applicable to all instances of the
   function driver */
#define USB_DEVICE_HID_QUEUE_DEPTH_COMBINED 2






// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************

/*** Application Instance 0 Configuration ***/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/

