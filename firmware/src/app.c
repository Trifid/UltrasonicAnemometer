/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "system_definitions.h"
#include "app.h"
#include "app_usb/app_usb.h"
#include "app_usb/app_usb_tasks.h"
#include "capture.h"
#include "calculate.h"
#include "app_i2c/app_i2c_internal.h"
#include "app_i2c/app_i2c_external.h"







// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

/* Recieve data buffer */
//extern uint8_t receiveBuffer[64];

/* Transmit data buffer */
//extern uint8_t transmitBuffer[64];

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
//uint8_t i2c_buffer[5];

//volatile APP_SINGLE_MEASUREMENT singleMeasurements[2][4];
//volatile APP_SINGLE_MEASUREMENT *singleMeasurementCapturePtr;
//volatile APP_SINGLE_MEASUREMENT *singleMeasurementProcessPtr;
//volatile bool singleMeasurementReady;
//
//
//volatile uint16_t timeslot; //bit0=AXIS, bit1=DIRECTION
//volatile uint16_t pulsecount;

//volatile uint32_t zcd_capture[NUMBER_OF_ZCD_CAPTURES];
//volatile int16_t amplitude[NUMBER_OF_ZCD_CAPTURES-1];
//volatile uint8_t zcd_count;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// USB Callback Function declarations are located in app_usb/app_usb_callback.h
// USB Callback Function definitions are located in app_usb/app_usb_callback.c

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************




// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    appData.appState = APP_STATE_INIT;
    
    //Initialize variables
//    timeslot = 0;
//    pulsecount = 0;
//    
//    singleMeasurementCapturePtr = &singleMeasurements[0][0];
//    singleMeasurementProcessPtr = &singleMeasurements[0][0];
//    singleMeasurementReady = false;
    
    app_usb_init();
    capture_init(&getTimeOfFlight);
    
    appData.appState = APP_STATE_WAITING_FOR_MEASUREMENT_SET;

    //PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
    //PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
    
    //Initialize the internal I2C module
    // Configure General I2C Options
    app_i2c_internal_init();
    app_i2c_external_init();
}










/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    

    switch(appData.appState)
    {
        case APP_STATE_WAITING_FOR_MEASUREMENT_SET:
            
            app_usb_tasks();
            //app_i2c_internal_run();
            
            break;
            
        case APP_STATE_MEASUREMENT_SET_READY:
            
            //Process data
            
            
            appData.measurementPtr = getMeasurementSetPtr();
            PGC_SS_LAT = 1;
            calculateAverageTimeOfFlight(&appData.timeOfFlight[0], appData.measurementPtr);
            PGC_SS_LAT = 0;
            
            calculateWindSpeed(&appData.result, &appData.timeOfFlight[0]);
            
            //app_i2c_internal_writeDigipot(40);
//            i2c_buffer[0] = 0x00;
//            i2c_buffer[1] = 40;
//            app_i2c_internal_write(0b01011100, &i2c_buffer[0], 2);
            //app_i2c_internal_writeDigipot(100);
            
            //Reset app state
            appData.appState = APP_STATE_WAITING_FOR_MEASUREMENT_SET;
            
            break;
            
        default:
            break;
    }


}
 

/*******************************************************************************
 End of File
 */
