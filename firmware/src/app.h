/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"

#define PGD_READY_LAT LATBbits.LATB0
#define PGC_SS_LAT LATBbits.LATB1
#define SCLK_LAT LATBbits.LATB15
#define MOSI_LAT LATBbits.LATB14

#define TIMESLOT_MASK 0b01111111
#define MEASUREMENT_MASK 0b01111100
#define AXIS_MASK 0b01
#define DIRECTION_MASK 0b10


#define OC1_ISR_DEBUG_OUTPUT SCLK_LAT
#define OC2_ISR_DEBUG_OUTPUT SCLK_LAT
#define IC4_ISR_DEBUG_OUTPUT MOSI_LAT

//Timing
#define NUMBER_OF_MEASUREMENTS 32
#define NUMBER_OF_DIRECTIONS 4
#define TIMER_2_OVERFLOW 93749
#define AXIS_DIRECTION_ADVANCE 500
#define PWM_PERIOD 1200
#define NUMBER_OF_PULSES 12
#define ENABLE_ZCD_INTERRUPTS_TRIGGER 45000
#define NUMBER_OF_ZCD_CAPTURES 34
#define KERNEL_LENGTH 17

//Derived timing definitions
#define PWM_HALF_PERIOD (PWM_PERIOD>>1)
#define NUMBER_OF_AMPLITUDE_CAPTURES NUMBER_OF_ZCD_CAPTURES-1
#define AXIS_DIRECTION_TRIGGER (TIMER_2_OVERFLOW-AXIS_DIRECTION_ADVANCE)
#define PWM_RISING_EDGE ((PWM_PERIOD>>1)-1)
#define PWM_FALLING_EDGE (PWM_PERIOD-1)
#define DISABLE_ZCD_INTERRUPTS_TRIGGER (ENABLE_ZCD_INTERRUPTS_TRIGGER+(((NUMBER_OF_ZCD_CAPTURES>>1)+3)*PWM_PERIOD))



// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	APP_STATE_INIT,
	APP_STATE_WAITING_FOR_MEASUREMENT_SET,
	APP_STATE_MEASUREMENT_SET_READY
} APP_STATES;


typedef struct
{
    /* Pointer to Recieve data buffer */
    uint8_t * receiveBuffer;
    
    /* Pointer to Transmit data buffer */
    uint8_t * transmitBuffer;

} APP_USB_DATA;


//typedef struct
//{
//    
//    uint32_t zerocrossings[NUMBER_OF_ZCD_CAPTURES];
//    
//    int16_t amplitudes[NUMBER_OF_AMPLITUDE_CAPTURES];
//    
//} APP_SINGLE_MEASUREMENT;



typedef struct
{
    uint32_t measurementSetId;
    uint8_t measurementId;
    uint8_t digipotValue;
	uint32_t zerocrossing[NUMBER_OF_ZCD_CAPTURES];
	int16_t amplitude[NUMBER_OF_AMPLITUDE_CAPTURES];
} SINGLE_MEASUREMENT;

typedef struct
{
    uint32_t measurementSetId;
	uint32_t tof[NUMBER_OF_DIRECTIONS][NUMBER_OF_MEASUREMENTS];
} MEASUREMENT_SET;

typedef struct
{
	uint8_t measurementId;
	uint8_t measurementId_mask;
	SINGLE_MEASUREMENT *target;
	bool inProgress;
} SINGLE_MEASUREMENT_GET;

typedef struct
{
	MEASUREMENT_SET *target;
	bool inProgress;
} MEASUREMENT_SET_GET;

typedef struct
{
	int16_t temperature;
	int32_t windSpeed[2];
} RESULTS;

//typedef struct
//{
//	APP_STATE appState;
//	MEASUREMENT_SET *measurementSetPtr;
//} NEW_APP_DATA;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */


typedef struct
{
    /* general purpose counter variable */
    //uint8_t cntr;
    APP_STATES appState;
    
    MEASUREMENT_SET *measurementPtr;
    uint32_t timeOfFlight[4];
    RESULTS result;
    
    /* All USB application data */
    APP_USB_DATA usb;
    
} APP_DATA;

static char direction_lookup[] = {'N','E','S','W'};

typedef enum
{
    N,
    E,
    S,
    W  
} APP_MEASUREMENT_DIRECTIONS;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/



	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony Demo application initialization routine

  Description:
    This routine initializes Harmony Demo application.  This function opens
    the necessary drivers, initializes the timer and registers the application
    callback with the USART driver.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks ( void );



extern const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1];
extern const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor;


#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

