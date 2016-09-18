/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"

#include "peripheral/ports/plib_ports.h"
#include "peripheral/devcon/plib_devcon.h"

// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

/*** DEVCFG1 ***/
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_50     // Watchdog Timer Window Size (Window Size is 50%)

/*** DEVCFG2 ***/
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

/*** DEVCFG3 ***/
#pragma config USERID = 0xffff
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
/****************************************************
 * Endpoint Table needed by the Device Layer.
 ****************************************************/
uint8_t __attribute__((aligned(512))) endPointTable[DRV_USBFS_ENDPOINTS_NUMBER * 32];
const DRV_USBFS_INIT drvUSBInit =
{
    /* Assign the endpoint table */
    .endpointTable= endPointTable,

    /* Interrupt Source for USB module */
    .interruptSource = INT_SOURCE_USB_1,
    
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    .operationMode = DRV_USBFS_OPMODE_DEVICE,
    
    .operationSpeed = USB_SPEED_FULL,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USB_ID_1
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
//<editor-fold defaultstate="collapsed" desc="SYS_DEVCON Initialization Data">
/*******************************************************************************
  Device Control System Service Initialization Data
*/

const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};

// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="USB Stack Initialization Data">


/****************************************************
 * Class specific descriptor - HID Report descriptor
 ****************************************************/
const uint8_t hid_rpt0[] =
{
    0x06, 0x00, 0xFF,   // Usage Page = 0xFF00 (Vendor Defined Page 1)
    0x09, 0x01,             // Usage (Vendor Usage 1)
    0xA1, 0x01,             // Colsslection (Application)
    0x19, 0x01,             // Usage Minimum
    0x29, 0x40,             // Usage Maximum 	//64 input usages total (0x01 to 0x40)
    0x15, 0x01,             // Logical Minimum (data bytes in the report may have minimum value = 0x00)
    0x25, 0x40,      	    // Logical Maximum (data bytes in the report may have maximum value = 0x00FF = unsigned 255)
    0x75, 0x08,             // Report Size: 8-bit field size
    0x95, 0x40,             // Report Count: Make sixty-four 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item)
    0x81, 0x00,             // Input (Data, Array, Abs): Instantiates input packet fields based on the above report size, count, logical min/max, and usage.
    0x19, 0x01,             // Usage Minimum
    0x29, 0x40,             // Usage Maximum 	//64 output usages total (0x01 to 0x40)
    0x91, 0x00,             // Output (Data, Array, Abs): Instantiates output packet fields.  Uses same report size and count as "Input" fields, since nothing new/different was specified to the parser since the "Input" item.
    0xC0                    // End Collection
};

/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
    const USB_DEVICE_HID_INIT hidInit0 =
    {
        .hidReportDescriptorSize = sizeof(hid_rpt0),
        .hidReportDescriptor = &hid_rpt0,
        .queueSizeReportReceive = 1,
        .queueSizeReportSend = 1
    };
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .numberOfInterfaces = 1,    /* Number of interfaces */
        .funcDriverIndex = 0,  /* Index of HID Function Driver */
        .driver = (void*)USB_DEVICE_HID_FUNCTION_DRIVER,    /* USB HID function data exposed to device layer */
        .funcDriverInit = (void*)&hidInit0,    /* Function driver init data*/
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/
/*******************************************
 *  USB Device Descriptor 
 *******************************************/
const USB_DEVICE_DESCRIPTOR deviceDescriptor =
{
    0x12,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,          // DEVICE descriptor type
    0x0200,                         // USB Spec Release Number in BCD format
    0x00,                           // Class Code
    0x00,                           // Subclass code
    0x00,                           // Protocol code
    USB_DEVICE_EP0_BUFFER_SIZE,     // Max packet size for EP0, see system_config.h
    0x04D8,                         // Vendor ID
    0xF160,                         // Product ID
    0x0100,                         // Device release number in BCD format
    0x01,                           // Manufacturer string index
    0x02,                           // Product string index
    0x00,                           // Device serial number string index
    0x01                            // Number of possible configurations
};


/*******************************************
 *  USB Full Speed Configuration Descriptor
 *******************************************/
const uint8_t fullSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    41,0,                //(41 Bytes)Size of the Config descriptor.e
    1,                                               // Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED, // Attributes
    50,                                                 // Max power consumption (2X mA)
    /* Descriptor for Function 1 - HID     */ 
    
	/* Interface Descriptor */

    0x09,                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,           // INTERFACE descriptor type
    0,                                  // Interface Number
    0,                                  // Alternate Setting Number
    2,                                  // Number of endpoints in this interface
    USB_HID_CLASS_CODE,                 // Class code
    USB_HID_SUBCLASS_CODE_NO_SUBCLASS , // Subclass code
    USB_HID_PROTOCOL_CODE_NONE,         // No Protocol
    0,                                  // Interface string index

    /* HID Class-Specific Descriptor */

    0x09,                           // Size of this descriptor in bytes
    USB_HID_DESCRIPTOR_TYPES_HID,   // HID descriptor type
    0x11,0x01,                      // HID Spec Release Number in BCD format (1.11)
    0x00,                           // Country Code (0x00 for Not supported)
    1,                              // Number of class descriptors, see usbcfg.h
    USB_HID_DESCRIPTOR_TYPES_REPORT,// Report descriptor type
    USB_DEVICE_16bitTo8bitArrange(sizeof(hid_rpt0)),   // Size of the report descriptor

    /* Endpoint Descriptor */

    0x07,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    1 | USB_EP_DIRECTION_IN,      // EndpointAddress
    USB_TRANSFER_TYPE_INTERRUPT,    // Attributes
    0x40,0x00,                      // Size
    0x01,                           // Interval

    /* Endpoint Descriptor */

    0x07,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    1 | USB_EP_DIRECTION_OUT,     // EndpointAddress
    USB_TRANSFER_TYPE_INTERRUPT,    // Attributes
    0x40,0x00,                      // size
    0x01,                           // Interval
    
    


};

/*******************************************
 * Array of Full speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor
};


/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;
        uint8_t bDscType;
        uint16_t string[1];
    }
    sd000 =
    {
        sizeof(sd000),          // Size of this descriptor in bytes
        USB_DESCRIPTOR_STRING,  // STRING descriptor type
        {0x0409}                // Language ID
    };
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[14];    // String
    }
    sd001 =
    {
        sizeof(sd001),
        USB_DESCRIPTOR_STRING,
        {'s','o','l','d','e','r','n','e','r','d','.','c','o','m'}
		
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[21];    // String
    }
    sd002 =
    {
        sizeof(sd002),
        USB_DESCRIPTOR_STRING,
		{'U','l','t','r','a','s','o','n','i','c',' ','A','n','e','m','o','m','e','t','e','r'}
    }; 

/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[3]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};

/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &deviceDescriptor,          /* Full speed descriptor */
    1,                          /* Total number of full speed configurations available */
    fullSpeedConfigDescSet,     /* Pointer to array of full speed configurations descriptors*/
    NULL, 
    0, 
    NULL, 
    3,                          // Total number of string descriptors available.
    stringDescriptors,          // Pointer to array of string descriptors.
    NULL, 
    NULL
};


/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/
const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 1,
    
    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_FULL,
    
    /* Index of the USB Driver to be used by this Device Layer Instance */
    .driverIndex = DRV_USBFS_INDEX_0,

    /* Pointer to the USB Driver Functions. */
    .usbDriverInterface = DRV_USBFS_DEVICE_INTERFACE,
    
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Static Initialization Functions
// *****************************************************************************
// *****************************************************************************

void SYS_PORTS_Initialize(void)
{
 
    // Pin 2: ADC+    
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_ADCPLUS_CHANNEL, PIN_ADCPLUS_BIT);
    #ifdef PIN_ADCPLUS_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_ADCPLUS_ANALOG_PIN, PORTS_PIN_MODE_ANALOG);
    #endif // PIN_ADCPLUS_ANALOG_PIN

    // Pin 2: ADC-    
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_ADCMINUS_CHANNEL, PIN_ADCMINUS_BIT);
    #ifdef PIN_ADCMINUS_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_ADCMINUS_ANALOG_PIN, PORTS_PIN_MODE_ANALOG);
    #endif // PIN_ADCMINUS_ANALOG_PIN

    // Pin 4: PGD/READY    
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_PGDREADY_CHANNEL, PIN_PGDREADY_BIT);
    #ifdef PIN_PGDREADY_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_PGDREADY_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_PGDREADY_ANALOG_PIN

    // Pin 5: PGC/SS   
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_PGCSS_CHANNEL, PIN_PGCSS_BIT);
    #ifdef PIN_ADCPLUS_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_PGCSS_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_ADCPLUS_ANALOG_PIN

    // Pin 6: SDA2 
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_SDA2_CHANNEL, PIN_SDA2_BIT);
    #ifdef PIN_SDA2_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_SDA2_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_SDA2_ANALOG_PIN

    // Pin 7: SCL2
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_SCL2_CHANNEL, PIN_SCL2_BIT);
    #ifdef PIN_SCL2_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_SCL2_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_SCL2_ANALOG_PIN

    // Pin 11: AXIS   
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_AXIS_CHANNEL, PIN_AXIS_BIT);
    #ifdef PIN_AXIS_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_AXIS_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_AXIS_ANALOG_PIN

    // Pin 12: DIRECTION 
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_DIRECTION_CHANNEL, PIN_DIRECTION_BIT);
    #ifdef PIN_DIRECTION_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_DIRECTION_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_DIRECTION_ANALOG_PIN

    //Pin 14: SIGNAL  
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_SIGNAL_CHANNEL, PIN_SIGNAL_BIT);
    #ifdef PIN_SIGNAL_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_SIGNAL_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_SIGNAL_ANALOG_PIN

    // Pin 16: ZCD 
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_ZCD_CHANNEL, PIN_ZCD_BIT);
    #ifdef PIN_ZCD_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_ZCD_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_ZCD_ANALOG_PIN

    // Pin 17: SCL1    
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_SCL1_CHANNEL, PIN_SCL1_BIT);
    #ifdef PIN_SCL1_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_SCL1_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_SCL1_ANALOG_PIN

    // Pin 18: SDA1   
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_SDA1_CHANNEL, PIN_SDA1_BIT);
    #ifdef PIN_SDA1_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_SDA1_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_SDA1_ANALOG_PIN

    // Pin 24: MISO  
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PIN_MISO_CHANNEL, PIN_MISO_BIT);
    #ifdef PIN_MISO_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_MISO_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_MISO_ANALOG_PIN

    // Pin 25: MOSI   
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_MOSI_CHANNEL, PIN_MOSI_BIT);
    #ifdef PIN_MOSI_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_MOSI_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_MOSI_ANALOG_PIN

    // Pin 26: SCLK 
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PIN_SCLK_CHANNEL, PIN_SCLK_BIT);
    #ifdef PIN_SCLK_ANALOG_PIN
        PLIB_PORTS_PinModeSelect(PORTS_ID_0, PIN_SCLK_ANALOG_PIN, PORTS_PIN_MODE_DIGITAL);
    #endif // PIN_SCLK_ANALOG_PIN  

    
    PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PPS_REGISTERS);
    
    // Map Output Compare 2 module to SIGNAL pin
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_OC2, OUTPUT_PIN_RPB5);
    
    // Map ZCD pin to Input Capture 4 module
    PLIB_PORTS_RemapInput(PORTS_ID_0, INPUT_FUNC_IC4, INPUT_PIN_RPB7);
    
    PLIB_DEVCON_DeviceRegistersLock(DEVCON_ID_0, DEVCON_PPS_REGISTERS);

}

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    
    SYS_PORTS_Initialize();

    /* Initialize Drivers */
    /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBFS_Initialize(DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBInit);

    /* Initialize System Services */
    
    

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();
  
    /* Initialize Middleware */
    
    /* Set priority of USB interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);
    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);
    
    /* Set priority of OC1 interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_OC1, INT_PRIORITY_LEVEL5);
    /* Set Sub-priority of OC1 interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_OC1, INT_SUBPRIORITY_LEVEL0);
    
    /* Set priority of OC2 interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_OC2, INT_PRIORITY_LEVEL5);
    /* Set Sub-priority of OC2 interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_OC2, INT_SUBPRIORITY_LEVEL0);
    
    /* Set priority of IC4 interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_IC4, INT_PRIORITY_LEVEL5);
    /* Set Sub-priority of OC2 interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_IC4, INT_SUBPRIORITY_LEVEL0);
    
    /* Set priority of I2C 1 interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_I2C1, INT_PRIORITY_LEVEL1);
    /* Set Sub-priority of I2C 1 interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_I2C1, INT_SUBPRIORITY_LEVEL0);
    
    /* Set priority of I2C 2 interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_I2C2, INT_PRIORITY_LEVEL2);
    /* Set Sub-priority of I2C 2 interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_I2C2, INT_SUBPRIORITY_LEVEL0);
    
    
    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);

    /* Enable Global Interrupts */
    SYS_INT_Enable();
    
    
    

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

