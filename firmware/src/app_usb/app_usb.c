
#include "app_usb.h"
#include "system_definitions.h"


typedef enum
{
    /* Application is initializing */
    APP_USB_STATE_INIT,

    /* Application is waiting for configuration */
    APP_USB_STATE_WAIT_FOR_CONFIGURATION,

    /* Application is running the main tasks */
    APP_USB_STATE_UP_AND_RUNNING,

    /* Application is in an error state */
    APP_USB_STATE_ERROR
            
} APP_USB_STATES;


/* The application's current state */
volatile APP_USB_STATES i2cInternalState;

/* Device layer handle returned by device layer open function */
volatile USB_DEVICE_HANDLE devHandle;

/* Device configured */
volatile bool configured;

/* Send report transfer handle*/
USB_DEVICE_HID_TRANSFER_HANDLE txTransferHandle;

/* Receive report transfer handle */
USB_DEVICE_HID_TRANSFER_HANDLE rxTransferHandle;

/* Configuration value selected by the host*/
volatile uint8_t configurationValue;

 /* USB HID current Idle */
/*volatile*/ uint8_t idleRate;

volatile bool rxInProgress;
volatile bool txInProgress;

/*volatile*/ uint8_t receiveBuffer[APP_USB_RECEIVE_CIRCULAR_BUFFER_SIZE][APP_USB_BUFFER_SIZE];
/*volatile*/ uint8_t transmitBuffer[APP_USB_TRANSMIT_CIRCULAR_BUFFER_SIZE][APP_USB_BUFFER_SIZE];

volatile uint8_t receiveBufferReadIndex;
volatile uint8_t receiveBufferWriteIndex;
volatile uint8_t receiveBufferCount;
volatile uint8_t transmitBufferReadIndex;
volatile uint8_t transmitBufferWriteIndex;
volatile uint8_t transmitBufferCount;

void app_usb_init(void)
{
    /* Private USB data*/
    receiveBufferReadIndex = 0;
    receiveBufferWriteIndex = 0;
    receiveBufferCount = 0;
    transmitBufferReadIndex = 0;
    transmitBufferWriteIndex = 0;
    transmitBufferCount = 0;
    
    i2cInternalState = APP_USB_STATE_INIT;
    devHandle = USB_DEVICE_HANDLE_INVALID;
    configured = false;
    txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    configurationValue = 0;
    idleRate = 0;
    txInProgress;
    rxInProgress = false;
    txInProgress = false;
}

void app_usb_reInit()
{
    receiveBufferReadIndex = 0;
    receiveBufferWriteIndex = 0;
    receiveBufferCount = 0;
    transmitBufferReadIndex = 0;
    transmitBufferWriteIndex = 0;
    transmitBufferCount = 0;
    
    configured = false;
    txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    rxInProgress = false;
    txInProgress = false; 
}


USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if(reportSent->handle == txTransferHandle )
            {
                // Transfer progressed.
                --transmitBufferCount;
                ++transmitBufferReadIndex;
                transmitBufferReadIndex &= APP_USB_TRANSMIT_BUFFER_MASK;
                txInProgress = false;
                //Check if more data needs to be sent
                if(transmitBufferCount)
                {
                    USB_DEVICE_HID_ReportSend(
                        USB_DEVICE_HID_INDEX_0,
                        &txTransferHandle, 
                        &transmitBuffer[transmitBufferReadIndex][0],
                        APP_USB_BUFFER_SIZE);
                    txInProgress = true;
                }
            }
            
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if(reportReceived->handle == rxTransferHandle )
            {
                // Transfer progressed.
                ++receiveBufferCount;
                ++receiveBufferWriteIndex;
                receiveBufferWriteIndex &= APP_USB_RECEIVE_BUFFER_MASK;
                rxInProgress = false;
                // Place another read request if we still have an empty read buffer
//                if(receiveBufferCount!=APP_USB_RECEIVE_BUFFER_FULL) 
//                {
//                    USB_DEVICE_HID_ReportReceive(
//                        USB_DEVICE_HID_INDEX_0,
//                        &rxTransferHandle,
//                        &receiveBuffer[receiveBufferWriteIndex][0],
//                        APP_USB_BUFFER_SIZE);
//                    rxInProgress = true;
//                }
            }
          
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(devHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(devHandle, & (idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}


void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
            /* Deliberate fall-through to USB_DEVICE_EVENT_DECONFIGURED */
        case USB_DEVICE_EVENT_DECONFIGURED:
            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */
            configured = false;
            i2cInternalState = APP_USB_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Save the other details for later use. */
            configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;
            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t) &i2cInternalState);
            /* Reset some configuration data */
            receiveBufferReadIndex = 0;
            receiveBufferWriteIndex = 0;
            receiveBufferCount = 0;
            transmitBufferReadIndex = 0;
            transmitBufferWriteIndex = 0;
            transmitBufferCount = 0;
            rxInProgress = false;
            txInProgress = false;            
            /* Device is now ready to run the main task */
            configured = true;
            i2cInternalState = APP_USB_STATE_UP_AND_RUNNING;
            /* Place a new read request. */
            USB_DEVICE_HID_ReportReceive(
                USB_DEVICE_HID_INDEX_0,
                &rxTransferHandle, 
                &receiveBuffer[receiveBufferWriteIndex][0], 
                APP_USB_BUFFER_SIZE);
            rxInProgress = true;
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(devHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* VBUS is not available, detach the device */
            USB_DEVICE_Detach(devHandle);
            break;
            
        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_RESUMED:
            break;
            
        case USB_DEVICE_EVENT_ERROR:
            break;
            
        default:
            break;
    }
}

inline bool app_usb_isReady(void)
{
    if(configured)
    {
        return true;
    }
    //Try to open the USB device if we're still in INIT state
    else if(i2cInternalState==APP_USB_STATE_INIT)
    {
        /* Open the device layer */
        devHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );
        if(devHandle != USB_DEVICE_HANDLE_INVALID)
        {
            /* Register a callback with device layer to get event notification (for end point 0) */
            USB_DEVICE_EventHandlerSet(devHandle, APP_USBDeviceEventHandler, 0);
            //Advance to next state
            i2cInternalState = APP_USB_STATE_WAIT_FOR_CONFIGURATION;
        }
        return false;
    }
}

inline uint8_t* app_usb_getReceiveBuffer(void)
{
    return (uint8_t*) &receiveBuffer[receiveBufferReadIndex][0];
}
 
inline uint8_t app_usb_getReceiveBufferCount(void)
{
    return receiveBufferCount;
}
 
inline uint8_t* app_usb_getTransmitBuffer(void)
{
    return (uint8_t*) &transmitBuffer[transmitBufferWriteIndex][0];
}

inline uint8_t* app_usb_getZeroedTransmitBuffer(void)
{
    uint8_t pos;
    //Zero transmit buffer
    for(pos=0;pos<APP_USB_BUFFER_SIZE;++pos)
    {
        transmitBuffer[transmitBufferWriteIndex][pos] = 0;
    }
    return (uint8_t*) &transmitBuffer[transmitBufferWriteIndex][0];
}
 
inline uint8_t app_usb_getTransmitBufferCount(void)
{
    return transmitBufferCount;
}
 
void app_usb_freeReceiveBuffer(void)
{
    //Ignore request if receive buffer is already empty
    if(receiveBufferCount!=APP_USB_RECEIVE_BUFFER_EMPTY)
    {
        --receiveBufferCount;
        //receiveBufferCount = 0;
        ++receiveBufferReadIndex;
        receiveBufferReadIndex &= APP_USB_RECEIVE_BUFFER_MASK;
        // Place a new read request if not already receive in progress
        if(!rxInProgress)
        {
            USB_DEVICE_HID_ReportReceive(
                USB_DEVICE_HID_INDEX_0,
                &rxTransferHandle,
                &receiveBuffer[receiveBufferWriteIndex][0],
                APP_USB_BUFFER_SIZE);
            rxInProgress = true;
        }
    }
}
 
void app_usb_sendTransmitBuffer()
{
    //Ignore request if transmit buffer is already full
    if(transmitBufferCount!=APP_USB_TRANSMIT_BUFFER_FULL)
    {
        ++transmitBufferCount;
        ++transmitBufferWriteIndex;
        transmitBufferWriteIndex &= APP_USB_TRANSMIT_BUFFER_MASK;
        // Place a send request if not already transmission in progress
        if(!txInProgress)
        {
            USB_DEVICE_HID_ReportSend(
                USB_DEVICE_HID_INDEX_0,
                &txTransferHandle, 
                &transmitBuffer[transmitBufferReadIndex][0],
                APP_USB_BUFFER_SIZE);
            txInProgress = true;
        }
    }
}

 
