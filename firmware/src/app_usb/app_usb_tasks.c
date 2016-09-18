
#include "app_usb_tasks.h"
#include "app_usb.h"
#include "../capture.h"

typedef enum
{
    APP_USB_TASK_INIT,
    APP_USB_TASK_WAITING_FOR_DATA,
    APP_USB_TASK_TRANSMITTING_DATA
} APP_USB_TASK_STATE;

//local variables
uint16_t remaining_measurements;
SINGLE_MEASUREMENT singleMeasurementCpy;
MEASUREMENT_SET measurementSetCpy;
APP_USB_TASK_STATE appUsbTaskState = APP_USB_TASK_INIT;
uint32_t previous_measurementSetId;

uint8_t get_direction(char c);
void reply_default(void);

void reply_singleMeasurementMultiple(char multiple);
void reply_singleMeasurement(char task, char direction);
void send_singleMeasurement(char task);

void reply_measurementSetMultiple(char multiple);
void reply_measurementSet();
void send_measurementSet(uint8_t direction_idx);

void reply_resultsMultiple(char multiple);
void send_results(void);


void app_usb_tasks(void)
{
    //USB device is ready to use
    if(app_usb_isReady())
    {
        //Message has been received
        if(app_usb_getReceiveBufferCount()!=APP_USB_RECEIVE_BUFFER_EMPTY)
        {
            //More data can be sent
            if(app_usb_getTransmitBufferCount()!=APP_USB_TRANSMIT_BUFFER_FULL)
            {
                PGD_READY_LAT = 1;
                
                //Get pointers to receive buffer
                appData.usb.receiveBuffer = app_usb_getReceiveBuffer();
                
                //Reply to requests based on first byte
                switch(appData.usb.receiveBuffer[0]) 
                {
                    case 'A': //Amplitudes
                    case 'Z': //Zero Crossings
                    case 'F': //Entire full single measurement
                        remaining_measurements = 1;
                        reply_singleMeasurement(appData.usb.receiveBuffer[0], appData.usb.receiveBuffer[1]);
                        break;
 
                    case 'S': //Series of 'F'
                        reply_singleMeasurementMultiple(appData.usb.receiveBuffer[1]);
                        break;
                        
                    case 'T': //Time-of-flight for an entire series of measurements
                        reply_measurementSetMultiple(appData.usb.receiveBuffer[1]);
                        break;
                        
                    case 'R': //Results
                        reply_resultsMultiple(appData.usb.receiveBuffer[1]);
                        break;

                    default:
                        reply_default();
                }
                
                PGD_READY_LAT = 0;
            }
        }
    }
}

uint8_t get_direction(char c)
{
    //Pick direction based on c
    switch(c) 
    {
        case 'N':
        case 'n':
            return 0;

        case 'E':
        case 'e':
            return 1;

        case 'S':
        case 's':
            return 2;

        case 'W':
        case 'w':
            return 3;
            
        default:
            return 0;
    }
}

void reply_singleMeasurementMultiple(char multiple)
{
    if(!remaining_measurements)
    {
        switch(multiple)
        {
            case '1':
                remaining_measurements = 4;
                break;
            case '2':
                remaining_measurements = 8;
                break;
            case '3':
                remaining_measurements = 16;
                break;
            case '4':
                remaining_measurements = 32;
                break;
            case '5':
                remaining_measurements = 64;
                break;
            case '6':
                remaining_measurements = 128;
                break;
            case '7':
                remaining_measurements = 256;
                break;
            case '8':
                remaining_measurements = 512;
                break;
            case '9':
                remaining_measurements = 1024;
                break;
            default:
                remaining_measurements = 4;
                break;    
        }
    }
    reply_singleMeasurement('F', direction_lookup[remaining_measurements&0b11]);
}


void reply_singleMeasurement(char task, char direction)
{ 
    switch(appUsbTaskState)
    {
        // Place data request
        case APP_USB_TASK_INIT:
            getSingleMeasurement
            (
                get_direction(direction),
                0b11, 
                &singleMeasurementCpy
            );
            appUsbTaskState = APP_USB_TASK_WAITING_FOR_DATA;
            break;
            
        // Wait for data to arrive
        case APP_USB_TASK_WAITING_FOR_DATA:
            if(!(getSingleMeasurementInProgress()))
            {
                appUsbTaskState = APP_USB_TASK_TRANSMITTING_DATA;
            }
            break;
        
        // Finally transmit data
        case APP_USB_TASK_TRANSMITTING_DATA:
            send_singleMeasurement(task);
            //Discard the received data just processed if all is done
            --remaining_measurements;
            if(!remaining_measurements)
            {
                app_usb_freeReceiveBuffer(); 
            }
            // All done, return to init state
            appUsbTaskState = APP_USB_TASK_INIT;
            break;
    } 
}   


void send_singleMeasurement(char task)
{
    uint8_t tmp[20];
    uint8_t tmp_pos;
    uint8_t pos = 0;
    uint8_t i;
    
    //Get a first frame
    while(app_usb_getTransmitBufferCount()==APP_USB_TRANSMIT_BUFFER_FULL); //blocking, change this later
    appData.usb.transmitBuffer = app_usb_getZeroedTransmitBuffer();
    
    //Print frame signature
    appData.usb.transmitBuffer[pos++] = task;
    appData.usb.transmitBuffer[pos++] = ';';
    
    //MeasurementSetId value
    itoa(&appData.usb.transmitBuffer[pos], singleMeasurementCpy.measurementSetId ,10);
    while(appData.usb.transmitBuffer[++pos]);
    appData.usb.transmitBuffer[pos++] = ';';
    
    //MeasurementId without direction part
    itoa(&appData.usb.transmitBuffer[pos], singleMeasurementCpy.measurementId>>2 ,10);
    while(appData.usb.transmitBuffer[++pos]);
    appData.usb.transmitBuffer[pos++] = ';';
    
    //Direction of measurement
    appData.usb.transmitBuffer[pos++] = direction_lookup[singleMeasurementCpy.measurementId & 0b11];
    appData.usb.transmitBuffer[pos++] = ';';
    
    //Digipot setting
    itoa(&appData.usb.transmitBuffer[pos], singleMeasurementCpy.digipotValue, 10);
    while(appData.usb.transmitBuffer[++pos]);
    appData.usb.transmitBuffer[pos++] = ';';
    
    for(i=0;i<NUMBER_OF_ZCD_CAPTURES;++i)
    {
        //write data to tmp buffer
        tmp_pos = 0;
        //Zero crossing
        if((task=='Z') || (task=='F'))
        {
            itoa(&tmp[tmp_pos], singleMeasurementCpy.zerocrossing[i] ,10);
            while(tmp[++tmp_pos]);
            tmp[tmp_pos++] = ';'; 
        }
        //Amplitude
        if((task=='A') || (task=='F'))
        {
            if(i<NUMBER_OF_AMPLITUDE_CAPTURES)
            {
                itoa(&tmp[tmp_pos], singleMeasurementCpy.amplitude[i] ,10);
                while(tmp[++tmp_pos]);
                tmp[tmp_pos++] = ';'; 
            }
        }
        //Line break
        if(i==NUMBER_OF_ZCD_CAPTURES-1)
        {
            tmp[tmp_pos++] = 13; // <CR>
            tmp[tmp_pos++] = 10; // <LF>
        }
        //Make sure our string is zero terminated
        tmp[tmp_pos] = 0;
        
        //copy data to usb transmit buffer
        tmp_pos = 0;
        while(tmp[tmp_pos])
        {
            appData.usb.transmitBuffer[pos++] = tmp[tmp_pos++];
            
            //Get a new buffer when necessary
            if(pos==APP_USB_BUFFER_SIZE)
            {
                //Transmit full frame
                app_usb_sendTransmitBuffer(); 
                //Get a new frame
                while(app_usb_getTransmitBufferCount()==APP_USB_TRANSMIT_BUFFER_FULL); //blocking, change this later
                appData.usb.transmitBuffer = app_usb_getZeroedTransmitBuffer();
                pos = 0;
            }
        }
    }
    
    //Transmit last frame if not empty
    if(pos)
    {
       app_usb_sendTransmitBuffer();  
    }
}


void reply_measurementSetMultiple(char multiple)
{
    if(!remaining_measurements)
    {
        switch(multiple)
        {
            case '1':
                remaining_measurements = 1;
                break;
            case '2':
                remaining_measurements = 2;
                break;
            case '3':
                remaining_measurements = 4;
                break;
            case '4':
                remaining_measurements = 8;
                break;
            case '5':
                remaining_measurements = 16;
                break;
            case '6':
                remaining_measurements = 32;
                break;
            case '7':
                remaining_measurements = 64;
                break;
            case '8':
                remaining_measurements = 128;
                break;
            case '9':
                remaining_measurements = 512;
                break;
            default:
                remaining_measurements = 1;
                break;    
        }
    }
    reply_measurementSet();
}


void reply_measurementSet()
{ 
    switch(appUsbTaskState)
    {
        // Place data request
        case APP_USB_TASK_INIT:
            getMeasurementSet(&measurementSetCpy);
            appUsbTaskState = APP_USB_TASK_WAITING_FOR_DATA;
            break;
            
        // Wait for data to arrive
        case APP_USB_TASK_WAITING_FOR_DATA:
            if(!(getMeasurementSetInProgress()))
            {
                appUsbTaskState = APP_USB_TASK_TRANSMITTING_DATA;
            }
            break;
        
        // Finally transmit data
        case APP_USB_TASK_TRANSMITTING_DATA:
            send_measurementSet(0);
            send_measurementSet(1);
            send_measurementSet(2);
            send_measurementSet(3);
            //Discard the received data just processed if all is done
            --remaining_measurements;
            if(!remaining_measurements)
            {
                app_usb_freeReceiveBuffer(); 
            }
            // All done, return to init state
            appUsbTaskState = APP_USB_TASK_INIT;
            break;
    } 
}   


void send_measurementSet(uint8_t direction_idx)
{
    uint8_t tmp[20];
    uint8_t tmp_pos;
    uint8_t pos = 0;
    uint8_t i;
    
    //Get a first frame
    while(app_usb_getTransmitBufferCount()==APP_USB_TRANSMIT_BUFFER_FULL); //blocking, change this later
    appData.usb.transmitBuffer = app_usb_getZeroedTransmitBuffer();
    
    //Print frame signature
    appData.usb.transmitBuffer[pos++] = 'T';
    appData.usb.transmitBuffer[pos++] = ';';
    
    //MeasurementSetId value
    itoa(&appData.usb.transmitBuffer[pos], measurementSetCpy.measurementSetId ,10);
    while(appData.usb.transmitBuffer[++pos]);
    appData.usb.transmitBuffer[pos++] = ';';
    
    //Direction of measurement
    appData.usb.transmitBuffer[pos++] = direction_lookup[direction_idx];
    appData.usb.transmitBuffer[pos++] = ';';
    
    for(i=0;i<NUMBER_OF_MEASUREMENTS;++i)
    {
        //write data to tmp buffer
        tmp_pos = 0;
        
        //Time of flight
        itoa(&tmp[tmp_pos], measurementSetCpy.tof[direction_idx][i] ,10);
        while(tmp[++tmp_pos]);
        tmp[tmp_pos++] = ';'; 
            
        //Line break
        if(i==NUMBER_OF_MEASUREMENTS-1)
        {
            tmp[tmp_pos++] = 13; // <CR>
            tmp[tmp_pos++] = 10; // <LF>
        }
        
        //Make sure our string is zero terminated
        tmp[tmp_pos] = 0;
        
        //copy data to usb transmit buffer
        tmp_pos = 0;
        while(tmp[tmp_pos])
        {
            appData.usb.transmitBuffer[pos++] = tmp[tmp_pos++];
            
            //Get a new buffer when necessary
            if(pos==APP_USB_BUFFER_SIZE)
            {
                //Transmit full frame
                app_usb_sendTransmitBuffer(); 
                //Get a new frame
                while(app_usb_getTransmitBufferCount()==APP_USB_TRANSMIT_BUFFER_FULL); //blocking, change this later
                appData.usb.transmitBuffer = app_usb_getZeroedTransmitBuffer();
                pos = 0;
            }
        }
    }
    
    //Transmit last frame if not empty
    if(pos)
    {
       app_usb_sendTransmitBuffer();  
    }
}


void reply_resultsMultiple(char multiple)
{
    if(!remaining_measurements)
    {
        switch(multiple)
        {
            case '1':
                remaining_measurements = 1;
                break;
            case '2':
                remaining_measurements = 2;
                break;
            case '3':
                remaining_measurements = 4;
                break;
            case '4':
                remaining_measurements = 8;
                break;
            case '5':
                remaining_measurements = 16;
                break;
            case '6':
                remaining_measurements = 32;
                break;
            case '7':
                remaining_measurements = 64;
                break;
            case '8':
                remaining_measurements = 128;
                break;
            case '9':
                remaining_measurements = 512;
                break;
            default:
                remaining_measurements = 1;
                break;    
        }
    }
    
    if(previous_measurementSetId!=(*appData.measurementPtr).measurementSetId)
    {
        send_results();
        previous_measurementSetId = (*appData.measurementPtr).measurementSetId;
        --remaining_measurements;
        if(!remaining_measurements)
        {
            app_usb_freeReceiveBuffer(); 
        }
    }  
}


void send_results(void)
{
    uint8_t direction_idx;
    uint8_t pos = 0;
    
    //Get a  frame
    while(app_usb_getTransmitBufferCount()==APP_USB_TRANSMIT_BUFFER_FULL); //blocking, change this later
    appData.usb.transmitBuffer = app_usb_getZeroedTransmitBuffer();
    
    //Print frame signature
    appData.usb.transmitBuffer[pos++] = 'R';
    appData.usb.transmitBuffer[pos++] = ';';
    
    //MeasurementSetId value
    itoa(&appData.usb.transmitBuffer[pos], (*appData.measurementPtr).measurementSetId ,10);
    while(appData.usb.transmitBuffer[++pos]);
    appData.usb.transmitBuffer[pos++] = ';';
    
    //Averaged Time of Flight
    for(direction_idx=0; direction_idx<NUMBER_OF_DIRECTIONS; ++direction_idx)
    {
        itoa(&appData.usb.transmitBuffer[pos], appData.timeOfFlight[direction_idx] ,10);
        while(appData.usb.transmitBuffer[++pos]);
        appData.usb.transmitBuffer[pos++] = ';';  
    }
    
    //Temperature
    itoa(&appData.usb.transmitBuffer[pos], appData.result.temperature ,10);
    while(appData.usb.transmitBuffer[++pos]);
    appData.usb.transmitBuffer[pos++] = ';';  
    
    //WindSpeed
    for(direction_idx=0; direction_idx<2; ++direction_idx)
    {
        itoa(&appData.usb.transmitBuffer[pos], appData.result.windSpeed[direction_idx] ,10);
        while(appData.usb.transmitBuffer[++pos]);
        appData.usb.transmitBuffer[pos++] = ';';  
    }
    
    //Line break
    appData.usb.transmitBuffer[pos++] = 13; // <CR>
    appData.usb.transmitBuffer[pos++] = 10; // <LF>
    
    //Schedule transmit buffer to be sent
    app_usb_sendTransmitBuffer(); 
}


void reply_default(void)
{
//    if(appData.usb.receiveBuffer[0])
//    {
//       //Copy received data to transmit buffer
//        appData.usb.transmitBuffer = app_usb_getZeroedTransmitBuffer();
//        memcpy(appData.usb.transmitBuffer, appData.usb.receiveBuffer, APP_USB_BUFFER_SIZE);
//        //Schedule transmit buffer to be sent
//        app_usb_sendTransmitBuffer(); 
//    }
    //Discard the received data just processed
    app_usb_freeReceiveBuffer();
}