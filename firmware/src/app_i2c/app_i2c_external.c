
//Other I2C parametes
#define I2C_BAUD_RATE_EXTERNAL 100000
#define I2C_EXTERNAL_TX_BUFFER_SIZE 8
#define I2C_EXTERNAL_RX_BUFFER_SIZE 8

#include "app_i2c_external.h"
#include "peripheral/i2c/plib_i2c.h"


typedef enum
{
	APP_I2C_EXTERNAL_STATE_SLAVE_IDLE,
    APP_I2C_EXTERNAL_STATE_SLAVE_RX,
    APP_I2C_EXTERNAL_STATE_SLAVE_TX
} APP_I2C_EXTERNAL_STATES;


volatile APP_I2C_EXTERNAL_STATES i2c_external_state;
volatile uint8_t i2c_external_data;
volatile uint8_t i2c_external_address;
volatile uint8_t i2cExternalRxBuffer[I2C_EXTERNAL_RX_BUFFER_SIZE];
volatile uint8_t i2cExternalRxBufferIdx;
volatile uint8_t i2cExternalTxBuffer[I2C_EXTERNAL_TX_BUFFER_SIZE];
volatile uint8_t i2cExternalTxBufferIdx;

void prepareTxBuffer();
void processRxBuffer();

i2c_external_ISR()
{
    //Slave interrupt
    if(PLIB_INT_SourceFlagGet(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_SLAVE))
    {
        if(PLIB_I2C_SlaveReadIsRequested(I2C_EXTERNAL))
        {
            //External master is requesting data
            
            //Byte has been received
            if(PLIB_I2C_ReceivedByteIsAvailable(I2C_EXTERNAL))
            {
                if (PLIB_I2C_SlaveAddressIsDetected(I2C_EXTERNAL))
                {
                    //Save address
                    i2c_external_address = PLIB_I2C_ReceivedByteGet(I2C_EXTERNAL);
                    //Prepare to transmit data
                    processRxBuffer();
                    prepareTxBuffer();
                    i2cExternalTxBufferIdx = 0;
                }
            }
            
            //More data can be transmitted
            if(PLIB_I2C_TransmitterIsReady(I2C_EXTERNAL))
            {
                // Write the byte to the TX buffer
                PLIB_I2C_TransmitterByteSend(I2C_EXTERNAL, i2cExternalTxBuffer[i2cExternalTxBufferIdx++]);
                i2cExternalTxBufferIdx &= (I2C_EXTERNAL_TX_BUFFER_SIZE-1);
                // Free the SCL line (only if clock stretching was enabled)
                PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
            }            
        }
        else
        {
            //External master is sending data
            
            
            //Byte has been received
            if(PLIB_I2C_ReceivedByteIsAvailable(I2C_EXTERNAL))
            {
                if (PLIB_I2C_SlaveAddressIsDetected(I2C_EXTERNAL))
                {
                    //Address, not data
                    i2c_external_address = PLIB_I2C_ReceivedByteGet(I2C_EXTERNAL);
                    //Prepare to receive data
                    i2cExternalRxBufferIdx = 0;
                    PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
                }
                else
                {
                    //Data, not address
                    
                    //Save data to receive buffer
                    i2cExternalRxBuffer[i2cExternalRxBufferIdx++] = PLIB_I2C_ReceivedByteGet(I2C_EXTERNAL);
                    i2cExternalRxBufferIdx &= (I2C_EXTERNAL_RX_BUFFER_SIZE-1);
                    PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
                }
            }
        }
                  
        //Finally, clear interrupt flag
        PLIB_INT_SourceFlagClear(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_SLAVE);
    }      
}

void app_i2c_external_init(void)
{
    i2c_external_state = APP_I2C_EXTERNAL_STATE_SLAVE_IDLE;
    
    //PLIB_I2C_SlaveInterruptOnStartDisable(I2C_EXTERNAL);
    //PLIB_I2C_SlaveInterruptOnStopEnable(I2C_EXTERNAL);
    PLIB_I2C_SlaveClockStretchingEnable(I2C_EXTERNAL);
    PLIB_I2C_GeneralCallEnable(I2C_EXTERNAL);
    PLIB_I2C_SMBDisable(I2C_EXTERNAL);
    PLIB_I2C_HighFrequencyDisable(I2C_EXTERNAL);
    PLIB_I2C_ReservedAddressProtectEnable(I2C_EXTERNAL);
    //PLIB_I2C_StopInIdleEnable(I2C_EXTERNAL);
    // Set Desired baud rate
    PLIB_I2C_BaudRateSet(I2C_EXTERNAL, SYS_CLK_BUS_PERIPHERAL_1, I2C_BAUD_RATE_EXTERNAL);
    // Set the appropriate slave address (slave only)
    PLIB_I2C_SlaveAddress7BitSet(I2C_EXTERNAL, I2C_SLAVE_ADDRESS);
    PLIB_I2C_SlaveMask7BitSet (I2C_EXTERNAL, 0xFF);
    // Optional: Clear and enable interrupts before enabling the I2C module.
    //PLIB_INT_SourceFlagClear(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_MASTER);
    //PLIB_INT_SourceEnable(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_MASTER);
    PLIB_INT_SourceFlagClear(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_SLAVE);
    PLIB_INT_SourceEnable(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_SLAVE);
    // Enable the module
    PLIB_I2C_Enable(I2C_EXTERNAL);    
}

void prepareTxBuffer()
{
    int16_t tmp;
    tmp = (int16_t) appData.result.windSpeed[0];
    i2cExternalTxBuffer[0] = 0xFF & (tmp>>8);
    i2cExternalTxBuffer[1] = 0xFF & tmp;
    tmp = (int16_t) appData.result.windSpeed[1];
    i2cExternalTxBuffer[2] = 0xFF & (tmp>>8);
    i2cExternalTxBuffer[3] = 0xFF & tmp;
    tmp = (int16_t) appData.result.temperature;
    i2cExternalTxBuffer[4] = 0xFF & (tmp>>8);
    i2cExternalTxBuffer[5] = 0xFF & tmp;
    i2cExternalTxBuffer[6] = 0;
    i2cExternalTxBuffer[7] = 0;
    
//Debug
//    i2cExternalTxBuffer[0] = 0x11;
//    i2cExternalTxBuffer[1] = 0x22;
//    i2cExternalTxBuffer[2] = 0x33;
//    i2cExternalTxBuffer[3] = 0x44;
//    i2cExternalTxBuffer[4] = 0x0A;
//    i2cExternalTxBuffer[5] = 0x5E;
    
//    memcpy(&i2cExternalTxBuffer[0], &i2cExternalRxBuffer[0], I2C_EXTERNAL_TX_BUFFER_SIZE);
}


void processRxBuffer()
{
    uint8_t i;
    for(i=0;i<I2C_EXTERNAL_RX_BUFFER_SIZE;++i)
    {
      i2cExternalRxBuffer[i] *= 2;  
    }
}




//i2c_external_ISR_old()
//{
//    //Slave interrupt
//    if(PLIB_INT_SourceFlagGet(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_SLAVE))
//    {
//        switch(i2c_external_state)
//        {
//            case APP_I2C_EXTERNAL_STATE_SLAVE_IDLE:
//            {
//                if(PLIB_I2C_ReceivedByteIsAvailable(I2C_EXTERNAL))
//                { 
//                    // Check to see if the byte is data or an address
//                    if (PLIB_I2C_SlaveAddressIsDetected(I2C_EXTERNAL))
//                    {
//                        //Save address
//                        i2c_external_address = PLIB_I2C_ReceivedByteGet(I2C_EXTERNAL);
//
//                        if ( PLIB_I2C_SlaveReadIsRequested(I2C_EXTERNAL) )
//                        {
//                            // External master is requesting data
//                            prepareTxBuffer();
//                            i2cExternalTxBufferIdx = 0;
//                            i2c_external_state = APP_I2C_EXTERNAL_STATE_SLAVE_TX;
//                            
//                            if(PLIB_I2C_TransmitterIsReady(I2C_EXTERNAL))
//                            {
//                                // Write the byte to the TX buffer
//                                PLIB_I2C_TransmitterByteSend(I2C_EXTERNAL, i2cExternalTxBuffer[i2cExternalTxBufferIdx++]);
//                                i2cExternalTxBufferIdx &= (I2C_EXTERNAL_TX_BUFFER_SIZE-1);
//                                // Free the SCL line (only if clock stretching was enabled)
//                                PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
//                            }                            
//                        }
//                        else
//                        {
//                            // External master is sending data
//                            i2cExternalRxBufferIdx = 0;
//                            PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
//                            i2c_external_state = APP_I2C_EXTERNAL_STATE_SLAVE_RX;
//                        }
//                    }
//                }
//                break;
//            }
//                 
//            case APP_I2C_EXTERNAL_STATE_SLAVE_TX:
//            {          
//                if(PLIB_I2C_TransmitterIsReady(I2C_EXTERNAL))
//                {
//                    // Write the byte to the TX buffer
//                    PLIB_I2C_TransmitterByteSend(I2C_EXTERNAL, i2cExternalTxBuffer[i2cExternalTxBufferIdx++]);
//                    i2cExternalTxBufferIdx &= (I2C_EXTERNAL_TX_BUFFER_SIZE-1);
//                    
//                    // Free the SCL line (only if clock stretching was enabled)
//                    PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
//                }                 
//                
//                if (PLIB_I2C_TransmitterByteHasCompleted(I2C_EXTERNAL))
//                {
//                    if (PLIB_I2C_TransmitterByteWasAcknowledged(I2C_EXTERNAL))
//                    {
//                        // More bytes to follow
//                    }
//                    else
//                    {
//                        // Transmssion completed
//                        i2c_external_state = APP_I2C_EXTERNAL_STATE_SLAVE_IDLE;
//                    }
//                }                
//                break;
//            }   
//            
//            case APP_I2C_EXTERNAL_STATE_SLAVE_RX:
//            {
//                if(PLIB_I2C_ReceivedByteIsAvailable(I2C_EXTERNAL))
//                { 
//                    // Check to see if the byte is data or an address
//                    if (!PLIB_I2C_SlaveAddressIsDetected(I2C_EXTERNAL))
//                    {
//                        //Save data to receive buffer
//                        i2cExternalRxBuffer[i2cExternalRxBufferIdx++] = PLIB_I2C_ReceivedByteGet(I2C_EXTERNAL);
//                        i2cExternalRxBufferIdx &= (I2C_EXTERNAL_RX_BUFFER_SIZE-1);
//                        PLIB_I2C_SlaveClockRelease(I2C_EXTERNAL);
//                    }
//                }
//                
//                break;
//            }
//            
//            default:
//            {    
//                i2c_external_state = APP_I2C_EXTERNAL_STATE_SLAVE_IDLE;
//            }
//        }  
//
//        PLIB_INT_SourceFlagClear(INT_ID_0, I2C_EXTERNAL_INT_SOURCE_SLAVE);
//    }      
//}