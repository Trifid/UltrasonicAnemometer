
//Other I2C parametes
#define I2C_BAUD_RATE_INTERNAL 400000

#include "app_i2c_internal.h"
#include "peripheral/i2c/plib_i2c.h"

typedef enum
{
	APP_I2C_INTERNAL_STATE_IDLE,
    APP_I2C_INTERNAL_STATE_START,
    APP_I2C_INTERNAL_STATE_DATA,
    APP_I2C_INTERNAL_STATE_COMPLETED,
    APP_I2C_INTERNAL_STATE_STOP
} APP_I2C_INTERNAL_STATES;

typedef struct
{
    uint8_t slave_address;
    uint8_t *data_ptr;
    uint8_t bytes_remaining;        
} I2C_SEND_BUFFER;

volatile I2C_SEND_BUFFER i2cInternalTxBuffer;
volatile APP_I2C_INTERNAL_STATES i2cInternalState;
volatile uint8_t digipotBuffer[2];

i2c_internal_ISR()
{
    if (PLIB_I2C_ArbitrationLossHasOccurred(I2C_INTERNAL))
    {
        // Abort transfer, retry later
        PLIB_I2C_ArbitrationLossClear(I2C_INTERNAL);
        i2cInternalState = APP_I2C_INTERNAL_STATE_IDLE;
        return;
    }
        
    switch(i2cInternalState)
    {
        case APP_I2C_INTERNAL_STATE_IDLE:
            break;
        
        case APP_I2C_INTERNAL_STATE_START:
            PLIB_I2C_TransmitterByteSend(I2C_INTERNAL, i2cInternalTxBuffer.slave_address);
            i2cInternalState = APP_I2C_INTERNAL_STATE_DATA;
            break;
            
        case APP_I2C_INTERNAL_STATE_DATA:
            PLIB_I2C_TransmitterByteSend(I2C_INTERNAL, *(i2cInternalTxBuffer.data_ptr));
            ++i2cInternalTxBuffer.data_ptr;
            --i2cInternalTxBuffer.bytes_remaining;
            if(!i2cInternalTxBuffer.bytes_remaining)
            {
               i2cInternalState = APP_I2C_INTERNAL_STATE_COMPLETED;
            }
            break;
        
        case APP_I2C_INTERNAL_STATE_COMPLETED:
            PLIB_I2C_MasterStop(I2C_INTERNAL);
            i2cInternalState = APP_I2C_INTERNAL_STATE_STOP;
            break;
            
        case APP_I2C_INTERNAL_STATE_STOP:
            i2cInternalState = APP_I2C_INTERNAL_STATE_IDLE;
            break;
            
        default:
            i2cInternalState = APP_I2C_INTERNAL_STATE_IDLE;
            break;
    }
    
   PLIB_INT_SourceFlagClear(INT_ID_0, I2C_INTERNAL_INT_SOURCE_MASTER); 
}

bool app_i2c_internal_write(uint8_t address, uint8_t *data, uint8_t length)
{
    if(PLIB_I2C_BusIsIdle(I2C_INTERNAL))
    {
        i2cInternalTxBuffer.slave_address = address;
        i2cInternalTxBuffer.data_ptr = data;
        i2cInternalTxBuffer.bytes_remaining = length;
    
        PLIB_I2C_MasterStart(I2C_INTERNAL);
        i2cInternalState = APP_I2C_INTERNAL_STATE_START;
        return true;
    }
    else
    {
        return false;
    }
}

void app_i2c_internal_writeDigipot(uint8_t value)
{
    digipotBuffer[0] = 0x00;
    digipotBuffer[1] = value;
    app_i2c_internal_write(I2C_DIGIPOT_ADDRESS, &digipotBuffer[0], 2);             
}


void app_i2c_internal_init(void)
{
    i2cInternalState = APP_I2C_INTERNAL_STATE_IDLE;
    
    
    PLIB_I2C_SlaveClockStretchingDisable(I2C_INTERNAL);
    PLIB_I2C_GeneralCallEnable(I2C_INTERNAL);
    PLIB_I2C_SMBDisable(I2C_INTERNAL);
    PLIB_I2C_HighFrequencyEnable(I2C_INTERNAL);
    PLIB_I2C_ReservedAddressProtectEnable(I2C_INTERNAL);
    //PLIB_I2C_StopInIdleEnable(I2C_INTERNAL);
    // Set Desired baud rate
    PLIB_I2C_BaudRateSet(I2C_INTERNAL, SYS_CLK_BUS_PERIPHERAL_1, I2C_BAUD_RATE_INTERNAL);
    // Set the appropriate slave address (slave only)
    //PLIB_I2C_SlaveAddress7BitSet(I2C_INTERNAL, MY_SLAVE_ADDRESS);
    // Optional: Clear and enable interrupts before enabling the I2C module.
    PLIB_INT_SourceFlagClear(INT_ID_0, I2C_INTERNAL_INT_SOURCE_MASTER);
    PLIB_INT_SourceEnable(INT_ID_0, I2C_INTERNAL_INT_SOURCE_MASTER);
    // Enable the module
    PLIB_I2C_Enable(I2C_INTERNAL);    
}
