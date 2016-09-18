#define S_LENGTH 33
#define K_LENGTH 17
#define CAPTURES_PER_SAMPLE 4
#define TARGET_AMPLITUDE 3.0
#define TARGET_AMPLITUDE_DAC_VALUE (0.5*1023*TARGET_AMPLITUDE/3.3)

#include "capture.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/oc/plib_oc.h"
#include "peripheral/adc/plib_adc.h"
#include "peripheral/ic/plib_ic.h"


uint8_t tmp;
double last_gain;
double new_gain;
double digipot;


// ISR counters
uint8_t pulse_count;
uint8_t zcd_count;

// Pointer to function doing the convolution
uint32_t (*getTofPtr)(SINGLE_MEASUREMENT *singleMeasurement);

// Core data structures
SINGLE_MEASUREMENT singleMeasurement;
MEASUREMENT_SET measurementSet[2];
uint8_t measurementSetCaptureIndex;
uint8_t measurementSetProcessIndex;
uint8_t digipotValues[4];
int16_t maximumAmplitude;

// Allow application to get debug data
SINGLE_MEASUREMENT_GET singleMeasurementGet;
MEASUREMENT_SET_GET measurementSetGet;


/******************************************************************************
 * Output Compare Module 1 Interrupt Service Routine                          *
 * Used for the following tasks:                                              *
 *   -Incrementing timeslot in the range 0...127                              * 
 *   -Setting AXIS and DIRECTION singnals                                     *                              
 ******************************************************************************/
//void __ISR(_OUTPUT_COMPARE_1_VECTOR, IPL3SOFT) oc1_ISR(void);
void oc1_ISR(void)
{
    #ifdef OC1_ISR_DEBUG_OUTPUT 
    OC1_ISR_DEBUG_OUTPUT = 1;
    #endif
    
    switch(OC1R)
    {
        
        case ENABLE_ZCD_INTERRUPTS_TRIGGER:
        {
            //Reset ZCD trigger count
            zcd_count = 0;
            //Enable ZCD interrupts after clearing interrupt flag and emptying buffer
            while(!PLIB_IC_BufferIsEmpty(IC_ID_4))
            {
                /* Read the buffer value */
                singleMeasurement.zerocrossing[0] = PLIB_IC_Buffer32BitGet(IC_ID_4);
                //singleMeasurement.zerocrossing[0] = IC4BUF;
            }
            //Rising edges only
            //PLIB_IC_ModeSelect(IC_ID_4, IC_INPUT_CAPTURE_RISING_EDGE_MODE);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
            //Prepare maximumAmplitude
            maximumAmplitude = 0;
            //Set next trigger time
            OC1R = DISABLE_ZCD_INTERRUPTS_TRIGGER;
            break;
        }
        case DISABLE_ZCD_INTERRUPTS_TRIGGER:
        {
            //Disable ZCD interrupts (just in case something went wrong)
            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
            
            //Set next trigger time
            OC1R = AXIS_DIRECTION_TRIGGER;
            
            //Check if a debug copy of a single measurement has been requested
            if(singleMeasurementGet.inProgress)
            {
                //Check if the this measurement matches the one requested
                if(singleMeasurementGet.measurementId == (singleMeasurement.measurementId & singleMeasurementGet.measurementId_mask))
                {
                    //Make a copy
                    memcpy(singleMeasurementGet.target, &singleMeasurement, sizeof(SINGLE_MEASUREMENT));
                    //Reset request flag
                    singleMeasurementGet.inProgress = false;
                }
            }
            
            //Calculate time of flight
            measurementSet[0].tof[singleMeasurement.measurementId&0b11][singleMeasurement.measurementId>>2] = (*getTofPtr)(&singleMeasurement);
            
            //Calculate digipot setting for next measurement in this direction
            last_gain = (double) singleMeasurement.digipotValue;
            last_gain /= (double) (256-singleMeasurement.digipotValue);
            last_gain += 1.0;
            new_gain = last_gain;
            new_gain *= TARGET_AMPLITUDE_DAC_VALUE;
            new_gain /= (double) maximumAmplitude;
            new_gain = (0.3*new_gain)+(0.7*last_gain);
            if(new_gain>4.0)
                new_gain = 4.0;
            if(new_gain<1.0)
                new_gain = 1.0;
            digipot = 256.0 * (new_gain-1.0);
            digipot /= new_gain;
            digipotValues[singleMeasurement.measurementId&0b11] = (uint8_t) digipot;
            //digipotValues[singleMeasurement.measurementId&0b11] = 0;
            
            //Is a set of measurements completed?
            if(singleMeasurement.measurementId==TIMESLOT_MASK) 
            {
                //Check if a debug copy of an entire measurement set has been requested
                if(measurementSetGet.inProgress)
                {
                    //Make a copy
                    memcpy(measurementSetGet.target, &measurementSet[0], sizeof(MEASUREMENT_SET));
                    //Reset request flag
                    measurementSetGet.inProgress = false;
                }
                
                //Update indexes and increment 
                tmp = measurementSetCaptureIndex;
                measurementSetCaptureIndex = measurementSetProcessIndex;
                measurementSetProcessIndex = tmp;
                
                //Increment and set measurementSetId
                ++singleMeasurement.measurementSetId;
                measurementSet[measurementSetCaptureIndex].measurementSetId = singleMeasurement.measurementSetId;
                
                //Notify application that data set is ready
                appData.appState = APP_STATE_MEASUREMENT_SET_READY;
            }
 
            break;
        }
        case AXIS_DIRECTION_TRIGGER:
        {
            //Increase timeslot in the range 0...127
            ++singleMeasurement.measurementId;
            singleMeasurement.measurementId &= TIMESLOT_MASK;
            
            //Set digipotValue
            singleMeasurement.digipotValue = digipotValues[singleMeasurement.measurementId&0b11];
            app_i2c_internal_writeDigipot(singleMeasurement.digipotValue);
            
            //Set AXIS
            if(singleMeasurement.measurementId&AXIS_MASK)
            {
                PLIB_PORTS_PinSet(PORTS_ID_0, PIN_AXIS_CHANNEL, PIN_AXIS_BIT);
            }
            else
            {
                PLIB_PORTS_PinClear(PORTS_ID_0, PIN_AXIS_CHANNEL, PIN_AXIS_BIT);
            }
            //Set DIRECTION
            if(singleMeasurement.measurementId&DIRECTION_MASK)
            {
                PLIB_PORTS_PinSet(PORTS_ID_0, PIN_DIRECTION_CHANNEL, PIN_DIRECTION_BIT);
            }
            else
            {
                PLIB_PORTS_PinClear(PORTS_ID_0, PIN_DIRECTION_CHANNEL, PIN_DIRECTION_BIT);
            }
            //Set next trigger time
            OC1R = ENABLE_ZCD_INTERRUPTS_TRIGGER;
            break;
        }
    }
    
    //Clear interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_1);
    
    #ifdef OC1_ISR_DEBUG_OUTPUT 
    OC1_ISR_DEBUG_OUTPUT = 0;
    #endif
}



/******************************************************************************
 * Output Compare Module 2 Interrupt Service Routine                          *
 * Used for sending PWM pulses                                                *
 ******************************************************************************/
//void __ISR(_OUTPUT_COMPARE_2_VECTOR, IPL2SRS) oc2_ISR(void);
void oc2_ISR(void)
{
    #ifdef OC2_ISR_DEBUG_OUTPUT 
    OC2_ISR_DEBUG_OUTPUT = 1;
    #endif
    
    //Take care of sending pulses
    ++pulse_count;
    switch(pulse_count)
    {
        case 1:
        {
            OC2R = PWM_RISING_EDGE;
            OC2RS = PWM_FALLING_EDGE;
            break; 
        }    
        case NUMBER_OF_PULSES:
        {
            OC2R += PWM_PERIOD;
            OC2RS = TIMER_2_OVERFLOW;
            pulse_count = 0;
            break;
        }
        default:
        {
            OC2R += PWM_PERIOD;
            OC2RS += PWM_PERIOD;
            break;
        }
    }
    //Clear Interrupt Flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_2);
    
    #ifdef OC2_ISR_DEBUG_OUTPUT 
    OC2_ISR_DEBUG_OUTPUT = 0;
    #endif
}

/******************************************************************************
 * Input Capture Module 4 Interrupt Service Routine                           *
 * Used to process zero crossing detector (ZCD) input                         *                            
 ******************************************************************************/
void ic4_ISR(void)
{
    #ifdef IC4_ISR_DEBUG_OUTPUT 
    IC4_ISR_DEBUG_OUTPUT = 1;
    #endif

    //save ADC value
    if(zcd_count)
    {
        singleMeasurement.amplitude[zcd_count-1] = PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0);
        if(abs(singleMeasurement.amplitude[zcd_count-1])>maximumAmplitude)
        {
            maximumAmplitude = abs(singleMeasurement.amplitude[zcd_count-1]);
        }
        
        //Enable every edge to be captured (instead of rising only)
        //PLIB_IC_ModeSelect(IC_ID_4, IC_INPUT_CAPTURE_EVERY_EDGE_MODE);
    }

    //start an adc conversion
    PLIB_ADC_SamplingStart(ADC_ID_1);
    
    //Save captured values (careful: there might be two values in the cue if a 'wrong' edge was captured first (?))
    while(!PLIB_IC_BufferIsEmpty(IC_ID_4))
    {
        /* Read the buffer value */
        singleMeasurement.zerocrossing[zcd_count] = PLIB_IC_Buffer32BitGet(IC_ID_4);
        //singleMeasurement.zerocrossing[zcd_count++] = IC4BUF;
    }
    //Enforce rising-edge first
    if(zcd_count || PLIB_PORTS_PinGet(PORTS_ID_0, PIN_ZCD_CHANNEL, PIN_ZCD_BIT))
    {
       ++zcd_count; 
    }
    
    
    
    //Turn ZCD interrupts off if enough values have been captured
    if(zcd_count==NUMBER_OF_ZCD_CAPTURES)
    {  
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
    }
    
    //Clear interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
    
    #ifdef IC4_ISR_DEBUG_OUTPUT 
    IC4_ISR_DEBUG_OUTPUT = 0;
    #endif
}



void capture_init(uint32_t (*functionPtr)(SINGLE_MEASUREMENT *singleMeasurement))
{
	getTofPtr = functionPtr;
    
    singleMeasurement.measurementSetId = 0;
    singleMeasurement.measurementId = 0;
    singleMeasurement.digipotValue = 0;
    
    digipotValues[0] = 0;
    digipotValues[1] = 0;
    digipotValues[2] = 0;
    digipotValues[3] = 0;
    maximumAmplitude = 0;
    
    //Set up ADC
    // Internal Counter triggers conversion
    PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT);
    // Conversion speed
    PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 27); //Theoretisch 30
    PLIB_ADC_ConversionClockSet(ADC_ID_1, 48000000, 4800000); // TAD=208ns
    //References and Inputs
    PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS);
    PLIB_ADC_MuxChannel0InputPositiveSelect(ADC_ID_1, ADC_MUX_A, ADC_INPUT_POSITIVE_AN0);
    PLIB_ADC_MuxChannel0InputNegativeSelect(ADC_ID_1, ADC_MUX_A, ADC_INPUT_NEGATIVE_VREF_MINUS);
    // Result format
    PLIB_ADC_ResultFormatSelect(ADC_ID_1, ADC_RESULT_FORMAT_SIGNED_INTEGER_16BIT);
    // Enable the ADC
    PLIB_ADC_Enable(ADC_ID_1);

    //Set up OC1 used to generate interrupts at any time
    //OC1R = ENABLE_ZCD_INTERRUPTS_TRIGGER;
    //OC1CON = 0b1000000000100011; //On, 32bit mode, timer2, single compare mode toggle output
    PLIB_OC_TimerSelect(OC_ID_1, OC_TIMER_16BIT_TMR2);
    PLIB_OC_ModeSelect(OC_ID_1, OC_TOGGLE_CONTINUOUS_PULSE_MODE);
    PLIB_OC_BufferSizeSelect(OC_ID_1, OC_BUFFER_SIZE_32BIT);
    PLIB_OC_Buffer32BitSet(OC_ID_1, ENABLE_ZCD_INTERRUPTS_TRIGGER);
    PLIB_OC_Enable(OC_ID_1);
    
    //Set up OC2 used to generate PWM pulses
    PLIB_OC_TimerSelect(OC_ID_2, OC_TIMER_16BIT_TMR2);
    PLIB_OC_ModeSelect(OC_ID_2, OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE);
    PLIB_OC_BufferSizeSelect(OC_ID_2, OC_BUFFER_SIZE_32BIT);
    PLIB_OC_Buffer32BitSet(OC_ID_2, PWM_RISING_EDGE);
    PLIB_OC_PulseWidth32BitSet(OC_ID_2, PWM_FALLING_EDGE);
    PLIB_OC_Enable(OC_ID_2);

     /* Enable the OCMP module and start the timer */
    PLIB_TMR_Stop(TMR_ID_2);
    PLIB_TMR_ClockSourceSelect(TMR_ID_2, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
    PLIB_TMR_PrescaleSelect(TMR_ID_2, TMR_PRESCALE_VALUE_1);
    PLIB_TMR_Mode32BitEnable(TMR_ID_2);
    PLIB_TMR_Counter32BitClear(TMR_ID_2);
    PLIB_TMR_Period32BitSet(TMR_ID_2, TIMER_2_OVERFLOW);
    PLIB_TMR_Start(TMR_ID_2);
    
    //Set up Input Capture 4 module used to capture Zero Crossing Detector (ZCD)
    //IC4CON = 0b1000001101100110;
    
    //PLIB_IC_EventsPerInterruptSelect(IC_ID_4, IC_INTERRUPT_ON_EVERY_4TH_CAPTURE_EVENT);
    PLIB_IC_EventsPerInterruptSelect(IC_ID_4, IC_INTERRUPT_ON_EVERY_CAPTURE_EVENT);
    /* Stop in Idle mode is disabled for IC module. IC mode functions even in Idle mode */
    PLIB_IC_StopInIdleDisable(IC_ID_4);
    /* Every Edge Capture mode of operation is selected for IC module */
    PLIB_IC_ModeSelect(IC_ID_4, IC_INPUT_CAPTURE_EVERY_EDGE_MODE);
    //PLIB_IC_ModeSelect(IC_ID_4, IC_INPUT_CAPTURE_RISING_EDGE_MODE);
    /* Timer2 is selected as a clock source for the IC module */
    PLIB_IC_TimerSelect(IC_ID_4, IC_TIMER_TMR2);
    /* 32-bit timer is selected*/
    PLIB_IC_BufferSizeSelect(IC_ID_4, IC_BUFFER_SIZE_32BIT);
    /* Start with a rising edge */
    PLIB_IC_FirstCaptureEdgeSelect(IC_ID_4, IC_EDGE_RISING);
    //IC4CON &= 0b1111111111111110;
    //IC4CON |= 0b0000001000000110;
    /* Enable IC module */
    PLIB_IC_Enable(IC_ID_4);

    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_1);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_1);
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_2);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_2);
}

MEASUREMENT_SET* getMeasurementSetPtr()
{
    return &measurementSet[measurementSetProcessIndex];
}


void getSingleMeasurement(uint8_t ts, uint8_t ts_mask, SINGLE_MEASUREMENT *target)
{
	singleMeasurementGet.measurementId = ts;
	singleMeasurementGet.measurementId_mask = ts_mask;
	singleMeasurementGet.target = target;
	singleMeasurementGet.inProgress = true;
}

inline bool getSingleMeasurementInProgress(void)
{
	return singleMeasurementGet.inProgress;
}


void getMeasurementSet(MEASUREMENT_SET *target)
{
	measurementSetGet.target = target;
	measurementSetGet.inProgress = true;
}

inline bool getMeasurementSetInProgress(void)
{
	return measurementSetGet.inProgress;
}






