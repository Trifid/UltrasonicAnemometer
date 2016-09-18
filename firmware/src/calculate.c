
#include "calculate.h"
#include "app.h"

// Calculation parameters
//int16_t kernel[KERNEL_LENGTH] = {-22800, 23700, -24600, 25500, -26400, 27300, -28200, 29100, -30000, 28900, -27800, 26700, -25600, 24500, -23400, 22300, -21200};
//int16_t kernel[KERNEL_LENGTH] = {-22800, 0, -24600, 0, -26400, 0, -28200, 0, -30000, 0, -27800, 0, -25600, 0, -23400, 0, -21200};
//int16_t kernel[KERNEL_LENGTH] = {-1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
int16_t kernel[KERNEL_LENGTH] = {347,-384,386,-418,406,-429,423,-443,461,-444,426,-422,405,-411,384,-380,348};

uint32_t getTimeOfFlight(SINGLE_MEASUREMENT *singleMeasurement)
{
    //PGD_READY_LAT = 1;
    
	uint8_t sample_idx = 0;
	uint8_t kernel_idx = 0;
	int32_t result = 0;
	int32_t maximum_result = 0;
	uint8_t maximum_result_idx = 0;
    
    for(sample_idx=0; sample_idx<(NUMBER_OF_AMPLITUDE_CAPTURES-KERNEL_LENGTH+1); sample_idx+=2)
    {
        result = 0;
		for(kernel_idx=0; kernel_idx<KERNEL_LENGTH; ++kernel_idx)
		{
			result += (*singleMeasurement).amplitude[sample_idx+kernel_idx] * kernel[kernel_idx];
		}
        //result -= (*singleMeasurement).amplitude[sample_idx];
        
        if(result>maximum_result)
        {
            maximum_result = result;
            maximum_result_idx = sample_idx;
        }
    }
    
    result = 0;
	for(sample_idx=maximum_result_idx; sample_idx<(maximum_result_idx+NUMBER_OF_ZEROCROSSINGS_TO_ADD); ++sample_idx)
	{
		result += (*singleMeasurement).zerocrossing[sample_idx];
	}
    //result = (*singleMeasurement).zerocrossing[maximum_result_idx+8];
    
    //PGD_READY_LAT = 0;
    return result;
}


int cmpfunc(const void *a, const void *b)
{
   return (*(int*)a - *(int*)b);
}


void calculateAverageTimeOfFlight(uint32_t *resultPtr, MEASUREMENT_SET *measurementPtr)
{
    uint8_t direction_idx;
    uint8_t measurement_idx;
    uint32_t tmp;
    
    for(direction_idx=0; direction_idx<NUMBER_OF_DIRECTIONS; ++direction_idx)
    {
        //Sort array
        qsort(&(*measurementPtr).tof[direction_idx][0], NUMBER_OF_MEASUREMENTS, sizeof(uint32_t), cmpfunc);
                
        tmp = 0;
        
        //Sum up middle 8 elements
        for(measurement_idx=(NUMBER_OF_MEASUREMENTS>>3)-4; measurement_idx<(NUMBER_OF_MEASUREMENTS>>3)+4; ++measurement_idx)
        {
            tmp += (*measurementPtr).tof[direction_idx][measurement_idx];
        }
        
        // Make the unit nanoseconds
        // Base unit is clock cycles, i.e 1/48000000
        // Multiply by 1000 and divide by 48 * [number of zerocrossings per measurement] * [number of averaged measurements]
        // Multiply by 1000/(48*16*8) = 125/768
        // Make sure the intermediate result (after multiplication) does not exceed the uint32_t range
        // Hint: wolframalpha.com is a great tool for simplifying fractions
        tmp *= 125;
        tmp /= 768;
        
        //Correct for number of pulses
        tmp -= ((NUMBER_OF_PULSES+4) * 25000);
        tmp -= 12500;
        
        resultPtr[direction_idx] = tmp;
    }
}

#define DISTANCE_NS 0.245
#define DISTANCE_EW 0.245

//Time of flight are measured in nanoseconds
//Distance is measured in meters
//Returned value is in meters per second (m/s)
long double getSpeedOfSound(uint32_t timeOfFlight1, uint32_t timeOfFlight2, long double distance)
{
    long double t1;
    long double t2;
    long double speedOfSound;
    
    //timeOfFlight1 -= (25000 * 16);
    t1 = (long double) timeOfFlight1;
    
    //timeOfFlight2 -= (25000 * 16);
    t2 = (long double) timeOfFlight2;
    
    //Compensating for nanoseconds
    //Includes a factor of 0.5 to save a multiplication later on
    distance *= 500000000.0; 
    
    speedOfSound = (distance/t1) + (distance/t2);
    return speedOfSound;
}

//Time of flight are measured in nanoseconds
//Distance is measured in meters
//Returned value is in meters per second (m/s)
long double getWindSpeed(uint32_t timeOfFlight1, long double distance, long double speedOfSound)
{
    long double t1;
    long double windSpeed;
    
    //timeOfFlight1 -= (25000 * 16);
    t1 = (long double) timeOfFlight1;
    
    //Compensating for nanoseconds
    distance *= 1000000000.0;
    
    windSpeed = (distance/t1) - speedOfSound;
    return windSpeed;
}

long double getTemperature(long double speedOfSound)
{
    return (speedOfSound - 331.0) / 0.6; 
}

void calculateWindSpeed(RESULTS *resultPtr, uint32_t *timeOfFlightPtr)
{
    long double speedOfSound;
    long double windSpeed; 
    long double temperature;

    speedOfSound = getSpeedOfSound(timeOfFlightPtr[0], timeOfFlightPtr[2], DISTANCE_NS);
    windSpeed = getWindSpeed(timeOfFlightPtr[0], DISTANCE_NS, speedOfSound);
    (*resultPtr).windSpeed[0] = (int32_t) (1000.0 * windSpeed);
    
    temperature = getTemperature(speedOfSound);
    
    speedOfSound = getSpeedOfSound(timeOfFlightPtr[1], timeOfFlightPtr[3], DISTANCE_NS);
    windSpeed = getWindSpeed(timeOfFlightPtr[1], DISTANCE_NS, speedOfSound);
    (*resultPtr).windSpeed[1] = (int32_t) (1000.0 * windSpeed);
    
    temperature += getTemperature(speedOfSound);
    temperature /= 2.0;
    (*resultPtr).temperature = (int16_t) (100.0*temperature);
}

