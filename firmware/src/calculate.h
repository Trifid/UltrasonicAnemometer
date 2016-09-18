
#include "app.h"

#define NUMBER_OF_ZEROCROSSINGS_TO_ADD 16

void oc1_ISR(void);
void oc2_ISR(void);
void c4_ISR(void);

uint32_t getTimeOfFlight(SINGLE_MEASUREMENT *singleMeasurement);
uint32_t getAverageTimeOfFlight(uint32_t *tofArray);
void calculateAverageTimeOfFlight(uint32_t *resultPtr, MEASUREMENT_SET *measurementPtr);
void calculateWindSpeed(RESULTS *resultPtr, uint32_t *timeOfFlightPtr);
