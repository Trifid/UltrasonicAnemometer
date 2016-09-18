
#ifndef _CAPTURE_H
#define _CAPTURE_H

#include "app.h"

extern APP_DATA appData;

void capture_init(uint32_t (*functionPtr)(SINGLE_MEASUREMENT *singleMeasurement));

MEASUREMENT_SET* getMeasurementSetPtr();

void getSingleMeasurement(uint8_t ts, uint8_t ts_mask, SINGLE_MEASUREMENT *target);
bool getSingleMeasurementInProgress(void);

void getMeasurementSet(MEASUREMENT_SET *target);
bool getMeasurementSetInProgress(void);

#endif /* _CAPTURE_H */