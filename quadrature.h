#ifndef QUADRATURE_H_
#define QUADRATURE_H_

#include <stdint.h>

void
PortBIntHandler (void);

void
PortCIntHandler (void);

void
initReferenceSensorInterupts (void);

void
initYawSensorInterupts (void);

void
quadratureEncoder(int32_t tempChannelAB);

int32_t
calculateYaw(void);

bool
returnReference(void);

int16_t
returnRawYaw(void);

int16_t
returnReferenceYaw (void);

#endif
