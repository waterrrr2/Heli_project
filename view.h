#ifndef VIEW_H_
#define VIEW_H_

void
initialiseUSB_UART (void);

void
UARTSend (char *pucBuffer);

void
initDisplay (void);

void
displayAltitude(int32_t small_yaw, int32_t altitude_percent);

void
displayMeanVal(int32_t mean_val, int32_t starting_val);

void
wipeDisplay(void);

#endif
