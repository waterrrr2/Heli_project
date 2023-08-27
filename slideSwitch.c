//*****************************************************************************
//
// slideWitch.c - Final helicopter project for ENCE361
//
// Author:  cro98 rla104
// Last modified:   12.05.2023
//
// slideswitch.c is the file that handles the slideswitch used for takeoff.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "stdlib.h"

static bool switch2 = 0;

/*
 * Initializes the slide switch.
 */
void
initSlideSwitch (void)
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA,
           GPIO_PIN_TYPE_STD_WPD);

    switch2 = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
}

/*
 * Returns the current state of the switch. HIGH is on and LOW is off.
 */
bool
getSwitch2 (void)
{
    switch2 = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
    return switch2;
}

