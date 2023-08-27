//*****************************************************************************
//
// quadrature.c - Final helicopter project for ENCE361
//
// Author:  cro98 rla104
// Last modified:   12.05.2023
//
// This file handles the quadrature encoder of the helicotper to give an
// accurate value for the current yaw of the helicopter.
// It also handles detecting if the helicopter is currently positioned at the
// reference value.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"

//*****************************************************************************
// Global variables
//*****************************************************************************
static int32_t channelAB = 0;
static int16_t yaw = 0;
static bool reference = 0;
static int16_t referenceYaw = 3600;

/*
 * Checks the current yaw of the helicopter and increments or decrements
 * a yaw counter. Goes back to 0 after 448 ticks in a single direction.
 * This yaw value needs to be converted to degrees since it is not in
 * degrees and only counts the steps.
 */
void
quadratureEncoder(int32_t tempChannelAB)
{

    if (tempChannelAB != channelAB) {

        switch(tempChannelAB) {

            case 0b00:
                if(channelAB == 0b10)
                {
                    yaw = yaw - 1;

                } else {
                    yaw = yaw + 1;

                }
                break;

            case 0b01:
                if(channelAB == 0b00)
                {
                    yaw = yaw - 1;

                } else {
                    yaw = yaw + 1;

                }
                break;

            case 0b11:
                if(channelAB == 0b01)
                {
                    yaw = yaw - 1;

                } else {
                    yaw = yaw + 1;

                }
                break;

            case 0b10:
                if(channelAB == 0b11)
                {
                    yaw = yaw - 1;

                } else {
                    yaw = yaw + 1;

                }
                break;
            }
    }

    if (yaw > 224)
    {
        yaw = -223;
    } else if( yaw < -223) {
        yaw = 224;
    }

}

//*****************************************************************************
//
// The interrupt handler for GPIO B.
//
//*****************************************************************************
int32_t
calculateYaw(void)
{
    return (yaw * 3600) / 448;
}

/*
 * The interupt handler for PORTB which is the quadrature encored pin0 and pin1.
 */
void
PortBIntHandler (void)
{
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_1);
    int32_t tempChannelAB = GPIOPinRead(GPIO_PORTB_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
    quadratureEncoder(tempChannelAB);
    channelAB = tempChannelAB;
}

/*
 * The interupt handler for PORTC which is the reference sensor. The interupt
 * is disabled once the reference value has been found.
 */
void
PortCIntHandler (void)
{
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    reference = GPIOPinRead(GPIO_PORTC_BASE, (GPIO_PIN_4));
    if (!reference)
    {
        referenceYaw = calculateYaw();
        GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
    }
}

/*
 * Initializes the reference sensor interupt for PORTC pin 4.
 */
void
initReferenceSensorInterupts (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }

    GPIOIntRegister(GPIO_PORTC_BASE, PortCIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,
                         GPIO_PIN_4);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);

    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);

}

/*
 * Initializes the interupt for PORTB pins 0 and 1.
 */
void
initYawSensorInterupts (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }

    GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,
                         GPIO_PIN_0 | GPIO_PIN_1);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    channelAB = GPIOPinRead(GPIO_PORTB_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
}

/*
 * Returns the boolean value of the reference sensor.
 */
bool
returnReference(void)
{
    return reference;
}

/*
 * Returns the yaw that is not in degrees.
 */
int16_t
returnRawYaw(void)
{
    return yaw;
}

/*
 * Returns the yaw that is in degrees * 10 to get the decimal value.
 */
int16_t
returnReferenceYaw (void)
{
    return referenceYaw;
}
