//*****************************************************************************
//
// pwmGen.c - Final helicopter project for ENCE361
//
// Author:  cro98 rla104
// Last modified:   12.05.2023
//
// pwmGen.c is a program that generates PWM signals on two pins of the TIVA board
// with the goal of making a helicotper fly. Ther are two different PWM controllers
// being used for the main and tail rotors. This file contains a PID controller
// that relies on outside calls to receive the necessary information to make adjustments.
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
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "stdlib.h"

#define KPM 6000
#define KPT 60000
#define KIM 15
#define KIT 45

#define DIVISOR 10000

// PWM configuration
#define PWM_RATE_HZ   200
#define PWM_TAIL_DUTY      2
#define PWM_MAIN_DUTY      2
#define PWM_MIN_DUTY       3
#define PWM_MAX_DUTY       60
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        1

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M0PWM7 (gen 2)
//  ---Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

static int32_t main_I = 0; // The initial integral gain accumulator for the rotor.
static int32_t tail_I = 0; // The initial integral gain accumulator for the tail.

/*
 * Sets the PWM for the main rotor given a duty cycle in %.
 */
void
setMainPWM (uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32Duty / 100);
}

/*
 * Sets the tail PWM given a duty cycle in %/
 */
void
setTailPWM (uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
     SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
        ui32Period * ui32Duty / 100);
}

/*
 * Initializes the main rotor PWM generator on PC5.
 */
void
initialiseMainPWM (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setMainPWM (PWM_MAIN_DUTY);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}

/*
 * Initializes the tail rotor PWM generator PF1.
 */
void
initialiseTailPWM (void)
{
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
             PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setTailPWM (PWM_TAIL_DUTY);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

// Enables the main rotor PWM generator
void
enableMainPWM (void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
}

// Disables the main rotor PWM generator
void
disableMainPWM (void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}

// Enables the tail rotor PWM generator
void
enableTailPWM (void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

// Disables the tail rotor PWM generator
void
disableTailPWM (void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

/*
 * Controls the main rotos using PID controller and its gains
 * that are constants in the pwmGen.c file
 */
uint32_t
mainPidCalculation(uint32_t altitude, uint32_t targetAlt)
{
    int32_t error = targetAlt - altitude;
    //Limiting the size of the error to avoid spikes in control output
    if (error > 10)
        {
            error = 10;
        }
    else if (error < -10)
        {
            error = -10;
        }
    int32_t proportional = KPM * error;
    main_I += KIM * error;
    int32_t control = ((proportional + main_I) / DIVISOR);
    if (control < PWM_MIN_DUTY)
    {
        control  = PWM_MIN_DUTY;
    }
    else if (control > PWM_MAX_DUTY)
    {
        control = PWM_MAX_DUTY;
    }
    return control;
}

/*
 * Controls the tail PWM based on the target yaw and the current yaw given
 * and gives a control output to feed to the pwm generator as a duty cycle.
 */
uint32_t
tailPidCalculation(int32_t yaw, int32_t targetYaw)
{
    int32_t error = 0;
    error = targetYaw - yaw/10;
    // Making sure that the shortest path is always chosen
    // since 180 and -179 is used instead of 360.
    if (error > 180)
    {
        error -= 360;
    }
    else if (error < -180)
    {
        error += 360;
    }
    /*if (error > 10)
    {
        error = 10;
    }
    else if (error < -10)
    {
        error = -10;
    }*/
    int32_t proportional = KPT * error;
    tail_I += KIT * error;
    if (error == 0)
    {
        tail_I = 0;
    }
    int32_t control = ((proportional + tail_I)/DIVISOR);
    if (control < PWM_MIN_DUTY)
    {
        control  = PWM_MIN_DUTY;
    }
    else if (control > PWM_MAX_DUTY)
    {
        control = PWM_MAX_DUTY;
    }
    return control;
}
