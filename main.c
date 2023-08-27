//*****************************************************************************
//
// main.c - Final helicopter project for ENCE361
//
// Author:  cro98 rla104
// Last modified:   12.05.2023
//
// The main program that handles the timings of the tasks that make a helicopter
// fly. This includes the systick interupt which is the centerpiece of the program.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "buttons4.h"
#include "circBufT.h"
#include "utils/ustdlib.h"
#include <stdio.h>
#include "quadrature.h"
#include "driverlib/gpio.h"
#include "view.h"
#include "pwmGen.h"
#include "slideSwitch.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 20
#define SAMPLE_RATE_HZ 300
#define MIN_ADC_VAL 1241
#define MAX_STR_LEN 16
#define ADC_TEN_PERCENT 124

// Possible states
#define LANDED 0
#define TAKEOFF 1
#define FLYING 2
#define LANDING 3

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer; // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt; // ADC sample count
static int32_t mean_val;     // Mean value from the buffer
static int32_t starting_val; // The value that the heli registers as 0% altitude
static int32_t altitude_percent; // Percentage value from 0 to 100 for altitude
static uint8_t current_state = LANDED;  // The state of the system for what
static int32_t target_alt = 0; // In percent, 100% = 1V adc difference
static int32_t target_yaw = 0; // In degrees
static int32_t yaw = 0;        // Actual yaw of helicopter in degrees
static int32_t reference = 0;  // The yaw in degrees of the reference point
static bool switch2 = 0;       // switch2 is the takeoff switch
static uint32_t ticks = 0;

static int32_t mainControl = 2;
static int32_t tailControl = 2;

char statusStr[MAX_STR_LEN + 1];
char currentStateStr[MAX_STR_LEN + 1];

static int32_t halfHzTick = 0; // tick counter for 150Hz
static int32_t fourHzTick = 0; // tick counter for 4Hz
static int32_t sixtyHzTick = 0; // tick counter for 60Hz
const int16_t ticksPerHalf = SAMPLE_RATE_HZ / 150;
const int16_t ticksPerFour = SAMPLE_RATE_HZ / 4;
const int16_t ticksPerSixty = SAMPLE_RATE_HZ / 60;
// Flags used for the scheduler
static bool fourHzFlag = false;
static bool sixtyHzFlag = false;
static bool halfHzFlag = false; // halfHz flag is 150Hz since systick is 300Hz

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
// Handles the time sensitive functions and sets up flags for them to
// be run in the main loop.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    reference = returnReferenceYaw();
    mainControl = mainPidCalculation(altitude_percent, target_alt);
    tailControl = tailPidCalculation(yaw, target_yaw);
    ticks++;

    if(++halfHzTick >= ticksPerHalf)
    {
        //
        // Initiate a conversion
        ADCProcessorTrigger(ADC0_BASE, 3);
        g_ulSampCnt++;
        halfHzFlag = true;
        halfHzTick = 0;
    }

    if(++fourHzTick >= ticksPerFour)
    {
        fourHzFlag = true;
        fourHzTick = 0;
    }

    if(++sixtyHzTick >= ticksPerSixty)
    {
        sixtyHzFlag = true;
        sixtyHzTick = 0;
    }
}

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

/*
 * Initializes the ADC with the correct port and ADC peripheral on the board as per
 * the project specifications and milestones.
 */
void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

/*
 * Initializes what is considered to be 0% altitude according to the mean ADC value.
 */
void
initAltitude()
{
   starting_val = mean_val;
   altitude_percent = 0;
}

/*
 * Calculates the ADC per volt value into a 100% per volt value.
 *
 * eg: Half a volt of change is 50%.
 */
void
altitudePercentage()
{
    int32_t temp_mean = mean_val;
    if(temp_mean > starting_val)
    {
        temp_mean = starting_val;
    }

    altitude_percent = ((starting_val - temp_mean) * 100) / (MIN_ADC_VAL);

}

/*
 * Calculates the mean of all the values inside the ADC buffer.
 */
void
calculateMeanVal()
{
    uint16_t i;
    int32_t sum = 0;
    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf (&g_inBuffer);
    mean_val = ((2 * sum) / 2 / BUF_SIZE);
}

/*
 * Initializes the pins used by the reset button.
 */
void
initResetButton()
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA,
           GPIO_PIN_TYPE_STD_WPU);
}

/*
 * Main handles initializing the components and checking if flags are raised
 * to handle the processes needed at certain intervals as soon as possible.
 * Any processes that need to happen on time are handled in the interupt.
 */
int
main(void)
{
    //
    // Resetting buttons before using them
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH);
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOA);

    //
    // Initializing all functions that involve timers, ADC, Display, Uart
    // Interupts, Buttons, PWM, Swithes.
    initButtons ();
    initClock ();
    initADC ();
    initYawSensorInterupts ();
    initialiseUSB_UART ();
    initDisplay ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initialiseMainPWM ();
    initialiseTailPWM ();
    initReferenceSensorInterupts ();
    initSlideSwitch ();
    initResetButton();


    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    //
    // Wait until the buffer is completely filled
    // before getting a mean value.
    // Used value of 180 since 40 still caused problems
    // with the initial mean ADC.
    while(g_ulSampCnt < 180)
    {
    }

    //
    // Calculating the mean value and setting up the initial altitude
    // after the buffer has been completely filled.
    calculateMeanVal ();
    initAltitude ();
    altitudePercentage ();

    while (1)
    {
        // Check to see if the reset button has been pressed.
        if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == false)
        {
            SysCtlReset();
        }

        // Making sure that buttons can only have an effect if the helicopter
        // is flying.
        if (current_state == FLYING)
        {
            if (checkButton (UP) == PUSHED)
                {
                    if (target_alt + 10  > 100)
                    {
                        target_alt = 100;
                    }
                    else
                    {
                        target_alt += 10;
                    }
                }

                if (checkButton (DOWN) == PUSHED)
                {
                    if (target_alt - 10 < 10)
                    {
                        target_alt = 10;
                    }
                    else
                    {
                        target_alt -= 10;
                    }
                }

                if (checkButton (RIGHT) == PUSHED)
                {
                    if (target_yaw + 15 > 180)
                    {
                        target_yaw = -165;
                    }
                    else
                    {
                        target_yaw += 15;
                    }
                }

                if (checkButton (LEFT) == PUSHED)
                {
                    if (target_yaw - 15 <= -180)
                    {
                        target_yaw = 165;
                    }
                    else
                    {
                        target_yaw -= 15;
                    }
                }
        }

        // Checking the flags
        if (halfHzFlag)
        {
            halfHzFlag = false;
            calculateMeanVal ();
            altitudePercentage ();
            updateButtons ();
            switch2 = getSwitch2 ();
            setMainPWM(mainControl);
            setTailPWM(tailControl);
        }

        if (sixtyHzFlag)
        {
            sixtyHzFlag = false;
            yaw = calculateYaw ();
            reference = returnReference ();
            displayAltitude(target_yaw, target_alt);
            //displayMeanVal(mean_val, starting_val);
            switch (current_state)
                {
                case LANDED:
                    usprintf (currentStateStr, "LANDED");
                    if (switch2)
                    {
                        enableMainPWM ();
                        enableTailPWM ();
                        current_state = TAKEOFF;
                    }
                    break;
                case LANDING:
                    usprintf (currentStateStr, "LANDING");
                    if (altitude_percent == 0 && target_alt == 0)
                    {
                        disableMainPWM ();
                        disableTailPWM ();
                        current_state = LANDED;
                    }
                    break;
                case TAKEOFF:
                    usprintf (currentStateStr, "TAKEOFF");
                    target_alt = 5;
                    if (reference != 3600)
                    {
                        target_alt = 10;
                        target_yaw = reference/10;
                        current_state = FLYING;
                    }
                    break;
                case FLYING:
                    usprintf (currentStateStr, "FLYING");
                    if (!switch2)
                    {
                        target_yaw = reference/10;
                        current_state = LANDING;
                    }
                    break;
                }
        }

        if (fourHzFlag)
        {
            if (reference == 3600 && current_state == TAKEOFF)
            {
                if (target_yaw + 1 > 180)
                    {
                        target_yaw = -179;
                    }
                    else
                    {
                        target_yaw += 3;
                    }
            }
            if (current_state == LANDING && yaw/10 < target_yaw + 2 && yaw/10 > target_yaw - 2)
            {
                if (target_alt - 2 < 0)
                {
                    target_alt = 0;
                }
                else
                {
                    target_alt-=1;
                }
            }
            fourHzFlag = false;
            usprintf (statusStr, "alt=%3d [%3d]\r\n", altitude_percent, target_alt);
            UARTSend (statusStr);
            usprintf (statusStr, "yaw=%4d [%4d]\r\n", yaw/10, target_yaw);
            UARTSend (statusStr);
            usprintf (statusStr, "main %3d tail %3d\r\n", mainControl, tailControl);
            UARTSend (statusStr);
            UARTSend (currentStateStr);
            usprintf (statusStr, "----------------\r\n");
            UARTSend (statusStr);
        }

    }
}

