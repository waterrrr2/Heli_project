//*****************************************************************************
//
// view.c - Final helicopter project for ENCE361
//
// Author:  cro98 rla104
// Last modified:   12.05.2023
//
// view.c handles all of the processes related to showing the user information.
// This includes the uart and oled display for the tiva board.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include <stdio.h>
#include "driverlib/uart.h"

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

/*
 * Initialises all of the UART peripherals to communicate via USB.
 */
void
initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

/*
 * Sends a message formated as a char through the UART.
 */
void
UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}

/*
 * Initialises the OLED display.
 */
void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

/*
 * Displays the altitude and yaw of the helicotper on the OLED display.
 */
void
displayAltitude(int32_t small_yaw, int32_t altitude_percent)
{

    char string[17];  // 16 characters across the display

    //OLEDStringDraw ("Current Altitude", 0, 0);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Altitude = %3d", altitude_percent);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    if(small_yaw/10 == 0 && small_yaw < 0)
    {
        usnprintf (string, sizeof(string), "Yaw =   -0.%01d", abs(small_yaw % 10));
    }
    else
    {
        usnprintf (string, sizeof(string), "Yaw = %4d.%01d", small_yaw/10, abs(small_yaw % 10));
    }

    OLEDStringDraw (string, 0, 2);

}

/*
 * Displays the mean adc value and the starting value of the helicoter on the OLED.
 */
void
displayMeanVal(int32_t mean_val, int32_t starting_val)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Milestone 1", 0, 0);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Mean ADC = %4d", mean_val);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    usnprintf (string, sizeof(string), "start# %5d", starting_val);
    OLEDStringDraw (string, 0, 3);
}

/*
 * Wipes the OLED display manually.
 */
void
wipeDisplay(void)
{
    OLEDStringDraw ("                ", 0, 0);
    OLEDStringDraw ("                ", 0, 1);
    OLEDStringDraw ("                ", 0, 2);
    OLEDStringDraw ("                ", 0, 3);
}
