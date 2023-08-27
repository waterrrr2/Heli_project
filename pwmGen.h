#ifndef PWMGEN_H_
#define PWMGEN_H_

/*
 * Initializes the main rotor PWM generator on PC5.
 */
void
initialiseMainPWM (void);

/*
 * Initializes the tail rotor PWM generator PF1.
 */
void
initialiseTailPWM (void);

/*
 * Sets the PWM for the main rotor given a duty cycle in %.
 */
void
setMainPWM (uint32_t ui32Duty);

/*
 * Sets the tail PWM given a duty cycle in %/
 */
void
setTailPWM (uint32_t ui32Duty);

// Enables the main rotor PWM generator
void
enableMainPWM (void);

// Disables the main rotor PWM generator
void
disableMainPWM (void);

// Enables the tail rotor PWM generator
void
enableTailPWM (void);

// Disables the tail rotor PWM generator
void
disableTailPWM (void);

/*
 * Controls the main rotos using PID controller and its gains
 * that are constants in the pwmGen.c file
 */
uint32_t
mainPidCalculation(uint32_t altitude, uint32_t targetAlt);

/*
 * Controls the tail PWM based on the target yaw and the current yaw given
 * and gives a control output to feed to the pwm generator as a duty cycle.
 */
uint32_t
tailPidCalculation(int32_t yaw, int32_t targetYaw);

#endif
