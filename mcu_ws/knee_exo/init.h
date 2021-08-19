#ifndef HEADER_H
#define HEADER_H

#include "valve.h"
#include "driverlib.h"
#include "msp.h"


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <array>


// function prototypes
void configClocks(void);
void configValvePins(valve_t * valve);
void configGpioPins(void);
void configAnalog(void);
void configTimers(void);
void configUart(void);
void configInterrupts(void);
void startTimers(void);


// macros
#define PWM_PERIOD 60000 // pulse-width modulation timer period


// timer configurations
extern const Timer_A_PWMConfig pwmTimerConfig1; // PWM timer
extern const Timer_A_PWMConfig pwmTimerConfig2; // PWM timer
extern const Timer_A_PWMConfig pwmTimerConfig3; // PWM timer
extern const Timer_A_PWMConfig pwmTimerConfig4; // PWM timer
extern const Timer_A_PWMConfig pwmTimerConfig5; // PWM timer
extern const Timer_A_UpModeConfig sensorTimerConfig; // sensor timer
extern const Timer_A_UpModeConfig controlTimerConfig; // control timer

// UART configuration
extern const eUSCI_UART_Config uartConfig;


#endif
