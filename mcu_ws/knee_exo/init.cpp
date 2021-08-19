#include <init.h>


// PWM timer configurations: 40 ms period (25 Hz)
const Timer_A_PWMConfig pwmTimerConfig1 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,          // tie timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_1,      // increment counter every 4 clock cycles
    PWM_PERIOD,                         // period of timer A
    TIMER_A_CAPTURECOMPARE_REGISTER_1,  // use capture compare register 1
    TIMER_A_OUTPUTMODE_RESET_SET,       // use reset set output mode
    0                                   // set initial duty cycle
};
const Timer_A_PWMConfig pwmTimerConfig2 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    PWM_PERIOD,
    TIMER_A_CAPTURECOMPARE_REGISTER_2,  // use capture compare register 2
    TIMER_A_OUTPUTMODE_RESET_SET,
    0
};
const Timer_A_PWMConfig pwmTimerConfig3 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    PWM_PERIOD,
    TIMER_A_CAPTURECOMPARE_REGISTER_3,  // use capture compare register 3
    TIMER_A_OUTPUTMODE_RESET_SET,
    0
};
const Timer_A_PWMConfig pwmTimerConfig4 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    PWM_PERIOD,
    TIMER_A_CAPTURECOMPARE_REGISTER_4,  // use capture compare register 4
    TIMER_A_OUTPUTMODE_RESET_SET,
    0
};
const Timer_A_PWMConfig pwmTimerConfig5 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    PWM_PERIOD,
    TIMER_A_CAPTURECOMPARE_REGISTER_5,  // use capture compare register 5
    TIMER_A_OUTPUTMODE_RESET_SET,
    0
};

// sensor measurement timer configuration: 500 Hz
const Timer_A_UpModeConfig sensorTimerConfig = // configure timer A in up mode
{
    TIMER_A_CLOCKSOURCE_SMCLK,          // tie timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_1,      // increment counter every 4 clock cycles
    3000,                              // period of timer A
    TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // enable capture compare interrupt
    TIMER_A_DO_CLEAR                    // clear counter upon initialization
};

// control update timer configuration: 1000 Hz
const Timer_A_UpModeConfig controlTimerConfig = // configure timer A in up mode
{   TIMER_A_CLOCKSOURCE_SMCLK,          // tie timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_1,      // increment counter every 4 clock cycles
    3000,                               // period of timer A
    TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // enable capture compare interrupt
    TIMER_A_DO_CLEAR                    // clear counter upon initialization
};

// UART configuration: 115200 baud rate
const eUSCI_UART_Config uartConfig = // configuration struct
{   EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK clock source
    13,                              // clock prescalar
    0,                             // firstModReg
    0,                              // secondModReg
    EUSCI_A_UART_NO_PARITY,         // no parity
    EUSCI_A_UART_LSB_FIRST,         // least significant bit first
    EUSCI_A_UART_ONE_STOP_BIT,      // one stop bit
    EUSCI_A_UART_MODE,              // UART mode
    0x00 //EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // oversampling set to 1
};


void configClocks(void) {
    /* set up clocks */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1); // higher voltage level to support 48 MHz
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_setDCOFrequency(48000000);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, 1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);
}


void configValvePins(valve_t * valve) {
    /* configure valve control pins */
    MAP_GPIO_setAsOutputPin(valve->port, valve->pin_inflate | valve->pin_vent);
    MAP_GPIO_setOutputLowOnPin(valve->port, valve->pin_inflate | valve->pin_vent);
}


void configGpioPins(void){
    /* configure general purpose pins */
    // configure pins P1.2 and P1.3 for UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, (GPIO_PIN2 | GPIO_PIN3), GPIO_PRIMARY_MODULE_FUNCTION);
    // configure buttons as inputs
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1); // button 1
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4); // button 2
    // configure LEDs
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 |GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0| GPIO_PIN1 |GPIO_PIN2);
    // configure debug pins
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
}


void configAnalog(void) {
    /* configure input pins for analog sensors and ADC */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION); // A0  --> P5.5
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION); // A1  --> P5.4
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION); // A2  --> P5.3
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION); // A3  --> P5.2
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION); // A4  --> P5.1
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION); // A5  --> P5.0
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN7, GPIO_TERTIARY_MODULE_FUNCTION); // A6  --> P4.7
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION); // A7  --> P4.6
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION); // A8  --> P4.5
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION); // A9  --> P4.4
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION); // A10 --> P4.3
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION); // A11 --> P4.2
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION); // A12 --> P4.1
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION); // A13 --> P4.0

    MAP_ADC14_enableModule(); // enable ADC
    MAP_ADC14_setResolution(ADC_12BIT); // set resolution of ADC module
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_1, ADC_NOROUTE); // use sub-master clock, external mapping // TODO: slower clock?
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM7, 0); // configure multiple memory sample scheme, no repeat

    MAP_ADC14_configureConversionMemory(ADC_MEM0,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
    MAP_ADC14_configureConversionMemory(ADC_MEM1,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM2,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A2,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
    MAP_ADC14_configureConversionMemory(ADC_MEM3,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A3,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
    MAP_ADC14_configureConversionMemory(ADC_MEM4,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A4,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
    MAP_ADC14_configureConversionMemory(ADC_MEM5,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A5,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
    MAP_ADC14_configureConversionMemory(ADC_MEM6,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A6,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
    MAP_ADC14_configureConversionMemory(ADC_MEM7,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A7,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM8,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A8,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM9,  ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9,  ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM10, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A10, ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM11, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM12, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A12, ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module
//    MAP_ADC14_configureConversionMemory(ADC_MEM13, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS); // configure individual memory location for ADC module

    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION); // configure sample timer to step through sequence
    MAP_ADC14_enableConversion();                         // enable conversion of ADC data
}



void configTimers(void) {
    // configure PWM timers: Timer A0
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmTimerConfig1); // TA0.1 --> P2.4
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmTimerConfig2); // TA0.2 --> P2.5
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmTimerConfig3); // TA0.3 --> P2.6
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmTimerConfig4); // TA0.4 --> P2.7
//    int i;
//    for (i = 0; i < NUM_PRES_SENSOR; i++) {
//        TIMER_A0->CCR[i+1] = 0; // update PWM timer duty cycle
//    }

    // configure sensor measurement timer: Timer A1
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &sensorTimerConfig); // configure timer
    MAP_Interrupt_enableInterrupt(INT_TA1_0);                       // enable timer interrupt on NVIC module
    // configure control loop timer: Timer A2
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &controlTimerConfig); // configure timer
    MAP_Interrupt_enableInterrupt(INT_TA2_0);                        // enable timer interrupt on NVIC module
}


void configUart(void) {
    // configure UART
    MAP_Interrupt_setPriority(INT_EUSCIA0, 0); // set highest interrupt priority
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig); // configure UART module instance A0
    MAP_UART_enableModule(EUSCI_A0_BASE); // enable UART module instance A0
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // clear receive interrupt flag
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // enable receive interrupt
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0); // enable interrupt for UART instance A0 in NVIC
}

void configInterrupts(void) {
    // configure button interrupts
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4); // clear interrupt flags
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);    // enable interrupts
    MAP_Interrupt_enableInterrupt(INT_PORT1);                         // enable interrupt in NVIC
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION); // set high to low transition
}

void startTimers(void) {
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);  // start Timer A0
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);  // start Timer A1
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);  // start Timer A2
}

