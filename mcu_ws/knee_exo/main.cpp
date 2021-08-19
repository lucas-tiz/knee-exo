//*****************************************************************************
//
// MSP432 main.c - knee exo control
//
// Lucas Tiziani
// 26 September 2019
//
//****************************************************************************


#include "init.h"
#include "byte_stuff.hpp"
#include "valve.h"
#include "util.h"

#include "driverlib.h"
#include "msp.h"

#include <vector>
#include <array>
#include <algorithm>
#include <iterator>


////////////////////////////////////////////////////////////////////////////////
/* Macros */
#define NUM_CHAN_MUSC 2 // number of muscle pressure channels
#define NUM_CHAN_BLAD 2 // number of bladder pressure channels

#define LEN_BLAD_FILT 10
#define NUM_UART_TX 9
#define SEND_DATA_COUNT 10 // number of sensor loops between each data send

#define CHAN_ADC_TANK    3
#define CHAN_ADC_MUSC_R0 0
#define CHAN_ADC_MUSC_R1 1
#define CHAN_ADC_MUSC_L0 6
#define CHAN_ADC_MUSC_L1 7
#define CHAN_ADC_BLAD_R  4
#define CHAN_ADC_BLAD_L  5

#define PORT_VALVE_MUSC_R       GPIO_PORT_P6
#define PIN_VALVE_MUSC_INF_R    GPIO_PIN7
#define PIN_VALVE_MUSC_VENT_R   GPIO_PIN6
#define PORT_VALVE_MUSC_L       GPIO_PORT_P3
#define PIN_VALVE_MUSC_INF_L    GPIO_PIN6
#define PIN_VALVE_MUSC_VENT_L   GPIO_PIN5

#define PORT_VALVE_BLAD_R       GPIO_PORT_P3
#define PIN_VALVE_BLAD_INF_R    GPIO_PIN2
#define PIN_VALVE_BLAD_VENT_R   GPIO_PIN3
#define PORT_VALVE_BLAD_L       GPIO_PORT_P6
#define PIN_VALVE_BLAD_INF_L    GPIO_PIN4
#define PIN_VALVE_BLAD_VENT_L   GPIO_PIN5


////////////////////////////////////////////////////////////////////////////////
/* Global variables */
float g_mode = 1;

volatile float g_uart_rx[63];

volatile int g_flag_sense = 0;
volatile bool g_flag_control = 0;
volatile bool g_flag_receive = 0;


////////////////////////////////////////////////////////////////////////////////
/* Prototypes */
void sensorUpdate(const int * adc_tank, float * pres_tank,
                  const int * adc_musc, float * pres_musc,
                  const int * adc_bladder, float * pres_bladder);
void controlMuscUpdate(float * pres_musc, float * pres_des_musc,
                       valve_t * valves);
void controlBladderUpdate(float * pres_bladder, float * pres_des_bladder,
                          valve_t * valves);
void receiveData(float * pres_des_musc, float * pres_des_bladder,
                 float * ctrl_params_musc, float * ctrl_params_bladder);
void sendData(float * uart_tx, int n_float_tx);

extern "C" void TA1_0_IRQHandler(void);
extern "C" void TA2_0_IRQHandler(void);
extern "C" void EUSCIA0_IRQHandler(void);


////////////////////////////////////////////////////////////////////////////////
/* Main */
void main(void) {
    ////////////////////////////////////////////////////////////////////////////////
    /* Configure MSP */
    MAP_WDT_A_holdTimer(); // hold the watchdog timer (stop from running)
    MAP_Interrupt_disableMaster(); // disable interrupts
    MAP_FPU_enableModule();

    const int adc_tank[1] = {CHAN_ADC_TANK};
    const int adc_musc[NUM_CHAN_MUSC*2] = {CHAN_ADC_MUSC_R0, CHAN_ADC_MUSC_R1,
                                           CHAN_ADC_MUSC_L0, CHAN_ADC_MUSC_L1};
    const int adc_bladder[NUM_CHAN_BLAD] = {CHAN_ADC_BLAD_R, CHAN_ADC_BLAD_L};

    valve_t valves_musc[NUM_CHAN_MUSC];
    valves_musc[0].port = PORT_VALVE_MUSC_R;
    valves_musc[0].pin_inflate = PIN_VALVE_MUSC_INF_R;
    valves_musc[0].pin_vent = PIN_VALVE_MUSC_VENT_R;
    valves_musc[1].port = PORT_VALVE_MUSC_L;
    valves_musc[1].pin_inflate = PIN_VALVE_MUSC_INF_L;
    valves_musc[1].pin_vent = PIN_VALVE_MUSC_VENT_L;

    valve_t valves_bladder[NUM_CHAN_BLAD];
    valves_bladder[0].port = PORT_VALVE_BLAD_R;
    valves_bladder[0].pin_inflate = PIN_VALVE_BLAD_INF_R;
    valves_bladder[0].pin_vent = PIN_VALVE_BLAD_VENT_R;
    valves_bladder[1].port = PORT_VALVE_BLAD_L;
    valves_bladder[1].pin_inflate = PIN_VALVE_BLAD_INF_L;
    valves_bladder[1].pin_vent = PIN_VALVE_BLAD_VENT_L;

    configClocks();
    configValvePins(&valves_musc[0]);
    configValvePins(&valves_musc[1]);
    configValvePins(&valves_bladder[0]);
    configValvePins(&valves_bladder[1]);
    configGpioPins();
    configTimers();
    configAnalog();
    configUart();

    startTimers();
    MAP_Interrupt_enableMaster(); // enable interrupts


    ////////////////////////////////////////////////////////////////////////////////
    /* Local variables */
    float pres_tank[1];
    float pres_musc[NUM_CHAN_MUSC*2];
    float pres_bladder[NUM_CHAN_BLAD];

    float pres_des_musc[NUM_CHAN_MUSC] = {-1.0,-1.0};
    float pres_des_bladder[NUM_CHAN_BLAD] = {1.0,-1.0};

    float ctrl_params_musc[3];
    float ctrl_params_bladder[3];

    float uart_tx[NUM_UART_TX];

    int n_sense = 0;


    ////////////////////////////////////////////////////////////////////////////
    /* Test custom PCBs */
    if (0 == g_mode) {
        int func = 0;
        while(1) {
            if (g_flag_sense) {
                sensorUpdate(adc_tank, pres_tank, adc_musc, pres_musc, adc_bladder, pres_bladder);
                g_flag_sense = 0; // clear sensor flag
                n_sense++; // increment sensor flag

                if (n_sense == SEND_DATA_COUNT) { // send data over UART at 50 Hz
                    uart_tx[0] = pres_tank[0];
                    uart_tx[1] = pres_musc[0];
                    uart_tx[2] = pres_musc[1];
                    uart_tx[3] = pres_musc[2];
                    uart_tx[4] = pres_musc[3];
                    uart_tx[5] = pres_bladder[0];
                    uart_tx[6] = pres_bladder[1];
                    uart_tx[7] = pres_des_musc[0];
                    uart_tx[8] = pres_des_musc[1];

                    sendData(uart_tx, NUM_UART_TX); // send sensor data
                    n_sense = 0; // clear sensor flag
                    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // toggle red LED
                }
            }


            if (0 == (func % 2)) { // if even
                if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0) { // if button 1
                    func++;
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // green RGB LED
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); // blue RGB LED
                }
            }
            else { // if odd
                if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0) { // if button 2
                    func++;
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // green RGB LED
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); // blue RGB LED
                }
            }

            // right muscle valve
            if (1 == func) { // right inflate
                MAP_GPIO_setOutputHighOnPin(valves_musc[0].port, valves_musc[0].pin_inflate);
            }
            else if (2 == func) { // right vent
                MAP_GPIO_setOutputLowOnPin(valves_musc[0].port, valves_musc[0].pin_inflate);
                MAP_GPIO_setOutputHighOnPin(valves_musc[0].port, valves_musc[0].pin_vent);
            }
            else if (3 == func) { // right seal
                MAP_GPIO_setOutputLowOnPin(valves_musc[0].port, valves_musc[0].pin_vent);
            }

            // left muscle valve
            else if (4 == func) { // left inflate
                MAP_GPIO_setOutputHighOnPin(valves_musc[1].port, valves_musc[1].pin_inflate);
            }
            else if (5 == func) { // left vent
                MAP_GPIO_setOutputLowOnPin(valves_musc[1].port, valves_musc[1].pin_inflate);
                MAP_GPIO_setOutputHighOnPin(valves_musc[1].port, valves_musc[1].pin_vent);
            }
            else if (6 == func) { // left seal
                MAP_GPIO_setOutputLowOnPin(valves_musc[1].port, valves_musc[1].pin_vent);
            }

            // right bladder valve
            else if (7 == func) { // right inflate
                MAP_GPIO_setOutputHighOnPin(valves_bladder[0].port, valves_bladder[0].pin_inflate);
            }
            else if (8 == func) { // right vent
                MAP_GPIO_setOutputLowOnPin(valves_bladder[0].port, valves_bladder[0].pin_inflate);
                MAP_GPIO_setOutputHighOnPin(valves_bladder[0].port, valves_bladder[0].pin_vent);
            }
            else if (9 == func) { // right seal
                MAP_GPIO_setOutputLowOnPin(valves_bladder[0].port, valves_bladder[0].pin_vent);
            }

            // left bladder valve
            else if (10 == func) { // right inflate
                MAP_GPIO_setOutputHighOnPin(valves_bladder[1].port, valves_bladder[1].pin_inflate);
            }
            else if (11 == func) { // right vent
                MAP_GPIO_setOutputLowOnPin(valves_bladder[1].port, valves_bladder[1].pin_inflate);
                MAP_GPIO_setOutputHighOnPin(valves_bladder[1].port, valves_bladder[1].pin_vent);
            }
            else if (12 == func) { // right seal
                MAP_GPIO_setOutputLowOnPin(valves_bladder[1].port, valves_bladder[1].pin_vent);
            }
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    /* Run */
    if (1 == g_mode) {
        while(1) {
            if (g_flag_sense) {
                sensorUpdate(adc_tank, pres_tank, adc_musc, pres_musc, adc_bladder, pres_bladder);
                g_flag_sense = 0; // clear sensor flag
                n_sense++; // increment sensor flag

                if (n_sense == SEND_DATA_COUNT) { // send data over UART at 50 Hz
                    uart_tx[0] = pres_tank[0];
                    uart_tx[1] = pres_musc[0];
                    uart_tx[2] = pres_musc[1];
                    uart_tx[3] = pres_musc[2];
                    uart_tx[4] = pres_musc[3];
                    uart_tx[5] = pres_bladder[0];
                    uart_tx[6] = pres_bladder[1];
                    uart_tx[7] = pres_des_musc[0];
                    uart_tx[8] = pres_des_musc[1];

                    sendData(uart_tx, NUM_UART_TX); // send sensor data
                    n_sense = 0; // clear sensor flag
                    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // toggle red LED
                }
            }
            if (g_flag_control) {
                controlMuscUpdate(pres_musc, pres_des_musc, valves_musc, ctrl_params_musc);
                controlBladderUpdate(pres_bladder, pres_des_bladder, valves_bladder,
                    ctrl_params_bladder);
                g_flag_control = 0;
            }
            if (g_flag_receive) {
                receiveData(pres_des_musc, pres_des_bladder, ctrl_params_musc,
                    ctrl_params_bladder);
                g_flag_receive = 0; // clear receive flag
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/* Interrupts */
extern "C" void TA1_0_IRQHandler(void) {
    /* Timer A1 interrupt: set sensor update flag */
    g_flag_sense = 1;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


extern "C" void TA2_0_IRQHandler(void) {
    /* Timer A2: interrupt: set control update flag */
    g_flag_control = 1;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


void EUSCIA0_IRQHandler(void) {
    /*eUSCI A module interrupt routine: receive data over UART */
    static uint8_t arr8_stuff_rx[256]; // array for stuffed data
    static uint8_t arr8_unstuff_rx[252]; // array for unstuffed data
    static float arr_float_cat_rx[63]; // array for concatenated data
    static uint8_t buf_rx; // byte receive buffer
    static int n8_rx = 0; // number of bytes received in packet

    if (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
            == EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) { // if receive interrupt flag

        buf_rx = MAP_UART_receiveData(EUSCI_A0_BASE); // access received data by reading RXBUF register; flag automatically cleared
        if (buf_rx == 0) { // if 0 received
            arr8_stuff_rx[n8_rx] = buf_rx; // add byte to array
            n8_rx++; // increment number of bytes in packet

            UnstuffArr(arr8_stuff_rx, n8_rx, arr8_unstuff_rx); // unstuff data in packet (auto clears unstuff vector)
            ConcatArr(arr8_unstuff_rx, n8_rx-2, arr_float_cat_rx); // concatenate bytes into uint16_ts (auto clears cat vector)
            int n_float_rx = (n8_rx-2)/4; // number of floats received
            std::copy(arr_float_cat_rx, arr_float_cat_rx + n_float_rx, g_uart_rx); // add received data into global array
            n8_rx = 0; // reset packet length
            g_flag_receive = 1; // set flag to process received data
        }
        else {
            arr8_stuff_rx[n8_rx] = buf_rx; // add byte to array
            n8_rx++; // increment number of bytes in packet
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/* Functions */
void sensorUpdate(const int * adc_tank, float * pres_tank,
                  const int * adc_musc, float * pres_musc,
                  const int * adc_bladder, float * pres_bladder) {
    /* Read ADC and convert voltages to pressures */

    // convert ADC voltages to pressures
    MAP_ADC14_toggleConversionTrigger();
    while(MAP_ADC14_isBusy()); // wait until ADC conversion complete
    int adc_conv;
    float v_out;
    float pres[8];
    for (int i = 0; i < 8; i++) {
        adc_conv = ADC14->MEM[i];
        v_out = (float)adc_conv*(3.3f/4095.0f)*3.0f/2.0f; // NOTE: 3/2 for 1k:2k voltage divider
        pres[i] = (v_out - 0.25f)*100.0f/4.5f; // SSCDRRN100PGAB5 transducer
    }

    // update tank pressure
    pres_tank[0] = pres[adc_tank[0]];

    // update muscle pressures
    for (int i = 0; i < NUM_CHAN_MUSC*2; i++) {
        pres_musc[i] = pres[adc_musc[i]];
    }

    // update bladder pressures
    for (int i = 0; i < NUM_CHAN_BLAD; i++) {
        pres_bladder[i] = pres[adc_bladder[i]];
    }
}


void controlMuscUpdate(float * pres_musc, float * pres_des_musc, valve_t * valves,
    float * ctrl_params) {
    /* Pneumatic muscle pressure control */
    static int idx_trans[2][2] = {{0,1},{2,3}}; // index of pressure transducers (right/left sides)

    float pres_deadband[2] = {ctrl_params[0], ctrl_params[0]}; // muscle pressure deadband
    float pres_undershoot[2] = {ctrl_params[1], ctrl_params[1]}; // muscle pressure undershoot
    float noise_threshold = ctrl_params[2]; // pressure transducer noise threshold    

    int idx_valve; // index of valve
    for (int i = 0; i < NUM_CHAN_MUSC; i++) {
        idx_valve = i;

        // seal (manual)
        if (pres_des_musc[idx_valve] < 0) {
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        // vent (manual, pressure sensor noise threshold)
        else if (pres_des_musc[idx_valve] <= noise_threshold) { // setpoint
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }

        // vent (feedback control)
        else if ((pres_musc[idx_trans[i][0]] > (pres_des_musc[idx_valve]+pres_deadband[idx_valve])) || // pres0 > setpoint+deadband or
                (pres_musc[idx_trans[i][1]] > (pres_des_musc[idx_valve]+pres_deadband[idx_valve]))) { // pres1 > setpoint+deadband
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        // inflate (feedback control)
        else if ((pres_musc[idx_trans[i][0]] < (pres_des_musc[idx_valve]-pres_undershoot[idx_valve])) && // pres0 < setpoint-undershoot and
                (pres_musc[idx_trans[i][1]] < (pres_des_musc[idx_valve]-pres_undershoot[idx_valve]))) { // pres1 < setpoint-undershoot
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        // seal (feedback control)
        else { // pres0 or pres1 in deadband and neither pressure are above deadband
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
    }
}


void controlBladderUpdate(float * pres_bladder, float * pres_des_bladder, valve_t * valves,
    float * ctrl_params) {
    /* Infrapatellar bladder pressure control */
    static float pres_hist[2][LEN_BLAD_FILT];
    static float pres[2];

    float pres_deadband[2] = {ctrl_params[0], ctrl_params[0]}; // muscle pressure deadband
    float pres_undershoot[2] = {ctrl_params[1], ctrl_params[1]}; // muscle pressure undershoot
    float noise_threshold = ctrl_params[2]; // pressure transducer noise threshold

    int idx_valve; // index of valve
    for (int i = 0; i < NUM_CHAN_BLAD; i++) {
        idx_valve = i;

        for (int j = (LEN_BLAD_FILT-1); j > 0; j--) {
            pres_hist[i][j] = pres_hist[i][j-1];
        }
        pres_hist[i][0] = pres_bladder[i];

        pres[i] = 0;
        for (int j = 0; j < LEN_BLAD_FILT; j++) {
            pres[i] = pres[i] + pres_hist[i][j];
        }
        pres[i] = pres[i]/LEN_BLAD_FILT;


        // seal (manual)
        if (pres_des_bladder[idx_valve] < 0) {
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        // vent (manual, pressure sensor noise threshold)
        else if (pres_des_bladder[idx_valve] <= noise_threshold) { // setpoint
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }

        // vent (feedback control)
        else if (pres[i] > (pres_des_bladder[idx_valve]+pres_deadband[idx_valve])) { // pres > setpoint+deadband
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        // inflate (feedback control)
        else if (pres[i] < (pres_des_bladder[idx_valve]-pres_undershoot[idx_valve])) { // pres < setpoint-undershoot
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        // seal (feedback control)
        else { // pres in deadband
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
    }
}


void receiveData(float * pres_des_musc, float * pres_des_bladder, 
    float * ctrl_params_musc, float * ctrl_params_bladder) {
    /* Update values from received UART data */
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0); // red RGB LED

    switch ((int)g_uart_rx[0] & 255) { // LSbyte of first float indicates message type
        float *start;

        case 0: { // update muscle pressure setpoints
            pres_des_musc[0] = g_uart_rx[1];
            break;
        }
        case 1: { // update left side pressure setpoint
            pres_des_bladder[1] = g_uart_rx[1];
            break;
        }
        case 2: { // update pressure controller gains
            start = g_uart_rx+1;
            std::copy(start, start+4, ctrl_params_musc);
            break;
        }
        case 3: { // update pressure controller gains
            start = g_uart_rx+1;
            std::copy(start, start+4, ctrl_params_bladder);
            break;
        }
    }
}


void sendData(float * uart_tx, int n_float_tx) {
    /* Send data over UART */
    int n8_tx = n_float_tx*4; // number of data bytes to transmit
    uint8_t arr8_unstuff_tx[252]; // array for unstuffed data
    uint8_t arr8_stuff_tx[256]; // array for stuffed data

    // separate, stuff, and send data
    SeparateArr(uart_tx, n_float_tx, arr8_unstuff_tx);  // separate floats into bytes
    StuffArr(arr8_unstuff_tx, n8_tx, arr8_stuff_tx); // create stuffed byte packet
    for (int i = 0; i < (n8_tx+2); i++) { // loop over data bytes + 2 stuff bytes
        MAP_UART_transmitData(EUSCI_A0_BASE, arr8_stuff_tx[i]); // transmit byte
        while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE,
            EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
            != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmission completion
    }
}

