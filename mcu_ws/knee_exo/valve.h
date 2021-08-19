/*
 * valve.h
 *
 *  Created on: Jan 27, 2021
 *      Author: lucas
 */

#ifndef VALVE_H_
#define VALVE_H_

#include <msp.h>


typedef struct {
    uint_fast8_t port;
    uint_fast16_t pin_inflate;
    uint_fast16_t pin_vent;
} valve_t;


#endif /* VALVE_H_ */
