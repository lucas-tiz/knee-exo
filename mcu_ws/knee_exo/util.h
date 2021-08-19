/*
 * util.h
 *
 *  Created on: Jan 27, 2021
 *      Author: lucas
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>
#include <stdlib.h>


void csvStringRead(int nVals, volatile uint8_t * strRead, volatile float * arrWrite);
void delay(int d);


#endif /* UTIL_H_ */
