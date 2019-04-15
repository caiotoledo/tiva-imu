/*
 * PWM_RGB.h
 *
 *  Created on: 14/05/2016
 *      Author: Caio
 */

#ifndef LED_RGB_H_
#define LED_RGB_H_

#include <stdint.h>

#define WIDTH_BASE			(1024)
#define RGB_MAX_VALUE		(WIDTH_BASE-1)
#define RGB_MIN_VALUE		(0)

typedef enum {
	RED_LED,
	BLUE_LED,
	GREEN_LED,
} LED_RGB;

void LED_Enable();
void LED_Set(LED_RGB led, bool state);
void RGB_Set(uint32_t rValue, uint32_t gValue, uint32_t bValue);

#endif /* LED_RGB_H_ */
