/*
 * PWM_RGB.h
 *
 *  Created on: 14/05/2016
 *      Author: Caio
 */

#ifndef LED_RGB_H_
#define LED_RGB_H_

#include <stdint.h>

/**
 * @brief Width base for the PWM control
 * 
 */
#define WIDTH_BASE			(1024)
/**
 * @brief Max RGB value
 * 
 */
#define RGB_MAX_VALUE		(WIDTH_BASE-1)
/**
 * @brief Min RGB Value
 * 
 */
#define RGB_MIN_VALUE		(0)

/**
 * @brief LED RGB definitions
 * 
 */
typedef enum {
	RED_LED,
	BLUE_LED,
	GREEN_LED,
} LED_RGB;

/**
 * @brief Enable All LEDs
 * 
 */
void LED_Enable();

/**
 * @brief Set LED state
 * 
 * @param led LED to be configured
 * @param state Desired State (True for ON and False for OFF)
 */
void LED_Set(LED_RGB led, bool state);

/**
 * @brief Configure the RGB values for each color
 * Values should be between #RGB_MAX_VALUE and #RGB_MIN_VALUE
 * 
 * @param rValue Set Red Color
 * @param gValue Set Green Color
 * @param bValue Set Blue Color
 */
void RGB_Set(uint32_t rValue, uint32_t gValue, uint32_t bValue);

#endif /* LED_RGB_H_ */
