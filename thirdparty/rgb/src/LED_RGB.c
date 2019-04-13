/*
 * PWM_RGB.c
 *
 *  Created on: 14/05/2016
 *      Author: Caio
 */

#include <stdint.h>
#include <stdbool.h>

#include <tm4c123gh6pm_target_definitions.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

#include "LED_RGB.h"

#define PWM_PERIOD		(64000)
#define WIDTH_BASE		(1024)
#define MAX_VALUE		(WIDTH_BASE-1)
#define MIN_VALUE		(0)

static void LED_PWM_Configure(LED_RGB led);
static void setLedDuty (LED_RGB led, uint32_t duty);

void LED_Enable()
{
	LED_PWM_Configure(RED_LED);
	LED_PWM_Configure(BLUE_LED);
	LED_PWM_Configure(GREEN_LED);
}

void LED_Set(LED_RGB led, bool state)
{
	uint32_t duty;

	duty = state ? MAX_VALUE : MIN_VALUE;
	setLedDuty(led, duty);
}

void RGB_Set(uint32_t rValue, uint32_t gValue, uint32_t bValue)
{
	setLedDuty(RED_LED, rValue);
	setLedDuty(GREEN_LED, gValue);
	setLedDuty(BLUE_LED, bValue);
}

static void LED_PWM_Configure(LED_RGB led)
{
	uint32_t GPIOPinConf;
	uint32_t GPIOPinType;
	uint32_t PWMGen;
	uint32_t PWMOut;
	uint32_t PWMOutBit;

	switch (led) {
		case RED_LED:
			GPIOPinConf = GPIO_PF1_M1PWM5;
			GPIOPinType = GPIO_PIN_1;
			PWMGen = PWM_GEN_2;
			PWMOut = PWM_OUT_5;
			PWMOutBit = PWM_OUT_5_BIT;
			break;
		case BLUE_LED:
			GPIOPinConf = GPIO_PF2_M1PWM6;
			GPIOPinType = GPIO_PIN_2;
			PWMGen = PWM_GEN_3;
			PWMOut = PWM_OUT_6;
			PWMOutBit = PWM_OUT_6_BIT;
			break;
		case GREEN_LED:
			GPIOPinConf = GPIO_PF3_M1PWM7;
			GPIOPinType = GPIO_PIN_3;
			PWMGen = PWM_GEN_3;
			PWMOut = PWM_OUT_7;
			PWMOutBit = PWM_OUT_7_BIT;
			break;
		default:
		//If the Value is wrong, stop the configuration:
			return;
	}

	/* Configure Peripheral */
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/* Configure Pins */
	GPIOPinConfigure(GPIOPinConf);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIOPinType);

	/* Configure PWM */
	PWMGenConfigure(PWM1_BASE, PWMGen, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWMGen, PWM_PERIOD);
	PWMOutputState(PWM1_BASE, PWMOutBit, true);
	PWMOutputInvert(PWM1_BASE, PWMOutBit, false);

	/* Start Duty Cycle */
	PWMPulseWidthSet(PWM1_BASE, PWMOut, 0U);

	/* Enable PWM */
	PWMGenEnable(PWM1_BASE, PWMGen);
}

static void setLedDuty (LED_RGB led, uint32_t duty)
{
	uint32_t base;
	uint32_t PWMOut;
	uint32_t PWMGen;

	switch (led) {
		case RED_LED:
			PWMOut = PWM_OUT_5;
			PWMGen = PWM_GEN_2;
			break;
		case BLUE_LED:
			PWMOut = PWM_OUT_6;
			PWMGen = PWM_GEN_3;
			break;
		case GREEN_LED:
			PWMOut = PWM_OUT_7;
			PWMGen = PWM_GEN_3;
			break;
		default:
			return;
	}

	/* Limit Duty Cycle to MAX_VALUE */
	duty = duty > MAX_VALUE ? MAX_VALUE : duty;

	/* Calculate base duty cycle */
	base = PWMGenPeriodGet(PWM1_BASE, PWMGen)*duty/WIDTH_BASE;

	/* Update Duty Cycle */
	PWMPulseWidthSet(PWM1_BASE, PWMOut, base);
}
