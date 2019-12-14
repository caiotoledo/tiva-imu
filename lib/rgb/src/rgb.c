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

#define PWM_PERIOD		(64000U)

typedef struct
{
	uint32_t GPIOPinConf;
	uint32_t GPIOPinType;
	uint32_t PWMGen;
	uint32_t PWMOut;
	uint32_t PWMOutBit;
} tPWM_Pin_Conf;

typedef struct
{
	LED_RGB led;
	tPWM_Pin_Conf conf;
} tLED_Conf;

static const tPWM_Pin_Conf *GetPWMConf(LED_RGB led);
static void LED_PWM_Configure(LED_RGB led);
static void setLedDuty (LED_RGB led, uint32_t duty);

/* Map for LEDs configurations: */
static const tLED_Conf led_configurations[] =
{
	{
		.led = RED_LED,
		.conf =
		{
			.GPIOPinConf = GPIO_PF1_M1PWM5,
			.GPIOPinType = GPIO_PIN_1,
			.PWMGen = PWM_GEN_2,
			.PWMOut = PWM_OUT_5,
			.PWMOutBit = PWM_OUT_5_BIT,
		},
	},
	{
		.led = BLUE_LED,
		.conf =
		{
			.GPIOPinConf = GPIO_PF2_M1PWM6,
			.GPIOPinType = GPIO_PIN_2,
			.PWMGen = PWM_GEN_3,
			.PWMOut = PWM_OUT_6,
			.PWMOutBit = PWM_OUT_6_BIT,
		},
	},
	{
		.led = GREEN_LED,
		.conf =
		{
			.GPIOPinConf = GPIO_PF3_M1PWM7,
			.GPIOPinType = GPIO_PIN_3,
			.PWMGen = PWM_GEN_3,
			.PWMOut = PWM_OUT_7,
			.PWMOutBit = PWM_OUT_7_BIT,
		},
	}
};

void LED_Enable()
{
	LED_PWM_Configure(RED_LED);
	LED_PWM_Configure(BLUE_LED);
	LED_PWM_Configure(GREEN_LED);
}

void LED_Set(LED_RGB led, bool state)
{
	uint32_t duty;

	duty = state ? RGB_MAX_VALUE : RGB_MIN_VALUE;
	setLedDuty(led, duty);
}

void RGB_Set(uint32_t rValue, uint32_t gValue, uint32_t bValue)
{
	setLedDuty(RED_LED, rValue);
	setLedDuty(GREEN_LED, gValue);
	setLedDuty(BLUE_LED, bValue);
}

static const tPWM_Pin_Conf *GetPWMConf(LED_RGB led)
{
	const tPWM_Pin_Conf *pwm_conf = 0U;
	/* Search for PWM configuration */
	for (uint8_t i = 0; i < sizeof(led_configurations)/sizeof(led_configurations[0]); i++)
	{
		if (led_configurations[i].led == led)
		{
			pwm_conf = &led_configurations[i].conf;
			break;
		}
	}
	return pwm_conf;
}

static void LED_PWM_Configure(LED_RGB led)
{
	const tPWM_Pin_Conf *pwm_conf;

	pwm_conf = GetPWMConf(led);
	/* Return if no valid configuration was found */
	if (pwm_conf == 0U)
	{
		return;
	}

	/* Configure Peripheral */
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/* Configure Pins */
	GPIOPinConfigure(pwm_conf->GPIOPinConf);
	GPIOPinTypePWM(GPIO_PORTF_BASE, pwm_conf->GPIOPinType);

	/* Configure PWM */
	PWMGenConfigure(PWM1_BASE, pwm_conf->PWMGen, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, pwm_conf->PWMGen, PWM_PERIOD);
	PWMOutputState(PWM1_BASE, pwm_conf->PWMOutBit, true);
	PWMOutputInvert(PWM1_BASE, pwm_conf->PWMOutBit, false);

	/* Start Duty Cycle */
	PWMPulseWidthSet(PWM1_BASE, pwm_conf->PWMOut, 0U);

	/* Enable PWM */
	PWMGenEnable(PWM1_BASE, pwm_conf->PWMGen);
}

static void setLedDuty (LED_RGB led, uint32_t duty)
{
	uint32_t base;
	const tPWM_Pin_Conf *pwm_conf;

	pwm_conf = GetPWMConf(led);
	/* Return if no valid configuration was found */
	if (pwm_conf == 0U)
	{
		return;
	}

	/* Limit Duty Cycle to MAX_VALUE */
	duty = duty > RGB_MAX_VALUE ? RGB_MAX_VALUE : duty;

	/* Calculate base duty cycle */
	base = PWMGenPeriodGet(PWM1_BASE, pwm_conf->PWMGen)*duty/RGB_PWM_WIDTH_BASE;

	/* Update Duty Cycle */
	PWMPulseWidthSet(PWM1_BASE, pwm_conf->PWMOut, base);
}
