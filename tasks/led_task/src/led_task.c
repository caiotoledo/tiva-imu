#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <log.h>
#include <LED_RGB.h>
#include <button.h>

#include <timer.h>

#include <FreeRTOS.h>
#include <timers.h>

#include <hal_uart.h>

#define RGB_RAINBOW_STEP_MAX            (500U)
#define RGB_RAINBOW_STEP_MIN            (0U)
#define RGB_RAINBOW_STEP_INITIAL        RGB_RAINBOW_STEP_MIN
#define RGB_RAINBOW_STEP_INCREMENT      (25U)

static void vStateMachineRainbowRGB(uint32_t rgbStep);

void vLedTask(void *pvParameters)
{
    uint32_t RGBRainbowStep = RGB_RAINBOW_STEP_INITIAL;

    /* Initialize button API */
    Button_Enable(GetMillis);

    for (;;)
    {
        if (RGBRainbowStep == 0)
        {
            RGB_Set(0U, 0U, 0U);
        }
        else
        {
            /* Call Rainbow State Machine */
            vStateMachineRainbowRGB(RGBRainbowStep);
        }

        /* Increment Rainbow Step */
        bool sw1 = Button_Get_State(ButtonSW1);
        if (sw1)
        {
            RGBRainbowStep += RGB_RAINBOW_STEP_INCREMENT;
            if (RGBRainbowStep > RGB_RAINBOW_STEP_MAX)
            {
                RGBRainbowStep = RGB_RAINBOW_STEP_MAX;
            }
            INFO("RGBRainbowStep [%u]", RGBRainbowStep);
        }

        /* Decrement Rainbow Step */
        bool sw2 = Button_Get_State(ButtonSW2);
        if (sw2)
        {
            RGBRainbowStep -= RGB_RAINBOW_STEP_INCREMENT;
            if (RGBRainbowStep > RGB_RAINBOW_STEP_MAX)
            {
                RGBRainbowStep = RGB_RAINBOW_STEP_MIN;
            }
            INFO("RGBRainbowStep [%u]", RGBRainbowStep);

            /* Notify LED off in logger */
            if (RGBRainbowStep == 0)
            {
                INFO("PowerOff RGB LED!");
            }
        }

        /* Task sleep */
        vTaskDelay(150 / portTICK_RATE_MS);
    }
}

/**
 * @brief Apply the RGB Step in color reference
 *
 * @param color Color pointer
 * @param step Step value for color variable
 * @param add Flag to increment or decrement the color value
 * @return true Color variable overflow
 * @return false Color variable didn't overflow
 */
static bool ApplyRGBStep(uint16_t *color, uint32_t step, bool add)
{
    /* Pointer parameter protection */
    if (color == NULL)
    {
        return false;
    }

    uint16_t c = *color;
    bool bIncrementStateMach = false;

    /* Update color variable */
    c += add ? step : (-step);

    /* Check variable overflow */
    if (c >= RGB_MAX_VALUE)
    {
        c = add ? RGB_MAX_VALUE : RGB_MIN_VALUE;
        bIncrementStateMach = true;
    }

    *color = c;
    return bIncrementStateMach;
}

/**
 * @brief Run the state machine for the rainbow RGB Led
 *
 * @param rgbStep RGB step for each iteration
 */
static void vStateMachineRainbowRGB(uint32_t rgbStep)
{

    /* Initializing variable values */
    static uint8_t state = 0;
    static uint16_t red = RGB_MAX_VALUE;
    static uint16_t green = RGB_MIN_VALUE;
    static uint16_t blue = RGB_MIN_VALUE;

    /* Apply the RGB step based on the current state */
    switch (state)
    {
    /* Function ApplyRGBStep return true in variable overflow, so the state should increment */
    case 0:
        if (ApplyRGBStep(&green, rgbStep, true))
        {
            state++;
        }
        break;
    case 1:
        if (ApplyRGBStep(&red, rgbStep, false))
        {
            state++;
        }
        break;
    case 2:
        if (ApplyRGBStep(&blue, rgbStep, true))
        {
            state++;
        }
        break;
    case 3:
        if (ApplyRGBStep(&green, rgbStep, false))
        {
            state++;
        }
        break;
    case 4:
        if (ApplyRGBStep(&red, rgbStep, true))
        {
            state++;
        }
        break;
    case 5:
        if (ApplyRGBStep(&blue, rgbStep, false))
        {
            state = 0;
        }
        break;
    default:
        state = 0;
        break;
    }

    /* Update the RGB values */
    RGB_Set(red, green, blue);
}