#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <LED_RGB.h>
#include <button.h>

#include <FreeRTOS.h>
#include <timers.h>

#include <hal_uart.h>

#define RGB_RAINBOW_STEP_MAX            500
#define RGB_RAINBOW_STEP_MIN            0
#define RGB_RAINBOW_STEP_INITIAL        50
#define RGB_RAINBOW_STEP_INCREMENT      25

static void vStateMachineRainbowRGB(void);
static uint32_t GetMillis(void);

static uint32_t RGBRainbowStep = RGB_RAINBOW_STEP_INITIAL;

void vLedTask(void *pvParameters)
{
    Button_Enable(GetMillis);

    for (;;)
    {
        vStateMachineRainbowRGB();

        bool sw1 = Button_Get_State(ButtonSW1);
        if (sw1)
        {
            RGBRainbowStep += RGB_RAINBOW_STEP_INCREMENT;
            if (RGBRainbowStep > RGB_RAINBOW_STEP_MAX)
            {
                RGBRainbowStep = RGB_RAINBOW_STEP_MAX;
            }
            UARTprintf("RGBRainbowStep [%u]\n", RGBRainbowStep);
        }

        bool sw2 = Button_Get_State(ButtonSW2);
        if (sw2)
        {
            RGBRainbowStep -= RGB_RAINBOW_STEP_INCREMENT;
            if (RGBRainbowStep > RGB_RAINBOW_STEP_MAX)
            {
                RGBRainbowStep = RGB_RAINBOW_STEP_MIN;
            }
            UARTprintf("RGBRainbowStep [%u]\n", RGBRainbowStep);
        }
    }
}

static uint32_t GetMillis(void)
{
    return (xTaskGetTickCountFromISR()/portTICK_RATE_MS);
}

static bool ApplyRGBStep(uint16_t *color, bool add)
{
    uint16_t c = *color;
    bool bIncrementStateMach = false;

    if (add)
    {
        c += RGBRainbowStep;
    }
    else
    {
        c -= RGBRainbowStep;
    }

    if (c >= RGB_MAX_VALUE)
    {
        c = add ? RGB_MAX_VALUE : RGB_MIN_VALUE;
        bIncrementStateMach = true;
    }

    *color = c;
    return bIncrementStateMach;
}

static void vStateMachineRainbowRGB(void)
{

    static uint8_t state = 0;
    static uint16_t red = RGB_MAX_VALUE;
    static uint16_t green = RGB_MIN_VALUE;
    static uint16_t blue = RGB_MIN_VALUE;

    switch (state)
    {
    case 0:
        if (ApplyRGBStep(&green, true))
        {
            state++;
        }
        break;
    case 1:
        if (ApplyRGBStep(&red, false))
        {
            state++;
        }
        break;
    case 2:
        if (ApplyRGBStep(&blue, true))
        {
            state++;
        }
        break;
    case 3:
        if (ApplyRGBStep(&green, false))
        {
            state++;
        }
        break;
    case 4:
        if (ApplyRGBStep(&red, true))
        {
            state++;
        }
        break;
    case 5:
        if (ApplyRGBStep(&blue, false))
        {
            state = 0;
        }
        break;
    default:
        state = 0;
        break;
    }

    RGB_Set(red, green, blue);
    vTaskDelay(150 / portTICK_RATE_MS);
}