#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <utils.h>
#include <log.h>
#include <rgb.h>
#include <button.h>

#include <timer.h>

#include <FreeRTOS.h>
#include <timers.h>

#include <cmd_task.h>

#define RGB_RAINBOW_STEP_MAX            (500U)
#define RGB_RAINBOW_STEP_MIN            (0U)
#define RGB_RAINBOW_STEP_INITIAL        RGB_RAINBOW_STEP_MIN
#define RGB_RAINBOW_STEP_INCREMENT      (25U)

#define RGB_TASK_DELAY                  (150/portTICK_RATE_MS)

typedef enum {
    INCREMENT,
    DECREMENT,
} eStepOperation;

static int SetRGBFreq(int value);
static bool ApplyRGBStep(uint16_t *color, uint32_t step, eStepOperation op);
static void vStateMachineRainbowRGB(uint32_t rgbStep);
static BaseType_t RGBFreqCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static uint32_t RGBRainbowStep = RGB_RAINBOW_STEP_INITIAL;

static const CLI_Command_Definition_t xRGBFreq =
{
    "rgb-freq", /* The command string. */
    "rgb-freq [value]:\r\n Configure RGB Frequency\r\n", /* Help string. */
    RGBFreqCommand, /* The function to run. */
    1 /* No parameters are expected. */
};

void vLedTask(void *pvParameters)
{
    /* Register LED Commands */
    CMD_RegFuncCommand(&xRGBFreq);

    /* Initialize button API */
    Button_Enable(GetMillisFromISR);

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
            /* Do not allow values greater than RGB_RAINBOW_STEP_MAX */
            RGBRainbowStep = MIN(RGBRainbowStep, RGB_RAINBOW_STEP_MAX);
            INFO("RGBRainbowStep [%u]", RGBRainbowStep);
        }

        /* Decrement Rainbow Step */
        bool sw2 = Button_Get_State(ButtonSW2);
        if (sw2)
        {
            RGBRainbowStep -= RGB_RAINBOW_STEP_INCREMENT;
            /* Check variable overflow */
            RGBRainbowStep = (RGBRainbowStep > RGB_RAINBOW_STEP_MAX) ? RGB_RAINBOW_STEP_MIN : RGBRainbowStep;
            INFO("RGBRainbowStep [%u]", RGBRainbowStep);

            /* Notify LED off in logger */
            if (RGBRainbowStep == 0)
            {
                INFO("PowerOff RGB LED!");
            }
        }

        /* Task sleep */
        vTaskDelay(RGB_TASK_DELAY);
    }
}

static BaseType_t RGBFreqCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t lParameterStringLength;

    /* Obtain the parameter string. */
    pcParameter = FreeRTOS_CLIGetParameter (
        pcCommandString,        /* The command string itself. */
        1,                      /* Return the first parameter. */
        &lParameterStringLength /* Store the parameter string length. */
    );

    int value = UTILS_ConvertStringToNumber(pcParameter, lParameterStringLength);

    /* Update frequency value */
    if (SetRGBFreq(value) == 0U)
    {
        LOG_UART("Update RGB Frequency value [%u]", value);
    }
    else
    {
        LOG_UART("Invalid value [%u] (MIN[%u] ~ MAX[%u])", value, RGB_RAINBOW_STEP_MIN, RGB_RAINBOW_STEP_MAX);
    }

    /* Do not use the pcWriteBuffer to output on console */
    if (xWriteBufferLen >= 1)
    {
        pcWriteBuffer[0] = '\0';
    }

    return pdFALSE;
}

/**
 * @brief Update RGB Frequency
 * 
 * @param value New value
 * @return int Returns 0 for valid values
 */
static int SetRGBFreq(int value)
{
    int ret = -1;
    int local_value = value;

    /* Check value bounds */
    local_value = MAX(local_value, RGB_RAINBOW_STEP_MIN);
    local_value = MIN(local_value, RGB_RAINBOW_STEP_MAX);

    if (local_value == value)
    {
        RGBRainbowStep = value;
        ret = 0;
    }

    return ret;
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
        if (ApplyRGBStep(&green, rgbStep, INCREMENT))
        {
            state++;
        }
        break;
    case 1:
        if (ApplyRGBStep(&red, rgbStep, DECREMENT))
        {
            state++;
        }
        break;
    case 2:
        if (ApplyRGBStep(&blue, rgbStep, INCREMENT))
        {
            state++;
        }
        break;
    case 3:
        if (ApplyRGBStep(&green, rgbStep, DECREMENT))
        {
            state++;
        }
        break;
    case 4:
        if (ApplyRGBStep(&red, rgbStep, INCREMENT))
        {
            state++;
        }
        break;
    case 5:
        if (ApplyRGBStep(&blue, rgbStep, DECREMENT))
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

/**
 * @brief Apply the RGB Step in color reference
 *
 * @param color Color pointer
 * @param step Step value for color variable
 * @param add Flag to increment or decrement the color value
 * @param op Operation increment or decrement in the color value
 * @return true Color variable overflow
 * @return false Color variable didn't overflow
 */
static bool ApplyRGBStep(uint16_t *color, uint32_t step, eStepOperation op)
{
    /* Pointer parameter protection */
    if (color == NULL)
    {
        return false;
    }

    uint16_t c = *color;
    bool bIncrementStateMach = false;

    /* Update color variable */
    c += (op == INCREMENT) ? step : (-step);

    /* Check variable overflow */
    if (c >= RGB_MAX_VALUE)
    {
        c = (op == INCREMENT) ? RGB_MAX_VALUE : RGB_MIN_VALUE;
        bIncrementStateMach = true;
    }

    *color = c;
    return bIncrementStateMach;
}
