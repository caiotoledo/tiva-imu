#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

#include <button.h>

#define DEBOUNCE_MS         (100U)

typedef struct
{
    uint32_t pin;
    uint32_t interrupt;
    uint32_t port;
    uint32_t debounce_ms;
} tButtonConf;

typedef struct
{
    bool state;
    uint32_t last_int;
} tButtonData;


typedef struct
{
    const eButtonSW button;
    const tButtonConf conf;
    tButtonData data;
} tMapButton;

tMapButton MapButton[] =
{
    {
        .button = ButtonSW1,
        .conf =
        {
            .port = GPIO_PORTF_BASE,
            .interrupt = GPIO_INT_PIN_4,
            .pin = GPIO_PIN_4,
            .debounce_ms = 0U,
        },
        .data =
        {
            .state = false,
            .last_int = 0U,
        }
    },
    {
        .button = ButtonSW2,
        .conf =
        {
            .port = GPIO_PORTF_BASE,
            .interrupt = GPIO_INT_PIN_0,
            .pin = GPIO_PIN_0,
            .debounce_ms = 0U,
        },
        .data =
        {
            .state = false,
            .last_int = 0U,
        }
    }
};

static const tButtonData *GetButtonData(eButtonSW button);
static bool GetButtonState(eButtonSW button);
static void Button_ISP_Handler();
static millis ms_func;

/*
*       EXTERNAL FUNCTIONS
*/
void Button_Enable(millis f)
{
    /* Store milliseconds function: */
    if (f != 0U)
    {
        ms_func = f;
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    for (size_t i = 0; i < sizeof(MapButton)/sizeof(tMapButton); i++)
    {
        GPIODirModeSet(MapButton[i].conf.port, MapButton[i].conf.pin, GPIO_DIR_MODE_IN);
        GPIOPadConfigSet(MapButton[i].conf.port, MapButton[i].conf.pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        GPIOIntTypeSet(MapButton[i].conf.port,MapButton[i].conf.pin,GPIO_BOTH_EDGES);
        GPIOIntRegister(MapButton[i].conf.port,Button_ISP_Handler);
        GPIOIntEnable(MapButton[i].conf.port, MapButton[i].conf.interrupt);
    }
}

bool Button_Get_State(eButtonSW button)
{
    return GetButtonState(button);
}

/*
*       LOCAL FUNCTIONS
*/
static void Button_ISP_Handler()
{
    uint32_t status=0;

    status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,status);

    for (size_t i = 0; i < sizeof(MapButton)/sizeof(tMapButton); i++)
    {
        if ((status & MapButton[i].conf.interrupt) == MapButton[i].conf.interrupt)
        {
            uint32_t now = ms_func();
            bool stButton = !GPIOPinRead(MapButton[i].conf.port, MapButton[i].conf.pin);

            if (stButton)
            {
                /* Apply Debounce only for Button Activation */
                if ( (now - MapButton[i].data.last_int) > DEBOUNCE_MS )
                {
                    MapButton[i].data.state = stButton;
                }
            }
            else
            {
                MapButton[i].data.state = stButton;
            }

            MapButton[i].data.last_int = now;
        }
    }
}

static const tButtonData *GetButtonData(eButtonSW button)
{
    const tButtonData *button_data = 0U;
    /* Search for Button data */
    for (uint8_t i = 0; i < sizeof(MapButton) / sizeof(tButtonConf); i++)
    {
        if (MapButton[i].button == button)
        {
            button_data = &MapButton[i].data;
            break;
        }
    }
    return button_data;
}

static bool GetButtonState(eButtonSW button)
{
    return GetButtonData(button)->state;
}