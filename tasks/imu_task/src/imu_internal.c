#include <FreeRTOS.h>
#include <semphr.h>

#include <log.h>

#include <timer.h>

#include "imu_types.h"
#include "imu_internal.h"

/* TODO: Move this dependencies to another library */
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

#define IMU_SAMPLE_RATE         (1000/portTICK_RATE_MS)

#define SYSCTL_PERIPH_GPIO      (SYSCTL_PERIPH_GPIOA)
#define GPIO_PORT_BASE          (GPIO_PORTA_BASE)
#define GPIO_PIN                (GPIO_PIN_5)
#define GPIO_INT_PIN            (GPIO_INT_PIN_5)

static void GPIO_ISP_Handler(void);
static void vConfigGPIOInt(void);

static SemaphoreHandle_t xIMUDataReadySemaphore = NULL;

void vMPU6050Task(void *pvParameters)
{
    int ret = 0;

    imuTaskConfig_t taskParam = *(( imuTaskConfig_t * ) pvParameters);

    if (MPU6050_Enable(taskParam.mpu, taskParam.i2c, GetMillis) != 0)
    {
        ERROR("[%s] Enable Error!", taskParam.name);
        goto end_mpu6050_task;
    }

    /* Initialize the Interrupt GPIO */
    vConfigGPIOInt();
    /* Attempt to create a semaphore. */
    if (xIMUDataReadySemaphore == NULL)
    {
        xIMUDataReadySemaphore = xSemaphoreCreateBinary();
    }

    /* Initialize the Mutex for this task only once */
    static SemaphoreHandle_t mtxIMU = NULL;
    if (mtxIMU == NULL)
    {
        mtxIMU = xSemaphoreCreateMutex();
    }

    /* Timer initialization */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /* Wait IMU Data to be ready */
        if (xSemaphoreTake(xIMUDataReadySemaphore, (10/portTICK_RATE_MS)) != pdTRUE)
        {
            WARN("IMU Data not Ready!");
        }

        xSemaphoreTake(xIMUDataReadySemaphore, (10/portTICK_RATE_MS));

        /* Lock IMU mutex */
        xSemaphoreTake(mtxIMU, portMAX_DELAY);

        /* Store sample time in ms */
        uint32_t time_ms = GetMillis();
        /* Sample Accelerometer */
        accel_t accel;
        ret += MPU6050_ReadAllAccel(taskParam.mpu, &accel);
        /* Sample Gyroscope */
        gyro_t gyro;
        ret += MPU6050_ReadAllGyro(taskParam.mpu, &gyro);
        /* Sample Temperature */
        double temp;
        ret += MPU6050_ReadTemperature(taskParam.mpu, &temp);

        /* Release IMU mutex */
        xSemaphoreGive(mtxIMU);

        /* Check if the data was successful sample */
        if (ret == 0)
        {
            /* Store IMU Data */
            dataIMU_t dataimu = {
                .imu = taskParam.name,
                .ms = time_ms,
                .accel = accel,
                .gyro = gyro,
                .temperature = temp,
            };
            /* Send IMU data via queue */
            if (xQueueSend(taskParam.queue, (void *)&dataimu, portMAX_DELAY) != pdTRUE)
            {
                ERROR("Full queue!");
            }
        }
        else
        {
            ERROR("Error IMU Read!");
        }

        vTaskDelayUntil(&xLastWakeTime, (TickType_t)IMU_SAMPLE_RATE);
    }

end_mpu6050_task:
    ERROR("Suspend %s Task!", taskParam.name);
    vTaskSuspend(NULL);
}

static void GPIO_ISP_Handler(void)
{
    uint32_t status = GPIOIntStatus(GPIO_PORT_BASE,true);
    GPIOIntClear(GPIO_PORT_BASE,status);

    if ((status & GPIO_INT_PIN) == GPIO_INT_PIN)
    {
        uint32_t stGpio = GPIOPinRead(GPIO_PORT_BASE, GPIO_INT_PIN);
        if ((stGpio & GPIO_INT_PIN) != 0)
        {
            xSemaphoreGiveFromISR(xIMUDataReadySemaphore, NULL);
        }
    }
}

static void vConfigGPIOInt(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO);

    GPIODirModeSet(GPIO_PORT_BASE, GPIO_PIN, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORT_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    GPIOIntTypeSet(GPIO_PORT_BASE,GPIO_PIN,GPIO_RISING_EDGE);
    GPIOIntRegister(GPIO_PORT_BASE,GPIO_ISP_Handler);
    GPIOIntEnable(GPIO_PORT_BASE, GPIO_INT_PIN);
}
