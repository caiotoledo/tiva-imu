#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <mpu6050.h>

#include <log.h>

#include <FreeRTOS.h>
#include <timers.h>

extern uint32_t GetMillis(void);

void vMPU6050Task(void *pvParameters)
{
    int ret = MPU6050_Enable(MPU6050_LOW, I2C1, GetMillis);
    if (ret != 0)
    {
        ERROR("MPU6050 Error, Suspend task!");
        /* Stop this Task */
        vTaskSuspend(NULL);
    }

    for(;;)
    {
        uint8_t val1 = I2C_Read_Reg(I2C1, 0x68, 0x3B);
        INFO("0x3B = [0x%02X]", val1);

        uint8_t values[2] = {0};
        int ret = I2C_Read_Multiple_Reg(I2C1, 0x68, 0x3B, sizeof(values), values);
        INFO("Multiple[%d] = [0x%02X%02X]", ret, values[0], values[1]);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
