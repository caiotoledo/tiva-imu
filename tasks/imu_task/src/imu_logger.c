#include <FreeRTOS.h>
#include <queue.h>

#include <utils.h>
#include <log.h>

#include "imu_types.h"
#include "imu_logger.h"

#define FRAC_PRECISION  (4U)

static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision);

void vIMULogTask(void *pvParameters)
{
    QueueHandle_t xQueueDataIMU = *((QueueHandle_t *)pvParameters);

    for(;;)
    {
        dataIMU_t data;
        /* Receive IMU data from Queue */
        if (xQueueReceive(xQueueDataIMU, &(data), portMAX_DELAY) != pdFALSE)
        {
            int integerAccel[3], integerGyro[3], integerTemp;
            uint32_t fracAccel[3], fracGyro[3], fracTemp;

            INFO("");
            INFO("[%s] TIME %d ms", data.imu, data.ms);

            /* Convert Accel Double values for print */
            vDouble2IntFrac(data.accel.x, &integerAccel[0], &fracAccel[0], FRAC_PRECISION);
            vDouble2IntFrac(data.accel.y, &integerAccel[1], &fracAccel[1], FRAC_PRECISION);
            vDouble2IntFrac(data.accel.z, &integerAccel[2], &fracAccel[2], FRAC_PRECISION);
            INFO("[Accel] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integerAccel[0], fracAccel[0], integerAccel[1], fracAccel[1], integerAccel[2], fracAccel[2]);

            /* Convert Gyro Double values for print */
            vDouble2IntFrac(data.gyro.x, &integerGyro[0], &fracGyro[0], FRAC_PRECISION);
            vDouble2IntFrac(data.gyro.y, &integerGyro[1], &fracGyro[1], FRAC_PRECISION);
            vDouble2IntFrac(data.gyro.z, &integerGyro[2], &fracGyro[2], FRAC_PRECISION);
            INFO("[Gyro] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integerGyro[0], fracGyro[0], integerGyro[1], fracGyro[1], integerGyro[2], fracGyro[2]);

            /* Convert Temperature Double values for print */
            vDouble2IntFrac(data.temperature, &integerTemp, &fracTemp, FRAC_PRECISION);
            INFO("[Temperature] [%d.%04u]", integerTemp, fracTemp);

            LOG_UART("%s;%d;%d.%04u;%d.%04u;%d.%04u;%d.%04u;%d.%04u;%d.%04u;%d.%04u",
                        data.imu, data.ms,
                        integerAccel[0], fracAccel[0], integerAccel[1], fracAccel[1], integerAccel[2], fracAccel[2],
                        integerGyro[0], fracGyro[0], integerGyro[1], fracGyro[1], integerGyro[2], fracGyro[2],
                        integerTemp, fracTemp);
        }
    }
}

/**
 * @brief Convert Double values in Integer and Decimal values
 *
 * @param input Double value to be converted
 * @param integer Integer part of the input
 * @param fraction Decimal part of the input
 * @param precision Precision of the decimal fraction output
 */
static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision)
{
    /* Extract signal */
    bool signal = (input < 0) ? false : true;

    /* Extract integer part */
    input = ABS(input);
    int tmpInt = input;

    /* Extract fraction part in float */
    double tmpFloatFrac = input - (double)tmpInt;
    /* Running power of 10 precision */
    double valPrec = 1;
    for (int i = 0; i < precision; i++)
    {
        valPrec *= 10;
    }
    /* Get fraction value based on the precision */
    uint32_t tmpFrac = tmpFloatFrac * valPrec;

    /* Return values */
    if (integer)
    {
        *integer = signal ? tmpInt : (-tmpInt);
    }
    if (fraction)
    {
        *fraction = tmpFrac;
    }
}
