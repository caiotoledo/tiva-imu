#include <FreeRTOS.h>
#include <queue.h>

#include <log.h>

#include "imu_types.h"
#include "imu_logger.h"

#define ABS(x)      (x < 0 ? (-x) : (x))

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
            int integer[3];
            uint32_t frac[3];

            INFO("");
            INFO("[%s] TIME %d ms", data.imu, data.ms);

            /* Convert Accel Double values for print */
            vDouble2IntFrac(data.accel.x, &integer[0], &frac[0], 4U);
            vDouble2IntFrac(data.accel.y, &integer[1], &frac[1], 4U);
            vDouble2IntFrac(data.accel.z, &integer[2], &frac[2], 4U);
            INFO("[Accel] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integer[0], frac[0], integer[1], frac[1], integer[2], frac[2]);

            /* Convert Gyro Double values for print */
            vDouble2IntFrac(data.gyro.x, &integer[0], &frac[0], 4U);
            vDouble2IntFrac(data.gyro.y, &integer[1], &frac[1], 4U);
            vDouble2IntFrac(data.gyro.z, &integer[2], &frac[2], 4U);
            INFO("[Gyro] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integer[0], frac[0], integer[1], frac[1], integer[2], frac[2]);

            /* Convert Temperature Double values for print */
            vDouble2IntFrac(data.temperature, &integer[0], &frac[0], 4U);
            INFO("[Temperature] [%d.%04u]", integer[0], frac[0]);
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
