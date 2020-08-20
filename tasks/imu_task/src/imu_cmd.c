#include "imu_cmd.h"

#include <cmd_task.h>
#include <log.h>

#include "imu_internal.h"
#include "imu_types.h"

static BaseType_t IMURunCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t IMUSampleCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static uint32_t ConvertStringToNumber(const char *buf, size_t len);

static const CLI_Command_Definition_t xIMURun =
{
  "imu-run", /* The command string. */
  "imu-run [time]:\r\n Run the IMU sample for [time] seconds\r\n", /* Help string. */
  IMURunCommand, /* The function to run. */
  1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xIMUSample =
{
  "imu-sample", /* The command string. */
  "imu-sample [period]:\r\n Configure IMU [period] sample rate in milliseconds\r\n", /* Help string. */
  IMUSampleCommand, /* The function to run. */
  1 /* No parameters are expected. */
};

void vIMU_ConfigureCommands(void)
{
  /* Register IMU Commands */
  CMD_RegFuncCommand(&xIMURun);
  CMD_RegFuncCommand(&xIMUSample);
}

static BaseType_t IMURunCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  const char *pcTime;
  BaseType_t lTimeLength;

  /* Get time value */
  pcTime = FreeRTOS_CLIGetParameter (
    pcCommandString,
    1,
    &lTimeLength
  );

  /* Convert String to Integer */
  uint32_t time = ConvertStringToNumber(pcTime, lTimeLength);

  /* Start and Stop MPU6050 Task */
  ManageMPU6050Tasks(false);
  INFO("Resume MPU6050 tasks for [%u] seconds", time);
  ManageMPU6050Tasks(true);
  vTaskDelay((time*1000U)/portTICK_RATE_MS);
  ManageMPU6050Tasks(false);

  /* Do not use the pcWriteBuffer to output on console */
  if (xWriteBufferLen >= 1)
  {
    pcWriteBuffer[0] = '\0';
  }

  return pdFALSE;
}

static BaseType_t IMUSampleCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  const char *pcTime;
  BaseType_t lTimeLength;

  /* Get time value */
  pcTime = FreeRTOS_CLIGetParameter (
    pcCommandString,
    1,
    &lTimeLength
  );

  /* Convert String to Integer */
  uint32_t time = ConvertStringToNumber(pcTime, lTimeLength);

  /* Update sample rate */
  int ret = MPU6050Task_SetSampleRate(time);
  if (ret == 0)
  {
    INFO("Update imu sample period [%u] ms", time);
  }
  else
  {
    ERROR("INVALID imu sample period [%u] ms", time);
  }

  /* Do not use the pcWriteBuffer to output on console */
  if (xWriteBufferLen >= 1)
  {
    pcWriteBuffer[0] = '\0';
  }

  return pdFALSE;
}

/**
 * @brief Convert a String to a Number
 *
 * @param buf String buffer with NULL Terminator
 * @param len String Length
 * @return uint32_t Return Number extracted
 */
static uint32_t ConvertStringToNumber(const char *buf, size_t len)
{
  uint32_t val = 0;
  for (size_t i = 0; (buf[i] != '\0' && i < len); i++)
  {
    val = val * 10 + buf[i] - '0';
  }
  return val;
}
