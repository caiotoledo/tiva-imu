#include "imu_cmd.h"

#include <utils.h>
#include <cmd_task.h>
#include <log.h>

#include "imu_internal.h"
#include "imu_types.h"

static BaseType_t IMURunCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t IMUSampleCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static const CLI_Command_Definition_t xIMURun =
{
  "imu-run", /* The command string. */
  "imu-run [time]:\r\n Run the IMU sample for [time] seconds\r\n", /* Help string. */
  IMURunCommand, /* The function to run. */
  1 /* One parameter is expected. */
};

static const CLI_Command_Definition_t xIMUSample =
{
  "imu-sample", /* The command string. */
  "imu-sample [period]:\r\n Configure IMU [period] sample rate in milliseconds\r\n", /* Help string. */
  IMUSampleCommand, /* The function to run. */
  1 /* One parameter is expected. */
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

  int64_t time = UTILS_ConvertStringToNumber(pcTime, lTimeLength);

  if (time >= 0)
  {
    /* Start and Stop MPU6050 Task */
    ManageMPU6050Tasks(false);
    INFO("Resume MPU6050 tasks for [%d] seconds", (int)time);
    ManageMPU6050Tasks(true);
    vTaskDelay((time*1000U)/portTICK_RATE_MS);
    ManageMPU6050Tasks(false);
  }
  else
  {
    ERROR("Invalid time [%d]", (int)time);
  }

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

  int64_t time = UTILS_ConvertStringToNumber(pcTime, lTimeLength);
  time = MAX(0,time); /* Allow only positive time */

  /* Update sample rate */
  int ret = MPU6050Task_SetSampleRate(time);
  if (ret == 0)
  {
    INFO("Update imu sample period [%d] ms", (int)time);
  }
  else
  {
    ERROR("INVALID imu sample period [%d] ms", (int)time);
  }

  /* Do not use the pcWriteBuffer to output on console */
  if (xWriteBufferLen >= 1)
  {
    pcWriteBuffer[0] = '\0';
  }

  return pdFALSE;
}

