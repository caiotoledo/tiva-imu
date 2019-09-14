#ifndef TASKS_CONFIG_H_
#define TASKS_CONFIG_H_

/* Upper limit for task priority */
#define TASK_PRIORITY(x)        (x > configMAX_PRIORITIES ? configMAX_PRIORITIES : x)

/* CONFIGURATION TASKS */
/* Led task configuration */
#define TASK_LED_PRIORITY       TASK_PRIORITY(tskIDLE_PRIORITY)
#define TASK_LED_STACKSIZE      (configMINIMAL_STACK_SIZE)
/* IMU Log task configuration */
#define TASK_IMULOG_PRIORITY    TASK_PRIORITY(TASK_LED_PRIORITY+1)
#define TASK_IMULOG_STACKSIZE   (configMINIMAL_STACK_SIZE)
/* MPU6050 task timer configuration */
#define TASK_MPU6050_PERIOD     (1000/portTICK_RATE_MS)
#define TASK_MPU6050_STACKSIZE  (configMINIMAL_STACK_SIZE)

#endif /* TASKS_CONFIG_H_ */