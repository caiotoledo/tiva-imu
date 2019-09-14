#ifndef TASKS_CONFIG_H_
#define TASKS_CONFIG_H_

/* CONFIGURATION TASKS */
/* Led task configuration */
#define TASK_LED_PRIORITY       (tskIDLE_PRIORITY)
#define TASK_LED_STACKSIZE      (configMINIMAL_STACK_SIZE)
/* IMU Log task configuration */
#define TASK_IMULOG_PRIORITY    (TASK_LED_PRIORITY+1)
#define TASK_IMULOG_STACKSIZE   (configMINIMAL_STACK_SIZE)
/* MPU6050 task timer configuration */
#define TASK_MPU6050_PERIOD     (1000/portTICK_RATE_MS)
#define TASK_MPU6050_STACKSIZE  (configMINIMAL_STACK_SIZE)

#endif /* TASKS_CONFIG_H_ */