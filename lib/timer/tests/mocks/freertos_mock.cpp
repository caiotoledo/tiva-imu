#include <gmock/gmock.h>

#include "freertos_mock.hpp"

extern "C" {
	TickType_t xTaskGetTickCount(void)
	{
		return SingletonEnvMock<FreeRtosMock>::GetInstance()->xTaskGetTickCount();
	}
	TickType_t xTaskGetTickCountFromISR(void)
	{
		return SingletonEnvMock<FreeRtosMock>::GetInstance()->xTaskGetTickCountFromISR();
	}
}
