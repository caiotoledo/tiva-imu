#include <gmock/gmock.h>

#include "freertos_mock.hpp"

extern "C" {
	TickType_t xTaskGetTickCount(void)
	{
		return SingletonFreertosMock::GetInstance()->xTaskGetTickCount();
	}
	TickType_t xTaskGetTickCountFromISR(void)
	{
		return SingletonFreertosMock::GetInstance()->xTaskGetTickCountFromISR();
	}
}
