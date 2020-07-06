#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "mocks/freertos_mock.hpp"

extern "C" {
	#include <timer.h>
}

using ::testing::Return;

TEST(lib_timer, SimpleCall_GetMillis)
{
	auto env = SingletonFreertosMock::GetInstance();

	TickType_t test_val = 10;
	EXPECT_CALL(*env, xTaskGetTickCount())
		.Times(1)
		.WillRepeatedly(Return(test_val));

	uint32_t val = GetMillis();

	EXPECT_EQ(val, (test_val/portTICK_RATE_MS));
}

TEST(lib_timer, SimpleCall_GetMillisFromISR)
{
	auto env = SingletonFreertosMock::GetInstance();

	TickType_t test_val = 10;
	EXPECT_CALL(*env, xTaskGetTickCountFromISR())
		.Times(1)
		.WillRepeatedly(Return(test_val));

	uint32_t val = GetMillisFromISR();

	EXPECT_EQ(val, (test_val/portTICK_RATE_MS));
}
