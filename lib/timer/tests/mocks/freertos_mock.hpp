#pragma once

#include <gmock/gmock.h>

#include <memory>

#include <FreeRTOS.h>

class FreeRtosMock {
 public:
  MOCK_METHOD0(xTaskGetTickCount, TickType_t());
  MOCK_METHOD0(xTaskGetTickCountFromISR, TickType_t());
};

class SingletonFreertosMock
{
private:
	FreeRtosMock mock;
	SingletonFreertosMock() = default;
public:
	static FreeRtosMock *GetInstance()
	{
		static SingletonFreertosMock instance;
		return &instance.mock;
	}

	~SingletonFreertosMock() = default;
};
