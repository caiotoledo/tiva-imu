#pragma once

#include <gmock/gmock.h>

#include <memory>

#include <FreeRTOS.h>

class FreeRtosMock {
 public:
  MOCK_METHOD0(xTaskGetTickCount, TickType_t());
  MOCK_METHOD0(xTaskGetTickCountFromISR, TickType_t());
};

/* TODO: Move this definition to a generic place */
template <typename T>
class SingletonEnvMock
{
private:
	T mock;
	SingletonEnvMock() = default;
public:
	static T *GetInstance()
	{
		static SingletonEnvMock<T> instance;
		return &instance.mock;
	}

	~SingletonEnvMock() = default;
};
