#include <stdbool.h>

#include <utils.h>

int64_t UTILS_ConvertStringToNumber(const char *buf, size_t len)
{
  int64_t val = 0;
  /* Check negative numbers */
  bool positive = (buf[0] != '-') ? true : false;

  uint8_t initialIndex = (positive ? 0 : 1);
  for (size_t i = initialIndex; (buf[i] != '\0' && i < len); i++)
  {
    val = val * 10 + buf[i] - '0';
  }

  return (positive ? val : -val);
}

int16_t UTILS_ConvertTwosComplementToNumber(uint16_t value)
{
  int16_t ret = 0;
  if (value == 0x8000)
  {
    ret = (int16_t)(0x8000);
  }
  else if (!((value & 0x8000) == 0x8000))
  {
    ret = value;
  }
  else
  {
    value = (((~value) + 1) & 0x7FFF);
    ret = -value;
  }
  return ret;
}
