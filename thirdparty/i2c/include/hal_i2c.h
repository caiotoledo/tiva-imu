#ifndef HAL_I2C_H_
#define HAL_I2C_H_

typedef enum {
    I2C0,
    I2C1,
    I2C2,
    I2C3
} eI2C_BASE;

void I2C_Enable(eI2C_BASE i2c);
uint32_t I2C_Read_Reg(eI2C_BASE i2c, uint8_t slave_addr, uint8_t reg_addr);

#endif /* HAL_I2C_H_ */
