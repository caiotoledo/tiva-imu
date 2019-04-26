#ifndef HAL_I2C_H_
#define HAL_I2C_H_

/**
 * @brief Enum definition for I2C Interface
 * 
 */
typedef enum {
    I2C0,
    I2C1,
    I2C2,
    I2C3
} eI2C_BASE;

/**
 * @brief Enable the I2C interface
 * 
 * @param i2c I2C Interface
 * @param func Pointer function to provide the ms counter
 */
void I2C_Enable(eI2C_BASE i2c, uint32_t (*func)());

/**
 * @brief Read a register from an slave address
 * 
 * @param i2c I2C Interface (see #eI2C_BASE)
 * @param slave_addr Slave Address
 * @param reg_addr Register Address
 * @return uint32_t Value read from the address
 */
uint32_t I2C_Read_Reg(eI2C_BASE i2c, uint8_t slave_addr, uint8_t reg_addr);

/**
 * @brief Write values to a register
 *
 * @param i2c I2C Interface (see #eI2C_BASE)
 * @param slave_addr Slave Address
 * @param reg_addr Register Address
 * @param num_of_args Number of arguments
 * @param ... Values to be written in the register
 */
void I2C_Send(eI2C_BASE i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t num_of_args, ...);

#endif /* HAL_I2C_H_ */
