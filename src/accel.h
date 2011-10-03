/*
 * accel.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Tomas
 */

#ifndef ACCEL_H_
#define ACCEL_H_

#define MEMS_I2C_RCC_APB2Periph_GPIOs RCC_APB2Periph_GPIOB
#define MEMS_I2C_GPIO                 GPIOB
#define MEMS_I2C_RCC_APB1Periph_I2C   RCC_APB1Periph_I2C1

#define SENSITIVITY 0.98

void AccelInit();
uint8_t AccelStatus();
void AccelData();

int8_t GetMA_X();
int8_t GetMA_Y();
int8_t GetMA_Z();
void UpdateMA_X(int8_t data);
void UpdateMA_Y(int8_t data);
void UpdateMA_Z(int8_t data);

#endif /* ACCEL_H_ */
