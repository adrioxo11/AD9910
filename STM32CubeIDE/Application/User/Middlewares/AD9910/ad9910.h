/*
 * ad9910.h
 *
 *  Created on: Nov 16, 2024
 *      Author: abess
 */

#ifndef APPLICATION_USER_MIDDLEWARE_AD9910_AD9910_H_
#define APPLICATION_USER_MIDDLEWARE_AD9910_AD9910_H_


#include "stm32h7xx_hal.h"
extern SPI_HandleTypeDef hspi1;
#define NB_AD9910_REG		0x17

// Définition des broches pour AD9910
#define AD9910_SPI_PORT       hspi1 // Modifier selon votre configuration
#define AD9910_CS_PIN         GPIO_PIN_1
#define AD9910_CS_PORT        GPIOB

#define AD9910_RESET_PIN      GPIO_PIN_2
#define AD9910_RESET_PORT     GPIOB

#define AD9910_IO_UPDATE_PIN  GPIO_PIN_10
#define AD9910_IO_UPDATE_PORT GPIOF

// Déclaration des fonctions
void AD9910_Init(SPI_HandleTypeDef *hspi);
void AD9910_Reset(void);
void AD9910_IOUpdate(void);
HAL_StatusTypeDef AD9910_WriteRegister(uint8_t reg_addr, uint64_t data);
HAL_StatusTypeDef AD9910_ReadRegister(uint8_t reg_addr, uint64_t *data);


#endif /* APPLICATION_USER_MIDDLEWARE_AD9910_AD9910_H_ */
