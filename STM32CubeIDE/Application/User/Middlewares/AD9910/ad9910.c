/*
 * ad9910.c
 *
 *  Created on: Nov 16, 2024
 *      Author: abess
 */

#include "ad9910.h"



// Initialisation de l'AD9910
void AD9910_Init(SPI_HandleTypeDef *hspi) {
    // Configuration des broches
    HAL_GPIO_WritePin(AD9910_CS_PORT, AD9910_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD9910_RESET_PORT, AD9910_RESET_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9910_IO_UPDATE_PORT, AD9910_IO_UPDATE_PIN, GPIO_PIN_RESET);

    // Reset DDS
    AD9910_Reset();
}


uint8_t AD9910_GetRegSize(uint8_t reg_id) {
    // Renvoit la taille du buffer
	switch(reg_id)
		{
			case 8:
				return 2;

			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 7:
			case 9:
			case 0x0A:
			case 0x0D:
			case 0x16:
				return 4;

			case 0x0B:
			case 0x0C:
			case 0x0E:
			case 0x0F:
			case 0x10:
			case 0x11:
			case 0x12:
			case 0x13:
			case 0x14:
			case 0x15:
				return 8;

			default :
	            break;
		}

		return 0; // other registers
}

// Fonction de réinitialisation
void AD9910_Reset(void) {
    HAL_GPIO_WritePin(AD9910_RESET_PORT, AD9910_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10); // 10 ms de délai
    HAL_GPIO_WritePin(AD9910_RESET_PORT, AD9910_RESET_PIN, GPIO_PIN_RESET);
}

// Mise à jour des registres
void AD9910_IOUpdate(void) {
    HAL_GPIO_WritePin(AD9910_IO_UPDATE_PORT, AD9910_IO_UPDATE_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(AD9910_IO_UPDATE_PORT, AD9910_IO_UPDATE_PIN, GPIO_PIN_RESET);
}

// Écriture dans un registre de l'AD9910
HAL_StatusTypeDef AD9910_WriteRegister(uint8_t reg_addr, uint64_t data) {
    uint8_t write_buffer[9] = {0};
    uint8_t reg_size = AD9910_GetRegSize(reg_addr);

    //write_buffer[0] = reg_addr & 0x7F; // MSB à 0 pour écrire

    write_buffer[0] = 0 | reg_addr;	//Write + Register ID

	int8_t shift = (int8_t)((reg_size-1) * 8);
	for(int i = 1; shift >= 0 ; i++)
	{
		if(i < sizeof(write_buffer))
		{
			write_buffer[i] = (uint8_t)(data >> shift);
		}else{
			printf("update_ad9910_reg overflow !");
		}
		shift -= 8;
	}

    HAL_GPIO_WritePin(AD9910_CS_PORT, AD9910_CS_PIN, GPIO_PIN_RESET); // Sélection du périphérique
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&AD9910_SPI_PORT, write_buffer, (reg_size+1), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AD9910_CS_PORT, AD9910_CS_PIN, GPIO_PIN_SET); // Désélection du périphérique
    return status;
}

// Lecture dans un registre de l'AD9910
HAL_StatusTypeDef AD9910_ReadRegister(uint8_t reg_addr, uint64_t *data) {
    uint8_t write_buffer[9] = {0};
    uint8_t read_buffer[9] = {0};
    uint8_t reg_size = AD9910_GetRegSize(reg_addr);

    write_buffer[0] = reg_addr | 0x80; // MSB à 1 pour lire

    HAL_GPIO_WritePin(AD9910_CS_PORT, AD9910_CS_PIN, GPIO_PIN_RESET); // Sélection du périphérique

    HAL_SPI_TransmitReceive(&AD9910_SPI_PORT, write_buffer, read_buffer,(reg_size+1), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AD9910_CS_PORT, AD9910_CS_PIN, GPIO_PIN_SET); // Désélection du périphérique

    *data = 0;  // Initialise la data à 0

       /*for (int i = 0; i < 8; i++) {
           *data |= ((uint64_t)read_buffer[i] << (i * 8));  // Reconstruction en Little Endian
       }*/

    switch (reg_size) {
                case 2: *data = ((uint64_t)read_buffer[1] << 8) | ((uint64_t)read_buffer[2]); break;
                case 4: *data = ((uint64_t)read_buffer[1] << 24) | ((uint64_t)read_buffer[2] << 16) | ((uint64_t)read_buffer[3] << 8) | (uint64_t)read_buffer[4]; break;
                case 8: *data = ((uint64_t)read_buffer[1] << 56) | ((uint64_t)read_buffer[2] << 48) | ((uint64_t)read_buffer[3] << 40) | ((uint64_t)read_buffer[4] << 32) | ((uint64_t)read_buffer[5] << 24) | ((uint64_t)read_buffer[6] << 16) | ((uint64_t)read_buffer[7] << 8) | (uint64_t)read_buffer[8]; break;
                default:
                    UART_SendString("Erreur de size sur regsize");
                    return;
            }


    return HAL_OK;
}

