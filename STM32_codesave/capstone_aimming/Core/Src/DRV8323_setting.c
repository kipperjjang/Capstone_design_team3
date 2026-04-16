#include "DRV8323_setting.h"
#include "main.h"

// DRV8323 setting status. 0x00 / 0x01 / 0x02 / 0x03 / 0x04 / 0x05 / 0x06 / ALL_OK
uint8_t DRV8323_setting_status = 0;

uint16_t DRV8323_data[7];
uint16_t DRV8323_readaddress[7]  = {0x8000 | (0x00<<11), 0x8000 | (0x01<<11), 0x8000 | (0x02<<11), 0x8000 | (0x03<<11), 0x8000 | (0x04<<11), 0x8000 | (0x05<<11), 0x8000 | (0x06<<11)};
uint16_t DRV8323_flag_reset = 0x1001;
//uint16_t DRV8323_setting[5] = {0x1020, 0x1e11, 0x2711, 0x2957, 0x3243}; // restore this

uint16_t DRV8323_setting[5] = {0x1020, 0x1e66, 0x2766, 0x2957, 0x3243}; // very low IDRIVE test
/*
DRV8323 spi setting info
0x02 : 0 0010 00000100000 -> 0x1020 (3x PWM mode)
0x03 : 0 0011 11000010001 -> 0x1e11 (Lock SPI command, HS MOSFET current 30/60mA)
0x04 : 0 0100 11100010001 -> 0x2711 (LS MOSFET current 30/60mA)
0x05 : 0 0101 00101010111 -> 0x2957 (VDS 0.6V)
0x06 : 0 0110 01001000011 -> 0x3243 (current amplifier gain 10V/V)

0x02 : 0 0010 00000100000 -> 0x1020 (3x PWM mode)
0x03 : 0 0011 110 0110 0110 -> 0x1e66 (Lock SPI command, HS MOSFET current 30/60mA)
0x04 : 0 0100 111 0110 0110 -> 0x2766 (LS MOSFET current 30/60mA)
0x05 : 0 0101 00101010111 -> 0x2957 (VDS 0.6V)
0x06 : 0 0110 01001000011 -> 0x3243 (current amplifier gain 10V/V)

other bit is default state
*/

void DRV8323_SPI_read(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[0], (uint8_t*)&DRV8323_data[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[1], (uint8_t*)&DRV8323_data[1], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[2], (uint8_t*)&DRV8323_data[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[3], (uint8_t*)&DRV8323_data[3], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[4], (uint8_t*)&DRV8323_data[4], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[5], (uint8_t*)&DRV8323_data[5], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_readaddress[6], (uint8_t*)&DRV8323_data[6], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

void DRV8323_SPI_setting(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8323_setting[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8323_setting[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8323_setting[3], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8323_setting[4], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8323_setting[1], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

void DRV8323_errorflag_reset(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8323_flag_reset, (uint8_t*)&DRV8323_data[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

int DRV8323_issetuped(){
	DRV8323_setting_status |= (DRV8323_data[0] == 0x0000) << 7;
	DRV8323_setting_status |= (DRV8323_data[1] == 0x0000) << 6;
	DRV8323_setting_status |= (DRV8323_data[2] == (DRV8323_setting[0] & 0x07FF)) << 5;
	DRV8323_setting_status |= (DRV8323_data[3] == (DRV8323_setting[1] & 0x07FF)) << 4; // change it later 00->11
	DRV8323_setting_status |= (DRV8323_data[4] == (DRV8323_setting[2] & 0x07FF)) << 3; // change it later 00->11
	DRV8323_setting_status |= (DRV8323_data[5] == (DRV8323_setting[3] & 0x07FF)) << 2;
	DRV8323_setting_status |= (DRV8323_data[6] == (DRV8323_setting[4] & 0x07FF)) << 1;

	DRV8323_setting_status |= (DRV8323_setting_status & 0xFE) == 0xFE;

	return DRV8323_setting_status;
}
