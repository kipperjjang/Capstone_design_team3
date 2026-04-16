#include "DRV8353_setting.h"
#include "main.h"

// DRV8353 setting status. 0x00 / 0x01 / 0x02 / 0x03 / 0x04 / 0x05 / 0x06 / ALL_OK
uint8_t DRV8353_setting_status = 0;

uint16_t DRV8353_data[7];
uint16_t DRV8353_readaddress[7]  = {0x8000 | (0x00<<11), 0x8000 | (0x01<<11), 0x8000 | (0x02<<11), 0x8000 | (0x03<<11), 0x8000 | (0x04<<11), 0x8000 | (0x05<<11), 0x8000 | (0x06<<11)};
uint16_t DRV8353_flag_reset = 0x1001;

uint16_t DRV8353_setting[5] = {0x1020, 0x1e33, 0x2733, 0x2969, 0x3243};
uint16_t DRV8353_current_cali[4] = {0x3801, 0x325F, 0x3243, 0x3800};
/*
DRV8353 spi setting info
0x02 : 0 0010 000 0010 0000 -> 0x1020 (3x PWM mode)
0x03 : 0 0011 110 0011 0011 -> 0x1e33 (Lock SPI command, HS MOSFET current 150/300mA)  //at DRV8323 : 170/340 mA
0x04 : 0 0100 111 0011 0011 -> 0x2733 (LS MOSFET current 150/300mA)
0x05 : 0 0101 001 0110 1001 -> 0x2969 (VDS 0.6V, 100ns deadtime)
0x06 : 0 0110 010 0100 0011 -> 0x3243 (current amplifier gain 10V/V)



current calibration sequence
0x07 : 0 0111 000 0000 0001 -> 0x3801 (CAL_MODE on)
0x06 : 0 0110 010 0101 1111 -> 0x325F (CSA_CAL_A/B/C on, current amplifier gain 10V/V)
50us halt
0x06 : 0 0110 010 0100 0011 -> 0x3243 (CSA_CAL_A/B/C off, current amplifier gain 10V/V)
0x07 : 0 0111 000 0000 0000 -> 0x3800 (CAL_MODE off)
*/


void DRV8353_SPI_read(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[0], (uint8_t*)&DRV8353_data[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[1], (uint8_t*)&DRV8353_data[1], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[2], (uint8_t*)&DRV8353_data[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[3], (uint8_t*)&DRV8353_data[3], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[4], (uint8_t*)&DRV8353_data[4], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[5], (uint8_t*)&DRV8353_data[5], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_readaddress[6], (uint8_t*)&DRV8353_data[6], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

void DRV8353_SPI_setting(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_setting[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_setting[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_setting[3], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_setting[4], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_setting[1], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

void DRV8353_errorflag_reset(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&DRV8353_flag_reset, (uint8_t*)&DRV8353_data[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

int DRV8353_issetuped(){
	DRV8353_setting_status |= (DRV8353_data[0] == 0x0000) << 7;
	DRV8353_setting_status |= (DRV8353_data[1] == 0x0000) << 6;
	DRV8353_setting_status |= (DRV8353_data[2] == (DRV8353_setting[0] & 0x07FF)) << 5;
	DRV8353_setting_status |= (DRV8353_data[3] == (DRV8353_setting[1] & 0x07FF)) << 4; // change it later 00->11
	DRV8353_setting_status |= (DRV8353_data[4] == (DRV8353_setting[2] & 0x07FF)) << 3; // change it later 00->11
	DRV8353_setting_status |= (DRV8353_data[5] == (DRV8353_setting[3] & 0x07FF)) << 2;
	DRV8353_setting_status |= (DRV8353_data[6] == (DRV8353_setting[4] & 0x07FF)) << 1;

	DRV8353_setting_status |= (DRV8353_setting_status & 0xFE) == 0xFE;

	return DRV8353_setting_status;
}

void DRV8353_current_calibration(){
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_current_cali[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_current_cali[1], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_current_cali[2], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&DRV8353_current_cali[3], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}
