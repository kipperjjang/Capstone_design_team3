#include "AS5048A.h"
#include "main.h"

uint16_t mech_angle;

int calculate_parity(uint16_t value) {
    uint8_t parity = 0;  //reset parity
    while (value) {
        parity ^= value & 0x01;  //end bit 1 -> parity ^= 1 (toggle), end bit 0 -> parity ^= 0 (remain)
        value >>= 1;  //move to next value bit (to left bit in value)
    }
    return parity;  //value has odd 1 -> return 1, value has even 1 -> return 0.
}

void AS5048A_ReadAngle(){
    uint16_t command = 0xFFFF; //read angle command is always 0xFFFF

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
    if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&command, (uint8_t*)&mech_angle, 1, HAL_MAX_DELAY) != HAL_OK){
    	while(1){
  		  HAL_Delay(500);
  	  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
    	}
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);

    mech_angle = mech_angle & 0x3FFF;
}

int AS5048A_ReadRegister(uint16_t reg) {
    uint16_t command = reg & 0x3FFF;  //cut 14bit
    uint16_t response;

    command |= (1 << 14);  //read mode

    uint8_t parity = calculate_parity(command);  //get parity bit. AS5048 parity bit type : even(make even number of 1 in 16-bit data)
    if (parity) {  //if parity = 1 -> command has odd 1. so, by adding 1 at 15th bit, command has even 1
        command |= (1 << 15);
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);  //CS low (begin transmission)
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&command, (uint8_t*)&response, 1, HAL_MAX_DELAY);  //CPOL low, CPHA 2 -> spi mode 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);  //CS high (end transmission)

    return response & 0x3FFF;
}
