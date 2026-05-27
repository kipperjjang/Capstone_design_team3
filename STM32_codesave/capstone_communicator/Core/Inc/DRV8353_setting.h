#include "main.h"

extern uint16_t DRV8353_data[7];
extern uint32_t spi3_status;

void DRV8353_SPI_read(void);
void DRV8353_SPI_setting(void);
void DRV8353_errorflag_reset(void);
int DRV8353_issetuped();

void DRV8353_current_calibration(void);

