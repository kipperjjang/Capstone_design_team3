#include "main.h"

extern uint16_t DRV8323_data[7];

void DRV8323_SPI_read(void);
void DRV8323_SPI_setting(void);
void DRV8323_errorflag_reset(void);
int DRV8323_issetuped();
