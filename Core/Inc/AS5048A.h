#include "main.h"

extern uint16_t mech_angle;

int calculate_parity(uint16_t);
void AS5048A_ReadAngle(void);
int AS5048A_ReadRegister(uint16_t);
