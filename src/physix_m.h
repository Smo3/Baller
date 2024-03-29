#include <stddef.h>
#include "stm32f10x.h"

#ifndef _PHYSIX_M_H_
#define _PHYSIX_M_H_

void get_pos(float dt, int tilt_x, int tilt_y, uint16_t* posX, uint16_t* posY);
void get_pos2(int dt, int tilt_x, int tilt_y, int* posX, int* posY);

#endif
