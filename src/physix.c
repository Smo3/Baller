/*
 * physix.c
 *
 *  Created on: Oct 2, 2011
 *      Author: Tomas
 */

#include <stddef.h>
#include <math.h>
#include "physix.h"

float _forceX = 0, _forceY = 0, _forceZ = 0;
int _posX = 120, _posY = 160;

void SetForce(int x, int y, int z)
{
	_forceX += x/2*FORCE_SCALE;
	_forceY += y/2*FORCE_SCALE;
	_forceZ += z/2*FORCE_SCALE;
}

void GetPosition(int* posX, int* posY)
{
	_posX +=_forceX;
	_posY +=_forceY;
	*posX = _posX;
	*posY = _posY;
}
