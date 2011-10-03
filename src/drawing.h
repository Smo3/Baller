/*
 * drawing.h
 *
 *  Created on: Oct 3, 2011
 *      Author: Tomas
 */

#include <stddef.h>
#include "stm32f10x.h"
#include "stm32_eval.h"

#ifndef DRAWING_H_
#define DRAWING_H_

void DrawImage(uint16_t Y, uint16_t X, uint16_t width, uint16_t height, const uint16_t *c);
void DrawChar(uint8_t Xpos, uint16_t Ypos, uint8_t scale, const uint16_t *c);
void DrawBall(uint16_t Xpos, uint16_t Ypos);
void SetBallPos(uint16_t Xpos, uint16_t Ypos);
void DrawBackground(uint16_t width, uint16_t height, const uint16_t *c);

#endif /* DRAWING_H_ */
