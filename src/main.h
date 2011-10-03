/*
 * main.h
 *
 *  Created on: Oct 1, 2011
 *      Author: Tomas
 */
/* Includes */
#include <stddef.h>
#include <math.h>
#include "stm32f10x.h"

#ifndef MAIN_H_
#define MAIN_H_

#define BALL_SIZE 10

#define RGB565(R, G, B) ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3))

#define MEMS_I2C I2C1

#define MEMS_DEVICE_ID 	0x3B
#define MEMS_I2C_ADDR 	0x38

#define MEMS_Who_Am_I 	0x0F
#define MEMS_Status_Reg	0x27
#define MEMS_Ctrl_Reg1 	0x20
#define MEMS_OutX 		0x29
#define MEMS_OutY 		0x2B
#define MEMS_OutZ 		0x2D


#define DRAW_STEP 1
#define BACK_COLOR 0x2945
#define TEXT_COLOR RGB565(0xC7, 0xC7, 0xC7)

#define XCOLOR RGB565(0x32, 0x64, 0xFF)
#define YCOLOR RGB565(0xFF, 0x9B, 0x00)
#define ZCOLOR RGB565(0x00, 0x9B, 0x00)

typedef enum {
	MENU_MAIN = 0,
	MENU_ACCEL = 1,
	MENU_AUTHORS = 2,
	PLAYING = 3,
	PAUSE = 4,
	GAMEOVER = 5,
} GAME_STATE;

#define MENU_BORDER RGB565(0x33, 0x33, 0x33)
#define BUTTON_BORDER RGB565(0x50, 0x50, 0x50)
#define BUTTON_BACKGROUND RGB565(0x40, 0x40, 0x40)

void Delay(uint16_t nTime);
void TimingDelay(void);
void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t col);
void RepaintScreen( void );
void Putpixel(uint16_t x, uint16_t y, uint16_t col);
void ShowStartup();
void ShowMenu();

#endif /* MAIN_H_ */
