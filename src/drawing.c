/*
 * drawing.c
 *
 *  Created on: Oct 3, 2011
 *      Author: Tomas
 */

#include <stddef.h>
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "stm3210c_eval_lcd.h"
#include "main.h"
#include "drawing.h"

static uint16_t Xold, Yold;

void DrawChar(uint8_t Xpos, uint16_t Ypos, uint8_t scale, const uint16_t *c)
{
  uint32_t index = 0, i = 0;
  uint8_t Xaddress = 0;

  Xaddress = Xpos;

  LCD_SetCursor(Xaddress, Ypos);

  for(index = 0; index < 16*scale; index++)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < 8*scale; i++)
    {
      if((((c[index] & ((0x80 << ((8*scale / 12 ) * 8 ) ) >> i)) == 0x00) &&(8*scale <= 12))||
        (((c[index] & (0x1 << i)) == 0x00)&&(8*scale > 12 )))

      {
        LCD_WriteRAM(BACK_COLOR);
      }
      else
      {
        LCD_WriteRAM(TEXT_COLOR);
      }
    }
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
    Xaddress++;
    LCD_SetCursor(Xaddress, Ypos);
  }
}

void DrawBackground(uint16_t width, uint16_t height, const uint16_t *c)
{
	int i, u, k;
	for(k = 0; k < 320/width; k++)
	{
		for(i = 0; i < height; i++)
		{
			LCD_SetCursor(i, k*width);
			LCD_WriteRAM_Prepare();
			for(u = 0; u < width; u++)
			{
				LCD_WriteRAM(c[i * width + u]);
			}
			LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
		}
	}
}

void DrawImage(uint16_t Y, uint16_t X, uint16_t width, uint16_t height, const uint16_t *c)
{
	int i, u;
	for(i = 0; i < height; i++)
	{
	   	LCD_SetCursor(Y+i, X);
	   	LCD_WriteRAM_Prepare();
	   	for(u = 0; u < width; u++)
	   	{
	 		LCD_WriteRAM(c[i * width + u]);
	  	}
	  	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
	}
}

void SetBallPos(uint16_t Xpos, uint16_t Ypos)
{
	Xold = Xpos;
	Yold = Ypos;
	DrawBall(Xpos, Ypos);
}

void DrawBall(uint16_t Xpos, uint16_t Ypos)
{
	LCD_SetBackColor(BACK_COLOR);
	LCD_DrawFullCircle(Yold, Xold, BALL_SIZE);
	LCD_SetBackColor(Yellow);
	LCD_DrawFullCircle(Ypos, Xpos, BALL_SIZE);
	Yold = Ypos;
	Xold = Xpos;
}
