/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO/STM32
**                STMicroelectronics STM32F10x Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include <math.h>
#include "stm32f10x.h"

#include "images.h"


#include "stm32_eval.h"
#include "stm3210c_eval_ioe.h"
#include "stm3210c_eval_lcd.h"
#include "stm32_eval_i2c_ee.h"
#define USE_BOARD
#define USE_LED
#define USE_SEE

#include "main.h"
#include "accel.h"
#include "physix_m.h"
#include "drawing.h"

/* Private macro */
/* Private variables */
static __IO uint32_t Timing;

/* Private function prototypes */
/* Private functions */
int __errno;

GAME_STATE gameStatus = MENU_MAIN;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
 int main(void)
 {

  uint32_t lpCnt;

  SystemInit();

  AccelInit();

  /* SysTick end of count event each 10ms with input clock equal to 9 MHz (HCLK/8, default) */
  SysTick_Config(SystemCoreClock / 100);

  /* Initialize LEDs and push-buttons mounted on STM3210X-EVAL board */
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);

  STM_EVAL_PBInit(Button_WAKEUP, BUTTON_MODE_GPIO);
  STM_EVAL_PBInit(Button_TAMPER, BUTTON_MODE_GPIO);
  STM_EVAL_PBInit(Button_KEY, BUTTON_MODE_GPIO);

  /* Initialize the LCD */
  STM3210C_LCD_Init();

  ShowStartup();
  ShowMenu();

  lpCnt = 1;

  /* Main Loop */
  uint16_t x, y;
  SetBallPos(160, 120);
  while(1)
  {

   if(AccelStatus() == 0xFF)
   {
	   LCD_SetBackColor(Black);

	   AccelData();
/*
	   LCD_SetBackColor(XCOLOR);
	   LCD_DrawFullRect( 210+(GetMA_X()/2), 320-lpCnt, 2, 2);
	   LCD_SetBackColor(YCOLOR);
	   LCD_DrawFullRect( 210+(GetMA_Y()/2), 320-lpCnt, 2, 2);
	   LCD_SetBackColor(ZCOLOR);
	   LCD_DrawFullRect( 210+(GetMA_Z()/2), 320-lpCnt, 2, 2);
*/
	   get_pos(5, GetMA_X(), GetMA_Y(), &x, &y);
	   Timing = 0;
	   DrawBall(x, y);
	   Delay(2);
/*
	   lpCnt+=DRAW_STEP;
	   if(lpCnt >= 320)
	   {
		   lpCnt = 1;
		   RepaintScreen();
	   }
	   */
   }
  }
 }

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void RepaintScreen( void )
{
	LCD_SetTextColor(Black);
	int i;
	for(i = 180; i < 240; i++)
	{
		LCD_DrawLine( i, 0, 320, Horizontal);
	}
	LCD_SetTextColor(Blue);
	LCD_DrawLine( 210, 0, 320, Horizontal);
	LCD_SetTextColor(Green);

}

void Putpixel(uint16_t x, uint16_t y, uint16_t col)
{
	LCD_DrawLine( y, 320-x, 1, Horizontal);
}

/**
  * @brief  Increments the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay(void)
{
  if (Timing != 0xFFFF)
  {
	  Timing++;
  }
}

void Delay(uint16_t nTime)
{
	while(Timing < nTime)
	{
	}
}

void ShowStartup()
{
	LCD_Clear(BACK_COLOR);
	LCD_SetBackColor(BACK_COLOR);
	LCD_SetTextColor(TEXT_COLOR);

	DrawImage(90, 190, LOGO_IMAGE_W, LOGO_IMAGE_H, logo_image);

	LCD_DisplayStringLine(LINE(10), (uint8_t*)"              Tap To Start");
	LCD_DisplayStringLine(LINE(12), (uint8_t*)"(c) 2011 T. Fedosejev & M. Buinevicius");
	LCD_DisplayStringLine(LINE(13), (uint8_t*)"M. Buinevicius - Physics Engine");
	LCD_DisplayStringLine(LINE(14), (uint8_t*)"T. Fedosejev - Graphics & Controls");

	TS_STATE* TouchScreen = IOE_TS_GetState();
	while(TouchScreen->TouchDetected != 128){
		TouchScreen = IOE_TS_GetState();
	}
	Delay(200);
}

void ShowMenu()
{
	int i, u;
	LCD_Clear(BACK_COLOR);

	//DrawBackground(8, 8, background_image);

	switch(gameStatus)
	{
		case MENU_MAIN:
			DrawImage(10, 310, BUTTON_IMAGE_W, BUTTON_IMAGE_H, newGame_image);
			DrawImage(10+BUTTON_IMAGE_H, 310, BUTTON_IMAGE_W, BUTTON_IMAGE_H, accelData_image);
			/*
			DrawImage(10, 310, MENUTOP_IMAGE_W, MENUTOP_IMAGE_H, menuTop_image);

			for(i = MENUTOP_IMAGE_H+10; i < 229; i++)
			{
				LCD_SetCursor(i, 310);
				LCD_WriteRAM_Prepare();
				for(u = 0; u < MENUTOP_IMAGE_W; u++)
				{
					LCD_WriteRAM(menuTop_image[(MENUTOP_IMAGE_H - 1) * MENUTOP_IMAGE_W + u]);
				}

				LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
			}

			for(i = MENUTOP_IMAGE_H-1; i > 0 ; i--)
			{
				LCD_SetCursor(230-i, 310);
				LCD_WriteRAM_Prepare();
				for(u = 0; u < MENUTOP_IMAGE_W; u++)
				{
					LCD_WriteRAM(menuTop_image[i * MENUTOP_IMAGE_W + u]);
				}
				LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
			}

			DrawChar(40, )

*/
			TS_STATE* TouchScreen = IOE_TS_GetState();
			uint8_t menuSelected = 0;
			while(menuSelected == 0){
				TouchScreen = IOE_TS_GetState();
				if(TouchScreen->TouchDetected == 128)
				{
					//char lcdLine[20];
					//sprintf(lcdLine, "Touch X: %d Y: %d     ", (int)TouchScreen->X, (int)TouchScreen->Y);
					//LCD_DisplayStringLine(LINE(10), (uint8_t*)lcdLine);
					if((TouchScreen->X >= 10) && (TouchScreen->X <= (BUTTON_IMAGE_W + 10))
							&& (TouchScreen->Y >= 10) && (TouchScreen->Y <= BUTTON_IMAGE_H + 10))
					{
						gameStatus = PLAYING;
						menuSelected = 1;
					}
					else if((TouchScreen->X >= 10+BUTTON_IMAGE_W) && (TouchScreen->X <= (BUTTON_IMAGE_W*2 + 10))
							&& (TouchScreen->Y >= 10+BUTTON_IMAGE_H) && (TouchScreen->Y <= BUTTON_IMAGE_H*2 + 10))
					{
							gameStatus = MENU_ACCEL;
							ShowMenu();
							menuSelected = 1;
					}
					else
					{
						menuSelected = 0;
					}
				}
			}
			Delay(400);
		break;
		default:

		break;
	}
}

void Bounced()
{

}

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval sEE_FAIL.
  */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* Return with error code */
  return sEE_FAIL;
}

#endif
#endif /* USE_SEE */



