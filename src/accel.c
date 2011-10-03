/*
 * accel.c
 *
 *  Created on: Oct 2, 2011
 *      Author: Tomas
 */

#include <stddef.h>
#include <math.h>
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "stm32_mems.h"
#include "stm32_mems_adapter.h"
#include "stm3210c_eval_ioe.h"
#include "stm3210c_eval_lcd.h"
#include "accel.h"
#include "main.h"

I2C_InitTypeDef  I2C_InitStructure;

static uint8_t memsData;
static uint32_t revid, devid;
int8_t memsX, memsY, memsZ;

/* Moving average calibration data */
static int8_t lastX[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int8_t lastY[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int8_t lastZ[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t cntX = 0, cntY = 0, cntZ = 0;

void AccelInit()
{
	char lcdLine[20];
	/* I2C configuration -------------------------------------------------------*/
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = LIS302DL_I2C_ADDR;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed = 400000;

	  I2C_Cmd(I2C1, ENABLE);
	  I2C_Init(I2C1, &I2C_InitStructure);
	  I2C_CalculatePEC(I2C1, ENABLE);
	  I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, ENABLE);

	  /* Configure the IO Expander */
	    if (IOE_Config() == IOE_OK)
	    {
	     LCD_DisplayStringLine(Line0, (uint8_t*)"IO Expander       OK");
	    }
	    else
	    {
	     LCD_DisplayStringLine(Line0, (uint8_t*)"IO Expander     FAIL");
	     LCD_DisplayStringLine(Line1, (uint8_t*)"Please Restart board");
	     while(1);
	    }

	    /* Configure MEMS */
	    memsData = I2C_ReadDeviceRegister(MEMS_I2C_ADDR, MEMS_Who_Am_I);
	    if (memsData == MEMS_DEVICE_ID)
	    {
	     LCD_DisplayStringLine(Line1, (uint8_t*)"MEMS comms        OK");
	    }
	    else
	    {
	     LCD_DisplayStringLine(Line1, (uint8_t*)"MEMS comms      FAIL");
	     LCD_DisplayStringLine(Line2, (uint8_t*)"Please Restart board");
	     while(1);
	    }

	    /* Power up MEMS */
	    if (I2C_WriteDeviceRegister(MEMS_I2C_ADDR, MEMS_Ctrl_Reg1, 0x47) == IOE_OK)
	    {
	     LCD_DisplayStringLine(Line2, (uint8_t*)"MEMS powerup      OK");
	    }
	    else
	    {
	     LCD_DisplayStringLine(Line2, (uint8_t*)"MEMS powerup    FAIL");
	     LCD_DisplayStringLine(Line3, (uint8_t*)"Please Restart board");
	     while(1);
	    }

	    revid = DBGMCU_GetREVID();
	    devid = DBGMCU_GetDEVID();
	    sprintf(lcdLine, "Rev %d   Dev %d ", (int)revid, (int)devid);
	    LCD_DisplayStringLine(Line5, (uint8_t*)lcdLine);
	    LCD_DisplayStringLine(Line4, (uint8_t*)"64=1G          +/-2G");
	    /* Configure the IO Expander */
	      if (IOE_Config() == IOE_OK)
	      {
	       LCD_DisplayStringLine(Line0, (uint8_t*)"IO Expander       OK");
	      }
	      else
	      {
	       LCD_DisplayStringLine(Line0, (uint8_t*)"IO Expander     FAIL");
	       LCD_DisplayStringLine(Line1, (uint8_t*)"Please Restart board");
	       while(1);
	      }

	      /* Configure MEMS */
	      memsData = I2C_ReadDeviceRegister(MEMS_I2C_ADDR, MEMS_Who_Am_I);
	      if (memsData == MEMS_DEVICE_ID)
	      {
	       LCD_DisplayStringLine(Line1, (uint8_t*)"MEMS comms        OK");
	      }
	      else
	      {
	       LCD_DisplayStringLine(Line1, (uint8_t*)"MEMS comms      FAIL");
	       LCD_DisplayStringLine(Line2, (uint8_t*)"Please Restart board");
	       while(1);
	      }

	      /* Power up MEMS */
	      if (I2C_WriteDeviceRegister(MEMS_I2C_ADDR, MEMS_Ctrl_Reg1, 0x47) == IOE_OK)
	      {
	       LCD_DisplayStringLine(Line2, (uint8_t*)"MEMS powerup      OK");
	      }
	      else
	      {
	       LCD_DisplayStringLine(Line2, (uint8_t*)"MEMS powerup    FAIL");
	       LCD_DisplayStringLine(Line3, (uint8_t*)"Please Restart board");
	       while(1);
	      }

	      revid = DBGMCU_GetREVID();
	      devid = DBGMCU_GetDEVID();
	      sprintf(lcdLine, "Rev %d   Dev %d ", (int)revid, (int)devid);
	      LCD_DisplayStringLine(Line5, (uint8_t*)lcdLine);
	      LCD_DisplayStringLine(Line4, (uint8_t*)"64=1G          +/-2G");

}

uint8_t AccelStatus()
{
	return I2C_ReadDeviceRegister(MEMS_I2C_ADDR, MEMS_Status_Reg);
}

void AccelData()
{
	memsX = (int8_t)(I2C_ReadDeviceRegister(MEMS_I2C_ADDR, MEMS_OutX));
	memsY = (int8_t)I2C_ReadDeviceRegister(MEMS_I2C_ADDR, MEMS_OutY);
	memsZ = (int8_t)I2C_ReadDeviceRegister(MEMS_I2C_ADDR, MEMS_OutZ);
	UpdateMA_X(memsX);
	UpdateMA_Y(memsY);
	UpdateMA_Z(memsZ);
}

int8_t GetMA_X()
{
	int i, sum = 0;
	for(i = 0; i <= cntX; i++)
	{
		sum += lastX[i];
	}
	return (sum / cntX+1)*SENSITIVITY;
}

int8_t GetMA_Y()
{
	int i, sum = 0;
	for(i = 0; i <= cntY; i++)
	{
		sum += lastY[i];
	}
	return (sum / cntY+1)*SENSITIVITY;
}

int8_t GetMA_Z()
{
	int i, sum = 0;
	for(i = 0; i <= cntZ; i++)
	{
		sum += lastZ[i];
	}
	return (sum / cntZ+1)*SENSITIVITY;
}

void UpdateMA_X(int8_t data)
{
	int i;
	for(i = 1; i <= cntX; i++)
	{
		lastX[i-1] = lastX[i];
	}
	if(cntX < 10){ cntX++; }
	lastX[cntX] = data;
}

void UpdateMA_Y(int8_t data)
{
	int i;
	for(i = 1; i <= cntY; i++)
	{
		lastY[i-1] = lastY[i];
	}
	if(cntY < 10){ cntY++; }
	lastY[cntY] = data;
}

void UpdateMA_Z(int8_t data)
{
	int i;
	for(i = 1; i <= cntZ; i++)
	{
		lastZ[i-1] = lastZ[i];
	}
	if(cntZ < 10){ cntZ++; }
	lastZ[cntZ] = data;
}
