/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : stm32_mems.h
* Author             : Industrial & Multi-Market Competence Center Europe - Connectivity & Sensors Team
* Version            : V1.0.1
* Date               : 07/20/2009
* Description        : Provides defines to work with digital MEMS.
********************************************************************************
* History:
* 07/20/2009: V1.0.1 
* 05/26/2009: V1.0.0 Initial revision
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_MEMS_H
#define __STM32_MEMS_H

/* Includes ------------------------------------------------------------------*/

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#include "stm32f10x.h"
#else
#include "stm32f10x_type.h"
#endif

/* Exported types ------------------------------------------------------------*/

/* MEMS Data Structure */
typedef struct {
  u8 outx_l;
  u8 outx_h;
  u8 outy_l;
  u8 outy_h;
  u8 outz_l;
  u8 outz_h;
} t_mems_data;

/* Exported constants --------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-----------------------           LIS302DL           -----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* Address of MEMS (I2C slave) */
#define LIS302DL_I2C_ADDR         0x38

/* Register addresses */
#define LIS302DL_WHO_AM_I         0x0F
#define LIS302DL_CTRL_REG1        0x20
#define LIS302DL_CTRL_REG2        0x21
#define LIS302DL_CTRL_REG3        0x22
#define LIS302DL_HP_FILTER_RESET  0x23
#define LIS302DL_STATUS_REG       0x27
#define LIS302DL_OUTX             0x29
#define LIS302DL_OUTY             0x2B
#define LIS302DL_OUTZ             0x2D
#define LIS302DL_FF_WU_CFG1       0x30
#define LIS302DL_FF_WU_SRC1       0x31
#define LIS302DL_FF_WU_THS1       0x32
#define LIS302DL_FF_WU_DURATION1  0x33
#define LIS302DL_FF_WU_CFG2       0x34
#define LIS302DL_FF_WU_SRC2       0x35
#define LIS302DL_FF_WU_THS2       0x36
#define LIS302DL_FF_WU_DURATION2  0x37
#define LIS302DL_CLICK_CFG        0x38
#define LIS302DL_CLICK_SRC        0x39
#define LIS302DL_CLICK_THSY_X     0x3B
#define LIS302DL_CLICK_THSZ       0x3C
#define LIS302DL_CLICK_TimeLimit  0x3D
#define LIS302DL_CLICK_Latency    0x3E
#define LIS302DL_CLICK_Window     0x3F


#define LIS302DL_WHO_AM_I_VALUE   0x3B


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-----------------------           LIS3LV02DL           ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* Address of MEMS (I2C slave) */
#define LIS3LV02DL_I2C_ADDR         0x3A

/* Register addresses */
#define LIS3LV02DL_WHO_AM_I         0x0F
#define LIS3LV02DL_OFFSET_X         0x16
#define LIS3LV02DL_OFFSET_Y         0x17
#define LIS3LV02DL_OFFSET_Z         0x18
#define LIS3LV02DL_GAIN_X           0x19
#define LIS3LV02DL_GAIN_Y           0x1A
#define LIS3LV02DL_GAIN_Z           0x1B
#define LIS3LV02DL_CTRL_REG1        0x20
#define LIS3LV02DL_CTRL_REG2        0x21
#define LIS3LV02DL_CTRL_REG3        0x22
#define LIS3LV02DL_HP_FILTER_RESET  0x23
#define LIS3LV02DL_STATUS_REG       0x27
#define LIS3LV02DL_OUTX_L           0x28
#define LIS3LV02DL_OUTX_H           0x29
#define LIS3LV02DL_OUTY_L           0x2A
#define LIS3LV02DL_OUTY_H           0x2B
#define LIS3LV02DL_OUTZ_L           0x2C
#define LIS3LV02DL_OUTZ_H           0x2D
#define LIS3LV02DL_FF_WU_CFG        0x30
#define LIS3LV02DL_FF_WU_SRC        0x31
#define LIS3LV02DL_FF_WU_ACK        0x32
#define LIS3LV02DL_FF_WU_THS_L      0x34
#define LIS3LV02DL_FF_WU_THS_H      0x35
#define LIS3LV02DL_FF_WU_DURATION   0x36
#define LIS3LV02DL_DD_CFG           0x38
#define LIS3LV02DL_DD_SRC           0x39
#define LIS3LV02DL_DD_ACK           0x3A
#define LIS3LV02DL_DD_THSI_L        0x3C
#define LIS3LV02DL_DD_THSI_H        0x3D
#define LIS3LV02DL_DD_THSE_L        0x3E
#define LIS3LV02DL_DD_THSE_H        0x3F

#define LIS3LV02DL_WHO_AM_I_VALUE   0x3A

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/*----------------------------------------------------------------------------*/

#endif /* __STM32_MEMS_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
