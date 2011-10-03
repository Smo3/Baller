/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : stm32_mems_adapter.c
* Author             : Industrial & Multi-Market Competence Center Europe - Connectivity & Sensors Team
* Version            : V1.0.1
* Date               : 07/20/2009
* Description        : Provides functions to work with STM32-MEMS
*                      demonstration board.
********************************************************************************
* History:
* 07/20/2009: V1.0.1 I2C speed increased to 400kHz 
* 05/26/2009: V1.0.0 Initial revision
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "stm32_mems.h"
#include "stm32_mems_adapter.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_exti.h"

#ifndef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library not used */
#include "stm32f10x_nvic.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Max number of I2C flag checks */
#define I2C_MAX_CHECK_CNT 10000
/* Max number of SPI flag checks */
#define SPI_MAX_CHECK_CNT 10000
/* Private variables ---------------------------------------------------------*/

DMA_InitTypeDef DMA_InitStructure;
s16 ADC_DataValue[3];
u8 MEMS_I2C_DEVICE_ADDRESS=0;

/* Private function prototypes -----------------------------------------------*/

void MEMS_ANL_ADC_Stop(void);
void MEMS_ANL_ADC_Start(void);

ErrorStatus I2C_WaitEvent (I2C_TypeDef* I2Cx, u32 I2C_Event);

ErrorStatus SPI_I2S_WaitFlag (SPI_TypeDef* SPIx, u16 SPI_I2S_FLA,
                              FlagStatus SPI_I2S_FLAValue);

/* Private functions ---------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------              Analog               ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/


/*******************************************************************************
* Function Name  : MEMS_ANL_Setup
* Description    : Setups all peripherals related to analog MEMS.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_Setup (void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef   ADC_InitStructure;

  /* Enable clocks -----------------------------------------------------------*/
  /* Enable ADC and GPIO clocks */
  RCC_APB2PeriphClockCmd(MEMS_ANL_RCC_APB2Periph_ADC |
                         MEMS_ANL_RCC_APB2Periph_GPIOs, ENABLE);

  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(MEMS_ANL_RCC_AHBPeriph_DMA, ENABLE);

  /* Configure GPIOs ---------------------------------------------------------*/
  /* Configure ADC_IN pins */
  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_X;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(MEMS_ANL_GPIO_X, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_Y;
  GPIO_Init(MEMS_ANL_GPIO_Y, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_Z;
  GPIO_Init(MEMS_ANL_GPIO_Z, &GPIO_InitStructure);

  /* Configure FS for analog MEMS control */
  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_FS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(MEMS_ANL_GPIO_FS, &GPIO_InitStructure);

  /* Configure PD for analog MEMS control */
  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_PD;
  GPIO_Init(MEMS_ANL_GPIO_PD, &GPIO_InitStructure);

  /* Set full scale to 0 (2g) */
  MEMS_ANL_Drive_FS (Bit_RESET);
  /* Set power down to 0 (MEMS in active state) */
  MEMS_ANL_Drive_PD (Bit_RESET);

  /* ADC configuration -------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(MEMS_ANL_ADC, &ADC_InitStructure);

  /* ADC regular channel configuration */
  ADC_RegularChannelConfig(MEMS_ANL_ADC, MEMS_ANL_ADC_Channel_X,  1,
                           ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(MEMS_ANL_ADC, MEMS_ANL_ADC_Channel_Y,  2,
                           ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(MEMS_ANL_ADC, MEMS_ANL_ADC_Channel_Z,  3,
                           ADC_SampleTime_239Cycles5);

  /* Enable ADC DMA */
  ADC_DMACmd(MEMS_ANL_ADC, ENABLE);
  /* Enable ADC */
  ADC_Cmd(MEMS_ANL_ADC, ENABLE);

  /* Enable ADC reset calibaration register */
  ADC_ResetCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC reset calibration register */
  while(ADC_GetResetCalibrationStatus(MEMS_ANL_ADC));

  /* Start ADC calibaration */
  ADC_StartCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC calibration */
  while(ADC_GetCalibrationStatus(MEMS_ANL_ADC));

 /* DMA channel configuration ------------------------------------------------*/
  DMA_DeInit(MEMS_ANL_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = MEMS_ANL_ADC_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_DataValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(MEMS_ANL_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA channel */
  DMA_Cmd(MEMS_ANL_DMA_Channel, ENABLE);

  /* Enable interrupt on buffer half- and full */
  DMA_ITConfig(MEMS_ANL_DMA_Channel, DMA_IT_TC, ENABLE);

}

/*******************************************************************************
* Function Name  : MEMS_ANL_Drive_FS
* Description    : Drives FS pin of analog MEMS.
* Input          : BitVal - new value of the FS pin
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_Drive_FS (BitAction BitVal) {
  GPIO_WriteBit(MEMS_ANL_GPIO_FS, MEMS_ANL_GPIO_Pin_FS, BitVal);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_Drive_PD
* Description    : Drives PD pin of analog MEMS.
* Input          : BitVal - new value of the PD pin
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_Drive_PD (BitAction BitVal) {
  GPIO_WriteBit(MEMS_ANL_GPIO_PD, MEMS_ANL_GPIO_Pin_PD, BitVal);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_ADC_Stop
* Description    : Stops ADC and DMA.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_ADC_Stop(void)
{
  ADC_Cmd(MEMS_ANL_ADC, DISABLE);
  DMA_Cmd(MEMS_ANL_DMA_Channel, DISABLE);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_ADC_Start
* Description    : Starts ADC and DMA.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_ADC_Start(void)
{
  DMA_Init(MEMS_ANL_DMA_Channel, &DMA_InitStructure);
  DMA_Cmd(MEMS_ANL_DMA_Channel, ENABLE);

  /* enable ADC again */
  ADC_Cmd(MEMS_ANL_ADC, ENABLE);

  /* Enable ADC reset calibaration register */
  ADC_ResetCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC reset calibration register */
  while(ADC_GetResetCalibrationStatus(MEMS_ANL_ADC));

  /* Start ADC calibaration */
  ADC_StartCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC calibration */
  while(ADC_GetCalibrationStatus(MEMS_ANL_ADC));

  /* Start ADC Software Conversion */
  ADC_SoftwareStartConvCmd(MEMS_ANL_ADC, ENABLE);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_ADC_Restart
* Description    : Restarts ADC and DMA.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_ADC_Restart(void)
{
  MEMS_ANL_ADC_Stop();
  MEMS_ANL_ADC_Start();
}

/*******************************************************************************
* Function Name  : MEMS_ANL_Get_Axis
* Description    : Gets values of all MEMS axis.
* Input          : none
* Output         : x - Value of X axe
*                  y - Value of Y axe
*                  z - Value of Z axe
* Return         : none
*******************************************************************************/
void MEMS_ANL_Get_Axis(s16 *x, s16 *y, s16 *z)
{
  *x = ADC_DataValue[0];
  *y = ADC_DataValue[1];
  *z = ADC_DataValue[2];
}



/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------              Digital              ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : MEMS_DIG_Setup_Int1
* Description    : Enables or disables EXTI for the Int1 interrupt signal.
* Input          : NewState: New state of the interrupt. This parameter
*                  can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void MEMS_DIG_Setup_Int1(FunctionalState NewState)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Initialize the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int1;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line */
  else
  {
    /* Enable the External Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = MEMS_DIG_EXTI_IRQn_Int1;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure Int1 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_Int1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MEMS_DIG_GPIO_Int1, &GPIO_InitStructure);

    /* Configure Int1 pin for external interrupt */
    GPIO_EXTILineConfig(MEMS_DIG_GPIO_PortSourceGPIO_Int1, MEMS_DIG_GPIO_PinSource_Int1);

    /* Clear the the EXTI line  */
    EXTI_ClearITPendingBit(MEMS_DIG_EXTI_Line_Int1);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int1;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}

/*******************************************************************************
* Function Name  : MEMS_DIG_Setup_Int2
* Description    : Enables or disables EXTI for the Int2 interrupt signal.
* Input          : NewState: New state of the interrupt. This parameter
*                  can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void MEMS_DIG_Setup_Int2(FunctionalState NewState)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Initialize the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int2;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line */
  else
  {
    /* Enable the External Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = MEMS_DIG_EXTI_IRQn_Int2;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure Int2 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_Int2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MEMS_DIG_GPIO_Int2, &GPIO_InitStructure);

    /* Configure Int2 pin for external interrupt */
    GPIO_EXTILineConfig(MEMS_DIG_GPIO_PortSourceGPIO_Int2, MEMS_DIG_GPIO_PinSource_Int2);

    /* Clear the the EXTI line  */
    EXTI_ClearITPendingBit(MEMS_DIG_EXTI_Line_Int2);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int2;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------               SPI                 ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : MEMS_SPI_Setup
* Description    : Setups all peripherals related to digital MEMS connected over SPI.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_SPI_Setup (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable clocks -----------------------------------------------------------*/
  /* Enable SPI and GPIO clocks */

  RCC_APB2PeriphClockCmd(MEMS_DIG_RCC_APB2Periph_GPIOs |
                         MEMS_SPI_RCC_APB2Periph_GPIOs, ENABLE);

#ifdef STM3210B_EVAL
  RCC_APB2PeriphClockCmd(MEMS_SPI_RCC_APB2Periph_SPI, ENABLE);
#endif

#ifdef STM3210E_EVAL
  RCC_APB1PeriphClockCmd(MEMS_SPI_RCC_APB1Periph_SPI, ENABLE);

  /* Disable JTAG pins PBx */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif

#ifdef STM3210B_SK_IAR
  RCC_APB1PeriphClockCmd(MEMS_SPI_RCC_APB1Periph_SPI, ENABLE);
#endif


  /* Configure GPIOs    ------------------------------------------------------*/
  /* Configure CS pin */
  GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_CS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(MEMS_DIG_GPIO_CS, &GPIO_InitStructure);
  /* Drive CS pin high */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  /* Configure SPI pins */
  GPIO_InitStructure.GPIO_Pin = MEMS_SPI_GPIO_Pin_SCK | MEMS_SPI_GPIO_Pin_SDI |
                                MEMS_SPI_GPIO_Pin_SD0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(MEMS_SPI_GPIO, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; /* No hardware CS generation */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(MEMS_SPI, &SPI_InitStructure);

  /* Enable SPI */
  SPI_Cmd(MEMS_SPI, ENABLE);

}

/*******************************************************************************
* Function Name  : SPI_I2S_WaitFlag
* Description    : Waits till the SPI flag is set to the defined value.
* Input          : - SPIx: where x can be 1 or 2 to select the SPI peripheral.
*                  - SPI_I2S_FLA: specifies the flag to be checked.
*                  - SPI_I2S_FLAValue: specifies the flag status to be checked.
* Output         : None
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Flag was set correctly
*                       - ERROR: Flag was not not set correctly
*******************************************************************************/
ErrorStatus SPI_I2S_WaitFlag (SPI_TypeDef* SPIx, u16 SPI_I2S_FLA,
                              FlagStatus SPI_I2S_FLAValue)
{
  u16 i;

  for (i=0; i < SPI_MAX_CHECK_CNT; i++)
    if (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLA) == SPI_I2S_FLAValue)
      return SUCCESS;

  return ERROR;
}

/*******************************************************************************
* Function Name  : MEMS_SPI_SendFrame
* Description    : Sends one frame over SPI.
* Input          : - RegAddress: Address of register.
*                  - pBuffer: Pointer to buffer with data.
*                  - NoOfBytes: Number of bytes to be sent.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame was sent
*                       - ERROR: Frame was not sent
*******************************************************************************/
ErrorStatus MEMS_SPI_SendFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u8 i;
  u8 addr = RegAddress + MEMS_SPI_WRITE;

  if (NoOfBytes > 1)
    addr += MEMS_SPI_MULTIPLE_BYTES;

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_RESET);

  /* Send address byte */
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;

  SPI_I2S_SendData(MEMS_SPI, addr);
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
  SPI_I2S_ReceiveData(MEMS_SPI);

  /* Send data bytes */
  for(i=0; i < NoOfBytes; i++) {
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;
    SPI_I2S_SendData(MEMS_SPI, *(pBuffer+i));
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
    /* Empty receive FIFO */
    SPI_I2S_ReceiveData(MEMS_SPI);
  }

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_SPI_ReceiveFrame
* Description    : Receives one frame over SPI.
* Input          : - RegAddress: Address of source register.
*                  - NoOfBytes: Number of bytes to be received.
* Output         : - pBuffer: Pointer to output buffer.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame was received
*                       - ERROR: Frame was not received
*******************************************************************************/
ErrorStatus MEMS_SPI_ReceiveFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u8 i;
  u16 addr = RegAddress + MEMS_SPI_READ;
  u16 dummy = 0xAA58;

  if (NoOfBytes > 1)
    addr += MEMS_SPI_MULTIPLE_BYTES;

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_RESET);

  /* Send Address byte */
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;
  SPI_I2S_SendData(MEMS_SPI, addr);
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
  SPI_I2S_ReceiveData(MEMS_SPI); /* skip byte received with the address */

  /* Receive Data byte(s) */
  for(i=0; i < NoOfBytes; i++) {
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;
    SPI_I2S_SendData(MEMS_SPI, dummy); /* send dummy byte to generate clock */
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
    *(pBuffer+i) = SPI_I2S_ReceiveData(MEMS_SPI);
  }

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_SPI_WriteReg
* Description    : Writes data to register of MEMS over SPI.
* Input          : - RegAddress: Address of register.
*                  - Data: Data to be written.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Register was written
*                       - ERROR: Register was not written
*******************************************************************************/
ErrorStatus MEMS_SPI_WriteReg (u8 RegAddress, u8 Data)
{
  return MEMS_SPI_SendFrame (RegAddress, &Data, 1);
}

/*******************************************************************************
* Function Name  : MEMS_SPI_ReadReg
* Description    : Reads data to register of MEMS over SPI.
* Input          : - RegAddress: Address of register.
* Output         : - Data: Data read.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Register was read
*                       - ERROR: Register was not read
*******************************************************************************/
ErrorStatus MEMS_SPI_ReadReg (u8 RegAddress, u8 *Data)
{
  return MEMS_SPI_ReceiveFrame (RegAddress, Data, 1);
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------               I2C                 ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : MEMS_I2C_Setup
* Description    : Setups all peripherals related to digital MEMS connected over I2C.
* Input          : MEMS_I2C_Address - I2C address of MEMS.
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_I2C_Setup (u8 MEMS_I2C_Address)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;

  /* Store I2C address of the MEMS sensor */
  MEMS_I2C_Set_Address (MEMS_I2C_Address);

  /* Enable clocks -----------------------------------------------------------*/
  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(MEMS_I2C_RCC_APB1Periph_I2C, ENABLE);

  RCC_APB2PeriphClockCmd(MEMS_DIG_RCC_APB2Periph_GPIOs |
                         MEMS_SPI_RCC_APB2Periph_GPIOs |
                         MEMS_I2C_RCC_APB2Periph_GPIOs, ENABLE);

#ifdef STM3210E_EVAL
  /* Remap pins used by MEMS */
  /* I2C1_SCL on PB.08, I2C1_SDA on PB.09 */
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

  /* Disable JTAG pins PBx */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif

  /* Configure GPIOs    ------------------------------------------------------*/
  /* Configure I2C  pins     -------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = MEMS_I2C_GPIO_Pin_SCL | MEMS_I2C_GPIO_Pin_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(MEMS_I2C_GPIO, &GPIO_InitStructure);

  /* Configure CS and SDO for I2C MEMS control*/
  /* Configure CS pin */
  GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_CS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(MEMS_DIG_GPIO_CS, &GPIO_InitStructure);
  /* Drive CS pin high */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  /* Configure SDO pin */
  GPIO_InitStructure.GPIO_Pin = MEMS_SPI_GPIO_Pin_SD0;
  GPIO_Init(MEMS_SPI_GPIO, &GPIO_InitStructure);
  /* Drive SDO (LSB of I2C address) pin low */
  GPIO_WriteBit(MEMS_SPI_GPIO, MEMS_SPI_GPIO_Pin_SD0, Bit_RESET);

  /* I2C configuration -------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x0A;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 400000;
  I2C_Init(MEMS_I2C, &I2C_InitStructure);

  /* Enable I2C */
  I2C_Cmd (MEMS_I2C, ENABLE);

}

/*******************************************************************************
* Function Name  : MEMS_I2C_Set_address
* Description    : Sets address of MEMS for I2C communication.
* Input          : MEMS_I2C_Address - I2C address of MEMS.
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_I2C_Set_Address (u8 MEMS_I2C_Address)
{
    MEMS_I2C_DEVICE_ADDRESS = MEMS_I2C_Address;
}

/*******************************************************************************
* Function Name  : I2C_WaitEvent
* Description    : Waits till the last I2Cx Event is equal to the one passed
*                  as parameter.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_EVENT: specifies the event to be checked.
* Output         : None
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Last event is equal to the I2C_EVENT
*                       - ERROR: Last event is different from the I2C_EVENT
*******************************************************************************/
ErrorStatus I2C_WaitEvent (I2C_TypeDef* I2Cx, u32 I2C_Event)
{
  u16 i;

  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
    if (I2C_CheckEvent(I2Cx, I2C_Event) == SUCCESS)
        return SUCCESS;

  return ERROR;
}

/*******************************************************************************
* Function Name  : MEMS_I2C_SendFrame
* Description    : Sends one frame over I2C.
* Input          : - RegAddress: Address of register.
*                  - pBuffer: Pointer to buffer with data.
*                  - NoOfBytes: Number of bytes to be sent.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame sent.
*                       - ERROR: Frame was not sent.
*******************************************************************************/
ErrorStatus MEMS_I2C_SendFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u16 i;

  /* Send START condition */
  I2C_GenerateSTART(MEMS_I2C, ENABLE);
  /* Test on EV5 and clear it */
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) return ERROR;
  /* Send slave address for write */
  I2C_Send7bitAddress(MEMS_I2C,MEMS_I2C_DEVICE_ADDRESS, I2C_Direction_Transmitter);
  /* Test on EV6 and clear it */
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    return ERROR;
  I2C_SendData (MEMS_I2C, RegAddress);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) return ERROR;
  /* While there is data to be written */
  while (NoOfBytes-- > 0)
  {
    /* Send the current byte */
    I2C_SendData (MEMS_I2C, *pBuffer);
    if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) return ERROR;
    /* Point to the next byte to be written */
    pBuffer++;
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(MEMS_I2C, ENABLE);
  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
    if ((MEMS_I2C->CR1&0x200) != 0x200) break;
  if ((MEMS_I2C->CR1&0x200) == 0x200) return ERROR;

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_I2C_ReceiveFrame
* Description    : Receives one frame over I2C.
* Input          : - RegAddress: Address of source register.
*                  - NoOfBytes: Number of bytes to be received.
* Output         : - pBuffer: Pointer to output buffer.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame transmitted.
*                       - ERROR: Frame was not transmitted.
*******************************************************************************/
ErrorStatus MEMS_I2C_ReceiveFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u16 i;
  u8 addr = RegAddress;

  if (NoOfBytes > 1) {
    I2C_AcknowledgeConfig (MEMS_I2C, ENABLE);
    addr += MEMS_I2C_REPETIR;
  } else
    I2C_AcknowledgeConfig (MEMS_I2C, DISABLE);

  /* StartBit */
  I2C_GenerateSTART(MEMS_I2C, ENABLE);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) return ERROR;

  /* Send Address of device & TRANSMITTER mode */
  I2C_Send7bitAddress(MEMS_I2C, MEMS_I2C_DEVICE_ADDRESS, I2C_Direction_Transmitter);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    return ERROR;

  /* Send Register address */
  I2C_SendData (MEMS_I2C, addr);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) return ERROR;

  /* Repeat StartBit */
  I2C_GenerateSTART(MEMS_I2C, ENABLE);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) return ERROR;

  /* Send Address of device & RECEIVER mode */
  I2C_Send7bitAddress(MEMS_I2C, MEMS_I2C_DEVICE_ADDRESS, I2C_Direction_Receiver);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    return ERROR;

  /* Receive bytes */
  for (i=0;i<NoOfBytes;i++)
  {
    if (i==(NoOfBytes-2))
      I2C_AcknowledgeConfig (MEMS_I2C, DISABLE);

    if (i==(NoOfBytes-1)) /* StopBit */
      I2C_GenerateSTOP (MEMS_I2C, ENABLE);

    if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS) return ERROR;

    *(pBuffer+i) = I2C_ReceiveData(MEMS_I2C);
  }

  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
    if ((MEMS_I2C->CR1&0x200) != 0x200) break;
  if ((MEMS_I2C->CR1&0x200) == 0x200) return ERROR;

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_I2C_WriteReg
* Description    : Writes data to register of MEMS over I2C.
* Input          : - RegAddress: Address of register.
*                  - Data: Data to be written.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Data was written.
*                       - ERROR: Data was not written.
*******************************************************************************/
ErrorStatus MEMS_I2C_WriteReg (u8 RegAddress, u8 Data)
{
  return MEMS_I2C_SendFrame (RegAddress, &Data, 1);
}

/*******************************************************************************
* Function Name  : MEMS_I2C_ReadReg
* Description    : Reads data from register of MEMS over I2C.
* Input          : - RegAddress: Address of register.
* Output         : - Data: Data read.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Data was read.
*                       - ERROR: Data was not read.
*******************************************************************************/
ErrorStatus MEMS_I2C_ReadReg (u8 RegAddress, u8 *Data)
{
  return MEMS_I2C_ReceiveFrame (RegAddress, Data, 1);
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
