/*
  *          variables and functions used to communicate between Touch Sensing
  *          Library and the customer code.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_LIS3DH.h"
//#include "i2c_mems.h"
//#include "stm32_eval.h"
#include <stdio.h>
#include <math.h>
#include <macro.h>

//float powf(float __x,float __y);

int16_t LIS3DH_accx=0,LIS3DH_accy=0,LIS3DH_accz=0;

__IO uint32_t  LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT; 

#define BIT3	(0x1<<3)

void LIS3DH_Delay(uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

int16_t LIS3DH_I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t ack) {
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) && LIS3DH_Timeout) 
	{
		if (--LIS3DH_Timeout == 0x00) 
		{
			return 1;
		}
	}

	if (ack) 
	{
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
	}
	
	I2C_Send7bitAddress(I2Cx, address, direction);

	if (direction == I2C_Direction_Transmitter) 
	{
		LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) && LIS3DH_Timeout) 
		{
			if (--LIS3DH_Timeout == 0x00) 
			{
				return 1;
			}
		}
	} else if (direction == I2C_Direction_Receiver) 
	{
		LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && LIS3DH_Timeout) 
		{
			if (--LIS3DH_Timeout == 0x00) 
			{
				return 1;
			}
		}
	}
	I2Cx->SR2;
	
	return 0;
}

void LIS3DH_I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
	LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && LIS3DH_Timeout) 
	{
		LIS3DH_Timeout--;
	}
	I2C_SendData(I2Cx, data);
}

uint8_t LIS3DH_I2C_ReadAck(I2C_TypeDef* I2Cx) {
	uint8_t data;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && LIS3DH_Timeout) 
	{
		LIS3DH_Timeout--;
	}

	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t LIS3DH_I2C_ReadNack(I2C_TypeDef* I2Cx) {
	uint8_t data;
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && LIS3DH_Timeout) 
	{
		LIS3DH_Timeout--;
	}

	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t LIS3DH_I2C_Stop(I2C_TypeDef* I2Cx) {	
	LIS3DH_Timeout = LIS3DH_LONG_TIMEOUT;
	while (((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))) && LIS3DH_Timeout) 
	{
		if (--LIS3DH_Timeout == 0x00) 
		{
			return 1;
		}
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return 0;
}

uint8_t LIS3DH_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg) {
	uint8_t received_data;
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
	LIS3DH_I2C_WriteData(I2Cx, reg);
	LIS3DH_I2C_Stop(I2Cx);
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Receiver, 0);
	received_data = LIS3DH_I2C_ReadNack(I2Cx);
	return received_data;
	
}

void LIS3DH_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) 
{
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
	LIS3DH_I2C_WriteData(I2Cx, reg);
	LIS3DH_I2C_WriteData(I2Cx, data);
	LIS3DH_I2C_Stop(I2Cx);
	
}

void LIS3DH_I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) 
{
	uint8_t i;
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 1);
	LIS3DH_I2C_WriteData(I2Cx, reg);
	LIS3DH_I2C_Stop(I2Cx);
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Receiver, 1);
	for (i = 0; i < count; i++) 
	{
		if (i == (count - 1)) 
		{
			//Last byte
			data[i] = LIS3DH_I2C_ReadNack(I2Cx);
		} else 
		{
			data[i] = LIS3DH_I2C_ReadAck(I2Cx);
		}
	}
}

void LIS3DH_I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count) 
{
	uint8_t i;
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Receiver, 1);
	for (i = 0; i < count; i++) 
	{
		if (i == (count - 1)) 
		{
			//Last byte
			data[i] = LIS3DH_I2C_ReadNack(I2Cx);
		} else 
		{
			data[i] = LIS3DH_I2C_ReadAck(I2Cx);
		}
	}
}

void LIS3DH_I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) 
{
	uint8_t i;
	LIS3DH_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
	LIS3DH_I2C_WriteData(I2Cx, reg);
	for (i = 0; i < count; i++) 
	{
		LIS3DH_I2C_WriteData(I2Cx, data[i]);
	}
	LIS3DH_I2C_Stop(I2Cx);
	
}

void LIS3DH_Read(uint8_t DeviceAddr, uint8_t RegisterAddr,
                              uint16_t NumByteToRead,
                              uint8_t* pBuffer)
{
	LIS3DH_I2C_ReadMulti(LIS3DH_I2C,DeviceAddr,RegisterAddr,pBuffer,NumByteToRead);
	
}
void LIS3DH_Write(uint8_t DeviceAddr, uint8_t RegisterAddr,
                               uint16_t NumByteToWrite,
                               uint8_t* pBuffer)
{
	LIS3DH_I2C_WriteMulti(LIS3DH_I2C,DeviceAddr,RegisterAddr,pBuffer,NumByteToWrite);
	
}

void SetupLIS3DH(void)
{
	uint8_t data;

	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_WHO_AM_I, 1,&data);
//	printf("LIS3DH_WHO_AM_I_ADDR = 0x%X\r\n",data);
	LIS3DH_Delay(5);

	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_CTRL_REG1, 1,&data);
	LIS3DH_Delay(5);	
	
	data = 0x47;	
	LIS3DH_Write(LIS3DH_ADDRESS, LIS3DH_CTRL_REG1, 1,&data);// ODR : 50Hz
	LIS3DH_Delay(5);
}	

uint16_t SelfTestLIS3DH(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t ax_s,ay_s,az_s;
//	uint16_t i=0;
	uint8_t u8WaitCnt=0;

	do{
		LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_STATUS_REG2, 1,&tmp);	
		if (u8WaitCnt++>30)
			break;		
	}while(!(tmp&BIT3));

	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_OUT_X_H, 1, &tmpxh);
	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_OUT_X_L, 1, &tmpxl);
	ax_s=(((int16_t) ((tmpxh << 8) | tmpxl)) >> 4);

	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_OUT_Y_H, 1, &tmpyh);
	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_OUT_Y_L, 1, &tmpyl);
	ay_s=(((int16_t) ((tmpyh << 8) | tmpyl)) >> 4);

	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_OUT_Z_H, 1, &tmpzh);
	LIS3DH_Read(LIS3DH_ADDRESS, LIS3DH_OUT_Z_L, 1, &tmpzl);
	az_s=(((int16_t) ((tmpzh << 8) | tmpzl)) >> 4);

	#if 0	//debug
	printf("Accelerometer(mg):     X-axis : %5d, ",(int16_t)ax_s);
	printf(" Y-axis : %5d,  ",(int16_t)ay_s);
	printf(" Z-axis :  %5d \n\r",(int16_t)az_s);
	#endif

	LIS3DH_accx = ax_s ;
	LIS3DH_accy = ay_s ;
	LIS3DH_accz = az_s ;
	
//	LIS3DH_Delay(0xFFFF);
	LIS3DH_Delay(5);
	return 0;
}

/*
	SDO_N6 , CS_N6 = HIGH

	VDD , VDD_IO = HIGH
*/

void LIS3DH_I2C_Config(void)
{ 
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(LIS3DH_I2C_SCL_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LIS3DH_I2C_SDA_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(LIS3DH_I2C_SCL_GPIO_PORT, LIS3DH_I2C_SCL_SOURCE, LIS3DH_I2C_SCL_AF);
	GPIO_PinAFConfig(LIS3DH_I2C_SDA_GPIO_PORT, LIS3DH_I2C_SDA_SOURCE, LIS3DH_I2C_SDA_AF);

	/* Configure SCL */
	GPIO_InitStructure.GPIO_Pin = LIS3DH_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LIS3DH_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* Configure SDA */
	GPIO_InitStructure.GPIO_Pin = LIS3DH_I2C_SDA_PIN;
	GPIO_Init(LIS3DH_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* LIS3DH_I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(LIS3DH_I2C_CLK, ENABLE);

	/* Configure LIS3DH_I2C for communication */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x33;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = LIS3DH_I2C_SPEED;

	I2C_DeInit(LIS3DH_I2C);
	I2C_Cmd(LIS3DH_I2C, ENABLE);
	I2C_Init(LIS3DH_I2C, &I2C_InitStructure);
  
}

void LIS3DH_I2C_DeConfig(void)
{
	/* sEE_I2C Peripheral Disable */
	I2C_Cmd(LIS3DH_I2C,DISABLE);
	/* sEE_I2C DeInit */
	I2C_DeInit(LIS3DH_I2C);
	/* sEE_I2C Periph clock disable */
	RCC_APB1PeriphClockCmd(LIS3DH_I2C_CLK, DISABLE);
	
}	

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS3DH_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
	LIS3DH_I2C_DeConfig();
	LIS3DH_I2C_Config();
	return LIS3DH_FAIL;
	
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

