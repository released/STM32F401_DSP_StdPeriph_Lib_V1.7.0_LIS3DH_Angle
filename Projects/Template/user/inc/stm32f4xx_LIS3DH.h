/**
  ******************************************************************************
  * @file    stm32_tsl_api.h
  * @author  MCD Application Team
  * @version V0.1.5
  * @date    18-November-2011
  * @brief   STM32 Touch Sensing Library - This file contains all functions
  *          prototype and macros of the API.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS3DH_API_H
#define __LIS3DH_API_H


#define LIS3DH_ADDRESS                     0x33

extern int16_t LIS3DH_accx,LIS3DH_accy,LIS3DH_accz;

#define LIS3DH_STATUS_REG_AUX 			0x07
#define LIS3DH_OUT_ADC1_L 					0x08
#define LIS3DH_OUT_ADC1_H 					0x09
#define LIS3DH_OUT_ADC2_L 					0x0A
#define LIS3DH_OUT_ADC2_H 					0x0B
#define LIS3DH_OUT_ADC3_L 					0x0C
#define LIS3DH_OUT_ADC3_H 					0x0D
#define LIS3DH_INT_COUNTER_REG 			0x0E
#define LIS3DH_WHO_AM_I 					0x0F
#define LIS3DH_TEMP_CFG_REG 				0x1F
#define LIS3DH_CTRL_REG1 					0x20
#define LIS3DH_CTRL_REG2 					0x21
#define LIS3DH_CTRL_REG3 					0x22
#define LIS3DH_CTRL_REG4 					0x23
#define LIS3DH_CTRL_REG5 					0x24
#define LIS3DH_CTRL_REG6 					0x25
#define LIS3DH_REFERENCE 					0x26
#define LIS3DH_STATUS_REG2 				0x27
#define LIS3DH_OUT_X_L 						0x28
#define LIS3DH_OUT_X_H 					0x29
#define LIS3DH_OUT_Y_L 						0x2A
#define LIS3DH_OUT_Y_H 					0x2B
#define LIS3DH_OUT_Z_L 						0x2C
#define LIS3DH_OUT_Z_H 					0x2D
#define LIS3DH_FIFO_CTRL_REG 				0x2E
#define LIS3DH_FIFO_SRC_REG 				0x2F
#define LIS3DH_INT1_CFG 					0x30
#define LIS3DH_INT1_SOURCE 				0x31
#define LIS3DH_LIS3DH_INT1_THS 				0x32
#define LIS3DH_INT1_DURATION 				0x33
#define LIS3DH_CLICK_CFG 					0x38
#define LIS3DH_CLICK_SRC 					0x39
#define LIS3DH_CLICK_THS 					0x3A
#define LIS3DH_TIME_LIMIT 					0x3B
#define LIS3DH_TIME_LATENCY 				0x3C
#define LIS3DH_TIME_WINDOW 				0x3D

typedef enum
{
  LIS3DH_DMA_TX = 0,
  LIS3DH_DMA_RX = 1
}LIS3DH_DMADirection_TypeDef;
typedef enum
{
  LIS3DH_OK = 0,
  LIS3DH_FAIL
}LIS3DH_Status_TypDef;


#define LIS3DH_FLAG_TIMEOUT         		((uint32_t)0x1000)
#define LIS3DH_LONG_TIMEOUT         		((uint32_t)(10 * LIS3DH_FLAG_TIMEOUT))

/**
  * @brief  Block Size
  */
#define LIS3DH_REG_TEMP       				0x00  /*!< Temperature Register of LIS3DH */
#define LIS3DH_REG_CONF       				0x01  /*!< Configuration Register of LIS3DH */
#define LIS3DH_REG_THYS       				0x02  /*!< Temperature Register of LIS3DH */
#define LIS3DH_REG_TOS        				0x03  /*!< Over-temp Shutdown threshold Register of LIS3DH */
#define I2C_TIMEOUT         				((uint32_t)0x3FFFF) /*!< I2C Time out */
#define LIS3DH_I2C_SPEED      				100000 /*!< I2C Speed */
  

  /* @brief  LIS3DH Temperature Sensor I2C Interface pins
  */  
#if 0
#define LIS3DH_I2C                         	I2C3
#define LIS3DH_I2C_CLK                     	RCC_APB1Periph_I2C3

#define LIS3DH_I2C_SCL_PIN                 	GPIO_Pin_8					/* PA.8 */
#define LIS3DH_I2C_SCL_GPIO_PORT           GPIOA                       /* GPIOA */
#define LIS3DH_I2C_SCL_GPIO_CLK            	RCC_AHB1Periph_GPIOA
#define LIS3DH_I2C_SCL_SOURCE             	GPIO_PinSource8
#define LIS3DH_I2C_SCL_AF                  	GPIO_AF_I2C3

#define LIS3DH_I2C_SDA_PIN                 	GPIO_Pin_9                  /* PC.9*/
#define LIS3DH_I2C_SDA_GPIO_PORT          	GPIOC                       /* GPIOC */
#define LIS3DH_I2C_SDA_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define LIS3DH_I2C_SDA_SOURCE             	GPIO_PinSource9
#define LIS3DH_I2C_SDA_AF                  	GPIO_AF_I2C3
#else
#define LIS3DH_I2C                         	I2C1
#define LIS3DH_I2C_CLK                     	RCC_APB1Periph_I2C1
#define LIS3DH_I2C_SCL_PIN                 	GPIO_Pin_8					/* PB.8 */
#define LIS3DH_I2C_SCL_GPIO_PORT           	GPIOB                       /* GPIOB */
#define LIS3DH_I2C_SCL_GPIO_CLK            	RCC_AHB1Periph_GPIOB
#define LIS3DH_I2C_SCL_SOURCE              	GPIO_PinSource8
#define LIS3DH_I2C_SCL_AF                  	GPIO_AF_I2C1
#define LIS3DH_I2C_SDA_PIN                 	GPIO_Pin_9                  /* PB.7*/
#define LIS3DH_I2C_SDA_GPIO_PORT          	GPIOB                       /* GPIOB */
#define LIS3DH_I2C_SDA_GPIO_CLK            	RCC_AHB1Periph_GPIOB
#define LIS3DH_I2C_SDA_SOURCE             	GPIO_PinSource9
#define LIS3DH_I2C_SDA_AF                  	GPIO_AF_I2C1

//#define LIS3DH_I2C_SMBUSALERT_PIN          GPIO_Pin_5                  /* PB.05 */
//#define LIS3DH_I2C_SMBUSALERT_GPIO_PORT    GPIOB                       /* GPIOB */
//#define LIS3DH_I2C_SMBUSALERT_GPIO_CLK     RCC_AHBPeriph_GPIOB
//#define LIS3DH_I2C_SMBUSALERT_SOURCE       GPIO_PinSource5
//#define LIS3DH_I2C_SMBUSALERT_AF           GPIO_AF_I2C2
#endif

#define LIS3DH_I2C_DR                      	((uint32_t)LIS3DH_I2C->DR)//((uint32_t)I2C1->DR)//((uint32_t)0x40005410)

//#define LIS3DH_DMA_CLK                     RCC_AHBPeriph_DMA1
//#define LIS3DH_DMA_TX_CHANNEL              DMA1_Channel6
//#define LIS3DH_DMA_RX_CHANNEL              DMA1_Channel7
//#define LIS3DH_DMA_TX_TCFLAG               DMA1_FLAG_TC6
//#define LIS3DH_DMA_RX_TCFLAG               DMA1_FLAG_TC7

//#define LIS3DH_TIMEOUT_UserCallback()  		LIS3DH_FAIL


/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C POS mask */
#define CR1_POS_Set             ((uint16_t)0x0800)
#define CR1_POS_Reset           ((uint16_t)0xF7FF)

/* Includes ------------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SetupLIS3DH(void);
uint16_t SelfTestLIS3DH(void);
void LIS3DH_I2C_Config(void);
void LIS3DH_I2C_DeConfig(void);

#endif /* __LIS3DH_API_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

