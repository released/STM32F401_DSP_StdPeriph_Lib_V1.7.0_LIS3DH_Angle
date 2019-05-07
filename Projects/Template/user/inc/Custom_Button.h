/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_BUTTON_H
#define __CUSTOM_BUTTON_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>

/* Define config -------------------------------------------------------------*/
#define ButtonDebounceDelay 		(3000)//(90)

//#define STM32F401_DISCOVERY
#define STM32F401_NUCLEO

#if defined (STM32F401_DISCOVERY)	//STM32F401 discovery 
#define BTN4_GPIO_PORT          	GPIOA
#define BTN4_GPIO_CLK         		RCC_AHB1Periph_GPIOA
#define BTN4_PIN                  	GPIO_Pin_0

#define Button4Data					(GPIO_ReadInputDataBit(BTN4_GPIO_PORT,BTN4_PIN))

#elif defined (STM32F401_NUCLEO)	//STM32F4 NUCLEO
#define BTN4_GPIO_PORT          	GPIOC
#define BTN4_GPIO_CLK         		RCC_AHB1Periph_GPIOC
#define BTN4_PIN                  	GPIO_Pin_13

#define Button4Data					(!GPIO_ReadInputDataBit(BTN4_GPIO_PORT,BTN4_PIN))
#endif

typedef struct
{
	uint8_t Trg;
	uint8_t Cont;//continous

	uint16_t Delay;//for button debounce
} ButtonState;

typedef enum 
{ 
	BTN_RELEASE = 0,
	BTN_PRESS = 1,

} ButtonPressed;

/* Macro ---------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

void Custom_ButtonDebounce(void);

uint8_t Custom_Button1PressedLong(void);
uint8_t Custom_Button2PressedLong(void);
uint8_t Custom_Button3PressedLong(void);
uint8_t Custom_Button4PressedLong(void);

uint8_t Custom_Button1PressedOnce(void);
uint8_t Custom_Button2PressedOnce(void);
uint8_t Custom_Button3PressedOnce(void);
uint8_t Custom_Button4PressedOnce(void);

void Custom_ButtonScan(void);

void Custom_Button1Read(void);
void Custom_Button2Read(void);
void Custom_Button3Read(void);
void Custom_Button4Read(void);

void Custom_ButtonConfig(void);

/* Exported constants --------------------------------------------------------*/

#endif  /* __CUSTOM_BUTTON_H */

