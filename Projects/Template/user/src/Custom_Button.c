/* Includes ------------------------------------------------------------------*/
#include "Custom_Button.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define	BTNMagicNumber 		(0x00)	//(0xFF)

/* Private function prototypes -----------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

ButtonState BTNState1 = {0,0,0};
ButtonState BTNState2 = {0,0,0};
ButtonState BTNState3 = {0,0,0};

ButtonState BTNState4 = {0,0,0};

/*Button variable*/


/* Private functions ---------------------------------------------------------*/
void Custom_ButtonDebounce(void)
{
//	BTNState1.Delay++;//for button debounce
//	BTNState2.Delay++;//for button debounce
//	BTNState3.Delay++;//for button debounce		
	BTNState4.Delay++;//for button debounce
}

uint8_t Custom_Button4PressedLong(void)
{
	#if 1
	uint8_t res = 0;

	res = (Button4Data)^(BTNMagicNumber);

	if ((BTNState4.Delay>ButtonDebounceDelay) && (res && BTNState4.Cont))
	{
		BTNState4.Delay = 0;
		return (uint8_t)BTN_PRESS;
	}
	else
	{
		return (uint8_t)BTN_RELEASE;
	}
	#else
	return BTNState4.Cont;
	#endif	

}

uint8_t Custom_Button4PressedOnce(void)
{
	return BTNState4.Trg;
}

void Custom_ButtonScan(void)
{
	Custom_Button4Read();
	
	#if 0
	printf(">>#1 : %X , %d ,>>##2 : %d , %X ,>>##3 : %X , %X \r\n",BTNState1.Trg,BTNState1.Cont,
												BTNState2.Trg,BTNState2.Cont,
												BTNState3.Trg,BTNState3.Cont);
	#endif
}

void Custom_Button4Read(void)
{
    uint8_t ReadData = (Button4Data)^(BTNMagicNumber);
    BTNState4.Trg = ReadData & (ReadData ^ BTNState4.Cont);
    BTNState4.Cont = ReadData;

//	printf("ReadData4 = 0x%2X , Trg1 = 0x%2X , Cont1 = 0x%2X\r\n",ReadData,BTNState4.Trg,BTNState4.Cont);	

}

void Custom_ButtonConfig(void)	//PC13 for NUCLEO board test
{

  	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(BTN4_GPIO_CLK, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = BTN4_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(BTN4_GPIO_PORT, &GPIO_InitStructure);

}



