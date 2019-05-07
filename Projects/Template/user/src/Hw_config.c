/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(void)  
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)  
#endif /* __GNUC__ */

// TODO: for printf function , need to confirm use USART2 or USART2
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t uwTimingDelay;

/*uart variable*/
__IO uint8_t UartRxBuffer = 0;
__IO uint8_t UartRxFlag = FALSE;

#define PI (float)3.14159265f
float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;

uint8_t FlagSwitch = 0;

/* Private functions ---------------------------------------------------------*/
void Angle_Calculate(void)
{  
	#if 1	//Tilt Test.xlsx @Yiven

	if (FlagSwitch)
	{
		printf("Accelerometer(mg):     X-axis : %5d, ",(int16_t)LIS3DH_accx);
		printf(" Y-axis : %5d,  ",(int16_t)LIS3DH_accy);
		printf(" Z-axis :  %5d \n\r",(int16_t)LIS3DH_accz);
	}
	else
	{
		float s1 = 0;
		float s2 = 0;	

		s1 = sqrt((float)((LIS3DH_accy*LIS3DH_accy)+(LIS3DH_accz*LIS3DH_accz)));
		s2 = sqrt((float)((LIS3DH_accx*LIS3DH_accx)+(LIS3DH_accz*LIS3DH_accz)));

		PitchAng = atan(LIS3DH_accx/s1)*180/PI;
		RollAng = atan(LIS3DH_accy/s2)*180/PI;

		printf("%s:RollAng(%10.3lf),PitchAng(%10.3lf)\r\n",__FUNCTION__,RollAng,PitchAng);
	}
	
	#endif
	
	#if 0
/*
	http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation

	Roll = atan2(Y, Z) * 180/M_PI;
	Pitch = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/M_PI;
	Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf 
*/


	RollAng= atan2(LIS3DH_accy, LIS3DH_accz) * 180/PI;
	PitchAng = atan2(-LIS3DH_accz, sqrt(LIS3DH_accy*LIS3DH_accy + LIS3DH_accz*LIS3DH_accz)) * 180/PI;

	printf("%s:RollAng(%10.3lf),PitchAng(%10.3lf)\r\n",__FUNCTION__,RollAng,PitchAng);
	#endif
	
	#if 0
    RollAng  = 180.0*atan(LIS3DH_accy/sqrt((float)(LIS3DH_accx*LIS3DH_accx+LIS3DH_accz*LIS3DH_accz)))/PI;       
    PitchAng = -180.0*atan(LIS3DH_accx/sqrt((float)(LIS3DH_accz*LIS3DH_accz+LIS3DH_accy*LIS3DH_accy)))/PI;
    if(LIS3DH_accz<0)
	{
        if(LIS3DH_accy>0)
		{
			RollAng  = 180.0 -RollAng;
		}
        else 
		{
			RollAng  = -(RollAng+180.0);
		}
    }
	printf("%s:RollAng(%10.3lf),PitchAng(%10.3lf)\r\n",__FUNCTION__,RollAng,PitchAng);
	#endif
	
	#if 0
	LIS3DH_accx /= 100.0f;
	LIS3DH_accy /= 100.0f;
	LIS3DH_accz /= 100.0f;	

	fNormAcc = sqrt((LIS3DH_accx*LIS3DH_accx)+(LIS3DH_accy*LIS3DH_accy)+(LIS3DH_accz*LIS3DH_accz));

	fSinRoll = -LIS3DH_accy/fNormAcc;
	fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
	fSinPitch = LIS3DH_accx/fNormAcc;
	fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
	if ( fSinRoll >0)
	{
		if (fCosRoll>0)
		{
			RollAng = acos(fCosRoll)*180/PI;
		}
		else
		{
			RollAng = acos(fCosRoll)*180/PI + 180;
		}
	}
	else
	{
		if (fCosRoll>0)
		{
			RollAng = acos(fCosRoll)*180/PI + 360;
		}
		else
		{
			RollAng = acos(fCosRoll)*180/PI + 180;
		}
	}

	if ( fSinPitch >0)
	{
		if (fCosPitch>0)
		{
			PitchAng = acos(fCosPitch)*180/PI;
		}
		else
		{
			PitchAng = acos(fCosPitch)*180/PI + 180;
		}
	}
	else
	{
		if (fCosPitch>0)
		{
			PitchAng = acos(fCosPitch)*180/PI + 360;
		}
		else
		{
			PitchAng = acos(fCosPitch)*180/PI + 180;
		}
	}

	if (RollAng >=360)
	{
		RollAng = RollAng - 360;
	}

	if (PitchAng >=360)
	{
		PitchAng = PitchAng - 360;
	}

	printf("%s:RollAng(%10.3lf),PitchAng(%10.3lf)\r\n",__FUNCTION__,RollAng,PitchAng);

	#endif
}

void Button_Procedure(void)
{
	Custom_ButtonScan();

	if (Custom_Button4PressedOnce())
	{
		FlagSwitch = (FlagSwitch==1)?(0):(1);
		printf("4444\r\n");	
	}	

	if (Custom_Button4PressedLong()&& !Custom_Button4PressedOnce())
	{
		printf("4444 long\r\n");
	}
}

void PollingProcedure(void)
{
	SelfTestLIS3DH();
	Angle_Calculate();

}

void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/*
	TIMxCLK = PCLK1 = HCLK = SystemCoreClock
	TIMx counter clock = TIMxCLK /((Prescaler + 1)*(Period + 1))
	                = 84 MHz / ((11+1)*(6999+1))
	                = 1000 Hz 
     ==> TIMx counter period = 1 ms
*/
void TIM2_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* PCLK1 = HCLK/4 */
	RCC_PCLK1Config(RCC_HCLK_Div2);//TIM3CLK = (HCLK/4)x2 = (180 MHz/4)x2 = 90 MHz 
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* TIMx base configuration */
	TIM_TimeBaseStructure.TIM_Period = (7000 -1);
	TIM_TimeBaseStructure.TIM_Prescaler = (12 -1);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* TIMx Interrupts enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM2, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

}

/*
	TIMxCLK = PCLK1 = HCLK = SystemCoreClock
	TIMx counter clock = TIMxCLK /((Prescaler + 1)*(Period + 1))
	                = 84 MHz / ((11+1)*(6999+1))
	                = 1000 Hz 
     ==> TIMx counter period = 1 ms
*/
void TIM3_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* PCLK1 = HCLK/4 */
	RCC_PCLK1Config(RCC_HCLK_Div2);//TIM3CLK = (HCLK/4)x2 = (180 MHz/4)x2 = 90 MHz 
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* TIMx base configuration */
	TIM_TimeBaseStructure.TIM_Period = (7000 -1);
	TIM_TimeBaseStructure.TIM_Prescaler = (12 -1);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* TIMx Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

}

void USART_Test(void)
{
	if (UartRxFlag)
	{
		switch (UartRxBuffer)
		{
			case '1':

				break;
				
			case 'Z':
			case 'z':				
				NVIC_SystemReset();
				break;				
			default : 
				UartRxFlag = FALSE;				
				break;
		}
		UartRxFlag = FALSE;
	}
}

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USARTx configured as follows:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 

	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{}
	
	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);


	printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");


}

void SysTickConfig(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	/* This function fills the RCC_ClockFreq structure with the current
	 frequencies of different on chip clocks (for debug purpose) */	
	RCC_GetClocksFreq(&RCC_Clocks);
	
	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d Hz\n\r",RCC_Clocks.SYSCLK_Frequency);	
	printf("AHB = %d Hz\n\r",RCC_Clocks.HCLK_Frequency);
	printf("APB1 = %d Hz\n\r",RCC_Clocks.PCLK1_Frequency);
	printf("APB2 = %d Hz\n\r",RCC_Clocks.PCLK2_Frequency);
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
	
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t uTime)
{ 
	uwTimingDelay = uTime;
	while(uwTimingDelay != 0);
}


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/
uint8_t UART_GetByte(void)
{
	while ( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
	{
	}
	return (uint8_t)USART_ReceiveData(USART2);
}

void UART_SendByte(uint8_t Data)
{
	USART_SendData(USART2 , (unsigned char)Data);
	while (USART_GetFlagStatus(USART2 , USART_FLAG_TC)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	#if 1
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
	#else	//ignore len
    while(*Data)  
    {  
        USART_SendData(USART2, (unsigned char) *Data++);  
        /* Loop until the end of transmission */  
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);  //USART_FLAG_TXE
    } 
	#endif
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


