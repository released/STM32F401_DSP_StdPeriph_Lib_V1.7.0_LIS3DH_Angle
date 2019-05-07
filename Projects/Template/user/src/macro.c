/* Includes ------------------------------------------------------------------*/
#include "macro.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

double trunc(double d){ return (d>0) ? floor(d) : ceil(d) ; }

/*******************************************************************************
* Function Name  : AscToHex.
* Description    : convert ASCII to Hex
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsigned char ascii_to_hex(unsigned char aHex)
{
	if((aHex>=0)&&(aHex<=9))
		aHex += 0x30;

	else if((aHex>=10)&&(aHex<=15))//A-F
		aHex += 0x37;

	else aHex = 0xff;
	
	return aHex;
}

/*******************************************************************************
* Function Name  : HexToAsc.
* Description    : convert Hex to ASCII
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsigned char hex_to_ascii(unsigned char aChar)
{	  
	if((aChar>=0x30)&&(aChar<=0x39))	  
		aChar -= 0x30;
	  
	else if((aChar>=0x41)&&(aChar<=0x46))//upper case letter	  
		aChar -= 0x37;
	  
	else if((aChar>=0x61)&&(aChar<=0x66))//lower case letter	  
		aChar -= 0x57;
	  
	else aChar = 0xff;
	  
	return aChar;
}

/*******************************************************************************
* Function Name  : StringToChar.
* Description    : Convert String into Char
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsigned char string_to_char(unsigned char* value) {
  unsigned char temp;
  
  if(value[0] >= 65) {
    temp = (value[0]-55)*16;
  }
  else {
    temp = (value[0]-48)*16;
  }
  if(value[1] >= 65) {
    temp += value[1]-55;
  }
  else {
    temp += value[1]-48;
  }	
  return temp;
}	

/*******************************************************************************
* Function Name  : CharToString.
* Description    : Convert Char into String
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void char_to_string(unsigned char value, unsigned char* res) 
{  
	unsigned char temp;

	temp = value/16;
	if(temp < 10)
		res[0] = temp + 48;
	else
		res[0] = temp + 55;

	temp = value % 16;
	if(temp < 10)
		res[1] = temp + 48;
	else
		res[1] = temp + 55;
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void int_to_unicode(unsigned long value , unsigned char *pbuf , unsigned char len)
{
	unsigned char idx = 0;

	for( idx = 0 ; idx < len ; idx ++)
	{
		if( ((value >> 28)) < 0xA )
		{
		  pbuf[ 2* idx] = (value >> 28) + '0';
		}
		else
		{
		  pbuf[2* idx] = (value >> 28) + 'A' - 10; 
		}

		value = value << 4;

		pbuf[ 2* idx + 1] = 0;
	}
}

/*******************************************************************************
* Function Name  : floatToInt.
* Description    : Splits a float into two integer values.
* Input          : in the float value as input
* Input          : dec_prec the decimal precision to be used
* Output         : out_int the pointer to the integer part as output
* Output         : out_dec the pointer to the decimal part as output
* Return         : None. 
*******************************************************************************/
void float_to_int(float in, signed long *out_int, signed long *out_dec, signed long dec_prec)
{
    *out_int = (signed long)in;
    in = in - (float)(*out_int);
    *out_dec = (signed long)trunc(in*pow(10,dec_prec));
}

void printf_to_row_16(unsigned long len,unsigned char* array)
{
	unsigned long i = 0;
	unsigned long j = 0;

	printf("Data = \r\n");
	for(i=0;i<len;i++)
	{
		printf("%2x, ", array[i]);
		if (++j>=16)
		{
			j=0;
			printf("\r\n");
		}
	}
	printf("\r\nend of data\r\n");
}

