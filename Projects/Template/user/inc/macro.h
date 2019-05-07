/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MACRO_H
#define __MACRO_H

/* Platform config -----------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Define config -------------------------------------------------------------*/
#define 	TRUE			1
#define 	FALSE           0
#define 	ON				1
#define 	OFF				0
typedef unsigned char   BOOL;

#ifndef MEMCPY
#define MEMCPY(dst,src,len)             	memcpy(dst,src,len)
#endif

#ifndef MEMSET
#define MEMSET(buff,data,len)             	memset(buff,data,len)
#endif

#define LOBYTE(x)  							((uint8_t)(x & 0x00FF))
#define HIBYTE(x)  							((uint8_t)((x & 0xFF00) >>8)) 

#define _ABS(X)  ((X) > 0 ? (X) : -(X))   

/*=====================================================================
  MACRO : Get array number
=====================================================================*/
#define SIZEOF(a) (sizeof(a) / sizeof(a[0]))
#define ENDOF(a) ((a) + SIZEOF(a))

/*=====================================================================
  MACRO : Number Comparison Macro
=====================================================================*/
#define _MAX(a,b) (((a) > (b)) ? (a) : (b))
#define _MIN(a,b) (((a) < (b)) ? (a) : (b))

/*=====================================================================
  MACRO : Swap Integers Macro
=====================================================================*/
#define SWAP(a, b)     {(a) ^= (b); (b) ^= (a); (a) ^= (b);}

/*=====================================================================
  MACRO : For Digial to HEX in ASCII conversion
=====================================================================*/
#define bDigToAsc(bDig) (((bDig) > 9) ? (((bDig) - 10) + 'A') : ((bDig) + '0'))

/*=====================================================================
  MACRO : Change character to upper case
=====================================================================*/
#define UPCASE( c ) ( ((c) >= 'a' && (c) <= 'z') ? ((c) - 0x20) : (c) )

/*=====================================================================
  MACRO : Check character if is decimal
=====================================================================*/
#define DECCHK( c ) ((c) >= '0' && (c) <= '9')

/*=====================================================================
  MACRO : Check character if is hex
=====================================================================*/
#define HEXCHK( c ) ( ((c) >= '0' && (c) <= '9') ||((c) >= 'A' && (c) <= 'F') ||((c) >= 'a' && (c) <= 'f') )

/*=====================================================================
  MACRO : Prevent overflow
=====================================================================*/
#define INC_SAT( val ) (val = ((val)+1 > (val)) ? (val)+1 : (val))

/*=====================================================================
  MACRO : Get field byte in a structure
=====================================================================*/
#define FBIS( type, field ) sizeof( ((type *) 0)->field )

/*=====================================================================
  MACRO : Get field address in a structure
=====================================================================*/
#define FAIS(type,field) (sizeof)&(((type *)0)->field)

/*=====================================================================
  MACRO : For BCD number
=====================================================================*/
#define BCD_HI(bcd)  ((bcd) >> 4)
#define BCD_LO(bcd)  ((bcd) & 0x0f)
#define BCD2DEC(bcd) ((BCD_HI(bcd) * 10) +BCD_LO(bcd))
#define DEC2BCD(dec) ((((dec) / 10) << 4) + ((dec) % 10))

/* Exported functions ----------------------------------------------- */
unsigned char ascii_to_hex(unsigned char aHex);
unsigned char hex_to_ascii(unsigned char aChar);
unsigned char string_to_char(unsigned char* value);
void char_to_string(unsigned char value, unsigned char* res);
void int_to_unicode (unsigned long value , unsigned char *pbuf , unsigned char len);
void float_to_int(float in, signed long *out_int, signed long *out_dec, signed long dec_prec);
void printf_to_row_16(unsigned long len,unsigned char* array);


#endif  /* __MACRO_H */

