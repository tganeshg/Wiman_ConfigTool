/*****************************************************************
*
*	Copyright (c) 2026
*	All rights reserved.
*
*	Project 		:
*	Last Updated on	:
*	Author			: Ganesh
*
*	Revision History
****************************************************************
*	Date			Version		Name		Description
****************************************************************
*	07/02/2026		1.0			Ganesh		Initial Development
*
*****************************************************************/

#ifndef _COMMON_H_
#define _COMMON_H_

typedef unsigned char	BYTE;
typedef unsigned short	WORD;
typedef unsigned long	DWORD;

#ifndef FALSE
#define FALSE	0
#endif

#ifndef TRUE
#define TRUE	1
#endif

#define BYTE_BSET(val, bit)	((val) |= (bit))
#define BYTE_BCLR(val, bit)	((val) &= ~(bit))
#define BYTE_BCHK(val, bit)	(((val) & (bit)) ? 1:0)

#define CHAR2DEC(x)			((x)-'0')
#define CHAR2HEX(x)			((((x)>'9')?((x)-'A'+10):((x)-'0'))&0xF)
#define IS_LEAP_YEAR(Y)		( ((Y)>0) && !((Y)%4) && ( ((Y)%100) || !((Y)%400) ) )

//Enums
//Size constants
typedef enum
{
	SIZE_2=2,
	SIZE_4=4,
	SIZE_6=6,
	SIZE_8=8,
	SIZE_16=16,
	SIZE_32=32,
	SIZE_64=64,
	SIZE_128=128,
	SIZE_256=256,
	SIZE_512=512,
	SIZE_640=640,
	SIZE_1024=1024,
	SIZE_2048=2048,
	SIZE_6144=6144
}BUFFER_SIZES;

//Common Error Code
typedef enum
{
	RET_FAILURE=-1,
	RET_OK=0
}ERROR_CODE;

#endif

/* EOF */
