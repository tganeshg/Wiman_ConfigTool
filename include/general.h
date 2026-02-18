/*
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
*/
#ifndef _GENERAL_H_
#define _GENERAL_H_

#include <stdio.h>
#include <string.h>

#include "common.h"

/*
*Macros
*/
#define APP_VERSION				"G 1.0.0 13012018"
#define DEBUG_LOG				1

//for Flags use only
extern UINT64 flag1;

#define	POWER_ON				0

#define SET_FLAG(n)				((flag1) |= (U64)(1ULL << (n)))
#define CLR_FLAG(n)				((flag1) &= (U64)~((1ULL) << (n)))
#define CHECK_FLAG(n)			((flag1) & (U64)(1ULL<<(n)))

/*
*Structure
*/
#pragma pack(push,1)

#pragma pack(pop)

/*
*Function declarations
*/

#endif

/* EOF */
