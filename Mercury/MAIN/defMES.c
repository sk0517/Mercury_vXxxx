/***********************************************/
/* File Name : defMES.c	 	         									   */
/*	Summary   : メッセージ通信定義                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "defMES.h"	//流量計測定義

//----------------------------------------------------------------------
//グローバル変数
//----------------------------------------------------------------------
struct stMES MES[CH_NUMMAX];	//流量計測
struct stMES_SUB MES_SUB[CH_NUMMAX];	//流量計測

#ifdef FRQSCH
struct stFrqSch FrqSch[CH_NUMMAX];
#endif
