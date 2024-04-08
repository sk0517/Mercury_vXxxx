/***********************************************/
/* File Name : ctlioport.h         									   */
/*	Summary   : I/Oポート処理			                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//インクルードガード
//----------------------------------------------------------------------
#ifndef CTLIOPORT_H
#define CTLIOPORT_H

//----------------------------------------------------------------------
//外部宣言
//----------------------------------------------------------------------
extern void portmesfwdW(unsigned char data, short pch);
extern void portmesrevW(unsigned char data, short pch);
extern void OutputRestartPulse(void);
extern void OutputStartPulse(void);
extern void CheckEndPulse(void);

#endif
