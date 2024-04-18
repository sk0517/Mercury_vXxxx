/***********************************************/
/* File Name : ctlioport.h         									   */
/*	Summary   : I/O�|�[�g����			                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//�C���N���[�h�K�[�h
//----------------------------------------------------------------------
#ifndef CTLIOPORT_H
#define CTLIOPORT_H

//----------------------------------------------------------------------
//�O���錾
//----------------------------------------------------------------------
extern void portmesfwdW(unsigned char data, short pch);
extern void portmesrevW(unsigned char data, short pch);
extern void OutputRestartPulse(void);
extern void OutputStartPulse(void);
extern void CheckEndPulse(void);

#endif
