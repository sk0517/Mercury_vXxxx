/***********************************************/
/* File Name : defMES.c	 	         									   */
/*	Summary   : ���b�Z�[�W�ʐM��`                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "defMES.h"	//���ʌv����`

//----------------------------------------------------------------------
//�O���[�o���ϐ�
//----------------------------------------------------------------------
struct stMES MES[CH_NUMMAX];	//���ʌv��
struct stMES_SUB MES_SUB[CH_NUMMAX];	//���ʌv��

#ifdef FRQSCH
struct stFrqSch FrqSch[CH_NUMMAX];
#endif
