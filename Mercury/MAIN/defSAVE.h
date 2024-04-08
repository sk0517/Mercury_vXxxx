/***********************************************/
/* File Name : defSAVE.h		         									   */
/*	Summary   : �ۑ��f�[�^��`		                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//�C���N���[�h�K�[�h
//----------------------------------------------------------------------
#ifndef DEFSAVE_H
#define DEFSAVE_H

#include "define.h"

struct stSAVE{	//�ۑ��f�[�^

	unsigned short control;		//�R���g���[��
	unsigned short control_old;	//����l�ۑ�
	unsigned short sum_abs_com[SUB_POINT];	/*��Βl�̘a�A�i�[�̈�*/
	short log_save_num;			//�G���[���O�o�^�ԍ�

};

extern struct stSAVE SAVE[CH_NUMMAX];//�ۑ��f�[�^

#endif
