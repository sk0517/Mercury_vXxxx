/***********************************************/
/* File Name : defMAIN.h 	         									   */
/*	Summary   : MAIN��` 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "define.h"

//----------------------------------------------------------------------
//�C���N���[�h�K�[�h
//----------------------------------------------------------------------
#ifndef DEFMAIN_H
#define DEFMAIN_H

extern struct stMAIN MAIN[CH_NUMMAX];

extern	short set_mode;			//1=set mode
extern	short	reset_factor;		//���Z�b�g�v��
extern	short	cunet_error;		//CUnet�G���[�v��
extern	short	send_err_status;	//���[�����M�G���[�X�e�[�^�X

extern	unsigned short	cunet_mcare;		//CUnet MCARE/LCARE���
extern	unsigned short	cunet_mfr1;	//CUnet MFR���1
extern	unsigned short	cunet_mfr2;	//CUnet MFR���2
extern	unsigned short	cunet_mfr3;	//CUnet MFR���3
extern	unsigned short	cunet_mfr4;	//CUnet MFR���4

//----------------------------------------------------------------------
//�\����
//----------------------------------------------------------------------
struct stMAIN{
	
	unsigned short err_judge;				//�G���[������i�Z���T�ُ퓙�j
	unsigned long err_judge_time[8];	//�G���[��������
	unsigned short com_err_status;		//�G���[�X�e�[�^�X�i�ʐM�p�j
	unsigned short com_act_status;		//����X�e�[�^�X�i�ʐM�p�j
	unsigned short err_condition;			//�G���[��ԁi�펞�X�V�p�j
	unsigned short err_condition_cu;		//�G���[��ԁi�펞�X�V�p�jCUnet�A�h���X�o�͗p
	unsigned short led_err_status;		//�G���[�X�e�[�^�X�i�G���[LED�p�j
	unsigned short alm_judge;					//�x��������
	unsigned long alm_judge_time[8];	//�x����������
	unsigned short total_judge;				//�ώZ�x��������
	unsigned short cvt_serial[8];			//�ϊ���V���A���ԍ�
	unsigned short sns_serial[8];			//�Z���T�V���A���ԍ�
	unsigned short err_sub_judge;			//�G���[������iMES_SUB[].err_status_sub���g�p����j
};

/********************************************************/
/*	�G���[����`										*/
/********************************************************/
typedef struct {
	short		err_code;		//�G���[�R�[�h
	short		err_priority;	//�D�揇��
	short		err_led; 		//�_��LED
} ERROR_INFO;

extern ERROR_INFO err_inf[];	

#endif


