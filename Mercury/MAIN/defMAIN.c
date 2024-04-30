/***********************************************/
/* File Name : defMAIN.c		         									   */
/*	Summary   : MAIN��` 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include "defMAIN.h"

struct stMAIN MAIN[CH_NUMMAX];

short set_mode = 0;	//1=set mode
short	reset_factor;	//���Z�b�g�v��
short	cunet_error;	//CUnet�G���[�v��
short	send_err_status;//���[�����M�G���[�X�e�[�^�X

unsigned short	cunet_mcare;	//CUnet MCARE/LCARE���
unsigned short	cunet_mfr1;	//CUnet MFR���1
unsigned short	cunet_mfr2;	//CUnet MFR���2
unsigned short	cunet_mfr3;	//CUnet MFR���3
unsigned short	cunet_mfr4;	//CUnet MFR���4

ERROR_INFO err_inf[];	

ERROR_INFO err_inf[] = {
	{							//0.None
		0,						//
		0,						//
		0						//
	},
	{							//1.�[�������G���v�e�B�Z���T
		ERR_ZERO_EMPTY,			//�G���[�R�[�h
		1,						//�D�揇��
		DSP_ALM1 | DSP_ALM3		//�_��LED
	},
	{							//2.�[�������g�`���� �i�g�`�ُ�j
		ERR_ZERO_WAVE,			//�G���[�R�[�h
		2,						//�D�揇��
		DSP_ALM3 | DSP_ALM4		//�_��LED
	},
	{							//3.�[���������Z�ُ�
		ERR_ZERO_CALC,			//�G���[�R�[�h
		3,						//�D�揇��
		DSP_ALM2 | DSP_ALM3		//�_��LED
	},
	{							//4.�[�������g�`�A���o�����X
		ERR_ZERO_LEVEL,			//�G���[�R�[�h
		4,						//�D�揇��
		DSP_ALM2 | DSP_ALM3		//�_��LED
	},
	{							//5.�[������AGC�s�\//
		ERR_ZERO_AGC,			//�G���[�R�[�h
		5,						//�D�揇��
		DSP_ALM3 | DSP_ALM4		//�_��LED
	},
	{							//6.����G���v�e�B�Z���TL
		ERR_MESR_EMPTY_L,		//�G���[�R�[�h
		12,						//�D�揇��
		DSP_ALM1				//�_��LED
	},
	{							//7.����G���v�e�B�Z���TH
		ERR_MESR_EMPTY_H,		//�G���[�R�[�h
		7,						//�D�揇��
		DSP_ALM1				//�_��LED
	},
	{							//8.����g�`����L
		ERR_MESR_WAVE_L,		//�G���[�R�[�h
		13,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//9.����g�`����H
		ERR_MESR_WAVE_H,		//�G���[�R�[�h
		8,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//10.���艉�Z�ُ�L
		ERR_MESR_CALC_L,		//�G���[�R�[�h
		14,						//�D�揇��
		DSP_ALM2				//�_��LED
	},
	{							//11.���艉�Z�ُ�H
		ERR_MESR_CALC_H,		//�G���[�R�[�h
		9,						//�D�揇��
		DSP_ALM2				//�_��LED
	},
	{							//12.����g�`�A���o�����XL
		ERR_MESR_LEVEL_L,		//�G���[�R�[�h
		15,						//�D�揇��
		DSP_ALM2				//�_��LED
	},
	{							//13.����g�`�A���o�����XH
		ERR_MESR_LEVEL_H,		//�G���[�R�[�h
		10,						//�D�揇��
		DSP_ALM2				//�_��LED
	},
	{							//14.����AGC�s�\L
		ERR_MESR_AGC_L,			//�G���[�R�[�h
		16,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//15.����AGC�s�\H
		ERR_MESR_AGC_H,			//�G���[�R�[�h
		11,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//16.����t���ُ�
		ERR_MESR_REVERSE,		//�G���[�R�[�h
		19,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//17.Reserve
		0,						//
		0,						//
		0						//
	},
	{							//18.Reserve
		0,						//
		0,						//
		0						//
	},
	{							//19.10�_�f�[�^����
		ERR_10POINT,			//�G���[�R�[�h
		21,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//20.EEPROM�G���[
		ERR_EEPROM,				//�G���[�R�[�h
		25,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//21.Reserve
		0,						//
		0,						//
		0						//
	},
	{							//22.�ċN��
		ERR_RESTART,			//�G���[�R�[�h
		27,						//�D�揇��
		0						//�_��LED
	},
	{							//23.����I�[�o�[�t���[
		ERR_MESR_OVERFLOW,		//�G���[�R�[�h
		20,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//24.CUnet�G���[
		ERR_CUNET,				//�G���[�R�[�h
		26,						//�D�揇��
		DSP_ALM4				//�_��LED
	},
	{							//25.�[�������v�����Ԕ��U
		ERR_ZERO_UNSTABLE,		//�G���[�R�[�h
		6,						//�D�揇��
		DSP_ALM3 | DSP_ALM4		//�_��LED
	},
	{							//26.����A���v�Q�C���}�ρi�x���j
		ALM_MESR_GAIN,		//�G���[�R�[�h
		17,						//�D�揇��
		0							//�_��LED
	},
	{							//27.����G���v�e�B�Z���T�i�x���j
		ALM_MESR_EMPTY,		//�G���[�R�[�h
		18,						//�D�揇��
		0							//�_��LED
	},
	{							//28.�ώZ�l���Z�ُ�
		TTL_CACL_ERR,		//�G���[�R�[�h
		22,						//�D�揇��
		DSP_ALM4			//�_��LED
	},
	{							//29.�ώZ�l�I�[�o�[�t���[
		TTL_OVERFLOW,		//�G���[�R�[�h
		23,						//�D�揇��
		DSP_ALM4			//�_��LED
	},
};

