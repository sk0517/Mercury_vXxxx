/***********************************************/
/* File Name : defLED.h 		         									   */
/*	Summary   : LED��`  					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//�C���N���[�h�K�[�h
//----------------------------------------------------------------------
#ifndef DEFLED_H
#define DEFLED_H

#include "define.h"

extern struct stLED LED[CH_NUMMAX];	//LED�\��

//----------------------------------------------------------------------
//�\����
//----------------------------------------------------------------------
struct stLED{	//LED�\��
	
	short zero_do_cnt;				/*�[���������s�J�E���^*/
	short vth_do_cnt;					/*Vth�������s�J�E���^*/
	short wave_do_cnt;				/*�g�`�F���������s�J�E���^*/

	short zero_cal_cnt;				/*�[�������p��Ts�J�E���^*/
	unsigned long zero_delta_ts;	/*�[�������p��Ts���Z�l*/

	short zero_retry_cnt;				/*�[���������g���C�J�E���^*/
	short zero_active;				/*�[���������s��*/

	short zero_dt_buf[3];				/*�[�������p��Ts�擾�l*/
	short zero_vth_buf[3];			/*�[������vth�擾�l*/

	float zero_dt_buf_f[3]; //��Ts�擾�l float�p
};

#endif


