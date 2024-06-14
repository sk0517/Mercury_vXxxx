/***********************************************/
/* File Name : util.c		            									   */
/*	Summary   : ���[�e�B���e�B�֐�	                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>

#include "define.h"
#include "SV_def.h"
#include "typedefine.h"
#include "defMES.h"
#include "defLOG.h"
#include "defMAIN.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"

/********************************************************/
/*	���W���[������`�֐�								*/
/********************************************************/
void	util_delay(short target_time);
short util_passed_time(unsigned long basic_time, unsigned long target_time);
void	err_judge_status(short pch);
void	err_judge_holdtime(short pch);
void	err_status_control(short pch);
void	action_status_control(short pch, short act);
short action_status_check(short pch);
void	pwon_count(void);
unsigned long invert_data(unsigned long data);
void delay(unsigned short delay_count);
void	reset_factor_check(void);
void	reset_control(void);
void	non_sensor_control(short pch);
void util_eep_allwrite(short pch, short opt);
void util_SnsMem_Write(short pch, short opt);
void util_eep_zerowrite(short pch);
void	read_serial_num(short pch);
void	write_serial_num(short pch);
void	err_priority(short ch, short new_err_code);
void	remove_errcode(short ch, short err_code);
short	err_zero_status(short err_status);
short	err_total_status(short err_status);
unsigned long long get_total_offset(short ch, unsigned long long val_total);
float RoundFunc(float src);
void	debug_mode(short pch);

/********************************************************/
/*	���W���[���O��`�֐�								*/
/********************************************************/
extern void	log_save(short ch, short code);
extern void	eep_write_ch(short, short, short);
extern void eep_write_ch_delay(short ch, short addr, short data);
extern short	eep_read(short rom_addr);
extern void WatchdogReloadSet_dl(uint32_t ui32Base, uint32_t ui32LoadVal);
extern void WatchdogResetEnable_dl(uint32_t ui32Base);

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/
short end_wdt;
unsigned long	cmi_count = 1;

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern uint32_t g_ui32SysClock;
extern short OWwrite[6];

/****************************************************/
/* Function : util_delay                       */
/* Summary  : �f�B���C	�i��5msec�P�ʁj   				*/
/* Argument : target_time:�~���b�ݒ�	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	util_delay(short target_time){

	unsigned long	rtc_count_buff;
	unsigned long	time_buff;
	unsigned long	target_time_buff;
	
	rtc_count_buff = cmi_count;				//�^�C�}������(5msec)�J�E���^�擾
	if(target_time < 0 || target_time > 100){
		target_time_buff = 0;							//�ُ�l�̏ꍇ�́u0�v�ݒ�
	}else{
		target_time_buff = (unsigned long)target_time;	//�ڕW���Ԑݒ�
	}
	time_buff = rtc_count_buff + (target_time_buff / 5);
	
	while(1){								//�o�ߊm�F
		if(time_buff > CMI_COUNT_MAX){
			if((time_buff - CMI_COUNT_MAX) < cmi_count){
				break;						//���Ԍo��
			}
		}else{
			if(time_buff < cmi_count){
				break;						//���Ԍo��
			}
		}
	}
}

/****************************************************/
/* Function : util_passed_time                      */
/* Summary  : �o�ߎ��Ԋm�F     				*/
/*	Argument : basic_time  : ��{����	(5msec�P��)				*/
/*		�@�@        target_time : �ڕW����	(5msec�P��)				*/
/* Return   : B_NO �F���o��,  B_YES�F�o��              */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		util_passed_time(unsigned long basic_time, unsigned long target_time){

	short	status;
	unsigned long	time_buff;

	status = B_NO;
	time_buff = basic_time + target_time;
	
	if(time_buff > CMI_COUNT_MAX){
		if((time_buff - CMI_COUNT_MAX) <= cmi_count){
			status = B_YES;					//�ڕW���Ԃ��o��
		}
	}else{
		if(time_buff <= cmi_count){
			status = B_YES;					//�ڕW���Ԃ��o��
		}
	}

	return (status);
}

/****************************************************/
/* Function : err_judge_status                      */
/* Summary  : �G���[���菈��    				*/
/* Argument : pch                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	err_judge_status(short pch){

	unsigned short err_status, alm_status, total_status;
	unsigned short err_status_buf, alm_status_buf, total_status_buf;

	if(pch >= CH_NUMMAX){
		return;
	}
	
	err_status = MES[pch].err_status;							//�����ʃG���[���
	err_status_buf = MAIN[pch].err_judge;			//���ʃG���[���o�b�t�@

	err_status_control(pch);								//�G���[�X�e�[�^�X�����i�ʐM�p�j
	
	if(err_status_buf != err_status){							//���ʃG���[���ω�����

		//�[���_�������r�b�g��0��1�ɕω������ꍇ
		if(((err_status_buf & ERR_JUDGE_ZERO) == 0)			//�[���_�������r�b�g
				&&((err_status & ERR_JUDGE_ZERO) != 0)){
					
			MAIN[pch].err_condition |= CON_ZERO;
					
			if((err_status & ERR_JUDGE_EMPTY) != 0){			//�G���v�e�B�Z���T�[
				MAIN[pch].err_judge_time[0] = 0;
				log_save(pch, ERR_ZERO_EMPTY);					//�G���[���O���̓o�^
			}else if((err_status & ERR_JUDGE_CALC) != 0){		//���Z�ُ�
				MAIN[pch].err_judge_time[1] = 0;
				log_save(pch, ERR_ZERO_CALC);					//�G���[���O���̓o�^
			}else if((err_status & ERR_JUDGE_LEVEL) != 0){	//�g�`�A���o�����X
				MAIN[pch].err_judge_time[2] = 0;
				log_save(pch, ERR_ZERO_LEVEL);					//�G���[���O���̓o�^
			}else if((err_status & ERR_JUDGE_AGC) != 0){		//AGC�s�\//
				MAIN[pch].err_judge_time[3] = 0;
				log_save(pch, ERR_ZERO_AGC);					//�G���[���O���̓o�^
			}else if((err_status & ERR_JUDGE_WAVE) != 0){		//�g�`����
				MAIN[pch].err_judge_time[5] = 0;
				log_save(pch, ERR_ZERO_WAVE);					//�G���[���O���̓o�^
			}
		}

		//���ʃG���[���r�b�g��0��1�ɕω������ꍇ
		if(((err_status_buf & ERR_JUDGE_EMPTY) == 0)			//�G���v�e�B�Z���T�[
			&&((err_status & ERR_JUDGE_EMPTY) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//�[���_������
				MAIN[pch].err_judge_time[0] = 0;
				log_save(pch, ERR_ZERO_EMPTY);					//�G���[���O���̓o�^
			}else{												//���ʑ��莞
				MAIN[pch].err_judge_time[0] = cmi_count;		//�G���[�������ԓo�^
				log_save(pch, ERR_MESR_EMPTY_L);				//�G���[���O���̓o�^
			}
		}

		if(((err_status_buf & ERR_JUDGE_CALC) == 0)			//���Z�ُ�
			&&((err_status & ERR_JUDGE_CALC) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//�[���_������
				MAIN[pch].err_judge_time[1] = 0;
				log_save(pch, ERR_ZERO_CALC);					//�G���[���O���̓o�^
			}else{												//���ʑ��莞
				MAIN[pch].err_judge_time[1] = cmi_count;		//�G���[�������ԓo�^
				log_save(pch, ERR_MESR_CALC_L);					//�G���[���O���̓o�^
			}
		}
			
		if(((err_status_buf & ERR_JUDGE_LEVEL) == 0)			//�g�`�A���o�����X
			&&((err_status & ERR_JUDGE_LEVEL) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//�[���_������
				MAIN[pch].err_judge_time[2] = 0;
				log_save(pch, ERR_ZERO_LEVEL);					//�G���[���O���̓o�^
			}else{
				MAIN[pch].err_judge_time[2] = cmi_count;		//�G���[�������ԓo�^
				log_save(pch, ERR_MESR_LEVEL_L);				//�G���[���O���̓o�^
			}
		}

		if(((err_status_buf & ERR_JUDGE_AGC) == 0)				//AGC�s�\//
			&&((err_status & ERR_JUDGE_AGC) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//�[���_������
				MAIN[pch].err_judge_time[3] = 0;
				log_save(pch, ERR_ZERO_AGC);					//�G���[���O���̓o�^
			}else{
				MAIN[pch].err_judge_time[3] = cmi_count;		//�G���[�������ԓo�^
				log_save(pch, ERR_MESR_AGC_L);					//�G���[���O���̓o�^
			}
		}

		if(((err_status_buf & ERR_JUDGE_REVERSE) == 0)			//�t���ُ�
			&&((err_status & ERR_JUDGE_REVERSE) != 0)){
			MAIN[pch].err_judge_time[4] = cmi_count;			//�G���[�������ԓo�^
		}
		
		if(((err_status_buf & ERR_JUDGE_WAVE) == 0)			//�g�`����
			&&((err_status & ERR_JUDGE_WAVE) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//�[���_������
				MAIN[pch].err_judge_time[5] = 0;
				log_save(pch, ERR_ZERO_WAVE);					//�G���[���O���̓o�^
			}else{
				MAIN[pch].err_judge_time[5] = cmi_count;		//�G���[�������ԓo�^
				log_save(pch, ERR_MESR_WAVE_L);					//�G���[���O���̓o�^
			}
		}

		if(((err_status_buf & ERR_JUDGE_OVERFLOW) == 0)		//�I�[�o�[�t���[
			&&((err_status & ERR_JUDGE_OVERFLOW) != 0)){
			MAIN[pch].err_judge_time[6] = cmi_count;			//�G���[�������ԓo�^
			log_save(pch, ERR_MESR_OVERFLOW);					//�G���[���O���̓o�^
		}
		
		if(((err_status_buf & ERR_JUDGE_10POINT) == 0)			//10�_�f�[�^����
			&&((err_status & ERR_JUDGE_10POINT) != 0)){
			log_save(pch, ERR_10POINT);							//�G���[���O���̓o�^
		}
		if(((err_status_buf & ERR_JUDGE_EEPROM) == 0)			//EEPROM�G���[
			&&((err_status & ERR_JUDGE_EEPROM) != 0)){
			log_save(pch, ERR_EEPROM);							//�G���[���O���̓o�^
		}
		if(((err_status_buf & ERR_JUDGE_CUNET) == 0)			//CUnet�G���[
			&&((err_status & ERR_JUDGE_CUNET) != 0)){
			log_save(pch, ERR_CUNET);							//�G���[���O���̓o�^
		}
		if(((err_status_buf & ERR_JUDGE_UNSTABLE) == 0)		//�[�������v�����Ԕ��U�G���[
			&&((err_status & ERR_JUDGE_UNSTABLE) != 0)){
			log_save(pch, ERR_ZERO_UNSTABLE);					//�G���[���O���̓o�^
		}
		if(((err_status_buf & ERR_JUDGE_RESTART) == 0)			//�ċN��
			&&((err_status & ERR_JUDGE_RESTART) != 0)){
			log_save(pch, ERR_RESTART);							//�G���[���O���̓o�^
		}

		//���ʃG���[���r�b�g��1��0�ɕω������ꍇ
		if(((err_status_buf & ERR_JUDGE_EMPTY) != 0)			//�G���v�e�B�Z���T�[
			&&((err_status & ERR_JUDGE_EMPTY) == 0)){
			MAIN[pch].err_judge_time[0] = 0;					//�G���[�������ԃN���A
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//���ʑ��莞�̃G���[
				remove_errcode(pch, ERR_MESR_EMPTY_L);			//�G���[�R�[�h����
				remove_errcode(pch, ERR_MESR_EMPTY_H);			//
			}													//�[���������̃G���[�R�[�h�͉������Ȃ�
		}

		if(((err_status_buf & ERR_JUDGE_CALC) != 0)			//���Z�ُ�
			&&((err_status & ERR_JUDGE_CALC) == 0)){
			MAIN[pch].err_judge_time[1] = 0;					//�G���[�������ԃN���A
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//���ʑ��莞�̃G���[
				remove_errcode(pch, ERR_MESR_CALC_L);			//�G���[�R�[�h����
				remove_errcode(pch, ERR_MESR_CALC_H);			//
			}													//�[���������̃G���[�R�[�h�͉������Ȃ�
		}

		if(((err_status_buf & ERR_JUDGE_LEVEL) != 0)			//�g�`�A���o�����X
			&&((err_status & ERR_JUDGE_LEVEL) == 0)){
			MAIN[pch].err_judge_time[2] = 0;					//�G���[�������ԃN���A
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//���ʑ��莞�̃G���[
				remove_errcode(pch, ERR_MESR_LEVEL_L);			//�G���[�R�[�h����
				remove_errcode(pch, ERR_MESR_LEVEL_H);			//
			}													//�[���������̃G���[�R�[�h�͉������Ȃ�
		}

		if(((err_status_buf & ERR_JUDGE_AGC) != 0)				//AGC�s�\//
			&&((err_status & ERR_JUDGE_AGC) == 0)){
			MAIN[pch].err_judge_time[3] = 0;					//�G���[�������ԃN���A
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//���ʑ��莞�̃G���[
				remove_errcode(pch, ERR_MESR_AGC_L);			//�G���[�R�[�h����
				remove_errcode(pch, ERR_MESR_AGC_H);			//
			}													//�[���������̃G���[�R�[�h�͉������Ȃ�
		}

		if(((err_status_buf & ERR_JUDGE_REVERSE) != 0)			//�t���ُ�
			&&((err_status & ERR_JUDGE_REVERSE) == 0)){
			MAIN[pch].err_judge_time[4] = 0;					//�G���[�������ԃN���A
			remove_errcode(pch, ERR_MESR_REVERSE);				//�G���[�R�[�h����
		}

		if(((err_status_buf & ERR_JUDGE_WAVE) != 0)			//�g�`����
			&&((err_status & ERR_JUDGE_WAVE) == 0)){
			MAIN[pch].err_judge_time[5] = 0;					//�G���[�������ԃN���A
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//���ʑ��莞�̃G���[
				remove_errcode(pch, ERR_MESR_WAVE_L);			//�G���[�R�[�h����
				remove_errcode(pch, ERR_MESR_WAVE_H);			//
			}													//�[���������̃G���[�R�[�h�͉������Ȃ�
		}

		if(((err_status_buf & ERR_JUDGE_OVERFLOW) != 0)		//�I�[�o�[�t���[
			&&((err_status & ERR_JUDGE_OVERFLOW) == 0)){
			MAIN[pch].err_judge_time[6] = 0;					//�G���[�������ԃN���A
			remove_errcode(pch, ERR_MESR_OVERFLOW);				//�G���[�R�[�h����
		}

		if(((err_status_buf & ERR_JUDGE_10POINT) != 0)			//10�_�f�[�^����
			&&((err_status & ERR_JUDGE_10POINT) == 0)){
			remove_errcode(pch, ERR_10POINT);					//�G���[�R�[�h����
		}
		if(((err_status_buf & ERR_JUDGE_EEPROM) != 0)			//EEPROM�G���[
			&&((err_status & ERR_JUDGE_EEPROM) == 0)){
			remove_errcode(pch, ERR_EEPROM);					//�G���[�R�[�h����
		}
		if(((err_status_buf & ERR_JUDGE_CUNET) != 0)			//CUnet�G���[
			&&((err_status & ERR_JUDGE_CUNET) == 0)){
			remove_errcode(pch, ERR_CUNET);						//�G���[�R�[�h����
		}
		
		MAIN[pch].err_judge = MES[pch].err_status;				//���ʃG���[���X�V
		
	}

	if(MAIN[pch].led_err_status == 0){		//�G���[�X�e�[�^�X�N���A
		if(reset_factor == RESTART_WDOG){	//�ċN���G���[�����������ꍇ
			MAIN[pch].com_err_status = ERR_RESTART;	//�ċN���G���[�Z�b�g	
		}
	}

	alm_status = MES[pch].alm_status;			//�����ʌx�����
	alm_status_buf = MAIN[pch].alm_judge;	//���ʌx�����o�b�t�@
	if(alm_status_buf != alm_status){			//���ʌx�����ω�����
		//���ʌx�����r�b�g��0��1�ɕω������ꍇ
		if(((alm_status_buf & ALM_JUDGE_EMPTY) == 0)	//�G���v�e�B�Z���T�[
				&&((alm_status & ALM_JUDGE_EMPTY) != 0)){
			MAIN[pch].alm_judge_time[0] = cmi_count;		//�x���������ԓo�^
		}

		//���ʌx�����r�b�g��1��0�ɕω������ꍇ
		if(((alm_status_buf & ALM_JUDGE_EMPTY) != 0)	//�G���v�e�B�Z���T�[
				&&((alm_status & ALM_JUDGE_EMPTY) == 0)){
			MAIN[pch].alm_judge_time[0] = 0;						//�x���������ԃN���A
		}

		MAIN[pch].alm_judge = MES[pch].alm_status;		//�x�����X�V
	}

	if((alm_status & ALM_JUDGE_GAIN) != 0){		//�A���v�Q�C���}��
		log_save(pch, ALM_MESR_GAIN);		//�G���[�X�e�[�^�X�̓o�^�̂݁i�G���[���O�͓o�^���Ȃ��j		
	}

	total_status = MES[pch].total_status;		//�ώZ�Ď��@�\���
	total_status_buf = MAIN[pch].total_judge;	//�ώZ�Ď��@�\���o�b�t�@
	
	//�ώZ�Ď��@�\���r�b�g��0��1�ɕω������ꍇ
	if(((total_status_buf & TTL_JUDGE_OVERFLOW) == 0)		//�ώZ�l�I�[�o�[�t���[
		&&((total_status & TTL_JUDGE_OVERFLOW) != 0)){
		log_save(pch, TTL_OVERFLOW);
	}else if((total_status & TTL_JUDGE_OVERFLOW) != 0){
		err_priority(pch, TTL_OVERFLOW);				//�G���[�X�e�[�^�X��ON����
	}else{
		;
	}
	if((total_status & TTL_JUDGE_CACL_ERR) != 0){			//�ώZ�l���Z�ُ�
		log_save(pch, TTL_CACL_ERR);
		MES[pch].total_status &= ~TTL_JUDGE_CACL_ERR;		//�N���A
	}
	MAIN[pch].total_judge = MES[pch].total_status;		//�ώZ�Ď��@�\���X�V
	
	//�G���[���̔���
	err_status = MES_SUB[pch].err_status_sub;	//�G���[���
	err_status_buf = MAIN[pch].err_sub_judge;	//�G���[���o�b�t�@
	if(err_status_buf != err_status){			//�G���[���ω�����
		//�G���[���r�b�g��0��1�ɕω������ꍇ
		if(((err_status_buf & ERR_JUDGE_DEVICE) == 0)	//�������f�o�C�X�ُ�
			&&((err_status & ERR_JUDGE_DEVICE) != 0)){
			log_save(pch, ERR_DEVICE);			//�G���[���O���̓o�^
		}
		//�G���[���r�b�g��1��0�ɕω������ꍇ
		if(((err_status_buf & ERR_JUDGE_DEVICE) != 0)	//�������f�o�C�X�ُ�
			&&((err_status & ERR_JUDGE_DEVICE) == 0)){
			remove_errcode(pch, ERR_DEVICE);	//�G���[�R�[�h����
		}
	}
	MAIN[pch].err_sub_judge = MES_SUB[pch].err_status_sub;	//�G���[���̍X�V
}

/****************************************************/
/* Function : err_judge_holdtime                    */
/* Summary  : �G���[�z�[���h�^�C�����菈��    				*/
/* Argument : pch                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	err_judge_holdtime(short pch){

	short i_cnt;
	unsigned long hold_time;
	unsigned long reverse_time;
	unsigned long judge_time[7], alm_judge_time[3];
	unsigned long alarm_time;

	if(pch >= CH_NUMMAX){
		return;
	}
	
	if((MES[pch].err_status & ERR_BURN_OUT) == 0){	//�o�[���A�E�g�ΏۃG���[����
		MES[pch].err_burn_out = B_OFF;					//�o�[���A�E�g���Ȃ�
	}
	if(MES[pch].err_status == 0){						//�G���[����
		MES[pch].err_hold_out = B_OFF;					//�z�[���h�A�E�g�N���A
	}

	hold_time = (unsigned long)SVD[pch].err_hold_time;				//�G���[�z�[���h�^�C��(1sec��1�ŕ\���j
	reverse_time = (unsigned long)SVD[pch].reverse_time;		//�t�����莞��(1.0sec��10�ŕ\��)
	alarm_time = (unsigned long)SVD[pch].alm_hold_time;			//�x���o�͎���(0.01sec��1�ŕ\���j
	//���Ԃ�5msec�P�ʂɕϊ�����
	hold_time *= 200;
	reverse_time *= 20;
	alarm_time *= 2;
	
	for(i_cnt=0; i_cnt<7; i_cnt++){
		judge_time[i_cnt] = MAIN[pch].err_judge_time[i_cnt];
	}
	for(i_cnt=0; i_cnt<3; i_cnt++){
		alm_judge_time[i_cnt] = MAIN[pch].alm_judge_time[i_cnt];
	}

	if(judge_time[4] != 0){											//�t���ُ�
		if(B_YES == util_passed_time(judge_time[4], reverse_time)){	//�t�����莞�Ԍo�ߊm�F
			MAIN[pch].err_judge_time[4] = 0;
			MES[pch].err_burn_out = B_OFF;								//�o�[���A�E�g���Ȃ�
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
			log_save(pch, ERR_MESR_REVERSE);							//�G���[���O���̓o�^		
		}
	}

	if(judge_time[5] != 0){										//�g�`����
		if(B_YES == util_passed_time(judge_time[5], hold_time)){	//�G���[�z�[���h�^�C���o�ߊm�F
			MAIN[pch].err_judge_time[5] = 0;
			MES[pch].err_burn_out = B_OFF;							//�o�[���A�E�g���Ȃ�
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
			log_save(pch, ERR_MESR_WAVE_H);							//�G���[���O���̓o�^		
		}
	}	
	
	if(judge_time[0] != 0){										//�G���v�e�B�Z���T�[
		if(B_YES == util_passed_time(judge_time[0], hold_time)){	//�G���[�z�[���h�^�C���o�ߊm�F
			MAIN[pch].err_judge_time[0] = 0;
			MES[pch].err_burn_out = B_ON;							//�o�[���A�E�g
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
			log_save(pch, ERR_MESR_EMPTY_H);						//�G���[���O���̓o�^		
		}
	}

	if(judge_time[1] != 0){										//���Z�ُ�
		if(B_YES == util_passed_time(judge_time[1], hold_time)){	//�G���[�z�[���h�^�C���o�ߊm�F
			MAIN[pch].err_judge_time[1] = 0;
			MES[pch].err_burn_out = B_ON;							//�o�[���A�E�g
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
			log_save(pch, ERR_MESR_CALC_H);							//�G���[���O���̓o�^		
		}
	}

	if(judge_time[2] != 0){										//�g�`�A���o�����X
		if(B_YES == util_passed_time(judge_time[2], hold_time)){	//�G���[�z�[���h�^�C���o�ߊm�F
			MAIN[pch].err_judge_time[2] = 0;
			MES[pch].err_burn_out = B_ON;							//�o�[���A�E�g
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
			log_save(pch, ERR_MESR_LEVEL_H);						//�G���[���O���̓o�^		
		}
	}
	
	if(judge_time[3] != 0){										//AGC�s�\//
		if(B_YES == util_passed_time(judge_time[3], hold_time)){	//�G���[�z�[���h�^�C���o�ߊm�F
			MAIN[pch].err_judge_time[3] = 0;
			MES[pch].err_burn_out = B_ON;							//�o�[���A�E�g
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
			log_save(pch, ERR_MESR_AGC_H);							//�G���[���O���̓o�^		
		}
	}

	if(judge_time[6] != 0){										//�I�[�o�[�t���[
		if(B_YES == util_passed_time(judge_time[6], hold_time)){	//�G���[�z�[���h�^�C���o�ߊm�F
			MAIN[pch].err_judge_time[6] = 0;
			MES[pch].err_burn_out = B_ON;							//�o�[���A�E�g
			MES[pch].err_hold_out = B_ON;							//�z�[���h�A�E�g
			err_status_control(pch);			//�G���[�X�e�[�^�X�����i�ʐM�p�j
		}
	}

	if(alm_judge_time[0] != 0){					//�G���v�e�B�Z���T�[�i�x���j
		if(B_YES == util_passed_time(alm_judge_time[0], alarm_time)){	//�x���o�͎��Ԍo�ߊm�F
			log_save(pch, ALM_MESR_EMPTY);	//�G���[�X�e�[�^�X�̓o�^�̂݁i�G���[���O�͓o�^���Ȃ��j		
		}
	}
}

/****************************************************/
/* Function : err_status_control                    */
/* Summary  : �G���[�X�e�[�^�X�����i�펞�X�V�p�j    				*/
/* Argument : pch                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	err_status_control(short pch){

	unsigned short err_status;
	unsigned short com_err_status;
	unsigned short total_status;
	unsigned short condition_bit;
	unsigned short condition_bit_cu;

	if(pch >= CH_NUMMAX){
		return;
	}
	
	err_status = MES[pch].err_status;			//�����ʃG���[���
	com_err_status = MAIN[pch].com_err_status;
	total_status = MES[pch].total_status;
	condition_bit = 0;
	condition_bit_cu = 0;

	if((err_status & ERR_JUDGE_EMPTY) != 0){	//�G���v�e�B�Z���T
		condition_bit	 |= CON_EMPTY;
		if(MES[pch].err_hold_out == B_ON){		//CUnet�ʐM�̃A���[�������X�e�[�^�X�̓z�[���h�A�E�g����bit�𗧂Ă�
 		condition_bit_cu |= CON_EMPTY_CU;
		}
	}

	if((err_status & ERR_JUDGE_CALC) != 0){	//���Z�ُ�
		condition_bit	 |= CON_CALC;
		condition_bit_cu |= CON_CALC_CU;
	}

	if((err_status & ERR_JUDGE_AGC) != 0){		//AGC�s�\//
		condition_bit	 |= CON_GAIN_OV;
		condition_bit_cu |= CON_AGC_CU;
	}
	
	if((err_status & ERR_JUDGE_LEVEL) != 0){	//�g�`�A���o�����X
		condition_bit	 |= CON_LEVEL;
		condition_bit_cu |= CON_LEVEL_CU;
	}

	if((err_status & ERR_JUDGE_REVERSE) != 0){	//�t���ُ�
		condition_bit	 |= CON_REV;
		condition_bit_cu |= CON_REV_CU;
	}

	if((err_status & ERR_JUDGE_WAVE) != 0){	//�g�`����
		condition_bit	 |= CON_WAVE;
		condition_bit_cu |= CON_WAVE_CU;
	}

	if((err_status & ERR_JUDGE_OVERFLOW) != 0){//�I�[�o�[�t���[
		condition_bit_cu |= CON_OVER_CU;
	}

	if((total_status & TTL_JUDGE_REACH) != 0){//�ώZ�ڕW�l���B
		condition_bit_cu |= CON_TOTAL_CU;
	}
	
	if((err_status & ~ERR_JUDGE_ZERO) != 0){			//�G���[�L��
		if(MES[pch].err_hold_out == B_ON){		//�z�[���h�A�E�g
			condition_bit |= CON_HOLDOUT;		//�z�[���h�^�C���ȏ�
		}else{
			condition_bit |= CON_HOLD;			//�z�[���h�^�C������
		}
	}

	//�[���_�����G���[�L��
	if(com_err_status == ERR_ZERO_EMPTY || com_err_status == ERR_ZERO_WAVE ||
		com_err_status == ERR_ZERO_CALC || com_err_status == ERR_ZERO_LEVEL ||
		com_err_status == ERR_ZERO_AGC || com_err_status == ERR_ZERO_UNSTABLE){
		condition_bit |= CON_ZERO;			//�[���_�����G���[
	}
	
	MAIN[pch].err_condition = condition_bit;		//�G���[��ԍX�V
	MAIN[pch].err_condition_cu = condition_bit_cu;	//�G���[��ԍX�V
}

/****************************************************/
/* Function : action_status_control                 */
/* Summary  : ����X�e�[�^�X�����i�ʐM�p�j    				*/
/* Argument : pch,  act             	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	action_status_control(short pch, short act){

	if(pch >= CH_NUMMAX){
		return;
	}
	if(act >= ACT_STS_OTHER){
		return;
	}
	
	/* �e�X�g���[�h���͋����X�V */
	if(act == ACT_STS_NORMAL){
		if(MES[pch].test_enable != 0 || MES[pch].test_err_enable != 0){
			act = ACT_STS_TEST;
		}
	}
	MAIN[pch].com_act_status = act;	//����X�e�[�^�X�X�V
}

/****************************************************/
/* Function : action_status_check                 */
/* Summary  : ����X�e�[�^�X�m�F�i�ʐM�p�j    				*/
/* Argument : pch                  	               */
/* Return   : ����X�e�[�^�X			                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		action_status_check(short pch){

	short act;

	act = ACT_STS_NORMAL;	
	if(pch >= CH_NUMMAX){
		return(act);
	}

	act = MAIN[pch].com_act_status;	//����X�e�[�^�X

	return(act);
}

/****************************************************/
/* Function : pwon_count                 */
/* Summary  : �ċN���񐔂̍X�V    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	pwon_count(void){

	short ch;
	unsigned short	count;
	
	for(ch = CH1; ch < CH_NUMMAX; ch ++){
		count = SVD[ch].pwon_count;
		count ++;
		if(count > 32767){
			count = 1;
		}
		SVD[ch].pwon_count = count;
		eep_write_ch(ch, (short)(&SVD[ch].pwon_count - &SVD[ch].max_flow), count);
	}
}

/****************************************************/
/* Function : invert_data                 */
/* Summary  : �r�b�g���]    				*/
/* Argument : �f�[�^�i32�r�b�g�j           	               */
/* Return   : �r�b�g���]�f�[�^		                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
unsigned long invert_data(unsigned long data){
	
	return (data ^ 0xFFFFFFFF);
}

/****************************************************/
/* Function : delay                 */
/* Summary  : �f�B���C    				*/
/* Argument : delay_count           	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : delay_count : 10 �� delaytime : 1us
 ****************************************************/
void delay(unsigned short delay_count){

	volatile unsigned short counter;
	
	for(counter = 0; counter < delay_count; counter++){
		;
	}
}

/****************************************************/
/* Function : reset_factor_check                 */
/* Summary  : ���Z�b�g�����v���̊m�F    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	reset_factor_check(void){

	short	ch;
	short	reset_sts;
	uint32_t	ui32Causes;

	reset_sts = RESTART_NORMAL;

	// [TM4C] : ���̑��̏ꍇ�� RX �� FW �ɍ��킹�� RESTART_TERM ���Z�b�g 
	ui32Causes = SysCtlResetCauseGet();
	SysCtlResetCauseClear(ui32Causes);
	if((ui32Causes & SYSCTL_CAUSE_WDOG0) != 0){					//�E�H�b�`�h�b�O�^�C�}���Z�b�g
		reset_sts = RESTART_WDOG;
		end_wdt = 1;
	}else{
		if((ui32Causes & SYSCTL_CAUSE_WDOG1) != 0){					//�ċN���p�E�H�b�`�h�b�O�^�C�}���Z�b�g
			reset_sts = RESTART_NORMAL;
		}else{
			if((ui32Causes & SYSCTL_CAUSE_BOR) != 0){					//Brown-out reset 
				reset_sts = RESTART_POWER;
			}else{
				if((ui32Causes & SYSCTL_CAUSE_POR) != 0){	//�p���[�I�����Z�b�g		
					reset_sts = RESTART_NORMAL;	
				}else{								//�[�q���Z�b�g 
					reset_sts = RESTART_TERM;
				}
			}
		}
	}

	reset_factor = reset_sts;							//�ċN���v���ۑ�
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		if(reset_sts == RESTART_WDOG){
			MES[ch].err_status |= ERR_JUDGE_RESTART;	//�ċN���G���[�Z�b�g	
		}
	}
}

/****************************************************/
/* Function : reset_control                 */
/* Summary  : ���Z�b�g����(�Ɨ��E�H�b�`�h�b�O�^�C�}���g�p����) 				*/
/* Argument : �Ȃ�                   	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	reset_control(void){

 WatchdogResetDisable(WATCHDOG0_BASE);

	IntDisable(INT_TIMER0A);	/*����������ݒ�~*/

	WatchdogReloadSet_dl(WATCHDOG1_BASE, g_ui32SysClock / 1000);	// 1ms ��ɍċN�� 
	WatchdogResetEnable_dl(WATCHDOG1_BASE);

	while(1);

}

/****************************************************/
/* Function : non_sensor_control                 */
/* Summary  : �Z���T�����ݒ莞�̏���                 				*/
/* Argument : pch                   	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	non_sensor_control(short pch){

	short	i_cnt;

	if(MES[pch].test_enable != 0){		//�����G���[�o�̓��[�h�m�F
		 MES[pch].ml_min_now = (long)MES[pch].max_flow_long * (long)MES[pch].test_flow / 100;
	}	
	else MES[pch].ml_min_now = 0;				//�u�����ʃN���A
	
	MES[pch].err_status = 0;					//�G���[���N���A
	MES[pch].alm_status = 0;					//�x����N���A
	MES[pch].total_status = 0;					//�ώZ���N���A

	MAIN[pch].err_judge = 0;					//�G���[������N���A
	MAIN[pch].err_condition = 0;				//�G���[�X�e�[�^�X�N���A
 if(MAIN[pch].com_act_status != ACT_STS_WRITE){ //����X�e�[�^�X���ݒ菑���ݒ��ȊO�̏ꍇ
	 MAIN[pch].com_act_status = 0;				//����X�e�[�^�X�N���A�i�ʐM�p�j 
	}
	MAIN[pch].com_err_status = 0;				//�G���[�X�e�[�^�X�N���A �i�ʐM�p�j
	MAIN[pch].led_err_status = 0;				//�G���[�X�e�[�^�X�N���A(�\���p)
	for(i_cnt=0; i_cnt<6; i_cnt++){
		MAIN[pch].err_judge_time[i_cnt] = 0;	//�G���[�������ԃN���A
	}
}

/****************************************************/
/* Function : util_eep_allwrite                 */
/* Summary  : EEPROM��������                      				*/
/* Argument : pch,   opt            	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void util_eep_allwrite(short pch, short opt){

	short i_cnt;
	short ch_cnt;
	short ch;
	unsigned short *pt;
	unsigned short *pt1;
	unsigned short *pt2;

	if(pch >= CH_NUMMAX) return;

	ch = pch;  //������CH

 /*** ���[�U�p�����[�^ ***/
	if((opt & WR_USER_PARA) != 0){	 /*���[�U�p�����[�^������*/
		eep_write_ch_delay(ch, (short)(&SVD[ch].burnout - &SVD[ch].max_flow), SVD[ch].burnout);						//�o�[���A�E�g���
		eep_write_ch_delay(ch, (short)(&SVD[ch].burnout_value - &SVD[ch].max_flow), SVD[ch].burnout_value);	//�o�[���A�E�g���͒l		
		eep_write_ch_delay(ch, (short)(&SVD[ch].viscos - &SVD[ch].max_flow), SVD[ch].viscos);					   //���S�x
		eep_write_ch_delay(ch, (short)(&SVD[ch].viscos_auto - &SVD[ch].max_flow), SVD[ch].viscos_auto);		//���S�x�Œ�/����
		eep_write_ch_delay(ch, (short)(&SVD[ch].err_hold_time - &SVD[ch].max_flow), SVD[ch].err_hold_time);		//�G���[�z�[���h�^�C��
		eep_write_ch_delay(ch, (short)(&SVD[ch].reverse_time - &SVD[ch].max_flow), SVD[ch].reverse_time);	//�t�����莞��
		eep_write_ch_delay(ch, (short)(&SVD[ch].reverse_level - &SVD[ch].max_flow), SVD[ch].reverse_level);	//�t������臒l
	}
	if((opt & WR_USPAR_DEVICE) != 0){	 /*���[�U�p�����[�^������(�������f�o�C�X)*/ /*������޲�����Ǎ��񂾾ݻ��񂾂���EEPROM�ɏ����ނ��߁AWR_USER_PARA��WR_USPAR_DEVICE�ɕ���*/
		eep_write_ch_delay(ch, (short)(&SVD[ch].max_flow - &SVD[ch].max_flow), SVD[ch].max_flow);				//�t���X�P�[��
		eep_write_ch_delay(ch, (short)(&SVD[ch].unit - &SVD[ch].max_flow), SVD[ch].unit);				//�t���X�P�[�������_�ʒu�A�P��
		eep_write_ch_delay(ch, (short)(&SVD[ch].sensor_size - &SVD[ch].max_flow), SVD[ch].sensor_size);			//�Z���T���
		eep_write_ch_delay(ch, (short)(&SVD[ch].k_factor - &SVD[ch].max_flow), SVD[ch].k_factor);					//K�t�@�N�^		
		eep_write_ch_delay(ch, (short)(&SVD[ch].damp - &SVD[ch].max_flow), SVD[ch].damp);				//�_���s���O		
		eep_write_ch_delay(ch, (short)(&SVD[ch].low_cut - &SVD[ch].max_flow), SVD[ch].low_cut);					//���[�J�b�gOFF
	}

	/*** ���[�U���j�A���C�Y ***/
	if((opt & WR_USER_LINR) != 0){	 /*���[�U���j�A���C�Y������*/
	 eep_write_ch_delay(ch, (short)(&SVD[ch].uslnr_num - &SVD[ch].max_flow), SVD[ch].uslnr_num);				//�␳�_���A�����_�ʒu
	 pt1 = &SVD[ch].uslnr_out1.WORD.low;
	 pt2 = &SVD[ch].uslnr_in1.WORD.low;
	 for(i_cnt=1; i_cnt<=30; i_cnt++){
		 eep_write_ch_delay(ch, (short)(pt1 - &SVD[ch].max_flow), *pt1);		//�␳�o��
		 eep_write_ch_delay(ch, (short)(pt2 - &SVD[ch].max_flow), *pt2);  //�␳����
		 pt1++;		pt2++;
	 }
	}

 /*** ���[�J�p�����[�^ ***/
	if((opt & WR_MAKER_PARA) != 0){	 /*���[�J�p�����[�^������*/
	 eep_write_ch_delay(ch, (short)(&SVD[ch].drive_freq - &SVD[ch].max_flow), SVD[ch].drive_freq);		//�쓮���g��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].drive_pls - &SVD[ch].max_flow), SVD[ch].drive_pls);				//�쓮�p���X��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].search_sw - &SVD[ch].max_flow), SVD[ch].search_sw);				//���֒l�T�[�`
	 eep_write_ch_delay(ch, (short)(&SVD[ch].gain_step - &SVD[ch].max_flow), SVD[ch].gain_step);				//�Q�C������X�e�b�v��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sound_vel_filter - &SVD[ch].max_flow), SVD[ch].sound_vel_filter);		//�����t�B���^���萔	
	 eep_write_ch_delay(ch, (short)(&SVD[ch].fifo_ch_init - &SVD[ch].max_flow), SVD[ch].fifo_ch_init);			//FIFO CH �����l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sound_vel_sel - &SVD[ch].max_flow), SVD[ch].sound_vel_sel);		//�����Œ�E����ؑ�
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sound_vel_fix - &SVD[ch].max_flow), SVD[ch].sound_vel_fix);		//�����Œ�l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].hldt - &SVD[ch].max_flow), SVD[ch].hldt);						//�ُ�z�[���h���ԑI��
	 eep_write_ch_delay(0, (short)(&SVD[0].cunet_delay - &SVD[0].max_flow), SVD[0].cunet_delay);			//CUnet �đ��M�ҋ@����	���SCH����
	 eep_write_ch_delay(ch, (short)(&SVD[ch].wave_vth - &SVD[ch].max_flow), SVD[ch].wave_vth);				//�G���v�e�B�Z���T����臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].balance_level - &SVD[ch].max_flow), SVD[ch].balance_level);		//�g�`�A���o�����X臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].saturation_level - &SVD[ch].max_flow), SVD[ch].saturation_level);//AGC�s�\����臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].correlate_level - &SVD[ch].max_flow), SVD[ch].correlate_level);	//���Z�ُ픻��臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].correlate_time - &SVD[ch].max_flow), SVD[ch].correlate_time);	//���Z�ُ픻���
	 eep_write_ch_delay(ch, (short)(&SVD[ch].attenuate_level - &SVD[ch].max_flow), SVD[ch].attenuate_level);	//�g�`��������臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_wave_vth - &SVD[ch].max_flow), SVD[ch].alm_wave_vth);				//�G���v�e�B�Z���T�x��臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_gain_level - &SVD[ch].max_flow), SVD[ch].alm_gain_level);		//�A���v�Q�C���x��臒l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_gain_count - &SVD[ch].max_flow), SVD[ch].alm_gain_count);	//�A���v�Q�C���x������
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_hold_time - &SVD[ch].max_flow), SVD[ch].alm_hold_time);	//�x���o�͎���	
	 eep_write_ch_delay(ch, (short)(&SVD[ch].filter_mode - &SVD[ch].max_flow), SVD[ch].filter_mode);			//�t�B���^���[�h
	 eep_write_ch_delay(ch, (short)(&SVD[ch].filter_avg - &SVD[ch].max_flow), SVD[ch].filter_avg);			//�ړ����ϒl
	 eep_write_ch_delay(ch, (short)(&SVD[ch].LL_enable - &SVD[ch].max_flow), SVD[ch].LL_enable);			//��t���j�A���C�Y���[�h�I��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].LL_kind - &SVD[ch].max_flow), SVD[ch].LL_kind);				//��t���j�A���C�Y���[�h�E��t���
	 eep_write_ch_delay(ch, (short)(&SVD[ch].total_offset_enable - &SVD[ch].max_flow), SVD[ch].total_offset_enable);	//�ώZ�I�t�Z�b�g
	 eep_write_ch_delay(ch, (short)(&SVD[ch].total_offset_value - &SVD[ch].max_flow), SVD[ch].total_offset_value);	//�ώZ�I�t�Z�b�g�l
		eep_write_ch_delay(ch, (short)(&SVD[ch].drive_search - &SVD[ch].max_flow), SVD[ch].drive_search);	//�������[�h
		eep_write_ch_delay(ch, (short)(&SVD[ch].start_freq - &SVD[ch].max_flow), SVD[ch].start_freq);	//�T�[�`�J�n���g��
		eep_write_ch_delay(ch, (short)(&SVD[ch].stop_freq - &SVD[ch].max_flow), SVD[ch].stop_freq);	//�T�[�`��~���g��
#if defined(FRQSCH)
	eep_write_ch_delay(ch, (short)(&SVD[ch].SchFrq - &SVD[ch].max_flow), SVD[ch].SchFrq);	//�T�[�`���ē������g��
#endif
	 eep_write_ch_delay(ch, (short)(&SVD[ch].target_total.DT_2BYTE.low  - &SVD[ch].max_flow), SVD[ch].target_total.DT_2BYTE.low);	//�ώZ�ڕW�l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].target_total.DT_2BYTE.high - &SVD[ch].max_flow), SVD[ch].target_total.DT_2BYTE.high);	//
	 eep_write_ch_delay(ch, (short)(&SVD[ch].damp_mode - &SVD[ch].max_flow), SVD[ch].damp_mode);				//�C�A�΍􃂁[�h
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2d - &SVD[ch].max_flow), SVD[ch].rl2d);						//D Rate Limit �_���s���O�{��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2hc - &SVD[ch].max_flow), SVD[ch].rl2hc);				//D Rate Limit �z�[���h�N���A
	 eep_write_ch_delay(ch, (short)(&SVD[ch].odpd - &SVD[ch].max_flow), SVD[ch].odpd);						//���C�g���~�b�g1st
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl1tg - &SVD[ch].max_flow), SVD[ch].rl1tg);				//D Rate Limit 1st �ڕW
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl1av - &SVD[ch].max_flow), SVD[ch].rl1av);				//D Rate limit 1st ����
	 eep_write_ch_delay(ch, (short)(&SVD[ch].odpl - &SVD[ch].max_flow), SVD[ch].odpl);						//���C�g���~�b�g2nd
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2tg - &SVD[ch].max_flow), SVD[ch].rl2tg);				//D Rate limit 2nd �ڕW
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2av - &SVD[ch].max_flow), SVD[ch].rl2av);				//D Rate limit 2nd ����
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rlt - &SVD[ch].max_flow), SVD[ch].rlt);							//���C�g���~�b�g
	 eep_write_ch_delay(ch, (short)(&SVD[ch].dump_var - &SVD[ch].max_flow), SVD[ch].dump_var);				//�σ_���s���O�΍��l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].dump_mul - &SVD[ch].max_flow), SVD[ch].dump_mul);				//�σ_���s���O�{��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].inc - &SVD[ch].max_flow), SVD[ch].inc);							//�ُ�z�[���h����
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_up.WORD.low - &SVD[ch].max_flow), SVD[ch].corr_up.WORD.low);		//���֒l������l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_up.WORD.high - &SVD[ch].max_flow), SVD[ch].corr_up.WORD.high);		//
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_low.WORD.low - &SVD[ch].max_flow), SVD[ch].corr_low.WORD.low);		//���֒l�������l
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_low.WORD.high - &SVD[ch].max_flow), SVD[ch].corr_low.WORD.high);	//

		//�]���p
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_option - &SVD[ch].max_flow), SVD[ch].sns_option); //�Z���T�I�v�V����
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_disL - &SVD[ch].max_flow), SVD[ch].sns_disL); //�Z���T�ԋ���(L)
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_disL_l - &SVD[ch].max_flow), SVD[ch].sns_disL_l); //�Z���T�ԋ���(L-l)
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_tau - &SVD[ch].max_flow), SVD[ch].sns_tau);		//���ʎ���
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_coef - &SVD[ch].max_flow), SVD[ch].sns_coef);			//�݊��W��
	 eep_write_ch_delay(ch, (short)(&SVD[ch].adc_clock - &SVD[ch].max_flow), SVD[ch].adc_clock);			//ADC�N���b�N
	 eep_write_ch_delay(ch, (short)(&SVD[ch].wind_offset - &SVD[ch].max_flow), SVD[ch].wind_offset);			//WINDOW�I�t�Z�b�g
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sum_start - &SVD[ch].max_flow), SVD[ch].sum_start);			//�������֊J�n�ʒu
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sum_end - &SVD[ch].max_flow), SVD[ch].sum_end);			//�������֏I���ʒu
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sum_step - &SVD[ch].max_flow), SVD[ch].sum_step);			//�������֊Ԋu

	 eep_write_ch_delay(ch, (short)(&SVD[ch].fix_data - &SVD[ch].max_flow), SVD[ch].fix_data);			//�Œ�l�ݒ�
	 eep_write_ch_delay(ch, (short)(&SVD[ch].fix_amp_gain_rev - &SVD[ch].max_flow), SVD[ch].fix_amp_gain_rev);			//Wiper Position(�Q�C���l)
	 eep_write_ch_delay(ch, (short)(&SVD[ch].fix_fifo_ch_read - &SVD[ch].max_flow), SVD[ch].fix_fifo_ch_read);			//FIFO CH
	 eep_write_ch_delay(ch, (short)(&SVD[ch].fix_fifo_no_read - &SVD[ch].max_flow), SVD[ch].fix_fifo_no_read);			//Leading Position
	 eep_write_ch_delay(ch, (short)(&SVD[ch].ZerCrsSttPnt - &SVD[ch].max_flow), SVD[ch].ZerCrsSttPnt);			//Zero Cross Start point
	 eep_write_ch_delay(ch, (short)(&SVD[ch].ZerCrsUseNum - &SVD[ch].max_flow), SVD[ch].ZerCrsUseNum);			//Zero Cross Use Number
	 
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltSwc - &SVD[ch].max_flow), SVD[ch].DgtFltSwc);		//Digital Filter Switch
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA00 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA00);	//Degital Filter Coefficient A00
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA01 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA01);	//Degital Filter Coefficient A01
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA10 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA10);	//Degital Filter Coefficient A10
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA11 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA11);	//Degital Filter Coefficient A11
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA20 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA20);	//Degital Filter Coefficient A20
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA21 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA21);	//Degital Filter Coefficient A21
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA30 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA30);	//Degital Filter Coefficient A30
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA31 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA31);	//Degital Filter Coefficient A31
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA40 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA40);	//Degital Filter Coefficient A40
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefA41 - &SVD[ch].max_flow), SVD[ch].DgtFltCefA41);	//Degital Filter Coefficient A41
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB00 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB00);	//Degital Filter Coefficient B00
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB01 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB01);	//Degital Filter Coefficient B01
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB10 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB10);	//Degital Filter Coefficient B10
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB11 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB11);	//Degital Filter Coefficient B11
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB20 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB20);	//Degital Filter Coefficient B20
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB21 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB21);	//Degital Filter Coefficient B21
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB30 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB30);	//Degital Filter Coefficient B30
	 eep_write_ch_delay(ch, (short)(&SVD[ch].DgtFltCefB31 - &SVD[ch].max_flow), SVD[ch].DgtFltCefB31);	//Degital Filter Coefficient B31
	 //�]���p
 }

	/*** ���[�J���j�A���C�Y ***/
	if((opt & WR_MAKER_LINR) != 0){	 /*���[�J���j�A���C�Y������*/
	 eep_write_ch_delay(ch, (short)(&SVD[ch].mklnr_num - &SVD[ch].max_flow), SVD[ch].mklnr_num);			//�␳�_��
	 pt1 = &SVD[ch].mklnr_out1.WORD.low;
	 pt2 = &SVD[ch].mklnr_in1.WORD.low;
	 for(i_cnt=1; i_cnt<=30; i_cnt++){
		 eep_write_ch_delay(ch, (short)(pt1 - &SVD[ch].max_flow), *pt1);		//�␳�o��
		 eep_write_ch_delay(ch, (short)(pt2 - &SVD[ch].max_flow), *pt2);  //�␳����
		 pt1++;	pt2++;
	 }
	}

	/*** �ϊ���V���A���i���o�[ ***/
	if((opt & WR_CVT_SERIAL) != 0){	 /*�ϊ���V���A���i���o�[������*/
	 for(ch_cnt = 0; ch_cnt < 6; ch_cnt++){
		 pt = &SVD[ch_cnt].c_serial[0];
		 for(i_cnt=1; i_cnt<=8; i_cnt++){
			 eep_write_ch_delay(ch_cnt, (short)(pt - &SVD[ch_cnt].max_flow), *pt);
			 pt++;
		 }
	 }
	}

	/*** �Z���T�V���A���i���o�[ ***/
	if((opt & WR_SNS_SERIAL) != 0){	 /*�Z���T�V���A���i���o�[������*/
	 pt = &SVD[ch].s_serial[0];
	 for(i_cnt=1; i_cnt<=8; i_cnt++){
		 eep_write_ch_delay(ch, (short)(pt - &SVD[ch].max_flow), *pt);
		 pt++;
	 }
	}

}

/****************************************************
 * Function : util_SnsMem_Write
 * Summary  : �Z���T�������������ݗv��
 * Argument : pch, opt
 * Return   : �Ȃ�
 * Caution  : �Ȃ�
 * notes    : �Z���T�������������݃t���[
 *            phase_owwrite = 1 : OWwrite = 1 ����
 *                            2 : MAIN[pch].com_act_status = ACT_STS_NORMAL or ACT_STS_ADDIT ����
 *                            3 : �������f�o�C�X�̑Ώۃ`�����l��������
 *                                & �������݃f�[�^������ΐ�p�z��ɃR�s�[����
 *                            4 : CRC���v�Z����
 *                            5 : �������f�o�C�X�ɏ�������
 *                            6 : �������񂾃f�[�^�̏ƍ�
 *                            7 : �����I�� & �e�ϐ��̏�����
 ****************************************************/
void util_SnsMem_Write(short pch, short opt)
{
	if ((opt & WR_DEVICE) != 0)
	{
		OWwrite[pch] = 1; // �������f�o�C�X�����ݗv��
	}
}

/****************************************************
 * Function : SavEepZerAdjPrm (Save Eeprom Zero Adjust Parameter)
 * Summary  : �[�������֘A�p�����[�^��EEPROM�ۑ�
 * Argument : pch : �`�����l���ԍ�
 * Return   : None
 * Caution  : None
 * Notes    : 
 ****************************************************/
void SavEepZerAdjPrm(short pch)
{
    short i;

	if(pch >= CH_NUMMAX) return;
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_qat.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_flow_qat.WORD.low);   /*�[���_�������F����*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_qat.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_flow_qat.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_vel.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_flow_vel.WORD.low);   /*�[���_�������F����*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_vel.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_flow_vel.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_sound_spd - &SVD[pch].max_flow), SVD[pch].zero_sound_spd);                  	/*�[���_�������F����*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.low1 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.low1);					  /*�[���_�������F�ώZ�l*/	
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.low2 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.low2);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.high1 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.high1);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.high2 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.high2);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_wave_max - &SVD[pch].max_flow), SVD[pch].zero_wave_max);		                  /*�[���_�������F��g�g�`�ő�l*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_wave_min - &SVD[pch].max_flow), SVD[pch].zero_wave_min);		                  /*�[���_�������F��g�g�`�ŏ��l*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_delta_ts.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_delta_ts.WORD.low);  /*�[���_�������F�`�����ԍ�*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_delta_ts.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_delta_ts.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_correlate - &SVD[pch].max_flow), SVD[pch].zero_correlate);	      /*�[���_�������F���֒l��*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_zero_offset - &SVD[pch].max_flow), SVD[pch].zero_zero_offset);	  /*�[���_�������F�[���_�I�t�Z�b�g*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_condition - &SVD[pch].max_flow), SVD[pch].zero_condition);       /*�[���_�������F�R���f�B�V����*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_fifo_pos - &SVD[pch].max_flow), SVD[pch].zero_fifo_pos);  /*�[���_�������FFIFO��g�g�`���o�ʒu*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_gain_1st - &SVD[pch].max_flow), SVD[pch].zero_gain_1st);  /*�[���_�������F�A���v�Q�C���l(1st)*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_gain_2nd - &SVD[pch].max_flow), SVD[pch].zero_gain_2nd);		/*�[���_�������F�A���v�Q�C���l(2nd)*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_fifo_ch - &SVD[pch].max_flow), SVD[pch].zero_fifo_ch);		  /*�[���_�������FFIFO CH*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_p1p2 - &SVD[pch].max_flow), SVD[pch].zero_p1p2);          /*�[���_�������F��g�̍�(P1-P2)*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_FwdTimDif.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_FwdTimDif.WORD.low);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_FwdTimDif.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_FwdTimDif.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_RevTimDif.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_RevTimDif.WORD.low);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_RevTimDif.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_RevTimDif.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_FwdSurplsTim - &SVD[pch].max_flow), SVD[pch].zero_FwdSurplsTim);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_RevSurplsTim - &SVD[pch].max_flow), SVD[pch].zero_RevSurplsTim);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_drive_freq - &SVD[pch].max_flow), SVD[pch].zero_drive_freq);
	for(i=0; i<WAV_PEK_NUM; i++){
		eep_write_ch_delay(pch, (short)(&SVD[pch].zero_FwdWavPekPosLst[i] - &SVD[pch].max_flow), SVD[pch].zero_FwdWavPekPosLst[i]);
		eep_write_ch_delay(pch, (short)(&SVD[pch].zero_FwdWavPekValLst[i] - &SVD[pch].max_flow), SVD[pch].zero_FwdWavPekValLst[i]);
		eep_write_ch_delay(pch, (short)(&SVD[pch].zero_RevWavPekPosLst[i] - &SVD[pch].max_flow), SVD[pch].zero_RevWavPekPosLst[i]);
		eep_write_ch_delay(pch, (short)(&SVD[pch].zero_RevWavPekValLst[i] - &SVD[pch].max_flow), SVD[pch].zero_RevWavPekValLst[i]);
	}
}

/****************************************************/
/* Function : util_eep_zerowrite                   */
/* Summary  : EEPROM�������݁i�[�������l�j             		*/
/* Argument : pch                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void util_eep_zerowrite(short pch){

	short i_cnt;

	if(pch >= CH_NUMMAX) return;

	// EEPROM��������
	SavEepZerAdjPrm(pch);
	
	SVD[pch].zero_offset = SVD[pch].zero_zero_offset;
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)SVD[pch].zero_offset);		/*�[���_�I�t�Z�b�g*/
}

/****************************************************/
/* Function : read_serial_num                       */
/* Summary  : �V���A���ԍ��Ǎ���    				*/
/* Argument : pch                               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ϊ���V���A���i���o�[�A �Z���T�V���A���i���o�[     */
/****************************************************/
void	read_serial_num(short pch){

 //�ϊ���V���A���i���o�[
	MAIN[pch].cvt_serial[0] = eep_read((short)(&SVD[pch].c_serial[0] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[1] = eep_read((short)(&SVD[pch].c_serial[1] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[2] = eep_read((short)(&SVD[pch].c_serial[2] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[3] = eep_read((short)(&SVD[pch].c_serial[3] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[4] = eep_read((short)(&SVD[pch].c_serial[4] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[5] = eep_read((short)(&SVD[pch].c_serial[5] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[6] = eep_read((short)(&SVD[pch].c_serial[6] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[7] = eep_read((short)(&SVD[pch].c_serial[7] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);

 //�Z���T�V���A���i���o�[
	MAIN[pch].sns_serial[0] = eep_read((short)(&SVD[pch].s_serial[0] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[1] = eep_read((short)(&SVD[pch].s_serial[1] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[2] = eep_read((short)(&SVD[pch].s_serial[2] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[3] = eep_read((short)(&SVD[pch].s_serial[3] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[4] = eep_read((short)(&SVD[pch].s_serial[4] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[5] = eep_read((short)(&SVD[pch].s_serial[5] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[6] = eep_read((short)(&SVD[pch].s_serial[6] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].sns_serial[7] = eep_read((short)(&SVD[pch].s_serial[7] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
}

/****************************************************/
/* Function : write_serial_num                       */
/* Summary  : �V���A���ԍ�������    				*/
/* Argument : pch                               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ϊ���V���A���i���o�[�A �Z���T�V���A���i���o�[     */
/****************************************************/
void	write_serial_num(short pch){

 //�ϊ���V���A���i���o�[
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[0] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[0]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[1] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[1]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[2] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[2]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[3] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[3]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[4] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[4]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[5] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[5]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[6] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[6]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[7] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[7]);

 //�Z���T�V���A���i���o�[
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[0] - &SVD[pch].max_flow), MAIN[pch].sns_serial[0]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[1] - &SVD[pch].max_flow), MAIN[pch].sns_serial[1]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[2] - &SVD[pch].max_flow), MAIN[pch].sns_serial[2]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[3] - &SVD[pch].max_flow), MAIN[pch].sns_serial[3]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[4] - &SVD[pch].max_flow), MAIN[pch].sns_serial[4]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[5] - &SVD[pch].max_flow), MAIN[pch].sns_serial[5]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[6] - &SVD[pch].max_flow), MAIN[pch].sns_serial[6]);
	eep_write_ch(pch, (short)(&SVD[pch].s_serial[7] - &SVD[pch].max_flow), MAIN[pch].sns_serial[7]);
}

/****************************************************/
/* Function : err_priority                        */
/* Summary  : �G���[�D�揇�ʂ̍X�V                    		*/
/* Argument : ch,  new_err_code     	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ���݃G���[�ƐV�G���[�̗D�揇�ʂ��r���āA       */
/*          : �V�G���[�̗D�揇�ʂ������ꍇ�ɃG���[�X�e�[�^�X���@�X�V����*/
/****************************************************/
void	err_priority(short ch, short new_err_code){

	short ret;
	
	//�ʐM�p
	ret = B_NO;
	if(MAIN[ch].com_err_status == 0){	//�G���[���o�^��
			ret = B_YES;
	}else{
		//�D�揇�ʂ̐����������������A�D�揇�ʂ�����
		if(err_inf[MAIN[ch].com_err_status].err_priority > err_inf[new_err_code].err_priority){
			ret = B_YES;
		}
	}
	if(ret == B_YES){
		MAIN[ch].com_err_status = new_err_code;	//�G���[�X�e�[�^�X�X�V	
	}

	//�G���[LED�p
	if(err_inf[new_err_code].err_led != 0){
		ret = B_NO;
		if(MAIN[ch].led_err_status == 0){	//�G���[���o�^��
				ret = B_YES;
		}else{
			//�D�揇�ʂ̐����������������A�D�揇�ʂ�����
			if(err_inf[MAIN[ch].led_err_status].err_priority > err_inf[new_err_code].err_priority){
				ret = B_YES;
			}
		}
		if(ret == B_YES){
			MAIN[ch].led_err_status = new_err_code;	//�G���[�X�e�[�^�X�X�V	
		}
	}
}
	
/****************************************************/
/* Function : remove_errcode                       */
/* Summary  : �G���[�R�[�h�̉���    				*/
/* Argument : ch,  err_code                         */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	remove_errcode(short ch, short err_code){

	if(MAIN[ch].led_err_status == 0){			//�G���[��������
		return;
	}	
	
	//�D�揇�ʂ̐����������������A�D�揇�ʂ�����
	if(err_inf[MAIN[ch].led_err_status].err_priority >= err_inf[err_code].err_priority){
		MAIN[ch].led_err_status = (short)0;		//�G���[�X�e�[�^�X�N���A
	}
}

/****************************************************/
/* Function : err_zero_status                       */
/* Summary  : �[���_�����G���[�L���m�F    				*/
/* Argument : err_status                            */
/* Return   : 0=�G���[�Ȃ�, 1=�G���[����                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		err_zero_status(short err_status){

	short	ret;
	unsigned short com_err_status;

	ret = B_NO;
	com_err_status = (unsigned short)err_status;

	if(com_err_status == ERR_ZERO_EMPTY || com_err_status == ERR_ZERO_WAVE ||
		com_err_status == ERR_ZERO_CALC || com_err_status == ERR_ZERO_LEVEL ||
		com_err_status == ERR_ZERO_AGC || com_err_status == ERR_ZERO_UNSTABLE){
		ret = B_YES;			//�[���_�����G���[�L��
	}

	return (ret);
}

/****************************************************/
/* Function : err_total_status                      */
/* Summary  : �ώZ�Ď��G���[�L���m�F    				*/
/* Argument : err_status                           */
/* Return   : 0=�G���[�Ȃ�, 1=�G���[����                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		err_total_status(short err_status){

	short	ret;
	unsigned short com_err_status;

	ret = B_NO;
	com_err_status = (unsigned short)err_status;

	if(com_err_status == TTL_CACL_ERR || com_err_status == TTL_OVERFLOW){		//�ώZ�l���Z�ُ� or �ώZ�l�I�[�o�[�t���[
		ret = B_YES;			//�G���[�L��
	}

	return (ret);
}

/****************************************************/
/* Function : get_total_offset                      */
/* Summary  : �ώZ�l�I�t�Z�b�g�̎擾    				*/
/* Argument : ch,  val_total                       */
/* Return   : �ώZ�l�I�t�Z�b�g                          */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
unsigned long long get_total_offset(short ch, unsigned long long val_total){

	unsigned long long val_result;

	val_result = val_total;
	if(SVD[ch].total_offset_enable == 1){	//�ώZ�l�I�t�Z�b�g�L��
		if(((long long)val_total + (long long)SVD[ch].total_offset_value * 10000) < 0){		//�ώZ�I�t�Z�b�g���Z���ʂ�����
			val_result = 0;
		}else if(((long long)val_total + (long long)SVD[ch].total_offset_value * 10000) <= 999999999990000){		//�ώZ�I�t�Z�b�g���Z���ʂ��ώZ�l�J�E���^�ȓ�
			val_result = val_total + ((long long)SVD[ch].total_offset_value * 10000);	//�ώZ�l�I�t�Z�b�g�l�����Z
		}else{
			;
		}
	}

	return val_result;
}

/****************************************************/
/* Function : RoundFunc								*/
/* Summary  : �����_�ȉ��̎l�̌ܓ�						*/
/* Argument : src									*/
/* Return   : �l�̌ܓ�����							*/
/* Caution  : �Ȃ�									*/
/* notes    : �����_1���ڂ̎l�̌ܓ����s��				*/
/****************************************************/
float RoundFunc(float src){

	return ( src >= 0 ) ? src + 0.5 : src - 0.5 ;
}

/****************************************************/
/* Function : debug_mode                      */
/* Summary  : �����G���[����i�f�o�b�O���[�h�j    				*/
/* Argument : pch                                 */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	debug_mode(short pch){

	short e;
	
//	if(MES[pch].test_enable == 0){					//�f�o�b�O���[�h�m�F
//		return;
//	}

//	MES[pch].err_status = MES[pch].test_err_code;	//�����G���[�R�[�h�Z�b�g
//	MES[pch].err_status &=(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_CALC+ERR_JUDGE_REVERSE+ERR_JUDGE_WAVE
//							+ERR_JUDGE_OVERFLOW+ERR_JUDGE_EEPROM+ERR_JUDGE_CUNET+ERR_JUDGE_RESTART	+ERR_JUDGE_ZERO);

	if(MES[pch].test_err_enable == 0){		//�����G���[�o�̓��[�h�m�F
		return;
	}
	e = MES[pch].test_err_code;

	switch(e){
		case  1:	MES[pch].err_status = ERR_JUDGE_EMPTY + ERR_JUDGE_ZERO;
					MAIN[pch].led_err_status = ERR_ZERO_EMPTY;		break;
		case  2:	MES[pch].err_status = ERR_JUDGE_WAVE + ERR_JUDGE_ZERO;
					MAIN[pch].led_err_status = ERR_ZERO_WAVE;		break;
		case  3:	MES[pch].err_status = ERR_JUDGE_CALC + ERR_JUDGE_ZERO;
					MAIN[pch].led_err_status = ERR_ZERO_CALC;		break;
		case  4:	MES[pch].err_status = ERR_JUDGE_LEVEL + ERR_JUDGE_ZERO;
					MAIN[pch].led_err_status = ERR_ZERO_LEVEL;		break;
		case  5:	MES[pch].err_status = ERR_JUDGE_AGC + ERR_JUDGE_ZERO;
					MAIN[pch].led_err_status = ERR_ZERO_AGC;		break;
		case  6:	MES[pch].err_status = ERR_JUDGE_EMPTY;
					MAIN[pch].led_err_status = ERR_MESR_EMPTY_L;	break;
		case  7:	MES[pch].err_status = ERR_JUDGE_EMPTY;
					MAIN[pch].led_err_status = ERR_MESR_EMPTY_H;	break;
		case  8:	MES[pch].err_status = ERR_JUDGE_WAVE;
					MAIN[pch].led_err_status = ERR_MESR_WAVE_L;		break;
		case  9:	MES[pch].err_status = ERR_JUDGE_WAVE;
					MAIN[pch].led_err_status = ERR_MESR_WAVE_H;		break;
		case 10:	MES[pch].err_status = ERR_JUDGE_CALC;
					MAIN[pch].led_err_status = ERR_MESR_CALC_L;		break;
		case 11:	MES[pch].err_status = ERR_JUDGE_CALC;
					MAIN[pch].led_err_status = ERR_MESR_CALC_H;		break;
		case 12:	MES[pch].err_status = ERR_JUDGE_LEVEL;
					MAIN[pch].led_err_status = ERR_MESR_LEVEL_L;	break;
		case 13:	MES[pch].err_status = ERR_JUDGE_LEVEL;
					MAIN[pch].led_err_status = ERR_MESR_LEVEL_H;	break;
		case 14:	MES[pch].err_status = ERR_JUDGE_AGC;
					MAIN[pch].led_err_status = ERR_MESR_AGC_L;		break;
		case 15:	MES[pch].err_status = ERR_JUDGE_AGC;
					MAIN[pch].led_err_status = ERR_MESR_AGC_H;		break;
		case 16:	MES[pch].err_status = ERR_JUDGE_REVERSE;
					MAIN[pch].led_err_status = ERR_MESR_REVERSE;	break;
		case 20:	MES[pch].err_status = ERR_JUDGE_EEPROM;
					MAIN[pch].led_err_status = ERR_EEPROM;		break;
		case 22:	MES[pch].err_status = ERR_JUDGE_RESTART;
					reset_factor = RESTART_WDOG;
					MAIN[pch].led_err_status = ERR_RESTART;		break;
		case 23:	MES[pch].err_status = ERR_JUDGE_OVERFLOW;
					MAIN[pch].led_err_status = ERR_MESR_OVERFLOW;	break;
		case 24:	MES[pch].err_status = ERR_JUDGE_CUNET;
					MAIN[pch].led_err_status = ERR_CUNET;		break;
		case 25:	MES[pch].err_status = ERR_JUDGE_UNSTABLE + ERR_JUDGE_ZERO;
					MAIN[pch].led_err_status = ERR_ZERO_UNSTABLE;	break;
		default:	MES[pch].err_status = 0;
					MAIN[pch].led_err_status = 0;			break;
		
	}
	
	MAIN[pch].com_err_status = e;

}
