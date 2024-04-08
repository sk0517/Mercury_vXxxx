/***********************************************/
/* File Name : log.c		             									   */
/*	Summary   : �G���[���O����			                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>
#include <string.h>	
#include "define.h"
#include "SV_def.h"
#include "typedefine.h"
#include "defMES.h"
#include "defLOG.h"
#include "defMAIN.h"
#include "defSAVE.h"

/********************************************************/
/*	���W���[������`�֐�								*/
/********************************************************/
void	log_init(void);
void	log_detailinfo_init(short ch);
void	log_basicinfo_save(short ch, short code);
void	log_detailinfo_save(short ch, short code);
void	log_save(short ch, short code);

/********************************************************/
/*	���W���[���O��`�֐�								*/
/********************************************************/
extern void	eep_write_ch_delay(short, short, short);
extern short get_attenuator_gain(short pch);
extern void	err_priority(short ch, short new_err_code);
extern short check_queue(void);

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern unsigned long	cmi_count;


/****************************************************/
/* Function : log_init                    */
/* Summary  : �G���[���O�̏�����    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	log_init(void){

	short		log_ch;
	short		log_num;
	short		log_count;
	short		i_count;
	unsigned short	pwon_count;
	unsigned long	err_time;
	
	for(log_ch = CH1; log_ch < CH_NUMMAX; log_ch ++){		//�G���[���O�ڍ׏��̏�����
		log_detailinfo_init(log_ch);
	}
	
	for(log_ch = CH1; log_ch < CH_NUMMAX; log_ch ++){		//�G���[���O��o�^����z��ԍ�������
		SAVE[log_ch].log_save_num = 0;
		pwon_count = 0xFFFF;
		err_time = 0xFFFFFFFF;
		for(log_num = 0; log_num < LOGMAX; log_num ++){		
			if(SVD[log_ch].err_code[log_num] != 0){
				if(SVD[log_ch].err_pwon_count[log_num] <= pwon_count){
					if(SVD[log_ch].err_pwon_count[log_num] != pwon_count){
						err_time = 0xFFFFFFFF;
					}
					pwon_count = SVD[log_ch].err_pwon_count[log_num];
					if(SVD[log_ch].err_time[log_num].DWORD < err_time){
						err_time = SVD[log_ch].err_time[log_num].DWORD;
						SAVE[log_ch].log_save_num = log_num;		//�G���[���O�o�^�J�n�ԍ�
					}
				}
			}
		}
	}
	
	for(log_ch = CH1; log_ch < CH_NUMMAX; log_ch ++){		//�G���[���O��{���(EEPROM)���G���[���O�ڍ׏��(RAM)�ɃR�s�[
		log_count = 0;
		log_num = SAVE[log_ch].log_save_num;
		for(i_count = 0; i_count < LOGMAX; i_count ++){
			if(SVD[log_ch].err_code[log_num] != 0){		//�G���[���O��{���ɓo�^��񂠂�
				LOG_DETAIL[log_ch][log_count].err_code = SVD[log_ch].err_code[log_num];					//�G���[�R�[�h
				LOG_DETAIL[log_ch][log_count].err_pwon_count = SVD[log_ch].err_pwon_count[log_num];		//�G���[�������̓d���I���J�E���^
				LOG_DETAIL[log_ch][log_count].err_time = SVD[log_ch].err_time[log_num].DWORD;			//�G���[����������
				log_count ++;
			}
			if(log_num == (LOGMAX - 1)){					//�G���[���O�o�^�ԍ��̍X�V
				log_num = 0;
			}else{
				log_num ++;
			}
		}
	}
}

/****************************************************/
/* Function : log_detailinfo_init                   */
/* Summary  : �G���[���O�ڍ׏��̏�����    				*/
/* Argument : ch                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	log_detailinfo_init(short ch){

	short		log_ch;
	short		log_num;
	
	if(ch >= CH_NUMMAX){		//CH�w��`�F�b�N
		return;				//�������s��
	}
	log_ch = ch;

	for(log_num = 0; log_num < LOGMAX; log_num ++){
		memcpy(&LOG_DETAIL[log_ch][log_num], &LOG_DETAIL_DEF, sizeof(LOG_DETAIL[log_ch][log_num]));
	}
}

/****************************************************/
/* Function : log_basicinfo_save                   */
/* Summary  : �G���[���O��{���̓o�^    				*/
/* Argument : ch,  code            	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	log_basicinfo_save(short ch, short code){

	short		log_ch;
	short		log_num;
	
	if(ch >= CH_NUMMAX){		//CH�w��`�F�b�N
		return;				//�o�^�s��
	}
	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		return;						//�o�^�s��
	}

	log_ch = ch;
	log_num = SAVE[log_ch].log_save_num;		//�G���[���O�o�^�ԍ�

	SVD[log_ch].err_code[log_num] = code;						//�G���[�R�[�h
	SVD[log_ch].err_pwon_count[log_num] = SVD[ch].pwon_count;	//�G���[�������̓d���I���J�E���^
	SVD[log_ch].err_time[log_num].DWORD = cmi_count;			//�G���[�������̎���

	//EEPROM��������
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_code[log_num] - &SVD[log_ch].max_flow), SVD[log_ch].err_code[log_num]);
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_pwon_count[log_num] - &SVD[log_ch].max_flow), SVD[log_ch].err_pwon_count[log_num]);
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_time[log_num].WORD.low - &SVD[log_ch].max_flow), SVD[log_ch].err_time[log_num].WORD.low);
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_time[log_num].WORD.high - &SVD[log_ch].max_flow), SVD[log_ch].err_time[log_num].WORD.high);

	//�G���[���O�o�^�ԍ��̍X�V
	if(log_num == (LOGMAX - 1)){
		SAVE[log_ch].log_save_num = 0;
	}else{
		SAVE[log_ch].log_save_num ++;
	}
}

/****************************************************/
/* Function : log_detailinfo_save                   */
/* Summary  : �G���[���O�ڍ׏��̓o�^    				*/
/* Argument : ch,  code            	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	log_detailinfo_save(short ch, short code){

	short		log_ch;
	short		log_num;
	short		log_shift;
	short		shift_count;
	short		i_cnt;
	
	if(ch >= CH_NUMMAX){		//CH�w��`�F�b�N
		return;				//�o�^�s��
	}
	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		return;						//�o�^�s��
	}
	log_ch = ch;
	log_shift = 1;

	for(log_num = 0; log_num < LOGMAX; log_num ++){		//�G���[���O��o�^����z��ԍ�������
		if(LOG_DETAIL[log_ch][log_num].err_code == 0){
			log_shift = 0;
			break;
		}
	}
	if(log_num == LOGMAX){
		log_num--;
	}	
	if(log_shift != 0){														//�G���[���O�z����V�t�g���Ă���o�^����
		for(shift_count = 0; shift_count < (LOGMAX-1); shift_count ++){		//�G���[���O�z����V�t�g����
			memcpy(&LOG_DETAIL[log_ch][shift_count], &LOG_DETAIL[log_ch][shift_count+1], sizeof(LOG_DETAIL[log_ch][shift_count+1]));	
		}
	}		
		
	LOG_DETAIL[log_ch][log_num].err_code = code;									//�G���[�R�[�h
	LOG_DETAIL[log_ch][log_num].err_pwon_count = SVD[log_ch].pwon_count;			//�G���[�������̓d���I���J�E���^
	LOG_DETAIL[log_ch][log_num].err_time = cmi_count;								//�G���[����������
	LOG_DETAIL[log_ch][log_num].flow_quantity = (long)MES[log_ch].ml_min_now;		//����
	LOG_DETAIL[log_ch][log_num].flow_velocity = (long)MES[log_ch].flow_vel_c / 100;//����
	LOG_DETAIL[log_ch][log_num].sound_speed = (long)MES[log_ch].sound_vel_f;		//����
	LOG_DETAIL[log_ch][log_num].total_count.DWORD = (MES[log_ch].addit_buff.DWORD);//�ώZ�l
	LOG_DETAIL[log_ch][log_num].wave_max = MES[log_ch].rev_max_data;				//��g�g�`�ő�l
	LOG_DETAIL[log_ch][log_num].wave_min = MES[log_ch].rev_min_data;				//��g�g�`�ŏ��l
	LOG_DETAIL[log_ch][log_num].dt = (long)MES[log_ch].delta_ts_zero;				//�`�����ԍ�
	LOG_DETAIL[log_ch][log_num].correlate = (MES[log_ch].correlate * 1000);		//���֒l��
	LOG_DETAIL[log_ch][log_num].zero_offset = (SVD[log_ch].zero_offset - 4000) * 32;	//�[���_�I�t�Z�b�g
	LOG_DETAIL[log_ch][log_num].status = MAIN[log_ch].err_condition;				//�X�e�[�^�X
	LOG_DETAIL[log_ch][log_num].fifo_position = MES[log_ch].fifo_no_read;			//FIFO��g�g�`���o�ʒu
	LOG_DETAIL[log_ch][log_num].gain_up = (short)get_attenuator_gain(log_ch);		//��1�Q�C��
	LOG_DETAIL[log_ch][log_num].gain_down = (short)MES[log_ch].amp_gain_for;		//��2�Q�C��
	LOG_DETAIL[log_ch][log_num].fifo_ch = MES[log_ch].fifo_ch_read;				//FIFO CH
	LOG_DETAIL[log_ch][log_num].p1_p2 = MES[log_ch].max_point_sub_f;				//��g�̍�(P1-P2)
	LOG_DETAIL[log_ch][log_num].mail_err_status = cunet_error;						//���[�����M�G���[�X�e�[�^�X

	if(code == ERR_ZERO_EMPTY || code == ERR_MESR_EMPTY_L || code == ERR_MESR_EMPTY_H){
		for(i_cnt = 0; i_cnt < SUB_POINT; i_cnt ++){
			LOG_DETAIL[log_ch][log_num].sum_abs_log[i_cnt] = 0;		//�g�`�f�[�^0
		}
	}else{	
		for(i_cnt = 0; i_cnt < SUB_POINT; i_cnt ++){
			LOG_DETAIL[log_ch][log_num].sum_abs_log[i_cnt] = SAVE[log_ch].sum_abs_com[i_cnt];		//�������֒l
		}
	}	
	cunet_error = 0;
}

/****************************************************/
/* Function : log_save                   */
/* Summary  : �G���[���O���̓o�^    				*/
/* Argument : ch,  code            	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	log_save(short ch, short code){

	if(ch >= CH_NUMMAX){				//CH�w��`�F�b�N
		return;						//�o�^�s��
	}

	switch(code){
		/*EEPROM�փ��O����ێ�����G���[*/
		case	ERR_ZERO_EMPTY:			//�[�������G���v�e�B�Z���T�[		
		case	ERR_ZERO_WAVE:			//�[�������g�`���� �i�g�`�ُ�j
		case	ERR_ZERO_CALC:			//�[���������Z�ُ�
		case	ERR_ZERO_LEVEL:			//�[������U/D���x�����ُ�
		case	ERR_ZERO_AGC:			//�[������AGC�s�\//
		case	ERR_MESR_EMPTY_H:		//����G���v�e�B�Z���T�[H	
		case	ERR_MESR_WAVE_H:		//����g�`���� �i�g�`�ُ�jH
		case	ERR_MESR_CALC_H:		//���艉�Z�ُ�H
		case	ERR_MESR_LEVEL_H:		//����U/D���x�����ُ�H
		case	ERR_MESR_AGC_H:			//����AGC�s�\H
		case	ERR_MESR_REVERSE:		//����t���ُ�
		case	ERR_MESR_OVERFLOW:		//�I�[�o�[�t���[
		case	ERR_EEPROM:				//EEPROM�G���[
		case	ERR_CUNET:				//CUnet�G���[
		case	ERR_ZERO_UNSTABLE:		//�[�������v�����Ԕ��U
		case	ERR_RESTART:			//�ċN��
		case	ERR_DEVICE:				//�������f�o�C�X�ُ�
			log_basicinfo_save(ch, code);		//�G���[���O��{���̓o�^
			log_detailinfo_save(ch, code);		//�G���[���O�ڍ׏��̓o�^
			break;
		/*EEPROM�փ��O����ێ����Ȃ��G���[*/
		case	ERR_MESR_EMPTY_L:		//����G���v�e�B�Z���T�[L		
		case	ERR_MESR_WAVE_L:		//����g�`���� �i�g�`�ُ�jL
		case	ERR_MESR_CALC_L:		//���艉�Z�ُ�L
		case	ERR_MESR_LEVEL_L:		//����U/D���x�����ُ�L
		case	ERR_MESR_AGC_L:			//����AGC�s�\L
		case	ERR_10POINT:			//10�_�f�[�^����
		case	TTL_CACL_ERR:				//�ώZ�l���Z�ُ�
		case	TTL_OVERFLOW:				//�ώZ�l�I�[�o�[�t���[
			log_detailinfo_save(ch, code);		//�G���[���O�ڍ׏��̓o�^
			break;
		case	ALM_MESR_GAIN:		//����A���v�Q�C���}�ρi�x���j	
		case	ALM_MESR_EMPTY:		//����G���v�e�B�Z���T�i�x���j	
		default:
			break;
	}

	err_priority(ch, code);				//�D�揇�ʊm�F

}
