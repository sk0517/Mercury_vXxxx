/***********************************************/
/* File Name : defLOG.h 		         									   */
/*	Summary   : �G���[���O����`                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include "define.h"

/************************************************************/
/*	C�t�@�C����`											*/
/************************************************************/
extern struct stLOG_DETAIL		LOG_DETAIL[CH_NUMMAX][LOGMAX];	//�G���[���O�ڍ׏��
extern struct stLOG_DETAIL_DEF	LOG_DETAIL_DEF;					//�G���[���O�ڍ׏��

/************************************************************/
/*	�\����													*/
/************************************************************/
struct stLOG_DETAIL{						//�G���[���O�ڍ׏��
	short	err_code;						//�G���[�R�[�h
	short	err_pwon_count;					//�G���[�������̓d���I���J�E���^
	long	err_time;						//�G���[����������
	long	err_time_now;					//���ݎ���
	long	flow_quantity;					//����
	long	flow_velocity;					//����
	long	sound_speed;					//����
	
	union{
		unsigned long long DWORD;
		struct{
			unsigned long low;
			unsigned long high;
		}WORD;
	}total_count;							//�ώZ�l
	
	long	wave_max;						//��g�g�`�ő�l
	long	wave_min;						//��g�g�`�ŏ��l
	long	dt;								//�`�����ԍ�
	long	correlate;						//���֒l��
	long	zero_offset;					//�[���_�I�t�Z�b�g
	short	status;							//�X�e�[�^�X
	short	fifo_position;					//FIFO��g�g�`���o�ʒu
	short	gain_up;						//�Q�C���i�A�b�v�j
	short	gain_down;						//�Q�C���i�_�E���j
	short	fifo_ch;						//FIFO CH
	short	p1_p2;							//��g�̍�(P1-P2)
	short	mail_err_status;				//���[�����M�G���[
	unsigned short	sum_abs_log[42];				//�������֒l
};

/************************************************************/
/*	�ݒ�l�̃f�t�H���g�l									*/
/************************************************************/
struct stLOG_DETAIL_DEF{					//�G���[���O��{���
	short	err_code;						//�G���[�R�[�h
	short	err_pwon_count;					//�G���[�������̓d���I���J�E���^
	long	err_time;						//�G���[�������̎���
	long	err_time_now;					//���ݎ���
	long	flow_quantity;					//����
	long	flow_velocity;					//����
	long	sound_speed;					//����
	
	union{
		unsigned long long DWORD;
		struct{
			unsigned long low;
			unsigned long high;
		}WORD;
	}total_count;							//�ώZ�l
	
	long	wave_max;						//��g�g�`�ő�l
	long	wave_min;						//��g�g�`�ŏ��l
	long	dt;								//�`�����ԍ�
	long	correlate;						//���֒l��
	long	zero_offset;					//�[���_�I�t�Z�b�g
	short	status;							//�X�e�[�^�X
	short	fifo_position;					//FIFO��g�g�`���o�ʒu
	short	gain_up;						//�Q�C���i�A�b�v�j
	short	gain_down;						//�Q�C���i�_�E���j
	short	fifo_ch;						//FIFO CH
	short	p1_p2;							//��g�̍�(P1-P2)
	short	mail_err_status;				//���[�����M�G���[
	unsigned short	sum_abs_log[42];		//�������֒l
};

