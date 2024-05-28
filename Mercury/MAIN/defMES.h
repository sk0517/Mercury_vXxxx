/***********************************************/
/* File Name : defMES.h 		         									   */
/*	Summary   : ���b�Z�[�W�ʐM����                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//�C���N���[�h�K�[�h
//----------------------------------------------------------------------
#ifndef DEFMES_H
#define DEFMES_H

#include "define.h"

// #define MESWAVEXP
#define MESWAVSIZ 600

#ifdef FRQSCH
extern struct stFrqSch FrqSch[CH_NUMMAX];
#endif
//----------------------------------------------------------------------
//C�t�@�C����`
//----------------------------------------------------------------------
extern struct stMES MES[CH_NUMMAX];	//���ʌv��
extern struct stMES_SUB MES_SUB[CH_NUMMAX];	//���ʌv��

//----------------------------------------------------------------------
//�\����
//----------------------------------------------------------------------

typedef struct
{
    long Flw; //����
    float Tup; //Time Difference Up
    float Tdw; //Time Difference Down
    short FowZcd[ZC_POINT_MAX][5]; //Foward Zerocross Data
    short RevZcd[ZC_POINT_MAX][5]; //Reverse Zerocross Data
    float FowClcZcd[ZC_POINT_MAX]; //Fow Calculate Zerocross Data
    float RevClcZcd[ZC_POINT_MAX]; //Fow Calculate Zerocross Data
}stZcLog;

struct stMES{	//���ʌv��
	
	short amp_gain_for;
	short amp_gain_rev;
	short amp_gain_old;
	unsigned char amp_gain_change[AMP_BUFF];
	short zero_amp_gain_rev;
	short amp_gain_fifo;

	short fifo_no;			//FIFO Read Point
	short fifo_ch;			//FIFO chanel	
	short fifo_no_read;
	short fifo_ch_read;
	short signal_count;			//FIFO�̐M���ʒu
	short non_signal_count;
	short zero_fifo_no_read;
	short zero_fifo_ch_read;
	short zero_signal_count;			//FIFO�̐M���ʒu
	
	short fifo_start;
	short fifo_end;
	short fifo_offset;

#if defined(MESWAVEXP)
	short fow_data[MESWAVSIZ + 100];		//fow data read area
	short rev_data[MESWAVSIZ + 100];		//rev data read area
#else
	short fow_data[300];		//fow data read area
	short rev_data[300];		//rev data read area
#endif
	short fow_max_data;	//�ő�
	short fow_min_data;	//�ŏ�
	short rev_max_data;	//�ő�
	short rev_min_data;	//�ŏ�
	short fow_max_data_point; //�g�`�U���̍ő�l���Ƃ�x���W
	short rev_max_data_point;
	
	short FwdWavPekNum;
	short RevWavPekNum;
	short FwdWavPekValLst[WAV_PEK_NUM]; //�㗬�g�`�s�[�N�l�O����
	short FwdWavPekPosLst[WAV_PEK_NUM]; //�㗬�g�`�s�[�N�ʒu�O����
	short RevWavPekValLst[WAV_PEK_NUM]; //�����g�`�s�[�N�l�O����
	short RevWavPekPosLst[WAV_PEK_NUM]; //�����g�`�s�[�N�ʒu�O����
	short FwdWavMinPekValLst[WAV_PEK_NUM];
	short FwdWavMaxPekValLst[WAV_PEK_NUM];
	short FwdWavMinPekPosLst[WAV_PEK_NUM];
	short FwdWavMaxPekPosLst[WAV_PEK_NUM];
	short RevWavMinPekValLst[WAV_PEK_NUM];
	short RevWavMaxPekValLst[WAV_PEK_NUM];
	short RevWavMinPekPosLst[WAV_PEK_NUM];
	short RevWavMaxPekPosLst[WAV_PEK_NUM];
	short FwdUseZerDat[ZC_POINT_MAX][5];
	float FwdClcZerPnt[ZC_POINT_MAX];
	short RevUseZerDat[ZC_POINT_MAX][5];
	float RevClcZerPnt[ZC_POINT_MAX];

	short m0_point_fow[3];	//�ŏ��|�C���g
	short m0_point_rev[3];	//�ŏ��|�C���g
	short v0_value_fow[3];	//�ŏ��l
	short v0_value_rev[3];	//�ŏ��l

	short fow_center_point;	/*MAX ��50%�̈ʒu*/
	short rev_center_point;	/*MAX ��50%�̈ʒu*/
	short fow_center_data;	/*MAX ��50%�̒l*/
	short rev_center_data;	/*MAX ��50%�̒l*/
	short center_sign_now;
	short center_sign_old;
	short m0_point_fow_50;
	short m0_point_rev_50;

	short ThreasholdPoint_Fow; //��g�������l��x���W
	short ThreasholdPoint_Rev; //��g�������l��x���W
	short fow_max_phase[4];
	short rev_max_phase[4];
	short fow_min_phase[4];
	short rev_min_phase[4];

	char read_ok[1000];
	short count_ok;
	short data_ok;
	
	short correlate;				//�����֒l���W��//
	long sum_abs[SUB_POINT];	//��Βl�̘a�A�i�[�̈�
	short min_point_m;	//�ŏ��l��offset�l(0-20)
	long sum_max;
	long sum_min;
	short search_start;
	short search_end;
	short max_point_sub;		//fow - rev
	short max_point_sub_f;	//fow - rev
		
	unsigned short delta_ts;		//���s��
	unsigned short delta_ts_buf[50];
	unsigned short delta_ts0;
	unsigned long delta_ts_sum;		//��ts���Z�f�[�^
	short ts_cont;				//�ړ����ϗp���ԍ��o�b�t�@�J�E���^

	short MvgAveFlwCnt; //�ړ����ϗp���ʃo�b�t�@�J�E���^
	long MvgAveFlwBuf[50]; //�ړ����ϗp���ʃo�b�t�@
	long long MvgAveFlwSum; //�ړ����ϗp���ʃo�b�t�@���a

	long vth_sum;
	short vth_count;
	long delta_ts_zero;			//�[�������ς݃f�[�^
	long delta_ts_zero_f;		//�[�������ς݃f�[�^,�t�B���^��
	unsigned short zero_adj_data;	//�[�������f�[�^
	
	short sonic_point_fow_p1;
	short sonic_point_rev_p2;
	short zero_sonic_point_fow_p1;
	short zero_sonic_point_rev_p2;
	
	short viscos_table[21];	//�������j�A���C�Y�e�[�u��
	short viscos_val;			//���S�x�ݒ�(30-4000)=0.30-40.00
	short sound_vel;			//����[m/s]
	short sound_vel_f;

	long flow_vel;		//����[m/s*10000]
	long flow_vel_a;	//����[m/s*10000]
	long flow_vel_b;	//����[m/s*10000]
	long flow_vel_c;	//����[m/s*10000]
	short ThreasholdPoint;	//���x�␳�p�A�����f�[�^
	
	long ml_min_now;
	long ml_min_old;
	long ml_min_a;
	long ml_min_b;
	long ml_min_c;
	long ml_min_d;
	long max_flow_long;	//�t���X�P�[��

	long past_flow[10];				/*�u������(10�_)*/
	
	//oq�R�}���h�i���[�J�b�g�I�t 10�_�u�����ʓǏo���j�p
	short pv_now_oq;			//�u������
	long ml_min_oq;			//�u������
	long past_flow_oq[10];	/*�u������(10�_)*/
	short past_flow_cnt;			/*�u������(10�_)�J�E���^*/
	long ml_min_OQ;			//�u������

	short flow_filt_sv;
	long flow_filt_in;
	long flow_filt_out;
	
	short flow_filt_sv_v;
	long flow_filt_in_v;
	long flow_filt_out_v;

	short sonic_filt_sv;	//sonic_point_sub(P1-P2) OVER/UNDER����p
	short sonic_filt_out;	//sonic_point_sub(P1-P2) OVER/UNDER����p
	long sonic_filt_in_s;
	long sonic_filt_out_s;

	unsigned short median_ptr;
	long median_buf[32];

	short addit_req;		/*�ώZ�v��*/
	short addit_req_w;	/*�ώZ�v��*/
	unsigned long	addit_pv;		/*0.1mL*/
	unsigned long	addit_pv_w;		/*0.1mL*/
	unsigned short total_status;
	short addit_watch;		/*�ώZ�Ď�*/
	
	union{
			unsigned long long DWORD;
			struct{
				unsigned long low;
				unsigned long high;
			}WORD;
		}addit_cont;			/*�ώZ���ʃJ�E���^*/
	union{
			unsigned long long DWORD;
			struct{
				unsigned long low;
				unsigned long high;
			}WORD;
		}addit_unit;			/*�ώZ���ʒP�ʊ��Z*/
	union{
			unsigned long long DWORD;
			struct{
				unsigned long low;
				unsigned long high;
			}WORD;
		}addit_buff;			/*�ώZ���ʃo�b�t�@(�ʐM�p)*/

	unsigned short	addit_mod;		/*�ώZ���ʗ]��*/	
	
	union{
			unsigned long long UINT64;
			struct{
				unsigned long low;
				unsigned long high;
			}DT_4BYTE;
		}addit_unit_ov;			/*�ώZ���ʒP�ʊ��Z(ov�p)*/
	union{
			unsigned long long UINT64;
			struct{
				unsigned long low;
				unsigned long high;
			}DT_4BYTE;
		}addit_buff_ov;			/*�ώZ���ʃo�b�t�@(ov�p)*/
	unsigned short	addit_mod_ov;		/*�ώZ���ʗ]��(ov�p)*/		

	short clk_phase;
	short wave_monitor_mode;
	short inspect_enable;

	short err_burn_out;
	short err_hold_out;
	unsigned short err_status;	//���ʃG���[���i�Z���T�ُ퓙�j
	unsigned short alm_status;	//���ʌx����i�G���v�e�B�Z���T�A�g�`�����A�g�`�A���o�����X�j
	
	short test_enable;
	short test_flow;
	short test_port_in;
	short test_port_out;
	short test_err_code;
	short test_err_enable;
	short pls_test_enable;
	short pls_test_on;
	short pls_test_ch1;
	short pls_test_ch2;

	//�[���N���X���Z�֘A
	short zc_nearzero_point[2][ZC_POINT_MAX];
	short zc_nearzero_data1[2][ZC_POINT_MAX];
	short zc_nearzero_data2[2][ZC_POINT_MAX];
	short zc_nearzero_data3[2][ZC_POINT_MAX];
	short zc_nearzero_data4[2][ZC_POINT_MAX];
	float zc_Tdata;
	float zc_zero_calc;
	float zc_zero_offset;

	short zc_peak;
	short zc_peak_UpdateFlg;
	unsigned short ThresholdWave;	/*�g�`�F��臒l*/
	unsigned short ThresholdPeak;	/*�g�`�F���s�[�N*/
	unsigned short ThresholdPeakPos;	 /*�g�`�F���s�[�N�ʒu*/
	unsigned short ThresholdReq;	/*�g�`�F�����s�v��*/
	
	//Window�T�[�`
	short ws_FifoCh;
	unsigned long ws_work_add;

	short FwdSurplsTim; //�㗬�g�`���ʎ���
	short RevSurplsTim; //�����g�`���ʎ���
	
	stZcLog ZcLog; //�[���N���X���O�f�[�^

	short ZerAdjPhs; //�[���_�����t�F�[�Y

	short ComVthCount; //�ʐM�f�o�b�O�p
	long ComVthSum; //�ʐM�f�o�b�O�p
};

struct stMES_SUB{	//���ʌv��
	float zc_delta_ts;
	float zc_delta_ts_buf[50];
	float zc_delta_ts_sum;
	float zc_delta_ts_zero;
	
	short fifo_result;
	short zc_peak_req;

	short ws_work_gain;			/*Window�T�[�`���̃Q�C���l*/
	unsigned long ws_add_max;	/*Window�T�[�`���̋ɒl�̉��Z�l*/
	short memory_side;		/*1Wire�������f�o�C�X����*/
	
	short sample_cnt;
	short ItvVal;
	
	unsigned short err_status_sub;	//�G���[���

	float zc_Tup;
	float zc_Tdown;
};

#ifdef FRQSCH
struct stFrqSch{
	short FrqSchSttFlg; //���g���T�[�`�J�n�t���O
	short MaxAmpLst[10]; //�e���g���ɂ�����U���ő�l(10��)
	short MaxAmpAvg; //�ő�U�����ϒl
	short MaxAmpFrq; //�ő�U�����g��
	short MesAmpCnt; //�e���g���ɂ�����ő���g������J�E���^
	short NowFrq; //���ݑ��蒆�̎��g��
	short MesErrCnt; //�g�`�G���[��
	short CenPntCnt; //���S���ʉ߃J�E���^
	short WitTim; //�ҋ@����
	short CenPntUndLim; //���S���ʉ߉񐔉���
	short CenPntOvrLim; //���S���ʉ߉񐔏��
};

#endif

#endif
