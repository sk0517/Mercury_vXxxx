/***********************************************/
/* File Name : SV_def.h 		         									   */
/*	Summary   : �p�����[�^�̒�`	                    */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "define.h"

/************************************************************/
/*	�p�����[�^�̒�`									                                     */
/*	���p�����[�^�����l(SV_def.c)�ƍ��킹�邱��									                */
/*                                                          */
/*  ���p�����[�^�ǉ����� util.c����util_eep_allwrite() �ɒǋL����*/
/************************************************************/
typedef struct {
	/*****             *****/
	/***   ���[�U�p�����[�^   ***/
	/*****             *****/
		short max_flow;			/*�t���X�P�[��*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		unsigned short unit;	/*�t���X�P�[�������_�ʒu�A�P��*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/  
		short sensor_size;		/*�Z���T���*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		short k_factor;			/*K�t�@�N�^*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		short damp;				/*�_���s���O*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		short low_cut;			/*���[�J�b�gOFF*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		unsigned short burnout;  /*�o�[���A�E�g*/
		short burnout_value;     /*�o�[���A�E�g���͒l*/
		short	viscos;        /*���S�x*/
		short viscos_auto;   /*���S�x�Œ�/����*/
		unsigned short err_hold_time;		/*�G���[�z�[���h�^�C��*/
		unsigned short reverse_time;	  /*�t�����莞��*/
		short	reverse_level;	/*�t������臒l*/
		short reserve_01[64];		    /*�\��[64]*/

	/*****             *****/
	/***   ���[�J�p�����[�^    ***/
	/*****             *****/
		short drive_freq;			/*�쓮���g��*/
		short drive_pls;			/*�h���C�u�p���X��*/
		short	search_sw;			/*���֒l�T�[�`�@�\*/
		short gain_step;			/*�Q�C������X�e�b�v��*/
		short sound_vel_filter;				/*�����t�B���^���萔*/
		short fifo_ch_init;				/*FIFO CH �����l*/
		short sound_vel_sel;		/*�����Œ�E����ؑ�*/
		short sound_vel_fix;		/*�����Œ�l*/
		unsigned short hldt;		/*�ُ�z�[���h���ԑI��*/
		short cunet_delay;				/*CUnet �đ��M�ҋ@����*/
		short wave_vth;			/*��g���o臒l*/
		short balance_level;		/*�g�`�A���o�����X���o臒l*/
		short saturation_level;	/*�g�`�O�a���o臒l*/
		short correlate_level;	/*�������֔�臒l*/
		short correlate_time;		/*���Z�ُ픻���*/
		short	attenuate_level;	/*�g�`����臒l*/
		short alm_wave_vth;					/*��g���o�x��臒l*/
		short alm_gain_level;		/*�A���v�Q�C���x��臒l*/
		short	alm_gain_count;		/*�A���v�Q�C���x������*/
		unsigned short alm_hold_time;		/*�x���o�͎���*/
		short zero_offset;		/*�[���_�I�t�Z�b�g*/
		unsigned short filter_mode;	/*�t�B���^���[�h*/	
		unsigned short filter_avg;		/*�ړ����ϐ�*/	
		unsigned short LL_enable;		/*��t���j�A���C�Y���[�h�I��*/
		unsigned short LL_kind;		  /*��t���j�A���C�Y���[�h�E��t���*/
		short total_offset_enable;	/*�ώZ�I�t�Z�b�g�l(ON/OFF)*/
		short total_offset_value;		/*�ώZ�I�t�Z�b�g�l(�ݒ�l)*/
		short drive_search;		/*�������[�h*/
		short start_freq;		  /*�T�[�`�J�n���g��*/
		short stop_freq;		   /*�T�[�`��~���g��*/
#if defined(FRQSCH)
		short SchFrq;           /* �T�[�`���ē������g�� */
#endif
		union{
			long INT32;
			struct{
				unsigned short low;
				unsigned short high;
			}DT_2BYTE;
		}target_total;			/*�ώZ�ڕW�l*/
		unsigned short damp_mode;	/*�C�A�΍􃂁[�h*/
		unsigned short rl2d;		/*D Rate Limit �_���s���O�{��*/
		unsigned short rl2hc;		/*D Rate Limit �z�[���h�N���A*/
		short odpd;				/*���C�g���~�b�g1st*/
		unsigned short rl1tg;		/*D Rate limit 1st �ڕW*/
		unsigned short rl1av;		/*D Rate limit 1st ����*/
		short odpl;				/*���C�g���~�b�g2nd*/
		unsigned short rl2tg;		/*D Rate limit 2nd �ڕW*/
		unsigned short rl2av;		/*D Rate limit 2nd ����*/
		short rlt;				/*���C�g���~�b�g*/
		unsigned short dump_var;	/*�σ_���s���O�΍��l*/
		unsigned short dump_mul;	/*�σ_���s���O�{��*/
		unsigned short inc;		/*�ُ�z�[���h����*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}corr_up;				/*���֒l������l*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}corr_low;				/*���֒l�������l*/
		
		
//�]���p
		unsigned short adc_clock;			//ADC�N���b�N
		unsigned short sns_option;			//�Z���T�I�v�V����
		unsigned short sns_disL; //�Z���T�ԋ���(L)
		unsigned short sns_disL_l; //�Z���T�ԋ���(L-l)
		unsigned short sns_tau; //���ʎ���
		unsigned short sns_coef; //�݊��W��
		unsigned short wind_offset; //WINDOW�I�t�Z�b�g
		unsigned short sum_start; //�������֊J�n�ʒu
		unsigned short sum_end;   //�������֏I���ʒu
		unsigned short sum_step;  //�������֊Ԋu

		unsigned short fix_data;  //�Œ�l�ݒ� bit0 :�f�B�U�����O�X�C�b�`
		                          //             1 : �Q�C���Œ�l�X�C�b�`
								  //             2 : fifo�J�n�A�I���ʒu�Œ�X�C�b�`
								  //             3 : fifo_no �Œ�X�C�b�`
								  //             4 : �g�`�擪�ʒu�Œ�X�C�b�`
		unsigned short fix_amp_gain_rev;  //Wiper Position(�Œ�l)
		unsigned short fix_fifo_ch_read;  //FIFO CH(�Œ�l)
		unsigned short fix_fifo_no_read;  //Leading Position(�Œ�l)

		unsigned short ZerCrsSttPnt; //Zero Cross Shift point
		unsigned short ZerCrsUseNum; //Zero Cross Use Number
		unsigned short ZerCrsOffset[7]; //�[���N���X�[���_�I�t�Z�b�g
		unsigned short ThresholdPeakPos;	//�g�`�F���s�[�N�ʒu
		unsigned short ZerPeakPos;	//�g�`�F���s�[�N�ʒu(�[���N���X�v�Z�J�n�ʒu�p)

		unsigned short DgtFltSwc; //Digital Filter Switch
		unsigned short DgtFltCefA00; //Digital Filter Coefficient A00
		unsigned short DgtFltCefA01; //Digital Filter Coefficient A01
		unsigned short DgtFltCefA10; //Digital Filter Coefficient A10
		unsigned short DgtFltCefA11; //Digital Filter Coefficient A11
		unsigned short DgtFltCefA20; //Digital Filter Coefficient A20
		unsigned short DgtFltCefA21; //Digital Filter Coefficient A21
		unsigned short DgtFltCefA30; //Digital Filter Coefficient A30
		unsigned short DgtFltCefA31; //Digital Filter Coefficient A31
		unsigned short DgtFltCefA40; //Digital Filter Coefficient A40
		unsigned short DgtFltCefA41; //Digital Filter Coefficient A41
		unsigned short DgtFltCefB00; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB01; //Digital Filter Coefficient B01
		unsigned short DgtFltCefB10; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB11; //Digital Filter Coefficient B01
		unsigned short DgtFltCefB20; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB21; //Digital Filter Coefficient B01
		unsigned short DgtFltCefB30; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB31; //Digital Filter Coefficient B01
//�]���p		

#if defined(FRQSCH)
		short reserve_02[28];		/*�\��[28]*/
#else
		short reserve_02[29];		/*�\��[29]*/
#endif

	/*****             *****/
	/***   ���[�J���j�A���C�Y   ***/
	/*****             *****/
		short mklnr_num;				/*�␳�_��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out1;					/*�␳�o��1(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out2;					/*�␳�o��2(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out3;					/*�␳�o��3(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out4;					/*�␳�o��4(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out5;					/*�␳�o��5(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out6;					/*�␳�o��6(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out7;					/*�␳�o��7(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out8;					/*�␳�o��8(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out9;					/*�␳�o��9(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out10;					/*�␳�o��10(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out11;					/*�␳�o��11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out12;					/*�␳�o��12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out13;					/*�␳�o��13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out14;					/*�␳�o��14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out15;					/*�␳�o��15(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in1;					/*�␳����1(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in2;					/*�␳����2(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in3;					/*�␳����3(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in4;					/*�␳����4(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in5;					/*�␳����5(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in6;					/*�␳����6(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in7;					/*�␳����7(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in8;					/*�␳����8(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in9;					/*�␳����9(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in10;					/*�␳����10(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in11;					/*�␳����11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in12;					/*�␳����12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in13;					/*�␳����13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in14;					/*�␳����14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in15;					/*�␳����15(L/m)*/

	/*****             *****/
	/***  ���[�U���j�A���C�Y   ***/
	/*****             *****/
		unsigned short uslnr_num;		/*�␳�_���A�����_�ʒu*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out1;				/*�␳�o��1(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out2;				/*�␳�o��2(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out3;				/*�␳�o��3(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out4;				/*�␳�o��4(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out5;				/*�␳�o��5(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out6;				/*�␳�o��6(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out7;				/*�␳�o��7(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out8;				/*�␳�o��8(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out9;				/*�␳�o��9(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out10;				/*�␳�o��10(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out11;				/*�␳�o��11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out12;				/*�␳�o��12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out13;				/*�␳�o��13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out14;				/*�␳�o��14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out15;				/*�␳�o��15(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in1;				/*�␳����1(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in2;				/*�␳����2(L/m)*/	    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in3;				/*�␳����3(L/m)*/	    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in4;				/*�␳����4(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in5;				/*�␳����5(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in6;				/*�␳����6(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in7;				/*�␳����7(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in8;				/*�␳����8(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in9;				/*�␳����9(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in10;				/*�␳����10(L/m)*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in11;				/*�␳����11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in12;				/*�␳����12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in13;				/*�␳����13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in14;				/*�␳����14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in15;				/*�␳����15(L/m)*/

	/*****                  *****/
	/***   �[���_�������X�e�[�^�X   ***/
	/*****                  *****/
		union{
			long DWORD;
			struct{
				short low;
				short high;
			}WORD;
		}zero_flow_qat;					/*�[���_�������F����*/
		union{
			long DWORD;
			struct{
				short low;
				short high;
			}WORD;
		}zero_flow_vel;					/*�[���_�������F����*/
		unsigned short zero_sound_spd;	/*�[���_�������F����*/
		union{
			unsigned long long DWORD;
			struct{
				short low1;
				short low2;
				short high1;
				short high2;
			}WORD;
		}zero_addit;					/*�[���_�������F�ώZ�l*/	
		unsigned short zero_wave_max;		/*�[���_�������F��g�g�`�ő�l*/
		unsigned short zero_wave_min;		/*�[���_�������F��g�g�`�ŏ��l*/
		union{
			long DWORD;
			struct{
				short low;
				short high;
			}WORD;
		}zero_delta_ts;					/*�[���_�������F�`�����ԍ�*/
		unsigned short zero_correlate;	/*�[���_�������F���֒l��*/
		unsigned short zero_zero_offset;	/*�[���_�������F�[���_�I�t�Z�b�g*/
		unsigned short zero_condition;	/*�[���_�������F�R���f�B�V����*/
		unsigned short zero_fifo_pos;		/*�[���_�������FFIFO��g�g�`���o�ʒu*/
		unsigned short zero_gain_1st;		/*�[���_�������F�A���v�Q�C���l(1st)*/
		unsigned short zero_gain_2nd;		/*�[���_�������F�A���v�Q�C���l(2nd)*/
		unsigned short zero_fifo_ch;		/*�[���_�������FFIFO CH*/
		unsigned short zero_p1p2;			/*�[���_�������F��g�̍�(P1-P2)*/

		short zero_FwdSurplsTim; //�㗬�g�`���ʎ���
		short zero_RevSurplsTim;
		union {
			long DWORD;
			struct {
				short low;
				short high;
			}WORD;
		}zero_FwdTimDif; //�㗬�g�`���ԍ�
		union {
			long DWORD;
			struct {
				short low;
				short high;
			}WORD;
		}zero_RevTimDif; //�����g�`���ԍ�
		short zero_drive_freq; //�ł����ݎ��g��
		short zero_FwdWavPekPosLst[WAV_PEK_NUM]; //�㗬�g�`�s�[�N�ʒu
		short zero_FwdWavPekValLst[WAV_PEK_NUM]; //�㗬�g�`�s�[�N�l
		short zero_RevWavPekPosLst[WAV_PEK_NUM]; //�����g�`�s�[�N�ʒu
		short zero_RevWavPekValLst[WAV_PEK_NUM]; //�����g�`�s�[�N�l

	/*****            *****/
	/***   �G���[���O�֘A   ***/
	/*****            *****/
		unsigned short pwon_count;/*�N����*/
		short	err_code[LOGMAX];			/*�G���[�R�[�h�i�ŐV10�_�j*/
		short	err_pwon_count[LOGMAX];		/*�G���[�������̓d���I���J�E���^�i�ŐV10�_�j*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}err_time[LOGMAX];				/*�G���[�������̎��ԁi�ŐV10�_�j*/

	/*****         *****/
	/***   �ʐM�֘A   ***/
	/*****         *****/
		short com_interval; /*�T�C�����g�C���^�[�o��*/
		short com_speed;    /*�ʐM�X�s�[�h*/
		short com_mode;     /*�ʐM���[�h*/
		short com_parity;			/*�p���e�B*/
		unsigned short sti;		/*�T�C�����g�C���^�[�o��(2byte)*/
		unsigned short cmod;		/*�ʐM���[�h�A�p���e�B(2byte)*/

	/*****                    *****/
	/***   �o�[�W�����A�V���A���i���o�[  ***/
	/*****                    *****/
		unsigned short soft_ver;	/*�\�t�g�E�F�A�o�[�W����*/
		unsigned short hard_ver;	/*�n�[�h�E�F�A�o�[�W����*/
		unsigned short c_serial[8];		/*�ϊ���V���A���i���o�[*/
		unsigned short s_serial[8];		/*�Z���T�V���A���i���o�[*/    /*<<�������f�o�C�X�ۑ�>> ���ύX�֎~��*/

 		short reserve_03[25];		/*�\��[25]*/

} stSVD;

//----------------------------------------------------------------------
//�O���錾
//----------------------------------------------------------------------
extern stSVD SVD[CH_NUMMAX];

