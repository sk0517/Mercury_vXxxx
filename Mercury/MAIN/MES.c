/***********************************************/
/* File Name : MES.c	    	         									   */
/*	Summary   : ���b�Z�[�W�ʐM����                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "machine.h"
#include "define.h"
#include "version.h"
#include "SV_def.h"
#include "defSAVE.h"
#include "defMES.h"
#include "defMAIN.h"
#include "defLOG.h"

/************************************************/
/*		�����錾								*/
/************************************************/
char 	comma[5] = ",";	// ','����p
short 	DLerr_code;
short		alm_output = 0;

#if defined(MESWAVEXP)
short	fow_wave_data[MESWAVSIZ + 100];
short	rev_wave_data[MESWAVSIZ + 100];
void WavInBuff(short com_mode);
#else
short	fow_wave_data[300];
short	rev_wave_data[300];
#endif
unsigned short sum_abs_data[40];

/************************************************/
/*		�O���錾								*/
/************************************************/
extern char		RX_buf[MSG_NUM][MSG_MAX];	/*��M�p�z��*/
extern char		TX_buf[MSG_NUM][MSG_MAX];	/*���M�p�z��*/
extern short		ch_no[MSG_NUM];
extern short		flow_check[MSG_NUM];
extern short		i_num[MSG_NUM];			// ����ԍ��p
extern short		size[MSG_NUM];			// ���͒l�f�[�^������p
extern short		digit_check[MSG_NUM];		// ����or�����_���� ����p
extern short		end_code[MSG_NUM];		// �G���h�R�[�h�����p
extern long long	value[MSG_NUM];			// ���͒l����p
extern short		com_type;
extern unsigned long	cmi_count;
extern short		led_flash;
extern short		mes_err[7];
extern short 	wave_hight[2][6];
extern short    OWwrite[6];

extern unsigned short AD_BASE;


/************************************************/
/*		�����֐��錾							*/
/************************************************/
void command_OV(short cm);
void command_ov(short cm);
void command_OE(short cm);
void command_OQ(short cm);
void command_oq(short cm);
void read_command_R(short cm);
void command_RS(short cm);
void command_Rs(short cm);
void command_Rg(short cm);
void command_Rr(short cm);
void command_Rk(short cm);
void command_Rd(short cm);
void command_Rl(short cm);
void command_Rb(short cm);
void command_Rv(short cm);
void command_Rh(short cm);
void command_Ru(short cm);
void command_Rm(short cm);
void command_RR(short cm);
void command_Rz(short cm);
void command_RF(short cm);
void command_RV(short cm);
void command_RG(short cm);
void command_RL(short cm);
void command_RD(short cm);
void command_RZ(short cm);
void command_RE(short cm);
void command_RA(short cm);
void command_Ry(short cm);
void command_RM(short cm);
void command_Rf(short cm);
void command_Rt(short cm);
void command_RT(short cm);
void command_R1(short cm);
void command_R2(short cm);
void command_R3(short cm);
void command_R4(short cm);
void command_R5(short cm);
void command_R6(short cm);
void command_R7(short cm);
void command_RC(short cm);
void command_R8(short cm);
void command_R9(short cm);
void command_RX(short cm);
void command_Ri(short cm);
void command_RW(short cm);
void command_Rp(short cm);
void command_Rn(short cm);
void command_Ra(short cm);
void read_change0X_status(short cm,unsigned short lst);
void read_change0X_2digit_status(short cm,unsigned short lst);
void read_change0X_version(short cm,long vr);
void read_change0X_versionRV(short cm,long vr);
void read_command_W(short cm);
void command_Wg(short cm);
void command_Wr(short cm);
void command_Wk(short cm);
void command_Wd(short cm);
void command_Wl(short cm);
void command_Wb(short cm);
void command_Wv(short cm);
void command_Wh(short cm);
void command_WR(short cm);
void command_Wu(short cm);
void command_WU(short cm);
void command_Wm(short cm);
void command_Wz(short cm);
void command_WZ(short cm);
void command_WP(short cm);
void command_WE(short cm);
void command_WA(short cm);
void command_Wy(short cm);
void command_Wf(short cm);
void command_Wt(short cm);
void command_WT(short cm);
void command_W1(short cm);
void command_W2(short cm);
void command_W5(short cm);
void command_W6(short cm);
void command_W7(short cm);
void command_W9(short cm);
void command_Wi(short cm);
void command_Wp(short cm);
void command_Wn(short cm);
void command_Wa(short cm);
void command_OS(short cm);
void command_Os(short cm);
void command_OJ(short cm);
void command_OL(short cm);
void command_OR(void);
void command_OT(void);
void command_OA(void);
void command_Ot(short cm);
void command_OZ (short cm);
void command_Dl (short cm); //FPGA�_�E�����[�h�R�}���h

#ifdef MEMDBG
void MemoryCom(short); //Debug
void command_MR(short cm); //Debug
void command_MW(short cm); //Debug
void command_MT(short cm); //Debug
void command_MZ(short cm);
void command_MS(short cm);
extern void	value_judge_Hex(short com_mode,short si,short pow); //Debug
#endif

#if defined(FRQSCH)
void command_OF(short cm);
#endif

void command_Wc(short cm);
void command_Rc(short cm);

/************************************************/
/*		�O���֐��錾							*/
/************************************************/
extern void	make_viscos_tbl(short);
extern void	value_judge(short com_mode,short si,short pow);
extern void	eep_write_ch(short,short,short);
extern void	err_change(unsigned short err);
extern void	read_change(short com_mode,long long read_value,char p, short pos);
extern void	action_status_control(short ch,short sts);
extern short	get_attenuator_gain(short pch);
extern void	InitFifoCh(short ch,short val);
extern void	err_thlevel_init(short ch, short val);
extern void	util_eep_allwrite(short pch, short opt);
extern void	util_SnsMem_Write(short pch, short opt);
extern void	util_eep_zerowrite(short pch);
extern void	log_detailinfo_init(short ch);
extern void	ram_clear_debug(void);
extern short	disp_zero_read(void);
extern short	disp_ch_read(void);
extern short	disp_cunet_read(void);
extern short	err_zero_status(short err_status);
extern short	err_total_status(short err_status);
extern void 	LLmode_kind(short vis, short pch);
extern short check_queue(void);
extern float GetTimDif(short pch, short Mod);

/* �ώZ�l�Ǐo��(OV) */			
void command_OV (short com_mode){

	unsigned long long value;
	unsigned short value_high;
	unsigned short value_low;
	unsigned short err_st;
	char 	add[20] = {0};

	strcat(TX_buf[com_mode],comma);
	err_st = MAIN[ch_no[com_mode] -1].com_err_status;			//�G���[�X�e�[�^�X
	if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
	if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
		if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
			&& MAIN[ch_no[com_mode] -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
		}
	}else{			//�ώZ�Ď��G���[������
		if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
			MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
		}
	}

	add[0] = err_st / 10 + 0x30;
	add[1] = err_st % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));				//�Ō���ɓǍ��l�ǉ�
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	value = MES[ch_no[com_mode] -1].addit_buff.DWORD / 10000;		//�ώZ�lx10000000 �� x1000
	read_change(com_mode,value,3,2);

}

/* �ώZ�l�Ǐo���E�ώZ���펞�X�V(ov) */			
void command_ov (short com_mode){

	unsigned long long value;
	unsigned short value_high;
	unsigned short value_low;
	unsigned short err_st;
	char 	add[20] = {0};

	strcat(TX_buf[com_mode],comma);
	err_st = MAIN[ch_no[com_mode] -1].com_err_status;			//�G���[�X�e�[�^�X
	if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
	if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
		if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
			&& MAIN[ch_no[com_mode] -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
		}
	}else{			//�ώZ�Ď��G���[������
		if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
			MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
		}
	}

	add[0] = err_st / 10 + 0x30;
	add[1] = err_st % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));				//�Ō���ɓǍ��l�ǉ�
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	value = MES[ch_no[com_mode] -1].addit_buff_ov.UINT64 / 10000;		//�ώZ�lx10000000 �� x1000
	read_change(com_mode,value,3,2);

}

/* �u������1�_�Ǐo��(OE) */
void command_OE(short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	
	unsigned short err_st;
	short mes_err_cnt[7] = {0};
	long  flow;
	unsigned long flow_z;
	unsigned long flow_cal;
	unsigned char flow_1;

	flow_check[com_mode] = 1;
	ch[0] = RX_buf[com_mode][7];					//CH������
	ch_z = ch[0] - 0x30;
	TX_buf[com_mode][8] = RX_buf[com_mode][6];
	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];			//CH�ԍ�����
		ch_number = ch[0] - 0x30;
		TX_buf[com_mode][10 + 9*m] = RX_buf[com_mode][8 + 2*m];
		TX_buf[com_mode][11 + 9*m] = RX_buf[com_mode][9 + 2*m];
		
		err_st = MAIN[ch_number -1].com_err_status;		//�G���[�X�e�[�^�X
		if(err_st != 0){
			mes_err_cnt[ch_no[com_mode]]++;
			mes_err_cnt[ch_number]++;
		}
		if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
			if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
				&& MAIN[ch_number -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
				MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
			}
		}else{			//�ώZ�Ď��G���[������
			if(err_zero_status((short)err_st) != B_YES){			//�[���_�����G���[�ȊO�ŁA
				MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				MAIN[ch_number -1].led_err_status = (short)0;
			}
		}		

		TX_buf[com_mode][12 + 9*m] = err_st / 10 + 0x30;
		TX_buf[com_mode][13 + 9*m] = err_st % 10 + 0x30;	

		flow = MES[ch_number -1].ml_min_now;	//�u�����ʒl

		if (flow >= 0)	{ flow_1 = 0x20; flow_z = flow;}
		else		{ flow_1 = 0xA0; flow_z = flow * (-1);}

		flow_cal = flow_z / 0x01000000;
		if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 8;}
		TX_buf[com_mode][15 + 9*m] = flow_cal;

		flow_cal = flow_z % 0x01000000;
		flow_cal /= 0x00010000;
		if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 4;}
		TX_buf[com_mode][16 + 9*m] = flow_cal;

		flow_cal = flow_z % 0x00010000;
		flow_cal /= 0x00000100;
		if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 2;}
		TX_buf[com_mode][17 + 9*m] = flow_cal;

		flow_cal = flow_z % 0x00000100;
		if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 1;}
		TX_buf[com_mode][18 + 9*m] = flow_cal;
				
		TX_buf[com_mode][14 + 9*m] = flow_1;
	}
	for (m = 0; m < 7; m++){
		if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
	}
	ch[0] = 0;

}

/* �u������10�_�Ǐo��(OQ) */
void command_OQ (short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	
	unsigned short err_st;
	short mes_err_cnt[7] = {0};
	long  flow;
	unsigned long flow_z;
	unsigned long flow_cal;
	unsigned char flow_1;

	flow_check[com_mode] = 2;
	ch[0] = RX_buf[com_mode][7];						//CH������
	ch_z = ch[0] - 0x30;
	TX_buf[com_mode][8] = RX_buf[com_mode][6];
	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];				//CH�ԍ�����
		ch_number = ch[0] - 0x30;
		TX_buf[com_mode][10 + 72*m] = RX_buf[com_mode][8 + 2*m];
		TX_buf[com_mode][11 + 72*m] = RX_buf[com_mode][9 + 2*m];
		for (l = 0; l < 10; l++){
			err_st = 0;
			if(l >= MES[ch_number -1].past_flow_cnt){			//10�_�f�[�^�o�b�t�@��10�_�Ȃ�
				err_st = ERR_10POINT;
			}
			//�D�揇�ʂ�10�_�f�[�^������荂���G���[������ꍇ
			if((MAIN[ch_number -1].com_err_status != 0) &&
				(err_inf[MAIN[ch_number -1].com_err_status].err_priority < err_inf[ERR_10POINT].err_priority)){
				err_st = MAIN[ch_number -1].com_err_status;		//�G���[�X�e�[�^�X
			}
			if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			}
			if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
				if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
					&& MAIN[ch_number -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
					MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				}
			}else{			//�ώZ�Ď��G���[������
				if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
					MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
					MAIN[ch_number -1].led_err_status = (short)0;
				}
			}

			TX_buf[com_mode][12 + 72*m + 7*l] = err_st / 10 + 0x30;
			TX_buf[com_mode][13 + 72*m + 7*l] = err_st % 10 + 0x30;

			flow = MES[ch_number -1].past_flow[l];

			if (flow >= 0)	{ flow_1 = 0x20; flow_z = flow;}
			else		{ flow_1 = 0xA0; flow_z = flow * (-1);}

			flow_cal = flow_z / 0x01000000;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 8;}
			TX_buf[com_mode][15 + 72*m + 7*l] = flow_cal;

			flow_cal = flow_z % 0x01000000;
			flow_cal /= 0x00010000;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 4;}
			TX_buf[com_mode][16 + 72*m + 7*l] = flow_cal;

			flow_cal = flow_z % 0x00010000;
			flow_cal /= 0x00000100;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 2;}
			TX_buf[com_mode][17 + 72*m + 7*l] = flow_cal;

			flow_cal = flow_z % 0x00000100;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 1;}
			TX_buf[com_mode][18 + 72*m + 7*l] = flow_cal;

			TX_buf[com_mode][14 + 72*m + 7*l] = flow_1;
			
		}
		if(MES[ch_number -1].past_flow_cnt < 10){					//10�_�f�[�^�o�b�t�@��10�_�Ȃ�
			MES[ch_number -1].err_status |= ERR_JUDGE_10POINT;		//10�_�f�[�^�����G���[�Z�b�g
		}
		MES[ch_number -1].past_flow_cnt = 0;						//10�_�f�[�^�ۑ��J�E���^�N���A
	}
	for (m = 0; m < 7; m++){
		if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
	}
	ch[0] = 0;
}

/* �u������10�_�Ǐo���E���[�J�b�g�I�t(oq) */
void command_oq (short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	
	unsigned short err_st;
	short mes_err_cnt[7] = {0};
	long  flow;
	unsigned long flow_z;
	unsigned long flow_cal;
	unsigned char flow_1;

	flow_check[com_mode] = 2;
	ch[0] = RX_buf[com_mode][7];						//CH������
	ch_z = ch[0] - 0x30;
	TX_buf[com_mode][8] = RX_buf[com_mode][6];
	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];				//CH�ԍ�����
		ch_number = ch[0] - 0x30;
		TX_buf[com_mode][10 + 72*m] = RX_buf[com_mode][8 + 2*m];
		TX_buf[com_mode][11 + 72*m] = RX_buf[com_mode][9 + 2*m];
		for (l = 0; l < 10; l++){
			err_st = 0;
			if(l >= MES[ch_number -1].past_flow_cnt){			//10�_�f�[�^�o�b�t�@��10�_�Ȃ�
				err_st = ERR_10POINT;
			}
			//�D�揇�ʂ�10�_�f�[�^������荂���G���[������ꍇ
			if((MAIN[ch_number -1].com_err_status != 0) &&
				(err_inf[MAIN[ch_number -1].com_err_status].err_priority < err_inf[ERR_10POINT].err_priority)){
				err_st = MAIN[ch_number -1].com_err_status;		//�G���[�X�e�[�^�X
			}
			if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			}
			if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
				if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
					&& MAIN[ch_number -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
					MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				}
			}else{			//�ώZ�Ď��G���[������
				if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
					MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
					MAIN[ch_number -1].led_err_status = (short)0;
				}
			}
			TX_buf[com_mode][12 + 72*m + 7*l] = err_st / 10 + 0x30;
			TX_buf[com_mode][13 + 72*m + 7*l] = err_st % 10 + 0x30;

			flow = MES[ch_number -1].past_flow_oq[l];

			if (flow >= 0)	{ flow_1 = 0x20; flow_z = flow;}
			else		{ flow_1 = 0xA0; flow_z = flow * (-1);}

			flow_cal = flow_z / 0x01000000;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 8;}
			TX_buf[com_mode][15 + 72*m + 7*l] = flow_cal;

			flow_cal = flow_z % 0x01000000;
			flow_cal /= 0x00010000;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 4;}
			TX_buf[com_mode][16 + 72*m + 7*l] = flow_cal;

			flow_cal = flow_z % 0x00010000;
			flow_cal /= 0x00000100;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 2;}
			TX_buf[com_mode][17 + 72*m + 7*l] = flow_cal;

			flow_cal = flow_z % 0x00000100;
			if (flow_cal == 0x40 || flow_cal == 0x0D) {flow_cal++; flow_1 += 1;}
			TX_buf[com_mode][18 + 72*m + 7*l] = flow_cal;

			TX_buf[com_mode][14 + 72*m + 7*l] = flow_1;
		}
		if(MES[ch_number -1].past_flow_cnt < 10){					//10�_�f�[�^�o�b�t�@��10�_�Ȃ�
			MES[ch_number -1].err_status |= ERR_JUDGE_10POINT;		//10�_�f�[�^�����G���[�Z�b�g
		}
		MES[ch_number -1].past_flow_cnt = 0;						//10�_�f�[�^�ۑ��J�E���^�N���A
	}
	for (m = 0; m < 7; m++){
		if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
	}
	ch[0] = 0;
}

/* �X�e�[�^�X�Ǐo��(SFC9000�Ή�)(RS) */
void command_RS (short com_mode){

	short m;
	short ch_z;
	short ch_number;
	unsigned short err_st;
	short mes_err_cnt[7] = {0};
	char ch[1] = {0};
	char add[20] = {0};
	
/* �V���O���R�}���h */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
	/* ����X�e�[�^�X */
		add[0] = MAIN[ch_no[com_mode] -1].com_act_status / 10 + 0x30;
		add[1] = MAIN[ch_no[com_mode] -1].com_act_status % 10 + 0x30;
		strncat(TX_buf[com_mode],add,sizeof(add));
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* �G���[�X�e�[�^�X */
		add[0] = MAIN[ch_no[com_mode] -1].com_err_status / 10 + 0x30;
		add[1] = MAIN[ch_no[com_mode] -1].com_err_status % 10 + 0x30;
		err_st = MAIN[ch_no[com_mode] -1].com_err_status;
		if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
		if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
			if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
				&& MAIN[ch_no[com_mode] -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
				MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
			}
		}else{			//�ώZ�Ď��G���[������
			if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
				MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
			}
		}
		strncat(TX_buf[com_mode],add,sizeof(add));
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* �ċN���X�e�[�^�X */
		err_st = reset_factor;
		read_change0X_2digit_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* CUnet�G���[�X�e�[�^�X */
		err_st = send_err_status;
		read_change0X_2digit_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* CUnet MCARE/LCARE��� */
		err_st = cunet_mcare;
		read_change0X_status(com_mode,err_st);
	}
/* �}���`�R�}���h */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];					//CH������
		ch_z = ch[0] - 0x30;
		ch[0] = 0;				
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH�ԍ�����
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* ����X�e�[�^�X */
			add[0] = MAIN[ch_number -1].com_act_status / 10 + 0x30;
			add[1] = MAIN[ch_number -1].com_act_status % 10 + 0x30;
			strncat(TX_buf[com_mode],add,sizeof(add));
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* �G���[�X�e�[�^�X */
			add[0] = MAIN[ch_number -1].com_err_status / 10 + 0x30;
			add[1] = MAIN[ch_number -1].com_err_status % 10 + 0x30;
			err_st = MAIN[ch_number -1].com_err_status;
			if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			}
			if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
				if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
					&& MAIN[ch_number -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
					MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				}
			}else{			//�ώZ�Ď��G���[������
				if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
					MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
					MAIN[ch_number -1].led_err_status = (short)0;
				}
			}
			strncat(TX_buf[com_mode],add,sizeof(add));
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* �ċN���X�e�[�^�X */
			err_st = reset_factor;
			read_change0X_2digit_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* CUnet�G���[�X�e�[�^�X */
			err_st = send_err_status;
			read_change0X_2digit_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* CUnet MCARE/LCARE��� */
			err_st = cunet_mcare;
			read_change0X_status(com_mode,err_st);
		}
		for (m = 0; m < 7; m++){
			if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
		}
	}

}

/* �X�e�[�^�X�Ǐo��(Rs) */
 void command_Rs (short com_mode){

	 short m;
	 short ch_z;
	 short ch_number;
	 unsigned short err_st;
	 short mes_err_cnt[7] = {0};
	 char ch[1] = {0};
	 char add[20] = {0};
	 
 /* �V���O���R�}���h */
	 if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
	 /* ����X�e�[�^�X */
		 add[0] = MAIN[ch_no[com_mode] -1].com_act_status / 10 + 0x30;
		 add[1] = MAIN[ch_no[com_mode] -1].com_act_status % 10 + 0x30;
		 strncat(TX_buf[com_mode],add,sizeof(add));
		 strncat(TX_buf[com_mode],comma,sizeof(comma));
	 /* �G���[�X�e�[�^�X */
		 add[0] = MAIN[ch_no[com_mode] -1].com_err_status / 10 + 0x30;
		 add[1] = MAIN[ch_no[com_mode] -1].com_err_status % 10 + 0x30;
		 err_st = MAIN[ch_no[com_mode] -1].com_err_status;
		 if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
		 if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
			 if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
				 && MAIN[ch_no[com_mode] -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
				 MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
			 }
		 }else{			//�ώZ�Ď��G���[������
			 if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
				 MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				 MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
			 }
		 }
		 strncat(TX_buf[com_mode],add,sizeof(add));
	 }
 /* �}���`�R�}���h */
	 else if (ch_no[com_mode] == 0){
		 ch[0] = RX_buf[com_mode][7];					//CH������
		 ch_z = ch[0] - 0x30;
		 ch[0] = 0;				
		 for (m = 0; m <= ch_z; m++){
			 TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			 TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		 }
		 for (m = 0; m < ch_z; m++){
			 ch[0] = RX_buf[com_mode][9 + 2*m];			//CH�ԍ�����
			 ch_number = ch[0] - 0x30;
			 strncat(TX_buf[com_mode],comma,sizeof(comma));
		 /* ����X�e�[�^�X */
			 add[0] = MAIN[ch_number -1].com_act_status / 10 + 0x30;
			 add[1] = MAIN[ch_number -1].com_act_status % 10 + 0x30;
			 strncat(TX_buf[com_mode],add,sizeof(add));
			 strncat(TX_buf[com_mode],comma,sizeof(comma));
		 /* �G���[�X�e�[�^�X */
			 add[0] = MAIN[ch_number -1].com_err_status / 10 + 0x30;
			 add[1] = MAIN[ch_number -1].com_err_status % 10 + 0x30;
			 err_st = MAIN[ch_number -1].com_err_status;
			 if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			 }
			 if(err_total_status((short)err_st) != B_YES){			//�ώZ�Ď��G���[�ȊO
				 if(err_zero_status((short)err_st) != B_YES			//�[���_�����G���[�ȊO�ŁA
					 && MAIN[ch_number -1].led_err_status == 0){		//�G���[�������Ă��Ȃ��ꍇ
					 MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
				 }
			 }else{			//�ώZ�Ď��G���[������
				 if(err_zero_status((short)err_st) != B_YES){						//�[���_�����G���[�ȊO�ŁA
					 MAIN[ch_number -1].com_err_status = (short)0;		//�G���[�X�e�[�^�X��Ǎ��񂾂�N���A
					 MAIN[ch_number -1].led_err_status = (short)0;
				 }
			 }
			 strncat(TX_buf[com_mode],add,sizeof(add));
		 }
		for (m = 0; m < 7; m++){
			if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
		}
	 }

 }

/* ���a�Ǐo��(Rg) */
void command_Rg (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].sensor_size;
	read_change(com_mode,value,0,0);
	
}

/* �t���X�P�[���Ǐo��(Rr) */
void command_Rr (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].max_flow;
	read_change(com_mode,value,0,0);
	
}

/* �j�t�@�N�^�Ǐo��(Rk) */
void command_Rk (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].k_factor;
	read_change(com_mode,value,3,0);

}

/* �_���s���O�Ǐo��(Rd) */
void command_Rd (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].damp;
	read_change(com_mode,value,1,0);

}

/* ���[�J�b�g�Ǐo��(Rl) */
void command_Rl (short com_mode){

	short m;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	short value;
	
/* �V���O���R�}���h */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		value = SVD[ch_no[com_mode] -1].low_cut;
		read_change(com_mode,value,1,0);
	}
/* �}���`�R�}���h */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];			//CH������
		ch_z = ch[0] - 0x30;
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];	//CH�ԍ�����
			ch_number = ch[0] - 0x30;
			value = SVD[ch_number -1].low_cut;
			read_change(com_mode,value,1,1);
		}
	}
	ch[0] = 0;

}

/* �o�[���A�E�g�Ǐo��(Rb) */
void command_Rb (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].burnout;		//���
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].burnout_value;		//���͒l
	read_change(com_mode,value,0,0);

}

/* ���S�x�Ǐo��(Rv) */
void command_Rv (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].viscos;
	read_change(com_mode,value,2,0);

}

/* �G���[�z�[���h�^�C���Ǐo��(Rh) */
void command_Rh (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].err_hold_time;
	read_change(com_mode,value,0,0);

}

/* ���[�U�[���j�A���C�Y�Ǐo��(Ru) */
void command_Ru (short com_mode){

	short l;
	unsigned short *pt;
	long value;
	unsigned short value_high;
	unsigned short value_low;

	short	point;

	point = (short)(SVD[ch_no[com_mode]-1].uslnr_num >> 8) & 0x00FF;	/*�ݒ�_���擾*/
	read_change(com_mode,point,0,0);
	pt = &SVD[ch_no[com_mode] -1].uslnr_out1.WORD.low;
	for (l = 0; l < 15; l++){
		value_low = *pt;
		pt++;
		value_high = *pt;
		value = value_low + value_high * 0x10000;
		read_change(com_mode,value,1,1);
		pt++;
	}
	pt = &SVD[ch_no[com_mode] -1].uslnr_in1.WORD.low;
	for (l = 0; l < 15; l++){
		value_low = *pt;
		pt++;
		value_high = *pt;
		value = value_low + value_high * 0x10000;
		read_change(com_mode,value,1,1);
		pt++;
	}

}

/* ���[�J�[���j�A���C�Y�Ǐo��(Rm) */
void command_Rm (short com_mode){

	short l;
	unsigned short *pt;
	long value;
	unsigned short value_high;
	unsigned short value_low;

	short	point;

	point = (short)(SVD[ch_no[com_mode]-1].mklnr_num >> 8) & 0x00FF;	/*�ݒ�_���擾*/
	read_change(com_mode,point,0,0);
	pt = &SVD[ch_no[com_mode] -1].mklnr_out1.WORD.low;
	for (l = 0; l < 15; l++){
		value_low = *pt;
		pt++;
		value_high = *pt;
		value = value_low + value_high * 0x10000;
		read_change(com_mode,value,3,1);
		pt++;
	}
	pt = &SVD[ch_no[com_mode] -1].mklnr_in1.WORD.low;
	for (l = 0; l < 15; l++){
		value_low = *pt;
		pt++;
		value_high = *pt;
		value = value_low + value_high * 0x10000;
		read_change(com_mode,value,3,1);
		pt++;
	}

}

/* �t������l�Ǐo��(RR) */
void command_RR (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].reverse_level;
	read_change(com_mode,value,0,2);	
	
	value = SVD[ch_no[com_mode] -1].reverse_time;
	read_change(com_mode,value,1,0);

}

/* �[���_������ԓǏo��(Rz) */
void command_Rz (short com_mode){

	short m;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	char add[20] = {0};

/* �V���O���R�}���h */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
	/* ����X�e�[�^�X */
		add[0] = MAIN[ch_no[com_mode] -1].com_act_status / 10 + 0x30;
		add[1] = MAIN[ch_no[com_mode] -1].com_act_status % 10 + 0x30;
		strncat(TX_buf[com_mode],add,sizeof(add));					//�Ō���ɓǍ��l�ǉ�
	}
/* �}���`�R�}���h */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];					//CH������
		ch_z = ch[0] - 0x30;
		ch[0] = 0;				
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH�ԍ�����
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma)); 
	/* ����X�e�[�^�X */
			add[0] = MAIN[ch_number -1].com_act_status / 10 + 0x30;
			add[1] = MAIN[ch_number -1].com_act_status % 10 + 0x30;
			strncat(TX_buf[com_mode],add,sizeof(add));					//�Ō���ɓǍ��l�ǉ�
		}
	}

}			
			
/* �o�[�W�����l�Ǐo��(SFC9000�Ή�)(RF) */
void command_RF (short com_mode){

	short l;
	unsigned long value;
	unsigned short *pt;
	short sn_version;
	short sn_ver_cal;
	char add[20] = {0};
	
	value = SVD[ch_no[com_mode] -1].soft_ver;		//software version
	read_change0X_version(com_mode,value);
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	if(com_type == COM_RS485){	
		value = 0;
		read_change(com_mode,value,0,2);
	}
	else{
		value = 1;
		read_change(com_mode,value,0,2);		
	}
	
	pt = &SVD[ch_no[com_mode] -1].c_serial[0];			//�V���A���i���o�[		
	for (l = 0; l < 8; l++){
		sn_version = *pt;
		sn_ver_cal = sn_version / 0x0100;
		if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			add[2*l] = sn_ver_cal;
		}
		sn_ver_cal = sn_version % 0x0100;
		if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			add[2*l + 1] = sn_ver_cal;
		}
		pt++;
	}
	strncat(TX_buf[com_mode],add,sizeof(add));
	
	if(com_type == COM_RS485){
		value = 0;
		read_change(com_mode,value,0,1);
	}
	else{
		value = disp_cunet_read();
		read_change(com_mode,value,0,1);
	}

}

/* �o�[�W�����l�Ǐo��(RV) */
 void command_RV (short com_mode){

	 unsigned long value;
	
	 value = SVD[ch_no[com_mode] -1].soft_ver;		//software version
	 read_change0X_versionRV(com_mode,value);
	 strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	 value = SVD[ch_no[com_mode] -1].hard_ver;		//hardware version
	 read_change0X_versionRV(com_mode,value);

 }

/* ���O�f�[�^�Ǐo��(RG)(SFC9000�Ή�) */
void command_RG (short com_mode){

	short l;
	short ch_z;
	unsigned short *pt;
	char ch[1] = {0};
	char add[20] = {0};
	long value;
	long long diff;
	unsigned short err_st;
	unsigned long long addit;
	
	TX_buf[com_mode][9] = RX_buf[com_mode][6];
	TX_buf[com_mode][10] = RX_buf[com_mode][7];
	ch[0] = RX_buf[com_mode][7];					//���O����ԍ�����
	ch_z = ch[0] - 0x30;
	if(ch_z == 0){ch_z = 10;}
	ch[0] = 0;

	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = SVD[ch_no[com_mode] -1].pwon_count;	//���ݓd��ON��
	read_change(com_mode,value,0,2);
	
	value = cmi_count;					//���݃G���[�^�C�}�[
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_pwon_count;	//�������d��ON��
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_time;		//�������G���[�^�C�}�[
	read_change(com_mode,value,0,2);
	
	add[0] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code / 10 + 0x30;		//ErrorLogCode
	add[1] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));					//�Ō���ɓǍ��l�ǉ�
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_quantity;		//����
	read_change(com_mode,value,2,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_velocity;		//����
	read_change(com_mode,value,3,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].sound_speed;		//����
	read_change(com_mode,value,0,2);
	
	addit = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].total_count.DWORD / 10000;	//�ώZ�l
	read_change(com_mode,addit,3,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].wave_max;			//��g�g�`�ő�l
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].wave_min;			//��g�g�`�ŏ��l
	read_change(com_mode,value,0,2);
	
	diff = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].dt * 32;		//�`�����ԍ�
	if (diff > 99999999) {diff = 99999999;}
	else if (diff < -99999999) {diff = -99999999;}
	read_change(com_mode,diff,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].correlate;			//���֒l��
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].zero_offset;		//�[���_�I�t�Z�b�g
	read_change(com_mode,value,0,2);

	err_st = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].status;				//status
	read_change0X_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].fifo_position;		//FIFO��g�g�`���o�ʒu
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_up;			//Gain 1st stage
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_down;		//Gain 2nd stage
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].fifo_ch;			//FIFO CH
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].p1_p2;				//��g�̍�(P1-P2)
	read_change(com_mode,value,0,2);

	err_st = reset_factor;				//�ċN���X�e�[�^�X
	read_change0X_2digit_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	err_st =  LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].mail_err_status;		//CUnet�G���[�X�e�[�^�X
	read_change0X_2digit_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	err_st =  cunet_mcare;				//CUnet MCARE/LCARE���
	read_change0X_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	err_st = cunet_mfr1;				//CUnet MFR���
	read_change0X_status(com_mode,err_st);
	err_st = cunet_mfr2;
	read_change0X_status(com_mode,err_st);
	err_st = cunet_mfr3;
	read_change0X_status(com_mode,err_st);
	err_st = cunet_mfr4;
	read_change0X_status(com_mode,err_st);

	pt = &LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].sum_abs_log[0];
	for (l = 0; l < 40; l++){
		value = *pt;
		read_change(com_mode,value,0,1);
		pt++;
	}

}

/* ���O�f�[�^�Ǐo��(RL) */
void command_RL (short com_mode){

	short l;
	short ch_z;
	short *pt;
	char ch[1] = {0};
	char add[20] = {0};
	long value;
	long long diff;
	unsigned short err_st;
	unsigned long long addit;
	
	TX_buf[com_mode][9] = RX_buf[com_mode][6];
	TX_buf[com_mode][10] = RX_buf[com_mode][7];
	ch[0] = RX_buf[com_mode][7];					//���O����ԍ�����
	ch_z = ch[0] - 0x30;
	if(ch_z == 0){ch_z = 10;}
	ch[0] = 0;

	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = SVD[ch_no[com_mode] -1].pwon_count;	//���ݓd��ON��
	read_change(com_mode,value,0,2);
	
	value = cmi_count;					//���݃G���[�^�C�}�[
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_pwon_count;	//�������d��ON��
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_time;	//�������G���[�^�C�}�[
	read_change(com_mode,value,0,2);
	
	add[0] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code / 10 + 0x30;		//ErrorLogCode
	add[1] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));					//�Ō���ɓǍ��l�ǉ�
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_quantity;		//����
	read_change(com_mode,value,2,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_velocity;		//����
	read_change(com_mode,value,3,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].sound_speed * 100;	//����
	read_change(com_mode,value,2,2);
	
	addit = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].total_count.DWORD / 1000000;	//�ώZ�l
	read_change(com_mode,addit,1,2);
	
	value = 0;		//UP�`������[ps]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,2);

	value = 0;	//DN�`������[ps]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,2);
	
	diff = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].dt * 32;		//�`�����ԍ�[ps]
	if (diff > 99999999) {diff = 99999999;}
	else if (diff < -99999999) {diff = -99999999;}
	read_change(com_mode,diff,0,2);

	value = 0;		//UP���ʎ���[ps]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,2);

	value = 0;		//DN���ʎ���[ps]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,2);

	err_st = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].status;		//status
	read_change0X_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	value = 0;	//Window Position[us]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_up;	//Gain 1st stage
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_down;	//Gain 2nd stage
	read_change(com_mode,value,0,2);

	value = 0;	//Peak[mV]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,2);

	value = 0;	//Threshold[mV]�i�Y���f�[�^�Ȃ��j
	read_change(com_mode,value,0,0);
	for (l = 0; l < 40; l++){
		value = 0;
		read_change(com_mode,value,0,1);
	}

}

/* �[�������f�[�^�Ǐo��(SFC9000�Ή�)(RD) */
void command_RD (short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	long value;
	long long diff;

	ch[0] = RX_buf[com_mode][7];                		//CH������
	ch_z = ch[0] - 0x30;
	ch[0] = 0;			
	for (m = 0; m <= ch_z; m++){
		TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
		TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
	}
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];		//CH�ԍ�����
		ch_number = ch[0] - 0x30;
		
		value = SVD[ch_number -1].viscos;								//���S�x
		read_change(com_mode,value,2,1);
		
		value = SVD[ch_number -1].zero_wave_max;		//��g�g�`�ő�l
		read_change(com_mode,value,0,1);
	
		value = SVD[ch_number -1].zero_wave_min;		//��g�g�`�ŏ��l
		read_change(com_mode,value,0,1);

		diff = SVD[ch_number -1].zero_delta_ts.DWORD * 32;	//�`�����ԍ�
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,1);

		value = SVD[ch_number -1].zero_fifo_pos;		//FIFO��g�g�`���o�ʒu
		read_change(com_mode,value,0,1);
		
		value = SVD[ch_number -1].zero_gain_1st;		//Gain1
		read_change(com_mode,value,0,1);

		value = SVD[ch_number -1].zero_gain_2nd;		//Gain2
		read_change(com_mode,value,0,1);

		value = SVD[ch_number -1].zero_p1p2;		//��g�̍�(P1-P2)
		read_change(com_mode,value,0,1);		
	}
	
}

/* �[�������f�[�^�Ǐo��(RZ) */
 void command_RZ (short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	long value;
	long long diff;

	ch[0] = RX_buf[com_mode][7];                		//CH������
	ch_z = ch[0] - 0x30;
	ch[0] = 0;			
	for (m = 0; m <= ch_z; m++){
		TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
		TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
	}
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];		//CH�ԍ�����
		ch_number = ch[0] - 0x30;
		
		value = SVD[ch_number -1].viscos;				//���S�x
		read_change(com_mode,value,2,1);
		
		value = 0;		//Tu�i�Y���f�[�^�Ȃ��j
		read_change(com_mode,value,0,1);
	
		value = 0;		//Td�i�Y���f�[�^�Ȃ��j
		read_change(com_mode,value,0,1);

		diff = SVD[ch_number -1].zero_delta_ts.DWORD * 32;		//Dt
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,1);

		value = 0;		//Window Position�i�Y���f�[�^�Ȃ��j
		read_change(com_mode,value,0,1);
		
		value = SVD[ch_number -1].zero_gain_1st;		//Gain1
		read_change(com_mode,value,0,1);

		value = SVD[ch_number -1].zero_gain_2nd;		//Gain2
		read_change(com_mode,value,0,1);

		value = 0;		//Threshold�i�Y���f�[�^�Ȃ��j
		read_change(com_mode,value,0,1);
	}

 }
/* �g�`�ُ픻��l�ݒ�Ǐo��(RE) */
 void command_RE (short com_mode){

	long value;

	value = SVD[ch_no[com_mode] -1].wave_vth;			//�G���v�e�B�Z���T����臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].correlate_level;		//���Z�ُ픻��臒l
	read_change(com_mode,value,1,2);

	value = SVD[ch_no[com_mode] -1].correlate_time;			//���Z�ُ픻���
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].balance_level;			//�g�`�A���o�����X臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].saturation_level;		//AGC�s�\����臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].attenuate_level;		//�g�`��������臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_wave_vth;			//�G���v�e�B�Z���T�x��臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_gain_level;	//�A���v�Q�C���x��臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_gain_count;	//�A���v�Q�C���x������
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_hold_time;		//�x�����莞��
	read_change(com_mode,value,2,0);
}

/* �A�b�e�l�[�^�Q�C���l�Ǐo��(RA) */
 void command_RA (short com_mode){

	long value;

//	value = SVD[ch_no[com_mode] -1].atn_gain;
	value = 0;
	read_change(com_mode,value,0,0);
		
}

/* ��t���j�A���C�Y���[�h�Ǐo��(Ry) */
 void command_Ry (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].LL_enable;
	read_change(com_mode,value,0,2);	
	
	value = SVD[ch_no[com_mode] -1].LL_kind;
	read_change(com_mode,value,0,0);
} 

/* �����Ǐo��(RM) */
 void command_RM (short com_mode){

	short value;

	value = MES[ch_no[com_mode] -1].sound_vel_f;				//����[m/s]
	read_change(com_mode,value,0,0);
}

/* �t�B���^�ݒ�Ǐo��(Rf) */
void command_Rf (short com_mode){
	
	short value;

	value = SVD[ch_no[com_mode] -1].filter_mode;	//�t�B���^���[�h
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].filter_avg;		//�ړ����ϐ�
	read_change(com_mode,value,0,0);
}

/* �ώZ�ڕW�l�Ǐo��(Rt) */
void command_Rt (short com_mode){

	long value;
	
	value = SVD[ch_no[com_mode] -1].target_total.INT32;
	read_change(com_mode,value,2,0);
}

/* �ώZ�I�t�Z�b�g�l�Ǐo��(RT) */
void command_RT (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].total_offset_enable;		//ON/OFF
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].total_offset_value;		//���͒l
	read_change(com_mode,value,3,0);
}

/* ���[�J�[�ݒ�Ǐo��(R1) */
 void command_R1 (short com_mode){

	 short l;
	 short value;
	 unsigned short *pt;
	 short sn_version;
	 short sn_ver_cal;
	 char add[20] = {0};

	 value = 0;		//�␳10%�i�Y���f�[�^�Ȃ��j
	 read_change(com_mode,value,2,2);

	 value = 0;		//�␳25%�i�Y���f�[�^�Ȃ��j
	 read_change(com_mode,value,2,2);

	 value = 0;		//�␳50%�i�Y���f�[�^�Ȃ��j
	 read_change(com_mode,value,2,2);

	 value = 0;		//�␳75%�i�Y���f�[�^�Ȃ��j
	 read_change(com_mode,value,2,2);

	 value = 0;		//�␳100%�i�Y���f�[�^�Ȃ��j
	 read_change(com_mode,value,2,2);

	 pt = &SVD[ch_no[com_mode] -1].c_serial[0];			//�V���A���i���o�[
	 for (l = 0; l < 8; l++){
		 sn_version = *pt;
		 sn_ver_cal = sn_version / 0x0100;
		 if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			 add[2*l] = sn_ver_cal;
		 }
		 sn_ver_cal = sn_version % 0x0100;
		 if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			 add[2*l + 1] = sn_ver_cal;
		 }
		 pt++;
	 }
		 strncat(TX_buf[com_mode],add,sizeof(add));

 }

/* �f�o�b�O���[�h�ݒ�Ǐo��(R2) */
 void command_R2 (short com_mode){

	 short value;

	 value = MES[ch_no[com_mode] -1].test_enable;				//���ʏo�̓e�X�g
	 read_change(com_mode,value,0,2);

	 value = MES[ch_no[com_mode] -1].test_flow;					//���ʏo�̓e�X�g�l
	 read_change(com_mode,value,2,2);

	 value = MES[ch_no[com_mode] -1].test_port_out;				//�o�̓|�[�g�e�X�g
	 read_change(com_mode,value,0,2);

	 value = MES[ch_no[com_mode] -1].test_port_in;				//���̓|�[�g���
	 read_change(com_mode,value,0,0);
		
 }

/*******************************************
 * Function : command_R3
 * Summary  : �[���N���X����f�[�^�ǂݏo��
 * Argument : 
 * Return   : 
 * Caution  : None
 * Note     : �������֑���f�[�^�ǂݏo����R7
 * *****************************************/
 void command_R3 (short com_mode){

	short l;
	long value;
	long long diff;
	unsigned long long addit;
	short pch = ch_no[com_mode] - 1;
	float TimDif;

	if(RX_buf[com_mode][7] != '0'){		//���莞�f�[�^
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//1. ����[mL/min]
		value = MES[ch_no[com_mode] -1].ml_min_now;
		read_change(com_mode,value,2,2);

		//2. ����[m/s]
		value = MES[ch_no[com_mode] -1].flow_vel_c / 100;
		read_change(com_mode,value,3,2);

		//3. ����[m/s]
		value = MES[ch_no[com_mode] -1].sound_vel_f * 100;
		read_change(com_mode,value,2,2);
		
		//4. �ώZ�l[mL/min]
		addit = MES[ch_no[com_mode] -1].addit_buff.DWORD / 10000;
		read_change(com_mode,addit,3,2);
		
		//5. UP�`������[us]
		TimDif = GetTimDif(pch, 0);
		read_change(com_mode, TimDif * 1000.0, 3, ADD_CMM_AFR_VAL);

		//6. DN�`������[us]
		TimDif = GetTimDif(pch, 1);
		read_change(com_mode, TimDif * 1000.0, 3, ADD_CMM_AFR_VAL);

		//7. �`�����ԍ�[ps]
		diff = MES[ch_no[com_mode] -1].delta_ts_zero * 32;
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);

		//8. UP���ʎ���[ps]
		value = MES[pch].FwdSurplsTim;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//9. DN���ʎ���[ps]
		value = MES[pch].RevSurplsTim;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//10. status
		value = MAIN[ch_no[com_mode] -1].err_condition;
		read_change0X_status(com_mode,value);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//11. Window Position[us]
		value = ((MES[pch].fifo_ch_read & 0xFF) << 8);
		value += (MES[pch].fifo_no_read & 0xFF);
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//12. Gain 1st stage(���g�p)
		value = get_attenuator_gain(ch_no[com_mode]-1);
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//13. Gain 2nd stage
		value = MES[ch_no[com_mode] -1].amp_gain_for;
		read_change(com_mode,value,0,2);

		//14. Peak[mV] (�����炭�A���v�������512mV���ő�)
		// value = 512 * (MES[pch].rev_max_data - AD_BASE_UNIT) / 4095;
        value = 512 * (MES[pch].rev_max_data - AD_BASE_UNIT) / AD_MAX_UNIT;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//15. �ł����ݎ��g��(014E�ł͑ł����ݎ��g����\��)
		value = SVD[pch].drive_freq;				
		read_change(com_mode,value, 0, ADD_NO_CMM);

		//16-25. �㗬�g�`�s�[�N�ʒu
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].FwdWavMaxPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].FwdWavMinPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
		}

		//26-35. �㗬�g�`�s�[�N�l
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].FwdWavMaxPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].FwdWavMinPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//36-45. �����g�`�s�[�N�ʒu
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].RevWavMaxPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].RevWavMinPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
		}

		//46-55. �㗬�g�`�s�[�N�ʒu
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].RevWavMaxPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].RevWavMinPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}
	}else{				//�[���������f�[�^
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//1. ����[mL/min]
		value = SVD[ch_no[com_mode] -1].zero_flow_qat.DWORD;
		read_change(com_mode,value,2,2);

		//2. ����[m/s]
		value = SVD[ch_no[com_mode] -1].zero_flow_vel.DWORD / 100;
		read_change(com_mode,value,3,2);

		//3. ����[m/s]
		value = SVD[ch_no[com_mode] -1].zero_sound_spd * 100;
		read_change(com_mode,value,2,2);
		
		//4. �ώZ�l[mL/min]
		addit = SVD[ch_no[com_mode] -1].zero_addit.DWORD / 10000;
		read_change(com_mode,addit,3,2);
	
		//5. UP�`������[ps]
		value = SVD[pch].zero_FwdTimDif.DWORD;
		read_change(com_mode,value, 3, ADD_CMM_AFR_VAL);
	
		//6. DN�`������[ps]
		value = SVD[pch].zero_RevTimDif.DWORD;
		read_change(com_mode,value, 3, ADD_CMM_AFR_VAL);

		//7. �`�����ԍ�[ps]
		diff = SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD * 32;
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);
	
		//8. UP���ʎ���[ps]
		value = SVD[pch].zero_FwdSurplsTim;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);
	
		//9. DN���ʎ���[ps]
		value = SVD[pch].zero_RevSurplsTim;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//10. status
		value = SVD[ch_no[com_mode] -1].zero_condition;
		read_change0X_status(com_mode,value);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//11. Window Position[us]
		value = ((SVD[pch].zero_fifo_ch & 0xFF) << 8);
		value += (SVD[pch].zero_fifo_pos & 0xFF);
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//12. Gain 1st stage(���g�p)
		value = SVD[ch_no[com_mode] -1].zero_gain_1st;
		read_change(com_mode,value,0,2);

		//13. Gain 2nd stage
		value = SVD[ch_no[com_mode] -1].zero_gain_2nd;
		read_change(com_mode,value,0,2);
	
		//14. Peak[mV]
		// value = 512 * (SVD[pch].zero_wave_max - AD_BASE_UNIT) / 4095;
        value = 512 * (SVD[pch].zero_wave_max - AD_BASE_UNIT) / AD_MAX_UNIT;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);
	
		//15. �ł����ݎ��g��(014E�ł͑ł����ݎ��g����\��)
		value = SVD[pch].zero_drive_freq;
		read_change(com_mode,value, 0, ADD_NO_CMM);

		//16-25. �㗬�g�`�s�[�N�ʒu
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_FwdWavPekPosLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//26-35. �㗬�g�`�s�[�N�l
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_FwdWavPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//36-45. �����g�`�s�[�N�ʒu
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_RevWavPekPosLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//46-55. �㗬�g�`�s�[�N�ʒu
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_RevWavPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}
	}
 }
#if defined(MESWAVEXP)

/************************************************************
 * Fucntion : WavInBuff
 * Summary  : �g�`�f�[�^�𑗐M�o�b�t�@�ɓ����
 * Argument : com_mode -> �ʐM���[�h
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : command_R4, command_R8�̃R�[�h���ڐA
 *            ���������L����͍폜
 *            if((MES[ch_no[com_mode]-1].err_status & ERR_JUDGE_EMPTY) != 0){	//Empty Sensor�� �g�`�f�[�^0
 * 	              ofs = 3;
 *            }
 *            ���R : ERR_FLOW_COUNT �� ERR_JUDGE_EMPTY ���܂܂��
************************************************************/
void WavInBuff(short com_mode){
	short ofs;
	short l;
	short value;
	
	if(MES[ch_no[com_mode]-1].wave_monitor_mode == 0 ||				//��ɍX�V�ݒ�
		(MES[ch_no[com_mode]-1].err_status & ERR_FLOW_CONT) == 0){	//���ʃG���[����
			
		if(RX_buf[com_mode][8] == 0x2c){		// 0�`2��FWD�̃f�[�^
			ofs = RX_buf[com_mode][7] - '0';
			if(ofs == 0) {
				for(l = 0; l < MESWAVSIZ + 100; l++){
					fow_wave_data[l] = MES[ch_no[com_mode]-1].fow_data[l];
				}
			}
		}else{									// 10�`12��REV�̃f�[�^
			ofs = RX_buf[com_mode][8] - '0';
			if(ofs == 0) {
				for(l = 0; l < MESWAVSIZ + 100; l++){
					rev_wave_data[l] = MES[ch_no[com_mode]-1].rev_data[l];
				}
			}
		}
			
	}

	if(ofs <= (MESWAVSIZ / 100)) {
		for (l = 0; l < 100; l++){
			if(RX_buf[com_mode][8] == 0x2c){						//FWD�f�[�^
//				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)/4;	//��g�g�`�f�[�^
				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
			}else{													//REV�f�[�^
//				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)/4;	//��g�g�`�f�[�^
				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
			}
			read_change(com_mode,value,0,1);
		}
	}else{
		for (l = 0; l < 100; l++){
			value = 0;								//�g�`�f�[�^0�F����Block(3�`9 or 13�`19)�� or Empty Sensor������
			read_change(com_mode,value,0,1);
		}
	}
}
#endif

/* ����g�`�Ǐo��(R4) */
void command_R4 (short com_mode){

	short l;
	short value;
	short *data;
	short ofs;

	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	if(RX_buf[com_mode][8] == 0x2c){						//Data block����1�����̏ꍇ
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//�Ō����','�ǉ�
		TX_buf[com_mode][11] = '0';
	}else{													//Data block����2�����̏ꍇ
		TX_buf[com_mode][10] = RX_buf[com_mode][8];
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//�Ō����','�ǉ�
		TX_buf[com_mode][12] = '0';
	}
#if 0
	for (l = 0; l < 100; l++){
		value = (MES[ch_no[com_mode] -1].fow_data[l]-AD_BASE)/4;		//��g�g�`�f�[�^
		read_change(com_mode,value,0,1);
	}
#else

#if defined(MESWAVEXP)
	WavInBuff(com_mode);
#else
	if(MES[ch_no[com_mode]-1].wave_monitor_mode == 0 ||				//��ɍX�V�ݒ�
		(MES[ch_no[com_mode]-1].err_status & ERR_FLOW_CONT) == 0){	//���ʃG���[����
		if(RX_buf[com_mode][8] == 0x2c){		// 0�`2��FWD�̃f�[�^
			ofs = RX_buf[com_mode][7] - '0';
			if(ofs == 0) {
				for(l = 0; l < 300; l++){
					fow_wave_data[l] = MES[ch_no[com_mode]-1].fow_data[l];
				}
			}
		}else{									// 10�`12��REV�̃f�[�^
			ofs = RX_buf[com_mode][8] - '0';
			if(ofs == 0) {
				for(l = 0; l < 300; l++){
					rev_wave_data[l] = MES[ch_no[com_mode]-1].rev_data[l];
				}
			}
		}
	}

	if(ofs <= 2) {
		for (l = 0; l < 100; l++){
			if(RX_buf[com_mode][8] == 0x2c){						//FWD�f�[�^
//				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)/4;	//��g�g�`�f�[�^
				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
			}else{													//REV�f�[�^
//				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)/4;	//��g�g�`�f�[�^
				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
			}
			read_change(com_mode,value,0,1);
		}
	}else{
		for (l = 0; l < 100; l++){
			value = 0;											//��g�g�`�f�[�^
			read_change(com_mode,value,0,1);
		}
	}
#endif //MESWAVEXP

#endif

}

/* ���[�J�[�ݒ�Ǐo��(SFC9000�Ή�)(R5) */
void command_R5 (short com_mode){

	short l;
	long value;
	unsigned short *pt;
	short sn_version;
	short sn_ver_cal;
	char add[20] = {0};

	value = SVD[ch_no[com_mode] -1].wave_vth;				//�G���v�e�B�Z���T����臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].correlate_level;			//���Z�ُ픻��臒l
	read_change(com_mode,value,1,2);

	value = SVD[ch_no[com_mode] -1].correlate_time;			//���Z�ُ픻���
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].balance_level;			//�g�`�A���o�����X臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].saturation_level;			//AGC�s�\����臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].attenuate_level;			//�g�`��������臒l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].gain_step;				//�Q�C���X�e�b�v��
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sound_vel_sel;			//�����Œ�or����
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sound_vel_fix;			//�����ݒ�
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sound_vel_filter;			//�����t�B���^���萔
	read_change(com_mode,value,1,2);
	
	value = SVD[ch_no[com_mode] -1].viscos_auto;				//���S�x�Œ�or����
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].drive_pls;				//�ō��݃p���X
	read_change(com_mode,value,0,2);

//	value = SVD[ch_no[com_mode] -1].atn_gain;				//�A�b�e�l�[�^�Q�C���l
	value = 0;
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].fifo_ch_init;				//FIFO CH �����l
	read_change(com_mode,value,0,2);
	
	value = SVD[0].cunet_delay;						//CUnet �đ��M�ҋ@����	���SCH����
	read_change(com_mode,value,0,2);	

	value = SVD[ch_no[com_mode] -1].search_sw;				//���֒l�T�[�`����or�L��
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].damp_mode;				//�C�A�΍􃂁[�h
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].corr_up.DWORD;			//���֒l������l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].corr_low.DWORD;			//���֒l�������l
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].inc;					//�ُ�z�[���h����
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].hldt;				//���C�g���~�b�g�z�[���h�N���A
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rlt;					//���C�g���~�b�g
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].odpd;				//���C�g���~�b�g1st
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl1tg;				//�^�[�Q�b�g1st
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl1av;				//�A�x���[�W1st
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].odpl;				//���C�g���~�b�g2nd
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl2tg;				//�^�[�Q�b�g2nd
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl2av;				//�A�x���[�W2nd
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].rl2d;				//�_���s���O�{��
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].rl2hc;				//�z�[���h�N���A
	read_change(com_mode,value,1,2);

	value = SVD[ch_no[com_mode] -1].dump_var;				//�σ_���s���O
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].dump_mul;				//�σ_���s���O�{��
	read_change(com_mode,value,2,2);
	
	pt = &SVD[ch_no[com_mode] -1].c_serial[0];			//�V���A���i���o�[		
	for (l = 0; l < 8; l++){
		sn_version = *pt;
		sn_ver_cal = sn_version / 0x0100;
		if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			add[2*l] = sn_ver_cal;
		}
		sn_ver_cal = sn_version % 0x0100;
		if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			add[2*l + 1] = sn_ver_cal;
		}
		pt++;
	}
		strncat(TX_buf[com_mode],add,sizeof(add));			//�Ō���ɓǍ��l�ǉ�
		
}

/* �f�o�b�O���[�h�ݒ�Ǐo��(SFC9000�Ή�)(R6) */
void command_R6 (short com_mode){

	short value;

	value = MES[ch_no[com_mode] -1].test_enable;		//���ʏo�̓e�X�gON/OFF
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].test_flow;		//���ʏo�̓e�X�g�l
	read_change(com_mode,value,2,2);
	
	value = MES[ch_no[com_mode] -1].test_err_enable;		//�G���[�e�X�gON/OFF
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].test_err_code;		//�����G���[���
	read_change(com_mode,value,0,0);
}

/*******************************************
 * Function : command_R7
 * Summary  : �������֑���f�[�^�ǂݏo��
 * Argument : 
 * Return   : 
 * Caution  : None
 * Note     : �[���N���X����f�[�^�ǂݏo����R3
 * *****************************************/
void command_R7 (short com_mode){

	short l;
	long value;
	long long diff;
	unsigned short err_st;
	unsigned long long addit;

	if(RX_buf[com_mode][7] != '0'){		//���莞�f�[�^
	
		//�g�`�\��(�S�g�`or����g�`)�I��
		if(RX_buf[com_mode][9] == '0'){MES[ch_no[com_mode] -1].wave_monitor_mode = 0;}
		else{MES[ch_no[com_mode] -1].wave_monitor_mode = 1;}
		
		//�ԐM��쐬
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//1. ����[mL/min]
		value = MES[ch_no[com_mode] -1].ml_min_now;
		read_change(com_mode,value,2,2);

		//2. ����[m/s]
		value = MES[ch_no[com_mode] -1].flow_vel_c / 100;
		read_change(com_mode,value,3,2);

		//3. ����[m/s]
		value = MES[ch_no[com_mode] -1].sound_vel_f;
		read_change(com_mode,value,0,2);

		//4. �ώZ�l[mL/min]
		addit = MES[ch_no[com_mode] -1].addit_buff.DWORD  / 10000;
		read_change(com_mode,addit,3,2);

		//5. ��g�g�`�ő�l
		value = MES[ch_no[com_mode] -1].rev_max_data;
		read_change(com_mode,value,0,2);

		//6. 
		value = MES[ch_no[com_mode] -1].rev_min_data;					
		read_change(com_mode,value,0,2);

		//7. �`�����ԍ�[ps]
		diff = MES[ch_no[com_mode] -1].delta_ts_zero * 32;
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);

		//8. ���֒l��
		value = (MES[ch_no[com_mode] -1].correlate * 1000);
		read_change(com_mode,value,0,2);

		//9. �[���_�I�t�Z�b�g
		value = (SVD[ch_no[com_mode] -1].zero_offset - 4000) * 32;
		read_change(com_mode,value,0,2);

		//10. status
		err_st = MAIN[ch_no[com_mode] -1].err_condition;
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//11. FIFO��g�g�`���o�ʒu
		if((SVD[ch_no[com_mode] -1].fix_data & 0x08) != 0){  //�Œ�l�ݒ�
			if(SVD[ch_no[com_mode] -1].fix_fifo_no_read == 0){
				value = MES[ch_no[com_mode] -1].zero_fifo_no_read;
			}else{
				value = SVD[ch_no[com_mode] -1].fix_fifo_no_read;
			}
		}else{
			value = MES[ch_no[com_mode] -1].fifo_no_read;
		}
		read_change(com_mode,value,0,2);
		
		//12. Gain 1st stage
		value = get_attenuator_gain(ch_no[com_mode]-1);
		read_change(com_mode,value,0,2);

		//13. Gain 2nd stage
		if((SVD[ch_no[com_mode] -1].fix_data & 0x02) != 0){  //�Œ�l�ݒ�
			if(SVD[ch_no[com_mode] -1].fix_amp_gain_rev == 0){
				value = MES[ch_no[com_mode] -1].zero_amp_gain_rev;
			}else{
				value = SVD[ch_no[com_mode] -1].fix_amp_gain_rev;
			}
		}else{
			value = MES[ch_no[com_mode] -1].amp_gain_for;
		}
		read_change(com_mode,value,0,2);

		//14. FIFO CH
		if((SVD[ch_no[com_mode] -1].fix_data & 0x04) != 0){  //�Œ�l�ݒ�
			if(SVD[ch_no[com_mode] -1].fix_fifo_ch_read == 0){
				value = MES[ch_no[com_mode] -1].zero_fifo_ch_read;
			}else{
				value = SVD[ch_no[com_mode] -1].fix_fifo_ch_read;
			}
		}else{
			value = MES[ch_no[com_mode] -1].fifo_ch_read;
		}
		read_change(com_mode,value,0,2);
		
		//15. ��g�̍�(P1-P2)
		if(MES[ch_no[com_mode] -1].max_point_sub_f >= LIM_OVERFLOW)	{ value = (LIM_OVERFLOW - 1); }
		else if(MES[ch_no[com_mode] -1].max_point_sub_f <= -LIM_OVERFLOW)	{ value = -(LIM_OVERFLOW - 1); }
		else { value = MES[ch_no[com_mode] -1].max_point_sub_f; }
		read_change(com_mode, 0, 0, ADD_NO_CMM);

		//16-55. �������֒l
		if(MES[ch_no[com_mode] -1].wave_monitor_mode == 0 ||					//��ɍX�V�ݒ�
		(MES[ch_no[com_mode]-1].err_status & ERR_FLOW_CONT) == 0){		//���ʃG���[����
				
			if((MES[ch_no[com_mode]-1].err_status & ERR_JUDGE_EMPTY) != 0){	//Empty Sensor�� �g�`�f�[�^0
				for (l = 0; l < 40; l++){
					sum_abs_data[l] = 0;	//�g�`�f�[�^0
				}
			}else{
				for (l = 0; l < 40; l++){
					sum_abs_data[l] = SAVE[ch_no[com_mode] -1].sum_abs_com[l];	//16-55. �������֒l
				}
			}
		}

		for (l = 0; l < 40; l++){
			value = sum_abs_data[l];		//�������֒l
			read_change(com_mode,value,0,1);
		}
	}else{				//�[���������f�[�^
		//�g�`�\��(�S�g�`or����g�`)�I��
		if(RX_buf[com_mode][9] == '0'){MES[ch_no[com_mode] -1].wave_monitor_mode = 0;}
		else{MES[ch_no[com_mode] -1].wave_monitor_mode = 1;}
		
		//�ԐM��쐬
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	
		value = SVD[ch_no[com_mode] -1].zero_flow_qat.DWORD;		//����[mL/min]
		read_change(com_mode,value,2,2);
	
		value = SVD[ch_no[com_mode] -1].zero_flow_vel.DWORD / 100;		//����[m/s]
		read_change(com_mode,value,3,2);
	
		value = SVD[ch_no[com_mode] -1].zero_sound_spd;			//����[m/s]
		read_change(com_mode,value,0,2);
	
		addit = SVD[ch_no[com_mode] -1].zero_addit.DWORD / 10000;		//�ώZ�l[mL/min]
		read_change(com_mode,addit,3,2);
	
		value = SVD[ch_no[com_mode] -1].zero_wave_max;			//��g�g�`�ő�l
		read_change(com_mode,value,0,2);
	
		value = SVD[ch_no[com_mode] -1].zero_wave_min;			//��g�g�`�ŏ��l
		read_change(com_mode,value,0,2);
	
		diff = SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD * 32;		//�`�����ԍ�[ps]
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);
	
		value = (SVD[ch_no[com_mode] -1].zero_correlate * 1000);	//���֒l��
		read_change(com_mode,value,0,2);
	
		value = (SVD[ch_no[com_mode] -1].zero_zero_offset - 4000) * 32;	//�[���_�I�t�Z�b�g
		read_change(com_mode,value,0,2);

		err_st = SVD[ch_no[com_mode] -1].zero_condition;			//status
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		value = SVD[ch_no[com_mode] -1].zero_fifo_pos;			//FIFO��g�g�`���o�ʒu
		read_change(com_mode,value,0,2);
	
//		value = SVD[ch_no[com_mode] -1].zero_gain_for;			//Gain 1st stage
		value = SVD[ch_no[com_mode] -1].zero_gain_1st;			//Gain 1st stage
		read_change(com_mode,value,0,2);

//		value = SVD[ch_no[com_mode] -1].zero_gain_rev;			//Gain 2nd stage
		value = SVD[ch_no[com_mode] -1].zero_gain_2nd;			//Gain 2nd stage
		read_change(com_mode,value,0,2);
	
		value = SVD[ch_no[com_mode] -1].zero_fifo_ch;				//FIFO CH
		read_change(com_mode,value,0,2);
	
		value = SVD[ch_no[com_mode] -1].zero_p1p2;				//��g�̍�(P1-P2)
		read_change(com_mode,value,0,0);

		if((SVD[ch_no[com_mode] -1].zero_condition & CON_EMPTY) != 0){	//Empty Sensor�� �g�`�f�[�^0
			for (l = 0; l < 40; l++){
				value = 0;		//�������֒l;
				read_change(com_mode,value,0,1);
			}
		}else{
			for (l = 0; l < 40; l++){
				// value = SVD[ch_no[com_mode] -1].zero_sum_abs[l];		//�������֒l;
				value = 0;
			    read_change(com_mode,value,0,1);
			}
		}
	}
}

/* ����f�[�^�Ǐo��(SFC9000�Ή��ER7�R�}���h�Z�k��)(RC) */
void command_RC (short com_mode){

	short l;
	long value;
	long long diff;
	unsigned short err_st;
	unsigned long long addit;

	short m;
	short ch_z;
	short ch_number;
	char ch[1] = {0};	
	
/* �V���O���R�}���h */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		
		value = MES[ch_no[com_mode] -1].ml_min_now;			//����[mL/min]
		read_change(com_mode,value,2,2);
		
		value = MES[ch_no[com_mode] -1].sound_vel_f;			//����[m/s]
		read_change(com_mode,value,0,2);
		
		value = MES[ch_no[com_mode] -1].rev_max_data;			//��g�g�`�ő�l
		read_change(com_mode,value,0,2);
	
		value = MES[ch_no[com_mode] -1].rev_min_data;			//��g�g�`�ŏ��l
		read_change(com_mode,value,0,2);
	
		diff = MES[ch_no[com_mode] -1].delta_ts_zero * 32;		//�`�����ԍ�[ps]
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);
	
		value = (SVD[ch_no[com_mode] -1].zero_offset - 4000) * 32;	//�[���_�I�t�Z�b�g
		read_change(com_mode,value,0,2);

		err_st = MAIN[ch_no[com_mode] -1].err_condition;		//status
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	
		value = get_attenuator_gain(ch_no[com_mode]-1);			//Gain 1st stage
		read_change(com_mode,value,0,2);

		value = MES[ch_no[com_mode] -1].amp_gain_for;			//Gain 2nd stage
		read_change(com_mode,value,0,2);
	
		value = MES[ch_no[com_mode] -1].max_point_sub_f;		//��g�̍�(P1-P2)
		read_change(com_mode,value,0,0);
	}
/* �}���`�R�}���h */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];					//CH������
		ch_z = ch[0] - 0x30;
		ch[0] = 0;				
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH�ԍ�����
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma));
			
			value = MES[ch_number -1].ml_min_now;			//����[mL/min]
			read_change(com_mode,value,2,2);
			
			value = MES[ch_number -1].sound_vel_f;			//����[m/s]
			read_change(com_mode,value,0,2);
			
			value = MES[ch_number -1].rev_max_data;			//��g�g�`�ő�l
			read_change(com_mode,value,0,2);
		
			value = MES[ch_number -1].rev_min_data;			//��g�g�`�ŏ��l
			read_change(com_mode,value,0,2);
		
			diff = MES[ch_number -1].delta_ts_zero * 32;		//�`�����ԍ�[ps]
			if (diff > 99999999) {diff = 99999999;}
			else if (diff < -99999999) {diff = -99999999;}
			read_change(com_mode,diff,0,2);
		
			value = (SVD[ch_number -1].zero_offset - 4000) * 32;	//�[���_�I�t�Z�b�g
			read_change(com_mode,value,0,2);
	
			err_st = MAIN[ch_number -1].err_condition;		//status
			read_change0X_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));
			
			value = get_attenuator_gain(ch_number-1);			//Gain 1st stage
			read_change(com_mode,value,0,2);
	
			value = MES[ch_number -1].amp_gain_for;			//Gain 2nd stage
			read_change(com_mode,value,0,2);
		
			value = MES[ch_number -1].max_point_sub_f;		//��g�̍�(P1-P2)
			read_change(com_mode,value,0,0);			
		}
	}
}

/* ����g�`�Ǐo��(SFC9000�Ή�)(R8) */
void command_R8 (short com_mode){

	short l;
	short value;
	short *data;
	short ofs;
	short pch = ch_no[com_mode] -1;
	short AmpOfs; //�g�`�U���I�t�Z�b�g�l

	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	if(RX_buf[com_mode][8] == 0x2c){						//Data block����1�����̏ꍇ
		//�g�`�\��(�S�g�`or����g�`)�I��
		if(RX_buf[com_mode][9] == '0'){MES[pch].wave_monitor_mode = 0;}
		else{MES[pch].wave_monitor_mode = 1;}
		//�ԐM��쐬
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//�Ō����','�ǉ�
		TX_buf[com_mode][11] = '0';
	}else{													//Data block����2�����̏ꍇ
		//�g�`�\��(�S�g�`or����g�`)�I��
		if(RX_buf[com_mode][10] == '0'){MES[pch].wave_monitor_mode = 0;}
		else{MES[pch].wave_monitor_mode = 1;}
		//�ԐM��쐬
		TX_buf[com_mode][10] = RX_buf[com_mode][8];
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//�Ō����','�ǉ�
		TX_buf[com_mode][12] = '0';
	}
#if 0
	for (l = 0; l < 100; l++){
		value = (MES[pch].fow_data[l]-AmpOfs)/4;		//��g�g�`�f�[�^
		read_change(com_mode,value,0,1);
	}
#else

#if defined(MESWAVEXP)
	WavInBuff(com_mode);
#else
	if(MES[pch].wave_monitor_mode == 0 ||				//��ɍX�V�ݒ�
		(MES[pch].err_status & ERR_FLOW_CONT) == 0){	//���ʃG���[����

		if((MES[pch].err_status & ERR_JUDGE_EMPTY) != 0){	//Empty Sensor�� �g�`�f�[�^0
			ofs = 3;
		}else{		
			if(RX_buf[com_mode][8] == 0x2c){		// 0�`2��FWD�̃f�[�^
				ofs = RX_buf[com_mode][7] - '0';
				if(ofs == 0) {
					for(l = 0; l < 300; l++){
						fow_wave_data[l] = MES[pch].fow_data[l];
					}
				}
			}else{									// 10�`12��REV�̃f�[�^
				ofs = RX_buf[com_mode][8] - '0';
				if(ofs == 0) {
					for(l = 0; l < 300; l++){
						rev_wave_data[l] = MES[pch].rev_data[l];
					}
				}
			}
		}		
	}



	//�]���p
	if(SVD[pch].sum_step == 2){	//�ō��݉񐔏㗬�����e2��
		MES_SUB[pch].sample_cnt = 2;
	}else if(SVD[pch].sum_step == 3){	//�ō��݉񐔏㗬�����e3��
		MES_SUB[pch].sample_cnt = 3;
	}else if(SVD[pch].sum_step == 4){	//�ō��݉񐔏㗬�����e4��
		MES_SUB[pch].sample_cnt = 4;
	}else{	//�ō��݉񐔏㗬�����e4��
		MES_SUB[pch].sample_cnt = 4;
	}
	// AmpOfs = 2047 * MES_SUB[pch].sample_cnt;
	AmpOfs = AD_BASE_UNIT * MES_SUB[pch].sample_cnt;
	//�]���p

	
	
	if(ofs <= 2) {
		for (l = 0; l < 100; l++){
			if(RX_buf[com_mode][8] == 0x2c){						//FWD�f�[�^
//				value = (fow_wave_data[(ofs*100)+l] - AmpOfs)/4;	//��g�g�`�f�[�^
				// value = (fow_wave_data[(ofs*100)+l] - AmpOfs)>>4;	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
				value = (fow_wave_data[(ofs*100)+l] - AmpOfs);	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
			}else{													//REV�f�[�^
//				value = (rev_wave_data[(ofs*100)+l] - AmpOfs)/4;	//��g�g�`�f�[�^
				// value = (rev_wave_data[(ofs*100)+l] - AmpOfs)>>4;	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
				value = (rev_wave_data[(ofs*100)+l] - AmpOfs);	//��g�g�`�f�[�^(4�񕪉��Z���Ă���̂ōX��4�Ŋ�����1/16�Ƃ���)
			}
			read_change(com_mode,value,0,1);
		}
	}else{
		for (l = 0; l < 100; l++){
			value = 0;								//�g�`�f�[�^0�F����Block(3�`9 or 13�`19)�� or Empty Sensor������
			read_change(com_mode,value,0,1);
		}
	}
#endif //MESWAVEXP

#endif

}

/* �p���X�o��(R9) */
void command_R9 (short com_mode){

	short value;

	value = MES[ch_no[com_mode] -1].pls_test_enable;		//�p���X�o�̓e�X�gON/OFF
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].pls_test_ch1;		//�p���X�o��CH�I��1
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].pls_test_ch2;		//�p���X�o��CH�I��2
	read_change(com_mode,value,0,0);
}

/* �������[�h�f�[�^�Ǐo��(SFC9000�Ή�)(Ri) */
void command_Ri (short com_mode){

	short l;
	long value;
	
	for (l = 0; l < 6; l++){
		value = wave_hight[0][l];		//�g�`�f�[�^(IN��)
		read_change(com_mode,value,0,1);
	}
	for (l = 0; l < 6; l++){
		value = wave_hight[1][l];		//�g�`�f�[�^(OUT��)
		read_change(com_mode,value,0,1);
	}
}

/* SW��ԃ`�F�b�N(RX) */
void command_RX (short com_mode){

	short value;

	value = disp_ch_read();					//CH���[�^���[SW���
	read_change(com_mode,value,0,2);

	value = disp_cunet_read();				//CUnet���[�^���[SW���
	read_change(com_mode,value,0,2);
	
	value = disp_zero_read();				//�[������SW���
	read_change(com_mode,value,0,0);
}

/* �g�`��ԓǏo��(SFC9000�Ή�)(RW) */
void command_RW (short com_mode){

	short m, ch_z, ch_number;
	long value;
	unsigned short err_st;
	char ch[1] = {0};

	/* �V���O���R�}���h */
	if(ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		err_st = MAIN[ch_no[com_mode] -1].err_condition;	//�A���[�������X�e�[�^�X
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		value = MES[ch_no[com_mode] -1].rev_max_data;			//��g�g�`�ő�l
		read_change(com_mode,value,0,2);

		value = MES[ch_no[com_mode] -1].rev_min_data;			//��g�g�`�ŏ��l
		read_change(com_mode,value,0,2);

		value = MES[ch_no[com_mode] -1].amp_gain_for;			//�㗬�Q�C���l
		read_change(com_mode,value,0,0);
	/* �}���`�R�}���h */
	}else if(ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];			//CH������
		ch_z = ch[0] - 0x30;
		for(m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for(m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH�ԍ�����
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma));

			err_st = MAIN[ch_number -1].err_condition;	//�A���[�������X�e�[�^�X
			read_change0X_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));

			value = MES[ch_number -1].rev_max_data;			//��g�g�`�ő�l
			read_change(com_mode,value,0,2);

			value = MES[ch_number -1].rev_min_data;			//��g�g�`�ŏ��l
			read_change(com_mode,value,0,2);

			value = MES[ch_number -1].amp_gain_for;			//�㗬�Q�C���l
			read_change(com_mode,value,0,0);
		}
	}
}

/* �Z���T�p���X�ݒ�Ǐo��(SFC014E�݊�)(Rp) */
 void command_Rp (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].drive_pls;			//�ō��݃p���X
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].drive_search;		//�������[�h
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].drive_freq;		//�쓮���g��
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].start_freq;		//�T�[�`�J�n���g��
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].stop_freq;			//�T�[�`��~���g��
	read_change(com_mode,value,0,0);
} 

/* �Z���T�V���A���i���o�[�Ǐo��(SFC014E�݊�)(Rn) */
 void command_Rn (short com_mode){

	short l;
	unsigned short *pt;
	short sn_version;
	short sn_ver_cal;
	char add[20] = {0};

	pt = &SVD[ch_no[com_mode] -1].s_serial[0];			//�Z���T�V���A���i���o�[		
	for (l = 0; l < 8; l++){
		sn_version = *pt;
		sn_ver_cal = sn_version / 0x0100;
		if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			add[2*l] = sn_ver_cal;
		}
		sn_ver_cal = sn_version % 0x0100;
		if ( (sn_ver_cal >= 0x30 && sn_ver_cal <= 0x39) || (sn_ver_cal >= 0x41 && sn_ver_cal <= 0x5A) ){	//0-9orA-Z
			add[2*l + 1] = sn_ver_cal;
		}
		pt++;
	}
	strncat(TX_buf[com_mode],add,sizeof(add));			//�Ō���ɓǍ��l�ǉ�
} 

/* �Z���T���Ǐo��(�]���p)(Ra) */
 void command_Ra (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].sns_option;			//�Z���T�I�v�V����
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sns_disL;		//�Z���T�ԋ���(L)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sns_disL_l;		//�Z���T�ԋ���(L_l)
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sns_tau;		//���ʎ���
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sns_coef;		//�݊��W��
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].adc_clock;		//ADC�N���b�N
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].wind_offset;		//WINDOW�I�t�Z�b�g
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sum_start;		//�������֊J�n�ʒu
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sum_end;		//�������֏I���ʒu
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sum_step;		//�������֊Ԋu
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].fix_data;		//�Œ�l�ݒ�
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].fix_amp_gain_rev;		//Wiper Position(�Œ�l)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].fix_fifo_ch_read;		//FIFO CH(�Œ�l)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].fix_fifo_no_read;		//Leading Position(�Œ�l)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].ZerCrsSttPnt;		//Zero Cross Start Point
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].ZerCrsUseNum;		//Zero Cross Use Number
	read_change(com_mode,value,0,0);
} 

/* �X�e�[�^�X�Ǎ��i16�i4�����j*/
void read_change0X_status(short com_mode,unsigned short log_st){

	unsigned short log_st_cal;
	char add[20] = {0};

	log_st_cal = log_st / 0x1000;

	if (log_st_cal <=9)		add[0] = log_st_cal  + 0x30;	// 0x30:ASCII��16�i�����ϊ�0-9
	else	/*A-F*/			add[0] = log_st_cal  + 0x37;	// 0x37:ASCII��16�i�����ϊ�A-F
	
	log_st_cal = (log_st % 0x1000) / 0x100;
	if (log_st_cal <=9)		add[1] = log_st_cal  + 0x30;
	else	/*A-F*/			add[1] = log_st_cal  + 0x37;
	
	log_st_cal = (log_st % 0x100) / 0x10;
	if (log_st_cal <=9)		add[2] = log_st_cal  + 0x30;
	else	/*A-F*/			add[2] = log_st_cal  + 0x37;

	log_st_cal = log_st % 0x10;
	if (log_st_cal <=9)		add[3] = log_st_cal  + 0x30;
	else	/*A-F*/			add[3] = log_st_cal  + 0x37;

	strncat(TX_buf[com_mode],add,sizeof(add));					//�Ō���ɓǍ��l�ǉ�

}

/* �X�e�[�^�X�Ǎ��i16�i2�����j*/
void read_change0X_2digit_status(short com_mode,unsigned short log_st){

	unsigned short log_st_cal;
	char add[20] = {0};

	log_st_cal = log_st / 0x10;
	if (log_st_cal <=9)		add[0] = log_st_cal  + 0x30;	// 0x30:ASCII��16�i�����ϊ�0-9
	else	/*A-F*/			add[0] = log_st_cal  + 0x37;	// 0x37:ASCII��16�i�����ϊ�A-F
	
	log_st_cal = log_st % 0x10;
	if (log_st_cal <=9)		add[1] = log_st_cal  + 0x30;
	else	/*A-F*/			add[1] = log_st_cal  + 0x37;

	strncat(TX_buf[com_mode],add,sizeof(add));					//�Ō���ɓǍ��l�ǉ�

}

/* �o�[�W�����Ǎ� */
void read_change0X_version(short com_mode,long version){

	short	z;
	long	ver_cal;
	short	level;
	char	add[20] = {0};
	char	ver[20] = {0};
	unsigned short	pow_1;
	unsigned short	pow_2;
	
	if 	(version >= 0x0000 && version <= 0x000F)	level = 1;
	else if	(version >= 0x0010 && version <= 0x00FF)	level = 2;
	else if	(version >= 0x0100 && version <= 0x0FFF)	level = 3;
	else if	(version >= 0x1000 && version <= 0xFFFF)	level = 4;
	else 							level = 0;

	pow_1 = 0x0010;
	pow_2 = 0x0001;
	
	for (z = 0; z < level; z++){
		ver_cal =  ( version % pow_1) / pow_2;
		if (ver_cal >= 0 && ver_cal <=9)	add[level - z -1] = ver_cal + 0x30;	// 0x30:ASCII��16�i�����ϊ�0-9
		else	/*A-F*/				add[level - z -1] = ver_cal + 0x37;	// 0x37:ASCII��16�i�����ϊ�A-F
		pow_1 *= 0x0010;
		pow_2 *= 0x0010;
	}
	
	ver[0] = add[0];
	for(z = 1; z < level; z++){
		ver[2*z -1] = 0x2E; // ="."
		ver[2*z] = add[z];
	}
	strncat(TX_buf[com_mode],ver,sizeof(ver));				//�Ō���ɓǍ��l�ǉ�

}

/* �o�[�W�����Ǎ� (RV�R�}���h)*/
void read_change0X_versionRV(short com_mode,long version){

	short	z;
	long	ver_cal;
	short	level;
	char	add[20] = {0};
	char	ver[20] = {0};
	unsigned short	pow_1;
	unsigned short	pow_2;
	
	level = 4;
	pow_1 = 0x0010;
	pow_2 = 0x0001;
	
	for (z = 0; z < level; z++){
		ver_cal =  ( version % pow_1) / pow_2;
		if (ver_cal >= 0 && ver_cal <=9)	add[level - z -1] = ver_cal + 0x30;	// 0x30:ASCII��16�i�����ϊ�0-9
		else	/*A-F*/				add[level - z -1] = ver_cal + 0x37;	// 0x37:ASCII��16�i�����ϊ�A-F
		pow_1 *= 0x0010;
		pow_2 *= 0x0010;
	}
	
	ver[0] = add[0];
	ver[1] = add[1];
	ver[2] = 0x2E; // ="."
	ver[3] = add[2];
	ver[4] = add[3];
	strncat(TX_buf[com_mode],ver,sizeof(ver));				//�Ō���ɓǍ��l�ǉ�

}


/****************************************************
 * Function : JdgIdxErr
 * Summary  : �f�W�^���t�B���^�z�u�C���f�b�N�X�̔���
 * Argument : Idx -> �C���f�b�N�X
 * Return   : JdgFlg -> B_OK : ����
 *                      B_NG : �ُ�
 * Caution  : None
 * Notes    : �V�f�W�^���t�B���^�d�l�̂��߁A
 *            �ȑO�Ɏg�p���Ă���6,7,8,9,11,16,17,18,19�͍폜
 ****************************************************/
short JdgIdxErr(short Idx)
{
	short JdgFlg = B_OK;

	//0~20�ȊO��NG
	if((Idx < 0) || (20 < Idx)){
		JdgFlg = B_NG;
	}
	else{
		//�w��ԍ��͎d�l�ύX�̂��ߍ폜
		switch (Idx)
		{
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 16:
		case 17:
		case 18:
		case 19:
			JdgFlg = B_NG;
			break;
		
		default:
			JdgFlg = B_OK;
			break;
		}
	}

	return JdgFlg;
}

/****************************************************
 * Function : command_Rc
 * Summary  : �f�W�^���t�B���^�W���ǂݏo��
 * Argument : com_mode -> 0 : �z�X�g
 *          :             1 : �����e
 * Return   : void
 * Caution  : None
 * Notes    : �t�H�[�}�b�g�͈ȉ�
 *          : @Rc00x,ii,FCS
 *          :   x -> �`�����l���ԍ�
 *          :   ii -> �f�W�^���t�B���^�C���f�b�N�X
 ****************************************************/
void command_Rc(short com_mode)
{
	short Idx;
	unsigned short Cef;
	//value���擾
	value_judge(com_mode,7,0);
	
	Idx = (short)value[com_mode];
	if(JdgIdxErr(Idx) == B_NG)
	{
		end_code[com_mode] = FORMAT_ERROR;
	}
	else
	{
		switch (Idx)
		{
		case 0:  Cef = FPGA_FILIN0_0; break;	/* �f�W�^���t�B���^�W��(���͑�0) */
		case 1:  Cef = FPGA_FILIN0_1; break;	/* �f�W�^���t�B���^�W��(���͑�0) */
		case 2:  Cef = FPGA_FILIN1_0; break;	/* �f�W�^���t�B���^�W��(���͑�1) */
		case 3:  Cef = FPGA_FILIN1_1; break;	/* �f�W�^���t�B���^�W��(���͑�1) */
		case 4:  Cef = FPGA_FILIN2_0; break;	/* �f�W�^���t�B���^�W��(���͑�2) */
		case 5:  Cef = FPGA_FILIN2_1; break;	/* �f�W�^���t�B���^�W��(���͑�2) */
//		case 6:  Cef = FPGA_FILCOE_A3_0; break;
//		case 7:  Cef = FPGA_FILCOE_A3_1; break;
//		case 8:  Cef = FPGA_FILCOE_A4_0; break;
//		case 9:  Cef = FPGA_FILCOE_A4_1; break;
		// case 10: Cef = FPGA_FIL_EN; break;		/* �f�W�^���t�B���^�L���E�����ݒ� */
//		case 11: Cef = FPGA_FILCOE_B0_1; break;
		case 12: Cef = FPGA_FILOUT1_0; break;	/* �f�W�^���t�B���^�W��(�o�͑�1) */
		case 13: Cef = FPGA_FILOUT1_1; break;	/* �f�W�^���t�B���^�W��(�o�͑�1) */
		case 14: Cef = FPGA_FILOUT2_0; break;	/* �f�W�^���t�B���^�W��(�o�͑�2) */
		case 15: Cef = FPGA_FILOUT2_1; break;	/* �f�W�^���t�B���^�W��(�o�͑�2) */
//		case 16: Cef = FPGA_FILCOE_B3_0; break;
//		case 17: Cef = FPGA_FILCOE_B3_1; break;
		case 20: Cef = FPGA_FIL_EN; break;	/* �f�W�^���t�B���^�L���E�����ݒ� */
		default:
			break;
		}
        read_change(com_mode, Idx, 0, 2);
		read_change(com_mode, Cef, 0, 0);
	}

}

void read_command_R(short com_mode){
	
	switch (RX_buf[com_mode][2]){		//�w�b�_�[�R�[�h����
/* �X�e�[�^�X�Ǐo��(SFC9000�Ή�)(RS) */
	case 'S':
		command_RS(com_mode);
		break;
/* �X�e�[�^�X�Ǐo��(Rs) */
	case 's':
		command_Rs(com_mode);
		break;
/* ���a�Ǐo��(Rg) */
	case 'g':
		command_Rg(com_mode);
		break;
/* �t���X�P�[���Ǐo��(Rr) */
	case 'r':
		command_Rr(com_mode);
		break;
/* �j�t�@�N�^�Ǐo��(Rk) */
	case 'k':
		command_Rk(com_mode);
		break;
/* �_���s���O�Ǐo��(Rd) */
	case 'd':
		command_Rd(com_mode);
		break;
/* ���[�J�b�g�Ǐo��(Rl) */
	case 'l':
		command_Rl(com_mode);
		break;
/* �o�[���A�E�g�Ǐo��(Rb) */
	case 'b':
		command_Rb(com_mode);
		break;
/* ���S�x�Ǐo��(Rv) */
	case 'v':
		command_Rv(com_mode);
		break;
/* �G���[�z�[���h�^�C���Ǐo��(Rh) */
	case 'h':
		command_Rh(com_mode);
		break;
/* ���[�U�[���j�A���C�Y�Ǐo��(Ru) */
	case 'u':
		command_Ru(com_mode);
		break;
/* ���[�J�[���j�A���C�Y�Ǐo��(Rm) */
	case 'm':
		command_Rm(com_mode);
		break;
/* �t������l�Ǐo��(RR) */
	case 'R':
		command_RR(com_mode);
		break;
/* �[���_������ԓǏo��(Rz) */
	case 'z':
		command_Rz(com_mode);
		break;
/* �o�[�W�����l�Ǐo��(SFC9000�Ή�)(RF) */
	case 'F':
		command_RF(com_mode);
		break;
/* �o�[�W�����l�Ǐo��(RV) */
	case 'V':
		command_RV(com_mode);
		break;
/* ���O�f�[�^�Ǐo��(SFC9000�Ή�)(RG) */
	case 'G':	
		command_RG(com_mode);
		break;
/* ���O�f�[�^�Ǐo��(RL) */
	case 'L':	
		command_RL(com_mode);
		break;
/* �[�������f�[�^�Ǐo��(SFC9000�Ή�)(RD) */
	case 'D':
		command_RD(com_mode);
		break;
/* �[�������f�[�^�Ǐo��(RZ) */
	case 'Z':
		command_RZ(com_mode);
		break;
/* �g�`�ُ픻��l�ݒ�Ǐo��(RE) */
	case 'E':
		command_RE(com_mode);
		break;
/* �A�b�e�l�[�^�Q�C���Ǐo��(RA) */
	case 'A':
		command_RA(com_mode);
		break;
/* ��t���j�A���C�Y���[�h�Ǐo��(Ry) */
	case 'y':
		command_Ry(com_mode);
		break;
/* �����Ǐo��(RM) */
	case 'M':
		command_RM(com_mode);
		break;
/* �t�B���^�ݒ�Ǐo��(Rf) */
	case 'f':
		command_Rf(com_mode);
		break;		
/* �ώZ�ڕW�l�Ǐo��(Rt) */
	case 't':
		command_Rt(com_mode);
		break;		
/* �ώZ�I�t�Z�b�g�l�Ǐo��(RT) */
	case 'T':
		command_RT(com_mode);
		break;				
/* ���[�J�[�ݒ�Ǐo��(R1) */
	case '1':
		command_R1(com_mode);
 		break;
/* �f�o�b�O���[�h�ݒ�Ǐo��(R2) */
	case '2':
		command_R2(com_mode);
		break;
/* ����f�[�^�Ǐo��(R3) */
	case '3':
		command_R3(com_mode);
		break;
/* ����g�`�Ǐo��(R4) */
	case '4':
		command_R4(com_mode);
		break;
/* ���[�J�[�ݒ�Ǐo��(SFC9000�Ή�)(R5) */
	case '5':
		command_R5(com_mode);
 		break;
/* �f�o�b�O���[�h�ݒ�Ǐo��(SFC9000�Ή�)(R6) */
	case '6':
		command_R6(com_mode);
		break;
/* ����f�[�^�Ǐo��(SFC9000�Ή�)(R7) */
	case '7':
		command_R7(com_mode);
		break;
/* ����f�[�^�Ǐo��(SFC9000�Ή��ER7�R�}���h�Z�k��)(RC) */
	case 'C':
		command_RC(com_mode);
		break;
/* ����g�`�Ǐo��(SFC9000�Ή�)(R8) */
	case '8':
		command_R8(com_mode);
		break;
/* �p���X�o��(R9) */
	case '9':
		command_R9(com_mode);
		break;
/* �������[�h�f�[�^�Ǐo��(SFC9000�Ή�)(Ri) */
	case 'i':
		command_Ri(com_mode);
		break;
/* SW��ԃ`�F�b�N(RX) */
	case 'X':
		command_RX(com_mode);
		break;	
		/* �g�`��ԓǏo��(SFC9000�Ή�)(RW) */
	case 'W':
		command_RW(com_mode);
		break;
/* �Z���T�p���X�ݒ�Ǐo��(Rp) */
	case 'p':
		command_Rp(com_mode);
		break;
/* �Z���T�V���A���i���o�[�Ǐo��(Rn) */
	case 'n':
		command_Rn(com_mode);
		break;
/* �Z���T���Ǐo���i�]���p�j(Ra) */
	case 'a':
		command_Ra(com_mode);
		break;
/* �f�W�^���t�B���^�W�� */
	case 'c':
		command_Rc(com_mode);
		break;
	}
}

/* ���a������(Wg) */		//���0�`5
void command_Wg (short com_mode){

	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >=0 && value[com_mode] <= 5 ) && size[com_mode] == 1 && digit_check[com_mode] <= 0){
		/*���a�ύX��*/
		if (SVD[ch_no[com_mode] -1].sensor_size != value[com_mode]){
			InitFifoCh(ch_no[com_mode] - 1, value[com_mode]);		/*FIFO CH������*/
			err_thlevel_init(ch_no[com_mode] -1, value[com_mode]);		/*�g�`�ُ픻��臒l������*/
		}
		SVD[ch_no[com_mode] -1].sensor_size = value[com_mode]; /*make_viscos_tbl(ch_no[com_mode] -1);*/
	}
	else if (size[com_mode] != 1 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = SENSORSIZE_ERROR;
	}
	
}

/* �t���X�P�[��������(Wr) */	//1�`20000mL/min
void command_Wr (short com_mode){

	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 1 && value[com_mode] <= 20000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].max_flow = value[com_mode];
		SVD[ch_no[com_mode] -1].unit = digit_check[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = FULLSCALE_ERROR;
	}
		
}

/* �j�t�@�N�^������(Wk) */	//0.000�`9.999
void command_Wk (short com_mode){

	value_judge(com_mode,7,3);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 3){
		SVD[ch_no[com_mode] -1].k_factor = value[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 3){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = KFACTOR_ERROR;
	}
			
}

/* �_���s���O������(Wd) */		//0.0�`25.0sec
void command_Wd (short com_mode){

	value_judge(com_mode,7,1);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 250 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
		SVD[ch_no[com_mode] -1].damp = value[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = DUMPING_ERROR;
	}
}

/* ���[�J�b�g������(Wl) */		//0.0�`25.0%FS
void command_Wl (short com_mode){

	value_judge(com_mode,7,1);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 250 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
		SVD[ch_no[com_mode] -1].low_cut = value[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = LOWCUT_ERROR;
	}
		
}
/* �o�[���A�E�g������(Wb) */
void command_Wb (short com_mode){
	
	/* �o�[���A�E�g��ʔ��� */	//���0�`4
	value_judge(com_mode,7,0);
	if (value[com_mode] >=0 && value[com_mode] <= 4 && size[com_mode] == 1){
		SVD[ch_no[com_mode] -1].burnout = value[com_mode];
	/* �o�[���A�E�g���l���� */	//-300�`300%
		i_num[com_mode]++;
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= -300 && value[com_mode] <= 300 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].burnout_value = value[com_mode];
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			end_code[com_mode] = FORMAT_ERROR;
		}
		else{
			end_code[com_mode] = BURNOUT_VALUE_ERROR;
		}
	}
	else if (size[com_mode] != 1){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = BURNOUT_CLASS_ERROR;
	}
			
}
			
/* ���S�x�W��������(Wv) */		//0.00�`40.00cSt
void command_Wv (short com_mode){

	value_judge(com_mode,7,2);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 4000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
		SVD[ch_no[com_mode] -1].viscos = value[com_mode]; /*make_viscos_tbl(ch_no[com_mode] -1);*/
		/* ��t���j�A���C�Y���[�hON = ��t��ʔ��� */
		if(SVD[ch_no[com_mode] -1].LL_enable == 1){
			LLmode_kind(value[com_mode], ch_no[com_mode] -1);
		}		
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = KVISCOSITY_ERROR;
	}
			
}

/* �G���[�z�[���h�^�C��������(Wh) */		//0�`99sec
void command_Wh (short com_mode){

	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].err_hold_time = value[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = ERRORHOLDTIME_ERROR;
	}

}

/* �t������l������(WR) */
void command_WR (short com_mode){
	/* 臒l�ݒ蔻�� */		//臒l-100�`0%
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= -100 && value[com_mode] <= 0 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].reverse_level = value[com_mode];
	/* ���Ԑݒ蔻�� */		//0.1�`10.0sec
		i_num[com_mode]++;
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].reverse_time = value[com_mode];
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){
			end_code[com_mode] = FORMAT_ERROR;
		}
		else{
			end_code[com_mode] = REVERSE_TIME_ERROR;
		}
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ end_code[com_mode] = FORMAT_ERROR;}
	else{ end_code[com_mode] = REVERSE_VALUE_ERROR;}
}

/* ���[�U�[���j�A���C�Y������(Wu) */
void command_Wu (short com_mode){

	short l;
	short check;
	short linear_point;		// ���j�A���C�Y�p
	long linear_check;		// ���j�A���C�Y�召����p
	unsigned short *pt;		// ���j�A���C�Y�p�|�C���^
	unsigned short point = 0;

	check = 0;
	/* �ݒ�l������ */		//�ݒ�_�� 0-15
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 15 && (size[com_mode] == 1 || size[com_mode] == 2) && digit_check[com_mode] <= 0){
		//point
		point = value[com_mode];
		point = point << 8;
		point &= 0xFF00;
	
		SVD[ch_no[com_mode] -1].uslnr_num &= 0x00FF;	//����������8�r�b�g�𗎂Ƃ�
		SVD[ch_no[com_mode] -1].uslnr_num |= point;	//���8bit�ɐݒ�

		linear_point = value[com_mode];
		linear_check = -1;		//output1 = 0 ���e�p
	/* �o�́E���͐ݒ�l ���� */
	/* �o�́~15 */
		pt = &SVD[ch_no[com_mode] -1].uslnr_out1.WORD.low;
		for (l=1; l<=15; l++){		//for-loop
		/* ���͒l */
			i_num[com_mode]++;		// ','��1���ֈړ�
			value_judge(com_mode,i_num[com_mode],1);
			if (digit_check[com_mode] > 1 || size[com_mode] < 1 || size[com_mode] > 7){
				end_code[com_mode] = FORMAT_ERROR;	//End Code 14  �����ُ�E������(�t�H�[�}�b�g�G���[)
				break;
			}			//�͈͊O for-loop �E�o
			if (value[com_mode] < 0 || value[com_mode] > 200000){
				end_code[com_mode] = USERLINEAR_VALUE_ERROR;			//End Code 44  ���ʐݒ�l ���l�͈͊O����
				break;
			}			//�͈͊O for-loop �E�o
			if (l <= linear_point && value[com_mode] <= linear_check){
				end_code[com_mode] = USERLINEAR_ORDER_ERROR;		//End Code 45  ���ʐݒ�l ���ԕs�K��
				break;
			}			//�͈͊O for-loop �E�o
			linear_check = value[com_mode];
			*pt = linear_check % 0x10000;
			pt++;
			linear_check = value[com_mode];
			*pt = linear_check / 0x10000;
			linear_check = value[com_mode];
			pt++;
			check++;
		}
			/* ���́~15 */
				if (check == 15){
					linear_check = -1;	//input1 = 0 ���e�p
					pt = &SVD[ch_no[com_mode] -1].uslnr_in1.WORD.low;
					for (l=1; l<=15; l++){	//for-loop
					/* ���͒l */
						i_num[com_mode]++;	// ','��1���ֈړ�
						value_judge(com_mode,i_num[com_mode],1);
						if (digit_check[com_mode] > 1 || size[com_mode] < 1 || size[com_mode] > 7){ 
							end_code[com_mode] = FORMAT_ERROR;	//End Code 14  �����ُ�E������(�t�H�[�}�b�g�G���[)
							break;
						}		//�͈͊O for-loop �E�o
						if (value[com_mode] < 0 || value[com_mode] > 200000){ 
							end_code[com_mode] = USERLINEAR_VALUE_ERROR;			//End Code 44  ���ʐݒ�l ���l�͈͊O����
							break;
						}		//�͈͊O for-loop �E�o
						if (l <= linear_point && value[com_mode] <= linear_check){ 
							end_code[com_mode] = USERLINEAR_ORDER_ERROR;		//End Code 45  ���ʐݒ�l ���ԕs�K��
							break;
						}		//�͈͊O for-loop �E�o
						linear_check = value[com_mode];
						*pt = linear_check % 0x10000;
						pt++;
						linear_check = value[com_mode];
						*pt = linear_check / 0x10000;
						linear_check = value[com_mode];
						pt++;
					}						//End Code 00  ����I��15�_�ȓ��E0-20000.0�E���l���͏�����j
				}
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ end_code[com_mode] = FORMAT_ERROR;}
			else{ end_code[com_mode] = USERLINEAR_POINT_ERROR;} 						//End Code 43 = �ݒ�_���ُ�
}

/* ���[�U�[���j�A���C�Y�ؑւ�(WU) */
void command_WU (short com_mode){

	short l;
	long value_linear;
	short check;
	unsigned short *pt;		// ���j�A���C�Y�p�|�C���^
	unsigned short point = 0;
	unsigned short value_high;
	unsigned short value_low;

	check = 0;	
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
		//ON
		if(value[com_mode] == 1){
			/* �o�́~5 ��m�F*/
			pt = &SVD[ch_no[com_mode] -1].uslnr_out1.WORD.low;
			for (l = 0; l < 5; l++){
				value_low = *pt;
				pt++;
				value_high = *pt;
				value_linear = value_low + value_high * 0x10000;
				pt++;
				if(value_linear == 0) check++;
			}
			/* ���́~5 ��m�F*/
			pt = &SVD[ch_no[com_mode] -1].uslnr_in1.WORD.low;
			for (l = 0; l < 5; l++){
				value_low = *pt;
				pt++;
				value_high = *pt;
				value_linear = value_low + value_high * 0x10000;
				pt++;
				if(value_linear == 0) check++;
			}			
			
			if(check == 0){
				point = 5;
				point = point << 8;
				point &= 0xFF00;
				
				SVD[ch_no[com_mode] -1].uslnr_num &= 0x00FF;	//����������8�r�b�g�𗎂Ƃ�
				SVD[ch_no[com_mode] -1].uslnr_num |= point;	//���8bit�ɐݒ�				
			}else{
				end_code[com_mode] = USERLINEAR_5POINT_EMPTY;
			}
		}
		//OFF
		else{
			SVD[ch_no[com_mode] -1].uslnr_num &= 0x00FF;	//����������8�r�b�g�𗎂Ƃ�
			SVD[ch_no[com_mode] -1].uslnr_num |= 0;	//���8bit�ɐݒ�
		}
	}
	else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = USERLINEAR_5POINT_ERROR;
	}
}

/* ���[�J�[���j�A���C�Y������(Wm) */
void command_Wm (short com_mode){

	short l;
	short check;
	short linear_point;		// ���j�A���C�Y�p
	long linear_check;		// ���j�A���C�Y�召����p
	unsigned short *pt;		// ���j�A���C�Y�p�|�C���^
	unsigned short point = 0;

	check = 0;
	/* �ݒ�l������ */		//�ݒ�_�� 0-15
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 15 && (size[com_mode] == 1 || size[com_mode] == 2) && digit_check[com_mode] <= 0){
		//point
		point = value[com_mode];
		point = point << 8;
		point &= 0xFF00;
	
		SVD[ch_no[com_mode] -1].mklnr_num &= 0x00FF;	//����������8�r�b�g�𗎂Ƃ�
		SVD[ch_no[com_mode] -1].mklnr_num |= point;	//���8bit�ɐݒ�

		linear_point = value[com_mode];
		linear_check = -1;		//output1 = 0 ���e�p
	/* �o�́E���͐ݒ�l ���� */
	/* �o�́~15 */
		pt = &SVD[ch_no[com_mode] -1].mklnr_out1.WORD.low;
		for (l=1; l<=15; l++){		//for-loop
		/* ���͒l */
			i_num[com_mode]++;		// ','��1���ֈړ�
			value_judge(com_mode,i_num[com_mode],3);
			if (digit_check[com_mode] > 3 || size[com_mode] < 1 || size[com_mode] > 5){
				end_code[com_mode] = FORMAT_ERROR;	//End Code 14  �����ُ�E������(�t�H�[�}�b�g�G���[)
				break;
			}			//�͈͊O for-loop �E�o
			if (value[com_mode] < 0 || value[com_mode] > 9999){
				end_code[com_mode] = MAKERLINEAR_VALUE_ERROR;			//End Code 56  ���ʐݒ�l ���l�͈͊O����
				break;
			}			//�͈͊O for-loop �E�o
			if (l <= linear_point && value[com_mode] <= linear_check){
				end_code[com_mode] = MAKERLINEAR_ORDER_ERROR;		//End Code 57  ���ʐݒ�l ���ԕs�K��
				break;
			}			//�͈͊O for-loop �E�o
			linear_check = value[com_mode];
			*pt = linear_check % 0x10000;
			pt++;
			linear_check = value[com_mode];
			*pt = linear_check / 0x10000;
			linear_check = value[com_mode];
			pt++;
			check++;
		}
			/* ���́~15 */
				if (check == 15){
					linear_check = -1;	//input1 = 0 ���e�p
					pt = &SVD[ch_no[com_mode] -1].mklnr_in1.WORD.low;
					for (l=1; l<=15; l++){	//for-loop
					/* ���͒l */
						i_num[com_mode]++;	// ','��1���ֈړ�
						value_judge(com_mode,i_num[com_mode],3);
						if (digit_check[com_mode] > 3 || size[com_mode] < 1 || size[com_mode] > 5){ 
							end_code[com_mode] = FORMAT_ERROR;	//End Code 14  �����ُ�E������(�t�H�[�}�b�g�G���[)
							break;
						}		//�͈͊O for-loop �E�o
						if (value[com_mode] < 0 || value[com_mode] > 9999){ 
							end_code[com_mode] = MAKERLINEAR_VALUE_ERROR;			//End Code 56  ���ʐݒ�l ���l�͈͊O����
							break;
						}		//�͈͊O for-loop �E�o
						if (l <= linear_point && value[com_mode] <= linear_check){ 
							end_code[com_mode] = MAKERLINEAR_ORDER_ERROR;		//End Code 57  ���ʐݒ�l ���ԕs�K��
							break;
						}		//�͈͊O for-loop �E�o
						linear_check = value[com_mode];
						*pt = linear_check % 0x10000;
						pt++;
						linear_check = value[com_mode];
						*pt = linear_check / 0x10000;
						linear_check = value[com_mode];
						pt++;
					}						//End Code 00  ����I��15�_�ȓ��E0.000-9.999�E���l���͏�����j
				}
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ end_code[com_mode] = FORMAT_ERROR;}
			else{ end_code[com_mode] = MAKERLINEAR_POINT_ERROR;} 						//End Code 55 = �ݒ�_���ُ�
}
			
/* �[�������f�[�^������(SFC9000�Ή�)(Wz) */
void command_Wz (short com_mode){

	short m;
	short ch_z;			// �`���l�����p
	short ch_number;			// �`���l�����Ԕ���p
	short check;
	short check_1;
	short check_14;
	unsigned short *pt;
	long value_check;
	char ch[1] = {0};			// �`���l��������p
	
	check = 0;
	check_1 = 0;
	check_14 = 0;

/* �V���O���R�}���h */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		ch_z = 1;
	/* ���S�x�W�� */
		value_judge(com_mode,7,2);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 4000) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].viscos = value[com_mode];
			/* ��t���j�A���C�Y���[�hON = ��t��ʔ��� */
			if(SVD[ch_no[com_mode] -1].LL_enable == 1){
				LLmode_kind(value[com_mode], ch_no[com_mode] -1);
			}			
			check_1++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
			check_14++;
		}
		i_num[com_mode]++;
	/* ��g�g�`�ő�l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_max = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* ��g�g�`�ŏ��l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_min = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* �`�����ԍ� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= -99999999 && value[com_mode] <= 99999999) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD = value[com_mode] / 32;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* FIFO��g�g�`���o�ʒu */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 3800) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_fifo_pos = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* Gain1 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_gain_1st = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* Gain2 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_gain_2nd = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* ��g�̍�(P1-P2) */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 99) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_p1p2 = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	}		
/* �}���`�R�}���h */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];			//CH������
		ch_z = ch[0] - 0x30;
		ch[0] = 0;
		/* �e�����f�[�^ */
		i_num[com_mode] = 9 + 2 * ch_z;			//���l���ֈړ��F�w�萔�ƌ��s��v�̓t�H�[�}�b�g�G���[���蕔��
		for (m=0; m<ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];	//CH�ԍ�����
			ch_number = ch[0] - 0x30;
		/* ���S�x�W�� */
			value_judge(com_mode,i_num[com_mode],2);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 4000) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
				SVD[ch_number -1].viscos = value[com_mode];
				/* ��t���j�A���C�Y���[�hON = ��t��ʔ��� */
				if(SVD[ch_number -1].LL_enable == 1){
					LLmode_kind(value[com_mode], ch_number -1);
				}				
				check_1++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
				check_14++;
			}
			i_num[com_mode]++;
		/* ��g�g�`�ő�l */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_wave_max = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* ��g�g�`�ŏ��l */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_wave_min = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* �`�����ԍ� */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= -99999999 && value[com_mode] <= 99999999) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_delta_ts.DWORD = value[com_mode] / 32;
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* FIFO��g�g�`���o�ʒu */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 3800) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_fifo_pos = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* Gain1 */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_gain_1st = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* Gain2 */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_gain_2nd = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* ��g�̍�(P1-P2) */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 99) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_p1p2 = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		}
	}
	/* ���۔��� */
	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
	else if (check_1 != ch_z){
		end_code[com_mode] = KVISCOSITY_ERROR;				//End Code 41 = �G���[�i���S�x�W���j
	}
	else if (check != 7 * ch_z){
		end_code[com_mode] = ZADJ_WRITE_ERROR;				//End Code 54 = �G���[�i�[�������E�ݒ�͈͊O�j
	}
}

/* �[�������f�[�^������(WZ) */
 void command_WZ (short com_mode){

	 short m;
	 short ch_z;			// �`���l�����p
	 short ch_number;			// �`���l�����Ԕ���p
	 short check;
	 short check_1;
	 short check_14;
	 unsigned short *pt;
	 long value_check;
	 char ch[1] = {0};			// �`���l��������p
	
	 check = 0;
	 check_1 = 0;
	 check_14 = 0;

 /* �}���`�R�}���h */
	 ch[0] = RX_buf[com_mode][7];			//CH������
	 ch_z = ch[0] - 0x30;
	 ch[0] = 0;
	 /* �e�����f�[�^ */
	 i_num[com_mode] = 9 + 2 * ch_z;			//���l���ֈړ��F�w�萔�ƌ��s��v�̓t�H�[�}�b�g�G���[���蕔��
	 for (m=0; m<ch_z; m++){
		 ch[0] = RX_buf[com_mode][9 + 2*m];	//CH�ԍ�����
		 ch_number = ch[0] - 0x30;
	 /* ���S�x�W�� */
		 value_judge(com_mode,i_num[com_mode],2);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 4000) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
			 SVD[ch_number -1].viscos = value[com_mode];
				/* ��t���j�A���C�Y���[�hON = ��t��ʔ��� */
				if(SVD[ch_number -1].LL_enable == 1){
					LLmode_kind(value[com_mode], ch_number -1);
				}
			 check_1++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Tu�i�Y���f�[�^�Ȃ��j */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 150000000) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			 check++;		
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Td�i�Y���f�[�^�Ȃ��j */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 150000000) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Dt */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= -99999999 && value[com_mode] <= 99999999) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			 SVD[ch_number -1].zero_delta_ts.DWORD = value[com_mode];
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Window Position�i�Y���f�[�^�Ȃ��j */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 5999) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Gain1 */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			 SVD[ch_number -1].zero_gain_1st = value[com_mode];
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Gain2 */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			 SVD[ch_number -1].zero_gain_2nd = value[com_mode];
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Threshold�i�Y���f�[�^�Ȃ��j */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 }
	 /* ���۔��� */
	 if (check_14 > 0){
		 end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	 }
	 else if (check_1 != ch_z){
		 end_code[com_mode] = KVISCOSITY_ERROR;				//End Code 41 = �G���[�i���S�x�W���j
	 }
	 else if (check != 7 * ch_z){
		 end_code[com_mode] = ZADJ_WRITE_ERROR;				//End Code 54 = �G���[�i�[�������E�ݒ�͈͊O�j
	 }
 }

/* �p�����[�^�Z�b�g������(WP) */
void command_WP (short com_mode){

	short m;
	short ch_z;			// �`���l�����p
	short ch_number;			// �`���l�����Ԕ���p
	short check_14;
	short check_1;
	short check_2;
	short check_3;	
	char ch[1] = {0};			// �`���l��������p

	check_1 = 0;
	check_2 = 0;
	check_3 = 0;
	check_14 = 0;
	
	ch[0] = RX_buf[com_mode][7];			//CH������
	ch_z = ch[0] - 0x30;
	ch[0] = 0;
	/* �e�����f�[�^ */
	i_num[com_mode] = 9 + 2 * ch_z;			//���l���ֈړ��F�w�萔�ƌ��s��v�̓t�H�[�}�b�g�G���[���蕔��
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];	//CH�ԍ�����
		ch_number = ch[0] - 0x30;
	/* �t���X�P�[�� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
			SVD[ch_number -1].max_flow = value[com_mode];
			check_1++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* �_���s���O */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 250 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
			SVD[ch_number -1].damp = value[com_mode];
			check_2++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* ���[�J�b�g */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 250 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){ 
			SVD[ch_number -1].low_cut = value[com_mode];
			check_3++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){
			check_14++;
		}
		i_num[com_mode]++;
	}
	/* ���۔��� */
	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
	else if (check_1 != ch_z){
		end_code[com_mode] = FULLSCALE_ERROR;	//End Code 35 = �G���[�i�t���X�P�[���j
	}
	else if (check_2 != ch_z){
		end_code[com_mode] = DUMPING_ERROR;	//End Code 37 = �G���[�i�_���s���O�j
	}
	else if (check_3 != ch_z){
		end_code[com_mode] = LOWCUT_ERROR;	//End Code 38 = �G���[�i���[�J�b�g�j
	}
			
}
/* �g�`�ُ픻��l�ݒ菑����(WE) */
void command_WE (short com_mode){

	short check;
	short check_14;
	short	empty;
	
	check = 0;
	check_14 = 0;
	/* �G���v�e�B�Z���T����臒l */
		value_judge(com_mode,7,0);
		empty = value[com_mode];
		if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].wave_vth = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ���Z�ُ픻��臒l */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 10 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].correlate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* ���Z�ُ픻��� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].correlate_time = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �g�`�A���o�����X臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].balance_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* AGC�s�\����臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].saturation_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �g�`��������臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].attenuate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �G���v�e�B�Z���T�x��臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0 && value[com_mode] < empty){
			SVD[ch_no[com_mode] -1].alm_wave_vth = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �A���v�Q�C���x��臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].alm_gain_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �A���v�Q�C���x������ */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].alm_gain_count = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �x�����莞�� */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].alm_hold_time = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;

	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
	else if (check != 10){
		end_code[com_mode] = JUDGEMENT_WAVE_ERROR;				//End Code 50 = �͈͊O�w��E�����ُ�
	}
}

/* �A�b�e�l�[�^�Q�C��������(WA) */		//0�`255
void command_WA (short com_mode){

	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
//			SVD[ch_no[com_mode] -1].atn_gain = value[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = ATTENUATOR_GAIN_ERROR;
	}
}

/* ��t���j�A���C�Y���[�h������(Wy) */		//0�`1
 void command_Wy (short com_mode){
 
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].LL_enable = value[com_mode];
		
		/* ��t���j�A���C�Y���[�hON = ��t��ʔ��� */
		if(SVD[ch_no[com_mode] -1].LL_enable == 1){
			LLmode_kind(SVD[ch_no[com_mode] -1].viscos, ch_no[com_mode] -1);
		}
	}
	else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = LLMODE_ERROR;
	}
	
	RX_buf[com_mode][9]  = SVD[ch_no[com_mode] -1].LL_kind / 10 + 0x30;
	RX_buf[com_mode][10] = SVD[ch_no[com_mode] -1].LL_kind % 10 + 0x30;
	RX_buf[com_mode][11] = 0x2C;		//","
}

/* �t�B���^�ݒ菑����(Wf) */
void command_Wf (short com_mode){
	
	/* �t�B���^���[�h���� */	//���0�`2
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 2 && size[com_mode] == 1){
		SVD[ch_no[com_mode] -1].filter_mode = value[com_mode];
	/* �ړ����ϐ��l���� */	//1�`50
		i_num[com_mode]++;
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 50 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].filter_avg = value[com_mode];
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
			end_code[com_mode] = FORMAT_ERROR;
		}
		else{
			end_code[com_mode] = FILTER_AVG_ERROR;
		}
	}
	else if (size[com_mode] != 1){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = FILTER_MODE_ERROR;
	}
}

 /* �ώZ�ڕW�l������(Wt) */		//0.0�`99999.99mL
 void command_Wt (short com_mode){

	value_judge(com_mode,7,2);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 9999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 8 ) && digit_check[com_mode] <= 2){
		SVD[ch_no[com_mode] -1].target_total.INT32 = value[com_mode];
	}
	else if ( size[com_mode] < 1 || size[com_mode] > 8 || digit_check[com_mode] > 3){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = TOTALTARGET_ERROR;
	}
}

 /* �ώZ�I�t�Z�b�g�l������(WT) */
void command_WT (short com_mode){
	
	/* �L���������� */	//ON/OFF 0�`1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		SVD[ch_no[com_mode] -1].total_offset_enable = value[com_mode];
	/* �o�[���A�E�g���l���� */	//-9.999�`9.999mL
		i_num[com_mode]++;
		value_judge(com_mode,i_num[com_mode],3);
		if ( ( value[com_mode] >= -9999 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 3){
			SVD[ch_no[com_mode] -1].total_offset_value = value[com_mode];
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 3){
			end_code[com_mode] = FORMAT_ERROR;
		}
		else{
			end_code[com_mode] = TOTALOFFSET_ERROR;
		}
	}
	else if (size[com_mode] != 1){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = TOTALOFFSET_ERROR;
	}
}

/* ���[�J�[�ݒ菑����(W1) */
 void command_W1 (short com_mode){

	 short z;
	 short l;
	 short h;
	 short serial_size;
	 short check;
	 short check_14;
	 short serial;
	 unsigned short *pt;
	 char ver[MES_CHAR_MAX] = {0};			// �o�[�W�����Ǎ��p
	
	 check = 0;
	 check_14 = 0;
	 serial_size = 0;
	 
	 /* �␳���͒l���� */
	 i_num[com_mode] = 7;
	 for(l=0; l<5; l++){
		 value_judge(com_mode,i_num[com_mode],2);
		 if ( (value[com_mode] >= -9999 && value[com_mode] <= 9999) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		 i_num[com_mode]++;
	 }
	 if (check == 5){
	 /* �V���A���i���o�[ */
		 z = 0;
		 while (RX_buf[com_mode][i_num[com_mode]] != ','){		//�f�[�^��؂�ʒu����
			if(i_num[com_mode] >= (MSG_MAX-1)){		//','���Ō�܂Ō�����Ȃ��ꍇ
				check_14++;
				break;
			}
			ver[z] = RX_buf[com_mode][i_num[com_mode]];
			 serial_size++;
			 i_num[com_mode]++;
			 z++;	
				if(z >= MES_CHAR_MAX){	//�z��O�����ݖh�~
					break;
				}
		 }
		 pt = &SVD[ch_no[com_mode] -1].c_serial[0];
		 if (serial_size % 2 == 0){					//������������
			 for (z = 0; z < serial_size; z = z+2){
				 serial = ver[z] * 0x0100 + ver[z+1];
				 *pt = serial;
				 pt++;
			 }
			 for (z = serial_size; z < 16; z = z+2){		//16�������� �󔒑��
				 serial = 0x2020;
				 *pt = serial;
				 pt++;
			 }
		 }
		 else{							//���������
			 for (z = 0; z < serial_size -1; z = z+2){
				 serial = ver[z] * 0x0100 + ver[z+1];
				 *pt = serial;
				 pt++;
			 }
			 serial = ver[z] * 0x0100 + 0x20;
			 *pt = serial;
			 pt++;
			 for (z = serial_size +1; z < 16; z = z+2){		//16�������� �󔒑��
				 serial = 0x2020;
				 *pt = serial;
				 pt++;
			 }
		 }

		 for (h = 0; h < MES_CHAR_MAX; h++){ver[h] = 0;}
	 }
	 if (check_14 > 0 || serial_size < 0 || serial_size > 16){
		 end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	 }
	 else if (check != 5){
		 end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = �͈͊O�w��E�����ُ�
	 }
 }

/* �f�o�b�O���[�h�ݒ菑����(W2) */
 void command_W2 (short com_mode){

	 /* ���ʏo�̓e�X�g���ڔ��� */	//���0�`1
	 value_judge(com_mode,7,0);
	 if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		 MES[ch_no[com_mode] -1].test_enable = value[com_mode];		//���ʏo�̓e�X�g
		 i_num[com_mode]++;
	 /* �o�̓e�X�g�l���� */			//0.00�`300.00%
		 value_judge(com_mode,i_num[com_mode],2);
		 if ( ( value[com_mode] >= 0 && value[com_mode] <= 30000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			 MES[ch_no[com_mode] -1].test_flow = value[com_mode];	//���ʏo�̓e�X�g�l
			 i_num[com_mode]++;
	 /* �o�̓|�[�g�e�X�g�l���� */	//���0�`1
			 value_judge(com_mode,i_num[com_mode],0);
			 if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
				 MES[ch_no[com_mode] -1].test_port_out = value[com_mode];		//�o�̓|�[�g�e�X�g
			 }else if (size[com_mode] != 1){
				 end_code[com_mode] = FORMAT_ERROR;
			 }else{
				 end_code[com_mode] = DEBUGMODE_ERROR;
			 }
		 }else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){
			 end_code[com_mode] = FORMAT_ERROR;
		 }else{
			 end_code[com_mode] = DEBUGMODE_ERROR;
		 }
	 }else if (size[com_mode] != 1){
		 end_code[com_mode] = FORMAT_ERROR;
	 }else{
		 end_code[com_mode] = DEBUGMODE_ERROR;
	 }

	 if(MES[ch_no[com_mode] -1].test_enable != 0){	
		 action_status_control(ch_no[com_mode]-1,ACT_STS_TEST);
	 }else{
		 action_status_control(ch_no[com_mode]-1,ACT_STS_NORMAL);
	 }
 }

/* ���[�J�[�ݒ菑����(SFC9000�Ή�)(W5) */
void command_W5 (short com_mode){

	short z;
	short l;
	short h;
	short serial_size;
	short check;
	short check_14;
	short serial;
	unsigned short *pt;
	char ver[MES_CHAR_MAX] = {0};			// �o�[�W�����Ǎ��p
	
	check = 0;
	check_14 = 0;
	serial_size = 0;
	/* �G���v�e�B�Z���T����臒l */
		value_judge(com_mode,7,0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].wave_vth = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ���Z�ُ픻��臒l */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 10 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].correlate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* ���Z�ُ픻��� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].correlate_time = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �g�`�A���o�����X臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].balance_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* AGC�s�\����臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].saturation_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �g�`��������臒l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].attenuate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �Q�C���X�e�b�v�� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 10 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].gain_step = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �����Œ�or���� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].sound_vel_sel = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �����ݒ� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 400 && value[com_mode] <= 3000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].sound_vel_fix = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �����t�B���^���萔 */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].sound_vel_filter = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* ���S�x�Œ�or���� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].viscos_auto = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �ō��݃p���X */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 8 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].drive_pls = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �A�b�e�l�[�^�Q�C���l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			//SVD[ch_no[com_mode] -1].atn_gain = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* FIFO CH �����l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 47 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].fifo_ch_init = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* CUnet �đ��M�ҋ@���� */	
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[0].cunet_delay = value[com_mode];	//���SCH����
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;		
	/* ���֒l�T�[�`����or�L�� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].search_sw = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �C�A�΍􃂁[�h */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 4 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].damp_mode = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ���֒l������l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].corr_up.DWORD = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ���֒l�������l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].corr_low.DWORD = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �ُ�z�[���h���� */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].inc = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* ���C�g���~�b�g�z�[���h�N���A */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].hldt = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* ���C�g���~�b�g */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].rlt = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* ���C�g���~�b�g1st */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].odpd = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* �^�[�Q�b�g1st */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].rl1tg = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* �A�x���[�W1st */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].rl1av = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ���C�g���~�b�g2nd */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].odpl = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* �^�[�Q�b�g2nd */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].rl2tg = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* �A�x���[�W2nd */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].rl2av = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �_���s���O�{�� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].rl2d = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �z�[���h�N���A */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].rl2hc = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* �σ_���s���O */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].dump_var = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �σ_���s���O�{�� */
		value_judge(com_mode,i_num[com_mode],2);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 10000) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].dump_mul = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;

	if (check == 32){
	/* �V���A���i���o�[ */
		z = 0;
		while (RX_buf[com_mode][i_num[com_mode]] != ','){		//�f�[�^��؂�ʒu����
			if(i_num[com_mode] >= (MSG_MAX-1)){		//','���Ō�܂Ō�����Ȃ��ꍇ
				check_14++;
				break;
			}
			ver[z] = RX_buf[com_mode][i_num[com_mode]];
			serial_size++;
			i_num[com_mode]++;
			z++;	
			if(z >= MES_CHAR_MAX){	//�z��O�����ݖh�~
				break;
			}
		}
		for(l = 0; l < 6; l++){
			pt = &SVD[l].c_serial[0];
			if (serial_size % 2 == 0){					//������������
				for (z = 0; z < serial_size; z = z+2){
					serial = ver[z] * 0x0100 + ver[z+1];
					*pt = serial;
					pt++;
				}
				for (z = serial_size; z < 16; z = z+2){		//16�������� �󔒑��
					serial = 0x2020;
					*pt = serial;
					pt++;
				}
			}
			else{							//���������
				for (z = 0; z < serial_size -1; z = z+2){
					serial = ver[z] * 0x0100 + ver[z+1];
					*pt = serial;
					pt++;
				}
				serial = ver[z] * 0x0100 + 0x20;
				*pt = serial;
				pt++;
				for (z = serial_size +1; z < 16; z = z+2){		//16�������� �󔒑��
					serial = 0x2020;
					*pt = serial;
					pt++;
				}
			}
		}
		for (h = 0; h < MES_CHAR_MAX; h++){ver[h] = 0;}
	}
	if (check_14 > 0 || serial_size < 0 || serial_size > 16){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
	else if (check != 32){
		end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = �͈͊O�w��E�����ُ�
	}
}
			
/* �f�o�b�O���[�h�ݒ菑����(SFC9000�Ή�)(W6) */
void command_W6 (short com_mode){

	/* ���ʏo�̓e�X�gON/OFF */	//���0�`1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		MES[ch_no[com_mode] -1].test_enable = value[com_mode];
		i_num[com_mode]++;
	/* �o�̓e�X�g�l���� */			//0.00�`300.00%
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 30000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			MES[ch_no[com_mode] -1].test_flow = value[com_mode];// * 100;	//���ʏo�̓e�X�g�l
			i_num[com_mode]++;
	/* �G���[�e�X�gON/OFF */	//���0�`1
			value_judge(com_mode,i_num[com_mode],0);
			if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
				MES[ch_no[com_mode] -1].test_err_enable = value[com_mode];
				i_num[com_mode]++;
	/* �����G���[��� */
				value_judge(com_mode,i_num[com_mode],0);
				if( (value[com_mode] >= 0 && value[com_mode] <= 99) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) ){
					MES[ch_no[com_mode] -1].test_err_code = value[com_mode];
				}else if ( size[com_mode] < 1 || size[com_mode] > 2){
					end_code[com_mode] = FORMAT_ERROR;
				}else{
					end_code[com_mode] = DEBUGMODE_ERROR;
				}
			}else if (size[com_mode] != 1){
				end_code[com_mode] = FORMAT_ERROR;
			}else{
				end_code[com_mode] = DEBUGMODE_ERROR;
			}
		}else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){
			end_code[com_mode] = FORMAT_ERROR;
		}else{
			end_code[com_mode] = DEBUGMODE_ERROR;
		}
	}else if (size[com_mode] != 1){
		end_code[com_mode] = FORMAT_ERROR;
	}else{
		end_code[com_mode] = DEBUGMODE_ERROR;
	}

	if(MES[ch_no[com_mode] -1].test_enable != 0){	
		action_status_control(ch_no[com_mode]-1,ACT_STS_TEST);
	}else{
		action_status_control(ch_no[com_mode]-1,ACT_STS_NORMAL);
	}
}

/* �[�������ڍ׃f�[�^������(SFC9000�Ή�)(W7) */
void command_W3 (short com_mode){

	short z;
	short l;
	short h;
	short check;
	short check_14;
	short ret;
	unsigned short status;
	char sta[MES_W7_STATUS] = {0};			// �X�e�[�^�X�Ǎ��p
	short pch = ch_no[com_mode] -1;
	short DgtNum = 0;
	
	check = 0;
	check_14 = 0;
	ret = 0;
	
	// //01. ����[mL/min]
	
	// DgtNum = 2;
	// value_judge(com_mode, 7, DgtNum);
	// check_14 += ChkFmtErr(com_mode, 1, 9, DgtNum);
	// if(check_14 == 0)
	// {
	// 	check += ChkRng(com_mode, -3000000, 3000000);
	// 	if(check == 0)
	// 	{
	// 		SVD[pch].zero_flow_qat.DWORD = value[com_mode];
	// 	}
	// }
	// i_num[com_mode]++;
	
	// //02. ����[m/s]
	// DgtNum = 3;
	// value_judge(com_mode, i_num[com_mode], DgtNum);
	// check_14 += ChkFmtErr(com_mode, 1, 7, DgtNum);
	// if(check_14 == 0)
	// {
	// 	check += ChkRng(com_mode, -30000, 30000);
	// 	if(check == 0)
	// 	{
	// 		SVD[pch].zero_flow_vel.DWORD = value[com_mode] * 10;
	// 	}
	// }
	// i_num[com_mode]++;
	
	// //03. ����[m/s]
	// DgtNum = 0;
	// value_judge(com_mode, i_num[com_mode], DgtNum);
	// check_14 += ChkFmtErr(com_mode, 1, 4, DgtNum);
	// if(check_14 == 0)
	// {
	// 	check += ChkRng(com_mode, 0, 3000);
	// 	if(check == 0)
	// 	{
	// 		SVD[pch].zero_flow_vel.DWORD = value[com_mode] * 10;
	// 	}
	// }
	// i_num[com_mode]++;
	
	// //04. �ώZ�l[mL/min]
	// DgtNum = 3;
	// value_judge(com_mode, i_num[com_mode], DgtNum);
	// check_14 += ChkFmtErr(com_mode, 1, 12, DgtNum);
	// if(check_14 == 0)
	// {
	// 	check += ChkRng(com_mode, 0, 99999999999);
	// 	if(check == 0)
	// 	{
	// 		SVD[pch].zero_flow_vel.DWORD = value[com_mode] * 10000;
	// 	}
	// }
	// i_num[com_mode]++;
	
	// /* ��g�g�`�ő�l */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_wave_max = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* ��g�g�`�ŏ��l */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_wave_min = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* �`�����ԍ�[ps] */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= -99999999 && value[com_mode] <= 99999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD = value[com_mode] / 32;
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* ���֒l�� */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_correlate = value[com_mode] / 1000;
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* �[���_�I�t�Z�b�g */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( ( value[com_mode] >= -100000 && value[com_mode] <= 100000 ) || value[com_mode] == -128000) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_zero_offset = (value[com_mode] / 32) + 4000;
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* status */
	// 	z = 0;
	// 	status = 0;
	// 	while (RX_buf[com_mode][i_num[com_mode]] != ','){
	// 		sta[z] = RX_buf[com_mode][i_num[com_mode]];
	// 		i_num[com_mode]++;
	// 		z++;
	// 		if(z >= MES_W7_STATUS){	//�z��O�����ݖh�~
	// 			break;
	// 		}
	// 	}
	// 	if (sta[3] >= 0x30 && sta[3] <= 0x39)		status = sta[3] - 0x30;
	// 	else if (sta[3] >= 0x41 && sta[3] <= 0x46)	status = sta[3] - 0x37;
		
	// 	if (sta[2] >= 0x30 && sta[2] <= 0x39)		status += (sta[2] - 0x30) * 0x10;
	// 	else if (sta[2] >= 0x41 && sta[2] <= 0x46)	status += (sta[2] - 0x37) * 0x10;
		
	// 	if (sta[1] >= 0x30 && sta[1] <= 0x39)		status += (sta[1] - 0x30) * 0x100;
	// 	else if (sta[1] >= 0x41 && sta[1] <= 0x46)	status += (sta[1] - 0x37) * 0x100;

	// 	if (sta[0] >= 0x30 && sta[0] <= 0x39)		status += (sta[0] - 0x30) * 0x1000;
	// 	else if (sta[0] >= 0x41 && sta[0] <= 0x46)	status += (sta[0] - 0x37) * 0x1000;		
		
	// 	SVD[ch_no[com_mode] -1].zero_condition = status;
		
	// 	for (h = 0; h < MES_W7_STATUS; h++){sta[h] = 0;}
	// 	i_num[com_mode]++;
	// /* FIFO��g�g�`���o�ʒu */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 3800 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_fifo_pos = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* Gain 1st stage */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_gain_1st = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* Gain 2nd stage */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_gain_2nd = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* FIFO CH */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 63 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_fifo_ch = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* ��g�̍�(P1-P2) */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_p1p2 = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* �������֒l */
	// 	for (l = 0; l < 40; l++){
	// 		value_judge(com_mode,i_num[com_mode],0);
	// 		if ( ( value[com_mode] >= 0 && value[com_mode] <= 65535 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
	// 			// SVD[ch_no[com_mode] -1].zero_sum_abs[l] = value[com_mode];
	// 			check++;
	// 		}
	// 		else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 0){ check_14++;}
	// 		i_num[com_mode]++;
	// 	}		
		
	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
	else if (check != 54){
		end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = �͈͊O�w��E�����ُ�
	}
}

/* �[�������ڍ׃f�[�^������(SFC9000�Ή�)(W7) */
void command_W7 (short com_mode){

	short z;
	short l;
	short h;
	short check;
	short check_14;
	short ret;
	unsigned short status;
	char sta[MES_W7_STATUS] = {0};			// �X�e�[�^�X�Ǎ��p
	
	check = 0;
	check_14 = 0;
	ret = 0;
	
		/* ����[mL/min] */
		value_judge(com_mode,7,2);
		if ( (value[com_mode] >= -3000000 && value[com_mode] <= 3000000) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].zero_flow_qat.DWORD = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* ����[m/s] */
		value_judge(com_mode,i_num[com_mode],3);
		if ( ( value[com_mode] >= -30000 && value[com_mode] <= 30000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 3){
			SVD[ch_no[com_mode] -1].zero_flow_vel.DWORD = value[com_mode] * 10;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 3){ check_14++;}
		i_num[com_mode]++;
	/* ����[m/s] */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 3000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_sound_spd = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �ώZ�l[mL/min] */
		value_judge(com_mode,i_num[com_mode],3);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99999999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 12 ) && digit_check[com_mode] <= 3){
			SVD[ch_no[com_mode] -1].zero_addit.DWORD = value[com_mode] * 10000;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 12 || digit_check[com_mode] > 3){ check_14++;}
		i_num[com_mode]++;
	/* ��g�g�`�ő�l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_max = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ��g�g�`�ŏ��l */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_min = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �`�����ԍ�[ps] */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= -99999999 && value[com_mode] <= 99999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD = value[com_mode] / 32;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ���֒l�� */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_correlate = value[com_mode] / 1000;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �[���_�I�t�Z�b�g */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( ( value[com_mode] >= -100000 && value[com_mode] <= 100000 ) || value[com_mode] == -128000) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_zero_offset = (value[com_mode] / 32) + 4000;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* status */
		z = 0;
		status = 0;
		while (RX_buf[com_mode][i_num[com_mode]] != ','){
			sta[z] = RX_buf[com_mode][i_num[com_mode]];
			i_num[com_mode]++;
			z++;
			if(z >= MES_W7_STATUS){	//�z��O�����ݖh�~
				break;
			}
		}
		if (sta[3] >= 0x30 && sta[3] <= 0x39)		status = sta[3] - 0x30;
		else if (sta[3] >= 0x41 && sta[3] <= 0x46)	status = sta[3] - 0x37;
		
		if (sta[2] >= 0x30 && sta[2] <= 0x39)		status += (sta[2] - 0x30) * 0x10;
		else if (sta[2] >= 0x41 && sta[2] <= 0x46)	status += (sta[2] - 0x37) * 0x10;
		
		if (sta[1] >= 0x30 && sta[1] <= 0x39)		status += (sta[1] - 0x30) * 0x100;
		else if (sta[1] >= 0x41 && sta[1] <= 0x46)	status += (sta[1] - 0x37) * 0x100;

		if (sta[0] >= 0x30 && sta[0] <= 0x39)		status += (sta[0] - 0x30) * 0x1000;
		else if (sta[0] >= 0x41 && sta[0] <= 0x46)	status += (sta[0] - 0x37) * 0x1000;		
		
		SVD[ch_no[com_mode] -1].zero_condition = status;
		
		for (h = 0; h < MES_W7_STATUS; h++){sta[h] = 0;}
		i_num[com_mode]++;
	/* FIFO��g�g�`���o�ʒu */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 3800 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_fifo_pos = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* Gain 1st stage */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_gain_1st = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* Gain 2nd stage */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_gain_2nd = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* FIFO CH */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 63 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_fifo_ch = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ��g�̍�(P1-P2) */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_p1p2 = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* �������֒l */
		for (l = 0; l < 40; l++){
			value_judge(com_mode,i_num[com_mode],0);
			if ( ( value[com_mode] >= 0 && value[com_mode] <= 65535 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
				// SVD[ch_no[com_mode] -1].zero_sum_abs[l] = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 0){ check_14++;}
			i_num[com_mode]++;
		}		
		
	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
	else if (check != 54){
		end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = �͈͊O�w��E�����ُ�
	}
}

/* �p���X�o��(W9) */
void command_W9 (short com_mode){
	
	short pls_ch;
	short test_mode = 0;

	/* �p���X�o�̓e�X�gON/OFF */	// ���0�`1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		for (pls_ch = 0; pls_ch < 6; pls_ch++){
			MES[pls_ch].pls_test_enable = value[com_mode];		// �p���X�o��ON/OFF�؂�ւ�
			MES[pls_ch].pls_test_on = 0;			// �SCH�p���XOFF
		}
		if (value[com_mode] == 1){
			test_mode = 1;					// ON�������p
		}
		i_num[com_mode]++;
	/* �p���X1�o��CH�I��*/			// 0(None),1(CH1)�`6(CH6)
		value_judge(com_mode,i_num[com_mode],0);
		if (value[com_mode] >= 0 && value[com_mode] <= 6 && size[com_mode] == 1) {
			for (pls_ch = 0; pls_ch < 6; pls_ch++){
				MES[pls_ch].pls_test_ch1 = value[com_mode];	// �I��CH No.1
			}
			if (value[com_mode] != 0 && test_mode == 1){		// �o��ON��CH�I����(None�ȊO)
				MES[(short)value[com_mode] -1].pls_test_on = 1;	// �I��CH No.1�p���XON
			}
			i_num[com_mode]++;
	/* �p���X2�o��CH�I��*/			// 0(None),1(CH1)�`6(CH6)
			value_judge(com_mode,i_num[com_mode],0);
			if (value[com_mode] >= 0 && value[com_mode] <= 6 && size[com_mode] == 1){
				for (pls_ch = 0; pls_ch < 6; pls_ch++){
					MES[pls_ch].pls_test_ch2 = value[com_mode];	// �I��CH No.2
				}
				if (value[com_mode] != 0 && test_mode == 1){		// �o��ON��CH�I����(None�ȊO)
					MES[(short)value[com_mode] -1].pls_test_on = 1;	// �I��CH No.2�p���XON
				}				
			}else if (size[com_mode] != 1){
				end_code[com_mode] = FORMAT_ERROR;
			}else{
				end_code[com_mode] = DEBUGMODE_ERROR;
			}
		}else if (size[com_mode] != 1){
			end_code[com_mode] = FORMAT_ERROR;
		}else{
			end_code[com_mode] = DEBUGMODE_ERROR;
		}
	}else if (size[com_mode] != 1){
		end_code[com_mode] = FORMAT_ERROR;
	}else{
		end_code[com_mode] = DEBUGMODE_ERROR;
	}
}

/* �������[�h������(SFC9000�Ή�)(Wi) */
void command_Wi (short com_mode){

	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		MES[ch_no[com_mode] -1].inspect_enable = value[com_mode];
	}else if (size[com_mode] != 1){
		end_code[com_mode] = FORMAT_ERROR;
	}else{
		end_code[com_mode] = DEBUGMODE_ERROR;
	}
}

/* �Z���T�p���X�ݒ菑����(SFC014E�݊�)(Wp) */
void command_Wp (short com_mode){

	/* �ō��݃p���X */  //1�`15
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 1 && value[com_mode] <= 15 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].drive_pls = value[com_mode];
 	i_num[com_mode]++;
 /* �������[�h */  //0�`1
 	value_judge(com_mode,i_num[com_mode],0);
		if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
 	 SVD[ch_no[com_mode] -1].drive_search = value[com_mode];
  	i_num[com_mode]++;
	/* �쓮���g�� */  //100�`5000kHz
 	 value_judge(com_mode,i_num[com_mode],0);
	  if ( ( value[com_mode] >= 100 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		  SVD[ch_no[com_mode] -1].drive_freq = value[com_mode];
  	 i_num[com_mode]++;
	/* �T�[�`�J�n���g�� */  //100�`5000kHz
 	  value_judge(com_mode,i_num[com_mode],0);
	   if ( ( value[com_mode] >= 100 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		   SVD[ch_no[com_mode] -1].start_freq = value[com_mode];
  	  i_num[com_mode]++;
	/* �T�[�`��~���g�� */  //100�`5000kHz
 	   value_judge(com_mode,i_num[com_mode],0);
	    if ( ( value[com_mode] >= 100 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		    SVD[ch_no[com_mode] -1].stop_freq = value[com_mode];
  	   i_num[com_mode]++;
     }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
      end_code[com_mode] = FORMAT_ERROR;
     }else{
 	    end_code[com_mode] = FREQUENCY_ERROR;
					}
    }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
     end_code[com_mode] = FORMAT_ERROR;
    }else{
	    end_code[com_mode] = FREQUENCY_ERROR;
				}
   }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
    end_code[com_mode] = FORMAT_ERROR;
   }else{
    end_code[com_mode] = FREQUENCY_ERROR;
   }
  }else if ( size[com_mode] != 1){
   end_code[com_mode] = FORMAT_ERROR;
  }else{
 		end_code[com_mode] = FREQUENCY_ERROR;
  }
 }else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
  end_code[com_mode] = FORMAT_ERROR;
 }else{
  end_code[com_mode] = FREQUENCY_ERROR;
 }
}

/* �Z���T�V���A���i���o�[������(SFC014E�݊�)(Wn) */
void command_Wn (short com_mode){

	short z;
	short l;
	short h;
	short serial;
	short serial_size;
	short check;
	unsigned short *pt;
	char ver[MES_CHAR_MAX] = {0};			// �o�[�W�����Ǎ��p

	z = 0;
	check = 0;
	serial_size = 0;
		/* �Z���T�V���A���i���o�[ */
 i_num[com_mode] = 7;
//	value_judge(com_mode,7,0);
	while (RX_buf[com_mode][i_num[com_mode]] != ','){		//�f�[�^��؂�ʒu����
		if(i_num[com_mode] >= (MSG_MAX-1)){		//','���Ō�܂Ō�����Ȃ��ꍇ
			check++;
			break;
		}
		ver[z] = RX_buf[com_mode][i_num[com_mode]];
		serial_size++;
		i_num[com_mode]++;
		z++;	
		if(z >= MES_CHAR_MAX){	//�z��O�����ݖh�~
			break;
		}
	}
//	for(l = 0; l < 6; l++){
		pt = &SVD[ch_no[com_mode] -1].s_serial[0];
		if (serial_size % 2 == 0){					//������������
			for (z = 0; z < serial_size; z = z+2){
				serial = ver[z] * 0x0100 + ver[z+1];
				*pt = serial;
				pt++;
			}
			for (z = serial_size; z < 16; z = z+2){		//16�������� �󔒑��
				serial = 0x2020;
				*pt = serial;
				pt++;
			}
		}
		else{							//���������
			for (z = 0; z < serial_size -1; z = z+2){
				serial = ver[z] * 0x0100 + ver[z+1];
				*pt = serial;
				pt++;
			}
			serial = ver[z] * 0x0100 + 0x20;
			*pt = serial;
			pt++;
			for (z = serial_size +1; z < 16; z = z+2){		//16�������� �󔒑��
				serial = 0x2020;
				*pt = serial;
				pt++;
			}
		}
//	}
	for (h = 0; h < MES_CHAR_MAX; h++){ver[h] = 0;}

	if (check > 0 || serial_size < 0 || serial_size > 16){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = �t�H�[�}�b�g�G���[
	}
}

/****************************************************
 * Function : GetValSiz (Get Value Size)
 * Summary  : Val�̌������擾����
 * Argument : Val : �Ώۂ̒l
 * Return   : Siz : ����
 * Caution  : 
 * notes    : �����̏ꍇ��-��1���ƃJ�E���g����
 ****************************************************/
short GetValSiz(long long Val){
	short Siz = 0;
	if(Val < 0)
	{
		Val *= -1;
		Siz = 1;
	}
	while(0 < Val)
	{
		Val /= 10;
		Siz++;
	}
	return Siz;
}

/****************************************************
 * Function : CngUstPrm (Change Signed short Parameter)
 * Summary  : �v���������p�����[�^�̒l�𔻒�A�X�V����
 * Argument : com_mode : 
 *            Prm : �p�����[�^�̃������A�h���X
 *            MinVal : ���͉\�����W�ŏ��l
 *            MaxVal : ���͉\�����W�ő�l
 *            pow : �����_�ȉ�����
 *            ErrCod : �G���[�R�[�h
 * Return   : Flg :  0 ����I��
 *                  -1 �t�H�[�}�b�g�G���[
 *                  -2 ���̓����W�O�G���[
 * Caution  : 
 * notes    : 
 ****************************************************/
short CngSstPrm(short com_mode, short *Prm, long long MinVal, long long MaxVal, short pow, short ErrCod)
{
	short Flg = 0;
	short MinSiz = GetValSiz(MinVal);
	short MaxSiz = GetValSiz(MaxVal);

	//�����_�ȉ��������g�p����ꍇ��.�̕�1���₷
	if(pow > 0)
	{
		MaxSiz += 1;
	}

	//value[], size[], digit_check[] ���擾����
	value_judge(com_mode, i_num[com_mode], pow);

	//�����񒷔���
	if(size[com_mode] < MinSiz || MaxSiz < size[com_mode])
	{
		Flg = -1; //FORMAT ERROR
		end_code[com_mode] = FORMAT_ERROR;
	}
	//���͔͈͔���
	else if(value[com_mode] < MinVal || MaxVal < value[com_mode])
	{
		Flg = -2; //
		end_code[com_mode] = ErrCod;
	}
	else if(digit_check[com_mode] > pow)
	{
		Flg = -1; //FORMAT ERROR
		end_code[com_mode] = FORMAT_ERROR;
	}
	else
	{
		*Prm = (short)value[com_mode];
		i_num[com_mode]++;
	}
	return Flg;
}

/****************************************************
 * Function : CngUstPrm (Change Unsigned short Parameter)
 * Summary  : �v���������p�����[�^�̒l�𔻒�A�X�V����
 * Argument : com_mode : 
 *            Prm : �p�����[�^�̃������A�h���X
 *            MinVal : ���͉\�����W�ŏ��l
 *            MaxVal : ���͉\�����W�ő�l
 *            pow : �����_�ȉ�����
 *            ErrCod : �G���[�R�[�h
 * Return   : Flg :  0 ����I��
 *                  -1 �t�H�[�}�b�g�G���[
 *                  -2 ���̓����W�O�G���[
 * Caution  : 
 * notes    : 
 ****************************************************/
short CngUstPrm(short com_mode, unsigned short *Prm, long long MinVal, long long MaxVal, short pow, short ErrCod)
{
	short Flg = 0;
	short MinSiz = GetValSiz(MinVal);
	short MaxSiz = GetValSiz(MaxVal);

	//�����_�ȉ��������g�p����ꍇ��.�̕�1���₷
	if(pow > 0)
	{
		MaxSiz += 1;
	}

	//value[], size[], digit_check[] ���擾����
	value_judge(com_mode, i_num[com_mode], pow);
	//�����񒷔���
	if(size[com_mode] < MinSiz || MaxSiz < size[com_mode])
	{
		Flg = -1; //FORMAT ERROR
		end_code[com_mode] = FORMAT_ERROR;
	}
	//���͔͈͔���
	else if(value[com_mode] < MinVal || MaxVal < value[com_mode])
	{
		Flg = -2; //
		end_code[com_mode] = ErrCod;
	}
	else if(digit_check[com_mode] > pow)
	{
		Flg = -1; //FORMAT ERROR
		end_code[com_mode] = FORMAT_ERROR;
	}
	else
	{
		*Prm = (unsigned short)value[com_mode];
		i_num[com_mode]++;
	}
	return Flg;
}

/* �Z���T��񏑍���(�]���p)(Wa) */
void command_Wa (short com_mode){
	short pch = ch_no[com_mode] -1;
	short Flg = 0;
#if 1
	i_num[com_mode] = 7;
	//1. �Z���T�I�v�V����
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_option, 0, 1, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//2. �Z���T�ԋ���(L)
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_disL, 100, 9999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//3. �Z���T�ԋ���(L_l)
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_disL_l, 100, 9999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//4. ���ʎ���
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_tau, 100, 9999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//5. �݊��W��
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_coef, 1000, 99999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//6. ADC�N���b�N
	Flg = CngUstPrm(com_mode, &SVD[pch].adc_clock, 0, 3, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//7. WINDOW OFFSET
	Flg = CngUstPrm(com_mode, &SVD[pch].wind_offset, 0, 63, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//8. �������֊J�n�ʒu
	Flg = CngUstPrm(com_mode, &SVD[pch].sum_start, 4, 210, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//9. �������֏I���ʒu
	Flg = CngUstPrm(com_mode, &SVD[pch].sum_end, 5, 210, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//10. �������֊Ԋu
	Flg = CngUstPrm(com_mode, &SVD[pch].sum_step, 1, 10, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//11. �Œ�l�ݒ�
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_data, 0x0000, 0xFFFF, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//12. Wiper Position(�Œ�l)
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_amp_gain_rev, 0, 255, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//13. FIFO CH(�Œ�l)
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_fifo_ch_read, 0, 63, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//14. Leading Position(�Œ�l)
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_fifo_no_read, 0, 1000, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//15. Zero Cross Start Point
	// Flg = CngSstPrm(com_mode, &SVD[pch].ZerCrsSttPnt, 0, 50, 0, FREQUENCY_ERROR);
	Flg = CngUstPrm(com_mode, &SVD[pch].ZerCrsSttPnt, 0, 300, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;
	SVD[pch].ZerPeakPos = MES[pch].zc_peak = SVD[pch].ZerCrsSttPnt;
	eep_write_ch_delay(pch, (short)(&SVD[pch].ZerPeakPos - &SVD[pch].max_flow), SVD[pch].ZerPeakPos);

	//16. Zero Cross Use Number
	Flg = CngUstPrm(com_mode, &SVD[pch].ZerCrsUseNum, 0, 50, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;
#else 
 /* �Z���T�I�v�V���� */  //0�`1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
	 SVD[ch_no[com_mode] -1].sns_option = value[com_mode];
 	i_num[com_mode]++;
	/* �Z���T�ԋ���(L) */  //100�`9999*0.1mm
	 value_judge(com_mode,i_num[com_mode],0);
  if ( ( value[com_mode] >= 100 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	  SVD[ch_no[com_mode] -1].sns_disL = value[com_mode];
 	 i_num[com_mode]++;
	/* �Z���T�ԋ���(L_l) */  //100�`9999*0.1mm
	 value_judge(com_mode,i_num[com_mode],0);
  if ( ( value[com_mode] >= 100 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	  SVD[ch_no[com_mode] -1].sns_disL_l = value[com_mode];
 	 i_num[com_mode]++;
	/* ���ʎ��� */  //100�`9999
	  value_judge(com_mode,i_num[com_mode],0);
   if ( ( value[com_mode] >= 100 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	   SVD[ch_no[com_mode] -1].sns_tau = value[com_mode];
 	  i_num[com_mode]++;
	/* �݊��W�� */  //1000�`99999
 	  value_judge(com_mode,i_num[com_mode],0);
	   if ( ( value[com_mode] >= 1000 && value[com_mode] <= 99999 ) && ( size[com_mode] >= 4 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
		   SVD[ch_no[com_mode] -1].sns_coef = value[com_mode];
  	  i_num[com_mode]++;
	/* adc_clock */  //0�`3
  	  value_judge(com_mode,i_num[com_mode],0);
	    if ( ( value[com_mode] >= 0 && value[com_mode] <= 3 ) && size[com_mode] == 1 ){
		    SVD[ch_no[com_mode] -1].adc_clock = value[com_mode];
  	   i_num[com_mode]++;
	/* WINDOW OFFSET */  //0�`63
   	  value_judge(com_mode,i_num[com_mode],0);
	     if ( ( value[com_mode] >= 0 && value[com_mode] <= 63 ) && size[com_mode] <= 2 ){
		     SVD[ch_no[com_mode] -1].wind_offset = value[com_mode];
  	    i_num[com_mode]++;
	/* �������֊J�n�ʒu */  //4�`210
    	  value_judge(com_mode,i_num[com_mode],0);
	      if ( ( value[com_mode] >= 4 && value[com_mode] <= 210 ) && size[com_mode] <= 3 ){
		      SVD[ch_no[com_mode] -1].sum_start = value[com_mode];
  	     i_num[com_mode]++;
	/* �������֏I���ʒu */  //5�`210
     	  value_judge(com_mode,i_num[com_mode],0);
	       if ( ( value[com_mode] >= 5 && value[com_mode] <= 210 ) && size[com_mode] <= 3 ){
		       SVD[ch_no[com_mode] -1].sum_end = value[com_mode];
  	      i_num[com_mode]++;
	/* �������֊Ԋu */  //1�`10
      	  value_judge(com_mode,i_num[com_mode],0);
	        if ( ( value[com_mode] >= 1 && value[com_mode] <= 10 ) && size[com_mode] <= 2 ){
		        SVD[ch_no[com_mode] -1].sum_step = value[com_mode];
  	       i_num[com_mode]++;
										
	/* �Œ�l�ݒ� */  //0x0000�`0xFFFF
      	   value_judge(com_mode,i_num[com_mode],0);
	         if ( ( value[com_mode] >= 0x0000 && value[com_mode] <= 0xFFFF ) && size[com_mode] <= 5 ){
		         SVD[ch_no[com_mode] -1].fix_data = value[com_mode];
  	        i_num[com_mode]++;
	/* Wiper Position(�Œ�l) */  //1�`255
       	   value_judge(com_mode,i_num[com_mode],0);
	          if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && size[com_mode] <= 3 ){
		          SVD[ch_no[com_mode] -1].fix_amp_gain_rev = value[com_mode];
  	         i_num[com_mode]++;
	/* FIFO CH(�Œ�l) */  //0�`63
       	    value_judge(com_mode,i_num[com_mode],0);
	           if ( ( value[com_mode] >= 0 && value[com_mode] <= 63 ) && size[com_mode] <= 2 ){
		           SVD[ch_no[com_mode] -1].fix_fifo_ch_read = value[com_mode];
  	          i_num[com_mode]++;
	/* Leading Position(�Œ�l) */  //0�`1000
        	  value_judge(com_mode,i_num[com_mode],0);
	          if ( ( value[com_mode] >= 0 && value[com_mode] <= 1000 ) && size[com_mode] <= 4 ){
		        SVD[ch_no[com_mode] -1].fix_fifo_no_read = value[com_mode];
  	           	i_num[com_mode]++;

/* Zero Cross Start Point */ //0~50
				value_judge(com_mode,i_num[com_mode],0);
	            // if ( ( value[com_mode] >= 0 && value[com_mode] <= 50 ) && size[com_mode] <= 2 )
	            if ( ( value[com_mode] >= 0 && value[com_mode] <= 300 ) && size[com_mode] <= 3 )
				{
		          SVD[ch_no[com_mode] -1].ZerCrsSttPnt = value[com_mode];
				  SVD[ch_no[com_mode] -1].ZerPeakPos = MES[ch_no[com_mode] -1].zc_peak = value[com_mode];
  	           	  i_num[com_mode]++;
				  eep_write_ch_delay(pch, (short)(&SVD[pch].ZerPeakPos - &SVD[pch].max_flow), SVD[pch].ZerPeakPos);

/* Zero Cross Use Number */ //0~50
				  value_judge(com_mode,i_num[com_mode],0);
				  if ( ( value[com_mode] >= 0 && value[com_mode] <= 50 ) && size[com_mode] <= 2 ){
					SVD[ch_no[com_mode] -1].ZerCrsUseNum = value[com_mode];
					i_num[com_mode]++;
				  }else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
					end_code[com_mode] = FORMAT_ERROR;
				  }else{
					end_code[com_mode] = FREQUENCY_ERROR;
				  }
/* Zero Cross Use Number �����܂� */  

				}else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
				  end_code[com_mode] = FORMAT_ERROR;
				}else {
				  end_code[com_mode] = FREQUENCY_ERROR;
				}
/* Zero Cross Start Point �����܂� */

  			  }else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
                end_code[com_mode] = FORMAT_ERROR;
              }else{
                end_code[com_mode] = FREQUENCY_ERROR;
              }
 											}else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
             end_code[com_mode] = FORMAT_ERROR;
            }else{
             end_code[com_mode] = FREQUENCY_ERROR;
            }
											}else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
            end_code[com_mode] = FORMAT_ERROR;
           }else{
            end_code[com_mode] = FREQUENCY_ERROR;
           }
          }else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 0){
           end_code[com_mode] = FORMAT_ERROR;
          }else{
           end_code[com_mode] = FREQUENCY_ERROR;
          }
         }else if ( size[com_mode] < 1 || size[com_mode] > 10 || digit_check[com_mode] > 0){
          end_code[com_mode] = FORMAT_ERROR;
         }else{
          end_code[com_mode] = FREQUENCY_ERROR;
         }
        }else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
         end_code[com_mode] = FORMAT_ERROR;
        }else{
         end_code[com_mode] = FREQUENCY_ERROR;
        }
       }else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
        end_code[com_mode] = FORMAT_ERROR;
       }else{
        end_code[com_mode] = FREQUENCY_ERROR;
       }
      }else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
       end_code[com_mode] = FORMAT_ERROR;
      }else{
       end_code[com_mode] = FREQUENCY_ERROR;
      }
     }else if ( size[com_mode] != 1){
      end_code[com_mode] = FORMAT_ERROR;
     }else{
 		   end_code[com_mode] = FREQUENCY_ERROR;
     }
    }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
     end_code[com_mode] = FORMAT_ERROR;
    }else{
	    end_code[com_mode] = FREQUENCY_ERROR;
			 }
   }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
    end_code[com_mode] = FORMAT_ERROR;
   }else{
    end_code[com_mode] = FREQUENCY_ERROR;
	 	}
  }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
    end_code[com_mode] = FORMAT_ERROR;
   }else{
    end_code[com_mode] = FREQUENCY_ERROR;
	 	}
  }else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){
   end_code[com_mode] = FORMAT_ERROR;
  }else{
   end_code[com_mode] = FREQUENCY_ERROR;
  }
 }else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
  end_code[com_mode] = FORMAT_ERROR;
 }else{
  end_code[com_mode] = FREQUENCY_ERROR;
 }
#endif
}

/* �ݒ�l�ۑ�(OS) */
void command_OS (short com_mode){

	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}

	action_status_control(ch_no[com_mode]-1,ACT_STS_WRITE);

	util_eep_allwrite(ch_no[com_mode]-1, WR_ALL);		//EEPROM������
}

/* �Z���T�������ݒ�l�ۑ�(Os) */
void command_Os (short com_mode){
	short pch = ch_no[com_mode] - 1;
	
	//���ɏ������ݗv������
	if(OWwrite[pch] != 0)
	{
		end_code[com_mode] = EEPROMWRITE_NOW; //EEPROM�������ݒ��G���[�ő�p
		return;
	}

	action_status_control(pch, ACT_STS_WRITE);

	util_SnsMem_Write(pch, WR_DEVICE); //�Z���T�f�o�C�X������
}

/* �[�������l�ۑ�(OJ) */
void command_OJ (short com_mode){

	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}

	action_status_control(ch_no[com_mode]-1,ACT_STS_WRITE);

	util_eep_zerowrite(ch_no[com_mode]-1);		//EEPROM������
}

/* ���O�N���A(OL) */
void command_OL (short com_mode){

	log_detailinfo_init(ch_no[com_mode]-1);		/*���O��񏉊���*/
}

/* �q�`�l�N���A(OR) */
void command_OR (void){

	ram_clear_debug();
}

/* �k�d�c�S�_��(OT) */
void command_OT (void){

	if(led_flash == 0){
		led_flash = 0x1000;
	}
	else{
		led_flash = 0;
	}
}

/* �A���[�������o��(OA) */
void command_OA (void){

	if(alm_output == 0){
		alm_output = 0x1000;
		// GPIO_PE7
		__bit_output(GPIO_PORTE_BASE, 7, 1);
		// GPIO_PE6
		__bit_output(GPIO_PORTE_BASE, 6, 1);
	}
	else{
		alm_output = 0;
		// GPIO_PE7
		__bit_output(GPIO_PORTE_BASE, 7, 0);
		// GPIO_PE6
		__bit_output(GPIO_PORTE_BASE, 6, 0);
	}
}

/* �ώZ�l���Z�b�g(Ot) */
void command_Ot (short com_mode){

	MES[ch_no[com_mode] -1].addit_unit_ov.UINT64 = 0;
	MES[ch_no[com_mode] -1].addit_mod_ov = 0;
	MES[ch_no[com_mode] -1].addit_buff_ov.UINT64 = 0;
	
	MES[ch_no[com_mode] -1].total_status &= ~TTL_JUDGE_REACH;		//�ώZ���B�o�̓��Z�b�g
	MES[ch_no[com_mode] -1].total_status &= ~TTL_JUDGE_OVERFLOW;	//�ώZ�l�I�[�o�[�t���[�N���A
	
	if(MES[ch_no[com_mode] -1].addit_watch == B_ON){		//�ώZ�Ď��L��
		if(MES[ch_no[com_mode] -1].addit_unit_ov.UINT64 != 0	//�ώZ�֘A�f�[�^���N���A���Ă��Ȃ��ꍇ
			|| MES[ch_no[com_mode] -1].addit_mod_ov != 0
			|| MES[ch_no[com_mode] -1].addit_buff_ov.UINT64 != 0){
			MES[ch_no[com_mode] -1].total_status |= TTL_JUDGE_CACL_ERR;		//�ώZ�l���Z�ُ�
		}
	}
}

/* �[���������s(OZ) */
void command_OZ (short com_mode){

	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}
 SAVE[ch_no[com_mode] -1].control |= 0x0001;
}

/*******************************************
 * Function : command_Wc
 * Summary  : �f�W�^���t�B���^�W����������
 * Argument : int com_mode -> 0 : �z�X�g
 *                            1 : �����e
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : @Wc000,n,dddd
 *          :   n : �f�W�^���t�B���^�W���̃C���f�b�N�X(����:��������16bit, �:�������2bit)
 *          :   dddd : �������ރf�[�^
 * *****************************************/
void command_Wc(short com_mode){
	short Idx, i;
	long Val;
	unsigned short MinVal, MaxVal;

	//1�ڂ̃f�[�^�擾
	i_num[com_mode] = 7;
	value_judge(com_mode, i_num[com_mode], 0);
	Idx = value[com_mode];

	//2�ڂ̃f�[�^�擾
	i_num[com_mode]++;
	value_judge(com_mode, i_num[com_mode], 0);
	Val = value[com_mode];

	//�f�W�^���t�B���^�X�C�b�`�؂�ւ�
	if(Idx == 20){
		MinVal = 0;
		MaxVal = 0xFFFF;
	}
	//������������(�ő�16bit)
	else if(Idx % 2 == 0){
		MinVal = 0;
		MaxVal = 0xFFFF;
	}
	//����/������������(�ő�2bit)
	else{
		MinVal = 0;
		MaxVal = 0x3;
	}
	
	if(JdgIdxErr(Idx) == B_NG)
	{
		end_code[com_mode] = FORMAT_ERROR;
	}
	else if((Val < MinVal) || (MaxVal < Val))
	{
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		switch (Idx)
		{
		case 0:  FPGA_FILIN0_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA00 = Val; break;	/* �f�W�^���t�B���^�W��(���͑�0) */
		case 1:  FPGA_FILIN0_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA01 = Val; break;	/* �f�W�^���t�B���^�W��(���͑�0) */
		case 2:  FPGA_FILIN1_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA10 = Val; break;	/* �f�W�^���t�B���^�W��(���͑�1) */
		case 3:  FPGA_FILIN1_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA11 = Val; break;	/* �f�W�^���t�B���^�W��(���͑�1) */
		case 4:  FPGA_FILIN2_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA20 = Val; break;	/* �f�W�^���t�B���^�W��(���͑�2) */
		case 5:  FPGA_FILIN2_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA21 = Val; break;	/* �f�W�^���t�B���^�W��(���͑�2) */
//		case 6:  FPGA_FILCOE_A3_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA30 = Val; break;
//		case 7:  FPGA_FILCOE_A3_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA31 = Val; break;
//		case 8:  FPGA_FILCOE_A4_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA40 = Val; break;
//		case 9:  FPGA_FILCOE_A4_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA41 = Val; break;
		// case 10: FPGA_FIL_EN = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB00 = Val; break;	/* �f�W�^���t�B���^�L���E�����ݒ� */
//		case 11: FPGA_FILCOE_B0_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB01 = Val; break;
		case 12: FPGA_FILOUT1_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB10 = Val; break;	/* �f�W�^���t�B���^�W��(�o�͑�1) */
		case 13: FPGA_FILOUT1_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB11 = Val; break;	/* �f�W�^���t�B���^�W��(�o�͑�1) */
		case 14: FPGA_FILOUT2_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB20 = Val; break;	/* �f�W�^���t�B���^�W��(�o�͑�2) */
		case 15: FPGA_FILOUT2_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB21 = Val; break;	/* �f�W�^���t�B���^�W��(�o�͑�2) */
//		case 16: FPGA_FILCOE_B3_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB30 = Val; break;
//		case 17: FPGA_FILCOE_B3_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB31 = Val; break;
		case 20: FPGA_FIL_EN = Val; for(i=0; i<6; i++) SVD[i].DgtFltSwc = Val; break;	/* �f�W�^���t�B���^�L���E�����ݒ� */
		default:
			break;
		}
	}
}

void read_command_W(short com_mode){

	switch (RX_buf[com_mode][2]){		//�w�b�_�[�R�[�h����
/* ���a������(Wg) */
	case 'g':
		command_Wg(com_mode);
		break;
/* �t���X�P�[��������(Wr) */
	case 'r':
		command_Wr(com_mode);
		break;
/* �j�t�@�N�^������(Wk) */
	case 'k':
		command_Wk(com_mode);
		break;
/* �_���s���O������(Wd) */
	case 'd':
		command_Wd(com_mode);
		break;
/* ���[�J�b�g������(Wl) */
	case 'l':
		command_Wl(com_mode);
		break;
/* �o�[���A�E�g������(Wb) */
	case 'b':
		command_Wb(com_mode);
		break;
/* ���S�x�W��������(Wv) */
	case 'v':
		command_Wv(com_mode);
		break;
/* �G���[�z�[���h�^�C��������(Wh) */
	case 'h':
		command_Wh(com_mode);
		break;
/* �t������l������(WR) */
	case 'R':
		command_WR(com_mode);
		break;
/* ���[�U�[���j�A���C�Y������(Wu) */
	case 'u':
		command_Wu(com_mode);
		break;
/* ���[�U�[���j�A���C�Y�؂�ւ�(WU) */		
	case 'U':
		command_WU(com_mode);
		break;
/* ���[�J�[���j�A���C�Y������(Wm) */
	case 'm':
		command_Wm(com_mode);
		break;
/* �[�������f�[�^������(SFC9000�Ή�)(Wz) */	
	case 'z':
		command_Wz(com_mode);
		break;		
/* �[�������f�[�^������(WZ) */
	/* �}���`�R�}���h */
	case 'Z':
		command_WZ(com_mode);
		break;
/* �p�����[�^�Z�b�g������(WP) */
	/* �}���`�R�}���h */
	case 'P':
		command_WP(com_mode);
		break;
/* �g�`�ُ픻��l�ݒ菑����(WE) */
	case 'E':
		command_WE(com_mode);
		break;
/* �A�b�e�l�[�^�Q�C��������(WA) */
	case 'A':
		command_WA(com_mode);
		break;
/* ��t���j�A���C�Y���[�h������(Wy) */
	case 'y':
		command_Wy(com_mode);
		break;
/* �t�B���^�ݒ菑����(Wf) */
	case 'f':
		command_Wf(com_mode);
		break;
/* �ώZ�ڕW�l������(Wt) */
	case 't':
		command_Wt(com_mode);
		break;	
/* �ώZ�I�t�Z�b�g�l������(WT) */
	case 'T':
		command_WT(com_mode);
		break;		
/* ���[�J�[�ݒ菑����(W1) */
	case '1':
		command_W1(com_mode);
		break;
/* �f�o�b�O���[�h�ݒ菑����(W2) */
	case '2':
		command_W2(com_mode);
		break;
/* ���[�J�[�ݒ菑����(SFC9000�Ή�)(W5) */
	case '5':
		command_W5(com_mode);
		break;		
/* �f�o�b�O���[�h�ݒ菑����(SFC9000�Ή�)(W6) */
	case '6':
		command_W6(com_mode);
		break;
/* �[�������ڍ׃f�[�^������(SFC9000�Ή�)(W7) */
	case '7':
		command_W7(com_mode);
		break;
/* �p���X�o��(W9) */
	case '9':
		command_W9(com_mode);
		break;		
		/* �������[�h������(SFC9000�Ή�)(Wi) */
	case 'i':
		command_Wi(com_mode);
		break;
/* �Z���T�p���X�ݒ菑����(Wp) */
	case 'p':
		command_Wp(com_mode);
		break;
/* �Z���T�V���A���i���o�[������(Wn) */
	case 'n':
		command_Wn(com_mode);
		break;		
/* �Z���T��񏑍���(WL)(�]���p) */
	case 'a':
		command_Wa(com_mode);
		break;
/* �f�W�^���t�B���^�W�� */
	case 'c':
		command_Wc(com_mode);
		break;
	}
	
}

#ifdef MEMDBG
//Debug Function
/*******************************************
 * Function : MemoryCom
 * Summary  : Mx�R�}���h����
 * Argument : int com_mode -> 0 : �z�X�g
 *                            1 : �����e
 * Return   : void
 * Caution  : �Ȃ�
 * Note     :
 * *****************************************/
void MemoryCom(short com_mode){
	switch (RX_buf[com_mode][2])
	{
	case 'R':
		/* �������ǂݏo�� */
		command_MR(com_mode);
		break;
	case 'W':
		/* �������������� */
		command_MW(com_mode);
		break;
	case 'T':
		command_MT(com_mode);
		break;
	case 'Z':
		command_MZ(com_mode);
		break;
	case 'S':
		/* �������f�o�C�X�V���A���i���o�[�Ǐo�� */
		command_MS(com_mode);
		break;
	default:
		break;
	}
}

/* Debug (MR) */
/*******************************************
 * Function : command_MR
 * Summary  : �������ǂݏo���R�}���h(MR)
 * Argument : int com_mode -> 0 : �z�X�g
 *                            1 : �����e
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : MR00x,�������A�h���X,�ǂݏo����(�ő�0x80)
 * *****************************************/
void command_MR (short com_mode){
	long Add;
	long Cnt;
	short i = 0;
	unsigned char pAdd;
	long TxCnt = 9;
	short TmpChr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}; 
	value_judge_Hex(com_mode,7,0);

	//1�p�����[�^�ځA�A�h���X
	if((value[com_mode] < 0x00000000) || (0xFFFFFFFF < value[com_mode]))
	{
	    end_code[com_mode] = FORMAT_ERROR;
	}
	// else if((size[com_mode] < 1) || (8 < size[com_mode]))
	else if(size[com_mode] != 8)
	{
	    end_code[com_mode] = FORMAT_ERROR;
	}
	else
	{
	    Add = value[com_mode];

	    i_num[com_mode]++;
	    value_judge_Hex(com_mode,i_num[com_mode],0);

	    //2�p�����[�^�ځA��
	    if((value[com_mode] < 0x00) || (0x80 < value[com_mode]))
	    {
	        end_code[com_mode] = FORMAT_ERROR;
	    }
	    else if(size[com_mode] != 0x02)
        {
	        end_code[com_mode] = FORMAT_ERROR;
        }
	    else
	    {
	        Cnt = value[com_mode];

	        TX_buf[com_mode][8] = ',';
	        for(i = 0; i< Cnt; i++){
	            pAdd = *(unsigned char*)(Add + i);
	            TX_buf[com_mode][TxCnt] = TmpChr[pAdd / 0x10];
	            TxCnt++;
	            TX_buf[com_mode][TxCnt] = TmpChr[pAdd % 0x10];
	            TxCnt++;
	            if((i % 4) == 3){
	                TX_buf[com_mode][TxCnt] = ' ';
	                TxCnt++;
	            }
	        }
	        TX_buf[com_mode][TxCnt] = ',';
	    }
	}
}

/* Debug (MW) */
	/*******************************************
	 * Function : command_MW
	 * Summary  : �������������݃R�}���h(MW)
	 * Argument : int com_mode -> 0 : �z�X�g
	 *                            1 : �����e
	 * Return   : void
	 * Caution  : �Ȃ�
	 * Note     : MW00x,�������A�h���X,�������݃o�C�g��(�ő�0x04),�������݃f�[�^
	 * *****************************************/
void command_MW (short com_mode){
	unsigned long Add;
	long Cnt;
	short i = 0;
	unsigned char pAdd;
	unsigned char ChrDat;
	unsigned short IntDat;
	unsigned long LngDat;
	long TxCnt = 9;
	short TmpChr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}; 
	value_judge_Hex(com_mode,7,0);

	//�������݃A�h���X
	if((value[com_mode] < 0x00000000) || (0xFFFFFFFF < value[com_mode]))
	{
	    end_code[com_mode] = FORMAT_ERROR;
	}
	// else if((size[com_mode] < 0x00) || (0x08 < size[com_mode]))
	else if((size[com_mode] != 0x08))
    {
        end_code[com_mode] = FORMAT_ERROR;
    }
	else
	{
	    Add = value[com_mode];

	    i_num[com_mode]++;
	    value_judge_Hex(com_mode,i_num[com_mode],0);

	    //�������݃o�C�g��
	    if((value[com_mode] < 0x01) || (0x04 < value[com_mode]))
	    {
	        end_code[com_mode] = FORMAT_ERROR;
	    }
	    else if((size[com_mode] < 0x02) || (0x02 < size[com_mode]))
	    {
	        end_code[com_mode] = FORMAT_ERROR;
	    }
	    else
	    {
	        Cnt = value[com_mode];
	        i_num[com_mode]++;
	        value_judge_Hex(com_mode,i_num[com_mode],0);

	        //�������݃f�[�^
	        if((value[com_mode] < 0x00000000) || (0xFFFFFFFF < value[com_mode])){
	            end_code[com_mode] = FORMAT_ERROR;
	        }
	        else if((size[com_mode] < 0x02) || (0x08 < size[com_mode])){
	            end_code[com_mode] = FORMAT_ERROR;
	        }
	        else{
				if(Cnt == 1){
					ChrDat = (value[com_mode] & 0xFF);
					*((volatile unsigned char *)Add) = ChrDat;
				}
				else if(Cnt == 2){
					IntDat = (value[com_mode] & 0xFFFF);
					*((volatile unsigned short *)Add) = IntDat;
				}
				else if(Cnt == 4){
					LngDat = (value[com_mode] & 0xFFFFFFFF);
					*((volatile unsigned long *)Add) = LngDat;
				}
				else{
					//1,2,4�����󂯕t���Ȃ�
					end_code[com_mode] = FORMAT_ERROR;
					return;
				}
	            
	            TX_buf[com_mode][8] = ',';
	            for(i = 0; i< Cnt; i++){
	                pAdd = *(unsigned long *)(Add + i);
	                TX_buf[com_mode][TxCnt] = TmpChr[pAdd / 0x10];
	                TxCnt++;
	                TX_buf[com_mode][TxCnt] = TmpChr[pAdd % 0x10];
	                TxCnt++;
	                if((i % 4) == 3){
	                    TX_buf[com_mode][TxCnt] = ' ';
	                    TxCnt++;
	                }
	            }
	            TX_buf[com_mode][TxCnt] = ',';
	        }
	    }
	}

}
#endif

#if defined(FRQSCH)
/*******************************************
 * Function : command_OF
 * Summary  : �����[�������{�R�}���h(OF)
 * Argument : int com_mode -> 0 : �z�X�g
 *                            1 : �����e
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : 014E�݊��R�}���h
 *            �����[�����̗���
 *            1. OF�R�}���h���{
 *            2. ���g������(�U�����ő�ɂȂ���g����T��)
 *            3. �ʏ�Z����
 * *****************************************/
void command_OF (short com_mode){
	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}
	FrqSch[ch_no[com_mode] -1].FrqSchSttFlg = 1;
}
#endif

/*** FPGA�_�E�����[�h�֘A ***/
extern void memcpy_dl(void *buf1, const void *buf2, size_t n);
// extern short FROM_WRITE(void *ptr, unsigned short data[]);
extern short FPGA_WRITE(void *ptr, unsigned short data[]);
extern void block_erase(void *ptr);
extern void memsetW(unsigned short *dst, unsigned short sdata, unsigned long size);
unsigned short WrtDat[128];
/*******************************************
 * Function : CvtDecAsc
 * Summary  : 10�i����char�ϊ�
 * Argument : DecNum -> �ϊ�������10�i���f�[�^
 * Return   : ChrDat -> �ϊ�����ascii�f�[�^
 * Caution  : None
 * Note     : 
 * *****************************************/
char CvtDecAsc(short DecNum){
	char ChrDat = 0;
	if((0 <= DecNum)&& (DecNum <= 9)){
		ChrDat = DecNum + 0x30;
	}
	else if((10 <= DecNum)&& (DecNum <= 15)){
		ChrDat = DecNum + 0x37;
	}
	else{
		ChrDat = 0;
	}
	return ChrDat;
}
/*******************************************
 * Function : CvtAscDec
 * Summary  : Ascii��10�i���ϊ�
 * Argument : ChrDat -> �ϊ�������ascii�f�[�^
 * Return   : DecNum -> �ϊ�����10�i���f�[�^
 * Caution  : None
 * Note     : 
 * *****************************************/
char CvtAscDec(char ChrDat){
	char DecNum = 0;
	//0-9
	if((0x30 <= ChrDat)&& (ChrDat <= 0x39)){
		DecNum = ChrDat - 0x30;
	}
	//A-F
	else if((0x41 <= ChrDat)&& (ChrDat <= 0x46)){
		DecNum = ChrDat - 0x37;
	}
	//a-f
	else if((0x61 <= ChrDat)&& (ChrDat <= 0x66)){
		DecNum = ChrDat - 0x57;
	}
	else{
		DecNum = 0;
	}
	return DecNum;
}
/*******************************************
 * Function : ErsBlk (Erase Block)
 * Summary  : Flash�f�[�^�̃u���b�N�P�ʂ̏���
 * Argument : Add -> Flash�̈�A�h���X
 * Return   : void
 * Caution  : None
 * Note     : �t���b�V���̈��0x4000�P�ʂŃu���b�N������Ă���
 * *****************************************/
void ErsBlk(unsigned long Add)
{
	// �Y���u���b�N����(0x4000�P��)
	block_erase((void *)Add);
}
/*******************************************
 * Function : ErsAllBlk (Erase All Blocks)
 * Summary  : FPGA�f�[�^�̏���
 * Argument : None
 * Return   : void
 * Caution  : None
 * Note     : �t���b�V���̈��0x4000�P�ʂŃu���b�N������Ă���
 *          : FPGA Config�f�[�^�̊i�[�ꏊ��0x00040000-0x00100000
 * *****************************************/
void ErsAllBlk()
{
	unsigned long Add = 0x00040000;
	for(Add = 0x00040000; Add < 0x00100000; Add += 0x4000){
		// �Y���u���b�N����(0x4000�P��)
		block_erase((void *)Add);
	}
}
/*******************************************
 * Function : DlRead
 * Summary  : Flash�f�[�^�̓ǂݏo��
 * Argument : Add -> Flash�̈�A�h���X
 *          : *TbfPtr -> Tx_buf�̃|�C���^
 * Return   : void
 * Caution  : None
 * Note     : 
 * *****************************************/
void DlRead(char *TbfPtr, unsigned long Add)
{
    short i;
    char TmpChr;
	*(TbfPtr++) = ',';
	for(i=0; i<0x100; i++){
	    TmpChr = (char)(*(unsigned char*)(Add + i));
		*(TbfPtr++) = CvtDecAsc(TmpChr / 0x10);
        *(TbfPtr++) = CvtDecAsc(TmpChr % 0x10);
	}
	*(TbfPtr++) = ',';
}
/*******************************************
 * Function : DlWrite
 * Summary  : Flash�f�[�^�̏�������
 * Argument : Add -> Flash�̈�A�h���X
 *          : *TbfPtr -> Tx_buf�̃|�C���^
 *          : RbfAdd -> RX_buf�̃f�[�^�i�[�����̐擪�A�h���X
 * Return   : void
 * Caution  : 1. �������ݑO�Ƀu���b�N�P�ʂ̃f�[�^�̏������K�v
 *          : 2. �������݂�128byte�P�ʂł����Ȃ�
 *          : 3. �u���b�N���܂������f�[�^�������݂͖��Ή�
 *          : 4. 0x00040000����0x100�P�ʂ̏������݂����Ή����Ă��Ȃ�
 * Note     : 
 * *****************************************/
void DlWrite(char *TbfPtr, unsigned long RbfAdd, unsigned long Add)
{
	short BufOfs = 0; //�o�b�t�@�I�t�Z�b�g
	short WrtFlg = 0; //�������ݐ����t���O

	short WrtSiz = 0; //�������݃T�C�Y
	short i;
	short RbfSiz = 0; //��M�f�[�^�T�C�Y
	short SftLst[] = {0x10, 0x1, 0x1000, 0x100}; //�f�[�^�V�t�g���X�g
	char *RbfPtr = (char *)RbfAdd; //��M�f�[�^�|�C���^
	short *DatPtr = (short *)&WrtDat[0];
#if 0

	//�ʐM�o�b�t�@���珑�����݃o�b�t�@�ւ̃R�s�[
	BufOfs = (short)(Add & 0x000000FFUL);

	if(BufOfs == 0){
		//�������݃o�b�t�@�̏�����
		memsetW(WrtDat, 0xffff, 128);
	}

	// memcpy_dl((void*)((unsigned long)(&WrtDat[0]) + BufOfs), (void*)(RbfAdd), WrtSiz);
	memcpy_dl((void*)((unsigned long)(&WrtDat[0]) + BufOfs), (void*)(RbfAdd), 0x100);
#else
	//�ʐM�o�b�t�@���珑�����݃o�b�t�@�ւ̃R�s�[
	BufOfs = (short)(Add % 0x100);
	
	//�������݃o�b�t�@�̏�����
	memsetW(WrtDat, 0x0000, 128);

	//��M�f�[�^���擾 (�}�C�i�X(�I�[�o�[�t���[), 512(�z���̍ő�T�C�Y)�ȏ�̏ꍇ��512�Œ�)
	RbfSiz = strlen((void *)RbfAdd) - 1; //������,��-1
	if((RbfSiz < 0) || (512 <= RbfSiz)) 
	{
		RbfSiz = 512;
	}
#if 1
	//��M�o�b�t�@���珑�����݃o�b�t�@�փR�s�[
	for(i=0; i<RbfSiz; i++)
	{
		WrtDat[i / 4] += CvtAscDec(*(RbfPtr + i)) * SftLst[i % 4];
	}
	//��M�o�b�t�@���珑�����݃o�b�t�@�փR�s�[(�]��)
	for(i=RbfSiz; i<512; i++)
	{
	    WrtDat[i / 4] += 0x0F * SftLst[i % 4];
	}
#else
	//��M�o�b�t�@���珑�����݃o�b�t�@�փR�s�[
	for(i=0; i<RbfSiz; i++)
	{
		*(WrtDat + (i / 4)) += CvtAscDec(*(RbfPtr + i)) * SftLst[i % 4];
	}
	//��M�o�b�t�@���珑�����݃o�b�t�@�փR�s�[(�]��)
	for(i=RbfSiz; i<512; i++)
	{
	    *(WrtDat + (i / 4)) += 0x0F * SftLst[i % 4];
	}
#endif

#endif
	//�������݃o�b�t�@����Flash�̈�ւ̏�������
	WrtFlg = FPGA_WRITE((void *)(Add & 0xFFFFFF00UL), WrtDat);

	//�������ݐ���
	if(WrtFlg == 0)
	{
		*(TbfPtr++) = ',';
		*(TbfPtr++) = 'O';
		*(TbfPtr++) = 'K';
		*(TbfPtr++) = ',';
	}
	//�������ݎ��s
	else
	{
		*(TbfPtr++) = ',';
		*(TbfPtr++) = 'N';
		*(TbfPtr++) = 'G';
		*(TbfPtr++) = ',';
	}
}

/*******************************************
 * Function : command_Dl
 * Summary  : FPGA�_�E�����[�h�R�}���h(Dl)
 * Argument : int com_mode -> 0 : �z�X�g
 *                            1 : �����e
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : @Dl000,n,xxxxxxxx,yyyyy...[128byte]
 *          :   n -> 0 : �u���b�N����
 *          :        1 : Flash�̈�̃f�[�^���[�h
 *          :        2 : Flash�̈�̃f�[�^���C�g
 *          :   xxxxxxxx -> �A�h���X
 *          :   yyy... -> �ő�128byte�̃f�[�^
 * *****************************************/
void command_Dl (short com_mode){
	unsigned long Add, TmpAdd;
	short i, ReqTyp;
	char *TbfPtr;

	TbfPtr = (char *)&TX_buf[com_mode][8];

	//value���擾
	value_judge_Hex(com_mode,7,0);
	ReqTyp = (short)value[com_mode];

	//0-2�̂ݎ�t
	if((ReqTyp < 0) || (2 < ReqTyp))
	{
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		//1, 2�̏ꍇ�̓A�h���X�̎w�肪�K�v
	    if(ReqTyp != 0)
	    {
	        i_num[com_mode]++;
	        value_judge_Hex(com_mode, i_num[com_mode], 0);
	        Add = (unsigned long)value[com_mode];
	    }

		//0�̏ꍇ�̓A�h���X�w��s�v
	    if(ReqTyp == 0)
	    {
			ErsAllBlk();
            *(TbfPtr++) = ',';
			*(TbfPtr++) = 'O';
			*(TbfPtr++) = 'K';
			*(TbfPtr++) = ',';
		}
		//�������݃A�h���X�͈�
		else if((Add < 0x00040000) || (0x00100000 < Add))
		{
			end_code[com_mode] = FORMAT_ERROR;
		}
		//�A�h���X��8��
		else if((size[com_mode] != 0x08))
		{
			end_code[com_mode] = FORMAT_ERROR;
		}
		else
		{
			//�ԑ��R�}���h�ɃA�h���X���R�s�[
			*(TbfPtr++) = ',';
			TmpAdd = Add;
			for(i=0; i<8; i++)
			{
				*(TbfPtr++) = (char)(CvtDecAsc(TmpAdd / 0x10000000));
				TmpAdd <<= 4;
			}

			switch (ReqTyp)
			{
			// case 0: //�u���b�N���� (�㕔�Ŏ��{)
			// 	break;
			case 1: //�f�[�^���[�h
				DlRead(TbfPtr, Add);
				break;
			case 2: //�f�[�^���C�g����
				//�������݃f�[�^�i�[�ꏊ��18�ȍ~
				DlWrite(TbfPtr, (unsigned long)(&RX_buf[com_mode][18]), Add);
				break;
			
			default: //���Ȃ��͂������O�̂���
				*(TbfPtr++) = ',';
				*(TbfPtr++) = 'N';
				*(TbfPtr++) = 'G';
				*(TbfPtr++) = ',';
				break;
			}
		}
	}
}
void command_MT(short com_mode)
{
	short pch = ch_no[com_mode] - 1;

	TX_buf[com_mode][6] = ',';
	TX_buf[com_mode][7] = RX_buf[com_mode][5];

	value_judge(com_mode,7,0);
	if(value[com_mode] < 0 || 1 < value[com_mode])
	{
		end_code[com_mode] = FORMAT_ERROR;
	}
	else
	{
		MES[pch].zc_peak_UpdateFlg = value[com_mode];
	}
	
	read_change(com_mode, MES[pch].zc_peak_UpdateFlg, 0, ADD_CMM_BFR_VAL);
}

/*******************************************
 * Function : read_change2
 * Summary  : ���l���w�肵��������TX_buf�Ɋi�[����
 * Argument : 
 * Return   : void
 * Caution  : None
 * Note     : ���l�擪��0���������Ȃ�
 * *****************************************/
void read_change2(short com_mode, long long Val, char point, short pos, short Odr)
{
	short i;
	long long Num = 1;
	for(i = 0; i < Odr - 1; i++)
	{
		Num *= 10;
	}
	for(i = 0; i < Odr; i++)
	{
		read_change(com_mode, (Val / Num), 0, ADD_NO_CMM);
		Val = Val % Num;
		Num /= 10;
	}
}

void command_MZ(short com_mode)
{
	short pch = ch_no[com_mode] - 1;
	short i;
	short Num = 0;

	short Mod = RX_buf[com_mode][7] - 0x30;
	short sPnt = 0;
	short ePnt = 15;
	long Hvl, Lvl;
	if(SVD[pch].ZerCrsUseNum > 15)
	{
		//1�_������30������, 500byte/30~16 �}�[�W��������15
		ePnt = sPnt + 15;
	}
	else
	{
		ePnt = sPnt + 15;
	}
    TX_buf[com_mode][6] = '0';
    TX_buf[com_mode][7] = '0';
	TX_buf[com_mode][8] = ',';
    TX_buf[com_mode][9] = RX_buf[com_mode][7];

	//������0�̎��̒l��ێ�����
	if(Mod == 0)
	{
		MES[pch].ZcLog.Flw = MES[pch].ml_min_now; //����
		MES[pch].ZcLog.Tup = MES_SUB[pch].zc_Tup; //�㗬���ԍ�
		MES[pch].ZcLog.Tdw = MES_SUB[pch].zc_Tdown; //�������ԍ�
		//�[���N���X�_
		for(i = 0; i < ZC_POINT_MAX; i++)
		{
			MES[pch].ZcLog.FowZcd[i][0] = MES[pch].zc_nearzero_point[0][i];
			MES[pch].ZcLog.FowZcd[i][1] = MES[pch].zc_nearzero_data1[0][i];
			MES[pch].ZcLog.FowZcd[i][2] = MES[pch].zc_nearzero_data2[0][i];
			MES[pch].ZcLog.FowZcd[i][3] = MES[pch].zc_nearzero_data3[0][i];
			MES[pch].ZcLog.FowZcd[i][4] = MES[pch].zc_nearzero_data4[0][i];
			MES[pch].ZcLog.RevZcd[i][0] = MES[pch].zc_nearzero_point[1][i];
			MES[pch].ZcLog.RevZcd[i][1] = MES[pch].zc_nearzero_data1[1][i];
			MES[pch].ZcLog.RevZcd[i][2] = MES[pch].zc_nearzero_data2[1][i];
			MES[pch].ZcLog.RevZcd[i][3] = MES[pch].zc_nearzero_data3[1][i];
			MES[pch].ZcLog.RevZcd[i][4] = MES[pch].zc_nearzero_data4[1][i];

			MES[pch].ZcLog.FowClcZcd[i] = MES[pch].FwdClcZerPnt[i];
			MES[pch].ZcLog.RevClcZcd[i] = MES[pch].RevClcZerPnt[i];
		}
	}

	//���[���N���X�_��ǂݏo��
	if(Mod == 0)
	{
		read_change(com_mode, MES[pch].ZcLog.Flw, 2, ADD_CMM_BFR_VAL);
		
		Hvl = (long)(MES[pch].ZcLog.Tup); //������
		Lvl = (long)((MES[pch].ZcLog.Tup - Hvl) * 10000000); //������
		read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
		strcat(TX_buf[com_mode], ".");
		read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);

		Hvl = (long)(MES[pch].ZcLog.Tdw); //������
		Lvl = (long)((MES[pch].ZcLog.Tdw - Hvl) * 10000000); //������
		read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
		strcat(TX_buf[com_mode], ".");
		read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);
	}
	//�㗬���[���N���X�_��ǂݏo��
	else if(Mod == 1)
	{
		for(i = sPnt; i < ePnt; i++)
		{
			read_change(com_mode, MES[pch].ZcLog.FowZcd[i][0], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.FowZcd[i][1], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.FowZcd[i][2], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.FowZcd[i][3], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.FowZcd[i][4], 0, ADD_CMM_BFR_VAL);
		}
	}
	//�������[���N���X�_��ǂݏo��
	else if(Mod == 2)
	{
		for(i = sPnt; i < ePnt; i++)
		{
			read_change(com_mode, MES[pch].ZcLog.RevZcd[i][0], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.RevZcd[i][1], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.RevZcd[i][2], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.RevZcd[i][3], 0, ADD_CMM_BFR_VAL);
			read_change(com_mode, MES[pch].ZcLog.RevZcd[i][4], 0, ADD_CMM_BFR_VAL);
		}
	}
	//�v�Z��[���N���X�_��ǂݏo��
	else if(Mod == 3)
	{
		//�㗬
		for(i = sPnt; i < ePnt; i++)
		{
			Hvl = (long)(MES[pch].ZcLog.FowClcZcd[i]); //������
			Lvl = (long)((MES[pch].ZcLog.FowClcZcd[i] - Hvl) * 10000000); //������
			read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
			strcat(TX_buf[com_mode], ".");
			read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);
		}
		//����
		for(i = sPnt; i < ePnt; i++)
		{
			Hvl = (long)(MES[pch].ZcLog.RevClcZcd[i]); //������
			Lvl = (long)((MES[pch].ZcLog.RevClcZcd[i] - Hvl) * 10000000); //������
			read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
			strcat(TX_buf[com_mode], ".");
			read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);
		}
	}
	else
	{
		end_code[com_mode] = FORMAT_ERROR;
	}

	
}

/* �������f�o�C�X�V���A���i���o�[�Ǐo��(MS) */
 void command_MS (short com_mode){

	long value;

	TX_buf[com_mode][6] = ',';
	TX_buf[com_mode][7] = RX_buf[com_mode][5];

	value = SVD[ch_no[com_mode] -1].m_serial[0];	//�������f�o�C�X�V���A���i���o�[
	read_change(com_mode,value,0,ADD_CMM_BFR_VAL);
	
	value = SVD[ch_no[com_mode] -1].m_serial[1];	//�������f�o�C�X�V���A���i���o�[
	read_change(com_mode,value,0,ADD_CMM_BFR_VAL);

	value = SVD[ch_no[com_mode] -1].m_serial[2];	//�������f�o�C�X�V���A���i���o�[
	read_change(com_mode,value,0,ADD_CMM_BFR_VAL);
} 
