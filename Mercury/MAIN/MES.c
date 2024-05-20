/***********************************************/
/* File Name : MES.c	    	         									   */
/*	Summary   : メッセージ通信処理                   */
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
/*		内部宣言								*/
/************************************************/
char 	comma[5] = ",";	// ','代入用
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
/*		外部宣言								*/
/************************************************/
extern char		RX_buf[MSG_NUM][MSG_MAX];	/*受信用配列*/
extern char		TX_buf[MSG_NUM][MSG_MAX];	/*送信用配列*/
extern short		ch_no[MSG_NUM];
extern short		flow_check[MSG_NUM];
extern short		i_num[MSG_NUM];			// 数列番号用
extern short		size[MSG_NUM];			// 入力値データ長判定用
extern short		digit_check[MSG_NUM];		// 整数or小数点桁数 判定用
extern short		end_code[MSG_NUM];		// エンドコード処理用
extern long long	value[MSG_NUM];			// 入力値判定用
extern short		com_type;
extern unsigned long	cmi_count;
extern short		led_flash;
extern short		mes_err[7];
extern short 	wave_hight[2][6];
extern short    OWwrite[6];

extern unsigned short AD_BASE;


/************************************************/
/*		内部関数宣言							*/
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
void command_Dl (short cm); //FPGAダウンロードコマンド

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
/*		外部関数宣言							*/
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

/* 積算値読出し(OV) */			
void command_OV (short com_mode){

	unsigned long long value;
	unsigned short value_high;
	unsigned short value_low;
	unsigned short err_st;
	char 	add[20] = {0};

	strcat(TX_buf[com_mode],comma);
	err_st = MAIN[ch_no[com_mode] -1].com_err_status;			//エラーステータス
	if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
	if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
		if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
			&& MAIN[ch_no[com_mode] -1].led_err_status == 0){		//エラー発生していない場合
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
		}
	}else{			//積算監視エラーが発生
		if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
			MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
		}
	}

	add[0] = err_st / 10 + 0x30;
	add[1] = err_st % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));				//最後尾に読込値追加
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	value = MES[ch_no[com_mode] -1].addit_buff.DWORD / 10000;		//積算値x10000000 → x1000
	read_change(com_mode,value,3,2);

}

/* 積算値読出し・積算中常時更新(ov) */			
void command_ov (short com_mode){

	unsigned long long value;
	unsigned short value_high;
	unsigned short value_low;
	unsigned short err_st;
	char 	add[20] = {0};

	strcat(TX_buf[com_mode],comma);
	err_st = MAIN[ch_no[com_mode] -1].com_err_status;			//エラーステータス
	if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
	if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
		if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
			&& MAIN[ch_no[com_mode] -1].led_err_status == 0){		//エラー発生していない場合
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
		}
	}else{			//積算監視エラーが発生
		if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
			MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
			MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
		}
	}

	add[0] = err_st / 10 + 0x30;
	add[1] = err_st % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));				//最後尾に読込値追加
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	value = MES[ch_no[com_mode] -1].addit_buff_ov.UINT64 / 10000;		//積算値x10000000 → x1000
	read_change(com_mode,value,3,2);

}

/* 瞬時流量1点読出し(OE) */
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
	ch[0] = RX_buf[com_mode][7];					//CH数判定
	ch_z = ch[0] - 0x30;
	TX_buf[com_mode][8] = RX_buf[com_mode][6];
	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];			//CH番号判定
		ch_number = ch[0] - 0x30;
		TX_buf[com_mode][10 + 9*m] = RX_buf[com_mode][8 + 2*m];
		TX_buf[com_mode][11 + 9*m] = RX_buf[com_mode][9 + 2*m];
		
		err_st = MAIN[ch_number -1].com_err_status;		//エラーステータス
		if(err_st != 0){
			mes_err_cnt[ch_no[com_mode]]++;
			mes_err_cnt[ch_number]++;
		}
		if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
			if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
				&& MAIN[ch_number -1].led_err_status == 0){		//エラー発生していない場合
				MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
			}
		}else{			//積算監視エラーが発生
			if(err_zero_status((short)err_st) != B_YES){			//ゼロ点調整エラー以外で、
				MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				MAIN[ch_number -1].led_err_status = (short)0;
			}
		}		

		TX_buf[com_mode][12 + 9*m] = err_st / 10 + 0x30;
		TX_buf[com_mode][13 + 9*m] = err_st % 10 + 0x30;	

		flow = MES[ch_number -1].ml_min_now;	//瞬時流量値

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

/* 瞬時流量10点読出し(OQ) */
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
	ch[0] = RX_buf[com_mode][7];						//CH数判定
	ch_z = ch[0] - 0x30;
	TX_buf[com_mode][8] = RX_buf[com_mode][6];
	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];				//CH番号判定
		ch_number = ch[0] - 0x30;
		TX_buf[com_mode][10 + 72*m] = RX_buf[com_mode][8 + 2*m];
		TX_buf[com_mode][11 + 72*m] = RX_buf[com_mode][9 + 2*m];
		for (l = 0; l < 10; l++){
			err_st = 0;
			if(l >= MES[ch_number -1].past_flow_cnt){			//10点データバッファに10点なし
				err_st = ERR_10POINT;
			}
			//優先順位が10点データ無効より高いエラーがある場合
			if((MAIN[ch_number -1].com_err_status != 0) &&
				(err_inf[MAIN[ch_number -1].com_err_status].err_priority < err_inf[ERR_10POINT].err_priority)){
				err_st = MAIN[ch_number -1].com_err_status;		//エラーステータス
			}
			if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			}
			if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
				if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
					&& MAIN[ch_number -1].led_err_status == 0){		//エラー発生していない場合
					MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				}
			}else{			//積算監視エラーが発生
				if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
					MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
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
		if(MES[ch_number -1].past_flow_cnt < 10){					//10点データバッファに10点なし
			MES[ch_number -1].err_status |= ERR_JUDGE_10POINT;		//10点データ無効エラーセット
		}
		MES[ch_number -1].past_flow_cnt = 0;						//10点データ保存カウンタクリア
	}
	for (m = 0; m < 7; m++){
		if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
	}
	ch[0] = 0;
}

/* 瞬時流量10点読出し・ローカットオフ(oq) */
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
	ch[0] = RX_buf[com_mode][7];						//CH数判定
	ch_z = ch[0] - 0x30;
	TX_buf[com_mode][8] = RX_buf[com_mode][6];
	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];				//CH番号判定
		ch_number = ch[0] - 0x30;
		TX_buf[com_mode][10 + 72*m] = RX_buf[com_mode][8 + 2*m];
		TX_buf[com_mode][11 + 72*m] = RX_buf[com_mode][9 + 2*m];
		for (l = 0; l < 10; l++){
			err_st = 0;
			if(l >= MES[ch_number -1].past_flow_cnt){			//10点データバッファに10点なし
				err_st = ERR_10POINT;
			}
			//優先順位が10点データ無効より高いエラーがある場合
			if((MAIN[ch_number -1].com_err_status != 0) &&
				(err_inf[MAIN[ch_number -1].com_err_status].err_priority < err_inf[ERR_10POINT].err_priority)){
				err_st = MAIN[ch_number -1].com_err_status;		//エラーステータス
			}
			if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			}
			if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
				if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
					&& MAIN[ch_number -1].led_err_status == 0){		//エラー発生していない場合
					MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				}
			}else{			//積算監視エラーが発生
				if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
					MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
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
		if(MES[ch_number -1].past_flow_cnt < 10){					//10点データバッファに10点なし
			MES[ch_number -1].err_status |= ERR_JUDGE_10POINT;		//10点データ無効エラーセット
		}
		MES[ch_number -1].past_flow_cnt = 0;						//10点データ保存カウンタクリア
	}
	for (m = 0; m < 7; m++){
		if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
	}
	ch[0] = 0;
}

/* ステータス読出し(SFC9000対応)(RS) */
void command_RS (short com_mode){

	short m;
	short ch_z;
	short ch_number;
	unsigned short err_st;
	short mes_err_cnt[7] = {0};
	char ch[1] = {0};
	char add[20] = {0};
	
/* シングルコマンド */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
	/* 動作ステータス */
		add[0] = MAIN[ch_no[com_mode] -1].com_act_status / 10 + 0x30;
		add[1] = MAIN[ch_no[com_mode] -1].com_act_status % 10 + 0x30;
		strncat(TX_buf[com_mode],add,sizeof(add));
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* エラーステータス */
		add[0] = MAIN[ch_no[com_mode] -1].com_err_status / 10 + 0x30;
		add[1] = MAIN[ch_no[com_mode] -1].com_err_status % 10 + 0x30;
		err_st = MAIN[ch_no[com_mode] -1].com_err_status;
		if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
		if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
			if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
				&& MAIN[ch_no[com_mode] -1].led_err_status == 0){		//エラー発生していない場合
				MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
			}
		}else{			//積算監視エラーが発生
			if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
				MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
			}
		}
		strncat(TX_buf[com_mode],add,sizeof(add));
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* 再起動ステータス */
		err_st = reset_factor;
		read_change0X_2digit_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* CUnetエラーステータス */
		err_st = send_err_status;
		read_change0X_2digit_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	/* CUnet MCARE/LCARE情報 */
		err_st = cunet_mcare;
		read_change0X_status(com_mode,err_st);
	}
/* マルチコマンド */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];					//CH数判定
		ch_z = ch[0] - 0x30;
		ch[0] = 0;				
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH番号判定
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* 動作ステータス */
			add[0] = MAIN[ch_number -1].com_act_status / 10 + 0x30;
			add[1] = MAIN[ch_number -1].com_act_status % 10 + 0x30;
			strncat(TX_buf[com_mode],add,sizeof(add));
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* エラーステータス */
			add[0] = MAIN[ch_number -1].com_err_status / 10 + 0x30;
			add[1] = MAIN[ch_number -1].com_err_status % 10 + 0x30;
			err_st = MAIN[ch_number -1].com_err_status;
			if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			}
			if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
				if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
					&& MAIN[ch_number -1].led_err_status == 0){		//エラー発生していない場合
					MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				}
			}else{			//積算監視エラーが発生
				if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
					MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
					MAIN[ch_number -1].led_err_status = (short)0;
				}
			}
			strncat(TX_buf[com_mode],add,sizeof(add));
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* 再起動ステータス */
			err_st = reset_factor;
			read_change0X_2digit_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* CUnetエラーステータス */
			err_st = send_err_status;
			read_change0X_2digit_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));
		/* CUnet MCARE/LCARE情報 */
			err_st = cunet_mcare;
			read_change0X_status(com_mode,err_st);
		}
		for (m = 0; m < 7; m++){
			if(mes_err_cnt[m] == 0)	mes_err[m] = 0;
		}
	}

}

/* ステータス読出し(Rs) */
 void command_Rs (short com_mode){

	 short m;
	 short ch_z;
	 short ch_number;
	 unsigned short err_st;
	 short mes_err_cnt[7] = {0};
	 char ch[1] = {0};
	 char add[20] = {0};
	 
 /* シングルコマンド */
	 if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
	 /* 動作ステータス */
		 add[0] = MAIN[ch_no[com_mode] -1].com_act_status / 10 + 0x30;
		 add[1] = MAIN[ch_no[com_mode] -1].com_act_status % 10 + 0x30;
		 strncat(TX_buf[com_mode],add,sizeof(add));
		 strncat(TX_buf[com_mode],comma,sizeof(comma));
	 /* エラーステータス */
		 add[0] = MAIN[ch_no[com_mode] -1].com_err_status / 10 + 0x30;
		 add[1] = MAIN[ch_no[com_mode] -1].com_err_status % 10 + 0x30;
		 err_st = MAIN[ch_no[com_mode] -1].com_err_status;
		 if(err_st == 0)	mes_err[ch_no[com_mode]] = 0;
		 if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
			 if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
				 && MAIN[ch_no[com_mode] -1].led_err_status == 0){		//エラー発生していない場合
				 MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
			 }
		 }else{			//積算監視エラーが発生
			 if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
				 MAIN[ch_no[com_mode] -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				 MAIN[ch_no[com_mode] -1].led_err_status = (short)0;
			 }
		 }
		 strncat(TX_buf[com_mode],add,sizeof(add));
	 }
 /* マルチコマンド */
	 else if (ch_no[com_mode] == 0){
		 ch[0] = RX_buf[com_mode][7];					//CH数判定
		 ch_z = ch[0] - 0x30;
		 ch[0] = 0;				
		 for (m = 0; m <= ch_z; m++){
			 TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			 TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		 }
		 for (m = 0; m < ch_z; m++){
			 ch[0] = RX_buf[com_mode][9 + 2*m];			//CH番号判定
			 ch_number = ch[0] - 0x30;
			 strncat(TX_buf[com_mode],comma,sizeof(comma));
		 /* 動作ステータス */
			 add[0] = MAIN[ch_number -1].com_act_status / 10 + 0x30;
			 add[1] = MAIN[ch_number -1].com_act_status % 10 + 0x30;
			 strncat(TX_buf[com_mode],add,sizeof(add));
			 strncat(TX_buf[com_mode],comma,sizeof(comma));
		 /* エラーステータス */
			 add[0] = MAIN[ch_number -1].com_err_status / 10 + 0x30;
			 add[1] = MAIN[ch_number -1].com_err_status % 10 + 0x30;
			 err_st = MAIN[ch_number -1].com_err_status;
			 if(err_st != 0){
				mes_err_cnt[ch_no[com_mode]]++;
				mes_err_cnt[ch_number]++;
			 }
			 if(err_total_status((short)err_st) != B_YES){			//積算監視エラー以外
				 if(err_zero_status((short)err_st) != B_YES			//ゼロ点調整エラー以外で、
					 && MAIN[ch_number -1].led_err_status == 0){		//エラー発生していない場合
					 MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
				 }
			 }else{			//積算監視エラーが発生
				 if(err_zero_status((short)err_st) != B_YES){						//ゼロ点調整エラー以外で、
					 MAIN[ch_number -1].com_err_status = (short)0;		//エラーステータスを読込んだらクリア
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

/* 口径読出し(Rg) */
void command_Rg (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].sensor_size;
	read_change(com_mode,value,0,0);
	
}

/* フルスケール読出し(Rr) */
void command_Rr (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].max_flow;
	read_change(com_mode,value,0,0);
	
}

/* Ｋファクタ読出し(Rk) */
void command_Rk (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].k_factor;
	read_change(com_mode,value,3,0);

}

/* ダンピング読出し(Rd) */
void command_Rd (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].damp;
	read_change(com_mode,value,1,0);

}

/* ローカット読出し(Rl) */
void command_Rl (short com_mode){

	short m;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	short value;
	
/* シングルコマンド */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		value = SVD[ch_no[com_mode] -1].low_cut;
		read_change(com_mode,value,1,0);
	}
/* マルチコマンド */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];			//CH数判定
		ch_z = ch[0] - 0x30;
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];	//CH番号判定
			ch_number = ch[0] - 0x30;
			value = SVD[ch_number -1].low_cut;
			read_change(com_mode,value,1,1);
		}
	}
	ch[0] = 0;

}

/* バーンアウト読出し(Rb) */
void command_Rb (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].burnout;		//種別
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].burnout_value;		//入力値
	read_change(com_mode,value,0,0);

}

/* 動粘度読出し(Rv) */
void command_Rv (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].viscos;
	read_change(com_mode,value,2,0);

}

/* エラーホールドタイム読出し(Rh) */
void command_Rh (short com_mode){

	short value;
	
	value = SVD[ch_no[com_mode] -1].err_hold_time;
	read_change(com_mode,value,0,0);

}

/* ユーザーリニアライズ読出し(Ru) */
void command_Ru (short com_mode){

	short l;
	unsigned short *pt;
	long value;
	unsigned short value_high;
	unsigned short value_low;

	short	point;

	point = (short)(SVD[ch_no[com_mode]-1].uslnr_num >> 8) & 0x00FF;	/*設定点数取得*/
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

/* メーカーリニアライズ読出し(Rm) */
void command_Rm (short com_mode){

	short l;
	unsigned short *pt;
	long value;
	unsigned short value_high;
	unsigned short value_low;

	short	point;

	point = (short)(SVD[ch_no[com_mode]-1].mklnr_num >> 8) & 0x00FF;	/*設定点数取得*/
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

/* 逆流判定値読出し(RR) */
void command_RR (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].reverse_level;
	read_change(com_mode,value,0,2);	
	
	value = SVD[ch_no[com_mode] -1].reverse_time;
	read_change(com_mode,value,1,0);

}

/* ゼロ点調整状態読出し(Rz) */
void command_Rz (short com_mode){

	short m;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	char add[20] = {0};

/* シングルコマンド */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
	/* 動作ステータス */
		add[0] = MAIN[ch_no[com_mode] -1].com_act_status / 10 + 0x30;
		add[1] = MAIN[ch_no[com_mode] -1].com_act_status % 10 + 0x30;
		strncat(TX_buf[com_mode],add,sizeof(add));					//最後尾に読込値追加
	}
/* マルチコマンド */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];					//CH数判定
		ch_z = ch[0] - 0x30;
		ch[0] = 0;				
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH番号判定
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma)); 
	/* 動作ステータス */
			add[0] = MAIN[ch_number -1].com_act_status / 10 + 0x30;
			add[1] = MAIN[ch_number -1].com_act_status % 10 + 0x30;
			strncat(TX_buf[com_mode],add,sizeof(add));					//最後尾に読込値追加
		}
	}

}			
			
/* バージョン値読出し(SFC9000対応)(RF) */
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
	
	pt = &SVD[ch_no[com_mode] -1].c_serial[0];			//シリアルナンバー		
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

/* バージョン値読出し(RV) */
 void command_RV (short com_mode){

	 unsigned long value;
	
	 value = SVD[ch_no[com_mode] -1].soft_ver;		//software version
	 read_change0X_versionRV(com_mode,value);
	 strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	 value = SVD[ch_no[com_mode] -1].hard_ver;		//hardware version
	 read_change0X_versionRV(com_mode,value);

 }

/* ログデータ読出し(RG)(SFC9000対応) */
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
	ch[0] = RX_buf[com_mode][7];					//ログ履歴番号判定
	ch_z = ch[0] - 0x30;
	if(ch_z == 0){ch_z = 10;}
	ch[0] = 0;

	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = SVD[ch_no[com_mode] -1].pwon_count;	//現在電源ON回数
	read_change(com_mode,value,0,2);
	
	value = cmi_count;					//現在エラータイマー
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_pwon_count;	//発生時電源ON回数
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_time;		//発生時エラータイマー
	read_change(com_mode,value,0,2);
	
	add[0] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code / 10 + 0x30;		//ErrorLogCode
	add[1] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));					//最後尾に読込値追加
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_quantity;		//流量
	read_change(com_mode,value,2,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_velocity;		//流速
	read_change(com_mode,value,3,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].sound_speed;		//音速
	read_change(com_mode,value,0,2);
	
	addit = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].total_count.DWORD / 10000;	//積算値
	read_change(com_mode,addit,3,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].wave_max;			//受波波形最大値
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].wave_min;			//受波波形最小値
	read_change(com_mode,value,0,2);
	
	diff = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].dt * 32;		//伝搬時間差
	if (diff > 99999999) {diff = 99999999;}
	else if (diff < -99999999) {diff = -99999999;}
	read_change(com_mode,diff,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].correlate;			//相関値幅
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].zero_offset;		//ゼロ点オフセット
	read_change(com_mode,value,0,2);

	err_st = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].status;				//status
	read_change0X_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].fifo_position;		//FIFO受波波形検出位置
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_up;			//Gain 1st stage
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_down;		//Gain 2nd stage
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].fifo_ch;			//FIFO CH
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].p1_p2;				//受波の差(P1-P2)
	read_change(com_mode,value,0,2);

	err_st = reset_factor;				//再起動ステータス
	read_change0X_2digit_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	err_st =  LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].mail_err_status;		//CUnetエラーステータス
	read_change0X_2digit_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	err_st =  cunet_mcare;				//CUnet MCARE/LCARE情報
	read_change0X_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	err_st = cunet_mfr1;				//CUnet MFR情報
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

/* ログデータ読出し(RL) */
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
	ch[0] = RX_buf[com_mode][7];					//ログ履歴番号判定
	ch_z = ch[0] - 0x30;
	if(ch_z == 0){ch_z = 10;}
	ch[0] = 0;

	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = SVD[ch_no[com_mode] -1].pwon_count;	//現在電源ON回数
	read_change(com_mode,value,0,2);
	
	value = cmi_count;					//現在エラータイマー
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_pwon_count;	//発生時電源ON回数
	read_change(com_mode,value,0,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_time;	//発生時エラータイマー
	read_change(com_mode,value,0,2);
	
	add[0] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code / 10 + 0x30;		//ErrorLogCode
	add[1] = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].err_code % 10 + 0x30;
	strncat(TX_buf[com_mode],add,sizeof(add));					//最後尾に読込値追加
	strncat(TX_buf[com_mode],comma,sizeof(comma));
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_quantity;		//流量
	read_change(com_mode,value,2,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].flow_velocity;		//流速
	read_change(com_mode,value,3,2);
	
	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].sound_speed * 100;	//音速
	read_change(com_mode,value,2,2);
	
	addit = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].total_count.DWORD / 1000000;	//積算値
	read_change(com_mode,addit,1,2);
	
	value = 0;		//UP伝搬時間[ps]（該当データなし）
	read_change(com_mode,value,0,2);

	value = 0;	//DN伝搬時間[ps]（該当データなし）
	read_change(com_mode,value,0,2);
	
	diff = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].dt * 32;		//伝搬時間差[ps]
	if (diff > 99999999) {diff = 99999999;}
	else if (diff < -99999999) {diff = -99999999;}
	read_change(com_mode,diff,0,2);

	value = 0;		//UP無駄時間[ps]（該当データなし）
	read_change(com_mode,value,0,2);

	value = 0;		//DN無駄時間[ps]（該当データなし）
	read_change(com_mode,value,0,2);

	err_st = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].status;		//status
	read_change0X_status(com_mode,err_st);
	strncat(TX_buf[com_mode],comma,sizeof(comma));

	value = 0;	//Window Position[us]（該当データなし）
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_up;	//Gain 1st stage
	read_change(com_mode,value,0,2);

	value = LOG_DETAIL[ch_no[com_mode] -1][ch_z -1].gain_down;	//Gain 2nd stage
	read_change(com_mode,value,0,2);

	value = 0;	//Peak[mV]（該当データなし）
	read_change(com_mode,value,0,2);

	value = 0;	//Threshold[mV]（該当データなし）
	read_change(com_mode,value,0,0);
	for (l = 0; l < 40; l++){
		value = 0;
		read_change(com_mode,value,0,1);
	}

}

/* ゼロ調整データ読出し(SFC9000対応)(RD) */
void command_RD (short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	long value;
	long long diff;

	ch[0] = RX_buf[com_mode][7];                		//CH数判定
	ch_z = ch[0] - 0x30;
	ch[0] = 0;			
	for (m = 0; m <= ch_z; m++){
		TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
		TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
	}
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];		//CH番号判定
		ch_number = ch[0] - 0x30;
		
		value = SVD[ch_number -1].viscos;								//動粘度
		read_change(com_mode,value,2,1);
		
		value = SVD[ch_number -1].zero_wave_max;		//受波波形最大値
		read_change(com_mode,value,0,1);
	
		value = SVD[ch_number -1].zero_wave_min;		//受波波形最小値
		read_change(com_mode,value,0,1);

		diff = SVD[ch_number -1].zero_delta_ts.DWORD * 32;	//伝搬時間差
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,1);

		value = SVD[ch_number -1].zero_fifo_pos;		//FIFO受波波形検出位置
		read_change(com_mode,value,0,1);
		
		value = SVD[ch_number -1].zero_gain_1st;		//Gain1
		read_change(com_mode,value,0,1);

		value = SVD[ch_number -1].zero_gain_2nd;		//Gain2
		read_change(com_mode,value,0,1);

		value = SVD[ch_number -1].zero_p1p2;		//受波の差(P1-P2)
		read_change(com_mode,value,0,1);		
	}
	
}

/* ゼロ調整データ読出し(RZ) */
 void command_RZ (short com_mode){

	short m;
	short l;
	short ch_z;
	short ch_number;
	char ch[1] = {0};
	long value;
	long long diff;

	ch[0] = RX_buf[com_mode][7];                		//CH数判定
	ch_z = ch[0] - 0x30;
	ch[0] = 0;			
	for (m = 0; m <= ch_z; m++){
		TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
		TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
	}
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];		//CH番号判定
		ch_number = ch[0] - 0x30;
		
		value = SVD[ch_number -1].viscos;				//動粘度
		read_change(com_mode,value,2,1);
		
		value = 0;		//Tu（該当データなし）
		read_change(com_mode,value,0,1);
	
		value = 0;		//Td（該当データなし）
		read_change(com_mode,value,0,1);

		diff = SVD[ch_number -1].zero_delta_ts.DWORD * 32;		//Dt
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,1);

		value = 0;		//Window Position（該当データなし）
		read_change(com_mode,value,0,1);
		
		value = SVD[ch_number -1].zero_gain_1st;		//Gain1
		read_change(com_mode,value,0,1);

		value = SVD[ch_number -1].zero_gain_2nd;		//Gain2
		read_change(com_mode,value,0,1);

		value = 0;		//Threshold（該当データなし）
		read_change(com_mode,value,0,1);
	}

 }
/* 波形異常判定値設定読出し(RE) */
 void command_RE (short com_mode){

	long value;

	value = SVD[ch_no[com_mode] -1].wave_vth;			//エンプティセンサ判定閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].correlate_level;		//演算異常判定閾値
	read_change(com_mode,value,1,2);

	value = SVD[ch_no[com_mode] -1].correlate_time;			//演算異常判定回数
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].balance_level;			//波形アンバランス閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].saturation_level;		//AGC不能判定閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].attenuate_level;		//波形減衰判定閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_wave_vth;			//エンプティセンサ警告閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_gain_level;	//アンプゲイン警告閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_gain_count;	//アンプゲイン警告ｶｳﾝﾄ
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].alm_hold_time;		//警告判定時間
	read_change(com_mode,value,2,0);
}

/* アッテネータゲイン値読出し(RA) */
 void command_RA (short com_mode){

	long value;

//	value = SVD[ch_no[com_mode] -1].atn_gain;
	value = 0;
	read_change(com_mode,value,0,0);
		
}

/* 薬液リニアライズモード読出し(Ry) */
 void command_Ry (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].LL_enable;
	read_change(com_mode,value,0,2);	
	
	value = SVD[ch_no[com_mode] -1].LL_kind;
	read_change(com_mode,value,0,0);
} 

/* 音速読出し(RM) */
 void command_RM (short com_mode){

	short value;

	value = MES[ch_no[com_mode] -1].sound_vel_f;				//音速[m/s]
	read_change(com_mode,value,0,0);
}

/* フィルタ設定読出し(Rf) */
void command_Rf (short com_mode){
	
	short value;

	value = SVD[ch_no[com_mode] -1].filter_mode;	//フィルタモード
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].filter_avg;		//移動平均数
	read_change(com_mode,value,0,0);
}

/* 積算目標値読出し(Rt) */
void command_Rt (short com_mode){

	long value;
	
	value = SVD[ch_no[com_mode] -1].target_total.INT32;
	read_change(com_mode,value,2,0);
}

/* 積算オフセット値読出し(RT) */
void command_RT (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].total_offset_enable;		//ON/OFF
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].total_offset_value;		//入力値
	read_change(com_mode,value,3,0);
}

/* メーカー設定読出し(R1) */
 void command_R1 (short com_mode){

	 short l;
	 short value;
	 unsigned short *pt;
	 short sn_version;
	 short sn_ver_cal;
	 char add[20] = {0};

	 value = 0;		//補正10%（該当データなし）
	 read_change(com_mode,value,2,2);

	 value = 0;		//補正25%（該当データなし）
	 read_change(com_mode,value,2,2);

	 value = 0;		//補正50%（該当データなし）
	 read_change(com_mode,value,2,2);

	 value = 0;		//補正75%（該当データなし）
	 read_change(com_mode,value,2,2);

	 value = 0;		//補正100%（該当データなし）
	 read_change(com_mode,value,2,2);

	 pt = &SVD[ch_no[com_mode] -1].c_serial[0];			//シリアルナンバー
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

/* デバッグモード設定読出し(R2) */
 void command_R2 (short com_mode){

	 short value;

	 value = MES[ch_no[com_mode] -1].test_enable;				//流量出力テスト
	 read_change(com_mode,value,0,2);

	 value = MES[ch_no[com_mode] -1].test_flow;					//流量出力テスト値
	 read_change(com_mode,value,2,2);

	 value = MES[ch_no[com_mode] -1].test_port_out;				//出力ポートテスト
	 read_change(com_mode,value,0,2);

	 value = MES[ch_no[com_mode] -1].test_port_in;				//入力ポート状態
	 read_change(com_mode,value,0,0);
		
 }

/*******************************************
 * Function : command_R3
 * Summary  : ゼロクロス測定データ読み出し
 * Argument : 
 * Return   : 
 * Caution  : None
 * Note     : 差分相関測定データ読み出しはR7
 * *****************************************/
 void command_R3 (short com_mode){

	short l;
	long value;
	long long diff;
	unsigned long long addit;
	short pch = ch_no[com_mode] - 1;
	float TimDif;

	if(RX_buf[com_mode][7] != '0'){		//測定時データ
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//1. 流量[mL/min]
		value = MES[ch_no[com_mode] -1].ml_min_now;
		read_change(com_mode,value,2,2);

		//2. 流速[m/s]
		value = MES[ch_no[com_mode] -1].flow_vel_c / 100;
		read_change(com_mode,value,3,2);

		//3. 音速[m/s]
		value = MES[ch_no[com_mode] -1].sound_vel_f * 100;
		read_change(com_mode,value,2,2);
		
		//4. 積算値[mL/min]
		addit = MES[ch_no[com_mode] -1].addit_buff.DWORD / 10000;
		read_change(com_mode,addit,3,2);
		
		//5. UP伝搬時間[us]
		TimDif = GetTimDif(pch, 0);
		read_change(com_mode, TimDif * 1000.0, 3, ADD_CMM_AFR_VAL);

		//6. DN伝搬時間[us]
		TimDif = GetTimDif(pch, 1);
		read_change(com_mode, TimDif * 1000.0, 3, ADD_CMM_AFR_VAL);

		//7. 伝搬時間差[ps]
		diff = MES[ch_no[com_mode] -1].delta_ts_zero * 32;
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);

		//8. UP無駄時間[ps]
		value = MES[pch].FwdSurplsTim;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//9. DN無駄時間[ps]
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

		//12. Gain 1st stage(未使用)
		value = get_attenuator_gain(ch_no[com_mode]-1);
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//13. Gain 2nd stage
		value = MES[ch_no[com_mode] -1].amp_gain_for;
		read_change(com_mode,value,0,2);

		//14. Peak[mV] (おそらくアンプ増幅後は512mVが最大)
		// value = 512 * (MES[pch].rev_max_data - AD_BASE_UNIT) / 4095;
        value = 512 * (MES[pch].rev_max_data - AD_BASE_UNIT) / AD_MAX_UNIT;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);

		//15. 打ち込み周波数(014Eでは打ち込み周波数を表示)
		value = SVD[pch].drive_freq;				
		read_change(com_mode,value, 0, ADD_NO_CMM);

		//16-25. 上流波形ピーク位置
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].FwdWavMaxPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].FwdWavMinPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
		}

		//26-35. 上流波形ピーク値
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].FwdWavMaxPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].FwdWavMinPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//36-45. 下流波形ピーク位置
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].RevWavMaxPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].RevWavMinPekPosLst[l];
			read_change(com_mode, value, 0, ADD_CMM_BFR_VAL);
		}

		//46-55. 上流波形ピーク位置
		for (l = 0; l < WAV_PEK_NUM / 2; l++){
			value = MES[pch].RevWavMaxPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
			value = MES[pch].RevWavMinPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}
	}else{				//ゼロ調整時データ
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//1. 流量[mL/min]
		value = SVD[ch_no[com_mode] -1].zero_flow_qat.DWORD;
		read_change(com_mode,value,2,2);

		//2. 流速[m/s]
		value = SVD[ch_no[com_mode] -1].zero_flow_vel.DWORD / 100;
		read_change(com_mode,value,3,2);

		//3. 音速[m/s]
		value = SVD[ch_no[com_mode] -1].zero_sound_spd * 100;
		read_change(com_mode,value,2,2);
		
		//4. 積算値[mL/min]
		addit = SVD[ch_no[com_mode] -1].zero_addit.DWORD / 10000;
		read_change(com_mode,addit,3,2);
	
		//5. UP伝搬時間[ps]
		value = SVD[pch].zero_FwdTimDif.DWORD;
		read_change(com_mode,value, 3, ADD_CMM_AFR_VAL);
	
		//6. DN伝搬時間[ps]
		value = SVD[pch].zero_RevTimDif.DWORD;
		read_change(com_mode,value, 3, ADD_CMM_AFR_VAL);

		//7. 伝搬時間差[ps]
		diff = SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD * 32;
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);
	
		//8. UP無駄時間[ps]
		value = SVD[pch].zero_FwdSurplsTim;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);
	
		//9. DN無駄時間[ps]
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

		//12. Gain 1st stage(未使用)
		value = SVD[ch_no[com_mode] -1].zero_gain_1st;
		read_change(com_mode,value,0,2);

		//13. Gain 2nd stage
		value = SVD[ch_no[com_mode] -1].zero_gain_2nd;
		read_change(com_mode,value,0,2);
	
		//14. Peak[mV]
		// value = 512 * (SVD[pch].zero_wave_max - AD_BASE_UNIT) / 4095;
        value = 512 * (SVD[pch].zero_wave_max - AD_BASE_UNIT) / AD_MAX_UNIT;
		read_change(com_mode,value, 0, ADD_CMM_AFR_VAL);
	
		//15. 打ち込み周波数(014Eでは打ち込み周波数を表示)
		value = SVD[pch].zero_drive_freq;
		read_change(com_mode,value, 0, ADD_NO_CMM);

		//16-25. 上流波形ピーク位置
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_FwdWavPekPosLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//26-35. 上流波形ピーク値
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_FwdWavPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//36-45. 下流波形ピーク位置
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_RevWavPekPosLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}

		//46-55. 上流波形ピーク位置
		for (l = 0; l < WAV_PEK_NUM; l++){
			value = SVD[pch].zero_RevWavPekValLst[l];
			read_change(com_mode,value, 0, ADD_CMM_BFR_VAL);
		}
	}
 }
#if defined(MESWAVEXP)

/************************************************************
 * Fucntion : WavInBuff
 * Summary  : 波形データを送信バッファに入れる
 * Argument : com_mode -> 通信モード
 * Return   : void
 * Caution  : なし
 * Note     : command_R4, command_R8のコードを移植
 *            ただし下記分岐は削除
 *            if((MES[ch_no[com_mode]-1].err_status & ERR_JUDGE_EMPTY) != 0){	//Empty Sensor時 波形データ0
 * 	              ofs = 3;
 *            }
 *            理由 : ERR_FLOW_COUNT に ERR_JUDGE_EMPTY が含まれる
************************************************************/
void WavInBuff(short com_mode){
	short ofs;
	short l;
	short value;
	
	if(MES[ch_no[com_mode]-1].wave_monitor_mode == 0 ||				//常に更新設定
		(MES[ch_no[com_mode]-1].err_status & ERR_FLOW_CONT) == 0){	//流量エラー無し
			
		if(RX_buf[com_mode][8] == 0x2c){		// 0～2はFWDのデータ
			ofs = RX_buf[com_mode][7] - '0';
			if(ofs == 0) {
				for(l = 0; l < MESWAVSIZ + 100; l++){
					fow_wave_data[l] = MES[ch_no[com_mode]-1].fow_data[l];
				}
			}
		}else{									// 10～12はREVのデータ
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
			if(RX_buf[com_mode][8] == 0x2c){						//FWDデータ
//				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)/4;	//受波波形データ
				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
			}else{													//REVデータ
//				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)/4;	//受波波形データ
				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
			}
			read_change(com_mode,value,0,1);
		}
	}else{
		for (l = 0; l < 100; l++){
			value = 0;								//波形データ0：無効Block(3～9 or 13～19)時 or Empty Sensor時処理
			read_change(com_mode,value,0,1);
		}
	}
}
#endif

/* 測定波形読出し(R4) */
void command_R4 (short com_mode){

	short l;
	short value;
	short *data;
	short ofs;

	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	if(RX_buf[com_mode][8] == 0x2c){						//Data block部が1文字の場合
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//最後尾に','追加
		TX_buf[com_mode][11] = '0';
	}else{													//Data block部が2文字の場合
		TX_buf[com_mode][10] = RX_buf[com_mode][8];
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//最後尾に','追加
		TX_buf[com_mode][12] = '0';
	}
#if 0
	for (l = 0; l < 100; l++){
		value = (MES[ch_no[com_mode] -1].fow_data[l]-AD_BASE)/4;		//受波波形データ
		read_change(com_mode,value,0,1);
	}
#else

#if defined(MESWAVEXP)
	WavInBuff(com_mode);
#else
	if(MES[ch_no[com_mode]-1].wave_monitor_mode == 0 ||				//常に更新設定
		(MES[ch_no[com_mode]-1].err_status & ERR_FLOW_CONT) == 0){	//流量エラー無し
		if(RX_buf[com_mode][8] == 0x2c){		// 0～2はFWDのデータ
			ofs = RX_buf[com_mode][7] - '0';
			if(ofs == 0) {
				for(l = 0; l < 300; l++){
					fow_wave_data[l] = MES[ch_no[com_mode]-1].fow_data[l];
				}
			}
		}else{									// 10～12はREVのデータ
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
			if(RX_buf[com_mode][8] == 0x2c){						//FWDデータ
//				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)/4;	//受波波形データ
				value = (fow_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
			}else{													//REVデータ
//				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)/4;	//受波波形データ
				value = (rev_wave_data[(ofs*100)+l] - AD_BASE)>>4;	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
			}
			read_change(com_mode,value,0,1);
		}
	}else{
		for (l = 0; l < 100; l++){
			value = 0;											//受波波形データ
			read_change(com_mode,value,0,1);
		}
	}
#endif //MESWAVEXP

#endif

}

/* メーカー設定読出し(SFC9000対応)(R5) */
void command_R5 (short com_mode){

	short l;
	long value;
	unsigned short *pt;
	short sn_version;
	short sn_ver_cal;
	char add[20] = {0};

	value = SVD[ch_no[com_mode] -1].wave_vth;				//エンプティセンサ判定閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].correlate_level;			//演算異常判定閾値
	read_change(com_mode,value,1,2);

	value = SVD[ch_no[com_mode] -1].correlate_time;			//演算異常判定回数
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].balance_level;			//波形アンバランス閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].saturation_level;			//AGC不能判定閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].attenuate_level;			//波形減衰判定閾値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].gain_step;				//ゲインステップ幅
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sound_vel_sel;			//音速固定or測定
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sound_vel_fix;			//音速設定
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sound_vel_filter;			//音速フィルタ時定数
	read_change(com_mode,value,1,2);
	
	value = SVD[ch_no[com_mode] -1].viscos_auto;				//動粘度固定or測定
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].drive_pls;				//打込みパルス
	read_change(com_mode,value,0,2);

//	value = SVD[ch_no[com_mode] -1].atn_gain;				//アッテネータゲイン値
	value = 0;
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].fifo_ch_init;				//FIFO CH 初期値
	read_change(com_mode,value,0,2);
	
	value = SVD[0].cunet_delay;						//CUnet 再送信待機時間	※全CH共通
	read_change(com_mode,value,0,2);	

	value = SVD[ch_no[com_mode] -1].search_sw;				//相関値サーチ無効or有効
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].damp_mode;				//気泡対策モード
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].corr_up.DWORD;			//相関値幅上限値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].corr_low.DWORD;			//相関値幅下限値
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].inc;					//異常ホールド時間
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].hldt;				//レイトリミットホールドクリア
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rlt;					//レイトリミット
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].odpd;				//レイトリミット1st
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl1tg;				//ターゲット1st
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl1av;				//アベレージ1st
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].odpl;				//レイトリミット2nd
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl2tg;				//ターゲット2nd
	read_change(com_mode,value,2,2);

	value = SVD[ch_no[com_mode] -1].rl2av;				//アベレージ2nd
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].rl2d;				//ダンピング倍率
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].rl2hc;				//ホールドクリア
	read_change(com_mode,value,1,2);

	value = SVD[ch_no[com_mode] -1].dump_var;				//可変ダンピング
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].dump_mul;				//可変ダンピング倍率
	read_change(com_mode,value,2,2);
	
	pt = &SVD[ch_no[com_mode] -1].c_serial[0];			//シリアルナンバー		
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
		strncat(TX_buf[com_mode],add,sizeof(add));			//最後尾に読込値追加
		
}

/* デバッグモード設定読出し(SFC9000対応)(R6) */
void command_R6 (short com_mode){

	short value;

	value = MES[ch_no[com_mode] -1].test_enable;		//流量出力テストON/OFF
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].test_flow;		//流量出力テスト値
	read_change(com_mode,value,2,2);
	
	value = MES[ch_no[com_mode] -1].test_err_enable;		//エラーテストON/OFF
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].test_err_code;		//強制エラー種別
	read_change(com_mode,value,0,0);
}

/*******************************************
 * Function : command_R7
 * Summary  : 差分相関測定データ読み出し
 * Argument : 
 * Return   : 
 * Caution  : None
 * Note     : ゼロクロス測定データ読み出しはR3
 * *****************************************/
void command_R7 (short com_mode){

	short l;
	long value;
	long long diff;
	unsigned short err_st;
	unsigned long long addit;

	if(RX_buf[com_mode][7] != '0'){		//測定時データ
	
		//波形表示(全波形or正常波形)選択
		if(RX_buf[com_mode][9] == '0'){MES[ch_no[com_mode] -1].wave_monitor_mode = 0;}
		else{MES[ch_no[com_mode] -1].wave_monitor_mode = 1;}
		
		//返信列作成
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//1. 流量[mL/min]
		value = MES[ch_no[com_mode] -1].ml_min_now;
		read_change(com_mode,value,2,2);

		//2. 流速[m/s]
		value = MES[ch_no[com_mode] -1].flow_vel_c / 100;
		read_change(com_mode,value,3,2);

		//3. 音速[m/s]
		value = MES[ch_no[com_mode] -1].sound_vel_f;
		read_change(com_mode,value,0,2);

		//4. 積算値[mL/min]
		addit = MES[ch_no[com_mode] -1].addit_buff.DWORD  / 10000;
		read_change(com_mode,addit,3,2);

		//5. 受波波形最大値
		value = MES[ch_no[com_mode] -1].rev_max_data;
		read_change(com_mode,value,0,2);

		//6. 
		value = MES[ch_no[com_mode] -1].rev_min_data;					
		read_change(com_mode,value,0,2);

		//7. 伝搬時間差[ps]
		diff = MES[ch_no[com_mode] -1].delta_ts_zero * 32;
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);

		//8. 相関値幅
		value = (MES[ch_no[com_mode] -1].correlate * 1000);
		read_change(com_mode,value,0,2);

		//9. ゼロ点オフセット
		value = (SVD[ch_no[com_mode] -1].zero_offset - 4000) * 32;
		read_change(com_mode,value,0,2);

		//10. status
		err_st = MAIN[ch_no[com_mode] -1].err_condition;
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		//11. FIFO受波波形検出位置
		if((SVD[ch_no[com_mode] -1].fix_data & 0x08) != 0){  //固定値設定
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
		if((SVD[ch_no[com_mode] -1].fix_data & 0x02) != 0){  //固定値設定
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
		if((SVD[ch_no[com_mode] -1].fix_data & 0x04) != 0){  //固定値設定
			if(SVD[ch_no[com_mode] -1].fix_fifo_ch_read == 0){
				value = MES[ch_no[com_mode] -1].zero_fifo_ch_read;
			}else{
				value = SVD[ch_no[com_mode] -1].fix_fifo_ch_read;
			}
		}else{
			value = MES[ch_no[com_mode] -1].fifo_ch_read;
		}
		read_change(com_mode,value,0,2);
		
		//15. 受波の差(P1-P2)
		if(MES[ch_no[com_mode] -1].max_point_sub_f >= LIM_OVERFLOW)	{ value = (LIM_OVERFLOW - 1); }
		else if(MES[ch_no[com_mode] -1].max_point_sub_f <= -LIM_OVERFLOW)	{ value = -(LIM_OVERFLOW - 1); }
		else { value = MES[ch_no[com_mode] -1].max_point_sub_f; }
		read_change(com_mode, 0, 0, ADD_NO_CMM);

		//16-55. 差分相関値
		if(MES[ch_no[com_mode] -1].wave_monitor_mode == 0 ||					//常に更新設定
		(MES[ch_no[com_mode]-1].err_status & ERR_FLOW_CONT) == 0){		//流量エラー無し
				
			if((MES[ch_no[com_mode]-1].err_status & ERR_JUDGE_EMPTY) != 0){	//Empty Sensor時 波形データ0
				for (l = 0; l < 40; l++){
					sum_abs_data[l] = 0;	//波形データ0
				}
			}else{
				for (l = 0; l < 40; l++){
					sum_abs_data[l] = SAVE[ch_no[com_mode] -1].sum_abs_com[l];	//16-55. 差分相関値
				}
			}
		}

		for (l = 0; l < 40; l++){
			value = sum_abs_data[l];		//差分相関値
			read_change(com_mode,value,0,1);
		}
	}else{				//ゼロ調整時データ
		//波形表示(全波形or正常波形)選択
		if(RX_buf[com_mode][9] == '0'){MES[ch_no[com_mode] -1].wave_monitor_mode = 0;}
		else{MES[ch_no[com_mode] -1].wave_monitor_mode = 1;}
		
		//返信列作成
		TX_buf[com_mode][9] = RX_buf[com_mode][7];
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	
		value = SVD[ch_no[com_mode] -1].zero_flow_qat.DWORD;		//流量[mL/min]
		read_change(com_mode,value,2,2);
	
		value = SVD[ch_no[com_mode] -1].zero_flow_vel.DWORD / 100;		//流速[m/s]
		read_change(com_mode,value,3,2);
	
		value = SVD[ch_no[com_mode] -1].zero_sound_spd;			//音速[m/s]
		read_change(com_mode,value,0,2);
	
		addit = SVD[ch_no[com_mode] -1].zero_addit.DWORD / 10000;		//積算値[mL/min]
		read_change(com_mode,addit,3,2);
	
		value = SVD[ch_no[com_mode] -1].zero_wave_max;			//受波波形最大値
		read_change(com_mode,value,0,2);
	
		value = SVD[ch_no[com_mode] -1].zero_wave_min;			//受波波形最小値
		read_change(com_mode,value,0,2);
	
		diff = SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD * 32;		//伝搬時間差[ps]
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);
	
		value = (SVD[ch_no[com_mode] -1].zero_correlate * 1000);	//相関値幅
		read_change(com_mode,value,0,2);
	
		value = (SVD[ch_no[com_mode] -1].zero_zero_offset - 4000) * 32;	//ゼロ点オフセット
		read_change(com_mode,value,0,2);

		err_st = SVD[ch_no[com_mode] -1].zero_condition;			//status
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		value = SVD[ch_no[com_mode] -1].zero_fifo_pos;			//FIFO受波波形検出位置
		read_change(com_mode,value,0,2);
	
//		value = SVD[ch_no[com_mode] -1].zero_gain_for;			//Gain 1st stage
		value = SVD[ch_no[com_mode] -1].zero_gain_1st;			//Gain 1st stage
		read_change(com_mode,value,0,2);

//		value = SVD[ch_no[com_mode] -1].zero_gain_rev;			//Gain 2nd stage
		value = SVD[ch_no[com_mode] -1].zero_gain_2nd;			//Gain 2nd stage
		read_change(com_mode,value,0,2);
	
		value = SVD[ch_no[com_mode] -1].zero_fifo_ch;				//FIFO CH
		read_change(com_mode,value,0,2);
	
		value = SVD[ch_no[com_mode] -1].zero_p1p2;				//受波の差(P1-P2)
		read_change(com_mode,value,0,0);

		if((SVD[ch_no[com_mode] -1].zero_condition & CON_EMPTY) != 0){	//Empty Sensor時 波形データ0
			for (l = 0; l < 40; l++){
				value = 0;		//差分相関値;
				read_change(com_mode,value,0,1);
			}
		}else{
			for (l = 0; l < 40; l++){
				// value = SVD[ch_no[com_mode] -1].zero_sum_abs[l];		//差分相関値;
				value = 0;
			    read_change(com_mode,value,0,1);
			}
		}
	}
}

/* 測定データ読出し(SFC9000対応・R7コマンド短縮版)(RC) */
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
	
/* シングルコマンド */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		
		value = MES[ch_no[com_mode] -1].ml_min_now;			//流量[mL/min]
		read_change(com_mode,value,2,2);
		
		value = MES[ch_no[com_mode] -1].sound_vel_f;			//音速[m/s]
		read_change(com_mode,value,0,2);
		
		value = MES[ch_no[com_mode] -1].rev_max_data;			//受波波形最大値
		read_change(com_mode,value,0,2);
	
		value = MES[ch_no[com_mode] -1].rev_min_data;			//受波波形最小値
		read_change(com_mode,value,0,2);
	
		diff = MES[ch_no[com_mode] -1].delta_ts_zero * 32;		//伝搬時間差[ps]
		if (diff > 99999999) {diff = 99999999;}
		else if (diff < -99999999) {diff = -99999999;}
		read_change(com_mode,diff,0,2);
	
		value = (SVD[ch_no[com_mode] -1].zero_offset - 4000) * 32;	//ゼロ点オフセット
		read_change(com_mode,value,0,2);

		err_st = MAIN[ch_no[com_mode] -1].err_condition;		//status
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));
	
		value = get_attenuator_gain(ch_no[com_mode]-1);			//Gain 1st stage
		read_change(com_mode,value,0,2);

		value = MES[ch_no[com_mode] -1].amp_gain_for;			//Gain 2nd stage
		read_change(com_mode,value,0,2);
	
		value = MES[ch_no[com_mode] -1].max_point_sub_f;		//受波の差(P1-P2)
		read_change(com_mode,value,0,0);
	}
/* マルチコマンド */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];					//CH数判定
		ch_z = ch[0] - 0x30;
		ch[0] = 0;				
		for (m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for (m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH番号判定
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma));
			
			value = MES[ch_number -1].ml_min_now;			//流量[mL/min]
			read_change(com_mode,value,2,2);
			
			value = MES[ch_number -1].sound_vel_f;			//音速[m/s]
			read_change(com_mode,value,0,2);
			
			value = MES[ch_number -1].rev_max_data;			//受波波形最大値
			read_change(com_mode,value,0,2);
		
			value = MES[ch_number -1].rev_min_data;			//受波波形最小値
			read_change(com_mode,value,0,2);
		
			diff = MES[ch_number -1].delta_ts_zero * 32;		//伝搬時間差[ps]
			if (diff > 99999999) {diff = 99999999;}
			else if (diff < -99999999) {diff = -99999999;}
			read_change(com_mode,diff,0,2);
		
			value = (SVD[ch_number -1].zero_offset - 4000) * 32;	//ゼロ点オフセット
			read_change(com_mode,value,0,2);
	
			err_st = MAIN[ch_number -1].err_condition;		//status
			read_change0X_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));
			
			value = get_attenuator_gain(ch_number-1);			//Gain 1st stage
			read_change(com_mode,value,0,2);
	
			value = MES[ch_number -1].amp_gain_for;			//Gain 2nd stage
			read_change(com_mode,value,0,2);
		
			value = MES[ch_number -1].max_point_sub_f;		//受波の差(P1-P2)
			read_change(com_mode,value,0,0);			
		}
	}
}

/* 測定波形読出し(SFC9000対応)(R8) */
void command_R8 (short com_mode){

	short l;
	short value;
	short *data;
	short ofs;
	short pch = ch_no[com_mode] -1;
	short AmpOfs; //波形振幅オフセット値

	TX_buf[com_mode][9] = RX_buf[com_mode][7];
	if(RX_buf[com_mode][8] == 0x2c){						//Data block部が1文字の場合
		//波形表示(全波形or正常波形)選択
		if(RX_buf[com_mode][9] == '0'){MES[pch].wave_monitor_mode = 0;}
		else{MES[pch].wave_monitor_mode = 1;}
		//返信列作成
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//最後尾に','追加
		TX_buf[com_mode][11] = '0';
	}else{													//Data block部が2文字の場合
		//波形表示(全波形or正常波形)選択
		if(RX_buf[com_mode][10] == '0'){MES[pch].wave_monitor_mode = 0;}
		else{MES[pch].wave_monitor_mode = 1;}
		//返信列作成
		TX_buf[com_mode][10] = RX_buf[com_mode][8];
		strncat(TX_buf[com_mode],comma,sizeof(comma));	//最後尾に','追加
		TX_buf[com_mode][12] = '0';
	}
#if 0
	for (l = 0; l < 100; l++){
		value = (MES[pch].fow_data[l]-AmpOfs)/4;		//受波波形データ
		read_change(com_mode,value,0,1);
	}
#else

#if defined(MESWAVEXP)
	WavInBuff(com_mode);
#else
	if(MES[pch].wave_monitor_mode == 0 ||				//常に更新設定
		(MES[pch].err_status & ERR_FLOW_CONT) == 0){	//流量エラー無し

		if((MES[pch].err_status & ERR_JUDGE_EMPTY) != 0){	//Empty Sensor時 波形データ0
			ofs = 3;
		}else{		
			if(RX_buf[com_mode][8] == 0x2c){		// 0～2はFWDのデータ
				ofs = RX_buf[com_mode][7] - '0';
				if(ofs == 0) {
					for(l = 0; l < 300; l++){
						fow_wave_data[l] = MES[pch].fow_data[l];
					}
				}
			}else{									// 10～12はREVのデータ
				ofs = RX_buf[com_mode][8] - '0';
				if(ofs == 0) {
					for(l = 0; l < 300; l++){
						rev_wave_data[l] = MES[pch].rev_data[l];
					}
				}
			}
		}		
	}



	//評価用
	if(SVD[pch].sum_step == 2){	//打込み回数上流下流各2回
		MES_SUB[pch].sample_cnt = 2;
	}else if(SVD[pch].sum_step == 3){	//打込み回数上流下流各3回
		MES_SUB[pch].sample_cnt = 3;
	}else if(SVD[pch].sum_step == 4){	//打込み回数上流下流各4回
		MES_SUB[pch].sample_cnt = 4;
	}else{	//打込み回数上流下流各4回
		MES_SUB[pch].sample_cnt = 4;
	}
	// AmpOfs = 2047 * MES_SUB[pch].sample_cnt;
	AmpOfs = AD_BASE_UNIT * MES_SUB[pch].sample_cnt;
	//評価用

	
	
	if(ofs <= 2) {
		for (l = 0; l < 100; l++){
			if(RX_buf[com_mode][8] == 0x2c){						//FWDデータ
//				value = (fow_wave_data[(ofs*100)+l] - AmpOfs)/4;	//受波波形データ
				// value = (fow_wave_data[(ofs*100)+l] - AmpOfs)>>4;	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
				value = (fow_wave_data[(ofs*100)+l] - AmpOfs);	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
			}else{													//REVデータ
//				value = (rev_wave_data[(ofs*100)+l] - AmpOfs)/4;	//受波波形データ
				// value = (rev_wave_data[(ofs*100)+l] - AmpOfs)>>4;	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
				value = (rev_wave_data[(ofs*100)+l] - AmpOfs);	//受波波形データ(4回分加算してあるので更に4で割って1/16とする)
			}
			read_change(com_mode,value,0,1);
		}
	}else{
		for (l = 0; l < 100; l++){
			value = 0;								//波形データ0：無効Block(3～9 or 13～19)時 or Empty Sensor時処理
			read_change(com_mode,value,0,1);
		}
	}
#endif //MESWAVEXP

#endif

}

/* パルス出力(R9) */
void command_R9 (short com_mode){

	short value;

	value = MES[ch_no[com_mode] -1].pls_test_enable;		//パルス出力テストON/OFF
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].pls_test_ch1;		//パルス出力CH選択1
	read_change(com_mode,value,0,2);
	
	value = MES[ch_no[com_mode] -1].pls_test_ch2;		//パルス出力CH選択2
	read_change(com_mode,value,0,0);
}

/* 検査モードデータ読出し(SFC9000対応)(Ri) */
void command_Ri (short com_mode){

	short l;
	long value;
	
	for (l = 0; l < 6; l++){
		value = wave_hight[0][l];		//波形データ(IN側)
		read_change(com_mode,value,0,1);
	}
	for (l = 0; l < 6; l++){
		value = wave_hight[1][l];		//波形データ(OUT側)
		read_change(com_mode,value,0,1);
	}
}

/* SW状態チェック(RX) */
void command_RX (short com_mode){

	short value;

	value = disp_ch_read();					//CHロータリーSW状態
	read_change(com_mode,value,0,2);

	value = disp_cunet_read();				//CUnetロータリーSW状態
	read_change(com_mode,value,0,2);
	
	value = disp_zero_read();				//ゼロ調整SW状態
	read_change(com_mode,value,0,0);
}

/* 波形状態読出し(SFC9000対応)(RW) */
void command_RW (short com_mode){

	short m, ch_z, ch_number;
	long value;
	unsigned short err_st;
	char ch[1] = {0};

	/* シングルコマンド */
	if(ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		err_st = MAIN[ch_no[com_mode] -1].err_condition;	//アラーム発生ステータス
		read_change0X_status(com_mode,err_st);
		strncat(TX_buf[com_mode],comma,sizeof(comma));

		value = MES[ch_no[com_mode] -1].rev_max_data;			//受波波形最大値
		read_change(com_mode,value,0,2);

		value = MES[ch_no[com_mode] -1].rev_min_data;			//受波波形最小値
		read_change(com_mode,value,0,2);

		value = MES[ch_no[com_mode] -1].amp_gain_for;			//上流ゲイン値
		read_change(com_mode,value,0,0);
	/* マルチコマンド */
	}else if(ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];			//CH数判定
		ch_z = ch[0] - 0x30;
		for(m = 0; m <= ch_z; m++){
			TX_buf[com_mode][8 + 2*m] = RX_buf[com_mode][6 + 2*m];
			TX_buf[com_mode][9 + 2*m] = RX_buf[com_mode][7 + 2*m];
		}
		for(m = 0; m < ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];			//CH番号判定
			ch_number = ch[0] - 0x30;
			strncat(TX_buf[com_mode],comma,sizeof(comma));

			err_st = MAIN[ch_number -1].err_condition;	//アラーム発生ステータス
			read_change0X_status(com_mode,err_st);
			strncat(TX_buf[com_mode],comma,sizeof(comma));

			value = MES[ch_number -1].rev_max_data;			//受波波形最大値
			read_change(com_mode,value,0,2);

			value = MES[ch_number -1].rev_min_data;			//受波波形最小値
			read_change(com_mode,value,0,2);

			value = MES[ch_number -1].amp_gain_for;			//上流ゲイン値
			read_change(com_mode,value,0,0);
		}
	}
}

/* センサパルス設定読出し(SFC014E互換)(Rp) */
 void command_Rp (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].drive_pls;			//打込みパルス
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].drive_search;		//調整モード
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].drive_freq;		//駆動周波数
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].start_freq;		//サーチ開始周波数
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].stop_freq;			//サーチ停止周波数
	read_change(com_mode,value,0,0);
} 

/* センサシリアルナンバー読出し(SFC014E互換)(Rn) */
 void command_Rn (short com_mode){

	short l;
	unsigned short *pt;
	short sn_version;
	short sn_ver_cal;
	char add[20] = {0};

	pt = &SVD[ch_no[com_mode] -1].s_serial[0];			//センサシリアルナンバー		
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
	strncat(TX_buf[com_mode],add,sizeof(add));			//最後尾に読込値追加
} 

/* センサ情報読出し(評価用)(Ra) */
 void command_Ra (short com_mode){

	short value;

	value = SVD[ch_no[com_mode] -1].sns_option;			//センサオプション
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sns_disL;		//センサ間距離(L)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sns_disL_l;		//センサ間距離(L_l)
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sns_tau;		//無駄時間
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sns_coef;		//互換係数
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].adc_clock;		//ADCクロック
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].wind_offset;		//WINDOWオフセット
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sum_start;		//差分相関開始位置
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].sum_end;		//差分相関終了位置
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].sum_step;		//差分相関間隔
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].fix_data;		//固定値設定
	read_change(com_mode,value,0,2);

	value = SVD[ch_no[com_mode] -1].fix_amp_gain_rev;		//Wiper Position(固定値)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].fix_fifo_ch_read;		//FIFO CH(固定値)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].fix_fifo_no_read;		//Leading Position(固定値)
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].ZerCrsSttPnt;		//Zero Cross Start Point
	read_change(com_mode,value,0,2);
	
	value = SVD[ch_no[com_mode] -1].ZerCrsUseNum;		//Zero Cross Use Number
	read_change(com_mode,value,0,0);
} 

/* ステータス読込（16進4文字）*/
void read_change0X_status(short com_mode,unsigned short log_st){

	unsigned short log_st_cal;
	char add[20] = {0};

	log_st_cal = log_st / 0x1000;

	if (log_st_cal <=9)		add[0] = log_st_cal  + 0x30;	// 0x30:ASCII→16進数字変換0-9
	else	/*A-F*/			add[0] = log_st_cal  + 0x37;	// 0x37:ASCII→16進数字変換A-F
	
	log_st_cal = (log_st % 0x1000) / 0x100;
	if (log_st_cal <=9)		add[1] = log_st_cal  + 0x30;
	else	/*A-F*/			add[1] = log_st_cal  + 0x37;
	
	log_st_cal = (log_st % 0x100) / 0x10;
	if (log_st_cal <=9)		add[2] = log_st_cal  + 0x30;
	else	/*A-F*/			add[2] = log_st_cal  + 0x37;

	log_st_cal = log_st % 0x10;
	if (log_st_cal <=9)		add[3] = log_st_cal  + 0x30;
	else	/*A-F*/			add[3] = log_st_cal  + 0x37;

	strncat(TX_buf[com_mode],add,sizeof(add));					//最後尾に読込値追加

}

/* ステータス読込（16進2文字）*/
void read_change0X_2digit_status(short com_mode,unsigned short log_st){

	unsigned short log_st_cal;
	char add[20] = {0};

	log_st_cal = log_st / 0x10;
	if (log_st_cal <=9)		add[0] = log_st_cal  + 0x30;	// 0x30:ASCII→16進数字変換0-9
	else	/*A-F*/			add[0] = log_st_cal  + 0x37;	// 0x37:ASCII→16進数字変換A-F
	
	log_st_cal = log_st % 0x10;
	if (log_st_cal <=9)		add[1] = log_st_cal  + 0x30;
	else	/*A-F*/			add[1] = log_st_cal  + 0x37;

	strncat(TX_buf[com_mode],add,sizeof(add));					//最後尾に読込値追加

}

/* バージョン読込 */
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
		if (ver_cal >= 0 && ver_cal <=9)	add[level - z -1] = ver_cal + 0x30;	// 0x30:ASCII→16進数字変換0-9
		else	/*A-F*/				add[level - z -1] = ver_cal + 0x37;	// 0x37:ASCII→16進数字変換A-F
		pow_1 *= 0x0010;
		pow_2 *= 0x0010;
	}
	
	ver[0] = add[0];
	for(z = 1; z < level; z++){
		ver[2*z -1] = 0x2E; // ="."
		ver[2*z] = add[z];
	}
	strncat(TX_buf[com_mode],ver,sizeof(ver));				//最後尾に読込値追加

}

/* バージョン読込 (RVコマンド)*/
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
		if (ver_cal >= 0 && ver_cal <=9)	add[level - z -1] = ver_cal + 0x30;	// 0x30:ASCII→16進数字変換0-9
		else	/*A-F*/				add[level - z -1] = ver_cal + 0x37;	// 0x37:ASCII→16進数字変換A-F
		pow_1 *= 0x0010;
		pow_2 *= 0x0010;
	}
	
	ver[0] = add[0];
	ver[1] = add[1];
	ver[2] = 0x2E; // ="."
	ver[3] = add[2];
	ver[4] = add[3];
	strncat(TX_buf[com_mode],ver,sizeof(ver));				//最後尾に読込値追加

}


/****************************************************
 * Function : JdgIdxErr
 * Summary  : デジタルフィルタ配置インデックスの判定
 * Argument : Idx -> インデックス
 * Return   : JdgFlg -> B_OK : 正常
 *                      B_NG : 異常
 * Caution  : None
 * Notes    : 新デジタルフィルタ仕様のため、
 *            以前に使用していた6,7,8,9,11,16,17,18,19は削除
 ****************************************************/
short JdgIdxErr(short Idx)
{
	short JdgFlg = B_OK;

	//0~20以外はNG
	if((Idx < 0) || (20 < Idx)){
		JdgFlg = B_NG;
	}
	else{
		//指定番号は仕様変更のため削除
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
 * Summary  : デジタルフィルタ係数読み出し
 * Argument : com_mode -> 0 : ホスト
 *          :             1 : メンテ
 * Return   : void
 * Caution  : None
 * Notes    : フォーマットは以下
 *          : @Rc00x,ii,FCS
 *          :   x -> チャンネル番号
 *          :   ii -> デジタルフィルタインデックス
 ****************************************************/
void command_Rc(short com_mode)
{
	short Idx;
	unsigned short Cef;
	//valueを取得
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
		case 0:  Cef = FPGA_FILIN0_0; break;	/* デジタルフィルタ係数(入力側0) */
		case 1:  Cef = FPGA_FILIN0_1; break;	/* デジタルフィルタ係数(入力側0) */
		case 2:  Cef = FPGA_FILIN1_0; break;	/* デジタルフィルタ係数(入力側1) */
		case 3:  Cef = FPGA_FILIN1_1; break;	/* デジタルフィルタ係数(入力側1) */
		case 4:  Cef = FPGA_FILIN2_0; break;	/* デジタルフィルタ係数(入力側2) */
		case 5:  Cef = FPGA_FILIN2_1; break;	/* デジタルフィルタ係数(入力側2) */
//		case 6:  Cef = FPGA_FILCOE_A3_0; break;
//		case 7:  Cef = FPGA_FILCOE_A3_1; break;
//		case 8:  Cef = FPGA_FILCOE_A4_0; break;
//		case 9:  Cef = FPGA_FILCOE_A4_1; break;
		// case 10: Cef = FPGA_FIL_EN; break;		/* デジタルフィルタ有効・無効設定 */
//		case 11: Cef = FPGA_FILCOE_B0_1; break;
		case 12: Cef = FPGA_FILOUT1_0; break;	/* デジタルフィルタ係数(出力側1) */
		case 13: Cef = FPGA_FILOUT1_1; break;	/* デジタルフィルタ係数(出力側1) */
		case 14: Cef = FPGA_FILOUT2_0; break;	/* デジタルフィルタ係数(出力側2) */
		case 15: Cef = FPGA_FILOUT2_1; break;	/* デジタルフィルタ係数(出力側2) */
//		case 16: Cef = FPGA_FILCOE_B3_0; break;
//		case 17: Cef = FPGA_FILCOE_B3_1; break;
		case 20: Cef = FPGA_FIL_EN; break;	/* デジタルフィルタ有効・無効設定 */
		default:
			break;
		}
        read_change(com_mode, Idx, 0, 2);
		read_change(com_mode, Cef, 0, 0);
	}

}

void read_command_R(short com_mode){
	
	switch (RX_buf[com_mode][2]){		//ヘッダーコード末尾
/* ステータス読出し(SFC9000対応)(RS) */
	case 'S':
		command_RS(com_mode);
		break;
/* ステータス読出し(Rs) */
	case 's':
		command_Rs(com_mode);
		break;
/* 口径読出し(Rg) */
	case 'g':
		command_Rg(com_mode);
		break;
/* フルスケール読出し(Rr) */
	case 'r':
		command_Rr(com_mode);
		break;
/* Ｋファクタ読出し(Rk) */
	case 'k':
		command_Rk(com_mode);
		break;
/* ダンピング読出し(Rd) */
	case 'd':
		command_Rd(com_mode);
		break;
/* ローカット読出し(Rl) */
	case 'l':
		command_Rl(com_mode);
		break;
/* バーンアウト読出し(Rb) */
	case 'b':
		command_Rb(com_mode);
		break;
/* 動粘度読出し(Rv) */
	case 'v':
		command_Rv(com_mode);
		break;
/* エラーホールドタイム読出し(Rh) */
	case 'h':
		command_Rh(com_mode);
		break;
/* ユーザーリニアライズ読出し(Ru) */
	case 'u':
		command_Ru(com_mode);
		break;
/* メーカーリニアライズ読出し(Rm) */
	case 'm':
		command_Rm(com_mode);
		break;
/* 逆流判定値読出し(RR) */
	case 'R':
		command_RR(com_mode);
		break;
/* ゼロ点調整状態読出し(Rz) */
	case 'z':
		command_Rz(com_mode);
		break;
/* バージョン値読出し(SFC9000対応)(RF) */
	case 'F':
		command_RF(com_mode);
		break;
/* バージョン値読出し(RV) */
	case 'V':
		command_RV(com_mode);
		break;
/* ログデータ読出し(SFC9000対応)(RG) */
	case 'G':	
		command_RG(com_mode);
		break;
/* ログデータ読出し(RL) */
	case 'L':	
		command_RL(com_mode);
		break;
/* ゼロ調整データ読出し(SFC9000対応)(RD) */
	case 'D':
		command_RD(com_mode);
		break;
/* ゼロ調整データ読出し(RZ) */
	case 'Z':
		command_RZ(com_mode);
		break;
/* 波形異常判定値設定読出し(RE) */
	case 'E':
		command_RE(com_mode);
		break;
/* アッテネータゲイン読出し(RA) */
	case 'A':
		command_RA(com_mode);
		break;
/* 薬液リニアライズモード読出し(Ry) */
	case 'y':
		command_Ry(com_mode);
		break;
/* 音速読出し(RM) */
	case 'M':
		command_RM(com_mode);
		break;
/* フィルタ設定読出し(Rf) */
	case 'f':
		command_Rf(com_mode);
		break;		
/* 積算目標値読出し(Rt) */
	case 't':
		command_Rt(com_mode);
		break;		
/* 積算オフセット値読出し(RT) */
	case 'T':
		command_RT(com_mode);
		break;				
/* メーカー設定読出し(R1) */
	case '1':
		command_R1(com_mode);
 		break;
/* デバッグモード設定読出し(R2) */
	case '2':
		command_R2(com_mode);
		break;
/* 測定データ読出し(R3) */
	case '3':
		command_R3(com_mode);
		break;
/* 測定波形読出し(R4) */
	case '4':
		command_R4(com_mode);
		break;
/* メーカー設定読出し(SFC9000対応)(R5) */
	case '5':
		command_R5(com_mode);
 		break;
/* デバッグモード設定読出し(SFC9000対応)(R6) */
	case '6':
		command_R6(com_mode);
		break;
/* 測定データ読出し(SFC9000対応)(R7) */
	case '7':
		command_R7(com_mode);
		break;
/* 測定データ読出し(SFC9000対応・R7コマンド短縮版)(RC) */
	case 'C':
		command_RC(com_mode);
		break;
/* 測定波形読出し(SFC9000対応)(R8) */
	case '8':
		command_R8(com_mode);
		break;
/* パルス出力(R9) */
	case '9':
		command_R9(com_mode);
		break;
/* 検査モードデータ読出し(SFC9000対応)(Ri) */
	case 'i':
		command_Ri(com_mode);
		break;
/* SW状態チェック(RX) */
	case 'X':
		command_RX(com_mode);
		break;	
		/* 波形状態読出し(SFC9000対応)(RW) */
	case 'W':
		command_RW(com_mode);
		break;
/* センサパルス設定読出し(Rp) */
	case 'p':
		command_Rp(com_mode);
		break;
/* センサシリアルナンバー読出し(Rn) */
	case 'n':
		command_Rn(com_mode);
		break;
/* センサ情報読出し（評価用）(Ra) */
	case 'a':
		command_Ra(com_mode);
		break;
/* デジタルフィルタ係数 */
	case 'c':
		command_Rc(com_mode);
		break;
	}
}

/* 口径書込み(Wg) */		//種別0～5
void command_Wg (short com_mode){

	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >=0 && value[com_mode] <= 5 ) && size[com_mode] == 1 && digit_check[com_mode] <= 0){
		/*口径変更時*/
		if (SVD[ch_no[com_mode] -1].sensor_size != value[com_mode]){
			InitFifoCh(ch_no[com_mode] - 1, value[com_mode]);		/*FIFO CH初期化*/
			err_thlevel_init(ch_no[com_mode] -1, value[com_mode]);		/*波形異常判定閾値初期化*/
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

/* フルスケール書込み(Wr) */	//1～20000mL/min
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

/* Ｋファクタ書込み(Wk) */	//0.000～9.999
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

/* ダンピング書込み(Wd) */		//0.0～25.0sec
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

/* ローカット書込み(Wl) */		//0.0～25.0%FS
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
/* バーンアウト書込み(Wb) */
void command_Wb (short com_mode){
	
	/* バーンアウト種別判定 */	//種別0～4
	value_judge(com_mode,7,0);
	if (value[com_mode] >=0 && value[com_mode] <= 4 && size[com_mode] == 1){
		SVD[ch_no[com_mode] -1].burnout = value[com_mode];
	/* バーンアウト数値判定 */	//-300～300%
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
			
/* 動粘度係数書込み(Wv) */		//0.00～40.00cSt
void command_Wv (short com_mode){

	value_judge(com_mode,7,2);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 4000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
		SVD[ch_no[com_mode] -1].viscos = value[com_mode]; /*make_viscos_tbl(ch_no[com_mode] -1);*/
		/* 薬液リニアライズモードON = 薬液種別判定 */
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

/* エラーホールドタイム書込み(Wh) */		//0～99sec
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

/* 逆流判定値書込み(WR) */
void command_WR (short com_mode){
	/* 閾値設定判定 */		//閾値-100～0%
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= -100 && value[com_mode] <= 0 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].reverse_level = value[com_mode];
	/* 時間設定判定 */		//0.1～10.0sec
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

/* ユーザーリニアライズ書込み(Wu) */
void command_Wu (short com_mode){

	short l;
	short check;
	short linear_point;		// リニアライズ用
	long linear_check;		// リニアライズ大小判定用
	unsigned short *pt;		// リニアライズ用ポインタ
	unsigned short point = 0;

	check = 0;
	/* 設定値数判定 */		//設定点数 0-15
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 15 && (size[com_mode] == 1 || size[com_mode] == 2) && digit_check[com_mode] <= 0){
		//point
		point = value[com_mode];
		point = point << 8;
		point &= 0xFF00;
	
		SVD[ch_no[com_mode] -1].uslnr_num &= 0x00FF;	//いったん上位8ビットを落とす
		SVD[ch_no[com_mode] -1].uslnr_num |= point;	//上位8bitに設定

		linear_point = value[com_mode];
		linear_check = -1;		//output1 = 0 許容用
	/* 出力・入力設定値 分離 */
	/* 出力×15 */
		pt = &SVD[ch_no[com_mode] -1].uslnr_out1.WORD.low;
		for (l=1; l<=15; l++){		//for-loop
		/* 入力値 */
			i_num[com_mode]++;		// ','の1つ後ろへ移動
			value_judge(com_mode,i_num[com_mode],1);
			if (digit_check[com_mode] > 1 || size[com_mode] < 1 || size[com_mode] > 7){
				end_code[com_mode] = FORMAT_ERROR;	//End Code 14  桁数異常・未入力(フォーマットエラー)
				break;
			}			//範囲外 for-loop 脱出
			if (value[com_mode] < 0 || value[com_mode] > 200000){
				end_code[com_mode] = USERLINEAR_VALUE_ERROR;			//End Code 44  流量設定値 数値範囲外入力
				break;
			}			//範囲外 for-loop 脱出
			if (l <= linear_point && value[com_mode] <= linear_check){
				end_code[com_mode] = USERLINEAR_ORDER_ERROR;		//End Code 45  流量設定値 順番不適合
				break;
			}			//範囲外 for-loop 脱出
			linear_check = value[com_mode];
			*pt = linear_check % 0x10000;
			pt++;
			linear_check = value[com_mode];
			*pt = linear_check / 0x10000;
			linear_check = value[com_mode];
			pt++;
			check++;
		}
			/* 入力×15 */
				if (check == 15){
					linear_check = -1;	//input1 = 0 許容用
					pt = &SVD[ch_no[com_mode] -1].uslnr_in1.WORD.low;
					for (l=1; l<=15; l++){	//for-loop
					/* 入力値 */
						i_num[com_mode]++;	// ','の1つ後ろへ移動
						value_judge(com_mode,i_num[com_mode],1);
						if (digit_check[com_mode] > 1 || size[com_mode] < 1 || size[com_mode] > 7){ 
							end_code[com_mode] = FORMAT_ERROR;	//End Code 14  桁数異常・未入力(フォーマットエラー)
							break;
						}		//範囲外 for-loop 脱出
						if (value[com_mode] < 0 || value[com_mode] > 200000){ 
							end_code[com_mode] = USERLINEAR_VALUE_ERROR;			//End Code 44  流量設定値 数値範囲外入力
							break;
						}		//範囲外 for-loop 脱出
						if (l <= linear_point && value[com_mode] <= linear_check){ 
							end_code[com_mode] = USERLINEAR_ORDER_ERROR;		//End Code 45  流量設定値 順番不適合
							break;
						}		//範囲外 for-loop 脱出
						linear_check = value[com_mode];
						*pt = linear_check % 0x10000;
						pt++;
						linear_check = value[com_mode];
						*pt = linear_check / 0x10000;
						linear_check = value[com_mode];
						pt++;
					}						//End Code 00  正常終了15点以内・0-20000.0・数値入力順正常）
				}
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ end_code[com_mode] = FORMAT_ERROR;}
			else{ end_code[com_mode] = USERLINEAR_POINT_ERROR;} 						//End Code 43 = 設定点数異常
}

/* ユーザーリニアライズ切替え(WU) */
void command_WU (short com_mode){

	short l;
	long value_linear;
	short check;
	unsigned short *pt;		// リニアライズ用ポインタ
	unsigned short point = 0;
	unsigned short value_high;
	unsigned short value_low;

	check = 0;	
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
		//ON
		if(value[com_mode] == 1){
			/* 出力×5 空確認*/
			pt = &SVD[ch_no[com_mode] -1].uslnr_out1.WORD.low;
			for (l = 0; l < 5; l++){
				value_low = *pt;
				pt++;
				value_high = *pt;
				value_linear = value_low + value_high * 0x10000;
				pt++;
				if(value_linear == 0) check++;
			}
			/* 入力×5 空確認*/
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
				
				SVD[ch_no[com_mode] -1].uslnr_num &= 0x00FF;	//いったん上位8ビットを落とす
				SVD[ch_no[com_mode] -1].uslnr_num |= point;	//上位8bitに設定				
			}else{
				end_code[com_mode] = USERLINEAR_5POINT_EMPTY;
			}
		}
		//OFF
		else{
			SVD[ch_no[com_mode] -1].uslnr_num &= 0x00FF;	//いったん上位8ビットを落とす
			SVD[ch_no[com_mode] -1].uslnr_num |= 0;	//上位8bitに設定
		}
	}
	else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		end_code[com_mode] = USERLINEAR_5POINT_ERROR;
	}
}

/* メーカーリニアライズ書込み(Wm) */
void command_Wm (short com_mode){

	short l;
	short check;
	short linear_point;		// リニアライズ用
	long linear_check;		// リニアライズ大小判定用
	unsigned short *pt;		// リニアライズ用ポインタ
	unsigned short point = 0;

	check = 0;
	/* 設定値数判定 */		//設定点数 0-15
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 15 && (size[com_mode] == 1 || size[com_mode] == 2) && digit_check[com_mode] <= 0){
		//point
		point = value[com_mode];
		point = point << 8;
		point &= 0xFF00;
	
		SVD[ch_no[com_mode] -1].mklnr_num &= 0x00FF;	//いったん上位8ビットを落とす
		SVD[ch_no[com_mode] -1].mklnr_num |= point;	//上位8bitに設定

		linear_point = value[com_mode];
		linear_check = -1;		//output1 = 0 許容用
	/* 出力・入力設定値 分離 */
	/* 出力×15 */
		pt = &SVD[ch_no[com_mode] -1].mklnr_out1.WORD.low;
		for (l=1; l<=15; l++){		//for-loop
		/* 入力値 */
			i_num[com_mode]++;		// ','の1つ後ろへ移動
			value_judge(com_mode,i_num[com_mode],3);
			if (digit_check[com_mode] > 3 || size[com_mode] < 1 || size[com_mode] > 5){
				end_code[com_mode] = FORMAT_ERROR;	//End Code 14  桁数異常・未入力(フォーマットエラー)
				break;
			}			//範囲外 for-loop 脱出
			if (value[com_mode] < 0 || value[com_mode] > 9999){
				end_code[com_mode] = MAKERLINEAR_VALUE_ERROR;			//End Code 56  流量設定値 数値範囲外入力
				break;
			}			//範囲外 for-loop 脱出
			if (l <= linear_point && value[com_mode] <= linear_check){
				end_code[com_mode] = MAKERLINEAR_ORDER_ERROR;		//End Code 57  流量設定値 順番不適合
				break;
			}			//範囲外 for-loop 脱出
			linear_check = value[com_mode];
			*pt = linear_check % 0x10000;
			pt++;
			linear_check = value[com_mode];
			*pt = linear_check / 0x10000;
			linear_check = value[com_mode];
			pt++;
			check++;
		}
			/* 入力×15 */
				if (check == 15){
					linear_check = -1;	//input1 = 0 許容用
					pt = &SVD[ch_no[com_mode] -1].mklnr_in1.WORD.low;
					for (l=1; l<=15; l++){	//for-loop
					/* 入力値 */
						i_num[com_mode]++;	// ','の1つ後ろへ移動
						value_judge(com_mode,i_num[com_mode],3);
						if (digit_check[com_mode] > 3 || size[com_mode] < 1 || size[com_mode] > 5){ 
							end_code[com_mode] = FORMAT_ERROR;	//End Code 14  桁数異常・未入力(フォーマットエラー)
							break;
						}		//範囲外 for-loop 脱出
						if (value[com_mode] < 0 || value[com_mode] > 9999){ 
							end_code[com_mode] = MAKERLINEAR_VALUE_ERROR;			//End Code 56  流量設定値 数値範囲外入力
							break;
						}		//範囲外 for-loop 脱出
						if (l <= linear_point && value[com_mode] <= linear_check){ 
							end_code[com_mode] = MAKERLINEAR_ORDER_ERROR;		//End Code 57  流量設定値 順番不適合
							break;
						}		//範囲外 for-loop 脱出
						linear_check = value[com_mode];
						*pt = linear_check % 0x10000;
						pt++;
						linear_check = value[com_mode];
						*pt = linear_check / 0x10000;
						linear_check = value[com_mode];
						pt++;
					}						//End Code 00  正常終了15点以内・0.000-9.999・数値入力順正常）
				}
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ end_code[com_mode] = FORMAT_ERROR;}
			else{ end_code[com_mode] = MAKERLINEAR_POINT_ERROR;} 						//End Code 55 = 設定点数異常
}
			
/* ゼロ調整データ書込み(SFC9000対応)(Wz) */
void command_Wz (short com_mode){

	short m;
	short ch_z;			// チャネル数用
	short ch_number;			// チャネル順番判定用
	short check;
	short check_1;
	short check_14;
	unsigned short *pt;
	long value_check;
	char ch[1] = {0};			// チャネル数判定用
	
	check = 0;
	check_1 = 0;
	check_14 = 0;

/* シングルコマンド */
	if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
		ch_z = 1;
	/* 動粘度係数 */
		value_judge(com_mode,7,2);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 4000) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].viscos = value[com_mode];
			/* 薬液リニアライズモードON = 薬液種別判定 */
			if(SVD[ch_no[com_mode] -1].LL_enable == 1){
				LLmode_kind(value[com_mode], ch_no[com_mode] -1);
			}			
			check_1++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
			check_14++;
		}
		i_num[com_mode]++;
	/* 受波波形最大値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_max = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* 受波波形最小値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_min = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* 伝搬時間差 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= -99999999 && value[com_mode] <= 99999999) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD = value[com_mode] / 32;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* FIFO受波波形検出位置 */
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
	/* 受波の差(P1-P2) */
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
/* マルチコマンド */
	else if (ch_no[com_mode] == 0){
		ch[0] = RX_buf[com_mode][7];			//CH数判定
		ch_z = ch[0] - 0x30;
		ch[0] = 0;
		/* 各ｃｈデータ */
		i_num[com_mode] = 9 + 2 * ch_z;			//数値部へ移動：指定数と個数不一致はフォーマットエラー判定部で
		for (m=0; m<ch_z; m++){
			ch[0] = RX_buf[com_mode][9 + 2*m];	//CH番号判定
			ch_number = ch[0] - 0x30;
		/* 動粘度係数 */
			value_judge(com_mode,i_num[com_mode],2);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 4000) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
				SVD[ch_number -1].viscos = value[com_mode];
				/* 薬液リニアライズモードON = 薬液種別判定 */
				if(SVD[ch_number -1].LL_enable == 1){
					LLmode_kind(value[com_mode], ch_number -1);
				}				
				check_1++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
				check_14++;
			}
			i_num[com_mode]++;
		/* 受波波形最大値 */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_wave_max = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* 受波波形最小値 */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= 0 && value[com_mode] <= 5000) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_wave_min = value[com_mode];
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* 伝搬時間差 */
			value_judge(com_mode,i_num[com_mode],0);
			if ( (value[com_mode] >= -99999999 && value[com_mode] <= 99999999) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
				SVD[ch_number -1].zero_delta_ts.DWORD = value[com_mode] / 32;
				check++;
			}
			else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
				check_14++;
			}
			i_num[com_mode]++;
		/* FIFO受波波形検出位置 */
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
		/* 受波の差(P1-P2) */
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
	/* 合否判定 */
	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
	else if (check_1 != ch_z){
		end_code[com_mode] = KVISCOSITY_ERROR;				//End Code 41 = エラー（動粘度係数）
	}
	else if (check != 7 * ch_z){
		end_code[com_mode] = ZADJ_WRITE_ERROR;				//End Code 54 = エラー（ゼロ調整・設定範囲外）
	}
}

/* ゼロ調整データ書込み(WZ) */
 void command_WZ (short com_mode){

	 short m;
	 short ch_z;			// チャネル数用
	 short ch_number;			// チャネル順番判定用
	 short check;
	 short check_1;
	 short check_14;
	 unsigned short *pt;
	 long value_check;
	 char ch[1] = {0};			// チャネル数判定用
	
	 check = 0;
	 check_1 = 0;
	 check_14 = 0;

 /* マルチコマンド */
	 ch[0] = RX_buf[com_mode][7];			//CH数判定
	 ch_z = ch[0] - 0x30;
	 ch[0] = 0;
	 /* 各ｃｈデータ */
	 i_num[com_mode] = 9 + 2 * ch_z;			//数値部へ移動：指定数と個数不一致はフォーマットエラー判定部で
	 for (m=0; m<ch_z; m++){
		 ch[0] = RX_buf[com_mode][9 + 2*m];	//CH番号判定
		 ch_number = ch[0] - 0x30;
	 /* 動粘度係数 */
		 value_judge(com_mode,i_num[com_mode],2);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 4000) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 2){
			 SVD[ch_number -1].viscos = value[com_mode];
				/* 薬液リニアライズモードON = 薬液種別判定 */
				if(SVD[ch_number -1].LL_enable == 1){
					LLmode_kind(value[com_mode], ch_number -1);
				}
			 check_1++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 2){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Tu（該当データなし） */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 150000000) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			 check++;		
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 /* Td（該当データなし） */
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
	 /* Window Position（該当データなし） */
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
	 /* Threshold（該当データなし） */
		 value_judge(com_mode,i_num[com_mode],0);
		 if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			 check++;
		 }
		 else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){
			 check_14++;
		 }
		 i_num[com_mode]++;
	 }
	 /* 合否判定 */
	 if (check_14 > 0){
		 end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	 }
	 else if (check_1 != ch_z){
		 end_code[com_mode] = KVISCOSITY_ERROR;				//End Code 41 = エラー（動粘度係数）
	 }
	 else if (check != 7 * ch_z){
		 end_code[com_mode] = ZADJ_WRITE_ERROR;				//End Code 54 = エラー（ゼロ調整・設定範囲外）
	 }
 }

/* パラメータセット書込み(WP) */
void command_WP (short com_mode){

	short m;
	short ch_z;			// チャネル数用
	short ch_number;			// チャネル順番判定用
	short check_14;
	short check_1;
	short check_2;
	short check_3;	
	char ch[1] = {0};			// チャネル数判定用

	check_1 = 0;
	check_2 = 0;
	check_3 = 0;
	check_14 = 0;
	
	ch[0] = RX_buf[com_mode][7];			//CH数判定
	ch_z = ch[0] - 0x30;
	ch[0] = 0;
	/* 各ｃｈデータ */
	i_num[com_mode] = 9 + 2 * ch_z;			//数値部へ移動：指定数と個数不一致はフォーマットエラー判定部で
	for (m = 0; m < ch_z; m++){
		ch[0] = RX_buf[com_mode][9 + 2*m];	//CH番号判定
		ch_number = ch[0] - 0x30;
	/* フルスケール */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
			SVD[ch_number -1].max_flow = value[com_mode];
			check_1++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 5 || digit_check[com_mode] > 0){
			check_14++;
		}
		i_num[com_mode]++;
	/* ダンピング */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 250 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
			SVD[ch_number -1].damp = value[com_mode];
			check_2++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* ローカット */
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
	/* 合否判定 */
	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
	else if (check_1 != ch_z){
		end_code[com_mode] = FULLSCALE_ERROR;	//End Code 35 = エラー（フルスケール）
	}
	else if (check_2 != ch_z){
		end_code[com_mode] = DUMPING_ERROR;	//End Code 37 = エラー（ダンピング）
	}
	else if (check_3 != ch_z){
		end_code[com_mode] = LOWCUT_ERROR;	//End Code 38 = エラー（ローカット）
	}
			
}
/* 波形異常判定値設定書込み(WE) */
void command_WE (short com_mode){

	short check;
	short check_14;
	short	empty;
	
	check = 0;
	check_14 = 0;
	/* エンプティセンサ判定閾値 */
		value_judge(com_mode,7,0);
		empty = value[com_mode];
		if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].wave_vth = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 演算異常判定閾値 */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 10 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].correlate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* 演算異常判定回数 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].correlate_time = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 波形アンバランス閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].balance_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* AGC不能判定閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].saturation_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 波形減衰判定閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].attenuate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* エンプティセンサ警告閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0 && value[com_mode] < empty){
			SVD[ch_no[com_mode] -1].alm_wave_vth = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* アンプゲイン警告閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].alm_gain_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* アンプゲイン警告ｶｳﾝﾄ */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].alm_gain_count = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 警告判定時間 */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].alm_hold_time = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;

	if (check_14 > 0){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
	else if (check != 10){
		end_code[com_mode] = JUDGEMENT_WAVE_ERROR;				//End Code 50 = 範囲外指定・桁数異常
	}
}

/* アッテネータゲイン書込み(WA) */		//0～255
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

/* 薬液リニアライズモード書込み(Wy) */		//0～1
 void command_Wy (short com_mode){
 
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].LL_enable = value[com_mode];
		
		/* 薬液リニアライズモードON = 薬液種別判定 */
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

/* フィルタ設定書込み(Wf) */
void command_Wf (short com_mode){
	
	/* フィルタモード判定 */	//種別0～2
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 2 && size[com_mode] == 1){
		SVD[ch_no[com_mode] -1].filter_mode = value[com_mode];
	/* 移動平均数値判定 */	//1～50
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

 /* 積算目標値書込み(Wt) */		//0.0～99999.99mL
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

 /* 積算オフセット値書込み(WT) */
void command_WT (short com_mode){
	
	/* 有効無効判定 */	//ON/OFF 0～1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		SVD[ch_no[com_mode] -1].total_offset_enable = value[com_mode];
	/* バーンアウト数値判定 */	//-9.999～9.999mL
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

/* メーカー設定書込み(W1) */
 void command_W1 (short com_mode){

	 short z;
	 short l;
	 short h;
	 short serial_size;
	 short check;
	 short check_14;
	 short serial;
	 unsigned short *pt;
	 char ver[MES_CHAR_MAX] = {0};			// バージョン読込用
	
	 check = 0;
	 check_14 = 0;
	 serial_size = 0;
	 
	 /* 補正入力値判定 */
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
	 /* シリアルナンバー */
		 z = 0;
		 while (RX_buf[com_mode][i_num[com_mode]] != ','){		//データ区切り位置判定
			if(i_num[com_mode] >= (MSG_MAX-1)){		//','が最後まで見つからない場合
				check_14++;
				break;
			}
			ver[z] = RX_buf[com_mode][i_num[com_mode]];
			 serial_size++;
			 i_num[com_mode]++;
			 z++;	
				if(z >= MES_CHAR_MAX){	//配列外書込み防止
					break;
				}
		 }
		 pt = &SVD[ch_no[com_mode] -1].c_serial[0];
		 if (serial_size % 2 == 0){					//文字数偶数時
			 for (z = 0; z < serial_size; z = z+2){
				 serial = ver[z] * 0x0100 + ver[z+1];
				 *pt = serial;
				 pt++;
			 }
			 for (z = serial_size; z < 16; z = z+2){		//16字未満時 空白代入
				 serial = 0x2020;
				 *pt = serial;
				 pt++;
			 }
		 }
		 else{							//文字数奇数時
			 for (z = 0; z < serial_size -1; z = z+2){
				 serial = ver[z] * 0x0100 + ver[z+1];
				 *pt = serial;
				 pt++;
			 }
			 serial = ver[z] * 0x0100 + 0x20;
			 *pt = serial;
			 pt++;
			 for (z = serial_size +1; z < 16; z = z+2){		//16字未満時 空白代入
				 serial = 0x2020;
				 *pt = serial;
				 pt++;
			 }
		 }

		 for (h = 0; h < MES_CHAR_MAX; h++){ver[h] = 0;}
	 }
	 if (check_14 > 0 || serial_size < 0 || serial_size > 16){
		 end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	 }
	 else if (check != 5){
		 end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = 範囲外指定・桁数異常
	 }
 }

/* デバッグモード設定書込み(W2) */
 void command_W2 (short com_mode){

	 /* 流量出力テスト項目判定 */	//種別0～1
	 value_judge(com_mode,7,0);
	 if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		 MES[ch_no[com_mode] -1].test_enable = value[com_mode];		//流量出力テスト
		 i_num[com_mode]++;
	 /* 出力テスト値判定 */			//0.00～300.00%
		 value_judge(com_mode,i_num[com_mode],2);
		 if ( ( value[com_mode] >= 0 && value[com_mode] <= 30000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			 MES[ch_no[com_mode] -1].test_flow = value[com_mode];	//流量出力テスト値
			 i_num[com_mode]++;
	 /* 出力ポートテスト値判定 */	//種別0～1
			 value_judge(com_mode,i_num[com_mode],0);
			 if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
				 MES[ch_no[com_mode] -1].test_port_out = value[com_mode];		//出力ポートテスト
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

/* メーカー設定書込み(SFC9000対応)(W5) */
void command_W5 (short com_mode){

	short z;
	short l;
	short h;
	short serial_size;
	short check;
	short check_14;
	short serial;
	unsigned short *pt;
	char ver[MES_CHAR_MAX] = {0};			// バージョン読込用
	
	check = 0;
	check_14 = 0;
	serial_size = 0;
	/* エンプティセンサ判定閾値 */
		value_judge(com_mode,7,0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 100) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].wave_vth = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 演算異常判定閾値 */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 10 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].correlate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* 演算異常判定回数 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].correlate_time = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 波形アンバランス閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].balance_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* AGC不能判定閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].saturation_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 波形減衰判定閾値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].attenuate_level = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ゲインステップ幅 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 10 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].gain_step = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 音速固定or測定 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].sound_vel_sel = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 音速設定 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 400 && value[com_mode] <= 3000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].sound_vel_fix = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 3 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 音速フィルタ時定数 */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].sound_vel_filter = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* 動粘度固定or測定 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].viscos_auto = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 打込みパルス */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 1 && value[com_mode] <= 8 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].drive_pls = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* アッテネータゲイン値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			//SVD[ch_no[com_mode] -1].atn_gain = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* FIFO CH 初期値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 47 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].fifo_ch_init = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* CUnet 再送信待機時間 */	
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 100 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[0].cunet_delay = value[com_mode];	//※全CH共通
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;		
	/* 相関値サーチ無効or有効 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 1 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].search_sw = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 気泡対策モード */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 4 ) && ( size[com_mode] == 1 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].damp_mode = value[com_mode];
			check++;
		}
		else if ( size[com_mode] != 1 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 相関値幅上限値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].corr_up.DWORD = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 相関値幅下限値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].corr_low.DWORD = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 異常ホールド時間 */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].inc = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* レイトリミットホールドクリア */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].hldt = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* レイトリミット */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].rlt = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* レイトリミット1st */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].odpd = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* ターゲット1st */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].rl1tg = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* アベレージ1st */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].rl1av = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* レイトリミット2nd */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 10000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].odpl = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* ターゲット2nd */
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].rl2tg = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* アベレージ2nd */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].rl2av = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ダンピング倍率 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].rl2d = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ホールドクリア */
		value_judge(com_mode,i_num[com_mode],1);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 1){
			SVD[ch_no[com_mode] -1].rl2hc = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 1){ check_14++;}
		i_num[com_mode]++;
	/* 可変ダンピング */
		value_judge(com_mode,i_num[com_mode],0);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 255) && ( size[com_mode] >= 1 && size[com_mode] <= 3 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].dump_var = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 3 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 可変ダンピング倍率 */
		value_judge(com_mode,i_num[com_mode],2);
		if ( (value[com_mode] >= 0 && value[com_mode] <= 10000) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].dump_mul = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 6 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;

	if (check == 32){
	/* シリアルナンバー */
		z = 0;
		while (RX_buf[com_mode][i_num[com_mode]] != ','){		//データ区切り位置判定
			if(i_num[com_mode] >= (MSG_MAX-1)){		//','が最後まで見つからない場合
				check_14++;
				break;
			}
			ver[z] = RX_buf[com_mode][i_num[com_mode]];
			serial_size++;
			i_num[com_mode]++;
			z++;	
			if(z >= MES_CHAR_MAX){	//配列外書込み防止
				break;
			}
		}
		for(l = 0; l < 6; l++){
			pt = &SVD[l].c_serial[0];
			if (serial_size % 2 == 0){					//文字数偶数時
				for (z = 0; z < serial_size; z = z+2){
					serial = ver[z] * 0x0100 + ver[z+1];
					*pt = serial;
					pt++;
				}
				for (z = serial_size; z < 16; z = z+2){		//16字未満時 空白代入
					serial = 0x2020;
					*pt = serial;
					pt++;
				}
			}
			else{							//文字数奇数時
				for (z = 0; z < serial_size -1; z = z+2){
					serial = ver[z] * 0x0100 + ver[z+1];
					*pt = serial;
					pt++;
				}
				serial = ver[z] * 0x0100 + 0x20;
				*pt = serial;
				pt++;
				for (z = serial_size +1; z < 16; z = z+2){		//16字未満時 空白代入
					serial = 0x2020;
					*pt = serial;
					pt++;
				}
			}
		}
		for (h = 0; h < MES_CHAR_MAX; h++){ver[h] = 0;}
	}
	if (check_14 > 0 || serial_size < 0 || serial_size > 16){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
	else if (check != 32){
		end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = 範囲外指定・桁数異常
	}
}
			
/* デバッグモード設定書込み(SFC9000対応)(W6) */
void command_W6 (short com_mode){

	/* 流量出力テストON/OFF */	//種別0～1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		MES[ch_no[com_mode] -1].test_enable = value[com_mode];
		i_num[com_mode]++;
	/* 出力テスト値判定 */			//0.00～300.00%
		value_judge(com_mode,i_num[com_mode],2);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 30000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 6 ) && digit_check[com_mode] <= 2){
			MES[ch_no[com_mode] -1].test_flow = value[com_mode];// * 100;	//流量出力テスト値
			i_num[com_mode]++;
	/* エラーテストON/OFF */	//種別0～1
			value_judge(com_mode,i_num[com_mode],0);
			if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
				MES[ch_no[com_mode] -1].test_err_enable = value[com_mode];
				i_num[com_mode]++;
	/* 強制エラー種別 */
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

/* ゼロ調整詳細データ書込み(SFC9000対応)(W7) */
void command_W3 (short com_mode){

	short z;
	short l;
	short h;
	short check;
	short check_14;
	short ret;
	unsigned short status;
	char sta[MES_W7_STATUS] = {0};			// ステータス読込用
	short pch = ch_no[com_mode] -1;
	short DgtNum = 0;
	
	check = 0;
	check_14 = 0;
	ret = 0;
	
	// //01. 流量[mL/min]
	
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
	
	// //02. 流速[m/s]
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
	
	// //03. 音速[m/s]
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
	
	// //04. 積算値[mL/min]
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
	
	// /* 受波波形最大値 */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_wave_max = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* 受波波形最小値 */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_wave_min = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* 伝搬時間差[ps] */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= -99999999 && value[com_mode] <= 99999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD = value[com_mode] / 32;
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* 相関値幅 */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_correlate = value[com_mode] / 1000;
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* ゼロ点オフセット */
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
	// 		if(z >= MES_W7_STATUS){	//配列外書込み防止
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
	// /* FIFO受波波形検出位置 */
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
	// /* 受波の差(P1-P2) */
	// 	value_judge(com_mode,i_num[com_mode],0);
	// 	if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
	// 		SVD[ch_no[com_mode] -1].zero_p1p2 = value[com_mode];
	// 		check++;
	// 	}
	// 	else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
	// 	i_num[com_mode]++;
	// /* 差分相関値 */
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
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
	else if (check != 54){
		end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = 範囲外指定・桁数異常
	}
}

/* ゼロ調整詳細データ書込み(SFC9000対応)(W7) */
void command_W7 (short com_mode){

	short z;
	short l;
	short h;
	short check;
	short check_14;
	short ret;
	unsigned short status;
	char sta[MES_W7_STATUS] = {0};			// ステータス読込用
	
	check = 0;
	check_14 = 0;
	ret = 0;
	
		/* 流量[mL/min] */
		value_judge(com_mode,7,2);
		if ( (value[com_mode] >= -3000000 && value[com_mode] <= 3000000) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 2){
			SVD[ch_no[com_mode] -1].zero_flow_qat.DWORD = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 2){ check_14++;}
		i_num[com_mode]++;
	/* 流速[m/s] */
		value_judge(com_mode,i_num[com_mode],3);
		if ( ( value[com_mode] >= -30000 && value[com_mode] <= 30000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 3){
			SVD[ch_no[com_mode] -1].zero_flow_vel.DWORD = value[com_mode] * 10;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 3){ check_14++;}
		i_num[com_mode]++;
	/* 音速[m/s] */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 3000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_sound_spd = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 積算値[mL/min] */
		value_judge(com_mode,i_num[com_mode],3);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99999999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 12 ) && digit_check[com_mode] <= 3){
			SVD[ch_no[com_mode] -1].zero_addit.DWORD = value[com_mode] * 10000;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 12 || digit_check[com_mode] > 3){ check_14++;}
		i_num[com_mode]++;
	/* 受波波形最大値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_max = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 受波波形最小値 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_wave_min = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 4 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 伝搬時間差[ps] */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= -99999999 && value[com_mode] <= 99999999 ) && ( size[com_mode] >= 1 && size[com_mode] <= 9 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_delta_ts.DWORD = value[com_mode] / 32;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 9 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 相関値幅 */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 2550000 ) && ( size[com_mode] >= 1 && size[com_mode] <= 7 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_correlate = value[com_mode] / 1000;
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 7 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* ゼロ点オフセット */
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
			if(z >= MES_W7_STATUS){	//配列外書込み防止
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
	/* FIFO受波波形検出位置 */
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
	/* 受波の差(P1-P2) */
		value_judge(com_mode,i_num[com_mode],0);
		if ( ( value[com_mode] >= 0 && value[com_mode] <= 99 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
			SVD[ch_no[com_mode] -1].zero_p1p2 = value[com_mode];
			check++;
		}
		else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){ check_14++;}
		i_num[com_mode]++;
	/* 差分相関値 */
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
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
	else if (check != 54){
		end_code[com_mode] = MAKERSET_ERROR;				//End Code 49 = 範囲外指定・桁数異常
	}
}

/* パルス出力(W9) */
void command_W9 (short com_mode){
	
	short pls_ch;
	short test_mode = 0;

	/* パルス出力テストON/OFF */	// 種別0～1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
		for (pls_ch = 0; pls_ch < 6; pls_ch++){
			MES[pls_ch].pls_test_enable = value[com_mode];		// パルス出力ON/OFF切り替え
			MES[pls_ch].pls_test_on = 0;			// 全CHパルスOFF
		}
		if (value[com_mode] == 1){
			test_mode = 1;					// ON時処理用
		}
		i_num[com_mode]++;
	/* パルス1出力CH選択*/			// 0(None),1(CH1)～6(CH6)
		value_judge(com_mode,i_num[com_mode],0);
		if (value[com_mode] >= 0 && value[com_mode] <= 6 && size[com_mode] == 1) {
			for (pls_ch = 0; pls_ch < 6; pls_ch++){
				MES[pls_ch].pls_test_ch1 = value[com_mode];	// 選択CH No.1
			}
			if (value[com_mode] != 0 && test_mode == 1){		// 出力ON＆CH選択時(None以外)
				MES[(short)value[com_mode] -1].pls_test_on = 1;	// 選択CH No.1パルスON
			}
			i_num[com_mode]++;
	/* パルス2出力CH選択*/			// 0(None),1(CH1)～6(CH6)
			value_judge(com_mode,i_num[com_mode],0);
			if (value[com_mode] >= 0 && value[com_mode] <= 6 && size[com_mode] == 1){
				for (pls_ch = 0; pls_ch < 6; pls_ch++){
					MES[pls_ch].pls_test_ch2 = value[com_mode];	// 選択CH No.2
				}
				if (value[com_mode] != 0 && test_mode == 1){		// 出力ON＆CH選択時(None以外)
					MES[(short)value[com_mode] -1].pls_test_on = 1;	// 選択CH No.2パルスON
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

/* 検査モード書込み(SFC9000対応)(Wi) */
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

/* センサパルス設定書込み(SFC014E互換)(Wp) */
void command_Wp (short com_mode){

	/* 打込みパルス */  //1～15
	value_judge(com_mode,7,0);
	if ( ( value[com_mode] >= 1 && value[com_mode] <= 15 ) && ( size[com_mode] >= 1 && size[com_mode] <= 2 ) && digit_check[com_mode] <= 0){
		SVD[ch_no[com_mode] -1].drive_pls = value[com_mode];
 	i_num[com_mode]++;
 /* 調整モード */  //0～1
 	value_judge(com_mode,i_num[com_mode],0);
		if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
 	 SVD[ch_no[com_mode] -1].drive_search = value[com_mode];
  	i_num[com_mode]++;
	/* 駆動周波数 */  //100～5000kHz
 	 value_judge(com_mode,i_num[com_mode],0);
	  if ( ( value[com_mode] >= 100 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		  SVD[ch_no[com_mode] -1].drive_freq = value[com_mode];
  	 i_num[com_mode]++;
	/* サーチ開始周波数 */  //100～5000kHz
 	  value_judge(com_mode,i_num[com_mode],0);
	   if ( ( value[com_mode] >= 100 && value[com_mode] <= 5000 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
		   SVD[ch_no[com_mode] -1].start_freq = value[com_mode];
  	  i_num[com_mode]++;
	/* サーチ停止周波数 */  //100～5000kHz
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

/* センサシリアルナンバー書込み(SFC014E互換)(Wn) */
void command_Wn (short com_mode){

	short z;
	short l;
	short h;
	short serial;
	short serial_size;
	short check;
	unsigned short *pt;
	char ver[MES_CHAR_MAX] = {0};			// バージョン読込用

	z = 0;
	check = 0;
	serial_size = 0;
		/* センサシリアルナンバー */
 i_num[com_mode] = 7;
//	value_judge(com_mode,7,0);
	while (RX_buf[com_mode][i_num[com_mode]] != ','){		//データ区切り位置判定
		if(i_num[com_mode] >= (MSG_MAX-1)){		//','が最後まで見つからない場合
			check++;
			break;
		}
		ver[z] = RX_buf[com_mode][i_num[com_mode]];
		serial_size++;
		i_num[com_mode]++;
		z++;	
		if(z >= MES_CHAR_MAX){	//配列外書込み防止
			break;
		}
	}
//	for(l = 0; l < 6; l++){
		pt = &SVD[ch_no[com_mode] -1].s_serial[0];
		if (serial_size % 2 == 0){					//文字数偶数時
			for (z = 0; z < serial_size; z = z+2){
				serial = ver[z] * 0x0100 + ver[z+1];
				*pt = serial;
				pt++;
			}
			for (z = serial_size; z < 16; z = z+2){		//16字未満時 空白代入
				serial = 0x2020;
				*pt = serial;
				pt++;
			}
		}
		else{							//文字数奇数時
			for (z = 0; z < serial_size -1; z = z+2){
				serial = ver[z] * 0x0100 + ver[z+1];
				*pt = serial;
				pt++;
			}
			serial = ver[z] * 0x0100 + 0x20;
			*pt = serial;
			pt++;
			for (z = serial_size +1; z < 16; z = z+2){		//16字未満時 空白代入
				serial = 0x2020;
				*pt = serial;
				pt++;
			}
		}
//	}
	for (h = 0; h < MES_CHAR_MAX; h++){ver[h] = 0;}

	if (check > 0 || serial_size < 0 || serial_size > 16){
		end_code[com_mode] = FORMAT_ERROR;	//End Code 14 = フォーマットエラー
	}
}

/****************************************************
 * Function : GetValSiz (Get Value Size)
 * Summary  : Valの桁数を取得する
 * Argument : Val : 対象の値
 * Return   : Siz : 桁数
 * Caution  : 
 * notes    : 負数の場合は-も1桁とカウントする
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
 * Summary  : 要求が来たパラメータの値を判定、更新する
 * Argument : com_mode : 
 *            Prm : パラメータのメモリアドレス
 *            MinVal : 入力可能レンジ最小値
 *            MaxVal : 入力可能レンジ最大値
 *            pow : 小数点以下桁数
 *            ErrCod : エラーコード
 * Return   : Flg :  0 正常終了
 *                  -1 フォーマットエラー
 *                  -2 入力レンジ外エラー
 * Caution  : 
 * notes    : 
 ****************************************************/
short CngSstPrm(short com_mode, short *Prm, long long MinVal, long long MaxVal, short pow, short ErrCod)
{
	short Flg = 0;
	short MinSiz = GetValSiz(MinVal);
	short MaxSiz = GetValSiz(MaxVal);

	//小数点以下桁数を使用する場合は.の分1増やす
	if(pow > 0)
	{
		MaxSiz += 1;
	}

	//value[], size[], digit_check[] を取得する
	value_judge(com_mode, i_num[com_mode], pow);

	//文字列長判定
	if(size[com_mode] < MinSiz || MaxSiz < size[com_mode])
	{
		Flg = -1; //FORMAT ERROR
		end_code[com_mode] = FORMAT_ERROR;
	}
	//入力範囲判定
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
 * Summary  : 要求が来たパラメータの値を判定、更新する
 * Argument : com_mode : 
 *            Prm : パラメータのメモリアドレス
 *            MinVal : 入力可能レンジ最小値
 *            MaxVal : 入力可能レンジ最大値
 *            pow : 小数点以下桁数
 *            ErrCod : エラーコード
 * Return   : Flg :  0 正常終了
 *                  -1 フォーマットエラー
 *                  -2 入力レンジ外エラー
 * Caution  : 
 * notes    : 
 ****************************************************/
short CngUstPrm(short com_mode, unsigned short *Prm, long long MinVal, long long MaxVal, short pow, short ErrCod)
{
	short Flg = 0;
	short MinSiz = GetValSiz(MinVal);
	short MaxSiz = GetValSiz(MaxVal);

	//小数点以下桁数を使用する場合は.の分1増やす
	if(pow > 0)
	{
		MaxSiz += 1;
	}

	//value[], size[], digit_check[] を取得する
	value_judge(com_mode, i_num[com_mode], pow);
	//文字列長判定
	if(size[com_mode] < MinSiz || MaxSiz < size[com_mode])
	{
		Flg = -1; //FORMAT ERROR
		end_code[com_mode] = FORMAT_ERROR;
	}
	//入力範囲判定
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

/* センサ情報書込み(評価用)(Wa) */
void command_Wa (short com_mode){
	short pch = ch_no[com_mode] -1;
	short Flg = 0;
#if 1
	i_num[com_mode] = 7;
	//1. センサオプション
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_option, 0, 1, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//2. センサ間距離(L)
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_disL, 100, 9999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//3. センサ間距離(L_l)
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_disL_l, 100, 9999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//4. 無駄時間
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_tau, 100, 9999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//5. 互換係数
	Flg = CngUstPrm(com_mode, &SVD[pch].sns_coef, 1000, 99999, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//6. ADCクロック
	Flg = CngUstPrm(com_mode, &SVD[pch].adc_clock, 0, 3, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//7. WINDOW OFFSET
	Flg = CngUstPrm(com_mode, &SVD[pch].wind_offset, 0, 63, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//8. 差分相関開始位置
	Flg = CngUstPrm(com_mode, &SVD[pch].sum_start, 4, 210, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//9. 差分相関終了位置
	Flg = CngUstPrm(com_mode, &SVD[pch].sum_end, 5, 210, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//10. 差分相関間隔
	Flg = CngUstPrm(com_mode, &SVD[pch].sum_step, 1, 10, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//11. 固定値設定
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_data, 0x0000, 0xFFFF, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//12. Wiper Position(固定値)
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_amp_gain_rev, 0, 255, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//13. FIFO CH(固定値)
	Flg = CngUstPrm(com_mode, &SVD[pch].fix_fifo_ch_read, 0, 63, 0, FREQUENCY_ERROR);
	if(Flg != 0) return;

	//14. Leading Position(固定値)
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
 /* センサオプション */  //0～1
	value_judge(com_mode,7,0);
	if (value[com_mode] >= 0 && value[com_mode] <= 1 && size[com_mode] == 1){
	 SVD[ch_no[com_mode] -1].sns_option = value[com_mode];
 	i_num[com_mode]++;
	/* センサ間距離(L) */  //100～9999*0.1mm
	 value_judge(com_mode,i_num[com_mode],0);
  if ( ( value[com_mode] >= 100 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	  SVD[ch_no[com_mode] -1].sns_disL = value[com_mode];
 	 i_num[com_mode]++;
	/* センサ間距離(L_l) */  //100～9999*0.1mm
	 value_judge(com_mode,i_num[com_mode],0);
  if ( ( value[com_mode] >= 100 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	  SVD[ch_no[com_mode] -1].sns_disL_l = value[com_mode];
 	 i_num[com_mode]++;
	/* 無駄時間 */  //100～9999
	  value_judge(com_mode,i_num[com_mode],0);
   if ( ( value[com_mode] >= 100 && value[com_mode] <= 9999 ) && ( size[com_mode] >= 3 && size[com_mode] <= 4 ) && digit_check[com_mode] <= 0){
	   SVD[ch_no[com_mode] -1].sns_tau = value[com_mode];
 	  i_num[com_mode]++;
	/* 互換係数 */  //1000～99999
 	  value_judge(com_mode,i_num[com_mode],0);
	   if ( ( value[com_mode] >= 1000 && value[com_mode] <= 99999 ) && ( size[com_mode] >= 4 && size[com_mode] <= 5 ) && digit_check[com_mode] <= 0){
		   SVD[ch_no[com_mode] -1].sns_coef = value[com_mode];
  	  i_num[com_mode]++;
	/* adc_clock */  //0～3
  	  value_judge(com_mode,i_num[com_mode],0);
	    if ( ( value[com_mode] >= 0 && value[com_mode] <= 3 ) && size[com_mode] == 1 ){
		    SVD[ch_no[com_mode] -1].adc_clock = value[com_mode];
  	   i_num[com_mode]++;
	/* WINDOW OFFSET */  //0～63
   	  value_judge(com_mode,i_num[com_mode],0);
	     if ( ( value[com_mode] >= 0 && value[com_mode] <= 63 ) && size[com_mode] <= 2 ){
		     SVD[ch_no[com_mode] -1].wind_offset = value[com_mode];
  	    i_num[com_mode]++;
	/* 差分相関開始位置 */  //4～210
    	  value_judge(com_mode,i_num[com_mode],0);
	      if ( ( value[com_mode] >= 4 && value[com_mode] <= 210 ) && size[com_mode] <= 3 ){
		      SVD[ch_no[com_mode] -1].sum_start = value[com_mode];
  	     i_num[com_mode]++;
	/* 差分相関終了位置 */  //5～210
     	  value_judge(com_mode,i_num[com_mode],0);
	       if ( ( value[com_mode] >= 5 && value[com_mode] <= 210 ) && size[com_mode] <= 3 ){
		       SVD[ch_no[com_mode] -1].sum_end = value[com_mode];
  	      i_num[com_mode]++;
	/* 差分相関間隔 */  //1～10
      	  value_judge(com_mode,i_num[com_mode],0);
	        if ( ( value[com_mode] >= 1 && value[com_mode] <= 10 ) && size[com_mode] <= 2 ){
		        SVD[ch_no[com_mode] -1].sum_step = value[com_mode];
  	       i_num[com_mode]++;
										
	/* 固定値設定 */  //0x0000～0xFFFF
      	   value_judge(com_mode,i_num[com_mode],0);
	         if ( ( value[com_mode] >= 0x0000 && value[com_mode] <= 0xFFFF ) && size[com_mode] <= 5 ){
		         SVD[ch_no[com_mode] -1].fix_data = value[com_mode];
  	        i_num[com_mode]++;
	/* Wiper Position(固定値) */  //1～255
       	   value_judge(com_mode,i_num[com_mode],0);
	          if ( ( value[com_mode] >= 0 && value[com_mode] <= 255 ) && size[com_mode] <= 3 ){
		          SVD[ch_no[com_mode] -1].fix_amp_gain_rev = value[com_mode];
  	         i_num[com_mode]++;
	/* FIFO CH(固定値) */  //0～63
       	    value_judge(com_mode,i_num[com_mode],0);
	           if ( ( value[com_mode] >= 0 && value[com_mode] <= 63 ) && size[com_mode] <= 2 ){
		           SVD[ch_no[com_mode] -1].fix_fifo_ch_read = value[com_mode];
  	          i_num[com_mode]++;
	/* Leading Position(固定値) */  //0～1000
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
/* Zero Cross Use Number ここまで */  

				}else if ( size[com_mode] < 1 || size[com_mode] > 2 || digit_check[com_mode] > 0){
				  end_code[com_mode] = FORMAT_ERROR;
				}else {
				  end_code[com_mode] = FREQUENCY_ERROR;
				}
/* Zero Cross Start Point ここまで */

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

/* 設定値保存(OS) */
void command_OS (short com_mode){

	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}

	action_status_control(ch_no[com_mode]-1,ACT_STS_WRITE);

	util_eep_allwrite(ch_no[com_mode]-1, WR_ALL);		//EEPROM書込み
}

/* センサメモリ設定値保存(Os) */
void command_Os (short com_mode){
	short pch = ch_no[com_mode] - 1;
	
	//既に書き込み要求あり
	if(OWwrite[pch] != 0)
	{
		end_code[com_mode] = EEPROMWRITE_NOW; //EEPROM書き込み中エラーで代用
		return;
	}

	action_status_control(pch, ACT_STS_WRITE);

	util_SnsMem_Write(pch, WR_DEVICE); //センサデバイス書込み
}

/* ゼロ調整値保存(OJ) */
void command_OJ (short com_mode){

	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}

	action_status_control(ch_no[com_mode]-1,ACT_STS_WRITE);

	util_eep_zerowrite(ch_no[com_mode]-1);		//EEPROM書込み
}

/* ログクリア(OL) */
void command_OL (short com_mode){

	log_detailinfo_init(ch_no[com_mode]-1);		/*ログ情報初期化*/
}

/* ＲＡＭクリア(OR) */
void command_OR (void){

	ram_clear_debug();
}

/* ＬＥＤ全点灯(OT) */
void command_OT (void){

	if(led_flash == 0){
		led_flash = 0x1000;
	}
	else{
		led_flash = 0;
	}
}

/* アラーム強制出力(OA) */
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

/* 積算値リセット(Ot) */
void command_Ot (short com_mode){

	MES[ch_no[com_mode] -1].addit_unit_ov.UINT64 = 0;
	MES[ch_no[com_mode] -1].addit_mod_ov = 0;
	MES[ch_no[com_mode] -1].addit_buff_ov.UINT64 = 0;
	
	MES[ch_no[com_mode] -1].total_status &= ~TTL_JUDGE_REACH;		//積算到達出力リセット
	MES[ch_no[com_mode] -1].total_status &= ~TTL_JUDGE_OVERFLOW;	//積算値オーバーフロークリア
	
	if(MES[ch_no[com_mode] -1].addit_watch == B_ON){		//積算監視有効
		if(MES[ch_no[com_mode] -1].addit_unit_ov.UINT64 != 0	//積算関連データがクリアしていない場合
			|| MES[ch_no[com_mode] -1].addit_mod_ov != 0
			|| MES[ch_no[com_mode] -1].addit_buff_ov.UINT64 != 0){
			MES[ch_no[com_mode] -1].total_status |= TTL_JUDGE_CACL_ERR;		//積算値演算異常
		}
	}
}

/* ゼロ調整実行(OZ) */
void command_OZ (short com_mode){

	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}
 SAVE[ch_no[com_mode] -1].control |= 0x0001;
}

/*******************************************
 * Function : command_Wc
 * Summary  : デジタルフィルタ係数書き込み
 * Argument : int com_mode -> 0 : ホスト
 *                            1 : メンテ
 * Return   : void
 * Caution  : なし
 * Note     : @Wc000,n,dddd
 *          :   n : デジタルフィルタ係数のインデックス(偶数:整数下位16bit, 奇数:整数上位2bit)
 *          :   dddd : 書き込むデータ
 * *****************************************/
void command_Wc(short com_mode){
	short Idx, i;
	long Val;
	unsigned short MinVal, MaxVal;

	//1つ目のデータ取得
	i_num[com_mode] = 7;
	value_judge(com_mode, i_num[com_mode], 0);
	Idx = value[com_mode];

	//2つ目のデータ取得
	i_num[com_mode]++;
	value_judge(com_mode, i_num[com_mode], 0);
	Val = value[com_mode];

	//デジタルフィルタスイッチ切り替え
	if(Idx == 20){
		MinVal = 0;
		MaxVal = 0xFFFF;
	}
	//整数書き込み(最大16bit)
	else if(Idx % 2 == 0){
		MinVal = 0;
		MaxVal = 0xFFFF;
	}
	//符号/整数書き込み(最大2bit)
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
		case 0:  FPGA_FILIN0_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA00 = Val; break;	/* デジタルフィルタ係数(入力側0) */
		case 1:  FPGA_FILIN0_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA01 = Val; break;	/* デジタルフィルタ係数(入力側0) */
		case 2:  FPGA_FILIN1_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA10 = Val; break;	/* デジタルフィルタ係数(入力側1) */
		case 3:  FPGA_FILIN1_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA11 = Val; break;	/* デジタルフィルタ係数(入力側1) */
		case 4:  FPGA_FILIN2_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA20 = Val; break;	/* デジタルフィルタ係数(入力側2) */
		case 5:  FPGA_FILIN2_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA21 = Val; break;	/* デジタルフィルタ係数(入力側2) */
//		case 6:  FPGA_FILCOE_A3_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA30 = Val; break;
//		case 7:  FPGA_FILCOE_A3_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA31 = Val; break;
//		case 8:  FPGA_FILCOE_A4_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA40 = Val; break;
//		case 9:  FPGA_FILCOE_A4_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefA41 = Val; break;
		// case 10: FPGA_FIL_EN = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB00 = Val; break;	/* デジタルフィルタ有効・無効設定 */
//		case 11: FPGA_FILCOE_B0_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB01 = Val; break;
		case 12: FPGA_FILOUT1_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB10 = Val; break;	/* デジタルフィルタ係数(出力側1) */
		case 13: FPGA_FILOUT1_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB11 = Val; break;	/* デジタルフィルタ係数(出力側1) */
		case 14: FPGA_FILOUT2_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB20 = Val; break;	/* デジタルフィルタ係数(出力側2) */
		case 15: FPGA_FILOUT2_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB21 = Val; break;	/* デジタルフィルタ係数(出力側2) */
//		case 16: FPGA_FILCOE_B3_0 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB30 = Val; break;
//		case 17: FPGA_FILCOE_B3_1 = Val; for(i=0; i<6; i++) SVD[i].DgtFltCefB31 = Val; break;
		case 20: FPGA_FIL_EN = Val; for(i=0; i<6; i++) SVD[i].DgtFltSwc = Val; break;	/* デジタルフィルタ有効・無効設定 */
		default:
			break;
		}
	}
}

void read_command_W(short com_mode){

	switch (RX_buf[com_mode][2]){		//ヘッダーコード末尾
/* 口径書込み(Wg) */
	case 'g':
		command_Wg(com_mode);
		break;
/* フルスケール書込み(Wr) */
	case 'r':
		command_Wr(com_mode);
		break;
/* Ｋファクタ書込み(Wk) */
	case 'k':
		command_Wk(com_mode);
		break;
/* ダンピング書込み(Wd) */
	case 'd':
		command_Wd(com_mode);
		break;
/* ローカット書込み(Wl) */
	case 'l':
		command_Wl(com_mode);
		break;
/* バーンアウト書込み(Wb) */
	case 'b':
		command_Wb(com_mode);
		break;
/* 動粘度係数書込み(Wv) */
	case 'v':
		command_Wv(com_mode);
		break;
/* エラーホールドタイム書込み(Wh) */
	case 'h':
		command_Wh(com_mode);
		break;
/* 逆流判定値書込み(WR) */
	case 'R':
		command_WR(com_mode);
		break;
/* ユーザーリニアライズ書込み(Wu) */
	case 'u':
		command_Wu(com_mode);
		break;
/* ユーザーリニアライズ切り替え(WU) */		
	case 'U':
		command_WU(com_mode);
		break;
/* メーカーリニアライズ書込み(Wm) */
	case 'm':
		command_Wm(com_mode);
		break;
/* ゼロ調整データ書込み(SFC9000対応)(Wz) */	
	case 'z':
		command_Wz(com_mode);
		break;		
/* ゼロ調整データ書込み(WZ) */
	/* マルチコマンド */
	case 'Z':
		command_WZ(com_mode);
		break;
/* パラメータセット書込み(WP) */
	/* マルチコマンド */
	case 'P':
		command_WP(com_mode);
		break;
/* 波形異常判定値設定書込み(WE) */
	case 'E':
		command_WE(com_mode);
		break;
/* アッテネータゲイン書込み(WA) */
	case 'A':
		command_WA(com_mode);
		break;
/* 薬液リニアライズモード書込み(Wy) */
	case 'y':
		command_Wy(com_mode);
		break;
/* フィルタ設定書込み(Wf) */
	case 'f':
		command_Wf(com_mode);
		break;
/* 積算目標値書込み(Wt) */
	case 't':
		command_Wt(com_mode);
		break;	
/* 積算オフセット値書込み(WT) */
	case 'T':
		command_WT(com_mode);
		break;		
/* メーカー設定書込み(W1) */
	case '1':
		command_W1(com_mode);
		break;
/* デバッグモード設定書込み(W2) */
	case '2':
		command_W2(com_mode);
		break;
/* メーカー設定書込み(SFC9000対応)(W5) */
	case '5':
		command_W5(com_mode);
		break;		
/* デバッグモード設定書込み(SFC9000対応)(W6) */
	case '6':
		command_W6(com_mode);
		break;
/* ゼロ調整詳細データ書込み(SFC9000対応)(W7) */
	case '7':
		command_W7(com_mode);
		break;
/* パルス出力(W9) */
	case '9':
		command_W9(com_mode);
		break;		
		/* 検査モード書込み(SFC9000対応)(Wi) */
	case 'i':
		command_Wi(com_mode);
		break;
/* センサパルス設定書込み(Wp) */
	case 'p':
		command_Wp(com_mode);
		break;
/* センサシリアルナンバー書込み(Wn) */
	case 'n':
		command_Wn(com_mode);
		break;		
/* センサ情報書込み(WL)(評価用) */
	case 'a':
		command_Wa(com_mode);
		break;
/* デジタルフィルタ係数 */
	case 'c':
		command_Wc(com_mode);
		break;
	}
	
}

#ifdef MEMDBG
//Debug Function
/*******************************************
 * Function : MemoryCom
 * Summary  : Mxコマンド分岐
 * Argument : int com_mode -> 0 : ホスト
 *                            1 : メンテ
 * Return   : void
 * Caution  : なし
 * Note     :
 * *****************************************/
void MemoryCom(short com_mode){
	switch (RX_buf[com_mode][2])
	{
	case 'R':
		/* メモリ読み出し */
		command_MR(com_mode);
		break;
	case 'W':
		/* メモリ書き込み */
		command_MW(com_mode);
		break;
	case 'T':
		command_MT(com_mode);
		break;
	case 'Z':
		command_MZ(com_mode);
		break;
	case 'S':
		/* メモリデバイスシリアルナンバー読出し */
		command_MS(com_mode);
		break;
	default:
		break;
	}
}

/* Debug (MR) */
/*******************************************
 * Function : command_MR
 * Summary  : メモリ読み出しコマンド(MR)
 * Argument : int com_mode -> 0 : ホスト
 *                            1 : メンテ
 * Return   : void
 * Caution  : なし
 * Note     : MR00x,メモリアドレス,読み出し個数(最大0x80)
 * *****************************************/
void command_MR (short com_mode){
	long Add;
	long Cnt;
	short i = 0;
	unsigned char pAdd;
	long TxCnt = 9;
	short TmpChr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}; 
	value_judge_Hex(com_mode,7,0);

	//1パラメータ目、アドレス
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

	    //2パラメータ目、個数
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
	 * Summary  : メモリ書き込みコマンド(MW)
	 * Argument : int com_mode -> 0 : ホスト
	 *                            1 : メンテ
	 * Return   : void
	 * Caution  : なし
	 * Note     : MW00x,メモリアドレス,書き込みバイト数(最大0x04),書き込みデータ
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

	//書き込みアドレス
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

	    //書き込みバイト数
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

	        //書き込みデータ
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
					//1,2,4しか受け付けない
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
 * Summary  : 初期ゼロ調実施コマンド(OF)
 * Argument : int com_mode -> 0 : ホスト
 *                            1 : メンテ
 * Return   : void
 * Caution  : なし
 * Note     : 014E互換コマンド
 *            初期ゼロ調の流れ
 *            1. OFコマンド実施
 *            2. 周波数調整(振幅が最大になる周波数を探す)
 *            3. 通常セロ調
 * *****************************************/
void command_OF (short com_mode){
	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		end_code[com_mode] = EEPROMWRITE_NOW;
		return;
	}
	FrqSch[ch_no[com_mode] -1].FrqSchSttFlg = 1;
}
#endif

/*** FPGAダウンロード関連 ***/
extern void memcpy_dl(void *buf1, const void *buf2, size_t n);
// extern short FROM_WRITE(void *ptr, unsigned short data[]);
extern short FPGA_WRITE(void *ptr, unsigned short data[]);
extern void block_erase(void *ptr);
extern void memsetW(unsigned short *dst, unsigned short sdata, unsigned long size);
unsigned short WrtDat[128];
/*******************************************
 * Function : CvtDecAsc
 * Summary  : 10進数のchar変換
 * Argument : DecNum -> 変換したい10進数データ
 * Return   : ChrDat -> 変換したasciiデータ
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
 * Summary  : Asciiの10進数変換
 * Argument : ChrDat -> 変換したいasciiデータ
 * Return   : DecNum -> 変換した10進数データ
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
 * Summary  : Flashデータのブロック単位の消去
 * Argument : Add -> Flash領域アドレス
 * Return   : void
 * Caution  : None
 * Note     : フラッシュ領域は0x4000単位でブロック化されている
 * *****************************************/
void ErsBlk(unsigned long Add)
{
	// 該当ブロック消去(0x4000単位)
	block_erase((void *)Add);
}
/*******************************************
 * Function : ErsAllBlk (Erase All Blocks)
 * Summary  : FPGAデータの消去
 * Argument : None
 * Return   : void
 * Caution  : None
 * Note     : フラッシュ領域は0x4000単位でブロック化されている
 *          : FPGA Configデータの格納場所は0x00040000-0x00100000
 * *****************************************/
void ErsAllBlk()
{
	unsigned long Add = 0x00040000;
	for(Add = 0x00040000; Add < 0x00100000; Add += 0x4000){
		// 該当ブロック消去(0x4000単位)
		block_erase((void *)Add);
	}
}
/*******************************************
 * Function : DlRead
 * Summary  : Flashデータの読み出し
 * Argument : Add -> Flash領域アドレス
 *          : *TbfPtr -> Tx_bufのポインタ
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
 * Summary  : Flashデータの書き込み
 * Argument : Add -> Flash領域アドレス
 *          : *TbfPtr -> Tx_bufのポインタ
 *          : RbfAdd -> RX_bufのデータ格納部分の先頭アドレス
 * Return   : void
 * Caution  : 1. 書き込み前にブロック単位のデータの消去が必要
 *          : 2. 書き込みは128byte単位でおこなう
 *          : 3. ブロックをまたいだデータ書き込みは未対応
 *          : 4. 0x00040000から0x100単位の書き込みしか対応していない
 * Note     : 
 * *****************************************/
void DlWrite(char *TbfPtr, unsigned long RbfAdd, unsigned long Add)
{
	short BufOfs = 0; //バッファオフセット
	short WrtFlg = 0; //書き込み成功フラグ

	short WrtSiz = 0; //書き込みサイズ
	short i;
	short RbfSiz = 0; //受信データサイズ
	short SftLst[] = {0x10, 0x1, 0x1000, 0x100}; //データシフトリスト
	char *RbfPtr = (char *)RbfAdd; //受信データポインタ
	short *DatPtr = (short *)&WrtDat[0];
#if 0

	//通信バッファから書き込みバッファへのコピー
	BufOfs = (short)(Add & 0x000000FFUL);

	if(BufOfs == 0){
		//書き込みバッファの初期化
		memsetW(WrtDat, 0xffff, 128);
	}

	// memcpy_dl((void*)((unsigned long)(&WrtDat[0]) + BufOfs), (void*)(RbfAdd), WrtSiz);
	memcpy_dl((void*)((unsigned long)(&WrtDat[0]) + BufOfs), (void*)(RbfAdd), 0x100);
#else
	//通信バッファから書き込みバッファへのコピー
	BufOfs = (short)(Add % 0x100);
	
	//書き込みバッファの初期化
	memsetW(WrtDat, 0x0000, 128);

	//受信データ長取得 (マイナス(オーバーフロー), 512(想定上の最大サイズ)以上の場合は512固定)
	RbfSiz = strlen((void *)RbfAdd) - 1; //末尾の,分-1
	if((RbfSiz < 0) || (512 <= RbfSiz)) 
	{
		RbfSiz = 512;
	}
#if 1
	//受信バッファから書き込みバッファへコピー
	for(i=0; i<RbfSiz; i++)
	{
		WrtDat[i / 4] += CvtAscDec(*(RbfPtr + i)) * SftLst[i % 4];
	}
	//受信バッファから書き込みバッファへコピー(余り)
	for(i=RbfSiz; i<512; i++)
	{
	    WrtDat[i / 4] += 0x0F * SftLst[i % 4];
	}
#else
	//受信バッファから書き込みバッファへコピー
	for(i=0; i<RbfSiz; i++)
	{
		*(WrtDat + (i / 4)) += CvtAscDec(*(RbfPtr + i)) * SftLst[i % 4];
	}
	//受信バッファから書き込みバッファへコピー(余り)
	for(i=RbfSiz; i<512; i++)
	{
	    *(WrtDat + (i / 4)) += 0x0F * SftLst[i % 4];
	}
#endif

#endif
	//書き込みバッファからFlash領域への書き込み
	WrtFlg = FPGA_WRITE((void *)(Add & 0xFFFFFF00UL), WrtDat);

	//書き込み成功
	if(WrtFlg == 0)
	{
		*(TbfPtr++) = ',';
		*(TbfPtr++) = 'O';
		*(TbfPtr++) = 'K';
		*(TbfPtr++) = ',';
	}
	//書き込み失敗
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
 * Summary  : FPGAダウンロードコマンド(Dl)
 * Argument : int com_mode -> 0 : ホスト
 *                            1 : メンテ
 * Return   : void
 * Caution  : なし
 * Note     : @Dl000,n,xxxxxxxx,yyyyy...[128byte]
 *          :   n -> 0 : ブロック消去
 *          :        1 : Flash領域のデータリード
 *          :        2 : Flash領域のデータライト
 *          :   xxxxxxxx -> アドレス
 *          :   yyy... -> 最大128byteのデータ
 * *****************************************/
void command_Dl (short com_mode){
	unsigned long Add, TmpAdd;
	short i, ReqTyp;
	char *TbfPtr;

	TbfPtr = (char *)&TX_buf[com_mode][8];

	//valueを取得
	value_judge_Hex(com_mode,7,0);
	ReqTyp = (short)value[com_mode];

	//0-2のみ受付
	if((ReqTyp < 0) || (2 < ReqTyp))
	{
		end_code[com_mode] = FORMAT_ERROR;
	}
	else{
		//1, 2の場合はアドレスの指定が必要
	    if(ReqTyp != 0)
	    {
	        i_num[com_mode]++;
	        value_judge_Hex(com_mode, i_num[com_mode], 0);
	        Add = (unsigned long)value[com_mode];
	    }

		//0の場合はアドレス指定不要
	    if(ReqTyp == 0)
	    {
			ErsAllBlk();
            *(TbfPtr++) = ',';
			*(TbfPtr++) = 'O';
			*(TbfPtr++) = 'K';
			*(TbfPtr++) = ',';
		}
		//書き込みアドレス範囲
		else if((Add < 0x00040000) || (0x00100000 < Add))
		{
			end_code[com_mode] = FORMAT_ERROR;
		}
		//アドレスは8桁
		else if((size[com_mode] != 0x08))
		{
			end_code[com_mode] = FORMAT_ERROR;
		}
		else
		{
			//返送コマンドにアドレスをコピー
			*(TbfPtr++) = ',';
			TmpAdd = Add;
			for(i=0; i<8; i++)
			{
				*(TbfPtr++) = (char)(CvtDecAsc(TmpAdd / 0x10000000));
				TmpAdd <<= 4;
			}

			switch (ReqTyp)
			{
			// case 0: //ブロック消去 (上部で実施)
			// 	break;
			case 1: //データリード
				DlRead(TbfPtr, Add);
				break;
			case 2: //データライト準備
				//書き込みデータ格納場所は18以降
				DlWrite(TbfPtr, (unsigned long)(&RX_buf[com_mode][18]), Add);
				break;
			
			default: //来ないはずだが念のため
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
 * Summary  : 数値を指定した桁数でTX_bufに格納する
 * Argument : 
 * Return   : void
 * Caution  : None
 * Note     : 数値先頭の0も無視しない
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
		//1点あたり30文字弱, 500byte/30~16 マージンを見て15
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

	//引数が0の時の値を保持する
	if(Mod == 0)
	{
		MES[pch].ZcLog.Flw = MES[pch].ml_min_now; //流量
		MES[pch].ZcLog.Tup = MES_SUB[pch].zc_Tup; //上流時間差
		MES[pch].ZcLog.Tdw = MES_SUB[pch].zc_Tdown; //下流時間差
		//ゼロクロス点
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

	//生ゼロクロス点を読み出す
	if(Mod == 0)
	{
		read_change(com_mode, MES[pch].ZcLog.Flw, 2, ADD_CMM_BFR_VAL);
		
		Hvl = (long)(MES[pch].ZcLog.Tup); //整数部
		Lvl = (long)((MES[pch].ZcLog.Tup - Hvl) * 10000000); //小数部
		read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
		strcat(TX_buf[com_mode], ".");
		read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);

		Hvl = (long)(MES[pch].ZcLog.Tdw); //整数部
		Lvl = (long)((MES[pch].ZcLog.Tdw - Hvl) * 10000000); //小数部
		read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
		strcat(TX_buf[com_mode], ".");
		read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);
	}
	//上流生ゼロクロス点を読み出す
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
	//下流生ゼロクロス点を読み出す
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
	//計算後ゼロクロス点を読み出す
	else if(Mod == 3)
	{
		//上流
		for(i = sPnt; i < ePnt; i++)
		{
			Hvl = (long)(MES[pch].ZcLog.FowClcZcd[i]); //整数部
			Lvl = (long)((MES[pch].ZcLog.FowClcZcd[i] - Hvl) * 10000000); //小数部
			read_change(com_mode, Hvl, 0, ADD_CMM_BFR_VAL);
			strcat(TX_buf[com_mode], ".");
			read_change2(com_mode, Lvl, 0, ADD_NO_CMM, 7);
		}
		//下流
		for(i = sPnt; i < ePnt; i++)
		{
			Hvl = (long)(MES[pch].ZcLog.RevClcZcd[i]); //整数部
			Lvl = (long)((MES[pch].ZcLog.RevClcZcd[i] - Hvl) * 10000000); //小数部
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

/* メモリデバイスシリアルナンバー読出し(MS) */
 void command_MS (short com_mode){

	long value;

	TX_buf[com_mode][6] = ',';
	TX_buf[com_mode][7] = RX_buf[com_mode][5];

	value = SVD[ch_no[com_mode] -1].m_serial[0];	//メモリデバイスシリアルナンバー
	read_change(com_mode,value,0,ADD_CMM_BFR_VAL);
	
	value = SVD[ch_no[com_mode] -1].m_serial[1];	//メモリデバイスシリアルナンバー
	read_change(com_mode,value,0,ADD_CMM_BFR_VAL);

	value = SVD[ch_no[com_mode] -1].m_serial[2];	//メモリデバイスシリアルナンバー
	read_change(com_mode,value,0,ADD_CMM_BFR_VAL);
} 
