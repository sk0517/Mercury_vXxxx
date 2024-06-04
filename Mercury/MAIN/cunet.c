/***********************************************/
/* File Name : cunet.c		           									   */
/*	Summary   : CUnet通信処理	                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>
#include <string.h>	
#include <stdlib.h>

#include "cunet.h"
#include "version.h"
#include "define.h"
#include "defMES.h"
#include "defMAIN.h"
#include "SV_def.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void	mky43_init(void);
void	mky43_start(void);
void	mky43_restart(void);
void	mky43_set_register(void);
void	mky43_ping_active(void);
void	mky43_mail_recv(void);
void	mky43_ready_recv0(void);
void	mky43_ready_recv1(void);
void	mky43_mail_send(unsigned short size, short com_mode);
void	mky43_mail_status(void);
unsigned long	mky43_read_data(short flow_no);
unsigned long	mky43_read_data2(short flow_no);
void	mky43_write_alarm(short ch);
void	mky43_write_flow(short ch);
long	flow_fs_cal(short ch);
void	mky43_check_datarenewal(void);
short	mky43_get_flow_num(void);
void	mky43_check_recv0(void);
void	mky43_check_recv1(void);
void	mky43_TX_start(short com_mode);
void	mky43_rxbuf_save(short buf_side, unsigned short recv_size, unsigned short recv_sa);
void	mky43_host_link_check(void);
short		mky43_ccr_check(void);
short		mky43_add_message(unsigned short len, short com_mode);
void	mky43_del_message(unsigned short len, unsigned short offset, short com_mode);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern short	disp_cunet_read(void);
extern void	protocol_timer_host(unsigned short value);
extern void protocol_timer_subhost(void);
extern short	util_passed_time(unsigned long basic_time, unsigned long target_time);
extern void	util_delay(short target_time);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short				sa_flow;		//ステーションアドレス
unsigned short	recv_sa_host;	//送信元ステーションアドレス
unsigned short	recv_sa_sub;	//送信元ステーションアドレス
unsigned short 	cr_check;
short				buf_count;
short				buf_count_sub;
short				com_start;
short				com_link;
short				len_err_host;
short				len_err_sub;
short				format_err_host;
short				format_err_sub;

unsigned short	recv_data0[32][4] = { 0 };		//メール受信データ(MRB0:メール受信バッファ0)
unsigned short	recv_data1[32][4] = { 0 };		//メール受信データ(MRB1:メール受信バッファ1)
unsigned short	send_data[32][4]  = { 0 };		//メール送信データ(ホスト用)
unsigned short	send_data_sub[32][4]  = { 0 };	//メール送信データ(サブホスト用)

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short 	sw_cunet_num;
extern char *RX_buf_host;
extern char *TX_buf_host;
extern char *RX_buf_subhost;
extern char *TX_buf_subhost;
extern short com_type;
extern unsigned long	cmi_count;

/****************************************************/
/* Function : mky43_init                    */
/* Summary  : MKY43の初期化    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_init(void){

	// GPIO_PK4
	__bit_output(GPIO_PORTK_BASE, 4, 1);				//MKY43 RST

	sa_flow = mky43_get_flow_num();		//流量計番号読込み/
	sw_cunet_num = sa_flow;				//CUnetSW状態保存

	mky43_start();						//MKY43の通信開始
	mky43_set_register();				//MKY43のレジスタ設定
	mky43_ping_active();				//PING動作レジスタ設定
}
	
/****************************************************/
/* Function : mky43_start                    */
/* Summary  : MKY43の通信開始    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_start(void){

	unsigned short	bcr_sa;
	unsigned short	bcr_bps;
	unsigned short	bcr_own;
	unsigned short	bcr_lfs;
	short i_cnt;
	short j_cnt;

	/*RAMクリア*/
	//GM領域のクリア
	for(i_cnt=0; i_cnt<64; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.GM.SA[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	//MSB領域のクリア
	for(i_cnt=0; i_cnt<32; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.MSB.SEND[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	//MRB0領域のクリア
	for(i_cnt=0; i_cnt<32; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.MRB0.RECV[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	//MRB1領域のクリア
	for(i_cnt=0; i_cnt<32; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.MRB1.RECV[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	
	/*基本(BCR)設定*/
	if(MKY43.REG.SCR.BIT.START == 1){			//CUnet通信起動中
		MKY43.REG.SCR.BIT.START = 0;			//CUnet通信停止（強制停止）
		while(MKY43.REG.SCR.BIT.START == 1){	//CUnet通信停止待機
			;
		}
	}

	MKY43.REG.SCR.BIT.GMM = 1;

	bcr_sa = sa_flow;					//ステーションアドレス設定
	bcr_bps = 3;						//転送レート設定（12Mbps固定）
	bcr_own = 3;						//占有幅設定（3固定）
	bcr_lfs = 0;						//フレームオプション設定（通常0）
	MKY43.REG.BCR.WORD = (bcr_sa + (bcr_bps<<6) + (bcr_own<<8) + (bcr_lfs<<15));

	MKY43.REG.SCR.BIT.GMM = 0;
 	send_err_status = 0;				//メール送信エラーステータスクリア(GMエリア)

	/*CUnet通信起動*/
	MKY43.REG.SCR.BIT.START = 1;			//CUnet通信起動

	while(1){
		if(MKY43.REG.SCR.BIT.RUN != 0){	//RUNフェーズ確認
			com_start = B_ON;				//通信オン状態
			break;
		}
		if(MKY43.REG.SCR.BIT.CALL != 0){	//CALLフェーズ確認
			com_start = B_OFF;				//通信オフ状態
			break;
		}
		if(MKY43.REG.SCR.BIT.BRK != 0){	//BRKフェーズ確認
			com_start = B_OFF;				//通信オフ状態
			break;
		}
	}

	/*CCTRレジスタ設定*/
	MKY43.REG.CCTR.WORD = 0x0101;			//LCARE,MCARE信号発生回数をクリア
}

/****************************************************/
/* Function : mky43_restart                    */
/* Summary  : MKY43の通信再開    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_restart(void){

	unsigned short	bcr_sa;
	unsigned short	bcr_bps;
	unsigned short	bcr_own;
	unsigned short	bcr_lfs;

	MKY43.REG.INT0SR.WORD = 0xFFFF;
	
	MKY43.REG.SCR.BIT.GMM = 1;

	bcr_sa = sa_flow;					//ステーションアドレス設定
	bcr_bps = 3;						//転送レート設定（12Mbps固定）
	bcr_own = 3;						//占有幅設定（3固定）
	bcr_lfs = 0;						//フレームオプション設定（通常0）
	MKY43.REG.BCR.WORD = (bcr_sa + (bcr_bps<<6) + (bcr_own<<8) + (bcr_lfs<<15));

	MKY43.REG.SCR.BIT.GMM = 0;
	MKY43.REG.MESR.WORD = 0;			//メール送信エラーステータスクリア

	MKY43.REG.SCR.BIT.START = 1;		//CUnet通信起動
}

/****************************************************/
/* Function : mky43_set_register                    */
/* Summary  : MKY43のレジスタ設定    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_set_register(void){

	short	set_bit;

	/*メール受信バッファ設定(MRB0,MRB1)*/
	MKY43.REG.INT0CR.BIT.MR = 1;			//メール受信完了割込み許可
	MKY43.REG.MR0CR.BIT.RDY = 1;			//メール受信許可

	/*メール送信バッファ設定*/
	MKY43.REG.INT0CR.BIT.MSF = 0;			//メール送信完了割込み禁止

	/*GMエリアのデータ遷移検出設定*/
	set_bit = 0x0000;
	switch(sa_flow){
		case	SA_FLOW01:
		case	SA_FLOW02:
		case	SA_FLOW03:
		case	SA_FLOW04:
				set_bit = 0x0001;
				break;
		case	SA_FLOW05:
		case	SA_FLOW06:
		case	SA_FLOW07:
		case	SA_FLOW08:
				set_bit = 0x0002;
				break;
		case	SA_FLOW09:
		case	SA_FLOW10:
		case	SA_FLOW11:  //CUnet 16ch
		case	SA_FLOW12:
				set_bit = 0x0004;
				break;
		case	SA_FLOW13:  //CUnet 16ch
		case	SA_FLOW14:
		case	SA_FLOW15:
		case	SA_FLOW16:
				set_bit = 0x0008;
				break;
		default:
				break;
	}
	MKY43.REG.DRCR.DATA[0].DATA = set_bit;
	MKY43.REG.INT1CR.BIT.DR = 1;			//データ更新割込み許可

}

/****************************************************/
/* Function : mky43_ping_active                    */
/* Summary  : PING動作レジスタ設定    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_ping_active(void){

	MKY43.REG.UTCR.BIT.OE1 = 1;		//#UTY1からの#PING出力を許可 
	MKY43.REG.UTCR.BIT.SS1 = 0;		//#PING出力 
	MKY43.REG.UTCR.BIT.OE2 = 0;		//#UTY2からの#PING出力を許可 
	MKY43.REG.UTCR.BIT.SS2 = 0;		//#PING出力 

}

/****************************************************/
/* Function : mky43_mail_recv                    */
/* Summary  : メール受信    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : MRB0:メール受信バッファ0/MRB1:メール受信バッファ1 */
/****************************************************/
void	mky43_mail_recv(void){

	short				i_cnt;
	short				j_cnt;
	unsigned short	recv_size;		//メールデータサイズ
	unsigned short	recv_sa;		//送信元ステーションアドレス

	/*MRB0:メール受信バッファ0*/
	if(MKY43.REG.MR0CR.BIT.RCV != 0){					//メール受信完了
		for(i_cnt=0; i_cnt<32; i_cnt++){				//受信バッファ初期化
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data0[i_cnt][j_cnt] = 0;
			}
		}		
		
		recv_sa = ((MKY43.REG.MR0CR.WORD & 0x3F00) >> 8);		//送信元ステーションアドレス取得
		recv_size = (MKY43.REG.MR0CR.WORD & 0x003F);	//メール受信データサイズ取得(8Byteを1単位)

		for(i_cnt=0; i_cnt<recv_size; i_cnt++){		//メール受信データ取得
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data0[i_cnt][j_cnt] = MKY43.MRB0.RECV[i_cnt].DATA[j_cnt].DATA;
			}
		}
		mky43_rxbuf_save(0, recv_size, recv_sa);		//取得データをRX_buf_host[]に保存
	}

	/*MRB1:メール受信バッファ1*/
	if(MKY43.REG.MR1CR.BIT.RCV != 0){					//メール受信完了
		for(i_cnt=0; i_cnt<32; i_cnt++){				//受信バッファ初期化
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data1[i_cnt][j_cnt] = 0;
			}
		}
		
		recv_sa = ((MKY43.REG.MR1CR.WORD & 0x3F00) >> 8);		//送信元ステーションアドレス取得
		recv_size = (MKY43.REG.MR1CR.WORD & 0x003F);	//メール受信データサイズ取得(8Byteを1単位)

		for(i_cnt=0; i_cnt<recv_size; i_cnt++){		//メール受信データ取得
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data1[i_cnt][j_cnt] = MKY43.MRB1.RECV[i_cnt].DATA[j_cnt].DATA;
			}
		}
		mky43_rxbuf_save(1, recv_size, recv_sa);		//取得データをRX_buf_host[]に保存
	}

	MKY43.REG.INT0CR.BIT.MR = 1;						//メール受信完了割込み許可	
	MKY43.REG.INT0SR.BIT.MR = 1;						//メール受信完了割込み解除
	MKY43.REG.MR0CR.BIT.RDY = 1;						//メール受信許可
}

/****************************************************/
/* Function : mky43_ready_recv0                    */
/* Summary  : メール受信許可    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : MRB0:メール受信バッファ0                    */
/****************************************************/
void	mky43_ready_recv0(void){

	if(com_type == COM_CUNET){
		MKY43.REG.MR0CR.BIT.RCV = 0;
		MKY43.REG.MR0CR.BIT.RDY = 1;		//メール受信許可

		MKY43.REG.INT0CR.BIT.MR = 1;		//メール受信完了割込み許可
		MKY43.REG.INT0SR.BIT.MR = 1;		//メール受信完了割込み解除
	}
}

/****************************************************/
/* Function : mky43_ready_recv1                    */
/* Summary  : メール受信許可    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : MRB1:メール受信バッファ1                    */
/****************************************************/
void	mky43_ready_recv1(void){

	if(com_type == COM_CUNET){
		MKY43.REG.MR1CR.BIT.RCV = 0;

		MKY43.REG.INT1CR.BIT.MR = 1;		//メール受信完了割込み許可
		MKY43.REG.INT1SR.BIT.MR = 1;		//メール受信完了割込み解除
	}
}

/****************************************************/
/* Function : mky43_mail_send                    */
/* Summary  : メール送信    				*/
/* Argument : size,   com_mode     	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : MSB:メール送信バッファ                   */
/****************************************************/
void	mky43_mail_send(unsigned short size, short com_mode){

	short				ch;
	short				i_cnt;
	short				j_cnt;
	short				send_retry;
	unsigned short	send_sa;			//送信先ステーションアドレス
	unsigned short	send_size;			//メールデータサイズ
	unsigned long	send_time;			//送信開始時間
	
	if(com_mode == SA_HOST){
		send_sa = recv_sa_host;			//メール送信先ステーションアドレス
	}else{
		send_sa = recv_sa_sub;			//メール送信先ステーションアドレス
	}
	send_size = size;

	IntPrioritySet(INT_TIMER1A, 5 << 5);			//一時的に優先順位を上げる（受信完了割り込み（メンテナンスポート））
	while(MKY43.REG.MSCR.BIT.SEND == 1);		//メール送信中は待機
	IntPrioritySet(INT_TIMER1A, 6 << 5);			//優先順位を元に戻す
		
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		MES[ch].err_status &= ~ERR_JUDGE_CUNET;	//CUnetエラーリセット
	}

	MKY43.REG.MESR.WORD = 0;					//メール送信エラーステータスクリア

	if(com_mode == SA_HOST){
		for(i_cnt=0; i_cnt<send_size; i_cnt++){	//メール送信データ作成
			for(j_cnt=0; j_cnt<4; j_cnt++){
				MKY43.MSB.SEND[i_cnt].DATA[j_cnt].DATA = send_data[i_cnt][j_cnt];
			}
		}
	}else{
		for(i_cnt=0; i_cnt<send_size; i_cnt++){	//メール送信データ作成
			for(j_cnt=0; j_cnt<4; j_cnt++){
				MKY43.MSB.SEND[i_cnt].DATA[j_cnt].DATA = send_data_sub[i_cnt][j_cnt];
			}
		}
	}

	send_time = cmi_count;						//メール送信開始時間
	send_retry = 0;								//メール送信リトライ回数

RESEND:
	
	MKY43.REG.MSCR.WORD = (0x0000 | (send_sa<<8) | send_size);	//送信先ステーションアドレス、データサイズ
	MKY43.REG.MSCR.BIT.SEND = 1;				//メール送信開始

	IntPrioritySet(INT_TIMER1A, 5 << 5);			//一時的に優先順位を上げる（受信完了割り込み（メンテナンスポート））
	while(MKY43.REG.MSCR.BIT.SEND == 1);		//メール送信中は待機
	IntPrioritySet(INT_TIMER1A, 6 << 5);			//優先順位を元に戻す

	mky43_mail_status();						//メール送信エラーステータス(CUnetエラー検出、GMエリアを更新)
	
	if(B_YES != util_passed_time(send_time, MES_RESEND_LIM)){	//再送信時間経過確認
		if(MKY43.REG.MSCR.BIT.ERR != 0){				//メール送信エラー発生
			if(MKY43.REG.MESR.BIT.NORDY != 0){			//送信先の受信バッファが受信許可でない
				MKY43.REG.MESR.WORD = 0;				//メール送信エラーステータスクリア

				IntPrioritySet(INT_TIMER1A, 5 << 5);		//一時的に優先順位を上げる（受信完了割り込み（メンテナンスポート））
				util_delay(SVD[0].cunet_delay);		//再送待機
				IntPrioritySet(INT_TIMER1A, 6 << 5);		//優先順位を元に戻す

				goto RESEND;							//再送処理
			}else{										//その他のメール送信エラーの場合
				if(send_retry < MES_RESEND_MAX){
					send_retry++;						//メール送信リトライ回数更新
					MKY43.REG.MESR.WORD = 0;			//メール送信エラーステータスクリア

					IntPrioritySet(INT_TIMER1A, 5 << 5);	//一時的に優先順位を上げる（受信完了割り込み（メンテナンスポート））
					util_delay(SVD[0].cunet_delay);	//再送待機
					IntPrioritySet(INT_TIMER1A, 6 << 5);	//優先順位を元に戻す

					goto RESEND;						//再送処理
				}
			}
		}
	}
}

/****************************************************/
/* Function : mky43_mail_status                    */
/* Summary  : メール送信エラーステータス    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : CUnetエラー検出、GMエリアを更新              */
/****************************************************/
void	mky43_mail_status(void){

	short 	ch;
	short		sa_flow_buf;

	sa_flow_buf = sa_flow;						//流量計ステーションアドレス取得

	if(MKY43.REG.MSCR.BIT.ERR != 0){			//メール送信エラー発生
		if(MKY43.REG.MESR.BIT.NORDY != 0){		//送信先の受信バッファが受信許可でない
			send_err_status |= MAIL_ERR_NORDY;
		}
		if(MKY43.REG.MESR.BIT.NOEX != 0){		//送信先のCUnetステーションが存在しない
			send_err_status |= MAIL_ERR_NOEX;
		}
		if(MKY43.REG.MESR.BIT.TOUT != 0){		//設定サイクル回数を経過してもメール送信が完了しない
			send_err_status |= MAIL_ERR_TOUT;
		}
		if(MKY43.REG.MESR.BIT.SZFLT != 0){		//メール送信サイズが不正値
			send_err_status |= MAIL_ERR_SZFLT;
		}
		if(MKY43.REG.MESR.BIT.LMFLT != 0){		//MSLRの設定値が不正値
			send_err_status |= MAIL_ERR_LMFLT;
		}
		if(MKY43.REG.MESR.BIT.STOP != 0){		//ネットワークが停止
			send_err_status |= MAIL_ERR_STOP;
		}
		if((send_err_status & ERR_CUNET_MAIL) != 0){		
			for(ch = CH1; ch < CH_NUMMAX; ch++){
				MES[ch].err_status |= ERR_JUDGE_CUNET;		//CUnetエラーセット
			}
			cunet_error = send_err_status;					//エラー要因をログに保存
		}
	}else{
		send_err_status = 0x0000;				//メール送信エラーステータスをクリア
	}

	/*メール送信エラーステータスをGMエリアに書込む*/
	MKY43.GM.SA[sa_flow_buf+2].DATA[3].DATA = (send_err_status<<8) & 0xFF00;

}

/****************************************************/
/* Function : mky43_read_data                    */
/* Summary  : GMエリアデータデータ読込み    				*/
/* Argument : flow_no              	               */
/* Return   : GMエリアデータ 									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
unsigned long	mky43_read_data(short flow_no){

	unsigned long read_data;

	read_data = 0;
	
	switch(flow_no){
		case	SA_FLOW01:				//流量計01
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW02:				//流量計02
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[1].DATA & 0x003F;
				break;
		case	SA_FLOW03:				//流量計03
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW04:				//流量計04
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[3].DATA & 0x003F;
				break;
		case	SA_FLOW05:				//流量計05
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW06:				//流量計06
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[1].DATA & 0x003F;
				break;
		case	SA_FLOW07:				//流量計07
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW08:				//流量計08
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[3].DATA & 0x003F;
				break;
		case	SA_FLOW09:				//流量計09
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW10:				//流量計10
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[1].DATA & 0x003F;
				break;
//CUnet 16ch
		case	SA_FLOW11:				//流量計11
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW12:				//流量計12
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[3].DATA & 0x003F;
				break;
		case	SA_FLOW13:				//流量計13
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW14:				//流量計14
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[1].DATA & 0x003F;
				break;
		case	SA_FLOW15:				//流量計15
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW16:				//流量計16
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[3].DATA & 0x003F;
				break;
		default:
				break;
	}
	return(read_data);

}

/****************************************************/
/* Function : mky43_read_data2                    */
/* Summary  : GMエリアデータデータ読込み（積算監視機能）   				*/
/* Argument : flow_no              	               */
/* Return   : GMエリアデータ 									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
unsigned long	mky43_read_data2(short flow_no){

	unsigned long read_data;

	read_data = 0;
	
	switch(flow_no){
		case	SA_FLOW01:				//流量計01
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW02:				//流量計02
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[1].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW03:				//流量計03
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW04:				//流量計04
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[3].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW05:				//流量計05
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW06:				//流量計06
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[1].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW07:				//流量計07
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW08:				//流量計08
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[3].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW09:				//流量計09
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW10:				//流量計10
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[1].DATA & 0x3F00) >> 8);
				break;
//CUnet 16ch
		case	SA_FLOW11:				//流量計11
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW12:				//流量計12
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[3].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW13:				//流量計13
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW14:				//流量計14
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[1].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW15:				//流量計15
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW16:				//流量計16
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[3].DATA & 0x3F00) >> 8);
				break;
		default:
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[0].DATA & 0x3F00) >> 8);
				break;
	}
	return(read_data);

}

/****************************************************/
/* Function : mky43_write_alarm                    */
/* Summary  : エラーステータス書込み   				*/
/* Argument : ch                  	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
void	mky43_write_alarm(short ch){

	short		sa_flow_buf;
	short		ch_num;

	if(ch >= CH_NUMMAX){
		return;
	}
	
	sa_flow_buf = sa_flow;				//流量計ステーションアドレス取得
	ch_num = ch;
	
	switch(ch_num){
		case CH1:
		case CH2:
			//アラームステータス書込み
			MKY43.GM.SA[sa_flow_buf].DATA[0].DATA = (MAIN[0].err_condition_cu) + (MAIN[1].err_condition_cu<<8); 	//CH1+CH2
			break;
		case CH3:
		case CH4:
			//アラームステータス書込み
			MKY43.GM.SA[sa_flow_buf].DATA[1].DATA = (MAIN[2].err_condition_cu) + (MAIN[3].err_condition_cu<<8); 	//CH3+CH4
			break;
		case CH5:
		case CH6:
			//アラームステータス書込み
			MKY43.GM.SA[sa_flow_buf].DATA[2].DATA = (MAIN[4].err_condition_cu) + (MAIN[5].err_condition_cu<<8); 	//CH5+CH6
			break;
		default:
			break;
	}
}

/****************************************************/
/* Function : mky43_write_flow                    */
/* Summary  : 瞬時流量データ書込み   				*/
/* Argument : ch                  	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
void	mky43_write_flow(short ch){

	long	flow_GM_buf;
	short		ch_num;
	unsigned short	mb_num;

	if(ch >= CH_NUMMAX){
		return;
	}
	
	ch_num = ch;
	mb_num = sa_flow;
	
	switch(ch_num){
		case CH1:
		case CH2:
		case CH3:
		case CH4:
			//瞬時流量書込み
			//ライトハザード防止機能を使用して、MBへ一括書込み
			flow_GM_buf = flow_fs_cal(CH1);
			MKY43.REG.WRHPB0.DATA[0].DATA = (flow_GM_buf & 0x0000ffff);	//CH1瞬時流量
			flow_GM_buf = flow_fs_cal(CH2);
			MKY43.REG.WRHPB0.DATA[1].DATA = (flow_GM_buf & 0x0000ffff);	//CH2瞬時流量
			flow_GM_buf = flow_fs_cal(CH3);
			MKY43.REG.WRHPB0.DATA[2].DATA = (flow_GM_buf & 0x0000ffff);	//CH3瞬時流量
			flow_GM_buf = flow_fs_cal(CH4);
			MKY43.REG.WRHPB0.DATA[3].DATA = (flow_GM_buf & 0x0000ffff);	//CH4瞬時流量
			MKY43.REG.WHCR0.WORD = (mb_num + 1);							//MBへ一括書込み
			break;
		case CH5:
		case CH6:
			//瞬時流量書込み
			//ライトハザード防止機能を使用して、MBへ一括書込み
			flow_GM_buf = flow_fs_cal(CH5);
			MKY43.REG.WRHPB0.DATA[0].DATA = (flow_GM_buf & 0x0000ffff);	//CH5瞬時流量
			flow_GM_buf = flow_fs_cal(CH6);
			MKY43.REG.WRHPB0.DATA[1].DATA = (flow_GM_buf & 0x0000ffff);	//CH6瞬時流量
			MKY43.REG.WRHPB0.DATA[2].DATA = 0;								//未使用領域
			MKY43.REG.WRHPB0.DATA[3].DATA = (send_err_status<<8) & 0xFF00;//メール送信エラーステータス
			MKY43.REG.WHCR0.WORD = (mb_num + 2);							//MBへ一括書込み
			break;
		default:
			break;
	}
}

/****************************************************/
/* Function : flow_fs_cal                           */
/* Summary  : 瞬時流量データ変換処理   				             */
/* Argument : ch                  	                 */
/* Return   : なし      									                    */
/* Caution  : なし                                    */
/* notes    : フルスケール設定によってGMへ書込む瞬時流量データを変換する*/
/****************************************************/
long	flow_fs_cal(short ch){

	long	flow;
	
	flow = (long)MES[ch].ml_min_now;	//瞬時流量値
	
	if (SVD[ch].max_flow <= 500){
		flow /= 1;	//FS ～500ml/min：瞬時流量値*100
		if (flow > SVD[ch].max_flow * 1.1 * 100)	flow = SVD[ch].max_flow * 1.1 * 100;	//110%over
	}
	else if (SVD[ch].max_flow > 500 && SVD[ch].max_flow <= 5000){	
		flow /= 10;	//FS 501～5000ml/min：瞬時流量値*10
		if (flow > SVD[ch].max_flow * 1.1 * 10)	flow = SVD[ch].max_flow * 1.1 * 10;		//110%over
	}
	else if (SVD[ch].max_flow > 5000 && SVD[ch].max_flow <= 50000){
		flow /= 100;	//FS 5001～50000ml/min：瞬時流量値*1
		if (flow > SVD[ch].max_flow * 1.1)	flow = SVD[ch].max_flow * 1.1;	//110%over
	}
	else {
		flow /= 1000;	//FS 50001～100000ml/min：瞬時流量値/10
		if (flow > SVD[ch].max_flow * 1.1 / 10)	flow = SVD[ch].max_flow * 1.1 / 10;	//110%over
	}

	if(flow < 0){
		flow = 0;		//マイナス時は「0」
	}

	return(flow);
}

/****************************************************/
/* Function : mky43_check_datarenewal               */
/* Summary  : GMエリアデータ更新確認   				*/
/* Argument : なし                  	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
void	mky43_check_datarenewal(void){

	short ch;
	unsigned long read_data, read_data2;

	read_data = mky43_read_data(sa_flow);	//GMエリアデータ読込み		
	read_data2 = mky43_read_data2(sa_flow);	//GMエリアデータ読込み（積算監視機能）		

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		if(read_data & 0x0001 == 1){		//積算開始指示確認
			MES[ch].addit_req = 1;			//積算開始
		}else{
			MES[ch].addit_req = 0;			//積算停止
		}
		read_data = (read_data >> 1);

		if(read_data2 & 0x0001 == 1){	//積算監視機能確認
			MES[ch].addit_watch = B_ON;		//積算監視有効
		}else{
			MES[ch].addit_watch = B_OFF;	//積算監視無効
		}
		read_data2 = (read_data2 >> 1);
	}
		
	MKY43.REG.INT1CR.BIT.DR = 1;			//データ更新割込み許可
	MKY43.REG.INT1SR.BIT.DR = 1;			//データ更新割込み解除

}

/****************************************************/
/* Function : mky43_get_flow_num               */
/* Summary  : 流量計番号読込み   				*/
/* Argument : なし                  	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
short	mky43_get_flow_num(void){

	short	dip_sw;
	short	flow_num;
	
	dip_sw = disp_cunet_read();		//DIP-SW番号取得

	switch(dip_sw){
		case	0:						//DIP-SW番号:0
				flow_num = SA_FLOW01;	//流量計01
				break;
		case	1:						//DIP-SW番号:1
				flow_num = SA_FLOW02;	//流量計02
				break;
		case	2:						//DIP-SW番号:2
				flow_num = SA_FLOW03;	//流量計03
				break;
		case	3:						//DIP-SW番号:3
				flow_num = SA_FLOW04;	//流量計04
				break;
		case	4:						//DIP-SW番号:4
				flow_num = SA_FLOW05;	//流量計05
				break;
		case	5:						//DIP-SW番号:5
				flow_num = SA_FLOW06;	//流量計06
				break;
		case	6:						//DIP-SW番号:6
				flow_num = SA_FLOW07;	//流量計07
				break;
		case	7:						//DIP-SW番号:7
				flow_num = SA_FLOW08;	//流量計08
				break;
		case	8:						//DIP-SW番号:8
				flow_num = SA_FLOW09;	//流量計09
				break;
		case	9:						//DIP-SW番号:9
				flow_num = SA_FLOW10;	//流量計10
				break;
//CUnet 16ch
		case	10:						//DIP-SW番号:10
				flow_num = SA_FLOW11;	//流量計11
				break;
		case	11:						//DIP-SW番号:11
				flow_num = SA_FLOW12;	//流量計12
				break;
		case	12:						//DIP-SW番号:12
				flow_num = SA_FLOW13;	//流量計13
				break;
		case	13:						//DIP-SW番号:13
				flow_num = SA_FLOW14;	//流量計14
				break;
		case	14:						//DIP-SW番号:14
				flow_num = SA_FLOW15;	//流量計15
				break;
		case	15:						//DIP-SW番号:15
				flow_num = SA_FLOW16;	//流量計16
				break;
		default:
				flow_num = SA_FLOW01;	//流量計01
				break;
	}
	return(flow_num);

}

/****************************************************/
/* Function : mky43_check_recv0               */
/* Summary  : MKY43割込み確認   				*/
/* Argument : なし                  	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
void	mky43_check_recv0(void){

	if(MKY43.REG.INT0SR.BIT.MR != 0){		//メール受信完了割込み発生
		mky43_mail_recv();					//メール受信(MRB0,MRB1)
	}
}

/****************************************************/
/* Function : mky43_check_recv1               */
/* Summary  : MKY43割込み確認   				*/
/* Argument : なし                  	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
void	mky43_check_recv1(void){

	if(MKY43.REG.INT1SR.BIT.DR != 0){		//GMエリアのデータ更新検出割込み発生
		mky43_check_datarenewal();
	}
}

/****************************************************/
/* Function : mky43_TX_start               */
/* Summary  : 送信データをsend_data[]に保存   				*/
/* Argument : com_mode           	               */
/* Return   : なし      									                  */
/* Caution  : なし                                   */
/* notes    : なし                                  */
/****************************************************/
void	mky43_TX_start(short com_mode){

	short i;
	short j;
	short cnt;
	short ch_num;
	short send_num;
	short	add_len;
	unsigned short size;
	unsigned short data_len=0;
	char ch[1] = {0};
	
	if(com_mode == SA_HOST){		//ホスト(SA=0)へ送信する場合
		//送信データ長計算
		if((TX_buf_host[1] == 'O' && (TX_buf_host[2] == 'E' || TX_buf_host[2] == 'Q')) || (TX_buf_host[1] == 'o' && TX_buf_host[2] == 'q')){
			ch[0] = TX_buf_host[9];
			ch_num = ch[0] - 0x30;
			if(TX_buf_host[2] == 'E'){			//'OE'コマンド
				data_len = 13 + 9*ch_num;
			}else{ 							//'OQ','oq'コマンド	
				data_len = 13 + 72*ch_num;
			}
		}else{
			data_len = strlen(TX_buf_host);
		}
	
		add_len = mky43_add_message(data_len, com_mode);	//通信メッセージに『CHデータ』『Lengthフィールド』を入れる
		data_len += add_len;						//データ長更新
	
		if(data_len%256 == 0){
			send_num = (data_len/256);				//データ送信回数
		}else{
			send_num = (data_len/256) + 1;			//データ送信回数
		}

		for(cnt=0; cnt<send_num; cnt++){
			memset(send_data, 0, sizeof(send_data));
			if(cnt == (send_num - 1)){				//最終送信時
				if(data_len%8 == 0){
					size = (data_len/8);			//送信サイズ
				}else{
					size = (data_len/8) + 1;		//送信サイズ
				}
			}else{
				size = 32;							//送信サイズ
			}

			for(i=0; i<size; i++){				//送信バッファのデータをメール送信用にコピー
				for(j=0; j<4; j++){
					send_data[i][j] = TX_buf_host[8*i + 2*j + (256*cnt+1)]*0x0100 + TX_buf_host[8*i + 2*j + (256*cnt)];
				}
			}
			mky43_mail_send(size, com_mode);		//メッセージ送信処理
			data_len -= 256;						//送信データサイズ更新
		}
		for (i=0; i<MSG_MAX; i++){
			TX_buf_host[i] = 0;						/*送信バッファクリア*/
		}
	}else{						//ホスト以外のSA(Station Address)へ送信する場合
		//送信データ長計算
		if((TX_buf_subhost[1] == 'O' && (TX_buf_subhost[2] == 'E' || TX_buf_subhost[2] == 'Q')) || (TX_buf_subhost[1] == 'o' && TX_buf_subhost[2] == 'q')){
			ch[0] = TX_buf_subhost[9];
			ch_num = ch[0] - 0x30;
			if(TX_buf_subhost[2] == 'E'){		//'OE'コマンド
				data_len = 13 + 9*ch_num;
			}else{ 							//'OQ','oq'コマンド	
				data_len = 13 + 72*ch_num;
			}
		}else{
			data_len = strlen(TX_buf_subhost);
		}
	
		add_len = mky43_add_message(data_len, com_mode);		//通信メッセージに『CHデータ』『Lengthフィールド』を入れる
		data_len += add_len;						//データ長更新
		
		if(data_len%256 == 0){
			send_num = (data_len/256);				//データ送信回数
		}else{
			send_num = (data_len/256) + 1;			//データ送信回数
		}

		for(cnt=0; cnt<send_num; cnt++){
			memset(send_data_sub, 0, sizeof(send_data_sub));
			if(cnt == (send_num - 1)){				//最終送信時
				if(data_len%8 == 0){
					size = (data_len/8);			//送信サイズ
				}else{
					size = (data_len/8) + 1;		//送信サイズ
				}
			}else{
				size = 32;							//送信サイズ
			}

			for(i=0; i<size; i++){				//送信バッファのデータをメール送信用にコピー
				for(j=0; j<4; j++){
					send_data_sub[i][j] = TX_buf_subhost[8*i + 2*j + (256*cnt+1)]*0x0100 + TX_buf_subhost[8*i + 2*j + (256*cnt)];
				}
			}
			mky43_mail_send(size, com_mode);		//メッセージ送信処理
			data_len -= 256;						//送信データサイズ更新
		}
		for (i=0; i<MSG_MAX; i++){
			TX_buf_subhost[i] = 0;					/*送信バッファクリア*/
		}
	}
}

/****************************************************/
/* Function : mky43_rxbuf_save                      */
/* Summary  : 取得データをRX_buf_host[],RX_buf_subhost[]に保存*/
/* Argument : buf_side,  recv_size,  recv_sa        */
/* Return   : なし      									                    */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_rxbuf_save(short buf_side, unsigned short recv_size, unsigned short recv_sa){

	short i;
	short j;
	short buf_num;
	short buf_offset;
	short data_len;
	short data_offset;
	
	if(recv_sa == SA_HOST){		//ホスト(SA=0)からの取得データ
		buf_offset = buf_count;
		if(buf_offset == 0){
			for (i=0; i<MSG_MAX; i++){
				RX_buf_host[i] = 0;
			}	
		}
		recv_sa_host = recv_sa;
		recv_size *= 8;
		for(buf_num=0; buf_num<recv_size; buf_num++){
			i = buf_num/8;
			j = (buf_num%8)/2;	
			if(buf_num%2 == 0){
				if(buf_side == 0){		//recv_data0[]を保存
					RX_buf_host[buf_num+buf_offset] = recv_data0[i][j]%0x0100;
				}else{					//recv_data1[]を保存
					RX_buf_host[buf_num+buf_offset] = recv_data1[i][j]%0x0100;
				}
			}else{
				if(buf_side == 0){		//recv_data0[]を保存
					RX_buf_host[buf_num+buf_offset] = recv_data0[i][j]/0x0100;
				}else{					//recv_data1[]を保存
					RX_buf_host[buf_num+buf_offset] = recv_data1[i][j]/0x0100;
				}
			}
			if(RX_buf_host[buf_num+buf_offset] != CR 	/*CR(0x0D)なし*/
							|| (buf_num == 1)){		/*データ長が格納されている配列は無視する(データ長が13(0x0D)の場合があるので)*/
				buf_count++;
			}else{							//CR(0x0D)あり
				buf_count = 0;
				break;
			}
			if((buf_num+buf_offset) > MSG_MAX){
				break;
			}
		}
	
		if(buf_count == 0){		//全メッセージ(CRまでのデータ)受信完了		
			/*フォーマット判定*/
			data_offset = 0;
			format_err_host = 0;
			while(RX_buf_host[data_offset]!= HEADER_CH){	//CHデータ検索
				data_offset++;
				if(data_offset >= MSG_MAX)	break;
			}		
			if(RX_buf_host[data_offset + 2] != 0x40)	format_err_host++;
		
			/*Header CH 判定*/
			if(RX_buf_host[MES_1ST_TOP + data_offset] != HEADER_CH){
				return;			// Header CH 異常
			}
			
			/*終端文字検索*/
			data_len = 2;
			while(RX_buf_host[data_len + data_offset]!= CR){	//@から検索を開始する
				data_len++;
				if((data_len + data_offset) == 257 || (data_len + data_offset) == 513){/*データ長が格納されている配列は無視する(データ長が13(0x0D)の場合があるので)*/
					data_len++;
				}
			}
			data_len++;				/*data_len = CH,Length含 メッセージ全受信データ*/
			if(data_len < 0) data_len = 0;

			mky43_del_message(data_len, data_offset, recv_sa);	//通信メッセージから『CHデータ』『Lengthフィールド』を削除する

			protocol_timer_host(1);		//通信プロトコル解析開始タイマーセット	
		}
	}else{				//ホスト以外のSA(Station Address)からの取得データ
		buf_offset = buf_count_sub;
		if(buf_offset == 0){
			for (i=0; i<MSG_MAX; i++){
				RX_buf_subhost[i] = 0;
			}	
		}
		recv_sa_sub = recv_sa;
		recv_size *= 8;
		for(buf_num=0; buf_num<recv_size; buf_num++){
			i = buf_num/8;
			j = (buf_num%8)/2;	
			if(buf_num%2 == 0){
				if(buf_side == 0){		//recv_data0[]を保存
					RX_buf_subhost[buf_num+buf_offset] = recv_data0[i][j]%0x0100;
				}else{					//recv_data1[]を保存
					RX_buf_subhost[buf_num+buf_offset] = recv_data1[i][j]%0x0100;
				}
			}else{
				if(buf_side == 0){		//recv_data0[]を保存
					RX_buf_subhost[buf_num+buf_offset] = recv_data0[i][j]/0x0100;
				}else{					//recv_data1[]を保存
					RX_buf_subhost[buf_num+buf_offset] = recv_data1[i][j]/0x0100;
				}
			}
			if(RX_buf_subhost[buf_num+buf_offset] != CR	/*CR(0x0D)なし*/
						|| (buf_num == 1)){				/*データ長が格納されている配列は無視する(データ長が13(0x0D)の場合があるので)*/		
				buf_count_sub++;
			}else{							//CR(0x0D)あり
				buf_count_sub = 0;
				break;
			}
			if((buf_num+buf_offset) > MSG_MAX){
				break;
			}
		}
	
		if(buf_count_sub == 0){		//全メッセージ(CRまでのデータ)受信完了
			/*フォーマット判定*/
			data_offset = 0;
			format_err_sub = 0;
			while(RX_buf_subhost[data_offset]!= HEADER_CH){	//CHデータ検索
				data_offset++;
				if(data_offset >= MSG_MAX)	break;
			}		
			if(RX_buf_subhost[data_offset + 2] != 0x40)	format_err_sub++;
			
			/*Header CH 判定*/
			if(RX_buf_subhost[MES_1ST_TOP] != HEADER_CH){
				return;			// Header CH 異常
			}

			/*終端文字検索*/
			data_len = 2;
			while(RX_buf_subhost[data_len + data_offset]!= CR){		//@から検索を開始する
				data_len++;
				if((data_len + data_offset) == 257 || (data_len + data_offset) == 513){/*データ長が格納されている配列は無視する(データ長が13(0x0D)の場合があるので)*/
					data_len++;
				}
			}
			data_len++;				/*data_len = CH,Length含 メッセージ全受信データ*/
			if (data_len < 0) data_len = 0;

			mky43_del_message(data_len, data_offset, recv_sa);	//通信メッセージから『CHデータ』『Lengthフィールド』を削除する
			
			protocol_timer_subhost();		//通信プロトコル解析開始タイマーセット	
		}
	}
}

/****************************************************/
/* Function : mky43_host_link_check                */
/* Summary  : ホスト通信リンク確認                       */
/* Argument : なし                                  */
/* Return   : なし      									                    */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	mky43_host_link_check(void){

	if(B_YES == mky43_ccr_check() &&
		(MKY43.REG.SCR.BIT.RUN != 0)){			//RUNフェーズ確認
		if(com_link == B_NG){					//リンク不成立から復帰した場合
			mky43_set_register();				//MKY43のレジスタ設定
			com_link = B_OK;
		}
		cunet_mcare = MKY43.REG.CCTR.WORD;		//LCARE,MCARE信号発生回数を保存
		
		cunet_mfr4 = MKY43.REG.MFR.DATA[0].DATA;
		cunet_mfr3 = MKY43.REG.MFR.DATA[1].DATA;
		cunet_mfr2 = MKY43.REG.MFR.DATA[2].DATA;
		cunet_mfr1 = MKY43.REG.MFR.DATA[3].DATA;
		
	}else{
		if((com_link != B_NG) && 				//ホストとの通信がリンク不成立
			(MKY43.REG.MFR.DATA[0].DATA & 0x0001) == 0){  	
			mky43_restart();					//MKY43の通信再開
			com_link = B_NG;
		}
	}
}

/****************************************************/
/* Function : mky43_ccr_check                      */
/* Summary  : RS485 or CUnet基板確認                */
/* Argument : なし                                  */
/* Return   : なし      									                   */
/* Caution  : なし                                   */
/* notes    : CCRレジスタから[MKY43_v0]が読込めた場合には、CUnet基板とする。*/
/*          : 読込めない場合には、RS485基板とする。         */
/****************************************************/
short		mky43_ccr_check(void){

	short ret;

	ret = B_NO;
	
	if(	MKY43.REG.CCR.DATA[0].DATA == 0x4B4D &&
		MKY43.REG.CCR.DATA[1].DATA == 0x3459 &&
		MKY43.REG.CCR.DATA[2].DATA == 0x5F33 	){
		com_type = COM_CUNET;		//CUnet基板
		ret = B_YES;
	}
	else{
		com_type = COM_RS485;		//RS485基板
		ret = B_NO;
	}
	
	return (ret);
}

/****************************************************/
/* Function : mky43_add_message                     */
/* Summary  : 通信メッセージのデータ追加              				 */
/* Argument : len,   com_mode      	                */
/* Return   : 追加データ長				                         */
/* Caution  : なし                                   */
/* notes    :                                       */
/*　notes    : 先頭アドレス(1byte)に『CHデータ(1固定)』を入れる */
/*　          次アドレス(1byte)に『Lengthフィールド』を入れる				*/
/*　          メッセージ分割時には、それぞれの先頭に『CHデータ』、		 */
/*           『Lengthフィールド』を入れる									          */
/*        	  Lengthフィールド：「@～CR」の長さ				           */
/*　         ※CUnet通信時、ホスト間のメッセージ時に限る					   */
/****************************************************/
short		mky43_add_message(unsigned short len, short com_mode){

	short cnt;
	short add_len=0;

	if(com_mode == SA_HOST){		//ホスト(SA=0)へ送信する場合
		/*254byte(256-2)以下（メッセージを分割しないで送信する場合）*/
		if(len <= 254){
			/*送信するメッセージを作成*/
			for(cnt=len-1; cnt>=0; cnt--){
				TX_buf_host[cnt+2] = TX_buf_host[cnt];		//2byteシフト
			}
			TX_buf_host[MES_1ST_TOP] = HEADER_CH;		//CHデータ
			TX_buf_host[MES_1ST_TOP+1] = len;			//Lengthフィールド
			add_len = 2;								//追加データ長
		}
		/*255byte～508byte（メッセージを2分割して送信する場合）*/
		else if(len > 254 && len <= 508){
			/*2回目に送信するメッセージを作成*/
			for(cnt=len-1; cnt>=254; cnt--){
				TX_buf_host[cnt+4] = TX_buf_host[cnt];		//4byteシフト
			}
			TX_buf_host[MES_2ND_TOP] = HEADER_CH;		//CHデータ
			TX_buf_host[MES_2ND_TOP+1] = (len-254);	//Lengthフィールド

			/*1回目に送信するメッセージを作成*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_host[cnt+2] = TX_buf_host[cnt];		//2byteシフト
			}
			TX_buf_host[MES_1ST_TOP] = HEADER_CH;		//CHデータ
			TX_buf_host[MES_1ST_TOP+1] = 254;			//Lengthフィールド
			add_len = 4;								//追加データ長
		}
		/*509byte以上（メッセージを3分割して送信する場合）*/
		else{
			/*3回目に送信するメッセージを作成*/
			for(cnt=len-1; cnt>=508; cnt--){
				TX_buf_host[cnt+6] = TX_buf_host[cnt];		//6byteシフト
			}
			TX_buf_host[MES_3RD_TOP] = HEADER_CH;		//CHデータ
			TX_buf_host[MES_3RD_TOP+1] = (len-508);	//Lengthフィールド
		
			/*2回目に送信するメッセージを作成*/
			for(cnt=508; cnt>=254; cnt--){
				TX_buf_host[cnt+4] = TX_buf_host[cnt];		//4byteシフト
			}
			TX_buf_host[MES_2ND_TOP] = HEADER_CH;			//CHデータ
			TX_buf_host[MES_2ND_TOP+1] = 254;				//Lengthフィールド

			/*1回目に送信するメッセージを作成*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_host[cnt+2] = TX_buf_host[cnt];		//2byteシフト
			}
			TX_buf_host[MES_1ST_TOP] = HEADER_CH;			//CHデータ
			TX_buf_host[MES_1ST_TOP+1] = 254;				//Lengthフィールド
			add_len = 6;									//追加データ長
		}
	}else{						//ホスト以外のSA(Station Address)へ送信する場合
		/*254byte(256-2)以下（メッセージを分割しないで送信する場合）*/
		if(len <= 254){
			/*送信するメッセージを作成*/
			for(cnt=len-1; cnt>=0; cnt--){
				TX_buf_subhost[cnt+2] = TX_buf_subhost[cnt];	//2byteシフト
			}
			TX_buf_subhost[MES_1ST_TOP] = HEADER_CH;		//CHデータ
			TX_buf_subhost[MES_1ST_TOP+1] = len;			//Lengthフィールド
			add_len = 2;									//追加データ長
		}
		/*255byte～508byte（メッセージを2分割して送信する場合）*/
		else if(len > 254 && len <= 508){
			/*2回目に送信するメッセージを作成*/
			for(cnt=len-1; cnt>=254; cnt--){
				TX_buf_subhost[cnt+4] = TX_buf_subhost[cnt];	//4byteシフト
			}
			TX_buf_subhost[MES_2ND_TOP] = HEADER_CH;		//CHデータ
			TX_buf_subhost[MES_2ND_TOP+1] = (len-254);		//Lengthフィールド

			/*1回目に送信するメッセージを作成*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_subhost[cnt+2] = TX_buf_subhost[cnt];	//2byteシフト
			}
			TX_buf_subhost[MES_1ST_TOP] = HEADER_CH;		//CHデータ
			TX_buf_subhost[MES_1ST_TOP+1] = 254;			//Lengthフィールド
			add_len = 4;									//追加データ長
		}
		/*509byte以上（メッセージを3分割して送信する場合）*/
		else{
			/*3回目に送信するメッセージを作成*/
			for(cnt=len-1; cnt>=508; cnt--){
				TX_buf_subhost[cnt+6] = TX_buf_subhost[cnt];	//6byteシフト
			}
			TX_buf_subhost[MES_3RD_TOP] = HEADER_CH;		//CHデータ
			TX_buf_subhost[MES_3RD_TOP+1] = (len-508);		//Lengthフィールド
		
			/*2回目に送信するメッセージを作成*/
			for(cnt=508; cnt>=254; cnt--){
				TX_buf_subhost[cnt+4] = TX_buf_subhost[cnt];	//4byteシフト
			}
			TX_buf_subhost[MES_2ND_TOP] = HEADER_CH;		//CHデータ
			TX_buf_subhost[MES_2ND_TOP+1] = 254;			//Lengthフィールド

			/*1回目に送信するメッセージを作成*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_subhost[cnt+2] = TX_buf_subhost[cnt];	//2byteシフト
			}
			TX_buf_subhost[MES_1ST_TOP] = HEADER_CH;		//CHデータ
			TX_buf_subhost[MES_1ST_TOP+1] = 254;			//Lengthフィールド
			add_len = 6;									//追加データ長
		}
	}
	return add_len;
}

/****************************************************/
/* Function : mky43_del_message                     */
/* Summary  : 通信メッセージのデータ削除              	 			*/
/* Argument : len,  offset,  com_mode               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/*　notes    : 先頭アドレス(1byte)の『CHデータ(1固定)』を削除する*/
/*　           次アドレス(1byte)の『Lengthフィールド』を削除する	*/
/*　           メッセージ分割時には、それぞれの先頭の『CHデータ』、		*/
/*            『Lengthフィールド』を削除する								         */
/*　           ※CUnet通信時、ホストからのメッセージ時に限る				 */
/****************************************************/
void	mky43_del_message(unsigned short len, unsigned short offset, short com_mode){

	short cnt;

	if(com_mode == SA_HOST){		//ホスト(SA=0)へ送信する場合
		len_err_host = 0;

		/*256byte以下（分割していないメッセージを受信した場合）*/
		if(len <= 256){
			/*メッセージ長判定*/
			if(RX_buf_host[(MES_1ST_TOP + 1) + offset] != (len-2)){
				len_err_host++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=0; cnt<(len-2); cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 2) + offset];
			}
			RX_buf_host[len-2] = 0;
		}
		/*257byte～512byte（2分割したメッセージを受信した場合）*/
		else if(len > 256 && len <= 512){
			/*1回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_host[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_host++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 2) + offset];
			}
			
			/*2回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_host[(MES_2ND_TOP + 1) + offset] != (len-258)){
				len_err_host++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=254; cnt<(len-4); cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 4) + offset];
			}
			RX_buf_host[len-4] = 0;
		}
		/*513byte以上（3分割したメッセージを受信した場合）*/
		else{
			/*1回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_host[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_host++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 2) + offset];
			}
			
			/*2回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_host[(MES_2ND_TOP + 1) + offset] != 254){
				len_err_host++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=254; cnt<508; cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 4) + offset];
			}			
			
			/*3回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_host[(MES_3RD_TOP + 1) + offset] != (len-514)){
				len_err_host++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=509; cnt<(len-6); cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 6) + offset];
			}
			RX_buf_host[len-6] = 0;
		}
	}else{						//ホスト以外のSA(Station Address)へ送信する場合
		len_err_sub = 0;

		/*256byte以下（分割していないメッセージを受信した場合）*/
		if(len <= 256){
			/*メッセージ長判定*/
			if(RX_buf_subhost[(MES_1ST_TOP + 1) + offset] != (len-2)){
				len_err_sub++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=0; cnt<(len-2); cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 2) + offset];
			}
			RX_buf_subhost[len-2] = 0;
		}
		/*257byte～512byte（2分割したメッセージを受信した場合）*/
		else if(len > 256 && len <= 512){
			/*1回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_subhost[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_sub++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 2) + offset];
			}
			
			/*2回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_subhost[(MES_2ND_TOP + 1) + offset] != (len-258)){
				len_err_sub++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=254; cnt<(len-4); cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 4) + offset];
			}
			RX_buf_subhost[len-4] = 0;
		}
		/*513byte以上（3分割したメッセージを受信した場合）*/
		else{
			/*1回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_subhost[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_sub++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 2) + offset];
			}
			
			/*2回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_subhost[(MES_2ND_TOP + 1) + offset] != 254){
				len_err_sub++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=254; cnt<508; cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 4) + offset];
			}			
			
			/*3回目に受信したメッセージ*/
			/*メッセージ長判定*/
			if(RX_buf_subhost[(MES_3RD_TOP + 1) + offset] != (len-514)){
				len_err_sub++;			//メッセージ長 異常
			}
			/*『CHデータ』、『Lengthフィールド』を削除*/
			for(cnt=509; cnt<(len-6); cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 6) + offset];
			}
			RX_buf_subhost[len-6] = 0;
		}
	}
}
