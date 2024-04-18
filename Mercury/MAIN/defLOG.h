/***********************************************/
/* File Name : defLOG.h 		         									   */
/*	Summary   : エラーログ情報定義                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include "define.h"

/************************************************************/
/*	Cファイル定義											*/
/************************************************************/
extern struct stLOG_DETAIL		LOG_DETAIL[CH_NUMMAX][LOGMAX];	//エラーログ詳細情報
extern struct stLOG_DETAIL_DEF	LOG_DETAIL_DEF;					//エラーログ詳細情報

/************************************************************/
/*	構造体													*/
/************************************************************/
struct stLOG_DETAIL{						//エラーログ詳細情報
	short	err_code;						//エラーコード
	short	err_pwon_count;					//エラー発生時の電源オンカウンタ
	long	err_time;						//エラー発生時時間
	long	err_time_now;					//現在時間
	long	flow_quantity;					//流量
	long	flow_velocity;					//流速
	long	sound_speed;					//音速
	
	union{
		unsigned long long DWORD;
		struct{
			unsigned long low;
			unsigned long high;
		}WORD;
	}total_count;							//積算値
	
	long	wave_max;						//受波波形最大値
	long	wave_min;						//受波波形最小値
	long	dt;								//伝搬時間差
	long	correlate;						//相関値幅
	long	zero_offset;					//ゼロ点オフセット
	short	status;							//ステータス
	short	fifo_position;					//FIFO受波波形検出位置
	short	gain_up;						//ゲイン（アップ）
	short	gain_down;						//ゲイン（ダウン）
	short	fifo_ch;						//FIFO CH
	short	p1_p2;							//受波の差(P1-P2)
	short	mail_err_status;				//メール送信エラー
	unsigned short	sum_abs_log[42];				//差分相関値
};

/************************************************************/
/*	設定値のデフォルト値									*/
/************************************************************/
struct stLOG_DETAIL_DEF{					//エラーログ基本情報
	short	err_code;						//エラーコード
	short	err_pwon_count;					//エラー発生時の電源オンカウンタ
	long	err_time;						//エラー発生時の時間
	long	err_time_now;					//現在時間
	long	flow_quantity;					//流量
	long	flow_velocity;					//流速
	long	sound_speed;					//音速
	
	union{
		unsigned long long DWORD;
		struct{
			unsigned long low;
			unsigned long high;
		}WORD;
	}total_count;							//積算値
	
	long	wave_max;						//受波波形最大値
	long	wave_min;						//受波波形最小値
	long	dt;								//伝搬時間差
	long	correlate;						//相関値幅
	long	zero_offset;					//ゼロ点オフセット
	short	status;							//ステータス
	short	fifo_position;					//FIFO受波波形検出位置
	short	gain_up;						//ゲイン（アップ）
	short	gain_down;						//ゲイン（ダウン）
	short	fifo_ch;						//FIFO CH
	short	p1_p2;							//受波の差(P1-P2)
	short	mail_err_status;				//メール送信エラー
	unsigned short	sum_abs_log[42];		//差分相関値
};

