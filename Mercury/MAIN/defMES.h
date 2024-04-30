/***********************************************/
/* File Name : defMES.h 		         									   */
/*	Summary   : メッセージ通信処理                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//インクルードガード
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
//Cファイル定義
//----------------------------------------------------------------------
extern struct stMES MES[CH_NUMMAX];	//流量計測
extern struct stMES_SUB MES_SUB[CH_NUMMAX];	//流量計測

//----------------------------------------------------------------------
//構造体
//----------------------------------------------------------------------
/*           !!!!!!!!!! 注意 !!!!!!!!!!                       */
/*                                                            */
/* 構造体stMESにメンバ変数を増やしていくと、ファームウェアが正常に起動しなくなるため */
/* 流量計測の変数を増やす場合は、構造体stMES_SUBに追加すること           */
/*                                                            */
/*           !!!!!!!!!! 注意 !!!!!!!!!!                       */
struct stMES{	//流量計測
	
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
	short signal_count;			//FIFOの信号位置
	short non_signal_count;
	short zero_fifo_no_read;
	short zero_fifo_ch_read;
	short zero_signal_count;			//FIFOの信号位置
	
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
	short fow_max_data;	//最大
	short fow_min_data;	//最小
	short rev_max_data;	//最大
	short rev_min_data;	//最小
	short fow_max_data_point; //波形振幅の最大値をとるx座標
	short rev_max_data_point;

	short m0_point_fow[3];	//最小ポイント
	short m0_point_rev[3];	//最小ポイント
	short v0_value_fow[3];	//最小値
	short v0_value_rev[3];	//最小値

	short fow_center_point;	/*MAX の50%の位置*/
	short rev_center_point;	/*MAX の50%の位置*/
	short fow_center_data;	/*MAX の50%の値*/
	short rev_center_data;	/*MAX の50%の値*/
	short center_sign_now;
	short center_sign_old;
	short m0_point_fow_50;
	short m0_point_rev_50;

	short ThreasholdPoint_Fow; //受波しきい値のx座標
	short ThreasholdPoint_Rev; //受波しきい値のx座標
	short fow_max_phase[4];
	short rev_max_phase[4];
	short fow_min_phase[4];
	short rev_min_phase[4];

	char read_ok[1000];
	short count_ok;
	short data_ok;
	
	short correlate;				//実相関値幅係数//
	long sum_abs[SUB_POINT];	//絶対値の和、格納領域
	short min_point_m;	//最小値のoffset値(0-20)
	long sum_max;
	long sum_min;
	short search_start;
	short search_end;
	short max_point_sub;		//fow - rev
	short max_point_sub_f;	//fow - rev
		
	unsigned short delta_ts;		//⊿Ｔｓ
	unsigned short delta_ts_buf[50];
	unsigned short delta_ts0;
	unsigned long delta_ts_sum;		//⊿ts加算データ
	short i_cont;				//移動平均カウンタ

	long vth_sum;
	short vth_count;
	long delta_ts_zero;			//ゼロ調整済みデータ
	long delta_ts_zero_f;		//ゼロ調整済みデータ,フィルタ後
	unsigned short zero_adj_data;	//ゼロ調整データ
	
	short sonic_point_fow_p1;
	short sonic_point_rev_p2;
	short zero_sonic_point_fow_p1;
	short zero_sonic_point_rev_p2;
	
	short viscos_table[21];	//自動リニアライズテーブル
	short viscos_val;			//動粘度設定(30-4000)=0.30-40.00
	short sound_vel;			//音速[m/s]
	short sound_vel_f;

	long flow_vel;		//流速[m/s*10000]
	long flow_vel_a;	//流速[m/s*10000]
	long flow_vel_b;	//流速[m/s*10000]
	long flow_vel_c;	//流速[m/s*10000]
	short ThreasholdPoint;	//温度補正用、音速データ
	
	long ml_min_now;
	long ml_min_old;
	long ml_min_a;
	long ml_min_b;
	long ml_min_c;
	long ml_min_d;
	long max_flow_long;	//フルスケール

	long past_flow[10];				/*瞬時流量(10点)*/
	
	//oqコマンド（ローカットオフ 10点瞬時流量読出し）用
	short pv_now_oq;			//瞬時流量
	long ml_min_oq;			//瞬時流量
	long past_flow_oq[10];	/*瞬時流量(10点)*/
	short past_flow_cnt;			/*瞬時流量(10点)カウンタ*/
	long ml_min_OQ;			//瞬時流量

	short flow_filt_sv;
	long flow_filt_in;
	long flow_filt_out;
	
	short flow_filt_sv_v;
	long flow_filt_in_v;
	long flow_filt_out_v;

	short sonic_filt_sv;	//sonic_point_sub(P1-P2) OVER/UNDER判定用
	short sonic_filt_out;	//sonic_point_sub(P1-P2) OVER/UNDER判定用
	long sonic_filt_in_s;
	long sonic_filt_out_s;

	unsigned short median_ptr;
	long median_buf[32];

	short addit_req;		/*積算要求*/
	short addit_req_w;	/*積算要求*/
	unsigned long	addit_pv;		/*0.1mL*/
	unsigned long	addit_pv_w;		/*0.1mL*/
	unsigned short total_status;
	short addit_watch;		/*積算監視*/
	
	union{
			unsigned long long DWORD;
			struct{
				unsigned long low;
				unsigned long high;
			}WORD;
		}addit_cont;			/*積算流量カウンタ*/
	union{
			unsigned long long DWORD;
			struct{
				unsigned long low;
				unsigned long high;
			}WORD;
		}addit_unit;			/*積算流量単位換算*/
	union{
			unsigned long long DWORD;
			struct{
				unsigned long low;
				unsigned long high;
			}WORD;
		}addit_buff;			/*積算流量バッファ(通信用)*/

	unsigned short	addit_mod;		/*積算流量余り*/	
	
	union{
			unsigned long long UINT64;
			struct{
				unsigned long low;
				unsigned long high;
			}DT_4BYTE;
		}addit_unit_ov;			/*積算流量単位換算(ov用)*/
	union{
			unsigned long long UINT64;
			struct{
				unsigned long low;
				unsigned long high;
			}DT_4BYTE;
		}addit_buff_ov;			/*積算流量バッファ(ov用)*/
	unsigned short	addit_mod_ov;		/*積算流量余り(ov用)*/		

	short clk_phase;
	short wave_monitor_mode;
	short inspect_enable;

	short err_burn_out;
	short err_hold_out;
	unsigned short err_status;	//流量エラー情報（センサ異常等）
	unsigned short alm_status;	//流量警報情報（エンプティセンサ、波形減衰、波形アンバランス）
	
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

	//ゼロクロス演算関連
	short zc_nearzero_point[2][ZC_POINT_MAX];
	short zc_nearzero_data1[2][ZC_POINT_MAX];
	short zc_nearzero_data2[2][ZC_POINT_MAX];
	short zc_nearzero_data3[2][ZC_POINT_MAX];
	short zc_nearzero_data4[2][ZC_POINT_MAX];
	float zc_Tdata_buf[4];
	float zc_Tdata;
	float zc_zero_calc;
	float zc_zero_offset;

	short zc_peak;
	unsigned short ThresholdWave;	/*波形認識閾値*/
	unsigned short ThresholdPeak;	/*波形認識ピーク*/
	unsigned short ThresholdPeakPos;	 /*波形認識ピーク位置*/
	unsigned short ThresholdReq;	/*波形認識実行要求*/
	
	//Windowサーチ
	short ws_FifoCh;
	unsigned long ws_work_add;
};

/*           !!!!!!!!!! 注意 !!!!!!!!!!                       */
/*                                                            */
/* 構造体stMESにメンバ変数を増やしていくと、ファームウェアが正常に起動しなくなるため */
/* 流量計測の変数を増やす場合は、構造体stMES_SUBに追加すること           */
/*                                                            */
/*           !!!!!!!!!! 注意 !!!!!!!!!!                       */
struct stMES_SUB{	//流量計測
	float zc_delta_ts;
	float zc_delta_ts_buf[18];
	float zc_delta_ts_sum;
	float zc_delta_ts_zero;
	
	short fifo_result;
	short zc_peak_req;
};

#ifdef FRQSCH
struct stFrqSch{
	short FrqSchSttFlg; //周波数サーチ開始フラグ
	short MaxAmpLst[10]; //各周波数における振幅最大値(10回分)
	short MaxAmpAvg; //最大振幅平均値
	short MaxAmpFrq; //最大振幅周波数
	short MesAmpCnt; //各周波数における最大周波数測定カウンタ
	short NowFrq; //現在測定中の周波数
	short MesErrCnt; //波形エラー数
	short CenPntCnt; //中心線通過カウンタ
	short WitTim; //待機時間
	short CenPntUndLim; //中心線通過回数下限
	short CenPntOvrLim; //中心線通過回数上限
};

#endif

#endif
