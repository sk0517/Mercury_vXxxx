/***********************************************/
/* File Name : util.c		            									   */
/*	Summary   : ユーティリティ関数	                   */
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
/*	モジュール内定義関数								*/
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
/*	モジュール外定義関数								*/
/********************************************************/
extern void	log_save(short ch, short code);
extern void	eep_write_ch(short, short, short);
extern void eep_write_ch_delay(short ch, short addr, short data);
extern short	eep_read(short rom_addr);
extern void WatchdogReloadSet_dl(uint32_t ui32Base, uint32_t ui32LoadVal);
extern void WatchdogResetEnable_dl(uint32_t ui32Base);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short end_wdt;
unsigned long	cmi_count = 1;

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern uint32_t g_ui32SysClock;
extern short OWwrite[6];

/****************************************************/
/* Function : util_delay                       */
/* Summary  : ディレイ	（※5msec単位）   				*/
/* Argument : target_time:ミリ秒設定	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	util_delay(short target_time){

	unsigned long	rtc_count_buff;
	unsigned long	time_buff;
	unsigned long	target_time_buff;
	
	rtc_count_buff = cmi_count;				//タイマ割込み(5msec)カウンタ取得
	if(target_time < 0 || target_time > 100){
		target_time_buff = 0;							//異常値の場合は「0」設定
	}else{
		target_time_buff = (unsigned long)target_time;	//目標時間設定
	}
	time_buff = rtc_count_buff + (target_time_buff / 5);
	
	while(1){								//経過確認
		if(time_buff > CMI_COUNT_MAX){
			if((time_buff - CMI_COUNT_MAX) < cmi_count){
				break;						//時間経過
			}
		}else{
			if(time_buff < cmi_count){
				break;						//時間経過
			}
		}
	}
}

/****************************************************/
/* Function : util_passed_time                      */
/* Summary  : 経過時間確認     				*/
/*	Argument : basic_time  : 基本時間	(5msec単位)				*/
/*		　　        target_time : 目標時間	(5msec単位)				*/
/* Return   : B_NO ：未経過,  B_YES：経過              */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		util_passed_time(unsigned long basic_time, unsigned long target_time){

	short	status;
	unsigned long	time_buff;

	status = B_NO;
	time_buff = basic_time + target_time;
	
	if(time_buff > CMI_COUNT_MAX){
		if((time_buff - CMI_COUNT_MAX) <= cmi_count){
			status = B_YES;					//目標時間を経過
		}
	}else{
		if(time_buff <= cmi_count){
			status = B_YES;					//目標時間を経過
		}
	}

	return (status);
}

/****************************************************/
/* Function : err_judge_status                      */
/* Summary  : エラー判定処理    				*/
/* Argument : pch                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	err_judge_status(short pch){

	unsigned short err_status, alm_status, total_status;
	unsigned short err_status_buf, alm_status_buf, total_status_buf;

	if(pch >= CH_NUMMAX){
		return;
	}
	
	err_status = MES[pch].err_status;							//現流量エラー情報
	err_status_buf = MAIN[pch].err_judge;			//流量エラー情報バッファ

	err_status_control(pch);								//エラーステータス処理（通信用）
	
	if(err_status_buf != err_status){							//流量エラー情報変化あり

		//ゼロ点調整情報ビットが0→1に変化した場合
		if(((err_status_buf & ERR_JUDGE_ZERO) == 0)			//ゼロ点調整情報ビット
				&&((err_status & ERR_JUDGE_ZERO) != 0)){
					
			MAIN[pch].err_condition |= CON_ZERO;
					
			if((err_status & ERR_JUDGE_EMPTY) != 0){			//エンプティセンサー
				MAIN[pch].err_judge_time[0] = 0;
				log_save(pch, ERR_ZERO_EMPTY);					//エラーログ情報の登録
			}else if((err_status & ERR_JUDGE_CALC) != 0){		//演算異常
				MAIN[pch].err_judge_time[1] = 0;
				log_save(pch, ERR_ZERO_CALC);					//エラーログ情報の登録
			}else if((err_status & ERR_JUDGE_LEVEL) != 0){	//波形アンバランス
				MAIN[pch].err_judge_time[2] = 0;
				log_save(pch, ERR_ZERO_LEVEL);					//エラーログ情報の登録
			}else if((err_status & ERR_JUDGE_AGC) != 0){		//AGC不能//
				MAIN[pch].err_judge_time[3] = 0;
				log_save(pch, ERR_ZERO_AGC);					//エラーログ情報の登録
			}else if((err_status & ERR_JUDGE_WAVE) != 0){		//波形減衰
				MAIN[pch].err_judge_time[5] = 0;
				log_save(pch, ERR_ZERO_WAVE);					//エラーログ情報の登録
			}
		}

		//流量エラー情報ビットが0→1に変化した場合
		if(((err_status_buf & ERR_JUDGE_EMPTY) == 0)			//エンプティセンサー
			&&((err_status & ERR_JUDGE_EMPTY) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//ゼロ点調整時
				MAIN[pch].err_judge_time[0] = 0;
				log_save(pch, ERR_ZERO_EMPTY);					//エラーログ情報の登録
			}else{												//流量測定時
				MAIN[pch].err_judge_time[0] = cmi_count;		//エラー発生時間登録
				log_save(pch, ERR_MESR_EMPTY_L);				//エラーログ情報の登録
			}
		}

		if(((err_status_buf & ERR_JUDGE_CALC) == 0)			//演算異常
			&&((err_status & ERR_JUDGE_CALC) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//ゼロ点調整時
				MAIN[pch].err_judge_time[1] = 0;
				log_save(pch, ERR_ZERO_CALC);					//エラーログ情報の登録
			}else{												//流量測定時
				MAIN[pch].err_judge_time[1] = cmi_count;		//エラー発生時間登録
				log_save(pch, ERR_MESR_CALC_L);					//エラーログ情報の登録
			}
		}
			
		if(((err_status_buf & ERR_JUDGE_LEVEL) == 0)			//波形アンバランス
			&&((err_status & ERR_JUDGE_LEVEL) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//ゼロ点調整時
				MAIN[pch].err_judge_time[2] = 0;
				log_save(pch, ERR_ZERO_LEVEL);					//エラーログ情報の登録
			}else{
				MAIN[pch].err_judge_time[2] = cmi_count;		//エラー発生時間登録
				log_save(pch, ERR_MESR_LEVEL_L);				//エラーログ情報の登録
			}
		}

		if(((err_status_buf & ERR_JUDGE_AGC) == 0)				//AGC不能//
			&&((err_status & ERR_JUDGE_AGC) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//ゼロ点調整時
				MAIN[pch].err_judge_time[3] = 0;
				log_save(pch, ERR_ZERO_AGC);					//エラーログ情報の登録
			}else{
				MAIN[pch].err_judge_time[3] = cmi_count;		//エラー発生時間登録
				log_save(pch, ERR_MESR_AGC_L);					//エラーログ情報の登録
			}
		}

		if(((err_status_buf & ERR_JUDGE_REVERSE) == 0)			//逆流異常
			&&((err_status & ERR_JUDGE_REVERSE) != 0)){
			MAIN[pch].err_judge_time[4] = cmi_count;			//エラー発生時間登録
		}
		
		if(((err_status_buf & ERR_JUDGE_WAVE) == 0)			//波形減衰
			&&((err_status & ERR_JUDGE_WAVE) != 0)){
			if((err_status & ERR_JUDGE_ZERO) != 0){			//ゼロ点調整時
				MAIN[pch].err_judge_time[5] = 0;
				log_save(pch, ERR_ZERO_WAVE);					//エラーログ情報の登録
			}else{
				MAIN[pch].err_judge_time[5] = cmi_count;		//エラー発生時間登録
				log_save(pch, ERR_MESR_WAVE_L);					//エラーログ情報の登録
			}
		}

		if(((err_status_buf & ERR_JUDGE_OVERFLOW) == 0)		//オーバーフロー
			&&((err_status & ERR_JUDGE_OVERFLOW) != 0)){
			MAIN[pch].err_judge_time[6] = cmi_count;			//エラー発生時間登録
			log_save(pch, ERR_MESR_OVERFLOW);					//エラーログ情報の登録
		}
		
		if(((err_status_buf & ERR_JUDGE_10POINT) == 0)			//10点データ無効
			&&((err_status & ERR_JUDGE_10POINT) != 0)){
			log_save(pch, ERR_10POINT);							//エラーログ情報の登録
		}
		if(((err_status_buf & ERR_JUDGE_EEPROM) == 0)			//EEPROMエラー
			&&((err_status & ERR_JUDGE_EEPROM) != 0)){
			log_save(pch, ERR_EEPROM);							//エラーログ情報の登録
		}
		if(((err_status_buf & ERR_JUDGE_CUNET) == 0)			//CUnetエラー
			&&((err_status & ERR_JUDGE_CUNET) != 0)){
			log_save(pch, ERR_CUNET);							//エラーログ情報の登録
		}
		if(((err_status_buf & ERR_JUDGE_UNSTABLE) == 0)		//ゼロ調整計測時間発散エラー
			&&((err_status & ERR_JUDGE_UNSTABLE) != 0)){
			log_save(pch, ERR_ZERO_UNSTABLE);					//エラーログ情報の登録
		}
		if(((err_status_buf & ERR_JUDGE_RESTART) == 0)			//再起動
			&&((err_status & ERR_JUDGE_RESTART) != 0)){
			log_save(pch, ERR_RESTART);							//エラーログ情報の登録
		}

		//流量エラー情報ビットが1→0に変化した場合
		if(((err_status_buf & ERR_JUDGE_EMPTY) != 0)			//エンプティセンサー
			&&((err_status & ERR_JUDGE_EMPTY) == 0)){
			MAIN[pch].err_judge_time[0] = 0;					//エラー発生時間クリア
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//流量測定時のエラー
				remove_errcode(pch, ERR_MESR_EMPTY_L);			//エラーコード解除
				remove_errcode(pch, ERR_MESR_EMPTY_H);			//
			}													//ゼロ調整時のエラーコードは解除しない
		}

		if(((err_status_buf & ERR_JUDGE_CALC) != 0)			//演算異常
			&&((err_status & ERR_JUDGE_CALC) == 0)){
			MAIN[pch].err_judge_time[1] = 0;					//エラー発生時間クリア
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//流量測定時のエラー
				remove_errcode(pch, ERR_MESR_CALC_L);			//エラーコード解除
				remove_errcode(pch, ERR_MESR_CALC_H);			//
			}													//ゼロ調整時のエラーコードは解除しない
		}

		if(((err_status_buf & ERR_JUDGE_LEVEL) != 0)			//波形アンバランス
			&&((err_status & ERR_JUDGE_LEVEL) == 0)){
			MAIN[pch].err_judge_time[2] = 0;					//エラー発生時間クリア
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//流量測定時のエラー
				remove_errcode(pch, ERR_MESR_LEVEL_L);			//エラーコード解除
				remove_errcode(pch, ERR_MESR_LEVEL_H);			//
			}													//ゼロ調整時のエラーコードは解除しない
		}

		if(((err_status_buf & ERR_JUDGE_AGC) != 0)				//AGC不能//
			&&((err_status & ERR_JUDGE_AGC) == 0)){
			MAIN[pch].err_judge_time[3] = 0;					//エラー発生時間クリア
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//流量測定時のエラー
				remove_errcode(pch, ERR_MESR_AGC_L);			//エラーコード解除
				remove_errcode(pch, ERR_MESR_AGC_H);			//
			}													//ゼロ調整時のエラーコードは解除しない
		}

		if(((err_status_buf & ERR_JUDGE_REVERSE) != 0)			//逆流異常
			&&((err_status & ERR_JUDGE_REVERSE) == 0)){
			MAIN[pch].err_judge_time[4] = 0;					//エラー発生時間クリア
			remove_errcode(pch, ERR_MESR_REVERSE);				//エラーコード解除
		}

		if(((err_status_buf & ERR_JUDGE_WAVE) != 0)			//波形減衰
			&&((err_status & ERR_JUDGE_WAVE) == 0)){
			MAIN[pch].err_judge_time[5] = 0;					//エラー発生時間クリア
			if((err_status_buf & ERR_JUDGE_ZERO) == 0){		//流量測定時のエラー
				remove_errcode(pch, ERR_MESR_WAVE_L);			//エラーコード解除
				remove_errcode(pch, ERR_MESR_WAVE_H);			//
			}													//ゼロ調整時のエラーコードは解除しない
		}

		if(((err_status_buf & ERR_JUDGE_OVERFLOW) != 0)		//オーバーフロー
			&&((err_status & ERR_JUDGE_OVERFLOW) == 0)){
			MAIN[pch].err_judge_time[6] = 0;					//エラー発生時間クリア
			remove_errcode(pch, ERR_MESR_OVERFLOW);				//エラーコード解除
		}

		if(((err_status_buf & ERR_JUDGE_10POINT) != 0)			//10点データ無効
			&&((err_status & ERR_JUDGE_10POINT) == 0)){
			remove_errcode(pch, ERR_10POINT);					//エラーコード解除
		}
		if(((err_status_buf & ERR_JUDGE_EEPROM) != 0)			//EEPROMエラー
			&&((err_status & ERR_JUDGE_EEPROM) == 0)){
			remove_errcode(pch, ERR_EEPROM);					//エラーコード解除
		}
		if(((err_status_buf & ERR_JUDGE_CUNET) != 0)			//CUnetエラー
			&&((err_status & ERR_JUDGE_CUNET) == 0)){
			remove_errcode(pch, ERR_CUNET);						//エラーコード解除
		}
		
		MAIN[pch].err_judge = MES[pch].err_status;				//流量エラー情報更新
		
	}

	if(MAIN[pch].led_err_status == 0){		//エラーステータスクリア
		if(reset_factor == RESTART_WDOG){	//再起動エラーが発生した場合
			MAIN[pch].com_err_status = ERR_RESTART;	//再起動エラーセット	
		}
	}

	alm_status = MES[pch].alm_status;			//現流量警告情報
	alm_status_buf = MAIN[pch].alm_judge;	//流量警告情報バッファ
	if(alm_status_buf != alm_status){			//流量警告情報変化あり
		//流量警告情報ビットが0→1に変化した場合
		if(((alm_status_buf & ALM_JUDGE_EMPTY) == 0)	//エンプティセンサー
				&&((alm_status & ALM_JUDGE_EMPTY) != 0)){
			MAIN[pch].alm_judge_time[0] = cmi_count;		//警告発生時間登録
		}

		//流量警告情報ビットが1→0に変化した場合
		if(((alm_status_buf & ALM_JUDGE_EMPTY) != 0)	//エンプティセンサー
				&&((alm_status & ALM_JUDGE_EMPTY) == 0)){
			MAIN[pch].alm_judge_time[0] = 0;						//警告発生時間クリア
		}

		MAIN[pch].alm_judge = MES[pch].alm_status;		//警告情報更新
	}

	if((alm_status & ALM_JUDGE_GAIN) != 0){		//アンプゲイン急変
		log_save(pch, ALM_MESR_GAIN);		//エラーステータスの登録のみ（エラーログは登録しない）		
	}

	total_status = MES[pch].total_status;		//積算監視機能情報
	total_status_buf = MAIN[pch].total_judge;	//積算監視機能情報バッファ
	
	//積算監視機能情報ビットが0→1に変化した場合
	if(((total_status_buf & TTL_JUDGE_OVERFLOW) == 0)		//積算値オーバーフロー
		&&((total_status & TTL_JUDGE_OVERFLOW) != 0)){
		log_save(pch, TTL_OVERFLOW);
	}else if((total_status & TTL_JUDGE_OVERFLOW) != 0){
		err_priority(pch, TTL_OVERFLOW);				//エラーステータスをONする
	}else{
		;
	}
	if((total_status & TTL_JUDGE_CACL_ERR) != 0){			//積算値演算異常
		log_save(pch, TTL_CACL_ERR);
		MES[pch].total_status &= ~TTL_JUDGE_CACL_ERR;		//クリア
	}
	MAIN[pch].total_judge = MES[pch].total_status;		//積算監視機能情報更新
	
	//エラー情報の判定
	err_status = MES_SUB[pch].err_status_sub;	//エラー情報
	err_status_buf = MAIN[pch].err_sub_judge;	//エラー情報バッファ
	if(err_status_buf != err_status){			//エラー情報変化あり
		//エラー情報ビットが0→1に変化した場合
		if(((err_status_buf & ERR_JUDGE_DEVICE) == 0)	//メモリデバイス異常
			&&((err_status & ERR_JUDGE_DEVICE) != 0)){
			log_save(pch, ERR_DEVICE);			//エラーログ情報の登録
		}
		//エラー情報ビットが1→0に変化した場合
		if(((err_status_buf & ERR_JUDGE_DEVICE) != 0)	//メモリデバイス異常
			&&((err_status & ERR_JUDGE_DEVICE) == 0)){
			remove_errcode(pch, ERR_DEVICE);	//エラーコード解除
		}
	}
	MAIN[pch].err_sub_judge = MES_SUB[pch].err_status_sub;	//エラー情報の更新
}

/****************************************************/
/* Function : err_judge_holdtime                    */
/* Summary  : エラーホールドタイム判定処理    				*/
/* Argument : pch                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
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
	
	if((MES[pch].err_status & ERR_BURN_OUT) == 0){	//バーンアウト対象エラー無し
		MES[pch].err_burn_out = B_OFF;					//バーンアウトしない
	}
	if(MES[pch].err_status == 0){						//エラー無し
		MES[pch].err_hold_out = B_OFF;					//ホールドアウトクリア
	}

	hold_time = (unsigned long)SVD[pch].err_hold_time;				//エラーホールドタイム(1secを1で表す）
	reverse_time = (unsigned long)SVD[pch].reverse_time;		//逆流判定時間(1.0secを10で表す)
	alarm_time = (unsigned long)SVD[pch].alm_hold_time;			//警告出力時間(0.01secを1で表す）
	//時間を5msec単位に変換する
	hold_time *= 200;
	reverse_time *= 20;
	alarm_time *= 2;
	
	for(i_cnt=0; i_cnt<7; i_cnt++){
		judge_time[i_cnt] = MAIN[pch].err_judge_time[i_cnt];
	}
	for(i_cnt=0; i_cnt<3; i_cnt++){
		alm_judge_time[i_cnt] = MAIN[pch].alm_judge_time[i_cnt];
	}

	if(judge_time[4] != 0){											//逆流異常
		if(B_YES == util_passed_time(judge_time[4], reverse_time)){	//逆流判定時間経過確認
			MAIN[pch].err_judge_time[4] = 0;
			MES[pch].err_burn_out = B_OFF;								//バーンアウトしない
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
			log_save(pch, ERR_MESR_REVERSE);							//エラーログ情報の登録		
		}
	}

	if(judge_time[5] != 0){										//波形減衰
		if(B_YES == util_passed_time(judge_time[5], hold_time)){	//エラーホールドタイム経過確認
			MAIN[pch].err_judge_time[5] = 0;
			MES[pch].err_burn_out = B_OFF;							//バーンアウトしない
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
			log_save(pch, ERR_MESR_WAVE_H);							//エラーログ情報の登録		
		}
	}	
	
	if(judge_time[0] != 0){										//エンプティセンサー
		if(B_YES == util_passed_time(judge_time[0], hold_time)){	//エラーホールドタイム経過確認
			MAIN[pch].err_judge_time[0] = 0;
			MES[pch].err_burn_out = B_ON;							//バーンアウト
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
			log_save(pch, ERR_MESR_EMPTY_H);						//エラーログ情報の登録		
		}
	}

	if(judge_time[1] != 0){										//演算異常
		if(B_YES == util_passed_time(judge_time[1], hold_time)){	//エラーホールドタイム経過確認
			MAIN[pch].err_judge_time[1] = 0;
			MES[pch].err_burn_out = B_ON;							//バーンアウト
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
			log_save(pch, ERR_MESR_CALC_H);							//エラーログ情報の登録		
		}
	}

	if(judge_time[2] != 0){										//波形アンバランス
		if(B_YES == util_passed_time(judge_time[2], hold_time)){	//エラーホールドタイム経過確認
			MAIN[pch].err_judge_time[2] = 0;
			MES[pch].err_burn_out = B_ON;							//バーンアウト
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
			log_save(pch, ERR_MESR_LEVEL_H);						//エラーログ情報の登録		
		}
	}
	
	if(judge_time[3] != 0){										//AGC不能//
		if(B_YES == util_passed_time(judge_time[3], hold_time)){	//エラーホールドタイム経過確認
			MAIN[pch].err_judge_time[3] = 0;
			MES[pch].err_burn_out = B_ON;							//バーンアウト
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
			log_save(pch, ERR_MESR_AGC_H);							//エラーログ情報の登録		
		}
	}

	if(judge_time[6] != 0){										//オーバーフロー
		if(B_YES == util_passed_time(judge_time[6], hold_time)){	//エラーホールドタイム経過確認
			MAIN[pch].err_judge_time[6] = 0;
			MES[pch].err_burn_out = B_ON;							//バーンアウト
			MES[pch].err_hold_out = B_ON;							//ホールドアウト
			err_status_control(pch);			//エラーステータス処理（通信用）
		}
	}

	if(alm_judge_time[0] != 0){					//エンプティセンサー（警告）
		if(B_YES == util_passed_time(alm_judge_time[0], alarm_time)){	//警告出力時間経過確認
			log_save(pch, ALM_MESR_EMPTY);	//エラーステータスの登録のみ（エラーログは登録しない）		
		}
	}
}

/****************************************************/
/* Function : err_status_control                    */
/* Summary  : エラーステータス処理（常時更新用）    				*/
/* Argument : pch                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
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
	
	err_status = MES[pch].err_status;			//現流量エラー情報
	com_err_status = MAIN[pch].com_err_status;
	total_status = MES[pch].total_status;
	condition_bit = 0;
	condition_bit_cu = 0;

	if((err_status & ERR_JUDGE_EMPTY) != 0){	//エンプティセンサ
		condition_bit	 |= CON_EMPTY;
		if(MES[pch].err_hold_out == B_ON){		//CUnet通信のアラーム発生ステータスはホールドアウト時にbitを立てる
 		condition_bit_cu |= CON_EMPTY_CU;
		}
	}

	if((err_status & ERR_JUDGE_CALC) != 0){	//演算異常
		condition_bit	 |= CON_CALC;
		condition_bit_cu |= CON_CALC_CU;
	}

	if((err_status & ERR_JUDGE_AGC) != 0){		//AGC不能//
		condition_bit	 |= CON_GAIN_OV;
		condition_bit_cu |= CON_AGC_CU;
	}
	
	if((err_status & ERR_JUDGE_LEVEL) != 0){	//波形アンバランス
		condition_bit	 |= CON_LEVEL;
		condition_bit_cu |= CON_LEVEL_CU;
	}

	if((err_status & ERR_JUDGE_REVERSE) != 0){	//逆流異常
		condition_bit	 |= CON_REV;
		condition_bit_cu |= CON_REV_CU;
	}

	if((err_status & ERR_JUDGE_WAVE) != 0){	//波形減衰
		condition_bit	 |= CON_WAVE;
		condition_bit_cu |= CON_WAVE_CU;
	}

	if((err_status & ERR_JUDGE_OVERFLOW) != 0){//オーバーフロー
		condition_bit_cu |= CON_OVER_CU;
	}

	if((total_status & TTL_JUDGE_REACH) != 0){//積算目標値到達
		condition_bit_cu |= CON_TOTAL_CU;
	}
	
	if((err_status & ~ERR_JUDGE_ZERO) != 0){			//エラー有り
		if(MES[pch].err_hold_out == B_ON){		//ホールドアウト
			condition_bit |= CON_HOLDOUT;		//ホールドタイム以上
		}else{
			condition_bit |= CON_HOLD;			//ホールドタイム未満
		}
	}

	//ゼロ点調整エラー有り
	if(com_err_status == ERR_ZERO_EMPTY || com_err_status == ERR_ZERO_WAVE ||
		com_err_status == ERR_ZERO_CALC || com_err_status == ERR_ZERO_LEVEL ||
		com_err_status == ERR_ZERO_AGC || com_err_status == ERR_ZERO_UNSTABLE){
		condition_bit |= CON_ZERO;			//ゼロ点調整エラー
	}
	
	MAIN[pch].err_condition = condition_bit;		//エラー状態更新
	MAIN[pch].err_condition_cu = condition_bit_cu;	//エラー状態更新
}

/****************************************************/
/* Function : action_status_control                 */
/* Summary  : 動作ステータス処理（通信用）    				*/
/* Argument : pch,  act             	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	action_status_control(short pch, short act){

	if(pch >= CH_NUMMAX){
		return;
	}
	if(act >= ACT_STS_OTHER){
		return;
	}
	
	/* テストモード時は強制更新 */
	if(act == ACT_STS_NORMAL){
		if(MES[pch].test_enable != 0 || MES[pch].test_err_enable != 0){
			act = ACT_STS_TEST;
		}
	}
	MAIN[pch].com_act_status = act;	//動作ステータス更新
}

/****************************************************/
/* Function : action_status_check                 */
/* Summary  : 動作ステータス確認（通信用）    				*/
/* Argument : pch                  	               */
/* Return   : 動作ステータス			                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		action_status_check(short pch){

	short act;

	act = ACT_STS_NORMAL;	
	if(pch >= CH_NUMMAX){
		return(act);
	}

	act = MAIN[pch].com_act_status;	//動作ステータス

	return(act);
}

/****************************************************/
/* Function : pwon_count                 */
/* Summary  : 再起動回数の更新    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
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
/* Summary  : ビット反転    				*/
/* Argument : データ（32ビット）           	               */
/* Return   : ビット反転データ		                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
unsigned long invert_data(unsigned long data){
	
	return (data ^ 0xFFFFFFFF);
}

/****************************************************/
/* Function : delay                 */
/* Summary  : ディレイ    				*/
/* Argument : delay_count           	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : delay_count : 10 ≒ delaytime : 1us
 ****************************************************/
void delay(unsigned short delay_count){

	volatile unsigned short counter;
	
	for(counter = 0; counter < delay_count; counter++){
		;
	}
}

/****************************************************/
/* Function : reset_factor_check                 */
/* Summary  : リセット発生要因の確認    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	reset_factor_check(void){

	short	ch;
	short	reset_sts;
	uint32_t	ui32Causes;

	reset_sts = RESTART_NORMAL;

	// [TM4C] : その他の場合は RX 版 FW に合わせて RESTART_TERM をセット 
	ui32Causes = SysCtlResetCauseGet();
	SysCtlResetCauseClear(ui32Causes);
	if((ui32Causes & SYSCTL_CAUSE_WDOG0) != 0){					//ウォッチドッグタイマリセット
		reset_sts = RESTART_WDOG;
		end_wdt = 1;
	}else{
		if((ui32Causes & SYSCTL_CAUSE_WDOG1) != 0){					//再起動用ウォッチドッグタイマリセット
			reset_sts = RESTART_NORMAL;
		}else{
			if((ui32Causes & SYSCTL_CAUSE_BOR) != 0){					//Brown-out reset 
				reset_sts = RESTART_POWER;
			}else{
				if((ui32Causes & SYSCTL_CAUSE_POR) != 0){	//パワーオンリセット		
					reset_sts = RESTART_NORMAL;	
				}else{								//端子リセット 
					reset_sts = RESTART_TERM;
				}
			}
		}
	}

	reset_factor = reset_sts;							//再起動要因保存
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		if(reset_sts == RESTART_WDOG){
			MES[ch].err_status |= ERR_JUDGE_RESTART;	//再起動エラーセット	
		}
	}
}

/****************************************************/
/* Function : reset_control                 */
/* Summary  : リセット処理(独立ウォッチドッグタイマを使用する) 				*/
/* Argument : なし                   	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	reset_control(void){

 WatchdogResetDisable(WATCHDOG0_BASE);

	IntDisable(INT_TIMER0A);	/*定周期割込み停止*/

	WatchdogReloadSet_dl(WATCHDOG1_BASE, g_ui32SysClock / 1000);	// 1ms 後に再起動 
	WatchdogResetEnable_dl(WATCHDOG1_BASE);

	while(1);

}

/****************************************************/
/* Function : non_sensor_control                 */
/* Summary  : センサ無し設定時の処理                 				*/
/* Argument : pch                   	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	non_sensor_control(short pch){

	short	i_cnt;

	if(MES[pch].test_enable != 0){		//強制エラー出力モード確認
		 MES[pch].ml_min_now = (long)MES[pch].max_flow_long * (long)MES[pch].test_flow / 100;
	}	
	else MES[pch].ml_min_now = 0;				//瞬時流量クリア
	
	MES[pch].err_status = 0;					//エラー情報クリア
	MES[pch].alm_status = 0;					//警報情報クリア
	MES[pch].total_status = 0;					//積算情報クリア

	MAIN[pch].err_judge = 0;					//エラー判定情報クリア
	MAIN[pch].err_condition = 0;				//エラーステータスクリア
 if(MAIN[pch].com_act_status != ACT_STS_WRITE){ //動作ステータスが設定書込み中以外の場合
	 MAIN[pch].com_act_status = 0;				//動作ステータスクリア（通信用） 
	}
	MAIN[pch].com_err_status = 0;				//エラーステータスクリア （通信用）
	MAIN[pch].led_err_status = 0;				//エラーステータスクリア(表示用)
	for(i_cnt=0; i_cnt<6; i_cnt++){
		MAIN[pch].err_judge_time[i_cnt] = 0;	//エラー発生時間クリア
	}
}

/****************************************************/
/* Function : util_eep_allwrite                 */
/* Summary  : EEPROM書き込み                      				*/
/* Argument : pch,   opt            	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void util_eep_allwrite(short pch, short opt){

	short i_cnt;
	short ch_cnt;
	short ch;
	unsigned short *pt;
	unsigned short *pt1;
	unsigned short *pt2;

	if(pch >= CH_NUMMAX) return;

	ch = pch;  //書込みCH

 /*** ユーザパラメータ ***/
	if((opt & WR_USER_PARA) != 0){	 /*ユーザパラメータ書込み*/
		eep_write_ch_delay(ch, (short)(&SVD[ch].burnout - &SVD[ch].max_flow), SVD[ch].burnout);						//バーンアウト種別
		eep_write_ch_delay(ch, (short)(&SVD[ch].burnout_value - &SVD[ch].max_flow), SVD[ch].burnout_value);	//バーンアウト入力値		
		eep_write_ch_delay(ch, (short)(&SVD[ch].viscos - &SVD[ch].max_flow), SVD[ch].viscos);					   //動粘度
		eep_write_ch_delay(ch, (short)(&SVD[ch].viscos_auto - &SVD[ch].max_flow), SVD[ch].viscos_auto);		//動粘度固定/測定
		eep_write_ch_delay(ch, (short)(&SVD[ch].err_hold_time - &SVD[ch].max_flow), SVD[ch].err_hold_time);		//エラーホールドタイム
		eep_write_ch_delay(ch, (short)(&SVD[ch].reverse_time - &SVD[ch].max_flow), SVD[ch].reverse_time);	//逆流判定時間
		eep_write_ch_delay(ch, (short)(&SVD[ch].reverse_level - &SVD[ch].max_flow), SVD[ch].reverse_level);	//逆流判定閾値
	}
	if((opt & WR_USPAR_DEVICE) != 0){	 /*ユーザパラメータ書込み(メモリデバイス)*/ /*ﾒﾓﾘﾃﾞﾊﾞｲｽから読込んだｾﾝｻ情報だけをEEPROMに書込むため、WR_USER_PARAとWR_USPAR_DEVICEに分割*/
		eep_write_ch_delay(ch, (short)(&SVD[ch].max_flow - &SVD[ch].max_flow), SVD[ch].max_flow);				//フルスケール
		eep_write_ch_delay(ch, (short)(&SVD[ch].unit - &SVD[ch].max_flow), SVD[ch].unit);				//フルスケール小数点位置、単位
		eep_write_ch_delay(ch, (short)(&SVD[ch].sensor_size - &SVD[ch].max_flow), SVD[ch].sensor_size);			//センサ種別
		eep_write_ch_delay(ch, (short)(&SVD[ch].k_factor - &SVD[ch].max_flow), SVD[ch].k_factor);					//Kファクタ		
		eep_write_ch_delay(ch, (short)(&SVD[ch].damp - &SVD[ch].max_flow), SVD[ch].damp);				//ダンピング		
		eep_write_ch_delay(ch, (short)(&SVD[ch].low_cut - &SVD[ch].max_flow), SVD[ch].low_cut);					//ローカットOFF
	}

	/*** ユーザリニアライズ ***/
	if((opt & WR_USER_LINR) != 0){	 /*ユーザリニアライズ書込み*/
	 eep_write_ch_delay(ch, (short)(&SVD[ch].uslnr_num - &SVD[ch].max_flow), SVD[ch].uslnr_num);				//補正点数、小数点位置
	 pt1 = &SVD[ch].uslnr_out1.WORD.low;
	 pt2 = &SVD[ch].uslnr_in1.WORD.low;
	 for(i_cnt=1; i_cnt<=30; i_cnt++){
		 eep_write_ch_delay(ch, (short)(pt1 - &SVD[ch].max_flow), *pt1);		//補正出力
		 eep_write_ch_delay(ch, (short)(pt2 - &SVD[ch].max_flow), *pt2);  //補正入力
		 pt1++;		pt2++;
	 }
	}

 /*** メーカパラメータ ***/
	if((opt & WR_MAKER_PARA) != 0){	 /*メーカパラメータ書込み*/
	 eep_write_ch_delay(ch, (short)(&SVD[ch].drive_freq - &SVD[ch].max_flow), SVD[ch].drive_freq);		//駆動周波数
	 eep_write_ch_delay(ch, (short)(&SVD[ch].drive_pls - &SVD[ch].max_flow), SVD[ch].drive_pls);				//駆動パルス数
	 eep_write_ch_delay(ch, (short)(&SVD[ch].search_sw - &SVD[ch].max_flow), SVD[ch].search_sw);				//相関値サーチ
	 eep_write_ch_delay(ch, (short)(&SVD[ch].gain_step - &SVD[ch].max_flow), SVD[ch].gain_step);				//ゲイン制御ステップ幅
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sound_vel_filter - &SVD[ch].max_flow), SVD[ch].sound_vel_filter);		//音速フィルタ時定数	
	 eep_write_ch_delay(ch, (short)(&SVD[ch].fifo_ch_init - &SVD[ch].max_flow), SVD[ch].fifo_ch_init);			//FIFO CH 初期値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sound_vel_sel - &SVD[ch].max_flow), SVD[ch].sound_vel_sel);		//音速固定・測定切替
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sound_vel_fix - &SVD[ch].max_flow), SVD[ch].sound_vel_fix);		//音速固定値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].hldt - &SVD[ch].max_flow), SVD[ch].hldt);						//異常ホールド時間選択
	 eep_write_ch_delay(0, (short)(&SVD[0].cunet_delay - &SVD[0].max_flow), SVD[0].cunet_delay);			//CUnet 再送信待機時間	※全CH共通
	 eep_write_ch_delay(ch, (short)(&SVD[ch].wave_vth - &SVD[ch].max_flow), SVD[ch].wave_vth);				//エンプティセンサ判定閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].balance_level - &SVD[ch].max_flow), SVD[ch].balance_level);		//波形アンバランス閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].saturation_level - &SVD[ch].max_flow), SVD[ch].saturation_level);//AGC不能判定閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].correlate_level - &SVD[ch].max_flow), SVD[ch].correlate_level);	//演算異常判定閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].correlate_time - &SVD[ch].max_flow), SVD[ch].correlate_time);	//演算異常判定回数
	 eep_write_ch_delay(ch, (short)(&SVD[ch].attenuate_level - &SVD[ch].max_flow), SVD[ch].attenuate_level);	//波形減衰判定閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_wave_vth - &SVD[ch].max_flow), SVD[ch].alm_wave_vth);				//エンプティセンサ警告閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_gain_level - &SVD[ch].max_flow), SVD[ch].alm_gain_level);		//アンプゲイン警告閾値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_gain_count - &SVD[ch].max_flow), SVD[ch].alm_gain_count);	//アンプゲイン警告ｶｳﾝﾄ
	 eep_write_ch_delay(ch, (short)(&SVD[ch].alm_hold_time - &SVD[ch].max_flow), SVD[ch].alm_hold_time);	//警告出力時間	
	 eep_write_ch_delay(ch, (short)(&SVD[ch].filter_mode - &SVD[ch].max_flow), SVD[ch].filter_mode);			//フィルタモード
	 eep_write_ch_delay(ch, (short)(&SVD[ch].filter_avg - &SVD[ch].max_flow), SVD[ch].filter_avg);			//移動平均値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].LL_enable - &SVD[ch].max_flow), SVD[ch].LL_enable);			//薬液リニアライズモード選択
	 eep_write_ch_delay(ch, (short)(&SVD[ch].LL_kind - &SVD[ch].max_flow), SVD[ch].LL_kind);				//薬液リニアライズモード・薬液種別
	 eep_write_ch_delay(ch, (short)(&SVD[ch].total_offset_enable - &SVD[ch].max_flow), SVD[ch].total_offset_enable);	//積算オフセット
	 eep_write_ch_delay(ch, (short)(&SVD[ch].total_offset_value - &SVD[ch].max_flow), SVD[ch].total_offset_value);	//積算オフセット値
		eep_write_ch_delay(ch, (short)(&SVD[ch].drive_search - &SVD[ch].max_flow), SVD[ch].drive_search);	//調整モード
		eep_write_ch_delay(ch, (short)(&SVD[ch].start_freq - &SVD[ch].max_flow), SVD[ch].start_freq);	//サーチ開始周波数
		eep_write_ch_delay(ch, (short)(&SVD[ch].stop_freq - &SVD[ch].max_flow), SVD[ch].stop_freq);	//サーチ停止周波数
#if defined(FRQSCH)
	eep_write_ch_delay(ch, (short)(&SVD[ch].SchFrq - &SVD[ch].max_flow), SVD[ch].SchFrq);	//サーチして得た周波数
#endif
	 eep_write_ch_delay(ch, (short)(&SVD[ch].target_total.DT_2BYTE.low  - &SVD[ch].max_flow), SVD[ch].target_total.DT_2BYTE.low);	//積算目標値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].target_total.DT_2BYTE.high - &SVD[ch].max_flow), SVD[ch].target_total.DT_2BYTE.high);	//
	 eep_write_ch_delay(ch, (short)(&SVD[ch].damp_mode - &SVD[ch].max_flow), SVD[ch].damp_mode);				//気泡対策モード
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2d - &SVD[ch].max_flow), SVD[ch].rl2d);						//D Rate Limit ダンピング倍率
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2hc - &SVD[ch].max_flow), SVD[ch].rl2hc);				//D Rate Limit ホールドクリア
	 eep_write_ch_delay(ch, (short)(&SVD[ch].odpd - &SVD[ch].max_flow), SVD[ch].odpd);						//レイトリミット1st
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl1tg - &SVD[ch].max_flow), SVD[ch].rl1tg);				//D Rate Limit 1st 目標
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl1av - &SVD[ch].max_flow), SVD[ch].rl1av);				//D Rate limit 1st 平均
	 eep_write_ch_delay(ch, (short)(&SVD[ch].odpl - &SVD[ch].max_flow), SVD[ch].odpl);						//レイトリミット2nd
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2tg - &SVD[ch].max_flow), SVD[ch].rl2tg);				//D Rate limit 2nd 目標
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rl2av - &SVD[ch].max_flow), SVD[ch].rl2av);				//D Rate limit 2nd 平均
	 eep_write_ch_delay(ch, (short)(&SVD[ch].rlt - &SVD[ch].max_flow), SVD[ch].rlt);							//レイトリミット
	 eep_write_ch_delay(ch, (short)(&SVD[ch].dump_var - &SVD[ch].max_flow), SVD[ch].dump_var);				//可変ダンピング偏差値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].dump_mul - &SVD[ch].max_flow), SVD[ch].dump_mul);				//可変ダンピング倍率
	 eep_write_ch_delay(ch, (short)(&SVD[ch].inc - &SVD[ch].max_flow), SVD[ch].inc);							//異常ホールド時間
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_up.WORD.low - &SVD[ch].max_flow), SVD[ch].corr_up.WORD.low);		//相関値幅上限値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_up.WORD.high - &SVD[ch].max_flow), SVD[ch].corr_up.WORD.high);		//
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_low.WORD.low - &SVD[ch].max_flow), SVD[ch].corr_low.WORD.low);		//相関値幅下限値
	 eep_write_ch_delay(ch, (short)(&SVD[ch].corr_low.WORD.high - &SVD[ch].max_flow), SVD[ch].corr_low.WORD.high);	//

		//評価用
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_option - &SVD[ch].max_flow), SVD[ch].sns_option); //センサオプション
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_disL - &SVD[ch].max_flow), SVD[ch].sns_disL); //センサ間距離(L)
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_disL_l - &SVD[ch].max_flow), SVD[ch].sns_disL_l); //センサ間距離(L-l)
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_tau - &SVD[ch].max_flow), SVD[ch].sns_tau);		//無駄時間
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sns_coef - &SVD[ch].max_flow), SVD[ch].sns_coef);			//互換係数
	 eep_write_ch_delay(ch, (short)(&SVD[ch].adc_clock - &SVD[ch].max_flow), SVD[ch].adc_clock);			//ADCクロック
	 eep_write_ch_delay(ch, (short)(&SVD[ch].wind_offset - &SVD[ch].max_flow), SVD[ch].wind_offset);			//WINDOWオフセット
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sum_start - &SVD[ch].max_flow), SVD[ch].sum_start);			//差分相関開始位置
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sum_end - &SVD[ch].max_flow), SVD[ch].sum_end);			//差分相関終了位置
	 eep_write_ch_delay(ch, (short)(&SVD[ch].sum_step - &SVD[ch].max_flow), SVD[ch].sum_step);			//差分相関間隔

	 eep_write_ch_delay(ch, (short)(&SVD[ch].fix_data - &SVD[ch].max_flow), SVD[ch].fix_data);			//固定値設定
	 eep_write_ch_delay(ch, (short)(&SVD[ch].fix_amp_gain_rev - &SVD[ch].max_flow), SVD[ch].fix_amp_gain_rev);			//Wiper Position(ゲイン値)
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
	 //評価用
 }

	/*** メーカリニアライズ ***/
	if((opt & WR_MAKER_LINR) != 0){	 /*メーカリニアライズ書込み*/
	 eep_write_ch_delay(ch, (short)(&SVD[ch].mklnr_num - &SVD[ch].max_flow), SVD[ch].mklnr_num);			//補正点数
	 pt1 = &SVD[ch].mklnr_out1.WORD.low;
	 pt2 = &SVD[ch].mklnr_in1.WORD.low;
	 for(i_cnt=1; i_cnt<=30; i_cnt++){
		 eep_write_ch_delay(ch, (short)(pt1 - &SVD[ch].max_flow), *pt1);		//補正出力
		 eep_write_ch_delay(ch, (short)(pt2 - &SVD[ch].max_flow), *pt2);  //補正入力
		 pt1++;	pt2++;
	 }
	}

	/*** 変換器シリアルナンバー ***/
	if((opt & WR_CVT_SERIAL) != 0){	 /*変換器シリアルナンバー書込み*/
	 for(ch_cnt = 0; ch_cnt < 6; ch_cnt++){
		 pt = &SVD[ch_cnt].c_serial[0];
		 for(i_cnt=1; i_cnt<=8; i_cnt++){
			 eep_write_ch_delay(ch_cnt, (short)(pt - &SVD[ch_cnt].max_flow), *pt);
			 pt++;
		 }
	 }
	}

	/*** センサシリアルナンバー ***/
	if((opt & WR_SNS_SERIAL) != 0){	 /*センサシリアルナンバー書込み*/
	 pt = &SVD[ch].s_serial[0];
	 for(i_cnt=1; i_cnt<=8; i_cnt++){
		 eep_write_ch_delay(ch, (short)(pt - &SVD[ch].max_flow), *pt);
		 pt++;
	 }
	}

}

/****************************************************
 * Function : util_SnsMem_Write
 * Summary  : センサメモリ書き込み要求
 * Argument : pch, opt
 * Return   : なし
 * Caution  : なし
 * notes    : センサメモリ書き込みフロー
 *            phase_owwrite = 1 : OWwrite = 1 判定
 *                            2 : MAIN[pch].com_act_status = ACT_STS_NORMAL or ACT_STS_ADDIT 判定
 *                            3 : メモリデバイスの対象チャンネルか判定
 *                                & 書き込みデータがあれば専用配列にコピーする
 *                            4 : CRCを計算する
 *                            5 : メモリデバイスに書き込む
 *                            6 : 書き込んだデータの照合
 *                            7 : 処理終了 & 各変数の初期化
 ****************************************************/
void util_SnsMem_Write(short pch, short opt)
{
	if ((opt & WR_DEVICE) != 0)
	{
		OWwrite[pch] = 1; // メモリデバイス書込み要求
	}
}

/****************************************************
 * Function : SavEepZerAdjPrm (Save Eeprom Zero Adjust Parameter)
 * Summary  : ゼロ調整関連パラメータのEEPROM保存
 * Argument : pch : チャンネル番号
 * Return   : None
 * Caution  : None
 * Notes    : 
 ****************************************************/
void SavEepZerAdjPrm(short pch)
{
    short i;

	if(pch >= CH_NUMMAX) return;
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_qat.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_flow_qat.WORD.low);   /*ゼロ点調整時：流量*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_qat.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_flow_qat.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_vel.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_flow_vel.WORD.low);   /*ゼロ点調整時：流速*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_flow_vel.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_flow_vel.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_sound_spd - &SVD[pch].max_flow), SVD[pch].zero_sound_spd);                  	/*ゼロ点調整時：音速*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.low1 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.low1);					  /*ゼロ点調整時：積算値*/	
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.low2 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.low2);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.high1 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.high1);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_addit.WORD.high2 - &SVD[pch].max_flow), SVD[pch].zero_addit.WORD.high2);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_wave_max - &SVD[pch].max_flow), SVD[pch].zero_wave_max);		                  /*ゼロ点調整時：受波波形最大値*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_wave_min - &SVD[pch].max_flow), SVD[pch].zero_wave_min);		                  /*ゼロ点調整時：受波波形最小値*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_delta_ts.WORD.low - &SVD[pch].max_flow), SVD[pch].zero_delta_ts.WORD.low);  /*ゼロ点調整時：伝搬時間差*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_delta_ts.WORD.high - &SVD[pch].max_flow), SVD[pch].zero_delta_ts.WORD.high);
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_correlate - &SVD[pch].max_flow), SVD[pch].zero_correlate);	      /*ゼロ点調整時：相関値幅*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_zero_offset - &SVD[pch].max_flow), SVD[pch].zero_zero_offset);	  /*ゼロ点調整時：ゼロ点オフセット*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_condition - &SVD[pch].max_flow), SVD[pch].zero_condition);       /*ゼロ点調整時：コンディション*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_fifo_pos - &SVD[pch].max_flow), SVD[pch].zero_fifo_pos);  /*ゼロ点調整時：FIFO受波波形検出位置*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_gain_1st - &SVD[pch].max_flow), SVD[pch].zero_gain_1st);  /*ゼロ点調整時：アンプゲイン値(1st)*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_gain_2nd - &SVD[pch].max_flow), SVD[pch].zero_gain_2nd);		/*ゼロ点調整時：アンプゲイン値(2nd)*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_fifo_ch - &SVD[pch].max_flow), SVD[pch].zero_fifo_ch);		  /*ゼロ点調整時：FIFO CH*/
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_p1p2 - &SVD[pch].max_flow), SVD[pch].zero_p1p2);          /*ゼロ点調整時：受波の差(P1-P2)*/
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
/* Summary  : EEPROM書き込み（ゼロ調整値）             		*/
/* Argument : pch                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void util_eep_zerowrite(short pch){

	short i_cnt;

	if(pch >= CH_NUMMAX) return;

	// EEPROM書き込み
	SavEepZerAdjPrm(pch);
	
	SVD[pch].zero_offset = SVD[pch].zero_zero_offset;
	eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)SVD[pch].zero_offset);		/*ゼロ点オフセット*/
}

/****************************************************/
/* Function : read_serial_num                       */
/* Summary  : シリアル番号読込み    				*/
/* Argument : pch                               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 変換器シリアルナンバー、 センサシリアルナンバー     */
/****************************************************/
void	read_serial_num(short pch){

 //変換器シリアルナンバー
	MAIN[pch].cvt_serial[0] = eep_read((short)(&SVD[pch].c_serial[0] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[1] = eep_read((short)(&SVD[pch].c_serial[1] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[2] = eep_read((short)(&SVD[pch].c_serial[2] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[3] = eep_read((short)(&SVD[pch].c_serial[3] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[4] = eep_read((short)(&SVD[pch].c_serial[4] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[5] = eep_read((short)(&SVD[pch].c_serial[5] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[6] = eep_read((short)(&SVD[pch].c_serial[6] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);
	MAIN[pch].cvt_serial[7] = eep_read((short)(&SVD[pch].c_serial[7] - &SVD[pch].max_flow) + SIZEOFMODBUSADDR * pch);

 //センサシリアルナンバー
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
/* Summary  : シリアル番号書込み    				*/
/* Argument : pch                               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 変換器シリアルナンバー、 センサシリアルナンバー     */
/****************************************************/
void	write_serial_num(short pch){

 //変換器シリアルナンバー
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[0] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[0]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[1] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[1]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[2] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[2]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[3] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[3]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[4] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[4]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[5] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[5]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[6] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[6]);
	eep_write_ch(pch, (short)(&SVD[pch].c_serial[7] - &SVD[pch].max_flow), MAIN[pch].cvt_serial[7]);

 //センサシリアルナンバー
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
/* Summary  : エラー優先順位の更新                    		*/
/* Argument : ch,  new_err_code     	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 現在エラーと新エラーの優先順位を比較して、       */
/*          : 新エラーの優先順位が高い場合にエラーステータスを　更新する*/
/****************************************************/
void	err_priority(short ch, short new_err_code){

	short ret;
	
	//通信用
	ret = B_NO;
	if(MAIN[ch].com_err_status == 0){	//エラー未登録時
			ret = B_YES;
	}else{
		//優先順位の数字が小さい方が、優先順位が高い
		if(err_inf[MAIN[ch].com_err_status].err_priority > err_inf[new_err_code].err_priority){
			ret = B_YES;
		}
	}
	if(ret == B_YES){
		MAIN[ch].com_err_status = new_err_code;	//エラーステータス更新	
	}

	//エラーLED用
	if(err_inf[new_err_code].err_led != 0){
		ret = B_NO;
		if(MAIN[ch].led_err_status == 0){	//エラー未登録時
				ret = B_YES;
		}else{
			//優先順位の数字が小さい方が、優先順位が高い
			if(err_inf[MAIN[ch].led_err_status].err_priority > err_inf[new_err_code].err_priority){
				ret = B_YES;
			}
		}
		if(ret == B_YES){
			MAIN[ch].led_err_status = new_err_code;	//エラーステータス更新	
		}
	}
}
	
/****************************************************/
/* Function : remove_errcode                       */
/* Summary  : エラーコードの解除    				*/
/* Argument : ch,  err_code                         */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	remove_errcode(short ch, short err_code){

	if(MAIN[ch].led_err_status == 0){			//エラー未発生時
		return;
	}	
	
	//優先順位の数字が小さい方が、優先順位が高い
	if(err_inf[MAIN[ch].led_err_status].err_priority >= err_inf[err_code].err_priority){
		MAIN[ch].led_err_status = (short)0;		//エラーステータスクリア
	}
}

/****************************************************/
/* Function : err_zero_status                       */
/* Summary  : ゼロ点調整エラー有無確認    				*/
/* Argument : err_status                            */
/* Return   : 0=エラーなし, 1=エラーあり                  */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		err_zero_status(short err_status){

	short	ret;
	unsigned short com_err_status;

	ret = B_NO;
	com_err_status = (unsigned short)err_status;

	if(com_err_status == ERR_ZERO_EMPTY || com_err_status == ERR_ZERO_WAVE ||
		com_err_status == ERR_ZERO_CALC || com_err_status == ERR_ZERO_LEVEL ||
		com_err_status == ERR_ZERO_AGC || com_err_status == ERR_ZERO_UNSTABLE){
		ret = B_YES;			//ゼロ点調整エラー有り
	}

	return (ret);
}

/****************************************************/
/* Function : err_total_status                      */
/* Summary  : 積算監視エラー有無確認    				*/
/* Argument : err_status                           */
/* Return   : 0=エラーなし, 1=エラーあり                  */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		err_total_status(short err_status){

	short	ret;
	unsigned short com_err_status;

	ret = B_NO;
	com_err_status = (unsigned short)err_status;

	if(com_err_status == TTL_CACL_ERR || com_err_status == TTL_OVERFLOW){		//積算値演算異常 or 積算値オーバーフロー
		ret = B_YES;			//エラー有り
	}

	return (ret);
}

/****************************************************/
/* Function : get_total_offset                      */
/* Summary  : 積算値オフセットの取得    				*/
/* Argument : ch,  val_total                       */
/* Return   : 積算値オフセット                          */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
unsigned long long get_total_offset(short ch, unsigned long long val_total){

	unsigned long long val_result;

	val_result = val_total;
	if(SVD[ch].total_offset_enable == 1){	//積算値オフセット有効
		if(((long long)val_total + (long long)SVD[ch].total_offset_value * 10000) < 0){		//積算オフセット加算結果が負数
			val_result = 0;
		}else if(((long long)val_total + (long long)SVD[ch].total_offset_value * 10000) <= 999999999990000){		//積算オフセット加算結果が積算値カウンタ以内
			val_result = val_total + ((long long)SVD[ch].total_offset_value * 10000);	//積算値オフセット値を加算
		}else{
			;
		}
	}

	return val_result;
}

/****************************************************/
/* Function : RoundFunc								*/
/* Summary  : 小数点以下の四捨五入						*/
/* Argument : src									*/
/* Return   : 四捨五入結果							*/
/* Caution  : なし									*/
/* notes    : 小数点1桁目の四捨五入を行う				*/
/****************************************************/
float RoundFunc(float src){

	return ( src >= 0 ) ? src + 0.5 : src - 0.5 ;
}

/****************************************************/
/* Function : debug_mode                      */
/* Summary  : 強制エラー制御（デバッグモード）    				*/
/* Argument : pch                                 */
/* Return   : なし                                  */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	debug_mode(short pch){

	short e;
	
//	if(MES[pch].test_enable == 0){					//デバッグモード確認
//		return;
//	}

//	MES[pch].err_status = MES[pch].test_err_code;	//強制エラーコードセット
//	MES[pch].err_status &=(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_CALC+ERR_JUDGE_REVERSE+ERR_JUDGE_WAVE
//							+ERR_JUDGE_OVERFLOW+ERR_JUDGE_EEPROM+ERR_JUDGE_CUNET+ERR_JUDGE_RESTART	+ERR_JUDGE_ZERO);

	if(MES[pch].test_err_enable == 0){		//強制エラー出力モード確認
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
