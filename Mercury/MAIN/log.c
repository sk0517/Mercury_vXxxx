/***********************************************/
/* File Name : log.c		             									   */
/*	Summary   : エラーログ処理			                   */
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
/*	モジュール内定義関数								*/
/********************************************************/
void	log_init(void);
void	log_detailinfo_init(short ch);
void	log_basicinfo_save(short ch, short code);
void	log_detailinfo_save(short ch, short code);
void	log_save(short ch, short code);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern void	eep_write_ch_delay(short, short, short);
extern short get_attenuator_gain(short pch);
extern void	err_priority(short ch, short new_err_code);
extern short check_queue(void);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern unsigned long	cmi_count;


/****************************************************/
/* Function : log_init                    */
/* Summary  : エラーログの初期化    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	log_init(void){

	short		log_ch;
	short		log_num;
	short		log_count;
	short		i_count;
	unsigned short	pwon_count;
	unsigned long	err_time;
	
	for(log_ch = CH1; log_ch < CH_NUMMAX; log_ch ++){		//エラーログ詳細情報の初期化
		log_detailinfo_init(log_ch);
	}
	
	for(log_ch = CH1; log_ch < CH_NUMMAX; log_ch ++){		//エラーログを登録する配列番号を検索
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
						SAVE[log_ch].log_save_num = log_num;		//エラーログ登録開始番号
					}
				}
			}
		}
	}
	
	for(log_ch = CH1; log_ch < CH_NUMMAX; log_ch ++){		//エラーログ基本情報(EEPROM)をエラーログ詳細情報(RAM)にコピー
		log_count = 0;
		log_num = SAVE[log_ch].log_save_num;
		for(i_count = 0; i_count < LOGMAX; i_count ++){
			if(SVD[log_ch].err_code[log_num] != 0){		//エラーログ基本情報に登録情報あり
				LOG_DETAIL[log_ch][log_count].err_code = SVD[log_ch].err_code[log_num];					//エラーコード
				LOG_DETAIL[log_ch][log_count].err_pwon_count = SVD[log_ch].err_pwon_count[log_num];		//エラー発生時の電源オンカウンタ
				LOG_DETAIL[log_ch][log_count].err_time = SVD[log_ch].err_time[log_num].DWORD;			//エラー発生時時間
				log_count ++;
			}
			if(log_num == (LOGMAX - 1)){					//エラーログ登録番号の更新
				log_num = 0;
			}else{
				log_num ++;
			}
		}
	}
}

/****************************************************/
/* Function : log_detailinfo_init                   */
/* Summary  : エラーログ詳細情報の初期化    				*/
/* Argument : ch                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	log_detailinfo_init(short ch){

	short		log_ch;
	short		log_num;
	
	if(ch >= CH_NUMMAX){		//CH指定チェック
		return;				//初期化不可
	}
	log_ch = ch;

	for(log_num = 0; log_num < LOGMAX; log_num ++){
		memcpy(&LOG_DETAIL[log_ch][log_num], &LOG_DETAIL_DEF, sizeof(LOG_DETAIL[log_ch][log_num]));
	}
}

/****************************************************/
/* Function : log_basicinfo_save                   */
/* Summary  : エラーログ基本情報の登録    				*/
/* Argument : ch,  code            	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	log_basicinfo_save(short ch, short code){

	short		log_ch;
	short		log_num;
	
	if(ch >= CH_NUMMAX){		//CH指定チェック
		return;				//登録不可
	}
	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		return;						//登録不可
	}

	log_ch = ch;
	log_num = SAVE[log_ch].log_save_num;		//エラーログ登録番号

	SVD[log_ch].err_code[log_num] = code;						//エラーコード
	SVD[log_ch].err_pwon_count[log_num] = SVD[ch].pwon_count;	//エラー発生時の電源オンカウンタ
	SVD[log_ch].err_time[log_num].DWORD = cmi_count;			//エラー発生時の時間

	//EEPROM書き込み
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_code[log_num] - &SVD[log_ch].max_flow), SVD[log_ch].err_code[log_num]);
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_pwon_count[log_num] - &SVD[log_ch].max_flow), SVD[log_ch].err_pwon_count[log_num]);
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_time[log_num].WORD.low - &SVD[log_ch].max_flow), SVD[log_ch].err_time[log_num].WORD.low);
	eep_write_ch_delay(log_ch, (short)(&SVD[log_ch].err_time[log_num].WORD.high - &SVD[log_ch].max_flow), SVD[log_ch].err_time[log_num].WORD.high);

	//エラーログ登録番号の更新
	if(log_num == (LOGMAX - 1)){
		SAVE[log_ch].log_save_num = 0;
	}else{
		SAVE[log_ch].log_save_num ++;
	}
}

/****************************************************/
/* Function : log_detailinfo_save                   */
/* Summary  : エラーログ詳細情報の登録    				*/
/* Argument : ch,  code            	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	log_detailinfo_save(short ch, short code){

	short		log_ch;
	short		log_num;
	short		log_shift;
	short		shift_count;
	short		i_cnt;
	
	if(ch >= CH_NUMMAX){		//CH指定チェック
		return;				//登録不可
	}
	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		return;						//登録不可
	}
	log_ch = ch;
	log_shift = 1;

	for(log_num = 0; log_num < LOGMAX; log_num ++){		//エラーログを登録する配列番号を検索
		if(LOG_DETAIL[log_ch][log_num].err_code == 0){
			log_shift = 0;
			break;
		}
	}
	if(log_num == LOGMAX){
		log_num--;
	}	
	if(log_shift != 0){														//エラーログ配列をシフトしてから登録する
		for(shift_count = 0; shift_count < (LOGMAX-1); shift_count ++){		//エラーログ配列をシフトする
			memcpy(&LOG_DETAIL[log_ch][shift_count], &LOG_DETAIL[log_ch][shift_count+1], sizeof(LOG_DETAIL[log_ch][shift_count+1]));	
		}
	}		
		
	LOG_DETAIL[log_ch][log_num].err_code = code;									//エラーコード
	LOG_DETAIL[log_ch][log_num].err_pwon_count = SVD[log_ch].pwon_count;			//エラー発生時の電源オンカウンタ
	LOG_DETAIL[log_ch][log_num].err_time = cmi_count;								//エラー発生時時間
	LOG_DETAIL[log_ch][log_num].flow_quantity = (long)MES[log_ch].ml_min_now;		//流量
	LOG_DETAIL[log_ch][log_num].flow_velocity = (long)MES[log_ch].flow_vel_c / 100;//流速
	LOG_DETAIL[log_ch][log_num].sound_speed = (long)MES[log_ch].sound_vel_f;		//音速
	LOG_DETAIL[log_ch][log_num].total_count.DWORD = (MES[log_ch].addit_buff.DWORD);//積算値
	LOG_DETAIL[log_ch][log_num].wave_max = MES[log_ch].rev_max_data;				//受波波形最大値
	LOG_DETAIL[log_ch][log_num].wave_min = MES[log_ch].rev_min_data;				//受波波形最小値
	LOG_DETAIL[log_ch][log_num].dt = (long)MES[log_ch].delta_ts_zero;				//伝搬時間差
	LOG_DETAIL[log_ch][log_num].correlate = (MES[log_ch].correlate * 1000);		//相関値幅
	LOG_DETAIL[log_ch][log_num].zero_offset = (SVD[log_ch].zero_offset - 4000) * 32;	//ゼロ点オフセット
	LOG_DETAIL[log_ch][log_num].status = MAIN[log_ch].err_condition;				//ステータス
	LOG_DETAIL[log_ch][log_num].fifo_position = MES[log_ch].fifo_no_read;			//FIFO受波波形検出位置
	LOG_DETAIL[log_ch][log_num].gain_up = (short)get_attenuator_gain(log_ch);		//第1ゲイン
	LOG_DETAIL[log_ch][log_num].gain_down = (short)MES[log_ch].amp_gain_for;		//第2ゲイン
	LOG_DETAIL[log_ch][log_num].fifo_ch = MES[log_ch].fifo_ch_read;				//FIFO CH
	LOG_DETAIL[log_ch][log_num].p1_p2 = MES[log_ch].max_point_sub_f;				//受波の差(P1-P2)
	LOG_DETAIL[log_ch][log_num].mail_err_status = cunet_error;						//メール送信エラーステータス

	if(code == ERR_ZERO_EMPTY || code == ERR_MESR_EMPTY_L || code == ERR_MESR_EMPTY_H){
		for(i_cnt = 0; i_cnt < SUB_POINT; i_cnt ++){
			LOG_DETAIL[log_ch][log_num].sum_abs_log[i_cnt] = 0;		//波形データ0
		}
	}else{	
		for(i_cnt = 0; i_cnt < SUB_POINT; i_cnt ++){
			LOG_DETAIL[log_ch][log_num].sum_abs_log[i_cnt] = SAVE[log_ch].sum_abs_com[i_cnt];		//差分相関値
		}
	}	
	cunet_error = 0;
}

/****************************************************/
/* Function : log_save                   */
/* Summary  : エラーログ情報の登録    				*/
/* Argument : ch,  code            	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	log_save(short ch, short code){

	if(ch >= CH_NUMMAX){				//CH指定チェック
		return;						//登録不可
	}

	switch(code){
		/*EEPROMへログ情報を保持するエラー*/
		case	ERR_ZERO_EMPTY:			//ゼロ調整エンプティセンサー		
		case	ERR_ZERO_WAVE:			//ゼロ調整波形減衰 （波形異常）
		case	ERR_ZERO_CALC:			//ゼロ調整演算異常
		case	ERR_ZERO_LEVEL:			//ゼロ調整U/Dレベル差異常
		case	ERR_ZERO_AGC:			//ゼロ調整AGC不能//
		case	ERR_MESR_EMPTY_H:		//測定エンプティセンサーH	
		case	ERR_MESR_WAVE_H:		//測定波形減衰 （波形異常）H
		case	ERR_MESR_CALC_H:		//測定演算異常H
		case	ERR_MESR_LEVEL_H:		//測定U/Dレベル差異常H
		case	ERR_MESR_AGC_H:			//測定AGC不能H
		case	ERR_MESR_REVERSE:		//測定逆流異常
		case	ERR_MESR_OVERFLOW:		//オーバーフロー
		case	ERR_EEPROM:				//EEPROMエラー
		case	ERR_CUNET:				//CUnetエラー
		case	ERR_ZERO_UNSTABLE:		//ゼロ調整計測時間発散
		case	ERR_RESTART:			//再起動
		case	ERR_DEVICE:				//メモリデバイス異常
			log_basicinfo_save(ch, code);		//エラーログ基本情報の登録
			log_detailinfo_save(ch, code);		//エラーログ詳細情報の登録
			break;
		/*EEPROMへログ情報を保持しないエラー*/
		case	ERR_MESR_EMPTY_L:		//測定エンプティセンサーL		
		case	ERR_MESR_WAVE_L:		//測定波形減衰 （波形異常）L
		case	ERR_MESR_CALC_L:		//測定演算異常L
		case	ERR_MESR_LEVEL_L:		//測定U/Dレベル差異常L
		case	ERR_MESR_AGC_L:			//測定AGC不能L
		case	ERR_10POINT:			//10点データ無効
		case	TTL_CACL_ERR:				//積算値演算異常
		case	TTL_OVERFLOW:				//積算値オーバーフロー
			log_detailinfo_save(ch, code);		//エラーログ詳細情報の登録
			break;
		case	ALM_MESR_GAIN:		//測定アンプゲイン急変（警告）	
		case	ALM_MESR_EMPTY:		//測定エンプティセンサ（警告）	
		default:
			break;
	}

	err_priority(ch, code);				//優先順位確認

}
