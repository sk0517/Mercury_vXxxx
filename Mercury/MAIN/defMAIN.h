/***********************************************/
/* File Name : defMAIN.h 	         									   */
/*	Summary   : MAIN定義 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "define.h"

//----------------------------------------------------------------------
//インクルードガード
//----------------------------------------------------------------------
#ifndef DEFMAIN_H
#define DEFMAIN_H

extern struct stMAIN MAIN[CH_NUMMAX];

extern	short set_mode;			//1=set mode
extern	short	reset_factor;		//リセット要因
extern	short	cunet_error;		//CUnetエラー要因
extern	short	send_err_status;	//メール送信エラーステータス

extern	unsigned short	cunet_mcare;		//CUnet MCARE/LCARE情報
extern	unsigned short	cunet_mfr1;	//CUnet MFR情報1
extern	unsigned short	cunet_mfr2;	//CUnet MFR情報2
extern	unsigned short	cunet_mfr3;	//CUnet MFR情報3
extern	unsigned short	cunet_mfr4;	//CUnet MFR情報4

//----------------------------------------------------------------------
//構造体
//----------------------------------------------------------------------
struct stMAIN{
	
	unsigned short err_judge;				//エラー判定情報（センサ異常等）
	unsigned long err_judge_time[8];	//エラー発生時間
	unsigned short com_err_status;		//エラーステータス（通信用）
	unsigned short com_act_status;		//動作ステータス（通信用）
	unsigned short err_condition;			//エラー状態（常時更新用）
	unsigned short err_condition_cu;		//エラー状態（常時更新用）CUnetアドレス出力用
	unsigned short led_err_status;		//エラーステータス（エラーLED用）
	unsigned short alm_judge;					//警告判定情報
	unsigned long alm_judge_time[8];	//警告発生時間
	unsigned short total_judge;				//積算警告判定情報
	unsigned short cvt_serial[8];			//変換器シリアル番号
	unsigned short sns_serial[8];			//センサシリアル番号
	unsigned short err_sub_judge;			//エラー判定情報（MES_SUB[].err_status_subを使用する）
};

/********************************************************/
/*	エラー情報定義										*/
/********************************************************/
typedef struct {
	short		err_code;		//エラーコード
	short		err_priority;	//優先順位
	short		err_led; 		//点灯LED
} ERROR_INFO;

extern ERROR_INFO err_inf[];	

#endif


