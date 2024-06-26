/***********************************************/
/* File Name : defLOG.c		          									   */
/*	Summary   : エラーログ情報初期値                 */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "define.h"
#include "defLOG.h"

/************************************************************/
/*	グローバル変数											*/
/************************************************************/
struct stLOG_DETAIL		LOG_DETAIL[CH_NUMMAX][LOGMAX];		//エラーログ詳細情報
struct stLOG_DETAIL_DEF	LOG_DETAIL_DEF;						//エラーログ詳細情報

/************************************************************/
/*	エラーログ詳細情報の初期値								*/
/************************************************************/
struct stLOG_DETAIL_DEF LOG_DETAIL_DEF={
	0,										//エラーコード
	0,										//エラー発生時の電源オンカウンタ
	0,										//エラー発生時の時間
	0,										//現在時間
	9999999,								//流量
	99999,									//流速
	9999999,								//音速
	999999999,								//積算値
	999999999,								//伝搬時間（アップ）
	999999999,								//伝搬時間（ダウン）
	999999999,								//伝搬時間差
	999999999,								//無駄時間（アップ）
	999999999,								//無駄時間（ダウン）
	0xFFFF,									//ステータス
	9999,									//窓位置
	999,									//ゲイン（アップ）
	999,									//ゲイン（ダウン）
	999,									//最大極値
	999,									//先頭波閾値
	0xFF,									//メール送信エラーステータス
	999,									//差分相関値[42]
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999,									//
	999										//
};
