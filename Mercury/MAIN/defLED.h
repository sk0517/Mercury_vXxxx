/***********************************************/
/* File Name : defLED.h 		         									   */
/*	Summary   : LED定義  					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//インクルードガード
//----------------------------------------------------------------------
#ifndef DEFLED_H
#define DEFLED_H

#include "define.h"

extern struct stLED LED[CH_NUMMAX];	//LED表示

//----------------------------------------------------------------------
//構造体
//----------------------------------------------------------------------
struct stLED{	//LED表示
	
	short zero_do_cnt;				/*ゼロ調整実行カウンタ*/
	short vth_do_cnt;					/*Vth調整実行カウンタ*/
	short wave_do_cnt;				/*波形認識調整実行カウンタ*/

	short zero_cal_cnt;				/*ゼロ調整用⊿Tsカウンタ*/
	unsigned long zero_delta_ts;	/*ゼロ調整用⊿Ts加算値*/

	short zero_retry_cnt;				/*ゼロ調整リトライカウンタ*/
	short zero_active;				/*ゼロ調整実行中*/

	short zero_dt_buf[3];				/*ゼロ調整用⊿Ts取得値*/
	short zero_vth_buf[3];			/*ゼロ調時vth取得値*/

	float zero_dt_buf_f[3]; //ΔTs取得値 float用
};

#endif


