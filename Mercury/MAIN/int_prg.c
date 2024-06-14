/***********************************************/
/* File Name : int_prg.c		         									   */
/*	Summary   : 割込み処理					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>
#include "define.h"
#include "SV_def.h"
#include "defMAIN.h"
#include "version.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void	int_prg(void);
void	int_cmt2(void);
void	int_mky_recv0(void);
void	int_mky_recv1(void);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern void int_flow(short pch);
extern void	err_judge_status(short pch);
extern void	err_judge_holdtime(short pch);
extern void	disp_led_control(void);
extern void disp_led_ch(void);
extern void disp_led_alm(void);
extern void	mky43_check_recv0(void);
extern void	mky43_check_recv1(void);
extern void	mky43_write_alarm(short ch);
extern void	mky43_write_flow(short ch);
extern void	flow_save_control(void);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short	disp_ch;
short	timer_main;

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern unsigned long	cmi_count;
extern short ChCtrlNow;
extern short com_type;


/****************************************************/
/* Function : int_prg                               */
/* Summary  : 定周期割込み処理(3msec周期) 			          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う */
/****************************************************/
void	int_prg(void){

	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	/*ウオッチドッグタイマ、リセット*/
	void watchdog_refresh(void);
	watchdog_refresh();

	/*多重割込み許可*/
	setpsw_i();						/*割込み許可*/
	
	int_flow(ChCtrlNow);			/*流量計測*/

//	debug_mode(ChCtrlNow);			/*強制エラー制御（デバッグモード）*/

	err_judge_status(ChCtrlNow);	/*エラー判定*/
	err_judge_holdtime(ChCtrlNow);	/*エラーホールドタイム判定*/

	if(com_type == COM_CUNET){
		mky43_write_alarm(ChCtrlNow);	/*エラーステータス書込み(CUnet)*/
		mky43_write_flow(ChCtrlNow);	/*瞬時流量書込み(CUnet)*/
	}

	//制御CH更新
	if(ChCtrlNow >= CH_IDXMAX)
//	if(ChCtrlNow >= 2) //6ms割込みに変更
	{
		ChCtrlNow = 0;
	}else{
		ChCtrlNow++;
	}
}

/****************************************************/
/* Function : int_cmt2                              */
/* Summary  : タイマ割込み処理(5msec周期)  				          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う */
/****************************************************/
void	int_cmt2(void){

	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	cmi_count ++;						//周期割込みカウンタ更新
	if(cmi_count > CMI_COUNT_MAX){
		cmi_count = 1;
	}

	/*メイン周期タイマー*/
	if(timer_main <= 0){
		timer_main = 0;					//タイマークリア
	}else{
		timer_main--;					//タイマー更新
	}

	setpsw_i();							//割込み許可//

	if((cmi_count % CMI_RATIO_100MSEC) == 0){ 	//100msec周期
		flow_save_control();  /*瞬時流量保存*/
	}
	
	if((cmi_count % CMI_RATIO_1SEC) == 0){ 	//1秒周期
		disp_led_control();
		disp_ch ++;								//表示CH更新
		if(disp_ch >= CH_NUMMAX)
//		if(disp_ch >= 3)    //6ms割込みに変更
		{
			disp_ch = CH1;
		}
	}

	clrpsw_i();      					//割込み禁止//

	if(cmi_count % 2 == 0){
		disp_led_ch();					//パネル部CH-LED更新
	}else{
		disp_led_alm();					//パネル部ALM-LED更新
	}

	setpsw_i();							//割込み許可//
}

/****************************************************/
/* Function : int_mky_recv0                         */
/* Summary  : MKY43割込み処理           				          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う */
/****************************************************/
void	int_mky_recv0(void){

	GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_7);	// GPIO_PP7 

	setpsw_i();					//割込み許可//

	if(com_type == COM_CUNET){
		mky43_check_recv0();	//MKY43割込み確認
	}
}

/****************************************************/
/* Function : int_mky_recv1                         */
/* Summary  : MKY43割込み処理           				          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う */
/****************************************************/
 void	int_mky_recv1(void){

	GPIOIntClear(GPIO_PORTQ_BASE, GPIO_PIN_7);	// GPIO_PQ7 

 	if(com_type == COM_CUNET){
		mky43_check_recv1();	//MKY43割込み確認
	}
}
