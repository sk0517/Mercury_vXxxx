/***********************************************/
/* File Name : disp.c		            									   */
/*	Summary   : 表示関連処理			                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>

#include "define.h"
#include "version.h"
#include "defMES.h"
#include "defLED.h"
#include "defMAIN.h"
#include "SV_def.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void	drive_led(unsigned short led_num);
void	disp_led_ch(void);
void	disp_led_alm(void);
void	disp_init(void);
void	disp_zerosw_check(void);
short	disp_zero_read(void);
short	disp_ch_read(void);
short	disp_cunet_read(void);
void	disp_cunet_set(void);
void	disp_led_control(void);
short	disp_led_judge(short pch);
void	disp_led_cpu(short on_off);
void	disp_cpu_status(void);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern	void	mky43_restart(void);
extern	short	mky43_get_flow_num(void);
extern unsigned long invert_data(unsigned long data);
extern void delay(unsigned short delay_count);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short sw_now;			//SW状態保存
short sw_old;			//SW状態保存
short sw_cont;		//SW状態保存
short sw_cunet_num;		//CUnetSW状態保存
short	led_flash = 0;		//全灯検査用
short	led_channel;		//Channel LED表示状態
short	led_alarm;		//Alarm LED表示状態
unsigned char sw_now_zeroadj;	//ゼロ点調整SW状態
unsigned char sw_now_almreset[6];	//ゼロ点調整SW状態
short CH_LED[6] = {DSP_LED1, DSP_LED2, DSP_LED3, DSP_LED4, DSP_LED5, DSP_LED6}; /*CH-LED情報*/

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short	sa_flow;
extern short	main_count;
extern short	disp_ch;
extern unsigned long	cmi_count;
extern short	com_type;

/****************************************************/
/* Function : drive_led                    */
/* Summary  : LED表示更新    				*/
/* Argument : led_num              	               */
/* Return   : なし 				                    */
/* Caution  : なし                                   */
/*	notes    : 								*/
/*		0x0001		//LED CH1				*/
/*		0x0002		//LED CH2				*/
/*		0x0004		//LED CH3				*/
/*		0x0008		//LED CH4				*/
/*		0x0010		//LED CH5				*/
/*		0x0020		//LED CH6				*/
/*		0x0100		//LED ALM1				*/
/*		0x0200		//LED ALM2				*/
/*		0x0400		//LED ALM3				*/
/*		0x0800		//LED ALM4				*/
/****************************************************/
void	drive_led(unsigned short led_num){

	FPGA_LED_CNT = led_num;		//LED制御レジスタに設定
}

/****************************************************/
/* Function : disp_led_ch                    */
/* Summary  : channel LEDを更新する。 				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 10ms毎に呼ばれる                         */
/****************************************************/
void	disp_led_ch(void) {

	drive_led(led_channel);
}

/****************************************************/
/* Function : disp_led_alm                    */
/* Summary  : alarm LEDを更新する。 				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 10ms毎に呼ばれる                         */
/****************************************************/
void	disp_led_alm(void) {

	drive_led(led_alarm);
}

/****************************************************/
/* Function : disp_init                    */
/* Summary  : LED初期化   				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	disp_init(void){

	led_channel = 0;
	led_alarm = 0;
}

/****************************************************/
/* Function : disp_zerosw_check                    */
/* Summary  : スイッチ制御 : ゼロ点調整スイッチ確認      				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	disp_zerosw_check(void){

	short sw_work;
	short ch_read;
	short	i_cnt;
	
	for(i_cnt=CH1; i_cnt<CH_NUMMAX; i_cnt++){
		if(LED[i_cnt].zero_active != 0){		//ゼロ点調整中は処理しない
			return;
		}
	}

	sw_work = disp_zero_read();			//ゼロ点調整SW読込み

	if(sw_work != sw_old){				//SW状態変化あり
		sw_now = sw_work;				//SW状態更新
		sw_cont = 0;
		sw_now_zeroadj = B_OFF;
	}else{													//SW状態変化なし
		if(sw_now == B_ON && sw_now_zeroadj == B_OFF){		//SW押下状態
			sw_cont++;
		}else{
			sw_cont = 0;
			sw_now_zeroadj = B_OFF;
		}
		
		if(sw_cont > SW_ZERO_START){	//SW押下3秒経過
			sw_cont = 0;
			sw_now_zeroadj = B_ON;		//ゼロ点調整要求
			
			ch_read = disp_ch_read();	//CHスイッチ読み込み
			if(ch_read == SW_CH0){
				led_channel = 0;
			}
			else if( (ch_read >= SW_CH1) && (ch_read <= SW_CH6)
				&& (SVD[ch_read -1].sensor_size != SNS_NONE)){
				led_channel = 0;
			}
		}
	}
	sw_old = sw_work;					//SW状態保存
}

/****************************************************/
/* Function : disp_zero_read                    */
/* Summary  : スイッチ制御 : ゼロスイッチ読込み    				*/
/* Argument : なし                  	               */
/* Return   : スイッチ状態					                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		disp_zero_read(void){

	short sw;

	// GPIO_PH4
	if(__bit_input(GPIO_PORTH_BASE, 4) == 0){		//Zero入力
		sw = B_ON;
	}else{
		sw = B_OFF;
	}

	return (sw);
}

/****************************************************/
/* Function : disp_ch_read                    */
/* Summary  : スイッチ制御 : CH番号読込み[SW2] 		*/
/* Argument : なし                  	               */
/* Return   : CH番号			                        */
/* Caution  : なし                                   */
/* notes    : 10ポジション(0～9)                        */
/****************************************************/
short	disp_ch_read(void){
	
	short sw_status;

//	sw_status = (FPGA_SW_DISP & 0x000F);	//SW表示レジスタ読込み
	sw_status = FPGA_SW_DISP;	//SW表示レジスタ読込み
	sw_status = (sw_status ^ 0xFFFFFFFF);	//ビット反転
	sw_status = (sw_status & 0x000F);	//CHスイッチの有効ビット

	return (sw_status);
}

/****************************************************/
/* Function : disp_cunet_read                    */
/* Summary  : スイッチ制御 : CUnetアドレス読込み	[SW3] 	*/
/* Argument : なし                  	               */
/* Return   : CUnetアドレス		                    */
/* Caution  : なし                                   */
/* notes    : 16ポジション(0～15)                       */
/****************************************************/
short	disp_cunet_read(void){
	
	short sw_status;
	
//	sw_status = ((FPGA_SW_DISP & 0x00F0) >> 4);	//SW表示レジスタ読込み
	sw_status = FPGA_SW_DISP;	//SW表示レジスタ読込み
	sw_status = (sw_status ^ 0xFFFFFFFF);	//ビット反転
	sw_status = ((sw_status & 0x00F0) >> 4);	//CUnetアドレススイッチの有効ビット
	
	return (sw_status);
}

/****************************************************/
/* Function : disp_cunet_set                    */
/* Summary  : CUnetアドレス設定	    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	disp_cunet_set(void){

	if(com_type == COM_CUNET){
		short sw_now;

		sw_now = mky43_get_flow_num();
		if(sw_now == sw_cunet_num){	//CUnetSW状態変化なし
			return;
		}
	
   	/*割込み禁止*/
		clrpsw_i();
	
	/*CUnetアドレス設定処理*/
		mky43_restart();		//MKY43の通信再開

 	/*割込み許可*/
		setpsw_i();
	
		sw_cunet_num = sw_now;		//CUnetSW状態保存
	}
}

/****************************************************/
/* Function : disp_led_control                    */
/* Summary  : パネル部LED処理		    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 1秒に1回呼ばれる                         */
/****************************************************/
void	disp_led_control(void){

	short		i_cnt;
	short		ch_num;
	short		ch_read;
	short 	oth_status;

	ch_num = CH1;
	oth_status = 0;

	/* 通常運用 */
	if(led_flash == 0){
		
		if(disp_ch >= CH_NUMMAX){
			return;
		}

		for(i_cnt=CH1; i_cnt<CH_NUMMAX; i_cnt++){
			if(LED[i_cnt].zero_active != 0){	//ゼロ点調整中はLED処理しない
				return;
			}
		}
	
		ch_read = disp_ch_read();			//表示切替CHの取得
		if(ch_read > SW_CH6){
			disp_init();				//全LED消灯
			return;
		}
		if(ch_read>=SW_CH1 && ch_read<=SW_CH6){		//CH1～6設定時（指定CH処理）
			ch_num = ch_read - 1;
			//他CHのエラー情報を、channel LEDに表示
			led_channel = 0;
			for(i_cnt=CH1; i_cnt<CH_NUMMAX; i_cnt++){	//他CHエラー情報確認
				oth_status = disp_led_judge(i_cnt);	//エラー情報読込み	
				if(oth_status != 0){			//エラー情報あり
					led_channel |= CH_LED[i_cnt];
				}
			}
			//自chのエラー情報を出力
			led_alarm = disp_led_judge(ch_num);		//エラー情報読込み
		}else{							//CH0設定時（全CH処理）
			led_channel = CH_LED[disp_ch];
			led_alarm = disp_led_judge(disp_ch);
		}
	}
	/* 全灯検査時 */
	else{
		led_channel = 0x003F;
		led_alarm = 0x0F00;
	}
}

/****************************************************/
/* Function : disp_led_judge                      */
/* Summary  : エラーLED種別判定    				              */
/* Argument : pch                  	               */
/*	Return   :	led	: エラーLED種別						              */
/*						      LED1:エンプティセンサ			                  */
/*						      LED2:波形異常/アンバランス	              	*/
/*						      LED3:ゼロ点調整失敗				                */
/*						      LED4:その他異常					                  */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		disp_led_judge(short pch){

	short		led;
	unsigned short led_err_status;
	
	led = 0;

	if(pch >= CH_NUMMAX){
		return (led);
	}

	led_err_status = MAIN[pch].led_err_status;
	
	if(led_err_status == 0 ||				//エラー発生無し
		led_err_status >= ERR_CODE_MAX){	//指定エラー以外の場合
		return (led);
	}
	
	led = err_inf[led_err_status].err_led;	//点灯LED

	return (led);
}

/****************************************************/
/* Function : disp_led_cpu                    */
/* Summary  : LED制御 : CPU LED制御    				*/
/* Argument : on_off                	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : LED点灯/消灯/点滅	                      */
/****************************************************/
void	disp_led_cpu(short on_off){

	/* 通常運用 */
	if(led_flash == 0){
	
		if(on_off == B_ON){			//LED点灯
			// GPIO_PB7
			__bit_output(GPIO_PORTB_BASE, 7, 1);
		}else if(on_off == B_OFF){	//LED消灯
			// GPIO_PB7
			__bit_output(GPIO_PORTB_BASE, 7, 0);
		}else{							//LED点滅
			// GPIO_PB7
			if(__bit_input(GPIO_PORTB_BASE, 7) != 0){
				// GPIO_PB7
				__bit_output(GPIO_PORTB_BASE, 7, 0);
			}else{
				// GPIO_PB7
				__bit_output(GPIO_PORTB_BASE, 7, 1);
			}
		}
	}
	/* 全灯検査時 */
	else{
		// GPIO_PB7
		__bit_output(GPIO_PORTB_BASE, 7, 1);
	}
}

/****************************************************/
/* Function : disp_cpu_status                    */
/* Summary  : LED制御 : CPU動作状態表示        				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/*	notes    :　CPU正常時：低速点滅(1sec周期)	          	*/
/*	　　　　       CPU異常時：高速点滅(300msec周期)	       */
/****************************************************/
void	disp_cpu_status(void){

	short timer;
	
	if(reset_factor == RESTART_WDOG){	//ウォッチドッグ再起動時
		timer = 3;						//300msec周期(100msec x 3回 = 300msec)
	}else{								//CPU正常動作時
		timer = 10;						//1sec周期(100msec x 10回 = 1sec)
	}
		
	if((main_count % timer) == 0){ 	//点滅更新周期
		disp_led_cpu(B_BLINK);			//CPU LED点滅
	}
}
