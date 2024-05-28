/***********************************************/
/* File Name : MAIN.c	   	         									   */
/*	Summary   : メイン処理 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "define.h"
#include "version.h"
#include "SV_def.h"
#include "defSAVE.h"
#include "defMES.h"
#include "defLED.h"
#include "defMAIN.h"
#include "ctlioport.h"
#if defined(FPGADOWNLOAD)
#include "fpga_config.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_flash.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/epi.h"
#include "driverlib/pwm.h"
#include "driverlib/watchdog.h"
#include "driverlib/udma.h"
#include "../pinout.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void init(void);
void	cpu_init(void);
void	cpu_init_rs485(void);
void	eeprom_init(void);
void	version_set(void);
#if defined(FPGADOWNLOAD)
short   fpga_config_send(void);
#endif
void	viscos_tbl_init(void);
void	recv_wave_init(void);
void	counter_control(void);
void	zero_alm_check(void);
short	zero_check_sub(short d[]);
void	zero_adj_control(short pch);
void	zero_adj_error(short pch);
void	alm_reset_control(short pch);
void	zero_adj_status(short pch);
void	com_req_check(void);
void	com_req_control(short pch);
void	ram_clear_check(void);
void	ram_clear_debug(void);
void	ReadSensDevice(void);
void	CheckDeviceDetect(void);
void	watchdog_refresh(void);
void CheckBOOTCFG(void);
void main(void);
void WatchdogIntHandler(void);
void SetZerAdjPrm(short pch, short Mod);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern short	eep_read(short);
extern void	eep_sv_read(short);
extern void	eep_sv_write(short);
extern void	com_init();
extern void	make_viscos_tbl(short pch);
extern short	gain_adj_init(short pch);
extern void	eep_write_ch(short,short,short);
extern short	ch_search(short, short);
extern void	log_init(void);
extern void	util_delay(short target_time);
extern void	pwon_count(void);
extern void	action_status_control(short pch, short act);
extern void err_judge_status(short pch);
extern void	disp_zerosw_check(void);
extern short	disp_ch_read(void);
extern short	disp_zero_read(void);
extern void	disp_init(void);
extern void	mky43_init(void);
extern short	mky43_ccr_check(void);
extern void	mky43_host_link_check(void);
extern void delay(unsigned short delay_count);
extern void	reset_factor_check(void);
extern void	disp_cpu_status(void);
extern void	reset_control(void);
extern void	drive_led(unsigned short led_num);
extern short get_attenuator_gain(short pch);
extern void eep_write_ch_delay(short ch, short addr, short data);
extern void eep_write_pending_data(void);
extern short check_queue(void);
extern void	SetFrequency(short ch);
extern void	util_eep_allwrite(short pch, short opt);
extern short	OWReadSensinfo(short ch);
extern void	OWWriteSensinfo(void);
extern void	read_serial_num(short pch);
extern void	write_serial_num(short pch);
extern void	InitFPGA(void);
extern void WaitCDONE(void);
extern short SearchWindow(short pch);
extern void	SetDigitalFilterRegister(void);
extern void OWCheckDeviceSide(void);
extern float GetTimDif(short pch, short Mod);
extern void SavEepZerAdjPrm(short pch);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short ChCtrlNow = CH1;	//現在CH(流量制御)
short main_count;
short com_type;
short initializing = 0;
uint32_t g_ui32SysClock;
static uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));	// uDMAControlBaseSet で使用 
short FpgaVersion;

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short timer_main;					/*メイン周期タイマー*/
extern unsigned char sw_now_zeroadj;	/*ゼロ点調整SW状態*/
extern unsigned char sw_now_almreset[6];/*アラームリセットSW状態*/
extern short	led_channel;			//Channel LED表示状態
extern short	led_alarm;				//Alarm LED表示状態
extern short 	CH_LED[6];				//CH-LED点灯情報

extern unsigned short AD_BASE;
extern unsigned short AD_MAX;


#ifdef __cplusplus
extern "C" {
void abort(void);
#endif
void main(void);
#ifdef __cplusplus
}
#endif

// #define WAVE_RECOGNITION //ゼロ点調整時に波形認識を実行する

/****************************************************/
/* Function : fpga_config_send_SSI                  */
/* Summary  : SPI Passive Mode - fpga_config_send   */
/* Argument : なし                                  */
/* Return   : 1=正常, 0=異常(タイムアウト)          */
/* Caution  : なし                                  */
/* notes    : 起動時                                */
/****************************************************/
#if defined(FPGADOWNLOAD)
short fpga_config_send(void)
{
	const unsigned long GPIO_PORTJ_CCK_DATA = GPIO_PORTJ_BASE + 0xC0;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) */
	const unsigned long GPIO_PORTJ_CCK      = GPIO_PORTJ_BASE + 0x40;    /* GPIO_PJ4 (CCK)     */
	const unsigned long GPIO_PORTJ_CDONE    = GPIO_PORTJ_BASE + 0x20;    /* GPIO_PJ3 (CDONE)   */
	const unsigned long GPIO_PORTJ_CRESET   = GPIO_PORTJ_BASE + 0x200;   /* GPIO_PJ7 (CRESETn) */
	
	const unsigned long TIMEOUT_CNT       = 1000000;                   /* 1000000us/2us*2sec = 1,000,000ループ */
	const unsigned int  USER_CCK_CNT      = 200;                       /* 200ループ */
	const unsigned char* conf_data        = ConfigDat;
	
	unsigned long timeout_cnt    = 0;
	unsigned long data_byte_size = 0;
	unsigned int  user_cycle     = 0;
	unsigned int  conf_try       = 0;
	unsigned int  ret            = 1;
	
	
	/* 3回連続でタイムアウトした場合は異常とする */
	for( conf_try=0; conf_try<3; conf_try++ )
	{
		timeout_cnt = 0;            /* タイムアウト監視 */
		conf_data   = ConfigDat;    /* コンフィグデータ */
		ret         = 1;            /* 正常 */
		
		HWREG(GPIO_PORTJ_CRESET) = 0;       /* GPIO_PJ7 (CRESETn) */
		delay(4);                           /* 400ns wait */
		HWREG(GPIO_PORTJ_CRESET) = 0x80;    /* GPIO_PJ7 (CRESETn) */		
		delay(12);    /* 1.2us wait */

		/* 初回のみCDONE == 0 になるまで待つ */
		if( conf_try == 0 )
		{
			while( HWREG(GPIO_PORTJ_CDONE) != 0 );
		}
		
		/* byte */
		for( data_byte_size=0; data_byte_size<sizeof(ConfigDat); data_byte_size++ )
		{
			/* コンフィグデータ送信 */
			/* 処理時間短縮のため、直接HWREGを使用しCCK lowとCDI0 setを同時にwrite */
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x80)>>2;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit7 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x40)>>1;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit6 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x20);       /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit5 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x10)<<1;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit4 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x08)<<2;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit3 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x04)<<3;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit2 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x02)<<4;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit1 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			HWREG(GPIO_PORTJ_CCK_DATA) = (*conf_data & 0x01)<<5;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) bit0 */
			HWREG(GPIO_PORTJ_CCK)      = 0x10;                      /* GPIO_PJ4 (CCK) */
			
			conf_data++;
		}
		
		/* クロックをlowに戻す */
		HWREG(GPIO_PORTJ_CCK) = 0;    /* GPIO_PJ4 (CCK) */
		
		
		/* CDONE == 1 になるまでクロック供給(タイムアウト監視:2sec) */
		/* 8クロック2.00us : 1sec -> 1000msec -> 1000000us/2us*2sec = 1,000,000ループ */
		do
		{
			/* 処理時間短縮のため、直接HWREGを使用し上記のように処理を1ループ-8回行う */
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			HWREG(GPIO_PORTJ_CCK) = 0x10;
			HWREG(GPIO_PORTJ_CCK) = 0;
			
			timeout_cnt++;
			
		}while( (HWREG(GPIO_PORTJ_CDONE) == 0) && (timeout_cnt < TIMEOUT_CNT) );
		
		
		if( HWREG(GPIO_PORTJ_CDONE) != 0 )
		{
			/* ユーザモード遷移用クロック供給 */
			for( user_cycle=0; user_cycle<USER_CCK_CNT; user_cycle++ )
			{
				HWREG(GPIO_PORTJ_CCK) = 0x10;
				HWREG(GPIO_PORTJ_CCK) = 0;
			}
			
			/* 不本意なCCKが出ないよう極性をINPUTに変更する */
			MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_4);
			
			ret = 1;    /* 正常 */
			break;
		}
		else
		{
			ret = 0;    /* タイムアウト */
			continue;
		}
	
	}
	
	return ret;
}
#endif

/****************************************************/
/* Function : init                           */
/* Summary  : 初期化処理    				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 起動時                                 */
/****************************************************/
void init(void){

 short ch;
#if defined(FPGADOWNLOAD)
 short conf_ret;    /* 1=正常, 0=異常(タイムアウト) */
#endif

 /*割込み禁止*/
	clrpsw_i();

	initializing = 1;		/*初期化中フラグ*/

	/*リセット発生要因の確認*/
	reset_factor_check();

	/*CPU初期化*/
	cpu_init();

	/*EEPROM初期化*/
	eeprom_init();
	
#if defined(FPGADOWNLOAD)
	/* FPGA Config Data Send(SPI Passive Mode) */
	conf_ret = fpga_config_send();
	//送信失敗時はエラービットを立てる必要あり?
	//or LEDの点灯パターンで通知?
	// if(conf_ret != 1){
	// 		__bit_output(GPIO_PORTB_BASE, 7, 1); //CPULED点灯
	// }
#endif

	/*CDONE信号の待機(FPGA起動待機)*/
	WaitCDONE();

	/*RAMクリアの確認*/
	ram_clear_check();
	
	/*メモリデバイス実装位置の判別*/
	OWCheckDeviceSide();

	/*メモリデバイスからセンサ情報を読込む*/
	ReadSensDevice();

	/*デジタルフィルタ係数レジスタの設定*/
	SetDigitalFilterRegister();
	
	/*起動回数の更新*/
	pwon_count();

	/*バージョンセット*/
	version_set();

	/*エラーログの初期化*/
	log_init();

	/*動粘度テーブル初期化*/
	viscos_tbl_init();
	
	/*受波波形データ初期化*/
	recv_wave_init();

	/*RS485 or CUnet判別確認(com_type決定)*/
	mky43_ccr_check();

	/*MKY43の初期化(CUnet通信開始)*/
	if(com_type == COM_CUNET){
		mky43_init();
	}
	else{
		cpu_init_rs485();
	}
	
	/*LED初期化(全LED消灯)*/
	disp_init();

	/*通信初期化*/
	com_init();

	/*メモリデバイス検知の確認*/
	CheckDeviceDetect();

	/*ウォッチドッグタイマ開始*/
	WatchdogStallEnable(WATCHDOG0_BASE);
	watchdog_refresh();
	WatchdogResetEnable(WATCHDOG0_BASE);

	// デバッガ動作中はウォッチドッグでリセットではなく割り込みが発生するようにする 
	// (デバッグしやすくするため) 
	// 0xe000edf0 は Debug Halting Control and Status Register 
	if((*(unsigned long*)0xe000edf0 & 1) != 0)
	{ // C_DEBUGEN ON 
		WatchdogResetDisable(WATCHDOG0_BASE);
		WatchdogIntEnable(WATCHDOG0_BASE);
		IntEnable(INT_WATCHDOG);
	}

	/*割込み要求許可*/
	if(com_type == COM_CUNET){
		IntEnable(INT_GPIOP7);
		IntEnable(INT_GPIOQ7);
	}
	
	/*定周期割込み*/
	TimerEnable(TIMER0_BASE, TIMER_A);		//定周期割込みスタート(3msec)
	TimerEnable(TIMER2_BASE, TIMER_A);		//定周期割込みスタート(5msec)

	initializing = 0;		/*初期化中フラグ解除*/

	/*割込み許可*/
	setpsw_i();
	
}

/****************************************************/
/* Function : cpu_init                        */
/* Summary  : CPU初期化    				*/
/* Argument : なし                           */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 各レジスタ設定                            */
/****************************************************/
void	cpu_init(void){

	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_12MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);

	// Enable processor interrupts.
	IntMasterEnable();

	PinoutSet();	/*ポート設定初期化*/

	__bit_output(GPIO_PORTA_BASE, 0, 1);	// GPIO_PA0 (CH6-REV)
	__bit_output(GPIO_PORTA_BASE, 1, 1);	// GPIO_PA1 (CH6-FWD)
	__bit_output(GPIO_PORTA_BASE, 2, 1);	// GPIO_PA2 (DISPLAY CLK)
	__bit_output(GPIO_PORTA_BASE, 3, 1);	// GPIO_PA3 (DISPLAY nCLR)
	__bit_output(GPIO_PORTA_BASE, 4, 1);	// GPIO_PA4 (DIPLAY DATA)
	__bit_output(GPIO_PORTA_BASE, 5, 1);	// GPIO_PA5 (DISP nSTB)
	
	__bit_output(GPIO_PORTB_BASE, 4, 0);	// GPIO_PB4 (EEPROM CS)
	__bit_output(GPIO_PORTB_BASE, 5, 0);	// GPIO_PB5 (EEPROM CLK)
	// TM4C 版では LED2 は実装されない。 PinoutSet で Input にしている。 
	//__bit_output(GPIO_PORTB_BASE, 6, 0);	// GPIO_PB6 (LED2)
	__bit_output(GPIO_PORTB_BASE, 7, 1);	// GPIO_PB7 (LED1)

	__bit_output(GPIO_PORTD_BASE, 1, 1);	// GPIO_PD1 (TLV5623CDGK SDI)
	__bit_output(GPIO_PORTD_BASE, 2, 1);	// GPIO_PD2 (TLV5623CDGK CS)
	__bit_output(GPIO_PORTD_BASE, 3, 1);	// GPIO_PD3 (TLV5623CDGK CLK)

 __bit_output(GPIO_PORTE_BASE, 0, 1);	// GPIO_PE0 (CH1-REV)
	__bit_output(GPIO_PORTE_BASE, 1, 1);	// GPIO_PE1 (CH1-REV)
	__bit_output(GPIO_PORTE_BASE, 4, 0);	// GPIO_PE4 (EEPROM DI)
	__bit_output(GPIO_PORTE_BASE, 6, 1);	// GPIO_PE6 (PULSE 3)
	__bit_output(GPIO_PORTE_BASE, 7, 1);	// GPIO_PE7 (PULSE 5)
	
	__bit_output(GPIO_PORTF_BASE, 1, 1);	// GPIO_PF1 (DISP R_SHnLD)
	__bit_output(GPIO_PORTF_BASE, 2, 0);	// GPIO_PF2 (DISP R_CLK)
	__bit_output(GPIO_PORTF_BASE, 3, 1);	// GPIO_PF3 (DISP R_CLK_INH)

	__bit_output(GPIO_PORTH_BASE, 5, 1);	// GPIO_PH5 (CH5-REV)
	__bit_output(GPIO_PORTH_BASE, 6, 1);	// GPIO_PH6 (CH5-FWD)

	__bit_output(GPIO_PORTJ_BASE, 0, 0);	// GPIO_PJ0 (CPLD START)
#if defined(FPGADOWNLOAD)
	__bit_output(GPIO_PORTJ_BASE, 4, 0);    // GPIO_PJ4 (CCK)
	__bit_output(GPIO_PORTJ_BASE, 5, 0);    // GPIO_PJ5 (CDI0)
	__bit_output(GPIO_PORTJ_BASE, 7, 1);    // GPIO_PJ7 (CRESETn)
#endif

	__bit_output(GPIO_PORTK_BASE, 0, 1);	// GPIO_PK0 (CH2-REV)
	__bit_output(GPIO_PORTK_BASE, 1, 1);	// GPIO_PK1 (CH2-FWD)
	__bit_output(GPIO_PORTK_BASE, 2, 1);	// GPIO_PK2 (CH3-REV)
	__bit_output(GPIO_PORTK_BASE, 3, 1);	// GPIO_PK3 (CH3-FWD)
	__bit_output(GPIO_PORTK_BASE, 4, 1);	// GPIO_PK4 (MKY43 RST)

	__bit_output(GPIO_PORTN_BASE, 0, 0);	// GPIO_PN0 (CPLD RESTART)	

	__bit_output(GPIO_PORTP_BASE, 4, 0);	// GPIO_PP4 (RS485(H) DE)
	
	__bit_output(GPIO_PORTR_BASE, 4, 1);	// GPIO_PR4 (CH4-REV)
	__bit_output(GPIO_PORTR_BASE, 5, 1);	// GPIO_PR5 (CH4-FWD)

	__bit_output(GPIO_PORTG_BASE, 2, 1);	// GPIO_PG2 (SENS_TX)
	__bit_output(GPIO_PORTG_BASE, 4, 0);	// GPIO_PG4 (SENS_SIG_SW)
	__bit_output(GPIO_PORTG_BASE, 5, 0);	// GPIO_PG5 (SENS_POW_SW)
	__bit_output(GPIO_PORTG_BASE, 7, 0);	// GPIO_PG7 (SENS_EN)

	__bit_output(GPIO_PORTM_BASE, 7, 0); //XRESET_FPGA	(FPGA初期化)

	// [TM4C] ALM_OUT0、ALM_OUT1は、予備のピンで、入力ピンとして設定 (QA17) 

	// 外部バス設定 
	// 0x60000000(FIFOﾃﾞｰﾀ) アクセスで PN3 がアサートされる CS0
	// 0x80000000(FPGAﾚｼﾞｽﾀ)アクセスで PB2 がアサートされる CS1
	// 0xA0000000(CUnetﾒﾓﾘ) アクセスで PN4 がアサートされる 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);
	EPIModeSet(EPI0_BASE, EPI_MODE_HB16);
 EPIAddressMapSet(EPI0_BASE, EPI_ADDR_PER_SIZE_16MB | EPI_ADDR_RAM_SIZE_16MB | EPI_ADDR_QUAD_MODE);

	EPIConfigHB16Set(EPI0_BASE, EPI_HB16_MODE_ADDEMUX | EPI_HB16_CSBAUD | EPI_HB16_CSCFG_QUAD_CS, 1); 
	EPIDividerSet(EPI0_BASE, 0);		// CS0用のEPI_CLK分周器設定 1/1分周 (CLK = 16ns) (デフォルト)
	EPIDividerCSSet(EPI0_BASE, 0, 0);		// CS0用のEPI_CLK分周器設定 1/1分周 (CLK = 16ns) (デフォルト)
	EPIConfigHB16CSSet(EPI0_BASE, 0, EPI_HB16_MODE_ADDEMUX | EPI_HB16_WRWAIT_2 | EPI_HB16_RDWAIT_2);  // CS0用 WR,RDウェイト2 (16ns+(16ns*2wait)=48ns)
	EPIDividerCSSet(EPI0_BASE, 1, 0);		// CS1用のEPI_CLK分周器設定 1/1分周 (CLK = 16ns) (デフォルト)
	EPIConfigHB16CSSet(EPI0_BASE, 1, EPI_HB16_MODE_ADDEMUX | EPI_HB16_WRWAIT_2 | EPI_HB16_RDWAIT_3);  // CS1用 WR,RDウェイト2 (16ns+(16ns*2wait)=48ns)
	EPIDividerCSSet(EPI0_BASE, 2, 1);		// CS2用のEPI_CLK分周器設定 1/2分周 (CLK = 32ns)
	EPIConfigHB16CSSet(EPI0_BASE, 2, EPI_HB16_MODE_ADDEMUX | EPI_HB16_WRWAIT_2 | EPI_HB16_RDWAIT_2);  // CS2用 WR,RDウェイト2 (32ns+(32ns*2wait)=98ns)
	
	// 定周期割込み(3msec)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / 1000) * 3);
//	TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / 1000) * 6); //6ms割込みに変更
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
	IntEnable(INT_TIMER4A);
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	//受信データ受信完了割込み RS485(メンテナンス)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// 定周期割込み(5msec)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER2_BASE, TIMER_A, (g_ui32SysClock / 1000) * 5);
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	//受信データ受信完了割込み RS485/CUnet(ホスト)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
	IntEnable(INT_TIMER3A);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	
	// 割込み優先度の変更（数字が小さいほど優先度が高い）(下位 5 ビットは reserved で無視される) 
	IntPrioritySet(INT_GPIOP7, 4 << 5);		// CUnet割り込み（メール受信(MRB0)）
	IntPrioritySet(INT_GPIOQ7, 1 << 5);		// CUnet割り込み（メール受信(MRB1)）
	IntPrioritySet(INT_GPIOP6, 2 << 5);					// FIFOデータ受信割り込み
	IntPrioritySet(INT_UART_COM_MENT, 1 << 5);		// 受信割り込み（メンテナンスポート）
	IntPrioritySet(INT_TIMER0A, 3 << 5);		// 定周期割り込み(測定処理用)
	IntPrioritySet(INT_TIMER1A, 6 << 5);		// 受信完了割り込み（メンテナンスポート）
	IntPrioritySet(INT_TIMER2A, 2 << 5);		// 定周期割り込み(表示部制御用)
	IntPrioritySet(INT_TIMER3A, 6 << 5);		// 受信完了割り込み（ホストポート）
	IntPrioritySet(INT_TIMER4A, 6 << 5);		// 受信完了割り込み（サブホストポート）

//	GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
//	GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_6);	// GPIO_PP6 

	// MKY43 #INT0 
	// 負論理の信号なので立ち下がりエッジで割り込み 
	GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_7);	// GPIO_PP7 

	// MKY43 #INT1 
	// 負論理の信号なので立ち下がりエッジで割り込み 
	GPIOIntTypeSet(GPIO_PORTQ_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTQ_BASE, GPIO_PIN_7);	// GPIO_PQ7 

	// DMA 共通部分初期化 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	uDMAEnable();
	uDMAControlBaseSet(pui8ControlTable);

	// Put the attributes in a known state for the uDMA software channel.
	// These should already be disabled by default.
	uDMAChannelAttributeDisable(UDMA_CHANNEL_SW, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK));
	// Configure the control parameters for the SW channel.
	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_8);

}

/****************************************************/
/* Function : cpu_init_rs485                         */
/* Summary  : CPU初期化(RS485)   				*/
/* Argument : なし                            */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 各レジスタ設定                            */
/****************************************************/
void	cpu_init_rs485(void){

	// GPIO_PK4
	__bit_output(GPIO_PORTK_BASE, 4, 0);				//MKY43 RST

	// MKY43 用 CS を High 固定出力 
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);	// GPIO_PN4 
	__bit_output(GPIO_PORTN_BASE, 4, 1);

	// 割込み優先度の変更（数字が小さいほど優先度が高い）(下位 5 ビットは reserved で無視される) 
	IntPrioritySet(INT_UART_COM_HOST, 0 << 5);		// 受信割り込み（ホストポート）

}

/****************************************************/
/* Function : eeprom_init                           */
/* Summary  : EEPROM初期化    				*/
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	eeprom_init(void){

	short data;
	short ch = 0;
	char work_ch[20];

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		data =eep_read((short)WRIT_CHECK + SIZEOFMODBUSADDR * ch);
		read_serial_num(ch);					/*シリアル番号読込み*/

		if(data != (short)EEPROM_WRITE_FLAG){	/*書込みフラグのチェック*/	
			drive_led(DSP_CH_ALL);	//CH-LED全灯
			eep_sv_write(ch);					/*初期値書込み*/
			drive_led(DSP_ALM_ALL);	//ALM-LED全灯
			eep_write_ch(ch, (short)WRIT_CHECK, (short)EEPROM_WRITE_FLAG);	/*書込みフラグ更新*/
			write_serial_num(ch);				/*シリアル番号書込み*/
		}
		eep_sv_read(ch);	/*EEPROMの読込み*/
		
		memcpy(&work_ch[0], &SVD[ch].ZerCrsOffset[0], sizeof(SVD[ch].ZerCrsOffset));
		MES[ch].zc_zero_offset = atof(&work_ch[0]);	//ゼロクロスゼロ点オフセット
		MES[ch].signal_count = MES[ch].ThresholdPeakPos = SVD[ch].ThresholdPeakPos;
		MES[ch].zc_peak = SVD[ch].ZerPeakPos;
	}
}

/****************************************************/
/* Function : version_set                           */
/* Summary  : バージョンセット    				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	version_set(void){

	short ch;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		/*ソフトウェアバージョン*/
		SVD[ch].soft_ver = SOFT_VERSION;

		/*ハードウェアバージョン*/
		SVD[ch].hard_ver = HARD_VERSION;
	}

	/*FPGAバージョン*/
	FpgaVersion = FPGA_VERSION;
}

/****************************************************/
/* Function : viscos_tbl_init                       */
/* Summary  : 動粘度テーブル初期化    				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	viscos_tbl_init(void){

	short ch = 0;

	/*自動リニアライズテーブル作成*/
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		make_viscos_tbl(ch);
	}
}

/****************************************************/
/* Function : recv_wave_init                       */
/* Summary  : 受波格納領域の初期化    				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	recv_wave_init(void){

	short ch = 0;
	short i_cnt;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
  MES[ch].clk_phase = 0;
  
  	//評価用
	if(SVD[ch].sum_step == 2){	//打込み回数上流下流各2回
		MES_SUB[ch].sample_cnt = 2;
	}else if(SVD[ch].sum_step == 3){	//打込み回数上流下流各3回
		MES_SUB[ch].sample_cnt = 3;
	}else if(SVD[ch].sum_step == 4){	//打込み回数上流下流各4回
		MES_SUB[ch].sample_cnt = 4;
	}else{	//打込み回数上流下流各4回
		MES_SUB[ch].sample_cnt = 4;
	}
	AD_BASE = AD_BASE_UNIT * MES_SUB[ch].sample_cnt;
	AD_MAX = AD_MAX_UNIT * MES_SUB[ch].sample_cnt;
	//評価用

  
  
 	/*受波格納領域の初期化*/
		for(i_cnt=0; i_cnt<300; i_cnt++)
		{
			MES[ch].fow_data[i_cnt] = MES[ch].rev_data[i_cnt] = AD_BASE;/*0V*/
		}
	
		/*アンプゲインの調整、および、信号異常チェック*/
		gain_adj_init(ch);
			
		/*水中の音速（水）*/
		MES[ch].sound_vel = SVD[ch].sound_vel_fix;

		/*delta_ts0*/
		MES[ch].delta_ts0 = MES[ch].delta_ts = (unsigned short)SVD[ch].zero_offset;
	}
}
	
/****************************************************/
/* Function : counter_control                       */
/* Summary  : カウンタ処理    				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	counter_control(void){

	short ch = 0;

	/*ゼロ調整中カウンタ*/
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		if(LED[ch].zero_do_cnt != 0){
			LED[ch].zero_do_cnt--;
		}
		if(LED[ch].vth_do_cnt != 0){
			LED[ch].vth_do_cnt--;
		}
		if(LED[ch].wave_do_cnt != 0){
			LED[ch].wave_do_cnt--;
		}
	}
	
	/*100msec周期メインカウンタ*/
	if(main_count > 600){			//1分経過
		main_count = 0;				//クリア
	}else{
		main_count++;				//更新
	}
}

/****************************************************/
/* Function : zero_alm_check                       */
/* Summary  : ゼロ点調整/アラームリセット確認          				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 100msに1度実行される
 ****************************************************/
void	zero_alm_check(void){

	short ch;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		zero_adj_control(ch);			//全CHをゼロ点調整
		alm_reset_control(ch);			//全CHをアラームリセット
	}
	sw_now_zeroadj = B_OFF;				//ゼロ点調整要求解除
}

/****************************************************/
/* Function : zero_check_sub                       */
/* Summary  : ゼロ点調整値候補３点のばらつきが、規定値以下であることを確認する	*/
/* Argument : d[]                                  */
/* Return   : 1=正常, 0=異常                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
//#define DIFF_LIMIT	(1)
//#define DIFF_LIMIT	(5)
//#define DIFF_LIMIT	(15)
#define DIFF_LIMIT	(60)  /*4回加算分(15*4)*/
short	zero_check_sub(short d[]) {

	if( (abs(d[0] - d[1]) > DIFF_LIMIT) ||
		(abs(d[1] - d[2]) > DIFF_LIMIT) ||
		(abs(d[2] - d[0]) > DIFF_LIMIT) ) {
		return 0;
	}
	return 1;
}
// #define FLOAT_DIFF_LIMIT (0.0001) //時間差は10^-6～10^-4のオーダー
#define FLOAT_DIFF_LIMIT (0.015) //3σ~0.15(@1/4"), 0.002(@3/8")
short	zero_check_sub_f(float d[]) {
	//absの戻り値はint
	if(
		(d[0] - d[1] < -FLOAT_DIFF_LIMIT) || (FLOAT_DIFF_LIMIT < d[0] - d[1])
		|| (d[1] - d[2] < -FLOAT_DIFF_LIMIT) || (FLOAT_DIFF_LIMIT < d[1] - d[2])
		|| (d[2] - d[0] < -FLOAT_DIFF_LIMIT) || (FLOAT_DIFF_LIMIT < d[2] - d[0])
		)
		{
			return 0;
		}
	return 1;
}

/****************************************************************************
 * Function : JdgZadErr (Judge Zero adjust Error)
 * Summary  : ゼロ点調整中のエラーを判定する
 * Argument : pch : チャンネル番号
 *            Phs : ゼロ点調整のフェーズ
 * Return   : void
 * Caution  : なし
 * Note     : Vth収束収束待ちとゼロ点調整完了処理時のエラーのみerr_statusを更新する
 *          : -> 理由は不明
 ***************************************************************************/
void JdgZadErr(short pch, short Phs)
{
	/*ゼロ調整失敗*/
	zero_adj_error(pch);		/*ゼロ調整エラー処理*/
	if(Phs == 2 || Phs == 4)
	{
		MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*ゼロ調整計測時間発散エラーセット*/
		MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
		err_judge_status(pch);
	}
	zero_adj_status(pch);		/*ゼロ点調整時の波形データを保持する*/
	
}

/****************************************************************************
 * Function : LedCtlZaj (LED Control for Zeroadjust)
 * Summary  : ゼロ点調整時のLED処理
 * Argument : pch : チャンネル番号
 * Return   : void
 * Caution  : なし
 * Note     : 100msに一度実行される
 ***************************************************************************/
void LedCtlZaj(short pch)
{
	static short ZajTmr[6];
	switch(MES[pch].ZerAdjPhs)
	{
		case 0:
			ZajTmr[pch] = 0;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
			ZajTmr[pch]++;
			if((ZajTmr[pch] % 5)== 0)
			{
				led_channel ^= CH_LED[pch];
			}
			break;
		default:
			break;
	}
}

/****************************************************************************
 * Function : EndAdjVth (End Adjust Vth)
 * Summary  : Vth調整終了処理
 * Argument : pch : チャンネル番号
 * Return   : Flg : 0x00  -> 正常
 *                  0x01  -> フェーズ更新
 *                  0x100 -> エラー
 * Caution  : なし
 * Note     : Phase2
 *            使用する変数    初期値  増減
 *            vth_do_cnt     20      100msに一度
 *            vth_count      0       100msに一度
 *            zero_retry_cnt 0       100msに一度
 *            vth_sum        0       3ms x 6 = 18msに一度
 ***************************************************************************/
short EndAdjVth(short pch)
{
	short Flg = 0;
	short wave_vth_buf;
	/*Vth調整時（2秒間）のVthを算出*/
	wave_vth_buf = (short)(((long long)MES[pch].vth_sum * 25) / ((long long)MES[pch].vth_count * AD_MAX));
	MES[pch].ComVthSum = MES[pch].vth_sum;
	MES[pch].ComVthCount = MES[pch].vth_count;

	LED[pch].zero_vth_buf[LED[pch].zero_retry_cnt] = wave_vth_buf;
	/*Vthデータの収束確認*/
	if( (SVD[pch].wave_vth == wave_vth_buf) ||
	    ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_vth_buf[0])) ) {
		/*Vth調整成功*/
		MES[pch].vth_count = 0;
		SVD[pch].wave_vth = wave_vth_buf;
		eep_write_ch_delay(pch, (short)(&SVD[pch].wave_vth - &SVD[pch].max_flow), SVD[pch].wave_vth);	//エンプティセンサ判定閾値
		Flg = 0x01;
	}else{
		if(LED[pch].zero_retry_cnt >= 2){			/*Vthが収束しない場合*/
			/*Vth調整失敗*/
			Flg = 0x100;
		}else{
			/*Vth調整リトライ*/
			Flg = 0x02;
		}
	}
	return Flg;
}

/****************************************************************************
 * Function : NowZerAdj (Now Zero Adjust)
 * Summary  : ゼロ点調整中処理
 * Argument : pch : チャンネル番号
 * Return   : Flg : 0x00  -> 正常
 *                  0x01  -> フェーズ更新
 *                  0x100 -> エラー
 * Caution  : なし
 * Note     : Phase3
 *            使用する変数    初期値  増減
 *            zero_do_cnt    0       100msに一度
 *            zero_cal_cnt   0       100msに一度
 *            zero_delta_ts  0       100msに一度
 *            zc_zero_calc   0       100msに一度
 ***************************************************************************/
short NowZerAdj(short pch)
{
	short Flg = 0;
	/*ゼロ点調整中*/
	if(LED[pch].zero_do_cnt > 0){
		/*ゼロ調整用⊿Tsの加算*/
		LED[pch].zero_cal_cnt ++;
		LED[pch].zero_delta_ts += (unsigned long)MES[pch].delta_ts;
		MES[pch].zc_zero_calc += MES[pch].zc_Tdata;  //ゼロクロス
	}
	else {
		Flg = 0x01;
	}
	return Flg;
}

/****************************************************************************
 * Function : SavZajPrm (Save Zero adjust Parameter)
 * Summary  : ゼロ点調整成功時のパラメータ保存
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : なし
 * Note     : MES[pch].zc_zero_offset : ゼロクロス用ゼロオフセット点
 ***************************************************************************/
void SavZajPrm(short pch)
{
	char work_ch[14];
	short cnt;

	/*ゼロ調整用⊿Tsの平均値(四捨五入)*/
	memset(work_ch, 0, sizeof(work_ch));
	MES[pch].zc_zero_offset = MES[pch].zc_zero_calc / LED[pch].zero_cal_cnt;  //ゼロクロス時のゼロ点オフセット
	sprintf(&work_ch[0], "%3.11f", MES[pch].zc_zero_offset);
	memcpy(&SVD[pch].ZerCrsOffset[0], &work_ch[0], sizeof(work_ch));
	for(cnt=0; cnt<7; cnt++){  //ゼロクロス時のゼロ点オフセットのEEPROM保存
		eep_write_ch_delay(pch, (short)(&SVD[pch].ZerCrsOffset[cnt] - &SVD[pch].max_flow), SVD[pch].ZerCrsOffset[cnt]);
	}
#if defined(WAVE_RECOGNITION)
	SVD[pch].ThresholdPeakPos = MES[pch].ThresholdPeakPos;	//受波波形検出しきい値のEEPROM保存
	eep_write_ch_delay(pch, (short)(&SVD[pch].ThresholdPeakPos - &SVD[pch].max_flow), SVD[pch].ThresholdPeakPos);
	SVD[pch].fifo_ch_init = MES[pch].ws_FifoCh;		//FIFO CHのEEPROM保存
	eep_write_ch_delay(pch, (short)(&SVD[pch].fifo_ch_init - &SVD[pch].max_flow), SVD[pch].fifo_ch_init);
#else
#endif

	//Debug
	/*ゼロ調整時のゲイン値を保持*/
	MES[pch].zero_amp_gain_rev = MES[pch].amp_gain_rev;
	/*ゼロ調整時のFIFO位置を保持*/
	MES[pch].zero_fifo_ch_read = MES[pch].fifo_ch_read;
	/*ゼロ調整時のFIFO受波波形検出位置を保持*/
	MES[pch].zero_signal_count = MES[pch].signal_count;
	MES[pch].zero_fifo_no_read = MES[pch].fifo_no_read;
	/*ゼロ調整時の波形先頭位置を保持*/
	MES[pch].zero_sonic_point_rev_p2 = MES[pch].sonic_point_rev_p2;
	MES[pch].zero_sonic_point_fow_p1 = MES[pch].sonic_point_fow_p1;
	//Debug

	if(MES[pch].zc_peak_UpdateFlg != 0)
	{
		MES[pch].zc_peak = 0;
	}
	MES_SUB[pch].zc_peak_req = 1;	//波形認識閾値付近のピーク位置を検索要求	
}

/****************************************************************************
 * Function : EndZerAdj (End Zero Adjust)
 * Summary  : ゼロ点終了時処理
 * Argument : pch : チャンネル番号
 * Return   : Flg : 0x00  -> 正常
 *                  0x01  -> フェーズ更新
 *                  0x100 -> エラー
 * Caution  : なし
 * Note     : Phase4
 *            
 ***************************************************************************/
#if 1 //ゼロクロス用
short EndZerAdj(short pch)
{
	short Flg = 0;
	char work_ch[14];
	float zc_work_delta_ts;
	float SvdWork;
	/*ゼロ調整終了処理*/
	// if(LED[pch].zero_do_cnt == 0 && LED[pch].zero_cal_cnt != 0){
	if(LED[pch].zero_cal_cnt != 0)
	{
		//現在のΔTsの計算
		zc_work_delta_ts = MES[pch].zc_zero_calc / LED[pch].zero_cal_cnt;
		LED[pch].zero_dt_buf_f[LED[pch].zero_retry_cnt] = zc_work_delta_ts;
		
		//前回のΔTsの計算
		memcpy(&work_ch[0], &SVD[pch].ZerCrsOffset[0], sizeof(SVD[pch].ZerCrsOffset));
		SvdWork = atof(&work_ch[0]);

		/*⊿Tsデータの収束確認*/
		if((SvdWork == zc_work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub_f(&LED[pch].zero_dt_buf_f[0])) ) {
			SavZajPrm(pch);
			Flg = 0x01;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*⊿Tsが収束しない場合*/
				/*ゼロ調整失敗*/
				Flg = 0x100;
			}else{
				/*ゼロ調整リトライ*/
				//ΔTs更新
				memset(work_ch, 0, sizeof(work_ch));
				sprintf(&work_ch[0], "%3.11f", zc_work_delta_ts);
				memcpy(&SVD[pch].ZerCrsOffset[0], &work_ch[0], sizeof(work_ch));
				
				Flg = 0x02;
			}
		}
	}
	else 
	{
		Flg = 0x100;
	}
	return Flg;
}
#else //差分相関用
short EndZerAdj(short pch)
{
	short Flg = 0;
	char work_ch[14];
	unsigned short work_delta_ts;
	short cnt;
	/*ゼロ調整終了処理*/
	// if(LED[pch].zero_do_cnt == 0 && LED[pch].zero_cal_cnt != 0){
	if(LED[pch].zero_cal_cnt != 0)
	{
		work_delta_ts = (unsigned short)((LED[pch].zero_delta_ts * 2 + 1) / (LED[pch].zero_cal_cnt * 2));
		LED[pch].zero_dt_buf[LED[pch].zero_retry_cnt] = (short)work_delta_ts;
		SavZajPrm(pch); //パラメータ保存

		if(MES[pch].zc_peak_UpdateFlg != 0)
		{
			MES[pch].zc_peak = 0;
		}
		MES_SUB[pch].zc_peak_req = 1;	//波形認識閾値付近のピーク位置を検索要求
		
		/*⊿Tsデータの収束確認*/
		if((SVD[pch].zero_offset == work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_dt_buf[0])) ) {
			/*ゼロ調整成功*/
			SVD[pch].zero_offset = work_delta_ts;
			eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)work_delta_ts);
			Flg = 0x01;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*⊿Tsが収束しない場合*/
				/*ゼロ調整失敗*/
				Flg = 0x100;
			}else{
				/*ゼロ調整リトライ*/
				SVD[pch].zero_offset = work_delta_ts;	/*⊿Ts更新*/
				Flg = 0x02;
			}
		}
		work_delta_ts = (unsigned short)((LED[pch].zero_delta_ts * 2 + 1) / (LED[pch].zero_cal_cnt * 2));
		LED[pch].zero_dt_buf[LED[pch].zero_retry_cnt] = (short)work_delta_ts;
		SavZajPrm(pch); //パラメータ保存

		if(MES[pch].zc_peak_UpdateFlg != 0)
		{
			MES[pch].zc_peak = 0;
		}
		MES_SUB[pch].zc_peak_req = 1;	//波形認識閾値付近のピーク位置を検索要求
		
		/*⊿Tsデータの収束確認*/
		if((SVD[pch].zero_offset == work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_dt_buf[0])) ) {
			/*ゼロ調整成功*/
			SVD[pch].zero_offset = work_delta_ts;
			eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)work_delta_ts);
			Flg = 0x01;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*⊿Tsが収束しない場合*/
				/*ゼロ調整失敗*/
				Flg = 0x100;
			}else{
				/*ゼロ調整リトライ*/
				SVD[pch].zero_offset = work_delta_ts;	/*⊿Ts更新*/
				Flg = 0x02;
			}
		}
	}
	else 
	{
		Flg = 0x100;
	}
	return Flg;
}
#endif //ゼロクロス用

/****************************************************************************
 * Function : IniZajPrm (Init Zeroadjust Parameter)
 * Summary  : ゼロ点調整各フェーズごとのパラメータ初期化
 * Argument : pch : チャンネル番号
 *            Phs : フェーズ番号
 * Return   : void
 * Caution  : なし
 * Note     : 
 ***************************************************************************/
void IniZajPrm(short pch, short Phs)
{
	switch(Phs)
	{
		//波形認識終了待ち
		case 1:
			LED[pch].wave_do_cnt = 0; //波形認識時間カウンタ
			break;
		//Vth調整中
		case 2:
			LED[pch].vth_do_cnt = VTH_ADJ_TIME; //vth調整時間カウンタ
			MES[pch].vth_count = 0; //vth加算数
			MES[pch].vth_sum = 0; //vth合計
			MES[pch].ComVthCount = 0;
			MES[pch].ComVthSum = 0;
			memset(LED[pch].zero_vth_buf, 0, sizeof(LED[pch].zero_vth_buf)); //リトライ用バッファ
			LED[pch].zero_retry_cnt = 0; //vthリトライ回数
			break;
		//Vth調整リトライ
		case 12:
			LED[pch].vth_do_cnt = VTH_ADJ_TIME;
			MES[pch].vth_count = 0;
			MES[pch].vth_sum = 0;
			// memset(LED[pch].zero_vth_buf, 0, sizeof(LED[pch].zero_vth_buf));
			LED[pch].zero_retry_cnt++;
			break;
		//時間差調整中
		case 3:
			LED[pch].zero_retry_cnt = 0;
			memset(LED[pch].zero_dt_buf, 0, sizeof(LED[pch].zero_dt_buf)); //差分相関リトライ用バッファ
			memset(LED[pch].zero_dt_buf_f, 0, sizeof(LED[pch].zero_dt_buf_f)); //ゼロクロスリトライ用バッファ
			LED[pch].zero_do_cnt = ZERO_ADJ_TIME; //dt調整時間
			LED[pch].zero_cal_cnt = 0; //dt加算数
			LED[pch].zero_delta_ts = 0; //フィルタ後dt(上下流時間差)合計
			MES[pch].zc_zero_calc = 0; //dt(上下流時間差)合計
			if(reset_factor != RESTART_WDOG){				//再起動エラー以外
				MAIN[pch].com_err_status = (short)0;			/*エラーステータスクリア*/
			}
			break;
		//時間差調整リトライ
		case 13:
			LED[pch].zero_retry_cnt++;
			// memset(LED[pch].zero_dt_buf, 0, sizeof(LED[pch].zero_dt_buf));
			LED[pch].zero_do_cnt = ZERO_ADJ_TIME;		/*ゼロ点調整時間セット*/
			LED[pch].zero_cal_cnt = 0;
			LED[pch].zero_delta_ts = 0;
			MES[pch].zc_zero_calc = 0;
			break;
		//時間差調整終了
		case 4:
			break;
		//ゼロ点調整正常終了
		case 5:
			/*ゼロ調整用⊿Tsの初期化*/
			LED[pch].zero_cal_cnt = 0;
			LED[pch].zero_delta_ts = 0;
			MES[pch].zc_zero_calc = 0;  //ゼロクロス
			MES[pch].err_status &= ~ERR_JUDGE_ZERO;	/*ゼロ点調整状態リセット*/
			if(reset_factor != RESTART_WDOG){				//再起動エラー以外
				MAIN[pch].com_err_status = (short)0;			/*エラーステータスクリア*/
				MAIN[pch].led_err_status = (short)0;			/*エラーステータスクリア*/
			}
			LED[pch].zero_active = 0;
			action_status_control(pch, ACT_STS_NORMAL);	/*動作ステータス更新*/
			zero_adj_status(pch);						/*ゼロ点調整時の波形データを保持する*/
			break;
		default:
			break;
	}
}

/****************************************************************************
 * Function : JdgZerAdjPhs (Judge Zero Adjust Phase)
 * Summary  : ゼロ点調整フェーズ判定
 * Argument : pch : チャンネル番号
 * Return   : void
 * Caution  : なし
 * Note     : Flg : 0x00  -> 正常、フェーズ更新なし
 *                  0x01  -> 正常、フェーズ更新あり
 *                  0x02  -> 異常、リトライ
 *                  0x100 -> 異常、ゼロ点調整終了
 ***************************************************************************/
void JdgZerAdjPhs(short pch)
{
	short Flg = 0;

	//LED処理
	LedCtlZaj(pch);

	//ゼロ点調整エラー
	if((LED[pch].zero_active != 0) && ((MES[pch].err_status & ERR_ZERO_ADJ) != 0))
	{
		Flg |= 0x100;
	}

	switch (MES[pch].ZerAdjPhs)
	{
		//通常測定中
		case 0:
			if(LED[pch].zero_active != 0)
			{
				MES[pch].ZerAdjPhs = 1;
				IniZajPrm(pch, 1); //Phase1用初期化
			}
			break;
		//波形認識終了待ち
		case 1:
#if defined(WAVE_RECOGNITION)
			if(MES[pch].ThresholdReq == 99)
			{
				MES[pch].ZerAdjPhs = 2;
				IniZajPrm(pch, 2); //Phase2用初期化
			}
#else
			MES[pch].ZerAdjPhs = 2;
			IniZajPrm(pch, 2); //Phase2用初期化
#endif
			break;
		//Vth収束待ち
		case 2:
			//フェーズ変更から2s経過
			if(LED[pch].vth_do_cnt == 0)
			{
				//2s間で1度はvthを取得
				if(MES[pch].vth_count != 0)
				{
					Flg |= EndAdjVth(pch);
				}
			}
			//Vth収束を確認(ゼロ調エラーなし&EndAdjVth()が正常終了)
			if(Flg == 0x01)
			{
				MES[pch].ZerAdjPhs = 3;
				IniZajPrm(pch, 3); //Phase3用初期化
			}
			//Vth調整リトライ(ゼロ調エラーなし&EndAdjVth()がリトライ)
			else if(Flg == 0x02)
			{
				MES[pch].ZerAdjPhs = 2; //変数初期化だけしてリトライ
				IniZajPrm(pch, 12); //Phase2リトライ用初期化
			}
			break;
		//ゼロ点調整中
		case 3:
			Flg |= NowZerAdj(pch);
			//zero_do_cnt分待ち完了(ゼロ調エラーなし&NowZerAdj()が正常終了)
			if(Flg == 0x01)
			{
				MES[pch].ZerAdjPhs = 4;
				IniZajPrm(pch, 4); //Phase4用初期化
			}
			break;
		//ゼロ点調整完了処理
		case 4:
			Flg |= EndZerAdj(pch);
			//ゼロ点調整完了(ゼロ調エラーなし&EndZerAdj()が正常終了)
			if(Flg == 0x01)
			{
				MES[pch].ZerAdjPhs = 0;
				IniZajPrm(pch, 5); //ゼロ点調整終了時用初期化
			}
			//Vthの調整からリトライ(ゼロ調エラーなし&EndZerAdj()がリトライ)
			else if(Flg == 0x02)
			{
				MES[pch].ZerAdjPhs = 3;
				IniZajPrm(pch, 13); //Phase3リトライ用初期化
			}
			break;
		//エラー処理
		default:
			MES[pch].ZerAdjPhs = 0;
			break;
	}

	//ゼロ点調整エラー判定
	if((Flg & 0x100) != 0)
	{
		JdgZadErr(pch, MES[pch].ZerAdjPhs);
		MES[pch].ZerAdjPhs = 0;
	}
}

/****************************************************/
/* Function : zero_adj_control                     */
/* Summary  : ゼロ点調整処理         				*/
/* Argument : pch                                 */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : Vth調整後にゼロ点調整を実施する             */
/****************************************************/
void	zero_adj_control(short pch){
#if 1
	JdgZerAdjPhs(pch);
#else
	unsigned short work_delta_ts;
	short wave_vth_buf;
	short cnt;
	char work_ch[14];

	/*Vth調整中*/
	if(LED[pch].vth_do_cnt != 0){
		if((MES[pch].err_status & ERR_ZERO_ADJ) == 0){
			if((LED[pch].vth_do_cnt % 5) == 0){ 		//500msec周期
				led_channel ^= CH_LED[pch];
			}
		}else{							/*Vth調整中のエラー*/
			/*Vth調整失敗*/
			zero_adj_error(pch);		/*ゼロ調整エラー処理*/
			zero_adj_status(pch);						/*ゼロ点調整時の波形データを保持する*/
		}
	}
	/*波形認識調整中(CH-LEDを点滅)*/
	if(LED[pch].wave_do_cnt != 0){
		if((LED[pch].wave_do_cnt % 5) == 0){ 		//500msec周期
			led_channel ^= CH_LED[pch];
		}
	}

	/*波形認識終了処理*/
	if(LED[pch].zero_active == 1 && MES[pch].ThresholdReq == 99){			/*波形認識終了*/
			MES[pch].ThresholdReq = 0;  /*波形認識処理の終了*/
			LED[pch].wave_do_cnt = 0;	/*波形認識調整時間クリア*/
		/*Vth調整開始*/
			LED[pch].vth_do_cnt = VTH_ADJ_TIME;		/*Vth調整時間セット*/
	}
	
	/*Vth調整終了処理*/
	if(LED[pch].vth_do_cnt == 0 && MES[pch].vth_count != 0){
		/*Vth調整時（2秒間）のVthを算出*/
		wave_vth_buf = (short)(((long long)MES[pch].vth_sum * 25) / ((long long)MES[pch].vth_count * AD_MAX));

		LED[pch].zero_vth_buf[LED[pch].zero_retry_cnt] = wave_vth_buf;
		/*Vthデータの収束確認*/
		if( (SVD[pch].wave_vth == wave_vth_buf) ||
		    ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_vth_buf[0])) ) {
			/*Vth調整成功*/
			MES[pch].vth_count = 0;
			SVD[pch].wave_vth = wave_vth_buf;
			eep_write_ch_delay(pch, (short)(&SVD[pch].wave_vth - &SVD[pch].max_flow), SVD[pch].wave_vth);	//エンプティセンサ判定閾値

			/*ゼロ調整開始*/
			if(reset_factor != RESTART_WDOG){				//再起動エラー以外
				MAIN[pch].com_err_status = (short)0;			/*エラーステータスクリア*/
			}
			LED[pch].zero_retry_cnt = 0;				/*ゼロ調整リトライカウンタ初期化*/
			LED[pch].zero_do_cnt = ZERO_ADJ_TIME;		/*ゼロ点調整時間セット*/
			LED[pch].zero_dt_buf[0] = LED[pch].zero_dt_buf[1] = LED[pch].zero_dt_buf[2] = 0;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*Vthが収束しない場合*/
				/*Vth調整失敗*/
				zero_adj_error(pch);					/*ゼロ調整エラー処理*/
				MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*ゼロ調整計測時間発散エラーセット*/
				MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
				err_judge_status(pch);
				zero_adj_status(pch);						/*ゼロ点調整時の波形データを保持する*/
			}else{
				/*Vth調整リトライ*/
				SVD[pch].wave_vth = wave_vth_buf;		/*Vth更新*/
				LED[pch].zero_retry_cnt ++;			/*ゼロ調整リトライカウンタ更新*/
				MES[pch].vth_sum = 0;
				MES[pch].vth_count = 0;
				LED[pch].vth_do_cnt = VTH_ADJ_TIME;	/*Vth調整時間セット*/
			}
		}
	}	
	
	/*ゼロ点調整中*/
	if(LED[pch].zero_do_cnt != 0){
		if((MES[pch].err_status & ERR_ZERO_ADJ) == 0){
			/*ゼロ調整用⊿Tsの加算*/
			LED[pch].zero_cal_cnt ++;
			LED[pch].zero_delta_ts += (unsigned long)MES[pch].delta_ts;
			MES[pch].zc_zero_calc += MES[pch].zc_Tdata;  //ゼロクロス
			if((LED[pch].zero_do_cnt % 5) == 0){ 	//500msec周期
				led_channel ^= CH_LED[pch];
			}
		}else{							/*zero調整中のエラー*/
			/*ゼロ調整失敗*/
			zero_adj_error(pch);		/*ゼロ調整エラー処理*/
			zero_adj_status(pch);						/*ゼロ点調整時の波形データを保持する*/
		}
	}
	
	/*ゼロ調整終了処理*/
	if(LED[pch].zero_do_cnt == 0 && LED[pch].zero_cal_cnt != 0){
		/*ゼロ調整用⊿Tsの平均値(四捨五入)*/
		memset(work_ch, 0, sizeof(work_ch));
		work_delta_ts = (unsigned short)((LED[pch].zero_delta_ts * 2 + 1) / (LED[pch].zero_cal_cnt * 2));
		MES[pch].zc_zero_offset = MES[pch].zc_zero_calc / LED[pch].zero_cal_cnt;  //ゼロクロス時のゼロ点オフセット
		LED[pch].zero_dt_buf[LED[pch].zero_retry_cnt] = (short)work_delta_ts;
		sprintf(&work_ch[0], "%3.11f", MES[pch].zc_zero_offset);
		memcpy(&SVD[pch].ZerCrsOffset[0], &work_ch[0], sizeof(work_ch));
		for(cnt=0; cnt<7; cnt++){  //ゼロクロス時のゼロ点オフセットのEEPROM保存
			eep_write_ch_delay(pch, (short)(&SVD[pch].ZerCrsOffset[cnt] - &SVD[pch].max_flow), SVD[pch].ZerCrsOffset[cnt]);
		}
		SVD[pch].ThresholdPeakPos = MES[pch].ThresholdPeakPos;	//受波波形検出しきい値のEEPROM保存
		eep_write_ch_delay(pch, (short)(&SVD[pch].ThresholdPeakPos - &SVD[pch].max_flow), SVD[pch].ThresholdPeakPos);
		SVD[pch].fifo_ch_init = MES[pch].ws_FifoCh;		//FIFO CHのEEPROM保存
		eep_write_ch_delay(pch, (short)(&SVD[pch].fifo_ch_init - &SVD[pch].max_flow), SVD[pch].fifo_ch_init);
		
		//Debug
		/*ゼロ調整時のゲイン値を保持*/
		MES[pch].zero_amp_gain_rev = MES[pch].amp_gain_rev;
		/*ゼロ調整時のFIFO位置を保持*/
		MES[pch].zero_fifo_ch_read = MES[pch].fifo_ch_read;
		/*ゼロ調整時のFIFO受波波形検出位置を保持*/
		MES[pch].zero_signal_count = MES[pch].signal_count;
		MES[pch].zero_fifo_no_read = MES[pch].fifo_no_read;
		/*ゼロ調整時の波形先頭位置を保持*/
		MES[pch].zero_sonic_point_rev_p2 = MES[pch].sonic_point_rev_p2;
		MES[pch].zero_sonic_point_fow_p1 = MES[pch].sonic_point_fow_p1;
		//Debug
		
		if(MES[pch].zc_peak_UpdateFlg != 0)
		{
			MES[pch].zc_peak = 0;
		}
		MES_SUB[pch].zc_peak_req = 1;	//波形認識閾値付近のピーク位置を検索要求
		
		/*⊿Tsデータの収束確認*/
		if((SVD[pch].zero_offset == work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_dt_buf[0])) ) {
			/*ゼロ調整成功*/
			SVD[pch].zero_offset = work_delta_ts;
			eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)work_delta_ts);
			
		
			/*ゼロ調整用⊿Tsの初期化*/
			LED[pch].zero_cal_cnt = 0;
			LED[pch].zero_delta_ts = 0;
			MES[pch].zc_zero_calc = 0;  //ゼロクロス
			MES[pch].err_status &= ~ERR_JUDGE_ZERO;	/*ゼロ点調整状態リセット*/
			if(reset_factor != RESTART_WDOG){				//再起動エラー以外
				MAIN[pch].com_err_status = (short)0;			/*エラーステータスクリア*/
				MAIN[pch].led_err_status = (short)0;			/*エラーステータスクリア*/
			}
			LED[pch].zero_active = 0;
			action_status_control(pch, ACT_STS_NORMAL);	/*動作ステータス更新*/
			zero_adj_status(pch);						/*ゼロ点調整時の波形データを保持する*/
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*⊿Tsが収束しない場合*/
				/*ゼロ調整失敗*/
				zero_adj_error(pch);					/*ゼロ調整エラー処理*/
				MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*ゼロ調整計測時間発散エラーセット*/
				MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
				err_judge_status(pch);
				zero_adj_status(pch);						/*ゼロ点調整時の波形データを保持する*/
			}else{
				/*ゼロ調整リトライ*/
				SVD[pch].zero_offset = work_delta_ts;	/*⊿Ts更新*/
				LED[pch].zero_retry_cnt ++;			/*ゼロ調整リトライカウンタ更新*/
				LED[pch].zero_cal_cnt = 0;
				LED[pch].zero_delta_ts = 0;
				MES[pch].zc_zero_calc = 0;  //ゼロクロス
				LED[pch].zero_do_cnt = ZERO_ADJ_TIME;	/*ゼロ点調整時間セット*/
			}
		}
	}
#endif
}

/****************************************************/
/* Function : zero_adj_error                       */
/* Summary  : ゼロ点調整エラー処理               				*/
/* Argument : pch                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	zero_adj_error(short pch){

	/*ゼロ調整用⊿Tsの初期化*/
	LED[pch].zero_active = 0;
	LED[pch].zero_cal_cnt = 0;
	LED[pch].zero_delta_ts = 0;
	LED[pch].zero_do_cnt = 0;
	LED[pch].vth_do_cnt = 0;
	LED[pch].wave_do_cnt = 0;
	MES[pch].vth_count = 0;
	MES[pch].err_status &= ~ERR_JUDGE_ZERO;	/*ゼロ点調整状態リセット*/
	MES[pch].zc_zero_calc = 0;  //ゼロクロス
	action_status_control(pch, ACT_STS_NORMAL);	/*動作ステータス更新*/

	MES[pch].ThresholdReq = 0; //zero_active=0になるので99ではなく0
}

/****************************************************/
/* Function : alm_reset_control                       */
/* Summary  : アラームリセット               				*/
/* Argument : pch                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	alm_reset_control(short pch){

	short ch_read;
	short l;

	/*アラームリセットSWによるアラームリセット*/
	if(sw_now_almreset[pch] == B_ON){				//アラームリセット
		ch_read = disp_ch_read();					//CHスイッチ読み込み
		if((ch_read == SW_CH0)||					//CH0設定時：全CHゼロ点調整
			(pch == (ch_read - 1))){				//指定CHゼロ点調整
			if(MAIN[pch].com_err_status == ERR_RESTART){	//再起動エラー
				;										//エラークリアしない
			}else if((MAIN[pch].com_err_status == ERR_EEPROM)||	//EEPROMエラーとCUnetエラーは全CHクリア
				(MAIN[pch].com_err_status == ERR_CUNET)){		
				for(l=0;l<6;l++){
					MES[l].err_status = (short)0;		//正常ステータスに更新
					MES[l].alm_status = (short)0;		//正常ステータスに更新
					MAIN[l].err_judge = (short)0;
					MAIN[l].com_err_status = (short)0;	//エラーステータスクリア
					MAIN[l].led_err_status = (short)0;	//エラーステータスクリア
				}
			}else{
				MES[pch].err_status = (short)0;			//正常ステータスに更新
				MES[pch].alm_status = (short)0;			//正常ステータスに更新
				MAIN[pch].err_judge = (short)0;
				MAIN[pch].com_err_status = (short)0;		//エラーステータスクリア
				MAIN[pch].led_err_status = (short)0;		//エラーステータスクリア
			}
		}
	}
}

/****************************************************/
/* Function : zero_adj_status                       */
/* Summary  : ゼロ点調整時の波形データを保持する        				*/
/* Argument : pch                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void zero_adj_status(short pch){

	short		i_cnt;

	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		return;						//登録不可
	}
	
	SVD[pch].zero_flow_qat.DWORD = MES[pch].ml_min_now;		//流量[mL/min]
	SVD[pch].zero_flow_vel.DWORD = MES[pch].flow_vel_c;		//流速[m/s]
	SVD[pch].zero_sound_spd = MES[pch].sound_vel_f;			//音速[m/s]
	SVD[pch].zero_addit.DWORD = MES[pch].addit_buff.DWORD;		//積算値[mL/min]
	SVD[pch].zero_wave_max = MES[pch].rev_max_data;			//受波波形最大値
	SVD[pch].zero_wave_min = MES[pch].rev_min_data;			//受波波形最小値
	SVD[pch].zero_delta_ts.DWORD = MES[pch].delta_ts_zero;		//伝搬時間差[ps]
	SVD[pch].zero_correlate = MES[pch].correlate;				//相関値幅
	SVD[pch].zero_zero_offset = SVD[pch].zero_offset;		//ゼロ点オフセット
	SVD[pch].zero_condition = MAIN[pch].err_condition;			//status
	SVD[pch].zero_fifo_pos = MES[pch].fifo_no_read;			//FIFO受波波形検出位置
	SVD[pch].zero_gain_1st = get_attenuator_gain(pch);			//Gain 1st stage
	SVD[pch].zero_gain_2nd = MES[pch].amp_gain_for;			//Gain 2nd stage
	SVD[pch].zero_fifo_ch = MES[pch].fifo_ch_read;				//FIFO CH
	SVD[pch].zero_p1p2 = MES[pch].max_point_sub_f;				//受波の差(P1-P2)
	SVD[pch].zero_FwdTimDif.DWORD = (long)(GetTimDif(pch, 0) * 1000.0);
	SVD[pch].zero_RevTimDif.DWORD = (long)(GetTimDif(pch, 1) * 1000.0);
	SVD[pch].zero_FwdSurplsTim = MES[pch].FwdSurplsTim;
	SVD[pch].zero_RevSurplsTim = MES[pch].RevSurplsTim;
	SVD[pch].zero_drive_freq = SVD[pch].drive_freq;
	for(i_cnt=0; i_cnt<WAV_PEK_NUM; i_cnt++){
		SVD[pch].zero_FwdWavPekPosLst[i_cnt] = MES[pch].FwdWavPekPosLst[i_cnt];
		SVD[pch].zero_FwdWavPekValLst[i_cnt] = MES[pch].FwdWavPekValLst[i_cnt];
		SVD[pch].zero_RevWavPekPosLst[i_cnt] = MES[pch].RevWavPekPosLst[i_cnt];
		SVD[pch].zero_RevWavPekValLst[i_cnt] = MES[pch].RevWavPekValLst[i_cnt];
	}
	
	//EEPROM書き込み
	SavEepZerAdjPrm(pch);
	
}

/****************************************************/
/* Function : com_req_check                       */
/* Summary  : 通信指令確認        				*/
/* Argument : なし                                 */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_req_check(void){

	short ch;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		com_req_control(ch);
	}
}

/*******************************************
 * Function : SetZerAdjPrm
 * Summary  : ゼロ調整のパラメータを設定する
 * Argument : Mod : Mode
 *                : 0 -> ゼロ調整用⊿Tsの初期化
 *                : 1 -> 波形認識処理開始
 * Return   : void
 * Caution  : None
 * Note     : None
 * *****************************************/
void SetZerAdjPrm(short pch, short Mod)
{
	if(Mod == 0)
	{
		/*ゼロ調整用⊿Tsの初期化*/
		LED[pch].zero_cal_cnt = 0;
		LED[pch].zero_delta_ts = 0;
		MES[pch].vth_sum = 0;
		MES[pch].vth_count = 0;
													/*コンフィグレータからのゼロ点調整は、エラーステータスがある場合でも*/
													/*ゼロ点調整を実行する（SFC-012と同仕様とする）						*/
		MES[pch].err_status = (short)0;				/*正常ステータスに更新	*/
		MES[pch].err_status |= ERR_JUDGE_ZERO;		/*ゼロ点調整状態セット	*/
		MES[pch].alm_status = (short)0;			//正常ステータスに更新
		MAIN[pch].err_sub_judge = MES_SUB[pch].err_status_sub = (short)0;	//正常ステータスに更新
		MES[pch].zc_zero_calc = 0;  //ゼロクロス
		LED[pch].zero_vth_buf[0] = LED[pch].zero_vth_buf[1] = LED[pch].zero_vth_buf[2] = 0;
	}
	else if(Mod == 1)
	{
		/*波形認識処理開始*/
		LED[pch].zero_active = 1;					/*ゼロ調整実行中*/
		LED[pch].zero_retry_cnt = 0;				/*ゼロ調整リトライカウンタ初期化*/
#if defined(WAVE_RECOGNITION)  //FIFO CHサーチを実行する
		MES[pch].ThresholdReq = 11;	/*波形認識実行要求*/
		MES[pch].ThresholdWave = AD_MAX_UNIT;		/*波形認識閾値*/
		MES[pch].ThresholdPeak = 0;	/*波形認識ピーク*/
		MES[pch].ThresholdPeakPos = 0;	 /*波形認識ピーク位置*/
#else //実行しない
		MES[pch].ThresholdReq = 0;	/*波形認識実行要求*/
#endif
		LED[pch].wave_do_cnt = WAVE_ADJ_TIME;	/*波形認識調整時間セット*/
		led_channel |= CH_LED[pch];				/*点滅させるLEDをセット*/
		led_alarm = 0;								/*ALM-LED消灯*/
		action_status_control(pch, ACT_STS_ZERO);	/*動作ステータス更新*/
	}
}

/*******************************************
 * Function : CheckSearchWindow
 * Summary  : ウィンドウサーチの判定をする
 * Argument : 
 * Return   : B_NG -> 失敗
 *            B_OK -> 成功
 * Caution  : None
 * Note     : None
 * *****************************************/
short CheckSearchWindow(short pch)
{
	/*Windowサーチ(最適なFIFO CHを探す)*/
	action_status_control(pch, ACT_STS_ZERO);	/*動作ステータス更新*/
	if(SearchWindow(pch) != B_OK){	/*Windowサーチ*/			
		/*ゼロ調整失敗*/
		zero_adj_error(pch);				/*ゼロ調整エラー処理*/
		MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*ゼロ調整計測時間発散エラーセット*/
		MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
		err_judge_status(pch);
		zero_adj_status(pch);				/*ゼロ点調整時の波形データを保持する*/
		return B_NG;
	}
	return B_OK;
}

/****************************************************/
/* Function : com_req_control                       */
/* Summary  : 指令制御        				*/
/* Argument : pch                                 */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_req_control(short pch){
	
	short l;
	short ch_read;

	/*ゼロ点調整SWによるゼロ点調整の確認*/
	if ((SVD[pch].sensor_size != SNS_NONE)
		&& (sw_now_zeroadj == B_ON) && (MES[pch].inspect_enable == 0)){
		ch_read = disp_ch_read();				//CHスイッチ読み込み
		if((ch_read == SW_CH0)||				//CH0設定時：全CHゼロ点調整
			(pch == (ch_read - 1))){			//指定CHゼロ点調整

			// /*ゼロ調整用⊿Tsの初期化*/
			SetZerAdjPrm(pch, 0);

			/*メモリデバイス実装位置の判別*/
			OWCheckDeviceSide();

			/*メモリデバイス検知の確認*/
			CheckDeviceDetect();

#if defined(WAVE_RECOGNITION)
			// /*Windowサーチ(最適なFIFO CHを探す)*/
			if(CheckSearchWindow(pch) == B_NG) return;
#endif

			// /*波形認識処理開始*/
			SetZerAdjPrm(pch, 1);
		}
	}

#if defined(FRQSCH)
	// 周波数サーチ終了時、ゼロ調実施
	if(FrqSch[pch].FrqSchSttFlg == 2){
		eep_write_ch_delay(pch, (short)(&SVD[pch].SchFrq - &SVD[pch].max_flow), SVD[pch].SchFrq);
		SAVE[pch].control |= 0x0001; //ゼロ調開始フラグON
		// 周波数サーチ開始フラグリセット
		FrqSch[pch].FrqSchSttFlg = 0;
	}
#endif
	
	/*通信指令によるゼロ点調整の確認*/
	if(((SAVE[pch].control_old & 0x0001) == 0) && ((SAVE[pch].control & 0x0001) != 0)){
		SAVE[pch].control &= ~0x0001;			/*ゼロ調整コマンドリセット*/

		if((SVD[pch].sensor_size != SNS_NONE) && (MES[pch].inspect_enable == 0)){

			/*ゼロ調整用⊿Tsの初期化*/
			SetZerAdjPrm(pch, 0);

			/*メモリデバイス実装位置の判別*/
			OWCheckDeviceSide();

			/*メモリデバイス検知の確認*/
			CheckDeviceDetect();

#if defined(WAVE_RECOGNITION)
			// /*Windowサーチ(最適なFIFO CHを探す)*/
			if(CheckSearchWindow(pch) == B_NG) return;
#endif

			// /*波形認識処理開始*/
			SetZerAdjPrm(pch, 1);
		}
	}	

	/*通信指令によるアラームリセットの確認*/
	if(((SAVE[pch].control_old & 0x0002) == 0) && ((SAVE[pch].control & 0x0002) != 0)){
		SAVE[pch].control &= ~0x0002;			/*アラームリセットコマンドリセット*/
		if(MAIN[pch].com_err_status == ERR_RESTART){			//再起動エラー
				;												//エラークリアしない
		}else if((MAIN[pch].com_err_status == ERR_EEPROM)||	//EEPROMエラーとCUnetエラーは全CHクリア
			(MAIN[pch].com_err_status == ERR_CUNET)){
			for(l=0;l<6;l++){
				MES[l].err_status = (short)0;		//正常ステータスに更新
				MES[l].alm_status = (short)0;			//正常ステータスに更新
				MAIN[l].err_judge = (short)0;
				MAIN[l].com_err_status = (short)0;	//エラーステータスクリア
				MAIN[l].led_err_status = (short)0;	//エラーステータスクリア
			}
			reset_factor = RESTART_NORMAL;			//再起動要因コードクリア
		}else{
			MES[pch].err_status = (short)0;			//正常ステータスに更新
			MES[pch].alm_status = (short)0;			//正常ステータスに更新
			MAIN[pch].err_judge = (short)0;
			MAIN[pch].com_err_status = (short)0;		/*エラーステータスクリア*/
			MAIN[pch].led_err_status = (short)0;		/*エラーステータスクリア*/
		}
	}

	/*通信指令によるリセット(再起動)の確認*/
	if(((SAVE[pch].control_old & 0x0008) == 0) && ((SAVE[pch].control & 0x0008) != 0)){
		SAVE[pch].control &= ~0x0008;			/*リセット(再起動)コマンドリセット*/
		delay(10000);							/*待機*/
		reset_control();				/*リセット処理*/
	}
	
	SAVE[pch].control_old = SAVE[pch].control;		/*状態保存*/	
}

/****************************************************/
/* Function : ram_clear_check                        */
/* Summary  : RAMクリア    				*/
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : CHスイッチ=9、ゼロ調整スイッチON起動でRAMクリア */
/****************************************************/
void	ram_clear_check(void){

	short ch = 0;
	
	if(disp_ch_read() == SW_CH9 &&				//RAMクリア実行条件の確認
		disp_zero_read() == B_ON){				//表示切替:CH9、ゼロ点調整SW:オン
		for(ch = CH1; ch < CH_NUMMAX; ch++){	//RAMクリア実行
			read_serial_num(ch);				//シリアル番号読込み
			drive_led(DSP_CH_ALL);	//CH-LED全灯
			eep_sv_write(ch);					//初期値書込み
			drive_led(DSP_ALM_ALL);	//ALM-LED全灯
			write_serial_num(ch);				//シリアル番号書込み*
			eep_sv_read(ch);					//EEPROMの読込み/
		}
	}
}

/****************************************************/
/* Function : ram_clear_debug                       */
/* Summary  : RAMクリア    				*/
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 通信(ORコマンド)によるRAMクリア               */
/****************************************************/
void	ram_clear_debug(void){

	short ch = 0;
	
	for(ch = CH1; ch < CH_NUMMAX; ch++){	//RAMクリア実行
		read_serial_num(ch);				//シリアル番号読込み
		eep_sv_write(ch);					//初期値書込み
		write_serial_num(ch);				//シリアル番号書込み*
		eep_sv_read(ch);					//EEPROMの読込み/
	}

	version_set();
}

/****************************************************/
/* Function : ReadSensDevice                      */
/* Summary  : メモリデバイスからセンサ情報を読込む    				*/
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	ReadSensDevice(void){

	short ch = 0;

	for(ch=CH1; ch<CH_NUMMAX; ch++){
		if(OWReadSensinfo(ch) == B_OK){     //センサ情報を読込む
			util_eep_allwrite(ch, WR_DEVICE);  //読込んだセンサ情報をEEPROM書込む
		}
	}
}

/****************************************************/
/* Function : CheckDeviceDetect						*/
/* Summary  : メモリデバイス検知の確認						*/
/* Argument : なし									*/
/* Return   : なし									*/
/* Caution  : なし									*/
/* notes    : メモリデバイスがIN側とOUT側共に検知できない場合	*/
/*          : にエラーとする								*/
/****************************************************/
void	CheckDeviceDetect(void){

	short ch;

	for(ch=CH1; ch<CH_NUMMAX; ch++){
		if(SVD[ch].sensor_size != SNS_NONE && MES_SUB[ch].memory_side == B_NG){	//メモリデバイスがIN側とOUT側共に検知できない場合
			MES_SUB[ch].err_status_sub |= ERR_JUDGE_DEVICE;	//異常：メモリデバイス検知エラーセット
		}else{
			MES_SUB[ch].err_status_sub &= ~ERR_JUDGE_DEVICE;	//正常：メモリデバイス検知エラークリア
		}
	}
}

/****************************************************/
/* Function : watchdog_refresh                      */
/* Summary  : ウォッチドッグリフレッシュ    				*/
/* Argument : なし                                  */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	watchdog_refresh(void)
{
	// RX 版 FW では「PCLK/32768（周期167.8ms）* 256」で約 4 秒なのでそれに合わせる 
	WatchdogReloadSet(WATCHDOG0_BASE, g_ui32SysClock* 4);
}

/****************************************************/
/* Function : CheckBOOTCFG                           */
/* Summary  : BOOTCFG レジスタが設定済みかどうかのチェックと設定	*/
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void CheckBOOTCFG(void){

	if(HWREG(FLASH_BOOTCFG) == 0xfffffffe)
	{
		HWREG(FLASH_FMA) = 0x75100000;
		//	BOOTCFG 
		//		reset 0xFFFF.FFFE
		//		31
		//			NW
		//				Not Written
		//				When set, this bit indicates that the values in this register can be changed
		//				from 1 to 0. When clear, this bit specifies that the contents of this register
		//				cannot be changed.
		//		15:13
		//			PORT
		//				Boot GPIO Port
		//					0x0 Port A
		//					0x1 Port B
		//					0x2 Port C
		//					0x3 Port D
		//					0x4 Port E
		//					0x5 Port F
		//					0x6 Port G
		//					0x7 Port H
		//		12:10
		//			PIN
		//				Boot GPIO Pin
		//		9
		//			POL
		//				Boot GPIO Polarity
		//				When set, this bit selects a high level for the GPIO port pin to enable
		//				the ROM boot loader at reset. When clear, this bit selects a low level
		//				for the GPIO port pin.
		//		8
		//			EN
		//				Boot GPIO Enable
		//				Clearing this bit enables the use of a GPIO pin to enable the ROM Boot
		//				Loader at reset.
		HWREG(FLASH_FMD) = 0x7fff00fe | FLASH_BOOTCFG_PORT_H | FLASH_BOOTCFG_PIN_7 | FLASH_BOOTCFG_POL;
		HWREG(FLASH_FMC) = FLASH_FMC_WRKEY | FLASH_FMC_COMT;
		while((HWREG(FLASH_FMC)&FLASH_FMC_COMT)== FLASH_FMC_COMT);
	}
}

/****************************************************/
/* Function : main                                  */
/* Summary  : メイン処理                          				 */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void main(void){

	// BOOTCFG レジスタが設定済みかどうかのチェックと設定 
	CheckBOOTCFG();

 /*初期化*/
	init();

	while(1){
		/*** 100msec待機 ***/
		while(timer_main != 0){
			/*EEPROMにデータを書込む*/
			eep_write_pending_data();

			/*メモリデバイスにセンサ情報を書込む*/
			OWWriteSensinfo();
		}
		timer_main = 20;
	
		/*** メイン処理               ***/
		/*** 以下100msec周期で処理する ***/
		/*カウンタ処理*/
		counter_control();

		/*CPU動作状態表示*/
		disp_cpu_status();

		/*ゼロ点調整SW/アラームリセットSW入力確認*/
		disp_zerosw_check();

		/*通信指令確認*/
		com_req_check();

		/*ゼロ点調整/アラームリセット確認*/
		zero_alm_check();

		/*CUnet通信リンク確認*/
		if(com_type == COM_CUNET){
			mky43_host_link_check();
		}
	}
}

/******************************************************************/
#ifdef __cplusplus
void abort(void)
{
	
}
#endif

// デバッガ動作中の WATCHDOG0 用の割り込みハンドラ 
void WatchdogIntHandler(void)
{
	WatchdogIntClear(WATCHDOG0_BASE);
	
	__asm(" bkpt #0");
	__asm(" nop");
	return;
}

