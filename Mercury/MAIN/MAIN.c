/***********************************************/
/* File Name : MAIN.c	   	         									   */
/*	Summary   : ���C������ 					                   */
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
/*	���W���[������`�֐�								*/
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
/*	���W���[���O��`�֐�								*/
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
/*	���W���[������`�ϐ�								*/
/********************************************************/
short ChCtrlNow = CH1;	//����CH(���ʐ���)
short main_count;
short com_type;
short initializing = 0;
uint32_t g_ui32SysClock;
static uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));	// uDMAControlBaseSet �Ŏg�p 
short FpgaVersion;

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern short timer_main;					/*���C�������^�C�}�[*/
extern unsigned char sw_now_zeroadj;	/*�[���_����SW���*/
extern unsigned char sw_now_almreset[6];/*�A���[�����Z�b�gSW���*/
extern short	led_channel;			//Channel LED�\�����
extern short	led_alarm;				//Alarm LED�\�����
extern short 	CH_LED[6];				//CH-LED�_�����

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

// #define WAVE_RECOGNITION //�[���_�������ɔg�`�F�������s����

/****************************************************/
/* Function : fpga_config_send_SSI                  */
/* Summary  : SPI Passive Mode - fpga_config_send   */
/* Argument : �Ȃ�                                  */
/* Return   : 1=����, 0=�ُ�(�^�C���A�E�g)          */
/* Caution  : �Ȃ�                                  */
/* notes    : �N����                                */
/****************************************************/
#if defined(FPGADOWNLOAD)
short fpga_config_send(void)
{
	const unsigned long GPIO_PORTJ_CCK_DATA = GPIO_PORTJ_BASE + 0xC0;    /* GPIO_PJ4 (CCK) & GPIO_PJ5 (CDI0) */
	const unsigned long GPIO_PORTJ_CCK      = GPIO_PORTJ_BASE + 0x40;    /* GPIO_PJ4 (CCK)     */
	const unsigned long GPIO_PORTJ_CDONE    = GPIO_PORTJ_BASE + 0x20;    /* GPIO_PJ3 (CDONE)   */
	const unsigned long GPIO_PORTJ_CRESET   = GPIO_PORTJ_BASE + 0x200;   /* GPIO_PJ7 (CRESETn) */
	
	const unsigned long TIMEOUT_CNT       = 1000000;                   /* 1000000us/2us*2sec = 1,000,000���[�v */
	const unsigned int  USER_CCK_CNT      = 200;                       /* 200���[�v */
	const unsigned char* conf_data        = ConfigDat;
	
	unsigned long timeout_cnt    = 0;
	unsigned long data_byte_size = 0;
	unsigned int  user_cycle     = 0;
	unsigned int  conf_try       = 0;
	unsigned int  ret            = 1;
	
	
	/* 3��A���Ń^�C���A�E�g�����ꍇ�ُ͈�Ƃ��� */
	for( conf_try=0; conf_try<3; conf_try++ )
	{
		timeout_cnt = 0;            /* �^�C���A�E�g�Ď� */
		conf_data   = ConfigDat;    /* �R���t�B�O�f�[�^ */
		ret         = 1;            /* ���� */
		
		HWREG(GPIO_PORTJ_CRESET) = 0;       /* GPIO_PJ7 (CRESETn) */
		delay(4);                           /* 400ns wait */
		HWREG(GPIO_PORTJ_CRESET) = 0x80;    /* GPIO_PJ7 (CRESETn) */		
		delay(12);    /* 1.2us wait */

		/* ����̂�CDONE == 0 �ɂȂ�܂ő҂� */
		if( conf_try == 0 )
		{
			while( HWREG(GPIO_PORTJ_CDONE) != 0 );
		}
		
		/* byte */
		for( data_byte_size=0; data_byte_size<sizeof(ConfigDat); data_byte_size++ )
		{
			/* �R���t�B�O�f�[�^���M */
			/* �������ԒZ�k�̂��߁A����HWREG���g�p��CCK low��CDI0 set�𓯎���write */
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
		
		/* �N���b�N��low�ɖ߂� */
		HWREG(GPIO_PORTJ_CCK) = 0;    /* GPIO_PJ4 (CCK) */
		
		
		/* CDONE == 1 �ɂȂ�܂ŃN���b�N����(�^�C���A�E�g�Ď�:2sec) */
		/* 8�N���b�N2.00us : 1sec -> 1000msec -> 1000000us/2us*2sec = 1,000,000���[�v */
		do
		{
			/* �������ԒZ�k�̂��߁A����HWREG���g�p����L�̂悤�ɏ�����1���[�v-8��s�� */
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
			/* ���[�U���[�h�J�ڗp�N���b�N���� */
			for( user_cycle=0; user_cycle<USER_CCK_CNT; user_cycle++ )
			{
				HWREG(GPIO_PORTJ_CCK) = 0x10;
				HWREG(GPIO_PORTJ_CCK) = 0;
			}
			
			/* �s�{�ӂ�CCK���o�Ȃ��悤�ɐ���INPUT�ɕύX���� */
			MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_4);
			
			ret = 1;    /* ���� */
			break;
		}
		else
		{
			ret = 0;    /* �^�C���A�E�g */
			continue;
		}
	
	}
	
	return ret;
}
#endif

/****************************************************/
/* Function : init                           */
/* Summary  : ����������    				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �N����                                 */
/****************************************************/
void init(void){

 short ch;
#if defined(FPGADOWNLOAD)
 short conf_ret;    /* 1=����, 0=�ُ�(�^�C���A�E�g) */
#endif

 /*�����݋֎~*/
	clrpsw_i();

	initializing = 1;		/*���������t���O*/

	/*���Z�b�g�����v���̊m�F*/
	reset_factor_check();

	/*CPU������*/
	cpu_init();

	/*EEPROM������*/
	eeprom_init();
	
#if defined(FPGADOWNLOAD)
	/* FPGA Config Data Send(SPI Passive Mode) */
	conf_ret = fpga_config_send();
	//���M���s���̓G���[�r�b�g�𗧂Ă�K�v����?
	//or LED�̓_���p�^�[���Œʒm?
	// if(conf_ret != 1){
	// 		__bit_output(GPIO_PORTB_BASE, 7, 1); //CPULED�_��
	// }
#endif

	/*CDONE�M���̑ҋ@(FPGA�N���ҋ@)*/
	WaitCDONE();

	/*RAM�N���A�̊m�F*/
	ram_clear_check();
	
	/*�������f�o�C�X�����ʒu�̔���*/
	OWCheckDeviceSide();

	/*�������f�o�C�X����Z���T����Ǎ���*/
	ReadSensDevice();

	/*�f�W�^���t�B���^�W�����W�X�^�̐ݒ�*/
	SetDigitalFilterRegister();
	
	/*�N���񐔂̍X�V*/
	pwon_count();

	/*�o�[�W�����Z�b�g*/
	version_set();

	/*�G���[���O�̏�����*/
	log_init();

	/*���S�x�e�[�u��������*/
	viscos_tbl_init();
	
	/*��g�g�`�f�[�^������*/
	recv_wave_init();

	/*RS485 or CUnet���ʊm�F(com_type����)*/
	mky43_ccr_check();

	/*MKY43�̏�����(CUnet�ʐM�J�n)*/
	if(com_type == COM_CUNET){
		mky43_init();
	}
	else{
		cpu_init_rs485();
	}
	
	/*LED������(�SLED����)*/
	disp_init();

	/*�ʐM������*/
	com_init();

	/*�������f�o�C�X���m�̊m�F*/
	CheckDeviceDetect();

	/*�E�H�b�`�h�b�O�^�C�}�J�n*/
	WatchdogStallEnable(WATCHDOG0_BASE);
	watchdog_refresh();
	WatchdogResetEnable(WATCHDOG0_BASE);

	// �f�o�b�K���쒆�̓E�H�b�`�h�b�O�Ń��Z�b�g�ł͂Ȃ����荞�݂���������悤�ɂ��� 
	// (�f�o�b�O���₷�����邽��) 
	// 0xe000edf0 �� Debug Halting Control and Status Register 
	if((*(unsigned long*)0xe000edf0 & 1) != 0)
	{ // C_DEBUGEN ON 
		WatchdogResetDisable(WATCHDOG0_BASE);
		WatchdogIntEnable(WATCHDOG0_BASE);
		IntEnable(INT_WATCHDOG);
	}

	/*�����ݗv������*/
	if(com_type == COM_CUNET){
		IntEnable(INT_GPIOP7);
		IntEnable(INT_GPIOQ7);
	}
	
	/*�����������*/
	TimerEnable(TIMER0_BASE, TIMER_A);		//����������݃X�^�[�g(3msec)
	TimerEnable(TIMER2_BASE, TIMER_A);		//����������݃X�^�[�g(5msec)

	initializing = 0;		/*���������t���O����*/

	/*�����݋���*/
	setpsw_i();
	
}

/****************************************************/
/* Function : cpu_init                        */
/* Summary  : CPU������    				*/
/* Argument : �Ȃ�                           */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �e���W�X�^�ݒ�                            */
/****************************************************/
void	cpu_init(void){

	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_12MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);

	// Enable processor interrupts.
	IntMasterEnable();

	PinoutSet();	/*�|�[�g�ݒ菉����*/

	__bit_output(GPIO_PORTA_BASE, 0, 1);	// GPIO_PA0 (CH6-REV)
	__bit_output(GPIO_PORTA_BASE, 1, 1);	// GPIO_PA1 (CH6-FWD)
	__bit_output(GPIO_PORTA_BASE, 2, 1);	// GPIO_PA2 (DISPLAY CLK)
	__bit_output(GPIO_PORTA_BASE, 3, 1);	// GPIO_PA3 (DISPLAY nCLR)
	__bit_output(GPIO_PORTA_BASE, 4, 1);	// GPIO_PA4 (DIPLAY DATA)
	__bit_output(GPIO_PORTA_BASE, 5, 1);	// GPIO_PA5 (DISP nSTB)
	
	__bit_output(GPIO_PORTB_BASE, 4, 0);	// GPIO_PB4 (EEPROM CS)
	__bit_output(GPIO_PORTB_BASE, 5, 0);	// GPIO_PB5 (EEPROM CLK)
	// TM4C �łł� LED2 �͎�������Ȃ��B PinoutSet �� Input �ɂ��Ă���B 
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

	__bit_output(GPIO_PORTM_BASE, 7, 0); //XRESET_FPGA	(FPGA������)

	// [TM4C] ALM_OUT0�AALM_OUT1�́A�\���̃s���ŁA���̓s���Ƃ��Đݒ� (QA17) 

	// �O���o�X�ݒ� 
	// 0x60000000(FIFO�ް�) �A�N�Z�X�� PN3 ���A�T�[�g����� CS0
	// 0x80000000(FPGAڼ޽�)�A�N�Z�X�� PB2 ���A�T�[�g����� CS1
	// 0xA0000000(CUnet���) �A�N�Z�X�� PN4 ���A�T�[�g����� 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);
	EPIModeSet(EPI0_BASE, EPI_MODE_HB16);
 EPIAddressMapSet(EPI0_BASE, EPI_ADDR_PER_SIZE_16MB | EPI_ADDR_RAM_SIZE_16MB | EPI_ADDR_QUAD_MODE);

	EPIConfigHB16Set(EPI0_BASE, EPI_HB16_MODE_ADDEMUX | EPI_HB16_CSBAUD | EPI_HB16_CSCFG_QUAD_CS, 1); 
	EPIDividerSet(EPI0_BASE, 0);		// CS0�p��EPI_CLK������ݒ� 1/1���� (CLK = 16ns) (�f�t�H���g)
	EPIDividerCSSet(EPI0_BASE, 0, 0);		// CS0�p��EPI_CLK������ݒ� 1/1���� (CLK = 16ns) (�f�t�H���g)
	EPIConfigHB16CSSet(EPI0_BASE, 0, EPI_HB16_MODE_ADDEMUX | EPI_HB16_WRWAIT_2 | EPI_HB16_RDWAIT_2);  // CS0�p WR,RD�E�F�C�g2 (16ns+(16ns*2wait)=48ns)
	EPIDividerCSSet(EPI0_BASE, 1, 0);		// CS1�p��EPI_CLK������ݒ� 1/1���� (CLK = 16ns) (�f�t�H���g)
	EPIConfigHB16CSSet(EPI0_BASE, 1, EPI_HB16_MODE_ADDEMUX | EPI_HB16_WRWAIT_2 | EPI_HB16_RDWAIT_3);  // CS1�p WR,RD�E�F�C�g2 (16ns+(16ns*2wait)=48ns)
	EPIDividerCSSet(EPI0_BASE, 2, 1);		// CS2�p��EPI_CLK������ݒ� 1/2���� (CLK = 32ns)
	EPIConfigHB16CSSet(EPI0_BASE, 2, EPI_HB16_MODE_ADDEMUX | EPI_HB16_WRWAIT_2 | EPI_HB16_RDWAIT_2);  // CS2�p WR,RD�E�F�C�g2 (32ns+(32ns*2wait)=98ns)
	
	// �����������(3msec)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / 1000) * 3);
//	TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / 1000) * 6); //6ms�����݂ɕύX
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
	IntEnable(INT_TIMER4A);
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	//��M�f�[�^��M���������� RS485(�����e�i���X)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// �����������(5msec)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER2_BASE, TIMER_A, (g_ui32SysClock / 1000) * 5);
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	//��M�f�[�^��M���������� RS485/CUnet(�z�X�g)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
	IntEnable(INT_TIMER3A);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	
	// �����ݗD��x�̕ύX�i�������������قǗD��x�������j(���� 5 �r�b�g�� reserved �Ŗ��������) 
	IntPrioritySet(INT_GPIOP7, 4 << 5);		// CUnet���荞�݁i���[����M(MRB0)�j
	IntPrioritySet(INT_GPIOQ7, 1 << 5);		// CUnet���荞�݁i���[����M(MRB1)�j
	IntPrioritySet(INT_GPIOP6, 2 << 5);					// FIFO�f�[�^��M���荞��
	IntPrioritySet(INT_UART_COM_MENT, 1 << 5);		// ��M���荞�݁i�����e�i���X�|�[�g�j
	IntPrioritySet(INT_TIMER0A, 3 << 5);		// ��������荞��(���菈���p)
	IntPrioritySet(INT_TIMER1A, 6 << 5);		// ��M�������荞�݁i�����e�i���X�|�[�g�j
	IntPrioritySet(INT_TIMER2A, 2 << 5);		// ��������荞��(�\��������p)
	IntPrioritySet(INT_TIMER3A, 6 << 5);		// ��M�������荞�݁i�z�X�g�|�[�g�j
	IntPrioritySet(INT_TIMER4A, 6 << 5);		// ��M�������荞�݁i�T�u�z�X�g�|�[�g�j

//	GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
//	GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_6);	// GPIO_PP6 

	// MKY43 #INT0 
	// ���_���̐M���Ȃ̂ŗ���������G�b�W�Ŋ��荞�� 
	GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_7);	// GPIO_PP7 

	// MKY43 #INT1 
	// ���_���̐M���Ȃ̂ŗ���������G�b�W�Ŋ��荞�� 
	GPIOIntTypeSet(GPIO_PORTQ_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTQ_BASE, GPIO_PIN_7);	// GPIO_PQ7 

	// DMA ���ʕ��������� 
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
/* Summary  : CPU������(RS485)   				*/
/* Argument : �Ȃ�                            */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �e���W�X�^�ݒ�                            */
/****************************************************/
void	cpu_init_rs485(void){

	// GPIO_PK4
	__bit_output(GPIO_PORTK_BASE, 4, 0);				//MKY43 RST

	// MKY43 �p CS �� High �Œ�o�� 
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);	// GPIO_PN4 
	__bit_output(GPIO_PORTN_BASE, 4, 1);

	// �����ݗD��x�̕ύX�i�������������قǗD��x�������j(���� 5 �r�b�g�� reserved �Ŗ��������) 
	IntPrioritySet(INT_UART_COM_HOST, 0 << 5);		// ��M���荞�݁i�z�X�g�|�[�g�j

}

/****************************************************/
/* Function : eeprom_init                           */
/* Summary  : EEPROM������    				*/
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	eeprom_init(void){

	short data;
	short ch = 0;
	char work_ch[20];

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		data =eep_read((short)WRIT_CHECK + SIZEOFMODBUSADDR * ch);
		read_serial_num(ch);					/*�V���A���ԍ��Ǎ���*/

		if(data != (short)EEPROM_WRITE_FLAG){	/*�����݃t���O�̃`�F�b�N*/	
			drive_led(DSP_CH_ALL);	//CH-LED�S��
			eep_sv_write(ch);					/*�����l������*/
			drive_led(DSP_ALM_ALL);	//ALM-LED�S��
			eep_write_ch(ch, (short)WRIT_CHECK, (short)EEPROM_WRITE_FLAG);	/*�����݃t���O�X�V*/
			write_serial_num(ch);				/*�V���A���ԍ�������*/
		}
		eep_sv_read(ch);	/*EEPROM�̓Ǎ���*/
		
		memcpy(&work_ch[0], &SVD[ch].ZerCrsOffset[0], sizeof(SVD[ch].ZerCrsOffset));
		MES[ch].zc_zero_offset = atof(&work_ch[0]);	//�[���N���X�[���_�I�t�Z�b�g
		MES[ch].signal_count = MES[ch].ThresholdPeakPos = SVD[ch].ThresholdPeakPos;
		MES[ch].zc_peak = SVD[ch].ZerPeakPos;
	}
}

/****************************************************/
/* Function : version_set                           */
/* Summary  : �o�[�W�����Z�b�g    				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	version_set(void){

	short ch;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		/*�\�t�g�E�F�A�o�[�W����*/
		SVD[ch].soft_ver = SOFT_VERSION;

		/*�n�[�h�E�F�A�o�[�W����*/
		SVD[ch].hard_ver = HARD_VERSION;
	}

	/*FPGA�o�[�W����*/
	FpgaVersion = FPGA_VERSION;
}

/****************************************************/
/* Function : viscos_tbl_init                       */
/* Summary  : ���S�x�e�[�u��������    				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	viscos_tbl_init(void){

	short ch = 0;

	/*�������j�A���C�Y�e�[�u���쐬*/
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		make_viscos_tbl(ch);
	}
}

/****************************************************/
/* Function : recv_wave_init                       */
/* Summary  : ��g�i�[�̈�̏�����    				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	recv_wave_init(void){

	short ch = 0;
	short i_cnt;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
  MES[ch].clk_phase = 0;
  
  	//�]���p
	if(SVD[ch].sum_step == 2){	//�ō��݉񐔏㗬�����e2��
		MES_SUB[ch].sample_cnt = 2;
	}else if(SVD[ch].sum_step == 3){	//�ō��݉񐔏㗬�����e3��
		MES_SUB[ch].sample_cnt = 3;
	}else if(SVD[ch].sum_step == 4){	//�ō��݉񐔏㗬�����e4��
		MES_SUB[ch].sample_cnt = 4;
	}else{	//�ō��݉񐔏㗬�����e4��
		MES_SUB[ch].sample_cnt = 4;
	}
	AD_BASE = AD_BASE_UNIT * MES_SUB[ch].sample_cnt;
	AD_MAX = AD_MAX_UNIT * MES_SUB[ch].sample_cnt;
	//�]���p

  
  
 	/*��g�i�[�̈�̏�����*/
		for(i_cnt=0; i_cnt<300; i_cnt++)
		{
			MES[ch].fow_data[i_cnt] = MES[ch].rev_data[i_cnt] = AD_BASE;/*0V*/
		}
	
		/*�A���v�Q�C���̒����A����сA�M���ُ�`�F�b�N*/
		gain_adj_init(ch);
			
		/*�����̉����i���j*/
		MES[ch].sound_vel = SVD[ch].sound_vel_fix;

		/*delta_ts0*/
		MES[ch].delta_ts0 = MES[ch].delta_ts = (unsigned short)SVD[ch].zero_offset;
	}
}
	
/****************************************************/
/* Function : counter_control                       */
/* Summary  : �J�E���^����    				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	counter_control(void){

	short ch = 0;

	/*�[���������J�E���^*/
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
	
	/*100msec�������C���J�E���^*/
	if(main_count > 600){			//1���o��
		main_count = 0;				//�N���A
	}else{
		main_count++;				//�X�V
	}
}

/****************************************************/
/* Function : zero_alm_check                       */
/* Summary  : �[���_����/�A���[�����Z�b�g�m�F          				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 100ms��1�x���s�����
 ****************************************************/
void	zero_alm_check(void){

	short ch;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		zero_adj_control(ch);			//�SCH���[���_����
		alm_reset_control(ch);			//�SCH���A���[�����Z�b�g
	}
	sw_now_zeroadj = B_OFF;				//�[���_�����v������
}

/****************************************************/
/* Function : zero_check_sub                       */
/* Summary  : �[���_�����l���R�_�̂΂�����A�K��l�ȉ��ł��邱�Ƃ��m�F����	*/
/* Argument : d[]                                  */
/* Return   : 1=����, 0=�ُ�                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
//#define DIFF_LIMIT	(1)
//#define DIFF_LIMIT	(5)
//#define DIFF_LIMIT	(15)
#define DIFF_LIMIT	(60)  /*4����Z��(15*4)*/
short	zero_check_sub(short d[]) {

	if( (abs(d[0] - d[1]) > DIFF_LIMIT) ||
		(abs(d[1] - d[2]) > DIFF_LIMIT) ||
		(abs(d[2] - d[0]) > DIFF_LIMIT) ) {
		return 0;
	}
	return 1;
}
// #define FLOAT_DIFF_LIMIT (0.0001) //���ԍ���10^-6�`10^-4�̃I�[�_�[
#define FLOAT_DIFF_LIMIT (0.015) //3��~0.15(@1/4"), 0.002(@3/8")
short	zero_check_sub_f(float d[]) {
	//abs�̖߂�l��int
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
 * Summary  : �[���_�������̃G���[�𔻒肷��
 * Argument : pch : �`�����l���ԍ�
 *            Phs : �[���_�����̃t�F�[�Y
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : Vth���������҂��ƃ[���_���������������̃G���[�̂�err_status���X�V����
 *          : -> ���R�͕s��
 ***************************************************************************/
void JdgZadErr(short pch, short Phs)
{
	/*�[���������s*/
	zero_adj_error(pch);		/*�[�������G���[����*/
	if(Phs == 2 || Phs == 4)
	{
		MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*�[�������v�����Ԕ��U�G���[�Z�b�g*/
		MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
		err_judge_status(pch);
	}
	zero_adj_status(pch);		/*�[���_�������̔g�`�f�[�^��ێ�����*/
	
}

/****************************************************************************
 * Function : LedCtlZaj (LED Control for Zeroadjust)
 * Summary  : �[���_��������LED����
 * Argument : pch : �`�����l���ԍ�
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : 100ms�Ɉ�x���s�����
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
 * Summary  : Vth�����I������
 * Argument : pch : �`�����l���ԍ�
 * Return   : Flg : 0x00  -> ����
 *                  0x01  -> �t�F�[�Y�X�V
 *                  0x100 -> �G���[
 * Caution  : �Ȃ�
 * Note     : Phase2
 *            �g�p����ϐ�    �����l  ����
 *            vth_do_cnt     20      100ms�Ɉ�x
 *            vth_count      0       100ms�Ɉ�x
 *            zero_retry_cnt 0       100ms�Ɉ�x
 *            vth_sum        0       3ms x 6 = 18ms�Ɉ�x
 ***************************************************************************/
short EndAdjVth(short pch)
{
	short Flg = 0;
	short wave_vth_buf;
	/*Vth�������i2�b�ԁj��Vth���Z�o*/
	wave_vth_buf = (short)(((long long)MES[pch].vth_sum * 25) / ((long long)MES[pch].vth_count * AD_MAX));
	MES[pch].ComVthSum = MES[pch].vth_sum;
	MES[pch].ComVthCount = MES[pch].vth_count;

	LED[pch].zero_vth_buf[LED[pch].zero_retry_cnt] = wave_vth_buf;
	/*Vth�f�[�^�̎����m�F*/
	if( (SVD[pch].wave_vth == wave_vth_buf) ||
	    ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_vth_buf[0])) ) {
		/*Vth��������*/
		MES[pch].vth_count = 0;
		SVD[pch].wave_vth = wave_vth_buf;
		eep_write_ch_delay(pch, (short)(&SVD[pch].wave_vth - &SVD[pch].max_flow), SVD[pch].wave_vth);	//�G���v�e�B�Z���T����臒l
		Flg = 0x01;
	}else{
		if(LED[pch].zero_retry_cnt >= 2){			/*Vth���������Ȃ��ꍇ*/
			/*Vth�������s*/
			Flg = 0x100;
		}else{
			/*Vth�������g���C*/
			Flg = 0x02;
		}
	}
	return Flg;
}

/****************************************************************************
 * Function : NowZerAdj (Now Zero Adjust)
 * Summary  : �[���_����������
 * Argument : pch : �`�����l���ԍ�
 * Return   : Flg : 0x00  -> ����
 *                  0x01  -> �t�F�[�Y�X�V
 *                  0x100 -> �G���[
 * Caution  : �Ȃ�
 * Note     : Phase3
 *            �g�p����ϐ�    �����l  ����
 *            zero_do_cnt    0       100ms�Ɉ�x
 *            zero_cal_cnt   0       100ms�Ɉ�x
 *            zero_delta_ts  0       100ms�Ɉ�x
 *            zc_zero_calc   0       100ms�Ɉ�x
 ***************************************************************************/
short NowZerAdj(short pch)
{
	short Flg = 0;
	/*�[���_������*/
	if(LED[pch].zero_do_cnt > 0){
		/*�[�������p��Ts�̉��Z*/
		LED[pch].zero_cal_cnt ++;
		LED[pch].zero_delta_ts += (unsigned long)MES[pch].delta_ts;
		MES[pch].zc_zero_calc += MES[pch].zc_Tdata;  //�[���N���X
	}
	else {
		Flg = 0x01;
	}
	return Flg;
}

/****************************************************************************
 * Function : SavZajPrm (Save Zero adjust Parameter)
 * Summary  : �[���_�����������̃p�����[�^�ۑ�
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : �Ȃ�
 * Note     : MES[pch].zc_zero_offset : �[���N���X�p�[���I�t�Z�b�g�_
 ***************************************************************************/
void SavZajPrm(short pch)
{
	char work_ch[14];
	short cnt;

	/*�[�������p��Ts�̕��ϒl(�l�̌ܓ�)*/
	memset(work_ch, 0, sizeof(work_ch));
	MES[pch].zc_zero_offset = MES[pch].zc_zero_calc / LED[pch].zero_cal_cnt;  //�[���N���X���̃[���_�I�t�Z�b�g
	sprintf(&work_ch[0], "%3.11f", MES[pch].zc_zero_offset);
	memcpy(&SVD[pch].ZerCrsOffset[0], &work_ch[0], sizeof(work_ch));
	for(cnt=0; cnt<7; cnt++){  //�[���N���X���̃[���_�I�t�Z�b�g��EEPROM�ۑ�
		eep_write_ch_delay(pch, (short)(&SVD[pch].ZerCrsOffset[cnt] - &SVD[pch].max_flow), SVD[pch].ZerCrsOffset[cnt]);
	}
#if defined(WAVE_RECOGNITION)
	SVD[pch].ThresholdPeakPos = MES[pch].ThresholdPeakPos;	//��g�g�`���o�������l��EEPROM�ۑ�
	eep_write_ch_delay(pch, (short)(&SVD[pch].ThresholdPeakPos - &SVD[pch].max_flow), SVD[pch].ThresholdPeakPos);
	SVD[pch].fifo_ch_init = MES[pch].ws_FifoCh;		//FIFO CH��EEPROM�ۑ�
	eep_write_ch_delay(pch, (short)(&SVD[pch].fifo_ch_init - &SVD[pch].max_flow), SVD[pch].fifo_ch_init);
#else
#endif

	//Debug
	/*�[���������̃Q�C���l��ێ�*/
	MES[pch].zero_amp_gain_rev = MES[pch].amp_gain_rev;
	/*�[����������FIFO�ʒu��ێ�*/
	MES[pch].zero_fifo_ch_read = MES[pch].fifo_ch_read;
	/*�[����������FIFO��g�g�`���o�ʒu��ێ�*/
	MES[pch].zero_signal_count = MES[pch].signal_count;
	MES[pch].zero_fifo_no_read = MES[pch].fifo_no_read;
	/*�[���������̔g�`�擪�ʒu��ێ�*/
	MES[pch].zero_sonic_point_rev_p2 = MES[pch].sonic_point_rev_p2;
	MES[pch].zero_sonic_point_fow_p1 = MES[pch].sonic_point_fow_p1;
	//Debug

	if(MES[pch].zc_peak_UpdateFlg != 0)
	{
		MES[pch].zc_peak = 0;
	}
	MES_SUB[pch].zc_peak_req = 1;	//�g�`�F��臒l�t�߂̃s�[�N�ʒu�������v��	
}

/****************************************************************************
 * Function : EndZerAdj (End Zero Adjust)
 * Summary  : �[���_�I��������
 * Argument : pch : �`�����l���ԍ�
 * Return   : Flg : 0x00  -> ����
 *                  0x01  -> �t�F�[�Y�X�V
 *                  0x100 -> �G���[
 * Caution  : �Ȃ�
 * Note     : Phase4
 *            
 ***************************************************************************/
#if 1 //�[���N���X�p
short EndZerAdj(short pch)
{
	short Flg = 0;
	char work_ch[14];
	float zc_work_delta_ts;
	float SvdWork;
	/*�[�������I������*/
	// if(LED[pch].zero_do_cnt == 0 && LED[pch].zero_cal_cnt != 0){
	if(LED[pch].zero_cal_cnt != 0)
	{
		//���݂̃�Ts�̌v�Z
		zc_work_delta_ts = MES[pch].zc_zero_calc / LED[pch].zero_cal_cnt;
		LED[pch].zero_dt_buf_f[LED[pch].zero_retry_cnt] = zc_work_delta_ts;
		
		//�O��̃�Ts�̌v�Z
		memcpy(&work_ch[0], &SVD[pch].ZerCrsOffset[0], sizeof(SVD[pch].ZerCrsOffset));
		SvdWork = atof(&work_ch[0]);

		/*��Ts�f�[�^�̎����m�F*/
		if((SvdWork == zc_work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub_f(&LED[pch].zero_dt_buf_f[0])) ) {
			SavZajPrm(pch);
			Flg = 0x01;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*��Ts���������Ȃ��ꍇ*/
				/*�[���������s*/
				Flg = 0x100;
			}else{
				/*�[���������g���C*/
				//��Ts�X�V
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
#else //�������֗p
short EndZerAdj(short pch)
{
	short Flg = 0;
	char work_ch[14];
	unsigned short work_delta_ts;
	short cnt;
	/*�[�������I������*/
	// if(LED[pch].zero_do_cnt == 0 && LED[pch].zero_cal_cnt != 0){
	if(LED[pch].zero_cal_cnt != 0)
	{
		work_delta_ts = (unsigned short)((LED[pch].zero_delta_ts * 2 + 1) / (LED[pch].zero_cal_cnt * 2));
		LED[pch].zero_dt_buf[LED[pch].zero_retry_cnt] = (short)work_delta_ts;
		SavZajPrm(pch); //�p�����[�^�ۑ�

		if(MES[pch].zc_peak_UpdateFlg != 0)
		{
			MES[pch].zc_peak = 0;
		}
		MES_SUB[pch].zc_peak_req = 1;	//�g�`�F��臒l�t�߂̃s�[�N�ʒu�������v��
		
		/*��Ts�f�[�^�̎����m�F*/
		if((SVD[pch].zero_offset == work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_dt_buf[0])) ) {
			/*�[����������*/
			SVD[pch].zero_offset = work_delta_ts;
			eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)work_delta_ts);
			Flg = 0x01;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*��Ts���������Ȃ��ꍇ*/
				/*�[���������s*/
				Flg = 0x100;
			}else{
				/*�[���������g���C*/
				SVD[pch].zero_offset = work_delta_ts;	/*��Ts�X�V*/
				Flg = 0x02;
			}
		}
		work_delta_ts = (unsigned short)((LED[pch].zero_delta_ts * 2 + 1) / (LED[pch].zero_cal_cnt * 2));
		LED[pch].zero_dt_buf[LED[pch].zero_retry_cnt] = (short)work_delta_ts;
		SavZajPrm(pch); //�p�����[�^�ۑ�

		if(MES[pch].zc_peak_UpdateFlg != 0)
		{
			MES[pch].zc_peak = 0;
		}
		MES_SUB[pch].zc_peak_req = 1;	//�g�`�F��臒l�t�߂̃s�[�N�ʒu�������v��
		
		/*��Ts�f�[�^�̎����m�F*/
		if((SVD[pch].zero_offset == work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_dt_buf[0])) ) {
			/*�[����������*/
			SVD[pch].zero_offset = work_delta_ts;
			eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)work_delta_ts);
			Flg = 0x01;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*��Ts���������Ȃ��ꍇ*/
				/*�[���������s*/
				Flg = 0x100;
			}else{
				/*�[���������g���C*/
				SVD[pch].zero_offset = work_delta_ts;	/*��Ts�X�V*/
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
#endif //�[���N���X�p

/****************************************************************************
 * Function : IniZajPrm (Init Zeroadjust Parameter)
 * Summary  : �[���_�����e�t�F�[�Y���Ƃ̃p�����[�^������
 * Argument : pch : �`�����l���ԍ�
 *            Phs : �t�F�[�Y�ԍ�
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : 
 ***************************************************************************/
void IniZajPrm(short pch, short Phs)
{
	switch(Phs)
	{
		//�g�`�F���I���҂�
		case 1:
			LED[pch].wave_do_cnt = 0; //�g�`�F�����ԃJ�E���^
			break;
		//Vth������
		case 2:
			LED[pch].vth_do_cnt = VTH_ADJ_TIME; //vth�������ԃJ�E���^
			MES[pch].vth_count = 0; //vth���Z��
			MES[pch].vth_sum = 0; //vth���v
			MES[pch].ComVthCount = 0;
			MES[pch].ComVthSum = 0;
			memset(LED[pch].zero_vth_buf, 0, sizeof(LED[pch].zero_vth_buf)); //���g���C�p�o�b�t�@
			LED[pch].zero_retry_cnt = 0; //vth���g���C��
			break;
		//Vth�������g���C
		case 12:
			LED[pch].vth_do_cnt = VTH_ADJ_TIME;
			MES[pch].vth_count = 0;
			MES[pch].vth_sum = 0;
			// memset(LED[pch].zero_vth_buf, 0, sizeof(LED[pch].zero_vth_buf));
			LED[pch].zero_retry_cnt++;
			break;
		//���ԍ�������
		case 3:
			LED[pch].zero_retry_cnt = 0;
			memset(LED[pch].zero_dt_buf, 0, sizeof(LED[pch].zero_dt_buf)); //�������փ��g���C�p�o�b�t�@
			memset(LED[pch].zero_dt_buf_f, 0, sizeof(LED[pch].zero_dt_buf_f)); //�[���N���X���g���C�p�o�b�t�@
			LED[pch].zero_do_cnt = ZERO_ADJ_TIME; //dt��������
			LED[pch].zero_cal_cnt = 0; //dt���Z��
			LED[pch].zero_delta_ts = 0; //�t�B���^��dt(�㉺�����ԍ�)���v
			MES[pch].zc_zero_calc = 0; //dt(�㉺�����ԍ�)���v
			if(reset_factor != RESTART_WDOG){				//�ċN���G���[�ȊO
				MAIN[pch].com_err_status = (short)0;			/*�G���[�X�e�[�^�X�N���A*/
			}
			break;
		//���ԍ��������g���C
		case 13:
			LED[pch].zero_retry_cnt++;
			// memset(LED[pch].zero_dt_buf, 0, sizeof(LED[pch].zero_dt_buf));
			LED[pch].zero_do_cnt = ZERO_ADJ_TIME;		/*�[���_�������ԃZ�b�g*/
			LED[pch].zero_cal_cnt = 0;
			LED[pch].zero_delta_ts = 0;
			MES[pch].zc_zero_calc = 0;
			break;
		//���ԍ������I��
		case 4:
			break;
		//�[���_��������I��
		case 5:
			/*�[�������p��Ts�̏�����*/
			LED[pch].zero_cal_cnt = 0;
			LED[pch].zero_delta_ts = 0;
			MES[pch].zc_zero_calc = 0;  //�[���N���X
			MES[pch].err_status &= ~ERR_JUDGE_ZERO;	/*�[���_������ԃ��Z�b�g*/
			if(reset_factor != RESTART_WDOG){				//�ċN���G���[�ȊO
				MAIN[pch].com_err_status = (short)0;			/*�G���[�X�e�[�^�X�N���A*/
				MAIN[pch].led_err_status = (short)0;			/*�G���[�X�e�[�^�X�N���A*/
			}
			LED[pch].zero_active = 0;
			action_status_control(pch, ACT_STS_NORMAL);	/*����X�e�[�^�X�X�V*/
			zero_adj_status(pch);						/*�[���_�������̔g�`�f�[�^��ێ�����*/
			break;
		default:
			break;
	}
}

/****************************************************************************
 * Function : JdgZerAdjPhs (Judge Zero Adjust Phase)
 * Summary  : �[���_�����t�F�[�Y����
 * Argument : pch : �`�����l���ԍ�
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : Flg : 0x00  -> ����A�t�F�[�Y�X�V�Ȃ�
 *                  0x01  -> ����A�t�F�[�Y�X�V����
 *                  0x02  -> �ُ�A���g���C
 *                  0x100 -> �ُ�A�[���_�����I��
 ***************************************************************************/
void JdgZerAdjPhs(short pch)
{
	short Flg = 0;

	//LED����
	LedCtlZaj(pch);

	//�[���_�����G���[
	if((LED[pch].zero_active != 0) && ((MES[pch].err_status & ERR_ZERO_ADJ) != 0))
	{
		Flg |= 0x100;
	}

	switch (MES[pch].ZerAdjPhs)
	{
		//�ʏ푪�蒆
		case 0:
			if(LED[pch].zero_active != 0)
			{
				MES[pch].ZerAdjPhs = 1;
				IniZajPrm(pch, 1); //Phase1�p������
			}
			break;
		//�g�`�F���I���҂�
		case 1:
#if defined(WAVE_RECOGNITION)
			if(MES[pch].ThresholdReq == 99)
			{
				MES[pch].ZerAdjPhs = 2;
				IniZajPrm(pch, 2); //Phase2�p������
			}
#else
			MES[pch].ZerAdjPhs = 2;
			IniZajPrm(pch, 2); //Phase2�p������
#endif
			break;
		//Vth�����҂�
		case 2:
			//�t�F�[�Y�ύX����2s�o��
			if(LED[pch].vth_do_cnt == 0)
			{
				//2s�Ԃ�1�x��vth���擾
				if(MES[pch].vth_count != 0)
				{
					Flg |= EndAdjVth(pch);
				}
			}
			//Vth�������m�F(�[�����G���[�Ȃ�&EndAdjVth()������I��)
			if(Flg == 0x01)
			{
				MES[pch].ZerAdjPhs = 3;
				IniZajPrm(pch, 3); //Phase3�p������
			}
			//Vth�������g���C(�[�����G���[�Ȃ�&EndAdjVth()�����g���C)
			else if(Flg == 0x02)
			{
				MES[pch].ZerAdjPhs = 2; //�ϐ��������������ă��g���C
				IniZajPrm(pch, 12); //Phase2���g���C�p������
			}
			break;
		//�[���_������
		case 3:
			Flg |= NowZerAdj(pch);
			//zero_do_cnt���҂�����(�[�����G���[�Ȃ�&NowZerAdj()������I��)
			if(Flg == 0x01)
			{
				MES[pch].ZerAdjPhs = 4;
				IniZajPrm(pch, 4); //Phase4�p������
			}
			break;
		//�[���_������������
		case 4:
			Flg |= EndZerAdj(pch);
			//�[���_��������(�[�����G���[�Ȃ�&EndZerAdj()������I��)
			if(Flg == 0x01)
			{
				MES[pch].ZerAdjPhs = 0;
				IniZajPrm(pch, 5); //�[���_�����I�����p������
			}
			//Vth�̒������烊�g���C(�[�����G���[�Ȃ�&EndZerAdj()�����g���C)
			else if(Flg == 0x02)
			{
				MES[pch].ZerAdjPhs = 3;
				IniZajPrm(pch, 13); //Phase3���g���C�p������
			}
			break;
		//�G���[����
		default:
			MES[pch].ZerAdjPhs = 0;
			break;
	}

	//�[���_�����G���[����
	if((Flg & 0x100) != 0)
	{
		JdgZadErr(pch, MES[pch].ZerAdjPhs);
		MES[pch].ZerAdjPhs = 0;
	}
}

/****************************************************/
/* Function : zero_adj_control                     */
/* Summary  : �[���_��������         				*/
/* Argument : pch                                 */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : Vth������Ƀ[���_���������{����             */
/****************************************************/
void	zero_adj_control(short pch){
#if 1
	JdgZerAdjPhs(pch);
#else
	unsigned short work_delta_ts;
	short wave_vth_buf;
	short cnt;
	char work_ch[14];

	/*Vth������*/
	if(LED[pch].vth_do_cnt != 0){
		if((MES[pch].err_status & ERR_ZERO_ADJ) == 0){
			if((LED[pch].vth_do_cnt % 5) == 0){ 		//500msec����
				led_channel ^= CH_LED[pch];
			}
		}else{							/*Vth�������̃G���[*/
			/*Vth�������s*/
			zero_adj_error(pch);		/*�[�������G���[����*/
			zero_adj_status(pch);						/*�[���_�������̔g�`�f�[�^��ێ�����*/
		}
	}
	/*�g�`�F��������(CH-LED��_��)*/
	if(LED[pch].wave_do_cnt != 0){
		if((LED[pch].wave_do_cnt % 5) == 0){ 		//500msec����
			led_channel ^= CH_LED[pch];
		}
	}

	/*�g�`�F���I������*/
	if(LED[pch].zero_active == 1 && MES[pch].ThresholdReq == 99){			/*�g�`�F���I��*/
			MES[pch].ThresholdReq = 0;  /*�g�`�F�������̏I��*/
			LED[pch].wave_do_cnt = 0;	/*�g�`�F���������ԃN���A*/
		/*Vth�����J�n*/
			LED[pch].vth_do_cnt = VTH_ADJ_TIME;		/*Vth�������ԃZ�b�g*/
	}
	
	/*Vth�����I������*/
	if(LED[pch].vth_do_cnt == 0 && MES[pch].vth_count != 0){
		/*Vth�������i2�b�ԁj��Vth���Z�o*/
		wave_vth_buf = (short)(((long long)MES[pch].vth_sum * 25) / ((long long)MES[pch].vth_count * AD_MAX));

		LED[pch].zero_vth_buf[LED[pch].zero_retry_cnt] = wave_vth_buf;
		/*Vth�f�[�^�̎����m�F*/
		if( (SVD[pch].wave_vth == wave_vth_buf) ||
		    ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_vth_buf[0])) ) {
			/*Vth��������*/
			MES[pch].vth_count = 0;
			SVD[pch].wave_vth = wave_vth_buf;
			eep_write_ch_delay(pch, (short)(&SVD[pch].wave_vth - &SVD[pch].max_flow), SVD[pch].wave_vth);	//�G���v�e�B�Z���T����臒l

			/*�[�������J�n*/
			if(reset_factor != RESTART_WDOG){				//�ċN���G���[�ȊO
				MAIN[pch].com_err_status = (short)0;			/*�G���[�X�e�[�^�X�N���A*/
			}
			LED[pch].zero_retry_cnt = 0;				/*�[���������g���C�J�E���^������*/
			LED[pch].zero_do_cnt = ZERO_ADJ_TIME;		/*�[���_�������ԃZ�b�g*/
			LED[pch].zero_dt_buf[0] = LED[pch].zero_dt_buf[1] = LED[pch].zero_dt_buf[2] = 0;
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*Vth���������Ȃ��ꍇ*/
				/*Vth�������s*/
				zero_adj_error(pch);					/*�[�������G���[����*/
				MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*�[�������v�����Ԕ��U�G���[�Z�b�g*/
				MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
				err_judge_status(pch);
				zero_adj_status(pch);						/*�[���_�������̔g�`�f�[�^��ێ�����*/
			}else{
				/*Vth�������g���C*/
				SVD[pch].wave_vth = wave_vth_buf;		/*Vth�X�V*/
				LED[pch].zero_retry_cnt ++;			/*�[���������g���C�J�E���^�X�V*/
				MES[pch].vth_sum = 0;
				MES[pch].vth_count = 0;
				LED[pch].vth_do_cnt = VTH_ADJ_TIME;	/*Vth�������ԃZ�b�g*/
			}
		}
	}	
	
	/*�[���_������*/
	if(LED[pch].zero_do_cnt != 0){
		if((MES[pch].err_status & ERR_ZERO_ADJ) == 0){
			/*�[�������p��Ts�̉��Z*/
			LED[pch].zero_cal_cnt ++;
			LED[pch].zero_delta_ts += (unsigned long)MES[pch].delta_ts;
			MES[pch].zc_zero_calc += MES[pch].zc_Tdata;  //�[���N���X
			if((LED[pch].zero_do_cnt % 5) == 0){ 	//500msec����
				led_channel ^= CH_LED[pch];
			}
		}else{							/*zero�������̃G���[*/
			/*�[���������s*/
			zero_adj_error(pch);		/*�[�������G���[����*/
			zero_adj_status(pch);						/*�[���_�������̔g�`�f�[�^��ێ�����*/
		}
	}
	
	/*�[�������I������*/
	if(LED[pch].zero_do_cnt == 0 && LED[pch].zero_cal_cnt != 0){
		/*�[�������p��Ts�̕��ϒl(�l�̌ܓ�)*/
		memset(work_ch, 0, sizeof(work_ch));
		work_delta_ts = (unsigned short)((LED[pch].zero_delta_ts * 2 + 1) / (LED[pch].zero_cal_cnt * 2));
		MES[pch].zc_zero_offset = MES[pch].zc_zero_calc / LED[pch].zero_cal_cnt;  //�[���N���X���̃[���_�I�t�Z�b�g
		LED[pch].zero_dt_buf[LED[pch].zero_retry_cnt] = (short)work_delta_ts;
		sprintf(&work_ch[0], "%3.11f", MES[pch].zc_zero_offset);
		memcpy(&SVD[pch].ZerCrsOffset[0], &work_ch[0], sizeof(work_ch));
		for(cnt=0; cnt<7; cnt++){  //�[���N���X���̃[���_�I�t�Z�b�g��EEPROM�ۑ�
			eep_write_ch_delay(pch, (short)(&SVD[pch].ZerCrsOffset[cnt] - &SVD[pch].max_flow), SVD[pch].ZerCrsOffset[cnt]);
		}
		SVD[pch].ThresholdPeakPos = MES[pch].ThresholdPeakPos;	//��g�g�`���o�������l��EEPROM�ۑ�
		eep_write_ch_delay(pch, (short)(&SVD[pch].ThresholdPeakPos - &SVD[pch].max_flow), SVD[pch].ThresholdPeakPos);
		SVD[pch].fifo_ch_init = MES[pch].ws_FifoCh;		//FIFO CH��EEPROM�ۑ�
		eep_write_ch_delay(pch, (short)(&SVD[pch].fifo_ch_init - &SVD[pch].max_flow), SVD[pch].fifo_ch_init);
		
		//Debug
		/*�[���������̃Q�C���l��ێ�*/
		MES[pch].zero_amp_gain_rev = MES[pch].amp_gain_rev;
		/*�[����������FIFO�ʒu��ێ�*/
		MES[pch].zero_fifo_ch_read = MES[pch].fifo_ch_read;
		/*�[����������FIFO��g�g�`���o�ʒu��ێ�*/
		MES[pch].zero_signal_count = MES[pch].signal_count;
		MES[pch].zero_fifo_no_read = MES[pch].fifo_no_read;
		/*�[���������̔g�`�擪�ʒu��ێ�*/
		MES[pch].zero_sonic_point_rev_p2 = MES[pch].sonic_point_rev_p2;
		MES[pch].zero_sonic_point_fow_p1 = MES[pch].sonic_point_fow_p1;
		//Debug
		
		if(MES[pch].zc_peak_UpdateFlg != 0)
		{
			MES[pch].zc_peak = 0;
		}
		MES_SUB[pch].zc_peak_req = 1;	//�g�`�F��臒l�t�߂̃s�[�N�ʒu�������v��
		
		/*��Ts�f�[�^�̎����m�F*/
		if((SVD[pch].zero_offset == work_delta_ts) ||
		   ((LED[pch].zero_retry_cnt >= 2) && zero_check_sub(&LED[pch].zero_dt_buf[0])) ) {
			/*�[����������*/
			SVD[pch].zero_offset = work_delta_ts;
			eep_write_ch_delay(pch, (short)(&SVD[pch].zero_offset - &SVD[pch].max_flow), (short)work_delta_ts);
			
		
			/*�[�������p��Ts�̏�����*/
			LED[pch].zero_cal_cnt = 0;
			LED[pch].zero_delta_ts = 0;
			MES[pch].zc_zero_calc = 0;  //�[���N���X
			MES[pch].err_status &= ~ERR_JUDGE_ZERO;	/*�[���_������ԃ��Z�b�g*/
			if(reset_factor != RESTART_WDOG){				//�ċN���G���[�ȊO
				MAIN[pch].com_err_status = (short)0;			/*�G���[�X�e�[�^�X�N���A*/
				MAIN[pch].led_err_status = (short)0;			/*�G���[�X�e�[�^�X�N���A*/
			}
			LED[pch].zero_active = 0;
			action_status_control(pch, ACT_STS_NORMAL);	/*����X�e�[�^�X�X�V*/
			zero_adj_status(pch);						/*�[���_�������̔g�`�f�[�^��ێ�����*/
		}else{
			if(LED[pch].zero_retry_cnt >= 2){			/*��Ts���������Ȃ��ꍇ*/
				/*�[���������s*/
				zero_adj_error(pch);					/*�[�������G���[����*/
				MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*�[�������v�����Ԕ��U�G���[�Z�b�g*/
				MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
				err_judge_status(pch);
				zero_adj_status(pch);						/*�[���_�������̔g�`�f�[�^��ێ�����*/
			}else{
				/*�[���������g���C*/
				SVD[pch].zero_offset = work_delta_ts;	/*��Ts�X�V*/
				LED[pch].zero_retry_cnt ++;			/*�[���������g���C�J�E���^�X�V*/
				LED[pch].zero_cal_cnt = 0;
				LED[pch].zero_delta_ts = 0;
				MES[pch].zc_zero_calc = 0;  //�[���N���X
				LED[pch].zero_do_cnt = ZERO_ADJ_TIME;	/*�[���_�������ԃZ�b�g*/
			}
		}
	}
#endif
}

/****************************************************/
/* Function : zero_adj_error                       */
/* Summary  : �[���_�����G���[����               				*/
/* Argument : pch                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	zero_adj_error(short pch){

	/*�[�������p��Ts�̏�����*/
	LED[pch].zero_active = 0;
	LED[pch].zero_cal_cnt = 0;
	LED[pch].zero_delta_ts = 0;
	LED[pch].zero_do_cnt = 0;
	LED[pch].vth_do_cnt = 0;
	LED[pch].wave_do_cnt = 0;
	MES[pch].vth_count = 0;
	MES[pch].err_status &= ~ERR_JUDGE_ZERO;	/*�[���_������ԃ��Z�b�g*/
	MES[pch].zc_zero_calc = 0;  //�[���N���X
	action_status_control(pch, ACT_STS_NORMAL);	/*����X�e�[�^�X�X�V*/

	MES[pch].ThresholdReq = 0; //zero_active=0�ɂȂ�̂�99�ł͂Ȃ�0
}

/****************************************************/
/* Function : alm_reset_control                       */
/* Summary  : �A���[�����Z�b�g               				*/
/* Argument : pch                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	alm_reset_control(short pch){

	short ch_read;
	short l;

	/*�A���[�����Z�b�gSW�ɂ��A���[�����Z�b�g*/
	if(sw_now_almreset[pch] == B_ON){				//�A���[�����Z�b�g
		ch_read = disp_ch_read();					//CH�X�C�b�`�ǂݍ���
		if((ch_read == SW_CH0)||					//CH0�ݒ莞�F�SCH�[���_����
			(pch == (ch_read - 1))){				//�w��CH�[���_����
			if(MAIN[pch].com_err_status == ERR_RESTART){	//�ċN���G���[
				;										//�G���[�N���A���Ȃ�
			}else if((MAIN[pch].com_err_status == ERR_EEPROM)||	//EEPROM�G���[��CUnet�G���[�͑SCH�N���A
				(MAIN[pch].com_err_status == ERR_CUNET)){		
				for(l=0;l<6;l++){
					MES[l].err_status = (short)0;		//����X�e�[�^�X�ɍX�V
					MES[l].alm_status = (short)0;		//����X�e�[�^�X�ɍX�V
					MAIN[l].err_judge = (short)0;
					MAIN[l].com_err_status = (short)0;	//�G���[�X�e�[�^�X�N���A
					MAIN[l].led_err_status = (short)0;	//�G���[�X�e�[�^�X�N���A
				}
			}else{
				MES[pch].err_status = (short)0;			//����X�e�[�^�X�ɍX�V
				MES[pch].alm_status = (short)0;			//����X�e�[�^�X�ɍX�V
				MAIN[pch].err_judge = (short)0;
				MAIN[pch].com_err_status = (short)0;		//�G���[�X�e�[�^�X�N���A
				MAIN[pch].led_err_status = (short)0;		//�G���[�X�e�[�^�X�N���A
			}
		}
	}
}

/****************************************************/
/* Function : zero_adj_status                       */
/* Summary  : �[���_�������̔g�`�f�[�^��ێ�����        				*/
/* Argument : pch                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void zero_adj_status(short pch){

	short		i_cnt;

	if(check_queue() != B_OK){			//EEPROM�����ݗpQueue���擾�\���m�F����
		return;						//�o�^�s��
	}
	
	SVD[pch].zero_flow_qat.DWORD = MES[pch].ml_min_now;		//����[mL/min]
	SVD[pch].zero_flow_vel.DWORD = MES[pch].flow_vel_c;		//����[m/s]
	SVD[pch].zero_sound_spd = MES[pch].sound_vel_f;			//����[m/s]
	SVD[pch].zero_addit.DWORD = MES[pch].addit_buff.DWORD;		//�ώZ�l[mL/min]
	SVD[pch].zero_wave_max = MES[pch].rev_max_data;			//��g�g�`�ő�l
	SVD[pch].zero_wave_min = MES[pch].rev_min_data;			//��g�g�`�ŏ��l
	SVD[pch].zero_delta_ts.DWORD = MES[pch].delta_ts_zero;		//�`�����ԍ�[ps]
	SVD[pch].zero_correlate = MES[pch].correlate;				//���֒l��
	SVD[pch].zero_zero_offset = SVD[pch].zero_offset;		//�[���_�I�t�Z�b�g
	SVD[pch].zero_condition = MAIN[pch].err_condition;			//status
	SVD[pch].zero_fifo_pos = MES[pch].fifo_no_read;			//FIFO��g�g�`���o�ʒu
	SVD[pch].zero_gain_1st = get_attenuator_gain(pch);			//Gain 1st stage
	SVD[pch].zero_gain_2nd = MES[pch].amp_gain_for;			//Gain 2nd stage
	SVD[pch].zero_fifo_ch = MES[pch].fifo_ch_read;				//FIFO CH
	SVD[pch].zero_p1p2 = MES[pch].max_point_sub_f;				//��g�̍�(P1-P2)
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
	
	//EEPROM��������
	SavEepZerAdjPrm(pch);
	
}

/****************************************************/
/* Function : com_req_check                       */
/* Summary  : �ʐM�w�ߊm�F        				*/
/* Argument : �Ȃ�                                 */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_req_check(void){

	short ch;

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		com_req_control(ch);
	}
}

/*******************************************
 * Function : SetZerAdjPrm
 * Summary  : �[�������̃p�����[�^��ݒ肷��
 * Argument : Mod : Mode
 *                : 0 -> �[�������p��Ts�̏�����
 *                : 1 -> �g�`�F�������J�n
 * Return   : void
 * Caution  : None
 * Note     : None
 * *****************************************/
void SetZerAdjPrm(short pch, short Mod)
{
	if(Mod == 0)
	{
		/*�[�������p��Ts�̏�����*/
		LED[pch].zero_cal_cnt = 0;
		LED[pch].zero_delta_ts = 0;
		MES[pch].vth_sum = 0;
		MES[pch].vth_count = 0;
													/*�R���t�B�O���[�^����̃[���_�����́A�G���[�X�e�[�^�X������ꍇ�ł�*/
													/*�[���_���������s����iSFC-012�Ɠ��d�l�Ƃ���j						*/
		MES[pch].err_status = (short)0;				/*����X�e�[�^�X�ɍX�V	*/
		MES[pch].err_status |= ERR_JUDGE_ZERO;		/*�[���_������ԃZ�b�g	*/
		MES[pch].alm_status = (short)0;			//����X�e�[�^�X�ɍX�V
		MAIN[pch].err_sub_judge = MES_SUB[pch].err_status_sub = (short)0;	//����X�e�[�^�X�ɍX�V
		MES[pch].zc_zero_calc = 0;  //�[���N���X
		LED[pch].zero_vth_buf[0] = LED[pch].zero_vth_buf[1] = LED[pch].zero_vth_buf[2] = 0;
	}
	else if(Mod == 1)
	{
		/*�g�`�F�������J�n*/
		LED[pch].zero_active = 1;					/*�[���������s��*/
		LED[pch].zero_retry_cnt = 0;				/*�[���������g���C�J�E���^������*/
#if defined(WAVE_RECOGNITION)  //FIFO CH�T�[�`�����s����
		MES[pch].ThresholdReq = 11;	/*�g�`�F�����s�v��*/
		MES[pch].ThresholdWave = AD_MAX_UNIT;		/*�g�`�F��臒l*/
		MES[pch].ThresholdPeak = 0;	/*�g�`�F���s�[�N*/
		MES[pch].ThresholdPeakPos = 0;	 /*�g�`�F���s�[�N�ʒu*/
#else //���s���Ȃ�
		MES[pch].ThresholdReq = 0;	/*�g�`�F�����s�v��*/
#endif
		LED[pch].wave_do_cnt = WAVE_ADJ_TIME;	/*�g�`�F���������ԃZ�b�g*/
		led_channel |= CH_LED[pch];				/*�_�ł�����LED���Z�b�g*/
		led_alarm = 0;								/*ALM-LED����*/
		action_status_control(pch, ACT_STS_ZERO);	/*����X�e�[�^�X�X�V*/
	}
}

/*******************************************
 * Function : CheckSearchWindow
 * Summary  : �E�B���h�E�T�[�`�̔��������
 * Argument : 
 * Return   : B_NG -> ���s
 *            B_OK -> ����
 * Caution  : None
 * Note     : None
 * *****************************************/
short CheckSearchWindow(short pch)
{
	/*Window�T�[�`(�œK��FIFO CH��T��)*/
	action_status_control(pch, ACT_STS_ZERO);	/*����X�e�[�^�X�X�V*/
	if(SearchWindow(pch) != B_OK){	/*Window�T�[�`*/			
		/*�[���������s*/
		zero_adj_error(pch);				/*�[�������G���[����*/
		MES[pch].err_status |= (ERR_JUDGE_ZERO + ERR_JUDGE_UNSTABLE);	/*�[�������v�����Ԕ��U�G���[�Z�b�g*/
		MAIN[pch].com_err_status = ERR_ZERO_UNSTABLE;
		err_judge_status(pch);
		zero_adj_status(pch);				/*�[���_�������̔g�`�f�[�^��ێ�����*/
		return B_NG;
	}
	return B_OK;
}

/****************************************************/
/* Function : com_req_control                       */
/* Summary  : �w�ߐ���        				*/
/* Argument : pch                                 */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_req_control(short pch){
	
	short l;
	short ch_read;

	/*�[���_����SW�ɂ��[���_�����̊m�F*/
	if ((SVD[pch].sensor_size != SNS_NONE)
		&& (sw_now_zeroadj == B_ON) && (MES[pch].inspect_enable == 0)){
		ch_read = disp_ch_read();				//CH�X�C�b�`�ǂݍ���
		if((ch_read == SW_CH0)||				//CH0�ݒ莞�F�SCH�[���_����
			(pch == (ch_read - 1))){			//�w��CH�[���_����

			// /*�[�������p��Ts�̏�����*/
			SetZerAdjPrm(pch, 0);

			/*�������f�o�C�X�����ʒu�̔���*/
			OWCheckDeviceSide();

			/*�������f�o�C�X���m�̊m�F*/
			CheckDeviceDetect();

#if defined(WAVE_RECOGNITION)
			// /*Window�T�[�`(�œK��FIFO CH��T��)*/
			if(CheckSearchWindow(pch) == B_NG) return;
#endif

			// /*�g�`�F�������J�n*/
			SetZerAdjPrm(pch, 1);
		}
	}

#if defined(FRQSCH)
	// ���g���T�[�`�I�����A�[�������{
	if(FrqSch[pch].FrqSchSttFlg == 2){
		eep_write_ch_delay(pch, (short)(&SVD[pch].SchFrq - &SVD[pch].max_flow), SVD[pch].SchFrq);
		SAVE[pch].control |= 0x0001; //�[�����J�n�t���OON
		// ���g���T�[�`�J�n�t���O���Z�b�g
		FrqSch[pch].FrqSchSttFlg = 0;
	}
#endif
	
	/*�ʐM�w�߂ɂ��[���_�����̊m�F*/
	if(((SAVE[pch].control_old & 0x0001) == 0) && ((SAVE[pch].control & 0x0001) != 0)){
		SAVE[pch].control &= ~0x0001;			/*�[�������R�}���h���Z�b�g*/

		if((SVD[pch].sensor_size != SNS_NONE) && (MES[pch].inspect_enable == 0)){

			/*�[�������p��Ts�̏�����*/
			SetZerAdjPrm(pch, 0);

			/*�������f�o�C�X�����ʒu�̔���*/
			OWCheckDeviceSide();

			/*�������f�o�C�X���m�̊m�F*/
			CheckDeviceDetect();

#if defined(WAVE_RECOGNITION)
			// /*Window�T�[�`(�œK��FIFO CH��T��)*/
			if(CheckSearchWindow(pch) == B_NG) return;
#endif

			// /*�g�`�F�������J�n*/
			SetZerAdjPrm(pch, 1);
		}
	}	

	/*�ʐM�w�߂ɂ��A���[�����Z�b�g�̊m�F*/
	if(((SAVE[pch].control_old & 0x0002) == 0) && ((SAVE[pch].control & 0x0002) != 0)){
		SAVE[pch].control &= ~0x0002;			/*�A���[�����Z�b�g�R�}���h���Z�b�g*/
		if(MAIN[pch].com_err_status == ERR_RESTART){			//�ċN���G���[
				;												//�G���[�N���A���Ȃ�
		}else if((MAIN[pch].com_err_status == ERR_EEPROM)||	//EEPROM�G���[��CUnet�G���[�͑SCH�N���A
			(MAIN[pch].com_err_status == ERR_CUNET)){
			for(l=0;l<6;l++){
				MES[l].err_status = (short)0;		//����X�e�[�^�X�ɍX�V
				MES[l].alm_status = (short)0;			//����X�e�[�^�X�ɍX�V
				MAIN[l].err_judge = (short)0;
				MAIN[l].com_err_status = (short)0;	//�G���[�X�e�[�^�X�N���A
				MAIN[l].led_err_status = (short)0;	//�G���[�X�e�[�^�X�N���A
			}
			reset_factor = RESTART_NORMAL;			//�ċN���v���R�[�h�N���A
		}else{
			MES[pch].err_status = (short)0;			//����X�e�[�^�X�ɍX�V
			MES[pch].alm_status = (short)0;			//����X�e�[�^�X�ɍX�V
			MAIN[pch].err_judge = (short)0;
			MAIN[pch].com_err_status = (short)0;		/*�G���[�X�e�[�^�X�N���A*/
			MAIN[pch].led_err_status = (short)0;		/*�G���[�X�e�[�^�X�N���A*/
		}
	}

	/*�ʐM�w�߂ɂ�郊�Z�b�g(�ċN��)�̊m�F*/
	if(((SAVE[pch].control_old & 0x0008) == 0) && ((SAVE[pch].control & 0x0008) != 0)){
		SAVE[pch].control &= ~0x0008;			/*���Z�b�g(�ċN��)�R�}���h���Z�b�g*/
		delay(10000);							/*�ҋ@*/
		reset_control();				/*���Z�b�g����*/
	}
	
	SAVE[pch].control_old = SAVE[pch].control;		/*��ԕۑ�*/	
}

/****************************************************/
/* Function : ram_clear_check                        */
/* Summary  : RAM�N���A    				*/
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : CH�X�C�b�`=9�A�[�������X�C�b�`ON�N����RAM�N���A */
/****************************************************/
void	ram_clear_check(void){

	short ch = 0;
	
	if(disp_ch_read() == SW_CH9 &&				//RAM�N���A���s�����̊m�F
		disp_zero_read() == B_ON){				//�\���ؑ�:CH9�A�[���_����SW:�I��
		for(ch = CH1; ch < CH_NUMMAX; ch++){	//RAM�N���A���s
			read_serial_num(ch);				//�V���A���ԍ��Ǎ���
			drive_led(DSP_CH_ALL);	//CH-LED�S��
			eep_sv_write(ch);					//�����l������
			drive_led(DSP_ALM_ALL);	//ALM-LED�S��
			write_serial_num(ch);				//�V���A���ԍ�������*
			eep_sv_read(ch);					//EEPROM�̓Ǎ���/
		}
	}
}

/****************************************************/
/* Function : ram_clear_debug                       */
/* Summary  : RAM�N���A    				*/
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ʐM(OR�R�}���h)�ɂ��RAM�N���A               */
/****************************************************/
void	ram_clear_debug(void){

	short ch = 0;
	
	for(ch = CH1; ch < CH_NUMMAX; ch++){	//RAM�N���A���s
		read_serial_num(ch);				//�V���A���ԍ��Ǎ���
		eep_sv_write(ch);					//�����l������
		write_serial_num(ch);				//�V���A���ԍ�������*
		eep_sv_read(ch);					//EEPROM�̓Ǎ���/
	}

	version_set();
}

/****************************************************/
/* Function : ReadSensDevice                      */
/* Summary  : �������f�o�C�X����Z���T����Ǎ���    				*/
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	ReadSensDevice(void){

	short ch = 0;

	for(ch=CH1; ch<CH_NUMMAX; ch++){
		if(OWReadSensinfo(ch) == B_OK){     //�Z���T����Ǎ���
			util_eep_allwrite(ch, WR_DEVICE);  //�Ǎ��񂾃Z���T����EEPROM������
		}
	}
}

/****************************************************/
/* Function : CheckDeviceDetect						*/
/* Summary  : �������f�o�C�X���m�̊m�F						*/
/* Argument : �Ȃ�									*/
/* Return   : �Ȃ�									*/
/* Caution  : �Ȃ�									*/
/* notes    : �������f�o�C�X��IN����OUT�����Ɍ��m�ł��Ȃ��ꍇ	*/
/*          : �ɃG���[�Ƃ���								*/
/****************************************************/
void	CheckDeviceDetect(void){

	short ch;

	for(ch=CH1; ch<CH_NUMMAX; ch++){
		if(SVD[ch].sensor_size != SNS_NONE && MES_SUB[ch].memory_side == B_NG){	//�������f�o�C�X��IN����OUT�����Ɍ��m�ł��Ȃ��ꍇ
			MES_SUB[ch].err_status_sub |= ERR_JUDGE_DEVICE;	//�ُ�F�������f�o�C�X���m�G���[�Z�b�g
		}else{
			MES_SUB[ch].err_status_sub &= ~ERR_JUDGE_DEVICE;	//����F�������f�o�C�X���m�G���[�N���A
		}
	}
}

/****************************************************/
/* Function : watchdog_refresh                      */
/* Summary  : �E�H�b�`�h�b�O���t���b�V��    				*/
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	watchdog_refresh(void)
{
	// RX �� FW �ł́uPCLK/32768�i����167.8ms�j* 256�v�Ŗ� 4 �b�Ȃ̂ł���ɍ��킹�� 
	WatchdogReloadSet(WATCHDOG0_BASE, g_ui32SysClock* 4);
}

/****************************************************/
/* Function : CheckBOOTCFG                           */
/* Summary  : BOOTCFG ���W�X�^���ݒ�ς݂��ǂ����̃`�F�b�N�Ɛݒ�	*/
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : ���C������                          				 */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void main(void){

	// BOOTCFG ���W�X�^���ݒ�ς݂��ǂ����̃`�F�b�N�Ɛݒ� 
	CheckBOOTCFG();

 /*������*/
	init();

	while(1){
		/*** 100msec�ҋ@ ***/
		while(timer_main != 0){
			/*EEPROM�Ƀf�[�^��������*/
			eep_write_pending_data();

			/*�������f�o�C�X�ɃZ���T����������*/
			OWWriteSensinfo();
		}
		timer_main = 20;
	
		/*** ���C������               ***/
		/*** �ȉ�100msec�����ŏ������� ***/
		/*�J�E���^����*/
		counter_control();

		/*CPU�����ԕ\��*/
		disp_cpu_status();

		/*�[���_����SW/�A���[�����Z�b�gSW���͊m�F*/
		disp_zerosw_check();

		/*�ʐM�w�ߊm�F*/
		com_req_check();

		/*�[���_����/�A���[�����Z�b�g�m�F*/
		zero_alm_check();

		/*CUnet�ʐM�����N�m�F*/
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

// �f�o�b�K���쒆�� WATCHDOG0 �p�̊��荞�݃n���h�� 
void WatchdogIntHandler(void)
{
	WatchdogIntClear(WATCHDOG0_BASE);
	
	__asm(" bkpt #0");
	__asm(" nop");
	return;
}

