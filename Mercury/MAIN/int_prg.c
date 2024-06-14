/***********************************************/
/* File Name : int_prg.c		         									   */
/*	Summary   : �����ݏ���					                   */
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
/*	���W���[������`�֐�								*/
/********************************************************/
void	int_prg(void);
void	int_cmt2(void);
void	int_mky_recv0(void);
void	int_mky_recv1(void);

/********************************************************/
/*	���W���[���O��`�֐�								*/
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
/*	���W���[������`�ϐ�								*/
/********************************************************/
short	disp_ch;
short	timer_main;

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern unsigned long	cmi_count;
extern short ChCtrlNow;
extern short com_type;


/****************************************************/
/* Function : int_prg                               */
/* Summary  : ����������ݏ���(3msec����) 			          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs�� */
/****************************************************/
void	int_prg(void){

	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	/*�E�I�b�`�h�b�O�^�C�}�A���Z�b�g*/
	void watchdog_refresh(void);
	watchdog_refresh();

	/*���d�����݋���*/
	setpsw_i();						/*�����݋���*/
	
	int_flow(ChCtrlNow);			/*���ʌv��*/

//	debug_mode(ChCtrlNow);			/*�����G���[����i�f�o�b�O���[�h�j*/

	err_judge_status(ChCtrlNow);	/*�G���[����*/
	err_judge_holdtime(ChCtrlNow);	/*�G���[�z�[���h�^�C������*/

	if(com_type == COM_CUNET){
		mky43_write_alarm(ChCtrlNow);	/*�G���[�X�e�[�^�X������(CUnet)*/
		mky43_write_flow(ChCtrlNow);	/*�u�����ʏ�����(CUnet)*/
	}

	//����CH�X�V
	if(ChCtrlNow >= CH_IDXMAX)
//	if(ChCtrlNow >= 2) //6ms�����݂ɕύX
	{
		ChCtrlNow = 0;
	}else{
		ChCtrlNow++;
	}
}

/****************************************************/
/* Function : int_cmt2                              */
/* Summary  : �^�C�}�����ݏ���(5msec����)  				          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs�� */
/****************************************************/
void	int_cmt2(void){

	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	cmi_count ++;						//���������݃J�E���^�X�V
	if(cmi_count > CMI_COUNT_MAX){
		cmi_count = 1;
	}

	/*���C�������^�C�}�[*/
	if(timer_main <= 0){
		timer_main = 0;					//�^�C�}�[�N���A
	}else{
		timer_main--;					//�^�C�}�[�X�V
	}

	setpsw_i();							//�����݋���//

	if((cmi_count % CMI_RATIO_100MSEC) == 0){ 	//100msec����
		flow_save_control();  /*�u�����ʕۑ�*/
	}
	
	if((cmi_count % CMI_RATIO_1SEC) == 0){ 	//1�b����
		disp_led_control();
		disp_ch ++;								//�\��CH�X�V
		if(disp_ch >= CH_NUMMAX)
//		if(disp_ch >= 3)    //6ms�����݂ɕύX
		{
			disp_ch = CH1;
		}
	}

	clrpsw_i();      					//�����݋֎~//

	if(cmi_count % 2 == 0){
		disp_led_ch();					//�p�l����CH-LED�X�V
	}else{
		disp_led_alm();					//�p�l����ALM-LED�X�V
	}

	setpsw_i();							//�����݋���//
}

/****************************************************/
/* Function : int_mky_recv0                         */
/* Summary  : MKY43�����ݏ���           				          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs�� */
/****************************************************/
void	int_mky_recv0(void){

	GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_7);	// GPIO_PP7 

	setpsw_i();					//�����݋���//

	if(com_type == COM_CUNET){
		mky43_check_recv0();	//MKY43�����݊m�F
	}
}

/****************************************************/
/* Function : int_mky_recv1                         */
/* Summary  : MKY43�����ݏ���           				          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs�� */
/****************************************************/
 void	int_mky_recv1(void){

	GPIOIntClear(GPIO_PORTQ_BASE, GPIO_PIN_7);	// GPIO_PQ7 

 	if(com_type == COM_CUNET){
		mky43_check_recv1();	//MKY43�����݊m�F
	}
}
