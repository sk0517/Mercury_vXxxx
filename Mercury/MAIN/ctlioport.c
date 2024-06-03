/***********************************************/
/* File Name : ctlioport.c         									   */
/*	Summary   : I/Oポート処理			                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "machine.h"
#include "ctlioport.h"
#include "define.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void portmesfwdW(unsigned char data, short pch);
void portmesrevW(unsigned char data, short pch);
void OutputRestartPulse(void);
void OutputStartPulse(void);
void CheckEndPulse(void);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/


/****************************************************/
/* Function : portmesfwdW                           */
/* Summary  : 流量計測 片道:FORWARD      			          */
/* Argument : data,   pch                          */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void portmesfwdW(unsigned char data, short pch){
pch = 0;
	if(data == 0) {
		switch(pch){
		// GPIO_PE1
		case CH1:	__bit_output(GPIO_PORTE_BASE, 1, 0);	break;
		// GPIO_PK1
		case CH2:	__bit_output(GPIO_PORTK_BASE, 1, 0);	break;
		// GPIO_PK3
		case CH3:	__bit_output(GPIO_PORTK_BASE, 3, 0);	break;
		// GPIO_PR5
		case CH4:	__bit_output(GPIO_PORTR_BASE, 5, 0);	break;
		// GPIO_PH6
		case CH5:	__bit_output(GPIO_PORTH_BASE, 6, 0);	break;
		// GPIO_PA1
		case CH6:	__bit_output(GPIO_PORTA_BASE, 1, 0);	break;
		default:	break;
		}
	} else {
		switch(pch){
		// GPIO_PE1
		case CH1:	__bit_output(GPIO_PORTE_BASE, 1, 1);	break;
		// GPIO_PK1
		case CH2:	__bit_output(GPIO_PORTK_BASE, 1, 1);	break;
		// GPIO_PK3
		case CH3:	__bit_output(GPIO_PORTK_BASE, 3, 1);	break;
		// GPIO_PR5
		case CH4:	__bit_output(GPIO_PORTR_BASE, 5, 1);	break;
		// GPIO_PH6
		case CH5:	__bit_output(GPIO_PORTH_BASE, 6, 1);	break;
		// GPIO_PA1
		case CH6:	__bit_output(GPIO_PORTA_BASE, 1, 1);	break;
		default:	break;
		}
	}
}

/****************************************************/
/* Function : portmesrevW                           */
/* Summary  : 流量計測 片道:REVERSE      			          */
/* Argument : data,   pch                          */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void portmesrevW(unsigned char data, short pch){
pch = 0;
	if(data == 0) {
		switch(pch){
		// GPIO_PE0
		case CH1:	__bit_output(GPIO_PORTE_BASE, 0, 0);	break;
		// GPIO_PK0
		case CH2:	__bit_output(GPIO_PORTK_BASE, 0, 0);	break;
		// GPIO_PK2
		case CH3:	__bit_output(GPIO_PORTK_BASE, 2, 0);	break;
		// GPIO_PR4
		case CH4:	__bit_output(GPIO_PORTR_BASE, 4, 0);	break;
		// GPIO_PH5
		case CH5:	__bit_output(GPIO_PORTH_BASE, 5, 0);	break;
		// GPIO_PA0
		case CH6:	__bit_output(GPIO_PORTA_BASE, 0, 0);	break;
		default:	break;
		}
	} else {
		switch(pch){
		// GPIO_PE0
		case CH1:	__bit_output(GPIO_PORTE_BASE, 0, 1);	break;
		// GPIO_PK0
		case CH2:	__bit_output(GPIO_PORTK_BASE, 0, 1);	break;
		// GPIO_PK2
		case CH3:	__bit_output(GPIO_PORTK_BASE, 2, 1);	break;
		// GPIO_PR4
		case CH4:	__bit_output(GPIO_PORTR_BASE, 4, 1);	break;
		// GPIO_PH5
		case CH5:	__bit_output(GPIO_PORTH_BASE, 5, 1);	break;
		// GPIO_PA0
		case CH6:	__bit_output(GPIO_PORTA_BASE, 0, 1);	break;
		default:	break;
		}
	}
}

/****************************************************/
/* Function : OutputRestartPulse                  */
/* Summary  : RESTART信号パルス出力      			          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void OutputRestartPulse(void){

	// GPIO_PN0
	__bit_output(GPIO_PORTN_BASE, 0, 1);	/*パルス出力*/
	// GPIO_PN0
	__bit_output(GPIO_PORTN_BASE, 0, 0);
}

/****************************************************/
/* Function : OutputStartPulse                     */
/* Summary  : START信号パルス出力        			          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void OutputStartPulse(void){

	// GPIO_PJ0
	__bit_output(GPIO_PORTJ_BASE, 0, 1);	/*パルス出力*/
	// GPIO_PJ0
	__bit_output(GPIO_PORTJ_BASE, 0, 0);
}

/****************************************************/
/* Function : CheckEndPulse                        */
/* Summary  : END信号確認              			          */
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void CheckEndPulse(void){

	// GPIO_PJ1
	while(__bit_input(GPIO_PORTJ_BASE, 1) == 1);		/*END信号確認*/
}
