/***********************************************/
/* File Name : ctlioport.c         									   */
/*	Summary   : I/O�|�[�g����			                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "machine.h"
#include "ctlioport.h"
#include "define.h"

/********************************************************/
/*	���W���[������`�֐�								*/
/********************************************************/
void portmesfwdW(unsigned char data, short pch);
void portmesrevW(unsigned char data, short pch);
void OutputRestartPulse(void);
void OutputStartPulse(void);
void CheckEndPulse(void);

/********************************************************/
/*	���W���[���O��`�֐�								*/
/********************************************************/

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/


/****************************************************/
/* Function : portmesfwdW                           */
/* Summary  : ���ʌv�� �Г�:FORWARD      			          */
/* Argument : data,   pch                          */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : ���ʌv�� �Г�:REVERSE      			          */
/* Argument : data,   pch                          */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : RESTART�M���p���X�o��      			          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void OutputRestartPulse(void){

	// GPIO_PN0
	__bit_output(GPIO_PORTN_BASE, 0, 1);	/*�p���X�o��*/
	// GPIO_PN0
	__bit_output(GPIO_PORTN_BASE, 0, 0);
}

/****************************************************/
/* Function : OutputStartPulse                     */
/* Summary  : START�M���p���X�o��        			          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void OutputStartPulse(void){

	// GPIO_PJ0
	__bit_output(GPIO_PORTJ_BASE, 0, 1);	/*�p���X�o��*/
	// GPIO_PJ0
	__bit_output(GPIO_PORTJ_BASE, 0, 0);
}

/****************************************************/
/* Function : CheckEndPulse                        */
/* Summary  : END�M���m�F              			          */
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void CheckEndPulse(void){

	// GPIO_PJ1
	while(__bit_input(GPIO_PORTJ_BASE, 1) == 1);		/*END�M���m�F*/
}
