/***********************************************/
/* File Name : disp.c		            									   */
/*	Summary   : �\���֘A����			                   */
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
/*	���W���[������`�֐�								*/
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
/*	���W���[���O��`�֐�								*/
/********************************************************/
extern	void	mky43_restart(void);
extern	short	mky43_get_flow_num(void);
extern unsigned long invert_data(unsigned long data);
extern void delay(unsigned short delay_count);

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/
short sw_now;			//SW��ԕۑ�
short sw_old;			//SW��ԕۑ�
short sw_cont;		//SW��ԕۑ�
short sw_cunet_num;		//CUnetSW��ԕۑ�
short	led_flash = 0;		//�S�������p
short	led_channel;		//Channel LED�\�����
short	led_alarm;		//Alarm LED�\�����
unsigned char sw_now_zeroadj;	//�[���_����SW���
unsigned char sw_now_almreset[6];	//�[���_����SW���
short CH_LED[6] = {DSP_LED1, DSP_LED2, DSP_LED3, DSP_LED4, DSP_LED5, DSP_LED6}; /*CH-LED���*/

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern short	sa_flow;
extern short	main_count;
extern short	disp_ch;
extern unsigned long	cmi_count;
extern short	com_type;

/****************************************************/
/* Function : drive_led                    */
/* Summary  : LED�\���X�V    				*/
/* Argument : led_num              	               */
/* Return   : �Ȃ� 				                    */
/* Caution  : �Ȃ�                                   */
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

	FPGA_LED_CNT = led_num;		//LED���䃌�W�X�^�ɐݒ�
}

/****************************************************/
/* Function : disp_led_ch                    */
/* Summary  : channel LED���X�V����B 				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 10ms���ɌĂ΂��                         */
/****************************************************/
void	disp_led_ch(void) {

	drive_led(led_channel);
}

/****************************************************/
/* Function : disp_led_alm                    */
/* Summary  : alarm LED���X�V����B 				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 10ms���ɌĂ΂��                         */
/****************************************************/
void	disp_led_alm(void) {

	drive_led(led_alarm);
}

/****************************************************/
/* Function : disp_init                    */
/* Summary  : LED������   				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	disp_init(void){

	led_channel = 0;
	led_alarm = 0;
}

/****************************************************/
/* Function : disp_zerosw_check                    */
/* Summary  : �X�C�b�`���� : �[���_�����X�C�b�`�m�F      				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	disp_zerosw_check(void){

	short sw_work;
	short ch_read;
	short	i_cnt;
	
	for(i_cnt=CH1; i_cnt<CH_NUMMAX; i_cnt++){
		if(LED[i_cnt].zero_active != 0){		//�[���_�������͏������Ȃ�
			return;
		}
	}

	sw_work = disp_zero_read();			//�[���_����SW�Ǎ���

	if(sw_work != sw_old){				//SW��ԕω�����
		sw_now = sw_work;				//SW��ԍX�V
		sw_cont = 0;
		sw_now_zeroadj = B_OFF;
	}else{													//SW��ԕω��Ȃ�
		if(sw_now == B_ON && sw_now_zeroadj == B_OFF){		//SW�������
			sw_cont++;
		}else{
			sw_cont = 0;
			sw_now_zeroadj = B_OFF;
		}
		
		if(sw_cont > SW_ZERO_START){	//SW����3�b�o��
			sw_cont = 0;
			sw_now_zeroadj = B_ON;		//�[���_�����v��
			
			ch_read = disp_ch_read();	//CH�X�C�b�`�ǂݍ���
			if(ch_read == SW_CH0){
				led_channel = 0;
			}
			else if( (ch_read >= SW_CH1) && (ch_read <= SW_CH6)
				&& (SVD[ch_read -1].sensor_size != SNS_NONE)){
				led_channel = 0;
			}
		}
	}
	sw_old = sw_work;					//SW��ԕۑ�
}

/****************************************************/
/* Function : disp_zero_read                    */
/* Summary  : �X�C�b�`���� : �[���X�C�b�`�Ǎ���    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �X�C�b�`���					                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		disp_zero_read(void){

	short sw;

	// GPIO_PH4
	if(__bit_input(GPIO_PORTH_BASE, 4) == 0){		//Zero����
		sw = B_ON;
	}else{
		sw = B_OFF;
	}

	return (sw);
}

/****************************************************/
/* Function : disp_ch_read                    */
/* Summary  : �X�C�b�`���� : CH�ԍ��Ǎ���[SW2] 		*/
/* Argument : �Ȃ�                  	               */
/* Return   : CH�ԍ�			                        */
/* Caution  : �Ȃ�                                   */
/* notes    : 10�|�W�V����(0�`9)                        */
/****************************************************/
short	disp_ch_read(void){
	
	short sw_status;

//	sw_status = (FPGA_SW_DISP & 0x000F);	//SW�\�����W�X�^�Ǎ���
	sw_status = FPGA_SW_DISP;	//SW�\�����W�X�^�Ǎ���
	sw_status = (sw_status ^ 0xFFFFFFFF);	//�r�b�g���]
	sw_status = (sw_status & 0x000F);	//CH�X�C�b�`�̗L���r�b�g

	return (sw_status);
}

/****************************************************/
/* Function : disp_cunet_read                    */
/* Summary  : �X�C�b�`���� : CUnet�A�h���X�Ǎ���	[SW3] 	*/
/* Argument : �Ȃ�                  	               */
/* Return   : CUnet�A�h���X		                    */
/* Caution  : �Ȃ�                                   */
/* notes    : 16�|�W�V����(0�`15)                       */
/****************************************************/
short	disp_cunet_read(void){
	
	short sw_status;
	
//	sw_status = ((FPGA_SW_DISP & 0x00F0) >> 4);	//SW�\�����W�X�^�Ǎ���
	sw_status = FPGA_SW_DISP;	//SW�\�����W�X�^�Ǎ���
	sw_status = (sw_status ^ 0xFFFFFFFF);	//�r�b�g���]
	sw_status = ((sw_status & 0x00F0) >> 4);	//CUnet�A�h���X�X�C�b�`�̗L���r�b�g
	
	return (sw_status);
}

/****************************************************/
/* Function : disp_cunet_set                    */
/* Summary  : CUnet�A�h���X�ݒ�	    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	disp_cunet_set(void){

	if(com_type == COM_CUNET){
		short sw_now;

		sw_now = mky43_get_flow_num();
		if(sw_now == sw_cunet_num){	//CUnetSW��ԕω��Ȃ�
			return;
		}
	
   	/*�����݋֎~*/
		clrpsw_i();
	
	/*CUnet�A�h���X�ݒ菈��*/
		mky43_restart();		//MKY43�̒ʐM�ĊJ

 	/*�����݋���*/
		setpsw_i();
	
		sw_cunet_num = sw_now;		//CUnetSW��ԕۑ�
	}
}

/****************************************************/
/* Function : disp_led_control                    */
/* Summary  : �p�l����LED����		    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 1�b��1��Ă΂��                         */
/****************************************************/
void	disp_led_control(void){

	short		i_cnt;
	short		ch_num;
	short		ch_read;
	short 	oth_status;

	ch_num = CH1;
	oth_status = 0;

	/* �ʏ�^�p */
	if(led_flash == 0){
		
		if(disp_ch >= CH_NUMMAX){
			return;
		}

		for(i_cnt=CH1; i_cnt<CH_NUMMAX; i_cnt++){
			if(LED[i_cnt].zero_active != 0){	//�[���_��������LED�������Ȃ�
				return;
			}
		}
	
		ch_read = disp_ch_read();			//�\���ؑ�CH�̎擾
		if(ch_read > SW_CH6){
			disp_init();				//�SLED����
			return;
		}
		if(ch_read>=SW_CH1 && ch_read<=SW_CH6){		//CH1�`6�ݒ莞�i�w��CH�����j
			ch_num = ch_read - 1;
			//��CH�̃G���[�����Achannel LED�ɕ\��
			led_channel = 0;
			for(i_cnt=CH1; i_cnt<CH_NUMMAX; i_cnt++){	//��CH�G���[���m�F
				oth_status = disp_led_judge(i_cnt);	//�G���[���Ǎ���	
				if(oth_status != 0){			//�G���[��񂠂�
					led_channel |= CH_LED[i_cnt];
				}
			}
			//��ch�̃G���[�����o��
			led_alarm = disp_led_judge(ch_num);		//�G���[���Ǎ���
		}else{							//CH0�ݒ莞�i�SCH�����j
			led_channel = CH_LED[disp_ch];
			led_alarm = disp_led_judge(disp_ch);
		}
	}
	/* �S�������� */
	else{
		led_channel = 0x003F;
		led_alarm = 0x0F00;
	}
}

/****************************************************/
/* Function : disp_led_judge                      */
/* Summary  : �G���[LED��ʔ���    				              */
/* Argument : pch                  	               */
/*	Return   :	led	: �G���[LED���						              */
/*						      LED1:�G���v�e�B�Z���T			                  */
/*						      LED2:�g�`�ُ�/�A���o�����X	              	*/
/*						      LED3:�[���_�������s				                */
/*						      LED4:���̑��ُ�					                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		disp_led_judge(short pch){

	short		led;
	unsigned short led_err_status;
	
	led = 0;

	if(pch >= CH_NUMMAX){
		return (led);
	}

	led_err_status = MAIN[pch].led_err_status;
	
	if(led_err_status == 0 ||				//�G���[��������
		led_err_status >= ERR_CODE_MAX){	//�w��G���[�ȊO�̏ꍇ
		return (led);
	}
	
	led = err_inf[led_err_status].err_led;	//�_��LED

	return (led);
}

/****************************************************/
/* Function : disp_led_cpu                    */
/* Summary  : LED���� : CPU LED����    				*/
/* Argument : on_off                	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : LED�_��/����/�_��	                      */
/****************************************************/
void	disp_led_cpu(short on_off){

	/* �ʏ�^�p */
	if(led_flash == 0){
	
		if(on_off == B_ON){			//LED�_��
			// GPIO_PB7
			__bit_output(GPIO_PORTB_BASE, 7, 1);
		}else if(on_off == B_OFF){	//LED����
			// GPIO_PB7
			__bit_output(GPIO_PORTB_BASE, 7, 0);
		}else{							//LED�_��
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
	/* �S�������� */
	else{
		// GPIO_PB7
		__bit_output(GPIO_PORTB_BASE, 7, 1);
	}
}

/****************************************************/
/* Function : disp_cpu_status                    */
/* Summary  : LED���� : CPU�����ԕ\��        				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/*	notes    :�@CPU���펞�F�ᑬ�_��(1sec����)	          	*/
/*	�@�@�@�@       CPU�ُ펞�F�����_��(300msec����)	       */
/****************************************************/
void	disp_cpu_status(void){

	short timer;
	
	if(reset_factor == RESTART_WDOG){	//�E�H�b�`�h�b�O�ċN����
		timer = 3;						//300msec����(100msec x 3�� = 300msec)
	}else{								//CPU���퓮�쎞
		timer = 10;						//1sec����(100msec x 10�� = 1sec)
	}
		
	if((main_count % timer) == 0){ 	//�_�ōX�V����
		disp_led_cpu(B_BLINK);			//CPU LED�_��
	}
}
