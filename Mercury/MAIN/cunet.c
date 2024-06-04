/***********************************************/
/* File Name : cunet.c		           									   */
/*	Summary   : CUnet�ʐM����	                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>
#include <string.h>	
#include <stdlib.h>

#include "cunet.h"
#include "version.h"
#include "define.h"
#include "defMES.h"
#include "defMAIN.h"
#include "SV_def.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

/********************************************************/
/*	���W���[������`�֐�								*/
/********************************************************/
void	mky43_init(void);
void	mky43_start(void);
void	mky43_restart(void);
void	mky43_set_register(void);
void	mky43_ping_active(void);
void	mky43_mail_recv(void);
void	mky43_ready_recv0(void);
void	mky43_ready_recv1(void);
void	mky43_mail_send(unsigned short size, short com_mode);
void	mky43_mail_status(void);
unsigned long	mky43_read_data(short flow_no);
unsigned long	mky43_read_data2(short flow_no);
void	mky43_write_alarm(short ch);
void	mky43_write_flow(short ch);
long	flow_fs_cal(short ch);
void	mky43_check_datarenewal(void);
short	mky43_get_flow_num(void);
void	mky43_check_recv0(void);
void	mky43_check_recv1(void);
void	mky43_TX_start(short com_mode);
void	mky43_rxbuf_save(short buf_side, unsigned short recv_size, unsigned short recv_sa);
void	mky43_host_link_check(void);
short		mky43_ccr_check(void);
short		mky43_add_message(unsigned short len, short com_mode);
void	mky43_del_message(unsigned short len, unsigned short offset, short com_mode);

/********************************************************/
/*	���W���[���O��`�֐�								*/
/********************************************************/
extern short	disp_cunet_read(void);
extern void	protocol_timer_host(unsigned short value);
extern void protocol_timer_subhost(void);
extern short	util_passed_time(unsigned long basic_time, unsigned long target_time);
extern void	util_delay(short target_time);

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/
short				sa_flow;		//�X�e�[�V�����A�h���X
unsigned short	recv_sa_host;	//���M���X�e�[�V�����A�h���X
unsigned short	recv_sa_sub;	//���M���X�e�[�V�����A�h���X
unsigned short 	cr_check;
short				buf_count;
short				buf_count_sub;
short				com_start;
short				com_link;
short				len_err_host;
short				len_err_sub;
short				format_err_host;
short				format_err_sub;

unsigned short	recv_data0[32][4] = { 0 };		//���[����M�f�[�^(MRB0:���[����M�o�b�t�@0)
unsigned short	recv_data1[32][4] = { 0 };		//���[����M�f�[�^(MRB1:���[����M�o�b�t�@1)
unsigned short	send_data[32][4]  = { 0 };		//���[�����M�f�[�^(�z�X�g�p)
unsigned short	send_data_sub[32][4]  = { 0 };	//���[�����M�f�[�^(�T�u�z�X�g�p)

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern short 	sw_cunet_num;
extern char *RX_buf_host;
extern char *TX_buf_host;
extern char *RX_buf_subhost;
extern char *TX_buf_subhost;
extern short com_type;
extern unsigned long	cmi_count;

/****************************************************/
/* Function : mky43_init                    */
/* Summary  : MKY43�̏�����    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_init(void){

	// GPIO_PK4
	__bit_output(GPIO_PORTK_BASE, 4, 1);				//MKY43 RST

	sa_flow = mky43_get_flow_num();		//���ʌv�ԍ��Ǎ���/
	sw_cunet_num = sa_flow;				//CUnetSW��ԕۑ�

	mky43_start();						//MKY43�̒ʐM�J�n
	mky43_set_register();				//MKY43�̃��W�X�^�ݒ�
	mky43_ping_active();				//PING���샌�W�X�^�ݒ�
}
	
/****************************************************/
/* Function : mky43_start                    */
/* Summary  : MKY43�̒ʐM�J�n    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_start(void){

	unsigned short	bcr_sa;
	unsigned short	bcr_bps;
	unsigned short	bcr_own;
	unsigned short	bcr_lfs;
	short i_cnt;
	short j_cnt;

	/*RAM�N���A*/
	//GM�̈�̃N���A
	for(i_cnt=0; i_cnt<64; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.GM.SA[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	//MSB�̈�̃N���A
	for(i_cnt=0; i_cnt<32; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.MSB.SEND[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	//MRB0�̈�̃N���A
	for(i_cnt=0; i_cnt<32; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.MRB0.RECV[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	//MRB1�̈�̃N���A
	for(i_cnt=0; i_cnt<32; i_cnt++){
		for(j_cnt=0; j_cnt<4; j_cnt++){
			MKY43.MRB1.RECV[i_cnt].DATA[j_cnt].DATA = 0;
		}
	}
	
	/*��{(BCR)�ݒ�*/
	if(MKY43.REG.SCR.BIT.START == 1){			//CUnet�ʐM�N����
		MKY43.REG.SCR.BIT.START = 0;			//CUnet�ʐM��~�i������~�j
		while(MKY43.REG.SCR.BIT.START == 1){	//CUnet�ʐM��~�ҋ@
			;
		}
	}

	MKY43.REG.SCR.BIT.GMM = 1;

	bcr_sa = sa_flow;					//�X�e�[�V�����A�h���X�ݒ�
	bcr_bps = 3;						//�]�����[�g�ݒ�i12Mbps�Œ�j
	bcr_own = 3;						//��L���ݒ�i3�Œ�j
	bcr_lfs = 0;						//�t���[���I�v�V�����ݒ�i�ʏ�0�j
	MKY43.REG.BCR.WORD = (bcr_sa + (bcr_bps<<6) + (bcr_own<<8) + (bcr_lfs<<15));

	MKY43.REG.SCR.BIT.GMM = 0;
 	send_err_status = 0;				//���[�����M�G���[�X�e�[�^�X�N���A(GM�G���A)

	/*CUnet�ʐM�N��*/
	MKY43.REG.SCR.BIT.START = 1;			//CUnet�ʐM�N��

	while(1){
		if(MKY43.REG.SCR.BIT.RUN != 0){	//RUN�t�F�[�Y�m�F
			com_start = B_ON;				//�ʐM�I�����
			break;
		}
		if(MKY43.REG.SCR.BIT.CALL != 0){	//CALL�t�F�[�Y�m�F
			com_start = B_OFF;				//�ʐM�I�t���
			break;
		}
		if(MKY43.REG.SCR.BIT.BRK != 0){	//BRK�t�F�[�Y�m�F
			com_start = B_OFF;				//�ʐM�I�t���
			break;
		}
	}

	/*CCTR���W�X�^�ݒ�*/
	MKY43.REG.CCTR.WORD = 0x0101;			//LCARE,MCARE�M�������񐔂��N���A
}

/****************************************************/
/* Function : mky43_restart                    */
/* Summary  : MKY43�̒ʐM�ĊJ    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_restart(void){

	unsigned short	bcr_sa;
	unsigned short	bcr_bps;
	unsigned short	bcr_own;
	unsigned short	bcr_lfs;

	MKY43.REG.INT0SR.WORD = 0xFFFF;
	
	MKY43.REG.SCR.BIT.GMM = 1;

	bcr_sa = sa_flow;					//�X�e�[�V�����A�h���X�ݒ�
	bcr_bps = 3;						//�]�����[�g�ݒ�i12Mbps�Œ�j
	bcr_own = 3;						//��L���ݒ�i3�Œ�j
	bcr_lfs = 0;						//�t���[���I�v�V�����ݒ�i�ʏ�0�j
	MKY43.REG.BCR.WORD = (bcr_sa + (bcr_bps<<6) + (bcr_own<<8) + (bcr_lfs<<15));

	MKY43.REG.SCR.BIT.GMM = 0;
	MKY43.REG.MESR.WORD = 0;			//���[�����M�G���[�X�e�[�^�X�N���A

	MKY43.REG.SCR.BIT.START = 1;		//CUnet�ʐM�N��
}

/****************************************************/
/* Function : mky43_set_register                    */
/* Summary  : MKY43�̃��W�X�^�ݒ�    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_set_register(void){

	short	set_bit;

	/*���[����M�o�b�t�@�ݒ�(MRB0,MRB1)*/
	MKY43.REG.INT0CR.BIT.MR = 1;			//���[����M���������݋���
	MKY43.REG.MR0CR.BIT.RDY = 1;			//���[����M����

	/*���[�����M�o�b�t�@�ݒ�*/
	MKY43.REG.INT0CR.BIT.MSF = 0;			//���[�����M���������݋֎~

	/*GM�G���A�̃f�[�^�J�ڌ��o�ݒ�*/
	set_bit = 0x0000;
	switch(sa_flow){
		case	SA_FLOW01:
		case	SA_FLOW02:
		case	SA_FLOW03:
		case	SA_FLOW04:
				set_bit = 0x0001;
				break;
		case	SA_FLOW05:
		case	SA_FLOW06:
		case	SA_FLOW07:
		case	SA_FLOW08:
				set_bit = 0x0002;
				break;
		case	SA_FLOW09:
		case	SA_FLOW10:
		case	SA_FLOW11:  //CUnet 16ch
		case	SA_FLOW12:
				set_bit = 0x0004;
				break;
		case	SA_FLOW13:  //CUnet 16ch
		case	SA_FLOW14:
		case	SA_FLOW15:
		case	SA_FLOW16:
				set_bit = 0x0008;
				break;
		default:
				break;
	}
	MKY43.REG.DRCR.DATA[0].DATA = set_bit;
	MKY43.REG.INT1CR.BIT.DR = 1;			//�f�[�^�X�V�����݋���

}

/****************************************************/
/* Function : mky43_ping_active                    */
/* Summary  : PING���샌�W�X�^�ݒ�    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_ping_active(void){

	MKY43.REG.UTCR.BIT.OE1 = 1;		//#UTY1�����#PING�o�͂����� 
	MKY43.REG.UTCR.BIT.SS1 = 0;		//#PING�o�� 
	MKY43.REG.UTCR.BIT.OE2 = 0;		//#UTY2�����#PING�o�͂����� 
	MKY43.REG.UTCR.BIT.SS2 = 0;		//#PING�o�� 

}

/****************************************************/
/* Function : mky43_mail_recv                    */
/* Summary  : ���[����M    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : MRB0:���[����M�o�b�t�@0/MRB1:���[����M�o�b�t�@1 */
/****************************************************/
void	mky43_mail_recv(void){

	short				i_cnt;
	short				j_cnt;
	unsigned short	recv_size;		//���[���f�[�^�T�C�Y
	unsigned short	recv_sa;		//���M���X�e�[�V�����A�h���X

	/*MRB0:���[����M�o�b�t�@0*/
	if(MKY43.REG.MR0CR.BIT.RCV != 0){					//���[����M����
		for(i_cnt=0; i_cnt<32; i_cnt++){				//��M�o�b�t�@������
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data0[i_cnt][j_cnt] = 0;
			}
		}		
		
		recv_sa = ((MKY43.REG.MR0CR.WORD & 0x3F00) >> 8);		//���M���X�e�[�V�����A�h���X�擾
		recv_size = (MKY43.REG.MR0CR.WORD & 0x003F);	//���[����M�f�[�^�T�C�Y�擾(8Byte��1�P��)

		for(i_cnt=0; i_cnt<recv_size; i_cnt++){		//���[����M�f�[�^�擾
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data0[i_cnt][j_cnt] = MKY43.MRB0.RECV[i_cnt].DATA[j_cnt].DATA;
			}
		}
		mky43_rxbuf_save(0, recv_size, recv_sa);		//�擾�f�[�^��RX_buf_host[]�ɕۑ�
	}

	/*MRB1:���[����M�o�b�t�@1*/
	if(MKY43.REG.MR1CR.BIT.RCV != 0){					//���[����M����
		for(i_cnt=0; i_cnt<32; i_cnt++){				//��M�o�b�t�@������
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data1[i_cnt][j_cnt] = 0;
			}
		}
		
		recv_sa = ((MKY43.REG.MR1CR.WORD & 0x3F00) >> 8);		//���M���X�e�[�V�����A�h���X�擾
		recv_size = (MKY43.REG.MR1CR.WORD & 0x003F);	//���[����M�f�[�^�T�C�Y�擾(8Byte��1�P��)

		for(i_cnt=0; i_cnt<recv_size; i_cnt++){		//���[����M�f�[�^�擾
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data1[i_cnt][j_cnt] = MKY43.MRB1.RECV[i_cnt].DATA[j_cnt].DATA;
			}
		}
		mky43_rxbuf_save(1, recv_size, recv_sa);		//�擾�f�[�^��RX_buf_host[]�ɕۑ�
	}

	MKY43.REG.INT0CR.BIT.MR = 1;						//���[����M���������݋���	
	MKY43.REG.INT0SR.BIT.MR = 1;						//���[����M���������݉���
	MKY43.REG.MR0CR.BIT.RDY = 1;						//���[����M����
}

/****************************************************/
/* Function : mky43_ready_recv0                    */
/* Summary  : ���[����M����    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : MRB0:���[����M�o�b�t�@0                    */
/****************************************************/
void	mky43_ready_recv0(void){

	if(com_type == COM_CUNET){
		MKY43.REG.MR0CR.BIT.RCV = 0;
		MKY43.REG.MR0CR.BIT.RDY = 1;		//���[����M����

		MKY43.REG.INT0CR.BIT.MR = 1;		//���[����M���������݋���
		MKY43.REG.INT0SR.BIT.MR = 1;		//���[����M���������݉���
	}
}

/****************************************************/
/* Function : mky43_ready_recv1                    */
/* Summary  : ���[����M����    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : MRB1:���[����M�o�b�t�@1                    */
/****************************************************/
void	mky43_ready_recv1(void){

	if(com_type == COM_CUNET){
		MKY43.REG.MR1CR.BIT.RCV = 0;

		MKY43.REG.INT1CR.BIT.MR = 1;		//���[����M���������݋���
		MKY43.REG.INT1SR.BIT.MR = 1;		//���[����M���������݉���
	}
}

/****************************************************/
/* Function : mky43_mail_send                    */
/* Summary  : ���[�����M    				*/
/* Argument : size,   com_mode     	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : MSB:���[�����M�o�b�t�@                   */
/****************************************************/
void	mky43_mail_send(unsigned short size, short com_mode){

	short				ch;
	short				i_cnt;
	short				j_cnt;
	short				send_retry;
	unsigned short	send_sa;			//���M��X�e�[�V�����A�h���X
	unsigned short	send_size;			//���[���f�[�^�T�C�Y
	unsigned long	send_time;			//���M�J�n����
	
	if(com_mode == SA_HOST){
		send_sa = recv_sa_host;			//���[�����M��X�e�[�V�����A�h���X
	}else{
		send_sa = recv_sa_sub;			//���[�����M��X�e�[�V�����A�h���X
	}
	send_size = size;

	IntPrioritySet(INT_TIMER1A, 5 << 5);			//�ꎞ�I�ɗD�揇�ʂ��グ��i��M�������荞�݁i�����e�i���X�|�[�g�j�j
	while(MKY43.REG.MSCR.BIT.SEND == 1);		//���[�����M���͑ҋ@
	IntPrioritySet(INT_TIMER1A, 6 << 5);			//�D�揇�ʂ����ɖ߂�
		
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		MES[ch].err_status &= ~ERR_JUDGE_CUNET;	//CUnet�G���[���Z�b�g
	}

	MKY43.REG.MESR.WORD = 0;					//���[�����M�G���[�X�e�[�^�X�N���A

	if(com_mode == SA_HOST){
		for(i_cnt=0; i_cnt<send_size; i_cnt++){	//���[�����M�f�[�^�쐬
			for(j_cnt=0; j_cnt<4; j_cnt++){
				MKY43.MSB.SEND[i_cnt].DATA[j_cnt].DATA = send_data[i_cnt][j_cnt];
			}
		}
	}else{
		for(i_cnt=0; i_cnt<send_size; i_cnt++){	//���[�����M�f�[�^�쐬
			for(j_cnt=0; j_cnt<4; j_cnt++){
				MKY43.MSB.SEND[i_cnt].DATA[j_cnt].DATA = send_data_sub[i_cnt][j_cnt];
			}
		}
	}

	send_time = cmi_count;						//���[�����M�J�n����
	send_retry = 0;								//���[�����M���g���C��

RESEND:
	
	MKY43.REG.MSCR.WORD = (0x0000 | (send_sa<<8) | send_size);	//���M��X�e�[�V�����A�h���X�A�f�[�^�T�C�Y
	MKY43.REG.MSCR.BIT.SEND = 1;				//���[�����M�J�n

	IntPrioritySet(INT_TIMER1A, 5 << 5);			//�ꎞ�I�ɗD�揇�ʂ��グ��i��M�������荞�݁i�����e�i���X�|�[�g�j�j
	while(MKY43.REG.MSCR.BIT.SEND == 1);		//���[�����M���͑ҋ@
	IntPrioritySet(INT_TIMER1A, 6 << 5);			//�D�揇�ʂ����ɖ߂�

	mky43_mail_status();						//���[�����M�G���[�X�e�[�^�X(CUnet�G���[���o�AGM�G���A���X�V)
	
	if(B_YES != util_passed_time(send_time, MES_RESEND_LIM)){	//�đ��M���Ԍo�ߊm�F
		if(MKY43.REG.MSCR.BIT.ERR != 0){				//���[�����M�G���[����
			if(MKY43.REG.MESR.BIT.NORDY != 0){			//���M��̎�M�o�b�t�@����M���łȂ�
				MKY43.REG.MESR.WORD = 0;				//���[�����M�G���[�X�e�[�^�X�N���A

				IntPrioritySet(INT_TIMER1A, 5 << 5);		//�ꎞ�I�ɗD�揇�ʂ��グ��i��M�������荞�݁i�����e�i���X�|�[�g�j�j
				util_delay(SVD[0].cunet_delay);		//�đ��ҋ@
				IntPrioritySet(INT_TIMER1A, 6 << 5);		//�D�揇�ʂ����ɖ߂�

				goto RESEND;							//�đ�����
			}else{										//���̑��̃��[�����M�G���[�̏ꍇ
				if(send_retry < MES_RESEND_MAX){
					send_retry++;						//���[�����M���g���C�񐔍X�V
					MKY43.REG.MESR.WORD = 0;			//���[�����M�G���[�X�e�[�^�X�N���A

					IntPrioritySet(INT_TIMER1A, 5 << 5);	//�ꎞ�I�ɗD�揇�ʂ��グ��i��M�������荞�݁i�����e�i���X�|�[�g�j�j
					util_delay(SVD[0].cunet_delay);	//�đ��ҋ@
					IntPrioritySet(INT_TIMER1A, 6 << 5);	//�D�揇�ʂ����ɖ߂�

					goto RESEND;						//�đ�����
				}
			}
		}
	}
}

/****************************************************/
/* Function : mky43_mail_status                    */
/* Summary  : ���[�����M�G���[�X�e�[�^�X    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : CUnet�G���[���o�AGM�G���A���X�V              */
/****************************************************/
void	mky43_mail_status(void){

	short 	ch;
	short		sa_flow_buf;

	sa_flow_buf = sa_flow;						//���ʌv�X�e�[�V�����A�h���X�擾

	if(MKY43.REG.MSCR.BIT.ERR != 0){			//���[�����M�G���[����
		if(MKY43.REG.MESR.BIT.NORDY != 0){		//���M��̎�M�o�b�t�@����M���łȂ�
			send_err_status |= MAIL_ERR_NORDY;
		}
		if(MKY43.REG.MESR.BIT.NOEX != 0){		//���M���CUnet�X�e�[�V���������݂��Ȃ�
			send_err_status |= MAIL_ERR_NOEX;
		}
		if(MKY43.REG.MESR.BIT.TOUT != 0){		//�ݒ�T�C�N���񐔂��o�߂��Ă����[�����M���������Ȃ�
			send_err_status |= MAIL_ERR_TOUT;
		}
		if(MKY43.REG.MESR.BIT.SZFLT != 0){		//���[�����M�T�C�Y���s���l
			send_err_status |= MAIL_ERR_SZFLT;
		}
		if(MKY43.REG.MESR.BIT.LMFLT != 0){		//MSLR�̐ݒ�l���s���l
			send_err_status |= MAIL_ERR_LMFLT;
		}
		if(MKY43.REG.MESR.BIT.STOP != 0){		//�l�b�g���[�N����~
			send_err_status |= MAIL_ERR_STOP;
		}
		if((send_err_status & ERR_CUNET_MAIL) != 0){		
			for(ch = CH1; ch < CH_NUMMAX; ch++){
				MES[ch].err_status |= ERR_JUDGE_CUNET;		//CUnet�G���[�Z�b�g
			}
			cunet_error = send_err_status;					//�G���[�v�������O�ɕۑ�
		}
	}else{
		send_err_status = 0x0000;				//���[�����M�G���[�X�e�[�^�X���N���A
	}

	/*���[�����M�G���[�X�e�[�^�X��GM�G���A�ɏ�����*/
	MKY43.GM.SA[sa_flow_buf+2].DATA[3].DATA = (send_err_status<<8) & 0xFF00;

}

/****************************************************/
/* Function : mky43_read_data                    */
/* Summary  : GM�G���A�f�[�^�f�[�^�Ǎ���    				*/
/* Argument : flow_no              	               */
/* Return   : GM�G���A�f�[�^ 									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
unsigned long	mky43_read_data(short flow_no){

	unsigned long read_data;

	read_data = 0;
	
	switch(flow_no){
		case	SA_FLOW01:				//���ʌv01
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW02:				//���ʌv02
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[1].DATA & 0x003F;
				break;
		case	SA_FLOW03:				//���ʌv03
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW04:				//���ʌv04
				read_data = (long)MKY43.GM.SA[SA_HOST].DATA[3].DATA & 0x003F;
				break;
		case	SA_FLOW05:				//���ʌv05
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW06:				//���ʌv06
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[1].DATA & 0x003F;
				break;
		case	SA_FLOW07:				//���ʌv07
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW08:				//���ʌv08
				read_data = (long)MKY43.GM.SA[SA_HOST+1].DATA[3].DATA & 0x003F;
				break;
		case	SA_FLOW09:				//���ʌv09
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW10:				//���ʌv10
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[1].DATA & 0x003F;
				break;
//CUnet 16ch
		case	SA_FLOW11:				//���ʌv11
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW12:				//���ʌv12
				read_data = (long)MKY43.GM.SA[SA_HOST+2].DATA[3].DATA & 0x003F;
				break;
		case	SA_FLOW13:				//���ʌv13
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[0].DATA & 0x003F;
				break;
		case	SA_FLOW14:				//���ʌv14
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[1].DATA & 0x003F;
				break;
		case	SA_FLOW15:				//���ʌv15
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[2].DATA & 0x003F;
				break;
		case	SA_FLOW16:				//���ʌv16
				read_data = (long)MKY43.GM.SA[SA_HOST+3].DATA[3].DATA & 0x003F;
				break;
		default:
				break;
	}
	return(read_data);

}

/****************************************************/
/* Function : mky43_read_data2                    */
/* Summary  : GM�G���A�f�[�^�f�[�^�Ǎ��݁i�ώZ�Ď��@�\�j   				*/
/* Argument : flow_no              	               */
/* Return   : GM�G���A�f�[�^ 									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
unsigned long	mky43_read_data2(short flow_no){

	unsigned long read_data;

	read_data = 0;
	
	switch(flow_no){
		case	SA_FLOW01:				//���ʌv01
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW02:				//���ʌv02
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[1].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW03:				//���ʌv03
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW04:				//���ʌv04
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[3].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW05:				//���ʌv05
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW06:				//���ʌv06
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[1].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW07:				//���ʌv07
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW08:				//���ʌv08
				read_data = (long)((MKY43.GM.SA[SA_HOST+1].DATA[3].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW09:				//���ʌv09
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW10:				//���ʌv10
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[1].DATA & 0x3F00) >> 8);
				break;
//CUnet 16ch
		case	SA_FLOW11:				//���ʌv11
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW12:				//���ʌv12
				read_data = (long)((MKY43.GM.SA[SA_HOST+2].DATA[3].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW13:				//���ʌv13
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[0].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW14:				//���ʌv14
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[1].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW15:				//���ʌv15
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[2].DATA & 0x3F00) >> 8);
				break;
		case	SA_FLOW16:				//���ʌv16
				read_data = (long)((MKY43.GM.SA[SA_HOST+3].DATA[3].DATA & 0x3F00) >> 8);
				break;
		default:
				read_data = (long)((MKY43.GM.SA[SA_HOST].DATA[0].DATA & 0x3F00) >> 8);
				break;
	}
	return(read_data);

}

/****************************************************/
/* Function : mky43_write_alarm                    */
/* Summary  : �G���[�X�e�[�^�X������   				*/
/* Argument : ch                  	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
void	mky43_write_alarm(short ch){

	short		sa_flow_buf;
	short		ch_num;

	if(ch >= CH_NUMMAX){
		return;
	}
	
	sa_flow_buf = sa_flow;				//���ʌv�X�e�[�V�����A�h���X�擾
	ch_num = ch;
	
	switch(ch_num){
		case CH1:
		case CH2:
			//�A���[���X�e�[�^�X������
			MKY43.GM.SA[sa_flow_buf].DATA[0].DATA = (MAIN[0].err_condition_cu) + (MAIN[1].err_condition_cu<<8); 	//CH1+CH2
			break;
		case CH3:
		case CH4:
			//�A���[���X�e�[�^�X������
			MKY43.GM.SA[sa_flow_buf].DATA[1].DATA = (MAIN[2].err_condition_cu) + (MAIN[3].err_condition_cu<<8); 	//CH3+CH4
			break;
		case CH5:
		case CH6:
			//�A���[���X�e�[�^�X������
			MKY43.GM.SA[sa_flow_buf].DATA[2].DATA = (MAIN[4].err_condition_cu) + (MAIN[5].err_condition_cu<<8); 	//CH5+CH6
			break;
		default:
			break;
	}
}

/****************************************************/
/* Function : mky43_write_flow                    */
/* Summary  : �u�����ʃf�[�^������   				*/
/* Argument : ch                  	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
void	mky43_write_flow(short ch){

	long	flow_GM_buf;
	short		ch_num;
	unsigned short	mb_num;

	if(ch >= CH_NUMMAX){
		return;
	}
	
	ch_num = ch;
	mb_num = sa_flow;
	
	switch(ch_num){
		case CH1:
		case CH2:
		case CH3:
		case CH4:
			//�u�����ʏ�����
			//���C�g�n�U�[�h�h�~�@�\���g�p���āAMB�ֈꊇ������
			flow_GM_buf = flow_fs_cal(CH1);
			MKY43.REG.WRHPB0.DATA[0].DATA = (flow_GM_buf & 0x0000ffff);	//CH1�u������
			flow_GM_buf = flow_fs_cal(CH2);
			MKY43.REG.WRHPB0.DATA[1].DATA = (flow_GM_buf & 0x0000ffff);	//CH2�u������
			flow_GM_buf = flow_fs_cal(CH3);
			MKY43.REG.WRHPB0.DATA[2].DATA = (flow_GM_buf & 0x0000ffff);	//CH3�u������
			flow_GM_buf = flow_fs_cal(CH4);
			MKY43.REG.WRHPB0.DATA[3].DATA = (flow_GM_buf & 0x0000ffff);	//CH4�u������
			MKY43.REG.WHCR0.WORD = (mb_num + 1);							//MB�ֈꊇ������
			break;
		case CH5:
		case CH6:
			//�u�����ʏ�����
			//���C�g�n�U�[�h�h�~�@�\���g�p���āAMB�ֈꊇ������
			flow_GM_buf = flow_fs_cal(CH5);
			MKY43.REG.WRHPB0.DATA[0].DATA = (flow_GM_buf & 0x0000ffff);	//CH5�u������
			flow_GM_buf = flow_fs_cal(CH6);
			MKY43.REG.WRHPB0.DATA[1].DATA = (flow_GM_buf & 0x0000ffff);	//CH6�u������
			MKY43.REG.WRHPB0.DATA[2].DATA = 0;								//���g�p�̈�
			MKY43.REG.WRHPB0.DATA[3].DATA = (send_err_status<<8) & 0xFF00;//���[�����M�G���[�X�e�[�^�X
			MKY43.REG.WHCR0.WORD = (mb_num + 2);							//MB�ֈꊇ������
			break;
		default:
			break;
	}
}

/****************************************************/
/* Function : flow_fs_cal                           */
/* Summary  : �u�����ʃf�[�^�ϊ�����   				             */
/* Argument : ch                  	                 */
/* Return   : �Ȃ�      									                    */
/* Caution  : �Ȃ�                                    */
/* notes    : �t���X�P�[���ݒ�ɂ����GM�֏����ޏu�����ʃf�[�^��ϊ�����*/
/****************************************************/
long	flow_fs_cal(short ch){

	long	flow;
	
	flow = (long)MES[ch].ml_min_now;	//�u�����ʒl
	
	if (SVD[ch].max_flow <= 500){
		flow /= 1;	//FS �`500ml/min�F�u�����ʒl*100
		if (flow > SVD[ch].max_flow * 1.1 * 100)	flow = SVD[ch].max_flow * 1.1 * 100;	//110%over
	}
	else if (SVD[ch].max_flow > 500 && SVD[ch].max_flow <= 5000){	
		flow /= 10;	//FS 501�`5000ml/min�F�u�����ʒl*10
		if (flow > SVD[ch].max_flow * 1.1 * 10)	flow = SVD[ch].max_flow * 1.1 * 10;		//110%over
	}
	else if (SVD[ch].max_flow > 5000 && SVD[ch].max_flow <= 50000){
		flow /= 100;	//FS 5001�`50000ml/min�F�u�����ʒl*1
		if (flow > SVD[ch].max_flow * 1.1)	flow = SVD[ch].max_flow * 1.1;	//110%over
	}
	else {
		flow /= 1000;	//FS 50001�`100000ml/min�F�u�����ʒl/10
		if (flow > SVD[ch].max_flow * 1.1 / 10)	flow = SVD[ch].max_flow * 1.1 / 10;	//110%over
	}

	if(flow < 0){
		flow = 0;		//�}�C�i�X���́u0�v
	}

	return(flow);
}

/****************************************************/
/* Function : mky43_check_datarenewal               */
/* Summary  : GM�G���A�f�[�^�X�V�m�F   				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
void	mky43_check_datarenewal(void){

	short ch;
	unsigned long read_data, read_data2;

	read_data = mky43_read_data(sa_flow);	//GM�G���A�f�[�^�Ǎ���		
	read_data2 = mky43_read_data2(sa_flow);	//GM�G���A�f�[�^�Ǎ��݁i�ώZ�Ď��@�\�j		

	for(ch = CH1; ch < CH_NUMMAX; ch++){
		if(read_data & 0x0001 == 1){		//�ώZ�J�n�w���m�F
			MES[ch].addit_req = 1;			//�ώZ�J�n
		}else{
			MES[ch].addit_req = 0;			//�ώZ��~
		}
		read_data = (read_data >> 1);

		if(read_data2 & 0x0001 == 1){	//�ώZ�Ď��@�\�m�F
			MES[ch].addit_watch = B_ON;		//�ώZ�Ď��L��
		}else{
			MES[ch].addit_watch = B_OFF;	//�ώZ�Ď�����
		}
		read_data2 = (read_data2 >> 1);
	}
		
	MKY43.REG.INT1CR.BIT.DR = 1;			//�f�[�^�X�V�����݋���
	MKY43.REG.INT1SR.BIT.DR = 1;			//�f�[�^�X�V�����݉���

}

/****************************************************/
/* Function : mky43_get_flow_num               */
/* Summary  : ���ʌv�ԍ��Ǎ���   				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
short	mky43_get_flow_num(void){

	short	dip_sw;
	short	flow_num;
	
	dip_sw = disp_cunet_read();		//DIP-SW�ԍ��擾

	switch(dip_sw){
		case	0:						//DIP-SW�ԍ�:0
				flow_num = SA_FLOW01;	//���ʌv01
				break;
		case	1:						//DIP-SW�ԍ�:1
				flow_num = SA_FLOW02;	//���ʌv02
				break;
		case	2:						//DIP-SW�ԍ�:2
				flow_num = SA_FLOW03;	//���ʌv03
				break;
		case	3:						//DIP-SW�ԍ�:3
				flow_num = SA_FLOW04;	//���ʌv04
				break;
		case	4:						//DIP-SW�ԍ�:4
				flow_num = SA_FLOW05;	//���ʌv05
				break;
		case	5:						//DIP-SW�ԍ�:5
				flow_num = SA_FLOW06;	//���ʌv06
				break;
		case	6:						//DIP-SW�ԍ�:6
				flow_num = SA_FLOW07;	//���ʌv07
				break;
		case	7:						//DIP-SW�ԍ�:7
				flow_num = SA_FLOW08;	//���ʌv08
				break;
		case	8:						//DIP-SW�ԍ�:8
				flow_num = SA_FLOW09;	//���ʌv09
				break;
		case	9:						//DIP-SW�ԍ�:9
				flow_num = SA_FLOW10;	//���ʌv10
				break;
//CUnet 16ch
		case	10:						//DIP-SW�ԍ�:10
				flow_num = SA_FLOW11;	//���ʌv11
				break;
		case	11:						//DIP-SW�ԍ�:11
				flow_num = SA_FLOW12;	//���ʌv12
				break;
		case	12:						//DIP-SW�ԍ�:12
				flow_num = SA_FLOW13;	//���ʌv13
				break;
		case	13:						//DIP-SW�ԍ�:13
				flow_num = SA_FLOW14;	//���ʌv14
				break;
		case	14:						//DIP-SW�ԍ�:14
				flow_num = SA_FLOW15;	//���ʌv15
				break;
		case	15:						//DIP-SW�ԍ�:15
				flow_num = SA_FLOW16;	//���ʌv16
				break;
		default:
				flow_num = SA_FLOW01;	//���ʌv01
				break;
	}
	return(flow_num);

}

/****************************************************/
/* Function : mky43_check_recv0               */
/* Summary  : MKY43�����݊m�F   				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
void	mky43_check_recv0(void){

	if(MKY43.REG.INT0SR.BIT.MR != 0){		//���[����M���������ݔ���
		mky43_mail_recv();					//���[����M(MRB0,MRB1)
	}
}

/****************************************************/
/* Function : mky43_check_recv1               */
/* Summary  : MKY43�����݊m�F   				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
void	mky43_check_recv1(void){

	if(MKY43.REG.INT1SR.BIT.DR != 0){		//GM�G���A�̃f�[�^�X�V���o�����ݔ���
		mky43_check_datarenewal();
	}
}

/****************************************************/
/* Function : mky43_TX_start               */
/* Summary  : ���M�f�[�^��send_data[]�ɕۑ�   				*/
/* Argument : com_mode           	               */
/* Return   : �Ȃ�      									                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                  */
/****************************************************/
void	mky43_TX_start(short com_mode){

	short i;
	short j;
	short cnt;
	short ch_num;
	short send_num;
	short	add_len;
	unsigned short size;
	unsigned short data_len=0;
	char ch[1] = {0};
	
	if(com_mode == SA_HOST){		//�z�X�g(SA=0)�֑��M����ꍇ
		//���M�f�[�^���v�Z
		if((TX_buf_host[1] == 'O' && (TX_buf_host[2] == 'E' || TX_buf_host[2] == 'Q')) || (TX_buf_host[1] == 'o' && TX_buf_host[2] == 'q')){
			ch[0] = TX_buf_host[9];
			ch_num = ch[0] - 0x30;
			if(TX_buf_host[2] == 'E'){			//'OE'�R�}���h
				data_len = 13 + 9*ch_num;
			}else{ 							//'OQ','oq'�R�}���h	
				data_len = 13 + 72*ch_num;
			}
		}else{
			data_len = strlen(TX_buf_host);
		}
	
		add_len = mky43_add_message(data_len, com_mode);	//�ʐM���b�Z�[�W�ɁwCH�f�[�^�x�wLength�t�B�[���h�x������
		data_len += add_len;						//�f�[�^���X�V
	
		if(data_len%256 == 0){
			send_num = (data_len/256);				//�f�[�^���M��
		}else{
			send_num = (data_len/256) + 1;			//�f�[�^���M��
		}

		for(cnt=0; cnt<send_num; cnt++){
			memset(send_data, 0, sizeof(send_data));
			if(cnt == (send_num - 1)){				//�ŏI���M��
				if(data_len%8 == 0){
					size = (data_len/8);			//���M�T�C�Y
				}else{
					size = (data_len/8) + 1;		//���M�T�C�Y
				}
			}else{
				size = 32;							//���M�T�C�Y
			}

			for(i=0; i<size; i++){				//���M�o�b�t�@�̃f�[�^�����[�����M�p�ɃR�s�[
				for(j=0; j<4; j++){
					send_data[i][j] = TX_buf_host[8*i + 2*j + (256*cnt+1)]*0x0100 + TX_buf_host[8*i + 2*j + (256*cnt)];
				}
			}
			mky43_mail_send(size, com_mode);		//���b�Z�[�W���M����
			data_len -= 256;						//���M�f�[�^�T�C�Y�X�V
		}
		for (i=0; i<MSG_MAX; i++){
			TX_buf_host[i] = 0;						/*���M�o�b�t�@�N���A*/
		}
	}else{						//�z�X�g�ȊO��SA(Station Address)�֑��M����ꍇ
		//���M�f�[�^���v�Z
		if((TX_buf_subhost[1] == 'O' && (TX_buf_subhost[2] == 'E' || TX_buf_subhost[2] == 'Q')) || (TX_buf_subhost[1] == 'o' && TX_buf_subhost[2] == 'q')){
			ch[0] = TX_buf_subhost[9];
			ch_num = ch[0] - 0x30;
			if(TX_buf_subhost[2] == 'E'){		//'OE'�R�}���h
				data_len = 13 + 9*ch_num;
			}else{ 							//'OQ','oq'�R�}���h	
				data_len = 13 + 72*ch_num;
			}
		}else{
			data_len = strlen(TX_buf_subhost);
		}
	
		add_len = mky43_add_message(data_len, com_mode);		//�ʐM���b�Z�[�W�ɁwCH�f�[�^�x�wLength�t�B�[���h�x������
		data_len += add_len;						//�f�[�^���X�V
		
		if(data_len%256 == 0){
			send_num = (data_len/256);				//�f�[�^���M��
		}else{
			send_num = (data_len/256) + 1;			//�f�[�^���M��
		}

		for(cnt=0; cnt<send_num; cnt++){
			memset(send_data_sub, 0, sizeof(send_data_sub));
			if(cnt == (send_num - 1)){				//�ŏI���M��
				if(data_len%8 == 0){
					size = (data_len/8);			//���M�T�C�Y
				}else{
					size = (data_len/8) + 1;		//���M�T�C�Y
				}
			}else{
				size = 32;							//���M�T�C�Y
			}

			for(i=0; i<size; i++){				//���M�o�b�t�@�̃f�[�^�����[�����M�p�ɃR�s�[
				for(j=0; j<4; j++){
					send_data_sub[i][j] = TX_buf_subhost[8*i + 2*j + (256*cnt+1)]*0x0100 + TX_buf_subhost[8*i + 2*j + (256*cnt)];
				}
			}
			mky43_mail_send(size, com_mode);		//���b�Z�[�W���M����
			data_len -= 256;						//���M�f�[�^�T�C�Y�X�V
		}
		for (i=0; i<MSG_MAX; i++){
			TX_buf_subhost[i] = 0;					/*���M�o�b�t�@�N���A*/
		}
	}
}

/****************************************************/
/* Function : mky43_rxbuf_save                      */
/* Summary  : �擾�f�[�^��RX_buf_host[],RX_buf_subhost[]�ɕۑ�*/
/* Argument : buf_side,  recv_size,  recv_sa        */
/* Return   : �Ȃ�      									                    */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_rxbuf_save(short buf_side, unsigned short recv_size, unsigned short recv_sa){

	short i;
	short j;
	short buf_num;
	short buf_offset;
	short data_len;
	short data_offset;
	
	if(recv_sa == SA_HOST){		//�z�X�g(SA=0)����̎擾�f�[�^
		buf_offset = buf_count;
		if(buf_offset == 0){
			for (i=0; i<MSG_MAX; i++){
				RX_buf_host[i] = 0;
			}	
		}
		recv_sa_host = recv_sa;
		recv_size *= 8;
		for(buf_num=0; buf_num<recv_size; buf_num++){
			i = buf_num/8;
			j = (buf_num%8)/2;	
			if(buf_num%2 == 0){
				if(buf_side == 0){		//recv_data0[]��ۑ�
					RX_buf_host[buf_num+buf_offset] = recv_data0[i][j]%0x0100;
				}else{					//recv_data1[]��ۑ�
					RX_buf_host[buf_num+buf_offset] = recv_data1[i][j]%0x0100;
				}
			}else{
				if(buf_side == 0){		//recv_data0[]��ۑ�
					RX_buf_host[buf_num+buf_offset] = recv_data0[i][j]/0x0100;
				}else{					//recv_data1[]��ۑ�
					RX_buf_host[buf_num+buf_offset] = recv_data1[i][j]/0x0100;
				}
			}
			if(RX_buf_host[buf_num+buf_offset] != CR 	/*CR(0x0D)�Ȃ�*/
							|| (buf_num == 1)){		/*�f�[�^�����i�[����Ă���z��͖�������(�f�[�^����13(0x0D)�̏ꍇ������̂�)*/
				buf_count++;
			}else{							//CR(0x0D)����
				buf_count = 0;
				break;
			}
			if((buf_num+buf_offset) > MSG_MAX){
				break;
			}
		}
	
		if(buf_count == 0){		//�S���b�Z�[�W(CR�܂ł̃f�[�^)��M����		
			/*�t�H�[�}�b�g����*/
			data_offset = 0;
			format_err_host = 0;
			while(RX_buf_host[data_offset]!= HEADER_CH){	//CH�f�[�^����
				data_offset++;
				if(data_offset >= MSG_MAX)	break;
			}		
			if(RX_buf_host[data_offset + 2] != 0x40)	format_err_host++;
		
			/*Header CH ����*/
			if(RX_buf_host[MES_1ST_TOP + data_offset] != HEADER_CH){
				return;			// Header CH �ُ�
			}
			
			/*�I�[��������*/
			data_len = 2;
			while(RX_buf_host[data_len + data_offset]!= CR){	//@���猟�����J�n����
				data_len++;
				if((data_len + data_offset) == 257 || (data_len + data_offset) == 513){/*�f�[�^�����i�[����Ă���z��͖�������(�f�[�^����13(0x0D)�̏ꍇ������̂�)*/
					data_len++;
				}
			}
			data_len++;				/*data_len = CH,Length�� ���b�Z�[�W�S��M�f�[�^*/
			if(data_len < 0) data_len = 0;

			mky43_del_message(data_len, data_offset, recv_sa);	//�ʐM���b�Z�[�W����wCH�f�[�^�x�wLength�t�B�[���h�x���폜����

			protocol_timer_host(1);		//�ʐM�v���g�R����͊J�n�^�C�}�[�Z�b�g	
		}
	}else{				//�z�X�g�ȊO��SA(Station Address)����̎擾�f�[�^
		buf_offset = buf_count_sub;
		if(buf_offset == 0){
			for (i=0; i<MSG_MAX; i++){
				RX_buf_subhost[i] = 0;
			}	
		}
		recv_sa_sub = recv_sa;
		recv_size *= 8;
		for(buf_num=0; buf_num<recv_size; buf_num++){
			i = buf_num/8;
			j = (buf_num%8)/2;	
			if(buf_num%2 == 0){
				if(buf_side == 0){		//recv_data0[]��ۑ�
					RX_buf_subhost[buf_num+buf_offset] = recv_data0[i][j]%0x0100;
				}else{					//recv_data1[]��ۑ�
					RX_buf_subhost[buf_num+buf_offset] = recv_data1[i][j]%0x0100;
				}
			}else{
				if(buf_side == 0){		//recv_data0[]��ۑ�
					RX_buf_subhost[buf_num+buf_offset] = recv_data0[i][j]/0x0100;
				}else{					//recv_data1[]��ۑ�
					RX_buf_subhost[buf_num+buf_offset] = recv_data1[i][j]/0x0100;
				}
			}
			if(RX_buf_subhost[buf_num+buf_offset] != CR	/*CR(0x0D)�Ȃ�*/
						|| (buf_num == 1)){				/*�f�[�^�����i�[����Ă���z��͖�������(�f�[�^����13(0x0D)�̏ꍇ������̂�)*/		
				buf_count_sub++;
			}else{							//CR(0x0D)����
				buf_count_sub = 0;
				break;
			}
			if((buf_num+buf_offset) > MSG_MAX){
				break;
			}
		}
	
		if(buf_count_sub == 0){		//�S���b�Z�[�W(CR�܂ł̃f�[�^)��M����
			/*�t�H�[�}�b�g����*/
			data_offset = 0;
			format_err_sub = 0;
			while(RX_buf_subhost[data_offset]!= HEADER_CH){	//CH�f�[�^����
				data_offset++;
				if(data_offset >= MSG_MAX)	break;
			}		
			if(RX_buf_subhost[data_offset + 2] != 0x40)	format_err_sub++;
			
			/*Header CH ����*/
			if(RX_buf_subhost[MES_1ST_TOP] != HEADER_CH){
				return;			// Header CH �ُ�
			}

			/*�I�[��������*/
			data_len = 2;
			while(RX_buf_subhost[data_len + data_offset]!= CR){		//@���猟�����J�n����
				data_len++;
				if((data_len + data_offset) == 257 || (data_len + data_offset) == 513){/*�f�[�^�����i�[����Ă���z��͖�������(�f�[�^����13(0x0D)�̏ꍇ������̂�)*/
					data_len++;
				}
			}
			data_len++;				/*data_len = CH,Length�� ���b�Z�[�W�S��M�f�[�^*/
			if (data_len < 0) data_len = 0;

			mky43_del_message(data_len, data_offset, recv_sa);	//�ʐM���b�Z�[�W����wCH�f�[�^�x�wLength�t�B�[���h�x���폜����
			
			protocol_timer_subhost();		//�ʐM�v���g�R����͊J�n�^�C�}�[�Z�b�g	
		}
	}
}

/****************************************************/
/* Function : mky43_host_link_check                */
/* Summary  : �z�X�g�ʐM�����N�m�F                       */
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ�      									                    */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	mky43_host_link_check(void){

	if(B_YES == mky43_ccr_check() &&
		(MKY43.REG.SCR.BIT.RUN != 0)){			//RUN�t�F�[�Y�m�F
		if(com_link == B_NG){					//�����N�s�������畜�A�����ꍇ
			mky43_set_register();				//MKY43�̃��W�X�^�ݒ�
			com_link = B_OK;
		}
		cunet_mcare = MKY43.REG.CCTR.WORD;		//LCARE,MCARE�M�������񐔂�ۑ�
		
		cunet_mfr4 = MKY43.REG.MFR.DATA[0].DATA;
		cunet_mfr3 = MKY43.REG.MFR.DATA[1].DATA;
		cunet_mfr2 = MKY43.REG.MFR.DATA[2].DATA;
		cunet_mfr1 = MKY43.REG.MFR.DATA[3].DATA;
		
	}else{
		if((com_link != B_NG) && 				//�z�X�g�Ƃ̒ʐM�������N�s����
			(MKY43.REG.MFR.DATA[0].DATA & 0x0001) == 0){  	
			mky43_restart();					//MKY43�̒ʐM�ĊJ
			com_link = B_NG;
		}
	}
}

/****************************************************/
/* Function : mky43_ccr_check                      */
/* Summary  : RS485 or CUnet��m�F                */
/* Argument : �Ȃ�                                  */
/* Return   : �Ȃ�      									                   */
/* Caution  : �Ȃ�                                   */
/* notes    : CCR���W�X�^����[MKY43_v0]���Ǎ��߂��ꍇ�ɂ́ACUnet��Ƃ���B*/
/*          : �Ǎ��߂Ȃ��ꍇ�ɂ́ARS485��Ƃ���B         */
/****************************************************/
short		mky43_ccr_check(void){

	short ret;

	ret = B_NO;
	
	if(	MKY43.REG.CCR.DATA[0].DATA == 0x4B4D &&
		MKY43.REG.CCR.DATA[1].DATA == 0x3459 &&
		MKY43.REG.CCR.DATA[2].DATA == 0x5F33 	){
		com_type = COM_CUNET;		//CUnet���
		ret = B_YES;
	}
	else{
		com_type = COM_RS485;		//RS485���
		ret = B_NO;
	}
	
	return (ret);
}

/****************************************************/
/* Function : mky43_add_message                     */
/* Summary  : �ʐM���b�Z�[�W�̃f�[�^�ǉ�              				 */
/* Argument : len,   com_mode      	                */
/* Return   : �ǉ��f�[�^��				                         */
/* Caution  : �Ȃ�                                   */
/* notes    :                                       */
/*�@notes    : �擪�A�h���X(1byte)�ɁwCH�f�[�^(1�Œ�)�x������ */
/*�@          ���A�h���X(1byte)�ɁwLength�t�B�[���h�x������				*/
/*�@          ���b�Z�[�W�������ɂ́A���ꂼ��̐擪�ɁwCH�f�[�^�x�A		 */
/*           �wLength�t�B�[���h�x������									          */
/*        	  Length�t�B�[���h�F�u@�`CR�v�̒���				           */
/*�@         ��CUnet�ʐM���A�z�X�g�Ԃ̃��b�Z�[�W���Ɍ���					   */
/****************************************************/
short		mky43_add_message(unsigned short len, short com_mode){

	short cnt;
	short add_len=0;

	if(com_mode == SA_HOST){		//�z�X�g(SA=0)�֑��M����ꍇ
		/*254byte(256-2)�ȉ��i���b�Z�[�W�𕪊����Ȃ��ő��M����ꍇ�j*/
		if(len <= 254){
			/*���M���郁�b�Z�[�W���쐬*/
			for(cnt=len-1; cnt>=0; cnt--){
				TX_buf_host[cnt+2] = TX_buf_host[cnt];		//2byte�V�t�g
			}
			TX_buf_host[MES_1ST_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_host[MES_1ST_TOP+1] = len;			//Length�t�B�[���h
			add_len = 2;								//�ǉ��f�[�^��
		}
		/*255byte�`508byte�i���b�Z�[�W��2�������đ��M����ꍇ�j*/
		else if(len > 254 && len <= 508){
			/*2��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=len-1; cnt>=254; cnt--){
				TX_buf_host[cnt+4] = TX_buf_host[cnt];		//4byte�V�t�g
			}
			TX_buf_host[MES_2ND_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_host[MES_2ND_TOP+1] = (len-254);	//Length�t�B�[���h

			/*1��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_host[cnt+2] = TX_buf_host[cnt];		//2byte�V�t�g
			}
			TX_buf_host[MES_1ST_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_host[MES_1ST_TOP+1] = 254;			//Length�t�B�[���h
			add_len = 4;								//�ǉ��f�[�^��
		}
		/*509byte�ȏ�i���b�Z�[�W��3�������đ��M����ꍇ�j*/
		else{
			/*3��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=len-1; cnt>=508; cnt--){
				TX_buf_host[cnt+6] = TX_buf_host[cnt];		//6byte�V�t�g
			}
			TX_buf_host[MES_3RD_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_host[MES_3RD_TOP+1] = (len-508);	//Length�t�B�[���h
		
			/*2��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=508; cnt>=254; cnt--){
				TX_buf_host[cnt+4] = TX_buf_host[cnt];		//4byte�V�t�g
			}
			TX_buf_host[MES_2ND_TOP] = HEADER_CH;			//CH�f�[�^
			TX_buf_host[MES_2ND_TOP+1] = 254;				//Length�t�B�[���h

			/*1��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_host[cnt+2] = TX_buf_host[cnt];		//2byte�V�t�g
			}
			TX_buf_host[MES_1ST_TOP] = HEADER_CH;			//CH�f�[�^
			TX_buf_host[MES_1ST_TOP+1] = 254;				//Length�t�B�[���h
			add_len = 6;									//�ǉ��f�[�^��
		}
	}else{						//�z�X�g�ȊO��SA(Station Address)�֑��M����ꍇ
		/*254byte(256-2)�ȉ��i���b�Z�[�W�𕪊����Ȃ��ő��M����ꍇ�j*/
		if(len <= 254){
			/*���M���郁�b�Z�[�W���쐬*/
			for(cnt=len-1; cnt>=0; cnt--){
				TX_buf_subhost[cnt+2] = TX_buf_subhost[cnt];	//2byte�V�t�g
			}
			TX_buf_subhost[MES_1ST_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_subhost[MES_1ST_TOP+1] = len;			//Length�t�B�[���h
			add_len = 2;									//�ǉ��f�[�^��
		}
		/*255byte�`508byte�i���b�Z�[�W��2�������đ��M����ꍇ�j*/
		else if(len > 254 && len <= 508){
			/*2��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=len-1; cnt>=254; cnt--){
				TX_buf_subhost[cnt+4] = TX_buf_subhost[cnt];	//4byte�V�t�g
			}
			TX_buf_subhost[MES_2ND_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_subhost[MES_2ND_TOP+1] = (len-254);		//Length�t�B�[���h

			/*1��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_subhost[cnt+2] = TX_buf_subhost[cnt];	//2byte�V�t�g
			}
			TX_buf_subhost[MES_1ST_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_subhost[MES_1ST_TOP+1] = 254;			//Length�t�B�[���h
			add_len = 4;									//�ǉ��f�[�^��
		}
		/*509byte�ȏ�i���b�Z�[�W��3�������đ��M����ꍇ�j*/
		else{
			/*3��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=len-1; cnt>=508; cnt--){
				TX_buf_subhost[cnt+6] = TX_buf_subhost[cnt];	//6byte�V�t�g
			}
			TX_buf_subhost[MES_3RD_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_subhost[MES_3RD_TOP+1] = (len-508);		//Length�t�B�[���h
		
			/*2��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=508; cnt>=254; cnt--){
				TX_buf_subhost[cnt+4] = TX_buf_subhost[cnt];	//4byte�V�t�g
			}
			TX_buf_subhost[MES_2ND_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_subhost[MES_2ND_TOP+1] = 254;			//Length�t�B�[���h

			/*1��ڂɑ��M���郁�b�Z�[�W���쐬*/
			for(cnt=253; cnt>=0; cnt--){
				TX_buf_subhost[cnt+2] = TX_buf_subhost[cnt];	//2byte�V�t�g
			}
			TX_buf_subhost[MES_1ST_TOP] = HEADER_CH;		//CH�f�[�^
			TX_buf_subhost[MES_1ST_TOP+1] = 254;			//Length�t�B�[���h
			add_len = 6;									//�ǉ��f�[�^��
		}
	}
	return add_len;
}

/****************************************************/
/* Function : mky43_del_message                     */
/* Summary  : �ʐM���b�Z�[�W�̃f�[�^�폜              	 			*/
/* Argument : len,  offset,  com_mode               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/*�@notes    : �擪�A�h���X(1byte)�́wCH�f�[�^(1�Œ�)�x���폜����*/
/*�@           ���A�h���X(1byte)�́wLength�t�B�[���h�x���폜����	*/
/*�@           ���b�Z�[�W�������ɂ́A���ꂼ��̐擪�́wCH�f�[�^�x�A		*/
/*            �wLength�t�B�[���h�x���폜����								         */
/*�@           ��CUnet�ʐM���A�z�X�g����̃��b�Z�[�W���Ɍ���				 */
/****************************************************/
void	mky43_del_message(unsigned short len, unsigned short offset, short com_mode){

	short cnt;

	if(com_mode == SA_HOST){		//�z�X�g(SA=0)�֑��M����ꍇ
		len_err_host = 0;

		/*256byte�ȉ��i�������Ă��Ȃ����b�Z�[�W����M�����ꍇ�j*/
		if(len <= 256){
			/*���b�Z�[�W������*/
			if(RX_buf_host[(MES_1ST_TOP + 1) + offset] != (len-2)){
				len_err_host++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=0; cnt<(len-2); cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 2) + offset];
			}
			RX_buf_host[len-2] = 0;
		}
		/*257byte�`512byte�i2�����������b�Z�[�W����M�����ꍇ�j*/
		else if(len > 256 && len <= 512){
			/*1��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_host[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_host++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 2) + offset];
			}
			
			/*2��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_host[(MES_2ND_TOP + 1) + offset] != (len-258)){
				len_err_host++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=254; cnt<(len-4); cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 4) + offset];
			}
			RX_buf_host[len-4] = 0;
		}
		/*513byte�ȏ�i3�����������b�Z�[�W����M�����ꍇ�j*/
		else{
			/*1��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_host[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_host++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 2) + offset];
			}
			
			/*2��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_host[(MES_2ND_TOP + 1) + offset] != 254){
				len_err_host++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=254; cnt<508; cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 4) + offset];
			}			
			
			/*3��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_host[(MES_3RD_TOP + 1) + offset] != (len-514)){
				len_err_host++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=509; cnt<(len-6); cnt++){
				RX_buf_host[cnt] = RX_buf_host[(cnt + 6) + offset];
			}
			RX_buf_host[len-6] = 0;
		}
	}else{						//�z�X�g�ȊO��SA(Station Address)�֑��M����ꍇ
		len_err_sub = 0;

		/*256byte�ȉ��i�������Ă��Ȃ����b�Z�[�W����M�����ꍇ�j*/
		if(len <= 256){
			/*���b�Z�[�W������*/
			if(RX_buf_subhost[(MES_1ST_TOP + 1) + offset] != (len-2)){
				len_err_sub++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=0; cnt<(len-2); cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 2) + offset];
			}
			RX_buf_subhost[len-2] = 0;
		}
		/*257byte�`512byte�i2�����������b�Z�[�W����M�����ꍇ�j*/
		else if(len > 256 && len <= 512){
			/*1��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_subhost[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_sub++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 2) + offset];
			}
			
			/*2��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_subhost[(MES_2ND_TOP + 1) + offset] != (len-258)){
				len_err_sub++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=254; cnt<(len-4); cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 4) + offset];
			}
			RX_buf_subhost[len-4] = 0;
		}
		/*513byte�ȏ�i3�����������b�Z�[�W����M�����ꍇ�j*/
		else{
			/*1��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_subhost[(MES_1ST_TOP + 1) + offset] != 254){
				len_err_sub++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=0; cnt<254; cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 2) + offset];
			}
			
			/*2��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_subhost[(MES_2ND_TOP + 1) + offset] != 254){
				len_err_sub++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=254; cnt<508; cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 4) + offset];
			}			
			
			/*3��ڂɎ�M�������b�Z�[�W*/
			/*���b�Z�[�W������*/
			if(RX_buf_subhost[(MES_3RD_TOP + 1) + offset] != (len-514)){
				len_err_sub++;			//���b�Z�[�W�� �ُ�
			}
			/*�wCH�f�[�^�x�A�wLength�t�B�[���h�x���폜*/
			for(cnt=509; cnt<(len-6); cnt++){
				RX_buf_subhost[cnt] = RX_buf_subhost[(cnt + 6) + offset];
			}
			RX_buf_subhost[len-6] = 0;
		}
	}
}
