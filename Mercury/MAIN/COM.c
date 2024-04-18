/***********************************************/
/* File Name : COM.c    		         									   */
/*	Summary   : �ʐM����  				                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include <machine.h>
#include <string.h>	

#include "define.h"
#include "SV_def.h"
#include "defSAVE.h"
#include "defMES.h"
#include "defMAIN.h"
#include "ctlioport.h"
#include "version.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"

/********************************************************/
/*	���W���[������`�֐�								*/
/********************************************************/
void	com_init(void);
void	com_init_host(void);
void	com_init_ment(void);
void abort_com(void);
void abort_com_host(void);
void abort_com_ment(void);
void	int_com_host(void);
void	int_com_ment(void);
void	TX_start_host(short byte_num);
void DMA2_end();
void TX_end_host();
void	TX_start_ment(short byte_num);
void DMA1_end();
void TX_end_ment();
void	RX_enable_host(void);
void	RX_enable_ment(void);
short ascii_to_bin(short byte_num);
short	bin_to_ascii(short input);
void	com_cont_host(void);
void	com_cont_subhost(void);
void	com_cont_ment(void);
void protocol_timer_ment(unsigned short value);
void protocol_timer_host(unsigned short value);
void protocol_timer_subhost(void);
void	int_cmt1(void);
void	int_cmt3(void);
void	int_mtu0(void);

/********************************************************/
/*	���W���[���O��`�֐�								*/
/********************************************************/
extern void protocol(short);
extern void delay(unsigned short delay_count);

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern short  com_type;
extern char TX_buf[MSG_NUM][MSG_MAX];
extern char RX_buf[MSG_NUM][MSG_MAX];
extern uint32_t g_ui32SysClock;

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/
char *RX_buf_host = &(RX_buf[HOST][0]);
char *TX_buf_host = &(TX_buf[HOST][0]);
char *RX_buf_ment = &(RX_buf[MENT][0]);
char *TX_buf_ment = &(TX_buf[MENT][0]);
char *RX_buf_subhost = &(RX_buf[SUB_HOST][0]);
char *TX_buf_subhost = &(TX_buf[SUB_HOST][0]);

short byte_count;			/*��M�o�C�g�J�E���^*/
short byte_count_host;		/*��M�o�C�g�J�E���^*/
short byte_count_ment;		/*��M�o�C�g�J�E���^*/
short err_stat;			/*�G���[�X�e�C�^�X*/
short rx_end_host;		/*��M�����t���O*/
short rx_end_subhost;		/*��M�����t���O*/
short rx_end_ment;		/*��M�����t���O*/

// TM4C �� uart.h �� API �Ɏ�M���������ɂ���֐��͂Ȃ��B��M�������Ȃ��M�f�[�^��ǂݎ̂Ă邱�Ƃő�p�B 
static char receive_enable_host= 0;	/* 0:��M�����A1:��M�L�� */
static char receive_enable_ment= 0;	/* 0:��M�����A1:��M�L�� */

//ascii_to_bin:�o�C�i���[��ASCII �ϊ�
char ascii_in[6];	/*ASCII�R�[�h���̓o�b�t�@*/
//bin_to_ascii:�o�C�i���[��ASCII �ϊ�
char ascii_out[6];	/*ASCII�R�[�h�o�̓o�b�t�@*/


/****************************************************/
/* Function : com_init                    */
/* Summary  : �ʐM�̏�����    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : CH1�ݒ���g�p:CH����                    */
/****************************************************/
void	com_init(void){
	
	SVD[CH1].sti = (SVD[CH1].com_interval << 8) | SVD[CH1].com_speed;
	SVD[CH1].cmod = (SVD[CH1].com_mode << 8) | SVD[CH1].com_parity;

	if(com_type == COM_RS485){
		com_init_host();		/*RS485(�z�X�g��)�ʐM�̏�����*/	
	}
	com_init_ment();			/*RS485(�����e�i���X��)�ʐM�̏�����*/	
}

/****************************************************/
/* Function : com_init_host                        */
/* Summary  : RS485(�z�X�g��)�ʐM�̏�����           				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_init_host(void){

uint32_t ui32TxLevel;
uint32_t ui32RxLevel;

	receive_enable_host = 0;		/*��MDISABLE*/

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
	UARTConfigSetExpClk(UART_COM_HOST_BASE, g_ui32SysClock, 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
	UARTTxIntModeSet(UART_COM_HOST_BASE, UART_TXINT_MODE_EOT);

	// DMA �̏����ݒ� 
	UARTEnable(UART_COM_HOST_BASE);
	UARTDMAEnable(UART_COM_HOST_BASE, UART_DMA_TX);
	uDMAChannelAssign(UDMA_CH11_UART6TX);
	uDMAChannelAttributeEnable(UDMA_CH11_UART6TX, UDMA_ATTR_USEBURST);
	uDMAChannelControlSet(UDMA_CH11_UART6TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);
	// Put the attributes in a known state for the uDMA UARTxTX channel.  These
	// should already be disabled by default.
	uDMAChannelAttributeDisable(UDMA_CH11_UART6TX, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	// Set the USEBURST attribute for the uDMA UART TX channel.  This will
	// force the controller to always use a burst when transferring data from
	// the TX buffer to the UART.  This is somewhat more effecient bus usage
	// than the default which allows single or burst transfers.
	uDMAChannelAttributeEnable(UDMA_CH11_UART6TX, UDMA_ATTR_USEBURST);
	// Configure the control parameters for the UART TX.  The uDMA UART TX
	// channel is used to transfer a block of data from a buffer to the UART.
	// The data size is 8 bits.  The source address increment is 8-bit bytes
	// since the data is coming from a buffer.  The destination increment is
	// none since the data is to be written to the UART data register.  The
	// arbitration size is set to 4, which matches the UART TX FIFO trigger
	// threshold.
	uDMAChannelControlSet(UDMA_CH11_UART6TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);

	delay(200);					/*�ҋ@*/

	// Receive interrupt at 1/8 Full �ɕύX 
	UARTFIFOLevelGet(UART_COM_HOST_BASE, &ui32TxLevel, &ui32RxLevel);
	UARTFIFOLevelSet(UART_COM_HOST_BASE, ui32TxLevel, UART_FIFO_RX1_8);

	IntEnable(INT_UART_COM_HOST);
	UARTIntEnable(UART_COM_HOST_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);

	RX_enable_host();			/*��M����*/

}

/****************************************************/
/* Function : com_init_ment                        */
/* Summary  : RS485(�����e�i���X��)�ʐM�̏�����       				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_init_ment(void){
	
uint32_t ui32TxLevel;
uint32_t ui32RxLevel;

	receive_enable_ment = 0;		/*��MDISABLE*/

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	UARTConfigSetExpClk(UART_COM_MENT_BASE, g_ui32SysClock, 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
	UARTTxIntModeSet(UART_COM_MENT_BASE, UART_TXINT_MODE_EOT);

	// DMA �̏����ݒ� 
	UARTEnable(UART_COM_MENT_BASE);
	UARTDMAEnable(UART_COM_MENT_BASE, UART_DMA_TX);
	uDMAChannelAssign(UDMA_CH13_UART2TX);
	uDMAChannelAttributeEnable(UDMA_CH13_UART2TX, UDMA_ATTR_USEBURST);
	uDMAChannelControlSet(UDMA_CH13_UART2TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);
	// Put the attributes in a known state for the uDMA UARTxTX channel.  These
	// should already be disabled by default.
	uDMAChannelAttributeDisable(UDMA_CH13_UART2TX, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	// Set the USEBURST attribute for the uDMA UART TX channel.  This will
	// force the controller to always use a burst when transferring data from
	// the TX buffer to the UART.  This is somewhat more effecient bus usage
	// than the default which allows single or burst transfers.
	uDMAChannelAttributeEnable(UDMA_CH13_UART2TX, UDMA_ATTR_USEBURST);
	// Configure the control parameters for the UART TX.  The uDMA UART TX
	// channel is used to transfer a block of data from a buffer to the UART.
	// The data size is 8 bits.  The source address increment is 8-bit bytes
	// since the data is coming from a buffer.  The destination increment is
	// none since the data is to be written to the UART data register.  The
	// arbitration size is set to 4, which matches the UART TX FIFO trigger
	// threshold.
	uDMAChannelControlSet(UDMA_CH13_UART2TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);

	delay(200);					/*�ҋ@*/

	// Receive interrupt at 1/8 Full �ɕύX 
	UARTFIFOLevelGet(UART_COM_MENT_BASE, &ui32TxLevel, &ui32RxLevel);
	UARTFIFOLevelSet(UART_COM_MENT_BASE, ui32TxLevel, UART_FIFO_RX1_8);

	IntEnable(INT_UART_COM_MENT);
	UARTIntEnable(UART_COM_MENT_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);

	RX_enable_ment();			/*��M����*/
}

/****************************************************/
/* Function : abort_com                           */
/* Summary  : �G���[�����ݏ���                    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void abort_com(void){
	
}

/****************************************************/
/* Function : abort_com_host                       */
/* Summary  : �G���[�����ݏ���(�z�X�g)               				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void abort_com_host(void){
	
}

/****************************************************/
/* Function : abort_com_ment                       */
/* Summary  : �G���[�����ݏ���(�����e�i���X)           				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void abort_com_ment(void){
	
}
 
/****************************************************/
/* Function : int_com_host                         */
/* Summary  : RS485(�z�X�g) UART �̊��荞�ݏ���     				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 1�o�C�g�f�[�^��M�������ݏ������܂�           */
/*          : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs��*/
/****************************************************/
void	int_com_host(void){
	
	char stat;
	short i,count;
	unsigned short work,data_len;
	uint32_t ui32Status;
	
//REPEAT:

	// Get the interrrupt status.
	ui32Status = UARTIntStatus(UART_COM_HOST_BASE, true);

	// UARTCharGetNonBlocking �ŃN���A�����r�b�g�͂����ł̓N���A���Ȃ� 
	ui32Status &= ~(UART_INT_RT | UART_INT_RX);

	// Clear the asserted interrupts.
	UARTIntClear(UART_COM_HOST_BASE, ui32Status);

	if((ui32Status & UART_MIS_TXMIS) != 0)
	{
		TX_end_host();
	}

	if(receive_enable_host != 1) { /* ��MDISABLE�� */
		/* ��M�f�[�^�ǂݎ̂� */
		if (UARTCharsAvail(UART_COM_HOST_BASE)) {
			UARTCharGetNonBlocking(UART_COM_HOST_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*����*/
		if (UARTCharsAvail(UART_COM_HOST_BASE)) {	/*��MOK*/
			RX_buf_host[byte_count_host] = UARTCharGetNonBlocking(UART_COM_HOST_BASE);/*1byte �f�[�^��M*/
			if ( byte_count_host == 0){			/*read top data*/
				if (RX_buf_host[byte_count_host]=='@'){	/*�擪����*/
					if ( byte_count_host < MSG_MAX) byte_count_host++;	/*next RX_buf pointer*/
				}
			}else{
				if (((RX_buf_host[byte_count_host]>='!') && (RX_buf_host[byte_count_host]<='~'))||(RX_buf_host[byte_count_host]==CR)){	/*�L���R�[�h*/
					if ( byte_count_host < MSG_MAX) byte_count_host++;	/*next RX_buf pointer*/
				}
				else{
					byte_count_host =0;							/*counter initialize*/
				}
			}
			
		}else{
			//goto 	REPEAT;				/*��蒼��*/
		}
	}else{								/*��M�G���[*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*�I�[�o�[�����G���[*/
			err_stat |= 0x0004;
		}
		if ((ui32Status & UART_INT_FE) != 0) {	/*�t���[�~���O�G���[*/
			err_stat |= 0x0002;
		}
		if ((ui32Status & UART_INT_PE) != 0) {	/*�p���e�B�G���[*/
			err_stat |= 0x0001;
		}
		return;
	}
	/*�I���`�F�b�N*/
	if (RX_buf_host[byte_count_host-1] == CR){	/*��M�I��*/
		RX_buf_host[byte_count_host] = 0;
		work = (SVD[CH1].sti >> 8) * 2;
		if (work > 230) work = 230;
		work += 6;
		protocol_timer_host(work);

		receive_enable_host = 0;	/*��MDISABLE*/
		rx_end_host = 1;			/*�t���[����M�����t���O���Z�b�g*/
	}
}

/****************************************************/
/* Function : int_com_ment                        */
/* Summary  : RX485(�����e�i���X) UART �̊��荞�ݏ��� 				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 1�o�C�g�f�[�^��M�������ݏ������܂�          */
/*          : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs��*/
/****************************************************/
void	int_com_ment(void){
	
	char stat;
	short i;
	unsigned short work,data_len;
	uint32_t ui32Status;
	
//REPEAT:

	// Get the interrrupt status.
	ui32Status = UARTIntStatus(UART_COM_MENT_BASE, true);

	// UARTCharGetNonBlocking �ŃN���A�����r�b�g�͂����ł̓N���A���Ȃ� 
	ui32Status &= ~(UART_INT_RT | UART_INT_RX);

	// Clear the asserted interrupts.
	UARTIntClear(UART_COM_MENT_BASE, ui32Status);

	if((ui32Status & UART_MIS_TXMIS) != 0)
	{
		TX_end_ment();
	}

	if(receive_enable_ment != 1) { /* ��MDISABLE�� */
		/* ��M�f�[�^�ǂݎ̂� */
		if (UARTCharsAvail(UART_COM_MENT_BASE)) {
			UARTCharGetNonBlocking(UART_COM_MENT_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*����*/
		if (UARTCharsAvail(UART_COM_MENT_BASE)) {	/*��MOK*/
			RX_buf_ment[byte_count_ment] = UARTCharGetNonBlocking(UART_COM_MENT_BASE);/*1byte �f�[�^��M*/
			if ( byte_count_ment == 0){			/*read top data*/
				if (RX_buf_ment[byte_count_ment]=='@'){	/*�擪����*/
					if ( byte_count_ment < MSG_MAX) byte_count_ment++;	/*next RX_buf pointer*/
				}
			}else{
				if (((RX_buf_ment[byte_count_ment]>='!') && (RX_buf_ment[byte_count_ment]<='~'))||(RX_buf_ment[byte_count_ment]==CR)){	/*�L���R�[�h*/
					if ( byte_count_ment < MSG_MAX) byte_count_ment++;	/*next RX_buf pointer*/
				}
				else{
					byte_count_ment =0;							/*counter initialize*/
				}
			}
			
		}else{
			//goto 	REPEAT;				/*��蒼��*/
		}
	}else{								/*��M�G���[*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*�I�[�o�[�����G���[*/
			err_stat |= 0x0004;
		}
		if ((ui32Status & UART_INT_FE) != 0) {	/*�t���[�~���O�G���[*/
			err_stat |= 0x0002;
		}
		if ((ui32Status & UART_INT_PE) != 0) {	/*�p���e�B�G���[*/
			err_stat |= 0x0001;
		}
		return;
	}
	/*�I���`�F�b�N*/
	if (RX_buf_ment[byte_count_ment-1] == CR){	/*��M�I��*/
		RX_buf_ment[byte_count_ment] = 0;
		work = (SVD[CH1].sti >> 8) * 2;
		if (work > 230) work = 230;
		work += 6;
		protocol_timer_ment(work);

		receive_enable_ment = 0;	/*��MDISABLE*/
		rx_end_ment = 1;			/*�t���[����M�����t���O���Z�b�g*/
	}
}

/****************************************************/
/* Function : TX_start_host                        */
/* Summary  : RS485(�z�X�g)���M�J�n�i1�t���[�����M�J�n�j 				*/
/* Argument : byte_num             	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : TX_buf �̓��e�𑗐M����                  */
/****************************************************/
void	TX_start_host(short byte_num){

	short i;
	
	if(byte_num == 0) return;		/*���M�f�[�^����*/

	/*�����ʐM����(DMA���g�p����SCI�ʐM)*/
	// GPIO_PP4 
	__bit_output(GPIO_PORTP_BASE, 4, 1);

	uDMAChannelTransferSet(UDMA_CH11_UART6TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_buf_host, (void *)(UART_COM_HOST_BASE + UART_O_DR), byte_num);
	uDMAChannelEnable(UDMA_CH11_UART6TX);

}

/****************************************************/
/* Function : DMA2_end                              */
/* Summary  : DMA�]���I��                       				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void DMA2_end() {
}

/****************************************************/
/* Function : TX_end_host                          */
/* Summary  : ���M�f�[�^�G���v�e�B                    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void TX_end_host() {

	// GPIO_PP4 
	__bit_output(GPIO_PORTP_BASE, 4, 0);
	
	RX_enable_host();					/*��M����*/
}

/****************************************************/
/* Function : TX_start_ment                         */
/* Summary  : RS485(�����e�i���X)���M�J�n�i1�t���[�����M�J�n�j		*/
/* Argument : byte_num              	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : TX_buf �̓��e�𑗐M����                   */
/****************************************************/
void	TX_start_ment(short byte_num){

	short i;
	
	if(byte_num == 0) return;		/*���M�f�[�^����*/

	/*�����ʐM����(DMA���g�p����SCI�ʐM)*/
	// GPIO_PD6 
	__bit_output(GPIO_PORTD_BASE, 6, 1);

	uDMAChannelTransferSet(UDMA_CH13_UART2TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_buf_ment, (void *)(UART_COM_MENT_BASE + UART_O_DR), byte_num);
	uDMAChannelEnable(UDMA_CH13_UART2TX);
}

/****************************************************/
/* Function : DMA1_end                             */
/* Summary  : DMA�]���I��                       				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void DMA1_end() {
}

/****************************************************/
/* Function : TX_end_ment                          */
/* Summary  : ���M�f�[�^�G���v�e�B                    				*/
/* Argument : �Ȃ�                  	               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void TX_end_ment() {

	// GPIO_PD6 
	__bit_output(GPIO_PORTD_BASE, 6, 0);

	RX_enable_ment();					/*��M����*/
}

/****************************************************/
/* Function : RX_enable_host                        */
/* Summary  : RS485(�z�X�g)��M���i1�t���[����M�J�n�j  				*/
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	RX_enable_host(void){

	short i;
	
	byte_count_host = 0;				/*�J�E���^���Z�b�g*/
	rx_end_host = rx_end_subhost = 0;	/*��M�����t���O���Z�b�g*/

	// GPIO_PP4 
	__bit_output(GPIO_PORTP_BASE, 4, 0);
	
	receive_enable_host = 1;	/*��MENABLE	*/
}

/****************************************************/
/* Function : RX_enable_ment                        */
/* Summary  : RS485(�����e�i���X)��M���i1�t���[����M�J�n�j  */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	RX_enable_ment(void){

	short i;
	
	byte_count_ment = 0;		/*�J�E���^���Z�b�g*/
	rx_end_ment = 0;			/*��M�����t���O���Z�b�g*/

	// GPIO_PD6 
	__bit_output(GPIO_PORTD_BASE, 6, 0);

	receive_enable_ment = 1;		/*��MENABLE*/
}

/****************************************************/
/* Function : ascii_to_bin                          */
/* Summary  : ASCII -> �o�C�i���[ �ϊ�                  */
/* Argument : ASCII �R�[�h�̃o�C�g��(1-5)                */
/*          : ASCII �R�[�h	ascii_in[6]                */
/* Return   : short (0-32000)                       */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short ascii_to_bin(short byte_num){

	short i,work,mul;

	mul=1;
	for (i=0; i<byte_num-1; i++){	/*�ő包��*/
		mul *= 10;
	}
	work=0;
	for (i=0; i<byte_num; i++){
		work += (short)(ascii_in[i] & 0x0F) * mul;
		mul /= 10;
	}
	return work;
}

/****************************************************/
/* Function : bin_to_ascii                          */
/* Summary  : �o�C�i���[ -> ASCII �ϊ�                  */
/* Argument : short (0-32000)                       */
/* Return   : ASCII �R�[�h�̃o�C�g��(1-5)                */
/*          : ASCII �R�[�h	ascii_in[6]                */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short		bin_to_ascii(short input){
	
	short byte_num,work,i;

	for ( i=0; i<6; i++) ascii_out[i] = 0;	/*�o�̓N���A*/

	if (input >= 0){
		if (input >= 10000) byte_num =5;
		else if (input >=1000) byte_num =4;
		else if (input >=100) byte_num =3;
		else if (input >=10) byte_num =2;
		else byte_num =1;

		work = input;
		for ( i=0; i<byte_num; i++){
			ascii_out[byte_num - 1 -i] = (char)((work % 10)+0x30);
			work /= 10;
		}
		return byte_num;
	}
	else{
		ascii_out[0]=0x30;
		return (short)1;
	}
}

/****************************************************/
/* Function : com_cont_host                         */
/* Summary  : RS458/CUnet(�z�X�g)�ʐM����              */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_cont_host(void){
	
	if(rx_end_host == 1){		/*1�t���[����M*/
		protocol(HOST);
	}
}

/****************************************************/
/* Function : com_cont_subhost                      */
/* Summary  : CUnet(�T�u�z�X�g)�ʐM����                 */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_cont_subhost(void){
	
	if(rx_end_subhost == 1){	/*1�t���[����M*/
		protocol(SUB_HOST);
	}
}

/****************************************************/
/* Function : com_cont_ment                         */
/* Summary  : RS458(�����e�i���X)�ʐM����                */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	com_cont_ment(void){
	
	if(rx_end_ment == 1){		/*1�t���[����M*/
		protocol(MENT);
	}
}

/****************************************************/
/* Function : protocol_timer_ment                  */
/* Summary  : �v���g�R���J�n�^�C�}�[                      */
/* Argument : value               	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void protocol_timer_ment(unsigned short value){

	TimerDisable(TIMER1_BASE, TIMER_A);
	TimerLoadSet(TIMER1_BASE, TIMER_A, calc_cnt(value * 24, 24, 512, 120, 1)); // RX �� FW �ł̐ݒ�l value * 24 ����v�Z 
	TimerEnable(TIMER1_BASE, TIMER_A);
}	

/****************************************************/
/* Function : protocol_timer_host                  */
/* Summary  : �v���g�R���J�n�^�C�}�[(�z�X�g)                */
/* Argument : value               	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void protocol_timer_host(unsigned short value){

	TimerDisable(TIMER3_BASE, TIMER_A);
	TimerLoadSet(TIMER3_BASE, TIMER_A, calc_cnt(value * 24, 24, 512, 120, 1)); // RX �� FW �ł̐ݒ�l value * 24 ����v�Z 
	TimerEnable(TIMER3_BASE, TIMER_A);
}	

/****************************************************/
/* Function : protocol_timer_subhost                */
/* Summary  : �v���g�R���J�n�^�C�}�[(�T�u�z�X�g)               */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void protocol_timer_subhost(void){

	TimerDisable(TIMER4_BASE, TIMER_A);
	TimerLoadSet(TIMER4_BASE, TIMER_A, calc_cnt(0xFFFF - 6, 24, 64, 120, 1)); // RX �� FW �ł̐ݒ�l 0xFFFF - 6 ����v�Z 
	TimerEnable(TIMER4_BASE, TIMER_A);
}	

/****************************************************/
/* Function : int_cmt1                              */
/* Summary  : ������(�R���y�A�}�b�`)                      */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs��*/
/****************************************************/
void	int_cmt1(void){

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	receive_enable_ment = 0;	/*��MDISABLE*/
	rx_end_ment = 1;		/*�t���[����M�����t���O���Z�b�g*/

	setpsw_i();				/*�����݋���*/

	com_cont_ment();		/*RS485(�����e�i���X)�ʐM����*/
}

/****************************************************/
/* Function : int_cmt3                              */
/* Summary  : ������(�R���y�A�}�b�`)                      */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs��*/
/****************************************************/
void	int_cmt3(void){
	
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	
	receive_enable_host = 0;	/*��MDISABLE*/
	rx_end_host = 1;		/*�t���[����M�����t���O���Z�b�g*/

	setpsw_i();				/*�����݋���*/
	
	com_cont_host();		/*RS485/CUnet(�z�X�g)�ʐM����*/
}

/****************************************************/
/* Function : int_mtu0                              */
/* Summary  : ������(�}���`�t�@���N�V�����^�C�}�p���X���j�b�g)      */
/* Argument : �Ȃ�                  	                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �ďo���� �utm4c1290nczad_startup_ccs.c�v �ōs��*/
/****************************************************/
void	int_mtu0(void){
	
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	receive_enable_host = 0;	/*��MDISABLE*/
	rx_end_subhost = 1;		/*�t���[����M�����t���O���Z�b�g*/

	setpsw_i();				/*�����݋���*/
	
	com_cont_subhost();		/*CUnet(�T�u�z�X�g)�ʐM����*/
}

