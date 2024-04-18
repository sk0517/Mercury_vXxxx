/***********************************************/
/* File Name : COM.c    		         									   */
/*	Summary   : 通信処理  				                   */
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
/*	モジュール内定義関数								*/
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
/*	モジュール外定義関数								*/
/********************************************************/
extern void protocol(short);
extern void delay(unsigned short delay_count);

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short  com_type;
extern char TX_buf[MSG_NUM][MSG_MAX];
extern char RX_buf[MSG_NUM][MSG_MAX];
extern uint32_t g_ui32SysClock;

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
char *RX_buf_host = &(RX_buf[HOST][0]);
char *TX_buf_host = &(TX_buf[HOST][0]);
char *RX_buf_ment = &(RX_buf[MENT][0]);
char *TX_buf_ment = &(TX_buf[MENT][0]);
char *RX_buf_subhost = &(RX_buf[SUB_HOST][0]);
char *TX_buf_subhost = &(TX_buf[SUB_HOST][0]);

short byte_count;			/*受信バイトカウンタ*/
short byte_count_host;		/*受信バイトカウンタ*/
short byte_count_ment;		/*受信バイトカウンタ*/
short err_stat;			/*エラーステイタス*/
short rx_end_host;		/*受信完了フラグ*/
short rx_end_subhost;		/*受信完了フラグ*/
short rx_end_ment;		/*受信完了フラグ*/

// TM4C の uart.h の API に受信だけ無効にする関数はない。受信無効中なら受信データを読み捨てることで代用。 
static char receive_enable_host= 0;	/* 0:受信無効、1:受信有効 */
static char receive_enable_ment= 0;	/* 0:受信無効、1:受信有効 */

//ascii_to_bin:バイナリー＞ASCII 変換
char ascii_in[6];	/*ASCIIコード入力バッファ*/
//bin_to_ascii:バイナリー＞ASCII 変換
char ascii_out[6];	/*ASCIIコード出力バッファ*/


/****************************************************/
/* Function : com_init                    */
/* Summary  : 通信の初期化    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : CH1設定を使用:CH共通                    */
/****************************************************/
void	com_init(void){
	
	SVD[CH1].sti = (SVD[CH1].com_interval << 8) | SVD[CH1].com_speed;
	SVD[CH1].cmod = (SVD[CH1].com_mode << 8) | SVD[CH1].com_parity;

	if(com_type == COM_RS485){
		com_init_host();		/*RS485(ホスト側)通信の初期化*/	
	}
	com_init_ment();			/*RS485(メンテナンス側)通信の初期化*/	
}

/****************************************************/
/* Function : com_init_host                        */
/* Summary  : RS485(ホスト側)通信の初期化           				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_init_host(void){

uint32_t ui32TxLevel;
uint32_t ui32RxLevel;

	receive_enable_host = 0;		/*受信DISABLE*/

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
	UARTConfigSetExpClk(UART_COM_HOST_BASE, g_ui32SysClock, 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
	UARTTxIntModeSet(UART_COM_HOST_BASE, UART_TXINT_MODE_EOT);

	// DMA の初期設定 
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

	delay(200);					/*待機*/

	// Receive interrupt at 1/8 Full に変更 
	UARTFIFOLevelGet(UART_COM_HOST_BASE, &ui32TxLevel, &ui32RxLevel);
	UARTFIFOLevelSet(UART_COM_HOST_BASE, ui32TxLevel, UART_FIFO_RX1_8);

	IntEnable(INT_UART_COM_HOST);
	UARTIntEnable(UART_COM_HOST_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);

	RX_enable_host();			/*受信許可*/

}

/****************************************************/
/* Function : com_init_ment                        */
/* Summary  : RS485(メンテナンス側)通信の初期化       				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_init_ment(void){
	
uint32_t ui32TxLevel;
uint32_t ui32RxLevel;

	receive_enable_ment = 0;		/*受信DISABLE*/

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	UARTConfigSetExpClk(UART_COM_MENT_BASE, g_ui32SysClock, 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
	UARTTxIntModeSet(UART_COM_MENT_BASE, UART_TXINT_MODE_EOT);

	// DMA の初期設定 
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

	delay(200);					/*待機*/

	// Receive interrupt at 1/8 Full に変更 
	UARTFIFOLevelGet(UART_COM_MENT_BASE, &ui32TxLevel, &ui32RxLevel);
	UARTFIFOLevelSet(UART_COM_MENT_BASE, ui32TxLevel, UART_FIFO_RX1_8);

	IntEnable(INT_UART_COM_MENT);
	UARTIntEnable(UART_COM_MENT_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);

	RX_enable_ment();			/*受信許可*/
}

/****************************************************/
/* Function : abort_com                           */
/* Summary  : エラー割込み処理                    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void abort_com(void){
	
}

/****************************************************/
/* Function : abort_com_host                       */
/* Summary  : エラー割込み処理(ホスト)               				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void abort_com_host(void){
	
}

/****************************************************/
/* Function : abort_com_ment                       */
/* Summary  : エラー割込み処理(メンテナンス)           				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void abort_com_ment(void){
	
}
 
/****************************************************/
/* Function : int_com_host                         */
/* Summary  : RS485(ホスト) UART の割り込み処理     				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 1バイトデータ受信時割込み処理を含む           */
/*          : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う*/
/****************************************************/
void	int_com_host(void){
	
	char stat;
	short i,count;
	unsigned short work,data_len;
	uint32_t ui32Status;
	
//REPEAT:

	// Get the interrrupt status.
	ui32Status = UARTIntStatus(UART_COM_HOST_BASE, true);

	// UARTCharGetNonBlocking でクリアされるビットはここではクリアしない 
	ui32Status &= ~(UART_INT_RT | UART_INT_RX);

	// Clear the asserted interrupts.
	UARTIntClear(UART_COM_HOST_BASE, ui32Status);

	if((ui32Status & UART_MIS_TXMIS) != 0)
	{
		TX_end_host();
	}

	if(receive_enable_host != 1) { /* 受信DISABLE中 */
		/* 受信データ読み捨て */
		if (UARTCharsAvail(UART_COM_HOST_BASE)) {
			UARTCharGetNonBlocking(UART_COM_HOST_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*正常*/
		if (UARTCharsAvail(UART_COM_HOST_BASE)) {	/*受信OK*/
			RX_buf_host[byte_count_host] = UARTCharGetNonBlocking(UART_COM_HOST_BASE);/*1byte データ受信*/
			if ( byte_count_host == 0){			/*read top data*/
				if (RX_buf_host[byte_count_host]=='@'){	/*先頭文字*/
					if ( byte_count_host < MSG_MAX) byte_count_host++;	/*next RX_buf pointer*/
				}
			}else{
				if (((RX_buf_host[byte_count_host]>='!') && (RX_buf_host[byte_count_host]<='~'))||(RX_buf_host[byte_count_host]==CR)){	/*有効コード*/
					if ( byte_count_host < MSG_MAX) byte_count_host++;	/*next RX_buf pointer*/
				}
				else{
					byte_count_host =0;							/*counter initialize*/
				}
			}
			
		}else{
			//goto 	REPEAT;				/*やり直し*/
		}
	}else{								/*受信エラー*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*オーバーランエラー*/
			err_stat |= 0x0004;
		}
		if ((ui32Status & UART_INT_FE) != 0) {	/*フレーミングエラー*/
			err_stat |= 0x0002;
		}
		if ((ui32Status & UART_INT_PE) != 0) {	/*パリティエラー*/
			err_stat |= 0x0001;
		}
		return;
	}
	/*終了チェック*/
	if (RX_buf_host[byte_count_host-1] == CR){	/*受信終了*/
		RX_buf_host[byte_count_host] = 0;
		work = (SVD[CH1].sti >> 8) * 2;
		if (work > 230) work = 230;
		work += 6;
		protocol_timer_host(work);

		receive_enable_host = 0;	/*受信DISABLE*/
		rx_end_host = 1;			/*フレーム受信完了フラグをセット*/
	}
}

/****************************************************/
/* Function : int_com_ment                        */
/* Summary  : RX485(メンテナンス) UART の割り込み処理 				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 1バイトデータ受信時割込み処理を含む          */
/*          : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う*/
/****************************************************/
void	int_com_ment(void){
	
	char stat;
	short i;
	unsigned short work,data_len;
	uint32_t ui32Status;
	
//REPEAT:

	// Get the interrrupt status.
	ui32Status = UARTIntStatus(UART_COM_MENT_BASE, true);

	// UARTCharGetNonBlocking でクリアされるビットはここではクリアしない 
	ui32Status &= ~(UART_INT_RT | UART_INT_RX);

	// Clear the asserted interrupts.
	UARTIntClear(UART_COM_MENT_BASE, ui32Status);

	if((ui32Status & UART_MIS_TXMIS) != 0)
	{
		TX_end_ment();
	}

	if(receive_enable_ment != 1) { /* 受信DISABLE中 */
		/* 受信データ読み捨て */
		if (UARTCharsAvail(UART_COM_MENT_BASE)) {
			UARTCharGetNonBlocking(UART_COM_MENT_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*正常*/
		if (UARTCharsAvail(UART_COM_MENT_BASE)) {	/*受信OK*/
			RX_buf_ment[byte_count_ment] = UARTCharGetNonBlocking(UART_COM_MENT_BASE);/*1byte データ受信*/
			if ( byte_count_ment == 0){			/*read top data*/
				if (RX_buf_ment[byte_count_ment]=='@'){	/*先頭文字*/
					if ( byte_count_ment < MSG_MAX) byte_count_ment++;	/*next RX_buf pointer*/
				}
			}else{
				if (((RX_buf_ment[byte_count_ment]>='!') && (RX_buf_ment[byte_count_ment]<='~'))||(RX_buf_ment[byte_count_ment]==CR)){	/*有効コード*/
					if ( byte_count_ment < MSG_MAX) byte_count_ment++;	/*next RX_buf pointer*/
				}
				else{
					byte_count_ment =0;							/*counter initialize*/
				}
			}
			
		}else{
			//goto 	REPEAT;				/*やり直し*/
		}
	}else{								/*受信エラー*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*オーバーランエラー*/
			err_stat |= 0x0004;
		}
		if ((ui32Status & UART_INT_FE) != 0) {	/*フレーミングエラー*/
			err_stat |= 0x0002;
		}
		if ((ui32Status & UART_INT_PE) != 0) {	/*パリティエラー*/
			err_stat |= 0x0001;
		}
		return;
	}
	/*終了チェック*/
	if (RX_buf_ment[byte_count_ment-1] == CR){	/*受信終了*/
		RX_buf_ment[byte_count_ment] = 0;
		work = (SVD[CH1].sti >> 8) * 2;
		if (work > 230) work = 230;
		work += 6;
		protocol_timer_ment(work);

		receive_enable_ment = 0;	/*受信DISABLE*/
		rx_end_ment = 1;			/*フレーム受信完了フラグをセット*/
	}
}

/****************************************************/
/* Function : TX_start_host                        */
/* Summary  : RS485(ホスト)送信開始（1フレーム送信開始） 				*/
/* Argument : byte_num             	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : TX_buf の内容を送信する                  */
/****************************************************/
void	TX_start_host(short byte_num){

	short i;
	
	if(byte_num == 0) return;		/*送信データ無し*/

	/*高速通信処理(DMAを使用したSCI通信)*/
	// GPIO_PP4 
	__bit_output(GPIO_PORTP_BASE, 4, 1);

	uDMAChannelTransferSet(UDMA_CH11_UART6TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_buf_host, (void *)(UART_COM_HOST_BASE + UART_O_DR), byte_num);
	uDMAChannelEnable(UDMA_CH11_UART6TX);

}

/****************************************************/
/* Function : DMA2_end                              */
/* Summary  : DMA転送終了                       				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void DMA2_end() {
}

/****************************************************/
/* Function : TX_end_host                          */
/* Summary  : 送信データエンプティ                    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void TX_end_host() {

	// GPIO_PP4 
	__bit_output(GPIO_PORTP_BASE, 4, 0);
	
	RX_enable_host();					/*受信許可*/
}

/****************************************************/
/* Function : TX_start_ment                         */
/* Summary  : RS485(メンテナンス)送信開始（1フレーム送信開始）		*/
/* Argument : byte_num              	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : TX_buf の内容を送信する                   */
/****************************************************/
void	TX_start_ment(short byte_num){

	short i;
	
	if(byte_num == 0) return;		/*送信データ無し*/

	/*高速通信処理(DMAを使用したSCI通信)*/
	// GPIO_PD6 
	__bit_output(GPIO_PORTD_BASE, 6, 1);

	uDMAChannelTransferSet(UDMA_CH13_UART2TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, TX_buf_ment, (void *)(UART_COM_MENT_BASE + UART_O_DR), byte_num);
	uDMAChannelEnable(UDMA_CH13_UART2TX);
}

/****************************************************/
/* Function : DMA1_end                             */
/* Summary  : DMA転送終了                       				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void DMA1_end() {
}

/****************************************************/
/* Function : TX_end_ment                          */
/* Summary  : 送信データエンプティ                    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void TX_end_ment() {

	// GPIO_PD6 
	__bit_output(GPIO_PORTD_BASE, 6, 0);

	RX_enable_ment();					/*受信許可*/
}

/****************************************************/
/* Function : RX_enable_host                        */
/* Summary  : RS485(ホスト)受信許可（1フレーム受信開始）  				*/
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	RX_enable_host(void){

	short i;
	
	byte_count_host = 0;				/*カウンタリセット*/
	rx_end_host = rx_end_subhost = 0;	/*受信完了フラグリセット*/

	// GPIO_PP4 
	__bit_output(GPIO_PORTP_BASE, 4, 0);
	
	receive_enable_host = 1;	/*受信ENABLE	*/
}

/****************************************************/
/* Function : RX_enable_ment                        */
/* Summary  : RS485(メンテナンス)受信許可（1フレーム受信開始）  */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	RX_enable_ment(void){

	short i;
	
	byte_count_ment = 0;		/*カウンタリセット*/
	rx_end_ment = 0;			/*受信完了フラグリセット*/

	// GPIO_PD6 
	__bit_output(GPIO_PORTD_BASE, 6, 0);

	receive_enable_ment = 1;		/*受信ENABLE*/
}

/****************************************************/
/* Function : ascii_to_bin                          */
/* Summary  : ASCII -> バイナリー 変換                  */
/* Argument : ASCII コードのバイト数(1-5)                */
/*          : ASCII コード	ascii_in[6]                */
/* Return   : short (0-32000)                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short ascii_to_bin(short byte_num){

	short i,work,mul;

	mul=1;
	for (i=0; i<byte_num-1; i++){	/*最大桁数*/
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
/* Summary  : バイナリー -> ASCII 変換                  */
/* Argument : short (0-32000)                       */
/* Return   : ASCII コードのバイト数(1-5)                */
/*          : ASCII コード	ascii_in[6]                */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		bin_to_ascii(short input){
	
	short byte_num,work,i;

	for ( i=0; i<6; i++) ascii_out[i] = 0;	/*出力クリア*/

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
/* Summary  : RS458/CUnet(ホスト)通信処理              */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_cont_host(void){
	
	if(rx_end_host == 1){		/*1フレーム受信*/
		protocol(HOST);
	}
}

/****************************************************/
/* Function : com_cont_subhost                      */
/* Summary  : CUnet(サブホスト)通信処理                 */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_cont_subhost(void){
	
	if(rx_end_subhost == 1){	/*1フレーム受信*/
		protocol(SUB_HOST);
	}
}

/****************************************************/
/* Function : com_cont_ment                         */
/* Summary  : RS458(メンテナンス)通信処理                */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	com_cont_ment(void){
	
	if(rx_end_ment == 1){		/*1フレーム受信*/
		protocol(MENT);
	}
}

/****************************************************/
/* Function : protocol_timer_ment                  */
/* Summary  : プロトコル開始タイマー                      */
/* Argument : value               	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void protocol_timer_ment(unsigned short value){

	TimerDisable(TIMER1_BASE, TIMER_A);
	TimerLoadSet(TIMER1_BASE, TIMER_A, calc_cnt(value * 24, 24, 512, 120, 1)); // RX 版 FW での設定値 value * 24 から計算 
	TimerEnable(TIMER1_BASE, TIMER_A);
}	

/****************************************************/
/* Function : protocol_timer_host                  */
/* Summary  : プロトコル開始タイマー(ホスト)                */
/* Argument : value               	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void protocol_timer_host(unsigned short value){

	TimerDisable(TIMER3_BASE, TIMER_A);
	TimerLoadSet(TIMER3_BASE, TIMER_A, calc_cnt(value * 24, 24, 512, 120, 1)); // RX 版 FW での設定値 value * 24 から計算 
	TimerEnable(TIMER3_BASE, TIMER_A);
}	

/****************************************************/
/* Function : protocol_timer_subhost                */
/* Summary  : プロトコル開始タイマー(サブホスト)               */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void protocol_timer_subhost(void){

	TimerDisable(TIMER4_BASE, TIMER_A);
	TimerLoadSet(TIMER4_BASE, TIMER_A, calc_cnt(0xFFFF - 6, 24, 64, 120, 1)); // RX 版 FW での設定値 0xFFFF - 6 から計算 
	TimerEnable(TIMER4_BASE, TIMER_A);
}	

/****************************************************/
/* Function : int_cmt1                              */
/* Summary  : 割込み(コンペアマッチ)                      */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う*/
/****************************************************/
void	int_cmt1(void){

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	receive_enable_ment = 0;	/*受信DISABLE*/
	rx_end_ment = 1;		/*フレーム受信完了フラグをセット*/

	setpsw_i();				/*割込み許可*/

	com_cont_ment();		/*RS485(メンテナンス)通信処理*/
}

/****************************************************/
/* Function : int_cmt3                              */
/* Summary  : 割込み(コンペアマッチ)                      */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う*/
/****************************************************/
void	int_cmt3(void){
	
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	
	receive_enable_host = 0;	/*受信DISABLE*/
	rx_end_host = 1;		/*フレーム受信完了フラグをセット*/

	setpsw_i();				/*割込み許可*/
	
	com_cont_host();		/*RS485/CUnet(ホスト)通信処理*/
}

/****************************************************/
/* Function : int_mtu0                              */
/* Summary  : 割込み(マルチファンクションタイマパルスユニット)      */
/* Argument : なし                  	                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 呼出しは 「tm4c1290nczad_startup_ccs.c」 で行う*/
/****************************************************/
void	int_mtu0(void){
	
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	receive_enable_host = 0;	/*受信DISABLE*/
	rx_end_subhost = 1;		/*フレーム受信完了フラグをセット*/

	setpsw_i();				/*割込み許可*/
	
	com_cont_subhost();		/*CUnet(サブホスト)通信処理*/
}

