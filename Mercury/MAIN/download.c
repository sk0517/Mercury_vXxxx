/***********************************************/
/* File Name : download.c	         									   */
/*	Summary   : ファームウェアダウンロード処理            */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <machine.h>
#include <string.h> 

#include "cunet.h"
#include "define.h"
#include "version.h"
#include "SV_def.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_flash.h"
#include "inc/hw_gpio.h"
#include "inc/hw_watchdog.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/flash.h"
#include "driverlib/watchdog.h"

#define DL_DEBUG	0

#pragma SET_CODE_SECTION("RAM_PRG")

extern uint32_t g_ui32SysClock;

extern char RAM_PRG_start[];
extern size_t RAM_PRG_size[];

/****************************************************/
/*  THIS MODULE DEFINE                              */
/****************************************************/
char RX_buf_dl[MSG_MAX_DL];   /*受信用配列*/
char TX_buf_dl[MSG_MAX_DL];   /*送信用配列*/

short byte_count_dl;          /*受信バイトカウンタ*/
short err_stat_dl;            /*エラーステイタス*/
short rx_end_dl;              /*受信完了フラグ*/
short	buf_count_dl;
unsigned short	recv_sa_dl;	//送信元ステーションアドレス

static char receive_enable= 0;	/* 0:受信無効、1:受信有効 */

// RAM 上で実行するコードをコピーするためのバッファ。 
// map ファイルの RAM_PRG のサイズ以上の領域を確保。
// 安全のため最適化無効にした場合の RAM_PRG のサイズ以上の領域を確保。 
char code_buffer[0x2000];

void com_init_dl_ment(void);
void com_init_dl_host(void);
void com_init_dl_cunet(void);
void delay_dl(unsigned short delay_count);
void RX_enable_dl_ment(void);
void RX_enable_dl_host(void);
static void _download(short mode, short type);
void enter_RAM_PRG(short mode, short type);





/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/


void download(short mode, short type) {

	void (*enter_RAM_PRG_p)(short mode, short type);
	unsigned long enter_RAM_PRG_offset;
	unsigned long size;
	
 	// 割込み禁止
	clrpsw_i();
	
	// WDT stop
	WatchdogResetDisable(WATCHDOG0_BASE);

	if(mode == HOST){				//ホストポート
		if(type == COM_RS485){		//RS485基板
			com_init_dl_host();
		}else{						//CUnet基板
			com_init_dl_cunet();
		}
	}else{							//メンテナンスポート
		com_init_dl_ment();
	}
	
	size = (unsigned long)RAM_PRG_size;
	
	// コードサイズが大きすぎる場合は、不正な FW を書き込まないようにここでフリーズ 
	if(size > sizeof(code_buffer))
	{
		while(1);
	}
	
	memcpy(code_buffer, (void *)RAM_PRG_start, size);
	
	enter_RAM_PRG_offset = (unsigned long)&enter_RAM_PRG - (unsigned long)RAM_PRG_start;
	
	enter_RAM_PRG_p= (void(*)(short, short))((unsigned long)&code_buffer[0] + enter_RAM_PRG_offset);
	(*enter_RAM_PRG_p)(mode, type);
}

/****************************************************/
/*  通信の初期化(メンテナンスポート)                */
/*  CH1設定を使用:CH共通                            */
/****************************************************/
void    com_init_dl_ment(void){
    
    SVD[CH1].sti = (SVD[CH1].com_interval << 8) | SVD[CH1].com_speed;
    SVD[CH1].cmod = (SVD[CH1].com_mode << 8) | SVD[CH1].com_parity;

    receive_enable = 0;	/*受信DISABLE*/

    IntDisable(INT_UART_COM_MENT);
    UARTDisable(UART_COM_MENT_BASE);
    UARTConfigSetExpClk(UART_COM_MENT_BASE, g_ui32SysClock, 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
    UARTEnable(UART_COM_MENT_BASE);

    delay_dl(200);              /*待機*/

    RX_enable_dl_ment();       /*受信許可*/
}

/****************************************************/
/*  通信の初期化(ホストポート)                      */
/*  CH1設定を使用:CH共通                            */
/****************************************************/
void    com_init_dl_host(void){
    
    SVD[CH1].sti = (SVD[CH1].com_interval << 8) | SVD[CH1].com_speed;
    SVD[CH1].cmod = (SVD[CH1].com_mode << 8) | SVD[CH1].com_parity;

    receive_enable = 0;	/*受信DISABLE*/

    IntDisable(INT_UART_COM_HOST);
    UARTDisable(UART_COM_HOST_BASE);
    UARTConfigSetExpClk(UART_COM_HOST_BASE, g_ui32SysClock, 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
    UARTEnable(UART_COM_HOST_BASE);

    delay_dl(200);              /*待機*/

    RX_enable_dl_host();       /*受信許可*/
}

/****************************************************/
/*  通信の初期化(CUnet) 		                    */
/****************************************************/
void    com_init_dl_cunet(void){

	MKY43.REG.MR0CR.BIT.RDY = 1;	//MRB1側メール受信許可
	MKY43.REG.MR1CR.BIT.RDY = 0;	//MRB1側メール受信禁止

	rx_end_dl = 0;                  /*受信完了フラグリセット*/

	memset(TX_buf_dl, 0, sizeof(TX_buf_dl));    /*送受信バッファクリア*/
    memset(RX_buf_dl, 0, sizeof(RX_buf_dl));
}

// 以後のコードは RAM にコピーして RAM 上で実行する 
#pragma SET_CODE_SECTION("RAM_PRG")

// RAM 上の実行開始関数 
void enter_RAM_PRG(short mode, short type)
{
	_download(mode, type);
}

void memset_dl(void *buf, short ch, size_t n)
{
	unsigned char *p;
	size_t i;
	p = buf;
	for(i = 0; i < n; i++)
	{
		*(p++) = (unsigned char)ch;
	}
}

void memcpy_dl(void *buf1, const void *buf2, size_t n)
{
	unsigned char *p1;
	const unsigned char *p2;
	size_t i;
	p1 = buf1;
	p2 = buf2;
	for(i = 0; i < n; i++)
	{
		*(p1++) = *(p2++);
	}
}

inline signed long min_dl(signed long data1, signed long data2)
{
	if(data1 < data2)
	{
		return data1;
	}
	else
	{
		return data2;
	}
}

// uart.c からコピーし関数名に _dl を付加。 
uint32_t
UARTIntStatus_dl(uint32_t ui32Base, bool bMasked)
{
    //
    // Check the arguments.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // Return either the interrupt status or the raw interrupt status as
    // requested.
    //
    if(bMasked)
    {
        return(HWREG(ui32Base + UART_O_MIS));
    }
    else
    {
        return(HWREG(ui32Base + UART_O_RIS));
    }
}

// uart.c からコピーし関数名に _dl を付加。 
void
UARTIntClear_dl(uint32_t ui32Base, uint32_t ui32IntFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // Clear the requested interrupt sources.
    //
    HWREG(ui32Base + UART_O_ICR) = ui32IntFlags;
}

// uart.c からコピーし関数名に _dl を付加。 
bool
UARTCharsAvail_dl(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // Return the availability of characters.
    //
    return((HWREG(ui32Base + UART_O_FR) & UART_FR_RXFE) ? false : true);
}

// uart.c からコピーし関数名に _dl を付加。 
int32_t
UARTCharGetNonBlocking_dl(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // See if there are any characters in the receive FIFO.
    //
    if(!(HWREG(ui32Base + UART_O_FR) & UART_FR_RXFE))
    {
        //
        // Read and return the next character.
        //
        return(HWREG(ui32Base + UART_O_DR));
    }
    else
    {
        //
        // There are no characters, so return a failure.
        //
        return(-1);
    }
}

// uart.c からコピーし関数名に _dl を付加。 
void
UARTCharPut_dl(uint32_t ui32Base, unsigned char ucData)
{
    //
    // Check the arguments.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // Wait until space is available.
    //
    while(HWREG(ui32Base + UART_O_FR) & UART_FR_TXFF)
    {
    }

    //
    // Send the char.
    //
    HWREG(ui32Base + UART_O_DR) = ucData;
}

// uart.c からコピーし関数名に _dl を付加。 
bool
UARTBusy_dl(uint32_t ui32Base)
{
    //
    // Check the argument.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // Determine if the UART is busy.
    //
    return((HWREG(ui32Base + UART_O_FR) & UART_FR_BUSY) ? true : false);
}

// flash.c からコピーし関数名に _dl を付加。 
int32_t
FlashErase_dl(uint32_t ui32Address)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ui32Address & (FLASH_ERASE_SIZE - 1)));

    //
    // Clear the flash access and error interrupts.
    //
    HWREG(FLASH_FCMISC) = (FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC |
                           FLASH_FCMISC_ERMISC);

    //
    // Erase the block.
    //
    HWREG(FLASH_FMA) = ui32Address;
    HWREG(FLASH_FMC) = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;

    //
    // Wait until the block has been erased.
    //
    while(HWREG(FLASH_FMC) & FLASH_FMC_ERASE)
    {
    }

    //
    // Return an error if an access violation or erase error occurred.
    //
    if(HWREG(FLASH_FCRIS) & (FLASH_FCRIS_ARIS | FLASH_FCRIS_VOLTRIS |
                             FLASH_FCRIS_ERRIS))
    {
        return(-1);
    }

    //
    // Success.
    //
    return(0);
}

// flash.c からコピーし関数名に _dl を付加。 
int32_t
FlashProgram(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ui32Address & 3));
    ASSERT(!(ui32Count & 3));

    //
    // Clear the flash access and error interrupts.
    //
    HWREG(FLASH_FCMISC) = (FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC |
                           FLASH_FCMISC_INVDMISC | FLASH_FCMISC_PROGMISC);

    //
    // Loop over the words to be programmed.
    //
    while(ui32Count)
    {
        //
        // Set the address of this block of words.
        //
        HWREG(FLASH_FMA) = ui32Address & ~(0x7f);

        //
        // Loop over the words in this 32-word block.
        //
        while(((ui32Address & 0x7c) || (HWREG(FLASH_FWBVAL) == 0)) &&
              (ui32Count != 0))
        {
            //
            // Write this word into the write buffer.
            //
            HWREG(FLASH_FWBN + (ui32Address & 0x7c)) = *pui32Data++;
            ui32Address += 4;
            ui32Count -= 4;
        }

        //
        // Program the contents of the write buffer into flash.
        //
        HWREG(FLASH_FMC2) = FLASH_FMC2_WRKEY | FLASH_FMC2_WRBUF;

        //
        // Wait until the write buffer has been programmed.
        //
        while(HWREG(FLASH_FMC2) & FLASH_FMC2_WRBUF)
        {
        }
    }

    //
    // Return an error if an access violation occurred.
    //
    if(HWREG(FLASH_FCRIS) & (FLASH_FCRIS_ARIS | FLASH_FCRIS_VOLTRIS |
                             FLASH_FCRIS_INVDRIS | FLASH_FCRIS_PROGRIS))
    {
        return(-1);
    }

    //
    // Success.
    //
    return(0);
}

// gpio.c からコピーし関数名に _dl を付加。 
void
GPIOPinWrite_dl(uint32_t ui32Port, uint8_t ui8Pins, uint8_t ui8Val)
{
    //
    // Check the arguments.
    //
    ASSERT(_GPIOBaseValid(ui32Port));

    //
    // Write the pins.
    //
    HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = ui8Val;
}

// watchdog.c からコピーし関数名に _dl を付加。 
void
WatchdogReloadSet_dl(uint32_t ui32Base, uint32_t ui32LoadVal)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == WATCHDOG0_BASE) || (ui32Base == WATCHDOG1_BASE));

    //
    // Set the load register.
    //
    HWREG(ui32Base + WDT_O_LOAD) = ui32LoadVal;
}

// watchdog.c からコピーし関数名に _dl を付加。 
void
WatchdogResetEnable_dl(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Base == WATCHDOG0_BASE) || (ui32Base == WATCHDOG1_BASE));

    //
    // Enable the watchdog reset.
    //
    HWREG(ui32Base + WDT_O_CTL) |= WDT_CTL_RESEN;
}

inline void __bit_output_dl(unsigned long port, unsigned char pin, unsigned char bit_data)
{
    GPIOPinWrite_dl(port, 1 << pin, bit_data << pin);
}

/********************************************************/
/*  ディレイ                                            */
/*      入力 delay   : カウント値                       */
/********************************************************/
void delay_dl(unsigned short delay_count){

    volatile unsigned short counter;
    
    for (counter = 0; counter < delay_count; counter++){
        ;
    }
}

/****************************************************/
/*  RS485受信許可（1フレーム受信開始）              */
/*    (メンテナンスポート)                          */
/*  byte_count_dl = 0;      カウンタリセット        */
/*  rx_end_dl = 0           受信完了フラグリセット  */
/*  SCI5.SCR.BIT.RE = 1;    受信ENABLE              */
/*                                                  */
/****************************************************/
void    RX_enable_dl_ment(void){

    byte_count_dl = 0;              /*カウンタリセット*/
    rx_end_dl = 0;                  /*受信完了フラグリセット*/

    memset_dl(TX_buf_dl, 0, sizeof(TX_buf_dl));    /*送受信バッファクリア*/
    memset_dl(RX_buf_dl, 0, sizeof(RX_buf_dl));

    // GPIO_PD6 
    __bit_output_dl(GPIO_PORTD_BASE, 6, 0);
    
    receive_enable = 1;	/*受信ENABLE	*/
}

/****************************************************/
/*  RS485受信許可（1フレーム受信開始）              */
/*    (ホストポート) 		                        */
/*  byte_count_dl = 0;      カウンタリセット        */
/*  rx_end_dl = 0           受信完了フラグリセット  */
/*  SCI5.SCR.BIT.RE = 1;    受信ENABLE              */
/*                                                  */
/****************************************************/
void    RX_enable_dl_host(void){

    byte_count_dl = 0;              /*カウンタリセット*/
    rx_end_dl = 0;                  /*受信完了フラグリセット*/

    memset_dl(TX_buf_dl, 0, sizeof(TX_buf_dl));    /*送受信バッファクリア*/
    memset_dl(RX_buf_dl, 0, sizeof(RX_buf_dl));

    // GPIO_PP4 
    __bit_output_dl(GPIO_PORTP_BASE, 4, 0);
    
    receive_enable = 1;	/*受信ENABLE	*/
}

/****************************************************/
/*  RS485受信割込み処理(メンテナンスポート)         */
/*                                                  */
/*  受信データをRX_buf_dl[]に入れる                 */
/*  byte_count_dl = 0;      カウンタリセット        */
/*  rx_end_dl = 0;          受信完了フラグリセット  */
/*  SCI5.SCR.BIT.RE = 1;    受信ENABLE              */
/*                                                  */
/****************************************************/
void    RX_data_dl_ment(void){
    
    //char stat,dumy;
    short i;
    unsigned short work;
    uint32_t ui32Status;

//REPEAT:

	// Get the interrrupt status.
	ui32Status = UARTIntStatus_dl(UART_COM_MENT_BASE, true);

	// Clear the asserted interrupts.
	UARTIntClear_dl(UART_COM_MENT_BASE, ui32Status);

	if(receive_enable != 1) { /* 受信DISABLE中 */
		/* 受信データ読み捨て */
		if (UARTCharsAvail_dl(UART_COM_MENT_BASE)) {
			UARTCharGetNonBlocking_dl(UART_COM_MENT_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*正常*/
		if (UARTCharsAvail_dl(UART_COM_MENT_BASE)) {	/*受信OK*/
            RX_buf_dl[byte_count_dl] = UARTCharGetNonBlocking_dl(UART_COM_MENT_BASE);/*1byte データ受信*/
            if ( byte_count_dl == 0) {          /*read top data*/
                if (RX_buf_dl[0]=='S'){ /*先頭文字*/
                    byte_count_dl++;    /*next RX_buf pointer*/
                } else {
                    //for(i=0; i<2; i++);
                    //dumy =100*dumy/10;
                }
            }else{
                if (((RX_buf_dl[byte_count_dl]>='A') && (RX_buf_dl[byte_count_dl]<='Z'))/*有効コード*/
                 ||((RX_buf_dl[byte_count_dl]>='a') && (RX_buf_dl[byte_count_dl]<='z'))
                 ||((RX_buf_dl[byte_count_dl]>='0') && (RX_buf_dl[byte_count_dl]<='9')) 
                 ||(RX_buf_dl[byte_count_dl]=='.')
                 ||(RX_buf_dl[byte_count_dl]==',')
                 ||(RX_buf_dl[byte_count_dl]==CR)
                ){
                    if ( byte_count_dl < MSG_MAX_DL) byte_count_dl++;  /*next RX_buf pointer*/
                }
                else{
                    byte_count_dl =0;                           /*counter initialize*/
                }
            }
        }else{
            //goto    REPEAT;             /*やり直し*/
        }
    } else {                            /*受信エラー*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*オーバーランエラー*/
            err_stat_dl |= 0x0004;
        }
		if ((ui32Status & UART_INT_FE) != 0) {	/*フレーミングエラー*/
            err_stat_dl |= 0x0002;
        }
		if ((ui32Status & UART_INT_PE) != 0) {	/*パリティエラー*/
            err_stat_dl |= 0x0001;
        }
        return;
    }
    /*終了チェック*/
    if (RX_buf_dl[byte_count_dl-1] == CR){  /*受信終了*/
		receive_enable = 0;            /*受信DISABLE*/

        rx_end_dl = 1;                  /*フレーム受信完了フラグをセット*/
    }
}

/****************************************************/
/*  RS485受信割込み処理(ホストポート)               */
/*                                                  */
/*  受信データをRX_buf_dl[]に入れる                 */
/*  byte_count_dl = 0;      カウンタリセット        */
/*  rx_end_dl = 0;          受信完了フラグリセット  */
/*  SCI6.SCR.BIT.RE = 1;    受信ENABLE              */
/*                                                  */
/****************************************************/
void    RX_data_dl_host(void){
    
    //char stat,dumy;
    short i;
    unsigned short work;
    uint32_t ui32Status;

//REPEAT:

	// Get the interrrupt status.
	ui32Status = UARTIntStatus_dl(UART_COM_HOST_BASE, true);

	// Clear the asserted interrupts.
	UARTIntClear_dl(UART_COM_HOST_BASE, ui32Status);

	if(receive_enable != 1) { /* 受信DISABLE中 */
		/* 受信データ読み捨て */
		if (UARTCharsAvail_dl(UART_COM_HOST_BASE)) {
			UARTCharGetNonBlocking_dl(UART_COM_HOST_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*正常*/
		if (UARTCharsAvail_dl(UART_COM_HOST_BASE)) {	/*受信OK*/
            RX_buf_dl[byte_count_dl] = UARTCharGetNonBlocking_dl(UART_COM_HOST_BASE);/*1byte データ受信*/
            if ( byte_count_dl == 0) {          /*read top data*/
                if (RX_buf_dl[0]=='S'){ /*先頭文字*/
                    byte_count_dl++;    /*next RX_buf pointer*/
                } else {
                    //for(i=0; i<2; i++);
                    //dumy =100*dumy/10;
                }
            }else{
                if (((RX_buf_dl[byte_count_dl]>='A') && (RX_buf_dl[byte_count_dl]<='Z'))/*有効コード*/
                 ||((RX_buf_dl[byte_count_dl]>='a') && (RX_buf_dl[byte_count_dl]<='z'))
                 ||((RX_buf_dl[byte_count_dl]>='0') && (RX_buf_dl[byte_count_dl]<='9')) 
                 ||(RX_buf_dl[byte_count_dl]=='.')
                 ||(RX_buf_dl[byte_count_dl]==',')
                 ||(RX_buf_dl[byte_count_dl]==CR)
                ){
                    if ( byte_count_dl < MSG_MAX_DL) byte_count_dl++;  /*next RX_buf pointer*/
                }
                else{
                    byte_count_dl =0;                           /*counter initialize*/
                }
            }
        }else{
            //goto    REPEAT;             /*やり直し*/
        }
    } else {                            /*受信エラー*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*オーバーランエラー*/
            err_stat_dl |= 0x0004;
        }
		if ((ui32Status & UART_INT_FE) != 0) {	/*フレーミングエラー*/
            err_stat_dl |= 0x0002;
        }
		if ((ui32Status & UART_INT_PE) != 0) {	/*パリティエラー*/
            err_stat_dl |= 0x0001;
        }
        return;
    }
    /*終了チェック*/
    if (RX_buf_dl[byte_count_dl-1] == CR){  /*受信終了*/
		receive_enable = 0;	/*受信DISABLE*/

        rx_end_dl = 1;                  /*フレーム受信完了フラグをセット*/
    }
}

/****************************************************/
/*  RS485送信開始（1フレーム送信開始）              */
/*    (メンテナンスポート)                          */
/*  TX_buf_dl[] の内容を送信する                    */
/*  入力：byte_su 送信バイト数                      */
/*                                                  */
/****************************************************/
void    TX_data_dl_ment(short byte_su){
    short i,j;
    
    if ( byte_su == 0) return;  /*BYTE数が０なら何もしない*/

    // GPIO_PD6 
    __bit_output_dl(GPIO_PORTD_BASE, 6, 1);

    delay_dl(10000);                /*待機*/

	for(i = 0; i < byte_su;  i++)
	{
		UARTCharPut_dl(UART_COM_MENT_BASE, TX_buf_dl[i]);
	}

	while(1)
	{
		if(UARTBusy_dl(UART_COM_MENT_BASE)!= true)
		{
			break;
		}
	}

    // GPIO_PD6 
    __bit_output_dl(GPIO_PORTD_BASE, 6, 0);

    RX_enable_dl_ment();            /*受信許可*/

//  setpsw_i();                     /* Initialize CCR/Interrupt Enable */
}

/****************************************************/
/*  RS485送信開始（1フレーム送信開始）              */
/*    (ホストポート)  		                        */
/*  TX_buf_dl[] の内容を送信する                    */
/*  入力：byte_su 送信バイト数                      */
/*                                                  */
/****************************************************/
void    TX_data_dl_host(short byte_su){
    short i,j;
    
    if ( byte_su == 0) return;  /*BYTE数が０なら何もしない*/

    // GPIO_PP4 
    __bit_output_dl(GPIO_PORTP_BASE, 4, 1);
    
    delay_dl(10000);                /*待機*/

	for(i = 0; i < byte_su;  i++)
	{
		UARTCharPut_dl(UART_COM_HOST_BASE, TX_buf_dl[i]);
	}

	while(1)
	{
		if(UARTBusy_dl(UART_COM_HOST_BASE)!= true)
		{
			break;
		}
	}

    // GPIO_PP4 
    __bit_output_dl(GPIO_PORTP_BASE, 4, 0);

    RX_enable_dl_host();            /*受信許可*/

//  setpsw_i();                     /* Initialize CCR/Interrupt Enable */
}

////////////////////////////////////////////////////////////
// ストリングセット ワード(16bit)
//  R1 ポインタ R2 データ　R3 サイズ
//  dst(R1)からsdata(R2)をsize(R3)分書込む
///////////////////////////////////////////////////////////
#pragma inline_asm memsetW
void memsetW(unsigned short *dst, unsigned short sdata, unsigned long size) {
	unsigned long i;
	for(i = 0; i < size; i++)
	{
		*(dst++) = sdata;
	}
}

//////////////////////////////////////////////////////////////
//// ストリングセット バイト(16bit)
////  R1 ポインタ R2 データ　R3 サイズ
////  dst(R1)からsdata(R2)をsize(R3)分書込む
/////////////////////////////////////////////////////////////
//#pragma inline_asm fdl_memcpy
//void fdl_memcpy(unsigned char *src, unsigned char *dst, short size) {
//    // R2：元 R1：先 R3:回数 処理サイズ:バイト
//    SMOVF
//}

//////////////////////////////////////////////////////////////////
//  RAMで実行                                                   //
//  アドレスを含むブロックのイレースを行う                      //
//リターン 0 正常                                               //
//                                                              //
//////////////////////////////////////////////////////////////////
void block_erase(void *ptr) {

	FlashErase_dl((uint32_t)ptr);
}

///////////////////////////////////////////////////////////////////////////////
//  RAMで実行
//  256バイトのデータを書き込む
//  data[]のうち書換えない部分は0xffffに予めする必要がある。
///////////////////////////////////////////////////////////////////////////////
short FROM_WRITE(void *ptr, unsigned short data[]) {

	if(((unsigned long)&data & 3) != 0)
	{
		return -1;
	}
	FlashProgram((uint32_t *)data, (uint32_t)ptr, 0x100);
	
	return 0;
}

/*******************************************
 * Function : FPGA_WRITE
 * Summary  : Flashデータの書き込み
 * Argument : *ptr -> 書き込み先アドレス
 *          : data -> 書き込むデータ
 * Return   : WrtFlg -> 0 : 成功
 *                     -1 : 失敗
 * Caution  : None
 * Note     : 
 * *****************************************/
short FPGA_WRITE(void *ptr, unsigned short data[]) {
    short WrtFlg = 0;

    //アドレス不正
	if(((unsigned long)&data & 3) != 0)
	{
		WrtFlg = -1;
	}
    else
    {
	    WrtFlg = FlashProgram((uint32_t *)data, (uint32_t)ptr, 0x100);
    }

	return WrtFlg;
}


typedef struct {
unsigned char   type;               // レコードのタイプ
unsigned short  record_len;         // レコード長
unsigned long   head_addr;          // 先頭のアドレス
unsigned long   tail_addr;          // 末尾のアドレス
unsigned char   data[256];          // データのバッファ最大256byte
unsigned char   data_len;           // データの長さ
unsigned char   sum;                // チェックサム
unsigned char   sum_calc;           // チェックサム計算値
}S_FORMAT;

S_FORMAT    s_data;

////////////////////////////////////////////////////////////////////////////////
//  アスキー1文字を数字に直す                                                 //
//    0-F（30-39、41-46まで出なければならない                                 //
//     他の場合は、動作未定                                                   //
//     Uchar ascii2chr(Uchar cdata)                                           //
////////////////////////////////////////////////////////////////////////////////
unsigned char ascii2char(unsigned char data) {

    if(((unsigned char)'0' <= data) && (data <= (unsigned char)'9')) return (unsigned char)(data - '0');
    if(((unsigned char)'A' <= data) && (data <= (unsigned char)'F')) return (unsigned char)(data - 'A' + 10);
    return  (unsigned char)0x20;
}

/******************************************************************************************
   Sフォーマット解析
    ptr      :Sフォーマットのデータ列
    s_data   :結果の格納場所
    リターン： 0:正常
              -1:先頭が'S'でない
              -2:タイプがおかしい
              -4:SUMエラー
*******************************************************************************************/
short s_read(char *ptr) {
    unsigned char data,datal,datah;
    unsigned long ltmp;
    short cnt;

    s_data.type         = (unsigned char)0;
    s_data.record_len   = (unsigned short)0;
    s_data.head_addr    = (unsigned long)0x00000000;
    s_data.tail_addr    = (unsigned long)0x00000000;
    s_data.data[0]      = NULL;
    s_data.data_len     = (unsigned char)0;
    s_data.sum          = (unsigned char)0;
    s_data.sum_calc     = (unsigned char)0;

    // 先頭が'S'で無ければ、エラー return -1
    if(*ptr++ != 'S') return -1;

    // レコードタイプ
    s_data.type = *ptr++ - '0';

    // レコード長
    datah = (unsigned char)ascii2char(*ptr++);
    datal = (unsigned char)ascii2char(*ptr++);
    data = (unsigned char)(datah * 16 + datal);
    s_data.record_len = s_data.sum_calc = data;

    switch (s_data.type){
    case 0:     // 4バイトアドレス
    case 1:
    case 9:
        s_data.data_len = s_data.record_len - ( 2+1 );
        break;
    case 2:     // 6バイトアドレス
    case 8:
        s_data.data_len = s_data.record_len - ( 3+1 );
        break;
    case 3:     // 8バイトアドレス
    case 7:
        s_data.data_len = s_data.record_len - ( 4+1 );
        break;
     }// switch (s_data.type)

    ltmp = 0UL;
    // アドレス
    switch (s_data.type){
    case 3:     // 8バイトアドレス
    case 7:
        datah =ascii2char(*ptr++);
        datal =ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 + datal);
        s_data.sum_calc += data;
        ltmp = data;
        break;
    case 2:     // 6バイトアドレス
    case 8:
        datah =ascii2char(*ptr++);
        datal =ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 + datal);
        s_data.sum_calc += data;
        ltmp = ltmp * 256UL + data;
        break;
    case 0:     // 4バイトアドレス
    case 1:
    case 9:
        datah =ascii2char(*ptr++);
        datal =ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 + datal);
        s_data.sum_calc += data;
        ltmp = ltmp * 256UL + data;
        datah =ascii2char(*ptr++);
        datal =ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 + datal);
        s_data.sum_calc += data;
        ltmp = ltmp * 256UL + data;
        break;
    default:
        return -2 ; /*エラー*/
    }// switch (s_data.type)
    s_data.head_addr = ltmp;
    s_data.tail_addr = s_data.head_addr + s_data.data_len - 1;

    // データ
    for(cnt=0 ; cnt < (s_data.data_len) ; cnt++ ){ /* バイトカウントは、アドレスからサムまでのため2減らす*/
        datah = ascii2char(*ptr++);
        datal = ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 +datal);
        s_data.sum_calc += data;
        s_data.data[cnt] = data;
    }

    datah = ascii2char(*ptr++);
    datal = ascii2char(*ptr++);
    s_data.sum = (unsigned char)(datah * 16 +datal);

    if((s_data.sum^s_data.sum_calc) == (unsigned char)0xff) {
        return 0;   /* 正常終了 */
    } else {
        return -4;  /* SUMエラー*/
    }
}


//////////////////////////////////////////////////////////////////////////////
//  short get_block_id(unsigned long addr)
//      入力：ROMアドレス
//      戻り値：ROMアドレスが含まれるブロックのID
//
//////////////////////////////////////////////////////////////////////////////
short get_block_id(unsigned long addr) {

	if(addr >= 0x00100000)
	{
		return -1;
	}
	else
	{
		return (addr / 0x4000);
	}
}

//////////////////////////////////////////////////////////////////////////////
//  1レコード単位の処理
//  short write_data()
//      入力：S_FORMAT s_data
//      戻り値：0 : 最終レコード
//              1 : 最終レコード以外
//
//  新ブロックなら、そのブロックを消去
//  書き込むべきデータはバッファに蓄えておく
//  256byte境界を越えたら、そこまでのバッファのデータを書き込み
//  最終レコードで、バッファのデータを書き込む
//
//////////////////////////////////////////////////////////////////////////////
unsigned short  ROM_WR_BUFF[128];
short           prev_block_id;
short           curr_block_id;
short           curr_block_id;
unsigned long   prev_addr_256;
unsigned long   curr_head_addr_256;
unsigned long   curr_tail_addr_256;
short           pending_size;

short write_data(void) {
    short offset;
    short write_size;

    // データブロックで、長さが1以上の場合
    if( ( (s_data.type == 1) || (s_data.type == 2) || (s_data.type == 3) ||
          (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) && (s_data.data_len > 0) ) {

        // 異なるブロックなら、新ブロックを消去
        curr_block_id = get_block_id(s_data.head_addr);
        if(prev_block_id != curr_block_id) {
            block_erase((void *)s_data.head_addr);
            prev_block_id = curr_block_id;
        }
        curr_block_id = get_block_id(s_data.tail_addr);
        if(prev_block_id != curr_block_id) {
            block_erase((void *)s_data.tail_addr);
            prev_block_id = curr_block_id;
        }

        // 異なる256byteなら、これまでのデータを書き込む
        curr_head_addr_256 = s_data.head_addr & 0xFFFFFF00UL;
        curr_tail_addr_256 = s_data.tail_addr & 0xFFFFFF00UL;

        if(prev_addr_256 != curr_head_addr_256) {                   // 先頭が異なる256byteの場合
            if(pending_size > 0) {
                FROM_WRITE((void *)prev_addr_256, ROM_WR_BUFF);
            }
            memsetW(ROM_WR_BUFF, 0xffff, 128);                      // 今後のためにバッファを0xffで埋める
            pending_size = 0;
            prev_addr_256 = curr_head_addr_256;
        }

        // バッファに書き込みデータをマージする
        offset = (short)(s_data.head_addr & 0x000000FFUL);
        write_size = min_dl(256 - offset, s_data.data_len);
        memcpy_dl((void*)((unsigned long)(&ROM_WR_BUFF[0]) + offset), (void*)(&s_data.data[0]), write_size);
        pending_size += write_size;

        if(curr_head_addr_256 != curr_tail_addr_256) {              // 分割される場合
            FROM_WRITE((void *)curr_head_addr_256, ROM_WR_BUFF);        // 前半を書き込む

            memsetW(ROM_WR_BUFF, 0xffff, 128);                          // 後半のためにバッファを0xffで埋める
            pending_size = s_data.data_len - write_size;
            memcpy_dl(ROM_WR_BUFF, (void*)(&s_data.data[write_size]), pending_size);
            prev_addr_256 = curr_tail_addr_256;
        }
    }
    // 最終レコードの場合
    if( (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) {
        if(pending_size > 0) {
            FROM_WRITE((void *)prev_addr_256, ROM_WR_BUFF);
        }
        return 0;
    }
    return 1;
}

#if DL_DEBUG
//////////////////////////////////////////////////////////////////////////////
//  1レコード単位の処理
//  short check_data()
//      入力：S_FORMAT s_data
//      戻り値：0 : 最終レコード
//              1 : 最終レコード以外
//
//      比較で異常があれば、無限ループに入る。デバッグ用
//
//////////////////////////////////////////////////////////////////////////////
short check_data(void) {
    short i;

    // データブロックで、長さが1以上の場合
    if( ( (s_data.type == 1) || (s_data.type == 2) || (s_data.type == 3) ||
          (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) && (s_data.data_len > 0) ) {

        for(i = 0; i < s_data.data_len; i++) {
            if(*((unsigned char *)(s_data.head_addr + (unsigned long)i)) != s_data.data[i]) {
                for(;;);
            }
        }
    }

    // 最終レコードの場合
    if( (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) {
        return 0;
    }
    return 1;
}
#endif

/************************************************/
/* LED表示更新                                  */
/*          入力：表示LED                       */
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
/************************************************/
void    drive_led_dl(unsigned short led_num){

	FPGA_LED_CNT = led_num;		//LED制御レジスタに設定
}

/***************************************************************/
/* LEDを、読み込み行数10増える毎にシフトするように点灯する     */
/* switch文を使うと別のSEGにテーブルが作られるので、if文で記述 */
/***************************************************************/
void led_dl(short count) {

    count /= 10;
    count %= 10;
    if(count == 0)      drive_led_dl(DSP_ALM1);
    else if(count == 1) drive_led_dl(DSP_ALM2);
    else if(count == 2) drive_led_dl(DSP_ALM3);
    else if(count == 3) drive_led_dl(DSP_ALM4);
    else if(count == 4) drive_led_dl(DSP_LED1);
    else if(count == 5) drive_led_dl(DSP_LED2);
    else if(count == 6) drive_led_dl(DSP_LED3);
    else if(count == 7) drive_led_dl(DSP_LED4);
    else if(count == 8) drive_led_dl(DSP_LED5);
    else                drive_led_dl(DSP_LED6);
}

/****************************************************************/
/*	メール受信待機												*/
/****************************************************************/
void	mky43_wait_mail_recv_dl(void){

	while(MKY43.REG.MR0CR.BIT.RCV == 0);		//メール受信完了待機
	
}

/****************************************************************/
/*	送信データをsend_data_dl[]に保存							*/
/****************************************************************/
void	mky43_TX_start_dl(short byte_num){

	short i;
	short				j_cnt;
	unsigned short	send_sa;			//送信先ステーションアドレス
	unsigned short	resend_cnt;

	// send_data_dl[32][4]  = { 0 } で初期値をセットすると例外発生する。 
	// 後続処理にて memset_dl で初期化しているのでここでの初期化は不要。 
	unsigned short	send_data_dl[32][4];		//メール送信データ
	
	/*先頭アドレス(1byte)に『CHデータ(0固定)』を入れる*/
	/*CUnet通信時、ホストからのメッセージ時に限る*/
	for(i=byte_num; i>=0; i--){	
		TX_buf_dl[i+1] = TX_buf_dl[i];
	}
	//TX_buf_dl[0] = 0x30;
	TX_buf_dl[0] = 0;	

	memset_dl(send_data_dl, 0, sizeof(send_data_dl));
	for(i=0; i<4; i++){
		send_data_dl[0][i] = TX_buf_dl[2*i + 1]*0x0100 + TX_buf_dl[2*i];
	}

	send_sa = recv_sa_dl;						//メール送信先ステーションアドレス
	resend_cnt = 0;

	while(MKY43.REG.MSCR.BIT.SEND == 1);		//メール送信中は待機
		
	MKY43.REG.MESR.WORD = 0;					//メール送信エラーステータスクリア

	for(j_cnt=0; j_cnt<4; j_cnt++){
		MKY43.MSB.SEND[0].DATA[j_cnt].DATA = send_data_dl[0][j_cnt];
	}

RESEND:
	
	MKY43.REG.MSCR.WORD = (0x0000 | (send_sa<<8) | 1);	//送信先ステーションアドレス、データサイズ
	MKY43.REG.MSCR.BIT.SEND = 1;				//メール送信開始

	while(MKY43.REG.MSCR.BIT.SEND == 1);		//メール送信中は待機

	if(resend_cnt < MES_RESEND_MAX){
		if(MKY43.REG.MSCR.BIT.ERR != 0){			//メール送信エラー発生
			if(MKY43.REG.MESR.BIT.NORDY != 0){		//送信先の受信バッファが受信許可でない
				MKY43.REG.MESR.WORD = 0;			//メール送信エラーステータスクリア
				resend_cnt ++;						//再送回数更新
				goto RESEND;						//再送処理
			}
		}
	}
    rx_end_dl = 0;             /*受信完了フラグリセット*/
	
	for (i=0; i<MSG_MAX_DL; i++){
		RX_buf_dl[i] = 0;		/*送受信バッファクリア*/
		TX_buf_dl[i] = 0;
	}
}

/****************************************************************/
/* メール受信(MRB0:メール受信バッファ0							*/
/****************************************************************/
void	mky43_mail_recv_dl(void){

	short	i_cnt;
	short	j_cnt;
	short i;
	short j;
	short buf_num;
	short buf_offset;
	short byte_count_host;
	unsigned short data_len;
	unsigned short	recv_size;		//メールデータサイズ

	// recv_data0_dl[32][4] = { 0 } で初期値をセットすると例外発生する。 
	// 後続処理にて memset_dl で初期化しているのでここでの初期化は不要。 
	unsigned short	recv_data0_dl[32][4];		//メール受信データ(MRB0:メール受信バッファ0)

	recv_sa_dl = 0;
	recv_size = 0;
	memset_dl(recv_data0_dl, 0, sizeof(recv_data0_dl));
	
	/*MRB0:メール受信バッファ0*/
	if(MKY43.REG.MR0CR.BIT.RCV != 0){					//メール受信完了
		recv_sa_dl = (MKY43.REG.MR0CR.WORD & 0x3F00);	//送信元ステーションアドレス取得
		recv_size = (MKY43.REG.MR0CR.WORD & 0x003F);	//メール受信データサイズ取得(8Byteを1単位)

		for(i_cnt=0; i_cnt<recv_size; i_cnt++){		//メール受信データ取得
			for(j_cnt=0; j_cnt<4; j_cnt++){
				recv_data0_dl[i_cnt][j_cnt] = MKY43.MRB0.RECV[i_cnt].DATA[j_cnt].DATA;
			}
		}
	
		buf_offset = buf_count_dl;
	
		recv_size *= 8;
		for(buf_num=0; buf_num<recv_size; buf_num++){
			i = buf_num/8;
			j = (buf_num%8)/2;	
			if(buf_num%2 == 0){
				/*recv_data0[]を保存*/
				RX_buf_dl[buf_num+buf_offset] = recv_data0_dl[i][j]%0x0100;
			}else{
				/*recv_data0[]を保存*/
				RX_buf_dl[buf_num+buf_offset] = recv_data0_dl[i][j]/0x0100;
			}
			if(RX_buf_dl[buf_num+buf_offset] != CR){		//CR(0x0D)なし
				buf_count_dl++;
			}else{											//CR(0x0D)あり
				buf_count_dl = 0;
				break;
			}
			if((buf_num+buf_offset) > MSG_MAX_DL){
				break;
			}
		}

		if(buf_count_dl == 0){		//全メッセージ(SからCRまでのデータ)受信完了
			byte_count_host = 0;
			//data_len = strlen(RX_buf_dl);
			data_len = 0;
			while (RX_buf_dl[byte_count_host]!='S'){	/*先頭文字*/
				byte_count_host++;
			}
			while (RX_buf_dl[data_len]!= CR){	/*終端文字*/
				data_len++;
			}
			data_len++;
			data_len -= byte_count_host;
			for(i=0;i<data_len;i++){
				RX_buf_dl[i] = RX_buf_dl[i+byte_count_host];
			}
			RX_buf_dl[data_len] = 0;
			rx_end_dl = 1;                  /*フレーム受信完了フラグをセット*/
		}
		MKY43.REG.MR0CR.BIT.RDY = 1;					//メール受信許可
	}

	MKY43.REG.INT0CR.BIT.MR = 1;						//メール受信完了割込み許可	
	MKY43.REG.INT0SR.BIT.MR = 1;						//メール受信完了割込み解除
}

/******************************************************************************************
    download
    SフォーマットのHEXファイルをダウンロードして、ROMを更新する
    ダウンロードに関するコードは全てRAM上に無ければならない
        1. 割り込み禁止、WDT停止
        2. 通信の(再)設定
        3. 一行読み込む
        4. 解析する
        5. 一行分ROMに書き込む
        6. ACK/NAKを返す
        7. 終わりでなければ、3に戻る
        8. 終了すれば、IWDTをexpireさせてリセットする

	入力
		mode: 　0：ホストポート側通信
				1：メンテナンスポート側通信
		type:　 0x1000：RS485基板
				0x2000：CUnet基板		
*******************************************************************************************/
static void _download(short mode, short type) {
    short cont_flag;
    short line_num;

    prev_block_id = -1;
    prev_addr_256 = 0x00000000UL;
    pending_size = 0;

    cont_flag = 1;
    line_num = 0;

    while(cont_flag){
		if(type == COM_CUNET && mode == HOST){		//CUnet通信で受信する場合
			mky43_wait_mail_recv_dl();				//メール受信完了待機
			mky43_mail_recv_dl();					//メール受信(MRB0,MRB1)
		}else{										//RS485通信で受信する場合
			do{										//通信経由で１行読み込む
				if(mode == HOST){					//ホストポート
					RX_data_dl_host();
				}else{								//メンテナンスポート
					RX_data_dl_ment();
				}
			}while(rx_end_dl == 0);
		}

        if(s_read(RX_buf_dl) < 0) {     // 通信内容のチェック
            TX_buf_dl[0] = 0x21;            // NAKを準備
            TX_buf_dl[1] = 0x0d;
            TX_buf_dl[2] = 0x0a;
        } else {                            // ACKを準備
            TX_buf_dl[0] = 0x06;
            TX_buf_dl[1] = 0x0d;
            TX_buf_dl[2] = 0x0a;
        }

#if DL_DEBUG
        cont_flag = check_data();       // ROMのデータと比較する
#else
		cont_flag = write_data();       // ROMにデータを書き込む
#endif

		if(mode == HOST){				//ホストポート
			if(type == COM_RS485){		//RS485基板
				TX_data_dl_host(3);		//ACK/NAKを返す
			}else{						//CUnet基板
				mky43_TX_start_dl(3);	//ACK/NAKを返す
			}
		}else{							//メンテナンスポート
			TX_data_dl_ment(3);			//ACK/NAKを返す
		}
		led_dl(line_num++);
    }

    WatchdogReloadSet_dl(WATCHDOG1_BASE, g_ui32SysClock / 1000);	// 1ms 後に再起動 
    WatchdogResetEnable_dl(WATCHDOG1_BASE);

    for(;;);    // wait for underflow of IWDT
}


