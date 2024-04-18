/***********************************************/
/* File Name : eeprom_spi.c		      									   */
/*	Summary   : EEPROM処理				                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "machine.h"
#include "define.h"
#include "SV_def.h"
#include "defMES.h"
#include "defMAIN.h"

#define BYTE unsigned char
#define WORD unsigned short
#define BOOL int
#define TRUE 1
#define FALSE 0

// GPIO_PB4
// 「IO_CS=」を「IO_CS_」に置換 
#define IO_CS_0 (__bit_output(GPIO_PORTB_BASE, 4, 0))
#define IO_CS_1 (__bit_output(GPIO_PORTB_BASE, 4, 1))
// GPIO_PB5
// 「IO_SCK=」を「IO_SCK_」に置換 
#define IO_SCK_0 (__bit_output(GPIO_PORTB_BASE, 5, 0))
#define IO_SCK_1 (__bit_output(GPIO_PORTB_BASE, 5, 1))
// GPIO_PE4
// 「IO_SI=」を「IO_SCK_」に置換 
#define IO_SI_0 (__bit_output(GPIO_PORTE_BASE, 4, 0))
#define IO_SI_1 (__bit_output(GPIO_PORTE_BASE, 4, 1))
// GPIO_PE5
#define IO_SO (__bit_input(GPIO_PORTE_BASE, 5))

#define IO_HOLD 
#define IO_WP 

//Define op-code
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_RDSR 0x05
#define CMD_WRSR 0x01
#define CMD_READ 0x03
#define CMD_WRITE 0x02
//Define maximum Write Cycle polling times
#define POLLING_NUM 500

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void WriteEE(BYTE Data);
BYTE ReadEE();
BOOL PollingEE();
void SetWEN();
void ClrWEN();
BYTE ReadSR();
void WriteSR(BYTE Data);
void Write(WORD Address,BYTE *Pdata,BYTE Length);
void Read(WORD Address,BYTE *Pdata,BYTE Length);
short	eep_read(short rom_addr);
void	eep_write(short rom_addr,short data);
void	eep_sv_read(short pch);
void	eep_sv_write(short pch);
void	eep_write_ch(short pch, short rom_addr,short data);
short eep_spi_error(void);
void put_queue(short addr, short data);
void get_queue(short *addr, short *data);
short check_queue(void);
void eep_write_ch_delay (short pch, short rom_addr, short data);
void eep_write_pending_data();

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern void	action_status_control(short pch, short act);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short eep_err;
extern stSVD SVD_default;

/****************************************************/
/* Function : WriteEE                    */
/* Summary  : Bitデータ書込み    				*/
/* Argument : Data                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void WriteEE(BYTE Data)
{
	BYTE i;
	IO_SCK_0;

	//Shift bit of Data from MSb to LSb
	for(i=0;i<8;i++)
	{
		if(Data&0x80)
			IO_SI_1;
		else
			IO_SI_0;
		Data<<=1;
		IO_SCK_1;
		IO_SCK_0;
	}
	//Release SI
	IO_SI_1;
}

/****************************************************/
/* Function : ReadEE                    */
/* Summary  : Bitデータ読込み    				*/
/* Argument : なし                  	               */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
BYTE ReadEE()
{
	BYTE i;
	BYTE SO_Temp;
	BYTE Data_In=0;
	IO_SCK_0;
	//Receive data bit by bit
	for(i=0;i<8;i++)
	{
		IO_SCK_1;
		if(IO_SO==1)
			SO_Temp=1;
		else
			SO_Temp=0;
		Data_In=(Data_In<<1)|SO_Temp;
		IO_SCK_0;
	}
	return Data_In;
}

/****************************************************/
/* Function : PollingEE                    */
/* Summary  : データ書込み時のポーリング    				*/
/* Argument : なし                  	               */
/* Return   : 1=正常　　0=異常                        */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
BOOL PollingEE()
{
	short i=POLLING_NUM;
	BYTE Status_Reg;
	while(i--) // Check the maximum Write Cycle polling times
	{
		Status_Reg=ReadSR(); //Read EEPROM status register
		Status_Reg&=0x01;
		if(!Status_Reg) {//Polling RDY bit in status register
			return TRUE;
		}
	}
	eep_err = 1;
	return FALSE;
}

/****************************************************/
/* Function : SetWEN                    */
/* Summary  : CMD_WREN書込み   				*/
/* Argument : なし                  	               */
/* Return   : なし　　                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void SetWEN()
{
	IO_CS_0;
	WriteEE(CMD_WREN);
	IO_CS_1;
}

/****************************************************/
/* Function : ClrWEN                    */
/* Summary  : CMD_WRDI書込み   				*/
/* Argument : なし                  	               */
/* Return   : なし　　                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void ClrWEN()
{
	IO_CS_0;
	WriteEE(CMD_WRDI);
	IO_CS_1;
}

/****************************************************/
/* Function : ReadSR                    */
/* Summary  : ステータスレジスタ読込み   				*/
/* Argument : なし                  	               */
/* Return   : なし　　                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
BYTE ReadSR()
{
	BYTE Status_Reg;
	IO_CS_0;
	WriteEE(CMD_RDSR);
	Status_Reg=ReadEE();
	IO_CS_1;
	return Status_Reg;
}

/****************************************************/
/* Function : WriteSR                    */
/* Summary  : ステータスレジスタ書込み   				*/
/* Argument : Data                  	               */
/* Return   : なし　　                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void WriteSR(BYTE Data)
{
	IO_CS_0;
	WriteEE(CMD_WRSR);
	WriteEE(Data);
	IO_CS_1;
}

/****************************************************/
/* Function : Write                    */
/* Summary  : 書込み処理   				*/
/* Argument : Address,  *Pdata, Length       */
/* Return   : なし　　                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void Write(WORD Address,BYTE *Pdata,BYTE Length)
{
	IO_CS_0;
	WriteEE(CMD_WRITE); //Send write instruction
	WriteEE(Address >> 8); //Send MSB of address
	WriteEE(Address & 0xff); //Send LSB of address
	while(Length--)
	{
		WriteEE(*Pdata); //Send data
		Pdata++;
	}
	IO_CS_1;
}

/****************************************************/
/* Function : Read                    */
/* Summary  : 読込み処理   				*/
/* Argument : Address,  *Pdata, Length       */
/* Return   : なし　　                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void Read(WORD Address,BYTE *Pdata,BYTE Length)
{
	IO_CS_0;
	WriteEE(CMD_READ); //Send read instruction
	WriteEE(Address >> 8); //Send MSB of address
	WriteEE(Address & 0xff); //Send LSB of address
	while(Length--)
	{
		*Pdata=ReadEE(); //Read data
		Pdata++;
	}
	IO_CS_1;
}

/****************************************************/
/* Function : eep_read                    */
/* Summary  : 読込み処理   				*/
/* Argument : rom_addr         */
/* Return   : EEPROMアドレス読込みデータ　                */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short	eep_read(short rom_addr)
{
	short data=0xffff;
	Read(rom_addr*2,(BYTE*)&data,2);
	return data;
}

/****************************************************/
/* Function : eep_write                    */
/* Summary  : 書込み処理   				*/
/* Argument : rom_addr,  data         */
/* Return   : なし                             */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	eep_write(short rom_addr,short data)
{
	SetWEN();
	Write(rom_addr*2,(BYTE*)&data,2);
//	ClrWEN();
	while(!PollingEE());
}

/****************************************************/
/* Function : eep_sv_read                    */
/* Summary  : CH読込み処理   				*/
/* Argument : pch        */
/* Return   : なし                             */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	eep_sv_read(short pch){

	short i;
	short *sv_add;
	
	if(pch >= CH_NUMMAX) return;
	sv_add = &SVD[pch].max_flow;	/*設定値、先頭アドレス*/
	for ( i=0; i<SV_NUM ;i++){
		*sv_add = eep_read(i+SIZEOFMODBUSADDR*pch);	/*read data*/
		sv_add++;
	}		
} 

/****************************************************/
/* Function : eep_sv_write                    */
/* Summary  : CH書込み処理   				*/
/* Argument : pch        */
/* Return   : なし                             */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	eep_sv_write(short pch){

	short i;
	short *sv_add;
		
	if(pch >= CH_NUMMAX) return;
	sv_add = &SVD_default.max_flow;	/*デフォルト値、先頭アドレス*/	
	for ( i=0; i<SV_NUM ;i++){
		eep_write(i+SIZEOFMODBUSADDR*pch,(short)(*sv_add));	/*write data*/
		sv_add++;
	}
} 

/****************************************************/
/* Function : eep_write_ch                    */
/* Summary  : CH書込み処理   				*/
/* Argument : pch, rom_addr, data        */
/* Return   : なし                             */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	eep_write_ch(short pch, short rom_addr,short data){
	
	//ch分のオフセットをつける
	if(pch >= CH_NUMMAX) return;

	eep_write(rom_addr + SIZEOFMODBUSADDR * pch, data);

	/*SPI通信エラーチェック*/
	if(eep_spi_error() != B_OK){
		MES[pch].err_status |= ERR_JUDGE_EEPROM;		/*EEPROMエラーセット*/
	}else{
		MES[pch].err_status &= ~ERR_JUDGE_EEPROM;		/*EEPROMエラーリセット*/
	}	
}

/****************************************************/
/* Function : eep_spi_error                    */
/* Summary  : SPI通信エラー検出 				*/
/* Argument : なし       */
/* Return   : エラーステータス                          */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short		eep_spi_error(void){

	short		sts;

	sts = B_OK;

	return (sts);
}

#define EEP_DEBUG	(1)

// EEPROM書き込みデータ用Queue
// EEPROMの書き込みに時間がかかるため、一旦Queueに情報を貯めておき
// 空き時間に書き込み処理を行う

#define QUEUE_SIZE			(1536)
#define EEP_WRITE_TIMEOUT	(500)
#define ENABLE_QUEUE_SIZE	(300)

typedef struct {
	short adr;
	short dat;
} EEP_QUEUE;

EEP_QUEUE eep_queue[QUEUE_SIZE];
short eep_rptr = 0;
short eep_wptr = 0;
short eep_empty = 1;

#if EEP_DEBUG
short eep_size = 0;		// デバッグ（モニタ）用
short eep_size_max = 0;
#endif

/****************************************************/
/* Function : put_queue                    */
/* Summary  : EEPROに書込むデータをQueueに追加する			*/
/* Argument : addr,  data       */
/* Return   : なし                           */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void put_queue(short addr, short data) {

#if EEP_DEBUG
if((eep_empty == 0) && (eep_wptr == eep_rptr)) for(;;);		// fullの時には追加できない
#endif
	
	eep_queue[eep_wptr].adr = addr;
	eep_queue[eep_wptr].dat = data;
	eep_wptr++;
	if(eep_wptr == QUEUE_SIZE)	eep_wptr = 0;
	eep_empty = 0;												// 追加したら、emptyではない

#if EEP_DEBUG
eep_size++;
if(eep_size > eep_size_max) eep_size_max = eep_size;
#endif
}

/****************************************************/
/* Function : get_queue                    */
/* Summary  : QueueからEEPROに書込むデータを取得する			*/
/* Argument : addr,  data       */
/* Return   : なし                           */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void get_queue(short *addr, short *data) {

#if EEP_DEBUG
if(eep_empty) for(;;);										// eep_emptyの時には読み出せない
#endif
	*addr = eep_queue[eep_rptr].adr;
	*data = eep_queue[eep_rptr].dat;
	eep_rptr++;
	if(eep_rptr == QUEUE_SIZE)	eep_rptr = 0;
	if(eep_rptr == eep_wptr) eep_empty = 1;						// 読み出した時、ptrが同じになればempty
	else                     eep_empty = 0;
#if EEP_DEBUG
eep_size--;
#endif
}

/****************************************************/
/* Function : check_queue                    */
/* Summary  : Queue取得可能か確認する		*/
/* Argument : なし      */
/* Return   : 0=可能,  -1=不可能                    */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short check_queue(void){

 short ret;

 ret = B_OK;
	if(((QUEUE_SIZE - eep_wptr) + eep_rptr) <= ENABLE_QUEUE_SIZE){
		ret = B_NG;
	}
	if((eep_rptr > eep_wptr) && ((eep_rptr - eep_wptr) <= ENABLE_QUEUE_SIZE)){
		ret = B_NG;
	}

 return ret;
}

/****************************************************/
/* Function : eep_write_ch_delay                    */
/* Summary  : EEPROMに空き時間に書き込む		*/
/* Argument : pch,  rom_addr,  data      */
/* Return   : なし　                   */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void eep_write_ch_delay (short pch, short rom_addr, short data){

	if(pch >= CH_NUMMAX) return;
	if(check_queue() != B_OK){			//EEPROM書込み用Queueが取得可能か確認する
		return;						//登録不可
	}

	//ch分のオフセットをつける
	put_queue(rom_addr + SIZEOFMODBUSADDR * pch, data);
}

short eep_write_retry = 0;

/****************************************************/
/* Function : eep_write_pending_data                */
/* Summary  : queueが空で無くて、EEPROMが書き込み中でなければ、		*/
/*          : EEPROMにデータを書き込む 		*/
/* Argument : pch,  rom_addr,  data      */
/* Return   : なし　                   */
/* Caution  : なし                                   */
/* notes    : main()の空き時間に呼ばれる                 */
/****************************************************/
void eep_write_pending_data() {
	BYTE Status_Reg;
	short rom_addr, data, ch;

	if(eep_empty){										// Queueが空の場合
  for(ch = CH1; ch < CH_NUMMAX; ch++){
   if(MAIN[ch].com_act_status == ACT_STS_WRITE){  //動作ステータスが設定書込み中の場合
    action_status_control(ch,ACT_STS_NORMAL);     //動作ステータスを通常にする
   }
  }
 	return;
 }

	Status_Reg = ReadSR();
	if(Status_Reg & 0x01) {			// Write In Progress
		if(eep_write_retry > 0) {
			eep_write_retry--;
			if(eep_write_retry == 0) eep_err = 1;		// EEP_WRITE_TIMEOUT回 WIPが続けばタイムアウト
		}
		return;
	} else {						// not Write In Progress
		get_queue(&rom_addr, &data);
		SetWEN();
		Write(rom_addr*2,(BYTE*)&data,2);
		eep_write_retry = EEP_WRITE_TIMEOUT;
	}
}

