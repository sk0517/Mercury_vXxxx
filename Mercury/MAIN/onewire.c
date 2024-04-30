/***********************************************/
/* File Name : onewire.c		         									   */
/*	Summary   : 1Wireメモリ読み書き処理              */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include <machine.h>

#include "define.h"
#include "SV_def.h"
#include "typedefine.h"
#include "defMES.h"
#include "defLOG.h"
#include "defMAIN.h"
#include "ctlioport.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

/********************************************************/
/*	定数定義(#define)								*/
/********************************************************/
#define DS28E07_FAMILYCODE 0x2D //DS28E07のFAMILY CODE
#define DS2431_FAMILYCODE 0x2D //DS2431のFAMILY CODE
#define DS18B20_FAMILYCODE 0x28 //DS18B20のFAMILY CODE
#define DS28E80_FAMILYCODE 0x4A //DS28E80のFAMILY CODE

#define FAMILY_CODE DS28E07_FAMILYCODE		//FAMILY CODE

#define TIM_tRSTL		500		//Reset Low Time(480-640us)
#define TIM_tPDH			15			//Presence Detect High Time(15-60us)
#define TIM_tPDL			60			//Presence Detect Low Time(60-240us)
#define TIM_tMSP			60		//Presence Detect Sample Time(60-75us)
#define TIM_tREC			5				//Recovery Time
#define TIM_tRSTH		305		//tPDHMAX(60)+tPDLMAX(240)+tRECMIN(5)	

#define TIM_tW1L			10				//Write-One Low Time(1-15us)
#define	TIM_tW0L  	60   //Write-Zero Low Time(52.1-120us)
#define TIM_tSLOT		65			//Time Slot Duration

#define	TIM_tRL   10   	//Read Low Time(5-15us)
#define TIM_tMSR  20				//Read Sample Time(TIM_tRL + 12)

#define	TIM_tPROGMAX   10   //Programming Time(10ms)

#define	WRITE_SCRATCHPAD	0x0F		//書込みコマンド
#define	READ_SCRATCHPAD		0xAA		//読込みコマンド
#define	COPY_SCRATCHPAD		0x55		//EEPROMコピーコマンド
#define	READ_MEMORY						0xF0		//メモリ読込みコマンド

#define	WRITE_BLOCK	0x55		//書込みコマンド
#define	WRITE_PRO_BLOCK		0xC3		//Write Protect Blockコマンド
#define	READ_PRO_BLOCK 		0xAA		//Read Block Protectionコマンド

#define	READ_ROM						0x33		//READ_ROMコマンド
#define	MATCH_ROM					0x55		//MATCH_ROMコマンド
#define	SEARCH_ROM				0xF0		//SERACH_ROMコマンド
#define	SKIP_ROM						0xCC		//SKIP_ROMコマンド

#define MEMORY_MAX			144		//メモリ最大バイト数

#define MSB_CRC8 (0x31)  // x8 + x5 + x4 + x0 標準 (先頭1bitは削除)
#define CHAR_BIT 8

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void OWSensSig(short sw);
void OWSensSigSW(short sw);
void OWSensPowSW(short sw);
void OWSensEnable(short sw);
short OWSensRX(void);
void OWSensSigPDR(short sw);
short OWResetDevice(void);
void OWWriteBit(short bit);
short OWReadBit(void);
void OWWriteByte(short data);
short OWReadByte(void);
unsigned char	GetReflect(unsigned char data);
unsigned char GetCRC8(const void *buff, short size);
short	OWCheckDevice(void);
short	OWCheckROMID(void);
void OWCheckConnect(short ch);
short	OWWriteData(short address, const void *buff);
short	OWCompareData(short address, const void *buff);
short	OWReadSensinfo(short ch);
void	OWWriteSensinfo(void);
void OWWaitUS(long us);
void OWWaitMS(long ms);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
//extern void portmesfwdW(unsigned char data, short pch);
extern void portmesrevW(unsigned char data, short pch);
extern void	util_eep_allwrite(short pch, short opt);
extern short	action_status_check(short pch);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
static short phase_owwrite = 1;
static	short phase_owconnect[6] = {1, 1, 1, 1, 1, 1};
short ow_pos, ow_cnt, ow_retry;
short OWwrite[6] = {0, 0, 0, 0, 0, 0};
short OWwriteCH = CH1;
char sens_info[256];
char memory_R_data[MEMORY_MAX];
char memory_W_data[MEMORY_MAX];
char memory_C_data[MEMORY_MAX];
short OW_read[5];
short OW_write[3];
short OW_comp[3];

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/


//メモリデバイスに保存するデータ
struct OWEeprom
{
	unsigned short	ch;		//CH番号
	short *p;		//保存先頭位置
	unsigned short	size;	//保存ｻｲｽﾞ
};

//メモリデバイスに保存するデータ
// ※各CHのメモリデバイスの保存データ量は1kbit以内であること
const struct OWEeprom OWEepromData[] = {
 //1ch ﾕｰｻﾞﾊﾟﾗﾒｰﾀ(12byte)
	{ CH1, (short *)&SVD[CH1].max_flow,	(2*(&SVD[CH1].low_cut - &SVD[CH1].max_flow + 1))},
	//1ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正点数、小数点位置)(2byte)
	{ CH1, (short *)&SVD[CH1].uslnr_num,	(2*(&SVD[CH1].uslnr_num - &SVD[CH1].uslnr_num + 1)) },
	//1ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正出力値1～10)(40byte)
	{ CH1, (short *)&SVD[CH1].uslnr_out1.WORD.low,	(2*(&SVD[CH1].uslnr_out10.WORD.high - &SVD[CH1].uslnr_out1.WORD.low + 1)) },
	//1ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正入力値1～10)(40byte)
	{ CH1, (short *)&SVD[CH1].uslnr_in1.WORD.low,	(2*(&SVD[CH1].uslnr_in10.WORD.high - &SVD[CH1].uslnr_in1.WORD.low + 1)) },
	//1ch ｾﾝｻｼﾘｱﾙﾅﾝﾊﾞｰ(16byte)
	{ CH1, (short *)&SVD[CH1].s_serial[0],	(2*(&SVD[CH1].s_serial[7] - &SVD[CH1].s_serial[0] + 1)) },

 //2ch ﾕｰｻﾞﾊﾟﾗﾒｰﾀ(12byte)
	{ CH2, (short *)&SVD[CH2].max_flow,	(2*(&SVD[CH2].low_cut - &SVD[CH2].max_flow + 1))},
	//2ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正点数、小数点位置)(2byte)
	{ CH2, (short *)&SVD[CH2].uslnr_num,	(2*(&SVD[CH2].uslnr_num - &SVD[CH2].uslnr_num + 1)) },
	//2ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正出力値1～10)(40byte)
	{ CH2, (short *)&SVD[CH2].uslnr_out1.WORD.low,	(2*(&SVD[CH2].uslnr_out10.WORD.high - &SVD[CH2].uslnr_out1.WORD.low + 1)) },
	//2ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正入力値1～10)(40byte)
	{ CH2, (short *)&SVD[CH2].uslnr_in1.WORD.low,	(2*(&SVD[CH2].uslnr_in10.WORD.high - &SVD[CH2].uslnr_in1.WORD.low + 1)) },
	//2ch ｾﾝｻｼﾘｱﾙﾅﾝﾊﾞｰ(16byte)
	{ CH2, (short *)&SVD[CH2].s_serial[0],	(2*(&SVD[CH2].s_serial[7] - &SVD[CH2].s_serial[0] + 1)) },

 //3ch ﾕｰｻﾞﾊﾟﾗﾒｰﾀ(12byte)
	{ CH3, (short *)&SVD[CH3].max_flow,	(2*(&SVD[CH3].low_cut - &SVD[CH3].max_flow + 1))},
	//3ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正点数、小数点位置)(2byte)
	{ CH3, (short *)&SVD[CH3].uslnr_num,	(2*(&SVD[CH3].uslnr_num - &SVD[CH3].uslnr_num + 1)) },
	//3ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正出力値1～10)(40byte)
	{ CH3, (short *)&SVD[CH3].uslnr_out1.WORD.low,	(2*(&SVD[CH3].uslnr_out10.WORD.high - &SVD[CH3].uslnr_out1.WORD.low + 1)) },
	//3ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正入力値1～10)(40byte)
	{ CH3, (short *)&SVD[CH3].uslnr_in1.WORD.low,	(2*(&SVD[CH3].uslnr_in10.WORD.high - &SVD[CH3].uslnr_in1.WORD.low + 1)) },
	//3ch ｾﾝｻｼﾘｱﾙﾅﾝﾊﾞｰ(16byte)
	{ CH3, (short *)&SVD[CH3].s_serial[0],	(2*(&SVD[CH3].s_serial[7] - &SVD[CH3].s_serial[0] + 1)) },

 //4ch ﾕｰｻﾞﾊﾟﾗﾒｰﾀ(12byte)
	{ CH4, (short *)&SVD[CH4].max_flow,	(2*(&SVD[CH4].low_cut - &SVD[CH4].max_flow + 1))},
	//4ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正点数、小数点位置)(2byte)
	{ CH4, (short *)&SVD[CH4].uslnr_num,	(2*(&SVD[CH4].uslnr_num - &SVD[CH4].uslnr_num + 1)) },
	//4ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正出力値1～10)(40byte)
	{ CH4, (short *)&SVD[CH4].uslnr_out1.WORD.low,	(2*(&SVD[CH4].uslnr_out10.WORD.high - &SVD[CH4].uslnr_out1.WORD.low + 1)) },
	//4ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正入力値1～10)(40byte)
	{ CH4, (short *)&SVD[CH4].uslnr_in1.WORD.low,	(2*(&SVD[CH4].uslnr_in10.WORD.high - &SVD[CH4].uslnr_in1.WORD.low + 1)) },
	//4ch ｾﾝｻｼﾘｱﾙﾅﾝﾊﾞｰ(16byte)
	{ CH4, (short *)&SVD[CH4].s_serial[0],	(2*(&SVD[CH4].s_serial[7] - &SVD[CH4].s_serial[0] + 1)) },

 //5ch ﾕｰｻﾞﾊﾟﾗﾒｰﾀ(12byte)
	{ CH5, (short *)&SVD[CH5].max_flow,	(2*(&SVD[CH5].low_cut - &SVD[CH5].max_flow + 1))},
	//5ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正点数、小数点位置)(2byte)
	{ CH5, (short *)&SVD[CH5].uslnr_num,	(2*(&SVD[CH5].uslnr_num - &SVD[CH5].uslnr_num + 1)) },
	//5ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正出力値1～10)(40byte)
	{ CH5, (short *)&SVD[CH5].uslnr_out1.WORD.low,	(2*(&SVD[CH5].uslnr_out10.WORD.high - &SVD[CH5].uslnr_out1.WORD.low + 1)) },
	//5ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正入力値1～10)(40byte)
	{ CH5, (short *)&SVD[CH5].uslnr_in1.WORD.low,	(2*(&SVD[CH5].uslnr_in10.WORD.high - &SVD[CH5].uslnr_in1.WORD.low + 1)) },
	//5ch ｾﾝｻｼﾘｱﾙﾅﾝﾊﾞｰ(16byte)
	{ CH5, (short *)&SVD[CH5].s_serial[0],	(2*(&SVD[CH5].s_serial[7] - &SVD[CH5].s_serial[0] + 1)) },

 //6ch ﾕｰｻﾞﾊﾟﾗﾒｰﾀ(12byte)
	{ CH6, (short *)&SVD[CH6].max_flow,	(2*(&SVD[CH6].low_cut - &SVD[CH6].max_flow + 1))},
	//6ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正点数、小数点位置)(2byte)
	{ CH6, (short *)&SVD[CH6].uslnr_num,	(2*(&SVD[CH6].uslnr_num - &SVD[CH6].uslnr_num + 1)) },
	//6ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正出力値1～10)(40byte)
	{ CH6, (short *)&SVD[CH6].uslnr_out1.WORD.low,	(2*(&SVD[CH6].uslnr_out10.WORD.high - &SVD[CH6].uslnr_out1.WORD.low + 1)) },
	//6ch ﾕｰｻﾞﾘﾆｱﾗｲｽﾞ (補正入力値1～10)(40byte)
	{ CH6, (short *)&SVD[CH6].uslnr_in1.WORD.low,	(2*(&SVD[CH6].uslnr_in10.WORD.high - &SVD[CH6].uslnr_in1.WORD.low + 1)) },
	//6ch ｾﾝｻｼﾘｱﾙﾅﾝﾊﾞｰ(16byte)
	{ CH6, (short *)&SVD[CH6].s_serial[0],	(2*(&SVD[CH6].s_serial[7] - &SVD[CH6].s_serial[0] + 1)) },

 //END
	{ 0xffff, (short *)0, 0 }
};

/****************************************************/
/* Function : OWSensSig                             */
/* Summary  : 1Wireメモリ制御 SENS_SIG(SENS_TX)	   				*/
/* Argument : short sw	                             */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : PORTG BIT2                            */
/****************************************************/
void OWSensSig(short sw){

	if(sw == B_ON){
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0x00);
	}else{
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0x04);
	}
}

/****************************************************/
/* Function : OWSensSigSW                          */
/* Summary  : 1Wireメモリ制御 SENS_SIG_SW	    			    	*/
/* Argument : short sw	                             */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : PORTG BIT4                            */
/****************************************************/
void OWSensSigSW(short sw){

	if(sw == B_ON){
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, 0x10);
	}else{
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, 0x00);
	}
}

/****************************************************/
/* Function : OWSensPowSW                          */
/* Summary  : 1Wireメモリ制御 SENS_POW_SW	        				*/
/* Argument : short sw	                             */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : PORTG BIT5                            */
/****************************************************/
void OWSensPowSW(short sw){

	if(sw == B_ON){
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0x20);
	}else{
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0x00);
	}
}

/****************************************************/
/* Function : OWSensEnable                        */
/* Summary  : 1Wireメモリ制御 SENS_EN  	    			     	*/
/* Argument : short sw	                             */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : PORTG BIT7                            */
/*          : DG469:NoCnnect DG470:Enable          */
/****************************************************/
void OWSensEnable(short sw){

	if(sw == B_ON){
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_7, 0x00);  //0設定でEnable
	}else{
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_7, 0x80);
	}
}

/****************************************************/
/* Function : OWSensRX                        */
/* Summary  : 1Wireメモリ制御 SENS_RX	    				*/
/* Argument : なし			                               */
/* Return   : PORTG Bit3の値                        */
/* Caution  : なし                                   */
/* notes    : PORTG BIT3                            */
/****************************************************/
short OWSensRX(void){

		short result;

		if(GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_3) == 0x00){
 		result = 0;
		}else{
 		result = 1;
		}
		
		return result;
}

/****************************************************/
/* Function : OWSensSigPDR                       */
/* Summary  : 1Wireメモリ制御 SENS_SIG(ポート方向切替え)*/
/* Argument : short sw	                             */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : PORTG BIT2                            */
/****************************************************/
void OWSensSigPDR(short sw){

	if(sw == B_ON){  //出力ポートをに切替え
		//SENS_SIG(PORTG BIT2) output
 	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
	}else{         //入力ポートをに切替え
		//SENS_SIG(PORTG BIT2) input
 	GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_2);
	}
}

/****************************************************/
/* Function : OWResetDevice                         */
/* Summary  : 1WireデバイスにRESET PULSEを出力し、    				*/
/*            1WireデバイスからのPRESENCE PULSEを確認する  */
/* Argument : なし                                   */
/* Return   : 0:正常　0以外:異常                       */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short OWResetDevice(void)
{
	short result, retry;

	OWSensSigPDR(B_OFF);			//SENS_SIGを入力ポート

	result = B_NG;
	retry = 125;
	do{
		if(--retry == 0){
			return result;
		}
		OWWaitUS(5);
	}while(OWSensRX()!=1);		//1Wireデバイスからのデータ読込み(PRESENCE PULSEがHI状態であることを確認する)

	OWSensSigPDR(B_ON);			//SENS_SIGを出力ポート
	OWSensSig(B_ON);					//1Wire通信信号Hi(RESET PULSEを出力)
	OWWaitUS(5);

	OWSensSig(B_OFF);					//1Wire通信信号Low(RESET PULSEを出力)
	OWWaitUS(TIM_tRSTL);
	OWSensSig(B_ON);					//1Wire通信信号Hi(RESET PULSEを出力)

	OWSensSigPDR(B_OFF);			//SENS_SIGを入力ポート

	retry = 125;
	do{
		if(--retry == 0){
			return result;
		}
		OWWaitUS(5);
	}while(OWSensRX()!=0);		//1Wireデバイスからのデータ読込み(PRESENCE PULSEがLOW状態であることを確認する)
	result = B_OK;

	OWWaitUS(TIM_tRSTH);

	return result;
}

/****************************************************/
/* Function : OWWriteBit                        */
/* Summary  : 1Wireデバイスにデータを書込む    				*/
/* Argument : short bit                             */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void OWWriteBit(short bit)
{

	if(bit){
		// Write '1' bit
		OWSensSigPDR(B_ON);			//SENS_SIGを出力ポート
		OWWaitUS(5);
		OWSensSig(B_ON);					 //1Wire通信信号Hi
		OWWaitUS(5);
		OWSensSig(B_OFF);			//1Wire通信信号Low
		OWWaitUS(TIM_tW1L);

		OWSensSig(B_ON);		//1Wire通信信号Hi
		OWWaitUS(TIM_tSLOT - TIM_tW1L);
	}else{
		// Write '0' bit
		OWSensSigPDR(B_ON);			//SENS_SIGを出力ポート
		OWWaitUS(5);
		OWSensSig(B_ON);					 //1Wire通信信号Hi
		OWWaitUS(5);
		OWSensSig(B_OFF);		//1Wire通信信号Low
		OWWaitUS(TIM_tW0L);

		OWSensSig(B_ON);		//1Wire通信信号Hi
		OWWaitUS(TIM_tSLOT - TIM_tW0L);
 }
}

/****************************************************/
/* Function : OWReadBit                        */
/* Summary  : 1Wireデバイスからデータを読込む    				*/
/* Argument : なし                                  */
/* Return   : 1Wireデバイスからのデータ                   */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short OWReadBit(void)
{
	short result;

	OWSensSigPDR(B_ON);			//SENS_SIGを出力ポート
	OWWaitUS(5);
	OWSensSig(B_ON);					 //1Wire通信信号Hi
	OWWaitUS(5);
	OWSensSig(B_OFF);					//1Wire通信信号Low
	OWWaitUS(TIM_tRL);

	OWSensSigPDR(B_OFF);			//SENS_SIGを入力ポート
	OWWaitUS(TIM_tMSR);

	result = OWSensRX() & 0x01;			//1Wireデバイスからのデータ読込み
	OWWaitUS(TIM_tSLOT - TIM_tMSR);

 return result;
}

/****************************************************/
/* Function : OWWriteByte                           */
/* Summary  : 1Wireデバイスにデータを書込む    				*/
/* Argument : short data                            */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void OWWriteByte(short data)
{
 short loop, write_data;

 write_data = data;
 for(loop = 0; loop < 8; loop++){		//8bit分書込む
		OWWriteBit(write_data & 0x01);			//1Wireデバイスにデータを書込む

		write_data >>= 1;
 }
}

/****************************************************/
/* Function : OWReadByte                          */
/* Summary  : 1Wireデバイスからデータを読込む    				*/
/* Argument : なし                                  */
/* Return   : 1Wireデバイスからのデータ                   */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short OWReadByte(void)
{
	short loop, result;

 result = 0;
 for(loop = 0; loop < 8; loop++){		//8bit分読込む
		result >>= 1;

		if(OWReadBit() != 0){				//1Wireデバイスからデータを読込む
				result |= 0x80;
		}
	}

 return result;
}

/****************************************************/
/* Function : GetReflect                        */
/* Summary  : bit0が最上位bit、bit7が最下位bitに変換する		*/
/* Argument : unsigned char data                    */
/* Return   : bit0が最上位bit、bit7が最下位bit         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
unsigned char	GetReflect(unsigned char data)
{
	short loop;
	unsigned char reflect = 0;

 for(loop = 0; loop < 8; loop++){
  reflect <<= 1;

  if((data & 0x01) != 0){
			reflect |= 0x01;
		}
		data >>= 1;
	}

 return reflect;
}

/****************************************************/
/* Function : GetCRC8                        */
/* Summary  : CRC8(MAXIM)の計算(生成多項式:0x31,左送り) */
/* Argument : buff : データバッファ                  */
/*          : size : データサイズ                   */
/* Return   : crc8 : crc8値                     */
/* Caution  : なし                               */
/* note     : 仕様    　　　　　　　                  */
/*          : 初期値 : 0x0000                    */
/*          : シフト方向 : 左                      */
/*          : 出力反転 : 非反転                   */
/*          : Input reflected : 有り    */
/*          : Result reflected : 有り   */
/****************************************************/
unsigned char GetCRC8( const void *buff, short size )
{
	unsigned char *p = (unsigned char *)buff;
	unsigned char crc8;
	short i = 0;
    
	//初期値0
	for ( crc8 = 0x00 ; size != 0 ; size-- ){
		crc8 ^= GetReflect(*p++);		//入力値bitを逆の順序にする(Input reflected)
        
		for ( i = 0 ; i < CHAR_BIT ; i++ ){
			//左シフト
			if ( crc8 & 0x80 ){
				crc8 <<= 1; 
				crc8 ^= MSB_CRC8;
			}
			else{
				crc8 <<= 1;
			}
		}
	}
	//出力非反転
	return GetReflect(crc8);	//戻り値bitを逆の順序にする(Result reflected)
}

/****************************************************/
/* Function : OWCheckDevice                        */
/* Summary  : メモリデバイスの接続判別 */
/* Argument : なし                  			             */
/* Return   : 接続:0 未接続:-1             */
/* Caution  : なし                                  */
/* note     : なし	                                 */
/****************************************************/
short	OWCheckDevice(void)
{
	short retry, result;

	result = B_NG;

	for(retry = 0; retry < 10; retry++){	//リトライ処理(リトライ10回)
		if(OWResetDevice()==B_OK){ //RESETパルスを出力し、PRESENCEパルス(返信)を確認する
			result = B_OK;
			break;
		}else{
		}
	}

	return result;
}

/****************************************************/
/* Function : OWCheckROMID                        */
/* Summary  : メモリデバイスのROM IDを取得し、 */
/* 　　　　　　   　FAMILY CODEを確認する 					*/
/* Argument : なし                            			*/
/* Return   : 正常:0 異常:0以外                   */
/* Caution  : なし                               */
/* note     : FAMILY CODEが0x2Dならば正常             */
/****************************************************/
short	OWCheckROMID(void)
{
	short i, retry, result;
	char rom_data[8];

	if(OWCheckDevice() != B_OK){	//メモリデバイスの接続判別
	return B_NG;
	}

 for(retry = 0; retry < 3; retry++){		//リトライ処理(リトライ3回)
		result = B_OK;
		memset(&rom_data, 0, sizeof(rom_data)); 
		if(OWResetDevice()!=B_OK){ //RESETパルスを出力し、PRESENCEパルス(返信)を確認する
			result = B_NG;
			continue;		//リトライ処理
		}

		OWWriteByte(READ_ROM); //"Read ROM(0x33)"コマンド発行
		for(i = 0; i < 8; i++){
			rom_data[i] = OWReadByte();		//FAMILY CODE(1byte), SERIAL NUMBER(6byte), CRC CODE(1byte)を読込む
		}
		if(rom_data[0] != FAMILY_CODE){		//FAMILY CODEを確認する
			result = B_NG;
			continue;		//リトライ処理
		}

		//ROM ID正常取得
		break;			//リトライ処理を抜ける
	}

	return result;
}

/****************************************************/
/* Function : OWCheckConnect                       */
/* Summary  : メモリデバイスの接続/未接続確認 */
/* Argument : short ch                  			*/
/* Return   : なし													        */
/* Caution  : なし                               */
/* note     : 未接続状態から接続状態となった場合にセンサ情報を読込む*/
/****************************************************/
void OWCheckConnect(short ch)
{

//	if(CNT_1SEC < GetTimeSpan32(time_sens_ctrl, gSysTimeOrg)){	//周期確認
//		time_sens_ctrl = gSysTimeOrg;
//	}else{
//		return;
//	}

	if((SVD[ch].sensor_size == SNS_NONE) ||   //センサ無し設定
   ((MES[ch].err_status & ERR_JUDGE_EMPTY) == 0)){	//受波波形有り時
		phase_owconnect[ch] = 1;
		return;
	}

//	portmesfwdW(0, ch);		//CH切替え有効
	portmesrevW(0, ch);		//CH切替え有効
	OWSensPowSW(B_ON);		//ﾒﾓﾘ保護回路有効
	OWSensSigSW(B_ON);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ有効
	OWWaitUS(5);					//5usec待機

	switch(phase_owconnect[ch]){
	
	//*******
	//どのCHが未接続となったかを判別する処理を作る
	//計測不能の場合に接続確認する
	//*******
	
		case 1:	//ﾒﾓﾘﾃﾞﾊﾞｲｽ未接続確認
			if(OWCheckDevice() == B_NG){		//ﾒﾓﾘﾃﾞﾊﾞｲｽ未接続状態
				phase_owconnect[ch]++;	//ﾌｪｰｽﾞ更新
			}
 		break;
		case 2:	//ﾒﾓﾘﾃﾞﾊﾞｲｽ接続確認
			if(OWCheckDevice() == B_OK){		//ﾒﾓﾘﾃﾞﾊﾞｲｽ接続状態
				phase_owconnect[ch]++;	//ﾌｪｰｽﾞ更新
			}
 		break;
		case 3:	//ｾﾝｻ情報を読込む
			OWReadSensinfo(ch);	//ﾒﾓﾘﾃﾞﾊﾞｲｽからｾﾝｻ情報を読込む
			phase_owconnect[ch]++;	//ﾌｪｰｽﾞ更新
			break;
		case 4:	//読込んだセンサ情報をEEPROM書込む
			util_eep_allwrite(ch, WR_DEVICE);  //EEPROM書込み
			phase_owconnect[ch]++;	//ﾌｪｰｽﾞ更新
			break;
		case 5:	//完了
		default:
			phase_owconnect[ch] = 1;	//ﾌｪｰｽﾞ初期化
			break;
	}

//	portmesfwdW(1, ch);		//CH切替え無効
	portmesrevW(1, ch);		//CH切替え無効
	OWSensSigSW(B_OFF);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ無効
	OWSensPowSW(B_OFF);		//ﾒﾓﾘ保護回路無効
}

/****************************************************/
/* Function : OWWriteData                       */
/* Summary  : メモリデバイスにデータを書込む */
/* Argument : アドレス、書込みデータ      			*/
/* Return   : 正常:0 異常:0以外                   */
/* Caution  : なし                               */
/* note     : なし                                   */
/****************************************************/
short	OWWriteData(short address, const void *buff)
{
	short i, ow_address, result;
	unsigned char *p = (unsigned char *)buff;

	ow_address = address;
	result = B_NG;

	if(OWResetDevice()!=B_OK){  //RESETパルスを出力し、PRESENCEパルス(返信)を確認する
		OW_write[0]++;
		return result;
	}

	OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"コマンド発行
	OWWriteByte(WRITE_SCRATCHPAD); //"WRITE SCRATCHPAD(0x0F)"コマンド発行
	OWWriteByte((char)(ow_address & 0x00FF)); 							// TA1 (書込み先アドレス)
	OWWriteByte((char)((ow_address & 0xFF00) >> 8)); // TA2

	for(i = 0; i < 8; i++){
		OWWriteByte(*p++); 		//データを書込む
	}

	// read CRC data
	for(i = 0; i < 2; i++){
		memory_W_data[i] = OWReadByte();
	}

	if(OWResetDevice()!=B_OK){  //RESETパルスを出力し、PRESENCEパルス(返信)を確認する
		OW_write[1]++;
		return result;
	}

	OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"コマンド発行
	OWWriteByte(COPY_SCRATCHPAD); //"COPY SCRATCHPAD(0x55)"コマンド発行
	OWWriteByte((char)(ow_address & 0x00FF)); 							// TA1 (書込み先アドレス)
	OWWriteByte((char)((ow_address & 0xFF00) >> 8)); // TA2
	OWWriteByte(0x07);
	result = B_OK;

	OWWaitMS(TIM_tPROGMAX);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ書込み待機
	OW_write[2]++;

	return result;
}

/****************************************************/
/* Function : OWCompareData                       */
/* Summary  : メモリデバイスに書込んだデータの照合処理 */
/* Argument : アドレス、書込みデータ      			*/
/* Return   : 正常:0 異常:0以外                   */
/* Caution  : なし                               */
/* note     : なし                                   */
/****************************************************/
short	OWCompareData(short address, const void *buff)
{
	short i, ow_address, result;

	ow_address = address;
	result = B_NG;

	if(OWResetDevice()!=B_OK){  //RESETパルスを出力し、PRESENCEパルス(返信)を確認する
		OW_comp[0]++;
		return result;
	}

	OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"コマンド発行
	OWWriteByte(READ_MEMORY); //"READ MEMORY(0xF0)"コマンド発行
	OWWriteByte((char)(ow_address & 0x00FF)); 	// TA1 (読込み開始アドレス)
	OWWriteByte((char)((ow_address & 0xFF00) >> 8)); // TA2

	// ﾒﾓﾘﾃﾞﾊﾞｲｽのﾃﾞｰﾀを読込む
	for(i = 0; i < 8; i++){
		memory_C_data[i] = OWReadByte();
	}

	//照合処理
	if(memcmp(&memory_C_data[0], buff, 8) == 0){		//ﾒﾓﾘﾃﾞﾊﾞｲｽに書込んだﾃﾞｰﾀとﾒﾓﾘﾃﾞﾊﾞｲｽ読込んだﾃﾞｰﾀが一致
		result = B_OK;
		OW_comp[2]++;
	}else{
		OW_comp[1]++;
	}

	return result;
}

/****************************************************/
/* Function : OWReadSensinfo                       */
/* Summary  : メモリデバイスからセンサ情報を読込む            */
/* Argument : short ch   読込むCH            	      */
/* Return   : 正常:0 異常:0以外                      */
/* Caution  : なし                                   */
/* note     : センサ情報(ﾕｰｻﾞﾊﾟﾗﾒｰﾀ)を読込む             */
/****************************************************/
short	OWReadSensinfo(short ch)
{
// char memory_data[MEMORY_MAX];
	short retry, result, size, cnt, pos;

	clrpsw_i();		/* 割り込み禁止 */
	result = B_NG;

//	portmesfwdW(0, ch);		//CH切替え有効
	portmesrevW(0, ch);		//CH切替え有効
	OWSensPowSW(B_ON);		//ﾒﾓﾘ保護回路有効
	OWSensSigSW(B_ON);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ有効
	OWWaitUS(5);					//5usec待機

	if(OWCheckROMID() != B_OK){		//ROM_IDの確認
		OW_read[0]++;
		MES[ch].err_status |= ERR_JUDGE_EEPROM;		/*EEPROMエラーセット*/
//		portmesfwdW(1, ch);		//CH切替え無効
		portmesrevW(1, ch);		//CH切替え無効
		OWSensSigSW(B_OFF);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ無効
		OWSensPowSW(B_OFF);	 //ﾒﾓﾘ保護回路無効
		setpsw_i();		/* 割り込み許可 */	
		return B_NG;
	}

	for(retry = 0; retry < 3; retry++){	//リトライ処理(リトライ3回)
		if(OWResetDevice() != B_OK){  //RESETパルスを出力し、PRESENCEパルス(返信)を確認する
			result = B_NG;
			OW_read[1]++;
			continue;		//リトライ処理
		}

		OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"コマンド発行
		OWWriteByte(READ_MEMORY); //"READ MEMORY(0xF0)"コマンド発行
	
		OWWriteByte(0x00); // TA1 (読込み開始アドレス:0x0000)
		OWWriteByte(0x00); // TA2
		
		//ﾒﾓﾘﾃﾞﾊﾞｲｽのﾒﾓﾘﾃﾞｰﾀを読込む
		for(cnt = 0; cnt < MEMORY_MAX; cnt++){
			memory_R_data[cnt] = OWReadByte();
		}

		//先頭にFAMILY CODEが保持されていることを確認する。
		if(memory_R_data[0] == FAMILY_CODE){ 	//「FAMILY CODE」が保持されている場合
			;
		}else{		//「FAMILY CODE」が保持されていない場合
			result = B_BLANK;		//EEPROM読み込みｴﾗｰにしない(何も書かれていない新品状態のﾒﾓﾘﾃﾞﾊﾞｲｽを読込んだ時は、Modbusにコピーしない為の処理。EEPROMに書き込まない。)
			OW_read[2]++;
			continue;		//リトライ処理
		}

		//CRCチェック
		size = 0;
		for(cnt=0; OWEepromData[cnt].ch!=0xffff; cnt++){		//センサ情報のサイズを取得
		 if(OWEepromData[cnt].ch == ch){  //対象CHのセンサ情報のサイズを計算する
 			size += OWEepromData[cnt].size;
			}
		}
		size += 1;		//先頭のFAMILY CODE分のサイズを加算
		if(memory_R_data[size] == GetCRC8(&memory_R_data[0], size)){		//保存したCRCと計算したCRCが一致
			result = B_OK;
			OW_read[4]+=10;

			pos = 1;  //先頭にFAMILY CODEが保持されているので1から開始する
			for(cnt=0; OWEepromData[cnt].ch!=0xffff; cnt++){
 		 if(OWEepromData[cnt].ch == ch){  //対象CH
	  		memcpy(OWEepromData[cnt].p, &memory_R_data[pos], OWEepromData[cnt].size);	//ﾒﾓﾘﾃﾞﾊﾞｲｽのﾃﾞｰﾀをModbusにコピー
		 		pos += OWEepromData[cnt].size;
				}
			}
			break;		//リトライ処理を抜ける		
		}else{		//CRC不一致
			result = B_NG;
			OW_read[3]++;
			continue;		//リトライ処理
		}
	}

	if(result == B_NG){		//センサ情報読込み失敗した場合
		MES[ch].err_status |= ERR_JUDGE_EEPROM;		/*EEPROMエラーセット*/
	}else{
		MES[ch].err_status &= ~ERR_JUDGE_EEPROM;		/*EEPROMエラーリセット*/
	}

//	portmesfwdW(1, ch);	 //CH切替え無効
	portmesrevW(1, ch);	 //CH切替え無効
	OWSensSigSW(B_OFF);	 //ﾒﾓﾘﾃﾞﾊﾞｲｽ無効
	OWSensPowSW(B_OFF);	 //ﾒﾓﾘ保護回路無効
	setpsw_i();		/* 割り込み許可 */	

	return result;
}

/****************************************************/
/* Function : OWWriteSensinfo                      */
/* Summary  : メモリデバイスにセンサ情報を書込む             */
/* Argument : なし                               		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : センサ情報(ﾕｰｻﾞﾊﾟﾗﾒｰﾀ)を書込む            */
/****************************************************/
void	OWWriteSensinfo(void)
{
	short cnt, num;

	switch(phase_owwrite){
		/** メモリデバイス書込み要求確認 **/
		case 1:
 		if(OWwrite[OWwriteCH] == 1){  //メモリデバイス書込み要求あり
 			phase_owwrite++;	//ﾌｪｰｽﾞ更新
			}else{
				OWwriteCH++;  //書込みCH更新
				if(OWwriteCH >= CH_NUMMAX){
				 OWwriteCH = CH1;
				}
			}
 		break;

		/** メモリデバイス書込み待機 **/
		case 2:
 		if((action_status_check(OWwriteCH) == ACT_STS_NORMAL) ||  //通常状態
 		   (action_status_check(OWwriteCH) == ACT_STS_ADDIT)){    //積算実行中
				clrpsw_i();		/* 割り込み禁止 */
//				portmesfwdW(0, OWwriteCH);		//CH切替え
				portmesrevW(0, OWwriteCH);		//CH切替え
				OWSensPowSW(B_ON);		//ﾒﾓﾘ保護回路有効
				OWSensSigSW(B_ON);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ有効
				phase_owwrite++;	//ﾌｪｰｽﾞ更新
			}
			break;

		/** 書込みデータをsens_infoに作成する **/
		case 3:
			clrpsw_i();		/* 割り込み禁止 */
			sens_info[0] = FAMILY_CODE;		//先頭に「FAMILY CODE」を書込む(新品状態のﾒﾓﾘﾃﾞﾊﾞｲｽを判別するため)
			ow_pos = 1;  //先頭にFAMILY CODEが保持されているので1から開始する
			for(cnt=0; OWEepromData[cnt].ch!=0xffff; cnt++){
			 if(OWEepromData[cnt].ch == OWwriteCH){  //対象CH
 				memcpy(&sens_info[ow_pos], OWEepromData[cnt].p, OWEepromData[cnt].size);	//EEPROM書込みデータをsens_infにコピー
	 			ow_pos += OWEepromData[cnt].size;
				}
			}
			phase_owwrite++;	//ﾌｪｰｽﾞ更新
			break;

		/** CRC8の計算 **/
		case 4:
			clrpsw_i();		/* 割り込み禁止 */
			sens_info[ow_pos] = GetCRC8(&sens_info, ow_pos);
			ow_cnt = ow_retry = 0;
			phase_owwrite++;	//ﾌｪｰｽﾞ更新
			break;
		/** メモリデバイスに書込む **/
		case 5:
			clrpsw_i();		/* 割り込み禁止 */
			if(OWWriteData(0x0008*ow_cnt, &sens_info[0x0008*ow_cnt]) == B_OK){	//書込み正常
				phase_owwrite++;	//ﾌｪｰｽﾞ更新
			}else{	//書込み異常
				ow_retry++;
				if(ow_retry > 3){		//ﾘﾄﾗｲ3回
					phase_owwrite = 7;	//ﾌｪｰｽﾞ更新(処理終了)
				}
			}
			break;

		/** メモリデバイスに書込んだデータの照合処理 **/
		case 6:
			clrpsw_i();		/* 割り込み禁止 */
			if(OWCompareData(0x0008*ow_cnt, &sens_info[0x0008*ow_cnt]) == B_OK){	//照合正常した場合
				ow_cnt++;
				num = ow_pos / 8 + 1;
				if(ow_cnt > num){
					phase_owwrite++;	//ﾌｪｰｽﾞ更新(全ｱﾄﾞﾚｽ書込み&照合終了)
				}else{
					phase_owwrite = 5;	//ﾌｪｰｽﾞ更新(次ｱﾄﾞﾚｽ書込み)
				}
			}else{		//照合異常した場合
				ow_retry++;
				if(ow_retry > 3){		//ﾘﾄﾗｲ3回
					phase_owwrite = 7;	//ﾌｪｰｽﾞ更新(処理終了)
				}else{
					phase_owwrite = 5;	//ﾌｪｰｽﾞ更新(再書込み)
				}
			}
			break;

		/** 処理終了 **/
		case 7:
		default:
//			portmesfwdW(1, OWwriteCH);	//CH切替え無効
			portmesrevW(1, OWwriteCH);	//CH切替え無効
			OWSensSigSW(B_OFF);		//ﾒﾓﾘﾃﾞﾊﾞｲｽ無効
			OWSensPowSW(B_OFF);		//ﾒﾓﾘ保護回路無効
			OWwrite[OWwriteCH] = 0;		//メモリデバイス書込み要求ｸﾘｱ
			OWwriteCH++;  //書込みCH更新
			if(OWwriteCH >= CH_NUMMAX){
			 OWwriteCH = CH1;
   }
			phase_owwrite = 1;	//ﾌｪｰｽﾞ初期化
			setpsw_i();		/* 割り込み許可 */			
			break;
		}
}

/****************************************************/
/* Function : OWWaitUS                   */
/* Summary  : 約1us単位待機 */
/* Argument : 待ち時間                              */
/* Return   : なし                                  */
/* Caution  : なし                                   */
/* notes    : 引数で指定した値×1us分待機する         */
/****************************************************/
void OWWaitUS(long us){

	volatile long i, j;

	for(i = 0; i < us; i++){
		for(j = 0; j < 6; j++){
			nop();
		}
	}
}

/****************************************************/
/* Function : OWWaitMS                  */
/* Summary  : 約1ms単位待機 */
/* Argument : 待ち時間                              */
/* Return   : なし                                  */
/* Caution  : なし                                   */
/* notes    : 引数で指定した値×1ms分待機する         */
/****************************************************/
void OWWaitMS(long ms){

	volatile long i, j;
	
	for(i = 0; i < ms; i++){
		for(j = 0; j < 200; j++){
			OWWaitUS(5);
		}
	}
}
