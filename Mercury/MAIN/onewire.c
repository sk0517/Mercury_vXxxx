/***********************************************/
/* File Name : onewire.c		         									   */
/*	Summary   : 1Wire�������ǂݏ�������              */
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
/*	�萔��`(#define)								*/
/********************************************************/
#define DS28E07_FAMILYCODE 0x2D //DS28E07��FAMILY CODE
#define DS2431_FAMILYCODE 0x2D //DS2431��FAMILY CODE
#define DS18B20_FAMILYCODE 0x28 //DS18B20��FAMILY CODE
#define DS28E80_FAMILYCODE 0x4A //DS28E80��FAMILY CODE

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

#define	WRITE_SCRATCHPAD	0x0F		//�����݃R�}���h
#define	READ_SCRATCHPAD		0xAA		//�Ǎ��݃R�}���h
#define	COPY_SCRATCHPAD		0x55		//EEPROM�R�s�[�R�}���h
#define	READ_MEMORY						0xF0		//�������Ǎ��݃R�}���h

#define	WRITE_BLOCK	0x55		//�����݃R�}���h
#define	WRITE_PRO_BLOCK		0xC3		//Write Protect Block�R�}���h
#define	READ_PRO_BLOCK 		0xAA		//Read Block Protection�R�}���h

#define	READ_ROM						0x33		//READ_ROM�R�}���h
#define	MATCH_ROM					0x55		//MATCH_ROM�R�}���h
#define	SEARCH_ROM				0xF0		//SERACH_ROM�R�}���h
#define	SKIP_ROM						0xCC		//SKIP_ROM�R�}���h

#define MEMORY_MAX			144		//�������ő�o�C�g��

#define MSB_CRC8 (0x31)  // x8 + x5 + x4 + x0 �W�� (�擪1bit�͍폜)
#define CHAR_BIT 8

/********************************************************/
/*	���W���[������`�֐�								*/
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
/*	���W���[���O��`�֐�								*/
/********************************************************/
//extern void portmesfwdW(unsigned char data, short pch);
extern void portmesrevW(unsigned char data, short pch);
extern void	util_eep_allwrite(short pch, short opt);
extern short	action_status_check(short pch);

/********************************************************/
/*	���W���[������`�ϐ�								*/
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
/*	���W���[���O��`�ϐ�								*/
/********************************************************/


//�������f�o�C�X�ɕۑ�����f�[�^
struct OWEeprom
{
	unsigned short	ch;		//CH�ԍ�
	short *p;		//�ۑ��擪�ʒu
	unsigned short	size;	//�ۑ�����
};

//�������f�o�C�X�ɕۑ�����f�[�^
// ���eCH�̃������f�o�C�X�̕ۑ��f�[�^�ʂ�1kbit�ȓ��ł��邱��
const struct OWEeprom OWEepromData[] = {
 //1ch հ�����Ұ�(12byte)
	{ CH1, (short *)&SVD[CH1].max_flow,	(2*(&SVD[CH1].low_cut - &SVD[CH1].max_flow + 1))},
	//1ch հ���Ʊײ�� (�␳�_���A�����_�ʒu)(2byte)
	{ CH1, (short *)&SVD[CH1].uslnr_num,	(2*(&SVD[CH1].uslnr_num - &SVD[CH1].uslnr_num + 1)) },
	//1ch հ���Ʊײ�� (�␳�o�͒l1�`10)(40byte)
	{ CH1, (short *)&SVD[CH1].uslnr_out1.WORD.low,	(2*(&SVD[CH1].uslnr_out10.WORD.high - &SVD[CH1].uslnr_out1.WORD.low + 1)) },
	//1ch հ���Ʊײ�� (�␳���͒l1�`10)(40byte)
	{ CH1, (short *)&SVD[CH1].uslnr_in1.WORD.low,	(2*(&SVD[CH1].uslnr_in10.WORD.high - &SVD[CH1].uslnr_in1.WORD.low + 1)) },
	//1ch �ݻ�ر����ް(16byte)
	{ CH1, (short *)&SVD[CH1].s_serial[0],	(2*(&SVD[CH1].s_serial[7] - &SVD[CH1].s_serial[0] + 1)) },

 //2ch հ�����Ұ�(12byte)
	{ CH2, (short *)&SVD[CH2].max_flow,	(2*(&SVD[CH2].low_cut - &SVD[CH2].max_flow + 1))},
	//2ch հ���Ʊײ�� (�␳�_���A�����_�ʒu)(2byte)
	{ CH2, (short *)&SVD[CH2].uslnr_num,	(2*(&SVD[CH2].uslnr_num - &SVD[CH2].uslnr_num + 1)) },
	//2ch հ���Ʊײ�� (�␳�o�͒l1�`10)(40byte)
	{ CH2, (short *)&SVD[CH2].uslnr_out1.WORD.low,	(2*(&SVD[CH2].uslnr_out10.WORD.high - &SVD[CH2].uslnr_out1.WORD.low + 1)) },
	//2ch հ���Ʊײ�� (�␳���͒l1�`10)(40byte)
	{ CH2, (short *)&SVD[CH2].uslnr_in1.WORD.low,	(2*(&SVD[CH2].uslnr_in10.WORD.high - &SVD[CH2].uslnr_in1.WORD.low + 1)) },
	//2ch �ݻ�ر����ް(16byte)
	{ CH2, (short *)&SVD[CH2].s_serial[0],	(2*(&SVD[CH2].s_serial[7] - &SVD[CH2].s_serial[0] + 1)) },

 //3ch հ�����Ұ�(12byte)
	{ CH3, (short *)&SVD[CH3].max_flow,	(2*(&SVD[CH3].low_cut - &SVD[CH3].max_flow + 1))},
	//3ch հ���Ʊײ�� (�␳�_���A�����_�ʒu)(2byte)
	{ CH3, (short *)&SVD[CH3].uslnr_num,	(2*(&SVD[CH3].uslnr_num - &SVD[CH3].uslnr_num + 1)) },
	//3ch հ���Ʊײ�� (�␳�o�͒l1�`10)(40byte)
	{ CH3, (short *)&SVD[CH3].uslnr_out1.WORD.low,	(2*(&SVD[CH3].uslnr_out10.WORD.high - &SVD[CH3].uslnr_out1.WORD.low + 1)) },
	//3ch հ���Ʊײ�� (�␳���͒l1�`10)(40byte)
	{ CH3, (short *)&SVD[CH3].uslnr_in1.WORD.low,	(2*(&SVD[CH3].uslnr_in10.WORD.high - &SVD[CH3].uslnr_in1.WORD.low + 1)) },
	//3ch �ݻ�ر����ް(16byte)
	{ CH3, (short *)&SVD[CH3].s_serial[0],	(2*(&SVD[CH3].s_serial[7] - &SVD[CH3].s_serial[0] + 1)) },

 //4ch հ�����Ұ�(12byte)
	{ CH4, (short *)&SVD[CH4].max_flow,	(2*(&SVD[CH4].low_cut - &SVD[CH4].max_flow + 1))},
	//4ch հ���Ʊײ�� (�␳�_���A�����_�ʒu)(2byte)
	{ CH4, (short *)&SVD[CH4].uslnr_num,	(2*(&SVD[CH4].uslnr_num - &SVD[CH4].uslnr_num + 1)) },
	//4ch հ���Ʊײ�� (�␳�o�͒l1�`10)(40byte)
	{ CH4, (short *)&SVD[CH4].uslnr_out1.WORD.low,	(2*(&SVD[CH4].uslnr_out10.WORD.high - &SVD[CH4].uslnr_out1.WORD.low + 1)) },
	//4ch հ���Ʊײ�� (�␳���͒l1�`10)(40byte)
	{ CH4, (short *)&SVD[CH4].uslnr_in1.WORD.low,	(2*(&SVD[CH4].uslnr_in10.WORD.high - &SVD[CH4].uslnr_in1.WORD.low + 1)) },
	//4ch �ݻ�ر����ް(16byte)
	{ CH4, (short *)&SVD[CH4].s_serial[0],	(2*(&SVD[CH4].s_serial[7] - &SVD[CH4].s_serial[0] + 1)) },

 //5ch հ�����Ұ�(12byte)
	{ CH5, (short *)&SVD[CH5].max_flow,	(2*(&SVD[CH5].low_cut - &SVD[CH5].max_flow + 1))},
	//5ch հ���Ʊײ�� (�␳�_���A�����_�ʒu)(2byte)
	{ CH5, (short *)&SVD[CH5].uslnr_num,	(2*(&SVD[CH5].uslnr_num - &SVD[CH5].uslnr_num + 1)) },
	//5ch հ���Ʊײ�� (�␳�o�͒l1�`10)(40byte)
	{ CH5, (short *)&SVD[CH5].uslnr_out1.WORD.low,	(2*(&SVD[CH5].uslnr_out10.WORD.high - &SVD[CH5].uslnr_out1.WORD.low + 1)) },
	//5ch հ���Ʊײ�� (�␳���͒l1�`10)(40byte)
	{ CH5, (short *)&SVD[CH5].uslnr_in1.WORD.low,	(2*(&SVD[CH5].uslnr_in10.WORD.high - &SVD[CH5].uslnr_in1.WORD.low + 1)) },
	//5ch �ݻ�ر����ް(16byte)
	{ CH5, (short *)&SVD[CH5].s_serial[0],	(2*(&SVD[CH5].s_serial[7] - &SVD[CH5].s_serial[0] + 1)) },

 //6ch հ�����Ұ�(12byte)
	{ CH6, (short *)&SVD[CH6].max_flow,	(2*(&SVD[CH6].low_cut - &SVD[CH6].max_flow + 1))},
	//6ch հ���Ʊײ�� (�␳�_���A�����_�ʒu)(2byte)
	{ CH6, (short *)&SVD[CH6].uslnr_num,	(2*(&SVD[CH6].uslnr_num - &SVD[CH6].uslnr_num + 1)) },
	//6ch հ���Ʊײ�� (�␳�o�͒l1�`10)(40byte)
	{ CH6, (short *)&SVD[CH6].uslnr_out1.WORD.low,	(2*(&SVD[CH6].uslnr_out10.WORD.high - &SVD[CH6].uslnr_out1.WORD.low + 1)) },
	//6ch հ���Ʊײ�� (�␳���͒l1�`10)(40byte)
	{ CH6, (short *)&SVD[CH6].uslnr_in1.WORD.low,	(2*(&SVD[CH6].uslnr_in10.WORD.high - &SVD[CH6].uslnr_in1.WORD.low + 1)) },
	//6ch �ݻ�ر����ް(16byte)
	{ CH6, (short *)&SVD[CH6].s_serial[0],	(2*(&SVD[CH6].s_serial[7] - &SVD[CH6].s_serial[0] + 1)) },

 //END
	{ 0xffff, (short *)0, 0 }
};

/****************************************************/
/* Function : OWSensSig                             */
/* Summary  : 1Wire���������� SENS_SIG(SENS_TX)	   				*/
/* Argument : short sw	                             */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
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
/* Summary  : 1Wire���������� SENS_SIG_SW	    			    	*/
/* Argument : short sw	                             */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
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
/* Summary  : 1Wire���������� SENS_POW_SW	        				*/
/* Argument : short sw	                             */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
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
/* Summary  : 1Wire���������� SENS_EN  	    			     	*/
/* Argument : short sw	                             */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : PORTG BIT7                            */
/*          : DG469:NoCnnect DG470:Enable          */
/****************************************************/
void OWSensEnable(short sw){

	if(sw == B_ON){
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_7, 0x00);  //0�ݒ��Enable
	}else{
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_7, 0x80);
	}
}

/****************************************************/
/* Function : OWSensRX                        */
/* Summary  : 1Wire���������� SENS_RX	    				*/
/* Argument : �Ȃ�			                               */
/* Return   : PORTG Bit3�̒l                        */
/* Caution  : �Ȃ�                                   */
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
/* Summary  : 1Wire���������� SENS_SIG(�|�[�g�����ؑւ�)*/
/* Argument : short sw	                             */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : PORTG BIT2                            */
/****************************************************/
void OWSensSigPDR(short sw){

	if(sw == B_ON){  //�o�̓|�[�g���ɐؑւ�
		//SENS_SIG(PORTG BIT2) output
 	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
	}else{         //���̓|�[�g���ɐؑւ�
		//SENS_SIG(PORTG BIT2) input
 	GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_2);
	}
}

/****************************************************/
/* Function : OWResetDevice                         */
/* Summary  : 1Wire�f�o�C�X��RESET PULSE���o�͂��A    				*/
/*            1Wire�f�o�C�X�����PRESENCE PULSE���m�F����  */
/* Argument : �Ȃ�                                   */
/* Return   : 0:����@0�ȊO:�ُ�                       */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short OWResetDevice(void)
{
	short result, retry;

	OWSensSigPDR(B_OFF);			//SENS_SIG����̓|�[�g

	result = B_NG;
	retry = 125;
	do{
		if(--retry == 0){
			return result;
		}
		OWWaitUS(5);
	}while(OWSensRX()!=1);		//1Wire�f�o�C�X����̃f�[�^�Ǎ���(PRESENCE PULSE��HI��Ԃł��邱�Ƃ��m�F����)

	OWSensSigPDR(B_ON);			//SENS_SIG���o�̓|�[�g
	OWSensSig(B_ON);					//1Wire�ʐM�M��Hi(RESET PULSE���o��)
	OWWaitUS(5);

	OWSensSig(B_OFF);					//1Wire�ʐM�M��Low(RESET PULSE���o��)
	OWWaitUS(TIM_tRSTL);
	OWSensSig(B_ON);					//1Wire�ʐM�M��Hi(RESET PULSE���o��)

	OWSensSigPDR(B_OFF);			//SENS_SIG����̓|�[�g

	retry = 125;
	do{
		if(--retry == 0){
			return result;
		}
		OWWaitUS(5);
	}while(OWSensRX()!=0);		//1Wire�f�o�C�X����̃f�[�^�Ǎ���(PRESENCE PULSE��LOW��Ԃł��邱�Ƃ��m�F����)
	result = B_OK;

	OWWaitUS(TIM_tRSTH);

	return result;
}

/****************************************************/
/* Function : OWWriteBit                        */
/* Summary  : 1Wire�f�o�C�X�Ƀf�[�^��������    				*/
/* Argument : short bit                             */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void OWWriteBit(short bit)
{

	if(bit){
		// Write '1' bit
		OWSensSigPDR(B_ON);			//SENS_SIG���o�̓|�[�g
		OWWaitUS(5);
		OWSensSig(B_ON);					 //1Wire�ʐM�M��Hi
		OWWaitUS(5);
		OWSensSig(B_OFF);			//1Wire�ʐM�M��Low
		OWWaitUS(TIM_tW1L);

		OWSensSig(B_ON);		//1Wire�ʐM�M��Hi
		OWWaitUS(TIM_tSLOT - TIM_tW1L);
	}else{
		// Write '0' bit
		OWSensSigPDR(B_ON);			//SENS_SIG���o�̓|�[�g
		OWWaitUS(5);
		OWSensSig(B_ON);					 //1Wire�ʐM�M��Hi
		OWWaitUS(5);
		OWSensSig(B_OFF);		//1Wire�ʐM�M��Low
		OWWaitUS(TIM_tW0L);

		OWSensSig(B_ON);		//1Wire�ʐM�M��Hi
		OWWaitUS(TIM_tSLOT - TIM_tW0L);
 }
}

/****************************************************/
/* Function : OWReadBit                        */
/* Summary  : 1Wire�f�o�C�X����f�[�^��Ǎ���    				*/
/* Argument : �Ȃ�                                  */
/* Return   : 1Wire�f�o�C�X����̃f�[�^                   */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short OWReadBit(void)
{
	short result;

	OWSensSigPDR(B_ON);			//SENS_SIG���o�̓|�[�g
	OWWaitUS(5);
	OWSensSig(B_ON);					 //1Wire�ʐM�M��Hi
	OWWaitUS(5);
	OWSensSig(B_OFF);					//1Wire�ʐM�M��Low
	OWWaitUS(TIM_tRL);

	OWSensSigPDR(B_OFF);			//SENS_SIG����̓|�[�g
	OWWaitUS(TIM_tMSR);

	result = OWSensRX() & 0x01;			//1Wire�f�o�C�X����̃f�[�^�Ǎ���
	OWWaitUS(TIM_tSLOT - TIM_tMSR);

 return result;
}

/****************************************************/
/* Function : OWWriteByte                           */
/* Summary  : 1Wire�f�o�C�X�Ƀf�[�^��������    				*/
/* Argument : short data                            */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void OWWriteByte(short data)
{
 short loop, write_data;

 write_data = data;
 for(loop = 0; loop < 8; loop++){		//8bit��������
		OWWriteBit(write_data & 0x01);			//1Wire�f�o�C�X�Ƀf�[�^��������

		write_data >>= 1;
 }
}

/****************************************************/
/* Function : OWReadByte                          */
/* Summary  : 1Wire�f�o�C�X����f�[�^��Ǎ���    				*/
/* Argument : �Ȃ�                                  */
/* Return   : 1Wire�f�o�C�X����̃f�[�^                   */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short OWReadByte(void)
{
	short loop, result;

 result = 0;
 for(loop = 0; loop < 8; loop++){		//8bit���Ǎ���
		result >>= 1;

		if(OWReadBit() != 0){				//1Wire�f�o�C�X����f�[�^��Ǎ���
				result |= 0x80;
		}
	}

 return result;
}

/****************************************************/
/* Function : GetReflect                        */
/* Summary  : bit0���ŏ��bit�Abit7���ŉ���bit�ɕϊ�����		*/
/* Argument : unsigned char data                    */
/* Return   : bit0���ŏ��bit�Abit7���ŉ���bit         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : CRC8(MAXIM)�̌v�Z(����������:0x31,������) */
/* Argument : buff : �f�[�^�o�b�t�@                  */
/*          : size : �f�[�^�T�C�Y                   */
/* Return   : crc8 : crc8�l                     */
/* Caution  : �Ȃ�                               */
/* note     : �d�l    �@�@�@�@�@�@�@                  */
/*          : �����l : 0x0000                    */
/*          : �V�t�g���� : ��                      */
/*          : �o�͔��] : �񔽓]                   */
/*          : Input reflected : �L��    */
/*          : Result reflected : �L��   */
/****************************************************/
unsigned char GetCRC8( const void *buff, short size )
{
	unsigned char *p = (unsigned char *)buff;
	unsigned char crc8;
	short i = 0;
    
	//�����l0
	for ( crc8 = 0x00 ; size != 0 ; size-- ){
		crc8 ^= GetReflect(*p++);		//���͒lbit���t�̏����ɂ���(Input reflected)
        
		for ( i = 0 ; i < CHAR_BIT ; i++ ){
			//���V�t�g
			if ( crc8 & 0x80 ){
				crc8 <<= 1; 
				crc8 ^= MSB_CRC8;
			}
			else{
				crc8 <<= 1;
			}
		}
	}
	//�o�͔񔽓]
	return GetReflect(crc8);	//�߂�lbit���t�̏����ɂ���(Result reflected)
}

/****************************************************/
/* Function : OWCheckDevice                        */
/* Summary  : �������f�o�C�X�̐ڑ����� */
/* Argument : �Ȃ�                  			             */
/* Return   : �ڑ�:0 ���ڑ�:-1             */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�	                                 */
/****************************************************/
short	OWCheckDevice(void)
{
	short retry, result;

	result = B_NG;

	for(retry = 0; retry < 10; retry++){	//���g���C����(���g���C10��)
		if(OWResetDevice()==B_OK){ //RESET�p���X���o�͂��APRESENCE�p���X(�ԐM)���m�F����
			result = B_OK;
			break;
		}else{
		}
	}

	return result;
}

/****************************************************/
/* Function : OWCheckROMID                        */
/* Summary  : �������f�o�C�X��ROM ID���擾���A */
/* �@�@�@�@�@�@   �@FAMILY CODE���m�F���� 					*/
/* Argument : �Ȃ�                            			*/
/* Return   : ����:0 �ُ�:0�ȊO                   */
/* Caution  : �Ȃ�                               */
/* note     : FAMILY CODE��0x2D�Ȃ�ΐ���             */
/****************************************************/
short	OWCheckROMID(void)
{
	short i, retry, result;
	char rom_data[8];

	if(OWCheckDevice() != B_OK){	//�������f�o�C�X�̐ڑ�����
	return B_NG;
	}

 for(retry = 0; retry < 3; retry++){		//���g���C����(���g���C3��)
		result = B_OK;
		memset(&rom_data, 0, sizeof(rom_data)); 
		if(OWResetDevice()!=B_OK){ //RESET�p���X���o�͂��APRESENCE�p���X(�ԐM)���m�F����
			result = B_NG;
			continue;		//���g���C����
		}

		OWWriteByte(READ_ROM); //"Read ROM(0x33)"�R�}���h���s
		for(i = 0; i < 8; i++){
			rom_data[i] = OWReadByte();		//FAMILY CODE(1byte), SERIAL NUMBER(6byte), CRC CODE(1byte)��Ǎ���
		}
		if(rom_data[0] != FAMILY_CODE){		//FAMILY CODE���m�F����
			result = B_NG;
			continue;		//���g���C����
		}

		//ROM ID����擾
		break;			//���g���C�����𔲂���
	}

	return result;
}

/****************************************************/
/* Function : OWCheckConnect                       */
/* Summary  : �������f�o�C�X�̐ڑ�/���ڑ��m�F */
/* Argument : short ch                  			*/
/* Return   : �Ȃ�													        */
/* Caution  : �Ȃ�                               */
/* note     : ���ڑ���Ԃ���ڑ���ԂƂȂ����ꍇ�ɃZ���T����Ǎ���*/
/****************************************************/
void OWCheckConnect(short ch)
{

//	if(CNT_1SEC < GetTimeSpan32(time_sens_ctrl, gSysTimeOrg)){	//�����m�F
//		time_sens_ctrl = gSysTimeOrg;
//	}else{
//		return;
//	}

	if((SVD[ch].sensor_size == SNS_NONE) ||   //�Z���T�����ݒ�
   ((MES[ch].err_status & ERR_JUDGE_EMPTY) == 0)){	//��g�g�`�L�莞
		phase_owconnect[ch] = 1;
		return;
	}

//	portmesfwdW(0, ch);		//CH�ؑւ��L��
	portmesrevW(0, ch);		//CH�ؑւ��L��
	OWSensPowSW(B_ON);		//��ؕی��H�L��
	OWSensSigSW(B_ON);		//������޲��L��
	OWWaitUS(5);					//5usec�ҋ@

	switch(phase_owconnect[ch]){
	
	//*******
	//�ǂ�CH�����ڑ��ƂȂ������𔻕ʂ��鏈�������
	//�v���s�\�̏ꍇ�ɐڑ��m�F����
	//*******
	
		case 1:	//������޲����ڑ��m�F
			if(OWCheckDevice() == B_NG){		//������޲����ڑ����
				phase_owconnect[ch]++;	//̪��ލX�V
			}
 		break;
		case 2:	//������޲��ڑ��m�F
			if(OWCheckDevice() == B_OK){		//������޲��ڑ����
				phase_owconnect[ch]++;	//̪��ލX�V
			}
 		break;
		case 3:	//�ݻ����Ǎ���
			OWReadSensinfo(ch);	//������޲�����ݻ����Ǎ���
			phase_owconnect[ch]++;	//̪��ލX�V
			break;
		case 4:	//�Ǎ��񂾃Z���T����EEPROM������
			util_eep_allwrite(ch, WR_DEVICE);  //EEPROM������
			phase_owconnect[ch]++;	//̪��ލX�V
			break;
		case 5:	//����
		default:
			phase_owconnect[ch] = 1;	//̪��ޏ�����
			break;
	}

//	portmesfwdW(1, ch);		//CH�ؑւ�����
	portmesrevW(1, ch);		//CH�ؑւ�����
	OWSensSigSW(B_OFF);		//������޲�����
	OWSensPowSW(B_OFF);		//��ؕی��H����
}

/****************************************************/
/* Function : OWWriteData                       */
/* Summary  : �������f�o�C�X�Ƀf�[�^�������� */
/* Argument : �A�h���X�A�����݃f�[�^      			*/
/* Return   : ����:0 �ُ�:0�ȊO                   */
/* Caution  : �Ȃ�                               */
/* note     : �Ȃ�                                   */
/****************************************************/
short	OWWriteData(short address, const void *buff)
{
	short i, ow_address, result;
	unsigned char *p = (unsigned char *)buff;

	ow_address = address;
	result = B_NG;

	if(OWResetDevice()!=B_OK){  //RESET�p���X���o�͂��APRESENCE�p���X(�ԐM)���m�F����
		OW_write[0]++;
		return result;
	}

	OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"�R�}���h���s
	OWWriteByte(WRITE_SCRATCHPAD); //"WRITE SCRATCHPAD(0x0F)"�R�}���h���s
	OWWriteByte((char)(ow_address & 0x00FF)); 							// TA1 (�����ݐ�A�h���X)
	OWWriteByte((char)((ow_address & 0xFF00) >> 8)); // TA2

	for(i = 0; i < 8; i++){
		OWWriteByte(*p++); 		//�f�[�^��������
	}

	// read CRC data
	for(i = 0; i < 2; i++){
		memory_W_data[i] = OWReadByte();
	}

	if(OWResetDevice()!=B_OK){  //RESET�p���X���o�͂��APRESENCE�p���X(�ԐM)���m�F����
		OW_write[1]++;
		return result;
	}

	OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"�R�}���h���s
	OWWriteByte(COPY_SCRATCHPAD); //"COPY SCRATCHPAD(0x55)"�R�}���h���s
	OWWriteByte((char)(ow_address & 0x00FF)); 							// TA1 (�����ݐ�A�h���X)
	OWWriteByte((char)((ow_address & 0xFF00) >> 8)); // TA2
	OWWriteByte(0x07);
	result = B_OK;

	OWWaitMS(TIM_tPROGMAX);		//������޲������ݑҋ@
	OW_write[2]++;

	return result;
}

/****************************************************/
/* Function : OWCompareData                       */
/* Summary  : �������f�o�C�X�ɏ����񂾃f�[�^�̏ƍ����� */
/* Argument : �A�h���X�A�����݃f�[�^      			*/
/* Return   : ����:0 �ُ�:0�ȊO                   */
/* Caution  : �Ȃ�                               */
/* note     : �Ȃ�                                   */
/****************************************************/
short	OWCompareData(short address, const void *buff)
{
	short i, ow_address, result;

	ow_address = address;
	result = B_NG;

	if(OWResetDevice()!=B_OK){  //RESET�p���X���o�͂��APRESENCE�p���X(�ԐM)���m�F����
		OW_comp[0]++;
		return result;
	}

	OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"�R�}���h���s
	OWWriteByte(READ_MEMORY); //"READ MEMORY(0xF0)"�R�}���h���s
	OWWriteByte((char)(ow_address & 0x00FF)); 	// TA1 (�Ǎ��݊J�n�A�h���X)
	OWWriteByte((char)((ow_address & 0xFF00) >> 8)); // TA2

	// ������޲����ް���Ǎ���
	for(i = 0; i < 8; i++){
		memory_C_data[i] = OWReadByte();
	}

	//�ƍ�����
	if(memcmp(&memory_C_data[0], buff, 8) == 0){		//������޲��ɏ������ް���������޲��Ǎ����ް�����v
		result = B_OK;
		OW_comp[2]++;
	}else{
		OW_comp[1]++;
	}

	return result;
}

/****************************************************/
/* Function : OWReadSensinfo                       */
/* Summary  : �������f�o�C�X����Z���T����Ǎ���            */
/* Argument : short ch   �Ǎ���CH            	      */
/* Return   : ����:0 �ُ�:0�ȊO                      */
/* Caution  : �Ȃ�                                   */
/* note     : �Z���T���(հ�����Ұ�)��Ǎ���             */
/****************************************************/
short	OWReadSensinfo(short ch)
{
// char memory_data[MEMORY_MAX];
	short retry, result, size, cnt, pos;

	clrpsw_i();		/* ���荞�݋֎~ */
	result = B_NG;

//	portmesfwdW(0, ch);		//CH�ؑւ��L��
	portmesrevW(0, ch);		//CH�ؑւ��L��
	OWSensPowSW(B_ON);		//��ؕی��H�L��
	OWSensSigSW(B_ON);		//������޲��L��
	OWWaitUS(5);					//5usec�ҋ@

	if(OWCheckROMID() != B_OK){		//ROM_ID�̊m�F
		OW_read[0]++;
		MES[ch].err_status |= ERR_JUDGE_EEPROM;		/*EEPROM�G���[�Z�b�g*/
//		portmesfwdW(1, ch);		//CH�ؑւ�����
		portmesrevW(1, ch);		//CH�ؑւ�����
		OWSensSigSW(B_OFF);		//������޲�����
		OWSensPowSW(B_OFF);	 //��ؕی��H����
		setpsw_i();		/* ���荞�݋��� */	
		return B_NG;
	}

	for(retry = 0; retry < 3; retry++){	//���g���C����(���g���C3��)
		if(OWResetDevice() != B_OK){  //RESET�p���X���o�͂��APRESENCE�p���X(�ԐM)���m�F����
			result = B_NG;
			OW_read[1]++;
			continue;		//���g���C����
		}

		OWWriteByte(SKIP_ROM); //"SKIP ROM(0xCC)"�R�}���h���s
		OWWriteByte(READ_MEMORY); //"READ MEMORY(0xF0)"�R�}���h���s
	
		OWWriteByte(0x00); // TA1 (�Ǎ��݊J�n�A�h���X:0x0000)
		OWWriteByte(0x00); // TA2
		
		//������޲�������ް���Ǎ���
		for(cnt = 0; cnt < MEMORY_MAX; cnt++){
			memory_R_data[cnt] = OWReadByte();
		}

		//�擪��FAMILY CODE���ێ�����Ă��邱�Ƃ��m�F����B
		if(memory_R_data[0] == FAMILY_CODE){ 	//�uFAMILY CODE�v���ێ�����Ă���ꍇ
			;
		}else{		//�uFAMILY CODE�v���ێ�����Ă��Ȃ��ꍇ
			result = B_BLANK;		//EEPROM�ǂݍ��ݴװ�ɂ��Ȃ�(����������Ă��Ȃ��V�i��Ԃ�������޲���Ǎ��񂾎��́AModbus�ɃR�s�[���Ȃ��ׂ̏����BEEPROM�ɏ������܂Ȃ��B)
			OW_read[2]++;
			continue;		//���g���C����
		}

		//CRC�`�F�b�N
		size = 0;
		for(cnt=0; OWEepromData[cnt].ch!=0xffff; cnt++){		//�Z���T���̃T�C�Y���擾
		 if(OWEepromData[cnt].ch == ch){  //�Ώ�CH�̃Z���T���̃T�C�Y���v�Z����
 			size += OWEepromData[cnt].size;
			}
		}
		size += 1;		//�擪��FAMILY CODE���̃T�C�Y�����Z
		if(memory_R_data[size] == GetCRC8(&memory_R_data[0], size)){		//�ۑ�����CRC�ƌv�Z����CRC����v
			result = B_OK;
			OW_read[4]+=10;

			pos = 1;  //�擪��FAMILY CODE���ێ�����Ă���̂�1����J�n����
			for(cnt=0; OWEepromData[cnt].ch!=0xffff; cnt++){
 		 if(OWEepromData[cnt].ch == ch){  //�Ώ�CH
	  		memcpy(OWEepromData[cnt].p, &memory_R_data[pos], OWEepromData[cnt].size);	//������޲����ް���Modbus�ɃR�s�[
		 		pos += OWEepromData[cnt].size;
				}
			}
			break;		//���g���C�����𔲂���		
		}else{		//CRC�s��v
			result = B_NG;
			OW_read[3]++;
			continue;		//���g���C����
		}
	}

	if(result == B_NG){		//�Z���T���Ǎ��ݎ��s�����ꍇ
		MES[ch].err_status |= ERR_JUDGE_EEPROM;		/*EEPROM�G���[�Z�b�g*/
	}else{
		MES[ch].err_status &= ~ERR_JUDGE_EEPROM;		/*EEPROM�G���[���Z�b�g*/
	}

//	portmesfwdW(1, ch);	 //CH�ؑւ�����
	portmesrevW(1, ch);	 //CH�ؑւ�����
	OWSensSigSW(B_OFF);	 //������޲�����
	OWSensPowSW(B_OFF);	 //��ؕی��H����
	setpsw_i();		/* ���荞�݋��� */	

	return result;
}

/****************************************************/
/* Function : OWWriteSensinfo                      */
/* Summary  : �������f�o�C�X�ɃZ���T����������             */
/* Argument : �Ȃ�                               		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
/* note     : �Z���T���(հ�����Ұ�)��������            */
/****************************************************/
void	OWWriteSensinfo(void)
{
	short cnt, num;

	switch(phase_owwrite){
		/** �������f�o�C�X�����ݗv���m�F **/
		case 1:
 		if(OWwrite[OWwriteCH] == 1){  //�������f�o�C�X�����ݗv������
 			phase_owwrite++;	//̪��ލX�V
			}else{
				OWwriteCH++;  //������CH�X�V
				if(OWwriteCH >= CH_NUMMAX){
				 OWwriteCH = CH1;
				}
			}
 		break;

		/** �������f�o�C�X�����ݑҋ@ **/
		case 2:
 		if((action_status_check(OWwriteCH) == ACT_STS_NORMAL) ||  //�ʏ���
 		   (action_status_check(OWwriteCH) == ACT_STS_ADDIT)){    //�ώZ���s��
				clrpsw_i();		/* ���荞�݋֎~ */
//				portmesfwdW(0, OWwriteCH);		//CH�ؑւ�
				portmesrevW(0, OWwriteCH);		//CH�ؑւ�
				OWSensPowSW(B_ON);		//��ؕی��H�L��
				OWSensSigSW(B_ON);		//������޲��L��
				phase_owwrite++;	//̪��ލX�V
			}
			break;

		/** �����݃f�[�^��sens_info�ɍ쐬���� **/
		case 3:
			clrpsw_i();		/* ���荞�݋֎~ */
			sens_info[0] = FAMILY_CODE;		//�擪�ɁuFAMILY CODE�v��������(�V�i��Ԃ�������޲��𔻕ʂ��邽��)
			ow_pos = 1;  //�擪��FAMILY CODE���ێ�����Ă���̂�1����J�n����
			for(cnt=0; OWEepromData[cnt].ch!=0xffff; cnt++){
			 if(OWEepromData[cnt].ch == OWwriteCH){  //�Ώ�CH
 				memcpy(&sens_info[ow_pos], OWEepromData[cnt].p, OWEepromData[cnt].size);	//EEPROM�����݃f�[�^��sens_inf�ɃR�s�[
	 			ow_pos += OWEepromData[cnt].size;
				}
			}
			phase_owwrite++;	//̪��ލX�V
			break;

		/** CRC8�̌v�Z **/
		case 4:
			clrpsw_i();		/* ���荞�݋֎~ */
			sens_info[ow_pos] = GetCRC8(&sens_info, ow_pos);
			ow_cnt = ow_retry = 0;
			phase_owwrite++;	//̪��ލX�V
			break;
		/** �������f�o�C�X�ɏ����� **/
		case 5:
			clrpsw_i();		/* ���荞�݋֎~ */
			if(OWWriteData(0x0008*ow_cnt, &sens_info[0x0008*ow_cnt]) == B_OK){	//�����ݐ���
				phase_owwrite++;	//̪��ލX�V
			}else{	//�����ُ݈�
				ow_retry++;
				if(ow_retry > 3){		//��ײ3��
					phase_owwrite = 7;	//̪��ލX�V(�����I��)
				}
			}
			break;

		/** �������f�o�C�X�ɏ����񂾃f�[�^�̏ƍ����� **/
		case 6:
			clrpsw_i();		/* ���荞�݋֎~ */
			if(OWCompareData(0x0008*ow_cnt, &sens_info[0x0008*ow_cnt]) == B_OK){	//�ƍ����킵���ꍇ
				ow_cnt++;
				num = ow_pos / 8 + 1;
				if(ow_cnt > num){
					phase_owwrite++;	//̪��ލX�V(�S���ڽ������&�ƍ��I��)
				}else{
					phase_owwrite = 5;	//̪��ލX�V(�����ڽ������)
				}
			}else{		//�ƍ��ُ킵���ꍇ
				ow_retry++;
				if(ow_retry > 3){		//��ײ3��
					phase_owwrite = 7;	//̪��ލX�V(�����I��)
				}else{
					phase_owwrite = 5;	//̪��ލX�V(�ď�����)
				}
			}
			break;

		/** �����I�� **/
		case 7:
		default:
//			portmesfwdW(1, OWwriteCH);	//CH�ؑւ�����
			portmesrevW(1, OWwriteCH);	//CH�ؑւ�����
			OWSensSigSW(B_OFF);		//������޲�����
			OWSensPowSW(B_OFF);		//��ؕی��H����
			OWwrite[OWwriteCH] = 0;		//�������f�o�C�X�����ݗv���ر
			OWwriteCH++;  //������CH�X�V
			if(OWwriteCH >= CH_NUMMAX){
			 OWwriteCH = CH1;
   }
			phase_owwrite = 1;	//̪��ޏ�����
			setpsw_i();		/* ���荞�݋��� */			
			break;
		}
}

/****************************************************/
/* Function : OWWaitUS                   */
/* Summary  : ��1us�P�ʑҋ@ */
/* Argument : �҂�����                              */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �����Ŏw�肵���l�~1us���ҋ@����         */
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
/* Summary  : ��1ms�P�ʑҋ@ */
/* Argument : �҂�����                              */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �����Ŏw�肵���l�~1ms���ҋ@����         */
/****************************************************/
void OWWaitMS(long ms){

	volatile long i, j;
	
	for(i = 0; i < ms; i++){
		for(j = 0; j < 200; j++){
			OWWaitUS(5);
		}
	}
}
