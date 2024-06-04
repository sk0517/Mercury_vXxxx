/***********************************************/
/* File Name : download.c	         									   */
/*	Summary   : �t�@�[���E�F�A�_�E�����[�h����            */
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
char RX_buf_dl[MSG_MAX_DL];   /*��M�p�z��*/
char TX_buf_dl[MSG_MAX_DL];   /*���M�p�z��*/

short byte_count_dl;          /*��M�o�C�g�J�E���^*/
short err_stat_dl;            /*�G���[�X�e�C�^�X*/
short rx_end_dl;              /*��M�����t���O*/
short	buf_count_dl;
unsigned short	recv_sa_dl;	//���M���X�e�[�V�����A�h���X

static char receive_enable= 0;	/* 0:��M�����A1:��M�L�� */

// RAM ��Ŏ��s����R�[�h���R�s�[���邽�߂̃o�b�t�@�B 
// map �t�@�C���� RAM_PRG �̃T�C�Y�ȏ�̗̈���m�ہB
// ���S�̂��ߍœK�������ɂ����ꍇ�� RAM_PRG �̃T�C�Y�ȏ�̗̈���m�ہB 
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
/*	���W���[������`�֐�								*/
/********************************************************/

/********************************************************/
/*	���W���[���O��`�֐�								*/
/********************************************************/

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/


void download(short mode, short type) {

	void (*enter_RAM_PRG_p)(short mode, short type);
	unsigned long enter_RAM_PRG_offset;
	unsigned long size;
	
 	// �����݋֎~
	clrpsw_i();
	
	// WDT stop
	WatchdogResetDisable(WATCHDOG0_BASE);

	if(mode == HOST){				//�z�X�g�|�[�g
		if(type == COM_RS485){		//RS485���
			com_init_dl_host();
		}else{						//CUnet���
			com_init_dl_cunet();
		}
	}else{							//�����e�i���X�|�[�g
		com_init_dl_ment();
	}
	
	size = (unsigned long)RAM_PRG_size;
	
	// �R�[�h�T�C�Y���傫������ꍇ�́A�s���� FW ���������܂Ȃ��悤�ɂ����Ńt���[�Y 
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
/*  �ʐM�̏�����(�����e�i���X�|�[�g)                */
/*  CH1�ݒ���g�p:CH����                            */
/****************************************************/
void    com_init_dl_ment(void){
    
    SVD[CH1].sti = (SVD[CH1].com_interval << 8) | SVD[CH1].com_speed;
    SVD[CH1].cmod = (SVD[CH1].com_mode << 8) | SVD[CH1].com_parity;

    receive_enable = 0;	/*��MDISABLE*/

    IntDisable(INT_UART_COM_MENT);
    UARTDisable(UART_COM_MENT_BASE);
    UARTConfigSetExpClk(UART_COM_MENT_BASE, g_ui32SysClock, 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
    UARTEnable(UART_COM_MENT_BASE);

    delay_dl(200);              /*�ҋ@*/

    RX_enable_dl_ment();       /*��M����*/
}

/****************************************************/
/*  �ʐM�̏�����(�z�X�g�|�[�g)                      */
/*  CH1�ݒ���g�p:CH����                            */
/****************************************************/
void    com_init_dl_host(void){
    
    SVD[CH1].sti = (SVD[CH1].com_interval << 8) | SVD[CH1].com_speed;
    SVD[CH1].cmod = (SVD[CH1].com_mode << 8) | SVD[CH1].com_parity;

    receive_enable = 0;	/*��MDISABLE*/

    IntDisable(INT_UART_COM_HOST);
    UARTDisable(UART_COM_HOST_BASE);
    UARTConfigSetExpClk(UART_COM_HOST_BASE, g_ui32SysClock, 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
    UARTEnable(UART_COM_HOST_BASE);

    delay_dl(200);              /*�ҋ@*/

    RX_enable_dl_host();       /*��M����*/
}

/****************************************************/
/*  �ʐM�̏�����(CUnet) 		                    */
/****************************************************/
void    com_init_dl_cunet(void){

	MKY43.REG.MR0CR.BIT.RDY = 1;	//MRB1�����[����M����
	MKY43.REG.MR1CR.BIT.RDY = 0;	//MRB1�����[����M�֎~

	rx_end_dl = 0;                  /*��M�����t���O���Z�b�g*/

	memset(TX_buf_dl, 0, sizeof(TX_buf_dl));    /*����M�o�b�t�@�N���A*/
    memset(RX_buf_dl, 0, sizeof(RX_buf_dl));
}

// �Ȍ�̃R�[�h�� RAM �ɃR�s�[���� RAM ��Ŏ��s���� 
#pragma SET_CODE_SECTION("RAM_PRG")

// RAM ��̎��s�J�n�֐� 
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

// uart.c ����R�s�[���֐����� _dl ��t���B 
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

// uart.c ����R�s�[���֐����� _dl ��t���B 
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

// uart.c ����R�s�[���֐����� _dl ��t���B 
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

// uart.c ����R�s�[���֐����� _dl ��t���B 
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

// uart.c ����R�s�[���֐����� _dl ��t���B 
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

// uart.c ����R�s�[���֐����� _dl ��t���B 
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

// flash.c ����R�s�[���֐����� _dl ��t���B 
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

// flash.c ����R�s�[���֐����� _dl ��t���B 
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

// gpio.c ����R�s�[���֐����� _dl ��t���B 
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

// watchdog.c ����R�s�[���֐����� _dl ��t���B 
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

// watchdog.c ����R�s�[���֐����� _dl ��t���B 
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
/*  �f�B���C                                            */
/*      ���� delay   : �J�E���g�l                       */
/********************************************************/
void delay_dl(unsigned short delay_count){

    volatile unsigned short counter;
    
    for (counter = 0; counter < delay_count; counter++){
        ;
    }
}

/****************************************************/
/*  RS485��M���i1�t���[����M�J�n�j              */
/*    (�����e�i���X�|�[�g)                          */
/*  byte_count_dl = 0;      �J�E���^���Z�b�g        */
/*  rx_end_dl = 0           ��M�����t���O���Z�b�g  */
/*  SCI5.SCR.BIT.RE = 1;    ��MENABLE              */
/*                                                  */
/****************************************************/
void    RX_enable_dl_ment(void){

    byte_count_dl = 0;              /*�J�E���^���Z�b�g*/
    rx_end_dl = 0;                  /*��M�����t���O���Z�b�g*/

    memset_dl(TX_buf_dl, 0, sizeof(TX_buf_dl));    /*����M�o�b�t�@�N���A*/
    memset_dl(RX_buf_dl, 0, sizeof(RX_buf_dl));

    // GPIO_PD6 
    __bit_output_dl(GPIO_PORTD_BASE, 6, 0);
    
    receive_enable = 1;	/*��MENABLE	*/
}

/****************************************************/
/*  RS485��M���i1�t���[����M�J�n�j              */
/*    (�z�X�g�|�[�g) 		                        */
/*  byte_count_dl = 0;      �J�E���^���Z�b�g        */
/*  rx_end_dl = 0           ��M�����t���O���Z�b�g  */
/*  SCI5.SCR.BIT.RE = 1;    ��MENABLE              */
/*                                                  */
/****************************************************/
void    RX_enable_dl_host(void){

    byte_count_dl = 0;              /*�J�E���^���Z�b�g*/
    rx_end_dl = 0;                  /*��M�����t���O���Z�b�g*/

    memset_dl(TX_buf_dl, 0, sizeof(TX_buf_dl));    /*����M�o�b�t�@�N���A*/
    memset_dl(RX_buf_dl, 0, sizeof(RX_buf_dl));

    // GPIO_PP4 
    __bit_output_dl(GPIO_PORTP_BASE, 4, 0);
    
    receive_enable = 1;	/*��MENABLE	*/
}

/****************************************************/
/*  RS485��M�����ݏ���(�����e�i���X�|�[�g)         */
/*                                                  */
/*  ��M�f�[�^��RX_buf_dl[]�ɓ����                 */
/*  byte_count_dl = 0;      �J�E���^���Z�b�g        */
/*  rx_end_dl = 0;          ��M�����t���O���Z�b�g  */
/*  SCI5.SCR.BIT.RE = 1;    ��MENABLE              */
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

	if(receive_enable != 1) { /* ��MDISABLE�� */
		/* ��M�f�[�^�ǂݎ̂� */
		if (UARTCharsAvail_dl(UART_COM_MENT_BASE)) {
			UARTCharGetNonBlocking_dl(UART_COM_MENT_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*����*/
		if (UARTCharsAvail_dl(UART_COM_MENT_BASE)) {	/*��MOK*/
            RX_buf_dl[byte_count_dl] = UARTCharGetNonBlocking_dl(UART_COM_MENT_BASE);/*1byte �f�[�^��M*/
            if ( byte_count_dl == 0) {          /*read top data*/
                if (RX_buf_dl[0]=='S'){ /*�擪����*/
                    byte_count_dl++;    /*next RX_buf pointer*/
                } else {
                    //for(i=0; i<2; i++);
                    //dumy =100*dumy/10;
                }
            }else{
                if (((RX_buf_dl[byte_count_dl]>='A') && (RX_buf_dl[byte_count_dl]<='Z'))/*�L���R�[�h*/
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
            //goto    REPEAT;             /*��蒼��*/
        }
    } else {                            /*��M�G���[*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*�I�[�o�[�����G���[*/
            err_stat_dl |= 0x0004;
        }
		if ((ui32Status & UART_INT_FE) != 0) {	/*�t���[�~���O�G���[*/
            err_stat_dl |= 0x0002;
        }
		if ((ui32Status & UART_INT_PE) != 0) {	/*�p���e�B�G���[*/
            err_stat_dl |= 0x0001;
        }
        return;
    }
    /*�I���`�F�b�N*/
    if (RX_buf_dl[byte_count_dl-1] == CR){  /*��M�I��*/
		receive_enable = 0;            /*��MDISABLE*/

        rx_end_dl = 1;                  /*�t���[����M�����t���O���Z�b�g*/
    }
}

/****************************************************/
/*  RS485��M�����ݏ���(�z�X�g�|�[�g)               */
/*                                                  */
/*  ��M�f�[�^��RX_buf_dl[]�ɓ����                 */
/*  byte_count_dl = 0;      �J�E���^���Z�b�g        */
/*  rx_end_dl = 0;          ��M�����t���O���Z�b�g  */
/*  SCI6.SCR.BIT.RE = 1;    ��MENABLE              */
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

	if(receive_enable != 1) { /* ��MDISABLE�� */
		/* ��M�f�[�^�ǂݎ̂� */
		if (UARTCharsAvail_dl(UART_COM_HOST_BASE)) {
			UARTCharGetNonBlocking_dl(UART_COM_HOST_BASE);
		}
		return;
	}

	if (( ui32Status & (UART_INT_OE | UART_INT_FE | UART_INT_PE) ) == 0) {	/*����*/
		if (UARTCharsAvail_dl(UART_COM_HOST_BASE)) {	/*��MOK*/
            RX_buf_dl[byte_count_dl] = UARTCharGetNonBlocking_dl(UART_COM_HOST_BASE);/*1byte �f�[�^��M*/
            if ( byte_count_dl == 0) {          /*read top data*/
                if (RX_buf_dl[0]=='S'){ /*�擪����*/
                    byte_count_dl++;    /*next RX_buf pointer*/
                } else {
                    //for(i=0; i<2; i++);
                    //dumy =100*dumy/10;
                }
            }else{
                if (((RX_buf_dl[byte_count_dl]>='A') && (RX_buf_dl[byte_count_dl]<='Z'))/*�L���R�[�h*/
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
            //goto    REPEAT;             /*��蒼��*/
        }
    } else {                            /*��M�G���[*/
		if ((ui32Status & UART_INT_OE) != 0) {	/*�I�[�o�[�����G���[*/
            err_stat_dl |= 0x0004;
        }
		if ((ui32Status & UART_INT_FE) != 0) {	/*�t���[�~���O�G���[*/
            err_stat_dl |= 0x0002;
        }
		if ((ui32Status & UART_INT_PE) != 0) {	/*�p���e�B�G���[*/
            err_stat_dl |= 0x0001;
        }
        return;
    }
    /*�I���`�F�b�N*/
    if (RX_buf_dl[byte_count_dl-1] == CR){  /*��M�I��*/
		receive_enable = 0;	/*��MDISABLE*/

        rx_end_dl = 1;                  /*�t���[����M�����t���O���Z�b�g*/
    }
}

/****************************************************/
/*  RS485���M�J�n�i1�t���[�����M�J�n�j              */
/*    (�����e�i���X�|�[�g)                          */
/*  TX_buf_dl[] �̓��e�𑗐M����                    */
/*  ���́Fbyte_su ���M�o�C�g��                      */
/*                                                  */
/****************************************************/
void    TX_data_dl_ment(short byte_su){
    short i,j;
    
    if ( byte_su == 0) return;  /*BYTE�����O�Ȃ牽�����Ȃ�*/

    // GPIO_PD6 
    __bit_output_dl(GPIO_PORTD_BASE, 6, 1);

    delay_dl(10000);                /*�ҋ@*/

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

    RX_enable_dl_ment();            /*��M����*/

//  setpsw_i();                     /* Initialize CCR/Interrupt Enable */
}

/****************************************************/
/*  RS485���M�J�n�i1�t���[�����M�J�n�j              */
/*    (�z�X�g�|�[�g)  		                        */
/*  TX_buf_dl[] �̓��e�𑗐M����                    */
/*  ���́Fbyte_su ���M�o�C�g��                      */
/*                                                  */
/****************************************************/
void    TX_data_dl_host(short byte_su){
    short i,j;
    
    if ( byte_su == 0) return;  /*BYTE�����O�Ȃ牽�����Ȃ�*/

    // GPIO_PP4 
    __bit_output_dl(GPIO_PORTP_BASE, 4, 1);
    
    delay_dl(10000);                /*�ҋ@*/

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

    RX_enable_dl_host();            /*��M����*/

//  setpsw_i();                     /* Initialize CCR/Interrupt Enable */
}

////////////////////////////////////////////////////////////
// �X�g�����O�Z�b�g ���[�h(16bit)
//  R1 �|�C���^ R2 �f�[�^�@R3 �T�C�Y
//  dst(R1)����sdata(R2)��size(R3)��������
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
//// �X�g�����O�Z�b�g �o�C�g(16bit)
////  R1 �|�C���^ R2 �f�[�^�@R3 �T�C�Y
////  dst(R1)����sdata(R2)��size(R3)��������
/////////////////////////////////////////////////////////////
//#pragma inline_asm fdl_memcpy
//void fdl_memcpy(unsigned char *src, unsigned char *dst, short size) {
//    // R2�F�� R1�F�� R3:�� �����T�C�Y:�o�C�g
//    SMOVF
//}

//////////////////////////////////////////////////////////////////
//  RAM�Ŏ��s                                                   //
//  �A�h���X���܂ރu���b�N�̃C���[�X���s��                      //
//���^�[�� 0 ����                                               //
//                                                              //
//////////////////////////////////////////////////////////////////
void block_erase(void *ptr) {

	FlashErase_dl((uint32_t)ptr);
}

///////////////////////////////////////////////////////////////////////////////
//  RAM�Ŏ��s
//  256�o�C�g�̃f�[�^����������
//  data[]�̂����������Ȃ�������0xffff�ɗ\�߂���K�v������B
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
 * Summary  : Flash�f�[�^�̏�������
 * Argument : *ptr -> �������ݐ�A�h���X
 *          : data -> �������ރf�[�^
 * Return   : WrtFlg -> 0 : ����
 *                     -1 : ���s
 * Caution  : None
 * Note     : 
 * *****************************************/
short FPGA_WRITE(void *ptr, unsigned short data[]) {
    short WrtFlg = 0;

    //�A�h���X�s��
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
unsigned char   type;               // ���R�[�h�̃^�C�v
unsigned short  record_len;         // ���R�[�h��
unsigned long   head_addr;          // �擪�̃A�h���X
unsigned long   tail_addr;          // �����̃A�h���X
unsigned char   data[256];          // �f�[�^�̃o�b�t�@�ő�256byte
unsigned char   data_len;           // �f�[�^�̒���
unsigned char   sum;                // �`�F�b�N�T��
unsigned char   sum_calc;           // �`�F�b�N�T���v�Z�l
}S_FORMAT;

S_FORMAT    s_data;

////////////////////////////////////////////////////////////////////////////////
//  �A�X�L�[1�����𐔎��ɒ���                                                 //
//    0-F�i30-39�A41-46�܂ŏo�Ȃ���΂Ȃ�Ȃ�                                 //
//     ���̏ꍇ�́A���얢��                                                   //
//     Uchar ascii2chr(Uchar cdata)                                           //
////////////////////////////////////////////////////////////////////////////////
unsigned char ascii2char(unsigned char data) {

    if(((unsigned char)'0' <= data) && (data <= (unsigned char)'9')) return (unsigned char)(data - '0');
    if(((unsigned char)'A' <= data) && (data <= (unsigned char)'F')) return (unsigned char)(data - 'A' + 10);
    return  (unsigned char)0x20;
}

/******************************************************************************************
   S�t�H�[�}�b�g���
    ptr      :S�t�H�[�}�b�g�̃f�[�^��
    s_data   :���ʂ̊i�[�ꏊ
    ���^�[���F 0:����
              -1:�擪��'S'�łȂ�
              -2:�^�C�v����������
              -4:SUM�G���[
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

    // �擪��'S'�Ŗ�����΁A�G���[ return -1
    if(*ptr++ != 'S') return -1;

    // ���R�[�h�^�C�v
    s_data.type = *ptr++ - '0';

    // ���R�[�h��
    datah = (unsigned char)ascii2char(*ptr++);
    datal = (unsigned char)ascii2char(*ptr++);
    data = (unsigned char)(datah * 16 + datal);
    s_data.record_len = s_data.sum_calc = data;

    switch (s_data.type){
    case 0:     // 4�o�C�g�A�h���X
    case 1:
    case 9:
        s_data.data_len = s_data.record_len - ( 2+1 );
        break;
    case 2:     // 6�o�C�g�A�h���X
    case 8:
        s_data.data_len = s_data.record_len - ( 3+1 );
        break;
    case 3:     // 8�o�C�g�A�h���X
    case 7:
        s_data.data_len = s_data.record_len - ( 4+1 );
        break;
     }// switch (s_data.type)

    ltmp = 0UL;
    // �A�h���X
    switch (s_data.type){
    case 3:     // 8�o�C�g�A�h���X
    case 7:
        datah =ascii2char(*ptr++);
        datal =ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 + datal);
        s_data.sum_calc += data;
        ltmp = data;
        break;
    case 2:     // 6�o�C�g�A�h���X
    case 8:
        datah =ascii2char(*ptr++);
        datal =ascii2char(*ptr++);
        data = (unsigned char)(datah * 16 + datal);
        s_data.sum_calc += data;
        ltmp = ltmp * 256UL + data;
        break;
    case 0:     // 4�o�C�g�A�h���X
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
        return -2 ; /*�G���[*/
    }// switch (s_data.type)
    s_data.head_addr = ltmp;
    s_data.tail_addr = s_data.head_addr + s_data.data_len - 1;

    // �f�[�^
    for(cnt=0 ; cnt < (s_data.data_len) ; cnt++ ){ /* �o�C�g�J�E���g�́A�A�h���X����T���܂ł̂���2���炷*/
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
        return 0;   /* ����I�� */
    } else {
        return -4;  /* SUM�G���[*/
    }
}


//////////////////////////////////////////////////////////////////////////////
//  short get_block_id(unsigned long addr)
//      ���́FROM�A�h���X
//      �߂�l�FROM�A�h���X���܂܂��u���b�N��ID
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
//  1���R�[�h�P�ʂ̏���
//  short write_data()
//      ���́FS_FORMAT s_data
//      �߂�l�F0 : �ŏI���R�[�h
//              1 : �ŏI���R�[�h�ȊO
//
//  �V�u���b�N�Ȃ�A���̃u���b�N������
//  �������ނׂ��f�[�^�̓o�b�t�@�ɒ~���Ă���
//  256byte���E���z������A�����܂ł̃o�b�t�@�̃f�[�^����������
//  �ŏI���R�[�h�ŁA�o�b�t�@�̃f�[�^����������
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

    // �f�[�^�u���b�N�ŁA������1�ȏ�̏ꍇ
    if( ( (s_data.type == 1) || (s_data.type == 2) || (s_data.type == 3) ||
          (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) && (s_data.data_len > 0) ) {

        // �قȂ�u���b�N�Ȃ�A�V�u���b�N������
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

        // �قȂ�256byte�Ȃ�A����܂ł̃f�[�^����������
        curr_head_addr_256 = s_data.head_addr & 0xFFFFFF00UL;
        curr_tail_addr_256 = s_data.tail_addr & 0xFFFFFF00UL;

        if(prev_addr_256 != curr_head_addr_256) {                   // �擪���قȂ�256byte�̏ꍇ
            if(pending_size > 0) {
                FROM_WRITE((void *)prev_addr_256, ROM_WR_BUFF);
            }
            memsetW(ROM_WR_BUFF, 0xffff, 128);                      // ����̂��߂Ƀo�b�t�@��0xff�Ŗ��߂�
            pending_size = 0;
            prev_addr_256 = curr_head_addr_256;
        }

        // �o�b�t�@�ɏ������݃f�[�^���}�[�W����
        offset = (short)(s_data.head_addr & 0x000000FFUL);
        write_size = min_dl(256 - offset, s_data.data_len);
        memcpy_dl((void*)((unsigned long)(&ROM_WR_BUFF[0]) + offset), (void*)(&s_data.data[0]), write_size);
        pending_size += write_size;

        if(curr_head_addr_256 != curr_tail_addr_256) {              // ���������ꍇ
            FROM_WRITE((void *)curr_head_addr_256, ROM_WR_BUFF);        // �O������������

            memsetW(ROM_WR_BUFF, 0xffff, 128);                          // �㔼�̂��߂Ƀo�b�t�@��0xff�Ŗ��߂�
            pending_size = s_data.data_len - write_size;
            memcpy_dl(ROM_WR_BUFF, (void*)(&s_data.data[write_size]), pending_size);
            prev_addr_256 = curr_tail_addr_256;
        }
    }
    // �ŏI���R�[�h�̏ꍇ
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
//  1���R�[�h�P�ʂ̏���
//  short check_data()
//      ���́FS_FORMAT s_data
//      �߂�l�F0 : �ŏI���R�[�h
//              1 : �ŏI���R�[�h�ȊO
//
//      ��r�ňُ킪����΁A�������[�v�ɓ���B�f�o�b�O�p
//
//////////////////////////////////////////////////////////////////////////////
short check_data(void) {
    short i;

    // �f�[�^�u���b�N�ŁA������1�ȏ�̏ꍇ
    if( ( (s_data.type == 1) || (s_data.type == 2) || (s_data.type == 3) ||
          (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) && (s_data.data_len > 0) ) {

        for(i = 0; i < s_data.data_len; i++) {
            if(*((unsigned char *)(s_data.head_addr + (unsigned long)i)) != s_data.data[i]) {
                for(;;);
            }
        }
    }

    // �ŏI���R�[�h�̏ꍇ
    if( (s_data.type == 7) || (s_data.type == 8) || (s_data.type == 9) ) {
        return 0;
    }
    return 1;
}
#endif

/************************************************/
/* LED�\���X�V                                  */
/*          ���́F�\��LED                       */
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

	FPGA_LED_CNT = led_num;		//LED���䃌�W�X�^�ɐݒ�
}

/***************************************************************/
/* LED���A�ǂݍ��ݍs��10�����閈�ɃV�t�g����悤�ɓ_������     */
/* switch�����g���ƕʂ�SEG�Ƀe�[�u���������̂ŁAif���ŋL�q */
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
/*	���[����M�ҋ@												*/
/****************************************************************/
void	mky43_wait_mail_recv_dl(void){

	while(MKY43.REG.MR0CR.BIT.RCV == 0);		//���[����M�����ҋ@
	
}

/****************************************************************/
/*	���M�f�[�^��send_data_dl[]�ɕۑ�							*/
/****************************************************************/
void	mky43_TX_start_dl(short byte_num){

	short i;
	short				j_cnt;
	unsigned short	send_sa;			//���M��X�e�[�V�����A�h���X
	unsigned short	resend_cnt;

	// send_data_dl[32][4]  = { 0 } �ŏ����l���Z�b�g����Ɨ�O��������B 
	// �㑱�����ɂ� memset_dl �ŏ��������Ă���̂ł����ł̏������͕s�v�B 
	unsigned short	send_data_dl[32][4];		//���[�����M�f�[�^
	
	/*�擪�A�h���X(1byte)�ɁwCH�f�[�^(0�Œ�)�x������*/
	/*CUnet�ʐM���A�z�X�g����̃��b�Z�[�W���Ɍ���*/
	for(i=byte_num; i>=0; i--){	
		TX_buf_dl[i+1] = TX_buf_dl[i];
	}
	//TX_buf_dl[0] = 0x30;
	TX_buf_dl[0] = 0;	

	memset_dl(send_data_dl, 0, sizeof(send_data_dl));
	for(i=0; i<4; i++){
		send_data_dl[0][i] = TX_buf_dl[2*i + 1]*0x0100 + TX_buf_dl[2*i];
	}

	send_sa = recv_sa_dl;						//���[�����M��X�e�[�V�����A�h���X
	resend_cnt = 0;

	while(MKY43.REG.MSCR.BIT.SEND == 1);		//���[�����M���͑ҋ@
		
	MKY43.REG.MESR.WORD = 0;					//���[�����M�G���[�X�e�[�^�X�N���A

	for(j_cnt=0; j_cnt<4; j_cnt++){
		MKY43.MSB.SEND[0].DATA[j_cnt].DATA = send_data_dl[0][j_cnt];
	}

RESEND:
	
	MKY43.REG.MSCR.WORD = (0x0000 | (send_sa<<8) | 1);	//���M��X�e�[�V�����A�h���X�A�f�[�^�T�C�Y
	MKY43.REG.MSCR.BIT.SEND = 1;				//���[�����M�J�n

	while(MKY43.REG.MSCR.BIT.SEND == 1);		//���[�����M���͑ҋ@

	if(resend_cnt < MES_RESEND_MAX){
		if(MKY43.REG.MSCR.BIT.ERR != 0){			//���[�����M�G���[����
			if(MKY43.REG.MESR.BIT.NORDY != 0){		//���M��̎�M�o�b�t�@����M���łȂ�
				MKY43.REG.MESR.WORD = 0;			//���[�����M�G���[�X�e�[�^�X�N���A
				resend_cnt ++;						//�đ��񐔍X�V
				goto RESEND;						//�đ�����
			}
		}
	}
    rx_end_dl = 0;             /*��M�����t���O���Z�b�g*/
	
	for (i=0; i<MSG_MAX_DL; i++){
		RX_buf_dl[i] = 0;		/*����M�o�b�t�@�N���A*/
		TX_buf_dl[i] = 0;
	}
}

/****************************************************************/
/* ���[����M(MRB0:���[����M�o�b�t�@0							*/
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
	unsigned short	recv_size;		//���[���f�[�^�T�C�Y

	// recv_data0_dl[32][4] = { 0 } �ŏ����l���Z�b�g����Ɨ�O��������B 
	// �㑱�����ɂ� memset_dl �ŏ��������Ă���̂ł����ł̏������͕s�v�B 
	unsigned short	recv_data0_dl[32][4];		//���[����M�f�[�^(MRB0:���[����M�o�b�t�@0)

	recv_sa_dl = 0;
	recv_size = 0;
	memset_dl(recv_data0_dl, 0, sizeof(recv_data0_dl));
	
	/*MRB0:���[����M�o�b�t�@0*/
	if(MKY43.REG.MR0CR.BIT.RCV != 0){					//���[����M����
		recv_sa_dl = (MKY43.REG.MR0CR.WORD & 0x3F00);	//���M���X�e�[�V�����A�h���X�擾
		recv_size = (MKY43.REG.MR0CR.WORD & 0x003F);	//���[����M�f�[�^�T�C�Y�擾(8Byte��1�P��)

		for(i_cnt=0; i_cnt<recv_size; i_cnt++){		//���[����M�f�[�^�擾
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
				/*recv_data0[]��ۑ�*/
				RX_buf_dl[buf_num+buf_offset] = recv_data0_dl[i][j]%0x0100;
			}else{
				/*recv_data0[]��ۑ�*/
				RX_buf_dl[buf_num+buf_offset] = recv_data0_dl[i][j]/0x0100;
			}
			if(RX_buf_dl[buf_num+buf_offset] != CR){		//CR(0x0D)�Ȃ�
				buf_count_dl++;
			}else{											//CR(0x0D)����
				buf_count_dl = 0;
				break;
			}
			if((buf_num+buf_offset) > MSG_MAX_DL){
				break;
			}
		}

		if(buf_count_dl == 0){		//�S���b�Z�[�W(S����CR�܂ł̃f�[�^)��M����
			byte_count_host = 0;
			//data_len = strlen(RX_buf_dl);
			data_len = 0;
			while (RX_buf_dl[byte_count_host]!='S'){	/*�擪����*/
				byte_count_host++;
			}
			while (RX_buf_dl[data_len]!= CR){	/*�I�[����*/
				data_len++;
			}
			data_len++;
			data_len -= byte_count_host;
			for(i=0;i<data_len;i++){
				RX_buf_dl[i] = RX_buf_dl[i+byte_count_host];
			}
			RX_buf_dl[data_len] = 0;
			rx_end_dl = 1;                  /*�t���[����M�����t���O���Z�b�g*/
		}
		MKY43.REG.MR0CR.BIT.RDY = 1;					//���[����M����
	}

	MKY43.REG.INT0CR.BIT.MR = 1;						//���[����M���������݋���	
	MKY43.REG.INT0SR.BIT.MR = 1;						//���[����M���������݉���
}

/******************************************************************************************
    download
    S�t�H�[�}�b�g��HEX�t�@�C�����_�E�����[�h���āAROM���X�V����
    �_�E�����[�h�Ɋւ���R�[�h�͑S��RAM��ɖ�����΂Ȃ�Ȃ�
        1. ���荞�݋֎~�AWDT��~
        2. �ʐM��(��)�ݒ�
        3. ��s�ǂݍ���
        4. ��͂���
        5. ��s��ROM�ɏ�������
        6. ACK/NAK��Ԃ�
        7. �I���łȂ���΁A3�ɖ߂�
        8. �I������΁AIWDT��expire�����ă��Z�b�g����

	����
		mode: �@0�F�z�X�g�|�[�g���ʐM
				1�F�����e�i���X�|�[�g���ʐM
		type:�@ 0x1000�FRS485���
				0x2000�FCUnet���		
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
		if(type == COM_CUNET && mode == HOST){		//CUnet�ʐM�Ŏ�M����ꍇ
			mky43_wait_mail_recv_dl();				//���[����M�����ҋ@
			mky43_mail_recv_dl();					//���[����M(MRB0,MRB1)
		}else{										//RS485�ʐM�Ŏ�M����ꍇ
			do{										//�ʐM�o�R�łP�s�ǂݍ���
				if(mode == HOST){					//�z�X�g�|�[�g
					RX_data_dl_host();
				}else{								//�����e�i���X�|�[�g
					RX_data_dl_ment();
				}
			}while(rx_end_dl == 0);
		}

        if(s_read(RX_buf_dl) < 0) {     // �ʐM���e�̃`�F�b�N
            TX_buf_dl[0] = 0x21;            // NAK������
            TX_buf_dl[1] = 0x0d;
            TX_buf_dl[2] = 0x0a;
        } else {                            // ACK������
            TX_buf_dl[0] = 0x06;
            TX_buf_dl[1] = 0x0d;
            TX_buf_dl[2] = 0x0a;
        }

#if DL_DEBUG
        cont_flag = check_data();       // ROM�̃f�[�^�Ɣ�r����
#else
		cont_flag = write_data();       // ROM�Ƀf�[�^����������
#endif

		if(mode == HOST){				//�z�X�g�|�[�g
			if(type == COM_RS485){		//RS485���
				TX_data_dl_host(3);		//ACK/NAK��Ԃ�
			}else{						//CUnet���
				mky43_TX_start_dl(3);	//ACK/NAK��Ԃ�
			}
		}else{							//�����e�i���X�|�[�g
			TX_data_dl_ment(3);			//ACK/NAK��Ԃ�
		}
		led_dl(line_num++);
    }

    WatchdogReloadSet_dl(WATCHDOG1_BASE, g_ui32SysClock / 1000);	// 1ms ��ɍċN�� 
    WatchdogResetEnable_dl(WATCHDOG1_BASE);

    for(;;);    // wait for underflow of IWDT
}


