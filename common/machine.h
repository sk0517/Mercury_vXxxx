#ifndef __MACHINE_HEADER__
#define __MACHINE_HEADER__


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"


// ���̃t�@�C���ł͈ȉ��̒�`���s���B 
// �ECC-RX �ł̑g�ݍ��݊֐���u���������` 
// �EMPU �̃��W�X�^����֐���` 


inline void nop(void)
{
	__asm volatile(" nop");
}

inline signed long min(signed long data1, signed long data2)
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

inline unsigned long revw(unsigned long data)
{
	return ((data >> 8) & 0x00ff00ff) | ((data << 8) & 0xff00ff00);
}

inline void clrpsw_i(void)
{
	__asm( "	cpsid i" );
}

inline void setpsw_i(void)
{
	__asm( "	cpsie i" );
}

inline void interrupt_disable(void)
{
	__asm( "	cpsid i" );
}

inline void interrupt_enable(void)
{
	__asm( "	cpsie i" );
}

inline void __bit_output(unsigned long port, unsigned char pin, unsigned char bit_data)
{
	GPIOPinWrite(port, 1 << pin, bit_data << pin);
}

inline unsigned char __bit_input(unsigned long port, unsigned char pin)
{
	return (unsigned char)((GPIOPinRead(port, 1 << pin) >> pin) & 1);
}

inline void __bit_set(unsigned long port, unsigned char pin)
{
	GPIOPinWrite(port, 1 << pin, 1 << pin);
}

inline void __bit_clr(unsigned long port, unsigned char pin)
{
	GPIOPinWrite(port, 1 << pin, 0);
}

inline void __byte_output(unsigned long port, unsigned char byte_data)
{
	GPIOPinWrite(port, 0xff, byte_data);
}

/************************************************/
/*	RX �ł̃J�E���g�l����J�E���g�l���v�Z		*/
/*	cnt_rx										*/
/*		RX �ł̃J�E���g�l						*/
/*	source_clock_mhz_rx							*/
/*		RX �ł̃\�[�X�N���b�N (�P�ʂ� MHz)		*/
/*	clock_div_rx								*/
/*		RX �ł̕�����							*/
/*	source_clock_mhz							*/
/*		�\�[�X�N���b�N (�P�ʂ� MHz)				*/
/*	clock_div									*/
/*		������									*/
/************************************************/
inline unsigned long calc_cnt(unsigned long cnt_rx, unsigned long source_clock_mhz_rx, unsigned long clock_div_rx, unsigned long source_clock_mhz, unsigned long clock_div)
{
	return cnt_rx * source_clock_mhz * clock_div_rx / (source_clock_mhz_rx * clock_div);
}

/************************************************/
/*	�J�E���g�l���� RX �ł̃J�E���g�l���v�Z		*/
/*	cnt_rx										*/
/*		�J�E���g�l								*/
/*	source_clock_mhz_rx							*/
/*		RX �ł̃\�[�X�N���b�N (�P�ʂ� MHz)		*/
/*	clock_div_rx								*/
/*		RX �ł̕�����							*/
/*	source_clock_mhz							*/
/*		�\�[�X�N���b�N (�P�ʂ� MHz)				*/
/*	clock_div									*/
/*		������									*/
/************************************************/
inline unsigned long calc_cnt_rx(unsigned long cnt, unsigned long source_clock_mhz_rx, unsigned long clock_div_rx, unsigned long source_clock_mhz, unsigned long clock_div)
{
	return cnt * (source_clock_mhz_rx * clock_div) / (source_clock_mhz * clock_div_rx);
}

#endif
