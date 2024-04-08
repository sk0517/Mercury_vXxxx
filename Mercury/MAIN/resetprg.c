//
//
// TI版ファームウェアではこのファイル中の処理は main.c などで行う 
//
//
#if 0

/***********************************************/
/* File Name : resetprg.c	         									   */
/*	Summary   : リセット処理					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include	<machine.h>
#include	<_h_c_lib.h>
//#include	<stddef.h>					// Remove the comment when you use errno
//#include 	<stdlib.h>					// Remove the comment when you use rand()
#include	"typedefine.h"		// Define Types
#include	"stacksct.h"		// Stack Sizes (Interrupt and User)

#ifdef __cplusplus
extern "C" {
#endif
void PowerON_Reset_PC(void);
void main(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus				// Use SIM I/O
extern "C" {
#endif
extern void _INIT_IOLIB(void);
extern void _CLOSEALL(void);
#ifdef __cplusplus
}
#endif

#define PSW_init  0x00010000	// PSW bit pattern
#define FPSW_init 0x00000000	// FPSW bit base pattern


extern void init(void);

//extern void srand(_UINT);		// Remove the comment when you use rand()
//extern _SBYTE *_s1ptr;				// Remove the comment when you use strtok()
		
//#ifdef __cplusplus				// Use Hardware Setup
//extern "C" {
//#endif
//extern void HardwareSetup(void);
//#ifdef __cplusplus
//}
//#endif
	
//#ifdef __cplusplus			// Remove the comment when you use global class object
//extern "C" {					// Sections C$INIT and C$END will be generated
//#endif
//extern void _CALL_INIT(void);
//extern void _CALL_END(void);
//#ifdef __cplusplus
//}
//#endif

#pragma section ResetPRG		// output PowerON_Reset to PResetPRG section

#pragma entry PowerON_Reset_PC

void PowerON_Reset_PC(void)
{ 
	set_intb(__sectop("C$VECT"));

#ifdef __ROZ					// Initialize FPSW
#define _ROUND 0x00000001			// Let FPSW RMbits=01 (round to zero)
#else
#define _ROUND 0x00000000			// Let FPSW RMbits=00 (round to nearest)
#endif
#ifdef __DOFF
#define _DENOM 0x00000100			// Let FPSW DNbit=1 (denormal as zero)
#else
#define _DENOM 0x00000000			// Let FPSW DNbit=0 (denormal as is)
#endif

	set_fpsw(FPSW_init | _ROUND | _DENOM);

	_INITSCT();

	_INIT_IOLIB();					// Use SIM I/O

	init();

    nop();
		
	set_psw(PSW_init);				// Set Ubit & Ibit for PSW
		
	main();

	_CLOSEALL();					// Use SIM I/O
	
	brk();
}

#endif
