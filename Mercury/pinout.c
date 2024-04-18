//*****************************************************************************
//
// Configure the device pins for different signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 2021/6/8 at 15:51:48
// by TI PinMux version 1.8.1+1900
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "pinout.h"

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the customer specific usage.
//!
//! \return None.
//
//*****************************************************************************
void
PinoutSet(void)
{
    //
    // Enable Peripheral Clocks 
    //
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOT);

    //
    // Configure the GPIO Pin Mux for PK6
	// for EPI0S25
    //
	MAP_GPIOPinConfigure(GPIO_PK6_EPI0S25);
	GPIOPinTypeEPI(GPIO_PORTK_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PL0
	// for EPI0S16
    //
	MAP_GPIOPinConfigure(GPIO_PL0_EPI0S16);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PG0
	// for EPI0S11
    //
	MAP_GPIOPinConfigure(GPIO_PG0_EPI0S11);
	GPIOPinTypeEPI(GPIO_PORTG_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA6
	// for EPI0S8
    //
	MAP_GPIOPinConfigure(GPIO_PA6_EPI0S8);
	GPIOPinTypeEPI(GPIO_PORTA_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PH1
	// for EPI0S1
    //
	MAP_GPIOPinConfigure(GPIO_PH1_EPI0S1);
	GPIOPinTypeEPI(GPIO_PORTH_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PN3
	// for EPI0S30
    //
	MAP_GPIOPinConfigure(GPIO_PN3_EPI0S30);
	GPIOPinTypeEPI(GPIO_PORTN_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PB2
	// for EPI0S27
    //
	MAP_GPIOPinConfigure(GPIO_PB2_EPI0S27);
	GPIOPinTypeEPI(GPIO_PORTB_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PN4
	// for EPI0S34
    //
	MAP_GPIOPinConfigure(GPIO_PN4_EPI0S34);
	GPIOPinTypeEPI(GPIO_PORTN_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PQ1
	// for EPI0S21
    //
	MAP_GPIOPinConfigure(GPIO_PQ1_EPI0S21);
	GPIOPinTypeEPI(GPIO_PORTQ_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PL3
	// for EPI0S19
    //
	MAP_GPIOPinConfigure(GPIO_PL3_EPI0S19);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PL4
	// for EPI0S26
    //
	MAP_GPIOPinConfigure(GPIO_PL4_EPI0S26);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PL5
	// for EPI0S33
    //
	MAP_GPIOPinConfigure(GPIO_PL5_EPI0S33);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PM0
	// for EPI0S15
    //
	MAP_GPIOPinConfigure(GPIO_PM0_EPI0S15);
	GPIOPinTypeEPI(GPIO_PORTM_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PM2
	// for EPI0S13
    //
	MAP_GPIOPinConfigure(GPIO_PM2_EPI0S13);
	GPIOPinTypeEPI(GPIO_PORTM_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PB3
	// for EPI0S28
    //
	MAP_GPIOPinConfigure(GPIO_PB3_EPI0S28);
	GPIOPinTypeEPI(GPIO_PORTB_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PQ3
	// for EPI0S23
    //
	MAP_GPIOPinConfigure(GPIO_PQ3_EPI0S23);
	GPIOPinTypeEPI(GPIO_PORTQ_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PG1
	// for EPI0S10
    //
	MAP_GPIOPinConfigure(GPIO_PG1_EPI0S10);
	GPIOPinTypeEPI(GPIO_PORTG_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PN2
	// for EPI0S29
    //
	MAP_GPIOPinConfigure(GPIO_PN2_EPI0S29);
	GPIOPinTypeEPI(GPIO_PORTN_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PH3
	// for EPI0S3
    //
	MAP_GPIOPinConfigure(GPIO_PH3_EPI0S3);
	GPIOPinTypeEPI(GPIO_PORTH_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PC5
	// for EPI0S6
    //
	MAP_GPIOPinConfigure(GPIO_PC5_EPI0S6);
	GPIOPinTypeEPI(GPIO_PORTC_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PA7
	// for EPI0S9
    //
	MAP_GPIOPinConfigure(GPIO_PA7_EPI0S9);
	GPIOPinTypeEPI(GPIO_PORTA_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PH0
	// for EPI0S0
    //
	MAP_GPIOPinConfigure(GPIO_PH0_EPI0S0);
	GPIOPinTypeEPI(GPIO_PORTH_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PC4
	// for EPI0S7
    //
	MAP_GPIOPinConfigure(GPIO_PC4_EPI0S7);
	GPIOPinTypeEPI(GPIO_PORTC_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PL1
	// for EPI0S17
    //
	MAP_GPIOPinConfigure(GPIO_PL1_EPI0S17);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PM1
	// for EPI0S14
    //
	MAP_GPIOPinConfigure(GPIO_PM1_EPI0S14);
	GPIOPinTypeEPI(GPIO_PORTM_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PK7
	// for EPI0S24
    //
	MAP_GPIOPinConfigure(GPIO_PK7_EPI0S24);
	GPIOPinTypeEPI(GPIO_PORTK_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PC7
	// for EPI0S4
    //
	MAP_GPIOPinConfigure(GPIO_PC7_EPI0S4);
	GPIOPinTypeEPI(GPIO_PORTC_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PL2
	// for EPI0S18
    //
	MAP_GPIOPinConfigure(GPIO_PL2_EPI0S18);
	GPIOPinTypeEPI(GPIO_PORTL_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PM3
	// for EPI0S12
    //
	MAP_GPIOPinConfigure(GPIO_PM3_EPI0S12);
	GPIOPinTypeEPI(GPIO_PORTM_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PQ0
	// for EPI0S20
    //
	MAP_GPIOPinConfigure(GPIO_PQ0_EPI0S20);
	GPIOPinTypeEPI(GPIO_PORTQ_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PQ2
	// for EPI0S22
    //
	MAP_GPIOPinConfigure(GPIO_PQ2_EPI0S22);
	GPIOPinTypeEPI(GPIO_PORTQ_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PC6
	// for EPI0S5
    //
	MAP_GPIOPinConfigure(GPIO_PC6_EPI0S5);
	GPIOPinTypeEPI(GPIO_PORTC_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PH2
	// for EPI0S2
    //
	MAP_GPIOPinConfigure(GPIO_PH2_EPI0S2);
	GPIOPinTypeEPI(GPIO_PORTH_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PA3
	// for GPIO_PA3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PA5
	// for GPIO_PA5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PF1
	// for GPIO_PF1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PF2
	// for GPIO_PF2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PD6
	// for GPIO_PD6
    //
	// MAP_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PP4
	// for GPIO_PP4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PH7
	// for GPIO_PH7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PB7
	// for GPIO_PB7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PB6
	// for GPIO_PB6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PH4
	// for GPIO_PH4
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PK4
	// for GPIO_PK4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4);

	    //
    // Configure the GPIO Pin Mux for PK5
	// for GPIO_PK5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PQ7
	// for GPIO_PQ7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_7);
	MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PP7
	// for GPIO_PP7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_7);
	MAP_GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PP6
	// for GPIO_PP6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_6);
//	MAP_GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure the GPIO Pin Mux for PJ0
	// for GPIO_PJ0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PJ1
	// for GPIO_PJ1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_1);
	MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PJ2
	// for GPIO_PJ2
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_2);
	
    //
    // Configure the GPIO Pin Mux for PJ3
	// for GPIO_PJ3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_3);
	
    //
    // Configure the GPIO Pin Mux for PJ4
	// for GPIO_PJ4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_4);

	    //
    // Configure the GPIO Pin Mux for PJ5
	// for GPIO_PJ5
    //
#if 1 //FPGADOWNLOAD
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_5);
#else
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_5);
#endif

	    //
    // Configure the GPIO Pin Mux for PJ7
	// for GPIO_PJ7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_7);
	
    //
    // Configure the GPIO Pin Mux for PN0
	// for GPIO_PN0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PN1
	// for GPIO_PN1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_1);
	MAP_GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PA0
	// for GPIO_PA0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA1
	// for GPIO_PA1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PH5
	// for GPIO_PH5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PH6
	// for GPIO_PH6
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PR4
	// for GPIO_PR4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTR_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PR5
	// for GPIO_PR5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTR_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PK2
	// for GPIO_PK2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PK3
	// for GPIO_PK3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK0
	// for GPIO_PK0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PK1
	// for GPIO_PK1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PE0
	// for GPIO_PE0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PE1
	// for GPIO_PE1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PE2
	// for GPIO_PE2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
	
    //
    // Configure the GPIO Pin Mux for PP2
	// for GPIO_PP2
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PP5
	// for GPIO_PP5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PS0
	// for GPIO_PS0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTS_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PS1
	// for GPIO_PS1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTS_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PS2
	// for GPIO_PS2
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTS_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PS3
	// for GPIO_PS3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTS_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PE6
	// for GPIO_PE6
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_6);

	//
	// Unlock the Port Pin and Set the Commit Bit
	//
	HWREG(GPIO_PORTE_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTE_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
	HWREG(GPIO_PORTE_BASE+GPIO_O_LOCK) = 0x0;
	
    //
    // Configure the GPIO Pin Mux for PE7
	// for GPIO_PE7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PT0
	// for GPIO_PT0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTT_BASE, GPIO_PIN_0);
	MAP_GPIOPadConfigSet(GPIO_PORTT_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PT1
	// for GPIO_PT1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTT_BASE, GPIO_PIN_1);
	MAP_GPIOPadConfigSet(GPIO_PORTT_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PQ5
	// for GPIO_PQ5
    //
	// MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_5);
	// MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PG6
	// for GPIO_PG6
    //
	// MAP_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_6);
	// MAP_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PR6
	// for GPIO_PR6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTR_BASE, GPIO_PIN_6);
	MAP_GPIOPadConfigSet(GPIO_PORTR_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PF5
	// for GPIO_PF5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_5);
	MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PJ6
	// for GPIO_PJ6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_6);
	MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PQ4
	// for GPIO_PQ4
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_4);
	MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    //
    // Configure the GPIO Pin Mux for PP3
	// for GPIO_PP3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_3);
	MAP_GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

	//
	// Unlock the Port Pin and Set the Commit Bit
	//
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;
	
    //
    // Configure the GPIO Pin Mux for PD7
	// for NMI
    //
    // NMI 時に特別な処理は不要なので単純に Input にする。 
    // ■備考
    //   データシートの「NMI Pin」の説明に「The active sense of the NMI signal is High;」と記述されている。 
    //   立下りエッジで NMI 割り込みを発生させることはできない。 
	//MAP_GPIOPinConfigure(GPIO_PD7_NMI);
    //GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_DIR_MODE_HW);
    //GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_7);

    ////
    //// Configure the GPIO Pin Mux for PA4
	//// for SSI0XDAT0
    ////
	// MAP_GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
	// MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PA4
	// for GPIO_PA4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);


    ////
    //// Configure the GPIO Pin Mux for PA2
	//// for SSI0CLK
    ////
	// MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	// MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PA2
	// for GPIO_PA2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    ////
    //// Configure the GPIO Pin Mux for PE4
	//// for SSI1XDAT0
    ////
	// MAP_GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
	// MAP_GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PE4
	// for GPIO_PE4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

    ////
    //// Configure the GPIO Pin Mux for PE5
	//// for SSI1XDAT1
    ////
	// MAP_GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
	// MAP_GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PE5
	// for GPIO_PE5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);

    ////
    //// Configure the GPIO Pin Mux for PB4
	//// for SSI1FSS
    ////
	// MAP_GPIOPinConfigure(GPIO_PB4_SSI1FSS);
	// MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PB4
	// for GPIO_PB4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);

    ////
    //// Configure the GPIO Pin Mux for PB5
	//// for SSI1CLK
    ////
	// MAP_GPIOPinConfigure(GPIO_PB5_SSI1CLK);
	// MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PB5
	// for GPIO_PB5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PD0
	// for GPIO_PD0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
	
    ////
    //// Configure the GPIO Pin Mux for PD1
	//// for SSI2XDAT0
    ////
	// MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
	// MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PD1
	// for GPIO_PD1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);

    ////
    //// Configure the GPIO Pin Mux for PD2
	//// for SSI2FSS
    ////
	// MAP_GPIOPinConfigure(GPIO_PD2_SSI2FSS);
	// MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PD2
	// for GPIO_PD2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);

    ////
    //// Configure the GPIO Pin Mux for PD3
	//// for SSI2CLK
    ////
	// MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
	// MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PD3
	// for GPIO_PD3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);

    ////
    //// Configure the GPIO Pin Mux for PF0
	//// for SSI3XDAT1
    ////
	// MAP_GPIOPinConfigure(GPIO_PF0_SSI3XDAT1);
	// MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PF0
	// for GPIO_PF0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    ////
    //// Configure the GPIO Pin Mux for PF3
	//// for SSI3CLK
    ////
	// MAP_GPIOPinConfigure(GPIO_PF3_SSI3CLK);
	// MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PF3
	// for GPIO_PF3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PD3
	// for GPIO_PD3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PD4
	// for U2RX
    //
	MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
	MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PD5
	// for U2TX
    //
	MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
	MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PP0
	// for U6RX
    //
	MAP_GPIOPinConfigure(GPIO_PP0_U6RX);
	MAP_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PP1
	// for U6TX
    //
	MAP_GPIOPinConfigure(GPIO_PP1_U6TX);
	MAP_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PL6
	// for USB0DP
    //
	MAP_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PL7
	// for USB0DM
    //
	MAP_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PB1
	// for USB0VBUS
    //
	MAP_GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_1);
	
    //
    // Configure the GPIO Pin Mux for PG2
	// for GPIO_PG2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
	
    //
    // Configure the GPIO Pin Mux for PG4
	// for GPIO_PG4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_4);
	
    //
    // Configure the GPIO Pin Mux for PG5
	// for GPIO_PG5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_5);
	
    //
    // Configure the GPIO Pin Mux for PG7
	// for GPIO_PG7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_7);

  	//
    // Configure the GPIO Pin Mux for PG3
	// for GPIO_PG3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_3);

  	//
    // Configure the GPIO Pin Mux for PM4
	// for GPIO_PM4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);

 	//
    // Configure the GPIO Pin Mux for PM7
	// for GPIO_PM7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);
	
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

