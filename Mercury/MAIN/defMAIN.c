/***********************************************/
/* File Name : defMAIN.c		         									   */
/*	Summary   : MAIN定義 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/
#include "defMAIN.h"

struct stMAIN MAIN[CH_NUMMAX];

short set_mode = 0;	//1=set mode
short	reset_factor;	//リセット要因
short	cunet_error;	//CUnetエラー要因
short	send_err_status;//メール送信エラーステータス

unsigned short	cunet_mcare;	//CUnet MCARE/LCARE情報
unsigned short	cunet_mfr1;	//CUnet MFR情報1
unsigned short	cunet_mfr2;	//CUnet MFR情報2
unsigned short	cunet_mfr3;	//CUnet MFR情報3
unsigned short	cunet_mfr4;	//CUnet MFR情報4

ERROR_INFO err_inf[];	

ERROR_INFO err_inf[] = {
	{							//0.None
		0,						//
		0,						//
		0						//
	},
	{							//1.ゼロ調整エンプティセンサ
		ERR_ZERO_EMPTY,			//エラーコード
		1,						//優先順位
		DSP_ALM1 | DSP_ALM3		//点灯LED
	},
	{							//2.ゼロ調整波形減衰 （波形異常）
		ERR_ZERO_WAVE,			//エラーコード
		2,						//優先順位
		DSP_ALM3 | DSP_ALM4		//点灯LED
	},
	{							//3.ゼロ調整演算異常
		ERR_ZERO_CALC,			//エラーコード
		3,						//優先順位
		DSP_ALM2 | DSP_ALM3		//点灯LED
	},
	{							//4.ゼロ調整波形アンバランス
		ERR_ZERO_LEVEL,			//エラーコード
		4,						//優先順位
		DSP_ALM2 | DSP_ALM3		//点灯LED
	},
	{							//5.ゼロ調整AGC不能//
		ERR_ZERO_AGC,			//エラーコード
		5,						//優先順位
		DSP_ALM3 | DSP_ALM4		//点灯LED
	},
	{							//6.測定エンプティセンサL
		ERR_MESR_EMPTY_L,		//エラーコード
		12,						//優先順位
		DSP_ALM1				//点灯LED
	},
	{							//7.測定エンプティセンサH
		ERR_MESR_EMPTY_H,		//エラーコード
		7,						//優先順位
		DSP_ALM1				//点灯LED
	},
	{							//8.測定波形減衰L
		ERR_MESR_WAVE_L,		//エラーコード
		13,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//9.測定波形減衰H
		ERR_MESR_WAVE_H,		//エラーコード
		8,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//10.測定演算異常L
		ERR_MESR_CALC_L,		//エラーコード
		14,						//優先順位
		DSP_ALM2				//点灯LED
	},
	{							//11.測定演算異常H
		ERR_MESR_CALC_H,		//エラーコード
		9,						//優先順位
		DSP_ALM2				//点灯LED
	},
	{							//12.測定波形アンバランスL
		ERR_MESR_LEVEL_L,		//エラーコード
		15,						//優先順位
		DSP_ALM2				//点灯LED
	},
	{							//13.測定波形アンバランスH
		ERR_MESR_LEVEL_H,		//エラーコード
		10,						//優先順位
		DSP_ALM2				//点灯LED
	},
	{							//14.測定AGC不能L
		ERR_MESR_AGC_L,			//エラーコード
		16,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//15.測定AGC不能H
		ERR_MESR_AGC_H,			//エラーコード
		11,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//16.測定逆流異常
		ERR_MESR_REVERSE,		//エラーコード
		19,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//17.Reserve
		0,						//
		0,						//
		0						//
	},
	{							//18.Reserve
		0,						//
		0,						//
		0						//
	},
	{							//19.10点データ無効
		ERR_10POINT,			//エラーコード
		21,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//20.EEPROMエラー
		ERR_EEPROM,				//エラーコード
		25,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//21.Reserve
		0,						//
		0,						//
		0						//
	},
	{							//22.再起動
		ERR_RESTART,			//エラーコード
		27,						//優先順位
		0						//点灯LED
	},
	{							//23.測定オーバーフロー
		ERR_MESR_OVERFLOW,		//エラーコード
		20,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//24.CUnetエラー
		ERR_CUNET,				//エラーコード
		26,						//優先順位
		DSP_ALM4				//点灯LED
	},
	{							//25.ゼロ調整計測時間発散
		ERR_ZERO_UNSTABLE,		//エラーコード
		6,						//優先順位
		DSP_ALM3 | DSP_ALM4		//点灯LED
	},
	{							//26.測定アンプゲイン急変（警告）
		ALM_MESR_GAIN,		//エラーコード
		17,						//優先順位
		0							//点灯LED
	},
	{							//27.測定エンプティセンサ（警告）
		ALM_MESR_EMPTY,		//エラーコード
		18,						//優先順位
		0							//点灯LED
	},
	{							//28.積算値演算異常
		TTL_CACL_ERR,		//エラーコード
		22,						//優先順位
		DSP_ALM4			//点灯LED
	},
	{							//29.積算値オーバーフロー
		TTL_OVERFLOW,		//エラーコード
		23,						//優先順位
		DSP_ALM4			//点灯LED
	},
};

