/***********************************************/
/* File Name : defSAVE.h		         									   */
/*	Summary   : 保存データ定義		                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

//----------------------------------------------------------------------
//インクルードガード
//----------------------------------------------------------------------
#ifndef DEFSAVE_H
#define DEFSAVE_H

#include "define.h"

struct stSAVE{	//保存データ

	unsigned short control;		//コントロール
	unsigned short control_old;	//今回値保存
	unsigned short sum_abs_com[SUB_POINT];	/*絶対値の和、格納領域*/
	short log_save_num;			//エラーログ登録番号

};

extern struct stSAVE SAVE[CH_NUMMAX];//保存データ

#endif
