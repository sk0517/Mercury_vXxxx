/***********************************************/
/* File Name : prtocol.c		         									   */
/*	Summary   : 通信プロトコル処理                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "machine.h"
#include "define.h"
#include "SV_def.h"
#include "defLOG.h"
#include "defSAVE.h"
#include "defMES.h"
#include "defMAIN.h"
#include "version.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void read_change(short com_mode,long long read_value,char point,short pos);
void value_judge(short com_mode,short si,short pow);
void letter_judge(short com_mode,short sl,short sm);
void check_channel(short com_mode);
void check_command(short com_mode);
void check_format(short com_mode);
void check_cu_length(short com_mode);
void check_wdt(short com_mode);
void check_act_disable(short com_mode);
void check_act_able(short com_mode);
void protocol(short com_mode);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern void TX_start_host(short);
extern void TX_start_ment(short);
extern void mky43_TX_start(short com_mode);
extern void eep_write_ch(short,short,short);
extern void make_viscos_tbl(short);
extern void command_OV(short);
extern void command_ov(short);
extern void command_OE(short);
extern void command_OQ(short);
extern void command_oq(short);
extern void read_command_R(short);
extern void read_command_W(short);
extern void command_OS(short);
extern void command_Os(short);
extern void command_OJ(short);
extern void command_OL(short);
extern void command_OR(void);
extern void command_OT(void);
extern void command_OA(void);
extern void command_Ot(short);
extern void command_OZ(short);
extern short	action_status_check(short pch);
extern void download(short mode, short type);
#if defined(FRQSCH)
extern void command_OF(short);
#endif
extern void command_Dl(short);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short i_num[MSG_NUM];			// 数列番号用
short ch_no[MSG_NUM];			// チャネル番号用
short size[MSG_NUM];				// 入力値データ長判定用
short digit_check[MSG_NUM];		// 整数or小数点桁数 判定用
short end_code[MSG_NUM];		// エンドコード処理用
short flow_check[MSG_NUM];	// OE/OQ文字数判定用
long long value[MSG_NUM];		// 入力値判定用
char RX_buf[MSG_NUM][MSG_MAX];
char TX_buf[MSG_NUM][MSG_MAX];
short	multi_ch_no[MSG_NUM][CH_NUMMAX];	// マルチコマンドチャネル番号
short ch_z[MSG_NUM];				// チャネル数用
short error_check[MSG_NUM];	// エラーチェック用
short echo_back[MSG_NUM];	// エコーバック用
short l_check[MSG_NUM];		// 使用文字種類判定用
short eep_err;
short mes_err[7];

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short	err_stat;
extern short	com_type;
extern short	end_wdt;
extern short	len_err_host;
extern short	len_err_sub;
extern short	format_err_host;
extern short	format_err_sub;

#ifdef MEMDBG
void value_judge_Hex(short, short, short); //Debug
extern void MemoryCom(int); //Debug
#endif

/************************************************/
/*		読込値変換用関数		*/
/************************************************/
void read_change(short com_mode,long long read_value,char point,short pos){

	short	z_add;
	long long	read;
	short	level;
	char	add_read[20] = {0};
	char	add_cal[20] = {0};
	char	add_out[20] = {0};
	long long	pow_1;
	unsigned long	long pow_2;
	unsigned short	l_add;
	
	//max11桁
	
	if ( read_value < 0 )	read = (-1)*read_value;
	else			read = read_value;

	if 	(read >= 0 && read < 10)							level = 1;
	else if	(read >= 10 && read < 100)					level = 2;
	else if	(read >= 100 && read < 1000)				level = 3;
	else if	(read >= 1000 && read < 10000)				level = 4;
	else if	(read >= 10000 && read < 100000)			level = 5;
	else if	(read >= 100000 && read < 1000000)			level = 6;
	else if	(read >= 1000000 && read < 10000000)		level = 7;
	else if	(read >= 10000000 && read < 100000000)		level = 8;
	else if	(read >= 100000000 && read < 1000000000)	level = 9;
	else if	(read >= 1000000000 && read < 10000000000)	level = 10;
	else if	(read >= 10000000000 && read < 100000000000)	level = 11;
	else 							level = 0;

	pow_1 = 10;
	pow_2 = 1;

	for (z_add = 0; z_add < level; z_add++){
		add_read[z_add] = ( read % pow_1) / pow_2 + 0x30;	// 0x30:ASCII→数字変換
		pow_1 *= 10;
		pow_2 *= 10;
	}
	switch(point){
		case 0://読込値そのまま
			for (z_add = 0; z_add < level; z_add++){
				add_cal[z_add] = add_read[z_add];
			}
			break;	
		case 1://読込値を1/10
			if (level == 1){				
				add_cal[0] = add_read[0];
				add_cal[1] = 0x2E; // ="."
				add_cal[2] = 0x30; // ="0"
			}
			else{				add_cal[0] = add_read[0];
				add_cal[1] = 0x2E; // ="."
				for (z_add = 1; z_add < level; z_add++){
					add_cal[z_add +1] = add_read[z_add];
				}
			}
			break;
		case 2://読込値を1/100
			if (level == 1){				
				add_cal[0] = add_read[0];
				add_cal[1] = 0x30; // ="0"				
				add_cal[2] = 0x2E; // ="."
				add_cal[3] = 0x30; // ="0"
			}
			else	if (level == 2){				
				add_cal[0] = add_read[0];
				add_cal[1] = add_read[1];
				add_cal[2] = 0x2E; // ="."
				add_cal[3] = 0x30; // ="0"
			}
			else{				add_cal[0] = add_read[0];
				add_cal[1] = add_read[1];
				add_cal[2] = 0x2E; // ="."
				for (z_add = 2; z_add < level; z_add++){
					add_cal[z_add +1] = add_read[z_add];
				}
			}
			break;
		case 3://読込値を1/1000
			if (level == 1){				
				add_cal[0] = add_read[0];
				add_cal[1] = 0x30; // ="0"
				add_cal[2] = 0x30; // ="0"
				add_cal[3] = 0x2E; // ="."
				add_cal[4] = 0x30; // ="0"
			}
			else	if (level == 2){				
				add_cal[0] = add_read[0];
				add_cal[1] = add_read[1];
				add_cal[2] = 0x30; // ="0"
				add_cal[3] = 0x2E; // ="."
				add_cal[4] = 0x30; // ="0"
			}
			else	if (level == 3){				
				add_cal[0] = add_read[0];
				add_cal[1] = add_read[1];
				add_cal[2] = add_read[2];
				add_cal[3] = 0x2E; // ="."
				add_cal[4] = 0x30; // ="0"
			}
			else{				add_cal[0] = add_read[0];
				add_cal[1] = add_read[1];
				add_cal[2] = add_read[2];
				add_cal[3] = 0x2E; // ="."
				for (z_add = 3; z_add < level; z_add++){
					add_cal[z_add +1] = add_read[z_add];
				}
			}
			break;
		default:
			break;
	}
	l_add = strlen(add_cal);

	if ( read_value < 0 )	add_cal[l_add] = 0x2D; // ="-"	
	
	if(pos == ADD_CMM_BFR_VAL){					//データの前に","を付加
		l_add = strlen(add_cal);
		add_cal[l_add] = 0x2C;		//","
	}

	l_add = strlen(add_cal);
	
	for (z_add = 0; z_add < l_add; z_add++){
	
		add_out[z_add] = add_cal[l_add - z_add -1];
	
	}

	if(pos == ADD_CMM_AFR_VAL){					//データの後に","を付加
		l_add = strlen(add_out);
		add_out[l_add] = 0x2C;		//","
	}

	strncat(TX_buf[com_mode],add_out,sizeof(add_out));
	
}

/************************************************/
/*		入力値判定用関数						*/
/************************************************/
void value_judge(short com_mode,short si,short pow){
	
	short j;
	short h;
	short z;
	long long value_z;
	short value_f;
	char input[MES_CHAR_MAX] ={0};			// 入力値判定用
	char input_z[MES_CHAR_MAX] ={0};			// 入力値判定用(整数部)
	char input_f[MES_CHAR_MAX] ={0};			// 入力値判定用(小数部)

	/* 入力値判定部 */
	z = 0;
	size[com_mode] = 0;
	while (RX_buf[com_mode][si] != ','){
		if(si >= (MSG_MAX-1)){
			size[com_mode] = -1;
			goto FUNCEND;
		}
		input[z] = RX_buf[com_mode][si];
		size[com_mode]++;
		si++;
		z++;
		if(z >= MES_CHAR_MAX){	//配列外書込み防止
			break;
		}
	}

	/* 小数点以下桁数判定部 */
	z = 0;
	for (j=0; j<size[com_mode]; j++){
		z++;
		if (input[j] == '.') break;	//小数点位置判定
	}
	digit_check[com_mode] = size[com_mode] - z;			//小数点以下桁数判定用

	/* 数値格納 */
	if(digit_check[com_mode] == 0){		//整数の場合
		value_z = atoi (input);			//入力数値格納
		for (j=0; j<pow; j++){
			value_z *= 10;
		}
		value[com_mode] = value_z;
	}
	else{					//小数ありの場合
		for (j=0; j<z-1; j++){
			input_z[j]=input[j];
			if(j >= MES_CHAR_MAX){	//配列外書込み防止
				break;
			}
		}		
		value_z = atoi (input_z);
		for (j=0; j<pow; j++){
			value_z *= 10;
		}
				
		for (j=0; j<digit_check[com_mode]; j++){
			input_f[j]=input[z+j];
			if(j >= MES_CHAR_MAX){	//配列外書込み防止
				break;
			}
		}
		value_f = atoi (input_f);
		for (j=0; j<pow-digit_check[com_mode]; j++){
			value_f *= 10;
		}	
		
		if(input[0] == '-'){
			if(input[1] == '0' || input[1] == '.'){
				value[com_mode] = (-1) * (value_z + value_f);
			}
			else{
				value[com_mode] = value_z - value_f;			
			}
		}
		else{
			value[com_mode] = value_z + value_f;
		}
	}

FUNCEND:
	i_num[com_mode] = si;
	for (h = 0; h < MES_CHAR_MAX; h++){input[h] = 0;}	//判定用配列 初期化
	
}

#ifdef MEMDBG
//Debug
/*******************************************
 * Function : value_judge_Hex
 * Summary  : 16進数データ取得&判定
 * Argument : int com_mode -> 0 : ホスト
 *                            1 : メンテ
 *            int si -> データサイズ
 *            int pow -> 小数点以下桁数
 * Return   : void
 * Caution  : なし
 * Note     : value_judge() は10進数しか対応していなかったため追加
 *            実行後、 value[] にデータが、 size[] にデータサイズが格納される
 *            最大8文字を4byteに変換する("FFFFFFFF" -> 0xFFFFFFFF)
 *            受信文字列には末尾に ",(FCS)" がついているので、値の終了コードは','
 * *****************************************/
void value_judge_Hex(short com_mode, short sPos, short pow)
{	
	short j;
	char input[20] ={0};			// 入力値判定用
	short Pos = sPos;
	unsigned long Val = 0;
	
	//入力文字列取得
	size[com_mode] = 0;
	while (RX_buf[com_mode][Pos] != ',')
	{
		if(Pos >= (MSG_MAX - 1))
		{
			size[com_mode] = -1;
			goto FUNCEND;
		}
		input[Pos - sPos] = RX_buf[com_mode][Pos];
		Pos++;
	}
	size[com_mode] = Pos - sPos;

	//小数点位置取得
	for (j = 0; j < size[com_mode]; j++)
	{
		if (input[j] == '.') break;	//小数点位置判定
	}
	digit_check[com_mode] = size[com_mode] - j;			//小数点以下桁数判定用

	/* 数値格納 */
	sscanf(input, "%x", &Val);
	value[com_mode] = (long long)Val;

FUNCEND:
	i_num[com_mode] = sPos + size[com_mode];
	memset(input, 0, sizeof(input));//判定用配列 初期化
}
#endif

/************************************************/
/*		入力文字判定用関数						*/
/************************************************/
void letter_judge(short com_mode,short sl,short sm){
	
	short l;
	short z;
	char letter;

	l_check[com_mode] = 0;
	
	if (sm == 0){	//通常処理・数値+','のみ
		for (z = 6; z < sl; z++){
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//上記以外
		}
	}
	if (sm == 1){	//W5専用・シリアルナンバー(A-Z許容)対応
		i_num[com_mode] = 7;
		for (l=0; l<32; l++){ value_judge(com_mode,i_num[com_mode],0); i_num[com_mode]++;}
		for (z = 6; z < i_num[com_mode]; z++){	//各パラメータ
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//上記以外
		}
		for (z =  i_num[com_mode]; z < sl; z++){	//シリアルナンバー
			letter = RX_buf[com_mode][z];
			if (	(letter == 0x2C)			||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x5A)	){}	// 'A-Z'				
			else { l_check[com_mode]++; }					//上記以外
		}
	}
	if (sm == 2){	//W1専用・シリアルナンバー(A-Z許容)対応
		i_num[com_mode] = 7;
		for (l=0; l<5; l++){ value_judge(com_mode,i_num[com_mode],0); i_num[com_mode]++;}
		for (z = 6; z < i_num[com_mode]; z++){	//リニアライズ数値
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//上記以外
		}
		for (z =  i_num[com_mode]; z < sl; z++){	//シリアルナンバー
			letter = RX_buf[com_mode][z];
			if (	(letter == 0x2C)			||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x5A)	){}	// 'A-Z'				
			else { l_check[com_mode]++; }					//上記以外
		}
	}	
	if (sm == 3){	//W7専用・ステータス(A-F許容)対応
		for (z = 6; z < sl; z++){
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x47)	){}	// 'A-F'				
			else { l_check[com_mode]++; }					//上記以外
		}
	}
	if (sm == 4){	//Wp専用・シリアルナンバー(A-Z許容)対応
		i_num[com_mode] = 7;
		for (z = 6; z < i_num[com_mode]; z++){	//各パラメータ
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//上記以外
		}
		for (z =  i_num[com_mode]; z < sl; z++){	//シリアルナンバー
			letter = RX_buf[com_mode][z];
			if (	(letter == 0x2C)			||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x5A)	){}	// 'A-Z'				
			else { l_check[com_mode]++; }					//上記以外
		}
	}
}
/************************************************/
/*    チャネル番号読出し・チャネル判定用関数    */
/************************************************/
void check_channel(short com_mode){
	
	short m;
	short check;
	char ch[5] ={0};

	ch_z[com_mode] = multi_ch_no[com_mode][0] = multi_ch_no[com_mode][1] = multi_ch_no[com_mode][2] = multi_ch_no[com_mode][3] = multi_ch_no[com_mode][4] = multi_ch_no[com_mode][5] = 0;
	switch (RX_buf[com_mode][4]){
	case '0':					//シングル用
		switch (RX_buf[com_mode][5]){
		case '1':	ch_no[com_mode] = 1;	break;
		case '2':	ch_no[com_mode] = 2;	break;
		case '3':	ch_no[com_mode] = 3;	break;
		case '4':	ch_no[com_mode] = 4;	break;
		case '5':	ch_no[com_mode] = 5;	break;
		case '6':	ch_no[com_mode] = 6;	break;
		default :	ch_no[com_mode] = 9;	break;	//CH番号指定エラー(シングル,末尾1-6以外)
		}
		break;
	case 'X':					//マルチ用
		switch (RX_buf[com_mode][5]){
		case 'X':	ch_no[com_mode] = 0;
			switch (RX_buf[com_mode][6]){
			case '0':
			/* ｃｈ数判定 */
				ch[0] = RX_buf[com_mode][7];	//マルチCH数判定
				ch_z[com_mode] = ch[0] - 0x30;
				ch[0] = 0;
				if (ch_z[com_mode] >=1 && ch_z[com_mode] <=6){
					for (m = 0; m < ch_z[com_mode]; m++){
						ch[0] = RX_buf[com_mode][8 + 2*m];	//マルチCH番号判定(先頭)
						check = ch[0] - 0x30;
						ch[0] = 0;
						if (check != 0){ch_no[com_mode] = 9;}		//マルチCH番号指定エラー(先頭0以外)
						
						ch[0] = RX_buf[com_mode][9 + 2*m];	//マルチCH番号判定(末尾)
						check = ch[0] - 0x30;
						ch[0] = 0;
						if (check < 1 || check > 6){	//異常時
							ch_no[com_mode] = 9;				//マルチCH番号指定エラー(末尾1-6以外)
						}else{												//正常時
							multi_ch_no[com_mode][m] = check;			//マルチCH番号を保持(1～6)
						}
					}
					check = 0;
				}
				else {ch_no[com_mode] = 9;}					//マルチCH数指定エラー(末尾1-6以外)
				break;
			case ',':							//ダウンロードコマンド用(シングルだが CHNo.=XX)
				if (RX_buf[com_mode][1] == 'd'){ch_no[com_mode] = 0;}
				else {ch_no[com_mode] = 9;}
				break;
			default:	ch_no[com_mode] = 9;	break;				//マルチCH数指定エラー(先頭0以外)
			}
			break;
		default:	ch_no[com_mode] = 9;	break;					//CH番号指定エラー(マルチ,末尾X以外)
		}
		break;
	default:		ch_no[com_mode] = 9;	break;					//CH番号指定エラー(先頭0,X以外)
	}
}
/************* コマンド・バンク・チャネル判定用関数 *************/
/*                                                              */
/*     未定義のコマンドが入力された場合、End Code:16 を返す     */
/*        バンクが指定値と異なる場合、End Code:04 を返す        */
/*     存在しないチャネルを指定した場合、End Code:04 を返す     */
/*                                                              */
/*                  16:優先度7  04：優先度8                     */
/*                                                              */
/****************************************************************/
void check_command(short com_mode){
	
	if( end_code[com_mode] != FRAMING_ERROR &&		//End Code 11
		end_code[com_mode] != PARITY_ERROR &&		//End Code 10
		end_code[com_mode] != OVERRUN_ERROR &&		//End Code 12
		end_code[com_mode] != CU_LENGTH_ERROR &&	//End Code 18
		end_code[com_mode] != FCS_ERROR)		//End Code 13
	{											//End Code 11,10,12,18,13以外
		
		switch (RX_buf[com_mode][1]){		//ヘッダーコード先頭
		case 'R': /**** 読出し系 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ バンク判定 = 0 */
			case 'g': // 口径読出し(Rg)
			case 'r': // フルスケール読出し(Rr)
			case 'k': // Ｋファクタ読出し(Rk)
			case 'd': // ダンピング読出し(Rd)
			case 'b': // バーンアウト読出し(Rb)
			case 'v': // 動粘度係数読出し(Rv)
			case 'h': // エラーホールドタイム読出し(Rh)
			case 'u': // ユーザーリニアライズ読出し(Ru)
			case 'R': // 逆流判定値読出し(RR)
			case 'F': // バージョン値読出し(SFC9000対応)(RF)
			case 'V': // バージョン値読出し(RV)
			case 'G': // ログデータ読出し(SFC9000対応)(RG)
			case 'L': // ログデータ読出し(RL)
			case 'y': // 薬液リニアライズモード読出し(Ry)
			case 'M': // 音速読出し(RM)
			case 'f': // フィルタ設定読出し(Rf)
			case 't': // 積算目標値読出し(Rt)
			case 'T': // 積算オフセット値読出し(RT)
			case 'p': // センサパルス設定読出し(SFC014E互換)(Rp)
			case 'n': // センサシリアルナンバー読出し(SFC014E互換)(Rn)
			case 'a': // センサ情報書込み(評価用)(Ra)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* シングルコマンドのみ バンク判定 = 1 */
			case 'm': // メーカーリニアライズ読出し(Rm)
			case '1': // メーカー設定読出し(R1)
			case '2': // デバックモード設定読出し(R2)
			case '3': // 測定データ読出し(R3)
			case '4': // 測定波形読出し(R4)
			case '5': // メーカー設定読出し(SFC9000対応)(R5)
			case '6': // デバックモード設定読出し(SFC9000対応)(R6)
			case '7': // 測定データ読出し(SFC9000対応)(R7)
			case '8': // 測定波形読出し(SFC9000対応)(R8)
			case '9': // パルス出力(R9)
			case 'X': // SW状態チェック(RX)
			case 'E': // 波形異常判定値設定読出し(RE)
			case 'A': // アッテネータゲイン読出し(RA)
			case 'i': // 検査モードデータ読出し(Ri)
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* マルチコマンドのみ */
			case 'D': // ゼロ調整データ読出し(SFC9000対応)(RD)
			case 'Z': // ゼロ調整データ読出し(RZ)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* シングル・マルチ対応  バンク判定 = 0 */
			case 'S': // ステータス読出し(SFC9000対応)(RS)
			case 's': // ステータス読出し(Rs)
			case 'l': // ローカット読出し(Rl)
			case 'z': // ゼロ点調整状態読出し(Rz)
			case 'W': // 波形状態読出し(RW)
			if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
			case 'C': // 測定データ読出し(SFC9000対応・R7コマンド短縮版)(RC)
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;			
			case 'c': // デジタルフィルタ係数読み出し
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;		
		/* 未定義コマンド */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}	
			break;
		case 'W': /**** 書込み系 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ バンク判定 = 0 */
			case 'g': // 口径書込み(Wg)
			case 'r': // フルスケール書込み(Wr)
			case 'k': // Ｋファクタ書込み(Wk)
			case 'd': // ダンピング書込み(Wd)
			case 'l': // ローカット書込み(Wl)
			case 'b': // バーンアウト書込み(Wb)
			case 'v': // 動粘度係数書込み(Wv)
			case 'h': // エラーホールドタイム書込み(Wh)
			case 'R': // 逆流判定値書込み(WR)
			case 'u': // ユーザーリニアライズ書込み(Wu)
			case 'U': // ユーザーリニアライズ切替え(WU)
			case 'y': // 薬液リニアライズモード書込み(Wy)	
			case 'f': // フィルタ設定書込み(Wf)
			case 't': // 積算目標値書込み(Wt)
			case 'T': // 積算オフセット値読出し(WT)
			case 'p': // センサパルス設定書込み(SFC014E互換)(Wp)
			case 'n': // センサシリアルナンバー書込み(SFC014E互換)(Wn)
			case 'a': // センサ情報書込み(評価用)(Wa)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* シングルコマンドのみ バンク判定 = 1 */
			case 'm': // メーカーリニアライズ書込み(Wm)
			case '1': // メーカー設定書込み(W1)
			case '2': // デバックモード設定書込み(W2)
			case '5': // メーカー設定書込み(SFC9000対応)(W5)
			case '6': // デバックモード設定書込み(SFC9000対応)(W6)
			case '7': // ゼロ調整詳細データ書込み(W7)
			case '9': // パルス出力(W9)
			case 'E': // 波形異常判定値設定書込み(WE)
			case 'A': // アッテネータゲイン書込み(WA)
			case 'i': // 検査モード書込み(Wi)
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* マルチコマンドのみ */
			case 'P': // パラメータセット書込み(WP)
			case 'Z': // ゼロ調整データ書込み(WZ)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* シングル・マルチ対応 */
			case 'z': // ゼロ調整データ書込み(SFC9000対応)(Wz)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
			case 'c': // デジタルフィルタ係数書き込み
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;		
		/* 未定義コマンド */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'O': /**** 主に動作指示 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ */
			case 'V': // 積算値読出し(OV)
			case 'Z': // ゼロ調整実行(OZ)
			case 'S': // 設定値保存(OS)
			case 's': // センサメモリ設定値保存(Os)
			case 'J': // ゼロ調整値保存(OJ)
			case 'C': // アラームリセット(OC)
			case 'L': // ログクリア(OL)
			case 'R': // ＲＡＭクリア(OR)
			case 'T': // ＬＥＤ全点灯(OT)
			case 'A': // アラーム強制出力(OA)
			case 't': // 積算リセット(Ot)
#if defined(FRQSCH)
			case 'F': // 初期ゼロ調実施(OF)
#endif
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* マルチコマンドのみ */
			case 'E': // 瞬時流量1点読出し(OE)
			case 'Q': // 瞬時流量10点読出し(OQ)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* 未定義コマンド */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'o': /**** oq,ovコマンド用 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ */
			case 'v': // 積算値読出し・積算中常時更新(ov)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* マルチコマンドのみ */
			case 'q': // 瞬時流量10点読出し・ローカットオフ(oq)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* 未定義コマンド */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'd': /**** ダウンロード系 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ ※CHNo.はXX */
			case 'p': // ダウンロード準備(dp)
			case 't': // ダウンロードデータ転送(dt)
			case 'w': // ダウンロードデータ書込み(dw)
			case 'c': // ダウンロード中断(dc)
			case 'r': // ダウンロード後リセット(dr)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* 未定義コマンド */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'D':
			switch(RX_buf[com_mode][2]) {
			case 'L':
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
			case 'l': //FpgaDownload bankno=1
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
					break;
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'E': /**** 再起動 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ */
			case 'R': // リセットコマンド(ER)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* 未定義コマンド */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
#ifdef MEMDBG
        case 'M':
            error_check[com_mode]++;
            error_check[com_mode]--;
            break;
#endif
		default:  /**** 未定義コマンド ****/
			error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
		}
	}	//コマンド・バンク・チャネル判定終了
	
}
/*********************** フォーマット判定 ***********************/
/*                                                              */
/* コマンドの書式に誤りがある場合、End Code:14 を返す (優先度9) */  // 書込み系はコマンド解析で判定（マルチCH個数判定以外）
/*                                                              */
/****************************************************************/
void check_format(short com_mode){
	
	short l;
	char letter;

	if( end_code[com_mode] != FRAMING_ERROR &&			//End Code 11
		end_code[com_mode] != PARITY_ERROR &&				//End Code 10
		end_code[com_mode] != OVERRUN_ERROR &&			//End Code 12
		end_code[com_mode] != CU_LENGTH_ERROR &&	//End Code 18
		end_code[com_mode] != FCS_ERROR &&					//End Code 13
		end_code[com_mode] != COMMAND_ERROR &&			//End Code 16
		end_code[com_mode] != ADDRESS_OVER)					//End Code 04
	{ 																	//End Code 11,10,12,18,13,16,04以外の場合
	/* CUnetメッセージ判定 */
		if (com_mode == HOST){
			if(format_err_host > 0){
				error_check[com_mode]++;
				end_code[com_mode] = FORMAT_ERROR;
				return;
			}
		}
		else{
			if(format_err_sub > 0){
				error_check[com_mode]++;
				end_code[com_mode] = FORMAT_ERROR;
				return;
			}
		}
	
		l = strlen(RX_buf[com_mode]);			//RX_bufデータ長 ※(FCS,CR削除済)

	/* 先頭文字判定(@) */
		letter = RX_buf[com_mode][0];
		if (letter != 0x40){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}		
	/* ヘッダーコード判定 */
		switch (RX_buf[com_mode][1]){			//ヘッダーコード先頭
#ifdef MEMDBG
		//Debug 
		case 'M':
		    switch(RX_buf[com_mode][2])
		    {
		        case 'R':
                    if(RX_buf[com_mode][l - 1] != ','){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
		            if (l != 19){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
		            break;
		        case 'W':
		            if(RX_buf[com_mode][l - 1] != ','){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
		            //if ((l < 22) || (28 < l)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
		            if((RX_buf[com_mode][16] == '0') && (RX_buf[com_mode][17] == '1') && (l != 22)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
                    if((RX_buf[com_mode][16] == '0') && (RX_buf[com_mode][17] == '2') && (l != 24)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
                    if((RX_buf[com_mode][16] == '0') && (RX_buf[com_mode][17] == '3') && (l != 26)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
                    if((RX_buf[com_mode][16] == '0') && (RX_buf[com_mode][17] == '4') && (l != 28)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
		            break;
		        default:
		            break;
		    }
		    break;
#endif
		case 'R': /**** 読出し系 ****/
			switch (RX_buf[com_mode][2]){		//ヘッダーコード末尾
		/* シングルコマンドのみ データ長:6 */
			case 'g': // 口径読出し(Rg)
			case 'r': // フルスケール読出し(Rr)
			case 'k': // Ｋファクタ読出し(Rk)
			case 'd': // ダンピング読出し(Rd)
			case 'b': // バーンアウト読出し(Rb)
			case 'v': // 動粘度係数読出し(Rv)
			case 'h': // エラーホールドタイム読出し(Rh)
			case 'u': // ユーザーリニアライズ読出し(Ru)
			case 'm': // メーカーリニアライズ読出し(Rm)
			case 'R': // 逆流判定値読出し(RR)
			case 'F': // バージョン値読出し(RF)(SFC9000対応)
			case 'V': // バージョン値読出し(RV)
			case 'E': // 波形異常判定値設定読出し(RE)
			case 'A': // アッテネータゲイン読出し(RA)
			case 'y': // 薬液リニアライズモード読出し(Ry)
			case 'M': // 音速読出し(RM)
			case 'f': // フィルタ設定読出し(Rf)
			case 't': // 積算目標値読出し(Rt)
			case 'T': // 積算オフセット値読出し(RT)
			case '1': // メーカー設定読出し(R1)
			case '2': // デバックモード設定読出し(R2)
			case '5': // メーカー設定読出し(SFC9000対応)(R5)
			case '6': // デバックモード設定読出し(SFC9000対応)(R6)
			case '9': // パルス出力(R9)
			case 'i': // 検査モードデータ読出し(Ri)
			case 'X': // SW状態チェック(RX)
			case 'p': // センサパルス読出し(SFC014E互換)(Rp)
			case 'n': // シリアルナンバー読出し(SFC014E互換)(Rn)
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングルコマンドのみ データ長:8 */
			case 'G': // ログデータ読出し(SFC9000対応)(RG)
			case 'L': // ログデータ読出し(RL)
				if (l != 8){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングルコマンドのみ データ長:9 */
			case '3': // 測定データ読出し(R3)
				if (l != 9){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングルコマンドのみ データ長:9or10 */
			case '4': // 測定波形読出し(R4)
				if (l != 9 && l != 10){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングルコマンドのみ データ長:11 */
			case '7': // 測定データ読出し(SFC9000対応)(R7)
				if (l != 11){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングルコマンドのみ データ長:11or12 */
			case '8': // 測定波形読出し(SFC9000対応)(R8)
				if (l != 11 && l != 12){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* マルチコマンドのみ */
			case 'D': // ゼロ調整データ読出し(SFC9000対応)(RD)
			case 'Z': // ゼロ調整データ読出し(RZ)
				if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}	//ch_z: マルチCH数
				break;
		/* シングル・マルチ対応 */
			case 'S': // ステータス読出し(SFC9000対応)(RS)
			case 's': // ステータス読出し(Rs)
			case 'l': // ローカット読出し(Rl)
			case 'z': // ゼロ点調整状態読出し(Rz)
			case 'W': // 波形状態読出し(RW)
			case 'C': // 測定データ読出し(SFC9000対応・R7コマンド短縮版)(RC)
				if (ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){		//シングルコマンド
					if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				}
				else {					//マルチコマンド
					if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				}
				break;
			case 'c': // デジタルフィルタ係数読み出し
				if ((l != 9) && (l != 10)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			}	
			break;
		case 'W': /***** 書込み系 *****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ 文字種別判定 */
			case 'r': // フルスケール書込み(Wr)
			case 'k': // Ｋファクタ書込み(Wk)
			case 'd': // ダンピング書込み(Wd)
			case 'l': // ローカット書込み(Wl)
			case 'b': // バーンアウト書込み(Wb)
			case 'v': // 動粘度係数書込み(Wv)
			case 'h': // エラーホールドタイム書込み(Wh)
			case 'R': // 逆流判定値書込み(WR)
			case 'E': // 波形異常判定値設定書込み(WE)
			case 'A': // アッテネータゲイン書込み(WA)
			case 'y': // 薬液リニアライズモード書込み(Wy)
			case 'f': // フィルタ設定書込み(Wf)
			case 't': // 積算目標値書込み(Wt)
			case 'T': // 積算オフセット値読出し(WT)
			case 'u': // ユーザーリニアライズ書込み(Wu)
			case 'U': // ユーザーリニアライズ切替え(WU)
			case 'm': // メーカーリニアライズ書込み(Wm)
			case '2': // デバックモード設定書込み(W2)
			case '6': // デバックモード設定書込み(SFC9000対応)(W6)
			case '9': // パルス出力(W9)
			case 'i': // 検査モード書込み(Wi)
			case 'g': // 口径書込み(Wg)
			case 'p': // センサパルス設定書込み(SFC014E互換)(Wp)
				letter_judge(com_mode,l,0);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case '1': // メーカー設定書込み(W1)
				letter_judge(com_mode,l,2);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;			
			case '5': // メーカー設定書込み(SFC9000対応)(W5)
				letter_judge(com_mode,l,1);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case '7': // ゼロ調整詳細データ書込み(SFC9000対応)(W7)
				letter_judge(com_mode,l,3);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case 'n': // シリアルナンバー書込み(SFC014E互換)(Wn)
				letter_judge(com_mode,l,4);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;

		/* マルチコマンドのみ */
			case 'P': // パラメータセット書込み(WP)
			case 'Z': // ゼロ調整データ書込み(WZ):改造後マルチのみに移動
				letter_judge(com_mode,l,0);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				l = 8 + 2*ch_z[com_mode];
				if (RX_buf[com_mode][l] != ','){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングル・マルチ対応 */
			case 'z': // ゼロ調整データ書込み(SFC9000対応)(Wz)		
				letter_judge(com_mode,l,0);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				if (ch_no[com_mode] == 0){	//マルチコマンド
					l = 8 + 2*ch_z[com_mode];
					if (RX_buf[com_mode][l] != ','){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				}
				break;
				
			//デジタルフィルタ係数書き込み
			case 'c':
				break;

			}
			break;
		case 'O': /**** 主に動作指示 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ */
			case 'V': // 積算値読出し(OV)
			case 'Z': // ゼロ調整実行(OZ)
			case 'S': // 設定値保存(OS)
			case 'J': // ゼロ調整値保存(OJ)
			case 'C': // アラームリセット(OC)
			case 'L': // ログクリア(OL)
			case 'R': // ＲＡＭクリア(OR)
			case 'T': // ＬＥＤ全点灯(OT)
			case 'A': // アラーム強制出力(OA)
			case 't': // 積算リセット(Ot)
#if defined(FRQSCH)
			case 'F': // 初期ゼロ調実施(OF)
#endif
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* マルチコマンドのみ */
			case 'E': // 瞬時流量1点読出し(OE)
			case 'Q': // 瞬時流量10点読出し(OQ)
				if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}	//ch_z: マルチCH数
				break;
			}
			break;
		case 'o': /**** oq,ovコマンド用 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ */
			case 'v': // 積算値読出し・積算中常時更新(ov)
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* マルチコマンドのみ */
			case 'q': // 瞬時流量10点読出し・ローカットオフ(oq)
				if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}	//ch_z: マルチCH数
				break;
			}
			break;
		case 'd': /**** ダウンロード系 **** ダウンロード準備(dp)のみデータ長が変動するため、コマンド解析で個別判定 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ データ長:544 */
			case 't': // ダウンロードデータ転送(dt)
				if (l != 544){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* シングルコマンドのみ データ長:12 */
			case 'w': // ダウンロードデータ書込み(dw)
			case 'c': // ダウンロード中断(dc)
			case 'r': // ダウンロード後リセット(dr)
				if (l != 12){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			}
			break;
		case 'D':
			switch(RX_buf[com_mode][2]) {
			case 'L':
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case 'l': //FpgaDownload
				//何かしらの入力判定
				// if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'E': /**** 再起動 ****/
			switch (RX_buf[com_mode][2]){	//ヘッダーコード末尾
		/* シングルコマンドのみ */
			case 'R': // リセットコマンド(ER)
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			}
			break;
		}
	}	//フォーマット判定終了

}

/************************************************/
/*    CUnet データ長異常 (End Code 15)    */
/************************************************/
void check_cu_length(short com_mode)
{
	if( end_code[com_mode] != FRAMING_ERROR &&			//End Code 11
		end_code[com_mode] != PARITY_ERROR &&			//End Code 10
		end_code[com_mode] != OVERRUN_ERROR)		//End Code 12		
	{
		if (com_mode == HOST){
			if(len_err_host > 0){
				error_check[com_mode]++;
				end_code[com_mode] = CU_LENGTH_ERROR;
			}
		}
		else{
			if(len_err_sub > 0){
				error_check[com_mode]++;
				end_code[com_mode] = CU_LENGTH_ERROR;
			}
		}
	}
}

/************************************************/
/*    WDT再起動時 (1回のみ End Code 33)    */
/************************************************/
void check_wdt(short com_mode)
{
	if( end_code[com_mode] != FRAMING_ERROR &&			//End Code 11
		end_code[com_mode] != PARITY_ERROR &&				//End Code 10
		end_code[com_mode] != OVERRUN_ERROR &&			//End Code 12
		end_code[com_mode] != CU_LENGTH_ERROR &&	//End Code 18
		end_code[com_mode] != FCS_ERROR &&					//End Code 13
		end_code[com_mode] != COMMAND_ERROR &&			//End Code 16
		end_code[com_mode] != ADDRESS_OVER &&		//End Code 04
		end_code[com_mode] != FORMAT_ERROR)		//End Code 14
	{	
		if(end_wdt == 1){
			error_check[com_mode]++;
			end_code[com_mode] = REBOOT_WDT;
			end_wdt = 0;
		}
	}
}

void JudgeErrCode21_23(short com_mode, short ch_check, short enable_20, short enable_21, short enable_22, short enable_23)
{
	/* EEPROM書込み中　実行不可 */	
	if(action_status_check(ch_check) == ACT_STS_WRITE){
		if (enable_23 != 0){
			end_code[com_mode] = EEPROMWRITE_NOW;
			error_check[com_mode]++;
		}
	}
	/* ダウンロード中　実行不可 */
	if(action_status_check(ch_check) == ACT_STS_DLOAD){
		if (enable_22 != 0){
			end_code[com_mode] = DOWNLOAD_NOW;
			error_check[com_mode]++;
			}
		}
	/* 積算中　実行不可 */
	if(action_status_check(ch_check) == ACT_STS_ADDIT){
		if (enable_21 != 0){
			end_code[com_mode] = INTEGRATE_NOW;
			error_check[com_mode]++;
		}
	}
	/* ゼロ調整中　実行不可 */
	if(action_status_check(ch_check) == ACT_STS_ZERO){
		if (enable_20 != 0){
			end_code[com_mode] = ZEROADJUST_NOW;
			error_check[com_mode]++;
		}
	}
}

/************************************************/
/*    実行不可 (End Code 20～23)    */
/************************************************/
void check_act_disable(short com_mode)
{
	short enable_20 = 0;
	short enable_21 = 0;
	short enable_22 = 0;
	short enable_23 = 0;
	short ch_check;
	short cnt;

	if( end_code[com_mode] != FRAMING_ERROR &&			//End Code 11
		end_code[com_mode] != PARITY_ERROR &&				//End Code 10
		end_code[com_mode] != OVERRUN_ERROR &&			//End Code 12
		end_code[com_mode] != CU_LENGTH_ERROR &&	//End Code 18
		end_code[com_mode] != FCS_ERROR &&					//End Code 13
		end_code[com_mode] != COMMAND_ERROR &&			//End Code 16
		end_code[com_mode] != ADDRESS_OVER &&		//End Code 04
		end_code[com_mode] != FORMAT_ERROR &&		//End Code 14
		end_code[com_mode] != REBOOT_WDT)			//End Code 33
	{
		/* 20 ゼロ調整中 実行可：R*,OE,OQ,oq */
		/* 21 積算実行中 実行可：R*,OE,OQ,oq,OV,ov */
		/* 22 ダウンロード中実行可： RS,Rs */
		/* 23 書込み中実行可： RS,Rs,OE,OQ,oq */
		
		switch (RX_buf[com_mode][1]){
		case 'R':
			switch (RX_buf[com_mode][2]){
			/* 20～23 実行可 */
			case 'S':
			case 's':
				break;
			/* 22,23 実行不可 */
			default:
				enable_22++;
				enable_23++;
				break;
			}
			break;
		case 'O':			
			switch (RX_buf[com_mode][2]){
			/* 22 実行不可 */
			case 'E':
			case 'Q':
				enable_22++;
				break;			
			/* 20,22,23 実行不可 */
			case 'V':
				enable_20++;
				enable_22++;
				enable_23++;
				break;
			/* 20～23 実行不可 */			
			default:
				enable_20++;
				enable_21++;
				enable_22++;
				enable_23++;
				break;
			}
			break;
		case 'o':			
			switch (RX_buf[com_mode][2]){
			/* 22 実行不可 */
			case 'q':
				enable_22++;
				break;			
			/* 20,22,23 実行不可 */
			case 'v':
				enable_20++;
				enable_22++;
				enable_23++;
				break;
			}
			break;
		/* 20～23 実行不可 W*コマンドは全て該当*/
		default:
			enable_20++;
			enable_21++;
			enable_22++;
			enable_23++;
			break;
		}
		
		if (ch_no[com_mode] == 0){		//マルチコマンド
			for(cnt = 0; cnt < ch_z[com_mode]; cnt++){		//マルチCH数(1～6)分実行不可をチェックする
				ch_check = multi_ch_no[com_mode][cnt] - 1;
				JudgeErrCode21_23(com_mode, ch_check, enable_20, enable_21, enable_22, enable_23);
			}
		}else{				//シングルコマンド
			ch_check = ch_no[com_mode] -1;
			JudgeErrCode21_23(com_mode, ch_check, enable_20, enable_21, enable_22, enable_23);
		}
	}
}

/************************************************/
/*    実行可 (測定障害：End Code 32)    */
/************************************************/
void check_act_able(short com_mode)
{
	short ch;
	short mes_err_cnt;
		
	if( end_code[com_mode] != FRAMING_ERROR &&		//End Code 11
		end_code[com_mode] != PARITY_ERROR &&		//End Code 10
		end_code[com_mode] != OVERRUN_ERROR &&		//End Code 12
		end_code[com_mode] != CU_LENGTH_ERROR &&	//End Code 18
		end_code[com_mode] != FCS_ERROR &&		//End Code 13
		end_code[com_mode] != COMMAND_ERROR &&		//End Code 16
		end_code[com_mode] != ADDRESS_OVER &&		//End Code 04
		end_code[com_mode] != FORMAT_ERROR &&		//End Code 14
		end_code[com_mode] != REBOOT_WDT &&		//End Code 33
		end_code[com_mode] != ZEROADJUST_NOW &&		//End Code 20
		end_code[com_mode] != INTEGRATE_NOW &&		//End Code 21
		end_code[com_mode] != DOWNLOAD_NOW &&		//End Code 22
		end_code[com_mode] != EEPROMWRITE_NOW )		//End Code 23	
	{
		if(ch_no[com_mode] == 0){	//マルチコマンド：全CHエラー判定
			mes_err_cnt = 0;
			for (ch = 0; ch < 6; ch++){
				if (MAIN[ch].com_err_status != 0) mes_err_cnt++;	
			}
			if (mes_err_cnt != 0) mes_err[0] = 1;
		}	
		else{				//シングルコマンド：該当CHのみエラー判定
			if (MAIN[ch_no[com_mode] -1].com_err_status != 0) mes_err[ch_no[com_mode]] = 1;
		}		
	}
}

short CalcFcs(char* Buf, short Len){
	short i;
	short Fcs = Buf[0];
	for(i = 1; i < Len; i++)
	{
		Fcs ^= Buf[i];
	}
	return Fcs;
}

char ConvertAscii(char Val){
	if('0' <= Val && Val <= '9'){
		Val -= 0x30;
	}
	else if('A' <= Val && Val <= 'F'){
		Val -= 0x37;
	}
	return Val;
}

short GetBufToVal(char Val1, char Val2)
{
	short BufFcs = 0;
	Val1 = ConvertAscii(Val1);
	Val2 = ConvertAscii(Val2);
	BufFcs = Val1 * 10 + Val2;
	return BufFcs;
}

short JudgeChNo(char* Buf)
{
	char c1 = Buf[1];
	char c4 = Buf[4];
	char c5 = Buf[5];
	char c6 = Buf[6];
	char c7 = Buf[7];
	short Flg = 0;
	if(c4 == '0' && ('1' <= c5 && c5 <= '6'))
	{
		Flg = 10;
	}
	else if(c4 == 'X' && c5 == 'X' && c6 == '0' && ('1' <= c7 && c7 <= '6'))
	{
		Flg = 20;
	}
	else if(c1 == 'd' && c4 == 'X' && c5 == 'X' && c6 == ',')
	{
		Flg = 30;
	}
	else{
		Flg = 90;
	}
	return Flg;
}

short ConvertAsciiIndex(char c)
{
	short Index = 0;
	if('a' <= c && c <= 'z')
	{
		Index = c - 'a';
	}
	else if('A' <= c && c <= 'Z')
	{
		Index = c - 'A' + 27;
	}
	else if('0' <= c && c <= '9')
	{
		Index = c - '0' + 53;
	}
	else {
		Index = 99;
	}
	return Index;
}
short JudgeMultiCmd(char c1, char c2){
	short MltiFlg = 0;
	//RS, Rs, OE, OQ, oq, Rl, Rz, RD, RZ, Wz, WZ, WP, RW
	if(c1 == 'R'){
		if(
			(c2 == 'S') 
			|| (c2 == 's')
			|| (c2 == 'l')
			|| (c2 == 'z')
			|| (c2 == 'D')
			|| (c2 == 'Z')
			|| (c2 == 'W')
		)
		{
			MltiFlg = 1;
		}
	}
	else if(c1 == 'W'){
		if(
			(c2 == 'z')
			|| (c2 == 'Z')
			|| (c2 == 'P')
		)
		{
			MltiFlg = 1;
		}
	}
	else if(c1 == 'O'){
		if(
			(c2 == 'E')
			|| (c2 == 'Q')
		)
		{
			MltiFlg = 1;
		}
	}
	else if(c1 == 'o'){
		if(
			(c2 == 'q')
		)
		{
			MltiFlg = 1;
		}
	}
	return MltiFlg;
}

short GetCmdIndex(char* Buf)
{
	//WXYZ
	// W : コマンド系統(R=0, W=1, O=2, M=3)
	// X : 0=シングル/1=マルチコマンド
	// YZ : a-z, A-Z, 0-9 (Max 62)
	short Index = 0;
	char c1 = Buf[1];
	char c2 = Buf[2];
	short MultiFlg = JudgeMultiCmd(c1, c2);
	if(c1 == 'R')
	{
		Index = 0;
	}
	else if(c1 == 'W')
	{
		Index = 1000;
	}
	Index += ConvertAsciiIndex(c2);
	if(MultiFlg != 0){
		Index += 100;
	}
	return Index;
}

const struct
{
	char Cmd[2];
	char Bank;
	char Multi;
}CmdList[]={
	{ 'R', 'g', 0, 0 },
	{ 'R', 'r', 0, 0 },
	{ 'R', 'k', 0, 0 },
	{ 'R', 'd', 0, 0 },
	{ 'R', 'b', 0, 0 },
	{ 'R', 'v', 0, 0 },
	{ 'R', 'h', 0, 0 },
	{ 'R', 'u', 0, 0 },
	{ 'R', 'R', 0, 0 },
	{ 'R', 'F', 0, 0 },
	{ 'R', 'V', 0, 0 },
	{ 'R', 'G', 0, 0 },
	{ 'R', 'L', 0, 0 },
	{ 'R', 'y', 0, 0 },
	{ 'R', 'M', 0, 0 },
	{ 'R', 'f', 0, 0 },
	{ 'R', 't', 0, 0 },
	{ 'R', 'T', 0, 0 },
	{ 'R', 'p', 0, 0 },
	{ 'R', 'n', 0, 0 },
	{ 'R', 'a', 0, 0 },
	{ 'R', 'm', 1, 0 },
	{ 'R', '1', 1, 0 },
	{ 'R', '2', 1, 0 },
	{ 'R', '3', 1, 0 },
	{ 'R', '4', 1, 0 },
	{ 'R', '5', 1, 0 },
	{ 'R', '6', 1, 0 },
	{ 'R', '7', 1, 0 },
	{ 'R', '8', 1, 0 },
	{ 'R', '9', 1, 0 },
	{ 'R', 'X', 1, 0 },
	{ 'R', 'E', 1, 0 },
	{ 'R', 'A', 1, 0 },
	{ 'R', 'i', 1, 0 },
	{ 'R', 'D', 1, 1 },
	{ 'R', 'Z', 1, 1 },
	{ 'R', 'S', 1, 2 },
	{ 'R', 's', 1, 2 },
	{ 'R', 'l', 1, 2 },
	{ 'R', 'z', 1, 2 },
	{ 'R', 'W', 1, 2 },
	{ 'R', 'C', 1, 2 },
	{ 'R', 'c', 1, 2 },
	{ 'W', 'g', 0, 0 },
	{ 'W', 'r', 0, 0 },
	{ 'W', 'k', 0, 0 },
	{ 'W', 'd', 0, 0 },
	{ 'W', 'l', 0, 0 },
	{ 'W', 'b', 0, 0 },
	{ 'W', 'v', 0, 0 },
	{ 'W', 'h', 0, 0 },
	{ 'W', 'R', 0, 0 },
	{ 'W', 'u', 0, 0 },
	{ 'W', 'U', 0, 0 },
	{ 'W', 'y', 0, 0 },
	{ 'W', 'f', 0, 0 },
	{ 'W', 't', 0, 0 },
	{ 'W', 't', 0, 0 },
	{ 'W', 'T', 0, 0 },
	{ 'W', 'p', 0, 0 },
	{ 'W', 'n', 0, 0 },
	{ 'W', 'a', 0, 0 },
	{ 'W', 'm', 1, 0 },
	{ 'W', '1', 1, 0 },
	{ 'W', '2', 1, 0 },
	{ 'W', '3', 1, 0 },
	{ 'W', '4', 1, 0 },
	{ 'W', '5', 1, 0 },
	{ 'W', '6', 1, 0 },
	{ 'W', '7', 1, 0 },
	{ 'W', '8', 1, 0 },
	{ 'W', '9', 1, 0 },
	{ 'W', 'E', 1, 0 },
	{ 'W', 'A', 1, 0 },
	{ 'W', 'i', 1, 0 },
	{ 'W', 'P', 1, 1 },
	{ 'W', 'Z', 1, 1 },
	{ 'W', 'z', 1, 2 },
	{ 'W', 'c', 1, 2 },
	{ 'O', 'V', 0, 0 },
	{ 'O', 'Z', 0, 0 },
	{ 'O', 'S', 0, 0 },
	{ 'O', 's', 0, 0 },
	{ 'O', 'J', 0, 0 },
	{ 'O', 'C', 0, 0 },
	{ 'O', 'L', 0, 0 },
	{ 'O', 'R', 0, 0 },
	{ 'O', 'T', 0, 0 },
	{ 'O', 'A', 0, 0 },
	{ 'O', 't', 0, 0 },
	{ 'O', 'F', 0, 0 },
	{ 'O', 'E', 0, 1 },
	{ 'O', 'Q', 0, 1 },
	{ 'o', 'v', 0, 0 },
	{ 'o', 'q', 0, 1 },
	{ 'd', 'p', 0, 0 },
	{ 'd', 't', 0, 0 },
	{ 'd', 'w', 0, 0 },
	{ 'd', 'c', 0, 0 },
	{ 'd', 'r', 0, 0 },
	{ 'D', 'L', 1, 0 },
	{ 'D', 'l', 1, 0 },
	{ 'E', 'R', 0, 0 },
	{ 'M', 'R', 0, 0 },
	{ 'M', 'W', 0, 0 },
	{ 'M', 'T', 0, 0 },
	{ 'M', 'Z', 0, 0 },
	{ 'M', 'S', 0, 0 },
	{ 0xFF, 0xFF, 0xFF, 0xFF }
};

short JudgeUsableCommand(char* Buf, short* Index, short *Bank, short *Multi)
{
	short i = 0;
	char* Cmd;
	short Flg = 0x04;
	short MaxNum = sizeof(CmdList) / (sizeof(char) * 4);
	Bank = 0;
	Multi = 0;
	Index = -1;
	for(i = 0; i < MaxNum; i++)
	{
		Cmd = (char*)CmdList[i].Cmd;
		Bank = CmdList[i].Bank;
		Multi = CmdList[i].Multi;
		if(strncmp(Buf, Cmd, 2) == 0)
		{
			Index = i;
			Flg = 0;
			break;
		}
	}
	return Flg;
}

short JudgeReceiveFormat(short com_mode)
{
	short Flg = 0;
    char* Buf = RX_buf[com_mode];
	short BufLen = strlen(Buf);
	short ClcFcs, BufFcs;
	short ChFlg = 0;
	short MultiChNo = 0;
	short ChCnt = 0;
	short MultiCh = 0;
	short CmdIndex = 0;
	short BankNo;
	short MultiFlg;
	short ErrFlg;

	//受信メッセージ長判定
	if(BufLen < 9)
	{
		Flg |= 0x01;
	}
	else
	{
		//FCS判定
		ClcFcs = CalcFcs(Buf, BufLen - 3); //FCS(2byte), CR(1byte)
		BufFcs = GetBufToVal(Buf[BufLen - 2], Buf[BufLen - 1]);
		if(ClcFcs != BufFcs)
		{
			Flg |= 0x02;
		}

		//コマンド有効判定
		Flg |= JudgeUsableCommand(Buf, CmdIndex, BankNo, MultiFlg);

		//コマンドが存在する
		if(Flg == 0)
		{
			//CH番号判定
			ChFlg = JudgeChNo(Buf);
			if(ChFlg == 10){
				ch_no[com_mode] = ConvertAscii(Buf[5]);
			}
			else if(ChFlg == 20){
				MultiChNo = ConvertAscii(Buf[7]);
				for(ChCnt = 0; ChCnt < MultiChNo; ChCnt++)
				{
					MultiCh = GetBufToVal(Buf[8 + 2 * ChCnt], Buf[8 + 2 * ChCnt + 1]);
					if(1 <= MultiCh && MultiCh <= 6){
						multi_ch_no[com_mode][ChCnt] = MultiCh;
					}
				}
			}
			else if(Flg == 30){
				ch_no[com_mode] = 0;
			}
			else{
				ch_no[com_mode] = 9;
			}

			//
		}
	}
}

/************************************************/
/*    通信プロトコル処理					    */
/************************************************/
void protocol(short com_mode)
{
#if 1
	//1. フォーマット判定
	// 1-1. コマンド長判定
	// 1-2. FCS判定
	// 1-3. コマンド・バンク・チャネル判定
	// 1-4. 各コマンドフォーマット判定
	//2. ウォッチドッグ再起動判定
	//3. 各コマンド処理
	// 3-1. Endcode20-23判定
	// 3-2. 測定障害判定
	// 3-3. コマンド解析&返送バッファ作成
	//4. 送信
#else
	short i;
	short h;
	short l;
	short z;
	short  fcs;				// FCS計算用
	short fcs_cal;
	long fcs_d;				// FCS判定用
	char fcs_c[5] ={0};	// FCS格納用
	char comma[5] = ",";	// ','代入用
	char add[20] = {0};
	short enter_download_mode = 0;

	for (i=0; i<MSG_MAX; i++){
		TX_buf[com_mode][i] = 0;	//送信バッファクリア
	}

	error_check[com_mode] = 0;
	echo_back[com_mode] = 0;
	end_code[com_mode] = 0;
	flow_check[com_mode] = 0;
	
	/************************* データ数判定 *************************/
	/*  データ数が最小数9バイト未満の場合 = エコーバック (優先度1)  */
	/****************************************************************/
	l = strlen(RX_buf[com_mode]);

	if (l < 9){					//ターミネータ=CR含む
		error_check[com_mode]++; echo_back[com_mode]++;
		for (i=0; i<=7; i++){
			TX_buf[com_mode][i] = RX_buf[com_mode][i];		//受信データを返信用列にそのまま格納
		}
	}
	/*************************** 判定終了 ***************************/
	else {
		/* FCS判定用処理 */
		fcs_d = GetBufFcs(RX_buf[com_mode][l - 3], RX_buf[com_mode][l - 2]);
	
		/* 共通：返信用列準備（End Code代入欄まで作成）*/
		RX_buf[com_mode][l-1] = 0;		//CR 削除
		RX_buf[com_mode][l-2] = 0;		//FCS削除
		RX_buf[com_mode][l-3] = 0;
	
		for (i=0; i<=5; i++){
			TX_buf[com_mode][i] = RX_buf[com_mode][i];		//@からCHNo.まで返信用列に格納
		}
		/* 共通：チャネル番号読出し・チャネル判定用処理 */
		check_channel(com_mode);
		
		/* オーバーランエラー (優先度4) */
		if ((err_stat & 0x0004) == 0x0004){ error_check[com_mode]++; end_code[com_mode] = OVERRUN_ERROR;}		
		/* パリティエラー (優先度3) */
		if ((err_stat & 0x0001) == 0x0001){ error_check[com_mode]++; end_code[com_mode] = PARITY_ERROR;}
		/* フレーミングエラー (優先度2) */
		if ((err_stat & 0x0002) == 0x0002){ error_check[com_mode]++; end_code[com_mode] = FRAMING_ERROR;}
		
		/* CUnetデータ長エラー (優先度5) */
		if(com_type == COM_CUNET)	check_cu_length(com_mode);
		
		/************************** ＦＣＳ判定 **************************/
		/*     ＦＣＳが一致しない場合、End Code:13 を返す (優先度6)     */
		/****************************************************************/
		if (fcs != fcs_d){ error_check[com_mode]++; end_code[com_mode] = FCS_ERROR;}
		fcs_c[0] = fcs_c[1] = 0;		//初期化

		/* コマンド・バンク・チャネル判定 */
		check_command(com_mode);
	
		/* フォーマット判定 */
		check_format(com_mode);
		
		/* その他エラー(WDT再起動・手動電源断検出) */
		check_wdt(com_mode);
		
		/* 実行不可エラー */
		check_act_disable(com_mode);
		
		/* 実行可エラー */
		check_act_able(com_mode);

	}//else(データ数判定正常・エコーバック区分け用)

	if (error_check[com_mode] == 0){
		/************************* コマンド解析 *************************/

		/*********************** コマンド解析開始 ***********************/
		switch (RX_buf[com_mode][1]){          // ヘッダーコード先頭
			/*************** データ読出し ***************/
			case 'R':
				TX_buf[com_mode][6] = 0x30;
				TX_buf[com_mode][7] = 0x30;
				
				if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
					TX_buf[com_mode][8] = ',';	//シングルのみEnd Code直後に','
				}
		
				read_command_R(com_mode);
		
				strncat(TX_buf[com_mode],comma,sizeof(comma));
				
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//測定障害
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM障害
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode]/ 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;

				end_code[com_mode] = 99;			//返信用列作成処理用
				break;
		
			/*************** データ書込み ***************/
			case 'W':
				if( MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				    MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
					read_command_W(com_mode);
				}
				break;
		
			/******* 動作指示＋流量・積算値読出し *******/
			case 'O':
				TX_buf[com_mode][6] = 0x30;
				TX_buf[com_mode][7] = 0x30;
				
				switch (RX_buf[com_mode][2]){
					/* 積算値読出し(OV) */			
					case 'V':
						command_OV(com_mode);
						break;
					/* 瞬時流量1点読出し(OE) */
					case 'E':
						command_OE(com_mode);
						break;
					/* 瞬時流量10点読出し(OQ) */
					case 'Q':
						command_OQ(com_mode);
						break;
					/* ゼロ調整実行(OZ) */			
					case 'Z':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				    		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OZ(com_mode);
//							SAVE[ch_no[com_mode] -1].control |= 0x0001;
						}
						break;
#if defined(FRQSCH)
					case 'F': // 初期ゼロ調実施(OF)
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
							MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OF(com_mode);
						}
						break;
#endif
					/* 設定値保存(OS) */
					case 'S':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				  		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OS(com_mode);
						}
						break;
					/* センサメモリ設定値保存(Os) */
					case 's':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				  		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_Os(com_mode);
						}
						break;
					/* ゼロ調整値保存(OJ) */
					case 'J':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				  		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OJ(com_mode);
						}
						break;						
					/* アラームリセット(OC) */
					case 'C':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				   	  	   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							SAVE[ch_no[com_mode] -1].control |= 0x0002;
						}
						break;
					/* ログクリア(OL) */	
					case 'L':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				   		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OL(com_mode);
						}
						break;
					/* ＲＡＭクリア(OR) */
					case 'R':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				   		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){					
							command_OR();
						}
						break;
					/* ＬＥＤ全点灯(OT) */
					case 'T':
						command_OT();
						break;
					/* アラーム強制出力(OA) */
					case 'A':
						command_OA();	
					break;
					/* 積算値リセット(Ot) */	
					case 't':
						command_Ot(com_mode);
						break;
					default:
						break;
				}
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1) end_code[com_mode] = MEASURE_ERROR;	//測定障害
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM障害
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
				
				end_code[com_mode] = 99;			//返信用列作成処理用
				break;			
#ifdef MEMDBG
			/*************** Debugメモリ ***************/
			case 'M':
				
				MemoryCom(com_mode);
		
				strncat(TX_buf[com_mode],comma,sizeof(comma));
				
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//測定障害
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM障害
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
				
				end_code[com_mode] = 99;			//返信用列作成処理用
				break;
					
#endif
			/******* 動作指示＋流量・積算値読出し *******/
			case 'o':
				TX_buf[com_mode][6] = 0x30;
				TX_buf[com_mode][7] = 0x30;
				
				switch (RX_buf[com_mode][2]){
					/* 瞬時流量10点読出し・ローカットオフ(oq) */
					case 'q':
						command_oq(com_mode);
						break;			
					/* 積算値読出し・積算中常時更新(ov) */
					case 'v':
						command_ov(com_mode);
						break;
					break;
				}				
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1) end_code[com_mode] = MEASURE_ERROR;	//測定障害
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM障害
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
				
				end_code[com_mode] = 99;			//返信用列作成処理用
				break;
				
			/*************** ダウンロード ***************/
			case 'D':
				switch (RX_buf[com_mode][2]) {
					case 'L':
						TX_buf[com_mode][6] = 0x30;
						TX_buf[com_mode][7] = 0x30;
						
						if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//測定障害
						if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM障害
							end_code[com_mode] = EEPROM_ERROR;
							eep_err = 0;
						}

						TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
						TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
						
						end_code[com_mode] = 99;			//返信用列作成処理用
						enter_download_mode = 1;
						break;
					case 'l': //FpgaDownload
						TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
						TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
						
						command_Dl(com_mode);
						end_code[com_mode] = 99;			//返信用列作成処理用
						break;
				}
				break;

			/*************** リセット(ER) ***************/
			case 'E':
				switch (RX_buf[com_mode][2]){
					/* リセットコマンド(ER) */
					case 'R':
						SAVE[ch_no[com_mode] -1].control |= 0x0008;
						end_code[com_mode] = REBOOT_MANUAL;
						break;
				}
				break;

		}//switch RX_buf[1]

	}//error_check[com_mode] == 0
	
	if (echo_back[com_mode] == 0){	
		/*************** End Code代入 ***************/
		if (end_code[com_mode] != 99){		//W*コマンド用
			if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//測定障害
			if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM障害
				end_code[com_mode] = EEPROM_ERROR;
				eep_err = 0;
			}
			
			add[0] = end_code[com_mode] / 10 + 0x30;
			add[1] = end_code[com_mode] % 10 + 0x30;
			strncat(TX_buf[com_mode],add,sizeof(add));
			/************* 書込処理時返信列 *************/
			for (i=6; i<=400; i++){TX_buf[com_mode][i+2] = RX_buf[com_mode][i];}
		}
		end_code[com_mode] = 0;				//EndCode初期化
		/*************** ＦＣＳ再計算 ***************/
		if (flow_check[com_mode] == 0)		{l = strlen(TX_buf[com_mode]);}
		else if (flow_check[com_mode] == 1)	{l = 10 +  9*ch_z[com_mode];}
		else if (flow_check[com_mode] == 2)	{l = 10 + 72*ch_z[com_mode];}

		for(h = l; h < MSG_MAX; h++){TX_buf[com_mode][h]= 0;}	//TEXT欄以降 配列初期化
		// fcs = TX_buf[com_mode][0];		//=@(0x40)
		// z = 1;
		// while (z != l){
		// 	fcs ^= TX_buf[com_mode][z];
		// 	z++;
		// }
		fcs = CalcFcs(TX_buf[com_mode], l);
		
		fcs_cal = fcs / 0x10;
		
		if (fcs_cal >= 0 && fcs_cal <=9)	fcs_c[0] = fcs_cal  + 0x30;	// 0x30:ASCII→16進数字変換0-9
		else	/*A-F*/							fcs_c[0] = fcs_cal  + 0x37;	// 0x37:ASCII→16進数字変換A-F
		
		fcs_cal = fcs % 0x10;

		if (fcs_cal >= 0 && fcs_cal <=9)	fcs_c[1] = fcs_cal  + 0x30;	// 0x30:ASCII→16進数字変換0-9
		else	/*A-F*/							fcs_c[1] = fcs_cal  + 0x37;	// 0x37:ASCII→16進数字変換A-F		
		
		
		if(flow_check[com_mode] == 0){
			strcat(TX_buf[com_mode],fcs_c);		//TX_buf最後尾に追加
		}
		else{
			TX_buf[com_mode][l]   = fcs_c[0];
			TX_buf[com_mode][l+1] = fcs_c[1];
		}	
		/*************** ターミネータ ***************/
		TX_buf[com_mode][l+2] = CR;			//=CR(0x0d)
	}//echo_back[com_mode] != 0
	error_check[com_mode] = 0; echo_back[com_mode] = 0;	flow_check[com_mode] = 0;	//初期化
	/* 送信 */
	l = l+3;
//host
	if (com_mode == HOST || com_mode == SUB_HOST){
		if(com_type == COM_RS485){
			TX_start_host(l);				//メッセージ送信開始(RS485通信)

		}else{
			mky43_TX_start(com_mode);		//メッセージ送信開始(CUnet通信)
		}
	}
//ment
	else{	
		TX_start_ment(l);
	}
	if(enter_download_mode) {
		if(com_mode == MENT) {
			// GPIO_PD6
			while(__bit_input(GPIO_PORTD_BASE, 6) == 1);		//最後の送信が終わるのを待つ
		} else {
			if(com_type == COM_RS485){
				// GPIO_PP4
				while(__bit_input(GPIO_PORTP_BASE, 4) == 1);	//最後の送信が終わるのを待つ
			}
		}		
		download(com_mode, com_type);				//ファームウェアダウンロード
	}
#endif
}
