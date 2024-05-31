/***********************************************/
/* File Name : prtocol.c		         									   */
/*	Summary   : �ʐM�v���g�R������                   */
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
/*	���W���[������`�֐�								*/
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
/*	���W���[���O��`�֐�								*/
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
/*	���W���[������`�ϐ�								*/
/********************************************************/
short i_num[MSG_NUM];			// ����ԍ��p
short ch_no[MSG_NUM];			// �`���l���ԍ��p
short size[MSG_NUM];				// ���͒l�f�[�^������p
short digit_check[MSG_NUM];		// ����or�����_���� ����p
short end_code[MSG_NUM];		// �G���h�R�[�h�����p
short flow_check[MSG_NUM];	// OE/OQ����������p
long long value[MSG_NUM];		// ���͒l����p
char RX_buf[MSG_NUM][MSG_MAX];
char TX_buf[MSG_NUM][MSG_MAX];
short	multi_ch_no[MSG_NUM][CH_NUMMAX];	// �}���`�R�}���h�`���l���ԍ�
short ch_z[MSG_NUM];				// �`���l�����p
short error_check[MSG_NUM];	// �G���[�`�F�b�N�p
short echo_back[MSG_NUM];	// �G�R�[�o�b�N�p
short l_check[MSG_NUM];		// �g�p������ޔ���p
short eep_err;
short mes_err[7];

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
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
/*		�Ǎ��l�ϊ��p�֐�		*/
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
	
	//max11��
	
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
		add_read[z_add] = ( read % pow_1) / pow_2 + 0x30;	// 0x30:ASCII�������ϊ�
		pow_1 *= 10;
		pow_2 *= 10;
	}
	switch(point){
		case 0://�Ǎ��l���̂܂�
			for (z_add = 0; z_add < level; z_add++){
				add_cal[z_add] = add_read[z_add];
			}
			break;	
		case 1://�Ǎ��l��1/10
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
		case 2://�Ǎ��l��1/100
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
		case 3://�Ǎ��l��1/1000
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
	
	if(pos == ADD_CMM_BFR_VAL){					//�f�[�^�̑O��","��t��
		l_add = strlen(add_cal);
		add_cal[l_add] = 0x2C;		//","
	}

	l_add = strlen(add_cal);
	
	for (z_add = 0; z_add < l_add; z_add++){
	
		add_out[z_add] = add_cal[l_add - z_add -1];
	
	}

	if(pos == ADD_CMM_AFR_VAL){					//�f�[�^�̌��","��t��
		l_add = strlen(add_out);
		add_out[l_add] = 0x2C;		//","
	}

	strncat(TX_buf[com_mode],add_out,sizeof(add_out));
	
}

/************************************************/
/*		���͒l����p�֐�						*/
/************************************************/
void value_judge(short com_mode,short si,short pow){
	
	short j;
	short h;
	short z;
	long long value_z;
	short value_f;
	char input[MES_CHAR_MAX] ={0};			// ���͒l����p
	char input_z[MES_CHAR_MAX] ={0};			// ���͒l����p(������)
	char input_f[MES_CHAR_MAX] ={0};			// ���͒l����p(������)

	/* ���͒l���蕔 */
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
		if(z >= MES_CHAR_MAX){	//�z��O�����ݖh�~
			break;
		}
	}

	/* �����_�ȉ��������蕔 */
	z = 0;
	for (j=0; j<size[com_mode]; j++){
		z++;
		if (input[j] == '.') break;	//�����_�ʒu����
	}
	digit_check[com_mode] = size[com_mode] - z;			//�����_�ȉ���������p

	/* ���l�i�[ */
	if(digit_check[com_mode] == 0){		//�����̏ꍇ
		value_z = atoi (input);			//���͐��l�i�[
		for (j=0; j<pow; j++){
			value_z *= 10;
		}
		value[com_mode] = value_z;
	}
	else{					//��������̏ꍇ
		for (j=0; j<z-1; j++){
			input_z[j]=input[j];
			if(j >= MES_CHAR_MAX){	//�z��O�����ݖh�~
				break;
			}
		}		
		value_z = atoi (input_z);
		for (j=0; j<pow; j++){
			value_z *= 10;
		}
				
		for (j=0; j<digit_check[com_mode]; j++){
			input_f[j]=input[z+j];
			if(j >= MES_CHAR_MAX){	//�z��O�����ݖh�~
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
	for (h = 0; h < MES_CHAR_MAX; h++){input[h] = 0;}	//����p�z�� ������
	
}

#ifdef MEMDBG
//Debug
/*******************************************
 * Function : value_judge_Hex
 * Summary  : 16�i���f�[�^�擾&����
 * Argument : int com_mode -> 0 : �z�X�g
 *                            1 : �����e
 *            int si -> �f�[�^�T�C�Y
 *            int pow -> �����_�ȉ�����
 * Return   : void
 * Caution  : �Ȃ�
 * Note     : value_judge() ��10�i�������Ή����Ă��Ȃ��������ߒǉ�
 *            ���s��A value[] �Ƀf�[�^���A size[] �Ƀf�[�^�T�C�Y���i�[�����
 *            �ő�8������4byte�ɕϊ�����("FFFFFFFF" -> 0xFFFFFFFF)
 *            ��M������ɂ͖����� ",(FCS)" �����Ă���̂ŁA�l�̏I���R�[�h��','
 * *****************************************/
void value_judge_Hex(short com_mode, short sPos, short pow)
{	
	short j;
	char input[20] ={0};			// ���͒l����p
	short Pos = sPos;
	unsigned long Val = 0;
	
	//���͕�����擾
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

	//�����_�ʒu�擾
	for (j = 0; j < size[com_mode]; j++)
	{
		if (input[j] == '.') break;	//�����_�ʒu����
	}
	digit_check[com_mode] = size[com_mode] - j;			//�����_�ȉ���������p

	/* ���l�i�[ */
	sscanf(input, "%x", &Val);
	value[com_mode] = (long long)Val;

FUNCEND:
	i_num[com_mode] = sPos + size[com_mode];
	memset(input, 0, sizeof(input));//����p�z�� ������
}
#endif

/************************************************/
/*		���͕�������p�֐�						*/
/************************************************/
void letter_judge(short com_mode,short sl,short sm){
	
	short l;
	short z;
	char letter;

	l_check[com_mode] = 0;
	
	if (sm == 0){	//�ʏ폈���E���l+','�̂�
		for (z = 6; z < sl; z++){
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
	}
	if (sm == 1){	//W5��p�E�V���A���i���o�[(A-Z���e)�Ή�
		i_num[com_mode] = 7;
		for (l=0; l<32; l++){ value_judge(com_mode,i_num[com_mode],0); i_num[com_mode]++;}
		for (z = 6; z < i_num[com_mode]; z++){	//�e�p�����[�^
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
		for (z =  i_num[com_mode]; z < sl; z++){	//�V���A���i���o�[
			letter = RX_buf[com_mode][z];
			if (	(letter == 0x2C)			||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x5A)	){}	// 'A-Z'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
	}
	if (sm == 2){	//W1��p�E�V���A���i���o�[(A-Z���e)�Ή�
		i_num[com_mode] = 7;
		for (l=0; l<5; l++){ value_judge(com_mode,i_num[com_mode],0); i_num[com_mode]++;}
		for (z = 6; z < i_num[com_mode]; z++){	//���j�A���C�Y���l
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
		for (z =  i_num[com_mode]; z < sl; z++){	//�V���A���i���o�[
			letter = RX_buf[com_mode][z];
			if (	(letter == 0x2C)			||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x5A)	){}	// 'A-Z'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
	}	
	if (sm == 3){	//W7��p�E�X�e�[�^�X(A-F���e)�Ή�
		for (z = 6; z < sl; z++){
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x47)	){}	// 'A-F'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
	}
	if (sm == 4){	//Wp��p�E�V���A���i���o�[(A-Z���e)�Ή�
		i_num[com_mode] = 7;
		for (z = 6; z < i_num[com_mode]; z++){	//�e�p�����[�^
			letter = RX_buf[com_mode][z];
			if (	(letter >= 0x2C && letter <= 0x2E)	||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	){}	// '0-9'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
		for (z =  i_num[com_mode]; z < sl; z++){	//�V���A���i���o�[
			letter = RX_buf[com_mode][z];
			if (	(letter == 0x2C)			||	// ','or'-'or'.'
				(letter >= 0x30 && letter <= 0x39)	||	// '0-9'
				(letter >= 0x41 && letter <= 0x5A)	){}	// 'A-Z'				
			else { l_check[com_mode]++; }					//��L�ȊO
		}
	}
}
/************************************************/
/*    �`���l���ԍ��Ǐo���E�`���l������p�֐�    */
/************************************************/
void check_channel(short com_mode){
	
	short m;
	short check;
	char ch[5] ={0};

	ch_z[com_mode] = multi_ch_no[com_mode][0] = multi_ch_no[com_mode][1] = multi_ch_no[com_mode][2] = multi_ch_no[com_mode][3] = multi_ch_no[com_mode][4] = multi_ch_no[com_mode][5] = 0;
	switch (RX_buf[com_mode][4]){
	case '0':					//�V���O���p
		switch (RX_buf[com_mode][5]){
		case '1':	ch_no[com_mode] = 1;	break;
		case '2':	ch_no[com_mode] = 2;	break;
		case '3':	ch_no[com_mode] = 3;	break;
		case '4':	ch_no[com_mode] = 4;	break;
		case '5':	ch_no[com_mode] = 5;	break;
		case '6':	ch_no[com_mode] = 6;	break;
		default :	ch_no[com_mode] = 9;	break;	//CH�ԍ��w��G���[(�V���O��,����1-6�ȊO)
		}
		break;
	case 'X':					//�}���`�p
		switch (RX_buf[com_mode][5]){
		case 'X':	ch_no[com_mode] = 0;
			switch (RX_buf[com_mode][6]){
			case '0':
			/* ���������� */
				ch[0] = RX_buf[com_mode][7];	//�}���`CH������
				ch_z[com_mode] = ch[0] - 0x30;
				ch[0] = 0;
				if (ch_z[com_mode] >=1 && ch_z[com_mode] <=6){
					for (m = 0; m < ch_z[com_mode]; m++){
						ch[0] = RX_buf[com_mode][8 + 2*m];	//�}���`CH�ԍ�����(�擪)
						check = ch[0] - 0x30;
						ch[0] = 0;
						if (check != 0){ch_no[com_mode] = 9;}		//�}���`CH�ԍ��w��G���[(�擪0�ȊO)
						
						ch[0] = RX_buf[com_mode][9 + 2*m];	//�}���`CH�ԍ�����(����)
						check = ch[0] - 0x30;
						ch[0] = 0;
						if (check < 1 || check > 6){	//�ُ펞
							ch_no[com_mode] = 9;				//�}���`CH�ԍ��w��G���[(����1-6�ȊO)
						}else{												//���펞
							multi_ch_no[com_mode][m] = check;			//�}���`CH�ԍ���ێ�(1�`6)
						}
					}
					check = 0;
				}
				else {ch_no[com_mode] = 9;}					//�}���`CH���w��G���[(����1-6�ȊO)
				break;
			case ',':							//�_�E�����[�h�R�}���h�p(�V���O������ CHNo.=XX)
				if (RX_buf[com_mode][1] == 'd'){ch_no[com_mode] = 0;}
				else {ch_no[com_mode] = 9;}
				break;
			default:	ch_no[com_mode] = 9;	break;				//�}���`CH���w��G���[(�擪0�ȊO)
			}
			break;
		default:	ch_no[com_mode] = 9;	break;					//CH�ԍ��w��G���[(�}���`,����X�ȊO)
		}
		break;
	default:		ch_no[com_mode] = 9;	break;					//CH�ԍ��w��G���[(�擪0,X�ȊO)
	}
}
/************* �R�}���h�E�o���N�E�`���l������p�֐� *************/
/*                                                              */
/*     ����`�̃R�}���h�����͂��ꂽ�ꍇ�AEnd Code:16 ��Ԃ�     */
/*        �o���N���w��l�ƈقȂ�ꍇ�AEnd Code:04 ��Ԃ�        */
/*     ���݂��Ȃ��`���l�����w�肵���ꍇ�AEnd Code:04 ��Ԃ�     */
/*                                                              */
/*                  16:�D��x7  04�F�D��x8                     */
/*                                                              */
/****************************************************************/
void check_command(short com_mode){
	
	if( end_code[com_mode] != FRAMING_ERROR &&		//End Code 11
		end_code[com_mode] != PARITY_ERROR &&		//End Code 10
		end_code[com_mode] != OVERRUN_ERROR &&		//End Code 12
		end_code[com_mode] != CU_LENGTH_ERROR &&	//End Code 18
		end_code[com_mode] != FCS_ERROR)		//End Code 13
	{											//End Code 11,10,12,18,13�ȊO
		
		switch (RX_buf[com_mode][1]){		//�w�b�_�[�R�[�h�擪
		case 'R': /**** �Ǐo���n ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� �o���N���� = 0 */
			case 'g': // ���a�Ǐo��(Rg)
			case 'r': // �t���X�P�[���Ǐo��(Rr)
			case 'k': // �j�t�@�N�^�Ǐo��(Rk)
			case 'd': // �_���s���O�Ǐo��(Rd)
			case 'b': // �o�[���A�E�g�Ǐo��(Rb)
			case 'v': // ���S�x�W���Ǐo��(Rv)
			case 'h': // �G���[�z�[���h�^�C���Ǐo��(Rh)
			case 'u': // ���[�U�[���j�A���C�Y�Ǐo��(Ru)
			case 'R': // �t������l�Ǐo��(RR)
			case 'F': // �o�[�W�����l�Ǐo��(SFC9000�Ή�)(RF)
			case 'V': // �o�[�W�����l�Ǐo��(RV)
			case 'G': // ���O�f�[�^�Ǐo��(SFC9000�Ή�)(RG)
			case 'L': // ���O�f�[�^�Ǐo��(RL)
			case 'y': // ��t���j�A���C�Y���[�h�Ǐo��(Ry)
			case 'M': // �����Ǐo��(RM)
			case 'f': // �t�B���^�ݒ�Ǐo��(Rf)
			case 't': // �ώZ�ڕW�l�Ǐo��(Rt)
			case 'T': // �ώZ�I�t�Z�b�g�l�Ǐo��(RT)
			case 'p': // �Z���T�p���X�ݒ�Ǐo��(SFC014E�݊�)(Rp)
			case 'n': // �Z���T�V���A���i���o�[�Ǐo��(SFC014E�݊�)(Rn)
			case 'a': // �Z���T��񏑍���(�]���p)(Ra)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �V���O���R�}���h�̂� �o���N���� = 1 */
			case 'm': // ���[�J�[���j�A���C�Y�Ǐo��(Rm)
			case '1': // ���[�J�[�ݒ�Ǐo��(R1)
			case '2': // �f�o�b�N���[�h�ݒ�Ǐo��(R2)
			case '3': // ����f�[�^�Ǐo��(R3)
			case '4': // ����g�`�Ǐo��(R4)
			case '5': // ���[�J�[�ݒ�Ǐo��(SFC9000�Ή�)(R5)
			case '6': // �f�o�b�N���[�h�ݒ�Ǐo��(SFC9000�Ή�)(R6)
			case '7': // ����f�[�^�Ǐo��(SFC9000�Ή�)(R7)
			case '8': // ����g�`�Ǐo��(SFC9000�Ή�)(R8)
			case '9': // �p���X�o��(R9)
			case 'X': // SW��ԃ`�F�b�N(RX)
			case 'E': // �g�`�ُ픻��l�ݒ�Ǐo��(RE)
			case 'A': // �A�b�e�l�[�^�Q�C���Ǐo��(RA)
			case 'i': // �������[�h�f�[�^�Ǐo��(Ri)
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'D': // �[�������f�[�^�Ǐo��(SFC9000�Ή�)(RD)
			case 'Z': // �[�������f�[�^�Ǐo��(RZ)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �V���O���E�}���`�Ή�  �o���N���� = 0 */
			case 'S': // �X�e�[�^�X�Ǐo��(SFC9000�Ή�)(RS)
			case 's': // �X�e�[�^�X�Ǐo��(Rs)
			case 'l': // ���[�J�b�g�Ǐo��(Rl)
			case 'z': // �[���_������ԓǏo��(Rz)
			case 'W': // �g�`��ԓǏo��(RW)
			if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
			case 'C': // ����f�[�^�Ǐo��(SFC9000�Ή��ER7�R�}���h�Z�k��)(RC)
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;			
			case 'c': // �f�W�^���t�B���^�W���ǂݏo��
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;		
		/* ����`�R�}���h */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}	
			break;
		case 'W': /**** �����݌n ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� �o���N���� = 0 */
			case 'g': // ���a������(Wg)
			case 'r': // �t���X�P�[��������(Wr)
			case 'k': // �j�t�@�N�^������(Wk)
			case 'd': // �_���s���O������(Wd)
			case 'l': // ���[�J�b�g������(Wl)
			case 'b': // �o�[���A�E�g������(Wb)
			case 'v': // ���S�x�W��������(Wv)
			case 'h': // �G���[�z�[���h�^�C��������(Wh)
			case 'R': // �t������l������(WR)
			case 'u': // ���[�U�[���j�A���C�Y������(Wu)
			case 'U': // ���[�U�[���j�A���C�Y�ؑւ�(WU)
			case 'y': // ��t���j�A���C�Y���[�h������(Wy)	
			case 'f': // �t�B���^�ݒ菑����(Wf)
			case 't': // �ώZ�ڕW�l������(Wt)
			case 'T': // �ώZ�I�t�Z�b�g�l�Ǐo��(WT)
			case 'p': // �Z���T�p���X�ݒ菑����(SFC014E�݊�)(Wp)
			case 'n': // �Z���T�V���A���i���o�[������(SFC014E�݊�)(Wn)
			case 'a': // �Z���T��񏑍���(�]���p)(Wa)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �V���O���R�}���h�̂� �o���N���� = 1 */
			case 'm': // ���[�J�[���j�A���C�Y������(Wm)
			case '1': // ���[�J�[�ݒ菑����(W1)
			case '2': // �f�o�b�N���[�h�ݒ菑����(W2)
			case '5': // ���[�J�[�ݒ菑����(SFC9000�Ή�)(W5)
			case '6': // �f�o�b�N���[�h�ݒ菑����(SFC9000�Ή�)(W6)
			case '7': // �[�������ڍ׃f�[�^������(W7)
			case '9': // �p���X�o��(W9)
			case 'E': // �g�`�ُ픻��l�ݒ菑����(WE)
			case 'A': // �A�b�e�l�[�^�Q�C��������(WA)
			case 'i': // �������[�h������(Wi)
				if (RX_buf[com_mode][3] == '1' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'P': // �p�����[�^�Z�b�g������(WP)
			case 'Z': // �[�������f�[�^������(WZ)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �V���O���E�}���`�Ή� */
			case 'z': // �[�������f�[�^������(SFC9000�Ή�)(Wz)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
			case 'c': // �f�W�^���t�B���^�W����������
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 0 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;		
		/* ����`�R�}���h */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'O': /**** ��ɓ���w�� ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� */
			case 'V': // �ώZ�l�Ǐo��(OV)
			case 'Z': // �[���������s(OZ)
			case 'S': // �ݒ�l�ۑ�(OS)
			case 's': // �Z���T�������ݒ�l�ۑ�(Os)
			case 'J': // �[�������l�ۑ�(OJ)
			case 'C': // �A���[�����Z�b�g(OC)
			case 'L': // ���O�N���A(OL)
			case 'R': // �q�`�l�N���A(OR)
			case 'T': // �k�d�c�S�_��(OT)
			case 'A': // �A���[�������o��(OA)
			case 't': // �ώZ���Z�b�g(Ot)
#if defined(FRQSCH)
			case 'F': // �����[�������{(OF)
#endif
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'E': // �u������1�_�Ǐo��(OE)
			case 'Q': // �u������10�_�Ǐo��(OQ)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* ����`�R�}���h */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'o': /**** oq,ov�R�}���h�p ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� */
			case 'v': // �ώZ�l�Ǐo���E�ώZ���펞�X�V(ov)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'q': // �u������10�_�Ǐo���E���[�J�b�g�I�t(oq)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* ����`�R�}���h */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'd': /**** �_�E�����[�h�n ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� ��CHNo.��XX */
			case 'p': // �_�E�����[�h����(dp)
			case 't': // �_�E�����[�h�f�[�^�]��(dt)
			case 'w': // �_�E�����[�h�f�[�^������(dw)
			case 'c': // �_�E�����[�h���f(dc)
			case 'r': // �_�E�����[�h�ナ�Z�b�g(dr)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] == 0){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* ����`�R�}���h */
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
		case 'E': /**** �ċN�� ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� */
			case 'R': // ���Z�b�g�R�}���h(ER)
				if (RX_buf[com_mode][3] == '0' && ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){ }
				else { error_check[com_mode]++; end_code[com_mode] = ADDRESS_OVER;}
				break;
		/* ����`�R�}���h */
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
#ifdef MEMDBG
        case 'M':
            error_check[com_mode]++;
            error_check[com_mode]--;
            break;
#endif
		default:  /**** ����`�R�}���h ****/
			error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
		}
	}	//�R�}���h�E�o���N�E�`���l������I��
	
}
/*********************** �t�H�[�}�b�g���� ***********************/
/*                                                              */
/* �R�}���h�̏����Ɍ�肪����ꍇ�AEnd Code:14 ��Ԃ� (�D��x9) */  // �����݌n�̓R�}���h��͂Ŕ���i�}���`CH������ȊO�j
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
	{ 																	//End Code 11,10,12,18,13,16,04�ȊO�̏ꍇ
	/* CUnet���b�Z�[�W���� */
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
	
		l = strlen(RX_buf[com_mode]);			//RX_buf�f�[�^�� ��(FCS,CR�폜��)

	/* �擪��������(@) */
		letter = RX_buf[com_mode][0];
		if (letter != 0x40){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}		
	/* �w�b�_�[�R�[�h���� */
		switch (RX_buf[com_mode][1]){			//�w�b�_�[�R�[�h�擪
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
		case 'R': /**** �Ǐo���n ****/
			switch (RX_buf[com_mode][2]){		//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� �f�[�^��:6 */
			case 'g': // ���a�Ǐo��(Rg)
			case 'r': // �t���X�P�[���Ǐo��(Rr)
			case 'k': // �j�t�@�N�^�Ǐo��(Rk)
			case 'd': // �_���s���O�Ǐo��(Rd)
			case 'b': // �o�[���A�E�g�Ǐo��(Rb)
			case 'v': // ���S�x�W���Ǐo��(Rv)
			case 'h': // �G���[�z�[���h�^�C���Ǐo��(Rh)
			case 'u': // ���[�U�[���j�A���C�Y�Ǐo��(Ru)
			case 'm': // ���[�J�[���j�A���C�Y�Ǐo��(Rm)
			case 'R': // �t������l�Ǐo��(RR)
			case 'F': // �o�[�W�����l�Ǐo��(RF)(SFC9000�Ή�)
			case 'V': // �o�[�W�����l�Ǐo��(RV)
			case 'E': // �g�`�ُ픻��l�ݒ�Ǐo��(RE)
			case 'A': // �A�b�e�l�[�^�Q�C���Ǐo��(RA)
			case 'y': // ��t���j�A���C�Y���[�h�Ǐo��(Ry)
			case 'M': // �����Ǐo��(RM)
			case 'f': // �t�B���^�ݒ�Ǐo��(Rf)
			case 't': // �ώZ�ڕW�l�Ǐo��(Rt)
			case 'T': // �ώZ�I�t�Z�b�g�l�Ǐo��(RT)
			case '1': // ���[�J�[�ݒ�Ǐo��(R1)
			case '2': // �f�o�b�N���[�h�ݒ�Ǐo��(R2)
			case '5': // ���[�J�[�ݒ�Ǐo��(SFC9000�Ή�)(R5)
			case '6': // �f�o�b�N���[�h�ݒ�Ǐo��(SFC9000�Ή�)(R6)
			case '9': // �p���X�o��(R9)
			case 'i': // �������[�h�f�[�^�Ǐo��(Ri)
			case 'X': // SW��ԃ`�F�b�N(RX)
			case 'p': // �Z���T�p���X�Ǐo��(SFC014E�݊�)(Rp)
			case 'n': // �V���A���i���o�[�Ǐo��(SFC014E�݊�)(Rn)
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���R�}���h�̂� �f�[�^��:8 */
			case 'G': // ���O�f�[�^�Ǐo��(SFC9000�Ή�)(RG)
			case 'L': // ���O�f�[�^�Ǐo��(RL)
				if (l != 8){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���R�}���h�̂� �f�[�^��:9 */
			case '3': // ����f�[�^�Ǐo��(R3)
				if (l != 9){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���R�}���h�̂� �f�[�^��:9or10 */
			case '4': // ����g�`�Ǐo��(R4)
				if (l != 9 && l != 10){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���R�}���h�̂� �f�[�^��:11 */
			case '7': // ����f�[�^�Ǐo��(SFC9000�Ή�)(R7)
				if (l != 11){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���R�}���h�̂� �f�[�^��:11or12 */
			case '8': // ����g�`�Ǐo��(SFC9000�Ή�)(R8)
				if (l != 11 && l != 12){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'D': // �[�������f�[�^�Ǐo��(SFC9000�Ή�)(RD)
			case 'Z': // �[�������f�[�^�Ǐo��(RZ)
				if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}	//ch_z: �}���`CH��
				break;
		/* �V���O���E�}���`�Ή� */
			case 'S': // �X�e�[�^�X�Ǐo��(SFC9000�Ή�)(RS)
			case 's': // �X�e�[�^�X�Ǐo��(Rs)
			case 'l': // ���[�J�b�g�Ǐo��(Rl)
			case 'z': // �[���_������ԓǏo��(Rz)
			case 'W': // �g�`��ԓǏo��(RW)
			case 'C': // ����f�[�^�Ǐo��(SFC9000�Ή��ER7�R�}���h�Z�k��)(RC)
				if (ch_no[com_mode] >= 1 && ch_no[com_mode] <= 6){		//�V���O���R�}���h
					if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				}
				else {					//�}���`�R�}���h
					if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				}
				break;
			case 'c': // �f�W�^���t�B���^�W���ǂݏo��
				if ((l != 9) && (l != 10)){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			}	
			break;
		case 'W': /***** �����݌n *****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� ������ʔ��� */
			case 'r': // �t���X�P�[��������(Wr)
			case 'k': // �j�t�@�N�^������(Wk)
			case 'd': // �_���s���O������(Wd)
			case 'l': // ���[�J�b�g������(Wl)
			case 'b': // �o�[���A�E�g������(Wb)
			case 'v': // ���S�x�W��������(Wv)
			case 'h': // �G���[�z�[���h�^�C��������(Wh)
			case 'R': // �t������l������(WR)
			case 'E': // �g�`�ُ픻��l�ݒ菑����(WE)
			case 'A': // �A�b�e�l�[�^�Q�C��������(WA)
			case 'y': // ��t���j�A���C�Y���[�h������(Wy)
			case 'f': // �t�B���^�ݒ菑����(Wf)
			case 't': // �ώZ�ڕW�l������(Wt)
			case 'T': // �ώZ�I�t�Z�b�g�l�Ǐo��(WT)
			case 'u': // ���[�U�[���j�A���C�Y������(Wu)
			case 'U': // ���[�U�[���j�A���C�Y�ؑւ�(WU)
			case 'm': // ���[�J�[���j�A���C�Y������(Wm)
			case '2': // �f�o�b�N���[�h�ݒ菑����(W2)
			case '6': // �f�o�b�N���[�h�ݒ菑����(SFC9000�Ή�)(W6)
			case '9': // �p���X�o��(W9)
			case 'i': // �������[�h������(Wi)
			case 'g': // ���a������(Wg)
			case 'p': // �Z���T�p���X�ݒ菑����(SFC014E�݊�)(Wp)
				letter_judge(com_mode,l,0);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case '1': // ���[�J�[�ݒ菑����(W1)
				letter_judge(com_mode,l,2);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;			
			case '5': // ���[�J�[�ݒ菑����(SFC9000�Ή�)(W5)
				letter_judge(com_mode,l,1);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case '7': // �[�������ڍ׃f�[�^������(SFC9000�Ή�)(W7)
				letter_judge(com_mode,l,3);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			case 'n': // �V���A���i���o�[������(SFC014E�݊�)(Wn)
				letter_judge(com_mode,l,4);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;

		/* �}���`�R�}���h�̂� */
			case 'P': // �p�����[�^�Z�b�g������(WP)
			case 'Z': // �[�������f�[�^������(WZ):������}���`�݂̂Ɉړ�
				letter_judge(com_mode,l,0);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				l = 8 + 2*ch_z[com_mode];
				if (RX_buf[com_mode][l] != ','){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���E�}���`�Ή� */
			case 'z': // �[�������f�[�^������(SFC9000�Ή�)(Wz)		
				letter_judge(com_mode,l,0);
				if (l_check[com_mode] > 0){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				if (ch_no[com_mode] == 0){	//�}���`�R�}���h
					l = 8 + 2*ch_z[com_mode];
					if (RX_buf[com_mode][l] != ','){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				}
				break;
				
			//�f�W�^���t�B���^�W����������
			case 'c':
				break;

			}
			break;
		case 'O': /**** ��ɓ���w�� ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� */
			case 'V': // �ώZ�l�Ǐo��(OV)
			case 'Z': // �[���������s(OZ)
			case 'S': // �ݒ�l�ۑ�(OS)
			case 'J': // �[�������l�ۑ�(OJ)
			case 'C': // �A���[�����Z�b�g(OC)
			case 'L': // ���O�N���A(OL)
			case 'R': // �q�`�l�N���A(OR)
			case 'T': // �k�d�c�S�_��(OT)
			case 'A': // �A���[�������o��(OA)
			case 't': // �ώZ���Z�b�g(Ot)
#if defined(FRQSCH)
			case 'F': // �����[�������{(OF)
#endif
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'E': // �u������1�_�Ǐo��(OE)
			case 'Q': // �u������10�_�Ǐo��(OQ)
				if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}	//ch_z: �}���`CH��
				break;
			}
			break;
		case 'o': /**** oq,ov�R�}���h�p ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� */
			case 'v': // �ώZ�l�Ǐo���E�ώZ���펞�X�V(ov)
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �}���`�R�}���h�̂� */
			case 'q': // �u������10�_�Ǐo���E���[�J�b�g�I�t(oq)
				if (l != 8 + 2*ch_z[com_mode]){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}	//ch_z: �}���`CH��
				break;
			}
			break;
		case 'd': /**** �_�E�����[�h�n **** �_�E�����[�h����(dp)�̂݃f�[�^�����ϓ����邽�߁A�R�}���h��͂Ōʔ��� ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� �f�[�^��:544 */
			case 't': // �_�E�����[�h�f�[�^�]��(dt)
				if (l != 544){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
		/* �V���O���R�}���h�̂� �f�[�^��:12 */
			case 'w': // �_�E�����[�h�f�[�^������(dw)
			case 'c': // �_�E�����[�h���f(dc)
			case 'r': // �_�E�����[�h�ナ�Z�b�g(dr)
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
				//��������̓��͔���
				// if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			default: error_check[com_mode]++; end_code[com_mode] = COMMAND_ERROR;
			}
			break;
		case 'E': /**** �ċN�� ****/
			switch (RX_buf[com_mode][2]){	//�w�b�_�[�R�[�h����
		/* �V���O���R�}���h�̂� */
			case 'R': // ���Z�b�g�R�}���h(ER)
				if (l != 6){ error_check[com_mode]++; end_code[com_mode] = FORMAT_ERROR;}
				break;
			}
			break;
		}
	}	//�t�H�[�}�b�g����I��

}

/************************************************/
/*    CUnet �f�[�^���ُ� (End Code 15)    */
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
/*    WDT�ċN���� (1��̂� End Code 33)    */
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
	/* EEPROM�����ݒ��@���s�s�� */	
	if(action_status_check(ch_check) == ACT_STS_WRITE){
		if (enable_23 != 0){
			end_code[com_mode] = EEPROMWRITE_NOW;
			error_check[com_mode]++;
		}
	}
	/* �_�E�����[�h���@���s�s�� */
	if(action_status_check(ch_check) == ACT_STS_DLOAD){
		if (enable_22 != 0){
			end_code[com_mode] = DOWNLOAD_NOW;
			error_check[com_mode]++;
			}
		}
	/* �ώZ���@���s�s�� */
	if(action_status_check(ch_check) == ACT_STS_ADDIT){
		if (enable_21 != 0){
			end_code[com_mode] = INTEGRATE_NOW;
			error_check[com_mode]++;
		}
	}
	/* �[���������@���s�s�� */
	if(action_status_check(ch_check) == ACT_STS_ZERO){
		if (enable_20 != 0){
			end_code[com_mode] = ZEROADJUST_NOW;
			error_check[com_mode]++;
		}
	}
}

/************************************************/
/*    ���s�s�� (End Code 20�`23)    */
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
		/* 20 �[�������� ���s�FR*,OE,OQ,oq */
		/* 21 �ώZ���s�� ���s�FR*,OE,OQ,oq,OV,ov */
		/* 22 �_�E�����[�h�����s�F RS,Rs */
		/* 23 �����ݒ����s�F RS,Rs,OE,OQ,oq */
		
		switch (RX_buf[com_mode][1]){
		case 'R':
			switch (RX_buf[com_mode][2]){
			/* 20�`23 ���s�� */
			case 'S':
			case 's':
				break;
			/* 22,23 ���s�s�� */
			default:
				enable_22++;
				enable_23++;
				break;
			}
			break;
		case 'O':			
			switch (RX_buf[com_mode][2]){
			/* 22 ���s�s�� */
			case 'E':
			case 'Q':
				enable_22++;
				break;			
			/* 20,22,23 ���s�s�� */
			case 'V':
				enable_20++;
				enable_22++;
				enable_23++;
				break;
			/* 20�`23 ���s�s�� */			
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
			/* 22 ���s�s�� */
			case 'q':
				enable_22++;
				break;			
			/* 20,22,23 ���s�s�� */
			case 'v':
				enable_20++;
				enable_22++;
				enable_23++;
				break;
			}
			break;
		/* 20�`23 ���s�s�� W*�R�}���h�͑S�ĊY��*/
		default:
			enable_20++;
			enable_21++;
			enable_22++;
			enable_23++;
			break;
		}
		
		if (ch_no[com_mode] == 0){		//�}���`�R�}���h
			for(cnt = 0; cnt < ch_z[com_mode]; cnt++){		//�}���`CH��(1�`6)�����s�s���`�F�b�N����
				ch_check = multi_ch_no[com_mode][cnt] - 1;
				JudgeErrCode21_23(com_mode, ch_check, enable_20, enable_21, enable_22, enable_23);
			}
		}else{				//�V���O���R�}���h
			ch_check = ch_no[com_mode] -1;
			JudgeErrCode21_23(com_mode, ch_check, enable_20, enable_21, enable_22, enable_23);
		}
	}
}

/************************************************/
/*    ���s�� (�����Q�FEnd Code 32)    */
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
		if(ch_no[com_mode] == 0){	//�}���`�R�}���h�F�SCH�G���[����
			mes_err_cnt = 0;
			for (ch = 0; ch < 6; ch++){
				if (MAIN[ch].com_err_status != 0) mes_err_cnt++;	
			}
			if (mes_err_cnt != 0) mes_err[0] = 1;
		}	
		else{				//�V���O���R�}���h�F�Y��CH�̂݃G���[����
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
	// W : �R�}���h�n��(R=0, W=1, O=2, M=3)
	// X : 0=�V���O��/1=�}���`�R�}���h
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

	//��M���b�Z�[�W������
	if(BufLen < 9)
	{
		Flg |= 0x01;
	}
	else
	{
		//FCS����
		ClcFcs = CalcFcs(Buf, BufLen - 3); //FCS(2byte), CR(1byte)
		BufFcs = GetBufToVal(Buf[BufLen - 2], Buf[BufLen - 1]);
		if(ClcFcs != BufFcs)
		{
			Flg |= 0x02;
		}

		//�R�}���h�L������
		Flg |= JudgeUsableCommand(Buf, CmdIndex, BankNo, MultiFlg);

		//�R�}���h�����݂���
		if(Flg == 0)
		{
			//CH�ԍ�����
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
/*    �ʐM�v���g�R������					    */
/************************************************/
void protocol(short com_mode)
{
#if 1
	//1. �t�H�[�}�b�g����
	// 1-1. �R�}���h������
	// 1-2. FCS����
	// 1-3. �R�}���h�E�o���N�E�`���l������
	// 1-4. �e�R�}���h�t�H�[�}�b�g����
	//2. �E�H�b�`�h�b�O�ċN������
	//3. �e�R�}���h����
	// 3-1. Endcode20-23����
	// 3-2. �����Q����
	// 3-3. �R�}���h���&�ԑ��o�b�t�@�쐬
	//4. ���M
#else
	short i;
	short h;
	short l;
	short z;
	short  fcs;				// FCS�v�Z�p
	short fcs_cal;
	long fcs_d;				// FCS����p
	char fcs_c[5] ={0};	// FCS�i�[�p
	char comma[5] = ",";	// ','����p
	char add[20] = {0};
	short enter_download_mode = 0;

	for (i=0; i<MSG_MAX; i++){
		TX_buf[com_mode][i] = 0;	//���M�o�b�t�@�N���A
	}

	error_check[com_mode] = 0;
	echo_back[com_mode] = 0;
	end_code[com_mode] = 0;
	flow_check[com_mode] = 0;
	
	/************************* �f�[�^������ *************************/
	/*  �f�[�^�����ŏ���9�o�C�g�����̏ꍇ = �G�R�[�o�b�N (�D��x1)  */
	/****************************************************************/
	l = strlen(RX_buf[com_mode]);

	if (l < 9){					//�^�[�~�l�[�^=CR�܂�
		error_check[com_mode]++; echo_back[com_mode]++;
		for (i=0; i<=7; i++){
			TX_buf[com_mode][i] = RX_buf[com_mode][i];		//��M�f�[�^��ԐM�p��ɂ��̂܂܊i�[
		}
	}
	/*************************** ����I�� ***************************/
	else {
		/* FCS����p���� */
		fcs_d = GetBufFcs(RX_buf[com_mode][l - 3], RX_buf[com_mode][l - 2]);
	
		/* ���ʁF�ԐM�p�񏀔��iEnd Code������܂ō쐬�j*/
		RX_buf[com_mode][l-1] = 0;		//CR �폜
		RX_buf[com_mode][l-2] = 0;		//FCS�폜
		RX_buf[com_mode][l-3] = 0;
	
		for (i=0; i<=5; i++){
			TX_buf[com_mode][i] = RX_buf[com_mode][i];		//@����CHNo.�܂ŕԐM�p��Ɋi�[
		}
		/* ���ʁF�`���l���ԍ��Ǐo���E�`���l������p���� */
		check_channel(com_mode);
		
		/* �I�[�o�[�����G���[ (�D��x4) */
		if ((err_stat & 0x0004) == 0x0004){ error_check[com_mode]++; end_code[com_mode] = OVERRUN_ERROR;}		
		/* �p���e�B�G���[ (�D��x3) */
		if ((err_stat & 0x0001) == 0x0001){ error_check[com_mode]++; end_code[com_mode] = PARITY_ERROR;}
		/* �t���[�~���O�G���[ (�D��x2) */
		if ((err_stat & 0x0002) == 0x0002){ error_check[com_mode]++; end_code[com_mode] = FRAMING_ERROR;}
		
		/* CUnet�f�[�^���G���[ (�D��x5) */
		if(com_type == COM_CUNET)	check_cu_length(com_mode);
		
		/************************** �e�b�r���� **************************/
		/*     �e�b�r����v���Ȃ��ꍇ�AEnd Code:13 ��Ԃ� (�D��x6)     */
		/****************************************************************/
		if (fcs != fcs_d){ error_check[com_mode]++; end_code[com_mode] = FCS_ERROR;}
		fcs_c[0] = fcs_c[1] = 0;		//������

		/* �R�}���h�E�o���N�E�`���l������ */
		check_command(com_mode);
	
		/* �t�H�[�}�b�g���� */
		check_format(com_mode);
		
		/* ���̑��G���[(WDT�ċN���E�蓮�d���f���o) */
		check_wdt(com_mode);
		
		/* ���s�s�G���[ */
		check_act_disable(com_mode);
		
		/* ���s�G���[ */
		check_act_able(com_mode);

	}//else(�f�[�^�����萳��E�G�R�[�o�b�N�敪���p)

	if (error_check[com_mode] == 0){
		/************************* �R�}���h��� *************************/

		/*********************** �R�}���h��͊J�n ***********************/
		switch (RX_buf[com_mode][1]){          // �w�b�_�[�R�[�h�擪
			/*************** �f�[�^�Ǐo�� ***************/
			case 'R':
				TX_buf[com_mode][6] = 0x30;
				TX_buf[com_mode][7] = 0x30;
				
				if (ch_no[com_mode] >=1 && ch_no[com_mode] <=6){
					TX_buf[com_mode][8] = ',';	//�V���O���̂�End Code�����','
				}
		
				read_command_R(com_mode);
		
				strncat(TX_buf[com_mode],comma,sizeof(comma));
				
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//�����Q
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM��Q
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode]/ 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;

				end_code[com_mode] = 99;			//�ԐM�p��쐬�����p
				break;
		
			/*************** �f�[�^������ ***************/
			case 'W':
				if( MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				    MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
					read_command_W(com_mode);
				}
				break;
		
			/******* ����w���{���ʁE�ώZ�l�Ǐo�� *******/
			case 'O':
				TX_buf[com_mode][6] = 0x30;
				TX_buf[com_mode][7] = 0x30;
				
				switch (RX_buf[com_mode][2]){
					/* �ώZ�l�Ǐo��(OV) */			
					case 'V':
						command_OV(com_mode);
						break;
					/* �u������1�_�Ǐo��(OE) */
					case 'E':
						command_OE(com_mode);
						break;
					/* �u������10�_�Ǐo��(OQ) */
					case 'Q':
						command_OQ(com_mode);
						break;
					/* �[���������s(OZ) */			
					case 'Z':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				    		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OZ(com_mode);
//							SAVE[ch_no[com_mode] -1].control |= 0x0001;
						}
						break;
#if defined(FRQSCH)
					case 'F': // �����[�������{(OF)
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
							MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OF(com_mode);
						}
						break;
#endif
					/* �ݒ�l�ۑ�(OS) */
					case 'S':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				  		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OS(com_mode);
						}
						break;
					/* �Z���T�������ݒ�l�ۑ�(Os) */
					case 's':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				  		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_Os(com_mode);
						}
						break;
					/* �[�������l�ۑ�(OJ) */
					case 'J':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				  		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OJ(com_mode);
						}
						break;						
					/* �A���[�����Z�b�g(OC) */
					case 'C':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				   	  	   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							SAVE[ch_no[com_mode] -1].control |= 0x0002;
						}
						break;
					/* ���O�N���A(OL) */	
					case 'L':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				   		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){
							command_OL(com_mode);
						}
						break;
					/* �q�`�l�N���A(OR) */
					case 'R':
						if(MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_NORMAL ||
				   		   MAIN[ch_no[com_mode] -1].com_act_status == ACT_STS_TEST	){					
							command_OR();
						}
						break;
					/* �k�d�c�S�_��(OT) */
					case 'T':
						command_OT();
						break;
					/* �A���[�������o��(OA) */
					case 'A':
						command_OA();	
					break;
					/* �ώZ�l���Z�b�g(Ot) */	
					case 't':
						command_Ot(com_mode);
						break;
					default:
						break;
				}
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1) end_code[com_mode] = MEASURE_ERROR;	//�����Q
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM��Q
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
				
				end_code[com_mode] = 99;			//�ԐM�p��쐬�����p
				break;			
#ifdef MEMDBG
			/*************** Debug������ ***************/
			case 'M':
				
				MemoryCom(com_mode);
		
				strncat(TX_buf[com_mode],comma,sizeof(comma));
				
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//�����Q
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM��Q
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
				
				end_code[com_mode] = 99;			//�ԐM�p��쐬�����p
				break;
					
#endif
			/******* ����w���{���ʁE�ώZ�l�Ǐo�� *******/
			case 'o':
				TX_buf[com_mode][6] = 0x30;
				TX_buf[com_mode][7] = 0x30;
				
				switch (RX_buf[com_mode][2]){
					/* �u������10�_�Ǐo���E���[�J�b�g�I�t(oq) */
					case 'q':
						command_oq(com_mode);
						break;			
					/* �ώZ�l�Ǐo���E�ώZ���펞�X�V(ov) */
					case 'v':
						command_ov(com_mode);
						break;
					break;
				}				
				if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1) end_code[com_mode] = MEASURE_ERROR;	//�����Q
				if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM��Q
					end_code[com_mode] = EEPROM_ERROR;
					eep_err = 0;
				}

				TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
				TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
				
				end_code[com_mode] = 99;			//�ԐM�p��쐬�����p
				break;
				
			/*************** �_�E�����[�h ***************/
			case 'D':
				switch (RX_buf[com_mode][2]) {
					case 'L':
						TX_buf[com_mode][6] = 0x30;
						TX_buf[com_mode][7] = 0x30;
						
						if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//�����Q
						if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM��Q
							end_code[com_mode] = EEPROM_ERROR;
							eep_err = 0;
						}

						TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
						TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
						
						end_code[com_mode] = 99;			//�ԐM�p��쐬�����p
						enter_download_mode = 1;
						break;
					case 'l': //FpgaDownload
						TX_buf[com_mode][6] = end_code[com_mode] / 10 + 0x30;
						TX_buf[com_mode][7] = end_code[com_mode] % 10 + 0x30;
						
						command_Dl(com_mode);
						end_code[com_mode] = 99;			//�ԐM�p��쐬�����p
						break;
				}
				break;

			/*************** ���Z�b�g(ER) ***************/
			case 'E':
				switch (RX_buf[com_mode][2]){
					/* ���Z�b�g�R�}���h(ER) */
					case 'R':
						SAVE[ch_no[com_mode] -1].control |= 0x0008;
						end_code[com_mode] = REBOOT_MANUAL;
						break;
				}
				break;

		}//switch RX_buf[1]

	}//error_check[com_mode] == 0
	
	if (echo_back[com_mode] == 0){	
		/*************** End Code��� ***************/
		if (end_code[com_mode] != 99){		//W*�R�}���h�p
			if(end_code[com_mode] == 0 && mes_err[ch_no[com_mode]] == 1)	end_code[com_mode] = MEASURE_ERROR;	//�����Q
			if(end_code[com_mode] == 0 && eep_err == 1){			//EEPROM��Q
				end_code[com_mode] = EEPROM_ERROR;
				eep_err = 0;
			}
			
			add[0] = end_code[com_mode] / 10 + 0x30;
			add[1] = end_code[com_mode] % 10 + 0x30;
			strncat(TX_buf[com_mode],add,sizeof(add));
			/************* �����������ԐM�� *************/
			for (i=6; i<=400; i++){TX_buf[com_mode][i+2] = RX_buf[com_mode][i];}
		}
		end_code[com_mode] = 0;				//EndCode������
		/*************** �e�b�r�Čv�Z ***************/
		if (flow_check[com_mode] == 0)		{l = strlen(TX_buf[com_mode]);}
		else if (flow_check[com_mode] == 1)	{l = 10 +  9*ch_z[com_mode];}
		else if (flow_check[com_mode] == 2)	{l = 10 + 72*ch_z[com_mode];}

		for(h = l; h < MSG_MAX; h++){TX_buf[com_mode][h]= 0;}	//TEXT���ȍ~ �z�񏉊���
		// fcs = TX_buf[com_mode][0];		//=@(0x40)
		// z = 1;
		// while (z != l){
		// 	fcs ^= TX_buf[com_mode][z];
		// 	z++;
		// }
		fcs = CalcFcs(TX_buf[com_mode], l);
		
		fcs_cal = fcs / 0x10;
		
		if (fcs_cal >= 0 && fcs_cal <=9)	fcs_c[0] = fcs_cal  + 0x30;	// 0x30:ASCII��16�i�����ϊ�0-9
		else	/*A-F*/							fcs_c[0] = fcs_cal  + 0x37;	// 0x37:ASCII��16�i�����ϊ�A-F
		
		fcs_cal = fcs % 0x10;

		if (fcs_cal >= 0 && fcs_cal <=9)	fcs_c[1] = fcs_cal  + 0x30;	// 0x30:ASCII��16�i�����ϊ�0-9
		else	/*A-F*/							fcs_c[1] = fcs_cal  + 0x37;	// 0x37:ASCII��16�i�����ϊ�A-F		
		
		
		if(flow_check[com_mode] == 0){
			strcat(TX_buf[com_mode],fcs_c);		//TX_buf�Ō���ɒǉ�
		}
		else{
			TX_buf[com_mode][l]   = fcs_c[0];
			TX_buf[com_mode][l+1] = fcs_c[1];
		}	
		/*************** �^�[�~�l�[�^ ***************/
		TX_buf[com_mode][l+2] = CR;			//=CR(0x0d)
	}//echo_back[com_mode] != 0
	error_check[com_mode] = 0; echo_back[com_mode] = 0;	flow_check[com_mode] = 0;	//������
	/* ���M */
	l = l+3;
//host
	if (com_mode == HOST || com_mode == SUB_HOST){
		if(com_type == COM_RS485){
			TX_start_host(l);				//���b�Z�[�W���M�J�n(RS485�ʐM)

		}else{
			mky43_TX_start(com_mode);		//���b�Z�[�W���M�J�n(CUnet�ʐM)
		}
	}
//ment
	else{	
		TX_start_ment(l);
	}
	if(enter_download_mode) {
		if(com_mode == MENT) {
			// GPIO_PD6
			while(__bit_input(GPIO_PORTD_BASE, 6) == 1);		//�Ō�̑��M���I���̂�҂�
		} else {
			if(com_type == COM_RS485){
				// GPIO_PP4
				while(__bit_input(GPIO_PORTP_BASE, 4) == 1);	//�Ō�̑��M���I���̂�҂�
			}
		}		
		download(com_mode, com_type);				//�t�@�[���E�F�A�_�E�����[�h
	}
#endif
}
