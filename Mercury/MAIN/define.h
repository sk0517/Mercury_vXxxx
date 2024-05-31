/***********************************************/
/* File Name : define.h  	         									   */
/*	Summary   : 固定定数 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

/*条件コンパイル*/
#define FPGADOWNLOAD //FPGAコンフィグデータをマイコンから読み出す
#define MEMDBG   //メモリ読み書き(MR/MWコマンド)機能
#define FRQSCH  //周波数サーチ機能

/*割込み回数(100msec)*/
#define T100MS 100/18 		/*1ch 18msec周期*/

/*相関演算個数（＊サンプリング時間）*/
//#define SUB_POINT 42		/*60以下（変更不可）*/
#define SUB_POINT 21		/*60以下（変更不可）*/

/*流量情報*/
//#define AD_BASE 2047
//#define AD_MAX 4095		/*12bit MAX*/
//#define GAIN_CONT_LOW 3700
// #define AD_BASE_UNIT 2047	/*中心線*/
#define AD_BASE_UNIT 4095	/*中心線*/
//#define AD_BASE 8188		/*4回加算分(2047*4)*/
// #define AD_MAX_UNIT 4095	/*12bit MAX*/
#define AD_MAX_UNIT 8191	/*12bit MAX*/
//#define AD_MAX 16380		/*4回加算分(4095*4)*/

//10bitデータ
// #define GAIN_INIT_LOW	196		/*初期化時の信号レベル*/
// #define GAIN_CONT_LOW	3700	/*ゲイン制御時の信号レベル*/
// #define GAIN_WAVE_LOW	1600	/*ゲイン制御時の信号レベル*/
// #define GAIN_WAVE_HI	2500	/*ゲイン制御時の信号レベル*/
// #define SONIC_WAVE_HI	3000	/*音速測定時の信号レベル*/
//13bitデータ
#define GAIN_INIT_LOW	392		/*初期化時の信号レベル*/
#define GAIN_CONT_LOW	7400	/*ゲイン制御時の信号レベル*/
#define GAIN_WAVE_LOW	3200	/*ゲイン制御時の信号レベル*/
#define GAIN_WAVE_HI	5000	/*ゲイン制御時の信号レベル*/
#define SONIC_WAVE_HI	6000	/*音速測定時の信号レベル*/

//#define GAIN_INIT_LOW	784		/*初期化時の信号レベル 4回加算分(196*4)*/
//#define GAIN_CONT_LOW	14800	/*ゲイン制御時の信号レベル 4回加算分(3700*4)*/
//#define GAIN_WAVE_LOW	6400	/*ゲイン制御時の信号レベル 4回加算分(1600*4)*/
//#define GAIN_WAVE_HI	10000	/*ゲイン制御時の信号レベル 4回加算分(2500*4)*/
//#define SONIC_WAVE_HI	12000	/*音速測定時の信号レベル 4回加算分(3000*4)*/

/*FPGA アドレス*/
#define FPGA_OFFSET (*(volatile unsigned short *)0x80000000) /* DP-RAM読出し開始オフセット(0-63) */ 
#define FPGA_FREQ   (*(volatile unsigned short *)0x80000002) /* 駆動周波数(0-490) */ 
#define FPGA_CLOCK  (*(volatile unsigned short *)0x80000004) /* ADC供給クロック(0-3) */ 
#define FPGA_PULSE  (*(volatile unsigned short *)0x80000006) /* 駆動パルス数(1-15) */ 
#define FPGA_START  (*(volatile unsigned short *)0x80000008) /* WINDOW開始時間(0-63) */ 
#define FPGA_END    (*(volatile unsigned short *)0x8000000A) /* WINDOW終了時間(0-63) */ 
#define FPGA_SYNC   (*(volatile unsigned short *)0x8000000C) /* 位相パルス制御(0-3) */ 
#define FPGA_ANALOG_FREQ  	(*(volatile unsigned short *)0x8000000E) /* アナログスイッチ周波数切替(0-1) */
#define FPGA_FILIN0_0	(*(volatile unsigned short *)0x80000010) /* デジタルフィルタ係数(入力側0) (符号1bit+整数17bit) */
#define FPGA_FILIN0_1	(*(volatile unsigned short *)0x80000012) /* デジタルフィルタ係数(入力側0) (符号1bit+整数17bit) */
#define FPGA_FILIN1_0	(*(volatile unsigned short *)0x80000014) /* デジタルフィルタ係数(入力側1) (符号1bit+整数17bit) */
#define FPGA_FILIN1_1	(*(volatile unsigned short *)0x80000016) /* デジタルフィルタ係数(入力側1) (符号1bit+整数17bit) */
#define FPGA_FILIN2_0	(*(volatile unsigned short *)0x80000018) /* デジタルフィルタ係数(入力側2) (符号1bit+整数17bit) */
#define FPGA_FILIN2_1	(*(volatile unsigned short *)0x8000001A) /* デジタルフィルタ係数(入力側2) (符号1bit+整数17bit) */
//<<reserve>>			(*(volatile unsigned short *)0x8000001C) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x8000001E) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x80000020) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x80000022) /* reserve */
#define FPGA_FIL_EN		(*(volatile unsigned short *)0x80000024) /* デジタルフィルタ有効・無効設定 (0-1) */
//<<reserve>>			(*(volatile unsigned short *)0x80000026) /* reserve */
#define FPGA_FILOUT1_0	(*(volatile unsigned short *)0x80000028) /* デジタルフィルタ係数(出力側1) (符号1bit+整数17bit) */
#define FPGA_FILOUT1_1	(*(volatile unsigned short *)0x8000002A) /* デジタルフィルタ係数(出力側1) (符号1bit+整数17bit) */
#define FPGA_FILOUT2_0	(*(volatile unsigned short *)0x8000002C) /* デジタルフィルタ係数(出力側2) (符号1bit+整数17bit) */
#define FPGA_FILOUT2_1	(*(volatile unsigned short *)0x8000002E) /* デジタルフィルタ係数(出力側2) (符号1bit+整数17bit) */
//<<reserve>>			(*(volatile unsigned short *)0x80000030) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x80000032) /* reserve */
#define FPGA_LED_CNT		(*(volatile unsigned short *)0x80000034) /* LED制御レジスタ */
#define FPGA_SW_DISP		(*(volatile unsigned short *)0x80000036) /* SW表示レジスタ */
#define FPGA_VERSION 		(*(volatile unsigned short *)0x8000003E) /* FPGAバージョン */ 

/*設定データ数*/
#define SV_NUM	491			/*SVD の総数*/
#define AMP_BUFF	60

/*EEPROM*/
#define SIZEOFMODBUSADDR 				0x400	/*CH1個分のアドレスサイズ*/
#define WRIT_CHECK (SIZEOFMODBUSADDR - 1)		/*EEPROM 書込みチェックアドレス*/

#define WR_USER_PARA  0x0001  /*ユーザパラメータ書込み*/
#define WR_USPAR_DEVICE  0x0002  /*ユーザパラメータ書込み(メモリデバイス)*/
#define WR_USER_LINR  0x0004  /*ユーザリニアライズ書込み*/
#define WR_MAKER_PARA 0x0008  /*メーカパラメータ書込み*/
#define WR_MAKER_LINR 0x0010  /*メーカリニアライズ書込み*/
#define WR_CVT_SERIAL 0x0020  /*変換器シリアルナンバー書込み*/
#define WR_SNS_SERIAL 0x0040  /*センサシリアルナンバー書込み*/

#define WR_ALL (WR_USER_PARA + WR_USPAR_DEVICE + WR_USER_LINR + WR_MAKER_PARA + WR_MAKER_LINR + WR_CVT_SERIAL + WR_SNS_SERIAL) /*全パラメータ書込み*/
#define WR_DEVICE (WR_USPAR_DEVICE + WR_USER_LINR + WR_SNS_SERIAL) /*メモリデバイス書込み*/

/*通信用*/
#define CR		0x0D		/*キャリッジリターン*/
#define LF		0x0A		/*ラインフィード*/

#define Dt		1			/*制御周期(10ms)1固定*/

/*ステータス情報*/
#define	B_OFF			0		//オフ状態
#define	B_ON				1		//オン状態
#define	B_BLINK				2		//反転
#define	B_BLANK				3		//空白

#define	B_NO				0		//No
#define	B_YES			1		//Yes

#define	B_OK				0		//OK
#define	B_NG				-1		//NG

#define	B_PLUS			1		//+方向
#define	B_MINUS			-1		//-方向

#define	B_POSI			1
#define	B_NEGA			2

#define	B_FORWARD		0
#define	B_REVERSE		1

/*ゼロ点調整情報*/
#define	SW_ZERO_START		30		//ゼロ点調整開始押下時間:3秒(100msec x 30回 = 3sec)
#define	ZERO_ADJ_TIME		20		//ゼロ点調整時間：2秒(100msec x 20回 = 2sec)
#define	VTH_ADJ_TIME		20		//Vth調整時間：2秒(100msec x 20回 = 2sec)
#define	WAVE_ADJ_TIME		300		//波形認識調整時間：3秒(100msec x 300回 = 30sec)

/*エラーしきい値*/
#define	LIM_OVERFLOW 100			/*オーバーフロー*/

/*エラーログ情報*/
#define	LOGMAX				10		//エラーログ保存数

/*エラーコード*/
#define	ERR_ZERO_EMPTY		1		//ゼロ調整エンプティセンサ
#define	ERR_ZERO_WAVE		2		//ゼロ調整波形減衰 （波形異常）
#define	ERR_ZERO_CALC		3		//ゼロ調整演算異常
#define	ERR_ZERO_LEVEL		4		//ゼロ調整波形アンバランス
#define	ERR_ZERO_AGC		5		//ゼロ調整AGC不能//

#define	ERR_MESR_EMPTY_L	6		//測定エンプティセンサL
#define	ERR_MESR_EMPTY_H	7		//測定エンプティセンサH
#define	ERR_MESR_WAVE_L		8		//測定波形減衰L
#define	ERR_MESR_WAVE_H		9		//測定波形減衰H
#define	ERR_MESR_CALC_L		10		//測定演算異常L
#define	ERR_MESR_CALC_H		11		//測定演算異常H
#define	ERR_MESR_LEVEL_L	12		//測定波形アンバランスL
#define	ERR_MESR_LEVEL_H	13		//測定波形アンバランスH
#define	ERR_MESR_AGC_L		14		//測定AGC不能L
#define	ERR_MESR_AGC_H		15		//測定AGC不能H
#define	ERR_MESR_REVERSE	16		//測定逆流異常

#define	ERR_10POINT			19		//10点データ無効
#define	ERR_EEPROM			20		//EEPROMエラー
#define	ERR_RESTART			22		//再起動

#define	ERR_MESR_OVERFLOW	23		//測定オーバーフロー
#define	ERR_CUNET			24		//CUnetエラー
#define	ERR_ZERO_UNSTABLE	25		//ゼロ調整計測時間発散

#define ALM_MESR_GAIN		26		//測定アンプゲイン急変（警告）
#define ALM_MESR_EMPTY	27		//測定エンプティセンサ（警告）

#define TTL_CACL_ERR	28		//積算値演算異常
#define TTL_OVERFLOW	29		//積算値オーバーフロー

#define	ERR_DEVICE		30		//メモリデバイス異常

#define	ERR_CODE_MAX		32		//エラーコード数

/*エラー判定ビット(MES[].err_statusを使用する)*/
#define	ERR_JUDGE_AGC		0x0001		//AGC不能//
#define	ERR_JUDGE_EMPTY		0x0002		//エンプティセンサ(異常判定回数以上)
#define	ERR_JUDGE_LEVEL		0x0004		//波形アンバランス
#define	ERR_JUDGE_BUBLE		0x0008		//気泡影響（センサ信号低レベル）
#define	ERR_JUDGE_CALC		0x0010		//演算異常(異常判定回数以上)
#define	ERR_JUDGE_REVERSE	0x0020		//逆流異常
#define	ERR_JUDGE_WAVE		0x0040		//波形減衰
#define	ERR_JUDGE_OVERFLOW	0x0080		//オーバーフロー
#define	ERR_JUDGE_10POINT	0x0100		//10点データ無効エラー
#define	ERR_JUDGE_EEPROM	0x0200		//EEPROMエラー
#define	ERR_JUDGE_CUNET		0x0400		//CUnetエラー
#define	ERR_JUDGE_RESTART	0x0800		//再起動
#define	ERR_JUDGE_PRE_EMPTY	0x1000		//エンプティセンサ(異常判定回数未満)
#define	ERR_JUDGE_PRE_CALC	0x2000		//演算異常(異常判定回数未満)
#define	ERR_JUDGE_UNSTABLE	0x4000		//ゼロ調整計測時間発散エラー
#define	ERR_JUDGE_ZERO		0x8000		//ゼロ点調整状態

/*エラー判定ビット(MES_SUB[].err_status_subを使用する)*/
#define	ERR_JUDGE_DEVICE	0x0001		//メモリデバイス異常

/*メール送信エラー(CUnet通信)*/
#define	MAIL_ERR_NORDY		0x0001		//送信先の受信バッファが受信許可でない
#define	MAIL_ERR_NOEX		0x0002		//送信先のCUnetステーションが存在しない
#define	MAIL_ERR_TOUT		0x0004		//設定サイクル回数を経過してもメール送信が完了しない
#define	MAIL_ERR_SZFLT		0x0008		//メール送信サイズが不正値
#define	MAIL_ERR_LMFLT		0x0010		//MSLRの設定値が不正値
#define	MAIL_ERR_STOP		0x0020		//ネットワークが停止

/*バーンアウト対象エラー*/
#define	ERR_BURN_OUT		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_CALC+ERR_JUDGE_OVERFLOW)

/*ゼロ調整不可対象エラー*/
#define	ERR_ZERO_ADJ		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_BUBLE+ERR_JUDGE_CALC+ERR_JUDGE_WAVE+ERR_JUDGE_OVERFLOW)

/*CUnetエラー対象エラー*/
#define	ERR_CUNET_MAIL		(MAIL_ERR_NOEX+MAIL_ERR_TOUT+MAIL_ERR_SZFLT+MAIL_ERR_LMFLT+MAIL_ERR_STOP)

/*流量エラー*/
#define	ERR_FLOW_CONT		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_BUBLE+ERR_JUDGE_CALC+ERR_JUDGE_REVERSE+ERR_JUDGE_WAVE+ERR_JUDGE_OVERFLOW)

/*メディアンフィルタ演算除外エラー*/
#define	ERR_MEDIAN_CALC		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_CALC+ERR_JUDGE_OVERFLOW+ERR_JUDGE_PRE_EMPTY+ERR_JUDGE_PRE_CALC)

/*警告判定ビット*/
#define	ALM_JUDGE_GAIN		0x0001		//ゲイン値急変（警告）
#define	ALM_JUDGE_EMPTY		0x0002		//エンプティセンサ（警告）

/*積算監視ビット*/
#define	TTL_JUDGE_REACH		0x0001		//積算目標値到達
#define	TTL_JUDGE_CACL_ERR	0x0002		//積算値演算異常
#define	TTL_JUDGE_OVERFLOW	0x0004		//積算値オーバーフロー

/*エラーコンディションビット*/
/*CUnetアドレス出力用*/
#define	CON_EMPTY_CU		0x0001		//エンプティセンサ
#define	CON_CALC_CU			0x0002		//演算異常
#define	CON_AGC_CU			0x0004		//AGC不能//
#define	CON_LEVEL_CU		0x0008		//波形アンバランス
#define	CON_REV_CU			0x0010		//逆流異常
#define	CON_WAVE_CU			0x0020		//波形減衰
#define	CON_OVER_CU			0x0040		//オーバーフロー
#define	CON_TOTAL_CU			0x0080		//積算目標値到達

/*エラーコンディションビット*/
#define	CON_EMPTY			0x0001		//エンプティセンサ
#define	CON_PEAK			0x0002		//極値異常
#define	CON_LEVEL			0x0004		//波形アンバランス
#define	CON_WAVE			0x0008		//波形減衰
#define	CON_CALC			0x0010		//演算異常
#define	CON_GAIN_OV			0x0020		//ゲインオーバー
#define	CON_GAIN_UD			0x0040		//ゲインアンダー
#define	CON_REV				0x0080		//逆流異常
#define	CON_HOLD			0x0100		//ホールドタイム未満
#define	CON_HOLDOUT			0x0200		//ホールドタイム以上
#define	CON_ZERO			0x0400		//ゼロ点調整エラー

/*動作ステータス*/
#define	ACT_STS_NORMAL		0			//通常
#define	ACT_STS_ZERO		1			//ゼロ点調整中
#define	ACT_STS_ADDIT		2			//積算実行中
#define	ACT_STS_WRITE		3			//パラメータ書込み中
#define	ACT_STS_DLOAD		4			//ファームウェアダウンロード中
#define	ACT_STS_TEST		5			//テスト中
#define	ACT_STS_OTHER		6			//その他

/*再起動要因コード*/
#define	RESTART_NORMAL		0			//パワーオンリセット(正常起動)
#define	RESTART_WDOG		1			//ウォッチドッグタイマリセット
#define	RESTART_POWER		2			//電源監視リセット
#define	RESTART_TERM		3			//端子リセット(MKY43)

/*カウンタ情報*/
#define	CMI_RATIO_100MSEC	19			//分解能：95msec周期
#define	CMI_RATIO_1SEC		200			//分解能：1秒周期
#define	CMI_COUNT_MAX		0x7FFFFFFF	//タイマ割込みカウンタMAX

/*CH情報*/
#define CH_NUMMAX 			6				//CH個数
#define CH_IDXMAX 			CH_NUMMAX - 1	//CHインデックス

/*CH番号(インデックスを兼ねるため0開始)*/
#define CH1 				0		//CH1
#define CH2 				1		//CH2
#define CH3 				2		//CH3
#define CH4 				3		//CH4
#define CH5 				4		//CH5
#define CH6 				5		//CH6

/*LED点灯処理*/
#define	DSP_LED1		0x0001		//LED CH1
#define	DSP_LED2		0x0002		//LED CH2
#define	DSP_LED3		0x0004		//LED CH3
#define	DSP_LED4		0x0008		//LED CH4
#define	DSP_LED5		0x0010		//LED CH5
#define	DSP_LED6		0x0020		//LED CH6
#define	DSP_ALM1		0x0100		//LED ALM1
#define	DSP_ALM2		0x0200		//LED ALM2
#define	DSP_ALM3		0x0400		//LED ALM3
#define	DSP_ALM4		0x0800		//LED ALM4

#define	DSP_CH_ALL		(DSP_LED1+DSP_LED2+DSP_LED3+DSP_LED4+DSP_LED5+DSP_LED6)		//全CH LED
#define	DSP_ALM_ALL		(DSP_ALM1+DSP_ALM2+DSP_ALM3+DSP_ALM4)		//全ALM LED

/*スイッチ情報*/
#define	SW_NUM			8			//スイッチ数
#define	SW_CH0			0			//CH0スイッチ
#define	SW_CH1			1			//CH1スイッチ
#define	SW_CH2			2			//CH2スイッチ
#define	SW_CH3			3			//CH3スイッチ
#define	SW_CH4			4			//CH4スイッチ
#define	SW_CH5			5			//CH5スイッチ
#define	SW_CH6			6			//CH6スイッチ
#define	SW_CH9			9			//CH9スイッチ

/*ステーションアドレス*/
#define SA_HOST			0			//HOST

//CUnet 16ch対応
#define SA_FLOW01		4			//流量計01
#define SA_FLOW02		7			//流量計02
#define SA_FLOW03		10			//流量計03
#define SA_FLOW04		13			//流量計04
#define SA_FLOW05		16			//流量計05
#define SA_FLOW06		19			//流量計06
#define SA_FLOW07		22			//流量計07
#define SA_FLOW08		25			//流量計08
#define SA_FLOW09		28			//流量計09
#define SA_FLOW10		31			//流量計10
#define SA_FLOW11		34			//流量計11
#define SA_FLOW12		37			//流量計12
#define SA_FLOW13		40			//流量計13
#define SA_FLOW14		43			//流量計14
#define SA_FLOW15		46			//流量計15
#define SA_FLOW16		49			//流量計16

/*占有幅*/
#define OWN_HOST		3			//HOST
#define OWN_FLOW		3			//流量計

/*通信メッセージ情報*/
#define MSG_MAX			700			//通信メッセージ最大数
#define MSG_MAX_DL		256 + 40	//通信メッセージ最大数(ダウンロード時) 128byte(2文字/1byte)+マージン
#define MES_RESEND_MAX	1000		//最大再送回数
#define MSG_NUM			3			//通信タイプ数（ホスト、サブホスト、メンテナンス）
#define MES_RESEND_LIM	(60 * CMI_RATIO_1SEC)	//最大再送時間:60秒(60 * カウンタ分解能)
#define MES_1ST_TOP		0			//送信メッセージ1回目先頭バイト
#define MES_2ND_TOP		256			//送信メッセージ2回目先頭バイト
#define MES_3RD_TOP		512			//送信メッセージ3回目先頭バイト
#define MES_CHAR_MAX 20		//コマンド内最大文字数
#define MES_W7_STATUS 5			//W7コマンドステータス文字数

#define HOST			0			//ホスト通信
#define MENT			1			//メンテナンス通信
#define SUB_HOST		2			//サブホスト通信

#define HEADER_CH		0x01			//CUnet Header CH

/*通信メッセージ情報	(End Code)*/
#define REBOOT_MANUAL			2
#define ADDRESS_OVER			4
#define PARITY_ERROR			10
#define FRAMING_ERROR			11
#define OVERRUN_ERROR			12
#define FCS_ERROR				13
#define FORMAT_ERROR			14
#define COMMAND_ERROR			16
#define CU_LENGTH_ERROR			18
#define ZEROADJUST_NOW			20
#define INTEGRATE_NOW			21
#define DOWNLOAD_NOW			22
#define EEPROMWRITE_NOW			23
#define EEPROM_ERROR			30
#define MEASURE_ERROR			32
#define REBOOT_WDT				33
#define SENSORSIZE_ERROR		34
#define FULLSCALE_ERROR			35
#define KFACTOR_ERROR			36
#define DUMPING_ERROR	 		37
#define LOWCUT_ERROR 			38
#define BURNOUT_CLASS_ERROR 	39
#define BURNOUT_VALUE_ERROR 	40
#define KVISCOSITY_ERROR 		41
#define ERRORHOLDTIME_ERROR 	42
#define USERLINEAR_POINT_ERROR 	43
#define USERLINEAR_VALUE_ERROR 	44
#define USERLINEAR_ORDER_ERROR 	45
#define REVERSE_VALUE_ERROR 	46
#define REVERSE_TIME_ERROR		47
#define READ_10POINTS_ERROR 	48
#define MAKERSET_ERROR 			49
#define JUDGEMENT_WAVE_ERROR	50
#define DEBUGMODE_ERROR 		51
#define ATTENUATOR_GAIN_ERROR 	52
#define ZADJ_WRITE_ERROR 		54
#define MAKERLINEAR_POINT_ERROR 55
#define MAKERLINEAR_VALUE_ERROR 56
#define MAKERLINEAR_ORDER_ERROR 57
#define LLMODE_ERROR 			58
#define USERLINEAR_5POINT_ERROR 	59
#define USERLINEAR_5POINT_EMPTY 	60
#define TOTALTARGET_ERROR 	61
#define TOTALOFFSET_ERROR 	62
#define FILTER_MODE_ERROR	65
#define FILTER_AVG_ERROR	66
#define FREQUENCY_ERROR	67

/*センサ情報*/
#define SNS_NONE		0			//センサ無し
#define SNS_TYPE_1_8	1			//PFA 1/8"
#define SNS_TYPE_4_3	2			//PFA 4x3
#define SNS_TYPE_1_4	3			//PFA 1/4"
#define SNS_TYPE_3_8	4			//0PFA 3/8"

/*メディアンフィルタ情報*/
#define MF_DEPTH	(32)
#define MF_AVE_ST	(8)
#define MF_AVE_EN	(23)

/*薬液リニア情報*/
#define LL_NUM		8			//薬液種別総数
#define LL_KV		0			//動粘度
#define LL_SS		1			//センサ種別
#define LL_FS		2			//フルスケール
#define LL_LP		3			//リニアライズ点数
#define LL_POINT1	4			//補正量1点目

/*フィルターモード情報*/
#define FILTER_MOVING 0 //移動平均
#define FILTER_DIRECT 1 //フィルタ無し
#define FILTER_MEDIAN 2 //メディアンフィルタ

/*ゼロクロス情報*/
#define ZC_POINT_MAX 40 //ゼロクロス点最大値

/*Windowサーチ情報*/
/*32MHz, 40MHz*/
#define WS_FIFO_START_14_3240 10	//Windowサーチ開始FIFO CH(1/4"用)
#define WS_FIFO_START_38_3240 10	//Windowサーチ開始FIFO CH(3/8"用)
#define WS_FIFO_END_3240 25	//Windowサーチ終了FIFO CH
/*65MHz*/
#define WS_FIFO_START_14 15	//Windowサーチ開始FIFO CH(1/4"用)
#define WS_FIFO_START_38 15	//Windowサーチ開始FIFO CH(3/8"用)
#define WS_FIFO_END 30	//Windowサーチ終了FIFO CH

#define WS_FIFO_RANGE 8	//FIFO取得範囲

// UART 割り付け定義 
#define	UART_COM_HOST_BASE	UART6_BASE
#define	INT_UART_COM_HOST	INT_UART6
#define	UART_COM_MENT_BASE	UART2_BASE
#define	INT_UART_COM_MENT	INT_UART2

#define WAV_PEK_NUM 10 //取得する波形のピーク値数
#define LDG_PNT_OFS 40  //Leading Point Offset ((従来値)40 / (オーバーサンプリング倍率)8 = 5)

#define ADD_NO_CMM 0 //Add No Comma
#define ADD_CMM_BFR_VAL 1 //Add Comma Before Value
#define ADD_CMM_AFR_VAL 2 //Add Comma After Value
