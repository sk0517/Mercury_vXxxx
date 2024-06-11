/***********************************************/
/* File Name : int_flow.c						*/
/*	Summary   : 流量計測処理						*/
/*	Date      : 2023/03/16						*/
/*												*/
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.		*/
/*			All rights reserved					*/
/***********************************************/
#include <machine.h>
#include <math.h>
#include <mathf.h>
#include <stdlib.h>
#include <limits.h>		

#include "define.h"
#include "version.h"
#include "SV_def.h"
#include "defSAVE.h"
#include "defMES.h"
#include "defMAIN.h"
#include "defLED.h"
#include "ctlioport.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"

/********************************************************/
/*	モジュール内定義関数								*/
/********************************************************/
void	reg_clock(void);
short	ch_search(short pch);
void	dma_dummy(short count, short pch);
void	dma_start(short *read_add);
void	us_dma_start(short *read_add);
void	int15_handler(void);
short	fifo_read_wait(short pch);
short	fifo_search(short pch, short fifo);
short	get_vth2(short data[], short vth);
void	fifo_read(short pch);
void	DriveFIFOFwd(short pch, short sample);
void	DriveFIFORev(short pch, short sample);
short	min_max_check(short pch);
short	gain_adj_init(short pch);
short	gain_adj_control(short pch);
void	sum_adder(short pch);
short	min_max_search(short pch);
void	delta_ts_cal(short pch);
short	temp_v(short pch);
void	sonic_search_max(short pch);
void	zero_adj(short pch);
void	make_viscos_tbl(short pch);
long	auto_linear(long viscos, short pch);
long	maker_linear(long in_work, short pch);
long	user_linear(long in_flow, short pch);
void	LLmode_kind(short vis, short pch);
long	flow_calc(short pch);
long	pv_calc(long in_flow, short pch);
void	sonic_filter_control(short ch);
void	correlate_check(short ch);
void	max_point_control(short ch);
long	median_filter_control(long in_flow, short ch);
void	damp_control(short ch);
void	lowcut_control(short ch);
void	burnout_control(short ch);
long	median_filter(long *d);
short	get_attenuator_gain(short pch);
void	Filt(const short *Tf, long *IN, long *OUTn);
short	addit_req_check(short ch);
void	addit_flow_calc(void);
void	reverse_flow_check(short ch);
void	test_flow_mode(short ch);
void	inspect_flow_mode(short ch);
void	InitFifoCh(short ch, short val);
void	err_thlevel_init(short ch, short val);
void	current_flow_control(short ch);
void	check_amp_gain(short ch);
void	flow_save_control(void);
void	filter_moving(short pch);
void	InitFPGA(void);
void	WaitCDONE(void);
void	SetADCClock(short pch);
void	SetDriveFreq(short ch);
void	SetDrivePulse(short pch);
void	SetWindowPosition(short pch);
void	SetClockPhase(short pch);
void	SetFpgaRegister(short pch);
void	SetDigitalFilterRegister(void);
void	WriteGainAmp(unsigned short gain);
void	SetNextGain(short pch);
short	SearchWindow(short pch);
void	zc_filter_moving(short pch);
void	int_flow(short pch);
#ifdef FRQSCH
short	FrqSchPrc(short pch);
#endif
long ClcNmlLnr(long work, short point, short pch);
long ClcChmLnr(long work, short point, short pch);
void SchZerPnt(short pch);
void SelectForwardOn(short pch);
void SelectReverseOn(short pch);
void SelectForwardOff(short pch);
void SelectReverseOff(short pch);
void ActBfrFifRed(short pch);
void ClcSumWavAmp(short pch, short sample);
float GetTimDif(short pch, short Mod);
long ClcFlwFlt(long in_flow, short pch);
long ClcFlwFlt_MvgAve(long in_flow, short pch);
void GetWav(short pch);
void SetFifoNo(short pch);
void SetFifoPos(short pch);
void SchZerDat(short pch, short *WavPtr, short *ZerPnt, short *ZerDat1, short *ZerDat2, short *ZerDat3, short *ZerDat4);
void SchTrshld(short pch, short *WavPtr, short *OutVal);
float DelAbnPnt(float *FwdPnt, float *RevPnt, float zc_SumOld, short zc_num, float SmpTdt);
float DelAbnPntMod(float *FwdPnt, float *RevPnt, float zc_SumOld, short zc_num, float SmpTdt);

/********************************************************/
/*	モジュール外定義関数								*/
/********************************************************/
extern void	action_status_control(short pch, short act);
extern void delay(unsigned short delay_count);
extern void	non_sensor_control(short pch);
extern unsigned long long get_total_offset(short ch, unsigned long long val_total);
extern float RoundFunc(float src);

/********************************************************/
/*	モジュール内定義変数								*/
/********************************************************/
short	addit_sts = B_OFF;
short	rev_max_point,fow_max_point; //波形振幅の最大値をとるx座標
short	fow_max,fow_min; //波形の振幅最大値
short	rev_max,rev_min;
short	wave_hight[2][6];
long work_oq;
volatile short fifo_data_ready = 0;
short	get_cyc[6] = {0, 0, 0, 0, 0, 0};

unsigned short AD_BASE = 2047;
unsigned short AD_MAX = 4095;

// #define FLWWAVEXP
#define FLWWAVSIZ 600
#if defined(FLWWAVEXP)
short fwd_temp_data[4][FLWWAVSIZ + 100];
short rev_temp_data[4][FLWWAVSIZ + 100];
#else
short fwd_temp_data[4][256];
short rev_temp_data[4][256];
#endif

const	short center_ratio={5000};		/*50%*/
const	short search_ratio={10000};		/*100%*/
long mf_data[MF_DEPTH];

short ClcActFlg = 1; //差分相関 / ゼロクロス
#define ClcAct_SumAdd 0
#define ClcAct_ZerCrs 1
short TopPosFix = 0; //先頭位置をゼロ調節時から移動させる / させない		※波形認識しきい値機能と被るので「0」固定
#define PosFix_Mov 0
#define PosFix_NotMov 1
short SumPntInc = 0; //差分相関点数を増加させない / させる
#define SumPnt_NotInc 0
#define SumPnt_Inc 1

#define DatInc //オーバーサンプリング
//#define ShtCntTwo //打ち込み回数2回に変更
#define ShtItv //打ち込み後インターバル (+200 = +20us)
//const short ItvVal = 1000; //200us
// const short ItvVal = 1250; //230us
//const short ItvVal = 1450; //250us
const short ItvVal = 2000; //300us
short Mgn = 8;

#define NEW_MEASURE_METHOD
#define METHOD_OLD  //従来のゼロクロス演算方法
// #define METHOD_CLFC1 //CLFC300の演算方法1
// #define METHOD_CLFC2 //CLFC300の演算方法2

/********************************************************/
/*	モジュール外定義変数								*/
/********************************************************/
extern short	com_type;
extern short initializing;
extern short FpgaVersion;

/********************************************************/
/*	センサ情報											*/
/* 0=None, 1=1/8", 2=, 3=1/4", 4=3/8" */
/********************************************************/
//サンプリング周期
short SmpTs0[] = { 3125, 2500, 1538, 1250 }; //1/32MHz, 1/40MHz, 1/65MHz, 1/80MHz [ns:100倍データ]
short SmpTs[] = { 0, 0, 0, 0 };
short AdcSmpFrq[] = {32, 40, 65, 80};

typedef struct {
	short	area;					// 断面積 (mm2 x 100)
	short	sns_disL_l;		// センサ距離L-l（*0.1mm）流速演算
	short	sns_disL; 			// センサ距離L（*0.1mm)音速測定用
	short	sns_TAU;			// τs(*ns)音速測定用
	short	k_scale;			// Kファクタ互換係数
	short	Ts;					// サンプリング周期 (10ps)
	short wave_vth;			// 受波検出閾値
	short balance_level;		// 波形アンバランス検出閾値
	short saturation_level;	// 波形飽和検出閾値
	short correlate_level;	// 差分相関比閾値
	short correlate_time;		// 演算異常判定回数
	short attenuate_level;	// 波形減衰閾値
	short fifo_ch;			// FIFO CH 初期値
} SENSOR_INFO;

const SENSOR_INFO sens_inf[] = {
	{// None
		-1,			// 断面積 (mm2 x 100)
		-1,			// センサ距離L-l（*0.1mm）流速演算
		-1,			// センサ距離L（*0.1mm)音速測定用
		-1,			// τs(*ns)音速測定用
		-1,			// Kファクタ互換係数
		-1,			// サンプリング周期 (10ps)
		30,			// 受波検出閾値
		15,			// 波形アンバランス検出閾値
		94,			// 波形飽和検出閾値
		80,			// 差分相関比閾値
		100,		// 演算異常判定回数
		10,			// 波形減衰閾値
		5			// FIFO CH 初期値
	},
	{// 1/8"
		370,		// 断面積 (mm2 x 100)  (管内径2.17mm)
		600,			// センサ距離L-l（*0.1mm）流速演算
		600,		// センサ距離L（*0.1mm)音速測定用
		6000,		// τs(*ns)音速測定用
		10000,		// Kファクタ互換係数
		3200,		// サンプリング周期 (10ps)
		30,			// 受波検出閾値
		15,			// 波形アンバランス検出閾値
		94,			// 波形飽和検出閾値
		80,			// 差分相関比閾値
		100,		// 演算異常判定回数
		10,			// 波形減衰閾値
		5			// FIFO CH 初期値
	},
	{// 4x3
		707,		// 断面積 (mm2 x 100)  (管内径3.00mm)
		600,		// センサ距離L-l（*0.1mm）流速演算
		600,		// センサ距離L（*0.1mm)音速測定用
		6000,		// τs(*ns)音速測定用
		10000,		// Kファクタ互換係数
		3200,		// サンプリング周期 (10ps)
		30,			// 受波検出閾値
		15,			// 波形アンバランス検出閾値
		94,			// 波形飽和検出閾値
		80,			// 差分相関比閾値
		100,		// 演算異常判定回数
		10,			// 波形減衰閾値
		5			// FIFO CH 初期値
	},
	{// 1/4"
		1486,		// 断面積 (mm2 x 100)  (管内径4.35mm)
		600,		// センサ距離L-l（*0.1mm）流速演算
		600,		// センサ距離L（*0.1mm)音速測定用
		6000,		// τs(*ns)音速測定用
		10000,		// Kファクタ互換係数
		3200,		// サンプリング周期 (10ps)
		30,			// 受波検出閾値
		15,			// 波形アンバランス検出閾値
		94,			// 波形飽和検出閾値
		80,			// 差分相関比閾値
		100,		// 演算異常判定回数
		10,			// 波形減衰閾値
		5			// FIFO CH 初期値
	},
	{// 3/8"
		4453,		// 断面積 (mm2 x 100)  (管内径7.53mm)
		600,			// センサ距離L-l（*0.1mm）流速演算
		600,		// センサ距離L（*0.1mm)音速測定用
		6000,		// τs(*ns)音速測定用
		10000,		// Kファクタ互換係数
		3200,		// サンプリング周期 (10ps)
		30,			// 受波検出閾値
		15,			// 波形アンバランス検出閾値
		94,			// 波形飽和検出閾値
		80,			// 差分相関比閾値
		100,		// 演算異常判定回数
		10,			// 波形減衰閾値
		5			// FIFO CH 初期値
	}
};

/********************************************************/
/*	差分相関情報										*/
/********************************************************/
//const	short abs_cont[5]={40,40,40,40,40};			/*相関値個数 SUB_POINT未満*/
const	short abs_cont[5]={20,20,20,20,20};			/*相関値個数 SUB_POINT未満*/

/********************************************************/
/*	FIFO情報											*/
/********************************************************/
#define FIFO   (*(volatile short *)0x60000000) 	/* FIFO Address*/ 
unsigned char FIFO_CH_SRT = 63;		//FIFO CH:開始
unsigned char FIFO_CH_END = 47;		//FIFO CH:終了

/********************************************************/
/*	流速設定テーブル									*/
/********************************************************/
const 	long flow_vel_tbl[21]={
			66429,48000,34684,25061,18109,13085,9455,6832,4936,3567,
			2577,1862,1346,972,703,508,367,265,192,138,100
};		/* 流速[cm/sec * 100]*/

typedef struct {
	short viscos;	/*動粘度[mm2/sec * 100]*/
	short dat[21];
	} KV_TBL;

/****************************************************/
/*	動粘度係数テーブル PFA 1/8"					*/
/****************************************************/
const KV_TBL AUTO_COEFF_1_8[]={
		/*0.3000*/
		{30,1212,1021,1090,1238,1381,1497,1602,1725,1893,2123,2413,2745,3091,3422,3725,4225,4602,4893,5071,5118,5028},
		/*0.4510*/
		{45,1024,1125,1277,1413,1524,1630,1762,1945,2191,2495,2833,3178,3501,3801,4327,4685,4950,5096,5108,4983,4724},
		/*0.6781*/
		{68,1163,1314,1443,1550,1660,1803,2002,2264,2579,2922,3263,3579,4020,4425,4763,4999,5112,5089,4929,4637,4226},
		/*1.0194*/
		{102,1349,1472,1577,1693,1848,2063,2340,2665,3010,3346,3656,4128,4519,4834,5040,5119,5061,4867,4543,4103,3559},
		/*1.5326*/
		{153,1499,1604,1727,1897,2128,2419,2752,3098,3428,3731,4233,4608,4898,5073,5118,5024,4796,4442,3974,3406,2745},
		/*2.3042*/
		{230,1633,1765,1950,2197,2501,2840,3184,3507,3920,4335,4691,4954,5097,5107,4979,4717,4333,3838,3246,2561,1780},
		/*3.4641*/
		{346,1807,2006,2270,2585,2929,3269,3585,4029,4433,4768,5003,5113,5087,4925,4630,4217,3696,3080,2372,1563,623},
		/*5.2080*/
		{521,2068,2346,2671,3017,3353,3661,4136,4526,4839,5043,5119,5059,4862,4536,4094,3548,2909,2176,1337,356,-894},
		/*7.8297*/
		{783,2425,2759,3104,3434,3737,4241,4615,4902,5075,5117,5021,4790,4434,3964,3394,2731,1974,1103,77,-1189,-2236},
		/*11.7713*/
		{1177,2847,3191,3513,3928,4342,4697,4958,5099,5106,4975,4711,4324,3828,3233,2547,1764,858,-218,-1474,-2468,-3216},
		/*17.6872*/
		{1769,3276,3591,4037,4440,4774,5006,5114,5085,4920,4624,4208,3685,3067,2357,1546,603,-616,-1747,-2683,-3364,-3792},
		/*26.6061*/
		{2661,3667,4144,4533,4844,5046,5120,5056,4857,4528,4084,3536,2895,2161,1320,336,-917,-2008,-2882,-3496,-3866,-4072},
		/*40.0000*/
		{4001,4248,4621,4907,5077,5117,5018,4785,4426,3954,3382,2717,1958,1085,55,-1211,-2254,-3064,-3611,-3930,-4114,-4318}
};

/****************************************************/
/*	動粘度係数テーブル PFA 4x3					*/
/****************************************************/
const KV_TBL AUTO_COEFF_4_3[]={
		/*0.3000*/
		{30,52,-370,-509,-501,-433,-355,-286,-223,-146,-30,152,415,760,1164,1572,1887,2085,2424,2818,3237,3636},
		/*0.4510*/
		{45,-426 ,-517,-486,-412,-336,-270,-205,-121,9,210,495,859,1271,1666,1875,2165,2520,2924,3342,3730,4051},
		/*0.6781*/
		{68,-517,-469,-392,-318,-254,-187,-93,53,275,581,962,1377,1752,1943,2249,2619,3031,3445,3820,4118,4318},
		/*1.0194*/
		{102,-451,-372,-301,-238,-167,-62,102,345,672,1066,1480,1828,2014,2338,2722,3138,3546,3904,4179,4353,4422},
		/*1.5326*/
		{153,-353,-285,-221,-144,-27,156,421,768,1173,1579,1891,2090,2431,2826,3245,3644,3983,4233,4380,4424,4385},
		/*2.3042*/
		{230,-269,-204,-119,13,215,502,867,1279,1673,1880,2171,2527,2932,3350,3737,4056,4281,4400,4421,4365,4264},
		/*3.4641*/
		{346,-185,-91,57,280,588,969,1385,1758,1948,2256,2627,3039,3453,3826,4123,4321,4414,4413,4342,4235,4122},
		/*5.2080*/
		{521,-60,106,350,679,1074,1488,1833,2020,2345,2730,3146,3554,3910,4184,4355,4422,4401,4317,4206,4096,4000},
		/*7.8297*/
		{783,160,427,775,1181,1587,1896,2096,2438,2834,3253,3651,3989,4237,4382,4424,4384,4290,4177,4070,3977,3866},
		/*11.7713*/
		{1177,508,874,1287,1679,1885,2177,2535,2940,3358,3744,4062,4284,4402,4421,4364,4262,4148,4046,3953,3826,3577},
		/*17.6872*/
		{1769,977,1393,1764,1953,2262,2635,3047,3461,3833,4128,4324,4415,4412,4341,4233,4120,4022,3927,3777,3524,3207},
		/*26.6061*/
		{2661,1495,1838,2026,2352,2738,3154,3561,3916,4188,4357,4423,4399,4315,4203,4094,3999,3898,3717,3460,3099,2591},
		/*40.0000*/
		{4001,1824,2102,2445,2842,3261,3658,3995,4241,4383,4424,4382,4288,4174,4068,3975,3864,3643,3385,2980,2448,1863}
};

/****************************************************/
/*	動粘度係数テーブル PFA 1/4"				*/
/****************************************************/
const KV_TBL AUTO_COEFF_1_4[]={
		/*0.3000*/
		{30,-1000,-1000,-1000,-1000,-1000,-1000,-1000,-975,-947,-928,-938,-930,-766,-567,-322,-35,274,579,840,1008,1000},
		/*0.4510*/
		{45 ,-1000 ,-1000,-1000,-1000,-1000,-1000,-967,-941,-927,-944,-895,-720,-509,-252,42,353,651,894,1030,1000,1000},
		/*0.6781*/
		{68,-1000,-1000,-1000,-1000,-993,-960,-935,-929,-947,-847,-671,-448,-180,121,432,720,941,1000,1000,1000,1000},
		/*1.0194*/
		{102,-1000,-1000,-1000,-983,-953,-931,-933,-944,-806,-618,-384,-106,200,509,785,980,1000,1000,1000,1000,1000},
		/*1.5326*/
		{153,-1000,-1000,-974,-946,-928,-938,-929,-763,-563,-317,-30,280,584,844,1010,1000,1000,1000,1000,1000,1000},
		/*2.3042*/
		{230,-1000,-966,-940,-928,-944,-891,-716,-505,-247,48,359,656,898,1031,1000,1000,1000,1000,1000,1000,1000},
		/*3.4641*/
		{346,-959,-935,-929,-947,-844,-667,-443,-174,127,438,725,944,1000,1000,1000,1000,1000,1000,1000,1000,1000},
		/*5.2080*/
		{521,-931,-933,-944,-803,-614,-379,-100,206,515,789,983,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000},
		/*7.8297*/
		{783,-939,-927,-759,-559,-311,-24,286,590,848,1012,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000},
		/*11.7713*/
		{1177,-887,-713,-500,-241,54,365,662,901,1032,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000},
		/*17.6872*/
		{1769,-663,-438,-169,133,444,730,947,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000},
		/*26.6061*/
		{2661,-374,-94,212,521,794,985,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000},
		/*40.0000*/
		{4001,-18,292,595,853,1014,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000}
};

/****************************************************/
/*	動粘度係数テーブル PFA 3/8"					*/
/****************************************************/
const KV_TBL AUTO_COEFF_3_8[]={
		/*0.3000*/
		{30,-1994,-2004,-2011,-2010,-2000,-1975,-1932,-1865,-1772,-1648,-1491,-1298,-1070,-809,-520,-211,106,412,685,895,1005},
		/*0.4510*/
		{45 ,-2006 ,-2011,-2009,-1995,-1966,-1917,-1844,-1744,-1611,-1445,-1243,-1007,-738,-443,-130,186,486,746,934,1011,928},
		/*0.6781*/
		{68,-2012,-2007,-1989,-1955,-1901,-1821,-1713,-1572,-1397,-1186,-941,-665,-364,-49,265,557,803,966,1007,874,506},
		/*1.0194*/
		{102,-2004,-1982,-1944,-1883,-1797,-1680,-1531,-1347,-1127,-873,-590,-285,32,342,626,854,990,992,805,366,0},
		/*1.5326*/
		{153,-1974,-1930,-1864,-1770,-1646,-1487,-1294,-1066,-804,-514,-205,112,418,690,899,1006,965,720,0,0,0},
		/*2.3042*/
		{230,-1916,-1843,-1741,-1609,-1442,-1239,-1002,-732,-437,-124,192,492,751,937,1011,924,617,0,0,0,0},
		/*3.4641*/
		{346,-1820,-1711,-1569,-1393,-1182,-936,-659,-358,-43,271,563,807,968,1006,869,496,0,0,0,0,0},
		/*5.2080*/
		{521,-1678,-1528,-1343,-1123,-868,-584,-279,38,348,631,857,992,990,799,355,0,0,0,0,0,0},
		/*7.8297*/
		{783,-1484,-1290,-1061,-799,-508,-198,118,424,695,902,1006,962,713,0,0,0,0,0,0,0,0},
		/*11.7713*/
		{1177,-1235,-997,-727,-431,-118,198,497,755,940,1011,921,609,0,0,0,0,0,0,0,0,0},
		/*17.6872*/
		{1769,-931,-654,-352,-37,276,568,811,971,1006,865,486,0,0,0,0,0,0,0,0,0,0},
		/*26.6061*/
		{2661,-579,-273,44,354,636,861,993,989,793,343,0,0,0,0,0,0,0,0,0,0,0},
		/*40.0000*/
		{4001,-192,124,429,700,905,1007,959,706,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

/********************************************************************************
 *			薬液リニアライズモードテーブル
 * 判定項目
 * 00 : 動粘度
 * 01 : センサ種類
 * 02 : フルスケール(センサ4種類に対応)
 * 03 : リニアライズ点数
 * 04-07 : FIFO CH (センサ4種類に対応)
 * 08-11 : ゼロクロス点検索開始位置(波形の極大値となるように実測, センサ4種類に対応)
 * 12-26 : リニアライズ設定値（mL/min 10倍データ）
 ********************************************************************************/
// const struct {
// 	short dat[19];
// }LL_TBL[]={
// 		/* 0: None(DIW) */
// 		{100, SNS_TYPE_1_4, 300, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 1: OK73*/
// 		{84, SNS_TYPE_1_4, 300, 5, 20, 30, 20, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 2: ONNR20*/
// 		{167, SNS_TYPE_1_4, 300, 5, 0, -40, -130, -190, -210, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 3: IPA*/
// 		{90, SNS_TYPE_1_4, 300, 5, 23, 28, 1, -3, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 4: Butyl Acetate*/
// 		{30, SNS_TYPE_1_4, 300, 5, 40, 100, 170, 240, 280, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 5: EL*/
// 		{260, SNS_TYPE_1_4, 300, 5, -5, -39, -119, -184, -243, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 6: LA95*/
// 		{160, SNS_TYPE_1_4, 300, 5, -6, -29, -61, -82, -94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// 		/* 7: PM*/
// 		{40, SNS_TYPE_1_4, 300, 5, 39, 84, 144, 184, 205, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
// };
struct strLLModeInfo
{
	short Vis;
	short SensorSize;
	short MaxFlow[SNS_KIND];
	short LinerPnt;
	short FifoCh[SNS_KIND];
	short ZerPeakPos[SNS_KIND];
	short LinerData[15];
};
const struct strLLModeInfo LL_TBL[] =
{
	/* 0: None(DIW) */
	{
		100, 
		SNS_TYPE_1_4, 
		800, 800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 1: OK73*/
	{
		84, 
		SNS_TYPE_1_4, 
		800, 800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		20, 30, 20, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 2: ONNR20*/
	{
		167, 
		SNS_TYPE_1_4, 
		800,  800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		0, -40, -130, -190, -210, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 3: IPA*/
	{
		90, 
		SNS_TYPE_1_4, 
		800,  800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		23, 28, 1, -3, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 4: Butyl Acetate*/
	{
		30, 
		SNS_TYPE_1_4, 
		800,  800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		40, 100, 170, 240, 280, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 5: EL*/
	{
		260, 
		SNS_TYPE_1_4, 
		800,  800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		-5, -39, -119, -184, -243, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 6: LA95*/
	{
		160, 
		SNS_TYPE_1_4, 
		800,  800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		-6, -29, -61, -82, -94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
	/* 7: PM*/
	{
		40, 
		SNS_TYPE_1_4, 
		800,  800, 800, 4000,
		5, 
		22, 22, 22, 22, 
		23, 23, 23, 23,
		39, 84, 144, 184, 205, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	},
};

/****************************************************/
/* Function : reg_clock                      */
/* Summary  : クロックパルス                            */
/* Argument : なし                               		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : TLV5623CDGK(Digital to analog converter)*/
/****************************************************/
void	reg_clock(void){

	// GPIO_PD3
	__bit_output(GPIO_PORTD_BASE, 3, 0);		/*clock=0*/
	// GPIO_PD3
	__bit_output(GPIO_PORTD_BASE, 3, 1);		/*clock=1*/
}

/****************************************************/
/* Function : ch_search                      */
/* Summary  : FIFOチャネルを出力             */
/* Argument : pch                              		 */
/* Return   : なし									 */
/* Caution  : なし                                  */
/* note     : なし            */
/****************************************************/
short	ch_search(short pch){

 short fifo;

	fifo = SVD[pch].fifo_ch_init;
	if(fifo < 0 || fifo > 47){		//リミットチェック
		fifo = 5;
	}

	return fifo;
}

/****************************************************/
/* Function : dma_dummy				*/
/* Summary  : DMA 開始（空読み）            */
/* Argument : count,  pch                          */
/* Return   : な                                   */
/* Caution  : なし                                  */
/* note     : FIFOから指定ワード空読みする            */
/****************************************************/
void dma_dummy( short count, short pch){
 
	unsigned short dumy_buffer[1];

	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_8);	// 転送先のアドレスをインクリメントしない 
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, dumy_buffer, count);
	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);	// DMA 転送開始 
	while(uDMAChannelModeGet(UDMA_CHANNEL_SW) != UDMA_MODE_STOP);	//転送待ち
	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_8);	// 通常設定に戻す 

} 

/****************************************************/
/* Function : dma_start                           */
/* Summary  : DMA 開始                            */
/* Argument : *read_add                         		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : FIFOから200ワード転送する                 */
/****************************************************/
void	dma_start(short *read_add){

#if defined(FLWWAVEXP)
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, read_add, FLWWAVSIZ);	// DMA 転送開始 
#else
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, read_add, 200);	// DMA 転送開始
#endif
	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);	// DMA 転送開始 
	while(uDMAChannelModeGet(UDMA_CHANNEL_SW) != UDMA_MODE_STOP);	//転送待ち

}
void	us_dma_start(short *read_add){
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, read_add, 240);	// DMA 転送開始

	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);	// DMA 転送開始 
	while(uDMAChannelModeGet(UDMA_CHANNEL_SW) != UDMA_MODE_STOP);	//転送待ち
}

/****************************************************/
/* Function : int15_handler                     */
/* Summary  : FIFOにデータが入り始めるまで待つ          */
/* Argument : なし                            		 */
/* Return   : なし								*/
/* Caution  : なし                                  */
/* note     : tm4c1290nczad_startup_ccs.c          */
/*            ※未使用※								*/
/*            FPGA側を変更してIRQ4をEMPTY検出とする		*/
/*            FIFOデータが空(FPGAがFIFOデータを書込んでいる期間)は待機する*/
/****************************************************/
void int15_handler(void) {

	GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);

	IntDisable(INT_GPIOP6);

	IntPendClear(INT_GPIOP6);

	fifo_data_ready++;
	if(fifo_data_ready >= 2) for(;;);
}

/****************************************************/
/* Function : fifo_read_wait                     */
/* Summary  : FIFOデータが空(FPGAがFIFOデータを書込んでいる期間)は待機する */
/* Argument : short pch                    		 */
/* Return   : 0:正常　　-1：異常                      */
/* Caution  : なし                                  */
/* note     : なし                                 */
/****************************************************/
short fifo_read_wait(short pch) {

	short ret, cnt;

	if(MES_SUB[pch].fifo_result == B_NG)	return (B_NG);

	cnt = 0;
	ret = B_OK;
	while(1){
		if(__bit_input(GPIO_PORTP_BASE, 6) == 0){	//FIFOデータが空(FPGAがFIFOデータを書込んでいる期間)は待機する
			break;	//正常
		}

		cnt++;
		if(cnt>1000){	//センサが外れている場合を考慮
			ret = MES_SUB[pch].fifo_result = B_NG;
			break;	//異常
		}
	}

	return (ret);	
}

/****************************************************/
/* Function : fifo_search                     */
/* Summary  : FIFOを読み信号の位置を求める         */
/* Argument : pch, search_ch                		 */
/* Return   : 0＝正常（信号あり）, -1異常(信号なし)       */
/* Caution  : なし                                  */
/* note     : なし                                 */
/****************************************************/
short fifo_search(short pch, short search_ch){

	short i,cnt,work;
	short amp;
	short ret;
	short non_signal_count_sv;
	short	atn_gain;
	unsigned short empty_level; 
	short work_old[2]={AD_BASE_UNIT, AD_BASE_UNIT};
 
	if(MES[pch].ThresholdReq == 2){	/*Windowサーチ要求あり(最適なFIFO CHを探す)*/
		WriteGainAmp((unsigned short)MES[pch].amp_gain_fifo);  /*ゲイン値固定*/
	}else{
		if((SVD[pch].fix_data & 0x02) != 0){  //固定値設定
			if(SVD[pch].fix_amp_gain_rev == 0){
				WriteGainAmp((unsigned short)MES[pch].zero_amp_gain_rev);  /*ゼロ点調整時のゲイン値*/
			}else{
				WriteGainAmp((unsigned short)SVD[pch].fix_amp_gain_rev);  /*固定設定時のゲイン値*/
			}
		}else{
			WriteGainAmp((unsigned short)MES[pch].amp_gain_rev);  /*ゲイン書込み*/
		}
	}

	OutputRestartPulse();			/*RESTARTパルス出力*/
	/*REV*/	
	SelectForwardOn(pch);		//IN/OUT打込み切替え

	OutputRestartPulse();		/*RESTARTパルス出力*/

	if(MES[pch].ThresholdReq == 0	/*波形認識実行要求なし*/
		|| MES[pch].ThresholdReq == 1){ 	/*Windowサーチ用のゲイン値調整*/
		MES[pch].fifo_ch = search_ch;
	}else if(MES[pch].ThresholdReq == 2){ /*Windowサーチ要求あり(最適なFIFO CHを探す)*/
		MES[pch].fifo_ch = SVD[pch].fix_fifo_ch_read;
	}else{
		MES[pch].fifo_ch = MES[pch].ws_FifoCh;
	}
	
	MES[pch].fifo_start = MES[pch].fifo_ch;	/*WINDOW開始時間を設定*/
	MES[pch].fifo_end = MES[pch].fifo_ch + 1;	/*WINDOW終了時間を設定*/
	MES[pch].fifo_offset = SVD[pch].wind_offset; /*WINDOWオフセット時間を設定*/
		
	if(initializing == 0){   //起動時の初期化処理が終了している場合
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}

	if(MES[pch].ThresholdReq == 2){	/*Windowサーチ要求*/
		MES[pch].clk_phase = 0; // 位相初期化
	}
	SetFpgaRegister(pch); // FPGAレジスタの設定
	OutputStartPulse();		/*STARTパルス出力*/

	/*FIFO読込み(ダミーリード)*/
	MES_SUB[pch].fifo_result = 0;		//FIFO読込みステータスのクリア
	if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
		work = FIFO;  /*最初の2word = 0 に成ってる*/
	}
	if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
		work = FIFO;  /*2回、ダミーリードする。   */
	}

	/*受波波形検出レベル(Emptyエラー)*/
	empty_level = (unsigned short)((long)AD_MAX_UNIT * (long)SVD[pch].wave_vth / 100);	/*最大値(12bit MAX)*判定閾値*/
	
	/*流量計測時(通常時)*/
	if(MES[pch].ThresholdReq == 0	/*波形認識実行要求なし*/
		|| MES[pch].ThresholdReq == 1	/*Windowサーチ用のゲイン値調整*/
		|| MES[pch].ThresholdReq == 2){	/*Windowサーチ要求あり(最適なFIFO CHを探す)*/

		// 高速化のためループ 1 回で 4 バイトずつリード 
		for(i=2; i<(4000-200); i+= 4){	  /*3800word*/
			if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
				work = FIFO;	//FIFOデータ読込み
				// work &= 0x0FFF;
				work &= 0x1FFF;
				if(work < empty_level){		/*受波検出閾値チェック(Emptyエラー)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*波形認識閾値の未設定(ゼロ点調整未実施)*/
						MES[pch].signal_count = i;
					}
					break;		//受波波形を検出
				}
			}else{
				i = 3800;
				break;
			}

			if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
				work = FIFO;
				work &= 0x1FFF;
				if(work < empty_level){		/*受波検出閾値チェック(Emptyエラー)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*波形認識閾値の未設定(ゼロ点調整未実施)*/
						MES[pch].signal_count = i + 1;
					}
					break;		//受波波形を検出
				}
			}else{
				i = 3800;
				break;
			}

			if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
				work = FIFO;
				work &= 0x1FFF;
				if(work < empty_level){		/*受波検出閾値チェック(Emptyエラー)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*波形認識閾値の未設定(ゼロ点調整未実施)*/
						MES[pch].signal_count = i + 2;
					}
					break;		//受波波形を検出
				}
			}else{
				i = 3800;
				break;
			}

			if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
				work = FIFO;
				work &= 0x1FFF;
				if(work < empty_level){		/*受波検出閾値チェック(Emptyエラー)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*波形認識閾値の未設定(ゼロ点調整未実施)*/
						MES[pch].signal_count = i + 3;
					}
					break;		//受波波形を検出
				}
			}else{
				i = 3800;
				break;
			}
		}
	/*ゼロ点調整時の波形認識処理*/
	}else{	/*波形認識実行要求あり*/
		cnt = 0;
		for(i=2; i<(4000-200); i+= 1){	  /*3800word*/
			if(fifo_read_wait(pch) == B_OK){	/*受波データ読込み待機*/
				work = FIFO;
				work &= 0x1FFF;
				//受波検出閾値チェック(波形認識)
				if(work_old[1] <= MES[pch].ThresholdWave && MES[pch].ThresholdWave <= work){		/*下から上方向に交差*/
					cnt++;
				}
				if(work_old[1] >= MES[pch].ThresholdWave && MES[pch].ThresholdWave >= work){		/*上から下方向に交差*/
					cnt++;
				}
				if(MES[pch].ThresholdReq == 12 && cnt == 0 && work <= work_old[1] && MES[pch].ThresholdPeak <= work){  /*ピーク検出*/
					MES[pch].ThresholdPeak = work_old[1];	/*1つ前が受波波形ピーク*/
					MES[pch].signal_count = MES[pch].ThresholdPeakPos = i - 1;	 /*1つ前が受波波形ピーク位置*/
				}
				work_old[1] = work;

				if(cnt >= 5){  /*受波波形と受波検出閾値が5回以上交差した場合*/
					if(MES[pch].ThresholdReq == 11){  /*高速サーチの場合*/
						MES[pch].ThresholdWave += 10;
						MES[pch].ThresholdReq = 12;  /*波形認識閾値+10戻して詳細サーチに移行する*/
					}else if(MES[pch].ThresholdReq == 12){  /*詳細サーチの場合*/
						MES[pch].ThresholdReq = 99;  /*波形認識処理の終了*/
					}else{
						;
					}
					break;
				}
			}else{
				i = 3800;
				break;
			}
		}

		if(MES[pch].ThresholdReq == 11){
			MES[pch].ThresholdWave -= 3; /*波形認識閾値を3ずつ下げて検索する(高速サーチ)*/
		}else if(MES[pch].ThresholdReq == 12){
			MES[pch].ThresholdWave -= 1;  /*波形認識閾値を1ずつ下げて検索する(詳細サーチ)*/
		}else{
			;
		}
	}

	CheckEndPulse();		/*END信号確認*/
	SelectForwardOff(pch);		//IN/OUT打込み切替え

	if((MES[pch].ThresholdReq == 0)	/*波形認識実行要求なし*/
		&& (i == 3800)){			//3800word読込んでも受波波形が見つからない場合(Emptyエラー)
		non_signal_count_sv = (short)((float)SVD[pch].inc / 100 / 0.018);
		/*受波なしをカウント*/
		if(MES[pch].non_signal_count < non_signal_count_sv){  /*100回カウント*/
			MES[pch].non_signal_count++;
			MES[pch].amp_gain_for++;
			if(MES[pch].amp_gain_for > 255) MES[pch].amp_gain_for = 255;
			MES[pch].amp_gain_rev++;
			if(MES[pch].amp_gain_rev > 255) MES[pch].amp_gain_rev = 255;
		}else{
			MES[pch].err_status |= ERR_JUDGE_EMPTY;	/*エラーセット*/
			MES[pch].amp_gain_for = 255;
			MES[pch].amp_gain_rev = 255;
		}
		amp = MES[pch].amp_gain_for = MES[pch].amp_gain_rev;
		MES[pch].err_status |= ERR_JUDGE_PRE_EMPTY;	/*エラーセット*/
		ret = (short)B_NG;									/*信号なし*/
	}else{			//受波波形を見つけた場合
		MES[pch].non_signal_count = 0;
		MES[pch].err_status &= ~(ERR_JUDGE_EMPTY+ERR_JUDGE_PRE_EMPTY);	/*エラーリセット*/
		ret = (short)B_OK;									/*信号あり*/
	}

	return (ret);
}

/****************************************************/
/* Function : get_vth2                     */
/* Summary  : Vthの取得        */
/* Argument : data[], vth                        		 */
/* Return   : Vth       */
/* Caution  : なし                                  */
/* note     : なし                                 */
/****************************************************/
short get_vth2(short data[], short vth){
	unsigned short wave_level;
	short ptr, cross_ptr;
	short min1, min2;
	short mode;

	wave_level = (unsigned short)((long)AD_MAX * (long)vth / 100);

#if defined(FLWWAVEXP)
	for(ptr = 12; ptr < FLWWAVSIZ + 10; ptr++) 
#else
	for(ptr = 12; ptr < 210; ptr++) 
#endif
	{
		if(data[ptr] < wave_level) break;
	}
	cross_ptr = ptr;

#if defined(FLWWAVEXP)
	for(ptr = cross_ptr; ptr < FLWWAVSIZ + 10; ptr++) 
#else
	// for(ptr = 12; ptr < 210; ptr++) 
	for(ptr = cross_ptr; ptr < 210; ptr++) 
#endif
	{
		if(data[ptr] < data[ptr+1]) break;
	}
	min1 = data[ptr];

	mode = 0;
	for(ptr = cross_ptr; (ptr >= 12) && (mode <= 1); ptr--) {
		switch(mode) {
		case 0:		// search for max peak
			if(data[ptr] > data[ptr-1]) {
				mode = 1;
			}
			break;
		case 1:		// search for min peak
			if(data[ptr] < data[ptr-1]) {
				mode = 2;
			}
			break;
		default:
			break;
		}
	}
	min2 = data[ptr];
	return (min1 + min2);
}
/*******************************************
 * Function : GetWav (GetWave)
 * Summary  : 波形を取得する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 一時変数 : fwd_temp_data / rev_temp_data
 *            平均波形 : MES.fow_data / MES.rev_data
 * *****************************************/
void GetWav(short pch)
{
	short sample, i;
	short *FowWav, *RevWav;

	FowWav = (short *)(&MES[pch].fow_data[0]);
	RevWav = (short *)(&MES[pch].rev_data[0]);

	//    for (sample = 0; sample < 4; sample++)
    for (sample = 0; sample < MES_SUB[pch].sample_cnt; sample++)
	{
        delay(MES_SUB[pch].ItvVal); /*打ち込み後インターバル*/
// __bit_output(GPIO_PORTG_BASE, 6, 1);
		/*上流側FIFO処理*/
		DriveFIFOFwd(pch, sample);	//パルス打ち込み、FIFO読込み
		for (i = 12; i < 250; i++)
		{
			if(sample == 0){
				*(FowWav + i) = fwd_temp_data[sample][i];
			}
			else{
				*(FowWav + i) += fwd_temp_data[sample][i];
			}
		}

        delay(MES_SUB[pch].ItvVal); /*打ち込み後インターバル*/
// __bit_output(GPIO_PORTG_BASE, 6, 0);
		/*下流側FIFO処理*/
		DriveFIFORev(pch, sample);	//パルス打ち込み、FIFO読込み
		for (i = 12; i < 250; i++)
		{
			if(sample == 0){
				*(RevWav + i) = rev_temp_data[sample][i];
			}
			else{
				*(RevWav + i) += rev_temp_data[sample][i];
			}
		}
	}
}

/*******************************************
 * Function : SetFifoNo (Set FIFO Number)
 * Summary  : FIFOの位置情報を更新する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void SetFifoNo(short pch)
{
	short fix_signal_count[6];
	if (TopPosFix == PosFix_NotMov)
	{
		MES[pch].fifo_no = MES[pch].zero_signal_count - LDG_PNT_OFS; /*FIFO Read Address Point*/
	}
	else
	{
		// 固定値設定
		if ((SVD[pch].fix_data & 0x08) != 0)
		{
			if (SVD[pch].fix_fifo_no_read == 0)
			{
				MES[pch].fifo_no = MES[pch].zero_signal_count - LDG_PNT_OFS; /*FIFO Read Address Point*/
			}
			else
			{
				MES[pch].fifo_no = MES[pch].signal_count - LDG_PNT_OFS; /*FIFO Read Address Point*/
				if (MES[pch].fifo_no < 0)
				{
					MES[pch].fifo_no = 0;
					MES[pch].fifo_no_read = MES[pch].fifo_no;
					MES[pch].fifo_ch_read = MES[pch].fifo_ch;
				}
				while (MES[pch].fifo_no_read > (256 + 128))
				{
					MES[pch].fifo_no_read -= 256;
					MES[pch].fifo_ch_read += 1;
				}
				fix_signal_count[pch] = ((MES[pch].fifo_ch_read - MES[pch].fifo_ch) * 256) + SVD[pch].fix_fifo_no_read + LDG_PNT_OFS;
				MES[pch].fifo_no = fix_signal_count[pch] - LDG_PNT_OFS; /*FIFO Read Address Point*/
			}
		}
		else
		{
			MES[pch].fifo_no = MES[pch].signal_count - LDG_PNT_OFS; /*FIFO Read Address Point*/
		}
	}

	if (MES[pch].fifo_no < 0)	MES[pch].fifo_no = 0;

	MES[pch].fifo_no_read = MES[pch].fifo_no;
	MES[pch].fifo_ch_read = MES[pch].fifo_ch;	//FIFO CHサーチで決めたCH

	while (MES[pch].fifo_no_read > (256 + 128))
	{
		MES[pch].fifo_no_read -= 256;
//		MES[pch].fifo_ch_read += 1;
	}
}

/*******************************************
 * Function : SetFifoPos (Set FIFO Position)
 * Summary  : FIFO開始位置、終了位置を決定する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void SetFifoPos(short pch)
{
	if ((SVD[pch].fix_data & 0x04) != 0)
	{
		// 固定値設定
		if (SVD[pch].fix_fifo_ch_read == 0)
		{
			MES[pch].fifo_start = MES[pch].zero_fifo_ch_read; /*ゼロ点調整時のFIFO位置*/
			MES[pch].fifo_end = MES[pch].zero_fifo_ch_read + WS_FIFO_RANGE;
		}
		else
		{
			MES[pch].fifo_start = SVD[pch].fix_fifo_ch_read; /*固定設定時のFIFO位置*/
			MES[pch].fifo_end = SVD[pch].fix_fifo_ch_read + WS_FIFO_RANGE;
		}
	}
	else
	{
		MES[pch].fifo_start = MES[pch].fifo_ch_read;	/*WINDOW開始時間を設定*/
		MES[pch].fifo_end = MES[pch].fifo_ch_read + WS_FIFO_RANGE; /*WINDOW終了時間を設定*/
	}
}

/*******************************************
 * Function : GetSkpPnt (Get Skip Point)
 * Summary  : スキップする点数を決定する
 * Argument : Div : 1周期の1/Divの値を取得する
 * Return   : SkpPnt : スキップする点数
 * Caution  : None
 * Note     : 1周期の点数 = ADCの周波数[MHz] / 波形の周波数[kHz] / オーバーサンプリング数
 * *****************************************/
short GetSkpPnt(short pch, short Div)
{
	short SkpPnt;
	
	//FPGAオーバーサンプリング4点バージョン
	if((FpgaVersion == 0x2211) || (FpgaVersion == 0x3211))
	{
		SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 4 / Div;
	}
	//FPGAオーバーサンプリング8点バージョン
	else
	{
		SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / Div;
	}

	return SkpPnt;
}

/*******************************************
 * Function : SchZerDat (Search Zero cross Data)
 * Summary  : ゼロクロス点を検索する
 * Argument : pch : チャンネル番号
 *          : *WavPtr : 検索対象の波形データ配列
 *          : *ZerPnt : 検索結果(ゼロクロス点)を入れるデータ配列
 *          : *ZerDat1 : 検索結果(ゼロクロスデータ)を入れるデータ配列
 *          : *ZerDat2 : 検索結果(ゼロクロスデータ)を入れるデータ配列
 *          : *ZerDat3 : 検索結果(ゼロクロスデータ)を入れるデータ配列
 *          : *ZerDat4 : 検索結果(ゼロクロスデータ)を入れるデータ配列
 * Return   : void
 * Caution  : None
 * Note     : 
 * *****************************************/
void SchZerDat(short pch, short *WavPtr, short *ZerPnt, short *ZerDat1, short *ZerDat2, short *ZerDat3, short *ZerDat4)
{
	short i, ZerCnt = 0;
	short Wav1, Wav2, Wav3, Wav4;
	short FndFlg = -1;
	short SkpPnt;

	// SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / 4;
	SkpPnt = GetSkpPnt(pch, 4);

	// i = 14;
	i = SVD[pch].ZerPeakPos;
	if(i < 14) i = 14;
	while(i < 250)
	{
		if(ZerCnt < ZC_POINT_MAX)
		{
			Wav1 = *(WavPtr + i - 2);
			Wav2 = *(WavPtr + i - 1);
			Wav3 = *(WavPtr + i + 0);
			Wav4 = *(WavPtr + i + 1);

			if(
				((Wav2 >= AD_BASE) && (Wav3 < AD_BASE))
				|| ((Wav2 <= AD_BASE) && (Wav3 > AD_BASE))
			)
			{
				*(ZerPnt + ZerCnt) = i - 1;
				*(ZerDat1 + ZerCnt) = Wav1 - AD_BASE;
				*(ZerDat2 + ZerCnt) = Wav2 - AD_BASE;
				*(ZerDat3 + ZerCnt) = Wav3 - AD_BASE;
				*(ZerDat4 + ZerCnt) = Wav4 - AD_BASE;
				ZerCnt++;
				FndFlg = 1;
			}
		}
		else
		{
			break;
		}
		//見つかっていればループ短縮のためにiを飛ばす
		if(FndFlg == 1)
		{
			//飛ばす点数 = 受波波長(f=600kHz) * サンプリング周波数(オーバーサンプリング8点) /(1/4波長)
			//3.3854 = 1/600kHz * 65MHz/8 /4
			// i = (short)((float)i + 3.3854);
			i += SkpPnt;
			FndFlg = 0;
		}
		else
		{
			i++;
		}
	}
}

/*******************************************
 * Function : SchTrshld (Search Threashold point)
 * Summary  : スレッシュホールド値を検索する
 * Argument : pch : チャンネル番号
 *          : *WavPtr : 検索対象の波形データ配列
 *          : *OutVal : 検索結果（スレッシュホールド値）
 * Return   : void
 * Caution  : None
 * Note     : 
 * *****************************************/
void SchTrshld(short pch, short *WavPtr, short *OutVal)
{
	short i;
	short Trshld = (short)((long)AD_MAX * (long)SVD[pch].wave_vth / 100);
	short TrshldPos;
	short TrshldVal;
	short WavVal;
	for(i = 12; i < 250; i++)
	{
		WavVal = *(WavPtr + i);
		if(WavVal < Trshld){
			TrshldPos = i;
			TrshldVal = WavVal;
			break;
		}
	}
	*OutVal = TrshldPos;
}

/*******************************************
 * Function : SchMaxPek (Search Max Peak point)
 * Summary  : 極大値を検索する
 * Argument : pch : チャンネル番号
 *          : *WavPtr : 検索対象の波形データ配列
 *          : *OutVal : 検索結果(極大値)
 *          : *OutPos : 検索結果(極大位置)
 * Return   : void
 * Caution  : None
 * Note     : 
 * *****************************************/
void SchMaxPek(short pch, short *WavPtr, short *OutVal, short *OutPos)
{
	short i = 13;
	short MaxPekCnt = 0;
	short FndFlg = -1;
	short DatM1, Dat0, DatP1;
	short SkpPnt;

	//飛ばす点数 = 受波波長(f=600kHz) * サンプリング周波数(オーバーサンプリング8点) = 1/600kHz * 65MHz/8
	SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / 2; //Skip Point (デフォルトで13.54)
	SkpPnt = GetSkpPnt(pch, 2);
	
	FndFlg = -1;
	while(i < 250)
	{
		DatM1 = *(WavPtr + i - 1);
		Dat0 = *(WavPtr + i);
		DatP1 = *(WavPtr + i + 1);
		//極大値
		if(MaxPekCnt < WAV_PEK_NUM)
		{
			if(DatM1 < Dat0)
			{
				if(Dat0 >= DatP1)
				{
					OutVal[MaxPekCnt] = Dat0;
					OutPos[MaxPekCnt] = i;
					MaxPekCnt++;
					FndFlg = 1;
				}
			}
		}
		else
		{
			break;
		}
		
		//見つかっていればループ短縮のためにiを飛ばす
		if(FndFlg == 1)
		{
			i += SkpPnt;
			FndFlg = 0;
		}
		else
		{
			i++;
		}
	}
}

/*******************************************
 * Function : SchMinPek (Search Min Peak point)
 * Summary  : 極小値を検索する
 * Argument : pch : チャンネル番号
 *          : *WavPtr : 検索対象の波形データ配列
 *          : *OutVal : 検索結果(極小値)
 *          : *OutPos : 検索結果(極小位置)
 * Return   : void
 * Caution  : None
 * Note     : 
 * *****************************************/
void SchMinPek(short pch, short *WavPtr, short *OutVal, short *OutPos)
{
	short i = 13;
	short MinPekCnt = 0;
	short FndFlg = -1;
	short DatM1, Dat0, DatP1;
	short SkpPnt;

	//飛ばす点数 = 受波波長(f=600kHz) * サンプリング周波数(オーバーサンプリング8点) = 1/600kHz * 65MHz/8
	// SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / 2; //Skip Point (デフォルトで13.54)
	SkpPnt = GetSkpPnt(pch, 2);

	FndFlg = -1;
	while(i < 250)
	{
		DatM1 = *(WavPtr + i - 1);
		Dat0 = *(WavPtr + i);
		DatP1 = *(WavPtr + i + 1);
		//極小値
		if(MinPekCnt < WAV_PEK_NUM)
		{
			if(DatM1 > Dat0)
			{
				if(Dat0 <= DatP1)
				{
					OutVal[MinPekCnt] = Dat0;
					OutPos[MinPekCnt] = i;
					MinPekCnt++;
					FndFlg = 1;
				}
			}
		}
		else
		{
			break;
		}
		
		//見つかっていればループ短縮のためにiを飛ばす
		if(FndFlg == 1)
		{
			i += SkpPnt;
			FndFlg = 0;
		}
		else
		{
			i++;
		}
	}
}

/*******************************************
 * Function : GetFwdAnyPnt (Get Foward Any Point)
 * Summary  : 上流波形の極値/ゼロクロス点/しきい値位置を探す
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void GetFwdAnyPnt(short pch)
{
	//ゼロクロス点を検索する
	SchZerDat(pch, &MES[pch].fow_data[0], &MES[pch].zc_nearzero_point[0][0], &MES[pch].zc_nearzero_data1[0][0], &MES[pch].zc_nearzero_data2[0][0], &MES[pch].zc_nearzero_data3[0][0], &MES[pch].zc_nearzero_data4[0][0]);
	
	SchMaxPek(pch, &MES[pch].fow_data[0], &MES[pch].FwdWavMaxPekValLst[0], &MES[pch].FwdWavMaxPekPosLst[0]);
	SchMinPek(pch, &MES[pch].fow_data[0], &MES[pch].FwdWavMinPekValLst[0], &MES[pch].FwdWavMinPekPosLst[0]);
	
	SchTrshld(pch, &MES[pch].fow_data[0], &MES[pch].ThreasholdPoint_Fow);
}

/*******************************************
 * Function : GetRevAnyPnt (Get Reverse Any Point)
 * Summary  : 下流波形の極値/ゼロクロス点/しきい値位置を探す
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void GetRevAnyPnt(short pch)
{
	//ゼロクロス点を検索する
	SchZerDat(pch, &MES[pch].rev_data[0], &MES[pch].zc_nearzero_point[1][0], &MES[pch].zc_nearzero_data1[1][0], &MES[pch].zc_nearzero_data2[1][0], &MES[pch].zc_nearzero_data3[1][0], &MES[pch].zc_nearzero_data4[1][0]);
	
	SchMaxPek(pch, &MES[pch].rev_data[0], &MES[pch].RevWavMaxPekValLst[0], &MES[pch].RevWavMaxPekPosLst[0]);
	SchMinPek(pch, &MES[pch].rev_data[0], &MES[pch].RevWavMinPekValLst[0], &MES[pch].RevWavMinPekPosLst[0]);
	
	SchTrshld(pch, &MES[pch].rev_data[0], &MES[pch].ThreasholdPoint_Rev);
}

/*******************************************
 * Function : SearchZerPeakPos
 * Summary  : ZerPeakPosを探す
 * Argument : pch : チャンネル番号
 *            StartPos : 探索開始点
 *            EndPos : 探索終了点
 * Return   : zerPeakPos
 * Caution  : None
 * Note     : 
 * *****************************************/
short SearchZerPeakPos(short pch, short StartPos, short EndPos)
{
//指定範囲内の極大値を探すver
#if 0
	short i;
	short zerPeakPos = 0;
	for(i = 0; i < WAV_PEK_NUM)
	{
		if(
			(StartPos <= MES[pch].FwdWavMaxPekPosLst[i]) 
			&& (MES[pch].FwdWavMaxPekPosLst[i] < EndPos)
		)
		{
			zerPeakPos = MES[pch].FwdWavMaxPekValLst[i];
			break;
		}
	}
//指定範囲内の極値を探すver
#elif 0
	short i;
	short zerPeakPos = 0;
	short PeakVal[2 * WAV_PEK_NUM];
	short PeakPos[2 * WAV_PEK_NUM];

	//極大極小混合リスト作成
	for(i = 0; i < WAV_PEK_NUM; i++)
	{
		if(MES[pch].FwdWavMaxPekPosLst[i] < MES[pch].FwdWavMinPekPosLst[i])
		{
			PeakPos[2 * i + 0] = MES[pch].FwdWavMaxPekPosLst[i];
			PeakVal[2 * i + 0] = MES[pch].FwdWavMaxPekValLst[i];
			PeakPos[2 * i + 1] = MES[pch].FwdWavMinPekPosLst[i];
			PeakVal[2 * i + 1] = MES[pch].FwdWavMinPekValLst[i];
		}
		else
		{
			PeakVal[2 * i + 0] = MES[pch].FwdWavMinPekValLst[i];
			PeakPos[2 * i + 0] = MES[pch].FwdWavMinPekPosLst[i];
			PeakVal[2 * i + 1] = MES[pch].FwdWavMaxPekValLst[i];
			PeakPos[2 * i + 1] = MES[pch].FwdWavMaxPekPosLst[i];
		}
	}

	for(i = 0; i < WAV_PEK_NUM)
	{
		if(
			(StartPos <= PeakPos[i]) 
			&& (PeakPos[i] < EndPos)
		)
		{
			zerPeakPos = PeakVal[i];
			break;
		}
	}
#else
	short j;
	short TrshldVal = 0;
	short TrshldPos = 0;
	short zerPeakPos;
	for(j = 0; j < WAV_PEK_NUM; j++){
		if(TrshldVal < MES[pch].RevWavMaxPekValLst[j])
		{
			TrshldVal = MES[pch].RevWavMaxPekValLst[j];
			TrshldPos = j;
		}
	}
	TrshldPos--;
	zerPeakPos = MES[pch].RevWavMaxPekPosLst[TrshldPos];
#endif
	return zerPeakPos;
}

/*******************************************
 * Function : SchMaxMinPnt (Search Max Min Point)
 * Summary  : 最大/最小値を検索する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void SchMaxMinPnt(short pch)
{
	short work_old = 0, i, j;
	short SelMult[] = { 32, 40, 65, 80 }; // 現行クランプオンではサンプリング周波数に近似?
	short ZerAdjYetFlg = 0; //0:ゼロ調整実施, 1:ゼロ調整未実施
	short CrsCnt = 0;
	short TrshldVal = 0;
	short TrshldPos = 0;

	SelMult[SVD[pch].adc_clock] = SelMult[SVD[pch].adc_clock] / Mgn;

	// MES[pch].ThresholdPeakPos != 0 : ゼロ点調整中に波形と5点交わる点が見つかっている
	// MES_SUB[pch].zc_peak_req == 1 : エラーが起きずにゼロ点調整が終了している
	// SVD[pch].ZerPeakPos == 0 : エラーが起きずにゼロ点調整が終了している(zc_peakはゼロ点調整終了時に0リセットされる)
	// if(MES[pch].ThresholdPeakPos != 0 && MES_SUB[pch].zc_peak_req == 1 && SVD[pch].ZerPeakPos == 0 && MES[pch].zc_peak_UpdateFlg != 0)
	//MES_SUB[pch].zc_peak_req == 1 : エラーが起きずにゼロ点調整が終了している
	//MES[pch].zc_peak_UpdateFlg != 0 : 通信によるzc_peak更新要求がある
	if(MES_SUB[pch].zc_peak_req == 1 && MES[pch].zc_peak_UpdateFlg != 0)
	{
		MES_SUB[pch].zc_peak_req = 0;	//波形認識閾値付近のピーク位置を検索要求をクリア
		ZerAdjYetFlg = 1;
	}
	
	fow_max = rev_max = 0;					   /*初期値*/
	fow_min = rev_min = AD_MAX;				   /*12bit MAX*/
	work_old = 0;

	//最大最小値検索
	for(i = 0; i < WAV_PEK_NUM; i++)
	{
		if(fow_max < MES[pch].FwdWavMaxPekValLst[i])
		{
			fow_max = MES[pch].FwdWavMaxPekValLst[i];
		}
		if(fow_min > MES[pch].FwdWavMinPekValLst[i])
		{
			fow_min = MES[pch].FwdWavMinPekValLst[i];
		}

		if(rev_max < MES[pch].RevWavMaxPekValLst[i])
		{
			rev_max = MES[pch].RevWavMaxPekValLst[i];
		}
		if(rev_min > MES[pch].RevWavMinPekValLst[i])
		{
			rev_min = MES[pch].RevWavMinPekValLst[i];
		}
	}

	//ゼロ調整未実施の場合
	if(ZerAdjYetFlg != 0)
	{
		//0~250の間でZerPeakPosを探す
		SVD[pch].ZerPeakPos = SearchZerPeakPos(pch, 0, 250);

		SVD[pch].ZerCrsSttPnt = SVD[pch].ZerPeakPos;		//波形認識ピーク位置(ゼロクロス計算開始位置用)のEEPROM保存
		eep_write_ch_delay(pch, (short)(&SVD[pch].ZerPeakPos - &SVD[pch].max_flow), SVD[pch].ZerPeakPos);
		eep_write_ch_delay(pch, (short)(&SVD[pch].ZerCrsSttPnt - &SVD[pch].max_flow), SVD[pch].ZerCrsSttPnt);
	}
}

/*******************************************
 * Function : SetWavTopPos (Set Wave Top Position)
 * Summary  : 波形先頭位置を計算する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 波形最大値の50%点に近い極大値周辺の3点から回帰直線で計算する
 * *****************************************/
void SetWavTopPos(short pch)
{
	short CenVal = 0, CenPos = 0;
	short work = 0;
	short i;
	short OffsetPoint[] = { 2355, 2944, 4784, 5888 };

	//1. 下流波形
	CenVal = (rev_max - AD_BASE) / 2;
	//1-1. 波形前半だけ検索
	for(i = 0; i < WAV_PEK_NUM / 2; i++)
	{
		work = abs(MES[pch].RevWavMaxPekValLst[i] - CenVal);
		if(work < CenVal){
			CenVal = work;
			CenPos = i;
		}
	}
	//1-2. 昇順確認
	if((MES[pch].RevWavMaxPekPosLst[CenPos - 1] < MES[pch].RevWavMaxPekPosLst[CenPos]) && (MES[pch].RevWavMaxPekPosLst[CenPos] < MES[pch].RevWavMaxPekPosLst[CenPos + 1]))
	{
		MES[pch].m0_point_rev_50 = (MES[pch].RevWavMaxPekValLst[CenPos - 1] + MES[pch].RevWavMaxPekValLst[CenPos] + MES[pch].RevWavMaxPekValLst[CenPos + 1]) / 3;
		MES[pch].sonic_point_rev_p2 =(short)((long)MES[pch].m0_point_rev_50 * 100) - OffsetPoint[SVD[pch].adc_clock];
		MES[pch].sonic_point_rev_p2 *= Mgn;
	}
	//2. 上流波形
	CenVal = (fow_max - AD_BASE) / 2;
	//2-1. 波形前半だけ検索
	for(i = 0; i < WAV_PEK_NUM / 2; i++)
	{
		work = abs(MES[pch].FwdWavMaxPekValLst[i] - CenVal);
		if(work < CenVal){
			CenVal = work;
			CenPos = i;
		}
	}
	//2-2. 昇順確認
	if((MES[pch].FwdWavMaxPekPosLst[CenPos - 1] < MES[pch].FwdWavMaxPekPosLst[CenPos]) && (MES[pch].FwdWavMaxPekPosLst[CenPos] < MES[pch].FwdWavMaxPekPosLst[CenPos + 1]))
	{
		MES[pch].m0_point_fow_50 = (MES[pch].FwdWavMaxPekValLst[CenPos - 1] + MES[pch].FwdWavMaxPekValLst[CenPos] + MES[pch].FwdWavMaxPekValLst[CenPos + 1]) / 3;
		MES[pch].sonic_point_fow_p1 =(short)((long)MES[pch].m0_point_fow_50 * 100) - OffsetPoint[SVD[pch].adc_clock];
		MES[pch].sonic_point_fow_p1 *= Mgn;
	}
}

/*******************************************
 * Function : ClcDifWavPos (Calculate Difference from both Wave Position)
 * Summary  : 受波波形の差を計算する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 極大/極小値, 最大/最小値, ゼロクロス点を探す
 * *****************************************/
void ClcDifWavPos(short pch)
{
	if((SVD[pch].fix_data & 0x10) != 0)
	{
		MES[pch].max_point_sub = (MES[pch].zero_sonic_point_fow_p1 - MES[pch].zero_sonic_point_rev_p2 + 50) / 100;		/*受波の差*/
	}
	else
	{
		MES[pch].max_point_sub = (MES[pch].sonic_point_fow_p1 - MES[pch].sonic_point_rev_p2 + 50) / 100;		/*受波の差*/
		MES[pch].max_point_sub *= Mgn;
	}
	
	//max_point_sub_fを計算する
	max_point_control(pch);

	/*波形認識実行時はチェックしない*/
	if(MES[pch].ThresholdReq == 0)
	{
		if(MES[pch].max_point_sub_f >= LIM_OVERFLOW)
		{
			;//オーバーフロー
		}
		else
		{
			MES[pch].err_status &= ~(ERR_JUDGE_REVERSE + ERR_JUDGE_OVERFLOW);	/*エラーリセット*/
		}
	}
}

/*******************************************
 * Function : AnalyzeWave
 * Summary  : 取得した波形データを解析する
 * Argument : pch : チャンネル番号
 * Return   : 
 * Caution  : None
 * Note     : 極大/極小値, 最大/最小値, ゼロクロス点を探す
 * *****************************************/
void AnalyzeWave(short pch)
{
	short i;
	//変数初期化
	memset(MES[pch].FwdWavMaxPekPosLst, 300, sizeof(MES[pch].FwdWavMaxPekPosLst));
	memset(MES[pch].FwdWavMinPekPosLst, 300, sizeof(MES[pch].FwdWavMinPekPosLst));
	memset(MES[pch].RevWavMaxPekPosLst, 300, sizeof(MES[pch].RevWavMaxPekPosLst));
	memset(MES[pch].RevWavMinPekPosLst, 300, sizeof(MES[pch].RevWavMinPekPosLst));
	memset(MES[pch].FwdWavMaxPekValLst, AD_BASE, sizeof(MES[pch].FwdWavMaxPekValLst));
	memset(MES[pch].FwdWavMinPekValLst, AD_BASE, sizeof(MES[pch].FwdWavMinPekValLst));
	memset(MES[pch].RevWavMaxPekValLst, AD_BASE, sizeof(MES[pch].RevWavMaxPekValLst));
	memset(MES[pch].RevWavMinPekValLst, AD_BASE, sizeof(MES[pch].RevWavMinPekValLst));
	memset(MES[pch].zc_nearzero_point, 0, sizeof(MES[pch].zc_nearzero_point));

	//上流波形
	GetFwdAnyPnt(pch);

	//下流波形
	GetRevAnyPnt(pch);

	MES[pch].ThreasholdPoint = (MES[pch].ThreasholdPoint_Fow + MES[pch].ThreasholdPoint_Rev - 20) / 2;
	
	// 音速固定なので以下の処理は不要
	// ClcDifWavPos(pch);

	//波形の最大/最小値を検索する
	SchMaxMinPnt(pch);

	//波形先頭位置計算
	SetWavTopPos(pch);
}


/****************************************************/
/* Function : fifo_read								*/
/* Summary  : FIFOから200byte読込む					*/
/* Argument : pch									*/
/* Return   : なし									*/
/* Caution  : 処理高速化のためにポインタ処理に変更
 * notes    : fifo_no はFIFOに取り込まれるデータから
 *          : LeadingPosition(受波バッファの先頭から10点目)までの点数
 ****************************************************/
void fifo_read(short pch)
{
	short i, j;
	short *FowWav1, *FowWav2, *FowWav3, *FowWav4;
	short *RevWav1, *RevWav2, *RevWav3, *RevWav4;
	short *FowWav, *RevWav;
	short fifo_ch;
	unsigned short alm_level;

	FowWav1 = (short *)(&fwd_temp_data[0][0]);
	FowWav2 = (short *)(&fwd_temp_data[1][0]);
	FowWav3 = (short *)(&fwd_temp_data[2][0]);
	FowWav4 = (short *)(&fwd_temp_data[3][0]);
	RevWav1 = (short *)(&rev_temp_data[0][0]);
	RevWav2 = (short *)(&rev_temp_data[1][0]);
	RevWav3 = (short *)(&rev_temp_data[2][0]);
	RevWav4 = (short *)(&rev_temp_data[3][0]);

	FowWav = (short *)(&MES[pch].fow_data[0]);
	RevWav = (short *)(&MES[pch].rev_data[0]);

	/*位相を計測毎に1/4ずつずらしていく (ディザリング機能)*/
	MES[pch].clk_phase++; // 位相更新(PhaseShift 0→90→180→270→0→)
	if (MES[pch].clk_phase > 3)
	{
		MES[pch].clk_phase = 0; // 位相初期化
	}

	/*上流/下流交互に4回ずつ打込む処理の前に波形有無を確認する*/
	fifo_ch = ch_search(pch);
	if (fifo_search(pch, fifo_ch) == B_NG)
	{
		// 信号なしの場合
		fow_max = rev_max = 0;	  /*初期値*/
		fow_min = rev_min = AD_MAX; /*12bit MAX*/

#if defined(FLWWAVEXP)
		for (i = 12; i < FLWWAVSIZ + 10; i++)
#else
		for (i = 12; i < 250; i++)
#endif
		{
			/*最大、最小を探す*/
			if (fow_max < *(FowWav + i))
			{ /*fow data max*/
				fow_max = *(FowWav + i);
				fow_max_point = i;
			}
			if (fow_min > *(FowWav + i))
			{ /*fow data min*/
				fow_min = *(FowWav + i);
			}
			if (rev_max < *(RevWav + i))
			{ /*rev data max*/
				rev_max = *(RevWav + i);
				rev_max_point = i;
			}
			if (rev_min > *(RevWav + i))
			{ /*rev data min*/
				rev_min = *(RevWav + i);
			}
		}
		return;
	}

#if defined(ShtItv)
		SelectReverseOn(pch);		//IN/OUT打込み切替え
#endif

	// MES.fifo_no, MES.fifo_no_read, MES.fifo_ch_readを決める
	SetFifoNo(pch);

	// MES.fifo_start, MES.fifo_endを決める
	SetFifoPos(pch);

	MES[pch].fifo_offset = SVD[pch].wind_offset; /*WINDOWオフセット時間を設定*/
	SetFpgaRegister(pch);   //FPGAレジスタの設定

	// 波形を取得する
	GetWav(pch);

	// 取得した波形を解析する
	AnalyzeWave(pch);

	if (LED[pch].vth_do_cnt != 0)
	{																			// ゼロ点調整中
		MES[pch].vth_sum += get_vth2(&MES[pch].fow_data[0], SVD[pch].wave_vth); // ゼロ点調整用Vthを取得する
		MES[pch].vth_sum += get_vth2(&MES[pch].rev_data[0], SVD[pch].wave_vth);
		MES[pch].vth_count++;
	}

	MES[pch].alm_status &= ~(ALM_JUDGE_EMPTY);
	if ((1 <= SVD[pch].alm_hold_time) && (SVD[pch].alm_hold_time <= 99))
	{ // 警告出力時間設定あり
		alm_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].alm_wave_vth / 100); /*受波検出警告閾値*/
		if (alm_level <= fow_min)
		{											// 受波検出警告閾値 ≦ ﾋﾟｰｸ値
			MES[pch].alm_status |= ALM_JUDGE_EMPTY; /*エンプティセンサ警告セット*/
		}
	}

}
/****************************************************
 * Function : GetTimDif
 * Summary  : 上流、下流の打ち込みからゼロクロス点までの時間差を求める
 * Argument : pch : チャンネル番号
 * Return   : TimDif : 打ち込みからゼロクロス平均点までの時間差
 * Caution  : 
 * Notes    : 時間差=(打ち込み~FIFOまでの時間) + (波形の先頭~ゼロクロス点までの時間)
 *          : 波形の先頭~ゼロクロス点までの時間はオーバーサンプリングされているのでx8
 ****************************************************/
float GetTimDif(short pch, short Mod)
{
	short SmpTim = 0;
	short SglCnt = MES[pch].signal_count; //FIFO先頭から波形検出しきい値までの点数
	short Trsold = 0; //受信バッファから波形検出しきい値までの点数
	float TimDif = 0.0;

	//UP
	if(Mod == 0){
		SmpTim = 1000 * 1000 / AdcSmpFrq[SVD[pch].adc_clock]; //1 = 0.0001ns
		Trsold = MES[pch].ThreasholdPoint_Fow;
		// TimDif = (float)(SmpTim * SVD[pch].fifo_ch_init * 256 / 1000.0) + (float)(SglCnt - Trsold + MES_SUB[pch].zc_Tup) * 8.0;
		TimDif = MES_SUB[pch].zc_Tup * 1000;
	}
	//DOWN
	else{
		SmpTim = 1000 * 1000 / AdcSmpFrq[SVD[pch].adc_clock]; //1 = 0.0001ns
		Trsold = MES[pch].ThreasholdPoint_Rev;
		// TimDif = (float)(SmpTim * SVD[pch].fifo_ch_init * 256 / 1000.0) + (float)(SglCnt - Trsold + MES_SUB[pch].zc_Tdown) * 8.0;
		TimDif = MES_SUB[pch].zc_Tdown * 1000;
	}
	return TimDif / 1000.0; //ns
}

//Sinカーブを64分割したテーブル
const float CmpTbl[65] =
{
    0.00000000F,    0.01587280F,    0.03172172F,    0.04754757F,
    0.06335117F,    0.07913333F,    0.09489484F,    0.11063651F,
    0.12635913F,    0.14206349F,    0.15775038F,    0.17342058F,
    0.18907488F,    0.20471404F,    0.22033885F,    0.23595008F,
    0.25154849F,    0.26713485F,    0.28270994F,    0.29827450F,
    0.31382929F,    0.32937509F,    0.34491264F,    0.36044270F,
    0.37596601F,    0.39148335F,    0.40699544F,    0.42250305F,
    0.43800692F,    0.45350781F,    0.46900645F,    0.48450360F,
    0.50000000F,    0.51549640F,    0.53099355F,    0.54649219F,
    0.56199308F,    0.57749695F,    0.59300456F,    0.60851665F,
    0.62403399F,    0.63955730F,    0.65508736F,    0.67062491F,
    0.68617071F,    0.70172550F,    0.71729006F,    0.73286515F,
    0.74845151F,    0.76404992F,    0.77966115F,    0.79528596F,
    0.81092512F,    0.82657942F,    0.84224962F,    0.85793651F,
    0.87364087F,    0.88936349F,    0.90510516F,    0.92086667F,
    0.93664883F,    0.95245243F,    0.96827828F,    0.98412720F,
    1.00000000F
};
/****************************************************
 * Function : CalFx
 * Summary  : 2点からsinカーブを利用してゼロクロス点を求める(CLFC演算方法1)
 * Argument : y0 : 1点目y座標
 *            y1 : 2点目y座標
 * Return   : fw : 補正後のゼロクロス点
 * Caution  : 
 * notes    : (0,y0),(1,y1)の2点をsinカーブで補正する
 ****************************************************/
/////// float   CalFx(float y0, float y1) ////////////////////
float   CalFx(float y0, float y1)
{
float   fW;
short   ii;
short   jj;
    fW = y0 / (y0 - y1);
    ii = 0;
    for(jj=32; jj>0; jj>>=1)
    {
        if(fW >= CmpTbl[ii + jj])
        {   ii += jj;                   }
    }
	// 0.015625 = 1/64
    fW = 0.015625F * (float)ii + 0.015625F * (fW - CmpTbl[ii]) / (CmpTbl[ii + 1] - CmpTbl[ii]);
    return  fW;
}       //  End of CalFx()

/****************************************************
 * Function : CalRegLin (Caluculate Regression Line)
 * Summary  : 4点から求めた回帰直線でゼロクロス点を計算する(CLFCの演算方法2)
 * Argument : xn : x座標
 *            yn : y座標
 * Return   : x : 補正後のゼロクロス点
 * Caution  : 
 * notes    : x1を基準(0)とした回帰直線からゼロクロス点を求め、x1との和をとる
 *            回帰直線 : y = ax + b
 *            Sum(x) = 0 + 1 + 2 + 3 = 6
 *            Sum(x^2) = 14
 *            Sum(x)^2 = 36
 *            N = 4
 *            a = Sum(xy) / Sum(xx)   = (Sum(x) * Sum(y) - N * Sum(xy)) / (Sum(x)^2 - N * Sum(x^2))        = (6 * Sum(y) - 4 * Sum(xy)) / 20
 *            b = Ave(y) - a * Ave(x) = (Sum(x) * Sum(xy) - Sum(y) * Sum(x^2)) / (Sum(x)^2 - N * Sum(x^2)) = (6 * Sum(xy) - 14 * Sum(y)) / 20
 ****************************************************/
float CalRegLin(float x1, float x2, float x3, float x4, float y1, float y2, float y3, float y4)
{
	float Sy = y1 + y2 + y3 + y4;
	float Sxy = y2 + 2 * y3 + 3 * y4;

	float a = 6 * Sy - 4 * Sxy;
	float b = 6 * Sxy - 14 * Sy;

	float x = x1 - b / a;

	return x;
}

/****************************************************
 * Function : CorZrcPnt (Correct Zero cross Point)
 * Summary  : ゼロクロス点を補正する
 * Argument : xn : x座標
 *            yn : y座標
 * Return   : x : 補正後のゼロクロス点
 * Caution  : 
 * notes    : METHOD_OLDでは、2点ずつのゼロクロス補正点の和を別のプロセスで使用していたので
 *            互換をとるためにtmp_x1とtmp_x2の和を返す
 ****************************************************/
float CorZrcPnt(short x1, short x2, short x3, short x4, short y1, short y2, short y3, short y4)
{
	float x, tmp_x1, tmp_x2;
#if defined(METHOD_OLD)
	tmp_x1 = x1 - (y1 * ((float)(x3 - x1) / (y3 - y1)));	//1点目と3点目の回帰直線
	tmp_x2 = x2 - (y2 * ((float)(x4 - x2) / (y4 - y2)));	//2点目と4点目の回帰直線
#elif defined(METHOD_CLFC1)
	tmp_x1 = x2 + CalFx((float)y2, (float)y3);
	tmp_x2 = tmp_x1;
#else
	tmp_x1 = CalRegLin((float)x1, (float)x2, (float)x3, (float)x4, (float)y1, (float)y2, (float)y3, (float)y4);
	tmp_x2 = tmp_x1;
#endif
	x = tmp_x1 + tmp_x2;
	return x;
}

/****************************************************
 * Function : DelAbnPnt (Delete Abnormal Point)
 * Summary  : 異常な点を取り除いて時間差を再計算する
 * Argument : *FwdPnt : 上流ゼロクロス点
 *            *RevPnt : 下流ゼロクロス点
              zc_SumOld : 上下流時間差の和
			  zc_num : ゼロクロス点使用点数
			  SmpTdt : サンプリング時間
 * Return   : zc_TdataNew : 異常点を除いた平均時間差
 * Caution  : 
 * notes    : 
 ****************************************************/
float DelAbnPnt(float *FwdPnt, float *RevPnt, float zc_SumOld, short zc_num, float SmpTdt)
{
	short i;
	float zc_TdataAve; //平均値
	float zc_TdataDif; //平均値からの差
	float zc_TdataDif1; //平均値からの差
	float zc_TdataDif2;
	float zc_TdataMax1; //平均値からの差が最大の点
	float zc_TdataMax2;
	float zc_TdataWork; //N番目の時間差
	short zc_TdataMax1Idx;
	float zc_SumNew;
	float zc_TdataNew;

	i = zc_TdataDif = zc_TdataMax1 = zc_TdataMax2 = 0;
	zc_TdataAve = zc_SumOld / zc_num;	//伝搬時間差の平均値(ゼロクロス点分)
	//最大値を探す
	for(i = 0; i < zc_num; i++)
	{
		// zc_TdataWork = (FwdPnt[i] - RevPnt[i]) * SmpTdt / 2.0 / zc_num; 
		zc_TdataWork = (FwdPnt[i] - RevPnt[i]) * SmpTdt / 2.0; 
		zc_TdataDif = zc_TdataWork - zc_TdataAve;
		if(zc_TdataDif < 0) zc_TdataDif *= -1;
		if(zc_TdataDif1 < zc_TdataDif)
		{
			zc_TdataDif1 = zc_TdataDif;
			zc_TdataMax1 = zc_TdataWork;
			zc_TdataMax1Idx = i;
		}
	}
	//次点の最大値を探す
	for(i = 0; i < zc_num; i++)
	{
		if(i == zc_TdataMax1Idx) continue; //飛ばして次へ
		// zc_TdataWork = (FwdPnt[i] - RevPnt[i]) * SmpTdt / 2.0 / zc_num; 
		zc_TdataWork = (FwdPnt[i] - RevPnt[i]) * SmpTdt / 2.0; 
		zc_TdataDif = zc_TdataWork - zc_TdataAve;
		if(zc_TdataDif < 0) zc_TdataDif *= -1;
		if(zc_TdataDif2 < zc_TdataDif)
		{
			zc_TdataDif2 = zc_TdataDif;
			zc_TdataMax2 = zc_TdataWork;
		}
	}

	//突出点を除いて再計算
	//最大値が閾値未満
	if(zc_TdataDif1 < 0.0001)
	{
		zc_SumNew = zc_SumOld;
		zc_TdataNew = zc_SumNew / zc_num;
	}
	//次点の最大値が閾値未満
	else if(zc_TdataDif2 < 0.0001)
	{
		zc_SumNew = zc_SumOld - zc_TdataMax1;
		zc_TdataNew = zc_SumNew / (zc_num - 1);
	}
	//次点の最大値も閾値以上
	else
	{
		zc_SumNew = zc_SumOld - zc_TdataMax1 - zc_TdataMax2;
		zc_TdataNew = zc_SumNew / (zc_num - 2);
	}

	return zc_TdataNew;
}

float DelAbnPntMod(float *FwdPnt, float *RevPnt, float zc_SumOld, short zc_num, float SmpTdt)
{
	short i;
	float zc_SumNew;
	float zc_TdataNew;
	float zc_TdataWorkSum;
	float zc_TdataWorkBuf[ZC_POINT_MAX];
	short zc_TdataWorkCnt;

	zc_TdataWorkSum = zc_TdataWorkCnt = 0;
	for(i = 0; i < zc_num; i++)
	{
		zc_TdataWorkBuf[i] = (FwdPnt[i] - RevPnt[i]) * SmpTdt / 2.0;
		if(-0.00015 <= zc_TdataWorkBuf[i] && zc_TdataWorkBuf[i] <= 0.00015){
			zc_TdataWorkCnt ++;
			zc_TdataWorkSum += zc_TdataWorkBuf[i];
		}
	}
	if(zc_TdataWorkCnt >= 5){
		zc_TdataNew = zc_TdataWorkSum / zc_TdataWorkCnt;
	}else{
		zc_SumNew = zc_SumOld;
		zc_TdataNew = zc_SumNew / zc_num;
	}

	return zc_TdataNew;
}

/****************************************************
 * Function : CaluculateDeltaT
 * Summary  : 上下流の時間差を計算する
 * Argument : *FwdPnt : 上流ゼロクロス点
 *            *RevPnt : 下流ゼロクロス点
 * Return   : 
 * Caution  : 
 * notes    : 
 ****************************************************/
void CalculateDeltaT(short pch, float *FwdPnt, float *RevPnt)
{
	float zc_Tup, zc_Tdw, zc_TdataDiff = 0;
	short zc_num = SVD[pch].ZerCrsUseNum;
	short i;
	float zc_limit = (1.0 / SVD[pch].drive_freq * 1000.0) / 3; //しきい値=1周期の1/3 (実測で決定)
	float SmpTdt = (float)SmpTs[SVD[pch].adc_clock] / 100000; //2点間の時間差[us]

	//--時間差計算--
	zc_Tup = zc_Tdw = 0;
	for(i = 0; i < zc_num; i++)
	{
		zc_Tup += FwdPnt[i];
		zc_Tdw += RevPnt[i];
	}
	//点->時間差
	zc_Tup *= (SmpTdt / 2.0);
	zc_Tdw *= (SmpTdt / 2.0);

	//--1波ずれ対策--
	//しきい値分を+方向に超える時間差が発生したら1波ズレと判定
	if(((zc_Tup - zc_Tdw) / zc_num) >= zc_limit)
	{
		//下流側のゼロクロス点を再計算する
		zc_Tdw = 0;
		for(i = 2; i < zc_num - 1; i++)
		{
			//下流側のゼロクロス点を先頭2データを削除して加算
			zc_Tdw += RevPnt[i] * (SmpTdt / 2.0);
			RevPnt[i - 2] = RevPnt[i];
		}
		for(i = zc_num - 1; i < ZC_POINT_MAX; i++)
		{
			RevPnt[i] = 0.0;
		}
		zc_num -= 3;
	}
	//しきい値分を-方向に超える時間差が発生したら1波ズレと判定
	else if(((zc_Tup - zc_Tdw) / zc_num) <= (zc_limit*-1))
	{
		//上流側のゼロクロス点を再計算する
		zc_Tup = 0;
		for(i = 2; i < zc_num - 1; i++)
		{
			//上流側のゼロクロス点を先頭2データを削除して加算
			zc_Tup += FwdPnt[i] * (SmpTdt / 2.0);
			FwdPnt[i - 2] = FwdPnt[i];
		}
		for(i = zc_num - 1; i < ZC_POINT_MAX; i++)
		{
			FwdPnt[i] = 0.0;
		}
		zc_num -= 3;
	}

	//評価用
	if(SVD[pch].sum_start == 11){	//DelAbnPntの計算結果を有効にする
		//--異常値2点除外--
		zc_TdataDiff = zc_Tup - zc_Tdw;
		zc_TdataDiff = DelAbnPnt(FwdPnt, RevPnt, zc_TdataDiff, zc_num, SmpTdt);
	}else if(SVD[pch].sum_start == 12){	//DelAbnPntの計算結果を有効にする
		//--異常値除外--
		zc_TdataDiff = zc_Tup - zc_Tdw;
		zc_TdataDiff = DelAbnPntMod(FwdPnt, RevPnt, zc_TdataDiff, zc_num, SmpTdt);
	}else{
		zc_TdataDiff = (zc_Tup - zc_Tdw) / zc_num;	//伝搬時間差
	}
	//評価用
	
	//--値を保持--
	MES_SUB[pch].zc_Tup = zc_Tup;
	MES_SUB[pch].zc_Tdown = zc_Tdw;
	MES[pch].zc_Tdata = zc_TdataDiff;
}

/****************************************************
 * Function : SchZerPnt
 * Summary  : ゼロクロス点を探索する
 * Argument : pch : チャンネル番号
 * Return   : void
 * Caution  : 処理高速化のためにポインタ処理に変更
 * notes    : 
 ****************************************************/
void SchZerPnt(short pch)
{
#ifdef NEW_MEASURE_METHOD  //計算だけする
	short i;
	short zc_num = SVD[pch].ZerCrsUseNum;
	float zc_limit;

	//ポインタ宣言
	short *FowZerCrsPnt, *RevZerCrsPnt;
	short *FowZerCrsDat1, *FowZerCrsDat2, *FowZerCrsDat3, *FowZerCrsDat4;
	short *RevZerCrsDat1, *RevZerCrsDat2, *RevZerCrsDat3, *RevZerCrsDat4;

	short x1, x2, x3, x4;
	float y1, y2, y3, y4;
	float ZerCrsPnt1_3, ZerCrsPnt2_4;
	float FwdClcDat[ZC_POINT_MAX], RevClcDat[ZC_POINT_MAX];

	short xu1, xu2, xu3, xu4;
	short yu1, yu2, yu3, yu4;
	short xd1, xd2, xd3, xd4;
	short yd1, yd2, yd3, yd4;

	short SttPnt = 0; //ゼロクロス点使用開始位置
	float SmpTdt = (float)SmpTs[SVD[pch].adc_clock] / 100000; //2点間の時間差[us]

	float zc_Tup = 0;
	float zc_Tdown = 0;
	float zc_TdataDiff = 0;
	
	//最終出力変数
	float *ZerTdtUp;
	float *ZerTdtDw;
	float *ZerTdtVal;

	//ポインタ初期値
	FowZerCrsPnt = (short *)(&MES[pch].zc_nearzero_point[0][0]);
	FowZerCrsDat1 = (short *)(&MES[pch].zc_nearzero_data1[0][0]);
	FowZerCrsDat2 = (short *)(&MES[pch].zc_nearzero_data2[0][0]);
	FowZerCrsDat3 = (short *)(&MES[pch].zc_nearzero_data3[0][0]);
	FowZerCrsDat4 = (short *)(&MES[pch].zc_nearzero_data4[0][0]);
	RevZerCrsPnt = (short *)(&MES[pch].zc_nearzero_point[1][0]);
	RevZerCrsDat1 = (short *)(&MES[pch].zc_nearzero_data1[1][0]);
	RevZerCrsDat2 = (short *)(&MES[pch].zc_nearzero_data2[1][0]);
	RevZerCrsDat3 = (short *)(&MES[pch].zc_nearzero_data3[1][0]);
	RevZerCrsDat4 = (short *)(&MES[pch].zc_nearzero_data4[1][0]);

	ZerTdtVal = (float *)(&MES[pch].zc_Tdata);
	ZerTdtUp = (float *)(&MES_SUB[pch].zc_Tup);
	ZerTdtDw = (float *)(&MES_SUB[pch].zc_Tdown);

	/*ゼロ点(中心線2047)を通過する前2点、後2点からゼロクロス点を計算する(回帰直線)*/
	for (i = 0; i < zc_num; i++)
	{
		// // 上流側
		xu1 = *(FowZerCrsPnt + i + SttPnt) - 1;	//1点目(通過2点前の点)
		xu2 = xu1 + 1;	//2点目(通過1点前の点)
		xu3 = xu1 + 2;	//3点目(通過後の1点目)
		xu4 = xu1 + 3;	//4点目(通過後の2点目)
		yu1 = *(FowZerCrsDat1 + i + SttPnt);
		yu2 = *(FowZerCrsDat2 + i + SttPnt);
		yu3 = *(FowZerCrsDat3 + i + SttPnt);
		yu4 = *(FowZerCrsDat4 + i + SttPnt);

		if((yu1 == yu3) || (yu2 == yu4))	return;  //ゼロ割り対策
		FwdClcDat[i] = CorZrcPnt(xu1, xu2, xu3, xu4, yu1, yu2, yu3, yu4);
		MES[pch].FwdClcZerPnt[i] = FwdClcDat[i];

		// // 下流側
		xd1 = *(RevZerCrsPnt + i + SttPnt) - 1;	//1点目(通過2点前の点)
		xd2 = xd1 + 1;	//2点目(通過1点前の点)
		xd3 = xd1 + 2;	//3点目(通過後の1点目)
		xd4 = xd1 + 3;	//4点目(通過後の2点目)
		yd1 = *(RevZerCrsDat1 + i + SttPnt);
		yd2 = *(RevZerCrsDat2 + i + SttPnt);
		yd3 = *(RevZerCrsDat3 + i + SttPnt);
		yd4 = *(RevZerCrsDat4 + i + SttPnt);

		if((yd1 == yd3) || (yd2 == yd4))	return;  //ゼロ割り対策
		RevClcDat[i] = CorZrcPnt(xd1, xd2, xd3, xd4, yd1, yd2, yd3, yd4);
		MES[pch].RevClcZerPnt[i] = RevClcDat[i];

		//ゼロクロス点を加算
		zc_Tup += FwdClcDat[i];
		zc_Tdown += RevClcDat[i];
	}

	//時間差を計算する
	CalculateDeltaT(pch, FwdClcDat, RevClcDat);
#else
	short cnt1, cnt2, cnt3;
	short zc_num;
	short s_d;
	short fow_before, rev_before;
	short x1, x2, x3, x4;
	float y1, y2, y3, y4;
	short zc_start = 0;
	short SelMult[] = { 32, 40, 65, 80 }; // 現行クランプオンではサンプリング周波数に近似?
	float zc_limit;
	short i = 12;

	//ポインタ宣言
	short *FowZerCrsPnt, *RevZerCrsPnt;
	short *FowZerCrsDat1, *FowZerCrsDat2, *FowZerCrsDat3, *FowZerCrsDat4;
	short *RevZerCrsDat1, *RevZerCrsDat2, *RevZerCrsDat3, *RevZerCrsDat4;

	short *TmpFowDat, *TmpRevDat;

	float *ZerCrsDat1, *ZerCrsDat2, *ZerCrsDat3, *ZerCrsDat4;
	float *ZerCrsDats1, *ZerCrsDats2;

	float *ZerTdtVal;

	short SttPnt;
	float SmpTdt;

	float zc_zero_data[4][30];
	float zc_zero_dataus[2][30];
	float zc_Tup;
	float zc_Tdown;
	float zc_TdataDiff;
	float *ZerTdtUp;
	float *ZerTdtDw;
	float zc_TdataAve;
	float zc_TdataWork;
	float zc_TdataMax1;
	float zc_TdataMax2;
	float zc_TdataAbs;

#if defined(DatInc)
	SelMult[SVD[pch].adc_clock] = SelMult[SVD[pch].adc_clock] / Mgn;
#endif

	cnt1 = cnt2 = cnt3 = 0;

	//ポインタ初期値
	TmpFowDat = (short *)(&MES[pch].fow_data[12]);
	TmpRevDat = (short *)(&MES[pch].rev_data[12]);
	FowZerCrsPnt = (short *)(&MES[pch].zc_nearzero_point[0][0]);
	FowZerCrsDat1 = (short *)(&MES[pch].zc_nearzero_data1[0][0]);
	FowZerCrsDat2 = (short *)(&MES[pch].zc_nearzero_data2[0][0]);
	FowZerCrsDat3 = (short *)(&MES[pch].zc_nearzero_data3[0][0]);
	FowZerCrsDat4 = (short *)(&MES[pch].zc_nearzero_data4[0][0]);
	RevZerCrsPnt = (short *)(&MES[pch].zc_nearzero_point[1][0]);
	RevZerCrsDat1 = (short *)(&MES[pch].zc_nearzero_data1[1][0]);
	RevZerCrsDat2 = (short *)(&MES[pch].zc_nearzero_data2[1][0]);
	RevZerCrsDat3 = (short *)(&MES[pch].zc_nearzero_data3[1][0]);
	RevZerCrsDat4 = (short *)(&MES[pch].zc_nearzero_data4[1][0]);

	ZerCrsDat1 = (float *)(&zc_zero_data[0][0]);
	ZerCrsDat2 = (float *)(&zc_zero_data[1][0]);
	ZerCrsDat3 = (float *)(&zc_zero_data[2][0]);
	ZerCrsDat4 = (float *)(&zc_zero_data[3][0]);
	ZerCrsDats1 = (float *)(&zc_zero_dataus[0][0]);
	ZerCrsDats2 = (float *)(&zc_zero_dataus[1][0]);

	ZerTdtVal = (float *)(&MES[pch].zc_Tdata);
	ZerTdtUp = (float *)(&MES_SUB[pch].zc_Tup);
	ZerTdtDw = (float *)(&MES_SUB[pch].zc_Tdown);

	SmpTdt = (float)SmpTs[SVD[pch].adc_clock] / 100000;

	zc_start = SVD[pch].ZerPeakPos;
	TmpFowDat = (short *)(&MES[pch].fow_data[0]);//ゼロクロス探索開始位置は[zc_start]番目から
	TmpRevDat = (short *)(&MES[pch].rev_data[0]);
	fow_before = *(TmpFowDat + zc_start - 1);
	rev_before = *(TmpRevDat + zc_start - 1);
	// for (i = 12; i < 210; i++)
	for (i = zc_start; i < 250; i++)
	{
		// TmpRevDat++;
		if(
			(i > 12) && (
				((rev_before < AD_BASE) && (AD_BASE <= *(TmpRevDat + i)))
				|| ((AD_BASE < rev_before) && (*(TmpRevDat + i) <= AD_BASE))
				)
		)
		{
			if(cnt2 < ZC_POINT_MAX){
				*(RevZerCrsPnt + cnt2) = i - 1;		//通過する前の点を保持
				*(RevZerCrsDat1 + cnt2) = *(TmpRevDat + i - 2) - AD_BASE;	//通過2点前の点を保持
				*(RevZerCrsDat2 + cnt2) = *(TmpRevDat + i - 1) - AD_BASE;	//通過1点前の点を保持
				*(RevZerCrsDat3 + cnt2) = *(TmpRevDat + i - 0) - AD_BASE;	//通過後の1点目を保持
				*(RevZerCrsDat4 + cnt2) = *(TmpRevDat + i + 1) - AD_BASE;	//通過後の2点目を保持
			}
			cnt2++;
		}
		rev_before = *(TmpRevDat + i);
		if((0 + SVD[pch].ZerCrsUseNum) <= cnt2)
		{
			break;  //「ゼロクロス開始位置+ゼロクロス回数」以上のゼロクロス点を探す必要がない
		}
	}
	for (i = zc_start; i < 250; i++)
	{
		// TmpFowDat++;
		if(
			(i > 12) && (
				((fow_before < AD_BASE) && (AD_BASE <= *(TmpFowDat + i)))
				|| ((AD_BASE < fow_before) && (*(TmpFowDat + i) <= AD_BASE))
				)
		)
		{
			if(cnt1 < ZC_POINT_MAX){
				*(FowZerCrsPnt + cnt1) = i - 1;		//通過する前の点を保持
				*(FowZerCrsDat1 + cnt1) = *(TmpFowDat + i - 2) - AD_BASE;	//通過2点前の点を保持
				*(FowZerCrsDat2 + cnt1) = *(TmpFowDat + i - 1) - AD_BASE;	//通過1点前の点を保持
				*(FowZerCrsDat3 + cnt1) = *(TmpFowDat + i - 0) - AD_BASE;	//通過後の1点目を保持
				*(FowZerCrsDat4 + cnt1) = *(TmpFowDat + i + 1) - AD_BASE;	//通過後の2点目を保持
			}
			cnt1++;
		}
		fow_before = *(TmpFowDat + i);
		// if(((SVD[pch].ZerCrsSttPnt + SVD[pch].ZerCrsUseNum) < cnt1) && ((SVD[pch].ZerCrsSttPnt + SVD[pch].ZerCrsUseNum) < cnt2))
		if((0 + SVD[pch].ZerCrsUseNum) <= cnt1)
		{
			break;  //「ゼロクロス開始位置+ゼロクロス回数」以上のゼロクロス点を探す必要がない
		}
	}

	/*ゼロ点(中心線2047)を通過する前2点、後2点からゼロクロス点を計算する(回帰直線)*/
	zc_num = SVD[pch].ZerCrsUseNum;
	// SttPnt = SVD[pch].ZerCrsSttPnt;
	SttPnt = 0;
	zc_Tup = zc_Tdown = zc_TdataDiff = 0;
	//前2点、後2点でゼロクロス計算
	for (cnt1 = 0; cnt1 < zc_num; cnt1++)
	{
		// 上流側
		x1 = *(FowZerCrsPnt + cnt1 + SttPnt) - 1;	//1点目(通過2点前の点)
		y1 = *(FowZerCrsDat1 + cnt1 + SttPnt);
		x2 = *(FowZerCrsPnt + cnt1 + SttPnt) + 0;	//2点目(通過1点前の点)
		y2 = *(FowZerCrsDat2 + cnt1 + SttPnt);
		x3 = *(FowZerCrsPnt + cnt1 + SttPnt) + 1;	//3点目(通過後の1点目)
		y3 = *(FowZerCrsDat3 + cnt1 + SttPnt);
		x4 = *(FowZerCrsPnt + cnt1 + SttPnt) + 2;	//4点目(通過後の2点目)
		y4 = *(FowZerCrsDat4 + cnt1 + SttPnt);
		if((y1 == y3) || (y2 == y4))	return;  //ゼロ割り対策
		MES[pch].FwdClcZerPnt[cnt1] = CorZrcPnt(x1, x2, x3, x4, y1, y2, y3, y4);
		*(ZerCrsDats1 + cnt1) = MES[pch].FwdClcZerPnt[cnt1] * SmpTdt / 2.0f;	//時間換算

		// 下流側
		x1 = *(RevZerCrsPnt + cnt1 + SttPnt) - 1;	//1点目(通過2点前の点)
		y1 = *(RevZerCrsDat1 + cnt1 + SttPnt);
		x2 = *(RevZerCrsPnt + cnt1 + SttPnt) + 0;	//2点目(通過1点前の点)
		y2 = *(RevZerCrsDat2 + cnt1 + SttPnt);
		x3 = *(RevZerCrsPnt + cnt1 + SttPnt) + 1;	//3点目(通過後の1点目)
		y3 = *(RevZerCrsDat3 + cnt1 + SttPnt);
		x4 = *(RevZerCrsPnt + cnt1 + SttPnt) + 2;	//4点目(通過後の2点目)
		y4 = *(RevZerCrsDat4 + cnt1 + SttPnt);
		if((y1 == y3) || (y2 == y4))	return;  //ゼロ割り対策
		MES[pch].RevClcZerPnt[cnt1] = CorZrcPnt(x1, x2, x3, x4, y1, y2, y3, y4);
		*(ZerCrsDats2 + cnt1) = MES[pch].RevClcZerPnt[cnt1] * SmpTdt / 2.0f;	//時間換算

		//ゼロクロス点を加算
		zc_Tup += *(ZerCrsDats1 + cnt1);	//上流側のゼロクロス点を加算
		zc_Tdown += *(ZerCrsDats2 + cnt1);	//下流側のゼロクロス点を加算
		zc_TdataDiff += *(ZerCrsDats1 + cnt1) - *(ZerCrsDats2 + cnt1);	//ゼロクロス点分の伝搬時間差
	}

 	i = zc_TdataWork = zc_TdataMax1 = zc_TdataMax2 = 0;
	zc_TdataAve = zc_TdataDiff / zc_num;	//伝搬時間差の平均値(ゼロクロス点分)
	if((zc_TdataAve < -0.0001f) || (0.0001f < zc_TdataAve)){	//伝搬時間差が充分小さい(ゼロ点付近)場合は処理しない
		for (cnt3 = 0; cnt3 < zc_num; cnt3++)	//伝搬時間差の平均値から外れている大きい2点を探す
		{
			zc_TdataWork = *(ZerCrsDats1 + cnt3) - *(ZerCrsDats2 + cnt3);	//伝搬時間差
			zc_TdataAbs = zc_TdataAve - zc_TdataWork;	//伝搬時間差の平均値からの差
			if(zc_TdataAbs < 0)	zc_TdataAbs *= -1;	//絶対値で比較する
			if(zc_TdataMax1 == 0 || zc_TdataAbs > zc_TdataMax1){
				if(zc_TdataMax1 != 0){
					zc_TdataMax2 = zc_TdataMax1;	//1番目に誤差の大きい伝搬時間差を2番目に移動
				}
				zc_TdataMax1 = zc_TdataWork;	//伝搬時間差の保持(1番目に誤差が大きい)
			}else if(zc_TdataMax2 == 0 || zc_TdataAbs > zc_TdataMax2){
				zc_TdataMax2 = zc_TdataWork;	//伝搬時間差の保持(2番目に誤差が大きい)
			}
		}
	}

	zc_Tup /= zc_num;	//上流側のゼロクロス点を平均
	zc_Tdown /= zc_num;	//下流側のゼロクロス点を平均
	if(zc_TdataMax1 == 0){
		zc_TdataDiff /= zc_num;		//平均値
	}else if(zc_TdataMax2 == 0){
		zc_TdataDiff -= zc_TdataMax1;	//伝搬時間差の平均値から外れている1点を減算
		zc_TdataDiff /= (zc_num - 1);	//1点減算した平均値
	}else{
		zc_TdataDiff -= zc_TdataMax1;	//伝搬時間差の平均値から外れている2点を減算
		zc_TdataDiff -= zc_TdataMax2;
		zc_TdataDiff /= (zc_num - 2);	//2点減算した平均値
	}

	/*1波ズレ対策*/
	zc_limit = (1.0 / SVD[pch].drive_freq * 1000.0) / 3;
	if(zc_TdataDiff >= zc_limit){ //駆動周波数(100kHz～5MHz)の1周期を+方向に超える時間差が発生したら1波ズレと判定
		//下流側のゼロクロス点を再計算する
		zc_Tdown = 0;
		for(cnt1=2; cnt1<zc_num; cnt1++){
			zc_Tdown = zc_Tdown + *(ZerCrsDats2 + cnt1);  //下流側のゼロクロス点を先頭2データを削除して加算
		}
		zc_Tdown = zc_Tdown / (zc_num - 2);  //下流側のゼロクロス点を平均
		zc_TdataDiff = zc_Tup - zc_Tdown;  //伝搬時間差
	}else if(zc_TdataDiff <= (zc_limit*-1)){ //駆動周波数(100kHz～5MHz)の1周期を-方向に超える時間差が発生したら1波ズレと判定
		//上流側のゼロクロス点を再計算する
		zc_Tup = 0;
		for(cnt1=2; cnt1<zc_num; cnt1++){
			zc_Tup = zc_Tup + *(ZerCrsDats1 + cnt1);   //上流側のゼロクロス点を先頭2データを削除して加算
		}
		zc_Tup = zc_Tup / (zc_num - 2);  //上流側のゼロクロス点を平均
		zc_TdataDiff = zc_Tup - zc_Tdown;  //伝搬時間差
	}else{
		;
	}

	*(ZerTdtVal) = zc_TdataDiff;  //伝搬時間差を保持
	*(ZerTdtUp) = zc_Tup;
	*(ZerTdtDw) = zc_Tdown;
#endif
}

/****************************************************
 * Function : ClcSumWavAmp (Caluculate Sum Wave Amplitude)
 * Summary  : 波形ピークの加算値を計算する
 * Argument : sample : 波形インデックス
 * Return   : void
 * Caution  : 
 * Notes    : 波形先頭から1600word分の波形ピーク(極大値)を加算する
 ****************************************************/
void ClcSumWavAmp(short pch, short sample)
{
	short i;
 	unsigned long work_add;
	short search_cnt;
	short ws_max, ws_min;
	short ws_max_cnt, ws_min_cnt;
	short ws_max_val[30], ws_min_val[30];
	short work;

	/*Windowサーチ要求あり(最適なFIFO CHを探す)*/
	if(MES[pch].ThresholdReq == 2){
		memset(&ws_max_val[0], 0, sizeof(ws_max_val));
		memset(&ws_min_val[0], 0, sizeof(ws_min_val));
		ws_max = ws_min = AD_BASE_UNIT;
		ws_max_cnt = ws_min_cnt = 0;

		// 波形検出した箇所から1600word分の波形データを解析する 
		search_cnt = 1600 / Mgn;
		for(i=0; i<search_cnt; i++){
			work = rev_temp_data[sample][10+i];
			work &= 0x1FFF;

			if(ws_max < work){	//最大値を更新
				ws_max = work;
				if(work > AD_BASE_UNIT && ws_min != AD_BASE_UNIT){//中心線を超えた時点で極値(ﾏｲﾅｽ側)を決定する
					ws_min_val[ws_min_cnt] = ws_min;	//(生波形が微小な上下を繰り返しながらプラス方向に移動しているため、中心線を超えるまでは極値(ﾏｲﾅｽ側)を決定しない)
					ws_min = AD_BASE_UNIT;
					if(ws_min_cnt < 30) ws_min_cnt++;
				}
			}
			if(work < ws_min){	//最小値を更新
				ws_min = work;
				if(work < AD_BASE_UNIT && ws_max != AD_BASE_UNIT){//中心線を下回った時点で極値(プラス側)を決定する
					ws_max_val[ws_max_cnt] = ws_max;	//(生波形が微小な上下を繰り返しながらﾏｲﾅｽ方向に移動しているため、中心線を下回るまでは極値(プラス側)を決定しない)
					ws_max = AD_BASE_UNIT;
					if(ws_max_cnt < 30) ws_max_cnt++;
				}
			}
		}

		MES[pch].ws_work_add = 0;
		for(i=0; i<ws_max_cnt; i++){
			MES[pch].ws_work_add += ws_max_val[i];	//極値(プラス側)を加算
		}
		MES[pch].ws_work_add = MES[pch].ws_work_add / ws_max_cnt; //極値の回数で平均化	
	}
}

/****************************************************
 * Function : ActBfrFifRed (Action Before FIFO Read)
 * Summary  : FIFORead前の処理
 * Argument : void
 * Return   : void
 * Caution  : 
 * Notes    : 
 ****************************************************/
void ActBfrFifRed(short pch)
{
	OutputRestartPulse(); /*RESTARTパルス出力*/

	// 起動時の初期化処理が終了している場合
	if (initializing == 0)
	{
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}
	
	// パルス打ち込み有り
	OutputStartPulse();	  /*STARTパルス出力*/
	CheckEndPulse(); /*END信号確認*/
	
	/*FIFO読込み*/
	if (MES[pch].fifo_no_read != 0 && MES[pch].ThresholdPeakPos == 0)/*波形認識閾値の未設定(ゼロ点調整未実施)*/
	{
		dma_dummy(MES[pch].fifo_no_read, pch); /*FIFO空読み*/
	}
}

/****************************************************/
/* Function : DriveFIFOFwd                         */
/* Summary  : 上流側FIFO処理(パルス打ち込み、FIFO読込み)		*/
/* Argument : short pch, short sample             */
/* Return   : なし                                  */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void DriveFIFOFwd(short pch, short sample){

#if defined(ShtItv)
	//打込み後インターバルの前で切り替えておく（処理時間短縮）
#else
	SelectReverseOn(pch);	//IN/OUT打込み切替え
	delay(50);			  /*上流側切替後、5usec待機*/
#endif   

	//打ち込み前処理
	ActBfrFifRed(pch);
	
	// dma_start(&fwd_temp_data[sample][10]); /*FIFO読込み開始*/
	us_dma_start(&fwd_temp_data[sample][10]); /*FIFO読込み開始*/
	OutputRestartPulse();		/*RESTARTパルス出力*/

	SelectReverseOff(pch);	//IN/OUT打込み切替え

#if defined(ShtItv)
	//打込み後インターバルの前で切り替えておく（処理時間短縮）
	SelectForwardOn(pch);	//IN/OUT打込み切替え
#endif
}

/****************************************************/
/* Function : DriveFIFORev                         */
/* Summary  : 下流側FIFO処理(パルス打ち込み、FIFO読込み)		*/
/* Argument : short pch, short sample             */
/* Return   : なし                                  */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void DriveFIFORev(short pch, short sample){

#if defined(ShtItv)
	//打込み後インターバルの前で切り替えておく（処理時間短縮）
#else
	SelectForwardOn(pch);		//IN/OUT打込み切替え
	delay(50);					/*下流側切替後、5usec待機*/
#endif

	//打ち込み前処理
	ActBfrFifRed(pch);

	// dma_start(&rev_temp_data[sample][10]); /*FIFO読込み開始*/
	us_dma_start(&rev_temp_data[sample][10]); /*FIFO読込み開始*/
	OutputRestartPulse();	/*RESTARTパルス出力*/

	SelectForwardOff(pch);	//IN/OUT打込み切替え

#if defined(ShtItv)
	//打込み後インターバルの前で切り替えておく（処理時間短縮）

#if defined(ShtCntTwo)
	if(sample < 1)		//最後は切り替えない
#else
//	if(sample < 3)		//最後は切り替えない
	if(sample < (MES_SUB[pch].sample_cnt-1))		//最後は切り替えない
#endif
	{
		SelectReverseOn(pch);	//IN/OUT打込み切替え
	}
#endif
	//波形ピークの加算値を計算する(最適なFIFO CHを探すため)
	ClcSumWavAmp(pch, sample);
}

/****************************************************/
/* Function : min_max_check                         */
/* Summary  : data_ok!=0が、演算異常判定回数続いたかどうかチェックする		*/
/* Argument : short data                            */
/* Return   : 0=続いてない、  -1=続いた	                */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short	min_max_check(short pch){

	short i;
	short corr_time;

#if defined(FRQSCH)
	//周波数サーチ中
	// if(FrqSch[pch].FrqSchSttFlg == 1){
	if(FrqSch[pch].FrqSchSttFlg != 0){
		corr_time = 1;
	}
	else{
		corr_time = (short)(SVD[pch].correlate_time);  //演算異常判定回数
	}
#else
	corr_time = (short)(SVD[pch].correlate_time);  //演算異常判定回数
#endif
	MES[pch].read_ok[MES[pch].count_ok] = (char)MES[pch].data_ok;  //データ有効(0)/データ無効(1)
	MES[pch].count_ok++;  //カウント更新
	if(MES[pch].count_ok >= corr_time){
		MES[pch].count_ok = 0;  //カウントクリア
	}

	for(i=0; i<corr_time; i++){  //演算異常判定回数
		if(MES[pch].read_ok[i] == 0){  //データ有効(0)があれば
 		return (0);  //演算異常判定回数続いてない(正常)
		}
	}
	return (-1);  //演算異常判定回数続いた(異常)
}

/****************************************************/
/* Function : gain_adj_init                         */
/* Summary  : アンプゲインの調整	                     		*/
/* Argument : pch                            */
/* Return   : 0=正常 -1=異常                        */
/* Caution  : なし                                   */
/* notes    : 初期化時のアンプゲイン調整                 */
/****************************************************/
short	gain_adj_init(short pch){

	short	i,j,no;
	short	ret_sts;
	short	forward_max,forward_min;
	short	reverse_max,reverse_min;
	short	forward_max_cal,forward_min_cal;
	short	reverse_max_cal,reverse_min_cal;
	short retry;
	short	atn_gain;
	unsigned short sat_level; 

	/*amp_gain*/
	retry = 20;								/*終了時のリトライ回数*/
	MES[pch].amp_gain_for = 255;
	MES[pch].amp_gain_rev = 255;

	/*抵抗設定*/
	no = 256;										/*0-255,0=max*/

	/*波形飽和チェックレベル*/
	sat_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].saturation_level / 100);	/*最大値(12bit MAX)*判定閾値*/

	/* ゲイン算出 */
	for(i=255; i>=0; i--){						/*amp gain MAX255*/
		/*抵抗選択*/
		MES[pch].amp_gain_for = MES[pch].amp_gain_rev = i;		
		WriteGainAmp((unsigned short)i);  /*ゲイン書込み*/

		delay(10);								/*待機*/
		fifo_read(pch);			/*受波データ読込み*/
		
		forward_max = reverse_max = 0;			/*初期値*/
		forward_min = reverse_min = AD_MAX;		/*12bit MAX*/

#if defined(FLWWAVEXP)
	 	for(j=10; j<FLWWAVSIZ + 10; j++)
#else
	 	for(j=10; j<250; j++)					/*最大、最小を探す*/
#endif
	 	{
			if(forward_max < MES[pch].fow_data[j]){	/*forward data max*/
				forward_max = MES[pch].fow_data[j];
			}
			if(forward_min > MES[pch].fow_data[j]){	/*forward data min*/
				forward_min = MES[pch].fow_data[j];	
			}
			if(reverse_max < MES[pch].rev_data[j]){	/*reverse data max*/
				reverse_max = MES[pch].rev_data[j];
			}
			if(reverse_min > MES[pch].rev_data[j]){	/*reverse data min*/
				reverse_min = MES[pch].rev_data[j];	
			}
		}

		if ((forward_max >= sat_level)||(forward_min < GAIN_INIT_LOW*MES_SUB[pch].sample_cnt)	/*波形飽和チェック*/
			||(reverse_max >= sat_level)||(reverse_min < GAIN_INIT_LOW*MES_SUB[pch].sample_cnt)){
			/*振幅チェック異常*/
			retry = 20;
			continue;							/*ゲイン変更(down)*/
		}else{									/*ゲインOK*/
			if (retry == 0){
				break;							/*調整終了（飽和してない）*/
			}else{
				retry--;
				if (i<255){
					i++;						/*ゲインそのままで、もう一回*/
				}
				continue;
			}
		}
	}

	MES[pch].fow_max_data = forward_max;		/*最大値保存*/
	MES[pch].fow_min_data = forward_min;		/*最小値保存*/
	MES[pch].rev_max_data = reverse_max;		/*最大値保存*/
	MES[pch].rev_min_data = reverse_min;		/*最小値保存*/

	if((i>=0) && (i<no)){						/*ゲイン調整OK（飽和してない）*/
		ret_sts = MES[pch].err_status &= ~(ERR_JUDGE_AGC + ERR_JUDGE_LEVEL);	/*正常*/
		/* ゲイン　*/
		if((forward_max < GAIN_WAVE_HI*MES_SUB[pch].sample_cnt)||(forward_min > GAIN_WAVE_LOW*MES_SUB[pch].sample_cnt)	/*波形確認*/
			||(reverse_max < GAIN_WAVE_HI*MES_SUB[pch].sample_cnt)||(reverse_min > GAIN_WAVE_LOW*MES_SUB[pch].sample_cnt)){
		/*センサー異常（信号レベルが低い）*/
			ret_sts = (short)-1;
			i = 0;									/*ゲインMAX*/
			WriteGainAmp((unsigned short)i);  /*ゲイン書込み*/
			MES[pch].amp_gain_for = MES[pch].amp_gain_rev = i;
		}
		/*波形アンバランスチェック*/
		if(MES[pch].ThresholdReq == 0){	/*波形認識実行時はチェックしない*/
			forward_max_cal = forward_max - AD_BASE;	/*最大値（中心線からのデータ量）*/
			reverse_max_cal = reverse_max - AD_BASE;	/*								*/
			forward_min_cal = AD_BASE - forward_min;	/*最小値（中心線からのデータ量）*/
			reverse_min_cal = AD_BASE - reverse_min;	/*								*/
			if((((float)(abs(forward_max_cal - reverse_max_cal)) / (float)forward_max_cal * 100) > SVD[pch].balance_level)
				||(((float)(abs(forward_min_cal - reverse_min_cal)) / (float)forward_min_cal * 100) > SVD[pch].balance_level)){
				ret_sts = (short)-1;
				MES[pch].err_status |= ERR_JUDGE_LEVEL;		/*波形アンバランス（上流側と下流側の波形が異なる）*/
			}
		}
	}else{													/*ゲイン調整不能*/
		ret_sts = (short)-1;
		MES[pch].err_status |= ERR_JUDGE_AGC;				/*アンプ不良（ゲインMINでも飽和している）*/ 
	}

	for(i=0; i<4; i++){
		MES[pch].fow_max_phase[i] = MES[pch].fow_max_data;
		MES[pch].fow_min_phase[i] = MES[pch].fow_min_data;
		MES[pch].rev_max_phase[i] = MES[pch].rev_max_data;
		MES[pch].rev_min_phase[i] = MES[pch].rev_min_data;
	}

	return ret_sts;
}

/****************************************************/
/* Function : gain_adj_control                      */
/* Summary  : アンプゲインの調整    				*/
/* Argument : pch                               */
/* Return   : 0=正常   -1=異常                       */
/* Caution  : なし                                   */
/* notes    : 流量測定時のアンプゲイン調整              */
/****************************************************/
short	gain_adj_control(short pch){

	short i,j;
	short amp;
	short	ret_sts;
	short	fow_max_cal,fow_min_cal;
	short	rev_max_cal,rev_min_cal;
	short atn_gain;
	short	fow_ratio,rev_ratio;
	unsigned short sat_level;
	short *ptr;

	/*波形飽和チェックレベル*/
	sat_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].saturation_level / 100);	/*最大値(12bit MAX)*判定閾値*/
	
	MES[pch].err_status &= ~(ERR_JUDGE_AGC + ERR_JUDGE_WAVE + ERR_JUDGE_LEVEL);
	ret_sts = MES[pch].err_status;

	MES[pch].fow_max_phase[MES[pch].clk_phase]=fow_max;
	MES[pch].rev_max_phase[MES[pch].clk_phase]=rev_max;
	MES[pch].fow_min_phase[MES[pch].clk_phase]=fow_min;
	MES[pch].rev_min_phase[MES[pch].clk_phase]=rev_min;
	
	ptr = &(MES[pch].fow_max_phase[0]);
	fow_max = (ptr[0] + ptr[1] + ptr[2] + ptr[3]) / 4;
	ptr = &(MES[pch].rev_max_phase[0]);
	rev_max = (ptr[0] + ptr[1] + ptr[2] + ptr[3]) / 4;
	ptr = &(MES[pch].fow_min_phase[0]);
	fow_min = (ptr[0] + ptr[1] + ptr[2] + ptr[3]) / 4;
	ptr = &(MES[pch].rev_min_phase[0]);
	rev_min = (ptr[0] + ptr[1] + ptr[2] + ptr[3]) / 4;

	/* 下流ゲイン算出 */
	if(rev_max >= sat_level){					/*波形飽和チェック*/
		MES[pch].amp_gain_rev -= SVD[pch].gain_step; /*ゲインステップ幅減算*/
		if (MES[pch].amp_gain_rev < 0){
			MES[pch].amp_gain_rev = 0;
			ret_sts = (short)-1;
			MES[pch].err_status |= ERR_JUDGE_AGC;		/*アンプ不良（ゲインMINでも飽和している）*/ 
		}
		amp = MES[pch].amp_gain_for = MES[pch].amp_gain_rev;
	}

	/*波形減衰チェック*/
	if(MES[pch].ThresholdReq == 0){	/*波形認識実行時はチェックしない*/
		/*上流側*/
		fow_ratio =	((float)fow_max / (float)MES[pch].fow_max_data)*100;		/*受波波形最大値の減衰率*/
		if(fow_ratio < 100){													/*波形が減衰(100%未満)*/
			if((100 - fow_ratio) >= SVD[pch].attenuate_level){				/*波形減衰チェック*/
				MES[pch].err_status |= ERR_JUDGE_WAVE;							/*波形減衰エラーセット*/ 
			}
		}
		/*下流側*/
		rev_ratio =	((float)rev_max / (float)MES[pch].rev_max_data)*100;		/*受波波形最大値の減衰率*/
		if(rev_ratio < 100){													/*波形が減衰(100%未満)*/
			if((100 - rev_ratio) >= SVD[pch].attenuate_level){				/*波形減衰チェック*/
				MES[pch].err_status |= ERR_JUDGE_WAVE;							/*波形減衰エラーセット*/ 
			}
		}
	}
	MES[pch].fow_max_data = fow_max;		/*最大値保存*/
	MES[pch].fow_min_data = fow_min;		/*最小値保存*/
	MES[pch].rev_max_data = rev_max;		/*最大値保存*/
	MES[pch].rev_min_data = rev_min;		/*最小値保存*/
	MES[pch].rev_max_data_point = rev_max_point;
	MES[pch].fow_max_data_point = fow_max_point;

	/* 下流ゲイン算出 */
	if (rev_max < GAIN_CONT_LOW*MES_SUB[pch].sample_cnt){						/* 波形確認*/
		/*センサー異常（信号レベルが低い）*/
		MES[pch].amp_gain_rev += SVD[pch].gain_step; /*ゲインステップ幅加算*/
		if (MES[pch].amp_gain_rev > 255){
			MES[pch].amp_gain_rev = 255;
		}
		amp = MES[pch].amp_gain_for = MES[pch].amp_gain_rev;	
	}	

	/*波形アンバランスチェック*/
	if(MES[pch].ThresholdReq == 0){	/*波形認識実行時はチェックしない*/
		fow_max_cal = fow_max - AD_BASE;		/*最大値（中心線からのデータ量）*/
		rev_max_cal = rev_max - AD_BASE;		/*								*/
		fow_min_cal = AD_BASE - fow_min;		/*最小値（中心線からのデータ量）*/
		rev_min_cal = AD_BASE - rev_min;		/*								*/
		if((((float)(abs(fow_max_cal - rev_max_cal)) / (float)fow_max_cal * 100) > SVD[pch].balance_level)
			||(((float)(abs(fow_min_cal - rev_min_cal)) / (float)fow_min_cal * 100) > SVD[pch].balance_level)){
				ret_sts = (short)-1;
				MES[pch].err_status |= ERR_JUDGE_LEVEL;	/*波形アンバランス（上流側と下流側の波形が異なる）*/
		}
	}

	return ret_sts;
}

/****************************************************/
/* Function : sum_adder                           */
/* Summary  : 絶対値の和を演算する    				*/
/* Argument : pch                               */
/* Return   : なし 									                         */
/* Caution  : 処理高速化のためにポインタ処理に変更
 * notes    : なし
 ****************************************************/
void	sum_adder(short pch){

	short j, offset;
	long sum_work;

	unsigned short SumSttPnt, SumEndPnt, SumStp;
	short *FowDat, *RevDat;
	long *SumAbs;
	unsigned short *SumAbsCom;

	SumSttPnt = SVD[pch].sum_start;
	SumEndPnt = SVD[pch].sum_end;
	SumStp = SVD[pch].sum_step;
	FowDat = (short *)(&MES[pch].fow_data[10]);
	RevDat = (short *)(&MES[pch].rev_data[10]);
	SumAbs = (long *)(&MES[pch].sum_abs[0]);
	SumAbsCom = (unsigned short *)(&SAVE[pch].sum_abs_com[0]);

	/*相関演算*/
	for (offset = 0; offset < SUB_POINT; offset++)
	{					  /*offset をずらしていく*/
		sum_work = 0; /*加算値クリア*/

		/*相関演算、差の絶対値を加算する*/
		if (SumPntInc == SumPnt_Inc)
		{
#if defined(FLWWAVEXP)
			for (j = 10; j < FLWWAVSIZ + 10; j = j + 2)
#else
			for (j = 10; j < 250; j = j + 2)
#endif
			{ /*10～210のデータを1つ飛ばして100word分の差分演算*/
				 sum_work += (long)abs(*(FowDat + j - 4 + offset) - *(RevDat + j));
			}
		}
		else
		{
			for (j = SumSttPnt; j < SumEndPnt; j = (j + SumStp))
			{	
				/*差分演算　(初期値:10～120の110個分)*/
				sum_work += (long)abs(*(FowDat + j - 4 + offset) - *(RevDat + j));
			}
		}

		*(SumAbs + offset) = sum_work;

		/*通信用*/
		if ((*(SumAbs + offset) / 5) <= 65535)
		{
			*(SumAbsCom + offset) = (unsigned short)(*(SumAbs + offset) / 5);
		}
		else
		{
			*(SumAbsCom + offset) = 65535;
		}
	}
}

/****************************************************/
/* Function : min_max_search                        */
/* Summary  : 最小値、最大値、オフセットの検索  				*/
/* Argument : short data                            */
/* Return   : 0=正常、	-1=データ無効                    */
/* Caution  : 処理高速化のためにポインタ処理に変更
 * notes    : なし
 ****************************************************/
short	min_max_search(short pch){

	short	i;
	short ret;
	long	max_work,min_work,m_work;
	long	up_limit,low_limit;

	short SttPnt, EndPnt;
	long *SumAbs;

	SumAbs = (long *)(&MES[pch].sum_abs[0]);
	
	/*相関値の存在範囲を予測*/
	if(SVD[pch].search_sw == 1 ){		/*相関値サーチ予測機能有効*/
		MES[pch].search_start = MES[pch].max_point_sub_f + 4 - 8;
		if (MES[pch].search_start < 0) MES[pch].search_start = 0;
		if (MES[pch].search_start > SUB_POINT-10) MES[pch].search_start = SUB_POINT-10;
		MES[pch].search_end = MES[pch].max_point_sub_f + 4 + 8;
		if (MES[pch].search_end > (SUB_POINT-1)) MES[pch].search_end = SUB_POINT-1;
		if (MES[pch].search_end < 6) MES[pch].search_end = 6;
	}else{ 							/*相関値サーチ予測機能無効（全範囲0～40をサーチ）*/
		MES[pch].search_start = 0;
		MES[pch].search_end = SUB_POINT-1;
	}

	SttPnt = MES[pch].search_start;
	EndPnt = MES[pch].search_end;

	/*相関値の最低点を見つける*/
	max_work = (long)0;			/*仮の値、最小*/
	min_work = (long)524288;	/*仮の値、最大(4096×128回)*/

	for (i = SttPnt; i < EndPnt; i++)
	{ /*最小値を見つける*/
		if (min_work > *(SumAbs + i))
		{
			min_work = *(SumAbs + i); /*最小値保存*/
			m_work = i;						/*最小オフセット格納*/
		}
		if (max_work < *(SumAbs + i))
		{
			max_work = *(SumAbs + i); /*最大値保存*/
		}
	}
	/*最大、最小の比をチェック*/
	MES[pch].correlate = (short)((max_work - min_work) / 1000);/*実相関値幅係数*/

	up_limit = SVD[pch].corr_up.DWORD;		/*相関値幅上限値*/
	low_limit = SVD[pch].corr_low.DWORD;	/*相関値幅下限値*/
	if(((min_work) < (max_work *10 / SVD[pch].correlate_level)) && 
	    ((max_work - min_work) > low_limit) && ((max_work - min_work) < up_limit) 
	){ /*上限リミット*/
		MES[pch].min_point_m = (short)m_work;	/*offset(0-19)*/
		MES[pch].sum_max = max_work;			/*最大値更新*/
		MES[pch].sum_min = min_work;			/*最小値更新*/
		MES[pch].data_ok = 0;					/*データ有効*/
		MES[pch].err_status &= ~ERR_JUDGE_PRE_CALC;	/*エラーリセット*/
		ret = (short)B_OK;
	}else{
		MES[pch].data_ok = 1;					/*データ無効*/
		MES[pch].err_status |= ERR_JUDGE_PRE_CALC;		/*エラーセット*/
		ret = (short)B_NG;							
	}
	
	return ret;
}

/****************************************************/
/* Function : delta_ts_cal                           */
/* Summary  : 時間差を演算する    				*/
/* Argument : pch                              */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 1000=31.25nsec                        */
/****************************************************/
void	delta_ts_cal(short pch){

	short m,i;
	long s_work;

	m = MES[pch].min_point_m;

	if(m <= 1){
		MES[pch].delta_ts0 = (unsigned short)1500;/*負の場合は下限値*/
	}else if( m >= abs_cont[SVD[pch].sensor_size]){
		MES[pch].delta_ts0 = (unsigned short)(abs_cont[SVD[pch].sensor_size]-1)*1000+500;/*⊿Ｔの上限値*/
	}else{
		/*⊿Ｔｓの算出*/
		if (MES[pch].sum_abs[m-1] > MES[pch].sum_abs[m+1]){/*s0>s2*/
			s_work = MES[pch].sum_abs[m-1];/*S0*/
		}else{/*s0<=s2*/
			s_work = MES[pch].sum_abs[m+1];/*S2*/
		}
		/*演算結果の格納メモリー選択（要注意）*/
		MES[pch].delta_ts0 = ((unsigned short)m * (unsigned short)1000) + (unsigned short)((MES[pch].sum_abs[m-1]-MES[pch].sum_abs[m+1])*500/(s_work - MES[pch].sum_abs[m])); 	
	}
}

/****************************************************/
/* Function : temp_v                           */
/* Summary  : 温度補正用、音速測定    				*/
/* Argument : pch                           */
/* Return   : 受波しきい値を下回るときのデータ点*/
/* Caution  : なし                                   */
/* notes    : 打ち込みパルスから波形先頭位置までの時間を測定するのに使用
 *          : LeadingPosition(受波バッファの先頭から10点目)からPまでの点数を求める
 ****************************************************/
short	temp_v(short pch){

	short	i;
	short v_work_fow,v_work_rev;
	unsigned short wave_level;
	short work_old;
	short SelMult[] = { 32, 40, 65, 80 }; // 現行クランプオンではサンプリング周波数に近似?

	/*受波波形検出レベル*/
	wave_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].wave_vth / 100);	/*最大値(12bit MAX)*判定閾値*/
	work_old = 0;
	
#if defined(DatInc)
	SelMult[SVD[pch].adc_clock] = SelMult[SVD[pch].adc_clock] / Mgn;
#endif

#if defined(FLWWAVEXP)
	for(i=10; i<FLWWAVSIZ; i++)
#else
	for(i=10; i<250; i++)
#endif
	{
		if(MES[pch].ThresholdPeakPos == 0){  /*波形認識閾値の未設定(ゼロ点調整未実施)*/
			/*受波しきい値を探す*/
			if( MES[pch].fow_data[i] < wave_level)
			{
				v_work_fow = i;
				break;
			}
		}else{
			/*signal_count-50位置のﾏｲﾅｽ半周期～プラス半周期のピークを探す*/
			if((50 - (SelMult[SVD[pch].adc_clock] / 2)) <= i && i <= (50 + (SelMult[SVD[pch].adc_clock] / 2))){
				if(MES[pch].fow_data[i] < work_old){
					v_work_fow = i - 1;  /*1つ前がピーク*/
					break;
				}else{
					work_old = MES[pch].fow_data[i];
				}
			}
		}

	}
#if defined(FLWWAVEXP)
	if(i != FLWWAVSIZ + 10)
#else
	if(i != 250)
#endif
	{
		MES[pch].ThreasholdPoint_Fow = v_work_fow;/*fow*/
	}

	 work_old = 0;
#if defined(FLWWAVEXP)
	for(i=10; i<FLWWAVSIZ + 10; i++)
#else
	for(i=10; i<250; i++)
#endif
	{
		if(MES[pch].ThresholdPeakPos == 0){  /*波形認識閾値の未設定(ゼロ点調整未実施)*/
			/*受波しきい値を探す*/
			if( MES[pch].rev_data[i] < wave_level)
			{
				v_work_rev = i;
				break;
			}
		}else{
			/*signal_count-50位置のﾏｲﾅｽ半周期～プラス半周期のピークを探す*/
			if((50 - (SelMult[SVD[pch].adc_clock] / 2)) <= i && i <= (50 + (SelMult[SVD[pch].adc_clock] / 2))){
				if(MES[pch].rev_data[i] < work_old){
					v_work_rev = i - 1;  /*1つ前がピーク*/
					break;
				}else{
					work_old = MES[pch].rev_data[i];
				}
			}
		}
	}
#if defined(FLWWAVEXP)
	if(i != FLWWAVSIZ)
#else
	if(i != 250)
#endif
	{
		MES[pch].ThreasholdPoint_Rev = v_work_rev;/*rev*/
	}

	if((SVD[pch].fix_data & 0x10) != 0){  //固定値設定
 	MES[pch].max_point_sub = (MES[pch].zero_sonic_point_fow_p1 - MES[pch].zero_sonic_point_rev_p2 + 50) / 100;		/*受波の差*/
 }else{
#if defined(DatInc)
	 MES[pch].max_point_sub = (MES[pch].sonic_point_fow_p1 - MES[pch].sonic_point_rev_p2 + 50) / 100;		/*受波の差*/
	 MES[pch].max_point_sub *= Mgn;
#else
	 MES[pch].max_point_sub = (MES[pch].sonic_point_fow_p1 - MES[pch].sonic_point_rev_p2 + 50) / 100;		/*受波の差*/
#endif
	}
	
	if(MES[pch].ThresholdReq == 0){	/*波形認識実行時はチェックしない*/
		if(MES[pch].max_point_sub_f >= LIM_OVERFLOW){
//			MES[pch].err_status |= ERR_JUDGE_OVERFLOW;		/*音速固定とするのでオーバーフローはチェックしない*/
		}else{
			MES[pch].err_status &= ~(ERR_JUDGE_REVERSE+ERR_JUDGE_OVERFLOW);	/*エラーリセット*/
		}
	}

	return (short)((MES[pch].ThreasholdPoint_Fow + MES[pch].ThreasholdPoint_Rev - 20) / 2);/*平均*/
	// return (short)((MES[pch].ThreasholdPoint_Fow + MES[pch].ThreasholdPoint_Rev) / 2);/*平均*/
	
}

/****************************************************/
/* Function : sonic_search_max                   */
/* Summary  : 音速測定    				*/
/* Argument : pch                                  */
/* Return   : なし 									                         */
/* Caution  : 処理高速化のためにポインタ処理に変更
 * notes    : ピーク値から波形先頭位置を求める
 *          : 受波振幅の50%閾値付近にある山とその前後の山から、50%閾値と重なるよう山と山との交線を引き、
 *          : 50%閾値との交点から23word前の位置を上流側はP1、下流側はP2とする。
 *          :
 *          : ADCClock=31.25MHzのとき23word(32ns*23=736ns)前を波形先頭位置にしていた。
 *          : サンプリング周波数[MHz] サンプリング周期[ns] 736nsまでの点数
 *          :                 32MHz             31.25ns          23.55
 *          :                 40MHz             25.00ns          29.44
 *          :                 65MHz             15.38ns          47.84
 *          :                 80MHz             12.50ns          58.88
 *          :                 10MHz            100.00ns           7.36 SFC014E互換
 *****************************************************/
//Y軸データが振れているためY軸3点から回帰直線では先頭位置が安定しない
//X軸データは安定しているためX軸から先頭位置を求める
void	sonic_search_max(short pch){

	short i,work;
	short mult;
	short center;
	short TopPntFow, TopPntRev;
	short SelMult[] = { 32, 40, 65, 80 }; // 現行クランプオンではサンプリング周波数に近似?
	short OffsetPoint[] = { 2355, 2944, 4784, 5888 };

	short RevMaxDatPnt, FowMaxDatPnt;
	short CtrSgnOld, CtrSgnNow;
	short RevCtrDat, FowCtrDat;
	short RevCtrPnt, FowCtrPnt;
	short *RevDat, *FowDat;

	RevMaxDatPnt = MES[pch].rev_max_data_point;
	FowMaxDatPnt = MES[pch].fow_max_data_point;

	RevDat = (short *)(&MES[pch].rev_data[0]);
	FowDat = (short *)(&MES[pch].fow_data[0]);

	/*エラーチェック*/
	if ((( MES[pch].err_status & 0x0007) != 0) 
		|| (MES[pch].rev_max_data_point <= 50) || (MES[pch].fow_max_data_point <= 50)
		|| (MES[pch].rev_max_data <= SONIC_WAVE_HI*MES_SUB[pch].sample_cnt) || (MES[pch].fow_max_data <= SONIC_WAVE_HI*MES_SUB[pch].sample_cnt)){
		return;		/*エラー*/
 }

#if defined(DatInc)
	SelMult[SVD[pch].adc_clock] = SelMult[SVD[pch].adc_clock] / Mgn;
	OffsetPoint[SVD[pch].adc_clock] = OffsetPoint[SVD[pch].adc_clock] / Mgn;
#endif

	mult = SelMult[SVD[pch].adc_clock];  //極大値間のデータ点数

	/*reverse側----------------------*/
	/*rev側のポイントサーチ*/
	MES[pch].rev_center_data = (short)((long)(MES[pch].rev_data[MES[pch].rev_max_data_point] - AD_BASE) * center_ratio / 10000) + AD_BASE;
	RevCtrDat = MES[pch].rev_center_data;

	center = AD_MAX;
	for (i = 0; i < RevMaxDatPnt; i++)
	{
		if ((*(RevDat + i) - *(RevDat + i + 1)) >= 0)
		{
			CtrSgnNow = 0; /*down*/
		}
		else
		{
			CtrSgnNow = 1; /*up*/
		}

		if ((CtrSgnNow == 0) && (CtrSgnOld == 1))
		{
			work = abs(*(RevDat + i) - RevCtrDat); /*centerからの差*/
			if (work < center)
			{
				center = work;
				RevCtrPnt = i;
			}
		}
		CtrSgnOld = CtrSgnNow;
	}
	MES[pch].rev_center_point = RevCtrPnt;
	MES[pch].center_sign_old = CtrSgnOld;
	MES[pch].center_sign_now = CtrSgnNow;

	MES[pch].m0_point_rev[0] = MES[pch].rev_center_point - (mult);
	MES[pch].m0_point_rev[1] = MES[pch].m0_point_rev[0] + (mult);
	MES[pch].m0_point_rev[2] = MES[pch].m0_point_rev[1] + (mult);
	
	for ( i=0; i<3; i++){
		if((0 <= MES[pch].m0_point_rev[i]) && (MES[pch].m0_point_rev[i] < 300)){
			MES[pch].v0_value_rev[i] = MES[pch].rev_data[MES[pch].m0_point_rev[i]] - AD_BASE;/*最小値-,0,+*/
		}
	}
	/*****/
	if ((MES[pch].v0_value_rev[0] >= MES[pch].v0_value_rev[1])
		|| (MES[pch].v0_value_rev[1] >= MES[pch].v0_value_rev[2])){  //3点が昇順であることを確認する
		return;
	}else{
		MES[pch].m0_point_rev_50 = (MES[pch].m0_point_rev[0] + MES[pch].m0_point_rev[1] + MES[pch].m0_point_rev[2]) / 3;
	 	MES[pch].sonic_point_rev_p2 =(short)((long)MES[pch].m0_point_rev_50 * 100) - OffsetPoint[SVD[pch].adc_clock];
	 	MES[pch].sonic_point_rev_p2 *= Mgn;
 }

	/*forward側----------------------*/
	/*forward側のポイントサーチ*/
	MES[pch].fow_center_data = (short)((long)(MES[pch].fow_data[MES[pch].fow_max_data_point] - AD_BASE) * center_ratio / 10000) + AD_BASE;
	FowCtrDat = MES[pch].fow_center_data;

	center = AD_MAX;
	for (i = 0; i < FowMaxDatPnt; i++)
	{
		if ((*(FowDat + i) - *(FowDat + i + 1)) >= 0)
		{
			CtrSgnNow = 0; /*down*/
		}
		else
		{
			CtrSgnNow = 1; /*up*/
		}

		if ((CtrSgnNow == 0) && (CtrSgnOld == 1))
		{
			work = abs(*(FowDat + i) - FowCtrDat); /*centerからの差*/
			if (work < center)
			{
				center = work;
				FowCtrPnt = i;
			}
		}
		CtrSgnOld = CtrSgnNow;
	}
	MES[pch].fow_center_point = FowCtrPnt;
	MES[pch].center_sign_old = CtrSgnOld;
	MES[pch].center_sign_now = CtrSgnNow;

	MES[pch].m0_point_fow[0] = MES[pch].fow_center_point - (mult);
	MES[pch].m0_point_fow[1] = MES[pch].m0_point_fow[0] + (mult);
	MES[pch].m0_point_fow[2] = MES[pch].m0_point_fow[1] + (mult);
	
	for ( i=0; i<3; i++){
		if((0 <= MES[pch].m0_point_fow[i]) && (MES[pch].m0_point_fow[i] < 300)){
			MES[pch].v0_value_fow[i] = MES[pch].fow_data[MES[pch].m0_point_fow[i]] - AD_BASE;/*最小値-,0,+*/
		}
	}

	if ((MES[pch].v0_value_fow[0] >= MES[pch].v0_value_fow[1])
		|| (MES[pch].v0_value_fow[1] >= MES[pch].v0_value_fow[2])){  //3点が昇順であることを確認する
		return;
	}else{
		MES[pch].m0_point_fow_50 = (MES[pch].m0_point_fow[0] + MES[pch].m0_point_fow[1] + MES[pch].m0_point_fow[2]) / 3; 
	 	MES[pch].sonic_point_fow_p1 = (short)((long)MES[pch].m0_point_fow_50 * 100) - OffsetPoint[SVD[pch].adc_clock];/*23は波の先頭*/
	 	MES[pch].sonic_point_fow_p1 *= Mgn;
 }
}

/****************************************************/
/* Function : zero_adj                           */
/* Summary  : ゼロ調整補正	    				*/
/* Argument : pch                          */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	zero_adj(short pch){

	MES[pch].zero_adj_data = (unsigned short)SVD[pch].zero_offset;
	MES[pch].delta_ts_zero = (long)MES[pch].delta_ts - (long)MES[pch].zero_adj_data;/*⊿Ts－ゼロ調整データ*/
}

/****************************************************/
/* Function : make_viscos_tbl                           */
/* Summary  : 自動リニアライズ（補正テーブル作成）    				*/
/* Argument : pch                              */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 動粘度に対する補正テーブル                   */
/****************************************************/
void	make_viscos_tbl(short pch){
	
	short	i, no;
	short ratio;
	long work;
	short swork;
	short iNum;
	short DataNum;

	KV_TBL *tbl;

	if (SVD[pch].viscos_auto == 0){	/*動粘度、設定固定*/
		/* 薬液リニアライズモードON = 動粘度1.00 */
		if(SVD[pch].LL_enable == 1 && SVD[pch].LL_kind != 0){
			MES[pch].viscos_val = 100;
		}else{
			MES[pch].viscos_val = SVD[pch].viscos;/*設定動粘度*/
		}
	}else{								/*動粘度の演算（水）*/
		/*動粘度cp = 12.069 - 0.00746 * V0(音速m/sec)*/
		work = ((long)1206900 - ((long)MES[pch].sound_vel_f * 746)+ 500) / 1000;

		if (work <= 30)   work = 30;	/* 0.30*/
		if (work >= 4000) work = 4000;	/*40.00*/
		MES[pch].viscos_val = (short)work;
	}

	switch(SVD[pch].sensor_size){					/*センサ選択*/
		case SNS_TYPE_1_8:							/* PFA 1/8" */
			tbl = (KV_TBL *)&AUTO_COEFF_1_8[0];
			break;
		case SNS_TYPE_4_3:							/* PFA 4x3 */
			tbl = (KV_TBL *)&AUTO_COEFF_4_3[0];
			break;
		case SNS_TYPE_1_4:							/* PFA 1/4" */
			tbl = (KV_TBL *)&AUTO_COEFF_1_4[0];
			break;
		case SNS_TYPE_3_8:							/* PFA 3/8" */
			tbl = (KV_TBL *)&AUTO_COEFF_3_8[0];
			break;
		default: 									/* others */
			tbl = (KV_TBL *)&AUTO_COEFF_1_4[0];
			break;
	}

	iNum = sizeof(tbl) / sizeof(KV_TBL);
	for (i=0; i<iNum; i++){/*動粘度テーブルを探す*/
		if ( MES[pch].viscos_val < tbl[i].viscos){
			no = i;
			break;
		}
	}
	/*no-1 と　no の間を補間 */
	DataNum = sizeof(tbl[0].dat) / sizeof(short);
	ratio = (short)((long)(MES[pch].viscos_val - tbl[no-1].viscos) * 1000 / (tbl[no].viscos - tbl[no-1].viscos));
	for (i=0; i<DataNum; i++){
		swork = (short)((long)(tbl[no].dat[i] - tbl[no-1].dat[i]) * ratio / 1000 + tbl[no-1].dat[i]);
		MES[pch].viscos_table[i] = (short)((long)100000000 / (swork + 10000));
		/*10000 = 1倍*/
	}
}

float CalcLinerCorrection(long x0, long x1, long y0, long y1, float x)
{
	float xf0 = (float)x0;
	float xf1 = (float)x1;
	float yf0 = (float)y0;
	float yf1 = (float)y1;
	float y = yf0;
	//分母=0になる場合は計算しない
	if(x0 != x1)
	{
		y = (x - xf0) * (yf1 - yf0) / (xf1 - xf0) + yf0;
	}
	return y;
}

/****************************************************/
/* Function : auto_linear                           */
/* Summary  : 自動リニアライズ    				*/
/* Argument : viscos,   pch                         */
/* Return   : 流速[m/sec*E4]                         */
/* Caution  : なし                                   */
/* notes    : 動粘度により流速を補正する                  */
/****************************************************/
long	auto_linear(long viscos, short pch){
#if 1
	short i,no;
	short ratio;
	short malt;
	long work,ret;
	short SignFlg;
	long x0, x1, y0, y1;
	float x, y;
	
	work = viscos;
	
	/*** 一時的に流速を絶対値へ変更 ***/
	if(work < 0){
		SignFlg = 1;
		work = work * -1;
	}else{
		SignFlg = 0;
	}

	for (i=0; i < 21; i++){				/*リ二アポイント数*/
		if ( work >= flow_vel_tbl[i]*10){		/*第一折線からｎ折線まで*/
			break;
		}
	}
	no = i;
	if(no < 1)
	{
		no = 1;
	}
	else if(20 < no)
	{
		no = 20;
	}
	x = (float)work;
	x0 = flow_vel_tbl[no - 1] * 10;
	x1 = flow_vel_tbl[no - 0] * 10;
	y0 = (long)MES[pch].viscos_table[no - 1];
	y1 = (long)MES[pch].viscos_table[no - 0];
	malt = (short)CalcLinerCorrection(x0, x1, y0, y1, x);

	ret = work * malt / 10000;

	/*** 符号判定 ***/
	if(SignFlg != 0){
		ret = ret * -1;				/* 流速が負の値だった場合、真値へ戻す */
	}
#else
	short i,no;
	short ratio;
	short malt;
	long work,ret;
	short SignFlg;
	
	work = viscos;
	
	/*** 一時的に流速を絶対値へ変更 ***/
	if(work < 0){
		SignFlg = 1;
		work = work * -1;
	}else{
		SignFlg = 0;
	}

	for (i=0; i < 21; i++){				/*リ二アポイント数*/
		if ( work >= flow_vel_tbl[i]*10){		/*第一折線からｎ折線まで*/
			break;
		}
	}
	no = i;
	if (no==0){										/*流速が664.29 cm/sec以上*/
		ratio = (flow_vel_tbl[0]*10 - work) * 10000
				/ (flow_vel_tbl[0]*10 - flow_vel_tbl[1]*10); 
		malt = (short) ((long)MES[pch].viscos_table[0] - ((long)( MES[pch].viscos_table[0] - MES[pch].viscos_table[1]) * ratio / 10000));
	}else if (no ==21){								/*流速が1.00 cm/sec 以下*/
		ratio = (flow_vel_tbl[19]*10 - work) * 10000
				/ (flow_vel_tbl[19]*10 - flow_vel_tbl[20]*10); 
		malt = (short) ((long)MES[pch].viscos_table[19] - ((long)( MES[pch].viscos_table[19] - MES[pch].viscos_table[20]) * ratio / 10000));
	}else{
		ratio = (flow_vel_tbl[no-1]*10 - work) * 10000
				/ (flow_vel_tbl[no-1]*10 - flow_vel_tbl[no]*10); 
		malt = (short) ((long)MES[pch].viscos_table[no-1] - ((long)( MES[pch].viscos_table[no-1] - MES[pch].viscos_table[no]) * ratio / 10000));
	}
	ret = work * malt / 10000;

	/*** 符号判定 ***/
	if(SignFlg != 0){
		ret = ret * -1;				/* 流速が負の値だった場合、真値へ戻す */
	}
#endif
	return ret;
}

/****************************************************/
/* Function : maker_linear                           */
/* Summary  : メーカリニアライズ処理   				*/
/* Argument : in_work,  pch                         */
/* Return   : 補正流速						                         */
/* Caution  : なし                                   */
/* notes    : 流速(m/sec)を補正する                    */
/****************************************************/
long	maker_linear(long in_work, short pch){

	short	i, point;
	long *x_ptr, *y_ptr;
	long work, ret;

	point = (short)(SVD[pch].mklnr_num >> 8);	/*補正点数*/
	/*補正処理実行確認*/
	if((point < 1) || (in_work < 0)){
		return (long)in_work;				/*補正しない*/
	}
	if(point > 15){
		point = 15;
	}
	ret = 10000;
	work = (long)in_work;					/*0-10000*/

	/*直線補正*/
	x_ptr = &SVD[pch].mklnr_out1.DWORD;		/*X軸データ先頭アドレス（出力データ）*/
	y_ptr = &SVD[pch].mklnr_in1.DWORD;		/*Y軸データ先頭アドレス（入力データ）*/

	/*1点補正処理*/
	if(point == 1){
		/*原点(0,0)と第1折線で補正処理*/
		ret =  work * ((float)x_ptr[0] / (float)y_ptr[0]);
			
	/*2点以上の補正処理*/
	}else{								
		/*原点(0,0)から第1折線までの補正処理*/
		if(in_work < (y_ptr[0] * 100)){
			ret =  work * ((float)x_ptr[0] / (float)y_ptr[0]);
			return (long)ret;
		}
	
		/*第1折線から第n折線までの補正処理*/
		for(i=0; i < point-1; i++){		/*リ二アポイント数*/
			if((work <= (y_ptr[i+1] * 100))
				&& (work >= (y_ptr[i] * 100))){
				ret = (work - (y_ptr[i] * 100))
					* ((float)(x_ptr[i+1] - x_ptr[i]) / (float)(y_ptr[i+1] - y_ptr[i]))
					+ (x_ptr[i] * 100);
				break;
			}
		}

		/*第n折線より大きい時の補正処理*/
		if(i == point-1){
			i--;
			ret = (work - (y_ptr[i] * 100))
				* ((float)(x_ptr[i+1] - x_ptr[i]) / (float)(y_ptr[i+1] - y_ptr[i]))
				+ (x_ptr[i] * 100);
		}
	}

	return (long)ret;
}

/****************************************************/
/* Function : user_linear                           */
/* Summary  : ユーザリニアライズ処理    				*/
/* Argument : in_flow,  pch                         */
/* Return   : 補正流量						                         */
/* Caution  : なし                                   */
/* notes    : 流量を補正する(1=0.1mL/min)              */
/****************************************************/
long	user_linear(long in_flow, short pch){

	short	i, point;
	long *x_ptr, *y_ptr;
	long work, out_flow;

	point = (short)(SVD[pch].uslnr_num >> 8);			/*補正点数*/
	/*補正処理実行確認*/
	if((point < 1) || (in_flow < 0)){
		return (long)in_flow;					/*補正しない*/
	}
	if(point > 15){
		point = 15;
	}
	out_flow = 10000;
	work = (long)in_flow;

	/*直線補正*/
	x_ptr = &SVD[pch].uslnr_out1.DWORD;			/*X軸データ先頭アドレス（出力データ）*/
	y_ptr = &SVD[pch].uslnr_in1.DWORD;			/*Y軸データ先頭アドレス（入力データ）*/

	/*1点補正処理*/
	if(point == 1){
		/*原点(0,0)と第1折線で補正処理*/
		out_flow =  work * ((float)x_ptr[0] / (float)y_ptr[0]);		
		
	/*2点以上の補正処理*/
	}else{
		/*薬液ﾘﾆｱﾗｲｽﾞﾓｰﾄﾞ(5点補正):補正量追加*/
		if(SVD[pch].LL_enable == 1){
			out_flow = ClcChmLnr(work, point, pch);
		}
		/*通常ﾘﾆｱﾗｲｽﾞﾓｰﾄﾞ*/
		else{
			out_flow = ClcNmlLnr(work, point, pch);
		}
	}

	return (long)out_flow;
}

/****************************************************
 * Function : ClcNmlLnr (Calculate Chemical solution Linerize)
 * Summary  : 薬液リニアライズを計算する
 * Argument : work -> 入力値
 *          : point -> リニアライズ点数
 *          : pch -> チャンネル番号
 * Return   : out_flow -> リニアライズ後流量
 * Caution  : なし
 * notes    : 
 ****************************************************/
long ClcChmLnr(long work, short point, short pch)
{
	short i;
	long *x_ptr, *y_ptr;
	long x_ptr_LL0, x_ptr_LL1;
	long out_flow;
	long x_val0, x_val1;
	long y_val0, y_val1;
	long tbl_val0, tbl_val1;
	long FlwMgn; //Flow Magnification

	//ポインタ初期化
	x_ptr = &SVD[pch].uslnr_out1.DWORD;			/*X軸データ先頭アドレス（出力データ）*/
	y_ptr = &SVD[pch].uslnr_in1.DWORD;			/*Y軸データ先頭アドレス（入力データ）*/

	//変数初期化
	FlwMgn = 10; //
	
	// work *= FlwMgn; //入力が0.1mL/minの場合
	x_val0 = x_ptr[0] * FlwMgn;
	y_val0 = y_ptr[0] * FlwMgn;
	tbl_val0 = LL_TBL[SVD[pch].LL_kind].LinerData[0] * FlwMgn;

	/*原点(0,0)から第1折線までの補正処理*/
	if(work < y_val0){
		x_ptr_LL0 = x_val0 + tbl_val0;
		out_flow =  work * ((float)x_ptr_LL0 / (float)y_val0);
		return (long)out_flow;
	}
	/*第1折線から第n折線までの補正処理*/
	for(i=0; i < point-1; i++){		/*リ二アポイント数*/
		x_val0 = x_ptr[i + 0] * FlwMgn;
		x_val1 = x_ptr[i + 1] * FlwMgn;
		y_val0 = y_ptr[i + 0] * FlwMgn;
		y_val1 = y_ptr[i + 1] * FlwMgn;
		tbl_val0 = LL_TBL[SVD[pch].LL_kind].LinerData[(i + 0)] * FlwMgn;
		tbl_val1 = LL_TBL[SVD[pch].LL_kind].LinerData[(i + 1)] * FlwMgn;

		if((work <= y_val1)
			&& (work >= y_val0)){
			x_ptr_LL1 = x_val1 + tbl_val1;
			x_ptr_LL0 = x_val0 + tbl_val0;
			out_flow = (work - y_val0)
				* ((float)(x_ptr_LL1 - x_ptr_LL0) / (float)(y_val1 - y_val0))
				+ x_ptr_LL0;
			break;
		}
	}
	/*第n折線より大きい時の補正処理*/
	if(i == point-1){
		i--;
		x_val0 = x_ptr[i + 0] * FlwMgn;
		x_val1 = x_ptr[i + 1] * FlwMgn;
		y_val0 = y_ptr[i + 0] * FlwMgn;
		y_val1 = y_ptr[i + 1] * FlwMgn;
		tbl_val0 = LL_TBL[SVD[pch].LL_kind].LinerData[(i + 0)] * FlwMgn;
		tbl_val1 = LL_TBL[SVD[pch].LL_kind].LinerData[(i + 1)] * FlwMgn;

		x_ptr_LL1 = x_val1 + tbl_val1;
		x_ptr_LL0 = x_val0 + tbl_val0;
		out_flow = (work - y_val0)
			* ((float)(x_ptr_LL1 - x_ptr_LL0) / (float)(y_val1 - y_val0))
			+ x_ptr_LL0;
	}
	return out_flow;
}

/****************************************************
 * Function : ClcNmlLnr (Calculate Normal Linerize)
 * Summary  : 通常リニアライズを計算する
 * Argument : work -> 入力値
 *          : point -> リニアライズ点数
 *          : pch -> チャンネル番号
 * Return   : out_flow -> リニアライズ後流量
 * Caution  : なし
 * notes    : 
 ****************************************************/
long ClcNmlLnr(long work, short point, short pch)
{
	short i;
	long *x_ptr, *y_ptr;
	long out_flow;
	long x_val0, x_val1;
	long y_val0, y_val1;
	long FlwMgn; //Flow Magnification

	//ポインタ初期化
	x_ptr = &SVD[pch].uslnr_out1.DWORD;			/*X軸データ先頭アドレス（出力データ）*/
	y_ptr = &SVD[pch].uslnr_in1.DWORD;			/*Y軸データ先頭アドレス（入力データ）*/

	//変数初期化
	FlwMgn = 10; //0.01mL/minを1と表す

	// work *= FlwMgn; //入力が0.1mL/minの場合
	x_val0 = x_ptr[0] * FlwMgn;
	y_val0 = y_ptr[0] * FlwMgn;

	/*原点(0,0)から第1折線までの補正処理*/
	if(work < y_val0){
		out_flow =  work * ((float)x_val0 / (float)y_val0);
		return (long)out_flow;
	}

	/*第1折線からn折線までの補正処理*/
	for(i=0; i < point-1; i++){		/*リ二アポイント数*/
		x_val0 = x_ptr[i + 0] * FlwMgn;
		x_val1 = x_ptr[i + 1] * FlwMgn;
		y_val0 = y_ptr[i + 0] * FlwMgn;
		y_val1 = y_ptr[i + 1] * FlwMgn;

		if((work <= y_val1)
			&& (work >= y_val0)){
			out_flow = (work - y_val0)
				* ((float)(x_val1 - x_val0) / (float)(y_val1 - y_val0))
				+ x_val0;
			break;
		}
	}

	/*第n折線より大きい時の補正処理*/
	if(i == point-1){
		i--;
		out_flow = (work - y_val0)
			* ((float)(x_val1 - x_val0) / (float)(y_val1 - y_val0))
			+ x_val0;
	}

	return out_flow;
}

/****************************************************/
/* Function : LLmode_kind                           */
/* Summary  : 薬液リニアライズモード・種別設定    				*/
/* Argument : vis,  pch                            */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void LLmode_kind(short vis, short pch){

	short i;
	short point;
	short SensorSize = SVD[pch].sensor_size;

	point = (short)(SVD[pch].uslnr_num >> 8) & 0x00FF;	/*リニアライズ点数取得*/
	
	SVD[pch].LL_kind = 0;
	//一致条件検索：ない場合は0設定に
	for(i = 0; i < LL_NUM; i++){
		if(	
			(LL_TBL[i].Vis == vis) &&
			(SensorSize == 3 || SensorSize == 4) &&
			(LL_TBL[i].MaxFlow[SensorSize] == SVD[pch].max_flow) &&
			(LL_TBL[i].LinerPnt == point)
			){
			SVD[pch].LL_kind = i;
		}
	}
}

/****************************************************/
/* Function : flow_calc                           */
/* Summary  : ⊿Tsを流量値に換算する    				*/
/* Argument : pch                                  */
/* Return   : 流量値								                         */
/* Caution  : なし                                   */
/* notes    : 1=0.1mL/min                           */
/*          : fv = L/(f_Ts*(N+(P1+P2)/2) - τs + FIFOch*8.192)
 *          :   fv -> 音速 
 *          :   L  -> 測定長
 *          :   f_Ts -> サンプリング周期
 *          :   N -> 波形先頭位置(FIFOに取り込んだデータ先頭から波形受信バッファ先頭までのデータ点)
 *          :   P1 -> 上流側受波位置(波形受信バッファ先頭から有効な波形の先頭までのデータ点)
 *          :   P2 -> 下流側受波位置(波形受信バッファ先頭から有効な波形の先頭までのデータ点)
 *          :   τs -> センサ遅れ[us]
 *          :   FIFOch -> 受波取り込み開始window位置
 *          :   8.192 -> f_Ts=32ns の時のFIFO1ch分(256word)読み出す時間[us]
 *          : 流速 = (fv*fv/(2*fl)) * ΔTs/1000 * f_Ts/10000
 *          :   fv -> 音速[m/s]
 *          :   fl -> センサ距離[mm]
 *          :   f_Ts -> サンプリング周期[10ps]
 * 
 *          : <------------FIFOに取り込んだデータ(3800word)------------>
 *          :           <-受波バッファ(200word)->
 *          :           <--0--><-有効波形-><-0-->
 *          : <---N----><--P-->
 ****************************************************/
long flow_calc(short pch){

	long work_vel;
	long out_flow;
	short s_d;
	short s_v;
	short f_Ts;
	short sns_L_l;
	short sns_L;
	short sns_TAU;
	long ReadTimePerFifoch; // FIFO CH 1ch(256word or 128word)を読むのにかかる時間
	long DataPointToWaveHead;
	long Dnmntr; //分母
	float zc_flow_val;
	float zc_deltaT;

	ReadTimePerFifoch = (long)(SmpTs[SVD[pch].adc_clock] * 256 / 100); //[us:1000倍データ] ,Mercuryセンサは全て256word?

	s_d = SVD[pch].sensor_size; // センサ口径
	s_v = MES[pch].sound_vel_f; // 水中の音速 (m/sec)
	if (SVD[pch].sns_option == 0)
	{
		sns_L_l = sens_inf[s_d].sns_disL_l; // センサ距離(L-l) (*0.1mm単位)
		sns_L = sens_inf[s_d].sns_disL;		// センサ距離(L) (*0.1mm単位)
		sns_TAU = sens_inf[s_d].sns_TAU;
		f_Ts = SmpTs[SVD[pch].adc_clock]; // サンプリング周期
	}
	else
	{
		sns_L_l = SVD[pch].sns_disL_l; // センサ距離(L-l) (*0.1mm単位)
		sns_L = SVD[pch].sns_disL;	   // センサ距離(L) (*0.1mm単位)
		sns_TAU = SVD[pch].sns_tau;
		f_Ts = SmpTs[SVD[pch].adc_clock]; // サンプリング周期
	}
	MES[pch].FwdSurplsTim = MES[pch].RevSurplsTim = sns_TAU;

	/*流速を求める V = 1/2 * C^2/L * ⊿t */
	//差分相関
	if (ClcActFlg == ClcAct_SumAdd)
	{
		MES[pch].delta_ts_zero_f = MES[pch].delta_ts_zero;

		work_vel = (long)((((float)s_v * (float)s_v / (2.0 * (float)sns_L_l)) * 10) * (float)f_Ts / 1000);
		MES[pch].flow_vel = work_vel * MES[pch].delta_ts_zero_f / 10000; /*流速[m/s*E4](V)*/
		MES[pch].flow_vel_a = (long)MES[pch].flow_vel;
	}
	//ゼロクロス
	else
	{
		zc_deltaT = MES_SUB[pch].zc_delta_ts_zero * 100;
	    zc_flow_val = (((float)s_v * (float)s_v / (2 * (float)sns_L_l)) * 10 * zc_deltaT);
		MES[pch].flow_vel_a = (long)zc_flow_val;
	}

	/*メーカリニアライズ*/
	MES[pch].flow_vel_b = maker_linear(MES[pch].flow_vel_a, pch);

	make_viscos_tbl(pch);

	if (SVD[pch].viscos == 0)
	{ /*動粘度設定「0」時は自動リニアライズ無効*/
		MES[pch].flow_vel_c = MES[pch].flow_vel_b;
	}
	else
	{																 /*自動リニアライズ有効*/
		MES[pch].flow_vel_c = auto_linear(MES[pch].flow_vel_b, pch); /*自動リニアライズ*/
	}

	/*流量[mL/min*10]*/ /*0.01ml/min*/
	out_flow = MES[pch].flow_vel_c * (long)sens_inf[s_d].area / 1000 * (long)6 / 10;

	/*3点から位置を求める。sonic_point上流下流の平均*/
	if ((MES[pch].err_status & 0x000F) != 0)
	{
		;
	}
	else
	{

		// Debug
		if (TopPosFix == PosFix_NotMov)
		{
			DataPointToWaveHead = (long)(((long)MES[pch].zero_signal_count - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
			// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
			// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
		}
		else
		{
			if ((SVD[pch].fix_data & 0x10) != 0 && (SVD[pch].fix_data & 0x08) == 0)
			{ // 固定値設定
				DataPointToWaveHead = (long)(((long)MES[pch].signal_count - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].signal_count - (long)50 + (((long)MES[pch].zero_sonic_point_rev_p2 + MES[pch].zero_sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
				MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			}
			else if ((SVD[pch].fix_data & 0x10) == 0 && (SVD[pch].fix_data & 0x08) != 0)
			{ // 固定値設定
				DataPointToWaveHead = (long)(((long)MES[pch].zero_signal_count - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
				MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			}
			else if ((SVD[pch].fix_data & 0x18) != 0)
			{ // 固定値設定
				DataPointToWaveHead = (long)(((long)MES[pch].zero_signal_count - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].zero_sonic_point_rev_p2 + MES[pch].zero_sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].zero_sonic_point_rev_p2 + MES[pch].zero_sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
				MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			}
			else
			{
#if defined(DatInc)
				MES[pch].ThreasholdPoint *= Mgn;
				// MES[pch].sonic_point_rev_p2 *= Mgn;
				// MES[pch].sonic_point_fow_p1 *= Mgn;
				DataPointToWaveHead = (long)(((long)MES[pch].signal_count * 100 - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
				// DataPointToWaveHead /= Mgn;
				f_Ts /= Mgn;
				ReadTimePerFifoch /= Mgn;
#else
				DataPointToWaveHead = (long)(((long)MES[pch].signal_count * 100 - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
#endif
				//  MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
				MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			}
		}
		// Debug
		MES[pch].sound_vel = (short)((long)MES[pch].sound_vel * sns_L / 1000); /*100mmを1倍とする*/
	}

	return out_flow;
}

/****************************************************/
/* Function : pv_calc                           */
/* Summary  : 流量値をKファクタ補正する   				*/
/* Argument : in_flow,   pch                        */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 入力流量 (1=0.01mL/min)                */
/*          : 出力流量 (1=0.01mL/min)               */
/****************************************************/
long	pv_calc(long in_flow, short pch){

	short i,work;
	short k_scale_work;
	float out_flow;
	
	/*Full Scale value*//*フルスケール値の決定*/
	if ((SVD[pch].unit & 0x00FF) == 6){/*unit=L/min*/
		work = SVD[pch].unit >> 8;/*decimal point*/
		work = 3 - work;
		MES[pch].max_flow_long = (long)SVD[pch].max_flow;
		for ( i=0; i<work; i++){
			MES[pch].max_flow_long *= 10;
		}
	}else if((SVD[pch].unit & 0x00FF) == 9){/*unit=m3/h*/
		work = SVD[pch].unit >> 8;/*decimal point*/
		work = 3 - work;
		MES[pch].max_flow_long = (long)SVD[pch].max_flow;
		for ( i=0; i<work; i++){
			MES[pch].max_flow_long *= 10;
		}
		MES[pch].max_flow_long = MES[pch].max_flow_long * 100 / 6;/* 10**6/60 */
	}else{
		work = SVD[pch].unit >> 8;/*decimal point*/
		MES[pch].max_flow_long = (long)SVD[pch].max_flow;/*mL/min*/
		for ( i=0; i<work; i++){
			MES[pch].max_flow_long /= 10;
		}
	}

	/*Kファクタ互換係数*/
	if(SVD[pch].sns_option == 0){	
		k_scale_work = sens_inf[SVD[pch].sensor_size].k_scale;
	}else{
		k_scale_work = SVD[pch].sns_coef;
	}
	out_flow = (float)in_flow * ((float)10000 / (float)k_scale_work);

	/*Kファクタ（0.700-3.000)*/
	out_flow = out_flow * SVD[pch].k_factor / 1000;

	/*流量値リミットチェック*/
	if(out_flow > (MES[pch].max_flow_long * 100 * 2)){
		out_flow = MES[pch].max_flow_long * 100 * 2;	/*上限(フルスケールの2倍)*/
	}
	if(out_flow < (-MES[pch].max_flow_long * 100 )){
		out_flow = -MES[pch].max_flow_long * 100;		/*下限(フルスケールの-1倍)*/
	}
		
	return (long)out_flow;
}

/****************************************************/
/* Function : sonic_filter_control                */
/* Summary  : 音速フィルタ処理    				*/
/* Argument : ch                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	sonic_filter_control(short ch){

	int	work_filt;
	
	MES[ch].flow_filt_sv_v = SVD[ch].sound_vel_filter * T100MS;	/* (音速フィルタ時定数*10)*(100msec割込み回数) */
	work_filt = MES[ch].sound_vel;
	MES[ch].flow_filt_in_v = (long)work_filt * 10000;
	Filt(&MES[ch].flow_filt_sv_v ,&MES[ch].flow_filt_in_v ,&MES[ch].flow_filt_out_v );

	if(SVD[ch].sound_vel_sel == 0){		/*音速固定の場合*/
		MES[ch].sound_vel_f = SVD[ch].sound_vel_fix;
	}else{
 	MES[ch].sound_vel_f = (short)(MES[ch].flow_filt_out_v / 10000);
	}
}

/****************************************************/
/* Function : correlate_check                       */
/* Summary  : 演算異常判定    				*/
/* Argument : ｃｈ                            */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	correlate_check(short ch){
	
	if((min_max_check(ch) < 0) || 			// 差分相関値の比のエラーが、演算異常判定回数連続で続いた場合
	    ((MES[ch].fow_max_data - MES[ch].fow_min_data) < 100)){	// 波形の振幅が100以下の場合
		MES[ch].err_status |= ERR_JUDGE_CALC;			/*エラーセット*/
	}else{												/*波形あり*/
		MES[ch].err_status &= ~ERR_JUDGE_CALC;		/*エラーリセット*/
	}
}

/****************************************************/
/* Function : max_point_control                   */
/* Summary  : 受波の差(p1-P2)    				*/
/* Argument : ch                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	max_point_control(short ch){

	int	work_filt;
	
	/*sonic_point_sub*/
	MES[ch].sonic_filt_sv = 3 * T100MS;	/* 0.3S *0.025 sec*/
	work_filt = (short)(MES[ch].max_point_sub);/*P1-P2*/
	MES[ch].sonic_filt_in_s = (long)work_filt * 10000;		/**/
	Filt(&MES[ch].sonic_filt_sv ,&MES[ch].sonic_filt_in_s ,&MES[ch].sonic_filt_out_s );
	MES[ch].max_point_sub_f = (short)(MES[ch].sonic_filt_out_s / 10000);
}

/****************************************************/
/* Function : median_filter_control                 */
/* Summary  : メディアンフィルタ    				*/
/* Argument : in_flow,   ch                         */
/* Return   : メディアンフィルタ後の値                       */
/* Caution  : なし                                   */
/* notes    : 1=0.01mL/min                          */
/****************************************************/
long median_filter_control(long in_flow, short ch){

	long	out_flow;

	out_flow = in_flow;

	if(SVD[ch].filter_mode == FILTER_DIRECT		//フィルタ無し設定時
	|| SVD[ch].filter_mode == FILTER_MOVING){	//移動平均(1-50回)設定時
		;
	}else{		//メディアンフィルタ設定時
		if((MES[ch].err_status & ERR_MEDIAN_CALC) == 0) {	/*対象エラー発生時はメディアン処理しない*/
			if(MES[ch].median_ptr < (MF_DEPTH-1)){
				MES[ch].median_ptr++;
			}else{
				MES[ch].median_ptr = 0;
			}
			MES[ch].median_buf[MES[ch].median_ptr] = in_flow;
			out_flow = median_filter(&(MES[ch].median_buf[0]));		/*ml_min_d (0.01mL/minを1で表す)*/
		}
	}
	return (long)out_flow;
}

/*******************************************
 * Function : ClcFlwFlt_MvgAve (Calculate Flow Filter Moving Average)
 * Summary  : 流量に対する移動平均の計算
 * Argument : 
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
long ClcFlwFlt_MvgAve(long in_flow, short pch)
{
	short i;
	float OutFlw = 0;
	unsigned short SumNum = SVD[pch].filter_avg;
	short BufCnt = MES[pch].MvgAveFlwCnt;
	
	//上限設定
	if(SumNum > 50){
		SumNum = 50;
	}
	MES[pch].MvgAveFlwBuf[BufCnt] = in_flow;

	//バッファの総和計算
	MES[pch].MvgAveFlwSum = 0;
	for(i=0; i<SumNum; i++){
		MES[pch].MvgAveFlwSum += MES[pch].MvgAveFlwBuf[i];
	}

	//移動平均値計算
	OutFlw = ((float)MES[pch].MvgAveFlwSum / SumNum);
	OutFlw = RoundFunc(OutFlw);	//四捨五入

	//カウンタ更新
	BufCnt++;
	if(BufCnt >= SumNum){
		BufCnt = 0;
	}
	MES[pch].MvgAveFlwCnt = BufCnt;

	return (long)OutFlw;
}

/*******************************************
 * Function : ClcFlwFlt (Calculate Flow Filter)
 * Summary  : 流量に対するフィルタの計算
 * Argument : 
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
long ClcFlwFlt(long in_flow, short pch)
{
	long OutFlw = 0;
	switch (SVD[pch].filter_mode)
	{
	case FILTER_MOVING: //移動平均
		OutFlw = ClcFlwFlt_MvgAve(in_flow, pch);
		break;
	case FILTER_MEDIAN: //メディアンフィルタ
		OutFlw = median_filter_control(in_flow, pch);
		break;
	default: //フィルタなし
		OutFlw = in_flow;
		break;
	}
	return OutFlw;
}

/****************************************************/
/* Function : damp_control                           */
/* Summary  : ダンピング処理    				*/
/* Argument : ch                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	damp_control(short ch){

short damp_work;

	/*フィルタ処理*/
	damp_work = SVD[ch].damp;
	MES[ch].flow_filt_sv = damp_work * T100MS;
	MES[ch].flow_filt_in = (long)MES[ch].ml_min_now * 1000;		/*ワークエリア拡大*/
	Filt(&MES[ch].flow_filt_sv ,&MES[ch].flow_filt_in ,&MES[ch].flow_filt_out );/*フィルタ処理*/
	MES[ch].ml_min_now = (MES[ch].flow_filt_out / 1000);			/*出力スケールを戻す*/
}

/****************************************************/
/* Function : lowcut_control                        */
/* Summary  : ローカット処理    				*/
/* Argument : ch                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	lowcut_control(short ch){

	work_oq = MES[ch].ml_min_now;	/*oqコマンド用流量値を保持*/

	if(SVD[ch].low_cut != 0){				/*ローカット0%設定時は無効*/
		if(MES[ch].ml_min_now <= ((long)MES[ch].max_flow_long * (long)SVD[ch].low_cut * 10) / 100){
			MES[ch].ml_min_now = 0;	
		}
	}
}

/****************************************************/
/* Function : burnout_control                       */
/* Summary  : バーンアウト処理    				*/
/* Argument : ch                            */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	burnout_control(short ch){
	
	short work_buf;

	if(((MES[ch].err_status & ERR_BURN_OUT) != 0)	/*波形異常時*/
			&& (MES[ch].err_burn_out == B_ON)){		/*エラーホールドタイム経過*/
		work_buf = SVD[ch].burnout;				/* バーンアウト判定 */
		/*バーンアウト処理*/
		switch(work_buf){						/*出力選択*/
			case 0:							/*0%設定*/
				MES[ch].ml_min_now = work_oq = 0;
				break;
			case 1:							/*-25%設定*/
				MES[ch].ml_min_now = work_oq = MES[ch].max_flow_long * (-25); 
				break;
			case 2:							/*125%設定*/
				MES[ch].ml_min_now = work_oq = MES[ch].max_flow_long * 125;
				break;
			case 3:							/*任意設定(-300～300%)*/
				MES[ch].ml_min_now = work_oq = MES[ch].max_flow_long * SVD[ch].burnout_value;
				break;
			case 4:							/*HOLD設定*/
				break;
			default:
				break;
		}
	}else{						/*正常波形時*/
		;
	}
}

/****************************************************/
/* Function : median_filter                        */
/* Summary  : メディアンフィルタ処理                  				*/
/* Argument : *d                                  */
/* Return   : メディアンフィル後の値	                      */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
long median_filter(long *d) {
	long t;
	long sum;
	short i, j;

#if 1
//copy data
	for(i=0; i<MF_DEPTH; i++) {
		mf_data[i] = d[i];
	}

//sort
	for(i=0; i<(MF_DEPTH-1); i++) {
		for(j=(MF_DEPTH-1);j>i;j--){
			if(mf_data[j]<mf_data[j-1]) {
				t=mf_data[j];
				mf_data[j]=mf_data[j-1];
				mf_data[j-1]=t;
			}
		}
	}
//culc avarage of medium 16
	sum = 0;
	for(i=MF_AVE_ST; i<=MF_AVE_EN; i++) {
		sum += mf_data[i];
	}
#else
	long *MdfDat;
	MdfDat = (long *)(&mf_data[0]);

//copy data
	for (i = 0; i < MF_DEPTH; i++)
	{
		*(MdfDat + i) = *(d + i);
	}

//sort
	for (i = 0; i < (MF_DEPTH - 1); i++)
	{
		for (j = (MF_DEPTH - 1); j > i; j--)
		{
			if (*(MdfDat + j) < *(MdfDat + j - 1))
			{
				t = *(MdfDat + j);
				*(MdfDat + j) = *(MdfDat + j - 1);
				*(MdfDat + j - 1) = t;
			}
		}
	}
//culc avarage of medium 16
	sum = 0;
	for (i = MF_AVE_ST; i <= MF_AVE_EN; i++)
	{
		sum += *(MdfDat + i);
	}
#endif
	return (sum / (MF_AVE_EN-MF_AVE_ST+1));
}

/****************************************************/
/* Function : get_attenuator_gain                   */
/* Summary  : アッテネータゲイン値の取得    				*/
/* Argument : pch                                 */
/* Return   : アッテネータゲイン値						                   */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short get_attenuator_gain(short pch)
{
	short	gain;

//	gain = SVD[pch].atn_gain;		/*アッテネータゲイン値取得*/
	gain = 0;
	
	if(gain < 0){					/*アッテネータゲイン値リミットチェック*/
		gain = 0;
	}
	if(gain > 255){				/*アッテネータゲイン値リミットチェック*/
		gain = 255;
	}

	return gain;
}

/****************************************************/
/* Function : Filt                           */
/* Summary  : フィルタ処理                         				*/
/* Argument : *Tf, *IN, *OUTn                       */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void Filt(const short *Tf, long *IN, long *OUTn){

 if (*Tf != 0) {
  *OUTn = *OUTn + ((long)Dt * (long)(*IN - *OUTn ) / *Tf);
 }else{
  *OUTn = *IN;
 }
}

/****************************************************/
/* Function : addit_req_check                      */
/* Summary  : 積算指示確認    				*/
/* Argument : ch                                */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
short	addit_req_check(short ch){

	short sts;
	short port_sts;
	short i_cnt;
	short on_cnt;
	short off_cnt;
	
	sts = B_OFF;
	port_sts = on_cnt = off_cnt = 0;

	if(ch >= CH_NUMMAX){
		return(sts);
	}

	if(com_type == COM_RS485){
		for(i_cnt=0; i_cnt<3; i_cnt++){				//3回読み込み（チャタリング対策）
			//RS485通信による積算要求
			port_sts = 0;

			if(__bit_input(GPIO_PORTS_BASE, 3)== 1)	// GPIO_PS3 (EXT_IN6)
			{
				port_sts |= (1 << 6);	// RX 版 FW の EXT_IN6 にビット位置を合わせる 
			}
			if(__bit_input(GPIO_PORTS_BASE, 2)== 1)	// GPIO_PS2 (EXT_IN5)
			{
				port_sts |= (1 << 4);
			}
			if(__bit_input(GPIO_PORTS_BASE, 1)== 1)	// GPIO_PS1 (EXT_IN4)
			{
				port_sts |= (1 << 3);
			}
			if(__bit_input(GPIO_PORTS_BASE, 0)== 1)	// GPIO_PS0 (EXT_IN3)
			{
				port_sts |= (1 << 2);
			}
			if(__bit_input(GPIO_PORTP_BASE, 5)== 1)	// GPIO_PP5 (EXT_IN2)
			{
				port_sts |= (1 << 1);
			}
			if(__bit_input(GPIO_PORTP_BASE, 2)== 1)	// GPIO_PP2 (EXT_IN1)
			{
				port_sts |= (1 << 0);
			}

			switch(ch){
				case	CH1:		//CH1積算指示
						if((port_sts & 0x0001) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH2:		//CH2積算指示
						if((port_sts & 0x0002) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH3:		//CH3積算指示
						if((port_sts & 0x0004) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH4:		//CH4積算指示
						if((port_sts & 0x0008) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH5:		//CH5積算指示
						if((port_sts & 0x0010) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH6:		//CH6積算指示
						if((port_sts & 0x0040) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				default:
						break;
			}
		}
		if(on_cnt == 3){			//3回連続オンで積算指示あり
			addit_sts = B_ON;
		}
		if(off_cnt == 3){			//3回連続オフで積算指示なし
			addit_sts = B_OFF;
		}
		sts = addit_sts;
	
	}else{
		if(MES[ch].addit_req != 0){		//CUnet通信による積算要求
			sts = B_ON;
		}

	}
	return(sts);
}

/****************************************************/
/* Function : addit_flow_calc                       */
/* Summary  : 積算流量処理    				*/
/* Argument : なし　　　　　　                           */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void addit_flow_calc(void){

	short ch;
	short unit_val;
	long pv_now_work;

	unit_val = 0;
	for(ch=CH1; ch<CH_NUMMAX; ch++){
		if(LED[ch].zero_active != 0){	/*ゼロ点調整中は積算流量処理しない*/
			continue;
		}

		if(addit_req_check(ch) != B_ON){					/*積算指示確認*/
			if(MAIN[ch].com_act_status == ACT_STS_ADDIT){ //動作ステータスが積算実行中の場合
				action_status_control(ch, ACT_STS_NORMAL);		/*動作ステータス更新*/
			}
			if(MES[ch].addit_req_w == B_ON){				/*積算指示ON→OFF*/
				MES[ch].addit_req_w = B_OFF;				/*積算指示保存*/
				MES[ch].addit_buff.DWORD = MES[ch].addit_unit.DWORD;	/*積算流量バッファ（通信用）更新*/
				MES[ch].addit_buff.DWORD = get_total_offset(ch, MES[ch].addit_buff.DWORD);	//積算値オフセット
			}
			MES[ch].addit_cont.DWORD = 0;					/*積算流量カウンタクリア*/
			MES[ch].addit_unit.DWORD = 0;					/*積算流量単位換算量クリア*/
			MES[ch].addit_mod = 0;							/*積算流量の余りクリア*/
			
			if(MES[ch].addit_watch == B_ON){		//積算監視有効
				if(MES[ch].addit_cont.DWORD != 0	//積算関連データがクリアしていない場合
					|| MES[ch].addit_unit.DWORD != 0
					|| MES[ch].addit_mod != 0){
					MES[ch].total_status |= TTL_JUDGE_CACL_ERR;		//積算値演算異常
				}
			}else{		//積算監視無効
				MES[ch].total_status &= ~TTL_JUDGE_REACH;		//積算到達出力リセット
			}
			continue;
		}
		MES[ch].addit_req_w = B_ON;					/*積算指示保存*/

		action_status_control(ch, ACT_STS_ADDIT);		/*動作ステータス更新*/

		if((MES[ch].err_status & ERR_JUDGE_EMPTY) != 0){	/*受波波形無し時は積算流量処理しない*/
			continue;
		}
		
		//積算流量計算
		if(MES[ch].ml_min_now <= 0){
			pv_now_work = 0;							/*マイナス流量時は「0」換算*/
		}else{
			pv_now_work = MES[ch].ml_min_now;			/*瞬時流量*/
		}
		//積算流量(mL/min)
		MES[ch].addit_pv_w = (unsigned long)pv_now_work * 100;

//#if 1 //6ms割込みに変更
//		//積算流量を6msec換算に変更する
//		//積算流量(mL/min) → 積算流量(mL/6msec)
//		MES[ch].addit_pv = (MES[ch].addit_pv_w + MES[ch].addit_mod) / 10;		/*20000で割るが、微小流量対応として1000倍する*/
//
//		//積算流量の余り計算
//		MES[ch].addit_mod = (MES[ch].addit_pv_w + MES[ch].addit_mod) % 10;
//		MES[ch].addit_mod_ov = (MES[ch].addit_pv_w + MES[ch].addit_mod_ov) % 10;
//#else
		//積算流量を3msec換算に変更する
		//積算流量(mL/min) → 積算流量(mL/3msec)
		MES[ch].addit_pv = (MES[ch].addit_pv_w + MES[ch].addit_mod) / 20;		/*20000で割るが、微小流量対応として1000倍する*/

		//積算流量の余り計算
		MES[ch].addit_mod = (MES[ch].addit_pv_w + MES[ch].addit_mod) % 20;
		MES[ch].addit_mod_ov = (MES[ch].addit_pv_w + MES[ch].addit_mod_ov) % 20;
//#endif

		//積算流量カウンタ更新
		MES[ch].addit_unit.DWORD += MES[ch].addit_pv;
		if(MES[ch].addit_unit.DWORD > 999999999990000){
			MES[ch].addit_unit.DWORD = 0;					/*積算流量単位換算量クリア*/

			if(MES[ch].addit_watch == B_ON){		//積算監視有効
				MES[ch].total_status |= TTL_JUDGE_OVERFLOW;		//積算値オーバーフロー
			}
		}
		MES[ch].addit_unit_ov.UINT64 += MES[ch].addit_pv;
		if(MES[ch].addit_unit_ov.UINT64 > 999999999990000){
			MES[ch].addit_unit_ov.UINT64 = 0;			/*積算流量単位換算量クリア*/

			if(MES[ch].addit_watch == B_ON){		//積算監視有効
				MES[ch].total_status |= TTL_JUDGE_OVERFLOW;		//積算値オーバーフロー
			}
		}

		//積算流量バッファ（ovコマンド通信用）更新
		MES[ch].addit_buff_ov.UINT64 = MES[ch].addit_unit_ov.UINT64;
		MES[ch].addit_buff_ov.UINT64 = get_total_offset(ch, MES[ch].addit_buff_ov.UINT64); //積算値オフセット
		/*積算監視機能*/
		if(MES[ch].addit_watch == B_ON){		//積算監視有効
			if(MES[ch].addit_unit_ov.UINT64 >= (unsigned long long)SVD[ch].target_total.INT32 * 100000){
				MES[ch].total_status |= TTL_JUDGE_REACH; //積算到達出力セット
			}
		}else{		//積算監視無効
			MES[ch].total_status &= ~TTL_JUDGE_REACH; //積算到達出力リセット
		}
	}
}

/****************************************************/
/* Function : reverse_flow_check                    */
/* Summary  : 逆流異常判定    				*/
/* Argument : ch                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	reverse_flow_check(short ch){

	short pv_now_work;
	short	rev_level;

	if(MES[ch].ThresholdReq != 0) return;	/*波形認識実行時はチェックしない*/

	pv_now_work = (short)((MES[ch].ml_min_d * 100) / (MES[ch].max_flow_long));/*瞬時流量(%) (100.00%を10000で表す)*/
	rev_level = SVD[ch].reverse_level;		/*逆流判定しきい値(0～100%)*/
	rev_level *= 100;										/*桁数合わせ(100%を10000で表す)*/

	if(pv_now_work <= 0){											/*マイナス瞬時流量時*/
		if(abs(pv_now_work) >= abs(rev_level)){			/*逆流異常判定*/
			MES[ch].err_status |= ERR_JUDGE_REVERSE;	/*エラーセット*/
		}else{
			MES[ch].err_status &= ~ERR_JUDGE_REVERSE;	/*エラーリセット*/
		}
	}else{																			/*プラス瞬時流量時*/
		MES[ch].err_status &= ~ERR_JUDGE_REVERSE;		/*エラーリセット*/
	}
}

/****************************************************/
/* Function : test_flow_mode                        */
/* Summary  : テスト出力モード    				*/
/* Argument : ch                                    */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	test_flow_mode(short ch){

	if(MES[ch].test_enable == 0)	return;

	if(ch >= CH_NUMMAX){
		return;
	}

	if(MES[ch].test_enable != 0){
		 MES[ch].ml_min_now = (long)MES[ch].max_flow_long * (long)MES[ch].test_flow / 100;
	}
}

/****************************************************/
/* Function : inspect_flow_mode                     */
/* Summary  : 検査モード    				*/
/* Argument : ch                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : なし                                   */
/****************************************************/
void	inspect_flow_mode(short ch){

	short i, work, max, min;

	FPGA_SYNC = 0;
	WriteGainAmp((unsigned short)255);  /*ゲイン書込み*/
	OutputRestartPulse();			/*RESTARTパルス出力*/

	/*REV*/	
	SelectReverseOn(ch);		//IN/OUT打込み切替え
	delay(50);					/*切替後、5usec待機*/
	OutputRestartPulse();		/*RESTARTパルス出力*/

	MES[ch].fifo_start = 0;	/*WINDOW開始時間を設定*/
	MES[ch].fifo_end = 1;	/*WINDOW終了時間を設定*/
	MES[ch].fifo_offset = 0; /*WINDOWオフセット時間を設定*/

	if(initializing == 0){   //起動時の初期化処理が終了している場合
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}

	SetFpgaRegister(ch);   //FPGAレジスタの設定
	OutputStartPulse();		/*STARTパルス出力*/

	fifo_read_wait(ch);  /*受波データ読込み待機*/

	/*FIFO読込み(ダミーリード)*/
	work = FIFO;  /*最初の2word = 0 に成ってる*/
	work = FIFO;  /*2回、ダミーリードする。   */

	max = 0;
	min = AD_MAX;
	for(i = 0; i < 100; i++) {
		work = FIFO;
		if(work > max) max = work;
		if(work < min) min = work;
	}
	wave_hight[0][ch] = max - min;

	// GPIO_PJ1
	while(__bit_input(GPIO_PORTJ_BASE, 1) == 1);		// Windowが終わるまで待つ
	
	SelectForwardOff(ch);		//IN/OUT打込み切替え
	delay(50);					/*切替後、5usec待機*/

	/*FWD*/	
	SelectReverseOn(ch);		//IN/OUT打込み切替え
	delay(50);					/*切替後、5usec待機*/
	OutputRestartPulse();		/*RESTARTパルス出力*/

	MES[ch].fifo_start = 0;	/*WINDOW開始時間を設定*/
	MES[ch].fifo_end = 1;	/*WINDOW終了時間を設定*/
	MES[ch].fifo_offset = 0; /*WINDOWオフセット時間を設定*/

	if(initializing == 0){   //起動時の初期化処理が終了している場合
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}

	SetFpgaRegister(ch);   //FPGAレジスタの設定
	OutputStartPulse();		/*STARTパルス出力*/

	fifo_read_wait(ch);  /*受波データ読込み待機*/

	/*FIFO読込み(ダミーリード)*/
	work = FIFO;/*最初の2word = 0 に成ってる*/
	work = FIFO;/*2回、ダミーリードする。   */

	max = 0;
	min = AD_MAX;
	for(i = 0; i < 100; i++) {
		work = FIFO;
		if(work > max) max = work;
		if(work < min) min = work;
	}
	wave_hight[1][ch] = max - min;

	// GPIO_PJ1
	while(__bit_input(GPIO_PORTJ_BASE, 1) == 1);		// Windowが終わるまで待つ

	SelectReverseOff(ch);		//IN/OUT打込み切替え
	delay(50);					/*切替後、5usec待機*/
}

/****************************************************/
/* Function : InitFifoCh                           */
/* Summary  : FIFO CH初期化    				*/
/* Argument : ch , val                           */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* notes    : 口径変更時に更新する                      */
/****************************************************/
void	InitFifoCh(short ch, short val){

//	SVD[ch].fifo_ch_init = sens_inf[val].fifo_ch;
}

/****************************************************/
/* Function : err_thlevel_init                */
/* Summary  : 波形異常判定閾値の初期化             */
/* Argument : ch,   val                        		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : 口径変更時に更新する        */
/****************************************************/
void	err_thlevel_init(short ch, short val){
	
//	SVD[ch].wave_vth = sens_inf[val].wave_vth;					/*受波検出閾値*/
//	SVD[ch].balance_level = sens_inf[val].balance_level;		/*波形アンバランス検出閾値*/
//	SVD[ch].saturation_level = sens_inf[val].saturation_level;	/*波形飽和検出閾値*/
//	SVD[ch].correlate_level	= sens_inf[val].correlate_level;	/*差分相関比閾値*/
//	SVD[ch].correlate_time 	= sens_inf[val].correlate_time;		/*演算異常判定回数*/
//	SVD[ch].attenuate_level = sens_inf[val].attenuate_level;	/*波形減衰閾値*/
}

/****************************************************/
/* Function : current_flow_control                */
/* Summary  : 瞬時流量値の更新             */
/* Argument : ch                               		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : エラー発生時は瞬時流量値を更新しない         */
/****************************************************/
void current_flow_control(short ch)
{

	if (ch >= CH_NUMMAX)
	{
		return;
	}

	// Debug
	if (ClcActFlg == ClcAct_ZerCrs)
	{
		MES[ch].ml_min_now = MES[ch].ml_min_d; // 瞬時流量を更新
	}
	else
	{
		// Debug

		if ((MES[ch].err_status & ERR_FLOW_CONT) == 0)
		{										   // 流量エラーなし
			MES[ch].ml_min_now = MES[ch].ml_min_d; // 瞬時流量を更新
		}
		else
		{	  // 流量エラーあり
			; // 瞬時流量を更新しない（流量をホールドする）
		}
	}
}

/****************************************************/
/* Function : check_amp_gain                 */
/* Summary  : アンプゲインチェック               */
/* Argument : ch                  			*/
/* Return   : なし		                  */
/* Caution  : なし                               */
/* note     : 予知保全機能                           */
/****************************************************/
void	check_amp_gain(short ch){

	short cnt, num, gain_diff, cyc_num;

	cyc_num = 4;	//ゲイン値取得周期：4回固定

	if(SVD[ch].alm_hold_time == 0){		//判定時間設定なし（予知保全機能の無効化）
		MES[ch].alm_status &= ~(ALM_JUDGE_GAIN);	//ゲイン値急変警告リセット
		return;
	}

	get_cyc[ch]++;		//ゲイン値取得周期更新
	if(get_cyc[ch] < cyc_num){		//ゲイン値取得周期確認
		return;
	}
	get_cyc[ch] = 0;	//ゲイン値取得周期クリア

	for(cnt=AMP_BUFF-1; cnt>0; cnt--){
		MES[ch].amp_gain_change[cnt] = MES[ch].amp_gain_change[cnt-1];	//ゲイン差有無をシフト
	}

	if(((MES[ch].err_status & ERR_JUDGE_EMPTY) == 0) &&
		((MES[ch].amp_gain_for - MES[ch].amp_gain_old) >= SVD[ch].alm_gain_level)){	//ゲイン値の差がアンプゲイン警告閾値以上の場合
		MES[ch].amp_gain_change[0] = B_ON;		//ゲイン差あり
	}else{
		MES[ch].amp_gain_change[0] = B_OFF;		//ゲイン差なし
	}

	gain_diff = 0;
	num = (short)((SVD[ch].alm_hold_time * 10) / (18 * cyc_num)) + 1;
	if(num >= AMP_BUFF){
		num = AMP_BUFF - 1;
	}
	for(cnt=0; cnt<num; cnt++){		//判定時間内のゲイン差を確認する
		if(MES[ch].amp_gain_change[cnt] == B_ON){		//ゲイン差あり？
			gain_diff++;
		}
	}
	if(gain_diff >= SVD[ch].alm_gain_count){	//ゲイン警告ｶｳﾝﾄ以上？
		MES[ch].alm_status |= ALM_JUDGE_GAIN;		//ゲイン値急変警告セット
	}else{
		MES[ch].alm_status &= ~(ALM_JUDGE_GAIN);	//ゲイン値急変警告リセット
	}

	MES[ch].amp_gain_old = MES[ch].amp_gain_for;		//ゲイン値を保持
}

/****************************************************/
/* Function : flow_save_control                     */
/* Summary  : 瞬時流量(10点)保存        				*/
/* Argument : なし                                   */
/* Return   : なし 									                         */
/* Caution  : なし                                   */
/* note     : 定周期(5msec)割込み処理                 */
/****************************************************/
void	flow_save_control(void){

	short		ch;
	short		i_cnt;
	short		past_cnt;
	
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		past_cnt = MES[ch].past_flow_cnt;
		if(past_cnt < 10){		//バッファに10点なし
			if(past_cnt==0){								//10点データ保存カウンタクリア時
				memset(&MES[ch].past_flow[0], 0, sizeof(long)*10);		//バッファクリア
				memset(&MES[ch].past_flow_oq[0], 0, sizeof(long)*10);
			}
			MES[ch].past_flow[past_cnt] = MES[ch].ml_min_OQ;			/*瞬時流量保存(OQ)*/
			MES[ch].past_flow_oq[past_cnt] = MES[ch].ml_min_oq;		/*瞬時流量保存(oq)*/
			if(past_cnt==9){	//バッファに10点あり
				MES[ch].err_status &= ~ ERR_JUDGE_10POINT;		//10点データ無効エラーリセット
			}
			MES[ch].past_flow_cnt++;							//10点データ保存カウンタインクリメント
		}else{					//バッファに10点あり
			memcpy(&MES[ch].past_flow[0], &MES[ch].past_flow[1], sizeof(long)*9);	//バッファ更新
			memcpy(&MES[ch].past_flow_oq[0], &MES[ch].past_flow_oq[1], sizeof(long)*9);

			MES[ch].past_flow[9] = MES[ch].ml_min_OQ;			/*瞬時流量保存(OQ)*/
			MES[ch].past_flow_oq[9] = MES[ch].ml_min_oq;		/*瞬時流量保存(oq)*/
			MES[ch].err_status &= ~ ERR_JUDGE_10POINT;		//10点データ無効エラーリセット
		}
	}
}

/****************************************************/
/* Function : filter_moving                 */
/* Summary  : 移動平均処理 */
/* Argument : なし                  			*/
/* Return   : なし		                  */
/* Caution  : なし                               */
/* note     : なし                                   */
/****************************************************/
void filter_moving(short pch)
{
 short i;
	if(SVD[pch].filter_mode == FILTER_DIRECT){		//フィルタ無し設定時
		MES[pch].delta_ts = MES[pch].delta_ts0;
	}else if(SVD[pch].filter_mode == FILTER_MOVING){		//移動平均(1-50回)設定時
	 if(SVD[pch].filter_avg > 50){
		 SVD[pch].filter_avg = 50;
		}
		MES[pch].delta_ts_buf[MES[pch].ts_cont] = MES[pch].delta_ts0;
		
		MES[pch].delta_ts_sum = 0;
		for(i=0; i<SVD[pch].filter_avg; i++){
			MES[pch].delta_ts_sum += (unsigned long)(MES[pch].delta_ts_buf[i]);
		}

		MES[pch].delta_ts = (unsigned short)(MES[pch].delta_ts_sum / SVD[pch].filter_avg);
	
		MES[pch].ts_cont++;
		if(MES[pch].ts_cont >= SVD[pch].filter_avg){
			MES[pch].ts_cont = 0;
		}
	}else{		//メディアンフィルタ設定時
		/*移動平均(4回)*/
		if(MES[pch].ts_cont < 3){
			MES[pch].ts_cont++;
		}else{
			MES[pch].ts_cont = 0;
		}
		MES[pch].delta_ts_buf[MES[pch].ts_cont] = MES[pch].delta_ts0;
		MES[pch].delta_ts = (unsigned short)(((long)MES[pch].delta_ts_buf[0] + (long)MES[pch].delta_ts_buf[1] + (long)MES[pch].delta_ts_buf[2] + (long)MES[pch].delta_ts_buf[3]) / 4);
 } 
}

/****************************************************
 * Function : zc_filter_moving
 * Summary  : ゼロクロス用移動平均処理
 * Argument : なし
 * Return   : なし
 * Caution  : なし
 * note     : ディザリング機能のため4回移動平均が入っている。
 *          : 0°, 90°, 180°, 270°シフト波形の平均。
 ****************************************************/
void zc_filter_moving(short pch)
{
	//ディザリング機能無効 or フィルタなし
	if(((SVD[pch].fix_data & 0x01) == 0) || (SVD[pch].filter_mode == FILTER_DIRECT))
	{
		MES_SUB[pch].zc_delta_ts = MES[pch].zc_Tdata;
	}
	//ディザリング機能有効 && (メディアンフィルタ or 移動平均 設定時)
	else
	{
		//移動平均(4回)
		if (MES[pch].ts_cont < 3)
		{
			MES[pch].ts_cont++;
		}
		else
		{
			MES[pch].ts_cont = 0;
		}
		MES_SUB[pch].zc_delta_ts_buf[MES[pch].ts_cont] = MES[pch].zc_Tdata;
		MES_SUB[pch].zc_delta_ts = ((MES_SUB[pch].zc_delta_ts_buf[0] + MES_SUB[pch].zc_delta_ts_buf[1] + MES_SUB[pch].zc_delta_ts_buf[2] + MES_SUB[pch].zc_delta_ts_buf[3]) / 4);
	}
	
	MES_SUB[pch].zc_delta_ts_zero = MES_SUB[pch].zc_delta_ts - MES[pch].zc_zero_offset;
}

/****************************************************/
/* Function : InitFPGA                  */
/* Summary  : FPGAの初期化 */
/* Argument : なし                 	           	*/
/* Return   : なし									                     */
/* Caution  : なし                               */
/* note     : なし                              */
/****************************************************/
void	InitFPGA(void)
{
	__bit_output(GPIO_PORTM_BASE, 7, 1);   //XRESET_FPGA ON
}

/****************************************************/
/* Function : WaitCDONE                     */
/* Summary  : CDONE信号の待機         */
/* Argument : なし                            		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : FPGAの起動待機                          */
/****************************************************/
void WaitCDONE(void)
{
	while(__bit_input(GPIO_PORTJ_BASE, 3) == 0);  //CDONE待ち
	delay(200);
	__bit_output(GPIO_PORTM_BASE, 7, 1);   //XRESET_FPGA (FPGA初期化)
	delay(20000);
}

/****************************************************/
/* Function : SetADCClock                  */
/* Summary  : ADC供給クロックを設定する */
/* Argument : short pch        		*/
/* Return   : なし									           */
/* Caution  : なし                               */
/* note     : 32MHz, 40MHz, 65MHz, 80MHz   */
/****************************************************/
void	SetADCClock(short pch)
{
	short data;

	data = SVD[pch].adc_clock;  //0=32MHz, 1=40MHz, 2=65MHz, 3=80MHz
	if((0 <= data) && (data <= 3)){
		FPGA_CLOCK = data;
	}
}

/****************************************************/
/* Function : SetDriveFreq                  */
/* Summary  : 駆動周波数を設定する */
/* Argument : short ch            		*/
/* Return   : なし									           */
/* Caution  : なし                               */
/* note     : 100kHz～5MHz → 1280～26 @SVD[ch].adc_clock = 0   */
/*          :                1600～32 @SVD[ch].adc_clock = 1   */
/*          :                2600～52 @SVD[ch].adc_clock = 2   */
/*          :                3200～64 @SVD[ch].adc_clock = 3   */
/****************************************************/
void	SetDriveFreq(short ch)
{
	short data, freq;
	double AdcClk[4] = {128.0, 160.0, 260.0, 320.0};
	double DblDat = 0.0;
	short RgsMinVal[4] = {26, 32, 52, 64};
	short RgsMaxVal[4] = {1280, 1600, 2600, 3200};

#ifdef FRQSCH
	//周波数サーチモード有効
	if(SVD[ch].drive_search != 0){
		//周波数サーチ中
		if(FrqSch[ch].FrqSchSttFlg == 1){
		// if(FrqSch[ch].FrqSchSttFlg != 0){
			freq = FrqSch[ch].NowFrq;
		}
		else{
			freq = SVD[ch].SchFrq;  //駆動周波数(100kHz～5MHz)
		}
	}
	//周波数サーチモード無効
	else{
		freq = SVD[ch].drive_freq;  //駆動周波数(100kHz～5MHz)
	}
#else
	freq = SVD[ch].drive_freq;  //駆動周波数(100kHz～5MHz)
#endif
	           //周期           /  カウント周期
	// data = ((1000000 / freq) / (1000 / AdcClk[SVD[ch].adc_clock]));
	DblDat = 1000000.0 * AdcClk[SVD[ch].adc_clock] / (double)freq / 1000.0;
	data = (short)(DblDat + 0.5);
	
	if((RgsMinVal[SVD[ch].adc_clock] <= data) && (data <= RgsMaxVal[SVD[ch].adc_clock])){
		FPGA_FREQ = data;  //駆動周波数
	}
}

/****************************************************/
/* Function : SetDrivePulse                  */
/* Summary  : 駆動パルス数を設定 */
/* Argument : short pch        		*/
/* Return   : なし									           */
/* Caution  : なし                               */
/* note     : 1-15          */
/****************************************************/
void	SetDrivePulse(short pch)
{
	short data;

	data = SVD[pch].drive_pls;  //駆動パルス(1-15)

	if((1 <= data) && (data <= 15)){
		FPGA_PULSE = data;
	}
}

/****************************************************/
/* Function : SetWindowPosition                  */
/* Summary  : 読出しWindow開始/終了/オフセットを設定する */
/* Argument : short pch         		*/
/* Return   : なし									           */
/* Caution  : WINDOW_WET は WINDOWWST よりも大きい値を設定する必要がある */
/*          : WINDOW_WET = WINDOWWST の場合、WINDOWWET = WINDOW_WST+1 で内部制御する */
/* note     : Window停止時間[μs] = WINDOW_WET設定値 * ADC_CLK *256 */
/****************************************************/
void	SetWindowPosition(short pch)
{
	short data;

	/*WINDOW開始時間を設定*/
	data = MES[pch].fifo_start;
	if((0 <= data) && (data <= 63)){
		FPGA_START = data;
	}	

	/*WINDOW終了時間を設定*/
	//WINDOW_WET > WINDOWWST となるよう制限
	if(MES[pch].fifo_end < MES[pch].fifo_start){
		data = MES[pch].fifo_start;
	}
	else{
		data = MES[pch].fifo_end;
	}
	if((0 <= data) && (data <= 63)){
		FPGA_END = data;
	}	

	/*読出し開始オフセットを設定*/
	data = MES[pch].fifo_offset;
	if((0 <= data) && (data <= 63)){
		FPGA_OFFSET = data;
	}
}

/****************************************************/
/* Function : SetClockPhase                  */
/* Summary  : 位相パルスを設定 */
/* Argument : short pch        		*/
/* Return   : なし									           */
/* Caution  : なし                               */
/* note     : 0-3          */
/****************************************************/
void	SetClockPhase(short pch)
{
	short data;

	if((SVD[pch].fix_data & 0x01) == 0){  //ディザリング無効
		data = 0;  //位相初期化(固定)
	}else{  //ディザリング有効
		data = MES[pch].clk_phase;
	}

	if((0 <= data) && (data <= 3)){
		FPGA_SYNC = data;
	}
}

/****************************************************/
/* Function : SetFpgaRegister                      */
/* Summary  : FPGAレジスタの設定                       */
/* Argument : pch                              		 */
/* Return   : なし									                         */
/* Caution  : なし                                  */
/* note     : 駆動周波数, 駆動パルス数, ADC供給クロック    */
/*          : 読出し開始オフセット, WINDOW開始/終了時間, 位相パルス*/
/****************************************************/
void	SetFpgaRegister(short pch){

	/*ADC供給クロックを設定*/
	SetADCClock(pch);

	/*駆動周波数を設定*/
	SetDriveFreq(pch);

	/*駆動パルス数を設定*/
	SetDrivePulse(pch);

	/*Window開始/終了/オフセットを設定*/
	SetWindowPosition(pch);

	/*位相を設定*/
	SetClockPhase(pch);

}

/****************************************************/
/* Function : SetDigitalFilterRegister              */
/* Summary  : デジタルフィルタ係数レジスタの設定               */
/* Argument : なし                              		 */
/* Return   : なし						            */
/* Caution  : なし                                  */
/* note     : なし                                   */
/****************************************************/
void	SetDigitalFilterRegister(void){

	FPGA_FIL_EN = SVD[0].DgtFltSwc;	/* デジタルフィルタ切り替え(0-1) */ 
	FPGA_FILIN0_0 = SVD[0].DgtFltCefA00;	/* デジタルフィルタ係数(入力側0) (符号1bit+整数17bit) */
	FPGA_FILIN0_1 = SVD[0].DgtFltCefA01;	/* デジタルフィルタ係数(入力側0) (符号1bit+整数17bit) */
	FPGA_FILIN1_0 = SVD[0].DgtFltCefA10;	/* デジタルフィルタ係数(入力側1) (符号1bit+整数17bit) */
	FPGA_FILIN1_1 = SVD[0].DgtFltCefA11;	/* デジタルフィルタ係数(入力側1) (符号1bit+整数17bit) */
	FPGA_FILIN2_0 = SVD[0].DgtFltCefA20;	/* デジタルフィルタ係数(入力側2) (符号1bit+整数17bit) */
	FPGA_FILIN2_1 = SVD[0].DgtFltCefA21;	/* デジタルフィルタ係数(入力側2) (符号1bit+整数17bit) */
	FPGA_FILOUT1_0 = SVD[0].DgtFltCefB10;	/* デジタルフィルタ係数(出力側1) (符号1bit+整数17bit) */
	FPGA_FILOUT1_1 = SVD[0].DgtFltCefB11;	/* デジタルフィルタ係数(出力側1) (符号1bit+整数17bit) */
	FPGA_FILOUT2_0 = SVD[0].DgtFltCefB20;	/* デジタルフィルタ係数(出力側2) (符号1bit+整数17bit) */
	FPGA_FILOUT2_1 = SVD[0].DgtFltCefB21;	/* デジタルフィルタ係数(出力側2) (符号1bit+整数17bit) */
}

/****************************************************/
/* Function : WriteGainAmp                         */
/* Summary  : ゲイン書込み(TLV5623CDGK)               */
/* Argument : short gain                           */
/* Return   : なし                                  */
/* Caution  : なし                                  */
/* notes    : DAC_SDI      PORTD BIT1              */
/*          : DAC_CLK      PORTD BIT3              */
/*          : DAC_CS_VDBS  PORTD BIT2              */
/*          : DAC_FS       PORTE BIT2              */
/* b15 14  13  12 11 10 9 8 7 6 5 4 3 2 1 0        */
/*  X  SPD PWR X  <- value(8bit) -> 0 0 0 0        */
/*  SPD(Speed control):1→fast mode  0→slow mode    */
/*  PWR(Power control):1→power down 0→normal operation*/
/****************************************************/
void	WriteGainAmp(unsigned short gain){

	short i;
	unsigned short data;

	gain &= 0x00FF;			/*data mask*/
	data = 0x4000;    /*SPD=1(fast mode), PWR=0(normal operation)*/
	data |= (gain << 4);
	
	// GPIO_PE2
	__bit_output(GPIO_PORTE_BASE, 2, 1);				/*FS=1*/
	// GPIO_PD2
	__bit_output(GPIO_PORTD_BASE, 2, 0);				/*CS=0*/
	nop();
	// GPIO_PE2
	__bit_output(GPIO_PORTE_BASE, 2, 0);				/*FS=0*/

	for(i=0; i<16; i++){			/*16bit send*/
		if((data & 0x8000) ==0){
			// GPIO_PD1
			__bit_output(GPIO_PORTD_BASE, 1, 0);		/*Data=0*/
		}else{
			// GPIO_PD1
			__bit_output(GPIO_PORTD_BASE, 1, 1);		/*Data=1*/
		}
		reg_clock();				/*clock*/
		data = data << 1;
	}

	// GPIO_PE2
	__bit_output(GPIO_PORTE_BASE, 2, 1);				/*FS=1*/
	// GPIO_PD2
	__bit_output(GPIO_PORTD_BASE, 2, 1);				/*CS=1*/
	// GPIO_PD1
	__bit_output(GPIO_PORTD_BASE, 1, 0);				/*Data=0*/
}

#ifdef FRQSCH
/****************************************************************************
 * Function : FrqSchPrc
 * Summary  : 周波数サーチ中処理
 * Argument : pch
 * Return   : 0 -> 周波数サーチ中でない
 *          : 1 -> 周波数サーチ中
 * Caution  : なし
 * note     : FrqSch.FrqSchSttFlg の値
 *          : 0 : 周波数サーチ開始前
 *              :   各種変数を初期化する
 *          : 1 : 周波数サーチ中(測定中)
 *              :   累計10回波形取得にトライする
 *              :   5回以上波形異常が起きれば次の周波数に移行
 *              :   10回分の振幅最大値を平均してその周波数の最大振幅とする
 *              :   瞬間的にまぎれた振幅の大きな波形の影響は平均化で少なくなるはず
 *          : 10~ : 周波数サーチ中(待機中)
 *                :   周波数変更後波形が安定するまでの待機
 *          : 2 : 周波数サーチ終了 & ゼロ調開始
 *              :   周波数サーチ終了後にゼロ調を実施する
 *              :   ゼロ調リクエスト時にFrqSchSttFlgを0に戻す
 ***************************************************************************/
short FrqSchPrc(short pch){
	short i;
	short MinAmp = AD_MAX; //最小振幅一時変数
	short MaxAmp = 0; //最大振幅一時変数
	short FrqSchNowFlg = 0; //周波数サーチ中一時フラグ
	short AmpMesCnt = 0; //振幅測定回数
	long TmpAmpAvg = 0; //最大振幅平均値一時変数
	short ErrFlg = 0; //波形異常回数

	AmpMesCnt = sizeof(FrqSch[pch].MaxAmpLst) / sizeof(short);

	//周波数サーチ中でない
	if(FrqSch[pch].FrqSchSttFlg == 0){
		FrqSchNowFlg = 0; //流量更新する

		//一時変数初期化
		FrqSch[pch].MaxAmpAvg = 0;
		for(i = 0; i < AmpMesCnt; i++){
			FrqSch[pch].MaxAmpLst[i] = 0;
		}
		FrqSch[pch].NowFrq = SVD[pch].start_freq; //測定開始周波数に更新
		
		FrqSch[pch].MesAmpCnt = 0; //波形測定回数クリア
		FrqSch[pch].MesErrCnt = 0; //波形エラーカウントクリア
		FrqSch[pch].WitTim = 30; //待機時間初期化
		FrqSch[pch].CenPntUndLim = 10; //中心線通過回数下限値
		FrqSch[pch].CenPntOvrLim = 20; //中心線通過回数上限値
		FrqSch[pch].MaxAmpFrq = SVD[pch].SchFrq;
	}
	//周波数サーチ中
	else if(FrqSch[pch].FrqSchSttFlg == 1){
		FrqSchNowFlg = 1; //流量更新しない

		if(SVD[pch].drive_search == 0){
			//周波数サーチ使用しない設定の場合は終了
			FrqSch[pch].FrqSchSttFlg = 0;
			FrqSchNowFlg = 0;
		}
		//異常波形(オーバーフロー以外の流量エラー)
		// else if((MES[pch].err_status & (ERR_FLOW_CONT & ~(ERR_JUDGE_OVERFLOW))) != 0){
		else if((MES[pch].err_status & (ERR_BURN_OUT & ~(ERR_JUDGE_OVERFLOW))) != 0){
		// else if((MES[pch].err_status & (ERR_FLOW_CONT)) != 0){
			FrqSch[pch].MesErrCnt++; //波形エラーカウント
			FrqSch[pch].MaxAmpLst[FrqSch[pch].MesAmpCnt] = 0;
		}
		//5回異常波形検出で次
		else if(FrqSch[pch].MesErrCnt >= 5){
			FrqSch[pch].MesErrCnt = 0; //エラー検出回数クリア
			FrqSch[pch].NowFrq += 3; //3kHz増加させて次へ
			FrqSch[pch].MesAmpCnt = 0; //測定回数クリア
		}
		//正常波形
		else{
			FrqSch[pch].CenPntCnt = 0;
#if defined(FLWWAVEXP)
			for(i = 10; i < FLWWAVSIZ + 10; i++)
#else
			for(i = 10; i < 250; i++)
#endif
			{
				/*最小値を探す*/
				if( MES[pch].fow_data[i] < MinAmp){	
					/*fow data max*/
					MinAmp = MES[pch].fow_data[i];
				}
				/*最大値を探す*/
				if( MaxAmp < MES[pch].fow_data[i]){	
					/*fow data max*/
					MaxAmp = MES[pch].fow_data[i];
				}
				
				//波形が中心線(2047)を通過する回数をカウント(正常波形判定用)
				if(((MES[pch].fow_data[i - 1] - AD_BASE) * (MES[pch].fow_data[i] - AD_BASE)) < 0){
					FrqSch[pch].CenPntCnt++;
				}

			}
			//中心線通過回数が3~15が正常と定義
			if((FrqSch[pch].CenPntCnt < FrqSch[pch].CenPntUndLim) || (FrqSch[pch].CenPntOvrLim < FrqSch[pch].CenPntCnt)){
				FrqSch[pch].MesErrCnt++;
				ErrFlg++;
			}

			//最大振幅更新
			if(ErrFlg == 0){
				FrqSch[pch].MaxAmpLst[FrqSch[pch].MesAmpCnt] = MaxAmp - MinAmp;
			}
			else{
				FrqSch[pch].MaxAmpLst[FrqSch[pch].MesAmpCnt] = 0;
			}
		}
		FrqSch[pch].MesAmpCnt++;
		if(AmpMesCnt <= FrqSch[pch].MesAmpCnt){
				
			// 波形エラーが一度も起きていないときに周波数を更新する
			if(FrqSch[pch].MesErrCnt == 0){
				//この周波数での振幅最大値の平均値を計算
				for(i = 0; i < AmpMesCnt; i++){
					TmpAmpAvg += FrqSch[pch].MaxAmpLst[i];
					FrqSch[pch].MaxAmpLst[i] = 0; //配列リセット
				}
				TmpAmpAvg /= AmpMesCnt;

				//最大振幅更新判定
				if(TmpAmpAvg > FrqSch[pch].MaxAmpAvg){
					FrqSch[pch].MaxAmpAvg = (short)TmpAmpAvg;
					FrqSch[pch].MaxAmpFrq = FrqSch[pch].NowFrq;
				}
			}
			FrqSch[pch].MesAmpCnt = 0; //この周波数での測定は終了
			FrqSch[pch].NowFrq += 3; //3kHz増加させて次へ
			FrqSch[pch].MesErrCnt = 0; //波形エラーカウントクリア

			FrqSch[pch].FrqSchSttFlg = 10;
		}
	}
	//波形が安定するまで待ち 18ms/6ch * 10 = 180msくらい
	else if(FrqSch[pch].FrqSchSttFlg >= 10){
		FrqSch[pch].FrqSchSttFlg++;
		// if(FrqSch[pch].FrqSchSttFlg > 20){
		if(FrqSch[pch].FrqSchSttFlg > FrqSch[pch].WitTim){
			FrqSch[pch].FrqSchSttFlg = 1;
		}
	}
	//ゼロ調整中
	else{
		;
	}
	
	//周波数サーチ終了判定
	if((FrqSch[pch].FrqSchSttFlg == 1) && (FrqSch[pch].NowFrq > SVD[pch].stop_freq)){
		FrqSch[pch].FrqSchSttFlg = 2; //周波数サーチ終了 & ゼロ調整に移行

		//測定周波数更新
		SVD[pch].SchFrq = FrqSch[pch].MaxAmpFrq;
		//EEPROMに書き込む
		// eep_write_ch_delay(pch, (short)(&SVD[pch].SchFrq - &SVD[pch].max_flow), SVD[pch].SchFrq);
	}

	return FrqSchNowFlg;
}
#endif

/****************************************************/
/* Function : SetNextGain                     */
/* Summary  : 次CHのゲイン値を設定する         */
/* Argument : pch                            		 */
/* Return   : なし                                  */
/* Caution  : なし                                  */
/* note     : なし                                 */
/****************************************************/
void SetNextGain(short pch){

	short cnt;
	short next_pch;

	next_pch = pch + 1;  //次CH
	for(cnt=0; cnt<CH_NUMMAX; cnt++){
		if(next_pch >= CH_NUMMAX){	
			next_pch = CH1;
		}
		if(SVD[next_pch].sensor_size != SNS_NONE){  //センサがNONE設定ならば次CHにする
			break;
		}else{
			next_pch++;
		}
	}

	WriteGainAmp((unsigned short)MES[next_pch].amp_gain_rev);  /*次CHのゲイン値を設定する*/
}	

/****************************************************
 * Function : ChkClcActFlg
 * Summary  : 流量演算動作を定義する, SFC9100ではゼロクロス演算固定のため未使用
 * Argument : pch : チャンネル番号
 * Return   : void
 * Caution  : 動作変更の際はここも変更する
 * note     : 以下、メールの内容
 * ・裏パラ/FIFO offset の設定
 *　０ ： 既存動作
 *　1 ： 先頭位置をゼロ調節時から移動させない　 
 *　2 ： 差分相関点数を増加させる
 *　　　(10～210のデータを1つ飛ばしで100word分を差分演算する)
 *　3 ： 1 と 2 が有効
 *　4 ： ゼロクロス演算 　(ゼロクロス7点)
 *　5 ： ゼロクロス演算 と 1が有効　(ゼロクロス7点)
 *　6 ： ゼロクロス演算　(ゼロクロス12点)
 *　7 ： ゼロクロス演算 と 1が有効 (ゼロクロス12点)
 *　8 ： ゼロクロス演算 　(ゼロクロス7点、ゼロクロス前後2点)
 *　9 ： ゼロクロス演算 と 1が有効　(ゼロクロス7点、ゼロクロス前後2点)
 *　10 ： ゼロクロス演算　(ゼロクロス12点、ゼロクロス前後2点)
 *　11 ： ゼロクロス演算 と 1が有効 (ゼロクロス12点、ゼロクロス前後2点)
 ****************************************************/
void ChkClcActFlg(short pch) {
	switch (SVD[pch].wind_offset)
	{
	case 0:
		ClcActFlg = ClcAct_SumAdd; //差分相関
		// TopPosFix = PosFix_Mov; //先頭位置をゼロ調節時から移動させる
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	case 1:
		ClcActFlg = ClcAct_SumAdd; //差分相関
		// TopPosFix = PosFix_NotMov; //先頭位置をゼロ調節時から移動させない
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	case 2:
		ClcActFlg = ClcAct_SumAdd; //差分相関
		// TopPosFix = PosFix_Mov; //先頭位置をゼロ調節時から移動させる
		SumPntInc = SumPnt_Inc; //差分相関点数を増加させる
		break;
	case 3:
		ClcActFlg = ClcAct_SumAdd; //差分相関
		// TopPosFix = PosFix_NotMov; //先頭位置をゼロ調節時から移動させない
		SumPntInc = SumPnt_Inc; //差分相関点数を増加させる
		break;
	case 4:
	case 6:
		ClcActFlg = ClcAct_ZerCrs; //ゼロクロス
		// TopPosFix = PosFix_Mov; //先頭位置をゼロ調節時から移動させる
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	case 5:
	case 7:
		ClcActFlg = ClcAct_ZerCrs; //ゼロクロス
		// TopPosFix = PosFix_NotMov; //先頭位置をゼロ調節時から移動させない
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	case 8:
	case 10:
		ClcActFlg = ClcAct_ZerCrs; //ゼロクロス
		// TopPosFix = PosFix_Mov; //先頭位置をゼロ調節時から移動させる
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	case 9:
	case 11:
		ClcActFlg = ClcAct_ZerCrs; //ゼロクロス
		// TopPosFix = PosFix_NotMov; //先頭位置をゼロ調節時から移動させない
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	default:
		ClcActFlg = ClcAct_SumAdd; //差分相関
		// TopPosFix = PosFix_Mov; //先頭位置をゼロ調節時から移動させる
		SumPntInc = SumPnt_NotInc; //差分相関点数を増加させない
		break;
	}
}

/****************************************************/
/* Function : SearchWindow                        */
/* Summary  : Windowサーチ                   */
/* Argument : pch                              		 */
/* Return   : 0:正常, -1:異常	                  */
/* Caution  : なし                                  */
/* note     : Windowサーチで使用するゲイン値を取得してから
 *            最適なFIFO CHを検索する
 *          : 
 *          : MES[].ThresholdReq -> 0  : 波形認識実行要求なし
 *          :                    -> 1  : Windowサーチ用のゲイン値調整要求
 *          :                    -> 2  : Windowサーチ要求
 *          :                    -> 11 : 波形認識実行要求
 *          :                    -> 12 : 詳細サーチの場合
 *          :                    -> 99 : 波形認識処理の終了(LED.zeroactive=1の時、zero_adj_control()内で0に移行する)
 ****************************************************/
short SearchWindow(short pch){

	short cnt, ret;
	short fifo_ch;
	short fifo_start;
	short fifo_end;

	MES_SUB[pch].ws_work_gain = 255;
	MES_SUB[pch].ws_add_max = 0;
	ret = B_OK;	

	if(SVD[pch].adc_clock == 2){  //0=32MHz, 1=40MHz, 2=65MHz, 3=80MHz
		if(SVD[pch].sensor_size == 3){ // センサ口径:1/4"
			fifo_start = WS_FIFO_START_14;
		}else{ 		// センサ口径:3/8"
			fifo_start = WS_FIFO_START_38;
		}
		fifo_end = WS_FIFO_END;
	}else{  //0=32MHz, 1=40MHz, 3=80MHz(未実装)
		if(SVD[pch].sensor_size == 3){ // センサ口径:1/4"
			fifo_start = WS_FIFO_START_14_3240;
		}else{ 		// センサ口径:3/8"
			fifo_start = WS_FIFO_START_38_3240;
		}
		fifo_end = WS_FIFO_END_3240;
	}
	
	//Windowサーチ用のゲイン値調整
	MES[pch].ThresholdReq = 1;	/*Windowサーチ用のゲイン値調整要求*/
	SVD[pch].fix_data |= 0x0C;	//FIFO CHを10～30で固定するため
	SVD[pch].fix_fifo_no_read = 0;
	for(fifo_ch=fifo_start; fifo_ch<fifo_end; fifo_ch++){	//FIFO CH(15～30)のゲイン値を取得する
		SVD[pch].fix_fifo_ch_read = fifo_ch;
		//6ms x 40 = 240ms待機
		for(cnt=1; cnt<40; cnt++){
			//6553us=6ms待機
			// delay(0xffff);	//ゲイン値が安定するまでの待機（int_flow()でゲイン調整させる）
		}
		if(MES_SUB[pch].ws_work_gain > MES[pch].amp_gain_rev){	//最低ゲイン値の更新
			MES_SUB[pch].ws_work_gain = MES[pch].amp_gain_rev;
		}
	}
	MES[pch].amp_gain_fifo = MES_SUB[pch].ws_work_gain;	//最低ゲイン値をWindowサーチで使用する

	//Windowサーチ(最適なFIFO CHを探す)
	MES[pch].ws_FifoCh = SVD[pch].fifo_ch_init;	//現在のFIFO CH
	// MES[pch].ThresholdReq = 2;	/*Windowサーチ要求*/
	// for(fifo_ch=fifo_start; fifo_ch<fifo_end; fifo_ch++){	//FIFO CH(15～30)でサーチする
	// 	SVD[pch].fix_fifo_ch_read = fifo_ch;
	// 	for(cnt=1; cnt<5; cnt++){
	// 		delay(0xffff);	//int_flow(),DriveFIFORev()で受波波形を解析させるための待機
	// 	}
	// 	if(MES_SUB[pch].ws_add_max < MES[pch].ws_work_add){	//加算値の最大値を更新
	// 		MES_SUB[pch].ws_add_max = MES[pch].ws_work_add;
	// 		MES[pch].ws_FifoCh = fifo_ch;		//加算値が最大値のFIFO CHを最適値とする
	// 	}
	// }
	SVD[pch].fifo_ch_init = SVD[pch].fix_fifo_ch_read = MES[pch].ws_FifoCh;		//検索した最適なFIFO CH
	SVD[pch].fix_data &= ~0x0C;		//FIFO CH固定の解除

	// if(MES_SUB[pch].ws_add_max == 0){	//加算値が更新されていない場合は、最適なFIFO CHがないと判断する(波形異常、センサが外れている、FIFOデータが受け取れない)
	// 	ret = B_NG;		//異常
	// }

	return (ret);
}

/****************************************************/
/* Function : SelectForwardOn						*/
/* Summary  : In/Out打込みを切り替える(出力有効)			*/
/* Argument : pch									*/
/* Return   : なし									*/
/* Caution  : なし									*/
/* note     : 自動IN/OUT切り替え機能					*/
/****************************************************/
void SelectForwardOn(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesfwdW(0, pch);	//上流側GPIOポート出力
	}else{
		portmesrevW(0, pch);	//下流側GPIOポート出力
	}
}

/****************************************************/
/* Function : SelectReverseOn						*/
/* Summary  : In/Out打込みを切り替える(出力有効)			*/
/* Argument : pch									*/
/* Return   : なし									*/
/* Caution  : なし									*/
/* note     : 自動IN/OUT切り替え機能					*/
/****************************************************/
void SelectReverseOn(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesrevW(0, pch);	//下流側GPIOポート出力
	}else{
		portmesfwdW(0, pch);	//上流側GPIOポート出力
	}
}

/****************************************************/
/* Function : SelectForwardOff						*/
/* Summary  : In/Out打込みを切り替える(出力無効)			*/
/* Argument : pch									*/
/* Return   : なし									*/
/* Caution  : なし									*/
/* note     : 自動IN/OUT切り替え機能					*/
/****************************************************/
void SelectForwardOff(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesfwdW(1, pch);	//上流側GPIOポート出力
	}else{
		portmesrevW(1, pch);	//下流側GPIOポート出力
	}
}

/****************************************************/
/* Function : SelectReverseOff						*/
/* Summary  : In/Out打込みを切り替える(出力無効)			*/
/* Argument : pch									*/
/* Return   : なし									*/
/* Caution  : なし									*/
/* note     : 自動IN/OUT切り替え機能					*/
/****************************************************/
void SelectReverseOff(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesrevW(1, pch);	//下流側GPIOポート出力
	}else{
		portmesfwdW(1, pch);	//上流側GPIOポート出力
	}
}

short iNum = 0;
short DataNum = 0;
/****************************************************/
/* Function : int_flow								*/
/* Summary  : 流量計測処理							*/
/* Argument : pch									*/
/* Return   : なし									*/
/* Caution  : なし									*/
/* note     : 定周期(3msec)割込み処理					*/
/****************************************************/
void int_flow(short pch)
{

	if (pch >= CH_NUMMAX)
		return;

	/*センサ無し設定時*/
	if (SVD[pch].sensor_size == SNS_NONE)
	{
		non_sensor_control(pch); // センサ無し設定時の処理
		addit_flow_calc();		 // 積算流量処理
		return;
	}

	
	//評価用
	if(SVD[pch].sum_step == 2){	//打込み回数上流下流各2回
		MES_SUB[pch].sample_cnt = 2;
	}else if(SVD[pch].sum_step == 3){	//打込み回数上流下流各3回
		MES_SUB[pch].sample_cnt = 3;
	}else if(SVD[pch].sum_step == 4){	//打込み回数上流下流各4回
		MES_SUB[pch].sample_cnt = 4;
	}else{	//打込み回数上流下流各4回
		MES_SUB[pch].sample_cnt = 4;
	}
	AD_BASE = AD_BASE_UNIT * MES_SUB[pch].sample_cnt;
	AD_MAX = AD_MAX_UNIT * MES_SUB[pch].sample_cnt;

	if(20<=SVD[pch].sum_end && SVD[pch].sum_end<=50){
		// MES_SUB[pch].ItvVal = 1000 + (SVD[pch].sum_end - 20)*100;
		MES_SUB[pch].ItvVal = 1000 + (SVD[pch].sum_end - 20)*100 - 500; //特殊仕様(波形取得後の波形加算時間約50usを考慮)
	}

	if((FpgaVersion == 0x2211) || (FpgaVersion == 0x3211))
	{	//FPGAオーバーサンプリング4点バージョン
		Mgn = 4;	//ADC65MHzを1/4して16.25MHzサンプリングとする
	}else{
		Mgn = 8;	//ADC65MHzを1/8して8.125MHzサンプリングとする
	}
	//評価用
	
	
	/*検査モード*/
	if (MES[pch].inspect_enable != 0)
	{ /*検査モード有効時*/
		inspect_flow_mode(pch);
		return;
	}
__bit_output(GPIO_PORTQ_BASE, 5, 1);
#if defined(DatInc)
	SmpTs[SVD[pch].adc_clock] = SmpTs0[SVD[pch].adc_clock] * Mgn;
#else
	SmpTs[SVD[pch].adc_clock] = SmpTs0[SVD[pch].adc_clock];
#endif

	/*受波データ読込み*/
	fifo_read(pch);

	/*アンプゲイン調整*/
	gain_adj_control(pch);

	/*アンプゲインチェック（予知保全機能）*/
	check_amp_gain(pch);

	//差分相関演算
	if (ClcActFlg == ClcAct_SumAdd)
	{
		/*絶対値の差を演算*/
		sum_adder(pch);

		/*最小値、最大値の検索*/
		if (min_max_search(pch) == B_OK)
		{
			/*時間差の演算*/
			delta_ts_cal(pch);
		}
	}
	//ゼロクロス演算
	else if (ClcActFlg == ClcAct_ZerCrs)
	{
		//ゼロクロス点を探索
		SchZerPnt(pch);
	}
	else 
	{
		; //その他は演算停止
	}

	/*ピーク検出*/
	MES[pch].ThreasholdPoint = temp_v(pch);
	sonic_search_max(pch);

#ifdef FRQSCH
	if (FrqSchPrc(pch) != 0)
	{
		return; // 周波数サーチ中
	}
#endif

	/*音速フィルタ処理*/
	sonic_filter_control(pch);

	if (ClcActFlg == ClcAct_SumAdd)
	{
		/*移動平均*/
		filter_moving(pch);

		/*ゼロ調整補正*/
		zero_adj(pch);

		/*演算異常判定*/
		correlate_check(pch);
	}
	else if (ClcActFlg == ClcAct_ZerCrs)
	{
		/* 時間差フィルタ処理 */
		zc_filter_moving(pch);
	}

	/*流量換算 （⊿tを流量値に換算）、メーカリニアライズ*/
	MES[pch].ml_min_a = flow_calc(pch); /*ml_min_a (0.1mL/minを1で表す)*/

	/*Kファクタ補正*/
	MES[pch].ml_min_b = pv_calc(MES[pch].ml_min_a, pch); /*ml_min_b (0.01mL/minを1で表す)*/

	/*ユーザリニアライズ*/
	MES[pch].ml_min_c = user_linear(MES[pch].ml_min_b, pch); /*ml_min_c (0.01mL/minを1で表す)*/

	/*受波の差(p1-P2)*/
	max_point_control(pch);

	/*流量フィルタ処理*/
	MES[pch].ml_min_d = ClcFlwFlt(MES[pch].ml_min_c, pch);

	/*逆流異常判定*/
	reverse_flow_check(pch);

	/*瞬時流量更新*/
	current_flow_control(pch); /*ml_min_nowに瞬時流量を更新する (0.01mL/minを1で表す)*/

	/*テスト出力モード*/
	test_flow_mode(pch);

	/*ダンピング処理*/
	damp_control(pch);

	/*ローカット処理*/
	lowcut_control(pch);

	/*積算流量処理*/
	addit_flow_calc();

	/*バーンアウト処理*/
	burnout_control(pch);

	/*アンプゲイン処理(次CH用)*/
	SetNextGain(pch);
__bit_output(GPIO_PORTQ_BASE, 5, 0);
	MES[pch].ml_min_oq = work_oq;				/*oqコマンド用*/
	MES[pch].ml_min_OQ = MES[pch].ml_min_now; /*OQコマンド用*/
}
