/***********************************************/
/* File Name : int_flow.c						*/
/*	Summary   : ���ʌv������						*/
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
/*	���W���[������`�֐�								*/
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
/*	���W���[���O��`�֐�								*/
/********************************************************/
extern void	action_status_control(short pch, short act);
extern void delay(unsigned short delay_count);
extern void	non_sensor_control(short pch);
extern unsigned long long get_total_offset(short ch, unsigned long long val_total);
extern float RoundFunc(float src);

/********************************************************/
/*	���W���[������`�ϐ�								*/
/********************************************************/
short	addit_sts = B_OFF;
short	rev_max_point,fow_max_point; //�g�`�U���̍ő�l���Ƃ�x���W
short	fow_max,fow_min; //�g�`�̐U���ő�l
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

short ClcActFlg = 1; //�������� / �[���N���X
#define ClcAct_SumAdd 0
#define ClcAct_ZerCrs 1
short TopPosFix = 0; //�擪�ʒu���[�����ߎ�����ړ������� / �����Ȃ�		���g�`�F���������l�@�\�Ɣ��̂Łu0�v�Œ�
#define PosFix_Mov 0
#define PosFix_NotMov 1
short SumPntInc = 0; //�������֓_���𑝉������Ȃ� / ������
#define SumPnt_NotInc 0
#define SumPnt_Inc 1

#define DatInc //�I�[�o�[�T���v�����O
//#define ShtCntTwo //�ł����݉�2��ɕύX
#define ShtItv //�ł����݌�C���^�[�o�� (+200 = +20us)
//const short ItvVal = 1000; //200us
// const short ItvVal = 1250; //230us
//const short ItvVal = 1450; //250us
const short ItvVal = 2000; //300us
short Mgn = 8;

#define NEW_MEASURE_METHOD
#define METHOD_OLD  //�]���̃[���N���X���Z���@
// #define METHOD_CLFC1 //CLFC300�̉��Z���@1
// #define METHOD_CLFC2 //CLFC300�̉��Z���@2

/********************************************************/
/*	���W���[���O��`�ϐ�								*/
/********************************************************/
extern short	com_type;
extern short initializing;
extern short FpgaVersion;

/********************************************************/
/*	�Z���T���											*/
/* 0=None, 1=1/8", 2=, 3=1/4", 4=3/8" */
/********************************************************/
//�T���v�����O����
short SmpTs0[] = { 3125, 2500, 1538, 1250 }; //1/32MHz, 1/40MHz, 1/65MHz, 1/80MHz [ns:100�{�f�[�^]
short SmpTs[] = { 0, 0, 0, 0 };
short AdcSmpFrq[] = {32, 40, 65, 80};

typedef struct {
	short	area;					// �f�ʐ� (mm2 x 100)
	short	sns_disL_l;		// �Z���T����L-l�i*0.1mm�j�������Z
	short	sns_disL; 			// �Z���T����L�i*0.1mm)��������p
	short	sns_TAU;			// ��s(*ns)��������p
	short	k_scale;			// K�t�@�N�^�݊��W��
	short	Ts;					// �T���v�����O���� (10ps)
	short wave_vth;			// ��g���o臒l
	short balance_level;		// �g�`�A���o�����X���o臒l
	short saturation_level;	// �g�`�O�a���o臒l
	short correlate_level;	// �������֔�臒l
	short correlate_time;		// ���Z�ُ픻���
	short attenuate_level;	// �g�`����臒l
	short fifo_ch;			// FIFO CH �����l
} SENSOR_INFO;

const SENSOR_INFO sens_inf[] = {
	{// None
		-1,			// �f�ʐ� (mm2 x 100)
		-1,			// �Z���T����L-l�i*0.1mm�j�������Z
		-1,			// �Z���T����L�i*0.1mm)��������p
		-1,			// ��s(*ns)��������p
		-1,			// K�t�@�N�^�݊��W��
		-1,			// �T���v�����O���� (10ps)
		30,			// ��g���o臒l
		15,			// �g�`�A���o�����X���o臒l
		94,			// �g�`�O�a���o臒l
		80,			// �������֔�臒l
		100,		// ���Z�ُ픻���
		10,			// �g�`����臒l
		5			// FIFO CH �����l
	},
	{// 1/8"
		370,		// �f�ʐ� (mm2 x 100)  (�Ǔ��a2.17mm)
		600,			// �Z���T����L-l�i*0.1mm�j�������Z
		600,		// �Z���T����L�i*0.1mm)��������p
		6000,		// ��s(*ns)��������p
		10000,		// K�t�@�N�^�݊��W��
		3200,		// �T���v�����O���� (10ps)
		30,			// ��g���o臒l
		15,			// �g�`�A���o�����X���o臒l
		94,			// �g�`�O�a���o臒l
		80,			// �������֔�臒l
		100,		// ���Z�ُ픻���
		10,			// �g�`����臒l
		5			// FIFO CH �����l
	},
	{// 4x3
		707,		// �f�ʐ� (mm2 x 100)  (�Ǔ��a3.00mm)
		600,		// �Z���T����L-l�i*0.1mm�j�������Z
		600,		// �Z���T����L�i*0.1mm)��������p
		6000,		// ��s(*ns)��������p
		10000,		// K�t�@�N�^�݊��W��
		3200,		// �T���v�����O���� (10ps)
		30,			// ��g���o臒l
		15,			// �g�`�A���o�����X���o臒l
		94,			// �g�`�O�a���o臒l
		80,			// �������֔�臒l
		100,		// ���Z�ُ픻���
		10,			// �g�`����臒l
		5			// FIFO CH �����l
	},
	{// 1/4"
		1486,		// �f�ʐ� (mm2 x 100)  (�Ǔ��a4.35mm)
		600,		// �Z���T����L-l�i*0.1mm�j�������Z
		600,		// �Z���T����L�i*0.1mm)��������p
		6000,		// ��s(*ns)��������p
		10000,		// K�t�@�N�^�݊��W��
		3200,		// �T���v�����O���� (10ps)
		30,			// ��g���o臒l
		15,			// �g�`�A���o�����X���o臒l
		94,			// �g�`�O�a���o臒l
		80,			// �������֔�臒l
		100,		// ���Z�ُ픻���
		10,			// �g�`����臒l
		5			// FIFO CH �����l
	},
	{// 3/8"
		4453,		// �f�ʐ� (mm2 x 100)  (�Ǔ��a7.53mm)
		600,			// �Z���T����L-l�i*0.1mm�j�������Z
		600,		// �Z���T����L�i*0.1mm)��������p
		6000,		// ��s(*ns)��������p
		10000,		// K�t�@�N�^�݊��W��
		3200,		// �T���v�����O���� (10ps)
		30,			// ��g���o臒l
		15,			// �g�`�A���o�����X���o臒l
		94,			// �g�`�O�a���o臒l
		80,			// �������֔�臒l
		100,		// ���Z�ُ픻���
		10,			// �g�`����臒l
		5			// FIFO CH �����l
	}
};

/********************************************************/
/*	�������֏��										*/
/********************************************************/
//const	short abs_cont[5]={40,40,40,40,40};			/*���֒l�� SUB_POINT����*/
const	short abs_cont[5]={20,20,20,20,20};			/*���֒l�� SUB_POINT����*/

/********************************************************/
/*	FIFO���											*/
/********************************************************/
#define FIFO   (*(volatile short *)0x60000000) 	/* FIFO Address*/ 
unsigned char FIFO_CH_SRT = 63;		//FIFO CH:�J�n
unsigned char FIFO_CH_END = 47;		//FIFO CH:�I��

/********************************************************/
/*	�����ݒ�e�[�u��									*/
/********************************************************/
const 	long flow_vel_tbl[21]={
			66429,48000,34684,25061,18109,13085,9455,6832,4936,3567,
			2577,1862,1346,972,703,508,367,265,192,138,100
};		/* ����[cm/sec * 100]*/

typedef struct {
	short viscos;	/*���S�x[mm2/sec * 100]*/
	short dat[21];
	} KV_TBL;

/****************************************************/
/*	���S�x�W���e�[�u�� PFA 1/8"					*/
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
/*	���S�x�W���e�[�u�� PFA 4x3					*/
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
/*	���S�x�W���e�[�u�� PFA 1/4"				*/
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
/*	���S�x�W���e�[�u�� PFA 3/8"					*/
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
 *			��t���j�A���C�Y���[�h�e�[�u��
 * ���荀��
 * 00 : ���S�x
 * 01 : �Z���T���
 * 02 : �t���X�P�[��(�Z���T4��ނɑΉ�)
 * 03 : ���j�A���C�Y�_��
 * 04-07 : FIFO CH (�Z���T4��ނɑΉ�)
 * 08-11 : �[���N���X�_�����J�n�ʒu(�g�`�̋ɑ�l�ƂȂ�悤�Ɏ���, �Z���T4��ނɑΉ�)
 * 12-26 : ���j�A���C�Y�ݒ�l�imL/min 10�{�f�[�^�j
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
/* Summary  : �N���b�N�p���X                            */
/* Argument : �Ȃ�                               		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
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
/* Summary  : FIFO�`���l�����o��             */
/* Argument : pch                              		 */
/* Return   : �Ȃ�									 */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�            */
/****************************************************/
short	ch_search(short pch){

 short fifo;

	fifo = SVD[pch].fifo_ch_init;
	if(fifo < 0 || fifo > 47){		//���~�b�g�`�F�b�N
		fifo = 5;
	}

	return fifo;
}

/****************************************************/
/* Function : dma_dummy				*/
/* Summary  : DMA �J�n�i��ǂ݁j            */
/* Argument : count,  pch                          */
/* Return   : ��                                   */
/* Caution  : �Ȃ�                                  */
/* note     : FIFO����w�胏�[�h��ǂ݂���            */
/****************************************************/
void dma_dummy( short count, short pch){
 
	unsigned short dumy_buffer[1];

	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_8);	// �]����̃A�h���X���C���N�������g���Ȃ� 
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, dumy_buffer, count);
	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);	// DMA �]���J�n 
	while(uDMAChannelModeGet(UDMA_CHANNEL_SW) != UDMA_MODE_STOP);	//�]���҂�
	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_8);	// �ʏ�ݒ�ɖ߂� 

} 

/****************************************************/
/* Function : dma_start                           */
/* Summary  : DMA �J�n                            */
/* Argument : *read_add                         		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
/* note     : FIFO����200���[�h�]������                 */
/****************************************************/
void	dma_start(short *read_add){

#if defined(FLWWAVEXP)
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, read_add, FLWWAVSIZ);	// DMA �]���J�n 
#else
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, read_add, 200);	// DMA �]���J�n
#endif
	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);	// DMA �]���J�n 
	while(uDMAChannelModeGet(UDMA_CHANNEL_SW) != UDMA_MODE_STOP);	//�]���҂�

}
void	us_dma_start(short *read_add){
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, (void*)&FIFO, read_add, 240);	// DMA �]���J�n

	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);	// DMA �]���J�n 
	while(uDMAChannelModeGet(UDMA_CHANNEL_SW) != UDMA_MODE_STOP);	//�]���҂�
}

/****************************************************/
/* Function : int15_handler                     */
/* Summary  : FIFO�Ƀf�[�^������n�߂�܂ő҂�          */
/* Argument : �Ȃ�                            		 */
/* Return   : �Ȃ�								*/
/* Caution  : �Ȃ�                                  */
/* note     : tm4c1290nczad_startup_ccs.c          */
/*            �����g�p��								*/
/*            FPGA����ύX����IRQ4��EMPTY���o�Ƃ���		*/
/*            FIFO�f�[�^����(FPGA��FIFO�f�[�^��������ł������)�͑ҋ@����*/
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
/* Summary  : FIFO�f�[�^����(FPGA��FIFO�f�[�^��������ł������)�͑ҋ@���� */
/* Argument : short pch                    		 */
/* Return   : 0:����@�@-1�F�ُ�                      */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�                                 */
/****************************************************/
short fifo_read_wait(short pch) {

	short ret, cnt;

	if(MES_SUB[pch].fifo_result == B_NG)	return (B_NG);

	cnt = 0;
	ret = B_OK;
	while(1){
		if(__bit_input(GPIO_PORTP_BASE, 6) == 0){	//FIFO�f�[�^����(FPGA��FIFO�f�[�^��������ł������)�͑ҋ@����
			break;	//����
		}

		cnt++;
		if(cnt>1000){	//�Z���T���O��Ă���ꍇ���l��
			ret = MES_SUB[pch].fifo_result = B_NG;
			break;	//�ُ�
		}
	}

	return (ret);	
}

/****************************************************/
/* Function : fifo_search                     */
/* Summary  : FIFO��ǂݐM���̈ʒu�����߂�         */
/* Argument : pch, search_ch                		 */
/* Return   : 0������i�M������j, -1�ُ�(�M���Ȃ�)       */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�                                 */
/****************************************************/
short fifo_search(short pch, short search_ch){

	short i,cnt,work;
	short amp;
	short ret;
	short non_signal_count_sv;
	short	atn_gain;
	unsigned short empty_level; 
	short work_old[2]={AD_BASE_UNIT, AD_BASE_UNIT};
 
	if(MES[pch].ThresholdReq == 2){	/*Window�T�[�`�v������(�œK��FIFO CH��T��)*/
		WriteGainAmp((unsigned short)MES[pch].amp_gain_fifo);  /*�Q�C���l�Œ�*/
	}else{
		if((SVD[pch].fix_data & 0x02) != 0){  //�Œ�l�ݒ�
			if(SVD[pch].fix_amp_gain_rev == 0){
				WriteGainAmp((unsigned short)MES[pch].zero_amp_gain_rev);  /*�[���_�������̃Q�C���l*/
			}else{
				WriteGainAmp((unsigned short)SVD[pch].fix_amp_gain_rev);  /*�Œ�ݒ莞�̃Q�C���l*/
			}
		}else{
			WriteGainAmp((unsigned short)MES[pch].amp_gain_rev);  /*�Q�C��������*/
		}
	}

	OutputRestartPulse();			/*RESTART�p���X�o��*/
	/*REV*/	
	SelectForwardOn(pch);		//IN/OUT�ō��ݐؑւ�

	OutputRestartPulse();		/*RESTART�p���X�o��*/

	if(MES[pch].ThresholdReq == 0	/*�g�`�F�����s�v���Ȃ�*/
		|| MES[pch].ThresholdReq == 1){ 	/*Window�T�[�`�p�̃Q�C���l����*/
		MES[pch].fifo_ch = search_ch;
	}else if(MES[pch].ThresholdReq == 2){ /*Window�T�[�`�v������(�œK��FIFO CH��T��)*/
		MES[pch].fifo_ch = SVD[pch].fix_fifo_ch_read;
	}else{
		MES[pch].fifo_ch = MES[pch].ws_FifoCh;
	}
	
	MES[pch].fifo_start = MES[pch].fifo_ch;	/*WINDOW�J�n���Ԃ�ݒ�*/
	MES[pch].fifo_end = MES[pch].fifo_ch + 1;	/*WINDOW�I�����Ԃ�ݒ�*/
	MES[pch].fifo_offset = SVD[pch].wind_offset; /*WINDOW�I�t�Z�b�g���Ԃ�ݒ�*/
		
	if(initializing == 0){   //�N�����̏������������I�����Ă���ꍇ
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}

	if(MES[pch].ThresholdReq == 2){	/*Window�T�[�`�v��*/
		MES[pch].clk_phase = 0; // �ʑ�������
	}
	SetFpgaRegister(pch); // FPGA���W�X�^�̐ݒ�
	OutputStartPulse();		/*START�p���X�o��*/

	/*FIFO�Ǎ���(�_�~�[���[�h)*/
	MES_SUB[pch].fifo_result = 0;		//FIFO�Ǎ��݃X�e�[�^�X�̃N���A
	if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
		work = FIFO;  /*�ŏ���2word = 0 �ɐ����Ă�*/
	}
	if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
		work = FIFO;  /*2��A�_�~�[���[�h����B   */
	}

	/*��g�g�`���o���x��(Empty�G���[)*/
	empty_level = (unsigned short)((long)AD_MAX_UNIT * (long)SVD[pch].wave_vth / 100);	/*�ő�l(12bit MAX)*����臒l*/
	
	/*���ʌv����(�ʏ펞)*/
	if(MES[pch].ThresholdReq == 0	/*�g�`�F�����s�v���Ȃ�*/
		|| MES[pch].ThresholdReq == 1	/*Window�T�[�`�p�̃Q�C���l����*/
		|| MES[pch].ThresholdReq == 2){	/*Window�T�[�`�v������(�œK��FIFO CH��T��)*/

		// �������̂��߃��[�v 1 ��� 4 �o�C�g�����[�h 
		for(i=2; i<(4000-200); i+= 4){	  /*3800word*/
			if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
				work = FIFO;	//FIFO�f�[�^�Ǎ���
				// work &= 0x0FFF;
				work &= 0x1FFF;
				if(work < empty_level){		/*��g���o臒l�`�F�b�N(Empty�G���[)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
						MES[pch].signal_count = i;
					}
					break;		//��g�g�`�����o
				}
			}else{
				i = 3800;
				break;
			}

			if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
				work = FIFO;
				work &= 0x1FFF;
				if(work < empty_level){		/*��g���o臒l�`�F�b�N(Empty�G���[)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
						MES[pch].signal_count = i + 1;
					}
					break;		//��g�g�`�����o
				}
			}else{
				i = 3800;
				break;
			}

			if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
				work = FIFO;
				work &= 0x1FFF;
				if(work < empty_level){		/*��g���o臒l�`�F�b�N(Empty�G���[)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
						MES[pch].signal_count = i + 2;
					}
					break;		//��g�g�`�����o
				}
			}else{
				i = 3800;
				break;
			}

			if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
				work = FIFO;
				work &= 0x1FFF;
				if(work < empty_level){		/*��g���o臒l�`�F�b�N(Empty�G���[)*/
					if(MES[pch].ThresholdPeakPos == 0){  /*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
						MES[pch].signal_count = i + 3;
					}
					break;		//��g�g�`�����o
				}
			}else{
				i = 3800;
				break;
			}
		}
	/*�[���_�������̔g�`�F������*/
	}else{	/*�g�`�F�����s�v������*/
		cnt = 0;
		for(i=2; i<(4000-200); i+= 1){	  /*3800word*/
			if(fifo_read_wait(pch) == B_OK){	/*��g�f�[�^�Ǎ��ݑҋ@*/
				work = FIFO;
				work &= 0x1FFF;
				//��g���o臒l�`�F�b�N(�g�`�F��)
				if(work_old[1] <= MES[pch].ThresholdWave && MES[pch].ThresholdWave <= work){		/*�����������Ɍ���*/
					cnt++;
				}
				if(work_old[1] >= MES[pch].ThresholdWave && MES[pch].ThresholdWave >= work){		/*�ォ�牺�����Ɍ���*/
					cnt++;
				}
				if(MES[pch].ThresholdReq == 12 && cnt == 0 && work <= work_old[1] && MES[pch].ThresholdPeak <= work){  /*�s�[�N���o*/
					MES[pch].ThresholdPeak = work_old[1];	/*1�O����g�g�`�s�[�N*/
					MES[pch].signal_count = MES[pch].ThresholdPeakPos = i - 1;	 /*1�O����g�g�`�s�[�N�ʒu*/
				}
				work_old[1] = work;

				if(cnt >= 5){  /*��g�g�`�Ǝ�g���o臒l��5��ȏ���������ꍇ*/
					if(MES[pch].ThresholdReq == 11){  /*�����T�[�`�̏ꍇ*/
						MES[pch].ThresholdWave += 10;
						MES[pch].ThresholdReq = 12;  /*�g�`�F��臒l+10�߂��ďڍ׃T�[�`�Ɉڍs����*/
					}else if(MES[pch].ThresholdReq == 12){  /*�ڍ׃T�[�`�̏ꍇ*/
						MES[pch].ThresholdReq = 99;  /*�g�`�F�������̏I��*/
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
			MES[pch].ThresholdWave -= 3; /*�g�`�F��臒l��3�������Č�������(�����T�[�`)*/
		}else if(MES[pch].ThresholdReq == 12){
			MES[pch].ThresholdWave -= 1;  /*�g�`�F��臒l��1�������Č�������(�ڍ׃T�[�`)*/
		}else{
			;
		}
	}

	CheckEndPulse();		/*END�M���m�F*/
	SelectForwardOff(pch);		//IN/OUT�ō��ݐؑւ�

	if((MES[pch].ThresholdReq == 0)	/*�g�`�F�����s�v���Ȃ�*/
		&& (i == 3800)){			//3800word�Ǎ���ł���g�g�`��������Ȃ��ꍇ(Empty�G���[)
		non_signal_count_sv = (short)((float)SVD[pch].inc / 100 / 0.018);
		/*��g�Ȃ����J�E���g*/
		if(MES[pch].non_signal_count < non_signal_count_sv){  /*100��J�E���g*/
			MES[pch].non_signal_count++;
			MES[pch].amp_gain_for++;
			if(MES[pch].amp_gain_for > 255) MES[pch].amp_gain_for = 255;
			MES[pch].amp_gain_rev++;
			if(MES[pch].amp_gain_rev > 255) MES[pch].amp_gain_rev = 255;
		}else{
			MES[pch].err_status |= ERR_JUDGE_EMPTY;	/*�G���[�Z�b�g*/
			MES[pch].amp_gain_for = 255;
			MES[pch].amp_gain_rev = 255;
		}
		amp = MES[pch].amp_gain_for = MES[pch].amp_gain_rev;
		MES[pch].err_status |= ERR_JUDGE_PRE_EMPTY;	/*�G���[�Z�b�g*/
		ret = (short)B_NG;									/*�M���Ȃ�*/
	}else{			//��g�g�`���������ꍇ
		MES[pch].non_signal_count = 0;
		MES[pch].err_status &= ~(ERR_JUDGE_EMPTY+ERR_JUDGE_PRE_EMPTY);	/*�G���[���Z�b�g*/
		ret = (short)B_OK;									/*�M������*/
	}

	return (ret);
}

/****************************************************/
/* Function : get_vth2                     */
/* Summary  : Vth�̎擾        */
/* Argument : data[], vth                        		 */
/* Return   : Vth       */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�                                 */
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
 * Summary  : �g�`���擾����
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : �ꎞ�ϐ� : fwd_temp_data / rev_temp_data
 *            ���ϔg�` : MES.fow_data / MES.rev_data
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
        delay(MES_SUB[pch].ItvVal); /*�ł����݌�C���^�[�o��*/
// __bit_output(GPIO_PORTG_BASE, 6, 1);
		/*�㗬��FIFO����*/
		DriveFIFOFwd(pch, sample);	//�p���X�ł����݁AFIFO�Ǎ���
		for (i = 12; i < 250; i++)
		{
			if(sample == 0){
				*(FowWav + i) = fwd_temp_data[sample][i];
			}
			else{
				*(FowWav + i) += fwd_temp_data[sample][i];
			}
		}

        delay(MES_SUB[pch].ItvVal); /*�ł����݌�C���^�[�o��*/
// __bit_output(GPIO_PORTG_BASE, 6, 0);
		/*������FIFO����*/
		DriveFIFORev(pch, sample);	//�p���X�ł����݁AFIFO�Ǎ���
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
 * Summary  : FIFO�̈ʒu�����X�V����
 * Argument : pch : �`�����l���ԍ�
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
		// �Œ�l�ݒ�
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
	MES[pch].fifo_ch_read = MES[pch].fifo_ch;	//FIFO CH�T�[�`�Ō��߂�CH

	while (MES[pch].fifo_no_read > (256 + 128))
	{
		MES[pch].fifo_no_read -= 256;
//		MES[pch].fifo_ch_read += 1;
	}
}

/*******************************************
 * Function : SetFifoPos (Set FIFO Position)
 * Summary  : FIFO�J�n�ʒu�A�I���ʒu�����肷��
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void SetFifoPos(short pch)
{
	if ((SVD[pch].fix_data & 0x04) != 0)
	{
		// �Œ�l�ݒ�
		if (SVD[pch].fix_fifo_ch_read == 0)
		{
			MES[pch].fifo_start = MES[pch].zero_fifo_ch_read; /*�[���_��������FIFO�ʒu*/
			MES[pch].fifo_end = MES[pch].zero_fifo_ch_read + WS_FIFO_RANGE;
		}
		else
		{
			MES[pch].fifo_start = SVD[pch].fix_fifo_ch_read; /*�Œ�ݒ莞��FIFO�ʒu*/
			MES[pch].fifo_end = SVD[pch].fix_fifo_ch_read + WS_FIFO_RANGE;
		}
	}
	else
	{
		MES[pch].fifo_start = MES[pch].fifo_ch_read;	/*WINDOW�J�n���Ԃ�ݒ�*/
		MES[pch].fifo_end = MES[pch].fifo_ch_read + WS_FIFO_RANGE; /*WINDOW�I�����Ԃ�ݒ�*/
	}
}

/*******************************************
 * Function : GetSkpPnt (Get Skip Point)
 * Summary  : �X�L�b�v����_�������肷��
 * Argument : Div : 1������1/Div�̒l���擾����
 * Return   : SkpPnt : �X�L�b�v����_��
 * Caution  : None
 * Note     : 1�����̓_�� = ADC�̎��g��[MHz] / �g�`�̎��g��[kHz] / �I�[�o�[�T���v�����O��
 * *****************************************/
short GetSkpPnt(short pch, short Div)
{
	short SkpPnt;
	
	//FPGA�I�[�o�[�T���v�����O4�_�o�[�W����
	if((FpgaVersion == 0x2211) || (FpgaVersion == 0x3211))
	{
		SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 4 / Div;
	}
	//FPGA�I�[�o�[�T���v�����O8�_�o�[�W����
	else
	{
		SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / Div;
	}

	return SkpPnt;
}

/*******************************************
 * Function : SchZerDat (Search Zero cross Data)
 * Summary  : �[���N���X�_����������
 * Argument : pch : �`�����l���ԍ�
 *          : *WavPtr : �����Ώۂ̔g�`�f�[�^�z��
 *          : *ZerPnt : ��������(�[���N���X�_)������f�[�^�z��
 *          : *ZerDat1 : ��������(�[���N���X�f�[�^)������f�[�^�z��
 *          : *ZerDat2 : ��������(�[���N���X�f�[�^)������f�[�^�z��
 *          : *ZerDat3 : ��������(�[���N���X�f�[�^)������f�[�^�z��
 *          : *ZerDat4 : ��������(�[���N���X�f�[�^)������f�[�^�z��
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
		//�������Ă���΃��[�v�Z�k�̂��߂�i���΂�
		if(FndFlg == 1)
		{
			//��΂��_�� = ��g�g��(f=600kHz) * �T���v�����O���g��(�I�[�o�[�T���v�����O8�_) /(1/4�g��)
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
 * Summary  : �X���b�V���z�[���h�l����������
 * Argument : pch : �`�����l���ԍ�
 *          : *WavPtr : �����Ώۂ̔g�`�f�[�^�z��
 *          : *OutVal : �������ʁi�X���b�V���z�[���h�l�j
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
 * Summary  : �ɑ�l����������
 * Argument : pch : �`�����l���ԍ�
 *          : *WavPtr : �����Ώۂ̔g�`�f�[�^�z��
 *          : *OutVal : ��������(�ɑ�l)
 *          : *OutPos : ��������(�ɑ�ʒu)
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

	//��΂��_�� = ��g�g��(f=600kHz) * �T���v�����O���g��(�I�[�o�[�T���v�����O8�_) = 1/600kHz * 65MHz/8
	SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / 2; //Skip Point (�f�t�H���g��13.54)
	SkpPnt = GetSkpPnt(pch, 2);
	
	FndFlg = -1;
	while(i < 250)
	{
		DatM1 = *(WavPtr + i - 1);
		Dat0 = *(WavPtr + i);
		DatP1 = *(WavPtr + i + 1);
		//�ɑ�l
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
		
		//�������Ă���΃��[�v�Z�k�̂��߂�i���΂�
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
 * Summary  : �ɏ��l����������
 * Argument : pch : �`�����l���ԍ�
 *          : *WavPtr : �����Ώۂ̔g�`�f�[�^�z��
 *          : *OutVal : ��������(�ɏ��l)
 *          : *OutPos : ��������(�ɏ��ʒu)
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

	//��΂��_�� = ��g�g��(f=600kHz) * �T���v�����O���g��(�I�[�o�[�T���v�����O8�_) = 1/600kHz * 65MHz/8
	// SkpPnt = AdcSmpFrq[SVD[pch].adc_clock] * 1000 / SVD[pch].drive_freq / 8 / 2; //Skip Point (�f�t�H���g��13.54)
	SkpPnt = GetSkpPnt(pch, 2);

	FndFlg = -1;
	while(i < 250)
	{
		DatM1 = *(WavPtr + i - 1);
		Dat0 = *(WavPtr + i);
		DatP1 = *(WavPtr + i + 1);
		//�ɏ��l
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
		
		//�������Ă���΃��[�v�Z�k�̂��߂�i���΂�
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
 * Summary  : �㗬�g�`�̋ɒl/�[���N���X�_/�������l�ʒu��T��
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void GetFwdAnyPnt(short pch)
{
	//�[���N���X�_����������
	SchZerDat(pch, &MES[pch].fow_data[0], &MES[pch].zc_nearzero_point[0][0], &MES[pch].zc_nearzero_data1[0][0], &MES[pch].zc_nearzero_data2[0][0], &MES[pch].zc_nearzero_data3[0][0], &MES[pch].zc_nearzero_data4[0][0]);
	
	SchMaxPek(pch, &MES[pch].fow_data[0], &MES[pch].FwdWavMaxPekValLst[0], &MES[pch].FwdWavMaxPekPosLst[0]);
	SchMinPek(pch, &MES[pch].fow_data[0], &MES[pch].FwdWavMinPekValLst[0], &MES[pch].FwdWavMinPekPosLst[0]);
	
	SchTrshld(pch, &MES[pch].fow_data[0], &MES[pch].ThreasholdPoint_Fow);
}

/*******************************************
 * Function : GetRevAnyPnt (Get Reverse Any Point)
 * Summary  : �����g�`�̋ɒl/�[���N���X�_/�������l�ʒu��T��
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void GetRevAnyPnt(short pch)
{
	//�[���N���X�_����������
	SchZerDat(pch, &MES[pch].rev_data[0], &MES[pch].zc_nearzero_point[1][0], &MES[pch].zc_nearzero_data1[1][0], &MES[pch].zc_nearzero_data2[1][0], &MES[pch].zc_nearzero_data3[1][0], &MES[pch].zc_nearzero_data4[1][0]);
	
	SchMaxPek(pch, &MES[pch].rev_data[0], &MES[pch].RevWavMaxPekValLst[0], &MES[pch].RevWavMaxPekPosLst[0]);
	SchMinPek(pch, &MES[pch].rev_data[0], &MES[pch].RevWavMinPekValLst[0], &MES[pch].RevWavMinPekPosLst[0]);
	
	SchTrshld(pch, &MES[pch].rev_data[0], &MES[pch].ThreasholdPoint_Rev);
}

/*******************************************
 * Function : SearchZerPeakPos
 * Summary  : ZerPeakPos��T��
 * Argument : pch : �`�����l���ԍ�
 *            StartPos : �T���J�n�_
 *            EndPos : �T���I���_
 * Return   : zerPeakPos
 * Caution  : None
 * Note     : 
 * *****************************************/
short SearchZerPeakPos(short pch, short StartPos, short EndPos)
{
//�w��͈͓��̋ɑ�l��T��ver
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
//�w��͈͓��̋ɒl��T��ver
#elif 0
	short i;
	short zerPeakPos = 0;
	short PeakVal[2 * WAV_PEK_NUM];
	short PeakPos[2 * WAV_PEK_NUM];

	//�ɑ�ɏ��������X�g�쐬
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
 * Summary  : �ő�/�ŏ��l����������
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : 
 * *****************************************/
void SchMaxMinPnt(short pch)
{
	short work_old = 0, i, j;
	short SelMult[] = { 32, 40, 65, 80 }; // ���s�N�����v�I���ł̓T���v�����O���g���ɋߎ�?
	short ZerAdjYetFlg = 0; //0:�[���������{, 1:�[�����������{
	short CrsCnt = 0;
	short TrshldVal = 0;
	short TrshldPos = 0;

	SelMult[SVD[pch].adc_clock] = SelMult[SVD[pch].adc_clock] / Mgn;

	// MES[pch].ThresholdPeakPos != 0 : �[���_�������ɔg�`��5�_�����_���������Ă���
	// MES_SUB[pch].zc_peak_req == 1 : �G���[���N�����Ƀ[���_�������I�����Ă���
	// SVD[pch].ZerPeakPos == 0 : �G���[���N�����Ƀ[���_�������I�����Ă���(zc_peak�̓[���_�����I������0���Z�b�g�����)
	// if(MES[pch].ThresholdPeakPos != 0 && MES_SUB[pch].zc_peak_req == 1 && SVD[pch].ZerPeakPos == 0 && MES[pch].zc_peak_UpdateFlg != 0)
	//MES_SUB[pch].zc_peak_req == 1 : �G���[���N�����Ƀ[���_�������I�����Ă���
	//MES[pch].zc_peak_UpdateFlg != 0 : �ʐM�ɂ��zc_peak�X�V�v��������
	if(MES_SUB[pch].zc_peak_req == 1 && MES[pch].zc_peak_UpdateFlg != 0)
	{
		MES_SUB[pch].zc_peak_req = 0;	//�g�`�F��臒l�t�߂̃s�[�N�ʒu�������v�����N���A
		ZerAdjYetFlg = 1;
	}
	
	fow_max = rev_max = 0;					   /*�����l*/
	fow_min = rev_min = AD_MAX;				   /*12bit MAX*/
	work_old = 0;

	//�ő�ŏ��l����
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

	//�[�����������{�̏ꍇ
	if(ZerAdjYetFlg != 0)
	{
		//0~250�̊Ԃ�ZerPeakPos��T��
		SVD[pch].ZerPeakPos = SearchZerPeakPos(pch, 0, 250);

		SVD[pch].ZerCrsSttPnt = SVD[pch].ZerPeakPos;		//�g�`�F���s�[�N�ʒu(�[���N���X�v�Z�J�n�ʒu�p)��EEPROM�ۑ�
		eep_write_ch_delay(pch, (short)(&SVD[pch].ZerPeakPos - &SVD[pch].max_flow), SVD[pch].ZerPeakPos);
		eep_write_ch_delay(pch, (short)(&SVD[pch].ZerCrsSttPnt - &SVD[pch].max_flow), SVD[pch].ZerCrsSttPnt);
	}
}

/*******************************************
 * Function : SetWavTopPos (Set Wave Top Position)
 * Summary  : �g�`�擪�ʒu���v�Z����
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : �g�`�ő�l��50%�_�ɋ߂��ɑ�l���ӂ�3�_�����A�����Ōv�Z����
 * *****************************************/
void SetWavTopPos(short pch)
{
	short CenVal = 0, CenPos = 0;
	short work = 0;
	short i;
	short OffsetPoint[] = { 2355, 2944, 4784, 5888 };

	//1. �����g�`
	CenVal = (rev_max - AD_BASE) / 2;
	//1-1. �g�`�O����������
	for(i = 0; i < WAV_PEK_NUM / 2; i++)
	{
		work = abs(MES[pch].RevWavMaxPekValLst[i] - CenVal);
		if(work < CenVal){
			CenVal = work;
			CenPos = i;
		}
	}
	//1-2. �����m�F
	if((MES[pch].RevWavMaxPekPosLst[CenPos - 1] < MES[pch].RevWavMaxPekPosLst[CenPos]) && (MES[pch].RevWavMaxPekPosLst[CenPos] < MES[pch].RevWavMaxPekPosLst[CenPos + 1]))
	{
		MES[pch].m0_point_rev_50 = (MES[pch].RevWavMaxPekValLst[CenPos - 1] + MES[pch].RevWavMaxPekValLst[CenPos] + MES[pch].RevWavMaxPekValLst[CenPos + 1]) / 3;
		MES[pch].sonic_point_rev_p2 =(short)((long)MES[pch].m0_point_rev_50 * 100) - OffsetPoint[SVD[pch].adc_clock];
		MES[pch].sonic_point_rev_p2 *= Mgn;
	}
	//2. �㗬�g�`
	CenVal = (fow_max - AD_BASE) / 2;
	//2-1. �g�`�O����������
	for(i = 0; i < WAV_PEK_NUM / 2; i++)
	{
		work = abs(MES[pch].FwdWavMaxPekValLst[i] - CenVal);
		if(work < CenVal){
			CenVal = work;
			CenPos = i;
		}
	}
	//2-2. �����m�F
	if((MES[pch].FwdWavMaxPekPosLst[CenPos - 1] < MES[pch].FwdWavMaxPekPosLst[CenPos]) && (MES[pch].FwdWavMaxPekPosLst[CenPos] < MES[pch].FwdWavMaxPekPosLst[CenPos + 1]))
	{
		MES[pch].m0_point_fow_50 = (MES[pch].FwdWavMaxPekValLst[CenPos - 1] + MES[pch].FwdWavMaxPekValLst[CenPos] + MES[pch].FwdWavMaxPekValLst[CenPos + 1]) / 3;
		MES[pch].sonic_point_fow_p1 =(short)((long)MES[pch].m0_point_fow_50 * 100) - OffsetPoint[SVD[pch].adc_clock];
		MES[pch].sonic_point_fow_p1 *= Mgn;
	}
}

/*******************************************
 * Function : ClcDifWavPos (Calculate Difference from both Wave Position)
 * Summary  : ��g�g�`�̍����v�Z����
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : �ɑ�/�ɏ��l, �ő�/�ŏ��l, �[���N���X�_��T��
 * *****************************************/
void ClcDifWavPos(short pch)
{
	if((SVD[pch].fix_data & 0x10) != 0)
	{
		MES[pch].max_point_sub = (MES[pch].zero_sonic_point_fow_p1 - MES[pch].zero_sonic_point_rev_p2 + 50) / 100;		/*��g�̍�*/
	}
	else
	{
		MES[pch].max_point_sub = (MES[pch].sonic_point_fow_p1 - MES[pch].sonic_point_rev_p2 + 50) / 100;		/*��g�̍�*/
		MES[pch].max_point_sub *= Mgn;
	}
	
	//max_point_sub_f���v�Z����
	max_point_control(pch);

	/*�g�`�F�����s���̓`�F�b�N���Ȃ�*/
	if(MES[pch].ThresholdReq == 0)
	{
		if(MES[pch].max_point_sub_f >= LIM_OVERFLOW)
		{
			;//�I�[�o�[�t���[
		}
		else
		{
			MES[pch].err_status &= ~(ERR_JUDGE_REVERSE + ERR_JUDGE_OVERFLOW);	/*�G���[���Z�b�g*/
		}
	}
}

/*******************************************
 * Function : AnalyzeWave
 * Summary  : �擾�����g�`�f�[�^����͂���
 * Argument : pch : �`�����l���ԍ�
 * Return   : 
 * Caution  : None
 * Note     : �ɑ�/�ɏ��l, �ő�/�ŏ��l, �[���N���X�_��T��
 * *****************************************/
void AnalyzeWave(short pch)
{
	short i;
	//�ϐ�������
	memset(MES[pch].FwdWavMaxPekPosLst, 300, sizeof(MES[pch].FwdWavMaxPekPosLst));
	memset(MES[pch].FwdWavMinPekPosLst, 300, sizeof(MES[pch].FwdWavMinPekPosLst));
	memset(MES[pch].RevWavMaxPekPosLst, 300, sizeof(MES[pch].RevWavMaxPekPosLst));
	memset(MES[pch].RevWavMinPekPosLst, 300, sizeof(MES[pch].RevWavMinPekPosLst));
	memset(MES[pch].FwdWavMaxPekValLst, AD_BASE, sizeof(MES[pch].FwdWavMaxPekValLst));
	memset(MES[pch].FwdWavMinPekValLst, AD_BASE, sizeof(MES[pch].FwdWavMinPekValLst));
	memset(MES[pch].RevWavMaxPekValLst, AD_BASE, sizeof(MES[pch].RevWavMaxPekValLst));
	memset(MES[pch].RevWavMinPekValLst, AD_BASE, sizeof(MES[pch].RevWavMinPekValLst));
	memset(MES[pch].zc_nearzero_point, 0, sizeof(MES[pch].zc_nearzero_point));

	//�㗬�g�`
	GetFwdAnyPnt(pch);

	//�����g�`
	GetRevAnyPnt(pch);

	MES[pch].ThreasholdPoint = (MES[pch].ThreasholdPoint_Fow + MES[pch].ThreasholdPoint_Rev - 20) / 2;
	
	// �����Œ�Ȃ̂ňȉ��̏����͕s�v
	// ClcDifWavPos(pch);

	//�g�`�̍ő�/�ŏ��l����������
	SchMaxMinPnt(pch);

	//�g�`�擪�ʒu�v�Z
	SetWavTopPos(pch);
}


/****************************************************/
/* Function : fifo_read								*/
/* Summary  : FIFO����200byte�Ǎ���					*/
/* Argument : pch									*/
/* Return   : �Ȃ�									*/
/* Caution  : �����������̂��߂Ƀ|�C���^�����ɕύX
 * notes    : fifo_no ��FIFO�Ɏ�荞�܂��f�[�^����
 *          : LeadingPosition(��g�o�b�t�@�̐擪����10�_��)�܂ł̓_��
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

	/*�ʑ����v������1/4�����炵�Ă��� (�f�B�U�����O�@�\)*/
	MES[pch].clk_phase++; // �ʑ��X�V(PhaseShift 0��90��180��270��0��)
	if (MES[pch].clk_phase > 3)
	{
		MES[pch].clk_phase = 0; // �ʑ�������
	}

	/*�㗬/�������݂�4�񂸂ō��ޏ����̑O�ɔg�`�L�����m�F����*/
	fifo_ch = ch_search(pch);
	if (fifo_search(pch, fifo_ch) == B_NG)
	{
		// �M���Ȃ��̏ꍇ
		fow_max = rev_max = 0;	  /*�����l*/
		fow_min = rev_min = AD_MAX; /*12bit MAX*/

#if defined(FLWWAVEXP)
		for (i = 12; i < FLWWAVSIZ + 10; i++)
#else
		for (i = 12; i < 250; i++)
#endif
		{
			/*�ő�A�ŏ���T��*/
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
		SelectReverseOn(pch);		//IN/OUT�ō��ݐؑւ�
#endif

	// MES.fifo_no, MES.fifo_no_read, MES.fifo_ch_read�����߂�
	SetFifoNo(pch);

	// MES.fifo_start, MES.fifo_end�����߂�
	SetFifoPos(pch);

	MES[pch].fifo_offset = SVD[pch].wind_offset; /*WINDOW�I�t�Z�b�g���Ԃ�ݒ�*/
	SetFpgaRegister(pch);   //FPGA���W�X�^�̐ݒ�

	// �g�`���擾����
	GetWav(pch);

	// �擾�����g�`����͂���
	AnalyzeWave(pch);

	if (LED[pch].vth_do_cnt != 0)
	{																			// �[���_������
		MES[pch].vth_sum += get_vth2(&MES[pch].fow_data[0], SVD[pch].wave_vth); // �[���_�����pVth���擾����
		MES[pch].vth_sum += get_vth2(&MES[pch].rev_data[0], SVD[pch].wave_vth);
		MES[pch].vth_count++;
	}

	MES[pch].alm_status &= ~(ALM_JUDGE_EMPTY);
	if ((1 <= SVD[pch].alm_hold_time) && (SVD[pch].alm_hold_time <= 99))
	{ // �x���o�͎��Ԑݒ肠��
		alm_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].alm_wave_vth / 100); /*��g���o�x��臒l*/
		if (alm_level <= fow_min)
		{											// ��g���o�x��臒l �� �߰��l
			MES[pch].alm_status |= ALM_JUDGE_EMPTY; /*�G���v�e�B�Z���T�x���Z�b�g*/
		}
	}

}
/****************************************************
 * Function : GetTimDif
 * Summary  : �㗬�A�����̑ł����݂���[���N���X�_�܂ł̎��ԍ������߂�
 * Argument : pch : �`�����l���ԍ�
 * Return   : TimDif : �ł����݂���[���N���X���ϓ_�܂ł̎��ԍ�
 * Caution  : 
 * Notes    : ���ԍ�=(�ł�����~FIFO�܂ł̎���) + (�g�`�̐擪~�[���N���X�_�܂ł̎���)
 *          : �g�`�̐擪~�[���N���X�_�܂ł̎��Ԃ̓I�[�o�[�T���v�����O����Ă���̂�x8
 ****************************************************/
float GetTimDif(short pch, short Mod)
{
	short SmpTim = 0;
	short SglCnt = MES[pch].signal_count; //FIFO�擪����g�`���o�������l�܂ł̓_��
	short Trsold = 0; //��M�o�b�t�@����g�`���o�������l�܂ł̓_��
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

//Sin�J�[�u��64���������e�[�u��
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
 * Summary  : 2�_����sin�J�[�u�𗘗p���ă[���N���X�_�����߂�(CLFC���Z���@1)
 * Argument : y0 : 1�_��y���W
 *            y1 : 2�_��y���W
 * Return   : fw : �␳��̃[���N���X�_
 * Caution  : 
 * notes    : (0,y0),(1,y1)��2�_��sin�J�[�u�ŕ␳����
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
 * Summary  : 4�_���狁�߂���A�����Ń[���N���X�_���v�Z����(CLFC�̉��Z���@2)
 * Argument : xn : x���W
 *            yn : y���W
 * Return   : x : �␳��̃[���N���X�_
 * Caution  : 
 * notes    : x1���(0)�Ƃ�����A��������[���N���X�_�����߁Ax1�Ƃ̘a���Ƃ�
 *            ��A���� : y = ax + b
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
 * Summary  : �[���N���X�_��␳����
 * Argument : xn : x���W
 *            yn : y���W
 * Return   : x : �␳��̃[���N���X�_
 * Caution  : 
 * notes    : METHOD_OLD�ł́A2�_���̃[���N���X�␳�_�̘a��ʂ̃v���Z�X�Ŏg�p���Ă����̂�
 *            �݊����Ƃ邽�߂�tmp_x1��tmp_x2�̘a��Ԃ�
 ****************************************************/
float CorZrcPnt(short x1, short x2, short x3, short x4, short y1, short y2, short y3, short y4)
{
	float x, tmp_x1, tmp_x2;
#if defined(METHOD_OLD)
	tmp_x1 = x1 - (y1 * ((float)(x3 - x1) / (y3 - y1)));	//1�_�ڂ�3�_�ڂ̉�A����
	tmp_x2 = x2 - (y2 * ((float)(x4 - x2) / (y4 - y2)));	//2�_�ڂ�4�_�ڂ̉�A����
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
 * Summary  : �ُ�ȓ_����菜���Ď��ԍ����Čv�Z����
 * Argument : *FwdPnt : �㗬�[���N���X�_
 *            *RevPnt : �����[���N���X�_
              zc_SumOld : �㉺�����ԍ��̘a
			  zc_num : �[���N���X�_�g�p�_��
			  SmpTdt : �T���v�����O����
 * Return   : zc_TdataNew : �ُ�_�����������ώ��ԍ�
 * Caution  : 
 * notes    : 
 ****************************************************/
float DelAbnPnt(float *FwdPnt, float *RevPnt, float zc_SumOld, short zc_num, float SmpTdt)
{
	short i;
	float zc_TdataAve; //���ϒl
	float zc_TdataDif; //���ϒl����̍�
	float zc_TdataDif1; //���ϒl����̍�
	float zc_TdataDif2;
	float zc_TdataMax1; //���ϒl����̍����ő�̓_
	float zc_TdataMax2;
	float zc_TdataWork; //N�Ԗڂ̎��ԍ�
	short zc_TdataMax1Idx;
	float zc_SumNew;
	float zc_TdataNew;

	i = zc_TdataDif = zc_TdataMax1 = zc_TdataMax2 = 0;
	zc_TdataAve = zc_SumOld / zc_num;	//�`�����ԍ��̕��ϒl(�[���N���X�_��)
	//�ő�l��T��
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
	//���_�̍ő�l��T��
	for(i = 0; i < zc_num; i++)
	{
		if(i == zc_TdataMax1Idx) continue; //��΂��Ď���
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

	//�ˏo�_�������čČv�Z
	//�ő�l��臒l����
	if(zc_TdataDif1 < 0.0001)
	{
		zc_SumNew = zc_SumOld;
		zc_TdataNew = zc_SumNew / zc_num;
	}
	//���_�̍ő�l��臒l����
	else if(zc_TdataDif2 < 0.0001)
	{
		zc_SumNew = zc_SumOld - zc_TdataMax1;
		zc_TdataNew = zc_SumNew / (zc_num - 1);
	}
	//���_�̍ő�l��臒l�ȏ�
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
 * Summary  : �㉺���̎��ԍ����v�Z����
 * Argument : *FwdPnt : �㗬�[���N���X�_
 *            *RevPnt : �����[���N���X�_
 * Return   : 
 * Caution  : 
 * notes    : 
 ****************************************************/
void CalculateDeltaT(short pch, float *FwdPnt, float *RevPnt)
{
	float zc_Tup, zc_Tdw, zc_TdataDiff = 0;
	short zc_num = SVD[pch].ZerCrsUseNum;
	short i;
	float zc_limit = (1.0 / SVD[pch].drive_freq * 1000.0) / 3; //�������l=1������1/3 (�����Ō���)
	float SmpTdt = (float)SmpTs[SVD[pch].adc_clock] / 100000; //2�_�Ԃ̎��ԍ�[us]

	//--���ԍ��v�Z--
	zc_Tup = zc_Tdw = 0;
	for(i = 0; i < zc_num; i++)
	{
		zc_Tup += FwdPnt[i];
		zc_Tdw += RevPnt[i];
	}
	//�_->���ԍ�
	zc_Tup *= (SmpTdt / 2.0);
	zc_Tdw *= (SmpTdt / 2.0);

	//--1�g����΍�--
	//�������l����+�����ɒ����鎞�ԍ�������������1�g�Y���Ɣ���
	if(((zc_Tup - zc_Tdw) / zc_num) >= zc_limit)
	{
		//�������̃[���N���X�_���Čv�Z����
		zc_Tdw = 0;
		for(i = 2; i < zc_num - 1; i++)
		{
			//�������̃[���N���X�_��擪2�f�[�^���폜���ĉ��Z
			zc_Tdw += RevPnt[i] * (SmpTdt / 2.0);
			RevPnt[i - 2] = RevPnt[i];
		}
		for(i = zc_num - 1; i < ZC_POINT_MAX; i++)
		{
			RevPnt[i] = 0.0;
		}
		zc_num -= 3;
	}
	//�������l����-�����ɒ����鎞�ԍ�������������1�g�Y���Ɣ���
	else if(((zc_Tup - zc_Tdw) / zc_num) <= (zc_limit*-1))
	{
		//�㗬���̃[���N���X�_���Čv�Z����
		zc_Tup = 0;
		for(i = 2; i < zc_num - 1; i++)
		{
			//�㗬���̃[���N���X�_��擪2�f�[�^���폜���ĉ��Z
			zc_Tup += FwdPnt[i] * (SmpTdt / 2.0);
			FwdPnt[i - 2] = FwdPnt[i];
		}
		for(i = zc_num - 1; i < ZC_POINT_MAX; i++)
		{
			FwdPnt[i] = 0.0;
		}
		zc_num -= 3;
	}

	//�]���p
	if(SVD[pch].sum_start == 11){	//DelAbnPnt�̌v�Z���ʂ�L���ɂ���
		//--�ُ�l2�_���O--
		zc_TdataDiff = zc_Tup - zc_Tdw;
		zc_TdataDiff = DelAbnPnt(FwdPnt, RevPnt, zc_TdataDiff, zc_num, SmpTdt);
	}else if(SVD[pch].sum_start == 12){	//DelAbnPnt�̌v�Z���ʂ�L���ɂ���
		//--�ُ�l���O--
		zc_TdataDiff = zc_Tup - zc_Tdw;
		zc_TdataDiff = DelAbnPntMod(FwdPnt, RevPnt, zc_TdataDiff, zc_num, SmpTdt);
	}else{
		zc_TdataDiff = (zc_Tup - zc_Tdw) / zc_num;	//�`�����ԍ�
	}
	//�]���p
	
	//--�l��ێ�--
	MES_SUB[pch].zc_Tup = zc_Tup;
	MES_SUB[pch].zc_Tdown = zc_Tdw;
	MES[pch].zc_Tdata = zc_TdataDiff;
}

/****************************************************
 * Function : SchZerPnt
 * Summary  : �[���N���X�_��T������
 * Argument : pch : �`�����l���ԍ�
 * Return   : void
 * Caution  : �����������̂��߂Ƀ|�C���^�����ɕύX
 * notes    : 
 ****************************************************/
void SchZerPnt(short pch)
{
#ifdef NEW_MEASURE_METHOD  //�v�Z��������
	short i;
	short zc_num = SVD[pch].ZerCrsUseNum;
	float zc_limit;

	//�|�C���^�錾
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

	short SttPnt = 0; //�[���N���X�_�g�p�J�n�ʒu
	float SmpTdt = (float)SmpTs[SVD[pch].adc_clock] / 100000; //2�_�Ԃ̎��ԍ�[us]

	float zc_Tup = 0;
	float zc_Tdown = 0;
	float zc_TdataDiff = 0;
	
	//�ŏI�o�͕ϐ�
	float *ZerTdtUp;
	float *ZerTdtDw;
	float *ZerTdtVal;

	//�|�C���^�����l
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

	/*�[���_(���S��2047)��ʉ߂���O2�_�A��2�_����[���N���X�_���v�Z����(��A����)*/
	for (i = 0; i < zc_num; i++)
	{
		// // �㗬��
		xu1 = *(FowZerCrsPnt + i + SttPnt) - 1;	//1�_��(�ʉ�2�_�O�̓_)
		xu2 = xu1 + 1;	//2�_��(�ʉ�1�_�O�̓_)
		xu3 = xu1 + 2;	//3�_��(�ʉߌ��1�_��)
		xu4 = xu1 + 3;	//4�_��(�ʉߌ��2�_��)
		yu1 = *(FowZerCrsDat1 + i + SttPnt);
		yu2 = *(FowZerCrsDat2 + i + SttPnt);
		yu3 = *(FowZerCrsDat3 + i + SttPnt);
		yu4 = *(FowZerCrsDat4 + i + SttPnt);

		if((yu1 == yu3) || (yu2 == yu4))	return;  //�[������΍�
		FwdClcDat[i] = CorZrcPnt(xu1, xu2, xu3, xu4, yu1, yu2, yu3, yu4);
		MES[pch].FwdClcZerPnt[i] = FwdClcDat[i];

		// // ������
		xd1 = *(RevZerCrsPnt + i + SttPnt) - 1;	//1�_��(�ʉ�2�_�O�̓_)
		xd2 = xd1 + 1;	//2�_��(�ʉ�1�_�O�̓_)
		xd3 = xd1 + 2;	//3�_��(�ʉߌ��1�_��)
		xd4 = xd1 + 3;	//4�_��(�ʉߌ��2�_��)
		yd1 = *(RevZerCrsDat1 + i + SttPnt);
		yd2 = *(RevZerCrsDat2 + i + SttPnt);
		yd3 = *(RevZerCrsDat3 + i + SttPnt);
		yd4 = *(RevZerCrsDat4 + i + SttPnt);

		if((yd1 == yd3) || (yd2 == yd4))	return;  //�[������΍�
		RevClcDat[i] = CorZrcPnt(xd1, xd2, xd3, xd4, yd1, yd2, yd3, yd4);
		MES[pch].RevClcZerPnt[i] = RevClcDat[i];

		//�[���N���X�_�����Z
		zc_Tup += FwdClcDat[i];
		zc_Tdown += RevClcDat[i];
	}

	//���ԍ����v�Z����
	CalculateDeltaT(pch, FwdClcDat, RevClcDat);
#else
	short cnt1, cnt2, cnt3;
	short zc_num;
	short s_d;
	short fow_before, rev_before;
	short x1, x2, x3, x4;
	float y1, y2, y3, y4;
	short zc_start = 0;
	short SelMult[] = { 32, 40, 65, 80 }; // ���s�N�����v�I���ł̓T���v�����O���g���ɋߎ�?
	float zc_limit;
	short i = 12;

	//�|�C���^�錾
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

	//�|�C���^�����l
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
	TmpFowDat = (short *)(&MES[pch].fow_data[0]);//�[���N���X�T���J�n�ʒu��[zc_start]�Ԗڂ���
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
				*(RevZerCrsPnt + cnt2) = i - 1;		//�ʉ߂���O�̓_��ێ�
				*(RevZerCrsDat1 + cnt2) = *(TmpRevDat + i - 2) - AD_BASE;	//�ʉ�2�_�O�̓_��ێ�
				*(RevZerCrsDat2 + cnt2) = *(TmpRevDat + i - 1) - AD_BASE;	//�ʉ�1�_�O�̓_��ێ�
				*(RevZerCrsDat3 + cnt2) = *(TmpRevDat + i - 0) - AD_BASE;	//�ʉߌ��1�_�ڂ�ێ�
				*(RevZerCrsDat4 + cnt2) = *(TmpRevDat + i + 1) - AD_BASE;	//�ʉߌ��2�_�ڂ�ێ�
			}
			cnt2++;
		}
		rev_before = *(TmpRevDat + i);
		if((0 + SVD[pch].ZerCrsUseNum) <= cnt2)
		{
			break;  //�u�[���N���X�J�n�ʒu+�[���N���X�񐔁v�ȏ�̃[���N���X�_��T���K�v���Ȃ�
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
				*(FowZerCrsPnt + cnt1) = i - 1;		//�ʉ߂���O�̓_��ێ�
				*(FowZerCrsDat1 + cnt1) = *(TmpFowDat + i - 2) - AD_BASE;	//�ʉ�2�_�O�̓_��ێ�
				*(FowZerCrsDat2 + cnt1) = *(TmpFowDat + i - 1) - AD_BASE;	//�ʉ�1�_�O�̓_��ێ�
				*(FowZerCrsDat3 + cnt1) = *(TmpFowDat + i - 0) - AD_BASE;	//�ʉߌ��1�_�ڂ�ێ�
				*(FowZerCrsDat4 + cnt1) = *(TmpFowDat + i + 1) - AD_BASE;	//�ʉߌ��2�_�ڂ�ێ�
			}
			cnt1++;
		}
		fow_before = *(TmpFowDat + i);
		// if(((SVD[pch].ZerCrsSttPnt + SVD[pch].ZerCrsUseNum) < cnt1) && ((SVD[pch].ZerCrsSttPnt + SVD[pch].ZerCrsUseNum) < cnt2))
		if((0 + SVD[pch].ZerCrsUseNum) <= cnt1)
		{
			break;  //�u�[���N���X�J�n�ʒu+�[���N���X�񐔁v�ȏ�̃[���N���X�_��T���K�v���Ȃ�
		}
	}

	/*�[���_(���S��2047)��ʉ߂���O2�_�A��2�_����[���N���X�_���v�Z����(��A����)*/
	zc_num = SVD[pch].ZerCrsUseNum;
	// SttPnt = SVD[pch].ZerCrsSttPnt;
	SttPnt = 0;
	zc_Tup = zc_Tdown = zc_TdataDiff = 0;
	//�O2�_�A��2�_�Ń[���N���X�v�Z
	for (cnt1 = 0; cnt1 < zc_num; cnt1++)
	{
		// �㗬��
		x1 = *(FowZerCrsPnt + cnt1 + SttPnt) - 1;	//1�_��(�ʉ�2�_�O�̓_)
		y1 = *(FowZerCrsDat1 + cnt1 + SttPnt);
		x2 = *(FowZerCrsPnt + cnt1 + SttPnt) + 0;	//2�_��(�ʉ�1�_�O�̓_)
		y2 = *(FowZerCrsDat2 + cnt1 + SttPnt);
		x3 = *(FowZerCrsPnt + cnt1 + SttPnt) + 1;	//3�_��(�ʉߌ��1�_��)
		y3 = *(FowZerCrsDat3 + cnt1 + SttPnt);
		x4 = *(FowZerCrsPnt + cnt1 + SttPnt) + 2;	//4�_��(�ʉߌ��2�_��)
		y4 = *(FowZerCrsDat4 + cnt1 + SttPnt);
		if((y1 == y3) || (y2 == y4))	return;  //�[������΍�
		MES[pch].FwdClcZerPnt[cnt1] = CorZrcPnt(x1, x2, x3, x4, y1, y2, y3, y4);
		*(ZerCrsDats1 + cnt1) = MES[pch].FwdClcZerPnt[cnt1] * SmpTdt / 2.0f;	//���Ԋ��Z

		// ������
		x1 = *(RevZerCrsPnt + cnt1 + SttPnt) - 1;	//1�_��(�ʉ�2�_�O�̓_)
		y1 = *(RevZerCrsDat1 + cnt1 + SttPnt);
		x2 = *(RevZerCrsPnt + cnt1 + SttPnt) + 0;	//2�_��(�ʉ�1�_�O�̓_)
		y2 = *(RevZerCrsDat2 + cnt1 + SttPnt);
		x3 = *(RevZerCrsPnt + cnt1 + SttPnt) + 1;	//3�_��(�ʉߌ��1�_��)
		y3 = *(RevZerCrsDat3 + cnt1 + SttPnt);
		x4 = *(RevZerCrsPnt + cnt1 + SttPnt) + 2;	//4�_��(�ʉߌ��2�_��)
		y4 = *(RevZerCrsDat4 + cnt1 + SttPnt);
		if((y1 == y3) || (y2 == y4))	return;  //�[������΍�
		MES[pch].RevClcZerPnt[cnt1] = CorZrcPnt(x1, x2, x3, x4, y1, y2, y3, y4);
		*(ZerCrsDats2 + cnt1) = MES[pch].RevClcZerPnt[cnt1] * SmpTdt / 2.0f;	//���Ԋ��Z

		//�[���N���X�_�����Z
		zc_Tup += *(ZerCrsDats1 + cnt1);	//�㗬���̃[���N���X�_�����Z
		zc_Tdown += *(ZerCrsDats2 + cnt1);	//�������̃[���N���X�_�����Z
		zc_TdataDiff += *(ZerCrsDats1 + cnt1) - *(ZerCrsDats2 + cnt1);	//�[���N���X�_���̓`�����ԍ�
	}

 	i = zc_TdataWork = zc_TdataMax1 = zc_TdataMax2 = 0;
	zc_TdataAve = zc_TdataDiff / zc_num;	//�`�����ԍ��̕��ϒl(�[���N���X�_��)
	if((zc_TdataAve < -0.0001f) || (0.0001f < zc_TdataAve)){	//�`�����ԍ����[��������(�[���_�t��)�ꍇ�͏������Ȃ�
		for (cnt3 = 0; cnt3 < zc_num; cnt3++)	//�`�����ԍ��̕��ϒl����O��Ă���傫��2�_��T��
		{
			zc_TdataWork = *(ZerCrsDats1 + cnt3) - *(ZerCrsDats2 + cnt3);	//�`�����ԍ�
			zc_TdataAbs = zc_TdataAve - zc_TdataWork;	//�`�����ԍ��̕��ϒl����̍�
			if(zc_TdataAbs < 0)	zc_TdataAbs *= -1;	//��Βl�Ŕ�r����
			if(zc_TdataMax1 == 0 || zc_TdataAbs > zc_TdataMax1){
				if(zc_TdataMax1 != 0){
					zc_TdataMax2 = zc_TdataMax1;	//1�ԖڂɌ덷�̑傫���`�����ԍ���2�ԖڂɈړ�
				}
				zc_TdataMax1 = zc_TdataWork;	//�`�����ԍ��̕ێ�(1�ԖڂɌ덷���傫��)
			}else if(zc_TdataMax2 == 0 || zc_TdataAbs > zc_TdataMax2){
				zc_TdataMax2 = zc_TdataWork;	//�`�����ԍ��̕ێ�(2�ԖڂɌ덷���傫��)
			}
		}
	}

	zc_Tup /= zc_num;	//�㗬���̃[���N���X�_�𕽋�
	zc_Tdown /= zc_num;	//�������̃[���N���X�_�𕽋�
	if(zc_TdataMax1 == 0){
		zc_TdataDiff /= zc_num;		//���ϒl
	}else if(zc_TdataMax2 == 0){
		zc_TdataDiff -= zc_TdataMax1;	//�`�����ԍ��̕��ϒl����O��Ă���1�_�����Z
		zc_TdataDiff /= (zc_num - 1);	//1�_���Z�������ϒl
	}else{
		zc_TdataDiff -= zc_TdataMax1;	//�`�����ԍ��̕��ϒl����O��Ă���2�_�����Z
		zc_TdataDiff -= zc_TdataMax2;
		zc_TdataDiff /= (zc_num - 2);	//2�_���Z�������ϒl
	}

	/*1�g�Y���΍�*/
	zc_limit = (1.0 / SVD[pch].drive_freq * 1000.0) / 3;
	if(zc_TdataDiff >= zc_limit){ //�쓮���g��(100kHz�`5MHz)��1������+�����ɒ����鎞�ԍ�������������1�g�Y���Ɣ���
		//�������̃[���N���X�_���Čv�Z����
		zc_Tdown = 0;
		for(cnt1=2; cnt1<zc_num; cnt1++){
			zc_Tdown = zc_Tdown + *(ZerCrsDats2 + cnt1);  //�������̃[���N���X�_��擪2�f�[�^���폜���ĉ��Z
		}
		zc_Tdown = zc_Tdown / (zc_num - 2);  //�������̃[���N���X�_�𕽋�
		zc_TdataDiff = zc_Tup - zc_Tdown;  //�`�����ԍ�
	}else if(zc_TdataDiff <= (zc_limit*-1)){ //�쓮���g��(100kHz�`5MHz)��1������-�����ɒ����鎞�ԍ�������������1�g�Y���Ɣ���
		//�㗬���̃[���N���X�_���Čv�Z����
		zc_Tup = 0;
		for(cnt1=2; cnt1<zc_num; cnt1++){
			zc_Tup = zc_Tup + *(ZerCrsDats1 + cnt1);   //�㗬���̃[���N���X�_��擪2�f�[�^���폜���ĉ��Z
		}
		zc_Tup = zc_Tup / (zc_num - 2);  //�㗬���̃[���N���X�_�𕽋�
		zc_TdataDiff = zc_Tup - zc_Tdown;  //�`�����ԍ�
	}else{
		;
	}

	*(ZerTdtVal) = zc_TdataDiff;  //�`�����ԍ���ێ�
	*(ZerTdtUp) = zc_Tup;
	*(ZerTdtDw) = zc_Tdown;
#endif
}

/****************************************************
 * Function : ClcSumWavAmp (Caluculate Sum Wave Amplitude)
 * Summary  : �g�`�s�[�N�̉��Z�l���v�Z����
 * Argument : sample : �g�`�C���f�b�N�X
 * Return   : void
 * Caution  : 
 * Notes    : �g�`�擪����1600word���̔g�`�s�[�N(�ɑ�l)�����Z����
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

	/*Window�T�[�`�v������(�œK��FIFO CH��T��)*/
	if(MES[pch].ThresholdReq == 2){
		memset(&ws_max_val[0], 0, sizeof(ws_max_val));
		memset(&ws_min_val[0], 0, sizeof(ws_min_val));
		ws_max = ws_min = AD_BASE_UNIT;
		ws_max_cnt = ws_min_cnt = 0;

		// �g�`���o�����ӏ�����1600word���̔g�`�f�[�^����͂��� 
		search_cnt = 1600 / Mgn;
		for(i=0; i<search_cnt; i++){
			work = rev_temp_data[sample][10+i];
			work &= 0x1FFF;

			if(ws_max < work){	//�ő�l���X�V
				ws_max = work;
				if(work > AD_BASE_UNIT && ws_min != AD_BASE_UNIT){//���S���𒴂������_�ŋɒl(ϲŽ��)�����肷��
					ws_min_val[ws_min_cnt] = ws_min;	//(���g�`�������ȏ㉺���J��Ԃ��Ȃ���v���X�����Ɉړ����Ă��邽�߁A���S���𒴂���܂ł͋ɒl(ϲŽ��)�����肵�Ȃ�)
					ws_min = AD_BASE_UNIT;
					if(ws_min_cnt < 30) ws_min_cnt++;
				}
			}
			if(work < ws_min){	//�ŏ��l���X�V
				ws_min = work;
				if(work < AD_BASE_UNIT && ws_max != AD_BASE_UNIT){//���S��������������_�ŋɒl(�v���X��)�����肷��
					ws_max_val[ws_max_cnt] = ws_max;	//(���g�`�������ȏ㉺���J��Ԃ��Ȃ���ϲŽ�����Ɉړ����Ă��邽�߁A���S���������܂ł͋ɒl(�v���X��)�����肵�Ȃ�)
					ws_max = AD_BASE_UNIT;
					if(ws_max_cnt < 30) ws_max_cnt++;
				}
			}
		}

		MES[pch].ws_work_add = 0;
		for(i=0; i<ws_max_cnt; i++){
			MES[pch].ws_work_add += ws_max_val[i];	//�ɒl(�v���X��)�����Z
		}
		MES[pch].ws_work_add = MES[pch].ws_work_add / ws_max_cnt; //�ɒl�̉񐔂ŕ��ω�	
	}
}

/****************************************************
 * Function : ActBfrFifRed (Action Before FIFO Read)
 * Summary  : FIFORead�O�̏���
 * Argument : void
 * Return   : void
 * Caution  : 
 * Notes    : 
 ****************************************************/
void ActBfrFifRed(short pch)
{
	OutputRestartPulse(); /*RESTART�p���X�o��*/

	// �N�����̏������������I�����Ă���ꍇ
	if (initializing == 0)
	{
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}
	
	// �p���X�ł����ݗL��
	OutputStartPulse();	  /*START�p���X�o��*/
	CheckEndPulse(); /*END�M���m�F*/
	
	/*FIFO�Ǎ���*/
	if (MES[pch].fifo_no_read != 0 && MES[pch].ThresholdPeakPos == 0)/*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
	{
		dma_dummy(MES[pch].fifo_no_read, pch); /*FIFO��ǂ�*/
	}
}

/****************************************************/
/* Function : DriveFIFOFwd                         */
/* Summary  : �㗬��FIFO����(�p���X�ł����݁AFIFO�Ǎ���)		*/
/* Argument : short pch, short sample             */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void DriveFIFOFwd(short pch, short sample){

#if defined(ShtItv)
	//�ō��݌�C���^�[�o���̑O�Ő؂�ւ��Ă����i�������ԒZ�k�j
#else
	SelectReverseOn(pch);	//IN/OUT�ō��ݐؑւ�
	delay(50);			  /*�㗬���ؑ֌�A5usec�ҋ@*/
#endif   

	//�ł����ݑO����
	ActBfrFifRed(pch);
	
	// dma_start(&fwd_temp_data[sample][10]); /*FIFO�Ǎ��݊J�n*/
	us_dma_start(&fwd_temp_data[sample][10]); /*FIFO�Ǎ��݊J�n*/
	OutputRestartPulse();		/*RESTART�p���X�o��*/

	SelectReverseOff(pch);	//IN/OUT�ō��ݐؑւ�

#if defined(ShtItv)
	//�ō��݌�C���^�[�o���̑O�Ő؂�ւ��Ă����i�������ԒZ�k�j
	SelectForwardOn(pch);	//IN/OUT�ō��ݐؑւ�
#endif
}

/****************************************************/
/* Function : DriveFIFORev                         */
/* Summary  : ������FIFO����(�p���X�ł����݁AFIFO�Ǎ���)		*/
/* Argument : short pch, short sample             */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void DriveFIFORev(short pch, short sample){

#if defined(ShtItv)
	//�ō��݌�C���^�[�o���̑O�Ő؂�ւ��Ă����i�������ԒZ�k�j
#else
	SelectForwardOn(pch);		//IN/OUT�ō��ݐؑւ�
	delay(50);					/*�������ؑ֌�A5usec�ҋ@*/
#endif

	//�ł����ݑO����
	ActBfrFifRed(pch);

	// dma_start(&rev_temp_data[sample][10]); /*FIFO�Ǎ��݊J�n*/
	us_dma_start(&rev_temp_data[sample][10]); /*FIFO�Ǎ��݊J�n*/
	OutputRestartPulse();	/*RESTART�p���X�o��*/

	SelectForwardOff(pch);	//IN/OUT�ō��ݐؑւ�

#if defined(ShtItv)
	//�ō��݌�C���^�[�o���̑O�Ő؂�ւ��Ă����i�������ԒZ�k�j

#if defined(ShtCntTwo)
	if(sample < 1)		//�Ō�͐؂�ւ��Ȃ�
#else
//	if(sample < 3)		//�Ō�͐؂�ւ��Ȃ�
	if(sample < (MES_SUB[pch].sample_cnt-1))		//�Ō�͐؂�ւ��Ȃ�
#endif
	{
		SelectReverseOn(pch);	//IN/OUT�ō��ݐؑւ�
	}
#endif
	//�g�`�s�[�N�̉��Z�l���v�Z����(�œK��FIFO CH��T������)
	ClcSumWavAmp(pch, sample);
}

/****************************************************/
/* Function : min_max_check                         */
/* Summary  : data_ok!=0���A���Z�ُ픻��񐔑��������ǂ����`�F�b�N����		*/
/* Argument : short data                            */
/* Return   : 0=�����ĂȂ��A  -1=������	                */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short	min_max_check(short pch){

	short i;
	short corr_time;

#if defined(FRQSCH)
	//���g���T�[�`��
	// if(FrqSch[pch].FrqSchSttFlg == 1){
	if(FrqSch[pch].FrqSchSttFlg != 0){
		corr_time = 1;
	}
	else{
		corr_time = (short)(SVD[pch].correlate_time);  //���Z�ُ픻���
	}
#else
	corr_time = (short)(SVD[pch].correlate_time);  //���Z�ُ픻���
#endif
	MES[pch].read_ok[MES[pch].count_ok] = (char)MES[pch].data_ok;  //�f�[�^�L��(0)/�f�[�^����(1)
	MES[pch].count_ok++;  //�J�E���g�X�V
	if(MES[pch].count_ok >= corr_time){
		MES[pch].count_ok = 0;  //�J�E���g�N���A
	}

	for(i=0; i<corr_time; i++){  //���Z�ُ픻���
		if(MES[pch].read_ok[i] == 0){  //�f�[�^�L��(0)�������
 		return (0);  //���Z�ُ픻��񐔑����ĂȂ�(����)
		}
	}
	return (-1);  //���Z�ُ픻��񐔑�����(�ُ�)
}

/****************************************************/
/* Function : gain_adj_init                         */
/* Summary  : �A���v�Q�C���̒���	                     		*/
/* Argument : pch                            */
/* Return   : 0=���� -1=�ُ�                        */
/* Caution  : �Ȃ�                                   */
/* notes    : ���������̃A���v�Q�C������                 */
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
	retry = 20;								/*�I�����̃��g���C��*/
	MES[pch].amp_gain_for = 255;
	MES[pch].amp_gain_rev = 255;

	/*��R�ݒ�*/
	no = 256;										/*0-255,0=max*/

	/*�g�`�O�a�`�F�b�N���x��*/
	sat_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].saturation_level / 100);	/*�ő�l(12bit MAX)*����臒l*/

	/* �Q�C���Z�o */
	for(i=255; i>=0; i--){						/*amp gain MAX255*/
		/*��R�I��*/
		MES[pch].amp_gain_for = MES[pch].amp_gain_rev = i;		
		WriteGainAmp((unsigned short)i);  /*�Q�C��������*/

		delay(10);								/*�ҋ@*/
		fifo_read(pch);			/*��g�f�[�^�Ǎ���*/
		
		forward_max = reverse_max = 0;			/*�����l*/
		forward_min = reverse_min = AD_MAX;		/*12bit MAX*/

#if defined(FLWWAVEXP)
	 	for(j=10; j<FLWWAVSIZ + 10; j++)
#else
	 	for(j=10; j<250; j++)					/*�ő�A�ŏ���T��*/
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

		if ((forward_max >= sat_level)||(forward_min < GAIN_INIT_LOW*MES_SUB[pch].sample_cnt)	/*�g�`�O�a�`�F�b�N*/
			||(reverse_max >= sat_level)||(reverse_min < GAIN_INIT_LOW*MES_SUB[pch].sample_cnt)){
			/*�U���`�F�b�N�ُ�*/
			retry = 20;
			continue;							/*�Q�C���ύX(down)*/
		}else{									/*�Q�C��OK*/
			if (retry == 0){
				break;							/*�����I���i�O�a���ĂȂ��j*/
			}else{
				retry--;
				if (i<255){
					i++;						/*�Q�C�����̂܂܂ŁA�������*/
				}
				continue;
			}
		}
	}

	MES[pch].fow_max_data = forward_max;		/*�ő�l�ۑ�*/
	MES[pch].fow_min_data = forward_min;		/*�ŏ��l�ۑ�*/
	MES[pch].rev_max_data = reverse_max;		/*�ő�l�ۑ�*/
	MES[pch].rev_min_data = reverse_min;		/*�ŏ��l�ۑ�*/

	if((i>=0) && (i<no)){						/*�Q�C������OK�i�O�a���ĂȂ��j*/
		ret_sts = MES[pch].err_status &= ~(ERR_JUDGE_AGC + ERR_JUDGE_LEVEL);	/*����*/
		/* �Q�C���@*/
		if((forward_max < GAIN_WAVE_HI*MES_SUB[pch].sample_cnt)||(forward_min > GAIN_WAVE_LOW*MES_SUB[pch].sample_cnt)	/*�g�`�m�F*/
			||(reverse_max < GAIN_WAVE_HI*MES_SUB[pch].sample_cnt)||(reverse_min > GAIN_WAVE_LOW*MES_SUB[pch].sample_cnt)){
		/*�Z���T�[�ُ�i�M�����x�����Ⴂ�j*/
			ret_sts = (short)-1;
			i = 0;									/*�Q�C��MAX*/
			WriteGainAmp((unsigned short)i);  /*�Q�C��������*/
			MES[pch].amp_gain_for = MES[pch].amp_gain_rev = i;
		}
		/*�g�`�A���o�����X�`�F�b�N*/
		if(MES[pch].ThresholdReq == 0){	/*�g�`�F�����s���̓`�F�b�N���Ȃ�*/
			forward_max_cal = forward_max - AD_BASE;	/*�ő�l�i���S������̃f�[�^�ʁj*/
			reverse_max_cal = reverse_max - AD_BASE;	/*								*/
			forward_min_cal = AD_BASE - forward_min;	/*�ŏ��l�i���S������̃f�[�^�ʁj*/
			reverse_min_cal = AD_BASE - reverse_min;	/*								*/
			if((((float)(abs(forward_max_cal - reverse_max_cal)) / (float)forward_max_cal * 100) > SVD[pch].balance_level)
				||(((float)(abs(forward_min_cal - reverse_min_cal)) / (float)forward_min_cal * 100) > SVD[pch].balance_level)){
				ret_sts = (short)-1;
				MES[pch].err_status |= ERR_JUDGE_LEVEL;		/*�g�`�A���o�����X�i�㗬���Ɖ������̔g�`���قȂ�j*/
			}
		}
	}else{													/*�Q�C�������s�\*/
		ret_sts = (short)-1;
		MES[pch].err_status |= ERR_JUDGE_AGC;				/*�A���v�s�ǁi�Q�C��MIN�ł��O�a���Ă���j*/ 
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
/* Summary  : �A���v�Q�C���̒���    				*/
/* Argument : pch                               */
/* Return   : 0=����   -1=�ُ�                       */
/* Caution  : �Ȃ�                                   */
/* notes    : ���ʑ��莞�̃A���v�Q�C������              */
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

	/*�g�`�O�a�`�F�b�N���x��*/
	sat_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].saturation_level / 100);	/*�ő�l(12bit MAX)*����臒l*/
	
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

	/* �����Q�C���Z�o */
	if(rev_max >= sat_level){					/*�g�`�O�a�`�F�b�N*/
		MES[pch].amp_gain_rev -= SVD[pch].gain_step; /*�Q�C���X�e�b�v�����Z*/
		if (MES[pch].amp_gain_rev < 0){
			MES[pch].amp_gain_rev = 0;
			ret_sts = (short)-1;
			MES[pch].err_status |= ERR_JUDGE_AGC;		/*�A���v�s�ǁi�Q�C��MIN�ł��O�a���Ă���j*/ 
		}
		amp = MES[pch].amp_gain_for = MES[pch].amp_gain_rev;
	}

	/*�g�`�����`�F�b�N*/
	if(MES[pch].ThresholdReq == 0){	/*�g�`�F�����s���̓`�F�b�N���Ȃ�*/
		/*�㗬��*/
		fow_ratio =	((float)fow_max / (float)MES[pch].fow_max_data)*100;		/*��g�g�`�ő�l�̌�����*/
		if(fow_ratio < 100){													/*�g�`������(100%����)*/
			if((100 - fow_ratio) >= SVD[pch].attenuate_level){				/*�g�`�����`�F�b�N*/
				MES[pch].err_status |= ERR_JUDGE_WAVE;							/*�g�`�����G���[�Z�b�g*/ 
			}
		}
		/*������*/
		rev_ratio =	((float)rev_max / (float)MES[pch].rev_max_data)*100;		/*��g�g�`�ő�l�̌�����*/
		if(rev_ratio < 100){													/*�g�`������(100%����)*/
			if((100 - rev_ratio) >= SVD[pch].attenuate_level){				/*�g�`�����`�F�b�N*/
				MES[pch].err_status |= ERR_JUDGE_WAVE;							/*�g�`�����G���[�Z�b�g*/ 
			}
		}
	}
	MES[pch].fow_max_data = fow_max;		/*�ő�l�ۑ�*/
	MES[pch].fow_min_data = fow_min;		/*�ŏ��l�ۑ�*/
	MES[pch].rev_max_data = rev_max;		/*�ő�l�ۑ�*/
	MES[pch].rev_min_data = rev_min;		/*�ŏ��l�ۑ�*/
	MES[pch].rev_max_data_point = rev_max_point;
	MES[pch].fow_max_data_point = fow_max_point;

	/* �����Q�C���Z�o */
	if (rev_max < GAIN_CONT_LOW*MES_SUB[pch].sample_cnt){						/* �g�`�m�F*/
		/*�Z���T�[�ُ�i�M�����x�����Ⴂ�j*/
		MES[pch].amp_gain_rev += SVD[pch].gain_step; /*�Q�C���X�e�b�v�����Z*/
		if (MES[pch].amp_gain_rev > 255){
			MES[pch].amp_gain_rev = 255;
		}
		amp = MES[pch].amp_gain_for = MES[pch].amp_gain_rev;	
	}	

	/*�g�`�A���o�����X�`�F�b�N*/
	if(MES[pch].ThresholdReq == 0){	/*�g�`�F�����s���̓`�F�b�N���Ȃ�*/
		fow_max_cal = fow_max - AD_BASE;		/*�ő�l�i���S������̃f�[�^�ʁj*/
		rev_max_cal = rev_max - AD_BASE;		/*								*/
		fow_min_cal = AD_BASE - fow_min;		/*�ŏ��l�i���S������̃f�[�^�ʁj*/
		rev_min_cal = AD_BASE - rev_min;		/*								*/
		if((((float)(abs(fow_max_cal - rev_max_cal)) / (float)fow_max_cal * 100) > SVD[pch].balance_level)
			||(((float)(abs(fow_min_cal - rev_min_cal)) / (float)fow_min_cal * 100) > SVD[pch].balance_level)){
				ret_sts = (short)-1;
				MES[pch].err_status |= ERR_JUDGE_LEVEL;	/*�g�`�A���o�����X�i�㗬���Ɖ������̔g�`���قȂ�j*/
		}
	}

	return ret_sts;
}

/****************************************************/
/* Function : sum_adder                           */
/* Summary  : ��Βl�̘a�����Z����    				*/
/* Argument : pch                               */
/* Return   : �Ȃ� 									                         */
/* Caution  : �����������̂��߂Ƀ|�C���^�����ɕύX
 * notes    : �Ȃ�
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

	/*���։��Z*/
	for (offset = 0; offset < SUB_POINT; offset++)
	{					  /*offset �����炵�Ă���*/
		sum_work = 0; /*���Z�l�N���A*/

		/*���։��Z�A���̐�Βl�����Z����*/
		if (SumPntInc == SumPnt_Inc)
		{
#if defined(FLWWAVEXP)
			for (j = 10; j < FLWWAVSIZ + 10; j = j + 2)
#else
			for (j = 10; j < 250; j = j + 2)
#endif
			{ /*10�`210�̃f�[�^��1��΂���100word���̍������Z*/
				 sum_work += (long)abs(*(FowDat + j - 4 + offset) - *(RevDat + j));
			}
		}
		else
		{
			for (j = SumSttPnt; j < SumEndPnt; j = (j + SumStp))
			{	
				/*�������Z�@(�����l:10�`120��110��)*/
				sum_work += (long)abs(*(FowDat + j - 4 + offset) - *(RevDat + j));
			}
		}

		*(SumAbs + offset) = sum_work;

		/*�ʐM�p*/
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
/* Summary  : �ŏ��l�A�ő�l�A�I�t�Z�b�g�̌���  				*/
/* Argument : short data                            */
/* Return   : 0=����A	-1=�f�[�^����                    */
/* Caution  : �����������̂��߂Ƀ|�C���^�����ɕύX
 * notes    : �Ȃ�
 ****************************************************/
short	min_max_search(short pch){

	short	i;
	short ret;
	long	max_work,min_work,m_work;
	long	up_limit,low_limit;

	short SttPnt, EndPnt;
	long *SumAbs;

	SumAbs = (long *)(&MES[pch].sum_abs[0]);
	
	/*���֒l�̑��ݔ͈͂�\��*/
	if(SVD[pch].search_sw == 1 ){		/*���֒l�T�[�`�\���@�\�L��*/
		MES[pch].search_start = MES[pch].max_point_sub_f + 4 - 8;
		if (MES[pch].search_start < 0) MES[pch].search_start = 0;
		if (MES[pch].search_start > SUB_POINT-10) MES[pch].search_start = SUB_POINT-10;
		MES[pch].search_end = MES[pch].max_point_sub_f + 4 + 8;
		if (MES[pch].search_end > (SUB_POINT-1)) MES[pch].search_end = SUB_POINT-1;
		if (MES[pch].search_end < 6) MES[pch].search_end = 6;
	}else{ 							/*���֒l�T�[�`�\���@�\�����i�S�͈�0�`40���T�[�`�j*/
		MES[pch].search_start = 0;
		MES[pch].search_end = SUB_POINT-1;
	}

	SttPnt = MES[pch].search_start;
	EndPnt = MES[pch].search_end;

	/*���֒l�̍Œ�_��������*/
	max_work = (long)0;			/*���̒l�A�ŏ�*/
	min_work = (long)524288;	/*���̒l�A�ő�(4096�~128��)*/

	for (i = SttPnt; i < EndPnt; i++)
	{ /*�ŏ��l��������*/
		if (min_work > *(SumAbs + i))
		{
			min_work = *(SumAbs + i); /*�ŏ��l�ۑ�*/
			m_work = i;						/*�ŏ��I�t�Z�b�g�i�[*/
		}
		if (max_work < *(SumAbs + i))
		{
			max_work = *(SumAbs + i); /*�ő�l�ۑ�*/
		}
	}
	/*�ő�A�ŏ��̔���`�F�b�N*/
	MES[pch].correlate = (short)((max_work - min_work) / 1000);/*�����֒l���W��*/

	up_limit = SVD[pch].corr_up.DWORD;		/*���֒l������l*/
	low_limit = SVD[pch].corr_low.DWORD;	/*���֒l�������l*/
	if(((min_work) < (max_work *10 / SVD[pch].correlate_level)) && 
	    ((max_work - min_work) > low_limit) && ((max_work - min_work) < up_limit) 
	){ /*������~�b�g*/
		MES[pch].min_point_m = (short)m_work;	/*offset(0-19)*/
		MES[pch].sum_max = max_work;			/*�ő�l�X�V*/
		MES[pch].sum_min = min_work;			/*�ŏ��l�X�V*/
		MES[pch].data_ok = 0;					/*�f�[�^�L��*/
		MES[pch].err_status &= ~ERR_JUDGE_PRE_CALC;	/*�G���[���Z�b�g*/
		ret = (short)B_OK;
	}else{
		MES[pch].data_ok = 1;					/*�f�[�^����*/
		MES[pch].err_status |= ERR_JUDGE_PRE_CALC;		/*�G���[�Z�b�g*/
		ret = (short)B_NG;							
	}
	
	return ret;
}

/****************************************************/
/* Function : delta_ts_cal                           */
/* Summary  : ���ԍ������Z����    				*/
/* Argument : pch                              */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 1000=31.25nsec                        */
/****************************************************/
void	delta_ts_cal(short pch){

	short m,i;
	long s_work;

	m = MES[pch].min_point_m;

	if(m <= 1){
		MES[pch].delta_ts0 = (unsigned short)1500;/*���̏ꍇ�͉����l*/
	}else if( m >= abs_cont[SVD[pch].sensor_size]){
		MES[pch].delta_ts0 = (unsigned short)(abs_cont[SVD[pch].sensor_size]-1)*1000+500;/*���s�̏���l*/
	}else{
		/*���s���̎Z�o*/
		if (MES[pch].sum_abs[m-1] > MES[pch].sum_abs[m+1]){/*s0>s2*/
			s_work = MES[pch].sum_abs[m-1];/*S0*/
		}else{/*s0<=s2*/
			s_work = MES[pch].sum_abs[m+1];/*S2*/
		}
		/*���Z���ʂ̊i�[�������[�I���i�v���Ӂj*/
		MES[pch].delta_ts0 = ((unsigned short)m * (unsigned short)1000) + (unsigned short)((MES[pch].sum_abs[m-1]-MES[pch].sum_abs[m+1])*500/(s_work - MES[pch].sum_abs[m])); 	
	}
}

/****************************************************/
/* Function : temp_v                           */
/* Summary  : ���x�␳�p�A��������    				*/
/* Argument : pch                           */
/* Return   : ��g�������l�������Ƃ��̃f�[�^�_*/
/* Caution  : �Ȃ�                                   */
/* notes    : �ł����݃p���X����g�`�擪�ʒu�܂ł̎��Ԃ𑪒肷��̂Ɏg�p
 *          : LeadingPosition(��g�o�b�t�@�̐擪����10�_��)����P�܂ł̓_�������߂�
 ****************************************************/
short	temp_v(short pch){

	short	i;
	short v_work_fow,v_work_rev;
	unsigned short wave_level;
	short work_old;
	short SelMult[] = { 32, 40, 65, 80 }; // ���s�N�����v�I���ł̓T���v�����O���g���ɋߎ�?

	/*��g�g�`���o���x��*/
	wave_level = (unsigned short)((long)AD_MAX * (long)SVD[pch].wave_vth / 100);	/*�ő�l(12bit MAX)*����臒l*/
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
		if(MES[pch].ThresholdPeakPos == 0){  /*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
			/*��g�������l��T��*/
			if( MES[pch].fow_data[i] < wave_level)
			{
				v_work_fow = i;
				break;
			}
		}else{
			/*signal_count-50�ʒu��ϲŽ�������`�v���X�������̃s�[�N��T��*/
			if((50 - (SelMult[SVD[pch].adc_clock] / 2)) <= i && i <= (50 + (SelMult[SVD[pch].adc_clock] / 2))){
				if(MES[pch].fow_data[i] < work_old){
					v_work_fow = i - 1;  /*1�O���s�[�N*/
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
		if(MES[pch].ThresholdPeakPos == 0){  /*�g�`�F��臒l�̖��ݒ�(�[���_���������{)*/
			/*��g�������l��T��*/
			if( MES[pch].rev_data[i] < wave_level)
			{
				v_work_rev = i;
				break;
			}
		}else{
			/*signal_count-50�ʒu��ϲŽ�������`�v���X�������̃s�[�N��T��*/
			if((50 - (SelMult[SVD[pch].adc_clock] / 2)) <= i && i <= (50 + (SelMult[SVD[pch].adc_clock] / 2))){
				if(MES[pch].rev_data[i] < work_old){
					v_work_rev = i - 1;  /*1�O���s�[�N*/
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

	if((SVD[pch].fix_data & 0x10) != 0){  //�Œ�l�ݒ�
 	MES[pch].max_point_sub = (MES[pch].zero_sonic_point_fow_p1 - MES[pch].zero_sonic_point_rev_p2 + 50) / 100;		/*��g�̍�*/
 }else{
#if defined(DatInc)
	 MES[pch].max_point_sub = (MES[pch].sonic_point_fow_p1 - MES[pch].sonic_point_rev_p2 + 50) / 100;		/*��g�̍�*/
	 MES[pch].max_point_sub *= Mgn;
#else
	 MES[pch].max_point_sub = (MES[pch].sonic_point_fow_p1 - MES[pch].sonic_point_rev_p2 + 50) / 100;		/*��g�̍�*/
#endif
	}
	
	if(MES[pch].ThresholdReq == 0){	/*�g�`�F�����s���̓`�F�b�N���Ȃ�*/
		if(MES[pch].max_point_sub_f >= LIM_OVERFLOW){
//			MES[pch].err_status |= ERR_JUDGE_OVERFLOW;		/*�����Œ�Ƃ���̂ŃI�[�o�[�t���[�̓`�F�b�N���Ȃ�*/
		}else{
			MES[pch].err_status &= ~(ERR_JUDGE_REVERSE+ERR_JUDGE_OVERFLOW);	/*�G���[���Z�b�g*/
		}
	}

	return (short)((MES[pch].ThreasholdPoint_Fow + MES[pch].ThreasholdPoint_Rev - 20) / 2);/*����*/
	// return (short)((MES[pch].ThreasholdPoint_Fow + MES[pch].ThreasholdPoint_Rev) / 2);/*����*/
	
}

/****************************************************/
/* Function : sonic_search_max                   */
/* Summary  : ��������    				*/
/* Argument : pch                                  */
/* Return   : �Ȃ� 									                         */
/* Caution  : �����������̂��߂Ƀ|�C���^�����ɕύX
 * notes    : �s�[�N�l����g�`�擪�ʒu�����߂�
 *          : ��g�U����50%臒l�t�߂ɂ���R�Ƃ��̑O��̎R����A50%臒l�Əd�Ȃ�悤�R�ƎR�Ƃ̌���������A
 *          : 50%臒l�Ƃ̌�_����23word�O�̈ʒu���㗬����P1�A��������P2�Ƃ���B
 *          :
 *          : ADCClock=31.25MHz�̂Ƃ�23word(32ns*23=736ns)�O��g�`�擪�ʒu�ɂ��Ă����B
 *          : �T���v�����O���g��[MHz] �T���v�����O����[ns] 736ns�܂ł̓_��
 *          :                 32MHz             31.25ns          23.55
 *          :                 40MHz             25.00ns          29.44
 *          :                 65MHz             15.38ns          47.84
 *          :                 80MHz             12.50ns          58.88
 *          :                 10MHz            100.00ns           7.36 SFC014E�݊�
 *****************************************************/
//Y���f�[�^���U��Ă��邽��Y��3�_�����A�����ł͐擪�ʒu�����肵�Ȃ�
//X���f�[�^�͈��肵�Ă��邽��X������擪�ʒu�����߂�
void	sonic_search_max(short pch){

	short i,work;
	short mult;
	short center;
	short TopPntFow, TopPntRev;
	short SelMult[] = { 32, 40, 65, 80 }; // ���s�N�����v�I���ł̓T���v�����O���g���ɋߎ�?
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

	/*�G���[�`�F�b�N*/
	if ((( MES[pch].err_status & 0x0007) != 0) 
		|| (MES[pch].rev_max_data_point <= 50) || (MES[pch].fow_max_data_point <= 50)
		|| (MES[pch].rev_max_data <= SONIC_WAVE_HI*MES_SUB[pch].sample_cnt) || (MES[pch].fow_max_data <= SONIC_WAVE_HI*MES_SUB[pch].sample_cnt)){
		return;		/*�G���[*/
 }

#if defined(DatInc)
	SelMult[SVD[pch].adc_clock] = SelMult[SVD[pch].adc_clock] / Mgn;
	OffsetPoint[SVD[pch].adc_clock] = OffsetPoint[SVD[pch].adc_clock] / Mgn;
#endif

	mult = SelMult[SVD[pch].adc_clock];  //�ɑ�l�Ԃ̃f�[�^�_��

	/*reverse��----------------------*/
	/*rev���̃|�C���g�T�[�`*/
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
			work = abs(*(RevDat + i) - RevCtrDat); /*center����̍�*/
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
			MES[pch].v0_value_rev[i] = MES[pch].rev_data[MES[pch].m0_point_rev[i]] - AD_BASE;/*�ŏ��l-,0,+*/
		}
	}
	/*****/
	if ((MES[pch].v0_value_rev[0] >= MES[pch].v0_value_rev[1])
		|| (MES[pch].v0_value_rev[1] >= MES[pch].v0_value_rev[2])){  //3�_�������ł��邱�Ƃ��m�F����
		return;
	}else{
		MES[pch].m0_point_rev_50 = (MES[pch].m0_point_rev[0] + MES[pch].m0_point_rev[1] + MES[pch].m0_point_rev[2]) / 3;
	 	MES[pch].sonic_point_rev_p2 =(short)((long)MES[pch].m0_point_rev_50 * 100) - OffsetPoint[SVD[pch].adc_clock];
	 	MES[pch].sonic_point_rev_p2 *= Mgn;
 }

	/*forward��----------------------*/
	/*forward���̃|�C���g�T�[�`*/
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
			work = abs(*(FowDat + i) - FowCtrDat); /*center����̍�*/
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
			MES[pch].v0_value_fow[i] = MES[pch].fow_data[MES[pch].m0_point_fow[i]] - AD_BASE;/*�ŏ��l-,0,+*/
		}
	}

	if ((MES[pch].v0_value_fow[0] >= MES[pch].v0_value_fow[1])
		|| (MES[pch].v0_value_fow[1] >= MES[pch].v0_value_fow[2])){  //3�_�������ł��邱�Ƃ��m�F����
		return;
	}else{
		MES[pch].m0_point_fow_50 = (MES[pch].m0_point_fow[0] + MES[pch].m0_point_fow[1] + MES[pch].m0_point_fow[2]) / 3; 
	 	MES[pch].sonic_point_fow_p1 = (short)((long)MES[pch].m0_point_fow_50 * 100) - OffsetPoint[SVD[pch].adc_clock];/*23�͔g�̐擪*/
	 	MES[pch].sonic_point_fow_p1 *= Mgn;
 }
}

/****************************************************/
/* Function : zero_adj                           */
/* Summary  : �[�������␳	    				*/
/* Argument : pch                          */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	zero_adj(short pch){

	MES[pch].zero_adj_data = (unsigned short)SVD[pch].zero_offset;
	MES[pch].delta_ts_zero = (long)MES[pch].delta_ts - (long)MES[pch].zero_adj_data;/*��Ts�|�[�������f�[�^*/
}

/****************************************************/
/* Function : make_viscos_tbl                           */
/* Summary  : �������j�A���C�Y�i�␳�e�[�u���쐬�j    				*/
/* Argument : pch                              */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ���S�x�ɑ΂���␳�e�[�u��                   */
/****************************************************/
void	make_viscos_tbl(short pch){
	
	short	i, no;
	short ratio;
	long work;
	short swork;
	short iNum;
	short DataNum;

	KV_TBL *tbl;

	if (SVD[pch].viscos_auto == 0){	/*���S�x�A�ݒ�Œ�*/
		/* ��t���j�A���C�Y���[�hON = ���S�x1.00 */
		if(SVD[pch].LL_enable == 1 && SVD[pch].LL_kind != 0){
			MES[pch].viscos_val = 100;
		}else{
			MES[pch].viscos_val = SVD[pch].viscos;/*�ݒ蓮�S�x*/
		}
	}else{								/*���S�x�̉��Z�i���j*/
		/*���S�xcp = 12.069 - 0.00746 * V0(����m/sec)*/
		work = ((long)1206900 - ((long)MES[pch].sound_vel_f * 746)+ 500) / 1000;

		if (work <= 30)   work = 30;	/* 0.30*/
		if (work >= 4000) work = 4000;	/*40.00*/
		MES[pch].viscos_val = (short)work;
	}

	switch(SVD[pch].sensor_size){					/*�Z���T�I��*/
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
	for (i=0; i<iNum; i++){/*���S�x�e�[�u����T��*/
		if ( MES[pch].viscos_val < tbl[i].viscos){
			no = i;
			break;
		}
	}
	/*no-1 �Ɓ@no �̊Ԃ��� */
	DataNum = sizeof(tbl[0].dat) / sizeof(short);
	ratio = (short)((long)(MES[pch].viscos_val - tbl[no-1].viscos) * 1000 / (tbl[no].viscos - tbl[no-1].viscos));
	for (i=0; i<DataNum; i++){
		swork = (short)((long)(tbl[no].dat[i] - tbl[no-1].dat[i]) * ratio / 1000 + tbl[no-1].dat[i]);
		MES[pch].viscos_table[i] = (short)((long)100000000 / (swork + 10000));
		/*10000 = 1�{*/
	}
}

float CalcLinerCorrection(long x0, long x1, long y0, long y1, float x)
{
	float xf0 = (float)x0;
	float xf1 = (float)x1;
	float yf0 = (float)y0;
	float yf1 = (float)y1;
	float y = yf0;
	//����=0�ɂȂ�ꍇ�͌v�Z���Ȃ�
	if(x0 != x1)
	{
		y = (x - xf0) * (yf1 - yf0) / (xf1 - xf0) + yf0;
	}
	return y;
}

/****************************************************/
/* Function : auto_linear                           */
/* Summary  : �������j�A���C�Y    				*/
/* Argument : viscos,   pch                         */
/* Return   : ����[m/sec*E4]                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ���S�x�ɂ�藬����␳����                  */
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
	
	/*** �ꎞ�I�ɗ������Βl�֕ύX ***/
	if(work < 0){
		SignFlg = 1;
		work = work * -1;
	}else{
		SignFlg = 0;
	}

	for (i=0; i < 21; i++){				/*����A�|�C���g��*/
		if ( work >= flow_vel_tbl[i]*10){		/*���ܐ����炎�ܐ��܂�*/
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

	/*** �������� ***/
	if(SignFlg != 0){
		ret = ret * -1;				/* ���������̒l�������ꍇ�A�^�l�֖߂� */
	}
#else
	short i,no;
	short ratio;
	short malt;
	long work,ret;
	short SignFlg;
	
	work = viscos;
	
	/*** �ꎞ�I�ɗ������Βl�֕ύX ***/
	if(work < 0){
		SignFlg = 1;
		work = work * -1;
	}else{
		SignFlg = 0;
	}

	for (i=0; i < 21; i++){				/*����A�|�C���g��*/
		if ( work >= flow_vel_tbl[i]*10){		/*���ܐ����炎�ܐ��܂�*/
			break;
		}
	}
	no = i;
	if (no==0){										/*������664.29 cm/sec�ȏ�*/
		ratio = (flow_vel_tbl[0]*10 - work) * 10000
				/ (flow_vel_tbl[0]*10 - flow_vel_tbl[1]*10); 
		malt = (short) ((long)MES[pch].viscos_table[0] - ((long)( MES[pch].viscos_table[0] - MES[pch].viscos_table[1]) * ratio / 10000));
	}else if (no ==21){								/*������1.00 cm/sec �ȉ�*/
		ratio = (flow_vel_tbl[19]*10 - work) * 10000
				/ (flow_vel_tbl[19]*10 - flow_vel_tbl[20]*10); 
		malt = (short) ((long)MES[pch].viscos_table[19] - ((long)( MES[pch].viscos_table[19] - MES[pch].viscos_table[20]) * ratio / 10000));
	}else{
		ratio = (flow_vel_tbl[no-1]*10 - work) * 10000
				/ (flow_vel_tbl[no-1]*10 - flow_vel_tbl[no]*10); 
		malt = (short) ((long)MES[pch].viscos_table[no-1] - ((long)( MES[pch].viscos_table[no-1] - MES[pch].viscos_table[no]) * ratio / 10000));
	}
	ret = work * malt / 10000;

	/*** �������� ***/
	if(SignFlg != 0){
		ret = ret * -1;				/* ���������̒l�������ꍇ�A�^�l�֖߂� */
	}
#endif
	return ret;
}

/****************************************************/
/* Function : maker_linear                           */
/* Summary  : ���[�J���j�A���C�Y����   				*/
/* Argument : in_work,  pch                         */
/* Return   : �␳����						                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ����(m/sec)��␳����                    */
/****************************************************/
long	maker_linear(long in_work, short pch){

	short	i, point;
	long *x_ptr, *y_ptr;
	long work, ret;

	point = (short)(SVD[pch].mklnr_num >> 8);	/*�␳�_��*/
	/*�␳�������s�m�F*/
	if((point < 1) || (in_work < 0)){
		return (long)in_work;				/*�␳���Ȃ�*/
	}
	if(point > 15){
		point = 15;
	}
	ret = 10000;
	work = (long)in_work;					/*0-10000*/

	/*�����␳*/
	x_ptr = &SVD[pch].mklnr_out1.DWORD;		/*X���f�[�^�擪�A�h���X�i�o�̓f�[�^�j*/
	y_ptr = &SVD[pch].mklnr_in1.DWORD;		/*Y���f�[�^�擪�A�h���X�i���̓f�[�^�j*/

	/*1�_�␳����*/
	if(point == 1){
		/*���_(0,0)�Ƒ�1�ܐ��ŕ␳����*/
		ret =  work * ((float)x_ptr[0] / (float)y_ptr[0]);
			
	/*2�_�ȏ�̕␳����*/
	}else{								
		/*���_(0,0)�����1�ܐ��܂ł̕␳����*/
		if(in_work < (y_ptr[0] * 100)){
			ret =  work * ((float)x_ptr[0] / (float)y_ptr[0]);
			return (long)ret;
		}
	
		/*��1�ܐ������n�ܐ��܂ł̕␳����*/
		for(i=0; i < point-1; i++){		/*����A�|�C���g��*/
			if((work <= (y_ptr[i+1] * 100))
				&& (work >= (y_ptr[i] * 100))){
				ret = (work - (y_ptr[i] * 100))
					* ((float)(x_ptr[i+1] - x_ptr[i]) / (float)(y_ptr[i+1] - y_ptr[i]))
					+ (x_ptr[i] * 100);
				break;
			}
		}

		/*��n�ܐ����傫�����̕␳����*/
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
/* Summary  : ���[�U���j�A���C�Y����    				*/
/* Argument : in_flow,  pch                         */
/* Return   : �␳����						                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ���ʂ�␳����(1=0.1mL/min)              */
/****************************************************/
long	user_linear(long in_flow, short pch){

	short	i, point;
	long *x_ptr, *y_ptr;
	long work, out_flow;

	point = (short)(SVD[pch].uslnr_num >> 8);			/*�␳�_��*/
	/*�␳�������s�m�F*/
	if((point < 1) || (in_flow < 0)){
		return (long)in_flow;					/*�␳���Ȃ�*/
	}
	if(point > 15){
		point = 15;
	}
	out_flow = 10000;
	work = (long)in_flow;

	/*�����␳*/
	x_ptr = &SVD[pch].uslnr_out1.DWORD;			/*X���f�[�^�擪�A�h���X�i�o�̓f�[�^�j*/
	y_ptr = &SVD[pch].uslnr_in1.DWORD;			/*Y���f�[�^�擪�A�h���X�i���̓f�[�^�j*/

	/*1�_�␳����*/
	if(point == 1){
		/*���_(0,0)�Ƒ�1�ܐ��ŕ␳����*/
		out_flow =  work * ((float)x_ptr[0] / (float)y_ptr[0]);		
		
	/*2�_�ȏ�̕␳����*/
	}else{
		/*��t�Ʊײ��Ӱ��(5�_�␳):�␳�ʒǉ�*/
		if(SVD[pch].LL_enable == 1){
			out_flow = ClcChmLnr(work, point, pch);
		}
		/*�ʏ��Ʊײ��Ӱ��*/
		else{
			out_flow = ClcNmlLnr(work, point, pch);
		}
	}

	return (long)out_flow;
}

/****************************************************
 * Function : ClcNmlLnr (Calculate Chemical solution Linerize)
 * Summary  : ��t���j�A���C�Y���v�Z����
 * Argument : work -> ���͒l
 *          : point -> ���j�A���C�Y�_��
 *          : pch -> �`�����l���ԍ�
 * Return   : out_flow -> ���j�A���C�Y�㗬��
 * Caution  : �Ȃ�
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

	//�|�C���^������
	x_ptr = &SVD[pch].uslnr_out1.DWORD;			/*X���f�[�^�擪�A�h���X�i�o�̓f�[�^�j*/
	y_ptr = &SVD[pch].uslnr_in1.DWORD;			/*Y���f�[�^�擪�A�h���X�i���̓f�[�^�j*/

	//�ϐ�������
	FlwMgn = 10; //
	
	// work *= FlwMgn; //���͂�0.1mL/min�̏ꍇ
	x_val0 = x_ptr[0] * FlwMgn;
	y_val0 = y_ptr[0] * FlwMgn;
	tbl_val0 = LL_TBL[SVD[pch].LL_kind].LinerData[0] * FlwMgn;

	/*���_(0,0)�����1�ܐ��܂ł̕␳����*/
	if(work < y_val0){
		x_ptr_LL0 = x_val0 + tbl_val0;
		out_flow =  work * ((float)x_ptr_LL0 / (float)y_val0);
		return (long)out_flow;
	}
	/*��1�ܐ������n�ܐ��܂ł̕␳����*/
	for(i=0; i < point-1; i++){		/*����A�|�C���g��*/
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
	/*��n�ܐ����傫�����̕␳����*/
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
 * Summary  : �ʏ탊�j�A���C�Y���v�Z����
 * Argument : work -> ���͒l
 *          : point -> ���j�A���C�Y�_��
 *          : pch -> �`�����l���ԍ�
 * Return   : out_flow -> ���j�A���C�Y�㗬��
 * Caution  : �Ȃ�
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

	//�|�C���^������
	x_ptr = &SVD[pch].uslnr_out1.DWORD;			/*X���f�[�^�擪�A�h���X�i�o�̓f�[�^�j*/
	y_ptr = &SVD[pch].uslnr_in1.DWORD;			/*Y���f�[�^�擪�A�h���X�i���̓f�[�^�j*/

	//�ϐ�������
	FlwMgn = 10; //0.01mL/min��1�ƕ\��

	// work *= FlwMgn; //���͂�0.1mL/min�̏ꍇ
	x_val0 = x_ptr[0] * FlwMgn;
	y_val0 = y_ptr[0] * FlwMgn;

	/*���_(0,0)�����1�ܐ��܂ł̕␳����*/
	if(work < y_val0){
		out_flow =  work * ((float)x_val0 / (float)y_val0);
		return (long)out_flow;
	}

	/*��1�ܐ�����n�ܐ��܂ł̕␳����*/
	for(i=0; i < point-1; i++){		/*����A�|�C���g��*/
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

	/*��n�ܐ����傫�����̕␳����*/
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
/* Summary  : ��t���j�A���C�Y���[�h�E��ʐݒ�    				*/
/* Argument : vis,  pch                            */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void LLmode_kind(short vis, short pch){

	short i;
	short point;
	short SensorSize = SVD[pch].sensor_size;

	point = (short)(SVD[pch].uslnr_num >> 8) & 0x00FF;	/*���j�A���C�Y�_���擾*/
	
	SVD[pch].LL_kind = 0;
	//��v���������F�Ȃ��ꍇ��0�ݒ��
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
/* Summary  : ��Ts�𗬗ʒl�Ɋ��Z����    				*/
/* Argument : pch                                  */
/* Return   : ���ʒl								                         */
/* Caution  : �Ȃ�                                   */
/* notes    : 1=0.1mL/min                           */
/*          : fv = L/(f_Ts*(N+(P1+P2)/2) - ��s + FIFOch*8.192)
 *          :   fv -> ���� 
 *          :   L  -> ���蒷
 *          :   f_Ts -> �T���v�����O����
 *          :   N -> �g�`�擪�ʒu(FIFO�Ɏ�荞�񂾃f�[�^�擪����g�`��M�o�b�t�@�擪�܂ł̃f�[�^�_)
 *          :   P1 -> �㗬����g�ʒu(�g�`��M�o�b�t�@�擪����L���Ȕg�`�̐擪�܂ł̃f�[�^�_)
 *          :   P2 -> ��������g�ʒu(�g�`��M�o�b�t�@�擪����L���Ȕg�`�̐擪�܂ł̃f�[�^�_)
 *          :   ��s -> �Z���T�x��[us]
 *          :   FIFOch -> ��g��荞�݊J�nwindow�ʒu
 *          :   8.192 -> f_Ts=32ns �̎���FIFO1ch��(256word)�ǂݏo������[us]
 *          : ���� = (fv*fv/(2*fl)) * ��Ts/1000 * f_Ts/10000
 *          :   fv -> ����[m/s]
 *          :   fl -> �Z���T����[mm]
 *          :   f_Ts -> �T���v�����O����[10ps]
 * 
 *          : <------------FIFO�Ɏ�荞�񂾃f�[�^(3800word)------------>
 *          :           <-��g�o�b�t�@(200word)->
 *          :           <--0--><-�L���g�`-><-0-->
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
	long ReadTimePerFifoch; // FIFO CH 1ch(256word or 128word)��ǂނ̂ɂ����鎞��
	long DataPointToWaveHead;
	long Dnmntr; //����
	float zc_flow_val;
	float zc_deltaT;

	ReadTimePerFifoch = (long)(SmpTs[SVD[pch].adc_clock] * 256 / 100); //[us:1000�{�f�[�^] ,Mercury�Z���T�͑S��256word?

	s_d = SVD[pch].sensor_size; // �Z���T���a
	s_v = MES[pch].sound_vel_f; // �����̉��� (m/sec)
	if (SVD[pch].sns_option == 0)
	{
		sns_L_l = sens_inf[s_d].sns_disL_l; // �Z���T����(L-l) (*0.1mm�P��)
		sns_L = sens_inf[s_d].sns_disL;		// �Z���T����(L) (*0.1mm�P��)
		sns_TAU = sens_inf[s_d].sns_TAU;
		f_Ts = SmpTs[SVD[pch].adc_clock]; // �T���v�����O����
	}
	else
	{
		sns_L_l = SVD[pch].sns_disL_l; // �Z���T����(L-l) (*0.1mm�P��)
		sns_L = SVD[pch].sns_disL;	   // �Z���T����(L) (*0.1mm�P��)
		sns_TAU = SVD[pch].sns_tau;
		f_Ts = SmpTs[SVD[pch].adc_clock]; // �T���v�����O����
	}
	MES[pch].FwdSurplsTim = MES[pch].RevSurplsTim = sns_TAU;

	/*���������߂� V = 1/2 * C^2/L * ��t */
	//��������
	if (ClcActFlg == ClcAct_SumAdd)
	{
		MES[pch].delta_ts_zero_f = MES[pch].delta_ts_zero;

		work_vel = (long)((((float)s_v * (float)s_v / (2.0 * (float)sns_L_l)) * 10) * (float)f_Ts / 1000);
		MES[pch].flow_vel = work_vel * MES[pch].delta_ts_zero_f / 10000; /*����[m/s*E4](V)*/
		MES[pch].flow_vel_a = (long)MES[pch].flow_vel;
	}
	//�[���N���X
	else
	{
		zc_deltaT = MES_SUB[pch].zc_delta_ts_zero * 100;
	    zc_flow_val = (((float)s_v * (float)s_v / (2 * (float)sns_L_l)) * 10 * zc_deltaT);
		MES[pch].flow_vel_a = (long)zc_flow_val;
	}

	/*���[�J���j�A���C�Y*/
	MES[pch].flow_vel_b = maker_linear(MES[pch].flow_vel_a, pch);

	make_viscos_tbl(pch);

	if (SVD[pch].viscos == 0)
	{ /*���S�x�ݒ�u0�v���͎������j�A���C�Y����*/
		MES[pch].flow_vel_c = MES[pch].flow_vel_b;
	}
	else
	{																 /*�������j�A���C�Y�L��*/
		MES[pch].flow_vel_c = auto_linear(MES[pch].flow_vel_b, pch); /*�������j�A���C�Y*/
	}

	/*����[mL/min*10]*/ /*0.01ml/min*/
	out_flow = MES[pch].flow_vel_c * (long)sens_inf[s_d].area / 1000 * (long)6 / 10;

	/*3�_����ʒu�����߂�Bsonic_point�㗬�����̕���*/
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
			{ // �Œ�l�ݒ�
				DataPointToWaveHead = (long)(((long)MES[pch].signal_count - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].signal_count - (long)50 + (((long)MES[pch].zero_sonic_point_rev_p2 + MES[pch].zero_sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
				MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			}
			else if ((SVD[pch].fix_data & 0x10) == 0 && (SVD[pch].fix_data & 0x08) != 0)
			{ // �Œ�l�ݒ�
				DataPointToWaveHead = (long)(((long)MES[pch].zero_signal_count - (long)(MES[pch].ThreasholdPoint * 100) + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100);
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * (long)8192)));
				// MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * ((long)MES[pch].zero_signal_count - (long)50 + (((long)MES[pch].sonic_point_rev_p2 + MES[pch].sonic_point_fow_p1) / 200)) / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
				MES[pch].sound_vel = (short)((long)100000000 / ((long)f_Ts * DataPointToWaveHead / 100 - sns_TAU + ((long)MES[pch].fifo_ch * ReadTimePerFifoch)));
			}
			else if ((SVD[pch].fix_data & 0x18) != 0)
			{ // �Œ�l�ݒ�
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
		MES[pch].sound_vel = (short)((long)MES[pch].sound_vel * sns_L / 1000); /*100mm��1�{�Ƃ���*/
	}

	return out_flow;
}

/****************************************************/
/* Function : pv_calc                           */
/* Summary  : ���ʒl��K�t�@�N�^�␳����   				*/
/* Argument : in_flow,   pch                        */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ���͗��� (1=0.01mL/min)                */
/*          : �o�͗��� (1=0.01mL/min)               */
/****************************************************/
long	pv_calc(long in_flow, short pch){

	short i,work;
	short k_scale_work;
	float out_flow;
	
	/*Full Scale value*//*�t���X�P�[���l�̌���*/
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

	/*K�t�@�N�^�݊��W��*/
	if(SVD[pch].sns_option == 0){	
		k_scale_work = sens_inf[SVD[pch].sensor_size].k_scale;
	}else{
		k_scale_work = SVD[pch].sns_coef;
	}
	out_flow = (float)in_flow * ((float)10000 / (float)k_scale_work);

	/*K�t�@�N�^�i0.700-3.000)*/
	out_flow = out_flow * SVD[pch].k_factor / 1000;

	/*���ʒl���~�b�g�`�F�b�N*/
	if(out_flow > (MES[pch].max_flow_long * 100 * 2)){
		out_flow = MES[pch].max_flow_long * 100 * 2;	/*���(�t���X�P�[����2�{)*/
	}
	if(out_flow < (-MES[pch].max_flow_long * 100 )){
		out_flow = -MES[pch].max_flow_long * 100;		/*����(�t���X�P�[����-1�{)*/
	}
		
	return (long)out_flow;
}

/****************************************************/
/* Function : sonic_filter_control                */
/* Summary  : �����t�B���^����    				*/
/* Argument : ch                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	sonic_filter_control(short ch){

	int	work_filt;
	
	MES[ch].flow_filt_sv_v = SVD[ch].sound_vel_filter * T100MS;	/* (�����t�B���^���萔*10)*(100msec�����݉�) */
	work_filt = MES[ch].sound_vel;
	MES[ch].flow_filt_in_v = (long)work_filt * 10000;
	Filt(&MES[ch].flow_filt_sv_v ,&MES[ch].flow_filt_in_v ,&MES[ch].flow_filt_out_v );

	if(SVD[ch].sound_vel_sel == 0){		/*�����Œ�̏ꍇ*/
		MES[ch].sound_vel_f = SVD[ch].sound_vel_fix;
	}else{
 	MES[ch].sound_vel_f = (short)(MES[ch].flow_filt_out_v / 10000);
	}
}

/****************************************************/
/* Function : correlate_check                       */
/* Summary  : ���Z�ُ픻��    				*/
/* Argument : ����                            */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	correlate_check(short ch){
	
	if((min_max_check(ch) < 0) || 			// �������֒l�̔�̃G���[���A���Z�ُ픻��񐔘A���ő������ꍇ
	    ((MES[ch].fow_max_data - MES[ch].fow_min_data) < 100)){	// �g�`�̐U����100�ȉ��̏ꍇ
		MES[ch].err_status |= ERR_JUDGE_CALC;			/*�G���[�Z�b�g*/
	}else{												/*�g�`����*/
		MES[ch].err_status &= ~ERR_JUDGE_CALC;		/*�G���[���Z�b�g*/
	}
}

/****************************************************/
/* Function : max_point_control                   */
/* Summary  : ��g�̍�(p1-P2)    				*/
/* Argument : ch                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : ���f�B�A���t�B���^    				*/
/* Argument : in_flow,   ch                         */
/* Return   : ���f�B�A���t�B���^��̒l                       */
/* Caution  : �Ȃ�                                   */
/* notes    : 1=0.01mL/min                          */
/****************************************************/
long median_filter_control(long in_flow, short ch){

	long	out_flow;

	out_flow = in_flow;

	if(SVD[ch].filter_mode == FILTER_DIRECT		//�t�B���^�����ݒ莞
	|| SVD[ch].filter_mode == FILTER_MOVING){	//�ړ�����(1-50��)�ݒ莞
		;
	}else{		//���f�B�A���t�B���^�ݒ莞
		if((MES[ch].err_status & ERR_MEDIAN_CALC) == 0) {	/*�ΏۃG���[�������̓��f�B�A���������Ȃ�*/
			if(MES[ch].median_ptr < (MF_DEPTH-1)){
				MES[ch].median_ptr++;
			}else{
				MES[ch].median_ptr = 0;
			}
			MES[ch].median_buf[MES[ch].median_ptr] = in_flow;
			out_flow = median_filter(&(MES[ch].median_buf[0]));		/*ml_min_d (0.01mL/min��1�ŕ\��)*/
		}
	}
	return (long)out_flow;
}

/*******************************************
 * Function : ClcFlwFlt_MvgAve (Calculate Flow Filter Moving Average)
 * Summary  : ���ʂɑ΂���ړ����ς̌v�Z
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
	
	//����ݒ�
	if(SumNum > 50){
		SumNum = 50;
	}
	MES[pch].MvgAveFlwBuf[BufCnt] = in_flow;

	//�o�b�t�@�̑��a�v�Z
	MES[pch].MvgAveFlwSum = 0;
	for(i=0; i<SumNum; i++){
		MES[pch].MvgAveFlwSum += MES[pch].MvgAveFlwBuf[i];
	}

	//�ړ����ϒl�v�Z
	OutFlw = ((float)MES[pch].MvgAveFlwSum / SumNum);
	OutFlw = RoundFunc(OutFlw);	//�l�̌ܓ�

	//�J�E���^�X�V
	BufCnt++;
	if(BufCnt >= SumNum){
		BufCnt = 0;
	}
	MES[pch].MvgAveFlwCnt = BufCnt;

	return (long)OutFlw;
}

/*******************************************
 * Function : ClcFlwFlt (Calculate Flow Filter)
 * Summary  : ���ʂɑ΂���t�B���^�̌v�Z
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
	case FILTER_MOVING: //�ړ�����
		OutFlw = ClcFlwFlt_MvgAve(in_flow, pch);
		break;
	case FILTER_MEDIAN: //���f�B�A���t�B���^
		OutFlw = median_filter_control(in_flow, pch);
		break;
	default: //�t�B���^�Ȃ�
		OutFlw = in_flow;
		break;
	}
	return OutFlw;
}

/****************************************************/
/* Function : damp_control                           */
/* Summary  : �_���s���O����    				*/
/* Argument : ch                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	damp_control(short ch){

short damp_work;

	/*�t�B���^����*/
	damp_work = SVD[ch].damp;
	MES[ch].flow_filt_sv = damp_work * T100MS;
	MES[ch].flow_filt_in = (long)MES[ch].ml_min_now * 1000;		/*���[�N�G���A�g��*/
	Filt(&MES[ch].flow_filt_sv ,&MES[ch].flow_filt_in ,&MES[ch].flow_filt_out );/*�t�B���^����*/
	MES[ch].ml_min_now = (MES[ch].flow_filt_out / 1000);			/*�o�̓X�P�[����߂�*/
}

/****************************************************/
/* Function : lowcut_control                        */
/* Summary  : ���[�J�b�g����    				*/
/* Argument : ch                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	lowcut_control(short ch){

	work_oq = MES[ch].ml_min_now;	/*oq�R�}���h�p���ʒl��ێ�*/

	if(SVD[ch].low_cut != 0){				/*���[�J�b�g0%�ݒ莞�͖���*/
		if(MES[ch].ml_min_now <= ((long)MES[ch].max_flow_long * (long)SVD[ch].low_cut * 10) / 100){
			MES[ch].ml_min_now = 0;	
		}
	}
}

/****************************************************/
/* Function : burnout_control                       */
/* Summary  : �o�[���A�E�g����    				*/
/* Argument : ch                            */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	burnout_control(short ch){
	
	short work_buf;

	if(((MES[ch].err_status & ERR_BURN_OUT) != 0)	/*�g�`�ُ펞*/
			&& (MES[ch].err_burn_out == B_ON)){		/*�G���[�z�[���h�^�C���o��*/
		work_buf = SVD[ch].burnout;				/* �o�[���A�E�g���� */
		/*�o�[���A�E�g����*/
		switch(work_buf){						/*�o�͑I��*/
			case 0:							/*0%�ݒ�*/
				MES[ch].ml_min_now = work_oq = 0;
				break;
			case 1:							/*-25%�ݒ�*/
				MES[ch].ml_min_now = work_oq = MES[ch].max_flow_long * (-25); 
				break;
			case 2:							/*125%�ݒ�*/
				MES[ch].ml_min_now = work_oq = MES[ch].max_flow_long * 125;
				break;
			case 3:							/*�C�Ӑݒ�(-300�`300%)*/
				MES[ch].ml_min_now = work_oq = MES[ch].max_flow_long * SVD[ch].burnout_value;
				break;
			case 4:							/*HOLD�ݒ�*/
				break;
			default:
				break;
		}
	}else{						/*����g�`��*/
		;
	}
}

/****************************************************/
/* Function : median_filter                        */
/* Summary  : ���f�B�A���t�B���^����                  				*/
/* Argument : *d                                  */
/* Return   : ���f�B�A���t�B����̒l	                      */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : �A�b�e�l�[�^�Q�C���l�̎擾    				*/
/* Argument : pch                                 */
/* Return   : �A�b�e�l�[�^�Q�C���l						                   */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
short get_attenuator_gain(short pch)
{
	short	gain;

//	gain = SVD[pch].atn_gain;		/*�A�b�e�l�[�^�Q�C���l�擾*/
	gain = 0;
	
	if(gain < 0){					/*�A�b�e�l�[�^�Q�C���l���~�b�g�`�F�b�N*/
		gain = 0;
	}
	if(gain > 255){				/*�A�b�e�l�[�^�Q�C���l���~�b�g�`�F�b�N*/
		gain = 255;
	}

	return gain;
}

/****************************************************/
/* Function : Filt                           */
/* Summary  : �t�B���^����                         				*/
/* Argument : *Tf, *IN, *OUTn                       */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : �ώZ�w���m�F    				*/
/* Argument : ch                                */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
		for(i_cnt=0; i_cnt<3; i_cnt++){				//3��ǂݍ��݁i�`���^�����O�΍�j
			//RS485�ʐM�ɂ��ώZ�v��
			port_sts = 0;

			if(__bit_input(GPIO_PORTS_BASE, 3)== 1)	// GPIO_PS3 (EXT_IN6)
			{
				port_sts |= (1 << 6);	// RX �� FW �� EXT_IN6 �Ƀr�b�g�ʒu�����킹�� 
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
				case	CH1:		//CH1�ώZ�w��
						if((port_sts & 0x0001) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH2:		//CH2�ώZ�w��
						if((port_sts & 0x0002) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH3:		//CH3�ώZ�w��
						if((port_sts & 0x0004) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH4:		//CH4�ώZ�w��
						if((port_sts & 0x0008) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH5:		//CH5�ώZ�w��
						if((port_sts & 0x0010) == 0){
							on_cnt++;
						}else{
							off_cnt++;
						}
						break;
				case	CH6:		//CH6�ώZ�w��
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
		if(on_cnt == 3){			//3��A���I���ŐώZ�w������
			addit_sts = B_ON;
		}
		if(off_cnt == 3){			//3��A���I�t�ŐώZ�w���Ȃ�
			addit_sts = B_OFF;
		}
		sts = addit_sts;
	
	}else{
		if(MES[ch].addit_req != 0){		//CUnet�ʐM�ɂ��ώZ�v��
			sts = B_ON;
		}

	}
	return(sts);
}

/****************************************************/
/* Function : addit_flow_calc                       */
/* Summary  : �ώZ���ʏ���    				*/
/* Argument : �Ȃ��@�@�@�@�@�@                           */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void addit_flow_calc(void){

	short ch;
	short unit_val;
	long pv_now_work;

	unit_val = 0;
	for(ch=CH1; ch<CH_NUMMAX; ch++){
		if(LED[ch].zero_active != 0){	/*�[���_�������͐ώZ���ʏ������Ȃ�*/
			continue;
		}

		if(addit_req_check(ch) != B_ON){					/*�ώZ�w���m�F*/
			if(MAIN[ch].com_act_status == ACT_STS_ADDIT){ //����X�e�[�^�X���ώZ���s���̏ꍇ
				action_status_control(ch, ACT_STS_NORMAL);		/*����X�e�[�^�X�X�V*/
			}
			if(MES[ch].addit_req_w == B_ON){				/*�ώZ�w��ON��OFF*/
				MES[ch].addit_req_w = B_OFF;				/*�ώZ�w���ۑ�*/
				MES[ch].addit_buff.DWORD = MES[ch].addit_unit.DWORD;	/*�ώZ���ʃo�b�t�@�i�ʐM�p�j�X�V*/
				MES[ch].addit_buff.DWORD = get_total_offset(ch, MES[ch].addit_buff.DWORD);	//�ώZ�l�I�t�Z�b�g
			}
			MES[ch].addit_cont.DWORD = 0;					/*�ώZ���ʃJ�E���^�N���A*/
			MES[ch].addit_unit.DWORD = 0;					/*�ώZ���ʒP�ʊ��Z�ʃN���A*/
			MES[ch].addit_mod = 0;							/*�ώZ���ʂ̗]��N���A*/
			
			if(MES[ch].addit_watch == B_ON){		//�ώZ�Ď��L��
				if(MES[ch].addit_cont.DWORD != 0	//�ώZ�֘A�f�[�^���N���A���Ă��Ȃ��ꍇ
					|| MES[ch].addit_unit.DWORD != 0
					|| MES[ch].addit_mod != 0){
					MES[ch].total_status |= TTL_JUDGE_CACL_ERR;		//�ώZ�l���Z�ُ�
				}
			}else{		//�ώZ�Ď�����
				MES[ch].total_status &= ~TTL_JUDGE_REACH;		//�ώZ���B�o�̓��Z�b�g
			}
			continue;
		}
		MES[ch].addit_req_w = B_ON;					/*�ώZ�w���ۑ�*/

		action_status_control(ch, ACT_STS_ADDIT);		/*����X�e�[�^�X�X�V*/

		if((MES[ch].err_status & ERR_JUDGE_EMPTY) != 0){	/*��g�g�`�������͐ώZ���ʏ������Ȃ�*/
			continue;
		}
		
		//�ώZ���ʌv�Z
		if(MES[ch].ml_min_now <= 0){
			pv_now_work = 0;							/*�}�C�i�X���ʎ��́u0�v���Z*/
		}else{
			pv_now_work = MES[ch].ml_min_now;			/*�u������*/
		}
		//�ώZ����(mL/min)
		MES[ch].addit_pv_w = (unsigned long)pv_now_work * 100;

//#if 1 //6ms�����݂ɕύX
//		//�ώZ���ʂ�6msec���Z�ɕύX����
//		//�ώZ����(mL/min) �� �ώZ����(mL/6msec)
//		MES[ch].addit_pv = (MES[ch].addit_pv_w + MES[ch].addit_mod) / 10;		/*20000�Ŋ��邪�A�������ʑΉ��Ƃ���1000�{����*/
//
//		//�ώZ���ʂ̗]��v�Z
//		MES[ch].addit_mod = (MES[ch].addit_pv_w + MES[ch].addit_mod) % 10;
//		MES[ch].addit_mod_ov = (MES[ch].addit_pv_w + MES[ch].addit_mod_ov) % 10;
//#else
		//�ώZ���ʂ�3msec���Z�ɕύX����
		//�ώZ����(mL/min) �� �ώZ����(mL/3msec)
		MES[ch].addit_pv = (MES[ch].addit_pv_w + MES[ch].addit_mod) / 20;		/*20000�Ŋ��邪�A�������ʑΉ��Ƃ���1000�{����*/

		//�ώZ���ʂ̗]��v�Z
		MES[ch].addit_mod = (MES[ch].addit_pv_w + MES[ch].addit_mod) % 20;
		MES[ch].addit_mod_ov = (MES[ch].addit_pv_w + MES[ch].addit_mod_ov) % 20;
//#endif

		//�ώZ���ʃJ�E���^�X�V
		MES[ch].addit_unit.DWORD += MES[ch].addit_pv;
		if(MES[ch].addit_unit.DWORD > 999999999990000){
			MES[ch].addit_unit.DWORD = 0;					/*�ώZ���ʒP�ʊ��Z�ʃN���A*/

			if(MES[ch].addit_watch == B_ON){		//�ώZ�Ď��L��
				MES[ch].total_status |= TTL_JUDGE_OVERFLOW;		//�ώZ�l�I�[�o�[�t���[
			}
		}
		MES[ch].addit_unit_ov.UINT64 += MES[ch].addit_pv;
		if(MES[ch].addit_unit_ov.UINT64 > 999999999990000){
			MES[ch].addit_unit_ov.UINT64 = 0;			/*�ώZ���ʒP�ʊ��Z�ʃN���A*/

			if(MES[ch].addit_watch == B_ON){		//�ώZ�Ď��L��
				MES[ch].total_status |= TTL_JUDGE_OVERFLOW;		//�ώZ�l�I�[�o�[�t���[
			}
		}

		//�ώZ���ʃo�b�t�@�iov�R�}���h�ʐM�p�j�X�V
		MES[ch].addit_buff_ov.UINT64 = MES[ch].addit_unit_ov.UINT64;
		MES[ch].addit_buff_ov.UINT64 = get_total_offset(ch, MES[ch].addit_buff_ov.UINT64); //�ώZ�l�I�t�Z�b�g
		/*�ώZ�Ď��@�\*/
		if(MES[ch].addit_watch == B_ON){		//�ώZ�Ď��L��
			if(MES[ch].addit_unit_ov.UINT64 >= (unsigned long long)SVD[ch].target_total.INT32 * 100000){
				MES[ch].total_status |= TTL_JUDGE_REACH; //�ώZ���B�o�̓Z�b�g
			}
		}else{		//�ώZ�Ď�����
			MES[ch].total_status &= ~TTL_JUDGE_REACH; //�ώZ���B�o�̓��Z�b�g
		}
	}
}

/****************************************************/
/* Function : reverse_flow_check                    */
/* Summary  : �t���ُ픻��    				*/
/* Argument : ch                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	reverse_flow_check(short ch){

	short pv_now_work;
	short	rev_level;

	if(MES[ch].ThresholdReq != 0) return;	/*�g�`�F�����s���̓`�F�b�N���Ȃ�*/

	pv_now_work = (short)((MES[ch].ml_min_d * 100) / (MES[ch].max_flow_long));/*�u������(%) (100.00%��10000�ŕ\��)*/
	rev_level = SVD[ch].reverse_level;		/*�t�����肵�����l(0�`100%)*/
	rev_level *= 100;										/*�������킹(100%��10000�ŕ\��)*/

	if(pv_now_work <= 0){											/*�}�C�i�X�u�����ʎ�*/
		if(abs(pv_now_work) >= abs(rev_level)){			/*�t���ُ픻��*/
			MES[ch].err_status |= ERR_JUDGE_REVERSE;	/*�G���[�Z�b�g*/
		}else{
			MES[ch].err_status &= ~ERR_JUDGE_REVERSE;	/*�G���[���Z�b�g*/
		}
	}else{																			/*�v���X�u�����ʎ�*/
		MES[ch].err_status &= ~ERR_JUDGE_REVERSE;		/*�G���[���Z�b�g*/
	}
}

/****************************************************/
/* Function : test_flow_mode                        */
/* Summary  : �e�X�g�o�̓��[�h    				*/
/* Argument : ch                                    */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
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
/* Summary  : �������[�h    				*/
/* Argument : ch                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : �Ȃ�                                   */
/****************************************************/
void	inspect_flow_mode(short ch){

	short i, work, max, min;

	FPGA_SYNC = 0;
	WriteGainAmp((unsigned short)255);  /*�Q�C��������*/
	OutputRestartPulse();			/*RESTART�p���X�o��*/

	/*REV*/	
	SelectReverseOn(ch);		//IN/OUT�ō��ݐؑւ�
	delay(50);					/*�ؑ֌�A5usec�ҋ@*/
	OutputRestartPulse();		/*RESTART�p���X�o��*/

	MES[ch].fifo_start = 0;	/*WINDOW�J�n���Ԃ�ݒ�*/
	MES[ch].fifo_end = 1;	/*WINDOW�I�����Ԃ�ݒ�*/
	MES[ch].fifo_offset = 0; /*WINDOW�I�t�Z�b�g���Ԃ�ݒ�*/

	if(initializing == 0){   //�N�����̏������������I�����Ă���ꍇ
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}

	SetFpgaRegister(ch);   //FPGA���W�X�^�̐ݒ�
	OutputStartPulse();		/*START�p���X�o��*/

	fifo_read_wait(ch);  /*��g�f�[�^�Ǎ��ݑҋ@*/

	/*FIFO�Ǎ���(�_�~�[���[�h)*/
	work = FIFO;  /*�ŏ���2word = 0 �ɐ����Ă�*/
	work = FIFO;  /*2��A�_�~�[���[�h����B   */

	max = 0;
	min = AD_MAX;
	for(i = 0; i < 100; i++) {
		work = FIFO;
		if(work > max) max = work;
		if(work < min) min = work;
	}
	wave_hight[0][ch] = max - min;

	// GPIO_PJ1
	while(__bit_input(GPIO_PORTJ_BASE, 1) == 1);		// Window���I���܂ő҂�
	
	SelectForwardOff(ch);		//IN/OUT�ō��ݐؑւ�
	delay(50);					/*�ؑ֌�A5usec�ҋ@*/

	/*FWD*/	
	SelectReverseOn(ch);		//IN/OUT�ō��ݐؑւ�
	delay(50);					/*�ؑ֌�A5usec�ҋ@*/
	OutputRestartPulse();		/*RESTART�p���X�o��*/

	MES[ch].fifo_start = 0;	/*WINDOW�J�n���Ԃ�ݒ�*/
	MES[ch].fifo_end = 1;	/*WINDOW�I�����Ԃ�ݒ�*/
	MES[ch].fifo_offset = 0; /*WINDOW�I�t�Z�b�g���Ԃ�ݒ�*/

	if(initializing == 0){   //�N�����̏������������I�����Ă���ꍇ
		GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_6);
		IntPendClear(INT_GPIOP6);
		IntEnable(INT_GPIOP6);
	}

	SetFpgaRegister(ch);   //FPGA���W�X�^�̐ݒ�
	OutputStartPulse();		/*START�p���X�o��*/

	fifo_read_wait(ch);  /*��g�f�[�^�Ǎ��ݑҋ@*/

	/*FIFO�Ǎ���(�_�~�[���[�h)*/
	work = FIFO;/*�ŏ���2word = 0 �ɐ����Ă�*/
	work = FIFO;/*2��A�_�~�[���[�h����B   */

	max = 0;
	min = AD_MAX;
	for(i = 0; i < 100; i++) {
		work = FIFO;
		if(work > max) max = work;
		if(work < min) min = work;
	}
	wave_hight[1][ch] = max - min;

	// GPIO_PJ1
	while(__bit_input(GPIO_PORTJ_BASE, 1) == 1);		// Window���I���܂ő҂�

	SelectReverseOff(ch);		//IN/OUT�ō��ݐؑւ�
	delay(50);					/*�ؑ֌�A5usec�ҋ@*/
}

/****************************************************/
/* Function : InitFifoCh                           */
/* Summary  : FIFO CH������    				*/
/* Argument : ch , val                           */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* notes    : ���a�ύX���ɍX�V����                      */
/****************************************************/
void	InitFifoCh(short ch, short val){

//	SVD[ch].fifo_ch_init = sens_inf[val].fifo_ch;
}

/****************************************************/
/* Function : err_thlevel_init                */
/* Summary  : �g�`�ُ픻��臒l�̏�����             */
/* Argument : ch,   val                        		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
/* note     : ���a�ύX���ɍX�V����        */
/****************************************************/
void	err_thlevel_init(short ch, short val){
	
//	SVD[ch].wave_vth = sens_inf[val].wave_vth;					/*��g���o臒l*/
//	SVD[ch].balance_level = sens_inf[val].balance_level;		/*�g�`�A���o�����X���o臒l*/
//	SVD[ch].saturation_level = sens_inf[val].saturation_level;	/*�g�`�O�a���o臒l*/
//	SVD[ch].correlate_level	= sens_inf[val].correlate_level;	/*�������֔�臒l*/
//	SVD[ch].correlate_time 	= sens_inf[val].correlate_time;		/*���Z�ُ픻���*/
//	SVD[ch].attenuate_level = sens_inf[val].attenuate_level;	/*�g�`����臒l*/
}

/****************************************************/
/* Function : current_flow_control                */
/* Summary  : �u�����ʒl�̍X�V             */
/* Argument : ch                               		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
/* note     : �G���[�������͏u�����ʒl���X�V���Ȃ�         */
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
		MES[ch].ml_min_now = MES[ch].ml_min_d; // �u�����ʂ��X�V
	}
	else
	{
		// Debug

		if ((MES[ch].err_status & ERR_FLOW_CONT) == 0)
		{										   // ���ʃG���[�Ȃ�
			MES[ch].ml_min_now = MES[ch].ml_min_d; // �u�����ʂ��X�V
		}
		else
		{	  // ���ʃG���[����
			; // �u�����ʂ��X�V���Ȃ��i���ʂ��z�[���h����j
		}
	}
}

/****************************************************/
/* Function : check_amp_gain                 */
/* Summary  : �A���v�Q�C���`�F�b�N               */
/* Argument : ch                  			*/
/* Return   : �Ȃ�		                  */
/* Caution  : �Ȃ�                               */
/* note     : �\�m�ۑS�@�\                           */
/****************************************************/
void	check_amp_gain(short ch){

	short cnt, num, gain_diff, cyc_num;

	cyc_num = 4;	//�Q�C���l�擾�����F4��Œ�

	if(SVD[ch].alm_hold_time == 0){		//���莞�Ԑݒ�Ȃ��i�\�m�ۑS�@�\�̖������j
		MES[ch].alm_status &= ~(ALM_JUDGE_GAIN);	//�Q�C���l�}�όx�����Z�b�g
		return;
	}

	get_cyc[ch]++;		//�Q�C���l�擾�����X�V
	if(get_cyc[ch] < cyc_num){		//�Q�C���l�擾�����m�F
		return;
	}
	get_cyc[ch] = 0;	//�Q�C���l�擾�����N���A

	for(cnt=AMP_BUFF-1; cnt>0; cnt--){
		MES[ch].amp_gain_change[cnt] = MES[ch].amp_gain_change[cnt-1];	//�Q�C�����L�����V�t�g
	}

	if(((MES[ch].err_status & ERR_JUDGE_EMPTY) == 0) &&
		((MES[ch].amp_gain_for - MES[ch].amp_gain_old) >= SVD[ch].alm_gain_level)){	//�Q�C���l�̍����A���v�Q�C���x��臒l�ȏ�̏ꍇ
		MES[ch].amp_gain_change[0] = B_ON;		//�Q�C��������
	}else{
		MES[ch].amp_gain_change[0] = B_OFF;		//�Q�C�����Ȃ�
	}

	gain_diff = 0;
	num = (short)((SVD[ch].alm_hold_time * 10) / (18 * cyc_num)) + 1;
	if(num >= AMP_BUFF){
		num = AMP_BUFF - 1;
	}
	for(cnt=0; cnt<num; cnt++){		//���莞�ԓ��̃Q�C�������m�F����
		if(MES[ch].amp_gain_change[cnt] == B_ON){		//�Q�C��������H
			gain_diff++;
		}
	}
	if(gain_diff >= SVD[ch].alm_gain_count){	//�Q�C���x�����Ĉȏ�H
		MES[ch].alm_status |= ALM_JUDGE_GAIN;		//�Q�C���l�}�όx���Z�b�g
	}else{
		MES[ch].alm_status &= ~(ALM_JUDGE_GAIN);	//�Q�C���l�}�όx�����Z�b�g
	}

	MES[ch].amp_gain_old = MES[ch].amp_gain_for;		//�Q�C���l��ێ�
}

/****************************************************/
/* Function : flow_save_control                     */
/* Summary  : �u������(10�_)�ۑ�        				*/
/* Argument : �Ȃ�                                   */
/* Return   : �Ȃ� 									                         */
/* Caution  : �Ȃ�                                   */
/* note     : �����(5msec)�����ݏ���                 */
/****************************************************/
void	flow_save_control(void){

	short		ch;
	short		i_cnt;
	short		past_cnt;
	
	for(ch = CH1; ch < CH_NUMMAX; ch++){
		past_cnt = MES[ch].past_flow_cnt;
		if(past_cnt < 10){		//�o�b�t�@��10�_�Ȃ�
			if(past_cnt==0){								//10�_�f�[�^�ۑ��J�E���^�N���A��
				memset(&MES[ch].past_flow[0], 0, sizeof(long)*10);		//�o�b�t�@�N���A
				memset(&MES[ch].past_flow_oq[0], 0, sizeof(long)*10);
			}
			MES[ch].past_flow[past_cnt] = MES[ch].ml_min_OQ;			/*�u�����ʕۑ�(OQ)*/
			MES[ch].past_flow_oq[past_cnt] = MES[ch].ml_min_oq;		/*�u�����ʕۑ�(oq)*/
			if(past_cnt==9){	//�o�b�t�@��10�_����
				MES[ch].err_status &= ~ ERR_JUDGE_10POINT;		//10�_�f�[�^�����G���[���Z�b�g
			}
			MES[ch].past_flow_cnt++;							//10�_�f�[�^�ۑ��J�E���^�C���N�������g
		}else{					//�o�b�t�@��10�_����
			memcpy(&MES[ch].past_flow[0], &MES[ch].past_flow[1], sizeof(long)*9);	//�o�b�t�@�X�V
			memcpy(&MES[ch].past_flow_oq[0], &MES[ch].past_flow_oq[1], sizeof(long)*9);

			MES[ch].past_flow[9] = MES[ch].ml_min_OQ;			/*�u�����ʕۑ�(OQ)*/
			MES[ch].past_flow_oq[9] = MES[ch].ml_min_oq;		/*�u�����ʕۑ�(oq)*/
			MES[ch].err_status &= ~ ERR_JUDGE_10POINT;		//10�_�f�[�^�����G���[���Z�b�g
		}
	}
}

/****************************************************/
/* Function : filter_moving                 */
/* Summary  : �ړ����Ϗ��� */
/* Argument : �Ȃ�                  			*/
/* Return   : �Ȃ�		                  */
/* Caution  : �Ȃ�                               */
/* note     : �Ȃ�                                   */
/****************************************************/
void filter_moving(short pch)
{
 short i;
	if(SVD[pch].filter_mode == FILTER_DIRECT){		//�t�B���^�����ݒ莞
		MES[pch].delta_ts = MES[pch].delta_ts0;
	}else if(SVD[pch].filter_mode == FILTER_MOVING){		//�ړ�����(1-50��)�ݒ莞
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
	}else{		//���f�B�A���t�B���^�ݒ莞
		/*�ړ�����(4��)*/
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
 * Summary  : �[���N���X�p�ړ����Ϗ���
 * Argument : �Ȃ�
 * Return   : �Ȃ�
 * Caution  : �Ȃ�
 * note     : �f�B�U�����O�@�\�̂���4��ړ����ς������Ă���B
 *          : 0��, 90��, 180��, 270���V�t�g�g�`�̕��ρB
 ****************************************************/
void zc_filter_moving(short pch)
{
	//�f�B�U�����O�@�\���� or �t�B���^�Ȃ�
	if(((SVD[pch].fix_data & 0x01) == 0) || (SVD[pch].filter_mode == FILTER_DIRECT))
	{
		MES_SUB[pch].zc_delta_ts = MES[pch].zc_Tdata;
	}
	//�f�B�U�����O�@�\�L�� && (���f�B�A���t�B���^ or �ړ����� �ݒ莞)
	else
	{
		//�ړ�����(4��)
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
/* Summary  : FPGA�̏����� */
/* Argument : �Ȃ�                 	           	*/
/* Return   : �Ȃ�									                     */
/* Caution  : �Ȃ�                               */
/* note     : �Ȃ�                              */
/****************************************************/
void	InitFPGA(void)
{
	__bit_output(GPIO_PORTM_BASE, 7, 1);   //XRESET_FPGA ON
}

/****************************************************/
/* Function : WaitCDONE                     */
/* Summary  : CDONE�M���̑ҋ@         */
/* Argument : �Ȃ�                            		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
/* note     : FPGA�̋N���ҋ@                          */
/****************************************************/
void WaitCDONE(void)
{
	while(__bit_input(GPIO_PORTJ_BASE, 3) == 0);  //CDONE�҂�
	delay(200);
	__bit_output(GPIO_PORTM_BASE, 7, 1);   //XRESET_FPGA (FPGA������)
	delay(20000);
}

/****************************************************/
/* Function : SetADCClock                  */
/* Summary  : ADC�����N���b�N��ݒ肷�� */
/* Argument : short pch        		*/
/* Return   : �Ȃ�									           */
/* Caution  : �Ȃ�                               */
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
/* Summary  : �쓮���g����ݒ肷�� */
/* Argument : short ch            		*/
/* Return   : �Ȃ�									           */
/* Caution  : �Ȃ�                               */
/* note     : 100kHz�`5MHz �� 1280�`26 @SVD[ch].adc_clock = 0   */
/*          :                1600�`32 @SVD[ch].adc_clock = 1   */
/*          :                2600�`52 @SVD[ch].adc_clock = 2   */
/*          :                3200�`64 @SVD[ch].adc_clock = 3   */
/****************************************************/
void	SetDriveFreq(short ch)
{
	short data, freq;
	double AdcClk[4] = {128.0, 160.0, 260.0, 320.0};
	double DblDat = 0.0;
	short RgsMinVal[4] = {26, 32, 52, 64};
	short RgsMaxVal[4] = {1280, 1600, 2600, 3200};

#ifdef FRQSCH
	//���g���T�[�`���[�h�L��
	if(SVD[ch].drive_search != 0){
		//���g���T�[�`��
		if(FrqSch[ch].FrqSchSttFlg == 1){
		// if(FrqSch[ch].FrqSchSttFlg != 0){
			freq = FrqSch[ch].NowFrq;
		}
		else{
			freq = SVD[ch].SchFrq;  //�쓮���g��(100kHz�`5MHz)
		}
	}
	//���g���T�[�`���[�h����
	else{
		freq = SVD[ch].drive_freq;  //�쓮���g��(100kHz�`5MHz)
	}
#else
	freq = SVD[ch].drive_freq;  //�쓮���g��(100kHz�`5MHz)
#endif
	           //����           /  �J�E���g����
	// data = ((1000000 / freq) / (1000 / AdcClk[SVD[ch].adc_clock]));
	DblDat = 1000000.0 * AdcClk[SVD[ch].adc_clock] / (double)freq / 1000.0;
	data = (short)(DblDat + 0.5);
	
	if((RgsMinVal[SVD[ch].adc_clock] <= data) && (data <= RgsMaxVal[SVD[ch].adc_clock])){
		FPGA_FREQ = data;  //�쓮���g��
	}
}

/****************************************************/
/* Function : SetDrivePulse                  */
/* Summary  : �쓮�p���X����ݒ� */
/* Argument : short pch        		*/
/* Return   : �Ȃ�									           */
/* Caution  : �Ȃ�                               */
/* note     : 1-15          */
/****************************************************/
void	SetDrivePulse(short pch)
{
	short data;

	data = SVD[pch].drive_pls;  //�쓮�p���X(1-15)

	if((1 <= data) && (data <= 15)){
		FPGA_PULSE = data;
	}
}

/****************************************************/
/* Function : SetWindowPosition                  */
/* Summary  : �Ǐo��Window�J�n/�I��/�I�t�Z�b�g��ݒ肷�� */
/* Argument : short pch         		*/
/* Return   : �Ȃ�									           */
/* Caution  : WINDOW_WET �� WINDOWWST �����傫���l��ݒ肷��K�v������ */
/*          : WINDOW_WET = WINDOWWST �̏ꍇ�AWINDOWWET = WINDOW_WST+1 �œ������䂷�� */
/* note     : Window��~����[��s] = WINDOW_WET�ݒ�l * ADC_CLK *256 */
/****************************************************/
void	SetWindowPosition(short pch)
{
	short data;

	/*WINDOW�J�n���Ԃ�ݒ�*/
	data = MES[pch].fifo_start;
	if((0 <= data) && (data <= 63)){
		FPGA_START = data;
	}	

	/*WINDOW�I�����Ԃ�ݒ�*/
	//WINDOW_WET > WINDOWWST �ƂȂ�悤����
	if(MES[pch].fifo_end < MES[pch].fifo_start){
		data = MES[pch].fifo_start;
	}
	else{
		data = MES[pch].fifo_end;
	}
	if((0 <= data) && (data <= 63)){
		FPGA_END = data;
	}	

	/*�Ǐo���J�n�I�t�Z�b�g��ݒ�*/
	data = MES[pch].fifo_offset;
	if((0 <= data) && (data <= 63)){
		FPGA_OFFSET = data;
	}
}

/****************************************************/
/* Function : SetClockPhase                  */
/* Summary  : �ʑ��p���X��ݒ� */
/* Argument : short pch        		*/
/* Return   : �Ȃ�									           */
/* Caution  : �Ȃ�                               */
/* note     : 0-3          */
/****************************************************/
void	SetClockPhase(short pch)
{
	short data;

	if((SVD[pch].fix_data & 0x01) == 0){  //�f�B�U�����O����
		data = 0;  //�ʑ�������(�Œ�)
	}else{  //�f�B�U�����O�L��
		data = MES[pch].clk_phase;
	}

	if((0 <= data) && (data <= 3)){
		FPGA_SYNC = data;
	}
}

/****************************************************/
/* Function : SetFpgaRegister                      */
/* Summary  : FPGA���W�X�^�̐ݒ�                       */
/* Argument : pch                              		 */
/* Return   : �Ȃ�									                         */
/* Caution  : �Ȃ�                                  */
/* note     : �쓮���g��, �쓮�p���X��, ADC�����N���b�N    */
/*          : �Ǐo���J�n�I�t�Z�b�g, WINDOW�J�n/�I������, �ʑ��p���X*/
/****************************************************/
void	SetFpgaRegister(short pch){

	/*ADC�����N���b�N��ݒ�*/
	SetADCClock(pch);

	/*�쓮���g����ݒ�*/
	SetDriveFreq(pch);

	/*�쓮�p���X����ݒ�*/
	SetDrivePulse(pch);

	/*Window�J�n/�I��/�I�t�Z�b�g��ݒ�*/
	SetWindowPosition(pch);

	/*�ʑ���ݒ�*/
	SetClockPhase(pch);

}

/****************************************************/
/* Function : SetDigitalFilterRegister              */
/* Summary  : �f�W�^���t�B���^�W�����W�X�^�̐ݒ�               */
/* Argument : �Ȃ�                              		 */
/* Return   : �Ȃ�						            */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�                                   */
/****************************************************/
void	SetDigitalFilterRegister(void){

	FPGA_FIL_EN = SVD[0].DgtFltSwc;	/* �f�W�^���t�B���^�؂�ւ�(0-1) */ 
	FPGA_FILIN0_0 = SVD[0].DgtFltCefA00;	/* �f�W�^���t�B���^�W��(���͑�0) (����1bit+����17bit) */
	FPGA_FILIN0_1 = SVD[0].DgtFltCefA01;	/* �f�W�^���t�B���^�W��(���͑�0) (����1bit+����17bit) */
	FPGA_FILIN1_0 = SVD[0].DgtFltCefA10;	/* �f�W�^���t�B���^�W��(���͑�1) (����1bit+����17bit) */
	FPGA_FILIN1_1 = SVD[0].DgtFltCefA11;	/* �f�W�^���t�B���^�W��(���͑�1) (����1bit+����17bit) */
	FPGA_FILIN2_0 = SVD[0].DgtFltCefA20;	/* �f�W�^���t�B���^�W��(���͑�2) (����1bit+����17bit) */
	FPGA_FILIN2_1 = SVD[0].DgtFltCefA21;	/* �f�W�^���t�B���^�W��(���͑�2) (����1bit+����17bit) */
	FPGA_FILOUT1_0 = SVD[0].DgtFltCefB10;	/* �f�W�^���t�B���^�W��(�o�͑�1) (����1bit+����17bit) */
	FPGA_FILOUT1_1 = SVD[0].DgtFltCefB11;	/* �f�W�^���t�B���^�W��(�o�͑�1) (����1bit+����17bit) */
	FPGA_FILOUT2_0 = SVD[0].DgtFltCefB20;	/* �f�W�^���t�B���^�W��(�o�͑�2) (����1bit+����17bit) */
	FPGA_FILOUT2_1 = SVD[0].DgtFltCefB21;	/* �f�W�^���t�B���^�W��(�o�͑�2) (����1bit+����17bit) */
}

/****************************************************/
/* Function : WriteGainAmp                         */
/* Summary  : �Q�C��������(TLV5623CDGK)               */
/* Argument : short gain                           */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                  */
/* notes    : DAC_SDI      PORTD BIT1              */
/*          : DAC_CLK      PORTD BIT3              */
/*          : DAC_CS_VDBS  PORTD BIT2              */
/*          : DAC_FS       PORTE BIT2              */
/* b15 14  13  12 11 10 9 8 7 6 5 4 3 2 1 0        */
/*  X  SPD PWR X  <- value(8bit) -> 0 0 0 0        */
/*  SPD(Speed control):1��fast mode  0��slow mode    */
/*  PWR(Power control):1��power down 0��normal operation*/
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
 * Summary  : ���g���T�[�`������
 * Argument : pch
 * Return   : 0 -> ���g���T�[�`���łȂ�
 *          : 1 -> ���g���T�[�`��
 * Caution  : �Ȃ�
 * note     : FrqSch.FrqSchSttFlg �̒l
 *          : 0 : ���g���T�[�`�J�n�O
 *              :   �e��ϐ�������������
 *          : 1 : ���g���T�[�`��(���蒆)
 *              :   �݌v10��g�`�擾�Ƀg���C����
 *              :   5��ȏ�g�`�ُ킪�N����Ύ��̎��g���Ɉڍs
 *              :   10�񕪂̐U���ő�l�𕽋ς��Ă��̎��g���̍ő�U���Ƃ���
 *              :   �u�ԓI�ɂ܂��ꂽ�U���̑傫�Ȕg�`�̉e���͕��ω��ŏ��Ȃ��Ȃ�͂�
 *          : 10~ : ���g���T�[�`��(�ҋ@��)
 *                :   ���g���ύX��g�`�����肷��܂ł̑ҋ@
 *          : 2 : ���g���T�[�`�I�� & �[�����J�n
 *              :   ���g���T�[�`�I����Ƀ[���������{����
 *              :   �[�������N�G�X�g����FrqSchSttFlg��0�ɖ߂�
 ***************************************************************************/
short FrqSchPrc(short pch){
	short i;
	short MinAmp = AD_MAX; //�ŏ��U���ꎞ�ϐ�
	short MaxAmp = 0; //�ő�U���ꎞ�ϐ�
	short FrqSchNowFlg = 0; //���g���T�[�`���ꎞ�t���O
	short AmpMesCnt = 0; //�U�������
	long TmpAmpAvg = 0; //�ő�U�����ϒl�ꎞ�ϐ�
	short ErrFlg = 0; //�g�`�ُ��

	AmpMesCnt = sizeof(FrqSch[pch].MaxAmpLst) / sizeof(short);

	//���g���T�[�`���łȂ�
	if(FrqSch[pch].FrqSchSttFlg == 0){
		FrqSchNowFlg = 0; //���ʍX�V����

		//�ꎞ�ϐ�������
		FrqSch[pch].MaxAmpAvg = 0;
		for(i = 0; i < AmpMesCnt; i++){
			FrqSch[pch].MaxAmpLst[i] = 0;
		}
		FrqSch[pch].NowFrq = SVD[pch].start_freq; //����J�n���g���ɍX�V
		
		FrqSch[pch].MesAmpCnt = 0; //�g�`����񐔃N���A
		FrqSch[pch].MesErrCnt = 0; //�g�`�G���[�J�E���g�N���A
		FrqSch[pch].WitTim = 30; //�ҋ@���ԏ�����
		FrqSch[pch].CenPntUndLim = 10; //���S���ʉ߉񐔉����l
		FrqSch[pch].CenPntOvrLim = 20; //���S���ʉ߉񐔏���l
		FrqSch[pch].MaxAmpFrq = SVD[pch].SchFrq;
	}
	//���g���T�[�`��
	else if(FrqSch[pch].FrqSchSttFlg == 1){
		FrqSchNowFlg = 1; //���ʍX�V���Ȃ�

		if(SVD[pch].drive_search == 0){
			//���g���T�[�`�g�p���Ȃ��ݒ�̏ꍇ�͏I��
			FrqSch[pch].FrqSchSttFlg = 0;
			FrqSchNowFlg = 0;
		}
		//�ُ�g�`(�I�[�o�[�t���[�ȊO�̗��ʃG���[)
		// else if((MES[pch].err_status & (ERR_FLOW_CONT & ~(ERR_JUDGE_OVERFLOW))) != 0){
		else if((MES[pch].err_status & (ERR_BURN_OUT & ~(ERR_JUDGE_OVERFLOW))) != 0){
		// else if((MES[pch].err_status & (ERR_FLOW_CONT)) != 0){
			FrqSch[pch].MesErrCnt++; //�g�`�G���[�J�E���g
			FrqSch[pch].MaxAmpLst[FrqSch[pch].MesAmpCnt] = 0;
		}
		//5��ُ�g�`���o�Ŏ�
		else if(FrqSch[pch].MesErrCnt >= 5){
			FrqSch[pch].MesErrCnt = 0; //�G���[���o�񐔃N���A
			FrqSch[pch].NowFrq += 3; //3kHz���������Ď���
			FrqSch[pch].MesAmpCnt = 0; //����񐔃N���A
		}
		//����g�`
		else{
			FrqSch[pch].CenPntCnt = 0;
#if defined(FLWWAVEXP)
			for(i = 10; i < FLWWAVSIZ + 10; i++)
#else
			for(i = 10; i < 250; i++)
#endif
			{
				/*�ŏ��l��T��*/
				if( MES[pch].fow_data[i] < MinAmp){	
					/*fow data max*/
					MinAmp = MES[pch].fow_data[i];
				}
				/*�ő�l��T��*/
				if( MaxAmp < MES[pch].fow_data[i]){	
					/*fow data max*/
					MaxAmp = MES[pch].fow_data[i];
				}
				
				//�g�`�����S��(2047)��ʉ߂���񐔂��J�E���g(����g�`����p)
				if(((MES[pch].fow_data[i - 1] - AD_BASE) * (MES[pch].fow_data[i] - AD_BASE)) < 0){
					FrqSch[pch].CenPntCnt++;
				}

			}
			//���S���ʉ߉񐔂�3~15������ƒ�`
			if((FrqSch[pch].CenPntCnt < FrqSch[pch].CenPntUndLim) || (FrqSch[pch].CenPntOvrLim < FrqSch[pch].CenPntCnt)){
				FrqSch[pch].MesErrCnt++;
				ErrFlg++;
			}

			//�ő�U���X�V
			if(ErrFlg == 0){
				FrqSch[pch].MaxAmpLst[FrqSch[pch].MesAmpCnt] = MaxAmp - MinAmp;
			}
			else{
				FrqSch[pch].MaxAmpLst[FrqSch[pch].MesAmpCnt] = 0;
			}
		}
		FrqSch[pch].MesAmpCnt++;
		if(AmpMesCnt <= FrqSch[pch].MesAmpCnt){
				
			// �g�`�G���[����x���N���Ă��Ȃ��Ƃ��Ɏ��g�����X�V����
			if(FrqSch[pch].MesErrCnt == 0){
				//���̎��g���ł̐U���ő�l�̕��ϒl���v�Z
				for(i = 0; i < AmpMesCnt; i++){
					TmpAmpAvg += FrqSch[pch].MaxAmpLst[i];
					FrqSch[pch].MaxAmpLst[i] = 0; //�z�񃊃Z�b�g
				}
				TmpAmpAvg /= AmpMesCnt;

				//�ő�U���X�V����
				if(TmpAmpAvg > FrqSch[pch].MaxAmpAvg){
					FrqSch[pch].MaxAmpAvg = (short)TmpAmpAvg;
					FrqSch[pch].MaxAmpFrq = FrqSch[pch].NowFrq;
				}
			}
			FrqSch[pch].MesAmpCnt = 0; //���̎��g���ł̑���͏I��
			FrqSch[pch].NowFrq += 3; //3kHz���������Ď���
			FrqSch[pch].MesErrCnt = 0; //�g�`�G���[�J�E���g�N���A

			FrqSch[pch].FrqSchSttFlg = 10;
		}
	}
	//�g�`�����肷��܂ő҂� 18ms/6ch * 10 = 180ms���炢
	else if(FrqSch[pch].FrqSchSttFlg >= 10){
		FrqSch[pch].FrqSchSttFlg++;
		// if(FrqSch[pch].FrqSchSttFlg > 20){
		if(FrqSch[pch].FrqSchSttFlg > FrqSch[pch].WitTim){
			FrqSch[pch].FrqSchSttFlg = 1;
		}
	}
	//�[��������
	else{
		;
	}
	
	//���g���T�[�`�I������
	if((FrqSch[pch].FrqSchSttFlg == 1) && (FrqSch[pch].NowFrq > SVD[pch].stop_freq)){
		FrqSch[pch].FrqSchSttFlg = 2; //���g���T�[�`�I�� & �[�������Ɉڍs

		//������g���X�V
		SVD[pch].SchFrq = FrqSch[pch].MaxAmpFrq;
		//EEPROM�ɏ�������
		// eep_write_ch_delay(pch, (short)(&SVD[pch].SchFrq - &SVD[pch].max_flow), SVD[pch].SchFrq);
	}

	return FrqSchNowFlg;
}
#endif

/****************************************************/
/* Function : SetNextGain                     */
/* Summary  : ��CH�̃Q�C���l��ݒ肷��         */
/* Argument : pch                            		 */
/* Return   : �Ȃ�                                  */
/* Caution  : �Ȃ�                                  */
/* note     : �Ȃ�                                 */
/****************************************************/
void SetNextGain(short pch){

	short cnt;
	short next_pch;

	next_pch = pch + 1;  //��CH
	for(cnt=0; cnt<CH_NUMMAX; cnt++){
		if(next_pch >= CH_NUMMAX){	
			next_pch = CH1;
		}
		if(SVD[next_pch].sensor_size != SNS_NONE){  //�Z���T��NONE�ݒ�Ȃ�Ύ�CH�ɂ���
			break;
		}else{
			next_pch++;
		}
	}

	WriteGainAmp((unsigned short)MES[next_pch].amp_gain_rev);  /*��CH�̃Q�C���l��ݒ肷��*/
}	

/****************************************************
 * Function : ChkClcActFlg
 * Summary  : ���ʉ��Z������`����, SFC9100�ł̓[���N���X���Z�Œ�̂��ߖ��g�p
 * Argument : pch : �`�����l���ԍ�
 * Return   : void
 * Caution  : ����ύX�̍ۂ͂������ύX����
 * note     : �ȉ��A���[���̓��e
 * �E���p��/FIFO offset �̐ݒ�
 *�@�O �F ��������
 *�@1 �F �擪�ʒu���[�����ߎ�����ړ������Ȃ��@ 
 *�@2 �F �������֓_���𑝉�������
 *�@�@�@(10�`210�̃f�[�^��1��΂���100word�����������Z����)
 *�@3 �F 1 �� 2 ���L��
 *�@4 �F �[���N���X���Z �@(�[���N���X7�_)
 *�@5 �F �[���N���X���Z �� 1���L���@(�[���N���X7�_)
 *�@6 �F �[���N���X���Z�@(�[���N���X12�_)
 *�@7 �F �[���N���X���Z �� 1���L�� (�[���N���X12�_)
 *�@8 �F �[���N���X���Z �@(�[���N���X7�_�A�[���N���X�O��2�_)
 *�@9 �F �[���N���X���Z �� 1���L���@(�[���N���X7�_�A�[���N���X�O��2�_)
 *�@10 �F �[���N���X���Z�@(�[���N���X12�_�A�[���N���X�O��2�_)
 *�@11 �F �[���N���X���Z �� 1���L�� (�[���N���X12�_�A�[���N���X�O��2�_)
 ****************************************************/
void ChkClcActFlg(short pch) {
	switch (SVD[pch].wind_offset)
	{
	case 0:
		ClcActFlg = ClcAct_SumAdd; //��������
		// TopPosFix = PosFix_Mov; //�擪�ʒu���[�����ߎ�����ړ�������
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	case 1:
		ClcActFlg = ClcAct_SumAdd; //��������
		// TopPosFix = PosFix_NotMov; //�擪�ʒu���[�����ߎ�����ړ������Ȃ�
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	case 2:
		ClcActFlg = ClcAct_SumAdd; //��������
		// TopPosFix = PosFix_Mov; //�擪�ʒu���[�����ߎ�����ړ�������
		SumPntInc = SumPnt_Inc; //�������֓_���𑝉�������
		break;
	case 3:
		ClcActFlg = ClcAct_SumAdd; //��������
		// TopPosFix = PosFix_NotMov; //�擪�ʒu���[�����ߎ�����ړ������Ȃ�
		SumPntInc = SumPnt_Inc; //�������֓_���𑝉�������
		break;
	case 4:
	case 6:
		ClcActFlg = ClcAct_ZerCrs; //�[���N���X
		// TopPosFix = PosFix_Mov; //�擪�ʒu���[�����ߎ�����ړ�������
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	case 5:
	case 7:
		ClcActFlg = ClcAct_ZerCrs; //�[���N���X
		// TopPosFix = PosFix_NotMov; //�擪�ʒu���[�����ߎ�����ړ������Ȃ�
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	case 8:
	case 10:
		ClcActFlg = ClcAct_ZerCrs; //�[���N���X
		// TopPosFix = PosFix_Mov; //�擪�ʒu���[�����ߎ�����ړ�������
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	case 9:
	case 11:
		ClcActFlg = ClcAct_ZerCrs; //�[���N���X
		// TopPosFix = PosFix_NotMov; //�擪�ʒu���[�����ߎ�����ړ������Ȃ�
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	default:
		ClcActFlg = ClcAct_SumAdd; //��������
		// TopPosFix = PosFix_Mov; //�擪�ʒu���[�����ߎ�����ړ�������
		SumPntInc = SumPnt_NotInc; //�������֓_���𑝉������Ȃ�
		break;
	}
}

/****************************************************/
/* Function : SearchWindow                        */
/* Summary  : Window�T�[�`                   */
/* Argument : pch                              		 */
/* Return   : 0:����, -1:�ُ�	                  */
/* Caution  : �Ȃ�                                  */
/* note     : Window�T�[�`�Ŏg�p����Q�C���l���擾���Ă���
 *            �œK��FIFO CH����������
 *          : 
 *          : MES[].ThresholdReq -> 0  : �g�`�F�����s�v���Ȃ�
 *          :                    -> 1  : Window�T�[�`�p�̃Q�C���l�����v��
 *          :                    -> 2  : Window�T�[�`�v��
 *          :                    -> 11 : �g�`�F�����s�v��
 *          :                    -> 12 : �ڍ׃T�[�`�̏ꍇ
 *          :                    -> 99 : �g�`�F�������̏I��(LED.zeroactive=1�̎��Azero_adj_control()����0�Ɉڍs����)
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
		if(SVD[pch].sensor_size == 3){ // �Z���T���a:1/4"
			fifo_start = WS_FIFO_START_14;
		}else{ 		// �Z���T���a:3/8"
			fifo_start = WS_FIFO_START_38;
		}
		fifo_end = WS_FIFO_END;
	}else{  //0=32MHz, 1=40MHz, 3=80MHz(������)
		if(SVD[pch].sensor_size == 3){ // �Z���T���a:1/4"
			fifo_start = WS_FIFO_START_14_3240;
		}else{ 		// �Z���T���a:3/8"
			fifo_start = WS_FIFO_START_38_3240;
		}
		fifo_end = WS_FIFO_END_3240;
	}
	
	//Window�T�[�`�p�̃Q�C���l����
	MES[pch].ThresholdReq = 1;	/*Window�T�[�`�p�̃Q�C���l�����v��*/
	SVD[pch].fix_data |= 0x0C;	//FIFO CH��10�`30�ŌŒ肷�邽��
	SVD[pch].fix_fifo_no_read = 0;
	for(fifo_ch=fifo_start; fifo_ch<fifo_end; fifo_ch++){	//FIFO CH(15�`30)�̃Q�C���l���擾����
		SVD[pch].fix_fifo_ch_read = fifo_ch;
		//6ms x 40 = 240ms�ҋ@
		for(cnt=1; cnt<40; cnt++){
			//6553us=6ms�ҋ@
			// delay(0xffff);	//�Q�C���l�����肷��܂ł̑ҋ@�iint_flow()�ŃQ�C������������j
		}
		if(MES_SUB[pch].ws_work_gain > MES[pch].amp_gain_rev){	//�Œ�Q�C���l�̍X�V
			MES_SUB[pch].ws_work_gain = MES[pch].amp_gain_rev;
		}
	}
	MES[pch].amp_gain_fifo = MES_SUB[pch].ws_work_gain;	//�Œ�Q�C���l��Window�T�[�`�Ŏg�p����

	//Window�T�[�`(�œK��FIFO CH��T��)
	MES[pch].ws_FifoCh = SVD[pch].fifo_ch_init;	//���݂�FIFO CH
	// MES[pch].ThresholdReq = 2;	/*Window�T�[�`�v��*/
	// for(fifo_ch=fifo_start; fifo_ch<fifo_end; fifo_ch++){	//FIFO CH(15�`30)�ŃT�[�`����
	// 	SVD[pch].fix_fifo_ch_read = fifo_ch;
	// 	for(cnt=1; cnt<5; cnt++){
	// 		delay(0xffff);	//int_flow(),DriveFIFORev()�Ŏ�g�g�`����͂����邽�߂̑ҋ@
	// 	}
	// 	if(MES_SUB[pch].ws_add_max < MES[pch].ws_work_add){	//���Z�l�̍ő�l���X�V
	// 		MES_SUB[pch].ws_add_max = MES[pch].ws_work_add;
	// 		MES[pch].ws_FifoCh = fifo_ch;		//���Z�l���ő�l��FIFO CH���œK�l�Ƃ���
	// 	}
	// }
	SVD[pch].fifo_ch_init = SVD[pch].fix_fifo_ch_read = MES[pch].ws_FifoCh;		//���������œK��FIFO CH
	SVD[pch].fix_data &= ~0x0C;		//FIFO CH�Œ�̉���

	// if(MES_SUB[pch].ws_add_max == 0){	//���Z�l���X�V����Ă��Ȃ��ꍇ�́A�œK��FIFO CH���Ȃ��Ɣ��f����(�g�`�ُ�A�Z���T���O��Ă���AFIFO�f�[�^���󂯎��Ȃ�)
	// 	ret = B_NG;		//�ُ�
	// }

	return (ret);
}

/****************************************************/
/* Function : SelectForwardOn						*/
/* Summary  : In/Out�ō��݂�؂�ւ���(�o�͗L��)			*/
/* Argument : pch									*/
/* Return   : �Ȃ�									*/
/* Caution  : �Ȃ�									*/
/* note     : ����IN/OUT�؂�ւ��@�\					*/
/****************************************************/
void SelectForwardOn(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesfwdW(0, pch);	//�㗬��GPIO�|�[�g�o��
	}else{
		portmesrevW(0, pch);	//������GPIO�|�[�g�o��
	}
}

/****************************************************/
/* Function : SelectReverseOn						*/
/* Summary  : In/Out�ō��݂�؂�ւ���(�o�͗L��)			*/
/* Argument : pch									*/
/* Return   : �Ȃ�									*/
/* Caution  : �Ȃ�									*/
/* note     : ����IN/OUT�؂�ւ��@�\					*/
/****************************************************/
void SelectReverseOn(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesrevW(0, pch);	//������GPIO�|�[�g�o��
	}else{
		portmesfwdW(0, pch);	//�㗬��GPIO�|�[�g�o��
	}
}

/****************************************************/
/* Function : SelectForwardOff						*/
/* Summary  : In/Out�ō��݂�؂�ւ���(�o�͖���)			*/
/* Argument : pch									*/
/* Return   : �Ȃ�									*/
/* Caution  : �Ȃ�									*/
/* note     : ����IN/OUT�؂�ւ��@�\					*/
/****************************************************/
void SelectForwardOff(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesfwdW(1, pch);	//�㗬��GPIO�|�[�g�o��
	}else{
		portmesrevW(1, pch);	//������GPIO�|�[�g�o��
	}
}

/****************************************************/
/* Function : SelectReverseOff						*/
/* Summary  : In/Out�ō��݂�؂�ւ���(�o�͖���)			*/
/* Argument : pch									*/
/* Return   : �Ȃ�									*/
/* Caution  : �Ȃ�									*/
/* note     : ����IN/OUT�؂�ւ��@�\					*/
/****************************************************/
void SelectReverseOff(short pch){

	if(MES_SUB[pch].memory_side == B_POSI){
		portmesrevW(1, pch);	//������GPIO�|�[�g�o��
	}else{
		portmesfwdW(1, pch);	//�㗬��GPIO�|�[�g�o��
	}
}

short iNum = 0;
short DataNum = 0;
/****************************************************/
/* Function : int_flow								*/
/* Summary  : ���ʌv������							*/
/* Argument : pch									*/
/* Return   : �Ȃ�									*/
/* Caution  : �Ȃ�									*/
/* note     : �����(3msec)�����ݏ���					*/
/****************************************************/
void int_flow(short pch)
{

	if (pch >= CH_NUMMAX)
		return;

	/*�Z���T�����ݒ莞*/
	if (SVD[pch].sensor_size == SNS_NONE)
	{
		non_sensor_control(pch); // �Z���T�����ݒ莞�̏���
		addit_flow_calc();		 // �ώZ���ʏ���
		return;
	}

	
	//�]���p
	if(SVD[pch].sum_step == 2){	//�ō��݉񐔏㗬�����e2��
		MES_SUB[pch].sample_cnt = 2;
	}else if(SVD[pch].sum_step == 3){	//�ō��݉񐔏㗬�����e3��
		MES_SUB[pch].sample_cnt = 3;
	}else if(SVD[pch].sum_step == 4){	//�ō��݉񐔏㗬�����e4��
		MES_SUB[pch].sample_cnt = 4;
	}else{	//�ō��݉񐔏㗬�����e4��
		MES_SUB[pch].sample_cnt = 4;
	}
	AD_BASE = AD_BASE_UNIT * MES_SUB[pch].sample_cnt;
	AD_MAX = AD_MAX_UNIT * MES_SUB[pch].sample_cnt;

	if(20<=SVD[pch].sum_end && SVD[pch].sum_end<=50){
		// MES_SUB[pch].ItvVal = 1000 + (SVD[pch].sum_end - 20)*100;
		MES_SUB[pch].ItvVal = 1000 + (SVD[pch].sum_end - 20)*100 - 500; //����d�l(�g�`�擾��̔g�`���Z���Ԗ�50us���l��)
	}

	if((FpgaVersion == 0x2211) || (FpgaVersion == 0x3211))
	{	//FPGA�I�[�o�[�T���v�����O4�_�o�[�W����
		Mgn = 4;	//ADC65MHz��1/4����16.25MHz�T���v�����O�Ƃ���
	}else{
		Mgn = 8;	//ADC65MHz��1/8����8.125MHz�T���v�����O�Ƃ���
	}
	//�]���p
	
	
	/*�������[�h*/
	if (MES[pch].inspect_enable != 0)
	{ /*�������[�h�L����*/
		inspect_flow_mode(pch);
		return;
	}
__bit_output(GPIO_PORTQ_BASE, 5, 1);
#if defined(DatInc)
	SmpTs[SVD[pch].adc_clock] = SmpTs0[SVD[pch].adc_clock] * Mgn;
#else
	SmpTs[SVD[pch].adc_clock] = SmpTs0[SVD[pch].adc_clock];
#endif

	/*��g�f�[�^�Ǎ���*/
	fifo_read(pch);

	/*�A���v�Q�C������*/
	gain_adj_control(pch);

	/*�A���v�Q�C���`�F�b�N�i�\�m�ۑS�@�\�j*/
	check_amp_gain(pch);

	//�������։��Z
	if (ClcActFlg == ClcAct_SumAdd)
	{
		/*��Βl�̍������Z*/
		sum_adder(pch);

		/*�ŏ��l�A�ő�l�̌���*/
		if (min_max_search(pch) == B_OK)
		{
			/*���ԍ��̉��Z*/
			delta_ts_cal(pch);
		}
	}
	//�[���N���X���Z
	else if (ClcActFlg == ClcAct_ZerCrs)
	{
		//�[���N���X�_��T��
		SchZerPnt(pch);
	}
	else 
	{
		; //���̑��͉��Z��~
	}

	/*�s�[�N���o*/
	MES[pch].ThreasholdPoint = temp_v(pch);
	sonic_search_max(pch);

#ifdef FRQSCH
	if (FrqSchPrc(pch) != 0)
	{
		return; // ���g���T�[�`��
	}
#endif

	/*�����t�B���^����*/
	sonic_filter_control(pch);

	if (ClcActFlg == ClcAct_SumAdd)
	{
		/*�ړ�����*/
		filter_moving(pch);

		/*�[�������␳*/
		zero_adj(pch);

		/*���Z�ُ픻��*/
		correlate_check(pch);
	}
	else if (ClcActFlg == ClcAct_ZerCrs)
	{
		/* ���ԍ��t�B���^���� */
		zc_filter_moving(pch);
	}

	/*���ʊ��Z �i��t�𗬗ʒl�Ɋ��Z�j�A���[�J���j�A���C�Y*/
	MES[pch].ml_min_a = flow_calc(pch); /*ml_min_a (0.1mL/min��1�ŕ\��)*/

	/*K�t�@�N�^�␳*/
	MES[pch].ml_min_b = pv_calc(MES[pch].ml_min_a, pch); /*ml_min_b (0.01mL/min��1�ŕ\��)*/

	/*���[�U���j�A���C�Y*/
	MES[pch].ml_min_c = user_linear(MES[pch].ml_min_b, pch); /*ml_min_c (0.01mL/min��1�ŕ\��)*/

	/*��g�̍�(p1-P2)*/
	max_point_control(pch);

	/*���ʃt�B���^����*/
	MES[pch].ml_min_d = ClcFlwFlt(MES[pch].ml_min_c, pch);

	/*�t���ُ픻��*/
	reverse_flow_check(pch);

	/*�u�����ʍX�V*/
	current_flow_control(pch); /*ml_min_now�ɏu�����ʂ��X�V���� (0.01mL/min��1�ŕ\��)*/

	/*�e�X�g�o�̓��[�h*/
	test_flow_mode(pch);

	/*�_���s���O����*/
	damp_control(pch);

	/*���[�J�b�g����*/
	lowcut_control(pch);

	/*�ώZ���ʏ���*/
	addit_flow_calc();

	/*�o�[���A�E�g����*/
	burnout_control(pch);

	/*�A���v�Q�C������(��CH�p)*/
	SetNextGain(pch);
__bit_output(GPIO_PORTQ_BASE, 5, 0);
	MES[pch].ml_min_oq = work_oq;				/*oq�R�}���h�p*/
	MES[pch].ml_min_OQ = MES[pch].ml_min_now; /*OQ�R�}���h�p*/
}
