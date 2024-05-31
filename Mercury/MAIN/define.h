/***********************************************/
/* File Name : define.h  	         									   */
/*	Summary   : �Œ�萔 					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

/*�����R���p�C��*/
#define FPGADOWNLOAD //FPGA�R���t�B�O�f�[�^���}�C�R������ǂݏo��
#define MEMDBG   //�������ǂݏ���(MR/MW�R�}���h)�@�\
#define FRQSCH  //���g���T�[�`�@�\

/*�����݉�(100msec)*/
#define T100MS 100/18 		/*1ch 18msec����*/

/*���։��Z���i���T���v�����O���ԁj*/
//#define SUB_POINT 42		/*60�ȉ��i�ύX�s�j*/
#define SUB_POINT 21		/*60�ȉ��i�ύX�s�j*/

/*���ʏ��*/
//#define AD_BASE 2047
//#define AD_MAX 4095		/*12bit MAX*/
//#define GAIN_CONT_LOW 3700
// #define AD_BASE_UNIT 2047	/*���S��*/
#define AD_BASE_UNIT 4095	/*���S��*/
//#define AD_BASE 8188		/*4����Z��(2047*4)*/
// #define AD_MAX_UNIT 4095	/*12bit MAX*/
#define AD_MAX_UNIT 8191	/*12bit MAX*/
//#define AD_MAX 16380		/*4����Z��(4095*4)*/

//10bit�f�[�^
// #define GAIN_INIT_LOW	196		/*���������̐M�����x��*/
// #define GAIN_CONT_LOW	3700	/*�Q�C�����䎞�̐M�����x��*/
// #define GAIN_WAVE_LOW	1600	/*�Q�C�����䎞�̐M�����x��*/
// #define GAIN_WAVE_HI	2500	/*�Q�C�����䎞�̐M�����x��*/
// #define SONIC_WAVE_HI	3000	/*�������莞�̐M�����x��*/
//13bit�f�[�^
#define GAIN_INIT_LOW	392		/*���������̐M�����x��*/
#define GAIN_CONT_LOW	7400	/*�Q�C�����䎞�̐M�����x��*/
#define GAIN_WAVE_LOW	3200	/*�Q�C�����䎞�̐M�����x��*/
#define GAIN_WAVE_HI	5000	/*�Q�C�����䎞�̐M�����x��*/
#define SONIC_WAVE_HI	6000	/*�������莞�̐M�����x��*/

//#define GAIN_INIT_LOW	784		/*���������̐M�����x�� 4����Z��(196*4)*/
//#define GAIN_CONT_LOW	14800	/*�Q�C�����䎞�̐M�����x�� 4����Z��(3700*4)*/
//#define GAIN_WAVE_LOW	6400	/*�Q�C�����䎞�̐M�����x�� 4����Z��(1600*4)*/
//#define GAIN_WAVE_HI	10000	/*�Q�C�����䎞�̐M�����x�� 4����Z��(2500*4)*/
//#define SONIC_WAVE_HI	12000	/*�������莞�̐M�����x�� 4����Z��(3000*4)*/

/*FPGA �A�h���X*/
#define FPGA_OFFSET (*(volatile unsigned short *)0x80000000) /* DP-RAM�Ǐo���J�n�I�t�Z�b�g(0-63) */ 
#define FPGA_FREQ   (*(volatile unsigned short *)0x80000002) /* �쓮���g��(0-490) */ 
#define FPGA_CLOCK  (*(volatile unsigned short *)0x80000004) /* ADC�����N���b�N(0-3) */ 
#define FPGA_PULSE  (*(volatile unsigned short *)0x80000006) /* �쓮�p���X��(1-15) */ 
#define FPGA_START  (*(volatile unsigned short *)0x80000008) /* WINDOW�J�n����(0-63) */ 
#define FPGA_END    (*(volatile unsigned short *)0x8000000A) /* WINDOW�I������(0-63) */ 
#define FPGA_SYNC   (*(volatile unsigned short *)0x8000000C) /* �ʑ��p���X����(0-3) */ 
#define FPGA_ANALOG_FREQ  	(*(volatile unsigned short *)0x8000000E) /* �A�i���O�X�C�b�`���g���ؑ�(0-1) */
#define FPGA_FILIN0_0	(*(volatile unsigned short *)0x80000010) /* �f�W�^���t�B���^�W��(���͑�0) (����1bit+����17bit) */
#define FPGA_FILIN0_1	(*(volatile unsigned short *)0x80000012) /* �f�W�^���t�B���^�W��(���͑�0) (����1bit+����17bit) */
#define FPGA_FILIN1_0	(*(volatile unsigned short *)0x80000014) /* �f�W�^���t�B���^�W��(���͑�1) (����1bit+����17bit) */
#define FPGA_FILIN1_1	(*(volatile unsigned short *)0x80000016) /* �f�W�^���t�B���^�W��(���͑�1) (����1bit+����17bit) */
#define FPGA_FILIN2_0	(*(volatile unsigned short *)0x80000018) /* �f�W�^���t�B���^�W��(���͑�2) (����1bit+����17bit) */
#define FPGA_FILIN2_1	(*(volatile unsigned short *)0x8000001A) /* �f�W�^���t�B���^�W��(���͑�2) (����1bit+����17bit) */
//<<reserve>>			(*(volatile unsigned short *)0x8000001C) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x8000001E) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x80000020) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x80000022) /* reserve */
#define FPGA_FIL_EN		(*(volatile unsigned short *)0x80000024) /* �f�W�^���t�B���^�L���E�����ݒ� (0-1) */
//<<reserve>>			(*(volatile unsigned short *)0x80000026) /* reserve */
#define FPGA_FILOUT1_0	(*(volatile unsigned short *)0x80000028) /* �f�W�^���t�B���^�W��(�o�͑�1) (����1bit+����17bit) */
#define FPGA_FILOUT1_1	(*(volatile unsigned short *)0x8000002A) /* �f�W�^���t�B���^�W��(�o�͑�1) (����1bit+����17bit) */
#define FPGA_FILOUT2_0	(*(volatile unsigned short *)0x8000002C) /* �f�W�^���t�B���^�W��(�o�͑�2) (����1bit+����17bit) */
#define FPGA_FILOUT2_1	(*(volatile unsigned short *)0x8000002E) /* �f�W�^���t�B���^�W��(�o�͑�2) (����1bit+����17bit) */
//<<reserve>>			(*(volatile unsigned short *)0x80000030) /* reserve */
//<<reserve>>			(*(volatile unsigned short *)0x80000032) /* reserve */
#define FPGA_LED_CNT		(*(volatile unsigned short *)0x80000034) /* LED���䃌�W�X�^ */
#define FPGA_SW_DISP		(*(volatile unsigned short *)0x80000036) /* SW�\�����W�X�^ */
#define FPGA_VERSION 		(*(volatile unsigned short *)0x8000003E) /* FPGA�o�[�W���� */ 

/*�ݒ�f�[�^��*/
#define SV_NUM	491			/*SVD �̑���*/
#define AMP_BUFF	60

/*EEPROM*/
#define SIZEOFMODBUSADDR 				0x400	/*CH1���̃A�h���X�T�C�Y*/
#define WRIT_CHECK (SIZEOFMODBUSADDR - 1)		/*EEPROM �����݃`�F�b�N�A�h���X*/

#define WR_USER_PARA  0x0001  /*���[�U�p�����[�^������*/
#define WR_USPAR_DEVICE  0x0002  /*���[�U�p�����[�^������(�������f�o�C�X)*/
#define WR_USER_LINR  0x0004  /*���[�U���j�A���C�Y������*/
#define WR_MAKER_PARA 0x0008  /*���[�J�p�����[�^������*/
#define WR_MAKER_LINR 0x0010  /*���[�J���j�A���C�Y������*/
#define WR_CVT_SERIAL 0x0020  /*�ϊ���V���A���i���o�[������*/
#define WR_SNS_SERIAL 0x0040  /*�Z���T�V���A���i���o�[������*/

#define WR_ALL (WR_USER_PARA + WR_USPAR_DEVICE + WR_USER_LINR + WR_MAKER_PARA + WR_MAKER_LINR + WR_CVT_SERIAL + WR_SNS_SERIAL) /*�S�p�����[�^������*/
#define WR_DEVICE (WR_USPAR_DEVICE + WR_USER_LINR + WR_SNS_SERIAL) /*�������f�o�C�X������*/

/*�ʐM�p*/
#define CR		0x0D		/*�L�����b�W���^�[��*/
#define LF		0x0A		/*���C���t�B�[�h*/

#define Dt		1			/*�������(10ms)1�Œ�*/

/*�X�e�[�^�X���*/
#define	B_OFF			0		//�I�t���
#define	B_ON				1		//�I�����
#define	B_BLINK				2		//���]
#define	B_BLANK				3		//��

#define	B_NO				0		//No
#define	B_YES			1		//Yes

#define	B_OK				0		//OK
#define	B_NG				-1		//NG

#define	B_PLUS			1		//+����
#define	B_MINUS			-1		//-����

#define	B_POSI			1
#define	B_NEGA			2

#define	B_FORWARD		0
#define	B_REVERSE		1

/*�[���_�������*/
#define	SW_ZERO_START		30		//�[���_�����J�n��������:3�b(100msec x 30�� = 3sec)
#define	ZERO_ADJ_TIME		20		//�[���_�������ԁF2�b(100msec x 20�� = 2sec)
#define	VTH_ADJ_TIME		20		//Vth�������ԁF2�b(100msec x 20�� = 2sec)
#define	WAVE_ADJ_TIME		300		//�g�`�F���������ԁF3�b(100msec x 300�� = 30sec)

/*�G���[�������l*/
#define	LIM_OVERFLOW 100			/*�I�[�o�[�t���[*/

/*�G���[���O���*/
#define	LOGMAX				10		//�G���[���O�ۑ���

/*�G���[�R�[�h*/
#define	ERR_ZERO_EMPTY		1		//�[�������G���v�e�B�Z���T
#define	ERR_ZERO_WAVE		2		//�[�������g�`���� �i�g�`�ُ�j
#define	ERR_ZERO_CALC		3		//�[���������Z�ُ�
#define	ERR_ZERO_LEVEL		4		//�[�������g�`�A���o�����X
#define	ERR_ZERO_AGC		5		//�[������AGC�s�\//

#define	ERR_MESR_EMPTY_L	6		//����G���v�e�B�Z���TL
#define	ERR_MESR_EMPTY_H	7		//����G���v�e�B�Z���TH
#define	ERR_MESR_WAVE_L		8		//����g�`����L
#define	ERR_MESR_WAVE_H		9		//����g�`����H
#define	ERR_MESR_CALC_L		10		//���艉�Z�ُ�L
#define	ERR_MESR_CALC_H		11		//���艉�Z�ُ�H
#define	ERR_MESR_LEVEL_L	12		//����g�`�A���o�����XL
#define	ERR_MESR_LEVEL_H	13		//����g�`�A���o�����XH
#define	ERR_MESR_AGC_L		14		//����AGC�s�\L
#define	ERR_MESR_AGC_H		15		//����AGC�s�\H
#define	ERR_MESR_REVERSE	16		//����t���ُ�

#define	ERR_10POINT			19		//10�_�f�[�^����
#define	ERR_EEPROM			20		//EEPROM�G���[
#define	ERR_RESTART			22		//�ċN��

#define	ERR_MESR_OVERFLOW	23		//����I�[�o�[�t���[
#define	ERR_CUNET			24		//CUnet�G���[
#define	ERR_ZERO_UNSTABLE	25		//�[�������v�����Ԕ��U

#define ALM_MESR_GAIN		26		//����A���v�Q�C���}�ρi�x���j
#define ALM_MESR_EMPTY	27		//����G���v�e�B�Z���T�i�x���j

#define TTL_CACL_ERR	28		//�ώZ�l���Z�ُ�
#define TTL_OVERFLOW	29		//�ώZ�l�I�[�o�[�t���[

#define	ERR_DEVICE		30		//�������f�o�C�X�ُ�

#define	ERR_CODE_MAX		32		//�G���[�R�[�h��

/*�G���[����r�b�g(MES[].err_status���g�p����)*/
#define	ERR_JUDGE_AGC		0x0001		//AGC�s�\//
#define	ERR_JUDGE_EMPTY		0x0002		//�G���v�e�B�Z���T(�ُ픻��񐔈ȏ�)
#define	ERR_JUDGE_LEVEL		0x0004		//�g�`�A���o�����X
#define	ERR_JUDGE_BUBLE		0x0008		//�C�A�e���i�Z���T�M���჌�x���j
#define	ERR_JUDGE_CALC		0x0010		//���Z�ُ�(�ُ픻��񐔈ȏ�)
#define	ERR_JUDGE_REVERSE	0x0020		//�t���ُ�
#define	ERR_JUDGE_WAVE		0x0040		//�g�`����
#define	ERR_JUDGE_OVERFLOW	0x0080		//�I�[�o�[�t���[
#define	ERR_JUDGE_10POINT	0x0100		//10�_�f�[�^�����G���[
#define	ERR_JUDGE_EEPROM	0x0200		//EEPROM�G���[
#define	ERR_JUDGE_CUNET		0x0400		//CUnet�G���[
#define	ERR_JUDGE_RESTART	0x0800		//�ċN��
#define	ERR_JUDGE_PRE_EMPTY	0x1000		//�G���v�e�B�Z���T(�ُ픻��񐔖���)
#define	ERR_JUDGE_PRE_CALC	0x2000		//���Z�ُ�(�ُ픻��񐔖���)
#define	ERR_JUDGE_UNSTABLE	0x4000		//�[�������v�����Ԕ��U�G���[
#define	ERR_JUDGE_ZERO		0x8000		//�[���_�������

/*�G���[����r�b�g(MES_SUB[].err_status_sub���g�p����)*/
#define	ERR_JUDGE_DEVICE	0x0001		//�������f�o�C�X�ُ�

/*���[�����M�G���[(CUnet�ʐM)*/
#define	MAIL_ERR_NORDY		0x0001		//���M��̎�M�o�b�t�@����M���łȂ�
#define	MAIL_ERR_NOEX		0x0002		//���M���CUnet�X�e�[�V���������݂��Ȃ�
#define	MAIL_ERR_TOUT		0x0004		//�ݒ�T�C�N���񐔂��o�߂��Ă����[�����M���������Ȃ�
#define	MAIL_ERR_SZFLT		0x0008		//���[�����M�T�C�Y���s���l
#define	MAIL_ERR_LMFLT		0x0010		//MSLR�̐ݒ�l���s���l
#define	MAIL_ERR_STOP		0x0020		//�l�b�g���[�N����~

/*�o�[���A�E�g�ΏۃG���[*/
#define	ERR_BURN_OUT		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_CALC+ERR_JUDGE_OVERFLOW)

/*�[�������s�ΏۃG���[*/
#define	ERR_ZERO_ADJ		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_BUBLE+ERR_JUDGE_CALC+ERR_JUDGE_WAVE+ERR_JUDGE_OVERFLOW)

/*CUnet�G���[�ΏۃG���[*/
#define	ERR_CUNET_MAIL		(MAIL_ERR_NOEX+MAIL_ERR_TOUT+MAIL_ERR_SZFLT+MAIL_ERR_LMFLT+MAIL_ERR_STOP)

/*���ʃG���[*/
#define	ERR_FLOW_CONT		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_BUBLE+ERR_JUDGE_CALC+ERR_JUDGE_REVERSE+ERR_JUDGE_WAVE+ERR_JUDGE_OVERFLOW)

/*���f�B�A���t�B���^���Z���O�G���[*/
#define	ERR_MEDIAN_CALC		(ERR_JUDGE_AGC+ERR_JUDGE_EMPTY+ERR_JUDGE_LEVEL+ERR_JUDGE_CALC+ERR_JUDGE_OVERFLOW+ERR_JUDGE_PRE_EMPTY+ERR_JUDGE_PRE_CALC)

/*�x������r�b�g*/
#define	ALM_JUDGE_GAIN		0x0001		//�Q�C���l�}�ρi�x���j
#define	ALM_JUDGE_EMPTY		0x0002		//�G���v�e�B�Z���T�i�x���j

/*�ώZ�Ď��r�b�g*/
#define	TTL_JUDGE_REACH		0x0001		//�ώZ�ڕW�l���B
#define	TTL_JUDGE_CACL_ERR	0x0002		//�ώZ�l���Z�ُ�
#define	TTL_JUDGE_OVERFLOW	0x0004		//�ώZ�l�I�[�o�[�t���[

/*�G���[�R���f�B�V�����r�b�g*/
/*CUnet�A�h���X�o�͗p*/
#define	CON_EMPTY_CU		0x0001		//�G���v�e�B�Z���T
#define	CON_CALC_CU			0x0002		//���Z�ُ�
#define	CON_AGC_CU			0x0004		//AGC�s�\//
#define	CON_LEVEL_CU		0x0008		//�g�`�A���o�����X
#define	CON_REV_CU			0x0010		//�t���ُ�
#define	CON_WAVE_CU			0x0020		//�g�`����
#define	CON_OVER_CU			0x0040		//�I�[�o�[�t���[
#define	CON_TOTAL_CU			0x0080		//�ώZ�ڕW�l���B

/*�G���[�R���f�B�V�����r�b�g*/
#define	CON_EMPTY			0x0001		//�G���v�e�B�Z���T
#define	CON_PEAK			0x0002		//�ɒl�ُ�
#define	CON_LEVEL			0x0004		//�g�`�A���o�����X
#define	CON_WAVE			0x0008		//�g�`����
#define	CON_CALC			0x0010		//���Z�ُ�
#define	CON_GAIN_OV			0x0020		//�Q�C���I�[�o�[
#define	CON_GAIN_UD			0x0040		//�Q�C���A���_�[
#define	CON_REV				0x0080		//�t���ُ�
#define	CON_HOLD			0x0100		//�z�[���h�^�C������
#define	CON_HOLDOUT			0x0200		//�z�[���h�^�C���ȏ�
#define	CON_ZERO			0x0400		//�[���_�����G���[

/*����X�e�[�^�X*/
#define	ACT_STS_NORMAL		0			//�ʏ�
#define	ACT_STS_ZERO		1			//�[���_������
#define	ACT_STS_ADDIT		2			//�ώZ���s��
#define	ACT_STS_WRITE		3			//�p�����[�^�����ݒ�
#define	ACT_STS_DLOAD		4			//�t�@�[���E�F�A�_�E�����[�h��
#define	ACT_STS_TEST		5			//�e�X�g��
#define	ACT_STS_OTHER		6			//���̑�

/*�ċN���v���R�[�h*/
#define	RESTART_NORMAL		0			//�p���[�I�����Z�b�g(����N��)
#define	RESTART_WDOG		1			//�E�H�b�`�h�b�O�^�C�}���Z�b�g
#define	RESTART_POWER		2			//�d���Ď����Z�b�g
#define	RESTART_TERM		3			//�[�q���Z�b�g(MKY43)

/*�J�E���^���*/
#define	CMI_RATIO_100MSEC	19			//����\�F95msec����
#define	CMI_RATIO_1SEC		200			//����\�F1�b����
#define	CMI_COUNT_MAX		0x7FFFFFFF	//�^�C�}�����݃J�E���^MAX

/*CH���*/
#define CH_NUMMAX 			6				//CH��
#define CH_IDXMAX 			CH_NUMMAX - 1	//CH�C���f�b�N�X

/*CH�ԍ�(�C���f�b�N�X�����˂邽��0�J�n)*/
#define CH1 				0		//CH1
#define CH2 				1		//CH2
#define CH3 				2		//CH3
#define CH4 				3		//CH4
#define CH5 				4		//CH5
#define CH6 				5		//CH6

/*LED�_������*/
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

#define	DSP_CH_ALL		(DSP_LED1+DSP_LED2+DSP_LED3+DSP_LED4+DSP_LED5+DSP_LED6)		//�SCH LED
#define	DSP_ALM_ALL		(DSP_ALM1+DSP_ALM2+DSP_ALM3+DSP_ALM4)		//�SALM LED

/*�X�C�b�`���*/
#define	SW_NUM			8			//�X�C�b�`��
#define	SW_CH0			0			//CH0�X�C�b�`
#define	SW_CH1			1			//CH1�X�C�b�`
#define	SW_CH2			2			//CH2�X�C�b�`
#define	SW_CH3			3			//CH3�X�C�b�`
#define	SW_CH4			4			//CH4�X�C�b�`
#define	SW_CH5			5			//CH5�X�C�b�`
#define	SW_CH6			6			//CH6�X�C�b�`
#define	SW_CH9			9			//CH9�X�C�b�`

/*�X�e�[�V�����A�h���X*/
#define SA_HOST			0			//HOST

//CUnet 16ch�Ή�
#define SA_FLOW01		4			//���ʌv01
#define SA_FLOW02		7			//���ʌv02
#define SA_FLOW03		10			//���ʌv03
#define SA_FLOW04		13			//���ʌv04
#define SA_FLOW05		16			//���ʌv05
#define SA_FLOW06		19			//���ʌv06
#define SA_FLOW07		22			//���ʌv07
#define SA_FLOW08		25			//���ʌv08
#define SA_FLOW09		28			//���ʌv09
#define SA_FLOW10		31			//���ʌv10
#define SA_FLOW11		34			//���ʌv11
#define SA_FLOW12		37			//���ʌv12
#define SA_FLOW13		40			//���ʌv13
#define SA_FLOW14		43			//���ʌv14
#define SA_FLOW15		46			//���ʌv15
#define SA_FLOW16		49			//���ʌv16

/*��L��*/
#define OWN_HOST		3			//HOST
#define OWN_FLOW		3			//���ʌv

/*�ʐM���b�Z�[�W���*/
#define MSG_MAX			700			//�ʐM���b�Z�[�W�ő吔
#define MSG_MAX_DL		256 + 40	//�ʐM���b�Z�[�W�ő吔(�_�E�����[�h��) 128byte(2����/1byte)+�}�[�W��
#define MES_RESEND_MAX	1000		//�ő�đ���
#define MSG_NUM			3			//�ʐM�^�C�v���i�z�X�g�A�T�u�z�X�g�A�����e�i���X�j
#define MES_RESEND_LIM	(60 * CMI_RATIO_1SEC)	//�ő�đ�����:60�b(60 * �J�E���^����\)
#define MES_1ST_TOP		0			//���M���b�Z�[�W1��ڐ擪�o�C�g
#define MES_2ND_TOP		256			//���M���b�Z�[�W2��ڐ擪�o�C�g
#define MES_3RD_TOP		512			//���M���b�Z�[�W3��ڐ擪�o�C�g
#define MES_CHAR_MAX 20		//�R�}���h���ő啶����
#define MES_W7_STATUS 5			//W7�R�}���h�X�e�[�^�X������

#define HOST			0			//�z�X�g�ʐM
#define MENT			1			//�����e�i���X�ʐM
#define SUB_HOST		2			//�T�u�z�X�g�ʐM

#define HEADER_CH		0x01			//CUnet Header CH

/*�ʐM���b�Z�[�W���	(End Code)*/
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

/*�Z���T���*/
#define SNS_NONE		0			//�Z���T����
#define SNS_TYPE_1_8	1			//PFA 1/8"
#define SNS_TYPE_4_3	2			//PFA 4x3
#define SNS_TYPE_1_4	3			//PFA 1/4"
#define SNS_TYPE_3_8	4			//0PFA 3/8"

/*���f�B�A���t�B���^���*/
#define MF_DEPTH	(32)
#define MF_AVE_ST	(8)
#define MF_AVE_EN	(23)

/*��t���j�A���*/
#define LL_NUM		8			//��t��ʑ���
#define LL_KV		0			//���S�x
#define LL_SS		1			//�Z���T���
#define LL_FS		2			//�t���X�P�[��
#define LL_LP		3			//���j�A���C�Y�_��
#define LL_POINT1	4			//�␳��1�_��

/*�t�B���^�[���[�h���*/
#define FILTER_MOVING 0 //�ړ�����
#define FILTER_DIRECT 1 //�t�B���^����
#define FILTER_MEDIAN 2 //���f�B�A���t�B���^

/*�[���N���X���*/
#define ZC_POINT_MAX 40 //�[���N���X�_�ő�l

/*Window�T�[�`���*/
/*32MHz, 40MHz*/
#define WS_FIFO_START_14_3240 10	//Window�T�[�`�J�nFIFO CH(1/4"�p)
#define WS_FIFO_START_38_3240 10	//Window�T�[�`�J�nFIFO CH(3/8"�p)
#define WS_FIFO_END_3240 25	//Window�T�[�`�I��FIFO CH
/*65MHz*/
#define WS_FIFO_START_14 15	//Window�T�[�`�J�nFIFO CH(1/4"�p)
#define WS_FIFO_START_38 15	//Window�T�[�`�J�nFIFO CH(3/8"�p)
#define WS_FIFO_END 30	//Window�T�[�`�I��FIFO CH

#define WS_FIFO_RANGE 8	//FIFO�擾�͈�

// UART ����t����` 
#define	UART_COM_HOST_BASE	UART6_BASE
#define	INT_UART_COM_HOST	INT_UART6
#define	UART_COM_MENT_BASE	UART2_BASE
#define	INT_UART_COM_MENT	INT_UART2

#define WAV_PEK_NUM 10 //�擾����g�`�̃s�[�N�l��
#define LDG_PNT_OFS 40  //Leading Point Offset ((�]���l)40 / (�I�[�o�[�T���v�����O�{��)8 = 5)

#define ADD_NO_CMM 0 //Add No Comma
#define ADD_CMM_BFR_VAL 1 //Add Comma Before Value
#define ADD_CMM_AFR_VAL 2 //Add Comma After Value
