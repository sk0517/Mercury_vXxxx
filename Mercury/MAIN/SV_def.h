/***********************************************/
/* File Name : SV_def.h 		         									   */
/*	Summary   : パラメータの定義	                    */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

#include "define.h"

/************************************************************/
/*	パラメータの定義									                                     */
/*	※パラメータ初期値(SV_def.c)と合わせること									                */
/*                                                          */
/*  ※パラメータ追加時は util.c内のutil_eep_allwrite() に追記する*/
/************************************************************/
typedef struct {
	/*****             *****/
	/***   ユーザパラメータ   ***/
	/*****             *****/
		short max_flow;			/*フルスケール*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		unsigned short unit;	/*フルスケール小数点位置、単位*/    /*<<メモリデバイス保存>> ※変更禁止※*/  
		short sensor_size;		/*センサ種別*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		short k_factor;			/*Kファクタ*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		short damp;				/*ダンピング*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		short low_cut;			/*ローカットOFF*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		unsigned short burnout;  /*バーンアウト*/
		short burnout_value;     /*バーンアウト入力値*/
		short	viscos;        /*動粘度*/
		short viscos_auto;   /*動粘度固定/測定*/
		unsigned short err_hold_time;		/*エラーホールドタイム*/
		unsigned short reverse_time;	  /*逆流判定時間*/
		short	reverse_level;	/*逆流判定閾値*/
		short reserve_01[64];		    /*予約[64]*/

	/*****             *****/
	/***   メーカパラメータ    ***/
	/*****             *****/
		short drive_freq;			/*駆動周波数*/
		short drive_pls;			/*ドライブパルス数*/
		short	search_sw;			/*相関値サーチ機能*/
		short gain_step;			/*ゲイン制御ステップ幅*/
		short sound_vel_filter;				/*音速フィルタ時定数*/
		short fifo_ch_init;				/*FIFO CH 初期値*/
		short sound_vel_sel;		/*音速固定・測定切替*/
		short sound_vel_fix;		/*音速固定値*/
		unsigned short hldt;		/*異常ホールド時間選択*/
		short cunet_delay;				/*CUnet 再送信待機時間*/
		short wave_vth;			/*受波検出閾値*/
		short balance_level;		/*波形アンバランス検出閾値*/
		short saturation_level;	/*波形飽和検出閾値*/
		short correlate_level;	/*差分相関比閾値*/
		short correlate_time;		/*演算異常判定回数*/
		short	attenuate_level;	/*波形減衰閾値*/
		short alm_wave_vth;					/*受波検出警告閾値*/
		short alm_gain_level;		/*アンプゲイン警告閾値*/
		short	alm_gain_count;		/*アンプゲイン警告ｶｳﾝﾄ*/
		unsigned short alm_hold_time;		/*警告出力時間*/
		short zero_offset;		/*ゼロ点オフセット*/
		unsigned short filter_mode;	/*フィルタモード*/	
		unsigned short filter_avg;		/*移動平均数*/	
		unsigned short LL_enable;		/*薬液リニアライズモード選択*/
		unsigned short LL_kind;		  /*薬液リニアライズモード・薬液種別*/
		short total_offset_enable;	/*積算オフセット値(ON/OFF)*/
		short total_offset_value;		/*積算オフセット値(設定値)*/
		short drive_search;		/*調整モード*/
		short start_freq;		  /*サーチ開始周波数*/
		short stop_freq;		   /*サーチ停止周波数*/
#if defined(FRQSCH)
		short SchFrq;           /* サーチして得た周波数 */
#endif
		union{
			long INT32;
			struct{
				unsigned short low;
				unsigned short high;
			}DT_2BYTE;
		}target_total;			/*積算目標値*/
		unsigned short damp_mode;	/*気泡対策モード*/
		unsigned short rl2d;		/*D Rate Limit ダンピング倍率*/
		unsigned short rl2hc;		/*D Rate Limit ホールドクリア*/
		short odpd;				/*レイトリミット1st*/
		unsigned short rl1tg;		/*D Rate limit 1st 目標*/
		unsigned short rl1av;		/*D Rate limit 1st 平均*/
		short odpl;				/*レイトリミット2nd*/
		unsigned short rl2tg;		/*D Rate limit 2nd 目標*/
		unsigned short rl2av;		/*D Rate limit 2nd 平均*/
		short rlt;				/*レイトリミット*/
		unsigned short dump_var;	/*可変ダンピング偏差値*/
		unsigned short dump_mul;	/*可変ダンピング倍率*/
		unsigned short inc;		/*異常ホールド時間*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}corr_up;				/*相関値幅上限値*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}corr_low;				/*相関値幅下限値*/
		
		
//評価用
		unsigned short adc_clock;			//ADCクロック
		unsigned short sns_option;			//センサオプション
		unsigned short sns_disL; //センサ間距離(L)
		unsigned short sns_disL_l; //センサ間距離(L-l)
		unsigned short sns_tau; //無駄時間
		unsigned short sns_coef; //互換係数
		unsigned short wind_offset; //WINDOWオフセット
		unsigned short sum_start; //差分相関開始位置
		unsigned short sum_end;   //差分相関終了位置
		unsigned short sum_step;  //差分相関間隔

		unsigned short fix_data;  //固定値設定 bit0 :ディザリングスイッチ
		                          //             1 : ゲイン固定値スイッチ
								  //             2 : fifo開始、終了位置固定スイッチ
								  //             3 : fifo_no 固定スイッチ
								  //             4 : 波形先頭位置固定スイッチ
		unsigned short fix_amp_gain_rev;  //Wiper Position(固定値)
		unsigned short fix_fifo_ch_read;  //FIFO CH(固定値)
		unsigned short fix_fifo_no_read;  //Leading Position(固定値)

		unsigned short ZerCrsSttPnt; //Zero Cross Shift point
		unsigned short ZerCrsUseNum; //Zero Cross Use Number
		unsigned short ZerCrsOffset[7]; //ゼロクロスゼロ点オフセット
		unsigned short ThresholdPeakPos;	//波形認識ピーク位置
		unsigned short ZerPeakPos;	//波形認識ピーク位置(ゼロクロス計算開始位置用)

		unsigned short DgtFltSwc; //Digital Filter Switch
		unsigned short DgtFltCefA00; //Digital Filter Coefficient A00
		unsigned short DgtFltCefA01; //Digital Filter Coefficient A01
		unsigned short DgtFltCefA10; //Digital Filter Coefficient A10
		unsigned short DgtFltCefA11; //Digital Filter Coefficient A11
		unsigned short DgtFltCefA20; //Digital Filter Coefficient A20
		unsigned short DgtFltCefA21; //Digital Filter Coefficient A21
		unsigned short DgtFltCefA30; //Digital Filter Coefficient A30
		unsigned short DgtFltCefA31; //Digital Filter Coefficient A31
		unsigned short DgtFltCefA40; //Digital Filter Coefficient A40
		unsigned short DgtFltCefA41; //Digital Filter Coefficient A41
		unsigned short DgtFltCefB00; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB01; //Digital Filter Coefficient B01
		unsigned short DgtFltCefB10; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB11; //Digital Filter Coefficient B01
		unsigned short DgtFltCefB20; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB21; //Digital Filter Coefficient B01
		unsigned short DgtFltCefB30; //Digital Filter Coefficient B00
		unsigned short DgtFltCefB31; //Digital Filter Coefficient B01
//評価用		

#if defined(FRQSCH)
		short reserve_02[28];		/*予約[28]*/
#else
		short reserve_02[29];		/*予約[29]*/
#endif

	/*****             *****/
	/***   メーカリニアライズ   ***/
	/*****             *****/
		short mklnr_num;				/*補正点数*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out1;					/*補正出力1(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out2;					/*補正出力2(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out3;					/*補正出力3(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out4;					/*補正出力4(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out5;					/*補正出力5(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out6;					/*補正出力6(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out7;					/*補正出力7(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out8;					/*補正出力8(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out9;					/*補正出力9(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out10;					/*補正出力10(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out11;					/*補正出力11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out12;					/*補正出力12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out13;					/*補正出力13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out14;					/*補正出力14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_out15;					/*補正出力15(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in1;					/*補正入力1(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in2;					/*補正入力2(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in3;					/*補正入力3(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in4;					/*補正入力4(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in5;					/*補正入力5(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in6;					/*補正入力6(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in7;					/*補正入力7(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in8;					/*補正入力8(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in9;					/*補正入力9(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in10;					/*補正入力10(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in11;					/*補正入力11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in12;					/*補正入力12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in13;					/*補正入力13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in14;					/*補正入力14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}mklnr_in15;					/*補正入力15(L/m)*/

	/*****             *****/
	/***  ユーザリニアライズ   ***/
	/*****             *****/
		unsigned short uslnr_num;		/*補正点数、小数点位置*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out1;				/*補正出力1(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out2;				/*補正出力2(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out3;				/*補正出力3(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out4;				/*補正出力4(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out5;				/*補正出力5(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out6;				/*補正出力6(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out7;				/*補正出力7(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out8;				/*補正出力8(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out9;				/*補正出力9(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out10;				/*補正出力10(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out11;				/*補正出力11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out12;				/*補正出力12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out13;				/*補正出力13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out14;				/*補正出力14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_out15;				/*補正出力15(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in1;				/*補正入力1(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in2;				/*補正入力2(L/m)*/	    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in3;				/*補正入力3(L/m)*/	    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in4;				/*補正入力4(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in5;				/*補正入力5(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in6;				/*補正入力6(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in7;				/*補正入力7(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in8;				/*補正入力8(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in9;				/*補正入力9(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in10;				/*補正入力10(L/m)*/    /*<<メモリデバイス保存>> ※変更禁止※*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in11;				/*補正入力11(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in12;				/*補正入力12(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in13;				/*補正入力13(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in14;				/*補正入力14(L/m)*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}uslnr_in15;				/*補正入力15(L/m)*/

	/*****                  *****/
	/***   ゼロ点調整時ステータス   ***/
	/*****                  *****/
		union{
			long DWORD;
			struct{
				short low;
				short high;
			}WORD;
		}zero_flow_qat;					/*ゼロ点調整時：流量*/
		union{
			long DWORD;
			struct{
				short low;
				short high;
			}WORD;
		}zero_flow_vel;					/*ゼロ点調整時：流速*/
		unsigned short zero_sound_spd;	/*ゼロ点調整時：音速*/
		union{
			unsigned long long DWORD;
			struct{
				short low1;
				short low2;
				short high1;
				short high2;
			}WORD;
		}zero_addit;					/*ゼロ点調整時：積算値*/	
		unsigned short zero_wave_max;		/*ゼロ点調整時：受波波形最大値*/
		unsigned short zero_wave_min;		/*ゼロ点調整時：受波波形最小値*/
		union{
			long DWORD;
			struct{
				short low;
				short high;
			}WORD;
		}zero_delta_ts;					/*ゼロ点調整時：伝搬時間差*/
		unsigned short zero_correlate;	/*ゼロ点調整時：相関値幅*/
		unsigned short zero_zero_offset;	/*ゼロ点調整時：ゼロ点オフセット*/
		unsigned short zero_condition;	/*ゼロ点調整時：コンディション*/
		unsigned short zero_fifo_pos;		/*ゼロ点調整時：FIFO受波波形検出位置*/
		unsigned short zero_gain_1st;		/*ゼロ点調整時：アンプゲイン値(1st)*/
		unsigned short zero_gain_2nd;		/*ゼロ点調整時：アンプゲイン値(2nd)*/
		unsigned short zero_fifo_ch;		/*ゼロ点調整時：FIFO CH*/
		unsigned short zero_p1p2;			/*ゼロ点調整時：受波の差(P1-P2)*/

		short zero_FwdSurplsTim; //上流波形無駄時間
		short zero_RevSurplsTim;
		union {
			long DWORD;
			struct {
				short low;
				short high;
			}WORD;
		}zero_FwdTimDif; //上流波形時間差
		union {
			long DWORD;
			struct {
				short low;
				short high;
			}WORD;
		}zero_RevTimDif; //下流波形時間差
		short zero_drive_freq; //打ち込み周波数
		short zero_FwdWavPekPosLst[WAV_PEK_NUM]; //上流波形ピーク位置
		short zero_FwdWavPekValLst[WAV_PEK_NUM]; //上流波形ピーク値
		short zero_RevWavPekPosLst[WAV_PEK_NUM]; //下流波形ピーク位置
		short zero_RevWavPekValLst[WAV_PEK_NUM]; //下流波形ピーク値

	/*****            *****/
	/***   エラーログ関連   ***/
	/*****            *****/
		unsigned short pwon_count;/*起動回数*/
		short	err_code[LOGMAX];			/*エラーコード（最新10点）*/
		short	err_pwon_count[LOGMAX];		/*エラー発生時の電源オンカウンタ（最新10点）*/
		union{
			long DWORD;
			struct{
				unsigned short low;
				unsigned short high;
			}WORD;
		}err_time[LOGMAX];				/*エラー発生時の時間（最新10点）*/

	/*****         *****/
	/***   通信関連   ***/
	/*****         *****/
		short com_interval; /*サイレントインターバル*/
		short com_speed;    /*通信スピード*/
		short com_mode;     /*通信モード*/
		short com_parity;			/*パリティ*/
		unsigned short sti;		/*サイレントインターバル(2byte)*/
		unsigned short cmod;		/*通信モード、パリティ(2byte)*/

	/*****                    *****/
	/***   バージョン、シリアルナンバー  ***/
	/*****                    *****/
		unsigned short soft_ver;	/*ソフトウェアバージョン*/
		unsigned short hard_ver;	/*ハードウェアバージョン*/
		unsigned short c_serial[8];		/*変換器シリアルナンバー*/
		unsigned short s_serial[8];		/*センサシリアルナンバー*/    /*<<メモリデバイス保存>> ※変更禁止※*/

 		short reserve_03[25];		/*予約[25]*/

} stSVD;

//----------------------------------------------------------------------
//外部宣言
//----------------------------------------------------------------------
extern stSVD SVD[CH_NUMMAX];

