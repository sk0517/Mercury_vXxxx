/***********************************************/
/* File Name : cunet.h  		         									   */
/*	Summary   : CUnetèàóù					                   */
/*	Date      : 2023/03/16										            */
/*																	                        	   */
/*	Copyright(c) 2023 Tokyo Keiso Co.Ltd.				   */
/*			All rights reserved															        */
/***********************************************/

// [RX] : #define SKIP_NUM	511
#define SKIP_NUM	0

struct st_mky43{
	struct {
	/*GM(Grobal Memory)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		}SA[64];
	}GM;

	/*MSB(Mail Send Buffer)*/
	struct {
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		}SEND[32];
	} MSB;
	
	/*REG(Register or Maker Reserve)*/
	struct {
		/*RFR(Recive Flag Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} RFR;

		/*LFR(Link Flag Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} LFR;

		/*MFR(Member Flag Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} MFR;

		/*DRFR(Data Renewal Flag Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} DRFR;
		
		/*LGR(Ling Group Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} LGR;

		/*MGR(Member Group Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} MGR;

		/*DRCR(Data Renewal Check Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} DRCR;
		
		/*RHCR0(Read Hazard Control Register0)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short MB0:1;
				unsigned short MB1:1;
				unsigned short MB2:1;
				unsigned short MB3:1;
				unsigned short MB4:1;
				unsigned short MB5:1;
				unsigned short :10;
			} BIT;
		} RHCR0;
		unsigned short skip_rhcr0[SKIP_NUM];

		/*RHCR1(Read Hazard Control Register1)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short MB0:1;
				unsigned short MB1:1;
				unsigned short MB2:1;
				unsigned short MB3:1;
				unsigned short MB4:1;
				unsigned short MB5:1;
				unsigned short :10;
			} BIT;
		} RHCR1;
		unsigned short skip_rhcr1[SKIP_NUM];

		/*WHCR0(Write Hazard Control Register0)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short MB0:1;
				unsigned short MB1:1;
				unsigned short MB2:1;
				unsigned short MB3:1;
				unsigned short MB4:1;
				unsigned short MB5:1;
				unsigned short :10;
			} BIT;
		} WHCR0;
		unsigned short skip_whcr0[SKIP_NUM];

		/*WHCR1(Write Hazard Control Register1)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short MB0:1;
				unsigned short MB1:1;
				unsigned short MB2:1;
				unsigned short MB3:1;
				unsigned short MB4:1;
				unsigned short MB5:1;
				unsigned short :10;
			} BIT;
		} WHCR1;
		unsigned short skip_whcr1[SKIP_NUM];

		/*MSLR(Mail Send Limit time Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short LMT0:1;
				unsigned short LMT1:1;
				unsigned short LMT2:1;
				unsigned short LMT3:1;
				unsigned short LMT4:1;
				unsigned short LMT5:1;
				unsigned short LMT6:1;
				unsigned short LMT7:1;
				unsigned short LMT8:1;
				unsigned short LMT9:1;
				unsigned short LMT10:1;
				unsigned short LMT11:1;
				unsigned short LMT12:1;
				unsigned short :3;
			} BIT;
		} MSLR;
		unsigned short skip_mslr[SKIP_NUM];

		/*MSRR(Mail Send Result Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short RLT0:1;
				unsigned short RLT1:1;
				unsigned short RLT2:1;
				unsigned short RLT3:1;
				unsigned short RLT4:1;
				unsigned short RLT5:1;
				unsigned short RLT6:1;
				unsigned short RLT7:1;
				unsigned short RLT8:1;
				unsigned short RLT9:1;
				unsigned short RLT10:1;
				unsigned short RLT11:1;
				unsigned short RLT12:1;
				unsigned short :3;
			} BIT;
		} MSRR;
		unsigned short skip_msrr[SKIP_NUM];

		/*MESR(Mail Error Status Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short NORDY:1;
				unsigned short NOEX:1;
				unsigned short TOUT:1;
				unsigned short SZFLT:1;
				unsigned short LMFLT:1;
				unsigned short STOP:1;
				unsigned short :10;
			} BIT;
		} MESR;
		unsigned short skip_mesr[SKIP_NUM];

		/*MSCR(Mail Send Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short SZ0:1;
				unsigned short SZ1:1;
				unsigned short SZ2:1;
				unsigned short SZ3:1;
				unsigned short SZ4:1;
				unsigned short SZ5:1;
				unsigned short :2;
				unsigned short DST0:1;
				unsigned short DST1:1;
				unsigned short DST2:1;
				unsigned short DST3:1;
				unsigned short DST4:1;
				unsigned short DST5:1;
				unsigned short SEND:1;
				unsigned short ERR:1;
			} BIT;
		} MSCR;
		unsigned short skip_mscr[SKIP_NUM];

		/*MR0CR(Mail Receive 0 Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short SZ0:1;
				unsigned short SZ1:1;
				unsigned short SZ2:1;
				unsigned short SZ3:1;
				unsigned short SZ4:1;
				unsigned short SZ5:1;
				unsigned short RDY:1;
				unsigned short RCV:1;
				unsigned short SRC0:1;
				unsigned short SRC1:1;
				unsigned short SRC2:1;
				unsigned short SRC3:1;
				unsigned short SRC4:1;
				unsigned short SRC5:1;
				unsigned short :2;
			} BIT;
		} MR0CR;
		unsigned short skip_mr0cr[SKIP_NUM];

		/*MR1CR(Mail Receive 1 Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short SZ0:1;
				unsigned short SZ1:1;
				unsigned short SZ2:1;
				unsigned short SZ3:1;
				unsigned short SZ4:1;
				unsigned short SZ5:1;
				unsigned short RDY:1;
				unsigned short RCV:1;
				unsigned short SRC0:1;
				unsigned short SRC1:1;
				unsigned short SRC2:1;
				unsigned short SRC3:1;
				unsigned short SRC4:1;
				unsigned short SRC5:1;
				unsigned short :2;
			} BIT;
		} MR1CR;
		unsigned short skip_mr1cr[SKIP_NUM];

		/*CCTR(Care CounTer Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short LCC0:1;
				unsigned short LCC1:1;
				unsigned short LCC2:1;
				unsigned short LCC3:1;
				unsigned short LCC4:1;
				unsigned short LCC5:1;
				unsigned short LCC6:1;
				unsigned short LCC7:1;
				unsigned short MCC0:1;
				unsigned short MCC1:1;
				unsigned short MCC2:1;
				unsigned short MCC3:1;
				unsigned short MCC4:1;
				unsigned short MCC5:1;
				unsigned short MCC6:1;
				unsigned short MCC7:1;
			} BIT;
		} CCTR;
		unsigned short skip_cctr[SKIP_NUM];

		/*UTCR(UTility pin Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short SS1:1;
				unsigned short OE1:1;
				unsigned short :6;
				unsigned short SS2:1;
				unsigned short OE2:1;
				unsigned short :6;
			} BIT;
		} UTCR;
		unsigned short skip_utcr[SKIP_NUM];

		/*QCR(Care CounTer Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short TS0:1;
				unsigned short TS1:1;
				unsigned short TS2:1;
				unsigned short TS3:1;
				unsigned short TS4:1;
				unsigned short TS5:1;
				unsigned short TQ:1;
				unsigned short PING:1;
				unsigned short TYP0:1;
				unsigned short TYP1:1;
				unsigned short TYP2:1;
				unsigned short TYP3:1;
				unsigned short TYP4:1;
				unsigned short :3;
			} BIT;
		} QCR;
		unsigned short skip_qcr[SKIP_NUM];

		/*NFSR(New Final Station Register)*/
		union {
			unsigned short WORD;
			struct {				
				unsigned short NFS0:1;
				unsigned short NFS1:1;
				unsigned short NFS2:1;
				unsigned short NFS3:1;
				unsigned short NFS4:1;
				unsigned short NFS5:1;
				unsigned short :10;
			} BIT;
		} NFSR;
		unsigned short skip_nfsr[SKIP_NUM];

		/*FSR(Final Station Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short FS0:1;
				unsigned short FS1:1;
				unsigned short FS2:1;
				unsigned short FS3:1;
				unsigned short FS4:1;
				unsigned short FS5:1;
				unsigned short :10;
			} BIT;
		} FSR;
		unsigned short skip_fsr[SKIP_NUM];

		/*BCR(Basic Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short SA0:1;
				unsigned short SA1:1;
				unsigned short SA2:1;
				unsigned short SA3:1;
				unsigned short SA4:1;
				unsigned short SA5:1;
				unsigned short BPS0:1;
				unsigned short BPS1:1;
				unsigned short OWN0:1;
				unsigned short OWN1:1;
				unsigned short OWN2:1;
				unsigned short OWN3:1;
				unsigned short OWN4:1;
				unsigned short OWN5:1;
				unsigned short :1;
				unsigned short LFS:1;
			} BIT;
		} BCR;
		unsigned short skip_bcr[SKIP_NUM];

		/*INT0CR(INTerrupt 0 Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ALM:1;
				unsigned short DR:1;
				unsigned short MR:1;
				unsigned short MSF:1;
				unsigned short MGNE:1;
				unsigned short MGNC:1;
				unsigned short RC:1;
				unsigned short RSTP:1;
				unsigned short RSTR:1;
				unsigned short MC:1;
				unsigned short LOK:1;
				unsigned short LNG:1;
				unsigned short BD:1;
				unsigned short RO:1;
				unsigned short PR:1;
				unsigned short JD:1;
			} BIT;
		} INT0CR;
		unsigned short skip_int0cr[SKIP_NUM];

		/*INT1CR(INTerrupt 1 Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ALM:1;
				unsigned short DR:1;
				unsigned short MR:1;
				unsigned short MSF:1;
				unsigned short MGNE:1;
				unsigned short MGNC:1;
				unsigned short RC:1;
				unsigned short RSTP:1;
				unsigned short RSTR:1;
				unsigned short MC:1;
				unsigned short LOK:1;
				unsigned short LNG:1;
				unsigned short BD:1;
				unsigned short RO:1;
				unsigned short PR:1;
				unsigned short JD:1;
			} BIT;
		} INT1CR;
		unsigned short skip_int1cr[SKIP_NUM];

		/*IT0CR(Interrupt Timing 0 Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ALM0:1;
				unsigned short ALM1:1;
				unsigned short ALM2:1;
				unsigned short ALM3:1;
				unsigned short ALM4:1;
				unsigned short ALM5:1;
				unsigned short ALM6:1;
				unsigned short :1;
				unsigned short DR0:1;
				unsigned short DR1:1;
				unsigned short DR2:1;
				unsigned short DR3:1;
				unsigned short DR4:1;
				unsigned short DR5:1;
				unsigned short DR6:1;
				unsigned short :1;
			} BIT;
		} IT0CR;
		unsigned short skip_it0cr[SKIP_NUM];

		/*IT1CR(Interrupt Timing 1 Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ALM0:1;
				unsigned short ALM1:1;
				unsigned short ALM2:1;
				unsigned short ALM3:1;
				unsigned short ALM4:1;
				unsigned short ALM5:1;
				unsigned short ALM6:1;
				unsigned short :1;
				unsigned short DR0:1;
				unsigned short DR1:1;
				unsigned short DR2:1;
				unsigned short DR3:1;
				unsigned short DR4:1;
				unsigned short DR5:1;
				unsigned short DR6:1;
				unsigned short :1;
			} BIT;
		} IT1CR;
		unsigned short skip_it1cr[SKIP_NUM];

		/*INT0SR(INTerrupt 0 Status Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ALM:1;
				unsigned short DR:1;
				unsigned short MR:1;
				unsigned short MSF:1;
				unsigned short MGNE:1;
				unsigned short MGNC:1;
				unsigned short RC:1;
				unsigned short RSTP:1;
				unsigned short RSTR:1;
				unsigned short MC:1;
				unsigned short LOK:1;
				unsigned short LNG:1;
				unsigned short BD:1;
				unsigned short RO:1;
				unsigned short PR:1;
				unsigned short JD:1;
			} BIT;
		} INT0SR;
		unsigned short skip_int0sr[SKIP_NUM];

		/*INT1SR(INTerrupt 1 Status Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ALM:1;
				unsigned short DR:1;
				unsigned short MR:1;
				unsigned short MSF:1;
				unsigned short MGNE:1;
				unsigned short MGNC:1;
				unsigned short RC:1;
				unsigned short RSTP:1;
				unsigned short RSTR:1;
				unsigned short MC:1;
				unsigned short LOK:1;
				unsigned short LNG:1;
				unsigned short BD:1;
				unsigned short RO:1;
				unsigned short PR:1;
				unsigned short JD:1;
			} BIT;
		} INT1SR;
		unsigned short skip_int1sr[SKIP_NUM];

		/*SSR(System Status Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short :4;
				unsigned short MGNE:1;
				unsigned short MGNC:1;
				unsigned short MR:1;
				unsigned short MSE:1;
				unsigned short RO:1;
				unsigned short JD:1;
				unsigned short BD:1;
				unsigned short DR:1;
				unsigned short LOK:1;
				unsigned short LNG:1;
				unsigned short NM:1;
				unsigned short MC:1;
			} BIT;
		} SSR;
		unsigned short skip_ssr[SKIP_NUM];

		/*SCR(System Control Register)*/
		union {
			unsigned short WORD;
			struct {
				unsigned short ST0:1;
				unsigned short ST1:1;
				unsigned short ST2:1;
				unsigned short ST3:1;
				unsigned short ST4:1;
				unsigned short ST5:1;
				unsigned short ST6:1;
				unsigned short :1;
				unsigned short START:1;
				unsigned short RUN:1;	
				unsigned short CALL:1;
				unsigned short BRK:1;
				unsigned short OC:1;
				unsigned short SNF:1;
				unsigned short LF:1;
				unsigned short GMM:1;
			} BIT;
		} SCR;
		unsigned short skip_scr[SKIP_NUM];

		/*CCR(Chip Code Register)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} CCR;

		/*RHPB0(Read Hazard Protection Buffer 0)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} RHPB0;

		/*RHPB1(Read Hazard Protection Buffer 1)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} RHPB1;
		
		/*WHPB0(Write Hazard Protection Buffer 0)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} WRHPB0;

		/*WHPB1(Write Hazard Protection Buffer 1)*/
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		} WRHPB1;
		
		struct {
			short reserve[56*(SKIP_NUM+1)];
		} RESV;
	} REG;

	/*MRB0(Mail Receive Buffer0)*/
	struct {
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		}RECV[32];
	} MRB0;

	/*MRB1(Mail Receive Buffer1)*/
	struct {
		struct {
			struct {
				unsigned short DATA;
				unsigned short skip[SKIP_NUM];
			}DATA[4];
		}RECV[32];
	} MRB1;

	struct {
		short reserve[256*(SKIP_NUM+1)];
	} RESV;

};

#define	MKY43	(*(volatile struct st_mky43 *)0xA0000000)


