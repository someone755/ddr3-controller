/////////////////////////////////////////////////
// Memory configuration (MR) and timing parametrization
/////////////////////////////////////////////////
// #################### DLL OFF, <125 MHZ #################### 
if	(p_DDR_CK_PS > 7999) begin: DLL
localparam lp_CL = 6;
localparam lp_CWL = 6;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL - 1;	// DLL OFF: -1
localparam	lpdiv_WL	= lp_WL/2;
localparam	lpdiv_RL	= lp_RL/2;
// Modified parameters:
localparam	lpdiv_RTW	= (lp_RL + lp_CCD + 2 - lp_WL + 1)/2;	// READ-to-WRITE = RL + CCD + 2 CK - WL (+1 to round off correctly)
localparam	lpdiv_WTR	= (lp_WL + lp_BL/2 + (p_WTR/p_DDR_CK_PS+1))/2;	// WRITE-to-READ = WL + BL/2 + WTR (WTR after WRITE DQS postamble)
localparam	lpdiv_WR	= 5;	// WRITE-to-PRECHARGE = WL + BL/2 + WR (WR after WRITE DQS postamble)
	// WR (min, DLL OFF) = 4 CK, but minimum allowed in MR0 is 5 CK

// MRx is {BA[3:0], A[13:0]}
					  /* 16 15 14 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 17'b__0__0__0__0__0__0__0__1__1__0__0__1__0__0__0__0__0;
localparam lp_MR1 = 17'b__0__0__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
localparam lp_MR2 = 17'b__0__1__0__0__0__0__0__0__0__0__0__0__0__1__0__0__0;
localparam lp_MR3 = 17'b__0__1__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0;

// #################### 300-333 MHz #################### 
end else if ((p_DDR_CK_PS > 2999) & (p_DDR_CK_PS < 3334)) begin: DLL
localparam lp_CL = 5;
localparam lp_CWL = 5;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL;
localparam	lpdiv_WL	= lp_WL/2;
localparam	lpdiv_RL	= lp_RL/2;
// Modified parameters:
localparam	lpdiv_RTW	= (lp_RL + lp_CCD + 2 - lp_WL + 1)/2;	// READ-to-WRITE = RL + CCD + 2 CK - WL (+1 to round off correctly)
localparam	lpdiv_WTR	= (lp_WL + lp_BL/2 + (p_WTR/p_DDR_CK_PS+1))/2;	// WRITE-to-READ = WL + BL/2 + WTR (WTR after WRITE DQS postamble)
localparam	lpdiv_WR	= (lp_WL + lp_BL/2 + (p_WR/p_DDR_CK_PS+1))/2;	// WRITE-to-PRECHARGE = WL + BL/2 + WR (WR after WRITE DQS postamble)
	// WR = roundup(tWR/tCK) = roundup(15 ns / (3.3~3.0 ss)) = 5 CK
					  /* 16 15 14 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 17'b__0__0__0__0__0__0__0__1__1__0__0__0__1__0__0__0__0;
localparam lp_MR1 = 17'b__0__0__1__0__0__0__0__0__0__0__1__0__0__0__1__0__0;
localparam lp_MR2 = 17'b__0__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
localparam lp_MR3 = 17'b__0__1__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0;

// #################### 333-400 MHz #################### 
end else if ((p_DDR_CK_PS > 2499) & (p_DDR_CK_PS < 3334)) begin: DLL
localparam lp_CL = 6;
localparam lp_CWL = 5;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL;
localparam	lpdiv_WL	= lp_WL/2;
localparam	lpdiv_RL	= lp_RL/2;
// Modified parameters:
localparam	lpdiv_RTW	= (lp_RL + lp_CCD + 2 - lp_WL + 1)/2;	// READ-to-WRITE = RL + CCD + 2 CK - WL (+1 to round off correctly)
localparam	lpdiv_WTR	= (lp_WL + lp_BL/2 + (p_WTR/p_DDR_CK_PS+1))/2;	// WRITE-to-READ = WL + BL/2 + WTR (WTR after WRITE DQS postamble)
localparam	lpdiv_WR	= (lp_WL + lp_BL/2 + (p_WR/p_DDR_CK_PS+1))/2;	// WRITE-to-PRECHARGE = WL + BL/2 + WR (WR after WRITE DQS postamble)
	// WR = roundup(tWR/tCK) = roundup(15 ns / (3.0~2.5 ss)) = 6 CK
					  /* 16 15 14 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 17'b__0__0__0__0__0__0__1__0__1__0__0__1__0__0__0__0__0;
localparam lp_MR1 = 17'b__0__0__1__0__0__0__0__0__0__0__1__0__0__0__1__0__0;
localparam lp_MR2 = 17'b__0__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
localparam lp_MR3 = 17'b__0__1__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0;

// #################### TODO 400-466 MHz #################### 
end else if ((p_DDR_CK_PS > 2141) & (p_DDR_CK_PS < 2499)) begin: DLL
localparam lp_CL = 7;
localparam lp_CWL = 6;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL;
localparam	lpdiv_WL	= lp_WL/2;
localparam	lpdiv_RL	= lp_RL/2;
// Modified parameters:
localparam	lpdiv_RTW	= (lp_RL + lp_CCD + 2 - lp_WL + 1)/2;	// READ-to-WRITE = RL + CCD + 2 CK - WL (+1 to round off correctly)
localparam	lpdiv_WTR	= (lp_WL + lp_BL/2 + (p_WTR/p_DDR_CK_PS+1))/2;	// WRITE-to-READ = WL + BL/2 + WTR (WTR after WRITE DQS postamble)
localparam	lpdiv_WR	= (lp_WL + lp_BL/2 + (p_WR/p_DDR_CK_PS+1))/2;	// WRITE-to-PRECHARGE = WL + BL/2 + WR (WR after WRITE DQS postamble)
		// WR = roundup(tWR/tCK) = roundup(15 ns / (2.500~2.143 ss)) = 7 (466 MHz)
		// WR = roundup(tWR/tCK) = roundup(15 ns / (2.143~1.875 ns)) = 8 (533 MHz)
					  /* 16 15 14 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 17'b__0__0__0__0__0__0__1__1__1__0__0__1__1__0__0__0__0;
localparam lp_MR1 = 17'b__0__0__1__0__0__0__0__0__0__0__1__0__0__0__1__0__0;
localparam lp_MR2 = 17'b__0__1__0__0__0__0__0__0__0__0__0__0__0__1__0__0__0;
localparam lp_MR3 = 17'b__0__1__1__0__0__0__0__0__0__0__0__0__0__0__0__0__0;

// #################### 125-300, >466 MHz ERROR #################### 
end else begin: DLL
localparam lp_CL = 0;
localparam lp_CWL = 0;
localparam lp_AL = 0;
unsupported_frequency_error_generation BAD_FREQ();
end

localparam	lpdiv_WL_MAX = 6/2;	// only CWL 5, 6 supported (no way this does > 533 MHz)
									// if CWL = 5, OSERDES must be triggered one CLKDIV period earlier
localparam lp_MR3_MPR = 14'b__0__0__0__0__0__0__0__0__0__0__0__1__0__0; // to be used when reading 1010 from MPR

/////////////////////////////////////////////////
// Timing parameter to DIV CK conversion
/////////////////////////////////////////////////
localparam	lp_CCD		= 4;	// 4 CK between RD-to-RD or WR-to-WR
localparam	lp_BL		= 8;	// 8 transfers per RD/WR command

localparam	lpdiv_MRD	= lp_CCD/2 - 1;	// MRS cycle time (4 CK)
localparam	lpdiv_CCD	= lp_BL/2/2 - 1;	// WRITE-to-WRITE (actually just BL/2 in DDR CK)(CCD = CAS#-to-CAS# delay)

localparam	lp_DIV_FREQ_MHZ	= p_DDR_FREQ_MHZ/2;
localparam	lp_DIV_CK_PS	= `ck2ps(lp_DIV_FREQ_MHZ);

// Parameters directly following from DDR nCKs:
localparam	lpdiv_RAS	= p_RAS/lp_DIV_CK_PS;	// ACTIVATE-to-PRECHARGE
localparam	lpdiv_RCD	= p_RCD/lp_DIV_CK_PS;	// ACTIVATE-to-READ or ACTIVATE-to-WRITE delay
localparam	lpdiv_REFI	= p_REFI/lp_DIV_CK_PS-10;	// Average periodic refresh interval // (-1) because it needs to be rounded DOWN
localparam	lpdiv_RFC_MIN	= p_RFC_MIN/lp_DIV_CK_PS;	// REFRESH-to-ACTIVATE or REFRESH-to-REFRESH
localparam	lpdiv_RFC_MAX	= p_RFC_MAX/lp_DIV_CK_PS-1;
localparam	lpdiv_RP	= p_RP/lp_DIV_CK_PS;	// PRECHARGE-to-ACTIVATE or PRECHARGE-to-REFRESH
localparam	lpdiv_RRD	= p_RRD/lp_DIV_CK_PS;	// ACTIVATE-to-ACTIVATE in different banks
localparam	lpdiv_RTP	= p_RTP/lp_DIV_CK_PS;	// READ-to-PRECHARGE
//localparam	lpdiv_WTR	= p_WTR/lp_DIV_CK_PS;	// WRITE-to-READ
//localparam	lpdiv_WR	= p_WR/lp_DIV_CK_PS;	// WRITE-to-PRECHARGE (WRITE recovery time)
localparam	lpdiv_XPR	= p_XPR/lp_DIV_CK_PS;	// Exit reset from CKE HIGH to valid command
localparam	lpdiv_MOD	= p_MOD/lp_DIV_CK_PS;	// MRS-to-non-MRS (MRS update delay)
localparam	lpdiv_ZQINIT	= p_ZQINIT/lp_DIV_CK_PS;	// Long calibration time

//localparam	lpdiv_WRAP	= (DLL.lp_WL + DLL.lp_BL/2 + ((p_WR + p_RP)/p_DDR_CK_PS+1))/2;	// WRITE with AP exit time
//localparam	lpdiv_RDAP	= (p_RTP + p_RP)/p_DDR_CK_PS;	// READ with AP exit time


// Initialization timing parameters:
`ifdef SIMULATION
localparam lpdiv_NRST_LO = 2 * p_DDR_FREQ_MHZ/2 - 1; // RESET#: After power stable, RESET# held LOW for >200 us.
localparam lpdiv_CKE_LO = 7 * p_DDR_FREQ_MHZ/2 - 1; // CKE: After RESET# transitions HIGH wait >500 us with CKE LOW.
`else
localparam lpdiv_NRST_LO = 250 * p_DDR_FREQ_MHZ/2 - 1; // RESET#: After power stable, RESET# held LOW for >200 us.
localparam lpdiv_CKE_LO = 501 * p_DDR_FREQ_MHZ/2 - 1; // CKE: After RESET# transitions HIGH wait >500 us with CKE LOW.
`endif
localparam lp_STATE_TMR_WIDTH = $clog2(lpdiv_CKE_LO);
//localparam lp_IDLE_TMR_WIDTH = $clog2(lpdiv_RFC_MAX);
//localparam lpdiv_INIT_CTR_PLAY = 50/2;
//localparam lpdiv_INIT_CTR_START = lpdiv_NRST_LO + lpdiv_CKE_LO + lpdiv_XPR + 3 * lpdiv_MRD + lpdiv_MOD + lpdiv_ZQINIT + lpdiv_INIT_CTR_PLAY;

/////////////////////////////////////////////////
// Command defines; {(nCS,) nRAS, nCAS, nWE}; See Micron datasheet Table 87
/////////////////////////////////////////////////
localparam lp_CMD_MRS	= 3'b0000;	// MODE REGISTER SET
localparam lp_CMD_REF	= 3'b0001;	// REFRESH
localparam lp_CMD_PRE	= 3'b0010;	// & A10 LOW: Single-bank PRECHARGE
									// & A10 HIGH: PRECHARGE all banks
localparam lp_CMD_ACT	= 3'b0011;	// Bank ACTIVATE
localparam lp_CMD_WR	= 3'b0100;	// & A10 LOW: normal WRITE (assuming BL8MRS)
									// & A10 HIGH: WRITE with auto precharge
localparam lp_CMD_RD	= 3'b0101;	// & A10 LOW: normal READ
									// & A10 HIGH: READ with auto precharge
localparam lp_CMD_NOP	= 3'b0111;	// NO OPERATION
localparam lp_CMD_ZQCL	= 3'b0110;	// ZQ CALIBRATION LONG

/////////////////////////////////////////////////
// State machine state definitions (one per cmd + init))
/////////////////////////////////////////////////
localparam STATE_INIT	= 4'd0;
localparam STATE_MRS	= 4'd1;
localparam STATE_REF	= 4'd2;
localparam STATE_PRE	= 4'd3;
localparam STATE_ACT	= 4'd4;
localparam STATE_WR		= 4'd5;
localparam STATE_RD		= 4'd6;
localparam STATE_IDLE	= 4'd7;
localparam STATE_ZQCL	= 4'd8;

/////////////////////////////////////////////////
// Input FIFO parameters
/////////////////////////////////////////////////
localparam	lp_CMDFIFO_OP_WR	= 1'b0;
localparam	lp_CMDFIFO_OP_RD	= 1'b1;
localparam	lp_CMDFIFO_OP_WIDTH	= 1;
// CMD FIFO stores OP (r/w), address (bank+row+col), wr data for writes (8*DQ), and data mask (1 bit per burst word)
localparam	lp_CMDFIFO_WIDTH	= lp_CMDFIFO_OP_WIDTH + p_BANK_W + p_ROW_W
									+ p_COL_W + 8 * p_DQ_W + lp_BL;
									
									
									