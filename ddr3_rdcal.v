`timescale 1ns / 1ps
/** To use the RDCAL module:
  *		- set p_IDELAY_INIT_DQ and p_IDELAY_INIT_DQS to '0'
  *		- set p_IDELAY_TYPE to "VAR_LOAD"
  *	TODO: Complete description
  */
module ddr3_rdcal(
	input	i_clk_div,
	input	i_rdcal_start,
	
	output	o_rdcal_done,
	output	o_rdcal_err,
	
	output	o_dqs_delay_ld,
	output	o_dq_delay_ld,
	
	output	[4:0]	o5_dqs_idelay_cnt,
	output	[4:0]	o5_dq_idelay_cnt,
	
	input	i_phy_init_done,
	input	i_phy_rddata_valid,
	input	[127:0]	in_phy_rddata,
	

	input	i_phy_cmd_full,

	input	i_rdc_cmd_en,
	input	i_rdc_cmd_sel,
	input	[2:0]	i3_rdc_bank,
	input	[13:0]	i14_rdc_row,
	input	[9:0]	i10_rdc_col,
	input	[127:0]	i128_rdc_wrdata,
	
	output	o_phy_cmd_en,
	output	o_phy_cmd_sel,
	output	[2:0]	o3_phy_bank,
	output	[13:0]	o14_phy_row,
	output	[9:0]	o10_phy_col,
	output	[127:0]	o128_phy_wrdata
);

reg	r_phy_cmd_en;
reg	r_phy_cmd_sel;

reg	[2:0]	r3_bank;
reg	[13:0]	r14_row;
reg	[9:0]	r10_col;

reg	[127:0]	r128_wrdata;

reg	[2:0]	r3_calib_state	= 4'b0;

reg	[127:0]	r128_caldata = 'h0000_ffff_0000_ffff_0000_ffff_0000_ffff;
reg	[4:0]	r5_dqs_delay_cnt, r5_dq_delay_cnt;
reg	r_dqs_delay_ld, r_dq_delay_ld;


reg	[4:0]	r5_calib_width_best,	// largest number of successful read attempts per DQ tap (as a function of DQS taps)
		r5_calib_width,	// number of successful read attempts per DQ tap (as a function of DQS taps) for the currently tested DQ tap value
		r5_calib_dq_best,	// DQ tap value corresponding to the widest range of workable DQS tap settings (denoted by r5_calib_width_best)
		r5_calib_dqs_min,	// minimum workable value of DQS tap setting for the currently tested DQ tap value
		r5_calib_dqs_min_best;
reg	r_rd_cal_done;
reg	r_rd_cal_err;


always @(posedge i_clk_div) begin: rd_calibration
	r_dqs_delay_ld <= 1'b0;
	r_dq_delay_ld <= 1'b0;
	
	r_phy_cmd_en <= 1'b0;

	case (r3_calib_state)
	'd0: begin	// init IDELAY, write calibration word to DRAM
		if (i_rdcal_start && !i_phy_cmd_full && i_phy_init_done) begin
			r3_bank <= 3'b000;
			r14_row <= 14'd0;
			r10_col <= 10'd0;
			r128_wrdata <= r128_caldata;
	
			r_phy_cmd_en <= 1'b1;	//	 write calibration word to SDRAM
			r_phy_cmd_sel <= 1'b0;
			
			// Reset tap and counter values
			r_rd_cal_done <= 1'b0;
			
			r5_calib_width_best <= 5'b0;
			r5_calib_width <= 5'b0;
			r5_calib_dq_best <= 5'b0;
			r5_calib_dqs_min <= 5'b0;
			r5_calib_dqs_min_best <= 5'b0;
			
			r5_dqs_delay_cnt <= 5'd2;	// keep DQS tap value minimum 2 taps larger than DQ tap value
			r5_dq_delay_cnt <= 5'd0;
			r_dqs_delay_ld <= 1'b1;
			r_dq_delay_ld <= 1'b1;
			
			r3_calib_state <= 'd1;
		end
	end
	'd1: begin	// repeat IDELAY tick (IDELAY tap settings can be buggy)
		r_dqs_delay_ld <= 1'b1;
		r_dq_delay_ld <= 1'b1;
		r3_calib_state <= 'd2;
	end
	'd2: begin	// read calibration word from SDRAM
		r3_bank <= 3'b000;
		r14_row <= 14'd0;
		r10_col <= 10'd0;
		if (!i_phy_cmd_full) begin
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b1;
			
			r3_calib_state <= 'd3;
		end
	end
	'd3: begin	// log whether current IDELAY tap counts are okay
		if (i_phy_rddata_valid) begin
			if (in_phy_rddata == r128_caldata) begin
				r5_calib_width <= r5_calib_width + 1'b1; // if ok, increase current calib_width
				if (r5_calib_width == 5'd0)
					r5_calib_dqs_min <= r5_dqs_delay_cnt;	// remember first valid DQS tap count
			end
			r3_calib_state <= 'd4;
		end
	end
	'd4: begin	// save best attempt, decide next state (increment DQS, DQ, or finish calibration?)
		if (r5_calib_width > r5_calib_width_best) begin
			r5_calib_width_best <= r5_calib_width;
			r5_calib_dq_best <= r5_dq_delay_cnt;
			r5_calib_dqs_min_best <= r5_calib_dqs_min;
		end
		if (r5_dqs_delay_cnt == 'd31) begin
			if (r5_dq_delay_cnt == 'd29) begin	// tap testing complete
				r3_calib_state <= 'd5;
			end else begin	// increment DQ tap, reset DQS tap to DQ+2
				r5_dq_delay_cnt <= r5_dq_delay_cnt + 1;
				r5_dqs_delay_cnt <= r5_dq_delay_cnt + 3;
				r_dqs_delay_ld <= 1'b1;
				r_dq_delay_ld <= 1'b1;
				
				r5_calib_width <= 5'b0;
				r3_calib_state <= 'd1;
			end
		end else begin	// only increment DQS tap
			r5_dqs_delay_cnt <= r5_dqs_delay_cnt + 1;
			r_dqs_delay_ld <= 1'b1;
			r_dq_delay_ld <= 1'b1;
			r3_calib_state <= 'd1;
		end
	end
	'd5: begin	// set measured best tap values for DQ and DQS ISERDESE
		r5_dq_delay_cnt <= r5_calib_dq_best;	// recall best DQ value
		r5_dqs_delay_cnt <= (r5_calib_width_best/2) + r5_calib_dqs_min_best; // move data strobe into center of valid window
		r_dqs_delay_ld <= 1'b1;
		r_dq_delay_ld <= 1'b1;
		
		r3_calib_state <= 'd6;
	end
	'd6: begin	// raise cal_done flag, repeat IDELAY tick (IDELAY tap settings can be buggy)
		r_dqs_delay_ld <= 1'b1;
		r_dq_delay_ld <= 1'b1;

		r_rd_cal_err <= (r5_dqs_delay_cnt == 5'b0);

		r_rd_cal_done <= 1'b1;
		r3_calib_state <= 'd7;
	end
	'd7: begin	// extend rd_cal_done for 1 cycle (make it safer to disable rdcal)
		r_rd_cal_done <= 1'b1;
		r3_calib_state <= 'd0;
	end
	default ; // should not be reached
	endcase
end

assign o_dqs_delay_ld = r_dqs_delay_ld;
assign o_dq_delay_ld = r_dq_delay_ld;

assign o5_dqs_idelay_cnt = r5_dqs_delay_cnt;
assign o5_dq_idelay_cnt = r5_dq_delay_cnt;

assign {o3_phy_bank, o14_phy_row, o10_phy_col, o128_phy_wrdata, o_phy_cmd_en, o_phy_cmd_sel} = (r_rd_cal_done)
	? {i3_rdc_bank, i14_rdc_row, i10_rdc_col, i128_rdc_wrdata, i_rdc_cmd_en, i_rdc_cmd_sel}
	: {r3_bank, r14_row, r10_col, r128_wrdata, r_phy_cmd_en, r_phy_cmd_sel};
assign o_rdcal_done = r_rd_cal_done;
assign o_rdcal_err = r_rd_cal_err; 

endmodule

