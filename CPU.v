//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Final Project: Customized ISA Processor 
//   Author              : Hsi-Hao Huang
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : CPU.v
//   Module Name : CPU.v
//   Release version : V1.0 (Release Date: 2023-May)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module CPU(

				clk,
			  rst_n,
  
		   IO_stall,

         awid_m_inf,
       awaddr_m_inf,
       awsize_m_inf,
      awburst_m_inf,
        awlen_m_inf,
      awvalid_m_inf,
      awready_m_inf,
                    
        wdata_m_inf,
        wlast_m_inf,
       wvalid_m_inf,
       wready_m_inf,
                    
          bid_m_inf,
        bresp_m_inf,
       bvalid_m_inf,
       bready_m_inf,
                    
         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,
                    
      arready_m_inf, 
          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf 

);
// Input port
input  wire clk, rst_n;
// Output port
output reg  IO_stall;

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;

// AXI Interface wire connecttion for pseudo DRAM read/write
/* Hint:
  your AXI-4 interface could be designed as convertor in submodule(which used reg for output signal),
  therefore I declared output of AXI as wire in CPU
*/



// axi write address channel 
output  wire [WRIT_NUMBER * ID_WIDTH-1:0]        awid_m_inf;
output  wire [WRIT_NUMBER * ADDR_WIDTH-1:0]    awaddr_m_inf;
output  wire [WRIT_NUMBER * 3 -1:0]            awsize_m_inf;
output  wire [WRIT_NUMBER * 2 -1:0]           awburst_m_inf;
output  wire [WRIT_NUMBER * 7 -1:0]             awlen_m_inf;
output  wire [WRIT_NUMBER-1:0]                awvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                awready_m_inf;
// axi write data channel 
output  wire [WRIT_NUMBER * DATA_WIDTH-1:0]     wdata_m_inf;
output  wire [WRIT_NUMBER-1:0]                  wlast_m_inf;
output  wire [WRIT_NUMBER-1:0]                 wvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                 wready_m_inf;
// axi write response channel
input   wire [WRIT_NUMBER * ID_WIDTH-1:0]         bid_m_inf;
input   wire [WRIT_NUMBER * 2 -1:0]             bresp_m_inf;
input   wire [WRIT_NUMBER-1:0]             	   bvalid_m_inf;
output  wire [WRIT_NUMBER-1:0]                 bready_m_inf;
// -----------------------------
// axi read address channel 
output  wire [DRAM_NUMBER * ID_WIDTH-1:0]       arid_m_inf;
output  wire [DRAM_NUMBER * ADDR_WIDTH-1:0]   araddr_m_inf;
output  wire [DRAM_NUMBER * 7 -1:0]            arlen_m_inf;
output  wire [DRAM_NUMBER * 3 -1:0]           arsize_m_inf;
output  wire [DRAM_NUMBER * 2 -1:0]          arburst_m_inf;
output  wire [DRAM_NUMBER-1:0]               arvalid_m_inf;
input   wire [DRAM_NUMBER-1:0]               arready_m_inf;
// -----------------------------
// axi read data channel 
input   wire [DRAM_NUMBER * ID_WIDTH-1:0]         rid_m_inf;
input   wire [DRAM_NUMBER * DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [DRAM_NUMBER * 2 -1:0]             rresp_m_inf;
input   wire [DRAM_NUMBER-1:0]                  rlast_m_inf;
input   wire [DRAM_NUMBER-1:0]                 rvalid_m_inf;
output  wire [DRAM_NUMBER-1:0]                 rready_m_inf;
// -----------------------------

//
//
// 
/* Register in each core:
  There are sixteen registers in your CPU. You should not change the name of those registers.
  TA will check the value in each register when your core is not busy.
  If you change the name of registers below, you must get the fail in this lab.
*/

reg signed [15:0] core_r0 , core_r1 , core_r2 , core_r3 ;
reg signed [15:0] core_r4 , core_r5 , core_r6 , core_r7 ;
reg signed [15:0] core_r8 , core_r9 , core_r10, core_r11;
reg signed [15:0] core_r12, core_r13, core_r14, core_r15;


//###########################################
//
// Wrtie down your design below
//
//###########################################


//================================================================
// wire & registers 
//================================================================

parameter signed OFFSET = 16'h1000 ;

//reg [10:0] pc, inst_group_base_pc;
//reg [11:0] inst_group_cap_pc;
reg signed [15:0] pc, inst_group_base_pc, inst_group_cap_pc;
reg dram_inst_access;
wire  dram_inst_fin, dram_inst_next_addr;


reg [15:0] inst;
wire [2:0] opcode;
wire [3:0] rs, rt, rd;
wire func;
wire signed [4:0] immediate;
wire [12:0] address;
reg signed [15:0] rs_value, rt_value, rd_value; //comb reg
wire signed [15:0] data_addr;
wire is_r_type;
wire is_load;

reg data_wr;
reg data_access;
wire data_fin;
wire [15:0] data_out;


//SRAM
wire [15:0] sram_q;
reg sram_wen;  //write:0  read:1
reg [6:0] sram_a;
wire [15:0] sram_d;


//================================================================
// FSM State Declaration 
//================================================================
//FSM states
parameter S_IDLE = 'd0;
parameter S_IF1 = 'd1;
parameter S_IF2 = 'd2;
parameter S_EX = 'd3;
parameter S_MEM = 'd4;
parameter S_STALL = 'd5;
reg [2:0] current_state, next_state;

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) 
		current_state <= S_IDLE;
	else
		current_state <= next_state;
end

always @(*) begin
    case(current_state)
        S_IDLE: begin
            next_state = S_IF1;
        end
        S_IF1: begin  //fetch 128 insts from dram to sram
            if(dram_inst_fin)  next_state = S_IF2;
            else               next_state = current_state;
        end
        S_IF2: begin  //fetch inst from sram
            next_state = S_EX;
        end
        S_EX: begin
            if(opcode == 3'b010 || opcode == 3'b011)  next_state = S_MEM;
            else                                      next_state = S_STALL;
        end
        S_MEM: begin
            if(data_fin)    next_state = S_STALL;
            else            next_state = current_state;
        end
        S_STALL: begin
            //inst still in inst_group(sram)
            if(pc >= inst_group_base_pc && pc <= inst_group_cap_pc)
                next_state = S_IF2;
            //inst not in inst_group, fetch from dram
            else
                next_state = S_IF1;
        end
	  default : next_state = S_IDLE;
    endcase
end

//================================================================
// DRAM_INST_BRIDGE 
//================================================================

DRAM_INST_BRIDGE DRAM_INST_BRIDGE(
    //input
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(dram_inst_access),
    .pc(pc[10:0]),
    //output
    .out_valid(dram_inst_next_addr),
    .out_inst(sram_d),
    .out_fin(dram_inst_fin),

    //read
    .arid_m_inf(arid_m_inf[DRAM_NUMBER * ID_WIDTH-1:ID_WIDTH]),
    .araddr_m_inf(araddr_m_inf[DRAM_NUMBER * ADDR_WIDTH-1:ADDR_WIDTH]),
    .arlen_m_inf(arlen_m_inf[DRAM_NUMBER * 7 -1:7]),
    .arsize_m_inf(arsize_m_inf[DRAM_NUMBER * 3 -1:3]),
    .arburst_m_inf(arburst_m_inf[DRAM_NUMBER * 2 -1:2]),
    .arvalid_m_inf(arvalid_m_inf[1]),
    .arready_m_inf(arready_m_inf[1]),

    .rid_m_inf(rid_m_inf[DRAM_NUMBER * ID_WIDTH-1:ID_WIDTH]),
    .rdata_m_inf(rdata_m_inf[DRAM_NUMBER * DATA_WIDTH-1:DATA_WIDTH]),
    .rresp_m_inf(rresp_m_inf[DRAM_NUMBER * 2 -1:2]),
    .rlast_m_inf(rlast_m_inf[1]),
    .rvalid_m_inf(rvalid_m_inf[1]),
    .rready_m_inf(rready_m_inf[1])
);

//================================================================
// SRAM 
//================================================================

SRAM_INST M1(.Q(sram_q), .CLK(clk), .CEN(1'b0), .WEN(sram_wen), .A(sram_a), .D(sram_d), .OEN(1'b0));

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        sram_wen <= 'd1;
    else begin
        if(next_state == S_IF1)
            sram_wen <= 'd0;
        else
            sram_wen <= 'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        sram_a <= 'd0;
    else begin
        if(next_state == S_IF1) begin
            if(current_state == S_IDLE || current_state == S_STALL)
                sram_a <= 'd0;
            else if(dram_inst_next_addr)
               sram_a <= sram_a + 'd1;
        end
        //convert pc to inst_group address
        else if(next_state == S_IF2)
            sram_a <= pc - inst_group_base_pc;
    end
end

//================================================================
// design 
//================================================================


always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        pc <= 'd0;
    else begin
        if(current_state == S_EX) begin
            //branch on equal
            if(opcode == 3'b101 && (rs_value == rt_value))
                pc <= pc + 'd1 + immediate;
            //jump
            else if(opcode == 3'b100)
                pc <= address[11:1];
            else
                pc <= pc + 'd1;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        inst_group_base_pc <= 'd0;
        inst_group_cap_pc <= 'd0;
    end
    else begin
        if(next_state == S_IF1 && (current_state == S_IDLE || current_state == S_STALL)) begin
            if(pc < 'd1920) begin
                inst_group_base_pc <= pc;
                inst_group_cap_pc <= pc + 'd127;
            end
            //avoiding read address greater than 'h2000
            else begin
                inst_group_base_pc <= 'd1920;
                inst_group_cap_pc <= 'd2047;
            end
        end
    end
end

//S_IF1
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        dram_inst_access <= 'd0;
    else begin
        //send a in_valid pulse
        if(next_state == S_IF1 && current_state != S_IF1)
            dram_inst_access <= 'd1;
        else
            dram_inst_access <= 'd0;
    end
end

//S_EX
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        inst <= 'd0;
    else begin
        if(current_state == S_EX)
            inst <= sram_q;
    end
end
//assign inst = sram_q;
assign opcode = (current_state == S_EX) ? sram_q[15:13]: inst[15:13];
assign rs = (current_state == S_EX) ? sram_q[12:9]: inst[12:9];
assign rt = (current_state == S_EX) ? sram_q[8:5]: inst[8:5];
assign rd = (current_state == S_EX) ? sram_q[4:1]: inst[4:1];
assign func = (current_state == S_EX) ? sram_q[0]: inst[0];
assign immediate = (current_state == S_EX) ? sram_q[4:0]: inst[4:0];
assign address = (current_state == S_EX) ? sram_q[12:0]: inst[12:0];
assign data_addr = (rs_value + immediate) * 2 + OFFSET;

assign is_r_type = (opcode == 3'b000 || opcode == 3'b001);
assign is_load = (opcode == 3'b011);

always @(*) begin
    case(rs)
        'd0: rs_value = core_r0;
        'd1: rs_value = core_r1;
        'd2: rs_value = core_r2;
        'd3: rs_value = core_r3;
        'd4: rs_value = core_r4;
        'd5: rs_value = core_r5;
        'd6: rs_value = core_r6;
        'd7: rs_value = core_r7;
        'd8: rs_value = core_r8;
        'd9: rs_value = core_r9;
        'd10: rs_value = core_r10;
        'd11: rs_value = core_r11;
        'd12: rs_value = core_r12;
        'd13: rs_value = core_r13;
        'd14: rs_value = core_r14;
        'd15: rs_value = core_r15;
    endcase
end

always @(*) begin
    case(rt)
        'd0: rt_value = core_r0;
        'd1: rt_value = core_r1;
        'd2: rt_value = core_r2;
        'd3: rt_value = core_r3;
        'd4: rt_value = core_r4;
        'd5: rt_value = core_r5;
        'd6: rt_value = core_r6;
        'd7: rt_value = core_r7;
        'd8: rt_value = core_r8;
        'd9: rt_value = core_r9;
        'd10: rt_value = core_r10;
        'd11: rt_value = core_r11;
        'd12: rt_value = core_r12;
        'd13: rt_value = core_r13;
        'd14: rt_value = core_r14;
        'd15: rt_value = core_r15;
    endcase
end
/*
always @(*) begin
    case({opcode, func})
        //add
        4'b0001: rd_value = rs_value + rt_value;
        4'b0000: rd_value = rs_value - rt_value;
        4'b0011: rd_value = (rs_value < rt_value) ? 1: 0;
        4'b0010: rd_value = rs_value * rt_value;
        default: rd_value = 0;
    endcase
end
*/
always @(*) begin
    case(opcode)
        //add
        3'b000: begin
            if(func)
                rd_value = rs_value + rt_value;
            else
                rd_value = rs_value - rt_value;
        end
        3'b001: begin
            if(func)
                rd_value = (rs_value < rt_value) ? 1: 0;
            else
                rd_value = rs_value * rt_value;
        end
        default: rd_value = 0;
    endcase
end

//================================================================
// core register 
//================================================================

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r0 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd0)
            core_r0 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd0)
            core_r0 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r1 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd1)
            core_r1 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd1)
            core_r1 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r2 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd2)
            core_r2 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd2)
            core_r2 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r3 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd3)
            core_r3 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd3)
            core_r3 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r4 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd4)
            core_r4 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd4)
            core_r4 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r5 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd5)
            core_r5 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd5)
            core_r5 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r6 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd6)
            core_r6 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd6)
            core_r6 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r7 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd7)
            core_r7 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd7)
            core_r7 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r8 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd8)
            core_r8 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd8)
            core_r8 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r9 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd9)
            core_r9 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd9)
            core_r9 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r10 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd10)
            core_r10 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd10)
            core_r10 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r11 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd11)
            core_r11 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd11)
            core_r11 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r12 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd12)
            core_r12 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd12)
            core_r12 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r13 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd13)
            core_r13 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd13)
            core_r13 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r14 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd14)
            core_r14 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd14)
            core_r14 <= data_out;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        core_r15 <= 'd0;
    else begin
        if(current_state == S_EX && is_r_type && rd == 'd15)
            core_r15 <= rd_value;
        else if(current_state == S_MEM && next_state == S_STALL && is_load && rt == 'd15)
            core_r15 <= data_out;
    end
end

//================================================================
// DRAM_DATA_BRIDGE 
//================================================================

DRAM_DATA_BRIDGE DRAM_DATA_BRIDGE(
    //input
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(data_access),
    .addr(data_addr[11:1]), //[4'b0001, data_addr, 1'b0]
    .in_data(rt_value),
    .wr(data_wr), //1:read 0:write

    //output
    .out_valid(data_fin),
    .out_data(data_out),

    //write
    .awid_m_inf(awid_m_inf),
    .awaddr_m_inf(awaddr_m_inf),
    .awsize_m_inf(awsize_m_inf),
    .awburst_m_inf(awburst_m_inf),
    .awlen_m_inf(awlen_m_inf),
    .awvalid_m_inf(awvalid_m_inf),
    .awready_m_inf(awready_m_inf),
                    
    .wdata_m_inf(wdata_m_inf),
    .wlast_m_inf(wlast_m_inf),
    .wvalid_m_inf(wvalid_m_inf),
    .wready_m_inf(wready_m_inf),
                    
    .bid_m_inf(bid_m_inf),
    .bresp_m_inf(bresp_m_inf),
    .bvalid_m_inf(bvalid_m_inf),
    .bready_m_inf(bready_m_inf),

    //read
    .arid_m_inf(arid_m_inf[ID_WIDTH-1:0]),
    .araddr_m_inf(araddr_m_inf[ADDR_WIDTH-1:0]),
    .arlen_m_inf(arlen_m_inf[7 -1:0]),
    .arsize_m_inf(arsize_m_inf[3 -1:0]),
    .arburst_m_inf(arburst_m_inf[2 -1:0]),
    .arvalid_m_inf(arvalid_m_inf[0]),
    .arready_m_inf(arready_m_inf[0]),

    .rid_m_inf(rid_m_inf[ID_WIDTH-1:0]),
    .rdata_m_inf(rdata_m_inf[DATA_WIDTH-1:0]),
    .rresp_m_inf(rresp_m_inf[2 -1:0]),
    .rlast_m_inf(rlast_m_inf[0]),
    .rvalid_m_inf(rvalid_m_inf[0]),
    .rready_m_inf(rready_m_inf[0])
);

//mem
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        data_wr <= 'd0;
    else begin
        //load(read)
        if(current_state == S_EX && opcode == 3'b011)
            data_wr <= 'd1;
        //store(write)
        else if(current_state == S_EX && opcode == 3'b010)
            data_wr <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        data_access <= 'd0;
    else begin
        //send a in_valid pulse
        if(next_state == S_MEM && current_state == S_EX)
            data_access <= 'd1;
        else
            data_access <= 'd0;
    end
end


//output
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        IO_stall <= 'd1;
    else begin
        if(next_state == S_STALL)
            IO_stall <= 'd0;
        else
            IO_stall <= 'd1;
    end
end

endmodule

//================================================================
// sub module 
//================================================================

module DRAM_INST_BRIDGE(
                clk,
              rst_n,
           in_valid,
                 pc,
          out_valid,
           out_inst,
            out_fin,

         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,               
      arready_m_inf, 

          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf 
);

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;

input clk, rst_n, in_valid;
input [10:0] pc;
output out_valid, out_fin;
output [15:0] out_inst;

// axi read address channel
output  wire [ID_WIDTH-1:0]       arid_m_inf;
output  reg  [ADDR_WIDTH-1:0]   araddr_m_inf;
output  wire [7 -1:0]            arlen_m_inf;
output  wire [3 -1:0]           arsize_m_inf;
output  wire [2 -1:0]          arburst_m_inf;
output  reg                    arvalid_m_inf;
input   wire                   arready_m_inf;
// -----------------------------
// axi read data channel 
input   wire [ID_WIDTH-1:0]         rid_m_inf;
input   wire [DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [2 -1:0]             rresp_m_inf;
input   wire                      rlast_m_inf;
input   wire                     rvalid_m_inf;
output  reg                      rready_m_inf;
// -----------------------------

//constant
assign arid_m_inf = 4'd0;
assign arlen_m_inf = 7'b111_1111;
assign arsize_m_inf = 3'b001;
assign arburst_m_inf = 2'b01;

assign out_valid = rvalid_m_inf;
assign out_inst = rdata_m_inf;
assign out_fin = rlast_m_inf;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        araddr_m_inf <= 'd0;
    else begin
        if(in_valid)
            if(pc < 'd2047 - 'd127)
                araddr_m_inf <= {16'd0, 4'b0001, pc, 1'b0};
            else
                araddr_m_inf <= {16'd0, 4'b0001, 11'd1920, 1'b0};
        else if(arready_m_inf)
            araddr_m_inf <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        arvalid_m_inf <= 'd0;
    else begin
        if(in_valid)
            arvalid_m_inf <= 'd1;
        else if(arready_m_inf)
            arvalid_m_inf <= 'd0;
    end
end

//keep receiving inst
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        rready_m_inf <= 'd0;
    else begin
        rready_m_inf <= 'd1;
    end
end

endmodule



module DRAM_DATA_BRIDGE(
                clk,
              rst_n,
           in_valid,
               addr,
            in_data,
                 wr, //1:read 0:write
          out_valid,
           out_data,

         awid_m_inf,
       awaddr_m_inf,
       awsize_m_inf,
      awburst_m_inf,
        awlen_m_inf,
      awvalid_m_inf,
      awready_m_inf,
                    
        wdata_m_inf,
        wlast_m_inf,
       wvalid_m_inf,
       wready_m_inf,
                    
          bid_m_inf,
        bresp_m_inf,
       bvalid_m_inf,
       bready_m_inf,

         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,               
      arready_m_inf, 

          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf 
);

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;

input clk, rst_n, in_valid;
input [15:0] in_data;
input [10:0] addr;
input wr;
output out_valid;
output [15:0] out_data;


// axi write address channel 
output  wire [WRIT_NUMBER * ID_WIDTH-1:0]        awid_m_inf;
output  reg  [WRIT_NUMBER * ADDR_WIDTH-1:0]    awaddr_m_inf;
output  wire [WRIT_NUMBER * 3 -1:0]            awsize_m_inf;
output  wire [WRIT_NUMBER * 2 -1:0]           awburst_m_inf;
output  wire [WRIT_NUMBER * 7 -1:0]             awlen_m_inf;
output  reg  [WRIT_NUMBER-1:0]                awvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                awready_m_inf;
// axi write data channel 
output  reg  [WRIT_NUMBER * DATA_WIDTH-1:0]     wdata_m_inf;
output  reg  [WRIT_NUMBER-1:0]                  wlast_m_inf;
output  reg  [WRIT_NUMBER-1:0]                 wvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                 wready_m_inf;
// axi write response channel
input   wire [WRIT_NUMBER * ID_WIDTH-1:0]         bid_m_inf;
input   wire [WRIT_NUMBER * 2 -1:0]             bresp_m_inf;
input   wire [WRIT_NUMBER-1:0]             	   bvalid_m_inf;
output  reg  [WRIT_NUMBER-1:0]                 bready_m_inf;
// -----------------------------
// axi read address channel
output  wire [ID_WIDTH-1:0]       arid_m_inf;
output  reg  [ADDR_WIDTH-1:0]   araddr_m_inf;
output  wire [7 -1:0]            arlen_m_inf;
output  wire [3 -1:0]           arsize_m_inf;
output  wire [2 -1:0]          arburst_m_inf;
output  reg                    arvalid_m_inf;
input   wire                   arready_m_inf;
// -----------------------------
// axi read data channel 
input   wire [ID_WIDTH-1:0]         rid_m_inf;
input   wire [DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [2 -1:0]             rresp_m_inf;
input   wire                      rlast_m_inf;
input   wire                     rvalid_m_inf;
output  reg                      rready_m_inf;
// -----------------------------

//constant
assign awid_m_inf = 4'd0;
assign awsize_m_inf = 3'b001;
assign awburst_m_inf = 2'b01;
assign awlen_m_inf = 7'b000_0000;

assign arid_m_inf = 4'd0;
assign arlen_m_inf = 7'b000_0000;
assign arsize_m_inf = 3'b001;
assign arburst_m_inf = 2'b01;

assign out_valid = rlast_m_inf || (bvalid_m_inf && bresp_m_inf == 2'b00);
assign out_data = rdata_m_inf;

//read from dram
//addr channel
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        arvalid_m_inf <= 'd0;
    else begin
        if(in_valid && wr)
            arvalid_m_inf <= 'd1;
        else if(arready_m_inf)
            arvalid_m_inf <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        araddr_m_inf <= 'd0;
    else begin
        if(in_valid && wr)
            araddr_m_inf <= {16'd0, 4'b0001, addr, 1'b0};
        else if(arready_m_inf)
            araddr_m_inf <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        rready_m_inf <= 'd0;
    else begin
        rready_m_inf <= 'd1;
    end
end


//write dram
//addr channel
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        awvalid_m_inf <= 'd0;
    else begin
        if(in_valid && !wr)
            awvalid_m_inf <= 'd1;
        else if(awready_m_inf)
            awvalid_m_inf <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        awaddr_m_inf <= 'd0;
    else begin
        if(in_valid && !wr)
            awaddr_m_inf <= {16'd0, 4'b0001, addr, 1'b0};
        else if(awready_m_inf)
            awaddr_m_inf <= 'd0;
    end
end

//data channel
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        wdata_m_inf <= 'd0;
    else begin
        if(awready_m_inf)
            wdata_m_inf <= in_data;
        else if(wready_m_inf)
            wdata_m_inf <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        wlast_m_inf <= 'd0;
    else begin
        if(awready_m_inf)
            wlast_m_inf <= 'd1;
        else if(wready_m_inf)
            wlast_m_inf <= 'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        wvalid_m_inf <= 'd0;
    else begin
        if(awready_m_inf)
            wvalid_m_inf <= 'd1;
        else if(wready_m_inf)
            wvalid_m_inf <= 'd0;
    end
end

//response channel
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        bready_m_inf <= 'd0;
    else begin
        bready_m_inf <= 'd1;
    end
end



endmodule