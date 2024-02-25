
`include "defines.vh"
//---------------------------------------------------------------------------
// DUT 
//---------------------------------------------------------------------------
module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//q_state_input SRAM interface
  output wire                                               q_state_input_sram_write_enable  ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_write_address ,
  output wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_write_data    ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_read_address  , 
  input  wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_read_data     ,

//---------------------------------------------------------------------------
//q_state_output SRAM interface
  output wire                                                q_state_output_sram_write_enable  ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address ,
  output wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data    ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address  , 
  input  wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_read_data     ,

//---------------------------------------------------------------------------
//scratchpad SRAM interface                                                       
  output wire                                                scratchpad_sram_write_enable        ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_write_address       ,
  output wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_write_data          ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address        , 
  input  wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_read_data           ,

//---------------------------------------------------------------------------
//q_gates SRAM interface                                                       
  output wire                                                q_gates_sram_write_enable           ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_write_address          ,
  output wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_write_data             ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_read_address           ,  
  input  wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_read_data              
);

  // SRAM interface
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_read_address_r ;

  reg                                                q_state_output_sram_write_enable_r;
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address_r ;
  reg [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data_r    ;

  reg [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_read_address_r ;

  reg                                                scratchpad_sram_write_enable_r; 
  reg [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_write_address_r;
  reg [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_write_data_r   ;
  reg [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address_r ;
  reg [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address_offset ;

  localparam inst_sig_width = 52;
  localparam inst_exp_width = 11;
  localparam inst_ieee_compliance = 1;

  reg  [inst_sig_width+inst_exp_width : 0] inst_a1, inst_a2;
  reg  [inst_sig_width+inst_exp_width : 0] inst_b1, inst_b2;
  reg  [inst_sig_width+inst_exp_width : 0] inst_c1, inst_c2;
  reg  [2 : 0] inst_rnd;
  wire [inst_sig_width+inst_exp_width : 0] z1_inst, z2_inst;
  reg [inst_sig_width+inst_exp_width : 0] z1_inst_r, z2_inst_r;
  wire [7 : 0] status_inst1, status_inst2;

  // This is test stub for passing input/outputs to a DP_fp_mac, there many
  // more DW macros that you can choose to use
  DW_fp_mac_inst FP_MAC1 ( 
    inst_a1,
    inst_b1,
    inst_c1,
    //inst_rnd,
    z1_inst,
    status_inst1
  );
  DW_fp_mac_inst FP_MAC2 ( 
    inst_a2,
    inst_b2,
    inst_c2,
    //inst_rnd,
    z2_inst,
    status_inst2
  );

  // GIVEN CODE ENDS

  parameter[2:0] // synopsys enum states
   S0 = 3'b000,
   S0_delay1 = 3'b100,
   S0_delay2 = 3'b101,
   S1 = 3'b001,
   S2 = 3'b010;
   reg [3:0] current_state, next_state;
   
   wire [31:0] next_q_gates_sram_read_address;
   wire [31:0] next_q_state_input_sram_read_address, next_scratchpad_sram_read_address;
   wire [31:0] next_q_state_output_sram_write_address, next_scratchpad_sram_write_address;
   reg [31:0] M;
   reg [2:0] Q;
   reg [12:0] total_OM_elements, op_switch_address, ip_switch_address;
   reg [3:0] rc_size, i;
   reg [4:0] j;
   wire [4:0] next_j;
   wire [4:0] next_i;
   reg [4:0] ip_size;
   reg compute_complete, ip_control, op_control, ij_inc, next_read_address_offset, next_write_address_offset;

  always @ (posedge clk) begin
    if (!reset_n) begin 
      current_state <= S0;
    end
    else begin      
      if (!dut_ready) begin
        // gate addreess control
        if(current_state == S0_delay2 || current_state == S0_delay1) q_gates_sram_read_address_r <= 32'b0;
        else q_gates_sram_read_address_r <= next_q_gates_sram_read_address;

        //setting input control
        if(q_gates_sram_read_address_r == ip_switch_address) ip_control <= 1;
        //setting output cotrol
        if(q_gates_sram_read_address_r == op_switch_address && current_state != S0_delay1) op_control <= 1;


        // input address control incl. input sram and scratchpad
        if(!ip_control) begin
          if(next_q_state_input_sram_read_address == ip_size+1) q_state_input_sram_read_address_r <= 32'h1;
          else q_state_input_sram_read_address_r <= next_q_state_input_sram_read_address; 
        end
        else begin
          if(next_scratchpad_sram_read_address == ip_size+scratchpad_sram_read_address_offset) begin
            if(!(i == rc_size && j == rc_size)) scratchpad_sram_read_address_r <= scratchpad_sram_read_address_offset;
            else scratchpad_sram_read_address_r <= next_scratchpad_sram_read_address;
          end 
          else scratchpad_sram_read_address_r <= next_scratchpad_sram_read_address; 
        end

        // i and j
        if(next_j == ip_size) j <= 1'b0;
        else j <= next_j;

        if(next_j == ip_size && next_i == ip_size) i <= 1'b0;
        else if(next_j == ip_size) i <= next_i;

        // setting scratchpad read offset 
        if (next_i == ip_size && next_j == ip_size && ip_control == 1'b1) scratchpad_sram_read_address_offset <= scratchpad_sram_read_address_offset + ip_size;
        
        
        if(!op_control) begin 
          scratchpad_sram_write_address_r <= next_scratchpad_sram_write_address;
        end
        else q_state_output_sram_write_address_r <= next_q_state_output_sram_write_address;
        
        if(q_state_input_sram_read_address_r == 0) begin 
          Q <= q_state_input_sram_read_data[66:64];
          M <= q_state_input_sram_read_data[63:0];
        end
      end
      else begin
        i <= 4'h0;
        j <= 4'h0;
        ip_control <= 4'h0;
        op_control <= 4'h0;
        q_state_input_sram_read_address_r <= 32'b0;
        q_gates_sram_read_address_r <= 32'b0;
        scratchpad_sram_read_address_r <= 32'b0;
        scratchpad_sram_read_address_offset <= 32'b0;
        q_state_output_sram_write_address_r <= 32'b0;
        scratchpad_sram_write_address_r <= 32'b0;
      end
      current_state <= next_state;
      z1_inst_r <= z1_inst;
      z2_inst_r <= z2_inst;
    end
  end

  always @ (*) begin
    compute_complete = 1'b0;
    inst_c1 = 64'b0;
    inst_c2 = 64'b0;
    inst_a1 = 64'b0;
    inst_a2 = 64'b0;
    inst_b1 = 64'b0;
    inst_b2 = 64'b0;
    

    next_read_address_offset = 1'b1;
    ij_inc = 1'b0;
    next_write_address_offset = 1'b0;
    q_state_output_sram_write_enable_r = 1'b0;
    q_state_output_sram_write_data_r = 128'b0;
    scratchpad_sram_write_enable_r = 1'b0;
    scratchpad_sram_write_data_r = 128'b0;

    casex(current_state)
      S0: begin
        compute_complete = 1;
        if(dut_valid) next_state = S0_delay1;
        else next_state = S0;
      end

      S0_delay1: begin
        next_state = S0_delay2;
      end

      S0_delay2: begin
        next_read_address_offset = 1'b0;
        next_state = S1;
      end

      S1: begin
        if (j!=0) begin 
          inst_c1 = z1_inst_r;
          inst_c2 = z2_inst_r;
        end

        if(!ip_control) begin
          inst_a1 = {!q_state_input_sram_read_data[63], q_state_input_sram_read_data[62:0]};
          inst_a2 = q_state_input_sram_read_data[127:64];
        end
        else begin
          inst_a1 = {!scratchpad_sram_read_data[63], scratchpad_sram_read_data[62:0]};
          inst_a2 = scratchpad_sram_read_data[127:64];
        end

        inst_b1 = q_gates_sram_read_data[63:0];
        inst_b2 = q_gates_sram_read_data[63:0];

        next_state = S2;
      end

      S2: begin
        inst_c1 = z1_inst_r;
        inst_c2 = z2_inst_r;

        if(!ip_control) begin
          inst_a1 = q_state_input_sram_read_data[127:64];
          inst_a2 = q_state_input_sram_read_data[63:0];
        end
        else begin
          inst_a1 = scratchpad_sram_read_data[127:64];
          inst_a2 = scratchpad_sram_read_data[63:0];
        end
        inst_b1 = q_gates_sram_read_data[127:64];
        inst_b2 = q_gates_sram_read_data[127:64];
        
        if (j == rc_size)
        begin
        next_write_address_offset = 1'b1;
          if(!op_control) begin
            scratchpad_sram_write_enable_r = 1;
            scratchpad_sram_write_data_r[127:64] = z1_inst;
            scratchpad_sram_write_data_r[63:0] = z2_inst;
          end
          else begin
            q_state_output_sram_write_enable_r = 1;
            q_state_output_sram_write_data_r[127:64] = z1_inst;
            q_state_output_sram_write_data_r[63:0] = z2_inst;
          end
        end

        next_read_address_offset = 1'b0;
        ij_inc = 1'b1;

        if(q_gates_sram_read_address_r == total_OM_elements) next_state = S0;
        else next_state = S1;
      end

      default: next_state = S0;
    endcase
  end

  always @ (Q, M)
  begin 
    if (Q == 1)      begin total_OM_elements = M<<2 ; rc_size = 1 ; ip_size = 2 ; op_switch_address = total_OM_elements - 4  ; ip_switch_address = 4  ; end
    else if (Q == 2) begin total_OM_elements = M<<4 ; rc_size = 3 ; ip_size = 4 ; op_switch_address = total_OM_elements - 16 ; ip_switch_address = 16 ; end
    else if (Q == 3) begin total_OM_elements = M<<6 ; rc_size = 7 ; ip_size = 8 ; op_switch_address = total_OM_elements - 64 ; ip_switch_address = 64 ; end
    else if (Q == 4) begin total_OM_elements = M<<8 ; rc_size = 15; ip_size = 16; op_switch_address = total_OM_elements - 256; ip_switch_address = 256; end
    else begin total_OM_elements = 1'b0 ; rc_size = 1'b0; ip_size = 1'b0; op_switch_address = 1'b0; ip_switch_address = 1'b0; end
  end

  assign dut_ready = compute_complete;

  assign next_i = i + ij_inc;
  assign next_j = j + ij_inc;

  assign q_state_input_sram_read_address  = q_state_input_sram_read_address_r ;
  assign q_gates_sram_read_address  = q_gates_sram_read_address_r ; 
  assign q_state_input_sram_write_enable = 1'b0;
  assign q_gates_sram_write_enable = 1'b0;

  assign q_state_output_sram_write_enable  = q_state_output_sram_write_enable_r  ;
  assign q_state_output_sram_write_address = q_state_output_sram_write_address_r ;
  assign q_state_output_sram_write_data    = q_state_output_sram_write_data_r    ;

  assign scratchpad_sram_write_enable  = scratchpad_sram_write_enable_r ; 
  assign scratchpad_sram_write_address = scratchpad_sram_write_address_r;
  assign scratchpad_sram_write_data    = scratchpad_sram_write_data_r   ;
  assign scratchpad_sram_read_address  = scratchpad_sram_read_address_r ;

  assign next_q_state_input_sram_read_address = q_state_input_sram_read_address_r + next_read_address_offset;
  assign next_scratchpad_sram_read_address = scratchpad_sram_read_address_r + next_read_address_offset;

  assign next_q_gates_sram_read_address = q_gates_sram_read_address_r + next_read_address_offset;
  assign next_q_state_output_sram_write_address = q_state_output_sram_write_address_r + next_write_address_offset;
  assign next_scratchpad_sram_write_address =  scratchpad_sram_write_address_r + next_write_address_offset;
endmodule


module DW_fp_mac_inst #(
  parameter inst_sig_width = 52,
  parameter inst_exp_width = 11,
  parameter inst_ieee_compliance = 1 // These need to be fixed to decrease error
) ( 
  input wire [inst_sig_width+inst_exp_width : 0] inst_a,
  input wire [inst_sig_width+inst_exp_width : 0] inst_b,
  input wire [inst_sig_width+inst_exp_width : 0] inst_c,
  //input wire [2 : 0] inst_rnd,
  output wire [inst_sig_width+inst_exp_width : 0] z_inst,
  output wire [7 : 0] status_inst
);

  // Instance of DW_fp_mac
  DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
    .a(inst_a),
    .b(inst_b),
    .c(inst_c),
    .rnd(3'b000),
    .z(z_inst),
    .status(status_inst) 
  );

endmodule

