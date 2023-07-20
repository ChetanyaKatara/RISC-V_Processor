module Fetch_cycle(clk,rst,PC_Exmux,PCT_Ex,PC_D,PC_1D,IR_out_D);
  
  input clk,rst,PC_Exmux;
  input [31:0]PCT_Ex;
  output [31:0] IR_out_D;
  output [31:0] PC_D,PC_1D;
  
  wire [31:0] PC_mux,PC_adder,PC_F,InstrF;
  
  reg [31:0] IR_reg,PC_1D_reg,PC_F_reg;
  
  MUX2 PC_mx(.b(PCT_Ex),
              .a(PC_adder),
              .c(PC_mux),
              .sel(PC_Exmux));
  
  Program_counter PC(.clk(clk),
                     .rst(rst),
                     .PC(PC_F),
                     .PCnxt(PC_mux));
  
  Adder PC_add(.a(PC_F),
                 .b(32'h00000001),
                 .c(PC_adder));
  
  Instr_mem Instruction_memory(.rst(rst),
                               .Addr(PC_F),
                               .IRout(InstrF));
  
  always @(posedge clk or negedge rst)
    begin
      if (rst == 1'b0) begin
        IR_reg <= 32'h00000000;
        PC_1D_reg <= 32'h00000000;
        PC_F_reg <= 32'h00000000;
      end
      else begin
        IR_reg <= InstrF;
        PC_1D_reg <= PC_adder;
        PC_F_reg <= PC_F;
      end
    end
  
  assign IR_out_D = IR_reg;
  assign PC_D = PC_F_reg;
  assign PC_1D = PC_1D_reg;
  
endmodule
    


module Program_counter(clk,PC,PCnxt,rst);
  input clk,rst;
  input [31:0]PCnxt;
  output [31:0]PC;
  reg [31:0]PC;
  
  always @(posedge clk or negedge rst)
    begin
      if (rst == 1'b0) begin
        PC[31:0] = {32{1'b0}};
      end
      else PC[31:0] <= PCnxt[31:0];
    end
endmodule

module Adder(a,b,c);
  input [31:0]a,b;
  output [31:0]c;
  
  assign c[31:0] = a[31:0] + b[31:0];
endmodule

module MUX2(a,b,sel,c);
  input [31:0]a,b;
  input sel;
  output [31:0]c;
  
  assign c[31:0] = (~sel) ? a[31:0]:b[31:0];
endmodule

module MUX3(a,b,c,sel,out);
  input [31:0]a,b,c;
  input [1:0]sel;
  output [31:0]out;

