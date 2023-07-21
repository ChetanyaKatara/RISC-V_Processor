module execute_cycle(,clk,rst,regwrt_E,oprsel_E,immdx_E,memwrite_E,resultctrl_E,branch_E,ALUcontrol_E,RD1_E,RD2_E,RD_E,PC_DE,PC_1DE,PC_Exmux,PCT_Ex,FrwdA_E,FrwdB_E,regwrt_M,memwrite_M,resultctrl_M,RD_M,PC_1DEM,writedata_M,ALUresult_M,ResultW);
  
  input regwrt_E,oprsel_E,memwrite_E,resultctrl_E,branch_E,clk,rst;
  input [3:0]ALUcontrol_E;
  input [31:0]RD1_E,RD2_E,PC_DE,PC_1DE,ResultW,immdx_E;
  output [31:0]PCT_Ex,PC_1DEM,ALUresult_M,writedata_M;
  output PC_Exmux,regwrt_M,memwrite_M,resultctrl_M;
  input [1:0]FrwdA_E,FrwdB_E;
  output [4:0]RD_M;
  input [4:0]RD_E;

  
  wire [31:0]Src_A,Src_B,Src_interim;
  wire [31:0]ResultE,Resultf;
  wire ZeroE,PC_1Exmux;
  wire wr3;
  
  reg regwrt_Er,memwrite_Er,resultctrl_Er;
  reg [4:0]RD_Er;
  reg [31:0]PC_1DEr,Src_R,Result_Er;


  brd brd(.inp(PC_1Exmux),
  .clk(clk),
  .rst(rst),
  .opt(wr3),
  .opt1(PC_Exmux));
  
  MUX3 srcA(.a(RD1_E),
            .b(ResultW),
            .c(ALUresult_M),
            .sel(FrwdA_E),
            .out(Src_A));
  
  
  MUX3 srcB(.a(RD2_E),
            .b(ResultW),
            .c(ALUresult_M),
            .sel(FrwdB_E),
            .out(Src_interim));
  
  MUX2 ALUsr(.a(Src_interim),
             .b(immdx_E),
             .sel(oprsel_E),
            .c(Src_B));
  
  ALU ALunit(.A(Src_A),
             .B(Src_B),
             .Result(ResultE),
             .ALUcontrol(ALUcontrol_E),
             .zero(ZeroE),
             .carry(),
             .negative());
  
  Adder PC_clc(.a(PC_DE),
               .b(immdx_E),
               .c(PCT_Ex));


  ot ott(.in1(ResultE),
        .in2(wr3),
        .out(Resultf));


  
   always @(posedge clk or negedge rst)
    begin
      if (rst == 1'b0) begin
            regwrt_Er <= 1'b0; 
            memwrite_Er <= 1'b0; 
            resultctrl_Er <= 1'b0;
            RD_Er <= 5'h00;
            PC_1DEr <= 32'h00000000; 
            Src_R <= 32'h00000000; 
            Result_Er <= 32'h00000000;
          end
        else 
          begin
            regwrt_Er <= regwrt_E; 
            memwrite_Er <= memwrite_E; 
            resultctrl_Er <= resultctrl_E;
            RD_Er <= RD_E;
            PC_1DEr <= PC_1DE; 
            Src_R <= Src_interim; 
            Result_Er <= Resultf;
          end
    end
    
    assign PC_1Exmux = ZeroE & branch_E;
    assign regwrt_M = regwrt_Er;
    assign memwrite_M = memwrite_Er;
    assign resultctrl_M = resultctrl_Er;
    assign RD_M = RD_Er;
    assign PC_1DEM = PC_1DEr;
    assign writedata_M = Src_R;
    assign ALUresult_M = Result_Er;

endmodule

  module ALU(A,B,Result,carry,zero,negative,ALUcontrol);
  input [31:0]A,B;
  output [31:0]Result;
  output carry,zero,negative;
  input [3:0]ALUcontrol;
  
  wire Cout;
  wire [31:0]Sum;
  assign Sum[31:0] = (ALUcontrol[3:0] == 4'b0000) ? A + B:A - B;
  
  
  assign {Cout,Result} = (ALUcontrol[3:0] == 4'b0000) ? Sum[31:0]: (ALUcontrol[3:0] == 4'b0001) ? Sum[31:0]: (ALUcontrol[3:0] == 4'b0010) ? A[31:0] ^ B[31:0] : (ALUcontrol[3:0] == 4'b0011) ? A[31:0] | B[31:0] : (ALUcontrol[3:0] == 4'b0100) ? A[31:0] & B[31:0] : {33{1'b0}};
  
  assign carry = ((ALUcontrol[3:0] == 4'b0000) & Cout);
  assign zero = (Result[31:0] == 32'b0);
  assign negative = Result[31]; 
  
endmodule

module ot(in1,in2,out);
 input [31:0]in1;
 input in2;
 output [31:0]out;

 assign out[31:0] = (in2) ? 32'h00000000:in1[31:0];

 endmodule

module brd(inp,opt,clk,rst,opt1);
input inp,rst,clk;
output opt1,opt;

wire A,B;
wire c,d;
assign c = A ^ B;
assign d = ((A|inp)&(~B));
assign opt = c | d;
assign opt1 = ((~A) & (~B)) & (inp);


dflipflop dffA(.inp(c),
            .clk(clk),
            .rst(rst),
            .opt(A));

dflipflop dffB(.inp(d),
            .clk(clk),
            .rst(rst),
            .opt(B)); 
                  
endmodule

module dflipflop(clk,rst,inp,opt);
input clk,rst,inp;
output reg opt;

always @(posedge clk or negedge rst)
        begin
          if (rst == 1'b0) begin
            opt <= 1'b0;
          end
          else opt <= inp;
        end
endmodule

