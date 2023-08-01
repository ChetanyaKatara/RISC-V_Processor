module TOP1(clk,rst);
  input clk,rst;
  
  wire PC_Exmux,regwrt_E,oprsel_E,memwrite_E,resultctrl_E,branch_E,regwrt_M,memwrite_M,resultctrl_M,resultctrl_W;
  wire [3:0]ALUcontrol_E;
  wire [4:0]RD_E,RD_M,RD_W,Rs1_E,Rs2_E;
  wire [31:0]PCT_Ex,IR_out_D,PC_D,PC_1D,Result_W,RD1_E,RD2_E,immdx_E,PC_DE,PC_1DE,writedata_M,ALUresult_M,PC_1DEM,PC_1DEMW,ALUresult_W,Readdata_W;
  wire [1:0]FrwdA_E,FrwdB_E;
  
  Fetch_cycle fetch(.clk(clk),
                    .rst(rst),
                    .PC_Exmux(PC_Exmux),
                    .PCT_Ex(PCT_Ex),
                    .IR_out_D(IR_out_D),
                    .PC_D(PC_D),
                    .PC_1D(PC_1D));
  
  decode_cycle decode(.clk(clk), 
                      .rst(rst), 
                      .IR_out_D(IR_out_D), 
                      .PC_D(PC_D), 
                      .PC_1D(PC_1D), 
                      .regwrt_W(regwrt_W), 
                      .RD_W(RD_W), 
                      .Result_W(Result_W), 
                      .regwrt_E(regwrt_E), 
                      .oprsel_E(oprsel_E), 
                      .memwrite_E(memwrite_E), 
                      .resultctrl_E(resultctrl_E),
                      .branch_E(branch_E),  
                      .ALUcontrol_E(ALUcontrol_E), 
                      .RD1_E(RD1_E), 
                      .RD2_E(RD2_E), 
                      .immdx_E(immdx_E), 
                      .RD_E(RD_E), 
                      .PC_DE(PC_DE), 
                      .PC_1DE(PC_1DE),
                      .Rs1_E(Rs1_E),
                      .Rs2_E(Rs2_E));
  
  execute_cycle Execute (.clk(clk), 
                        .rst(rst), 
                        .regwrt_E(regwrt_E), 
                        .oprsel_E(oprsel_E), 
                        .memwrite_E(memwrite_E), 
                        .resultctrl_E(resultctrl_E), 
                        .branch_E(branch_E), 
                        .ALUcontrol_E(ALUcontrol_E), 
                        .RD1_E(RD1_E), 
                        .RD2_E(RD2_E), 
                        .immdx_E(immdx_E), 
                        .RD_E(RD_E), 
                        .PC_DE(PC_DE), 
                        .PC_1DE(PC_1DE), 
                        .PC_Exmux(PC_Exmux), 
                        .PCT_Ex(PCT_Ex), 
                        .regwrt_M(regwrt_M), 
                        .memwrite_M(memwrite_M), 
                        .resultctrl_M(resultctrl_M), 
                        .RD_M(RD_M), 
                        .PC_1DEM(PC_1DEM), 
                        .writedata_M(writedata_M), 
                        .ALUresult_M(ALUresult_M),
                        .ResultW(Result_W),
                        .FrwdA_E(FrwdA_E),
                       .FrwdB_E(FrwdB_E)
                    );
  
  memory_cycle Memory (.clk(clk), 
                       .rst(rst), 
                       .regwrt_M(regwrt_M), 
                       .memwrite_M(memwrite_M), 
                       .resultctrl_M(resultctrl_M), 
                       .RD_M(RD_M), 
                       .PC_1DEM(PC_1DEM), 
                       .writedata_M(writedata_M), 
                       .ALUresult_M(ALUresult_M), 
                       .regwrt_W(regwrt_W), 
                       .resultctrl_W(resultctrl_W), 
                       .RD_W(RD_W), 
                       .PC_1DEMW(PC_1DEMW), 
                       .ALUresult_W(ALUresult_W), 
                       .Readdata_W(Readdata_W));
  
  writeback_cycle WriteBack (.clk(clk), 
                             .rst(rst), 
                             .resultctrl_W(resultctrl_W), 
                             .PC_1DEMW(PC_1DEMW), 
                             .ALUresult_W(ALUresult_W), 
                             .Readdata_W(Readdata_W), 
                             .Result_W(Result_W));
  
  hazard_unit Forwarding_block (.rst(rst), 
                                .regwrt_M(regwrt_M), 
                                .regwrt_W(regwrt_W), 
                                .RD_M(RD_M),
                                .RD_W(RD_W), 
                                .Rs1_E(Rs1_E), 
                                .Rs2_E(Rs2_E), 
                                .FrwdA_E(FrwdA_E), 
                                .FrwdB_E(FrwdB_E));
  
endmodule
  
  
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
  
  assign out[31:0] = (sel[1:0] == 2'b00) ? a[31:0] : (sel[1:0] == 2'b01) ? b[31:0] : (sel[1:0] == 2'b10) ? c[31:0] : 32'h00000000;
endmodule

module Instr_mem(Addr,IRout,rst);
  input rst;
  input [31:0]Addr;
  output [31:0]IRout;
  
  reg [31:0] ir_memory[1023:0];
  
  assign IRout[31:0] = (rst == 1'b0) ? {32{1'b0}} : ir_memory[Addr[31:0]];
  
   initial 
     begin
       $readmemh("memfile.hex",ir_memory);
     end
endmodule
  
module decode_cycle(validate,PC_Exmux,clk,rst,IR_out_D,PC_D,PC_1D,regwrt_W,RD_W,Result_W,regwrt_E,oprsel_E,RD1_E,RD2_E,immdx_E,RD_E,PC_DE,PC_1DE,memwrite_E,branch_E,ALUcontrol_E,Rs1_E,Rs2_E,resultctrl_E);
  input clk,rst,regwrt_W,PC_Exmux;
  input [4:0]RD_W;
  input [31:0]IR_out_D,PC_D,PC_1D,Result_W;
  
  output regwrt_E,oprsel_E,memwrite_E,branch_E,resultctrl_E,validate;
  output [3:0]ALUcontrol_E;
  output [31:0]RD1_E,RD2_E,immdx_E;
  output [4:0] Rs1_E,Rs2_E,RD_E;
  output [31:0]PC_DE,PC_1DE;
  
  wire regwrt_D,oprsel_D,memwrite_D,branch_D,resultctrl_D,immdc_D;
  wire [3:0]ALUcontrol_D;
  wire [31:0]RD1_D,RD2_D,immdx_D;

  
  reg regwrt_Dr,oprsel_Dr,memwrite_Dr,branch_Dr,resultctrl_Dr,immdc_Dr;
  reg [3:0]ALUcontrol_Dr;
  reg [31:0]RD1_Dr,RD2_Dr,immdx_Dr;
  reg [4:0]RD_Dr,Rs1_Dr, Rs2_Dr;
  reg [31:0]PC_Dr,PC_1Dr;
  
  
  Control_Unit_Top CTRLUNIT(.Op(IR_out_D[6:0]),
                            .regwrt(regwrt_D),
                            .immdc(immdc_D),
                            .oprsel(oprsel_D),
                            .memwrite(memwrite_D),
                            .resultctrl(resultctrl_D),
                            .branch(branch_D),
                            .funct3(IR_out_D[14:12]),
                            .funct7(IR_out_D[31:25]),
                            .ALUcontrol(ALUcontrol_D));
  
  reg_file regf(.clk(clk),
                .rst(rst),
                .WD(Result_W),
                .WE(regwrt_W),
                .A1(IR_out_D[19:15]),
                .A2(IR_out_D[24:20]),
                .A3(RD_W),
                .RD1(RD1_D),
                .RD2(RD2_D));
  
  extender sign_ext(.Input(IR_out_D[31:0]),
                    .immdx(immdx_D),
                    .immdc(immdc_D));



  
   always @(posedge clk or negedge rst)
    begin
      if (rst == 1'b0) begin
        regwrt_Dr <= 1'b0;
        oprsel_Dr <= 1'b0;
        memwrite_Dr <= 1'b0;
  	    resultctrl_Dr <= 1'b0;
        branch_Dr <= 1'b0;
        ALUcontrol_Dr <= 3'b000;
        RD1_Dr <= 32'h00000000; 
        RD2_Dr <= 32'h00000000; 
        immdx_Dr <= 32'h00000000;
        RD_Dr <= 5'h00;
        PC_Dr <= 32'h00000000; 
        PC_1Dr <= 32'h00000000;
        Rs1_Dr <= 5'h00;
        Rs2_Dr <= 5'h00;
        
      end
      
      else begin
        regwrt_Dr <= regwrt_D;
        oprsel_Dr <= oprsel_D;
        memwrite_Dr <= memwrite_D;
  	    resultctrl_Dr <= resultctrl_D;
        branch_Dr <= branch_D;
        ALUcontrol_Dr <= ALUcontrol_D;
        RD1_Dr <= ((regwrt_W) && (RD_W[4:0] == (IR_out_D[19:15]))) ? Result_W : RD1_D; 
        RD2_Dr <= ((regwrt_W) && (RD_W[4:0] == (IR_out_D[24:20]))) ? Result_W : RD2_D; 
        immdx_Dr <= immdx_D;
        RD_Dr <= IR_out_D[11:7];
        PC_Dr <= PC_D; 
        PC_1Dr <= PC_1D;
        Rs1_Dr <= IR_out_D[19:15];
        Rs2_Dr <= IR_out_D[24:20];
      end
    end
  
  assign regwrt_E = regwrt_Dr;
  assign oprsel_E = oprsel_Dr;
  assign memwrite_E = memwrite_Dr;
  assign resultctrl_E = resultctrl_Dr;
  assign branch_E = branch_Dr;
  assign ALUcontrol_E = ALUcontrol_Dr;
  assign RD1_E = RD1_Dr; 
  assign RD2_E = RD2_Dr; 
  assign immdx_E = immdx_Dr;
  assign RD_E = RD_Dr;
  assign PC_DE = PC_Dr; 
  assign PC_1DE = PC_1Dr;
  assign Rs1_E = Rs1_Dr;
  assign Rs2_E = Rs2_Dr;
  
endmodule
  

  


module Control_Unit_Top(Op,regwrt,immdc,oprsel,memwrite,resultctrl,branch,funct3,funct7,ALUcontrol);
    input [6:0]Op,funct7;
    input [2:0]funct3;
    output regwrt,oprsel,memwrite,resultctrl,branch,immdc;
  output [3:0]ALUcontrol;
  wire [1:0]ALUcode;
  
   Decoder Main_Decoder(.Op(Op),
                        .regwrt(regwrt),
                        .immdc(immdc),
                        .memwrite(memwrite),
                        .resultctrl(resultctrl),
                        .branch(branch),
                        .oprsel(oprsel),
                        .ALUcode(ALUcode));
  
  ALUdecoder ALU_Decoder(.ALUcode(ALUcode),
                         .funct3(funct3),
                         .funct7(funct7),
                         .Op(Op),
                         .ALUcontrol(ALUcontrol));
  
endmodule
  
  


module Decoder(Op,regwrt,immdc,oprsel,resultctrl,memwrite,branch,ALUcode);
  input [6:0]Op;
  output regwrt,immdc,oprsel,resultctrl,memwrite,branch;
  output [1:0]ALUcode;
  
  assign regwrt = (Op == 7'b0000011 | Op == 7'b0110011 | Op == 7'b0010011 ) ? 1'b1 : 1'b0 ;
  assign immdc = (Op == 7'b0100011 | Op == 7'b1100011) ? 1'b1 : 1'b0 ;
  assign oprsel = (Op == 7'b0000011 | Op == 7'b0100011 | Op == 7'b0010011) ? 1'b1 : 1'b0;
  assign resultctrl = (Op == 7'b0000011) ? 1'b1 : 1'b0;
  assign memwrite = (Op == 7'b0100011) ? 1'b1 : 1'b0;
  assign branch = (Op == 7'b1100011) ? 1'b1 : 1'b0;
  assign ALUcode = (Op == 7'b0110011 | Op == 7'b0010011) ? 2'b10 : (Op == 7'b1100011) ? 2'b01 : 2'b00 ;
  
endmodule

module ALUdecoder(ALUcode,ALUcontrol,funct3,funct7,Op);
  input [6:0]Op,funct7;
  input [1:0]ALUcode;
  input [2:0]funct3;
  output [3:0]ALUcontrol;
  
  assign ALUcontrol[3:0] = (ALUcode == 2'b00) ? 4'b0000 : (((ALUcode == 2'b10)&(funct3 == 3'b000)&({Op[5],funct7[5]} == 2'b11)) | (ALUcode == 2'b01)) ? 4'b0001 : ((ALUcode == 2'b10)&(funct3 == 3'b000)&({Op[5],funct7[5]} !== 2'b11)) ? 4'b0000 : ((ALUcode == 2'b10)&(funct3 == 3'b100)) ? 4'b0010 : ((ALUcode == 2'b10)&(funct3 == 3'b110)) ? 4'b0011 : ((ALUcode == 2'b10)&(funct3 == 3'b111)) ? 4'b0100 : 4'b0000;
  
endmodule


module reg_file(clk,rst,WE,WD,A1,A2,A3,RD1,RD2);
  input [4:0]A1,A2,A3;
  input [31:0]WD;
  output [31:0]RD1,RD2;
  input clk,rst,WE;
  
  reg [31:0] Register[31:0];
  always @ (posedge clk)
    begin
       if(WE & (A3 != 5'h00))
            Register[A3] <= WD;
    end

    assign RD1 = (rst==1'b0) ? 32'd0 : Register[A1];
    assign RD2 = (rst==1'b0) ? 32'd0 : Register[A2];

    initial begin
        Register[0] = 32'h00000000;
    end

endmodule
      
module extender(Input,immdc,immdx);
  input [31:0]Input;
  input immdc;
  output [31:0]immdx;
  
  assign immdx[31:0] = (immdc) ? {{20{Input[31]}},Input[31:25],Input[11:7]} : {{20{Input[31]}},Input[31:20]};
endmodule
      

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
  wire [31:0]ResultE;
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
            regwrt_Er <= (~wr3) & regwrt_E; 
            memwrite_Er <= (~wr3) & memwrite_E; 
            resultctrl_Er <= resultctrl_E;
            RD_Er <= RD_E;
            PC_1DEr <= PC_1DE; 
            Src_R <= Src_interim; 
            Result_Er <= (~wr3) & ResultE;
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

module memory_cycle(clk,rst,regwrt_M,memwrite_M,resultctrl_M,RD_M,PC_1DEM,writedata_M,ALUresult_M,regwrt_W,resultctrl_W,RD_W,ALUresult_W,Readdata_W,PC_1DEMW);
  input clk,rst,regwrt_M,memwrite_M,resultctrl_M;
  input [31:0]PC_1DEM,writedata_M,ALUresult_M;
  input [4:0]RD_M;
  output regwrt_W,resultctrl_W;
  output [4:0]RD_W;
  output [31:0]ALUresult_W,Readdata_W,PC_1DEMW;
  
  wire [31:0]ReaddataM;
  
  reg regwrt_Mr,resultctrl_Mr;
  reg [4:0]RD_Mr;
  reg [31:0]PC_1DEMr,ALUresult_Mr,Readdata_Mr;
  
  Data_Memory dmem(.clk(clk),
                   .rst(rst),
                   .WE(memwrite_M),
                   .WD(writedata_M),
                   .A(ALUresult_M),
                   .RD(ReaddataM));
  
  always @(posedge clk or negedge rst) begin
        if (rst == 1'b0) begin
            regwrt_Mr <= 1'b0; 
            resultctrl_Mr <= 1'b0;
            RD_Mr <= 5'h00;
            PC_1DEMr <= 32'h00000000; 
            ALUresult_Mr <= 32'h00000000; 
            Readdata_Mr <= 32'h00000000;
        end
        else begin
            regwrt_Mr <= regwrt_M; 
            resultctrl_Mr <= resultctrl_M;
            RD_Mr <= RD_M;
            PC_1DEMr <= PC_1DEM; 
            ALUresult_Mr <= ALUresult_M; 
            Readdata_Mr <= ReaddataM;
        end
    end 

    assign regwrt_W = regwrt_Mr;
    assign resultctrl_W = resultctrl_Mr;
    assign RD_W = RD_Mr;
    assign PC_1DEMW = PC_1DEMr;
    assign ALUresult_W = ALUresult_Mr;
    assign Readdata_W = Readdata_Mr;
endmodule
  
  

module Data_Memory(clk,rst,WE,WD,A,RD);

    input clk,rst,WE;
    input [31:0]A,WD;
    output [31:0]RD;

    reg [31:0] mem [1023:0];

    always @ (posedge clk)
    begin
        if(WE)
            mem[A] <= WD;
    end

    assign RD = (~rst) ? 32'd0 : mem[A];

    initial begin
        mem[0] = 32'h00000000;
        mem[40] = 32'h00000002;
    end


endmodule

module writeback_cycle(clk, rst, resultctrl_W, PC_1DEMW, ALUresult_W, Readdata_W, Result_W);

input clk, rst, resultctrl_W;
  input [31:0] PC_1DEMW, ALUresult_W, Readdata_W;

  output [31:0] Result_W;


  MUX2 result_mux (.a(ALUresult_W),
                 .b(Readdata_W),
                 .sel(resultctrl_W),
                 .c(Result_W));
endmodule

module hazard_unit(rst, regwrt_M, regwrt_W, RD_M, RD_W, Rs1_E, Rs2_E, FrwdA_E, FrwdB_E);
    input rst, regwrt_M, regwrt_W;
    input [4:0] RD_M, RD_W, Rs1_E, Rs2_E;
  output [1:0] FrwdA_E, FrwdB_E;
    
    assign FrwdA_E = (rst == 1'b0) ? 2'b00 : 
      ((regwrt_M == 1'b1) & (~(RD_M == 5'h00)) & (RD_M == Rs1_E)) ? 2'b10 :
      (((regwrt_W == 1'b1) & (~(RD_W == 5'h00)) & (RD_W == Rs1_E))) ? 2'b01 : 2'b00;
                       
    assign FrwdB_E = (rst == 1'b0) ? 2'b00 : 
      ((regwrt_M == 1'b1) & (~(RD_M == 5'h00)) & (RD_M == Rs2_E)) ? 2'b10 :
      (((regwrt_W == 1'b1) & (~(RD_W == 5'h00)) & (RD_W == Rs2_E))) ? 2'b01 : 2'b00;

endmodule
