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
