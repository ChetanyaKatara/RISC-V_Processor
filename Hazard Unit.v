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