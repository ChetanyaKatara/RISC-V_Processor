module writeback_cycle(clk, rst, resultctrl_W, PC_1DEMW, ALUresult_W, Readdata_W, Result_W);

input clk, rst, resultctrl_W;
  input [31:0] PC_1DEMW, ALUresult_W, Readdata_W;

  output [31:0] Result_W;


  MUX2 result_mux (.a(ALUresult_W),
                 .b(Readdata_W),
                 .sel(resultctrl_W),
                 .c(Result_W));
endmodule
