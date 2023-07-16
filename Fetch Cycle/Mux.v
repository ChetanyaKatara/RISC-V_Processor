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
