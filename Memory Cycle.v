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
