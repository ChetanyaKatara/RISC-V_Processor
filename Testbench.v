module tb();

    reg clk=0, rst;
    
    always begin
        clk = ~clk;
        #50;
    end

    initial begin
        rst <= 1'b0;
        #200;
        rst <= 1'b1;
        #2900;
        $finish;    
    end

    initial begin
        $dumpfile("dumprdr.vcd");
        $dumpvars(0);
    end

    TOP1 dut (.clk(clk), .rst(rst));
endmodule