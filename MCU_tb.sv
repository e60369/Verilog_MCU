//This is the test bench, it runs until we hit the last line of the program.
`timescale 1ns/1ps
module lab5_tb ();
logic [7:0] alu_out, W_REG, PC;
logic [3:0] OPCODE;
logic clk, SWO, SW1, KEY0;

initial begin
SW1 = 0; KEY0 = 1; clk = 1; SWO = 1; #5; SWO = 0; clk = 0; #5;

while (PC != 8'h14) begin
clk = 1; #5;
clk = ~clk;#5; 
end 

end

lab5 L1 (clk, SWO, SW1, KEY0, OPCODE, PC, alu_out, W_REG);
endmodule