//We use this module for board programming.
module lab5_pv (input CLK, SW0, SW1, KEY0, SW2, SW3, SW4, output logic [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0, output logic LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7);
logic clkt,clkout;
logic [3:0] OPCODE;
logic [7:0] PC, alu_out, W_REG;
logic [7:0] number [5:0];
logic [20:0] count_max, count;
assign count_max = 20'd24400; //To divide the clk down to ~1kHz.
assign LED0 = PC[0];
assign LED1 = PC[1];
assign LED2 = PC[2];
assign LED3 = PC[3];
assign LED4 = PC[4];
assign LED5 = PC[5];
assign LED6 = PC[6];
assign LED7 = PC[7];

	//This determines whether or not we are in single step mode.
	always_comb begin
	if (SW1 == 1)
		clkt = ~KEY0;
	else
		clkt = clkout;
	end

	//This determines 7Seg output depending on switch positions.
	always_comb begin
			number[5] = 0;
			number[4] = 0;
			number[3] = 0;
                 	number[2] = 0;
                 	number[1] = 0;
                 	number[0] = 0;
			
	if (SW4 == 0 && SW3 == 0 && SW2 == 0) begin //Displays my name
			number[5] = 8'h4D;
			number[4] = 8'h55;
			number[3] = 8'h52;
			number[2] = 8'h50;
			number[1] = 8'h48;
			number[0] = 8'h59;
		end
	else if (SW4 == 1 && SW3 == 1 && SW2 == 0)begin //Displays the PC
			number[5] = PC[5];
			number[4] = PC[4];
			number[3] = PC[3];
                 	number[2] = PC[2];
                 	number[1] = PC[1];
                 	number[0] = PC[0];
		end
	else if (SW4 == 1 && SW3 == 0 && SW2 == 1) begin //Displays the W_REG
			number[5] = W_REG[5];
			number[4] = W_REG[4];
			number[3] = W_REG[3];
                 	number[2] = W_REG[2];
                 	number[1] = W_REG[1];
                 	number[0] = W_REG[0];
		end
	else if (SW4 == 0 && SW3 == 1 && SW2 == 1) begin //Displays alu_out
			number[5] = alu_out[5];
			number[4] = alu_out[4];
			number[3] = alu_out[3];
                 	number[2] = alu_out[2];
                 	number[1] = alu_out[1];
                 	number[0] = alu_out[0];
		end
	else if (SW4 == 1 && SW3 == 1 && SW2 == 1) begin //Displays OPCODE
			number[5] = 0;
			number[4] = 0;
			number[3] = OPCODE[3];
                 	number[2] = OPCODE[2];
                 	number[1] = OPCODE[1];
                 	number[0] = OPCODE[0];
		end
	else begin //Displays blanks under other undefined switch cases.
			number[5] = 0;
			number[4] = 0;
			number[3] = 0;
                 	number[2] = 0;
                 	number[1] = 0;
                 	number[0] = 0;
		end
	end

lab5 M1 (clkt, SW0, SW1, KEY0, OPCODE, PC, alu_out, W_REG);
FreqDiv #(20)F1 (CLK, SW0, count_max, count, clkout);
ASCII27Seg2 A7 (number[5],HEX5);
ASCII27Seg2 A8 (number[4],HEX4);
ASCII27Seg2 A9 (number[3],HEX3);
ASCII27Seg2 A10 (number[2],HEX2);
ASCII27Seg2 A11 (number[1],HEX1);
ASCII27Seg2 A12 (number[0],HEX0);
endmodule
//This is the parent module of the MCU.
module lab5 (input clk, SWO, SW1, KEY0, output logic [3:0] OPCODE, output logic [7:0] PC, alu_out, W_REG);
localparam IF=3'b000, ID=3'b001, FD=3'b010, EX=3'b011, RWB=3'b100, HLT=3'b101;
logic [2:0] state, next_state;
logic [15:0] IR;
logic reset;
logic [3:0] RA, RB, RD;
logic [7:0] D,A,B,next_PC;
integer MC;
assign OPCODE = IR [15:12];
assign RA = IR [11:8];
assign RB = IR [7:4];
assign RD = IR [3:0];
assign reset = SWO;

//		//This writes to our log file.
//		initial begin
//		MC = $fopen("MCU.csv");
//		$fwrite (MC,"PC,IR,OPCODE,RA,RB,RD,A,B,RF[RD]\n");
//		end
//		always_comb begin
//		if (state == EX) begin
//		$fwrite(MC,"%h,%h,%h,%h,%h,%h,%h,%h,%h\n",PC,IR,OPCODE,RA,RB,RD,A,B,W_REG); end
//		if (OPCODE == 4'b0000)begin
//		$fclose(MC);end
//		end

		//This handles machine reset and basic setup.
		always_ff @ (posedge clk or posedge reset) begin
			if (reset) begin
			   state <= 3'b0;
			   PC <= 8'b0;
			   W_REG <= 8'b0;
			end
			else begin
			   state <= next_state;
			   PC <= next_PC;
			   W_REG <= alu_out;
			end
		   end
		//This controls the state of of our MCU.   
		always_comb begin
			next_state = state;
			case(state)
			   IF : next_state = ID;
			   ID : next_state = FD;
			   FD : next_state = EX;
			   EX : next_state = RWB;
			   default : next_state = state;
			   endcase
			if (state == RWB && OPCODE == 4'b0000) begin
				next_state = RWB;
				end
			if (state == RWB && OPCODE != 4'b0000)begin
				next_state = IF;
	       		        end
			end
			
		//This controls our program counter.
		always_ff @ (posedge reset or posedge clk) begin
			if (reset)
			   next_PC <= 8'b0;
			else if ((state == RWB) && ~((OPCODE == 4'b0000)||(OPCODE == 4'b1101)||(OPCODE == 4'b1110)))
			   next_PC <= PC + 1;
			else if (state == RWB && OPCODE == 4'b0000)
			   next_PC <= PC;
			else if (state == RWB && OPCODE == 4'b1101)
			   next_PC <= {RA,RB};
			else if (state == RWB && OPCODE == 4'b1110) begin
			        if (A>=B)
				next_PC <= PC + RD;
				else 
				next_PC <= PC + 1;
		           end
			end

RegFile R1 (reset, clk, RA, RB, RD, OPCODE, state, W_REG, A, B);
INSMEM I1 (PC, IR);
ALU A1 (OPCODE, RA, RB, A, B, alu_out);
endmodule
//This is our register file.
module RegFile (input reset, clk, input [3:0] RA, RB, RD, OPCODE, input [2:0] current_state, input [7:0] RF_data_in, output logic [7:0] RF_data_out0, RF_data_out1);
localparam One = 8'd1, Zero = 8'd0, one = 1'b1, zero = 1'b0;
logic [4:0] i;
logic [7:0] RF [15:0];
parameter IF = 3'b000;
parameter ID = 3'b001;
parameter FD = 3'b010;
parameter EX = 3'b011;
parameter RWB = 3'b100;
parameter HLT = 3'b101;

	always_ff @ (posedge reset or posedge clk) begin
		i = 5'd0;
		if (reset) begin
		   RF_data_out0 <= Zero;
		   RF_data_out1 <= Zero;
		   for (i = 5'd0; i <5'd16; i = i + 5'd1) RF[i] <= Zero;
		end
		else begin
		   RF_data_out0 <= RF[RA];
		   RF_data_out1 <= RF[RB];
		if ((current_state == RWB) && ~((OPCODE == 4'd13) || (OPCODE == 4'd14) || (OPCODE == 4'd15)))
		   RF[RD] <= RF_data_in;
	end
			
   end
endmodule
//This is the read only memory that contains the program we want to run.
module INSMEM (input [7:0] PC, output logic [15:0] data);
logic [15:0] mem [20:0];

	assign mem [0] = 16'h1000;
	assign mem [1] = 16'h1011;
	assign mem [2] = 16'h1002;
	assign mem [3] = 16'h10A3;
	assign mem [4] = 16'hE236;
	assign mem [5] = 16'h2014;
	assign mem [6] = 16'h3100;
	assign mem [7] = 16'h3401;
	assign mem [8] = 16'h7022;
	assign mem [9] = 16'hD040;
	assign mem [10] = 16'h3405;
	assign mem [11] = 16'h6536;
	assign mem [12] = 16'h5637;
	assign mem [13] = 16'h4578;
	assign mem [14] = 16'h3828;
	assign mem [15] = 16'h8089;
	assign mem [16] = 16'h809A;
	assign mem [17] = 16'hB89B;
	assign mem [18] = 16'hA9AC;
	assign mem [19] = 16'hC0CD;
	assign mem [20] = 16'h0000;
	assign data = mem[PC];

endmodule
//This is our ALU which controls the logical and arithmetic operations of our MCU.
module ALU (input [3:0] OPCODE, RA, RB, input [7:0] A, B, output logic [7:0] alu_out);
always_comb begin
	alu_out = 8'b0;
	case (OPCODE)
	4'b0000 : alu_out = alu_out; //Halt
	4'b0001 : alu_out = {RA,RB}; // LDI
	4'b0010 : alu_out = A + B; // ADD
	4'b0011 : alu_out = A + RB; // ADI
	4'b0100 : alu_out = A - B; // SUB
	4'b0101 : alu_out = A*B; // MUL
	4'b0110 : alu_out = A/B; // DIV
	4'b0111 : alu_out = B + 1; // INC
	4'b1000 : alu_out = B - 1; // DEC
	4'b1001 : alu_out = ~(A | B); // NOR
	4'b1010 : alu_out = ~(A & B); // NAND
	4'b1011 : alu_out = A^B; // XOR
	4'b1100 : alu_out = ~B; //COMP
	default : alu_out = 8'b0;
	endcase
end
endmodule

//This module is our frequency divider which will divide down the 50Mhz clock.
module FreqDiv #(parameter Size = 5)(input clk, reset, input [Size-1:0] count_max, output logic [Size-1:0] count, output logic clkout);
always_ff @ (posedge clk or posedge reset)begin
	if (reset) begin
		count <= {Size{1'b0}};
		clkout <= 1'b0;
		end
	else if(count < count_max)
		count <= count + {{(Size-1){1'b0}},1'b1};
	else 	begin 
		count <= {Size{1'b0}};
		clkout <= ~clkout;
		end
	end
endmodule

module ASCII27Seg2 (input [7:0] number ,output reg [6:0] HexSeg);
logic [7:0] number_;
    always @ (*) begin
        HexSeg = 7'd0;
	if (number > 8'h39)
	number_ = number;
	else
        number_ = number + 8'h30;
        case (number_)
//            M
            8'h4D : begin
                  HexSeg[1] = 1; HexSeg[3] = 1; HexSeg[5] = 1; HexSeg[6] = 1;
               end
//            U
            8'h55 : begin
                  HexSeg[0] = 1; HexSeg[6] = 1;
               end
//            R
            8'h52 : begin
                  HexSeg[0] = 1; HexSeg[1] = 1; HexSeg[2] = 1; HexSeg[3] = 1; HexSeg[5] = 1;
               end
//            P
            8'h50 : begin
                  HexSeg[2] = 1; HexSeg[3] = 1;
               end
//            H
            8'h48 : begin
                  HexSeg[0] = 1; HexSeg[3] = 1;
               end
//            Y
            8'h59 : begin
                  HexSeg[0] = 1; HexSeg[4] = 1;
               end
//            0
            8'h30 : begin
                  HexSeg[6] = 1;
               end
//            1
            8'h31 : begin
                  HexSeg[0] = 1; HexSeg[3] = 1; HexSeg[4] = 1; HexSeg[5] = 1; HexSeg[6] = 1;
               end
//            2
            8'h32 : begin
                  HexSeg[2] = 1; HexSeg[5] = 1;
               end
//            3
            8'h33 : begin
                  HexSeg[4] = 1; HexSeg[5] = 1;
               end
//            4
            8'h34 : begin
                  HexSeg[0] = 1; HexSeg[3] = 1; HexSeg[4] = 1;
               end
//            5
            8'h35 : begin
                  HexSeg[1] = 1; HexSeg[4] = 1;
               end
//            6
            8'h36 : begin
                  HexSeg[1] = 1;
               end
//            7
            8'h37 : begin
                  HexSeg[3] = 1; HexSeg[4] = 1; HexSeg[5] = 1; HexSeg[6] = 1;
               end
//            8
            8'h38 : begin
               end
//            9
            8'h39 : begin
                  HexSeg[4] = 1;
               end
        default : HexSeg = 7'b1111111;
      endcase
    end
endmodule