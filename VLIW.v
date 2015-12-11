// --------------Pipeline Registers------------------------------
module D_ff (input clk, input reset, input regWrite, input decOut1b, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1'b1)
		q=0;
	else
		if(regWrite == 1'b1 && decOut1b==1'b1) begin q=d; end
	end
endmodule

module register1bit(input clk, input reset, input regWrite, input decOut1b, input writeData, output outR);
	D_ff d0(clk, reset, regWrite, decOut1b, writeData, outR);
	
endmodule

module register2bit(input clk, input reset, input regWrite, input decOut1b, input [1:0] writeData, output [1:0] outR);
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
endmodule
module register3bit(input clk, input reset, input regWrite, input decOut1b, input [2:0] writeData, output [2:0] outR);
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
endmodule

module register5bit(input clk, input reset, input regWrite, input decOut1b, input [4:0] writeData, output [4:0] outR);
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	
endmodule

module register8bit(input clk, input reset, input regWrite, input decOut1b, input [7:0] writeData, output  [7:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
endmodule

module register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
endmodule

module register32bit( input clk, input reset, input regWrite, input decOut1b, input [31:0] writeData, output  [31:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
	D_ff d16(clk, reset, regWrite, decOut1b, writeData[16], outR[16]);
	D_ff d17(clk, reset, regWrite, decOut1b, writeData[17], outR[17]);
	D_ff d18(clk, reset, regWrite, decOut1b, writeData[18], outR[18]);
	D_ff d19(clk, reset, regWrite, decOut1b, writeData[19], outR[19]);
	D_ff d20(clk, reset, regWrite, decOut1b, writeData[20], outR[20]);
	D_ff d21(clk, reset, regWrite, decOut1b, writeData[21], outR[21]);
	D_ff d22(clk, reset, regWrite, decOut1b, writeData[22], outR[22]);
	D_ff d23(clk, reset, regWrite, decOut1b, writeData[23], outR[23]);
	D_ff d24(clk, reset, regWrite, decOut1b, writeData[24], outR[24]);
	D_ff d25(clk, reset, regWrite, decOut1b, writeData[25], outR[25]);
	D_ff d26(clk, reset, regWrite, decOut1b, writeData[26], outR[26]);
	D_ff d27(clk, reset, regWrite, decOut1b, writeData[27], outR[27]);
	D_ff d28(clk, reset, regWrite, decOut1b, writeData[28], outR[28]);
	D_ff d29(clk, reset, regWrite, decOut1b, writeData[29], outR[29]);
	D_ff d30(clk, reset, regWrite, decOut1b, writeData[30], outR[30]);
	D_ff d31(clk, reset, regWrite, decOut1b, writeData[31], outR[31]);
endmodule


// ------------------------------Register file Design Begins----------------------------------------
module D_ff_registerfile( input clk, input reset, input regWrite1, input regWrite2, input decOut1b1, input decOut1b2, input d1, input d2, output reg q);

		always @ (posedge clk)
			begin
				if(reset==1)
					q=0;
				else
				   if(regWrite1 == 1 && decOut1b1==1)
						begin
									q=d1;
						end
					if(regWrite2 == 1 && decOut1b2==1)
						begin
									q=d2;
						end
			end

endmodule

module register32bit_registerfile( input clk, input reset, input regWrite1, input regWrite2, input decOut1b1, input decOut1b2, input [31:0] writeData1, input [31:0] writeData2,
output [31:0] outR );

	D_ff_registerfile d0(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[0], writeData2[0], outR[0]);
	D_ff_registerfile d1(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[1], writeData2[1], outR[1]);
	D_ff_registerfile d2(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[2], writeData2[2], outR[2]);
	D_ff_registerfile d3(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[3], writeData2[3], outR[3]);
	D_ff_registerfile d4(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[4], writeData2[4], outR[4]);
	D_ff_registerfile d5(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[5], writeData2[5], outR[5]);
	D_ff_registerfile d6(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[6], writeData2[6], outR[6]);
	D_ff_registerfile d7(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[7], writeData2[7], outR[7]);
	D_ff_registerfile d8(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[8], writeData2[8], outR[8]);
	D_ff_registerfile d9(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[9], writeData2[9], outR[9]);
	D_ff_registerfile d10(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[10], writeData2[10], outR[10]);
	D_ff_registerfile d11(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[11], writeData2[11], outR[11]);
	D_ff_registerfile d12(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[12], writeData2[12], outR[12]);
	D_ff_registerfile d13(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[13], writeData2[13], outR[13]);
	D_ff_registerfile d14(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[14], writeData2[14], outR[14]);
	D_ff_registerfile d15(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[15], writeData2[15], outR[15]);
	D_ff_registerfile d16(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[16], writeData2[16], outR[16]);
	D_ff_registerfile d17(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[17], writeData2[17], outR[17]);
	D_ff_registerfile d18(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[18], writeData2[18], outR[18]);
	D_ff_registerfile d19(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[19], writeData2[19], outR[19]);
	D_ff_registerfile d20(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[20], writeData2[20], outR[20]);
	D_ff_registerfile d21(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[21], writeData2[21], outR[21]);
	D_ff_registerfile d22(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[22], writeData2[22], outR[22]);
	D_ff_registerfile d23(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[23], writeData2[23], outR[23]);
	D_ff_registerfile d24(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[24], writeData2[24], outR[24]);
	D_ff_registerfile d25(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[25], writeData2[25], outR[25]);
	D_ff_registerfile d26(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[26], writeData2[26], outR[26]);
	D_ff_registerfile d27(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[27], writeData2[27], outR[27]);
	D_ff_registerfile d28(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[28], writeData2[28], outR[28]);
	D_ff_registerfile d29(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[29], writeData2[29], outR[29]);	
	D_ff_registerfile d30(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[30], writeData2[30], outR[30]);
	D_ff_registerfile d31(clk, reset, regWrite1,regWrite2, decOut1b1, decOut1b2, writeData1[31], writeData2[31], outR[31]);

endmodule
// ---------------------- Register Set -----------------------------------

module registerSet( input clk, input reset, input regWrite1, input regWrite2, input [7:0] decOut1, input [7:0] decOut2, input [31:0] writeData1, input [31:0] writeData2,
output [31:0] outR0, output [31:0] outR1, output [31:0] outR2, output [31:0] outR3, output [31:0] outR4,
output [31:0] outR5, output [31:0] outR6, output [31:0] outR7);
		
		register32bit_registerfile r0 (clk, reset, regWrite1, regWrite2, decOut1[0], decOut2[0], writeData1, writeData2 , outR0 );
		register32bit_registerfile r1 (clk, reset, regWrite1, regWrite2, decOut1[1], decOut2[1], writeData1, writeData2 , outR1 );
		register32bit_registerfile r2 (clk, reset, regWrite1, regWrite2, decOut1[2], decOut2[2], writeData1, writeData2 , outR2 );
		register32bit_registerfile r3 (clk, reset, regWrite1, regWrite2, decOut1[3], decOut2[3], writeData1, writeData2 , outR3 );
		register32bit_registerfile r4 (clk, reset, regWrite1, regWrite2, decOut1[4], decOut2[4], writeData1, writeData2 , outR4 );
		register32bit_registerfile r5 (clk, reset, regWrite1, regWrite2, decOut1[5], decOut2[5], writeData1, writeData2 , outR5 );
		register32bit_registerfile r6 (clk, reset, regWrite1, regWrite2, decOut1[6], decOut2[6], writeData1, writeData2 , outR6 );
		register32bit_registerfile r7 (clk, reset, regWrite1, regWrite2, decOut1[7], decOut2[7], writeData1, writeData2 , outR7 );

endmodule

// ---------------------- 3 to 8 DECODER -----------------------------------

module Dec3to8(input [2:0] destReg, output reg [7:0] decOut);
	always@(destReg)
		case(destReg)
				3'b000: decOut=8'b00000001; 
				3'b001: decOut=8'b00000010;
				3'b010: decOut=8'b00000100;
				3'b011: decOut=8'b00001000;
				3'b100: decOut=8'b00010000;
				3'b101: decOut=8'b00100000;
				3'b110: decOut=8'b01000000;
				3'b111: decOut=8'b10000000;
		endcase	
endmodule

// -----------------8x1 Mux----------------------------------------------
module Mux8to1( input [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7, input [2:0] Sel, output reg [31:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or Sel)
	case (Sel)
				3'b000: outBus=outR0;
				3'b001: outBus=outR1;
				3'b010: outBus=outR2;
				3'b011: outBus=outR3;
				3'b100: outBus=outR4;
				3'b101: outBus=outR5;
				3'b110: outBus=outR6;
				3'b111: outBus=outR7;
				
	endcase
endmodule

// ---------------------- Register File ( Main ) -----------------------------------

module RegisterFile(input clk, input reset, input regWrite1, input regWrite2, input [2:0] M_Rm,input [2:0] M_Rn, input [2:0] M_Rd, input [2:0] A_Rm,input [2:0] A_Rd,  input [2:0] MdestReg,input [2:0] AdestReg , input [31:0] AwriteData,input [31:0] MwriteData, output [31:0] Reg_A_Rm, output [31:0] Reg_A_Rd, output [31:0] Reg_M_Rm,   output [31:0] Reg_M_Rn, output [31:0] Reg_M_Rd);
	wire [7:0] decOut1, decOut2;
	wire [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7;

	
	Dec3to8 d0 (AdestReg,decOut1);
	Dec3to8 d1 (MdestReg,decOut2);
	registerSet rSet( clk, reset, regWrite1, regWrite2, decOut1, decOut2, AwriteData, MwriteData, outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7);
	Mux8to1 m1(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,A_Rm,Reg_A_Rm);
	Mux8to1 m2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,A_Rd,Reg_A_Rd);
	Mux8to1 m3(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,M_Rm,Reg_M_Rm);
	Mux8to1 m4(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,M_Rn,Reg_M_Rn);
	Mux8to1 m5(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,M_Rd,Reg_M_Rd);
endmodule
// --------------------------------Register File Design ends ---------------------------------------------------------------


// ---------------------- Adder -------------------------------------------------

module Adder(input [31:0] in1, input[31:0] in2, output reg [31:0] out);
	always@(in1 or in2)
		out = in1 + in2;
endmodule

// ------------------------------------------ Control Circuit-------------------------------------------------------------------------
module CtrlCkt(input [31:0] opcode, 
				output reg MemRead_mem, output reg MemWrite_mem, output reg RegWrite_mem, //Mem Control Signals
				output reg [1:0] ALUSrcA_alu, output reg [1:0] ALUSrcB_alu, output reg RegDst_alu, 
				output reg [1:0] ALUOp_alu, output reg RegWrite_alu  , output reg Jump , output reg Branch, //ALU Control Signals
				output reg EPCWrite, CauseWrite, ValidInstruction, //Exception register signals
				output reg FlagWrite
			   );
	reg temp1, temp2;
	initial
	begin
		
		MemRead_mem = 1'b0;
		MemWrite_mem = 1'b0;
		RegWrite_mem = 1'b0;
		RegDst_alu = 1'b0;
		ALUSrcA_alu = 2'b00;
		ALUSrcB_alu = 2'b00;
		ALUOp_alu = 2'b00;
		RegWrite_alu = 1'b0;
		Branch = 1'b0 ;
		Jump = 1'b0 ;
		EPCWrite = 1'b0;
		CauseWrite = 1'b0;
		temp1 = 1'b1;
		temp2 = 1'b1;
		ValidInstruction = 1'b1;
		FlagWrite = 1'b0;
	end
	always@(opcode)
	begin
		case(opcode[15:11])
					5'b00000:   begin	MemRead_mem=1'b0; MemWrite_mem=1'b0;  RegWrite_mem=1'b0; Jump = 0; Branch = 0; temp1 = 1'b1; end //nop
					5'b00001:   begin	MemRead_mem=1'b0; MemWrite_mem=1'b0;  RegWrite_mem=1'b0; Jump = 0; Branch = 0; temp1 = 1'b1; end //nop
					5'b01010:	begin	MemRead_mem=1'b1; MemWrite_mem=1'b0;  RegWrite_mem=1'b1; Jump = 0; Branch = 0; temp1 = 1'b1;end //Load Word
					5'b01011:	begin	MemRead_mem=1'b0; MemWrite_mem=1'b1;  RegWrite_mem=1'b0; Jump = 0; Branch = 0; temp1 = 1'b1;end //Store Word
					5'b11100:	begin	MemRead_mem=1'b0; MemWrite_mem=1'b0;  RegWrite_mem=1'b0; Jump = 1; Branch = 0; temp1 = 1'b1;end //Jump
					5'b11010:	begin   MemRead_mem=1'b1; MemWrite_mem=1'b0;  RegWrite_mem=1'b0; Jump = 0; Branch = 1; temp1 = 1'b1;end //Branch
					default :   temp1 = 1'b0;
				
		endcase
		case(opcode[31:27])
		
					5'b00000:   begin	MemRead_mem=1'b0; MemWrite_mem=1'b0;  RegWrite_mem=1'b0; Jump = 0; Branch = 0; temp2 = 1'b1; FlagWrite = 1'b0; end //nop
					5'b00001:   begin	ALUSrcA_alu=2'b00;  ALUSrcB_alu=2'b00;  RegDst_alu=1'b0;  ALUOp_alu=2'b00; RegWrite_alu=1'b0; temp2 = 1'b1;	FlagWrite = 1'b0; end //nop
					5'b10101:	begin	ALUSrcA_alu=2'b00;  ALUSrcB_alu=2'b00;  RegDst_alu=1'b0;  ALUOp_alu=2'b00; RegWrite_alu=1'b1; temp2 = 1'b1;	FlagWrite = 1'b1; end //Add
					5'b01000:	
						begin	
							if(opcode[23]==1'b0)
							begin
								ALUSrcA_alu=2'b01;  
								ALUSrcB_alu=2'b01;  
								RegDst_alu=1'b1;  
								ALUOp_alu=2'b01; 
								RegWrite_alu=1'b1;
								FlagWrite = 1'b1;
							end
							else
							begin
								ALUSrcA_alu=2'b11; 
								ALUSrcB_alu=2'b11;  
								RegDst_alu=1'b1;  
								ALUOp_alu=2'b11; 
								RegWrite_alu=1'b1;
								FlagWrite = 1'b1;
							end
							temp2 = 1'b1;
						end //Sub
					5'b11111:	begin	ALUSrcA_alu=2'b10;  ALUSrcB_alu=2'b10;  RegDst_alu=1'b1;  ALUOp_alu=2'b10; RegWrite_alu=1'b1; temp2 = 1'b1; FlagWrite = 1'b0;end //Shft			
					default : begin  temp2 = 1'b0; FlagWrite = 1'b0; end
		endcase

		ValidInstruction = temp1 && temp2;

		if(ValidInstruction == 1'b0) begin CauseWrite = 1'b1; EPCWrite = 1'b1; end
	end
endmodule		
// ---------------------- Mux 2x1 - 1bit -------------------------------------------
module Mux2to1_1bits(input in1, input in2, input Sel, output reg MuxOut);
  always@(in1 or in2 or Sel)
  begin
    case(Sel)
			1'b0:MuxOut = in1;
			1'b1:MuxOut = in2;  
			endcase  
  end
endmodule

// ---------------------- Mux 2x1 - 3bits -------------------------------------------
module Mux2to1_3bits(input [2:0] in1, input [2:0] in2, input Sel, output reg [2:0] MuxOut);
  always@(in1 or in2 or Sel)
  begin
    case(Sel)
			1'b0:MuxOut = in1;
			1'b1:MuxOut = in2;  
			endcase  
  end
endmodule

// ---------------------- Mux 2x1 - 4bits -------------------------------------------
module Mux2to1_4bits(input [3:0] in1, input [3:0] in2, input Sel, output reg [3:0] MuxOut);
  always@(in1 or in2 or Sel)
  begin
    case(Sel)
			1'b0:MuxOut = in1;
			1'b1:MuxOut = in2;  
			endcase  
  end
endmodule

// ---------------------- Mux 2x1 - 32bits -------------------------------------------
module Mux2to1_32bits(input [31:0] in1, input [31:0] in2, input Sel, output reg [31:0] MuxOut);
  always@(in1 or in2 or Sel)
  begin
    case(Sel)
			1'b0:MuxOut = in1;
			1'b1:MuxOut = in2;  
			endcase  
  end
endmodule

// ------------------------- mux 4 to 1 32bit --------------------------
module Mux4to1_32bits(input [31:0] in1,input [31:0] in2,input [31:0] in3, input [31:0] in4, input [1:0] Sel, output reg [31:0] MuxOut);
  always@(in1 or in2 or in3 or in4 or Sel)
	 	case(Sel)
			2'b00: MuxOut = in1;	
			2'b01: MuxOut = in2;
			2'b10: MuxOut = in3;
			2'b11: MuxOut = in4;   
		endcase
endmodule

// ---------------------- Mux 2x1 - 11bits -------------------------------------------
module Mux2to1_11bits(input [10:0] in1, input [10:0] in2, input Sel, output reg [10:0] MuxOut);
  always@(in1 or in2 or Sel)
  begin
    case(Sel)
			1'b0:MuxOut = in1;
			1'b1:MuxOut = in2;  
			endcase  
  end
endmodule

// ---------------------Left Shift by 2-------------------------------------------------
module shft_2(input [7:0] in, output reg [7:0] out);
	always @(in)
		out = in<<2;
endmodule

module ALU(input [31:0] in1, input [31:0] in2, input carry_flag_in,input [1:0] ALUOp, output reg [31:0] ALUOut , output reg neg_flag , output reg carry_flag , output reg zero_flag , output reg overflow_flag );
	reg[32:0] res; 
	always@(in1 or in2 or ALUOp or carry_flag_in)
		case(ALUOp)
			2'b00:	begin
					ALUOut=in1+in2; 
					neg_flag = 0;
					zero_flag = 0;
					carry_flag = 0;
					overflow_flag = 0;
					
					end	 
			2'b01:	begin
					res=in1-in2-{31'd0,carry_flag_in};
					ALUOut=res[31:0];
					carry_flag = res[32];
					
					neg_flag = ALUOut[31];
					 
					if( ALUOut == 32'd0)
						zero_flag = 1;
					else    
						zero_flag = 0;
					if(((in1[31] == 1'b1 && in2[31] == 1'b0 && ALUOut[31]==1'b0) || (in1[31] == 1'b0 && in2[31] == 1'b1 && ALUOut[31]==1'b1)))
						overflow_flag = 1 ;
					else    
						overflow_flag = 0;	 
					
					end	 
			2'b10:	begin
					res=in1<<in2;
					ALUOut=res[31:0];
					carry_flag = res[32];
					
					neg_flag = ALUOut[31];
					
					if( ALUOut == 32'd0)
						zero_flag = 1 ;
					else    
						zero_flag = 0;
					overflow_flag = 0;
					
					
					end	 
			2'b11:  begin
					ALUOut=in1*in2;
				
					neg_flag = ALUOut[31];
					 
					if( ALUOut == 32'd0)
						zero_flag = 1;
					else    
						zero_flag = 0;
					carry_flag = 0;
					overflow_flag = 0;
					
				end		
		endcase
endmodule

// ---------------------Zero extend-------------------------------------------------
module zext(input [16:0] in, output reg [31:0] out);
	always @(in)
		out = {15'b00000_00000_00000,in};
endmodule

// ---------------------Sign extend 11bit to 32bit-------------------------------------------------
module sext_11_to_32(input [10:0] in, output reg [31:0] out);
	always @(in)
		out = {in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in[9],in};
endmodule

// ---------------------Sign extend 8bit to 32bit-------------------------------------------------
module sext_8_to_32(input [7:0] in, output reg [31:0] out);
	always @(in)
		out = {in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7],in};
endmodule

module Concatenate(input [3:0]p0_PC, input [27:0] JumpIn, output reg [31:0] JumpOut);
	always@(p0_PC or JumpIn)
	JumpOut = {p0_PC,JumpIn};
endmodule

//----------------------- PC Src Generation ---------------------------------------
module PCSrcGen(input Jump  , input  Branch , input neg_flag, input overflow_flag , input Exception ,  output reg[1:0] PCSrc); 
	always @ (overflow_flag  or Jump or Branch or Exception) 
	begin
	if( Jump  == 1) 
		PCSrc = 2'b01 ; 
		
	else if( (overflow_flag  == 1 || Exception == 1)  ) 
		PCSrc = 2'b11 ; 

	else if( Branch == 1 && neg_flag == 1 ) 
		PCSrc = 2'b10 ; 
	
	else 
		PCSrc = 2'b00 ; 
	end
endmodule 



// ---------------------- IF/ID Pipeline Register -----------------------------------

module IF_ID(input clk, input reset, input regWrite, input decOut1b, input[31:0] PC, input [15:0] instr1, input [15:0] instr2, output [15:0] p0_intr1, output [15:0] p0_intr2, output[31:0] p0_PC);

	//Write your code here
	register16bit instruction1(clk, reset, regWrite, decOut1b, instr1, p0_intr1);
	register16bit instruction2(clk, reset, regWrite, decOut1b, instr2, p0_intr2);
	register32bit PCin(clk, reset, regWrite, decOut1b, PC, p0_PC);
	
endmodule

// ---------------------- ID/EX Pipeline Register -----------------------------------
module ID_EX(input clk, input reset, input regWrite, input decOut1b,input [31:0] p0_PC, input [31:0] Reg_A_Rm, input [31:0] Reg_A_Rd, input [31:0] Reg_M_Rm, input [31:0] Reg_M_Rn, 
				input [31:0] Reg_M_Rd, input [7:0] Imm_4, input [4:0] Immediate, input [2:0] M_Rd, input [2:0] A_Rd1, input [2:0] A_Rd2, M_Rm, M_Rn, A_Rm,
				input [1:0] ALUSrcA_alu, input [1:0] ALUSrcB_alu, input RegDst_alu, input [1:0] ALUOp1_alu, input RegWrite_alu, 
				input MemRead_mem, input MemWrite_mem, input RegWrite_mem, ID_FlagWrite,
				output [31:0] p1_PC, output [31:0] p1_Reg_A_Rm, output [31:0] p1_Reg_A_Rd, output [31:0] p1_Reg_M_Rm, output [31:0] p1_Reg_M_Rn, 
				output [31:0] p1_Reg_M_Rd, output [7:0] p1_Imm_4, output [4:0] p1_Immediate, output [2:0] p1_M_Rd, output [2:0] p1_A_Rd1, output [2:0] p1_A_Rd2, p1_M_Rm, p1_M_Rn, p1_A_Rm,
				output [1:0] p1_ALUSrcA_alu, output [1:0] p1_ALUSrcB_alu, output p1_RegDst_alu, output [1:0] p1_ALUOp1_alu, output p1_RegWrite_alu,
				output p1_MemRead_mem, output p1_MemWrite_mem, output p1_RegWrite_mem, EX_FlagWrite);

	register32bit PC_in (clk, reset, regWrite, decOut1b, p0_PC, p1_PC);
	register32bit rm1 (clk, reset, regWrite, decOut1b, Reg_A_Rm, p1_Reg_A_Rm);
	register32bit rd1 (clk, reset, regWrite, decOut1b, Reg_A_Rd, p1_Reg_A_Rd);
	register32bit rm2 (clk, reset, regWrite, decOut1b, Reg_M_Rm, p1_Reg_M_Rm);
	register32bit rn2 (clk, reset, regWrite, decOut1b, Reg_M_Rn, p1_Reg_M_Rn);
	register32bit rd2 (clk, reset, regWrite, decOut1b, Reg_M_Rd, p1_Reg_M_Rd);
	
	register8bit Imm8(clk, reset, regWrite, decOut1b, Imm_4, p1_Imm_4);
	register5bit Immed(clk, reset, regWrite, decOut1b, Immediate, p1_Immediate);
	register3bit MRd(clk, reset, regWrite, decOut1b, M_Rd, p1_M_Rd);
	register3bit ARd1(clk, reset, regWrite, decOut1b, A_Rd1, p1_A_Rd1);
	register3bit ARd2(clk, reset, regWrite, decOut1b, A_Rd2, p1_A_Rd2);
	register3bit MRm(clk, reset, regWrite, decOut1b, M_Rm, p1_M_Rm);
	register3bit MRn(clk, reset, regWrite, decOut1b, M_Rn, p1_M_Rn);
	register3bit ARm(clk, reset, regWrite, decOut1b, A_Rm, p1_A_Rm);
	
	register2bit aluA(clk, reset, regWrite, decOut1b, ALUSrcA_alu, p1_ALUSrcA_alu);
	register2bit aluB(clk, reset, regWrite, decOut1b, ALUSrcB_alu, p1_ALUSrcB_alu);
	register1bit RegDstAlu(clk, reset, regWrite, decOut1b, RegDst_alu, p1_RegDst_alu);
	register2bit ALUOp1Alu(clk, reset, regWrite, decOut1b, ALUOp1_alu, p1_ALUOp1_alu);
	register1bit RegWriteAlu(clk, reset, regWrite, decOut1b, RegWrite_alu, p1_RegWrite_alu);
	
	register1bit MemReadMem(clk, reset, regWrite, decOut1b, MemRead_mem, p1_MemRead_mem);
	register1bit MemWriteMem(clk, reset, regWrite, decOut1b, MemWrite_mem, p1_MemWrite_mem);
	register1bit RegWriteMem(clk, reset, regWrite, decOut1b, RegWrite_mem, p1_RegWrite_mem);
	
	register1bit FlagOut3(clk, reset, regWrite, decOut1b , ID_FlagWrite, EX_FlagWrite);
	
endmodule

// ---------------------- EX/MEM Pipeline Register -----------------------------------
module EX_MEM_PipelineRegister(input clk, input reset, input regWrite,input decout1B ,input p1_RegWrite_alu , input p1_RegWrite_mem , 
								input [2:0] p1_M_Rd, input [31:0] ALUOut, input [31:0] adderOut, input [2:0] p1_A_Rd_mux, input  p1_MemRead_mem, 
								input p1_MemWrite_mem,input[31:0] p1_Reg_M_Rd ,input neg_flag_out ,input carry_flag_out ,input zero_flag_out ,
								input overflow_flag_out ,  EX_FlagWrite, output [31:0] p2_ALUOut,output [31:0] p2_adderOut, output [2:0] p2_M_Rd, 
								output [2:0] p2_A_Rd_mux,  output p2_RegWrite_alu, output p2_MemRead_mem, output p2_MemWrite_mem, 
								output p2_RegWrite_mem ,output [31:0] p2_Reg_M_Rd , output p2_neg_flag_out ,output p2_carry_flag_out ,
								output p2_zero_flag_out ,output p2_overflow_flag_out, MEM_FlagWrite);	
	register32bit  aluOut12 (clk, reset, regWrite,decout1B ,  ALUOut, p2_ALUOut);	
	register32bit  adderOutReg (clk, reset, regWrite,decout1B, adderOut, p2_adderOut);
	
	register32bit  Rd_M(clk, reset, regWrite,decout1B, p1_Reg_M_Rd, p2_Reg_M_Rd);	
	
	register3bit P3r2  (clk, reset, regWrite,decout1B, p1_A_Rd_mux, p2_A_Rd_mux);	
	//register1bit P3r3  (clk, reset, regWrite,decout1B, p2_MemRead_alu, P3MemRead_alu);	
	//register1bit P3r4  (clk, reset, regWrite, decout1B,p2_MemWrite_alu, P3MemWrite_alu);	
		
	register1bit P3r6  (clk, reset, regWrite, decout1B,p1_RegWrite_alu , p2_RegWrite_alu);	
	
	register3bit P3r21  (clk, reset, regWrite, decout1B , p1_M_Rd, p2_M_Rd);	
	register1bit P3r31 (clk, reset, regWrite, decout1B ,  p1_MemRead_mem, p2_MemRead_mem);	
	register1bit P3r41  (clk, reset, regWrite, decout1B , p1_MemWrite_mem, p2_MemWrite_mem);	
	
	register1bit P3r61  (clk, reset, regWrite, decout1B , p1_RegWrite_mem, p2_RegWrite_mem);
	
	register1bit   neg_flag_out1(clk, reset, regWrite, decout1B , neg_flag_out, p2_neg_flag_out);
	register1bit   carry_flag_out1(clk, reset, regWrite, decout1B , carry_flag_out, p2_carry_flag_out);
	register1bit   zero_flag_out1(clk, reset, regWrite, decout1B , zero_flag_out, p2_zero_flag_out);
	register1bit   overflow_flag_out1(clk, reset, regWrite, decout1B , overflow_flag_out, p2_overflow_flag_out);
	
	register1bit   FlagOut2(clk, reset, regWrite, decout1B , EX_FlagWrite, MEM_FlagWrite);
		
endmodule

// ---------------------- MEM/WB Pipeline Register -----------------------------------
module MEM_WB(input clk, input reset, input regWrite, input decOut1b, input [31:0] p2_ALUOut, input [31:0] p2_adderOut, input [2:0] p2_A_Rd_mux,
				input [2:0] p2_M_Rd, input p2_RegWrite_alu, p2_RegWrite_mem,input p2_neg_flag_out ,input p2_carry_flag_out ,input p2_zero_flag_out ,
				input p2_overflow_flag_out ,input MEM_FlagWrite, output [31:0] p3_ALUOut, output [31:0] p3_adderOut, output [2:0] p3_A_Rd_mux,
				output [2:0] p3_M_Rd, output p3_RegWrite_alu, p3_RegWrite_mem  ,output p3_neg_flag_out ,output p3_carry_flag_out ,
				output p3_zero_flag_out ,output p3_overflow_flag_out, WB_FlagWrite);
	register32bit ALUOutReg(clk, reset, regWrite, decOut1b, p2_ALUOut, p3_ALUOut);
	register32bit AdderOutReg(clk, reset, regWrite, decOut1b, p2_adderOut, p3_adderOut);
	
	register3bit ARdMux(clk, reset, regWrite, decOut1b, p2_A_Rd_mux, p3_A_Rd_mux);
	register3bit ALUOut(clk, reset, regWrite, decOut1b, p2_M_Rd, p3_M_Rd);
	
	register1bit regWriteSignal1 (clk, reset, regWrite, decOut1b ,  p2_RegWrite_alu, p3_RegWrite_alu);	
	register1bit regWriteSignal2 (clk, reset, regWrite, decOut1b ,  p2_RegWrite_mem, p3_RegWrite_mem);
	
	register1bit   p3_neg_flag_out1(clk, reset, regWrite, decOut1b , p2_neg_flag_out, p3_neg_flag_out);
	register1bit   p3_carry_flag_out1(clk, reset, regWrite, decOut1b , p2_carry_flag_out, p3_carry_flag_out);
	register1bit   p3_zero_flag_out1(clk, reset, regWrite, decOut1b , p2_zero_flag_out, p3_zero_flag_out);
	register1bit   p3_overflow_flag_out1(clk, reset, regWrite, decOut1b , p2_overflow_flag_out, p3_overflow_flag_out);
	
	register1bit   FlagOut1(clk, reset, regWrite, decOut1b , MEM_FlagWrite, WB_FlagWrite);
		
	
endmodule

module CauseRegister(input clk, reset, regWrite, decOut1b, CauseIn, CauseOut);
	register1bit cause1(clk, reset, regWrite, decOut1b , CauseIn, CauseOut);
endmodule

module EPC(input clk, reset, regWrite, decOut1b, input [31:0] EPCIn, EPCOut);
	register32bit epc1(clk, reset, regWrite, decOut1b , EPCIn, EPCOut);
endmodule

module flags(input clk, input reset, input regWrite, input p3_neg_flag_out ,input p3_carry_flag_out ,input p3_zero_flag_out ,input p3_overflow_flag_out ,output global_neg_flag_out ,output global_carry_flag_out ,output global_zero_flag_out ,output global_overflow_flag_out);
	
	register1bit   global_neg_flag_out1(clk, reset, regWrite, 1'b1 , p3_neg_flag_out, global_neg_flag_out);
	register1bit   global_carry_flag_out1(clk, reset, regWrite, 1'b1 , p3_carry_flag_out, global_carry_flag_out);
	register1bit   global_zero_flag_out1(clk, reset, regWrite, 1'b1 , p3_zero_flag_out, global_zero_flag_out);
	register1bit   global_overflow_flag_out1(clk, reset, regWrite, 1'b1 , p3_overflow_flag_out, global_overflow_flag_out);
	
endmodule

module zext_8_to_32(input[7:0] in, output reg[31:0] out);
	always@(in)
	out = {24'd0,in};
endmodule

module zext_5_to_32(input[4:0] in, output reg[31:0] out);
	always@(in)
	out = {27'd0,in};
endmodule

module TopModule(input clk, input reset, input[255:0] instruction_mem,output [31:0] p3_ALUOut, output [31:0] p3_adderOut);

	wire [31:0] outPC, p1_PC, pcAdderOut, memOut, p0_PC, Reg_A_Rm, Reg_A_Rd,  Reg_M_Rm,  Reg_M_Rn,  Reg_M_Rd ,  p1_Reg_A_Rm,  
	p1_Reg_A_Rd, p1_Reg_M_Rm,  p1_Reg_M_Rn, p1_Reg_M_Rd, zexout, DMout, MuxOutA, MuxOutB, adderOut, ALUOut,p1_Imm_4_ext, 
	p1_Immediate_ext, p2_AluOut, p2_adderOut, p2_Reg_M_Rd, p1_Reg_Rd_A, p1_Reg_Rm_A, 
	p1_Reg_Rm_M, p1_Reg_Rn_M, p1_Reg_Rd_M, DMin, JumpOut, BranchOut, PCin, JumpIn, BranchIn,
	PCAdder_Result, EPCIn,EPCOut,Reg_of_Rd;
 	wire [2:0] p3_A_Rd_mux,p1_A_Rd_mux,p1_A_Rd1,p1_A_Rd2,p1_M_Rd, p3_M_Rd, p2_M_Rd,p2_A_Rd_mux, p1_M_Rm, p1_M_Rn, p1_A_Rm,p2_M_Rd11 ;
	wire [15:0] p0_intr1,  p0_intr2;
	wire [7:0] shiftOut , p1_Imm_4;
	wire [4:0] p1_Immediate;  
	wire [1:0] ALUSrcA_alu, ALUSrcB_alu, ALUOp1_alu,p1_ALUSrcA_alu, p1_ALUSrcB_alu, p1_ALUOp1_alu, 
				ForwardA_ALU, ForwardB_ALU, ForwardA_Adder, ForwardB_Adder, ForwardC, PCSrc_Mem,
				ALUSrcA_alu_mod,  ALUSrcB_alu_mod, ALUOp1_alu_mod;
	wire RegDst_mem, ForwardD, MemRead_mem,  MemWrite_mem, RegWrite_mem, RegDst_alu, RegWrite_alu,
	  p1_RegDst_alu, p1_RegWrite_alu, p1_RegDst_mem,  p1_MemRead_mem,  p1_MemWrite_mem, p1_RegWrite_mem ,  
	  p2_RegWrite_alu , p2_MemRead_mem , p2_MemWrite_mem , p2_RegWrite_mem,p3_RegWrite_alu, p3_RegWrite_mem,
	  neg_flag ,  carry_flag ,  zero_flag ,  overflow_flag, p3_neg_flag_out ,p3_carry_flag_out , p3_zero_flag_out ,p3_overflow_flag_out,
		global_neg_flag_out , global_carry_flag_out , global_zero_flag_out , global_overflow_flag_out,
		Jump, Branch, IF_ID_Write, PCWrite, CntrlSel,
		RegDst_alu_mod,RegDst_mem_mod,  MemRead_mem_mod,  MemWrite_mem_mod, RegWrite_mem_mod,RegWrite_alu_mod,
		p1_RegWrite_alu_1, p1_MemRead_mem_1,  p1_MemWrite_mem_1, p1_RegWrite_mem_1,
		CauseWrite,EPCWrite, ValidInstruction,CauseIn,CauseOut, ID_FlagWrite, EX_FlagWrite, MEM_FlagWrite, WB_FlagWrite ;
	// -----------------IF stage--------------------------------------------------
	Mux4to1_32bits PC_Mux(pcAdderOut, JumpOut, BranchOut, 32'd108, PCSrc_Mem, PCin);
	
	Mux2to1_32bits PCAdderMux(32'd4, 32'd0, overflow_flag, PCAdder_Result);
	register32bit PC0 (clk, reset,PCWrite,1'b1, PCin, outPC);	
	
	Adder pcAdder( outPC, PCAdder_Result,  pcAdderOut);
	
	//Mem_IM I_Mem(clk, reset, 1'b0,1'b1, outPC,32'd0, memOut);	
	Memory instr_mem(clk, reset, instruction_mem[255:224],instruction_mem[223:192], instruction_mem[191:160], 
				instruction_mem[159:128], instruction_mem[127:96], instruction_mem[95:64], instruction_mem[63:32], instruction_mem[31:0],
				1'b0, 1'b1, outPC, 32'd0, 
				memOut[7:0], memOut[15:8], memOut[23:16], memOut[31:24] );
	
	IF_ID Pipe1( clk, reset, IF_ID_Write&(1'b1 - overflow_flag)	, ValidInstruction, outPC,  memOut[15:0], memOut[31:16],  p0_intr1,  p0_intr2, p0_PC);
	
	// -----------------ID stage--------------------------------------------------
	sext_11_to_32 sext_JMP(p0_intr1[10:0], JumpIn);
	sext_8_to_32 sext_BRN(p0_intr1[7:0], BranchIn);
	
	Concatenate JumpConcat(p0_PC[31:28], {JumpIn[26:0],1'b0}, JumpOut);
	Adder BranchAdder(p0_PC, {BranchIn[30:0],1'b0}, BranchOut);

	CtrlCkt C1({p0_intr2,p0_intr1},  MemRead_mem_mod,  MemWrite_mem_mod, RegWrite_mem_mod,
					ALUSrcA_alu_mod,  ALUSrcB_alu_mod, RegDst_alu_mod,  ALUOp1_alu_mod, RegWrite_alu_mod, Jump, Branch, 
					CauseWrite, EPCWrite, ValidInstruction, ID_FlagWrite);
					
	PCSrcGen PCsource(Jump , Branch , neg_flag, overflow_flag , (1'b1 - ValidInstruction) , PCSrc_Mem);//Add Undefined instr flag 
	
	RegisterFile RegFile( clk,  reset,  p3_RegWrite_alu,  p3_RegWrite_mem, p0_intr1[8:6] , p0_intr1[5:3], p0_intr1[2:0],  p0_intr2[5:3], p0_intr2[2:0], p3_M_Rd, p3_A_Rd_mux , p3_ALUOut, p3_adderOut, Reg_A_Rm,  Reg_A_Rd,  Reg_M_Rm,  Reg_M_Rn,  Reg_of_Rd);


	shft_2 lolol( p0_intr2[7:0],  shiftOut);
	
	zext zext2(Reg_of_Rd[16:0], Reg_M_Rd);
	
	// --------------------------------------------Hazard Detection Unit---------------------------------------------
	HazardDetectionUnit H1(Branch, neg_flag, ALUOp1_alu_mod, IF_ID_Write, PCWrite, CntrlSel,
                           p1_MemRead_mem, p1_M_Rd, 
						   p0_intr1[8:6], p0_intr1[5:3], p0_intr1[2:0],
                           p0_intr2[5:3], p0_intr2[2:0]);
	
	Mux2to1_11bits HazardMux_ID_EX(11'd0, {MemRead_mem_mod,  MemWrite_mem_mod, RegWrite_mem_mod,
					ALUSrcA_alu_mod,  ALUSrcB_alu_mod, RegDst_alu_mod,  ALUOp1_alu_mod, RegWrite_alu_mod}, CntrlSel&(1'b1 - overflow_flag),
					{MemRead_mem,  MemWrite_mem, RegWrite_mem,
					ALUSrcA_alu,  ALUSrcB_alu, RegDst_alu,  ALUOp1_alu, RegWrite_alu}
					);
					
	
					
	ID_EX id_ex_pipe( clk,  reset, 1'b1, 1'b1, p0_PC,  Reg_A_Rm,  Reg_A_Rd, Reg_M_Rm,  Reg_M_Rn, 
				 Reg_M_Rd,  shiftOut, p0_intr2[10:6], p0_intr1[2:0], p0_intr2[10:8] , p0_intr2[2:0], p0_intr1[8:6], p0_intr1[5:3], p0_intr2[5:3],
				ALUSrcA_alu,  ALUSrcB_alu, RegDst_alu, ALUOp1_alu,  RegWrite_alu, 
				 MemRead_mem,  MemWrite_mem, RegWrite_mem, ID_FlagWrite,
				 p1_PC, p1_Reg_A_Rm,  p1_Reg_A_Rd, p1_Reg_M_Rm,  p1_Reg_M_Rn, 
				 p1_Reg_M_Rd, p1_Imm_4, p1_Immediate,  p1_M_Rd,  p1_A_Rd1,  p1_A_Rd2, p1_M_Rm, p1_M_Rn, p1_A_Rm,
				 p1_ALUSrcA_alu, p1_ALUSrcB_alu,  p1_RegDst_alu, p1_ALUOp1_alu,  p1_RegWrite_alu_1,
				 p1_MemRead_mem_1,  p1_MemWrite_mem_1, p1_RegWrite_mem_1, EX_FlagWrite);
	
	zext_8_to_32 z1(p1_Imm_4, p1_Imm_4_ext);
	zext_5_to_32 z2(p1_Immediate,p1_Immediate_ext);
	// -----------------EX stage--------------------------------------------------
	Mux2to1_3bits  A_rd(p1_A_Rd1, p1_A_Rd2, p1_RegDst_alu, p1_A_Rd_mux);
	
	//-----------ForwardingUnit--------------------------------------------
	ForwardingUnit F1( p1_M_Rm, p1_M_Rn, p1_M_Rd, p1_A_Rm, p1_A_Rd2,
						p2_A_Rd_mux, p2_M_Rd, p2_RegWrite_alu,
						p3_M_Rd, p3_A_Rd_mux, p3_RegWrite_mem, p3_RegWrite_alu, 
						ForwardA_ALU, ForwardB_ALU, ForwardA_Adder, ForwardB_Adder, ForwardC, ForwardD
					 );

	//-----------Forward Muxes---------------------------------------------
	Mux4to1_32bits  Forward_A_ALU(p1_Reg_A_Rd, p2_AluOut, p3_ALUOut, p3_adderOut, ForwardA_ALU, p1_Reg_Rd_A);
	Mux4to1_32bits  Forward_B_ALU(p1_Reg_A_Rm, p2_AluOut, p3_ALUOut, p3_adderOut, ForwardB_ALU, p1_Reg_Rm_A);	
	Mux4to1_32bits  Forward_A_Adder(p1_Reg_M_Rm, p2_AluOut, p3_ALUOut, p3_adderOut, ForwardA_Adder, p1_Reg_Rm_M);	
	Mux4to1_32bits  Forward_B_Adder(p1_Reg_M_Rn, p2_AluOut, p3_ALUOut, p3_adderOut, ForwardB_Adder, p1_Reg_Rn_M);	
	Mux4to1_32bits  Forward_C(p1_Reg_M_Rd, p2_AluOut, p3_ALUOut, p3_adderOut, ForwardC, p1_Reg_Rd_M);	
	
	Mux4to1_32bits  AluSrcA11(( p1_PC & 32'hFFFFFFFC), p1_Reg_Rd_A,p1_Reg_Rm_A, p1_Reg_Rd_A, p1_ALUSrcA_alu,  MuxOutA);
	
	Mux4to1_32bits ALUSrcB11(p1_Imm_4_ext,p1_Reg_Rm_A ,p1_Immediate_ext, p1_Reg_Rm_A, p1_ALUSrcB_alu, MuxOutB);	
	
	Adder memAdder(p1_Reg_Rm_M, p1_Reg_Rn_M, adderOut);
	
	ALU alu(MuxOutA, MuxOutB, p2_carry_flag_out,
			p1_ALUOp1_alu, ALUOut ,  neg_flag ,  carry_flag ,  zero_flag ,  overflow_flag );
			
	Mux2to1_4bits singalMux_EX_MEM({p1_RegWrite_alu_1, p1_MemRead_mem_1,  p1_MemWrite_mem_1, p1_RegWrite_mem_1},4'd0,overflow_flag,
									{p1_RegWrite_alu, p1_MemRead_mem,  p1_MemWrite_mem, p1_RegWrite_mem});
	
	EX_MEM_PipelineRegister ex_mem_pipe( clk, reset, 1'b1,1'b1, p1_RegWrite_alu, p1_RegWrite_mem, p1_M_Rd, ALUOut,  adderOut,  
										p1_A_Rd_mux,  p1_MemRead_mem, p1_MemWrite_mem,p1_Reg_Rd_M , neg_flag ,carry_flag , zero_flag , overflow_flag , EX_FlagWrite,
										p2_AluOut, p2_adderOut,p2_M_Rd, p2_A_Rd_mux,  p2_RegWrite_alu, p2_MemRead_mem,  p2_MemWrite_mem,  p2_RegWrite_mem , 
										p2_Reg_M_Rd ,  p2_neg_flag_out , p2_carry_flag_out , p2_zero_flag_out , p2_overflow_flag_out, MEM_FlagWrite);
	Mux2to1_32bits DatamemIn(p2_Reg_M_Rd, p3_adderOut, ForwardD, DMin);
	// -----------------MEM stage--------------------------------------------------
	//Mem_DM dm( clk,  reset, p2_MemWrite_mem, p2_MemRead_mem,  p2_adderOut, DMin, DMout );
	Memory byte_mem(clk, reset, 32'd7, 32'd9, 32'd10, 32'd11, 32'd12, 32'd13, 32'd14, 32'd8,
				p2_MemWrite_mem, p2_MemRead_mem,  p2_adderOut, DMin, 
				DMout[7:0], DMout[15:8], DMout[23:16], DMout[31:24] );
	zext zext1(DMout[16:0],  zexout);
	
	MEM_WB mem_wb_pipe(clk,  reset,1'b1, 1'b1,  p2_AluOut, zexout, p2_A_Rd_mux,
				p2_M_Rd, p2_RegWrite_alu, p2_RegWrite_mem,p2_neg_flag_out ,p2_carry_flag_out , p2_zero_flag_out ,p2_overflow_flag_out , MEM_FlagWrite,
				p3_ALUOut, p3_adderOut, p3_A_Rd_mux, 
				p3_M_Rd, p3_RegWrite_alu, p3_RegWrite_mem , p3_neg_flag_out ,p3_carry_flag_out , p3_zero_flag_out ,p3_overflow_flag_out ,
				WB_FlagWrite);
	// -----------------WB stage---------------------------------------------------
	flags F2( clk, reset, WB_FlagWrite, p3_neg_flag_out , p3_carry_flag_out , p3_zero_flag_out , p3_overflow_flag_out , 
	global_neg_flag_out , global_carry_flag_out , global_zero_flag_out , global_overflow_flag_out);	
		
	//******************Exception registers****************************************
	CauseRegister cc(clk,reset,CauseWrite|overflow_flag,1'b1,CauseIn,CauseOut);
	EPC epc(clk,reset,EPCWrite|overflow_flag,1'b1,EPCIn,EPCOut);
	
	Mux2to1_1bits intcause(1'b0,1'b1,overflow_flag,CauseIn);
	Mux2to1_32bits epc_in(p0_PC,p1_PC,overflow_flag,EPCIn); 
	
endmodule

module ProjectBench;
	reg clk;
	reg reset;
	wire [31:0] p3_ALUOut, p3_adderOut;
	reg [255:0] instruction_mem;
	
							  
	TopModule uut (.clk(clk), .reset(reset), .instruction_mem(instruction_mem), .p3_ALUOut(p3_ALUOut), .p3_adderOut(p3_adderOut));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#20  instruction_mem = {16'b10101_000_00000100,5'b00001, 11'd0, //add $r0,4
							  5'b00001, 11'd0, 5'b01010,2'b01,9'b010_011_100,  //load $r4,$r2,$r3								
							  5'b01000, 5'b00111, 6'b000100, 5'b01011, 2'b10,9'b010001101,  // mul $r4,$r4,$r0, store $r5,$r2,$r1
							  5'b11111, 5'd27, 6'b000110, 5'b00001, 11'd0,  // shft $r6 = $r0 << imm
							  5'b01000, 5'b00101, 6'b100000, 5'b00001, 11'd0,  //sub $r0 = $r0- $r4 - C  
							  5'b00001, 11'd0, 5'b11010,3'b011,8'b00000100,	   //branch
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b01110,11'd0,5'b00001,11'd0}; //invalid instruction
		#20  reset=0;	
		
		#210 $finish; 
	end
endmodule

module ForwardingUnit( input[2:0] ID_EX_rm1, ID_EX_rn1, ID_EX_rd1, ID_EX_rm2, ID_EX_rd2,
						EX_MEM_rd, EX_MEM_rd11, input EX_MEM_RegWrite,
						input [2:0] MEM_WB_rd1, MEM_WB_rd2, input MEM_WB_RegWrite1, MEM_WB_RegWrite2, 
						output reg [1:0] ForwardA_ALU, ForwardB_ALU, ForwardA_Adder, ForwardB_Adder, ForwardC, output reg ForwardD 
					 );
	initial
	begin
		ForwardA_ALU = 2'b00;
		ForwardB_ALU = 2'b00;
		ForwardA_Adder = 2'b00;
		ForwardB_Adder = 2'b00;
		ForwardC = 2'b00;
	end
	always @(ID_EX_rm1 or ID_EX_rn1 or ID_EX_rd1 or ID_EX_rm2 or ID_EX_rd2 or
				EX_MEM_rd or EX_MEM_RegWrite or EX_MEM_rd11 or
				MEM_WB_rd1 or MEM_WB_rd2 or MEM_WB_RegWrite1 or MEM_WB_RegWrite2)
	begin
		if( (ID_EX_rd2 == EX_MEM_rd) && EX_MEM_RegWrite == 1'b1 ) ForwardA_ALU = 2'b01;
		else if( (ID_EX_rd2 == MEM_WB_rd2) && MEM_WB_RegWrite2 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rd2 == EX_MEM_rd))) ForwardA_ALU = 2'b10;
		else if( (ID_EX_rd2 == MEM_WB_rd1) && MEM_WB_RegWrite1 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rd2 == EX_MEM_rd))) ForwardA_ALU = 2'b11;
		else ForwardA_ALU = 2'b00;
		
		if( (ID_EX_rm2 == EX_MEM_rd) && EX_MEM_RegWrite == 1'b1 ) ForwardB_ALU = 2'b01;
		else if( (ID_EX_rm2 == MEM_WB_rd2) && MEM_WB_RegWrite2 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rm2 == EX_MEM_rd))) ForwardB_ALU = 2'b10;
		else if( (ID_EX_rm2 == MEM_WB_rd1) && MEM_WB_RegWrite1 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rm2 == EX_MEM_rd))) ForwardB_ALU = 2'b11;
		else ForwardB_ALU = 2'b00;
		
		if( (ID_EX_rm1 == EX_MEM_rd) && EX_MEM_RegWrite == 1'b1 ) ForwardA_Adder = 2'b01;
		else if( (ID_EX_rm1 == MEM_WB_rd2) && MEM_WB_RegWrite2 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rm1 == EX_MEM_rd))) ForwardA_Adder = 2'b10;
		else if( (ID_EX_rm1 == MEM_WB_rd1) && MEM_WB_RegWrite1 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rm1 == EX_MEM_rd))) ForwardA_Adder = 2'b11;
		else ForwardA_Adder = 2'b00;
		
		if( (ID_EX_rn1 == EX_MEM_rd) && EX_MEM_RegWrite == 1'b1 ) ForwardB_Adder = 2'b01;
		else if( (ID_EX_rn1 == MEM_WB_rd2) && MEM_WB_RegWrite2 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rn1 == EX_MEM_rd))) ForwardB_Adder = 2'b10;
		else if( (ID_EX_rn1 == MEM_WB_rd1) && MEM_WB_RegWrite1 == 1'b1 && !(EX_MEM_RegWrite == 1'b0 && (ID_EX_rn1 == EX_MEM_rd))) ForwardB_Adder = 2'b11;
		else ForwardB_Adder = 2'b00;

		if( (ID_EX_rd1 == EX_MEM_rd) && EX_MEM_RegWrite == 1'b1 ) ForwardC = 2'b01;
		else if( (ID_EX_rd1 == MEM_WB_rd2) && MEM_WB_RegWrite2 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rd1 == EX_MEM_rd))) ForwardC = 2'b10;
		else if( (ID_EX_rd1 == MEM_WB_rd1) && MEM_WB_RegWrite1 == 1'b1 && !(EX_MEM_RegWrite == 1'b1 && (ID_EX_rd1 == EX_MEM_rd))) ForwardC = 2'b11;
		else ForwardC = 2'b00;
		
		if ( (MEM_WB_RegWrite1 == 1'b1) && (MEM_WB_rd1 == EX_MEM_rd11) ) ForwardD = 1'b1;
		else ForwardD = 1'b0;
		
	end

endmodule

module HazardDetectionUnit(input Branch, input neg_flag, input[1:0] ALUOp, output reg IF_ID_Write, output reg PCWrite, output reg CntrlSel,
                           input ID_EX_MemRead, input [2:0] ID_EX_Mem_Rd, 
						   input [2:0] IF_ID_inst1_Rm, IF_ID_inst1_Rn, IF_ID_inst1_Rd,
                           IF_ID_inst2_Rm, IF_ID_inst2_Rd);
  
  always@(Branch or neg_flag or ALUOp or ID_EX_MemRead or ID_EX_Mem_Rd or IF_ID_inst1_Rm or IF_ID_inst1_Rn or IF_ID_inst1_Rd or IF_ID_inst2_Rm or IF_ID_inst2_Rd )
  begin 
    IF_ID_Write = 1'b1;
    PCWrite = 1'b1;
    CntrlSel=1'b1;
    
    // ALU ops and Branch 
    if(Branch == 1'b1 && ALUOp!=2'b00)
    begin
      IF_ID_Write = 1'b0;
      PCWrite = 1'b0;
      CntrlSel = 1'b0;
    end
    
    
    // Load and R-Type(excluding Add) 
    if(ID_EX_MemRead == 1'b1 && ( ID_EX_Mem_Rd == IF_ID_inst2_Rm || ID_EX_Mem_Rd == IF_ID_inst2_Rd ) && ALUOp != 2'b00)
    begin
      IF_ID_Write = 1'b0;
      PCWrite = 1'b0;
      CntrlSel = 1'b0;
    end
    
    // Load and Load/Store 
    if(ID_EX_MemRead == 1'b1 && (ID_EX_Mem_Rd == IF_ID_inst1_Rm || ID_EX_Mem_Rd == IF_ID_inst1_Rn || ID_EX_Mem_Rd == IF_ID_inst1_Rd))
    begin
      IF_ID_Write = 1'b0;
      PCWrite = 1'b0;
      CntrlSel = 1'b0;
    end
    
  end
endmodule

// ---------------------- D flip flop memory -----------------------------

module D_ff_Mem11(input clk, input reset, input init, input writeEn, input d, output reg q);
	always@(negedge clk)
		begin
			if(reset)			q=init;
			else
				if(writeEn==1)	q=d;
		end
endmodule

// --------------------- 8 Bit MEMORY ----------------------------------

module Mem8bit(input clk, input reset, input [7:0] init, input writeEn, input [7:0] writeData, output [7:0] outR);
	D_ff_Mem11 D0(clk, reset, init[0], writeEn, writeData[0], outR[0]);
	D_ff_Mem11 D1(clk, reset, init[1], writeEn, writeData[1], outR[1]);
	D_ff_Mem11 D2(clk, reset, init[2], writeEn, writeData[2], outR[2]);
	D_ff_Mem11 D3(clk, reset, init[3], writeEn, writeData[3], outR[3]);
	D_ff_Mem11 D4(clk, reset, init[4], writeEn, writeData[4], outR[4]);
	D_ff_Mem11 D5(clk, reset, init[5], writeEn, writeData[5], outR[5]);
	D_ff_Mem11 D6(clk, reset, init[6], writeEn, writeData[6], outR[6]);
	D_ff_Mem11 D7(clk, reset, init[7], writeEn, writeData[7], outR[7]);
endmodule

// ---------------------- MEMORY BLOCK -----------------------------------

module MemBlock(input clk, input reset, input [31:0] init0, init1, init2, init3, init4, init5, init6, init7, input MemWrite, 
				input [7:0] decOut, input [31:0] writeData, 
				output [7:0] outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, 
				outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31);

	Mem8bit m00(clk, reset, init0[7:0], MemWrite&decOut[0], writeData[7:0], outR0);
	Mem8bit m01(clk, reset, init0[15:8], MemWrite&decOut[0], writeData[15:8], outR1);
	Mem8bit m02(clk, reset, init0[23:16], MemWrite&decOut[0], writeData[23:16], outR2);
	Mem8bit m03(clk, reset, init0[31:24], MemWrite&decOut[0], writeData[31:24], outR3);
	
	Mem8bit m10(clk, reset, init1[7:0], MemWrite&decOut[1], writeData[7:0], outR4);
	Mem8bit m11(clk, reset, init1[15:8], MemWrite&decOut[1], writeData[15:8], outR5);
	Mem8bit m12(clk, reset, init1[23:16], MemWrite&decOut[1], writeData[23:16], outR6);
	Mem8bit m13(clk, reset, init1[31:24], MemWrite&decOut[1], writeData[31:24], outR7);
	
	Mem8bit m20(clk, reset, init2[7:0], MemWrite&decOut[2], writeData[7:0], outR8);
	Mem8bit m21(clk, reset, init2[15:8], MemWrite&decOut[2], writeData[15:8], outR9);
	Mem8bit m22(clk, reset, init2[23:16], MemWrite&decOut[2], writeData[23:16], outR10);
	Mem8bit m23(clk, reset, init2[31:24], MemWrite&decOut[2], writeData[31:24], outR11);
	
	Mem8bit m30(clk, reset, init3[7:0], MemWrite&decOut[3], writeData[7:0], outR12);
	Mem8bit m31(clk, reset, init3[15:8], MemWrite&decOut[3], writeData[15:8], outR13);
	Mem8bit m32(clk, reset, init3[23:16], MemWrite&decOut[3], writeData[23:16], outR14);
	Mem8bit m33(clk, reset, init3[31:24], MemWrite&decOut[3], writeData[31:24], outR15);
	
	Mem8bit m40(clk, reset, init4[7:0], MemWrite&decOut[4], writeData[7:0], outR16);
	Mem8bit m41(clk, reset, init4[15:8], MemWrite&decOut[4], writeData[15:8], outR17);
	Mem8bit m42(clk, reset, init4[23:16], MemWrite&decOut[4], writeData[23:16], outR18);
	Mem8bit m43(clk, reset, init4[31:24], MemWrite&decOut[4], writeData[31:24], outR19);
	
	Mem8bit m50(clk, reset, init5[7:0], MemWrite&decOut[5], writeData[7:0], outR20);
	Mem8bit m51(clk, reset, init5[15:8], MemWrite&decOut[5], writeData[15:8], outR21);
	Mem8bit m52(clk, reset, init5[23:16], MemWrite&decOut[5], writeData[23:16], outR22);
	Mem8bit m53(clk, reset, init5[31:24], MemWrite&decOut[5], writeData[31:24], outR23);
	
	Mem8bit m60(clk, reset, init6[7:0], MemWrite&decOut[6], writeData[7:0], outR24);
	Mem8bit m61(clk, reset, init6[15:8], MemWrite&decOut[6], writeData[15:8], outR25);
	Mem8bit m62(clk, reset, init6[23:16], MemWrite&decOut[6], writeData[23:16], outR26);
	Mem8bit m63(clk, reset, init6[31:24], MemWrite&decOut[6], writeData[31:24], outR27);
	
	Mem8bit m70(clk, reset, init7[7:0], MemWrite&decOut[7], writeData[7:0], outR28);
	Mem8bit m71(clk, reset, init7[15:8], MemWrite&decOut[7], writeData[15:8], outR29);
	Mem8bit m72(clk, reset, init7[23:16], MemWrite&decOut[7], writeData[23:16], outR30);
	Mem8bit m73(clk, reset, init7[31:24], MemWrite&decOut[7], writeData[31:24], outR31);

endmodule


// ---------------------- MEMORY Decoder 3to8 -----------------------------------

module MemDec3to8(input [2:0] MemDest, output reg [7:0] MemDecOut);	 
	 always@(MemDest)
	 	case(MemDest)
			3'b000: MemDecOut=8'b00000001;
			3'b001: MemDecOut=8'b00000010;
			3'b010: MemDecOut=8'b00000100;
			3'b011: MemDecOut=8'b00001000;
			3'b100: MemDecOut=8'b00010000;
			3'b101: MemDecOut=8'b00100000;
			3'b110: MemDecOut=8'b01000000;
			3'b111: MemDecOut=8'b10000000;
		endcase
endmodule

// ---------------------- MEMORY Mux 8 to 1 -----------------------------------
 
module MemMux8to1(input [7:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7, input [2:0] Sel, output reg [7:0] outBus);	
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or Sel)
		case (Sel)
			3'b000: outBus=outR0;
			3'b001: outBus=outR1;
			3'b010: outBus=outR2;
			3'b011: outBus=outR3;
			3'b100: outBus=outR4;
			3'b101: outBus=outR5;
			3'b110: outBus=outR6;
			3'b111: outBus=outR7;
		endcase
endmodule 


// ---------------------- MEMORY Mux 2 to 1 -----------------------------------

module MemMux2to1(input [7:0] outBus, input [7:0] zero, input MemRead, output reg [7:0] MemOut);	
	always@(outBus or zero or MemRead)
		case (MemRead)
			1'b0: MemOut=outBus;
			1'b1: MemOut=zero;
		endcase	
endmodule

// ---------------------- MEMORY (MAIN) -----------------------------------

module Memory(input clk, input reset, input [31:0] init0, init1, init2, init3, init4, init5, init6, init7, 
				input MemWrite, input MemRead, input [31:0] Addr, input [31:0] writeData, 
				output [7:0] MemBus0, MemBus1, MemBus2, MemBus3);
	wire [7:0] MemDecOut, outBus0, outBus1, outBus2, outBus3;
	wire [7:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
	outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31;
	
	MemDec3to8 MD0(Addr[4:2], MemDecOut);		
	MemBlock MB0(clk, reset, init0, init1, init2, init3, init4, init5, init6, init7, 
	MemWrite, MemDecOut,writeData, 
	outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, 
	outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31);	
	MemMux8to1 MM8_0(outR0, outR4, outR8, outR12, outR16, outR20, outR24, outR28,Addr[4:2], outBus0	);
	MemMux8to1 MM8_1(outR1, outR5, outR9, outR13, outR17, outR21, outR25, outR29,Addr[4:2], outBus1);
	MemMux8to1 MM8_2(outR2, outR6, outR10, outR14, outR18, outR22, outR26, outR30,Addr[4:2], outBus2);
	MemMux8to1 MM8_3(outR3, outR7, outR11, outR15, outR19, outR23, outR27, outR31,Addr[4:2], outBus3);
	
	MemMux2to1 MM2_0(8'd0, outBus0, MemRead, MemBus0);
	MemMux2to1 MM2_1(8'd0, outBus1, MemRead, MemBus1);
	MemMux2to1 MM2_2(8'd0, outBus2, MemRead, MemBus2);
	MemMux2to1 MM2_3(8'd0, outBus3, MemRead, MemBus3);
	
endmodule

module Test1;
	reg clk;
	reg reset;
	wire [31:0] p3_ALUOut, p3_adderOut;
	reg [255:0] instruction_mem;
	
							  
	TopModule uut (.clk(clk), .reset(reset), .instruction_mem(instruction_mem), .p3_ALUOut(p3_ALUOut), .p3_adderOut(p3_adderOut));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#20  instruction_mem = {5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0,   //nop								
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0,   //nop 
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0};   //nop
		#20  reset=0;	
		
		#210 $finish; 
	end
endmodule

module Test2;
	reg clk;
	reg reset;
	wire [31:0] p3_ALUOut, p3_adderOut;
	reg [255:0] instruction_mem;
	
							  
	TopModule uut (.clk(clk), .reset(reset), .instruction_mem(instruction_mem), .p3_ALUOut(p3_ALUOut), .p3_adderOut(p3_adderOut));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#20  instruction_mem = {16'b10101_000_00000100,5'b00001, 11'd0, //add $r0,4
							  5'b00001, 11'd0, 5'b01010,2'b01,9'b010_011_100,  //load $r4,$r2,$r3								
							  5'b01000, 5'b00111, 6'b000100, 5'b01011, 2'b10,9'b010001101,  // mul $r4,$r4,$r0, store $r5,$r2,$r1
							  5'b11111, 5'd27, 6'b000110, 5'b00001, 11'd0,  // shft $r6 = $r0 << imm
							  5'b01000, 5'b00101, 6'b100000, 5'b00001, 11'd0,  //sub $r0 = $r0- $r4 - C  
							  5'b00001, 11'd0, 5'b11010,3'b011,8'b00000100,	   //branch
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b01110,11'd0,5'b00001,11'd0}; //invalid instruction
		#20  reset=0;	
		
		#210 $finish; 
	end
endmodule

module Test3;
	reg clk;
	reg reset;
	wire [31:0] p3_ALUOut, p3_adderOut;
	reg [255:0] instruction_mem;
	
							  
	TopModule uut (.clk(clk), .reset(reset), .instruction_mem(instruction_mem), .p3_ALUOut(p3_ALUOut), .p3_adderOut(p3_adderOut));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#20  instruction_mem = {5'b00001, 11'd0, 5'b01010,2'b01,9'b010_011_100,  //load $r4,$r2,$r3
							  5'b00001, 11'd0, 5'b01010,2'b01,9'b100_011_110,  //load $r6,$r4,$r3							
							  5'b01000, 5'b00101, 6'b110100, 5'b00001, 11'd0,  //sub $r4 = $r4- $r6 - C 
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0,   //nop 
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0,   //nop
							  5'b00001,11'd0,5'b00001,11'd0};   //nop
		#20  reset=0;	
		
		#120 $finish; 
	end
endmodule