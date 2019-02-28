`timescale 1 ns / 1 ns

// 选择开发板
`define	DE2
//`define	DE2_70
//`define	DE2_115

// 选择 Z80 软核
`define	TV80
//`define	NEXTZ80


`ifdef DE1

`define FPGA_ALTERA
`define FPGA_ALTERA_C2
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_4K

`endif


`ifdef DE2

`define FPGA_ALTERA
`define FPGA_ALTERA_C2
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_4K

`endif


`ifdef DE2_70

`define FPGA_ALTERA
`define FPGA_ALTERA_C2
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_4K

`endif


`ifdef DE2_115

`define FPGA_ALTERA
`define FPGA_ALTERA_C4
`define CLOCK_27MHZ
`define AUDIO_WM8731
`define	BASE_SYS_ROM
`define RAM_ON_FPGA
`define	BASE_RAM_4K

`endif



module TP801_TOP(
	CLK50MHZ,

`ifdef CLOCK_27MHZ
	CLK27MHZ,
`endif

	////////////////////	7-SEG Dispaly	////////////////////
	HEX0_D,							//	Seven Segment Digit 0
	HEX1_D,							//	Seven Segment Digit 1
	HEX2_D,							//	Seven Segment Digit 2
	HEX3_D,							//	Seven Segment Digit 3
	HEX4_D,							//	Seven Segment Digit 4
	HEX5_D,							//	Seven Segment Digit 5
	HEX6_D,							//	Seven Segment Digit 6
	HEX7_D,							//	Seven Segment Digit 7

	// PS/2
	PS2_KBCLK,
	PS2_KBDAT,

	LED,

`ifdef AUDIO_WM8731
	////////////////	Audio CODEC		////////////////////////
	AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
	AUD_ADCDAT,						//	Audio CODEC ADC Data
	AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
	AUD_DACDAT,						//	Audio CODEC DAC Data
	AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
	AUD_XCK,						//	Audio CODEC Chip Clock

	////////////////////	I2C		////////////////////////////
	I2C_SDAT,						//	I2C Data
	I2C_SCLK,						//	I2C Clock

`endif

`ifdef GPIO_PIN
	////////////////////	GPIO	////////////////////////////
	//GPIO_0,							//	GPIO Connection 0
	//GPIO_1,							//	GPIO Connection 1
`endif

	BUTTON_N
);


input				CLK50MHZ;

`ifdef CLOCK_27MHZ
input				CLK27MHZ;
`endif


////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0_D;					//	Seven Segment Digit 0
output	[6:0]	HEX1_D;					//	Seven Segment Digit 1
output	[6:0]	HEX2_D;					//	Seven Segment Digit 2
output	[6:0]	HEX3_D;					//	Seven Segment Digit 3
output	[6:0]	HEX4_D;					//	Seven Segment Digit 4
output	[6:0]	HEX5_D;					//	Seven Segment Digit 5
output	[6:0]	HEX6_D;					//	Seven Segment Digit 6
output	[6:0]	HEX7_D;					//	Seven Segment Digit 7

// PS/2
input 			PS2_KBCLK;
input			PS2_KBDAT;

// LEDs
output	wire	[9:0]	LED;


`ifdef AUDIO_WM8731

////////////////////	Audio CODEC		////////////////////////////
// ADC
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data

// DAC
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output	wire	AUD_DACDAT;				//	Audio CODEC DAC Data

inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output	wire	AUD_XCK;				//	Audio CODEC Chip Clock

////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock

`endif


input	[3:0]		BUTTON_N;
wire	[3:0]		BUTTON;
assign	BUTTON	=	~BUTTON_N;


`ifdef GPIO_PIN
////////////////////////	GPIO	////////////////////////////////
//inout	[35:0]	GPIO_0;		//	GPIO Connection 0
//inout	[35:0]	GPIO_1;		//	GPIO Connection 1
//output	[35:0]	GPIO_0;		//	GPIO Connection 0
//output	[35:0]	GPIO_1;		//	GPIO Connection 1
`endif

// CLOCK & BUS
wire				BASE_CLK = CLK50MHZ;

reg		[4:0]		CLK;

reg					MEM_OP_WR;
//reg					MEM_RD;

// Processor
reg					CPU_CLK;
(*keep*)wire	[15:0]		CPU_A;
(*keep*)wire	[7:0]		CPU_DI;
(*keep*)wire	[7:0]		CPU_DO;

wire				CPU_RESET;
wire				CPU_HALT;
wire				CPU_WAIT;

wire				CPU_MREQ;
wire				CPU_RD;
wire				CPU_WR;
wire				CPU_IORQ;

reg					CPU_INT;
wire				CPU_NMI;
wire				CPU_M1;


wire				CPU_BUSRQ;
wire				CPU_BUSAK;

wire				CPU_RFSH;

`ifdef NEXTZ80

wire				CPU_RESET_N;
wire				CPU_HALT_N;
wire				CPU_WAIT_N;

wire				CPU_MREQ_N;
wire				CPU_RD_N;
wire				CPU_WR_N;
wire				CPU_IORQ_N;

wire				CPU_INT_N;
wire				CPU_NMI_N;
wire				CPU_M1_N;

wire				CPU_BUSRQ_N;
wire				CPU_BUSAK_N;

wire				CPU_RFSH_N;

`endif


// ROM IO RAM
wire	[7:0]		SYS_ROM_DATA;

wire	[7:0]		PROM1_DATA;
wire	[7:0]		PROM2_DATA;

wire				RAM_2_WR;
wire	[7:0]		RAM_2_DATA_OUT;

wire				ADDRESS_ROM;
wire				ADDRESS_PROM1;
wire				ADDRESS_PROM2;
wire				ADDRESS_2;

wire				ADDRESS_IO_PIO;
wire				ADDRESS_IO_CTC;

wire				ADDRESS_IO_PIO_N;
wire				ADDRESS_IO_CTC_N;

wire				ADDRESS_IO_SEG;
wire				ADDRESS_IO_DIGT;
wire				ADDRESS_IO_KB;


wire	[7:0]		Z80PIO_DATA_OUT;
wire	[7:0]		Z80CTC_DATA_OUT;


reg		[7:0]		LATCHED_IO_SEG;
reg		[7:0]		LATCHED_IO_DIGT;

// keyboard
reg		[4:0]		KB_CLK;

wire	[7:0]		SCAN;
wire				PRESS;
wire				PRESS_N;
wire				EXTENDED;

reg		[29:0]		KEY;
reg		[9:0]		KEY_EX;
reg		[11:0]		KEY_Fxx;
wire	[7:0]		KB_DATA;
//reg	[63:0]		LAST_KEY;
//reg				CAPS_CLK;
//reg				CAPS;
(*keep*)wire				A_KEY_PRESSED;

// cassette

wire	[1:0]		CASS_OUT;
wire				CASS_IN;
wire				CASS_IN_L;
wire				CASS_IN_R;



// other
wire				SYS_RESET_N;
wire				RESET_N;
wire				RESET_AHEAD_N;

reg		[16:0]		RESET_KEY_COUNT;
wire				RESET_KEY_N;


//	All inout port turn to tri-state
//assign	DRAM_DQ		=	16'hzzzz;
//assign	FL_DQ		=	8'hzz;
//assign	SRAM_DQ		=	16'hzzzz;
//assign	SD_DAT		=	1'bz;
assign	I2C_SDAT	=	1'bz;
assign	AUD_ADCLRCK	=	1'bz;
assign	AUD_DACLRCK	=	1'bz;
assign	AUD_BCLK	=	1'bz;


// reset

assign SYS_RESET_N = !BUTTON[0];

RESET_DE RESET_DE(
	.CLK(CLK50MHZ),			// 50MHz
	.SYS_RESET_N(SYS_RESET_N && RESET_KEY_N),
	.RESET_N(RESET_N),		// 50MHz/32/65536
	.RESET_AHEAD_N(RESET_AHEAD_N)	// 提前恢复，可以接 FL_RESET_N
);


// 键盘 ctrl + f12 系统复位
assign RESET_KEY_N = RESET_KEY_COUNT[16];


`ifdef SIMULATE
initial
	begin
		CLK = 5'b0;
	end
`endif


`ifdef CLOCK_27MHZ

VGA_Audio_PLL  VGA_AUDIO_PLL(.inclk0(CLK27MHZ),.c0(),.c1(AUD_CTRL_CLK));

`endif

// CPU clock

// 3.9936MHz/2 = 1.9968MHz

// 正常速度 50MHZ / 25 = 2MHz

// 同步内存操作
// 写 0 CPU 写信号和地址 1 锁存写和地址 2 完成写操作
// 读 0 CPU 读信号和地址 1 锁存读和地址 2 完读写操作，开始输出数据

// 读取需要中间间隔一个时钟


`ifdef SIMULATE
initial
	begin
		CLK = 5'b0;
	end
`endif

always @(posedge BASE_CLK or negedge RESET_N)
	if(~RESET_N)
	begin
		CPU_CLK					<=	1'b0;

		MEM_OP_WR				<=	1'b0;

		LATCHED_IO_SEG			<=	8'b0;
		LATCHED_IO_DIGT			<=	8'b0;

		CLK						<=	5'd0;
	end
	else
	begin
		case (CLK[4:0])
		5'd0:
			begin
				// 同步内存，等待读写信号建立
				CPU_CLK				<=	1'b1;

				MEM_OP_WR			<=	1'b1;

				CLK					<=	5'd1;
			end

		5'd1:
			begin
				// 同步内存，锁存读写信号和地址
				CPU_CLK				<=	1'b0;
				MEM_OP_WR			<=	1'b0;

				if({CPU_IORQ,CPU_RD,CPU_WR,ADDRESS_IO_SEG}==4'b1011)
					LATCHED_IO_SEG		<=	CPU_DO;

				if({CPU_IORQ,CPU_RD,CPU_WR,ADDRESS_IO_DIGT}==4'b1011)
					LATCHED_IO_DIGT		<=	CPU_DO;

				CLK					<=	5'd2;
			end

		5'd2:
			begin
				// 完成读写操作，开始输出
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	5'd3;
			end


		5'd7:
			begin
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	5'd8;
			end

		// 正常速度
		5'd24:
			begin
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	5'd0;
			end
		default:
			begin
				CPU_CLK				<=	1'b0;

				MEM_OP_WR			<=	1'b0;

				CLK					<=	CLK + 1'b1;
			end
		endcase
	end

	//vga_pll vgapll(CLK50MHZ, VGA_CLOCK);
	/* This module generates a clock with half the frequency of the input clock.
	 * For the VGA adapter to operate correctly the clock signal 'clock' must be
	 * a 50MHz clock. The derived clock, which will then operate at 25MHz, is
	 * required to set the monitor into the 640x480@60Hz display mode (also known as
	 * the VGA mode).
	 */


wire [7:0] InPort;

assign	InPort	=	(ADDRESS_IO_KB)		?	KB_DATA			:
					(ADDRESS_IO_PIO)	?	Z80PIO_DATA_OUT	:
					(ADDRESS_IO_CTC)	?	Z80CTC_DATA_OUT	:
											8'hFF			;

// CPU

`ifdef NEXTZ80

// 输入控制信号 RESET_N INT_N NMI_N WAIT_N BUSRQ_N DI

NextZ80 Z80CPU (
	.DI(CPU_IORQ ? (CPU_M1 ? 8'b00000000 : InPort) : CPU_DI),
	.DO(CPU_DO),
	.ADDR(CPU_A),
	.WR(CPU_WR),
	.MREQ(CPU_MREQ),
	.IORQ(CPU_IORQ),
	.HALT(CPU_HALT),
	.CLK(CPU_CLK),
	.RESET(CPU_RESET),
	.INT(CPU_INT),
	.NMI(CPU_NMI),
	.WAIT(CPU_WAIT),
	.M1(CPU_M1)
);

`endif


`ifdef TV80

assign CPU_M1 = ~CPU_M1_N;
assign CPU_MREQ = ~CPU_MREQ_N;
assign CPU_IORQ = ~CPU_IORQ_N;
assign CPU_RD = ~CPU_RD_N;
assign CPU_WR = ~CPU_WR_N;
assign CPU_RFSH = ~CPU_RFSH_N;
assign CPU_HALT= ~CPU_HALT_N;
assign CPU_BUSAK = ~CPU_BUSAK_N;

assign CPU_RESET_N = ~CPU_RESET;
assign CPU_WAIT_N = ~CPU_WAIT;
assign CPU_INT_N = ~CPU_INT;	// 50HZ
//assign CPU_INT_N = ~VGA_VS;	// 接 VGA 垂直回扫信号 60HZ
assign CPU_NMI_N = ~CPU_NMI;
assign CPU_BUSRQ_N = ~CPU_BUSRQ;

/*
  // Outputs
  m1_n, mreq_n, iorq_n, rd_n, wr_n, rfsh_n, halt_n, busak_n, A, dout,
  // Inputs
  reset_n, clk, wait_n, int_n, nmi_n, busrq_n, di
*/

tv80s Z80CPU (
	.m1_n(CPU_M1_N),
	.mreq_n(CPU_MREQ_N),
	.iorq_n(CPU_IORQ_N),
	.rd_n(CPU_RD_N),
	.wr_n(CPU_WR_N),
	.rfsh_n(CPU_RFSH_N),
	.halt_n(CPU_HALT_N),
	.busak_n(CPU_BUSAK_N),
	.A(CPU_A),
	.dout(CPU_DO),
	.reset_n(CPU_RESET_N),
	.clk(CPU_CLK),
	.wait_n(CPU_WAIT_N),
	.int_n(CPU_INT_N),
	.nmi_n(CPU_NMI_N),
	.busrq_n(CPU_BUSRQ_N),
	.di(CPU_IORQ_N ? CPU_DI : (CPU_M1_N ? InPort: 8'b0000_0000))
);

`endif

assign CPU_RESET = ~RESET_N;

assign CPU_NMI = 1'b0;

// LASER310 的 WAIT_N 始终是高电平。
assign CPU_WAIT = 1'b0;

//assign CPU_WAIT = CPU_MREQ && (~CLKStage[2]);


// 0000 -- 07FF ROM 2KB
// 0800 -- 0FFF PROM1 2KB
// 1000 -- 17FF PROM2 2KB
// 2000 -- 2FFF RAM 4KB

assign ADDRESS_ROM			=	(CPU_A[15:11] == 5'b00000)?1'b1:1'b0;
assign ADDRESS_PROM1		=	(CPU_A[15:11] == 5'b00001)?1'b1:1'b0;
assign ADDRESS_PROM2		=	(CPU_A[15:11] == 5'b00010)?1'b1:1'b0;

// 2000 -- 2FFF RAM 4KB
assign ADDRESS_RAM_2		=	(CPU_A[15:12] == 4'h2)?1'b1:1'b0;

// 80H --- 83H
assign ADDRESS_IO_PIO		=	(CPU_A[7:2] == 6'b1000_00)?1'b1:1'b0;
assign ADDRESS_IO_PIO_N		=	(CPU_A[7:2] == 6'b1000_00)?1'b0:1'b1;

// 84H --- 87H
assign ADDRESS_IO_CTC		=	(CPU_A[7:2] == 6'b1000_01)?1'b1:1'b0;
assign ADDRESS_IO_CTC_N		=	(CPU_A[7:2] == 6'b1000_01)?1'b0:1'b1;

// 88H --- 8BH
assign ADDRESS_IO_SEG		=	(CPU_A[7:2] == 6'b1000_10)?1'b1:1'b0;

// 8CH --- 8FH
assign ADDRESS_IO_DIGT		=	(CPU_A[7:2] == 6'b1000_11)?1'b1:1'b0;

// 90H --- 93H
assign ADDRESS_IO_KB		=	(CPU_A[7:2] == 6'b1001_00)?1'b1:1'b0;



assign RAM_2_WR			= ({ADDRESS_RAM_2,MEM_OP_WR,CPU_WR,CPU_IORQ} == 4'b1110)?1'b1:1'b0;


`ifdef	RAM_ON_FPGA

assign CPU_DI = 	ADDRESS_ROM			? SYS_ROM_DATA		:
					ADDRESS_PROM1		? PROM1_DATA		:
					ADDRESS_PROM2		? PROM2_DATA		:
					ADDRESS_RAM_2		? RAM_2_DATA_OUT	:
					8'hzz;

`endif


assign	PROM1_DATA	= 8'bz;
assign	PROM2_DATA	= 8'bz;


`ifdef BASE_SYS_ROM

`ifdef FPGA_ALTERA

sys_rom_altera sys_rom(
	.address(CPU_A[10:0]),
	.clock(BASE_CLK),
	.q(SYS_ROM_DATA)
);

`endif

`endif


`ifdef	RAM_ON_FPGA


`ifdef BASE_RAM_4K

`ifdef FPGA_ALTERA

ram_4k_altera sys_ram_2(
	.address(CPU_A[10:0]),
	.clock(BASE_CLK),
	.data(CPU_DO),
	.wren(CPU_MREQ & RAM_2_WR),
	.q(RAM_2_DATA_OUT)
);

`endif

`endif

`endif


// display

// tp801
//  a
// f b
//  g
// e c
//  d

// 6 5 4 3 2 1 0
// g f e d c b a

// DE系列开发板
//  0
// 5 1
//  6
// 4 2
//  3

// TP801和DE系列开发板数码管顺序相同

// SEG 对应位置为 0 数码管对应位置显示

assign HEX0_D	= LATCHED_IO_DIGT[0]?	LATCHED_IO_SEG[6:0]	:	7'b111_1111;
assign HEX1_D	= LATCHED_IO_DIGT[1]?	LATCHED_IO_SEG[6:0]	:	7'b111_1111;
assign HEX2_D	= LATCHED_IO_DIGT[2]?	LATCHED_IO_SEG[6:0]	:	7'b111_1111;
assign HEX3_D	= LATCHED_IO_DIGT[3]?	LATCHED_IO_SEG[6:0]	:	7'b111_1111;
assign HEX4_D	= LATCHED_IO_DIGT[4]?	LATCHED_IO_SEG[6:0]	:	7'b111_1111;
assign HEX5_D	= LATCHED_IO_DIGT[5]?	LATCHED_IO_SEG[6:0]	:	7'b111_1111;

assign HEX6_D	= 7'b111_1111;
assign HEX7_D	= 7'b111_1111;


// keyboard

/*****************************************************************************
* Convert PS/2 keyboard to ASCII keyboard
******************************************************************************/

/*

TP801 KEYBOARD

PROG LOAD DUMP BP
MEM  PORT REG  REG'
7    8    9    A    NEXT
4    5    6    B    MON
1    2    3    C    SETP
0    F    E    D    EXEC


TP801A KEYBOARD

2FB8 2FBA 2FBC 2FBE
PROG MOVE PORT BP

DISP REG' DUMP LOAD
MEM  REG  LAST NEXT

7    8    9    A    MON'
H    L

4    5    6    B    MON
IX   IY   I

1    2    3    C    SETP
PC   SP   IFF

0    F    E    D    EXEC

*/


// P  [  ]  \
// K  L  ;  '
// 7  8  9  A   N
// 4  5  6  B   M
// 1  2  3  C   ,
// 0  F  E  D   .

// 29 23 17 11  5
// 28 22 16 10  4
// 27 21 15  9  3
// 26 20 14  8  2
// 25 19 13  7  1
// 24 18 12  6  0


/*
`ifdef SIMULATE
initial
	begin
		LAST_KEY = 30'b0;
	end
`endif
*/

wire	[29:0]	KEY_SEL		=	KEY|(~{5{LATCHED_IO_DIGT[5:0]}});

wire KB_DATA_BIT7 = 1'b1;
wire KB_DATA_BIT6 = 1'b1;
wire KB_DATA_BIT5 = 1'b1;
wire KB_DATA_BIT4 = (KEY_SEL[29:24]==6'b11_1111);
wire KB_DATA_BIT3 = (KEY_SEL[23:18]==6'b11_1111);
wire KB_DATA_BIT2 = (KEY_SEL[17:12]==6'b11_1111);
wire KB_DATA_BIT1 = (KEY_SEL[11: 6]==6'b11_1111);
wire KB_DATA_BIT0 = (KEY_SEL[ 5: 0]==6'b11_1111);


/*
// 写法差别

wire [4:0]	KB_SEL0 = (LATCHED_IO_DIGT[0])	?	{KEY[24], KEY[18], KEY[12], KEY[ 6], KEY[ 0]}	:	5'b11111;
wire [4:0]	KB_SEL1 = (LATCHED_IO_DIGT[1])	?	{KEY[25], KEY[19], KEY[13], KEY[ 7], KEY[ 1]}	:	5'b11111;
wire [4:0]	KB_SEL2 = (LATCHED_IO_DIGT[2])	?	{KEY[26], KEY[20], KEY[14], KEY[ 8], KEY[ 2]}	:	5'b11111;
wire [4:0]	KB_SEL3 = (LATCHED_IO_DIGT[3])	?	{KEY[27], KEY[21], KEY[15], KEY[ 9], KEY[ 3]}	:	5'b11111;
wire [4:0]	KB_SEL4 = (LATCHED_IO_DIGT[4])	?	{KEY[28], KEY[22], KEY[16], KEY[10], KEY[ 4]}	:	5'b11111;
wire [4:0]	KB_SEL5 = (LATCHED_IO_DIGT[5])	?	{KEY[29], KEY[23], KEY[17], KEY[11], KEY[ 5]}	:	5'b11111;

wire KB_DATA_BIT7 = 1'b1;
wire KB_DATA_BIT6 = 1'b1;
wire KB_DATA_BIT5 = 1'b1;
wire KB_DATA_BIT4 = ({KB_SEL5[4], KB_SEL4[4], KB_SEL3[4], KB_SEL2[4], KB_SEL1[4], KB_SEL0[4]}==6'b11_1111);
wire KB_DATA_BIT3 = ({KB_SEL5[3], KB_SEL4[3], KB_SEL3[3], KB_SEL2[3], KB_SEL1[3], KB_SEL0[3]}==6'b11_1111);
wire KB_DATA_BIT2 = ({KB_SEL5[2], KB_SEL4[2], KB_SEL3[2], KB_SEL2[2], KB_SEL1[2], KB_SEL0[2]}==6'b11_1111);
wire KB_DATA_BIT1 = ({KB_SEL5[1], KB_SEL4[1], KB_SEL3[1], KB_SEL2[1], KB_SEL1[1], KB_SEL0[1]}==6'b11_1111);
wire KB_DATA_BIT0 = ({KB_SEL5[0], KB_SEL4[0], KB_SEL3[0], KB_SEL2[0], KB_SEL1[0], KB_SEL0[0]}==6'b11_1111);
*/

// BIT7 是录音读入信号

assign KB_DATA = { KB_DATA_BIT7, KB_DATA_BIT6, KB_DATA_BIT5, KB_DATA_BIT4, KB_DATA_BIT3, KB_DATA_BIT2, KB_DATA_BIT1, KB_DATA_BIT0 };

assign A_KEY_PRESSED = (KEY[29:0] == 30'h3FFF_FFFF) ? 1'b0:1'b1;

always @(posedge KB_CLK[3] or negedge SYS_RESET_N)
begin
	if(~SYS_RESET_N)
	begin
		KEY					<=	30'h3FFF_FFFF;
		KEY_EX				<=	10'h3FF;
		KEY_Fxx				<=	12'h000;
//		CAPS_CLK			<=	1'b0;
		RESET_KEY_COUNT		<=	17'h1FFFF;
	end
	else
	begin
		//KEY[?] <= CAPS;
		if(RESET_KEY_COUNT[16]==1'b0)
			RESET_KEY_COUNT <= RESET_KEY_COUNT+1;

		case(SCAN)
		8'h07:
		begin
			KEY_Fxx[11]	<= PRESS;	// F12 RESET
				if(PRESS && (KEY_EX[0]==PRESS_N))
				begin
					RESET_KEY_COUNT		<=	17'h0;
				end
		end
		//8'h78:	KEY_Fxx[10] <= PRESS;	// F11
		//8'h09:	KEY_Fxx[ 9] <= PRESS;	// F10
		//8'h01:	KEY_Fxx[ 8] <= PRESS;	// F9
		//8'h0A:	KEY_Fxx[ 7] <= PRESS;	// F8
		//8'h83:	KEY_Fxx[ 6] <= PRESS;	// F7
		//8'h0B:	KEY_Fxx[ 5] <= PRESS;	// F6
		//8'h03:	KEY_Fxx[ 4] <= PRESS;	// F5
		//8'h0C:	KEY_Fxx[ 3] <= PRESS;	// F4
		//8'h04:	KEY_Fxx[ 2] <= PRESS;	// F3
		//8'h06:	KEY_Fxx[ 1] <= PRESS;	// F2
		//8'h05:	KEY_Fxx[ 0] <= PRESS;	// F1

		8'h16:	KEY[25] <= PRESS_N;	// 1 !
		8'h1E:	KEY[19] <= PRESS_N;	// 2 @
		8'h26:	KEY[13] <= PRESS_N;	// 3 #
		8'h25:	KEY[26] <= PRESS_N;	// 4 $
		8'h2E:	KEY[20] <= PRESS_N;	// 5 %
		8'h36:	KEY[14] <= PRESS_N;	// 6 ^
		8'h3D:	KEY[27] <= PRESS_N;	// 7 &
		8'h3E:	KEY[21] <= PRESS_N;	// 8 *
		8'h46:	KEY[15] <= PRESS_N;	// 9 (
		8'h45:	KEY[24] <= PRESS_N;	// 0 )
		8'h1C:	KEY[ 9] <= PRESS_N;	// A
		8'h32:	KEY[ 8] <= PRESS_N;	// B
		8'h21:	KEY[ 7] <= PRESS_N;	// C
		8'h23:	KEY[ 6] <= PRESS_N;	// D
		8'h24:	KEY[12] <= PRESS_N;	// E
		8'h2B:	KEY[18] <= PRESS_N;	// F
		8'h42:	KEY[28] <= PRESS_N;	// K
		8'h4b:	KEY[22] <= PRESS_N;	// L
		8'h3a:	KEY[ 2] <= PRESS_N;	// M
		8'h31:	KEY[ 3] <= PRESS_N;	// N
		8'h4D:	KEY[29] <= PRESS_N;	// P

		8'h54:	KEY[23] <= PRESS_N;	// [ {
		8'h5B:	KEY[17] <= PRESS_N;	// ] }
		8'h5D:	KEY[11] <= PRESS_N;	// \ |

		8'h4C:	KEY[16] <= PRESS_N;	// ; :
		8'h52:	KEY[10] <= PRESS_N;	// ' "

		8'h41:	KEY[ 1] <= PRESS_N;	// , <
		8'h49:	KEY[ 0] <= PRESS_N;	// . >

//		8'h4E:	KEY[42] <= PRESS_N;	// - _
//		8'h0D:	KEY[?] <= PRESS_N;	// TAB
//		8'h55:	KEY[?] <= PRESS_N;	// = +
//		8'h66:	KEY_EX[8] <= PRESS_N;	// backspace
//		8'h0E:	KEY[?] <= PRESS_N;	// ` ~
//		8'h11	KEY[?] <= PRESS_N; // line feed (really right ALT (Extended) see below
//		8'h5A:	KEY[50] <= PRESS_N;	// CR
//		8'h44:	KEY[49] <= PRESS_N;	// O
//		8'h1D:	KEY[ 1] <= PRESS_N;	// W
//		8'h2D:	KEY[ 5] <= PRESS_N;	// R
//		8'h2C:	KEY[ 0] <= PRESS_N;	// T
//		8'h35:	KEY[48] <= PRESS_N;	// Y
//		8'h3C:	KEY[53] <= PRESS_N;	// U
//		8'h43:	KEY[51] <= PRESS_N;	// I
//		8'h1B:	KEY[ 9] <= PRESS_N;	// S
//		8'h34:	KEY[ 8] <= PRESS_N;	// G
//		8'h33:	KEY[56] <= PRESS_N;	// H
//		8'h3B:	KEY[61] <= PRESS_N;	// J
//		8'h42:	KEY[59] <= PRESS_N;	// K
//		8'h22:	KEY[17] <= PRESS_N;	// X
//		8'h2a:	KEY[21] <= PRESS_N;	// V
//		8'h15:	KEY[ 4] <= PRESS_N;	// Q
//		8'h1A:	KEY[20] <= PRESS_N;	// Z
//		8'h29:	KEY[36] <= PRESS_N;	// Space
//		8'h4A:	KEY[?] <= PRESS_N;	// / ?
//		8'h76:	KEY_EX[3] <= PRESS_N;	// Esc

		8'h14:	KEY_EX[0] <= PRESS_N;	// Ctrl either left or right
		8'h12:	KEY_EX[1] <= PRESS_N;	// L-Shift
		8'h59:	KEY_EX[2] <= PRESS_N;	// R-Shift
		8'h11:
		begin
			if(~EXTENDED)
					KEY_EX[3] <= PRESS_N;	// Repeat really left ALT
			else
					KEY_EX[4] <= PRESS_N;	// LF really right ALT
		end
		8'h75:	KEY_EX[5] <= PRESS_N;	// up
		8'h6B:	KEY_EX[6] <= PRESS_N;	// left
		8'h74:	KEY_EX[7] <= PRESS_N;	// right
		8'h72:	KEY_EX[8] <= PRESS_N;	// down
		endcase
	end
end


wire	PIO_IEI, PIO_IEO;

Z80PIO Z80PIO_IF(
	CPU_CLK,
	PA,
	PB,
//	D,
	CPU_DO,
	Z80PIO_DATA_OUT,
	CPU_A[1],
	CPU_A[0],
	CPU_M1_N,
	CPU_IORQ_N,
	RD_N,
	PIO_IEI,
	PIO_IEO,
	INT_N,
	ASTB_N,
	BSTB_N,
	ARDY,
	BRDY,
	ADDRESS_IO_PIO_N,
	CPU_RESET_N
);


wire	CTC_IEI, CTC_IEO;
wire	ZCTO0, ZCTO1, ZCTO2;

Z80CTC Z80CTC_IF(
	CPU_CLK,
//	D,
	CPU_DO,
	Z80CTC_DATA_OUT,
	CPU_M1_N,
	CPU_IORQ_N,
	CPU_RD_N,
	CTC_IEI,
	CTC_IEO,
	INT_N,
	CPU_A[0],
	CPU_A[1],
	TRG0,
	TRG1,
	TRG2,
	TRG3,
	ZCTO0,
	ZCTO1,
	ZCTO2,
	ADDRESS_IO_CTC_N,
	CPU_RESET_N
);


assign	CTC_IEI		=	1'b1;
assign	PIO_IEI		=	CTC_IEO;
assign	CASS_OUT	=	{1'b0,ZCTO1};


`ifdef SIMULATE
initial
	begin
		KB_CLK = 5'b0;
	end
`endif

always @ (posedge CLK50MHZ)				// 50MHz
	KB_CLK <= KB_CLK + 1'b1;			// 50/32 = 1.5625 MHz

ps2_keyboard KEYBOARD(
		.RESET_N(RESET_N),
		.CLK(KB_CLK[4]),
		.PS2_CLK(PS2_KBCLK),
		.PS2_DATA(PS2_KBDAT),
		.RX_SCAN(SCAN),
		.RX_PRESSED(PRESS),
		.RX_EXTENDED(EXTENDED)
);

assign PRESS_N = ~PRESS;


`ifdef AUDIO_WM8731

AUDIO_IF AUD_IF(
	//	Audio Side
	.oAUD_BCLK(AUD_BCLK),
	.oAUD_DACLRCK(AUD_DACLRCK),
	.oAUD_DACDAT(AUD_DACDAT),
	.oAUD_ADCLRCK(AUD_ADCLRCK),
	.iAUD_ADCDAT(AUD_ADCDAT),
	//	Control Signals
	.iSPK_A(SPEAKER_A),
	.iSPK_B(SPEAKER_B),
	.iCASS_OUT(CASS_OUT),
	.oCASS_IN_L(CASS_IN_L),
	.oCASS_IN_R(CASS_IN_R),
	// System
	.iCLK_18_4(AUD_CTRL_CLK),
	.iRST_N(RESET_N)
);

I2C_AV_Config AUD_I2C(
	//	Host Side
	.iCLK(CLK50MHZ),
	.iRST_N(RESET_N),
	//	I2C Side
	.I2C_SCLK(I2C_SCLK),
	.I2C_SDAT(I2C_SDAT)
);

assign	AUD_XCK = AUD_CTRL_CLK;

`endif


`ifdef AUDIO_WM8731
assign	CASS_IN			=	CASS_IN_L;
`endif


`ifdef GPIO_PIN
//assign	GPIO_0[35:0]	=	36'bz;		//	GPIO Connection 0
//assign	GPIO_1[35:0]	=	36'bz;		//	GPIO Connection 1
`endif

// other

//(*keep*)wire trap = CPU_IORQ && (CPU_WR||CPU_RD) && ADDRESS_IO_KB && A_KEY_PRESSED && (LATCHED_IO_DIGT!=8'h3F);
//(*keep*)wire trap = CPU_IORQ && (CPU_WR||CPU_RD) && ADDRESS_IO_CTC;
(*keep*)wire trap = CPU_IORQ && (CPU_WR||CPU_RD) && ADDRESS_IO_CTC && (CPU_A[1:0]==2'b01);
//(*keep*)wire trap = CPU_IORQ && (CPU_WR||CPU_RD) && ADDRESS_IO_PIO;

assign LED = {4'b0, trap, CASS_IN, CASS_OUT[0], A_KEY_PRESSED, CPU_CLK, CPU_RESET};

endmodule
