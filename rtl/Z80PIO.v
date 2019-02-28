/*
PIO芯片有两个独立的口子（口A、口B）可以直接与外设相连，每一个口子又有四种工作方式，这四种工作方式为：
①输出方式（方式0）；
②输入方式（方式1）；
③双向工作方式（方式2）；
④位控方式（方式3）。
每个口子只有两个地址单元供CPU访问，这两个单元一个是数据寄存器，另一个是控制寄存器（即C/信号），而实有的寄存器是6个，并且需要CPU向每个口子写入6个字。
这个矛后的解决，仍然采用在CPU向PIO写入的字上加进一些特征码，以便各个口子自行进行分辨，把它们存入6个寄存器中的对口寄存器。
CPU向PIO每个口子写入的六个字是：
①方式选择字；
②中断矢量低8位；
③中断允许控制字；
④I/O选择字；
⑤中断控制字；
➅屏蔽字。
*/

`define		MODE_OUTPUT			0
`define		MODE_INPUT			1
`define		MODE_BIDIRECTIONAL	2
`define		MODE_BIT_CONTROL	3

`define		NEXT_ANY			0
`define		NEXT_IOR			1
`define		NEXT_MASK			2

`define		ICW_ENABLE_INT		0x80
`define		ICW_AND_OR			0x40
`define		ICW_AND				0x40
`define		ICW_OR				0x00
`define		ICW_HIGH_LOW		0x20
`define		ICW_HIGH			0x20
`define		ICW_LOW				0x00
`define		ICW_MASK_FOLLOWS	0x10

module	Z80PIO(
	CLK,
	PA,
	PB,
//	D,
	DI,
	DO,
	CD,
	BA,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	IEO,
	INT_N,
	ASTB_N,
	BSTB_N,
	ARDY,
	BRDY,
	CE_N,
	RESET_N
);

input	[7:0]	PA;
input	[7:0]	PB;

//inout		[7:0]	D;
input	[7:0]	DI;
output	[7:0]	DO;

input	CLK, CD, BA, M1_N, IORQ_N, RD_N, ASTB_N, BSTB_N, IEI, CE_N, RESET_N;
output	reg IEO;
output	reg ARDY;
output	reg BRDY;
output	reg INT_N;

wire	INT_A_N, INT_B_N;
wire	IEO_A, IEO_B;

Z80PIO_channel	ch_A(
	CLK,
	P,
	DI,
	DO,
	CD,
//	BA,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	IEO_A,
	INT_A_N,
	STB_N,
	RDY,
	CE_N,
	RESET_N
);

Z80PIO_channel	ch_B(
	CLK,
	P,
	DI,
	DO,
	CD,
//	BA,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	IEO_B,
	INT_B_N,
	STB_N,
	RDY,
	CE_N,
	RESET_N
);

endmodule


module	Z80PIO_channel(
	CLK,
	P,
	DI,
	DO,
	CD,
//	BA,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	IEO,
	INT_N,
	STB_N,
	RDY,
	CE_N,
	RESET_N
);

input	[7:0]	P;

//inout		[7:0]	D;
input	[7:0]	DI;
output	[7:0]	DO;

input	CLK, CD, M1_N, IORQ_N, RD_N, STB_N, IEI, CE_N, RESET_N;
output	reg IEO;
output	reg RDY;
output	reg INT_N;

/*
①方式选择字；
②中断矢量低8位；
③中断允许控制字；
④I/O选择字；
⑤中断控制字；
➅屏蔽字。
*/

reg		[1:0]	ch_mode;		// mode register
reg		[7:0]	ch_int_v;		// interrupt vector

reg		[7:0]	ch_ior;			// input/output register
reg		[3:0]	ch_icw;			// interrupt control word
reg		[7:0]	ch_mask;		// interrupt mask
reg				ch_ie;			// interrupt enabled

reg		[7:0]	ch_data;		// latch data
reg		[7:0]	ch_input;		// input latch
reg		[7:0]	ch_output;		// output latch

/*
		bool m_rdy;                 // ready
		bool m_stb;                 // strobe

		// interrupts
		bool m_ie;                  // interrupt enabled
		bool m_ip;                  // interrupt pending
		bool m_ius;                 // interrupt under service
		bool m_match;               // logic equation match
*/

// CPU 向 PIO 写入

reg		[1:0]	ch_next_control_word;

always @(posedge CLK or negedge RESET_N)
	if(~RESET_N)
	begin
		ch_next_control_word	<=	`NEXT_ANY;
		ch_mode					<=	`MODE_INPUT;
		//ch_int_v				<=	8'b0;
		ch_ior					<=	8'b0;
		ch_mask					<=	8'hFF;
		ch_icw[3]				<=	1'b0;
		ch_ie					<=	1'b0;
	end
	else
	begin
		if({CE_N,IEI,M1_N,IORQ_N,RD_N}==5'b01101)
		begin
			if(CD)
			begin
				// 控制寄存器
				case(ch_next_control_word)
					`NEXT_ANY:
					begin
						if(DI[0]==1'b0)
						begin
							// 中断矢量
							ch_int_v				<=	DI;
						end
						else
						if(DI[3:0]==4'b1111)
						begin
							// 方式选择字
							ch_mode					<=	DI[7:6];
							if(DI[7:6]==2'b11)
								ch_next_control_word	<=	`NEXT_IOR;
						end
						else
						if(DI[3:0]==4'b0111)
						begin
							//	中断控制字
							ch_icw					<=	DI[7:4];
							if(DI[4])
								ch_next_control_word	<=	`NEXT_MASK;
						end
						else
						if(DI[3:0]==4'b0011)
						begin
							ch_ie					<=	DI[7];
						end
					end

					`NEXT_IOR:
					begin
						ch_ior					<=	DI;
						ch_next_control_word	<=	`NEXT_ANY;
					end

					`NEXT_MASK:
					begin
						ch_mask					<=	DI;
						ch_next_control_word	<=	`NEXT_ANY;
					end
				endcase
			end
			else
			begin
				// 数据寄存器
				ch_data		<=	DI;
				// 中断处理
			end
		end
	end

/*
always @(posedge CLK or negedge RESET_N)
	if(ch_mode==`MODE_BIT_CONTROL)


assign	DO	=	(ch_mode==`MODE_OUTPUT)	?	ch_data;
*/

assign	DO	=	(ch_mode==`MODE_INPUT)	?	ch_data		:	8'bz;


endmodule
