// 把输入输出信号 D 分离为 DI DO
//inout		[7:0]	D;
//input		[7:0]	DI;
//output	[7:0]	DO;

// Z-80的计数定时器CTC的功能
// 1.定时功能：定时地发出脉冲，在发出定时脉冲的同时，还可以提出中断请求INT。此定时脉冲定时的起始时间、起始时间的控制方法及定时间隔等均可由程序来设定。
// 2.计数功能：它能对外界的事件进行计数，当达到程序规定的计数时，输出一脉冲信号，并可同时发出中断请求信号INT。

/*
（1）D0：凡是控制字，D0一定为1，这是它的识别标志。CTC一收到D0=1的字，它会自动按控制字处理，把它存到各通道的“通道控制寄存器”内。
（2）D7：中断请求选择位。当D7=1允许通道中断；当D7=0禁止通道中断。
（3）D6：通道工作方式选择位。当D6=1通道进行计数方式工作；当D6=0通道进行定时方式工作。
（4）D5：定时工作时，确定对主频中的分额比。当D5=1表示定标器系数为256；当D5=0表示定标器系数为16，D5只在定时方式工作时才有意义，定标系数存在定标器内。定标器输入是主时钟，对主时钟中进行256或16分额后，输出一脉冲加到减1计数器。
（5）D4：对外加脉冲有效沿选择位。当D4=1表示CLK/TRG的上升沿有效；当D4=0表示CLK/TRG的下降沿有效。计数工作方式时，外输入脉冲就是计数脉冲；定时工作方式时，外输输入脉冲是启动定时器工作的触发脉冲，只有此触发脉冲加来时，定时器才开始工作。本位就是选择外输入脉冲的有效沿。
（6）D3：定时器开始工作的启动方式选择位。当D3=1表示定时器由CLK/TRG信号启动工作，即外触发工作；当D3=0表示定时器由装入时间常数启动工作，即装入时间常数字后立即开始工作，为内触发自行工作。D3只对定时方式工作有意义。
（7）D2：确定在本控制字之后写入CTC的是什么字。当D2=1表示紧跟控制字之后，CPU装入CTC的下一个字是时间常数；当D2=0表示下一个写入的字不是时间常数。
（8）D1：控制CTC停止工作的复位位。当D1=1通知CTC立即停止通道工作，ZC/TO不起作用，并禁止通道中断逻辑；当D1=0表示每当减1计数器减到0时，ZC/TO发出一脉冲信号，根据允许中断与否发中断请求，并将时间常数寄存器内容立即装入减1计数器。
*/

module Z80CTC(
	CLK,
//	D,
	DI,
	DO,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	IEO,
	INT_N,
	CS0,
	CS1,
	TRG0,
	TRG1,
	TRG2,
	TRG3,
	ZCTO0,
	ZCTO1,
	ZCTO2,
	CE_N,
	RESET_N
);

//inout		[7:0]	D;
input	[7:0]	DI;
output	[7:0]	DO;

input	CLK, M1_N, IORQ_N, RD_N, CS0, CS1, IEI, CE_N, RESET_N;
input	TRG0, TRG1, TRG2, TRG3;

output			INT_N;
output			ZCTO0;
output			ZCTO1;
output			ZCTO2;
wire			ZCTO3;
//output	reg		IEO;
output			IEO	=	INT_N;

wire	[7:0]	INT_V;

wire			INT0, INT1, INT2, INT3;

wire	[4:0]	INT0_V;
wire	[4:0]	INT1_V;
wire	[4:0]	INT2_V;
wire	[4:0]	INT3_V;

wire	[7:0]	DO0;
wire	[7:0]	DO1;
wire	[7:0]	DO2;
wire	[7:0]	DO3;

wire	CH0		=	({CS1,CS0}==2'b00);
wire	CH1		=	({CS1,CS0}==2'b01);
wire	CH2		=	({CS1,CS0}==2'b10);
wire	CH3		=	({CS1,CS0}==2'b11);


assign	DO	=	CE_N	?	8'bzzzz_zzzz	:
				INT_N	?	INT_V			:
				(~IEO)	?	8'bzzzz_zzzz	:
				CH0		?	DO0	:
				CH1		?	DO1	:
				CH2		?	DO2	:
							DO3	;

assign	INT_V	=	INT0	?	{INT0_V,2'b00,1'b0}	:
					INT1	?	{INT1_V,2'b01,1'b0}	:
					INT2	?	{INT2_V,2'b10,1'b0}	:
								{INT3_V,2'b11,1'b0}	;

assign	INT_N	=	({INT0,INT1,INT2,INT3}==4'b0);

Z80CTC_channel	ch0(
	CLK,
	DI,
	DO0,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	INT0,
	INT0_V,
	CH0,
	TRG0,
	ZCTO0,
	RESET_N
);

Z80CTC_channel	ch1(
	CLK,
	DI,
	DO1,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	INT1,
	INT1_V,
	CH1,
	TRG1,
	ZCTO1,
	RESET_N
);

Z80CTC_channel	ch2(
	CLK,
	DI,
	DO2,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	INT2,
	INT2_V,
	CH2,
	TRG2,
	ZCTO2,
	RESET_N
);

Z80CTC_channel	ch3(
	CLK,
	DI,
	DO3,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	INT3,
	INT3_V,
	CH3,
	TRG3,
	ZCTO3,
	RESET_N
);

endmodule


module Z80CTC_channel(
	CLK,
	DI,
	DO,
	M1_N,
	IORQ_N,
	RD_N,
	IEI,
	INT,
	INT_V,
	CS,
	TRG,
	ZCTO,
	RESET_N
);

//inout		[7:0]	D;
input	[7:0]	DI;
output	[7:0]	DO;

input	CLK, M1_N, IORQ_N, RD_N, CS, IEI, RESET_N;
input	TRG;

output			INT;
output	[4:0]	INT_V;
output			ZCTO;

reg		[7:0]	ch_mode;		// current mode
reg		[4:0]	ch_int_v;		// interrupt vector
reg		[7:0]	ch_tconst;		// time constant

reg		[7:0]	ch_div;

reg		[7:0]	TC;
reg		[7:0]	CC;

reg	ZCTO_C, INT_C;
reg	ZCTO_T, INT_T, TCTRG;


assign	INT_V	=	ch_int_v;
assign	DO	=	ch_mode[6]			?	CC				:
										TC				;

// CPU 向 CTC 写入
always @(posedge CLK or negedge RESET_N)
	if(~RESET_N)
	begin
		ch_mode		<=	8'b0000_0011;
		ch_int_v	<=	5'b0;
		ch_tconst	<=	8'hff;
	end
	else
	begin
		if({CS,IEI,M1_N,IORQ_N,RD_N}==5'b11101)
		begin
			if(ch_mode[2])
				begin
					ch_tconst		<=	DI;
					ch_mode[2]		<=	1'b0;
				end
			else
			begin
				// 中断矢量
				if(DI[0]==1'b0)
					ch_int_v		<=	DI[7:3];
				// 控制字
				if(DI[0]==1'b1)
					ch_mode			<=	DI;
			end
		end
	end

`ifdef SIMULATE
initial
	begin
		ch_div = 8'b0;
	end
`endif

always @ (posedge CLK)
	ch_div <= ch_div + 1'b1;

// 分频 控制字5 D5==1 256 D5==0 16
assign	ch_tclk	=	ch_mode[5]?ch_div[7]:ch_div[3];

assign	ZCTO	=	(~ch_mode[1])&(ch_mode[6]?ZCTO_C:ZCTO_T);
assign	INT		=	ch_mode[7]&(~ch_mode[1])&(ch_mode[6]?INT_C:INT_T);


// 逻辑电路做触发时钟？？？
/*
//wire	ch_clk		=	ch_mode[6]?((~ch_mode[4])^TRG):ch_tclk;
wire	ch_clk		=	(~ch_mode[6])	?	ch_tclk	:
						ch_mode[4]		?	TRG		:
											~TRG	;
*/

// 为简化设计，分离计数方式和定时方式，同时工作
wire	ch_trg	=	ch_mode[4]		?	TRG		:
										~TRG	;
// 计数方式
always @(posedge ch_trg or negedge RESET_N)
	if(~RESET_N)
	begin
		CC		<=	8'HFF;
		ZCTO_C	<=	1'b0;
		INT_C	<=	1'b0;
	end
	else
	begin
		if(~ch_mode[1])
			begin
				// 计数方式
				if(CC==8'b0)
				begin
						CC		<=	ch_tconst;
						INT_C	<=	1'b1;
						ZCTO_C	<=	~INT_C;	//保留一个时钟周期
				end
				else
				begin
						CC		<=	CC-1;
						INT_C	<=	1'b0;
						ZCTO_C	<=	1'b0;
				end
			end
	end

// 定时方式

// 定时启动（外触发）
always @(posedge ch_trg or negedge RESET_N)
	if(~RESET_N)
	begin
		TCTRG		<=	1'b0;
	end
	else
	begin
		if(TC==ch_tconst)
			TCTRG		<=	1'b1;
		else
			TCTRG		<=	1'b0;
	end

always @(posedge ch_tclk or negedge RESET_N)
	if(~RESET_N)
	begin
		TC		<=	8'HFF;
		ZCTO_T	<=	1'b0;
		INT_T	<=	1'b0;
	end
	else
	begin
		if(~ch_mode[1])
		begin
			if(TC==8'b0)
			begin
					TC		<=	ch_tconst;
					INT_T	<=	1'b0;
					ZCTO_T	<=	~INT_T;	//保留一个时钟周期
			end
			else
			begin
				INT_T	<=	1'b0;
				ZCTO_T	<=	1'b0;
				if(TC==ch_tconst)
				begin
					if( (~ch_mode[3]) || (ch_mode[3]&&TCTRG))
						TC		<=	TC-1;
				end
				else
				begin
						TC		<=	TC-1;
				end
			end
		end
	end

endmodule
