// synopsys translate_off
`timescale 1ns / 10ps
// synopsys translate_on  
`ifndef SLICE
  `define SLICE(vector, size, index) vector[((index)+'d1)*(size)-'d1 : (index)*(size)]
`endif
//========================================================================================================================================================Whole module<<<<
//==================================================================================================================================================Module declaration<<<<
module L1IC
#(
  parameter PN_Ways         = 4,
  parameter PN_Lines        = 16,
  parameter PN_LineBytes    = 4,
  parameter PW_LineBytes    = $clog2(PN_LineBytes),
  parameter PW_L2Data       = PN_LineBytes*8,
  parameter PW_FullAddr     = 32,
  parameter PW_L2Addr       = PW_FullAddr-PW_LineBytes,
  parameter PW_RData        = PW_L2Data/2,
  parameter PW_RAddr        = PW_L2Addr+1,
  parameter PW_EvictCnt     = 2,
  parameter PW_BanksData    = 32,
  parameter PN_DataBanks    = PW_RData/PW_BanksData,
  parameter PN_L2WriteQueue = 2,
  parameter PN_L2ReadQueue  = 4
)
(
  input                               CLK_I,RST_I,

  input                               AValid,
  input  [PW_RAddr              -1:0] AAddr,
  output reg                          RValid,
  output reg [PW_RData          -1:0] RData,

// Wishbone
  output                              CYC_O,
  output reg                          STB_O,
  output                              WE_O,
  output [PW_L2Addr             -1:0] ADR_O,
  output [PN_LineBytes          -1:0] SEL_O,
  input                               STALL_I,
  input                               ERR_I,
  input                               RTY_I,
  input                               ACK_I,
  input  [PW_L2Data             -1:0] DAT_I
);
assign SEL_O = {PN_LineBytes{1'b1}};
assign WE_O = 0;
//====================================================================================================================================================================>>>>
//=============================================================================================================================================Localparams and genvars<<<<
genvar G_Way,G_Line,G_Block,G_Bit;
localparam LW_Lines     = $clog2(PN_Lines);
localparam LW_Ways      = $clog2(PN_Ways);
localparam LW_DataBanks = $clog2(PN_DataBanks);
localparam LW_BanksMask = PW_BanksData>>3;
localparam LW_BankAddr  = LW_Lines + LW_Ways + 1;// Each Line takes two slots in bank

localparam L_RLineStartBit = 0;
localparam L_RLineEndBit   = L_RLineStartBit + LW_Lines - 1;
localparam L_RTagStartBit  = L_RLineEndBit+1;
localparam L_RTagEndBit    = PW_RAddr-1;
localparam LW_RTag         = L_RTagEndBit - L_RTagStartBit + 1;

localparam L_LineStartBit = 0;
localparam L_LineEndBit   = L_LineStartBit + LW_Lines - 1;
localparam L_TagStartBit  = L_LineEndBit+1;
localparam L_TagEndBit    = PW_L2Addr-1;
localparam LW_Tag = L_TagEndBit - L_TagStartBit + 1;
//====================================================================================================================================================================>>>>
//==================================================================================================================================Global Regs and Wires Declarations<<<<
reg                              ReadValid_r;
reg  [PW_RAddr             -1:0] ReadAddr_r;

wire [LW_RTag              -1:0] ReadAddrTag;
wire [LW_Lines             -1:0] ReadAddrLine;
wire [LW_RTag              -1:0] PrefetchAddrTag;
wire [LW_Lines             -1:0] PrefetchAddrLine;

reg  [LW_RTag              -1:0]   Tag_r         [PN_Ways-1:0][PN_Lines-1:0];
reg  [PN_Lines             -1:0] Valid_r         [PN_Ways-1:0];
reg  [PW_EvictCnt          -1:0] Evict_r                      [PN_Lines-1:0];

wire [PN_Ways              -1:0]         Hit;
wire [PN_Ways              -1:0] PrefetchHit;

wire [LW_Ways              -1:0] HitWayWA        [PN_Ways-1:0];
wire [PN_Ways              -1:0] HitWayWA1       [LW_Ways-1:0];
wire [LW_Ways              -1:0] HitWay;

wire                             HitAnyWay;
wire                             SendToWB;
wire                             SendToDst;
wire                             PrefetchHitAnyWay;
wire                             PrefetchNext;

reg                              PrefetchValid;
reg  [PW_L2Addr            -1:0] PrefetchAddr;
reg                              DoPrefetch;

reg                              WB_CYC_r;
wire                             ReadResponse;

wire                             HitReadEn       [PN_DataBanks-1:0];
wire                             L2WriteEn       [PN_DataBanks-1:0];
wire [LW_BankAddr          -1:0] HitReadAddr     [PN_DataBanks-1:0];
wire [LW_BankAddr          -1:0] L2WriteAddr     [PN_DataBanks-1:0];
wire [PW_BanksData         -1:0] L2WriteData     [PN_DataBanks-1:0];

wire                             WE              [PN_DataBanks-1:0];
wire                             RE              [PN_DataBanks-1:0];
wire [LW_BankAddr          -1:0] BankAddr        [PN_DataBanks-1:0];
wire [PW_BanksData         -1:0] BankWData       [PN_DataBanks-1:0];
wire [PW_BanksData         -1:0] BankRData       [PN_DataBanks-1:0];

wire [PW_BanksData*PN_DataBanks-1:0] QB_P;

reg                              DataMemoryAccess;

wire                             SendDataAfterHit;
wire                             SendDataAfterMiss;
//====================================================================================================================================================================>>>>
//============================================================================================================================================Input Address buses save<<<<
always @(posedge CLK or negedge RST)
if (!RST) begin
  ReadValid_r <= 0;
  ReadAddr_r  <= 0;
end
else
  if (ReadValid_r & RValid)
      ReadValid_r   <= 1'b0;
  else
    if (AValid) begin
        ReadValid_r <= 1'b1;
        ReadAddr_r  <= AAddr;
    end

assign     ReadAddrLine       = ReadAddr_r[L_RLineEndBit:L_RLineStartBit];
assign     ReadAddrTag        = ReadAddr_r[ L_RTagEndBit: L_RTagStartBit];

assign PrefetchAddrLine       = PrefetchAddr[L_LineEndBit:L_LineStartBit];
assign PrefetchAddrTag        = PrefetchAddr[ L_TagEndBit: L_TagStartBit];
//====================================================================================================================================================================>>>>
//==============================================================================================================================================================TagMem<<<<
generate
  for (G_Line=0;G_Line<PN_Lines;G_Line=G_Line+1) begin:TagMem_Evict_LineGenblock
    always @(posedge CLK or negedge RST)
      if(!RST)                                       Evict_r[G_Line] <= {PW_EvictCnt{1'b0}};
      else if(ReadResponse & (ReadAddrLine==G_Line)) Evict_r[G_Line] <= Evict_r[G_Line] + {{(PW_EvictCnt-1){1'b0}},1'b1};
  end
  for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:TagMem_WayGenblock
    for (G_Line=0;G_Line<PN_Lines;G_Line=G_Line+1) begin:TagMem_LineGenblock
      always @(posedge CLK or negedge RST)
        if(!RST)                                                                          Valid_r[G_Way][G_Line] <= 1'b0;
        else if(ReadResponse & (Evict_r[ReadAddrLine]==G_Way) & (ReadAddrLine == G_Line)) Valid_r[G_Way][G_Line] <= 1'b1;

      always @(posedge CLK or negedge RST)
        if(!RST)                                                                  Tag_r[G_Way][G_Line] <= 0;
        else if(ReadResponse & (Evict_r[G_Line]==G_Way) & (ReadAddrLine==G_Line)) Tag_r[G_Way][G_Line] <= ReadAddrTag;
    end
  end
endgenerate
//====================================================================================================================================================================>>>>
//=================================================================================================================================================================Hit<<<<
generate
  for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:HITWays_Genblock
    assign         Hit[G_Way] = Valid_r[G_Way][    ReadAddrLine] & (    ReadAddrTag == Tag_r[G_Way][    ReadAddrLine]);
    assign PrefetchHit[G_Way] = Valid_r[G_Way][PrefetchAddrLine] & (PrefetchAddrTag == Tag_r[G_Way][PrefetchAddrLine]);

    assign    HitWayWA[G_Way] = Hit[G_Way] ? G_Way : 0;
    for (G_Bit=0;G_Bit<LW_Ways;G_Bit=G_Bit+1) begin:HitWaysWABits_Genblock
      assign       HitWayWA1[G_Bit][G_Way] = HitWayWA[G_Way][G_Bit];
    end
  end
  for (G_Bit=0;G_Bit<LW_Ways;G_Bit=G_Bit+1) begin:HitWaysBits_Genblock
    assign       HitWay[G_Bit] = |HitWayWA1[G_Bit];
  end
endgenerate
//====================================================================================================================================================================>>>>
//====================================================================================================================================================MissHit decision<<<<
assign HitAnyWay          = |Hit;
assign SendToWB           = ReadValid_r & ~HitAnyWay & ~WB_CYC_r & ~RValid;//By LAValid & RValid it'll send another requests to WB.
assign SendToDst          = ReadValid_r &  HitAnyWay & ~DoPrefetch;
assign PrefetchHitAnyWay  = |PrefetchHit;
assign PrefetchNext       = PrefetchValid & ~PrefetchHitAnyWay & ~WB_CYC_r;

always @(posedge CLK or negedge RST)
  if (!RST) begin
    PrefetchValid <= 0;
    PrefetchAddr  <= 0;
    DoPrefetch    <= 0;
  end
  else begin
    if (SendToWB | SendToDst) begin
      PrefetchValid<= 1'b1;
      PrefetchAddr <= ReadAddr_r[PW_RAddr-1:0] + {{(PW_L2Addr-1){1'b0}},1'b1};
    end
    else
      if (PrefetchNext)
        PrefetchValid <= 1'b0;
    if (PrefetchNext)
      DoPrefetch <= 1'b1;
    else
      if (ReadResponse)
      DoPrefetch <= 1'b0;
  end
//====================================================================================================================================================================>>>>
//======================================================================================================================================================L2 Access FIFO<<<<
assign CYC_O          = WB_CYC_r;
assign ADR_O          = DoPrefetch ? PrefetchAddr : AAddr;
assign STB_O          = WB_CYC_r;

always @(posedge CLK or negedge RST)
if (!RST) begin
  WB_CYC_r <= 0;
end
else
  if (WB_CYC_r & (ACK_I | ERR_I))
    WB_CYC_r <= 1'b0;
  else
    if (SendToWB | PrefetchNext)
      WB_CYC_r <= 1'b1;

assign ReadResponse = CYC_O & ACK_I;

//====================================================================================================================================================================>>>>
//=========================================================================================================================================================Data memory<<<<
generate
  for (G_Block=0;G_Block<PN_DataBanks;G_Block=G_Block+1) begin:DataMemBlocks_Genblock
    assign HitReadEn  [G_Block] = SendToDst;
    assign HitReadAddr[G_Block] = {HitWay, ReadAddrLine};

    assign L2WriteEn        [G_Block] = ReadResponse;
    assign L2WriteAddr      [G_Block] = DoPrefetch ? {Evict_r[PrefetchAddr],PrefetchAddr} : {Evict_r[ReadAddrLine],ReadAddrLine};

    assign BankAddr [G_Block] = HitReadEn[G_Block] ? HitReadAddr[G_Block] : L2WriteEn[G_Block]  ? L2WriteAddr[G_Block] : 0;

    assign L2WriteData      [G_Block] = DAT_I;

    assign BankWData [G_Block] = L2WriteEn[G_Block] ? L2WriteData[G_Block] : 0;

    assign WE[G_Block] = L2WriteEn[G_Block];
    assign RE[G_Block] = HitReadEn[G_Block];

    block_spram L1ic (.re1(RE[G_Block]), .addr1(BankAddr[G_Block]), .data1w(BankRData[G_Block]), .we1(WE[G_Block]), .data1r(BankWData[G_Block]), .reset(~RST_I), .clk(CLK));

    assign `SLICE(QB_P,PW_BanksData,G_Block) = BankRData[G_Block];
  end
endgenerate
always @(posedge CLK or negedge RST)
  if (!RST) begin
    DataMemoryAccess          <= 0;
  end
  else
    DataMemoryAccess        <= SendToDst;
  end
//====================================================================================================================================================================>>>>
//============================================================================================================================================Send Data to destination<<<<
assign SendDataAfterHit     =     DataMemoryAccess;
assign SendDataAfterMiss    = ReadResponse & ~DoPrefetch;

always @(posedge CLK or negedge RST)
if (!RST) begin
  RValid <= 0;
  RData  <= 0;
end
else begin
  if (SendDataAfterHit) begin
    RValid <= 1'b1;
    RData  <= QB_P;
  end
  else
    if (SendDataAfterMiss) begin
      RValid <= 1'b1;
      RData  <= DAT_I;
    end
    else
      RValid <= 1'b0;
end
//====================================================================================================================================================================>>>>
endmodule

`ifdef SLICE
  `undef SLICE
`endif
//====================================================================================================================================================================>>>>
