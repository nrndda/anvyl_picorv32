// synopsys translate_off
`timescale 1ns / 10ps
// synopsys translate_on
`ifndef SLICE
  `define SLICE(vector, size, index) vector[((index)+'d1)*(size)-'d1 : (index)*(size)]
`endif
//===========================================================================================================================================================NOTE/TODO<<<<
/*
1) Answer on two input requests to different WayGroups. DONE
2) Put blocked requests into queue(L2ReadQueueFIFO) after sending current to L2. Could lead to more latencies for this requests.
3) AHB and invalidate together, blocks.
4) heavy pathes for synthesis from input regs to memory through Utag.
*/
//====================================================================================================================================================================>>>>
//========================================================================================================================================================Whole module<<<<
//==================================================================================================================================================Module declaration<<<<
module L1DC
#(
  parameter PN_Ports        = 4,
  parameter PN_WaysGroups   = 2,
  parameter PN_Pointers     = PN_WaysGroups,
  parameter PN_Ways         = 4,//Ways per WaysGroup
  parameter PN_Lines        = 32,// Lines per Ways per WaysGroup.
  parameter PN_LineBytes    = 64,
  parameter PW_LineBytes    = $clog2(PN_LineBytes),
  parameter PW_Data         = PN_LineBytes*8,
  parameter PW_FullAddr     = 32,
  parameter PW_Address      = PW_FullAddr-PW_LineBytes,
  parameter PW_EvictCnt     = 2,
  parameter PN_DataBanks    = 32,
  parameter PW_DataBanks    = 32,
  parameter PN_L2WriteQueue = 4,//Was 8
  parameter PN_L2ReadQueue  = PN_L2WriteQueue*2,//Was*4
  parameter PW_ExtData      = PW_DataBanks,
  parameter P_ExAddrSBit    = 2,
  parameter P_ExAddrEBit    = 13
)
(
  input                               CLK, RST,
  input                               InvAll,
//--- RC addresses & control -------
 input   [4:0]                        RC_GRP, RC_ADR,
 input                                WR_RC, RD_RC,
 input   [1:0]                        GDBW,
 output  [PW_ExtData            -1:0] GDBR,
// AHB interface
  input  ExtAcsData, ExtAcsTag, ExtAcsValid, ExtAcsWay, ExtAcsRegs, ExtRead, ExtWrite,
  input  [P_ExAddrEBit:P_ExAddrSBit]  ExtAddr,
  input  [PW_ExtData            -1:0] ExtWData,
  output [PW_ExtData            -1:0] ExtRData,
// L2 Address/Write Port
  output                              L2AVALID,
  input                               L2AREADY,
  output                              L2AWRITE,
  output [PW_Address            -1:0] L2AADDR,
  output [PN_LineBytes          -1:0] L2AWSTRB,
  output [PW_Data               -1:0] L2AWDATA,
// L2 Invalidate interface
  input  [PW_Address            -1:0] L2INV_ADDR,
  input                               L2INV_VAL,
  output                              L2INV_RESP,
// L2 Read Port
  input                               L2RVALID,
  input  [PW_Data               -1:0] L2RDATA,
  output                              L2RREADY,
// L1 Address/Write Ports
  input  [PN_Ports              -1:0] AVALID,
  output [PN_Ports              -1:0] AREADY,
  input  [PN_Ports              -1:0] AWRITE,
  input  [PN_Ports*PW_Address   -1:0] AADDR,
  input  [PN_Ports*PW_Data      -1:0] AWDATA,
  input  [PN_Ports*PN_LineBytes -1:0] AWSTRB,
// L1 Read Ports
  output [PN_Ports              -1:0] RVALID,
  input  [PN_Ports              -1:0] RREADY,
  output [PN_Ports*PW_Data      -1:0] RDATA
);
//====================================================================================================================================================================>>>>
//=============================================================================================================================================Localparams and genvars<<<<
genvar G_Port,G_Group,G_Way,G_Line,G_Block,G_Slot,G_Pointer;
localparam LW_Lines     = $clog2(PN_Lines);
localparam LW_Ways      = $clog2(PN_Ways);
localparam LW_WaysGroups= $clog2(PN_WaysGroups);
localparam LW_Ports     = $clog2(PN_Ports);
localparam LW_DataBanks = $clog2(PN_DataBanks);
localparam LW_BanksMask = PW_DataBanks>>3;
localparam LW_BankAddr  = LW_Lines + LW_Ways;

localparam L_WaysGroupBit = 0; // Note First PW_LineBytes are useless here as L1 doesn't need bytes numbers in Lines.
localparam L_LineStartBit = L_WaysGroupBit + 1;
localparam L_LineEndBit   = L_LineStartBit + LW_Lines - 1;

localparam L_ExtBlockStartBit = 2;//32bit,4,bytes
localparam L_ExtBlockEndBit   = L_ExtBlockStartBit+LW_DataBanks-1;
localparam L_ExtLineStartBit  = L_ExtBlockEndBit+1;
localparam L_ExtLineEndBit    = L_ExtLineStartBit+LW_Ways+LW_Lines-1;
//====================================================================================================================================================================>>>>
//==================================================================================================================================Global Regs and Wires Declarations<<<<
reg  [P_ExAddrEBit:P_ExAddrSBit] ExtAddr_r;
reg  [PW_ExtData           -1:0] ExtWData_r;
reg                              ExtWrite_r;
reg                              ExtRead_r;
reg                              ExtAcsData_r;
reg                              ExtAcsData_2r;
reg                              ExtAcsRegs_r;
reg                              ExtAcsTag_r;
reg                              ExtAcsValid_r;
reg                              ExtAcsWay_r;

wire [PW_ExtData-1:0] HRReg;
wire [PW_ExtData-1:0] TagmemData;

wire [L_ExtBlockEndBit-L_ExtBlockStartBit:0] ExtAddrBlock;
wire [L_ExtLineEndBit -L_ExtLineStartBit :0] ExtAddrLine;

wire ExtCollisions;

wire [PW_Address                 -1:0] SrcAAddr   [PN_Ports-1:0];
wire [PW_Data                    -1:0] SrcAWData  [PN_Ports-1:0];
wire [PN_LineBytes               -1:0] SrcAWSTRB  [PN_Ports-1:0];

wire [PN_Ports                   -1:0] SameAddrDifDirection  [PN_Ports-1:0];
wire [PN_Ports                   -1:0] SrcASent;

reg [PN_Ports                    -1:0] SrcAValid_r;
reg [PN_Ports                    -1:0] SrcAWrite_r;
reg [PW_Address                  -1:0] SrcAAddr_r [PN_Ports-1:0];
reg [PW_Data                     -1:0] SrcAWData_r[PN_Ports-1:0];
reg [PN_LineBytes                -1:0] SrcAWSTRB_r[PN_Ports-1:0];

reg                                    InvValid_r;
reg [PW_Address                  -1:0] InvAddr_r;
wire[PW_Address*PN_Ports         -1:0] SrcAAddr_rP;

wire [LW_Ways      *PN_Pointers  -1:0] SrcAWay;
wire [LW_WaysGroups*PN_Pointers  -1:0] SrcAWaysGroup;
wire [PW_EvictCnt  *PN_WaysGroups-1:0] SrcAEvictCnt;

wire                                   BlockL2Access;
wire                                   ReadMissToL2;
wire                                   WriteToL2;
wire                                   CurSendToL2;
wire                                   CurWriteUpponHit;
wire                                   CurPrefetchNext;
wire                                   CurSendToDst;
wire                                   NextSendToDst;

wire                                   CurHit;
wire                                   CurSrcAValid;
wire                                   CurSrcAWrite;
wire [PW_Address                 -1:0] CurSrcAAddr;
wire [PW_Data                    -1:0] CurSrcAWData;
wire [PN_LineBytes               -1:0] CurSrcAWSTRB;
wire [LW_WaysGroups              -1:0] CurSrcAWaysGroup;
wire [LW_Ways                    -1:0] CurSrcAWay;
wire                                   CurORegEmptyAfterHit;
wire                                   CurPrefetchValid;
wire [PW_Address                 -1:0] CurPrefetchAAddr;
wire                                   CurPrefetchHit;

wire                                   NextHit;
wire                                   NextSrcAValid;
wire                                   NextSrcAWrite;
wire [PW_Address                 -1:0] NextSrcAAddr;
wire [LW_WaysGroups              -1:0] NextSrcAWaysGroup;
wire [LW_Ways                    -1:0] NextSrcAWay;
wire                                   NextORegEmptyAfterHit;

wire                                   ExtCtrlAcs;
wire                                   Ext_L1En;
wire                                   Ext_PrefetchEn;
reg                                    L1En;
reg                                    PrefetchEn;
reg  [PW_ExtData                 -1:0] ReadCounter;
reg  [PW_ExtData                 -1:0] ReadMissCounter;
reg  [PW_ExtData                 -1:0] PrefetchCounter;
reg  [PW_ExtData                 -1:0] WriteCounter;
reg  [PW_ExtData                 -1:0] WriteHitCounter;
reg  [PW_ExtData                 -1:0] L2ReadsCounter;
reg  [PW_ExtData                 -1:0] L2RespsCounter;
reg  [PW_ExtData                 -1:0] L2WritesCounter;

wire [PN_Ports                   -1:0] SrcAValid_mod;
wire                                   NextValidEnabled;
wire                                   PrevValidEnabled;
wire [LW_Ports                   -1:0] NextValid;
wire [LW_Ports                   -1:0] PrevValid;
wire                                   ChangeReadPointer;
wire [PN_Ports                   -1:0] PrefetchValid_mod;
wire                                   NextPrefetchValidEnabled;
wire                                   PrevPrefetchValidEnabled;
wire [LW_Ports                   -1:0] NextPrefetchValid;
wire [LW_Ports                   -1:0] PrevPrefetchValid;
wire                                   ResetPrefetchValid;
wire                                   ChangePrefetchPointer;
wire [LW_Ports                   -1:0] NextReadPointer;
wire [LW_Ports                   -1:0] NextPrefetchPointer;
reg  [LW_Ports                   -1:0] ReadPointer;
reg  [LW_Ports                   -1:0] PrefetchPointer;
reg  [PN_Ports                   -1:0] PrefetchValid;
reg  [PW_Address                 -1:0] PrefetchAddr     [PN_Ports-1:0];

wire                                   SrcAddrAlreadySent;
wire                                   PrefetchAddrAlreadySent;
wire [PN_L2WriteQueue            -1:0] AddrAlreadySent1;
wire [PN_L2ReadQueue             -1:0] AddrAlreadySent2;
wire [PN_L2WriteQueue            -1:0] PrefetchAddrAlreadySent1;
wire [PN_L2ReadQueue             -1:0] PrefetchAddrAlreadySent2;
wire                                   AddrAlreadySentAValid;
wire                                   AddrAlreadySentRValid;
wire                                   PortBlockedByL2Access;
wire                                   NextPortBlockedByL2Access;
wire                                   PrefetchFlagL2A;
wire                                   PrefetchFlag;
wire [PN_L2WriteQueue            -1:0] PortBlockedByL2Access1;
wire [PN_L2ReadQueue             -1:0] PortBlockedByL2Access2;
wire                                   PortBlockedByL2AccessAValid;
wire                                   PortBlockedByL2AccessAWrite;
wire                                   PortBlockedByL2AccessRValid;
wire [PN_L2WriteQueue            -1:0] NextPortBlockedByL2Access1;
wire [PN_L2ReadQueue             -1:0] NextPortBlockedByL2Access2;

wire                                   L2AValidTrans;
wire                                   L2RValidTrans;
wire                                   WFIFOEmpty,WFIFOFull;
wire                                   L2WFIFOWrite;
wire [LW_Ports                   -1:0] SentReadPointer;

wire                                   RFIFOEmpty;
wire                                   RFIFOFull;
wire                                   L2RFIFOWrite;
wire [PW_Address                 -1:0] L2RAddr;
wire [LW_Ports                   -1:0] L2RecPointer;

wire [PN_DataBanks               -1:0] ExtReadEn;
wire                                   HitReadEn        [PN_DataBanks-1:0];
wire [PN_DataBanks               -1:0] ExtWriteEn;
wire                                   L2WriteEn        [PN_DataBanks-1:0];
wire                                   WriteUpponHitEn  [PN_DataBanks-1:0];
wire [LW_BanksMask               -1:0] WriteUpponHitSTRB[PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0]       ExtReadAddr[PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0]       HitReadAddr[PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0]      ExtWriteAddr[PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0]       L2WriteAddr[PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0] WriteUpponHitAddr[PN_DataBanks-1:0];
wire [PW_DataBanks               -1:0]      ExtWriteData[PN_DataBanks-1:0];
wire [PW_DataBanks               -1:0]       L2WriteData[PN_DataBanks-1:0];
wire [PW_DataBanks               -1:0] WriteUpponHitData[PN_DataBanks-1:0];

wire                                   CEAN             [PN_DataBanks-1:0];
wire                                   WEAN             [PN_DataBanks-1:0];
wire                                   CEBN             [PN_DataBanks-1:0];
wire [LW_BanksMask               -1:0] WEM              [PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0] AA               [PN_DataBanks-1:0];
wire [LW_BankAddr                -1:0] AB               [PN_DataBanks-1:0];
wire [PW_DataBanks               -1:0] DA               [PN_DataBanks-1:0];
wire [PW_DataBanks               -1:0] QB               [PN_DataBanks-1:0];

wire [PN_DataBanks*(PN_DataBanks/PN_WaysGroups)-1:0] QBPerWayGroup  [PN_WaysGroups-1:0];

reg                                    DataMemoryAccess;
reg  [LW_WaysGroups              -1:0] DataMemoryWaysGroup;
reg                                    NextDataMemoryAccess;
reg  [LW_WaysGroups              -1:0] NextDataMemoryWaysGroup;
reg  [LW_DataBanks               -1:0] ExtReadBankPointer;
reg  [LW_Ports                   -1:0] DstPointer;
reg  [LW_Ports                   -1:0] NextDstPointer;

wire                                   SendDataAfterHit    [PN_Ports-1:0];
wire                                   SendNextDataAfterHit[PN_Ports-1:0];
wire                                   ORegEmptyAfterHit   [PN_Ports-1:0];
wire                                   SendDataAfterMiss   [PN_Ports-1:0];
wire                                   ORegEmptyAfterMiss  [PN_Ports-1:0];
wire                                   ClearORegValid      [PN_Ports-1:0];
wire                                   ORegValidTrans      [PN_Ports-1:0];

reg  [PN_Ports                   -1:0] DstRValid_r;
reg  [PW_Data                    -1:0] DstRData_r          [PN_Ports-1:0];
//====================================================================================================================================================================>>>>
//=================================================================================================================================================================PCU<<<<
assign GDBR       = (RC_GRP == 5'hB) & RD_RC ? (
                    (RC_ADR == 5'h10) ? {{PW_ExtData-2{1'b0}},PrefetchEn,L1En} :
                    (RC_ADR == 5'h11) ? ReadCounter :
                    (RC_ADR == 5'h12) ? ReadMissCounter :
                    (RC_ADR == 5'h13) ? PrefetchCounter :
                    (RC_ADR == 5'h14) ? WriteCounter :
                    (RC_ADR == 5'h15) ? WriteHitCounter :
                    (RC_ADR == 5'h16) ? L2ReadsCounter :
                    (RC_ADR == 5'h17) ? L2RespsCounter :
                    (RC_ADR == 5'h18) ? L2WritesCounter : 0
                    ) : 0;
//====================================================================================================================================================================>>>>
//=================================================================================================================================================================AHB<<<<
always @(posedge CLK or negedge RST)
  if(!RST) begin
    ExtAddr_r     <= 0;
    ExtWData_r    <= 0;
    ExtWrite_r    <= 0;
    ExtRead_r     <= 0;
    ExtAcsData_r  <= 0;
    ExtAcsData_2r <= 0;
    ExtAcsRegs_r  <= 0;
    ExtAcsTag_r   <= 0;
    ExtAcsValid_r <= 0;
    ExtAcsWay_r   <= 0;
  end
  else begin
    ExtAddr_r     <= ExtAddr;
    ExtWData_r    <= ExtWData;
    ExtWrite_r    <= ExtWrite;
    ExtRead_r     <= ExtRead;
    ExtAcsData_r  <= ExtAcsData;
    ExtAcsData_2r <= ExtAcsData_r;
    ExtAcsRegs_r  <= ExtAcsRegs;
    ExtAcsTag_r   <= ExtAcsTag;
    ExtAcsValid_r <= ExtAcsValid;
    ExtAcsWay_r   <= ExtAcsWay;
  end

assign ExtRData  = {PW_ExtData{ExtAcsData_2r                          }} & QB[ExtReadBankPointer] |
                   {PW_ExtData{(ExtAcsTag_r|ExtAcsValid_r|ExtAcsWay_r)}} & TagmemData |
                   {PW_ExtData{ExtAcsRegs_r                           }} & HRReg;

assign HRReg      = (ExtAddr_r[9:2] == 8'h0) ? {{PW_ExtData-2{1'b0}},PrefetchEn,L1En} :
                    (ExtAddr_r[9:2] == 8'h1) ? ReadCounter :
                    (ExtAddr_r[9:2] == 8'h2) ? ReadMissCounter :
                    (ExtAddr_r[9:2] == 8'h3) ? PrefetchCounter :
                    (ExtAddr_r[9:2] == 8'h4) ? WriteCounter :
                    (ExtAddr_r[9:2] == 8'h5) ? WriteHitCounter :
                    (ExtAddr_r[9:2] == 8'h6) ? L2ReadsCounter :
                    (ExtAddr_r[9:2] == 8'h7) ? L2RespsCounter :
                    (ExtAddr_r[9:2] == 8'h8) ? L2WritesCounter : 0;

assign ExtAddrBlock   = ExtAddr_r[L_ExtBlockEndBit:L_ExtBlockStartBit];
assign ExtAddrLine    = ExtAddr_r[L_ExtLineEndBit :L_ExtLineStartBit ];

assign ExtCollisions  = ExtAcsTag_r | ExtAcsValid_r | ExtAcsWay_r | ExtAcsData_r | ExtAcsRegs_r;// Merge together and block all activities by it.
//====================================================================================================================================================================>>>>
//==================================================================================================================================================Input Ports unpack<<<<
for (G_Port=0;G_Port<PN_Ports;G_Port=G_Port+1) begin:InputPortsUnpack_Genblock
  assign SrcAAddr [G_Port] = `SLICE(AADDR ,PW_Address  ,G_Port);
  assign SrcAWData[G_Port] = `SLICE(AWDATA,PW_Data     ,G_Port);
  assign SrcAWSTRB[G_Port] = `SLICE(AWSTRB,PN_LineBytes,G_Port);
end
//====================================================================================================================================================================>>>>
//============================================================================================================================================Input Address buses save<<<<
for (G_Port=0;G_Port<PN_Ports;G_Port=G_Port+1) begin:InputRegPerPorts_Genblock
  for (G_Slot=0;G_Slot<PN_Ports;G_Slot=G_Slot+1) begin:InputRegPerPorts_Genblock
    //TODO Could be heavy pat for synthesis. If so make it after receiving, by modulating SrcAValid_r for last received.
    assign SameAddrDifDirection[G_Port][G_Slot] = (SrcAAddr [G_Port] == SrcAAddr_r[G_Slot]) & (AWRITE[G_Port] & SrcAWrite_r[G_Slot] ? (|(SrcAWSTRB[G_Port] & SrcAWSTRB_r[G_Slot])) : (AWRITE[G_Port] ^ SrcAWrite_r[G_Slot])) & SrcAValid_r[G_Slot] & (G_Port != G_Slot);
  end
  assign SrcASent [G_Port] = (ReadPointer==G_Port) & (CurSendToL2 |  CurSendToDst) | (NextReadPointer==G_Port) & NextSendToDst;
  assign AREADY   [G_Port] = ~(|SameAddrDifDirection[G_Port]) & (~SrcAValid_r[G_Port] | SrcASent[G_Port]);

  always @(posedge CLK or negedge RST)
  if (!RST) begin
    SrcAValid_r[G_Port] <= 0;
    SrcAWrite_r[G_Port] <= 0;
    SrcAAddr_r [G_Port] <= 0;
    SrcAWData_r[G_Port] <= 0;
    SrcAWSTRB_r[G_Port] <= 0;
  end
  else begin
    if (AREADY       [G_Port]) begin
      if (AVALID     [G_Port]) begin
          SrcAValid_r[G_Port] <= 1'b1;
          SrcAWrite_r[G_Port] <= AWRITE   [G_Port];
          SrcAAddr_r [G_Port] <= SrcAAddr [G_Port];
        if (AWRITE   [G_Port]) begin
          SrcAWData_r[G_Port] <= SrcAWData[G_Port];
          SrcAWSTRB_r[G_Port] <= SrcAWSTRB[G_Port];
        end
      end
      else
        SrcAValid_r  [G_Port] <= 1'b0;
    end
    else
      if (SrcASent   [G_Port])
        SrcAValid_r  [G_Port] <= 1'b0;
  end
end
//====================================================================================================================================================================>>>>
//==============================================================================================================================================================TagMem<<<<
assign L2INV_RESP = InvValid_r;
always @(posedge CLK or negedge RST)
if (!RST) begin
  InvValid_r <= 0;
  InvAddr_r  <= 0;
end
else begin
  if (L1En) begin
    InvValid_r <= L2INV_VAL;
    if (L2INV_VAL)
      InvAddr_r  <= L2INV_ADDR;
  end
  else
    InvValid_r <= 1'b1;
end

for (G_Port=0;G_Port<PN_Ports;G_Port=G_Port+1) begin:TagMemAddrPack_Genblock
  assign `SLICE(SrcAAddr_rP,PW_Address,G_Port) = SrcAAddr_r[G_Port];
end
assign  CurSrcAWaysGroup = `SLICE(SrcAWaysGroup,LW_WaysGroups,0);
assign NextSrcAWaysGroup = `SLICE(SrcAWaysGroup,LW_WaysGroups,1);
assign  CurSrcAWay       = `SLICE(SrcAWay      ,LW_Ways      ,0);
assign NextSrcAWay       = `SLICE(SrcAWay      ,LW_Ways      ,1);
l1dc_tagmem
#(
  .PN_HitPorts(PN_Pointers),
  .PN_WaysGroups(PN_WaysGroups),
  .PN_Ways(PN_Ways),
  .PN_Lines(PN_Lines),
  .PW_FullAddr(PW_FullAddr),
  .PW_Address(PW_Address),
  .P_WaysGroupBit(L_WaysGroupBit),
  .PW_EvictCnt(PW_EvictCnt)
)
Utag (
.CLK(CLK), .RST(RST),
.INV_Val(InvValid_r), .INV_Addr_i(InvAddr_r), .InvAll(InvAll | ExtCtrlAcs & ExtWData_r[0] & ~L1En/*If L1 turned on force invalidate*/),
.ExtAcsTag(ExtAcsTag_r), .ExtAcsValid(ExtAcsValid_r), .ExtAcsWay(ExtAcsWay_r), .ExtWrite(ExtWrite_r), .ExtAddr(ExtAddr_r[9:2]),  .ExtWData(ExtWData_r), .ExtRData(TagmemData),
.AValid_i({NextSrcAValid,CurSrcAValid}), .Addr_i({NextSrcAAddr,CurSrcAAddr}), .PrefetchValid(CurPrefetchValid), .PrefetchAddr(CurPrefetchAAddr),
.HitPerPort({NextHit,CurHit}), .PrefetchHit_oc(CurPrefetchHit), .HitWayPerPort(SrcAWay),.HitWaysGroupPerPort(SrcAWaysGroup), .Evict_c(SrcAEvictCnt),
.RAddr(L2RAddr),.RValid(L2RValidTrans & L1En)
);
//====================================================================================================================================================================>>>>
//====================================================================================================================================================MissHit decision<<<<
assign BlockL2Access        = ~WFIFOFull & ~ExtCollisions;

assign ReadMissToL2         = CurSrcAValid & ~CurSrcAWrite & (~CurHit & ~SrcAddrAlreadySent | ~L1En) & ~RFIFOFull & BlockL2Access;
assign WriteToL2            = CurSrcAValid &  CurSrcAWrite & ( CurHit | ~SrcAddrAlreadySent | ~L1En)              & BlockL2Access;//NOTE If there is Write check for same address in read queue and only write new data if address is unique.
assign CurSendToL2          = ReadMissToL2 |  WriteToL2;
assign CurWriteUpponHit     = L1En & WriteToL2 & CurHit;

assign CurPrefetchNext      = L1En & PrefetchEn & CurPrefetchValid & ~CurPrefetchHit & ~RFIFOFull & BlockL2Access & ~CurSendToL2 & ~PrefetchAddrAlreadySent;

assign CurSendToDst         = L1En &  CurSrcAValid &  CurHit & ~ CurSrcAWrite &  CurORegEmptyAfterHit & ~SendDataAfterMiss[    ReadPointer]  & ~    PortBlockedByL2Access & ~ExtCollisions;//ExtReadCollision
assign NextSendToDst        = L1En & NextSrcAValid & NextHit & ~NextSrcAWrite & NextORegEmptyAfterHit & ~SendDataAfterMiss[NextReadPointer]  & ~NextPortBlockedByL2Access & ~ExtCollisions & (~CurWriteUpponHit | CurSrcAAddr != NextSrcAAddr) & (~CurSendToDst | (CurSrcAWaysGroup != NextSrcAWaysGroup));//TODO Make this absolutely independant. This NextReadPointer could also be used for L2 write when ReadPointer is being used for SendToDst.

assign CurSrcAValid         = SrcAValid_r     [    ReadPointer];
assign CurSrcAWrite         = SrcAWrite_r     [    ReadPointer];
assign CurSrcAAddr          = SrcAAddr_r      [    ReadPointer];
assign CurSrcAWData         = SrcAWData_r     [    ReadPointer];
assign CurSrcAWSTRB         = SrcAWSTRB_r     [    ReadPointer];
assign CurORegEmptyAfterHit =ORegEmptyAfterHit[    ReadPointer];

assign NextSrcAValid        = SrcAValid_r     [NextReadPointer];
assign NextSrcAWrite        = SrcAWrite_r     [NextReadPointer];
assign NextSrcAAddr         = SrcAAddr_r      [NextReadPointer];
assign NextORegEmptyAfterHit=ORegEmptyAfterHit[NextReadPointer];

assign CurPrefetchAAddr     = PrefetchAddr    [PrefetchPointer];
assign CurPrefetchValid     = PrefetchValid   [PrefetchPointer];

assign SrcAValid_mod = ({{(PN_Ports-1){1'b1}},1'b0} << ReadPointer) & SrcAValid_r;

MSB #(.PW_Data(PN_Ports)) NextValidPort (.LSB_i(1'b1), .Data_i(SrcAValid_mod),.Number_o(NextValid),.ValidMSB_o(NextValidEnabled));
MSB #(.PW_Data(PN_Ports)) PrevValidPort (.LSB_i(1'b1), .Data_i(SrcAValid_r  ),.Number_o(PrevValid),.ValidMSB_o(PrevValidEnabled));

assign NextReadPointer = NextValidEnabled ? NextValid : PrevValid;

assign ChangeReadPointer = CurSrcAValid ?
                            (
                              CurSendToL2 |
                              CurSendToDst & ~NextSendToDst |
                              ~CurORegEmptyAfterHit |
                              SrcAddrAlreadySent |
                              PortBlockedByL2Access
                            ) :
                            (|SrcAValid_r);

always @(posedge CLK or negedge RST) if (!RST) ReadPointer <= 0; else  if (ChangeReadPointer) ReadPointer <= NextReadPointer;

assign PrefetchValid_mod = ({{(PN_Ports-1){1'b1}},1'b0} << PrefetchPointer) & PrefetchValid;

MSB #(.PW_Data(PN_Ports)) NextPrefetchValidPort (.LSB_i(1'b1), .Data_i(PrefetchValid_mod),.Number_o(NextPrefetchValid),.ValidMSB_o(NextPrefetchValidEnabled));
MSB #(.PW_Data(PN_Ports)) PrevPrefetchValidPort (.LSB_i(1'b1), .Data_i(PrefetchValid    ),.Number_o(PrevPrefetchValid),.ValidMSB_o(PrevPrefetchValidEnabled));

assign NextPrefetchPointer = NextPrefetchValidEnabled ? NextPrefetchValid : PrevPrefetchValid;

assign ResetPrefetchValid = CurPrefetchNext | CurPrefetchHit | PrefetchAddrAlreadySent;
assign ChangePrefetchPointer = CurPrefetchValid ? ResetPrefetchValid : (|PrefetchValid);

always @(posedge CLK or negedge RST) if (!RST) PrefetchPointer <= 0; else  if (ChangePrefetchPointer) PrefetchPointer <= NextPrefetchPointer;

for (G_Port=0;G_Port<PN_Ports;G_Port=G_Port+1) begin:Prefetch_PortGenblock
  always @(posedge CLK or negedge RST)
    if (!RST) begin
      PrefetchValid [G_Port]    <= 0;
      PrefetchAddr  [G_Port]    <= 0;
    end
    else begin
      if (PrefetchEn & L1En) begin
        if ((CurSendToL2 & ~CurSrcAWrite | CurSendToDst) & (G_Port == ReadPointer) | NextSendToDst & (G_Port == NextReadPointer)) begin
          PrefetchValid [G_Port]  <= 1;
          PrefetchAddr  [G_Port]  <= ((CurSendToL2 & ~CurSrcAWrite | CurSendToDst) ? CurSrcAAddr : NextSrcAAddr) + {{(PW_Address-1){1'b0}},1'b1};
        end
        else
          if (ResetPrefetchValid & (G_Port == PrefetchPointer))
            PrefetchValid[G_Port] <= 0;
      end
      else
        PrefetchValid[G_Port] <= 0;
    end
end
//====================================================================================================================================================================>>>>
//=================================================================================================================================================Internal debug regs<<<<
assign ExtCtrlAcs = ExtAcsRegs_r & ExtWrite_r & (ExtAddr_r[9:2] == 8'h00) | (RC_GRP == 5'hB) & (RC_ADR == 5'h10) & WR_RC;
assign Ext_L1En       = ((RC_GRP == 5'hB) & (RC_ADR == 5'h10) & WR_RC) ? GDBW[0] : ExtWData_r[0];
assign Ext_PrefetchEn = ((RC_GRP == 5'hB) & (RC_ADR == 5'h10) & WR_RC) ? GDBW[1] : ExtWData_r[1];
always @(posedge CLK or negedge RST)
  if (!RST) begin
    L1En            <= 1'b1;
    PrefetchEn      <= 1'b1;
    ReadCounter     <= 0;
    ReadMissCounter <= 0;
    PrefetchCounter <= 0;
    WriteCounter    <= 0;
    WriteHitCounter <= 0;
    L2ReadsCounter  <= 0;
    L2RespsCounter  <= 0;
    L2WritesCounter <= 0;
  end
  else begin
    if (ExtCtrlAcs) begin
      L1En            <= Ext_L1En;
      PrefetchEn      <= Ext_PrefetchEn;
      ReadCounter     <= 0;
      ReadMissCounter <= 0;
      PrefetchCounter <= 0;
      WriteCounter    <= 0;
      WriteHitCounter <= 0;
      L2ReadsCounter  <= 0;
      L2RespsCounter  <= 0;
      L2WritesCounter <= 0;
    end
    else if (L1En) begin
      if      ((~CurSrcAWrite & CurSendToL2 | CurSendToDst) & NextSendToDst) ReadCounter     <= ReadCounter     + 2;
      else if ((~CurSrcAWrite & CurSendToL2 | CurSendToDst) | NextSendToDst) ReadCounter     <= ReadCounter     + 1;
      if      (ReadMissToL2    )                                             ReadMissCounter <= ReadMissCounter + 1;
      if      (CurPrefetchNext )                                             PrefetchCounter <= PrefetchCounter + 1;
      if      (WriteToL2       )                                             WriteCounter    <= WriteCounter    + 1;
      if      (CurWriteUpponHit)                                             WriteHitCounter <= WriteHitCounter + 1;

      if ( L2AValidTrans & ~L2AWRITE)                                        L2ReadsCounter  <= L2ReadsCounter  + 1;
      if ( L2RValidTrans)                                                    L2RespsCounter  <= L2RespsCounter  + 1;
      if ( L2AValidTrans &  L2AWRITE)                                        L2WritesCounter <= L2WritesCounter + 1;
    end
  end
//====================================================================================================================================================================>>>>
//======================================================================================================================================================L2 Access FIFO<<<<
assign L2AVALID       = ~WFIFOEmpty & ~RFIFOFull;
assign L2AValidTrans  =  L2AVALID   &  L2AREADY;

assign L2RREADY       = (ORegEmptyAfterMiss[L2RecPointer] | PrefetchFlag) & ~RFIFOEmpty & ~ExtCollisions;
assign L2RValidTrans  =  L2RVALID   &  L2RREADY;

//Old: Need to check writes also because could be like this:
// Read(miss), write into same line but different bytes and then read same line. Third read must read valid line.
//NOTE This is completely covered by checking only Read in queue, so don't check writes
assign      SrcAddrAlreadySent = (|        AddrAlreadySent1) | (|        AddrAlreadySent2);
assign PrefetchAddrAlreadySent = (|PrefetchAddrAlreadySent1) | (|PrefetchAddrAlreadySent2);

//NOTE Need to check if for this port pointed by ReadPointer we already sent request to L2. We cannot answer on this port by hit.
assign     PortBlockedByL2Access = (|    PortBlockedByL2Access1) | (|    PortBlockedByL2Access2);
assign NextPortBlockedByL2Access = (|NextPortBlockedByL2Access1) | (|NextPortBlockedByL2Access2);

assign L2WFIFOWrite = CurSendToL2 | CurPrefetchNext;

FIFO #(.W_WRITE(PN_LineBytes + PW_Data),.C_NUMBERWORDS(PN_L2WriteQueue))
L2WriteQueueFIFO1 (
  .sClk_i  (CLK           ), .snRst_i (RST         ),
  .Read_i  (L2AValidTrans ), .Write_i (L2WFIFOWrite),
  .Empty_oc(WFIFOEmpty    ), .Full_oc (WFIFOFull   ),

  .WriteData_32i(CurSrcAWrite ? {CurSrcAWSTRB,CurSrcAWData} : {PN_LineBytes+PW_Data{1'b0}}),
  .ReadData_32oc(               {    L2AWSTRB,    L2AWDATA}                               )
);

FIFO_compare2 #(.W_WRITE(1+PW_Address+1),.C_NUMBERWORDS(PN_L2WriteQueue), .W_COMPARE(1+PW_Address+1), .P_COMPSBIT(0))
L2WriteQueueFIFO2 (
  .sClk_i  (CLK           ), .snRst_i (RST         ),
  .Read_i  (L2AValidTrans ), .Write_i (L2WFIFOWrite),
  .Empty_oc(              ), .Full_oc (            ),

  .CompareEn (L1En           ), .CompareData_32i ({1'b0,CurSrcAAddr     ,1'b1}), .CompareResult_oc (        AddrAlreadySent1),
  .Compare2En(L1En&PrefetchEn), .Compare2Data_32i({1'b0,CurPrefetchAAddr,1'b1}), .Compare2Result_oc(PrefetchAddrAlreadySent1),

  .WriteData_32i({CurSrcAWrite,CurPrefetchNext ? CurPrefetchAAddr :CurSrcAAddr,CurPrefetchNext ? 1'b1 :CurSrcAValid}),
  .ReadData_32oc({    L2AWRITE,                                        L2AADDR,               AddrAlreadySentAValid})
);

FIFO_compare2 #(.W_WRITE(1+LW_Ports+2),.C_NUMBERWORDS(PN_L2WriteQueue), .W_COMPARE(LW_Ports+2), .P_COMPSBIT(0))
L2WriteQueueFIFO3 (
  .sClk_i  (CLK           ), .snRst_i (RST         ),
  .Read_i  (L2AValidTrans ), .Write_i (L2WFIFOWrite),
  .Empty_oc(              ), .Full_oc (            ),

  .CompareEn (L1En), .CompareData_32i ({    ReadPointer,2'b01}), .CompareResult_oc (    PortBlockedByL2Access1),
  .Compare2En(L1En), .Compare2Data_32i({NextReadPointer,2'b01}), .Compare2Result_oc(NextPortBlockedByL2Access1),

  .WriteData_32i({CurPrefetchNext,     ReadPointer,               CurSrcAWrite,               CurSrcAValid}),
  .ReadData_32oc({PrefetchFlagL2A, SentReadPointer,PortBlockedByL2AccessAWrite,PortBlockedByL2AccessAValid})
);

assign L2RFIFOWrite = L2AValidTrans & ~L2AWRITE; // NOTE Do not put in queue if we write to L2

FIFO_compare2 #(.W_WRITE(PW_Address + 1),.C_NUMBERWORDS(PN_L2ReadQueue), .W_COMPARE(PW_Address+1), .P_COMPSBIT(0))
L2ReadQueueFIFO1 (
  .sClk_i  (CLK          ), .snRst_i (RST         ),
  .Read_i  (L2RValidTrans), .Write_i (L2RFIFOWrite),
  .Empty_oc(RFIFOEmpty   ), .Full_oc (RFIFOFull   ),

  .CompareEn (L1En           ), .CompareData_32i ({CurSrcAAddr     ,1'b1}), .CompareResult_oc (        AddrAlreadySent2),
  .Compare2En(L1En&PrefetchEn), .Compare2Data_32i({CurPrefetchAAddr,1'b1}), .Compare2Result_oc(PrefetchAddrAlreadySent2),

  .WriteData_32i({L2AADDR, AddrAlreadySentAValid}),
  .ReadData_32oc({L2RAddr, AddrAlreadySentRValid})
);

FIFO_compare2 #(.W_WRITE(1+LW_Ports+1),.C_NUMBERWORDS(PN_L2ReadQueue), .W_COMPARE(LW_Ports+1), .P_COMPSBIT(0))
L2ReadQueueFIFO2 (
  .sClk_i  (CLK          ), .snRst_i (RST         ),
  .Read_i  (L2RValidTrans), .Write_i (L2RFIFOWrite),
  .Empty_oc(             ), .Full_oc (            ),

  .CompareEn (L1En),  .CompareData_32i({    ReadPointer,1'b1}), .CompareResult_oc (    PortBlockedByL2Access2),
  .Compare2En(L1En), .Compare2Data_32i({NextReadPointer,1'b1}), .Compare2Result_oc(NextPortBlockedByL2Access2),

  .WriteData_32i({PrefetchFlagL2A,SentReadPointer,PortBlockedByL2AccessAValid}),
  .ReadData_32oc({PrefetchFlag   ,   L2RecPointer,PortBlockedByL2AccessRValid})
);
//====================================================================================================================================================================>>>>
//=========================================================================================================================================================Data memory<<<<
generate
  for (G_Block=0;G_Block<PN_DataBanks;G_Block=G_Block+1) begin:DataMemBlocks_Genblock
    assign ExtReadEn  [G_Block] = ExtAcsData_r & ExtRead_r && (ExtAddrBlock == G_Block);
    assign ExtReadAddr[G_Block] = ExtAddrLine;
    assign HitReadEn  [G_Block] = CurSendToDst & (G_Block[LW_DataBanks-1] ? CurSrcAWaysGroup : ~CurSrcAWaysGroup) | NextSendToDst & (G_Block[LW_DataBanks-1] ? NextSrcAWaysGroup : ~NextSrcAWaysGroup);
    assign HitReadAddr[G_Block] = CurSendToDst & (G_Block[LW_DataBanks-1] ? CurSrcAWaysGroup : ~CurSrcAWaysGroup) ? { CurSrcAWay, CurSrcAAddr[L_LineEndBit:L_LineStartBit]} :
                                                                                                                    {NextSrcAWay,NextSrcAAddr[L_LineEndBit:L_LineStartBit]};

    assign AB [G_Block] = ExtReadEn[G_Block] ? ExtReadAddr[G_Block]: HitReadEn[G_Block] ? HitReadAddr[G_Block] : 0;

    assign ExtWriteEn       [G_Block] = ExtAcsData_r & ExtWrite_r && (ExtAddrBlock == G_Block);
    assign ExtWriteAddr     [G_Block] = ExtAddrLine;
    assign L2WriteEn        [G_Block] = L1En & L2RValidTrans & (G_Block[LW_DataBanks-1] ? L2RAddr[L_WaysGroupBit] : !L2RAddr[L_WaysGroupBit]);
    assign L2WriteAddr      [G_Block] = {`SLICE(SrcAEvictCnt,PW_EvictCnt,G_Block[LW_DataBanks-1]),L2RAddr[L_LineEndBit:L_LineStartBit]};
    assign WriteUpponHitAddr[G_Block] = { CurSrcAWay, CurSrcAAddr[L_LineEndBit:L_LineStartBit]};// WriteUpponHit could be only from Cur pointer.

    assign AA [G_Block] = ExtWriteEn[G_Block] ? ExtWriteAddr[G_Block] : L2WriteEn[G_Block]  ? L2WriteAddr[G_Block] : WriteUpponHitEn  [G_Block] ? WriteUpponHitAddr[G_Block] : 0;

    assign ExtWriteData     [G_Block] = ExtWData_r;
    assign L2WriteData      [G_Block] = `SLICE(L2RDATA,PW_ExtData,G_Block[3:0]);
    assign WriteUpponHitEn  [G_Block] =  CurWriteUpponHit & (G_Block[LW_DataBanks-1] ? CurSrcAWaysGroup : ~CurSrcAWaysGroup);
    assign WriteUpponHitData[G_Block] =  `SLICE(CurSrcAWData,PW_DataBanks,G_Block[3:0]);

    assign DA [G_Block] = ExtWriteEn[G_Block] ? ExtWriteData[G_Block] : L2WriteEn[G_Block] ? L2WriteData[G_Block] : WriteUpponHitEn[G_Block] ? WriteUpponHitData[G_Block] : 0;

    assign WriteUpponHitSTRB[G_Block] = `SLICE(CurSrcAWSTRB,LW_BanksMask,G_Block[3:0]);

    assign WEM[G_Block] = (ExtWriteEn[G_Block] | L2WriteEn[G_Block]) ? {LW_BanksMask{1'b1}}: WriteUpponHitEn[G_Block] ?  WriteUpponHitSTRB[G_Block] : 0;

    assign WEAN[G_Block] = ~(L2WriteEn[G_Block] | WriteUpponHitEn[G_Block] | ExtWriteEn[G_Block]);
    assign CEBN[G_Block] = ~(ExtReadEn[G_Block] | HitReadEn[G_Block]);
    assign CEAN[G_Block] = WEAN[G_Block];

    rf2_128x32 L1dc (.CEBN(CEBN[G_Block]), .AB(AB[G_Block]), .QB(QB[G_Block]), .CEAN(CEAN[G_Block]), .AA(AA[G_Block]), .DA(DA[G_Block]), .WEM(WEM[G_Block]), .WEAN(WEAN[G_Block]), .CLK(CLK));

    assign `SLICE(QBPerWayGroup[G_Block[LW_DataBanks-1]],PW_DataBanks,G_Block[3:0]) = QB[G_Block];
  end
endgenerate
always @(posedge CLK or negedge RST)
  if (!RST) begin
    DataMemoryAccess          <= 0;
    DataMemoryWaysGroup       <= 0;
    NextDataMemoryAccess      <= 0;
    NextDataMemoryWaysGroup   <= 0;
    ExtReadBankPointer        <= 0;
    DstPointer                <= 0;
    NextDstPointer            <= 0;
//     ClearDstPointer         <= 0;
  end
  else begin
    if (CurORegEmptyAfterHit) begin
      DstPointer              <= ReadPointer;
      DataMemoryAccess        <= CurSendToDst;
      DataMemoryWaysGroup     <= CurSrcAWaysGroup;
    end
    if (NextORegEmptyAfterHit) begin
      NextDstPointer          <= NextReadPointer;
      NextDataMemoryAccess    <= NextSendToDst;
      NextDataMemoryWaysGroup <= NextSrcAWaysGroup;
    end
      if (ExtAcsData_r & ExtRead_r)
        ExtReadBankPointer    <= ExtAddr[L_ExtLineEndBit:L_ExtLineStartBit];
//       ClearDstPointer       <= DstPointer;
  end
//====================================================================================================================================================================>>>>
//============================================================================================================================================Send Data to destination<<<<
for (G_Port=0;G_Port<PN_Ports;G_Port=G_Port+1) begin:OutputRegsPerPort_Genblock
  assign RVALID[G_Port] = DstRValid_r[G_Port];
  assign  `SLICE(RDATA,PW_Data,G_Port) = DstRData_r[G_Port];

  assign ORegValidTrans       [G_Port] = RREADY[G_Port] & RVALID[G_Port];

  assign SendDataAfterHit     [G_Port] =     DataMemoryAccess & (G_Port == DstPointer);
  assign SendNextDataAfterHit [G_Port] = NextDataMemoryAccess & (G_Port == NextDstPointer);

  assign SendDataAfterMiss    [G_Port] = L2RValidTrans & (G_Port == L2RecPointer) & ~PrefetchFlag;

  assign ORegEmptyAfterHit    [G_Port] = ~DstRValid_r[G_Port] | ORegValidTrans[G_Port];
  assign ORegEmptyAfterMiss   [G_Port] = ~DstRValid_r[G_Port];//NOTE Cannot make output L2RREADY depend on input RREADY[G_Port].

  assign ClearORegValid       [G_Port] = ~DataMemoryAccess & (G_Port == DstPointer) | ~NextDataMemoryAccess & (G_Port == NextDstPointer) | ~L2RVALID & (G_Port == L2RecPointer);

  always @(posedge CLK or negedge RST)
  if (!RST) begin
    DstRValid_r[G_Port] <= 0;
    DstRData_r [G_Port] <= 0;
  end
  else begin
    if (ORegEmptyAfterHit[G_Port] & (SendDataAfterHit[G_Port] | SendNextDataAfterHit[G_Port])) begin
      DstRValid_r[G_Port] <= 1'b1;
      if (SendDataAfterHit[G_Port])
        DstRData_r [G_Port] <= QBPerWayGroup[    DataMemoryWaysGroup];
      else
        DstRData_r [G_Port] <= QBPerWayGroup[NextDataMemoryWaysGroup];
    end
    else
      if (ORegEmptyAfterMiss[G_Port] & SendDataAfterMiss[G_Port]) begin
        DstRValid_r[G_Port] <= 1'b1;
        DstRData_r [G_Port] <= L2RDATA;
      end
      else if (/*ClearORegValid[G_Port] | */ORegValidTrans[G_Port])
        DstRValid_r[G_Port] <= 1'b0;
  end
end
//====================================================================================================================================================================>>>>
endmodule

`ifdef SLICE
  `undef SLICE
`endif
//====================================================================================================================================================================>>>>
