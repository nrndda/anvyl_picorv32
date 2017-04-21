// synopsys translate_off
`timescale 1ns / 10ps
// synopsys translate_on
`ifndef SLICE
  `define SLICE(vector, size, index) vector[((index)+'d1)*(size)-'d1 : (index)*(size)]
`endif
//===================================================================================================================================================Module defenition<<<<
module l1dc_tagmem
#(
  parameter PW_ExtData        = 32,
  parameter PN_HitPorts       = 2,//Hit ports
  parameter PN_WaysGroups     = 2,
  parameter PW_WaysGroups     = $clog2(PN_WaysGroups),
  parameter PN_Ways           = 4,
  parameter PW_Ways           = $clog2(PN_Ways),
  parameter PN_Lines          = 32,
  parameter PW_Lines          = $clog2(PN_Lines),
  parameter PW_FullAddr       = 32,
  parameter PW_Address        = PW_FullAddr-6,// NOTE Remove 6 bits from input address as they are for addressing bytes in line.
  parameter PW_EvictCnt       = PW_Ways,
  parameter P_WaysGroupBit    = 0,
  parameter P_LineStartBit    = P_WaysGroupBit+1,
  parameter P_LineEndBit      = P_LineStartBit + PW_Lines - 1,
  parameter P_TagStartBit     = P_LineEndBit+1,
  parameter P_TagEndBit       = PW_Address-1,
  parameter P_ExtAddrStartBit = 2,
  parameter P_ExtAddrEndBit   = 9
)
(
  input    CLK, RST,
  // Invalidate interface
  input                      InvAll,
  input                      INV_Val,
  input [PW_Address    -1:0] INV_Addr_i, /*Input address to invalidate*/
  // AHB
  input  ExtAcsTag, ExtAcsValid, ExtAcsWay, ExtWrite,
  input  [P_ExtAddrEndBit:P_ExtAddrStartBit] ExtAddr,
  input  [              PW_ExtData    -1:0]  ExtWData,
  output [              PW_ExtData    -1:0]  ExtRData,
  // L2 Response uppon miss
  input  [              PW_Address    -1:0]  RAddr,
  input                                      RValid,
  // L1 Cache control
  input  [PN_HitPorts                 -1:0]  AValid_i, /*Input address valid*/
  input  [PN_HitPorts  *PW_Address    -1:0]  Addr_i, /*Input address*/
  output [PN_HitPorts                 -1:0]  HitPerPort,
  output [PN_HitPorts  *PW_Ways       -1:0]  HitWayPerPort,
  output [PN_HitPorts  *PW_WaysGroups -1:0]  HitWaysGroupPerPort,
  input                                      PrefetchValid,
  input  [PW_Address                  -1:0]  PrefetchAddr,
  output                                     PrefetchHit_oc,
  output [PN_WaysGroups*PW_EvictCnt   -1:0]  Evict_c
);
//====================================================================================================================================================================>>>>
//===============================================================================================================================================Global wires and regs<<<<
localparam PW_Tag = P_TagEndBit - P_TagStartBit + 1;

wire [PW_ExtData-1:0] ExtTag, ExtVal, ExtWay;

reg  [PW_Tag               -1:0]   Tag_r                           [PN_WaysGroups-1:0][PN_Ways-1:0][PN_Lines-1:0];
reg  [PN_Lines             -1:0] Valid_r                           [PN_WaysGroups-1:0][PN_Ways-1:0];
reg  [PW_EvictCnt          -1:0] Evict_r                           [PN_WaysGroups-1:0]             [PN_Lines-1:0];

wire [PW_Address           -1:0] Addr            [PN_HitPorts-1:0];
wire [PW_Tag               -1:0] AddrTag         [PN_HitPorts-1:0];
wire [PW_Lines             -1:0] AddrLine        [PN_HitPorts-1:0];
wire [PN_WaysGroups        -1:0] AddrGroup       [PN_HitPorts-1:0];
wire [PW_Tag               -1:0] PrefetchTag;
wire [PW_Lines             -1:0] PrefetchLine;
wire [PN_WaysGroups        -1:0] PrefetchGroup;
wire [PN_Ways              -1:0] Hit             [PN_HitPorts-1:0] [PN_WaysGroups-1:0];
wire [PN_Ways              -1:0] PrefetchHit     [PN_WaysGroups-1:0];
wire [PN_WaysGroups        -1:0] HitPerGroup     [PN_HitPorts-1:0];
wire [PN_WaysGroups        -1:0] PrefetchHitPerGroup;

wire [PW_Ways              -1:0] HitWayWA        [PN_HitPorts-1:0] [PN_Ways*PN_WaysGroups-1:0];
wire [PN_Ways*PN_WaysGroups-1:0] HitWayWA1       [PN_HitPorts-1:0] [PW_Ways              -1:0];
wire [PW_Ways              -1:0] HitWayWA2       [PN_HitPorts-1:0];
wire [PW_WaysGroups        -1:0] HitWaysGroupWA  [PN_HitPorts-1:0] [PN_Ways*PN_WaysGroups-1:0];
wire [PN_Ways*PN_WaysGroups-1:0] HitWaysGroupWA1 [PN_HitPorts-1:0] [PW_WaysGroups        -1:0];
wire [PW_WaysGroups        -1:0] HitWaysGroupWA2 [PN_HitPorts-1:0];

wire [PW_Tag               -1:0] RAddrTag;
wire [PW_Lines             -1:0] RAddrLine;
wire [PN_WaysGroups        -1:0] RAddrGroup;

wire [PW_Tag               -1:0] InvTag;
wire [PW_Lines             -1:0] InvLine;
wire [PN_WaysGroups        -1:0] InvGroup;
wire [PN_Ways              -1:0] InvHit                         [PN_WaysGroups-1:0];

genvar G_Port,G_Group,G_Way,G_Line,G_Bit;
//====================================================================================================================================================================>>>>
//=======================================================================================================================================================AHB interface<<<<
assign ExtRData  = {PW_ExtData{ExtAcsTag}} & ExtTag | {PW_ExtData{ExtAcsValid}} & ExtVal | {PW_ExtData{ExtAcsWay}} & ExtWay;

wire       ExtTagTagGroup = ExtAddr[9];
wire [1:0] ExtTagTagWay   = ExtAddr[8:7];
wire [4:0] ExtTagTagLine  = ExtAddr[6:2];
assign ExtTag = {Tag_r[ExtTagTagGroup][ExtTagTagWay][ExtTagTagLine],{(PW_FullAddr-PW_Tag){1'b0}}};

wire       ExtValGroup = ExtAddr[4];
wire [1:0] ExtValWay   = ExtAddr[3:2];
assign ExtVal = Valid_r[ExtValGroup][ExtValWay];

wire ExtWayGroup     = ExtAddr[3];
wire ExtWayGroupPart = ExtAddr[2];
generate
  for (G_Line=0;G_Line<(PN_Lines/PW_EvictCnt);G_Line=G_Line+1) begin:HalfLines_Genblock
    assign `SLICE(ExtWay,PW_EvictCnt,G_Line) = ExtWayGroupPart ? Evict_r[ExtWayGroup][(PN_Lines/PW_EvictCnt)+G_Line] : Evict_r[ExtWayGroup][G_Line];
  end
endgenerate
//====================================================================================================================================================================>>>>
//=======================================================================================================================================Address per input port decode<<<<
generate
  for (G_Port=0;G_Port<PN_HitPorts;G_Port=G_Port+1) begin:InputAddrDecode_Genblock
    assign Addr     [G_Port] = `SLICE(Addr_i,PW_Address,G_Port);
    assign AddrTag  [G_Port] = Addr[G_Port][ P_TagEndBit:P_TagStartBit];
    assign AddrLine [G_Port] = Addr[G_Port][P_LineEndBit:P_LineStartBit];
    assign AddrGroup[G_Port] = Addr[G_Port][P_WaysGroupBit];
  end

  assign PrefetchTag   = PrefetchAddr[ P_TagEndBit:P_TagStartBit];
  assign PrefetchLine  = PrefetchAddr[P_LineEndBit:P_LineStartBit];
  assign PrefetchGroup = PrefetchAddr[P_WaysGroupBit];
endgenerate
//====================================================================================================================================================================>>>>
//=======================================================================================================================================================HIT calculate<<<<
generate
  for (G_Port=0;G_Port<PN_HitPorts;G_Port=G_Port+1) begin:HITPorts_Genblock
    for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:HITWaysGroup_Genblock
      for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:HITWays_Genblock
        assign     Hit[G_Port][G_Group][G_Way] =        AValid_i[G_Port] & Valid_r[G_Group][G_Way][    AddrLine[G_Port]] & (    AddrTag[G_Port] == Tag_r[G_Group][G_Way][    AddrLine[G_Port]]) & (    AddrGroup[G_Port] == G_Group);
      end
    end
  end
  for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:PrefetchHITWaysGroup_Genblock
    for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:PrefetchHITWay_Genblock
      assign PrefetchHit[G_Group][G_Way] = PrefetchValid & Valid_r[G_Group][G_Way][PrefetchLine] & (PrefetchTag == Tag_r[G_Group][G_Way][PrefetchLine]) & (PrefetchGroup == G_Group);
    end
    assign PrefetchHitPerGroup[G_Group] = |PrefetchHit[G_Group];
  end
  assign PrefetchHit_oc =  |PrefetchHitPerGroup;
endgenerate
//====================================================================================================================================================================>>>>
//==================================================================================================================================================Cache Data control<<<<
generate
  for (G_Port=0;G_Port<PN_HitPorts;G_Port=G_Port+1) begin:CacheCtrlPorts_Genblock
    for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:CacheCtrlWaysGroup_Genblock
      assign            HitPerGroup [G_Port][G_Group] =         |Hit[G_Port][G_Group];
      for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:CacheCtrlWays_Genblock
        assign         HitWayWA [G_Port]       [G_Group*PN_Ways+G_Way] = Hit[G_Port][G_Group][G_Way] ? G_Way   : 0;
        for (G_Bit=0;G_Bit<PW_Ways;G_Bit=G_Bit+1) begin:CacheCtrlGroupBits_Genblock
          assign       HitWayWA1[G_Port][G_Bit][G_Group*PN_Ways+G_Way] = HitWayWA[G_Port][G_Group*PN_Ways+G_Way][G_Bit];
        end
        assign   HitWaysGroupWA [G_Port]       [G_Group*PN_Ways+G_Way] = Hit[G_Port][G_Group][G_Way] ? G_Group : 0;
        for (G_Bit=0;G_Bit<PW_WaysGroups;G_Bit=G_Bit+1) begin:CacheCtrlWayBits_Genblock
          assign HitWaysGroupWA1[G_Port][G_Bit][G_Group*PN_Ways+G_Way] = HitWaysGroupWA[G_Port][G_Group*PN_Ways+G_Way][G_Bit];
        end
      end
    end
    for (G_Bit=0;G_Bit<PW_Ways;G_Bit=G_Bit+1) begin:CacheCtrlGroupBits2_Genblock
      assign       HitWayWA2  [G_Port][G_Bit]                        = |HitWayWA1[G_Port][G_Bit];
    end
    for (G_Bit=0;G_Bit<PW_WaysGroups;G_Bit=G_Bit+1) begin:CacheCtrlWayBits2_Genblock
      assign HitWaysGroupWA2  [G_Port][G_Bit]                        = |HitWaysGroupWA1[G_Port][G_Bit];
    end
    assign                 HitPerPort              [G_Port] =          |HitPerGroup[G_Port];
    assign `SLICE(      HitWayPerPort,PW_Ways      ,G_Port) =       HitWayWA2  [G_Port];
    assign `SLICE(HitWaysGroupPerPort,PW_WaysGroups,G_Port) = HitWaysGroupWA2  [G_Port];
  end
endgenerate
//====================================================================================================================================================================>>>>
//=================================================================================================================================================Read address decode<<<<
assign RAddrTag   = RAddr[ P_TagEndBit:P_TagStartBit];
assign RAddrLine  = RAddr[P_LineEndBit:P_LineStartBit];
assign RAddrGroup = RAddr[P_WaysGroupBit];
//====================================================================================================================================================================>>>>
//==============================================================================================================================Invalidate address decode and response<<<<
assign InvTag    = INV_Addr_i[ P_TagEndBit:P_TagStartBit];
assign InvLine   = INV_Addr_i[P_LineEndBit:P_LineStartBit];
assign InvGroup  = INV_Addr_i[P_WaysGroupBit];
generate
  for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:InvWaysGroup_Genblock
    for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:InvWays_Genblock
      assign InvHit[G_Group][G_Way] = Valid_r[G_Group][G_Way][InvLine] & (InvTag == Tag_r[G_Group][G_Way][InvLine]) & (InvGroup == G_Group);
    end
  end
endgenerate
//====================================================================================================================================================================>>>>
//================================================================================================================================================Evict victim counter<<<<
generate
  for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:EvictWaysGroup_Genblock
    assign `SLICE(Evict_c,PW_EvictCnt,G_Group)  = Evict_r[G_Group][RAddrLine];
    for (G_Line=0;G_Line<PN_Lines;G_Line=G_Line+1) begin:EvictLines_Genblock
      always @(posedge CLK or negedge RST)
        if(!RST)                                                          Evict_r[G_Group][G_Line] <= {PW_EvictCnt{1'b0}};
        else if(ExtWrite & ExtAcsWay & (ExtAddr[3:2]==G_Line[4:3]))       Evict_r[G_Group][G_Line] <= ExtWData[PW_EvictCnt-1:0];
        else if(RValid & (RAddrGroup == G_Group) & (RAddrLine==G_Line))   Evict_r[G_Group][G_Line] <= Evict_r[G_Group][G_Line] + {{(PW_EvictCnt-1){1'b0}},1'b1};
    end
  end
endgenerate
//====================================================================================================================================================================>>>>
//=====================================================================================================================================================Valid registers<<<<
generate
  for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:ValidWaysGroup_Genblock
    for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:ValidWays_Genblock
      for (G_Line=0;G_Line<PN_Lines;G_Line=G_Line+1) begin:ValidLines_Genblock
        always @(posedge CLK or negedge RST)
          if(!RST)                                                                                                Valid_r[G_Group][G_Way][G_Line] <= 1'b0;
          else if(INV_Val & InvHit[G_Group][G_Way] & (InvLine == G_Line) | InvAll)                                Valid_r[G_Group][G_Way][G_Line] <= 1'b0;
          else if(ExtWrite & ExtAcsValid & ({{(32-3){1'b0}},ExtAddr[4:2]}==G_Group*PN_Ways+G_Way))                Valid_r[G_Group][G_Way][G_Line] <= ExtWData[G_Line];
          else if(RValid & (RAddrGroup == G_Group) & (Evict_r[G_Group][RAddrLine]==G_Way) & (RAddrLine == G_Line))Valid_r[G_Group][G_Way][G_Line] <= 1'b1;
      end
    end
  end
endgenerate
//====================================================================================================================================================================>>>>
//=======================================================================================================================================================Tag registers<<<<
generate
  for (G_Group=0;G_Group<PN_WaysGroups;G_Group=G_Group+1) begin:TagWaysGroup_Genblock
    for (G_Way=0;G_Way<PN_Ways;G_Way=G_Way+1) begin:TagWays_Genblock
      for (G_Line=0;G_Line<PN_Lines;G_Line=G_Line+1) begin:TagLines_Genblock
        always @(posedge CLK or negedge RST)
          if(!RST)                                                                                                    Tag_r[G_Group][G_Way][G_Line] <= 0;
          else if(ExtWrite & ExtAcsTag & ({{(32-8){1'b0}},ExtAddr[9:2]}==((G_Group*PN_Ways+G_Way)*PN_Lines+G_Line)))  Tag_r[G_Group][G_Way][G_Line] <= ExtWData[P_TagEndBit:P_TagStartBit];// NOTE Use Tag bits from real adress
          else if(RValid & (RAddrGroup == G_Group) & (Evict_r[G_Group][G_Line]==G_Way) & (RAddrLine==G_Line))         Tag_r[G_Group][G_Way][G_Line] <= RAddrTag;
      end
    end
  end
endgenerate
//====================================================================================================================================================================>>>>
endmodule

`ifdef SLICE
  `undef SLICE
`endif
