`ifndef FIFO_ver1_compare2_wk79qt619p9ykzQS
  `define FIFO_ver1_compare2_wk79qt619p9ykzQS

//===========================================================================================================================================================NOTE=TODO<<<<
//     This is direvative from FIFO.v but with shift instead of RAddr_7r and with registers on outputs.
//====================================================================================================================================================================>>>>

module FIFO_compare2
#(
  parameter W_WRITE         = 32,
  parameter W_COMPARE       = W_WRITE,
  parameter P_COMPSBIT      = 0,
  parameter P_COMPEBIT      = P_COMPSBIT + W_COMPARE-1,
  parameter C_NUMBERWORDS   = 128
)
(
  input  wire              sClk_i,   snRst_i,
  input  wire[W_WRITE-1:0] WriteData_32i,
  input  wire[W_COMPARE-1:0] CompareData_32i,
  input  wire[W_COMPARE-1:0] Compare2Data_32i,
  input  wire              CompareEn,
  input  wire              Compare2En,
  input  wire              Read_i,   Write_i,
  output wire              Empty_oc,  Full_oc,
  output wire[W_WRITE-1:0] ReadData_32oc,
  output wire[C_NUMBERWORDS-1:0] CompareResult_oc,
  output wire[C_NUMBERWORDS-1:0] Compare2Result_oc
);

genvar G_Slot;
generate
if (C_NUMBERWORDS == 1) begin:SingleReg_Genblock

  reg  [W_WRITE-1:0] Fifo32x128_r;
  reg  Full_r;

  assign CompareResult_oc  = CompareEn  ? (Fifo32x128_r[P_COMPEBIT:P_COMPSBIT] == CompareData_32i ) : 0;
  assign Compare2Result_oc = Compare2En ? (Fifo32x128_r[P_COMPEBIT:P_COMPSBIT] == Compare2Data_32i) : 0;

    /* read enable */
  wire ReadEn_w   = Read_i & Full_r;

  /* write enable */
  wire WriteEn_w  = Write_i & (~Full_r | ReadEn_w);

  always @(posedge sClk_i or negedge snRst_i)
    if (!snRst_i)
      Fifo32x128_r <= 0;
    else
      if(WriteEn_w)
        Fifo32x128_r <= WriteData_32i;
      else
        if (ReadEn_w)
          Fifo32x128_r <= 0;//NOTE For FIFO with compare feature we need to clear regs after read.

  assign ReadData_32oc = Empty_oc ? {W_WRITE{1'b0}} : Fifo32x128_r;
  assign Empty_oc  = ~Full_r;
  assign Full_oc   =  Full_r;
  /* control logic */
  always @(posedge sClk_i or negedge snRst_i) begin
    if(!snRst_i) begin
      Full_r        <= 1'b0;
    end
    else begin
      case ({WriteEn_w, ReadEn_w})
        /* read */
        2'b01   : begin Full_r <= 1'b0  ; end
        /* write */
        2'b10   : begin Full_r <= 1'b1  ; end
        default : begin Full_r <= Full_r; end
      endcase
    end
  end
end
else begin:NormalFIFO

  localparam LW_ADDRESS = $clog2(C_NUMBERWORDS);
  localparam [LW_ADDRESS-1:0] C_NUMBERWORDS1 = C_NUMBERWORDS - 1;

  /* wire & register declaration */
  reg  [W_WRITE-1:0] Fifo32x128_r [C_NUMBERWORDS-1:0];
  reg  [LW_ADDRESS-1:0] WAddr_7r, WAddrPrev_7r;
  reg  Full_r, Empty_r;

  for (G_Slot=0;G_Slot<C_NUMBERWORDS;G_Slot=G_Slot+1) begin:Compare_Genblock
    assign CompareResult_oc [G_Slot] = CompareEn  ? (Fifo32x128_r[G_Slot][P_COMPEBIT:P_COMPSBIT] == CompareData_32i ) : 0;
    assign Compare2Result_oc[G_Slot] = Compare2En ? (Fifo32x128_r[G_Slot][P_COMPEBIT:P_COMPSBIT] == Compare2Data_32i) : 0;
  end

  /* read enable */
  wire ReadEn_w   = Read_i & ~Empty_r;

  /* write enable */
  wire WriteEn_w  = Write_i & (~Full_r | ReadEn_w);

  for (G_Slot=0;G_Slot<C_NUMBERWORDS;G_Slot=G_Slot+1) begin:FIFO_Main_genblock
    if (G_Slot == C_NUMBERWORDS-1) begin:Edge_genblock
      //NOTE This slot can be written only from input, but not during shift.
      always @(posedge sClk_i or negedge snRst_i)
        if (!snRst_i)
          Fifo32x128_r[G_Slot] <= 0;
        else begin
          if(ReadEn_w & WriteEn_w) begin
            if (G_Slot == WAddrPrev_7r)
              Fifo32x128_r[G_Slot] <= WriteData_32i;
          end
          else
            if (WriteEn_w & (G_Slot == WAddr_7r))
              Fifo32x128_r[G_Slot] <= WriteData_32i;
            else
              if (ReadEn_w & (G_Slot == WAddr_7r))
                Fifo32x128_r[G_Slot] <= 0;//NOTE For FIFO with compare feature we need to clear regs after read.
        end
    end
    else begin
      always @(posedge sClk_i or negedge snRst_i)
        if (!snRst_i)
          Fifo32x128_r[G_Slot] <= 0;
        else begin
          if(ReadEn_w & WriteEn_w) begin
            if (G_Slot < WAddrPrev_7r)
              Fifo32x128_r[G_Slot] <= Fifo32x128_r[G_Slot+1];
            else
              if (G_Slot == WAddrPrev_7r)
                Fifo32x128_r[G_Slot] <= WriteData_32i;
          end
          else
            if (WriteEn_w & (G_Slot == WAddr_7r))
              Fifo32x128_r[G_Slot] <= WriteData_32i;
            else
              if (ReadEn_w) begin
                if (G_Slot < WAddrPrev_7r)
                  Fifo32x128_r[G_Slot] <= Fifo32x128_r[G_Slot+1];
                else
                  if (G_Slot == WAddrPrev_7r)
                    Fifo32x128_r[G_Slot] <= 0;//NOTE For FIFO with compare feature we need to clear regs after read.
              end
        end
    end
  end

  assign ReadData_32oc = Empty_r ? {W_WRITE{1'b0}} : Fifo32x128_r[0];

  assign Empty_oc  = Empty_r;
  assign Full_oc   = Full_r;

  /* control logic */
  always @(posedge sClk_i or negedge snRst_i) begin
    if(!snRst_i) begin
      Full_r        <= 1'b0;
      Empty_r       <= 1'b1;
      WAddrPrev_7r  <= C_NUMBERWORDS1;
      WAddr_7r      <= {{LW_ADDRESS{1'b0}},1'b0};
    end
    else begin

      case ({WriteEn_w, ReadEn_w})

        /* read */
        2'b01 : begin
          WAddr_7r    <= WAddrPrev_7r;
          WAddrPrev_7r<= WAddrPrev_7r - 1;
          Full_r      <= 1'b0;
          if(WAddrPrev_7r == {LW_ADDRESS{1'b0}}) Empty_r <= 1'b1;
        end

        /* write */
        2'b10 : begin
          WAddr_7r      <= WAddr_7r + 1;
          WAddrPrev_7r  <= WAddr_7r;
          Empty_r       <= 1'b0;
          if(WAddr_7r == C_NUMBERWORDS1) Full_r <= 1'b1;
        end
        default : begin
          WAddrPrev_7r <= WAddrPrev_7r;
          WAddr_7r     <= WAddr_7r;
          Empty_r      <= Empty_r;
          Full_r       <= Full_r;
        end

      endcase

    end
  end
end
endgenerate
endmodule
`endif
