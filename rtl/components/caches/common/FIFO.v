`ifndef FIFO_wk79qt619p9ykzQS
  `define FIFO_wk79qt619p9ykzQS

module FIFO
#(
  parameter W_WRITE         = 32,
  parameter C_NUMBERWORDS   = 128
)
(
  input  wire              sClk_i,   snRst_i,
  input  wire[W_WRITE-1:0] WriteData_32i,
  input  wire              Read_i,   Write_i,
  output wire              Empty_oc,  Full_oc,
  output wire[W_WRITE-1:0] ReadData_32oc
);


generate
if (C_NUMBERWORDS == 1) begin:SingleReg_Genblock

  reg  [W_WRITE-1:0] Fifo32x128_r;
  reg  Full_r;

    /* read enable */
  wire ReadEn_w   = Read_i & Full_r;
  /* write enable */
  wire WriteEn_w  = Write_i & ~Full_r;// We can write if we read at the same time. Leads to additional checks outside FIFO

  always @(posedge sClk_i) begin
    if(WriteEn_w)
      Fifo32x128_r <= WriteData_32i;
  end
  assign ReadData_32oc = Fifo32x128_r;
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

  /* wire & register declaration */
  reg  [W_WRITE-1:0] Fifo32x128_r [C_NUMBERWORDS-1:0];
  reg  [LW_ADDRESS-1:0] WAddr_7r, WAddrNext_7r;
  reg  [LW_ADDRESS-1:0] RAddr_7r, RAddrNext_7r;
  reg  Full_r, Empty_r;

  /* read enable */
  wire ReadEn_w   = Read_i & ~Empty_r;

  /* write enable */
  wire WriteEn_w  = Write_i & (~Full_r/* | Full_r & ReadEn_w*/);// We can write if we read at the same time. Leads to additional checks outside FIFO

  always @(posedge sClk_i) begin
    if(WriteEn_w)
      Fifo32x128_r[WAddr_7r] <= WriteData_32i;
  end

  assign ReadData_32oc = Empty_r ? {W_WRITE{1'b0}} : Fifo32x128_r[RAddr_7r];

  assign Empty_oc  = Empty_r;
  assign Full_oc   = Full_r;

  /* control logic */
  always @(posedge sClk_i or negedge snRst_i) begin
    if(!snRst_i) begin
      Full_r        <= 1'b0;
      Empty_r       <= 1'b1;
      WAddr_7r      <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},1'b1};
      RAddr_7r      <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},1'b1};
      WAddrNext_7r  <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},2'd2};
      RAddrNext_7r  <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},2'd2};
    end
    else begin

      case ({WriteEn_w, ReadEn_w})

        /* read */
        2'b01 : begin
          if(~|RAddrNext_7r)  RAddrNext_7r <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},1'b1};
          else                RAddrNext_7r <= RAddrNext_7r - {{LW_ADDRESS{1'b0}},1'b1};
          RAddr_7r  <= RAddrNext_7r;
          Full_r    <= 1'b0;
          if(RAddrNext_7r == WAddr_7r) Empty_r <= 1'b1;
        end

        /* write */
        2'b10 : begin
          if(~|WAddrNext_7r)  WAddrNext_7r <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},1'b1};
          else                WAddrNext_7r <= WAddrNext_7r - {{LW_ADDRESS{1'b0}},1'b1};
          WAddr_7r  <= WAddrNext_7r;
          Empty_r   <= 1'b0;
          if(WAddrNext_7r == RAddr_7r) Full_r <= 1'b1;
        end

        /* read & write */
        2'b11 : begin
          if(~|RAddrNext_7r)  RAddrNext_7r <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},1'b1};
          else                RAddrNext_7r <= RAddrNext_7r - {{LW_ADDRESS{1'b0}},1'b1};
          RAddr_7r <= RAddrNext_7r;
          if(~|WAddrNext_7r)  WAddrNext_7r <= C_NUMBERWORDS - {{LW_ADDRESS{1'b0}},1'b1};
          else                WAddrNext_7r <= WAddrNext_7r - {{LW_ADDRESS{1'b0}},1'b1};
          WAddr_7r <= WAddrNext_7r;
        end
        default : begin
          RAddrNext_7r <= RAddrNext_7r;
          WAddrNext_7r <= WAddrNext_7r;
          RAddr_7r     <= RAddr_7r;
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
