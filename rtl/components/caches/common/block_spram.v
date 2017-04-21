`timescale 1ns / 1ps

// single-port block RAM 
// port 1: read/write
module block_spram(
  clk, 
  reset, 
  addr1, 
  re1, 
  we1, 
  data1r,
  data1w 
);

parameter ADDR_WIDTH = 4;
parameter DATA_WIDTH = 32;
parameter KEEP_DATAR = 0;

input clk;
input reset;
input [ADDR_WIDTH-1:0] addr1;
input we1;
input re1;
input [DATA_WIDTH-1:0] data1w;
output [DATA_WIDTH-1:0] data1r;

reg [DATA_WIDTH-1:0] ram [(1<<ADDR_WIDTH)-1:0];
reg [ADDR_WIDTH-1:0] r_addr1;
reg re1_delayed;

always @(posedge clk)
begin
  if(we1) 
    ram[addr1] <= data1w;
    
  if(reset) begin
    r_addr1 <= 0;
    re1_delayed <= 0;
  end else begin
    if(re1) r_addr1 <= addr1;
    re1_delayed <= re1;
  end
end

generate
    if(KEEP_DATAR)
        assign data1r = ram[r_addr1];
    else
        assign data1r = re1_delayed ? ram[r_addr1] : {DATA_WIDTH{1'bx}};
endgenerate

endmodule
