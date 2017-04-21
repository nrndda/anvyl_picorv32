`timescale 1ns / 1ps

// dual-port block RAM
// port 1: read only
// port 2: write only
module block_dpram(
  clk,
  reset,
  addr1,
  addr2,
  re1,
  we2,
  data1r,
  data2w
);

parameter ADDR_WIDTH = 4;
parameter DATA_WIDTH = 32;

input clk;
input reset;
input [ADDR_WIDTH-1:0] addr1;
input [ADDR_WIDTH-1:0] addr2;
input we2;
input re1;
input [DATA_WIDTH-1:0] data2w;
output [DATA_WIDTH-1:0] data1r;

reg [DATA_WIDTH-1:0] ram [(1<<ADDR_WIDTH)-1:0];
reg [ADDR_WIDTH-1:0] r_addr1;
reg re1_delayed;

always @(posedge clk)
begin
  if(we2)
    ram[addr2] <= data2w;

  if(reset) begin
    r_addr1 <= 0;
    re1_delayed <= 0;
  end else begin
    if(re1) r_addr1 <= addr1;
    re1_delayed <= re1;
  end
end

assign data1r = re1_delayed ? ram[r_addr1] : {DATA_WIDTH{1'bx}};

endmodule
