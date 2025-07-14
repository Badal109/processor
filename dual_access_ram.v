module dp_ram (
  input clk,
  input rst,
  input enable,
  input wr,
  input rd,
  input [5:0] wr_addr,
  input [5:0] rd_addr,
  input [7:0] wr_data,
  output reg [7:0] rd_data
);

  reg [7:0] mem [0:63];
  integer i;

  always @(posedge clk) begin
    if (!rst) begin
      // synchronous reset
      for (i = 0; i < 64; i = i + 1)
        mem[i] <= 8'd0;
      rd_data <= 8'd0;
    end else if (enable) begin
      if (wr)
        mem[wr_addr] <= wr_data;
      if (rd)
        rd_data <= mem[rd_addr];
    end
  end

endmodule
