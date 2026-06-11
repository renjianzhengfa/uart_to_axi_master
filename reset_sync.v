`timescale 1ns / 1ps
`default_nettype none

module reset_sync (
    input  wire i_clk,
    input  wire i_rst_n,
    output wire o_rst
);

reg r_rst_ff0;
reg r_rst_ff1;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        r_rst_ff0 <= 1'b1;
        r_rst_ff1 <= 1'b1;
    end
    else begin
        r_rst_ff0 <= 1'b0;
        r_rst_ff1 <= r_rst_ff0;
    end
end

assign o_rst = r_rst_ff1;

endmodule

`default_nettype wire