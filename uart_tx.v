`timescale 1ns / 1ps
`default_nettype none

module uart_tx #(
    parameter integer CLKS_PER_BIT = 434
) (
    input  wire      i_clk,
    input  wire      i_rst,
    input  wire      i_tx_valid,
    input  wire [7:0] i_tx_byte,
    output wire      o_tx_ready,
    output reg       o_tx_serial,
    output reg       o_tx_active
);

localparam [2:0] ST_IDLE    = 3'd0;
localparam [2:0] ST_START   = 3'd1;
localparam [2:0] ST_DATA    = 3'd2;
localparam [2:0] ST_STOP    = 3'd3;
localparam [2:0] ST_CLEANUP = 3'd4;

reg [2:0] r_state;
reg [15:0] r_clk_cnt;
reg [2:0] r_bit_idx;
reg [7:0] r_tx_shift;

assign o_tx_ready = (r_state == ST_IDLE);

always @(posedge i_clk) begin
    if (i_rst) begin
        r_state       <= ST_IDLE;
        r_clk_cnt     <= 16'd0;
        r_bit_idx     <= 3'd0;
        r_tx_shift    <= 8'd0;
        o_tx_serial <= 1'b1;
        o_tx_active <= 1'b0;
    end
    else begin
        case (r_state)
            ST_IDLE: begin
                o_tx_serial <= 1'b1;
                o_tx_active <= 1'b0;
                r_clk_cnt     <= 16'd0;
                r_bit_idx     <= 3'd0;

                if (i_tx_valid) begin
                    r_tx_shift    <= i_tx_byte;
                    o_tx_active <= 1'b1;
                    r_state       <= ST_START;
                end
            end

            ST_START: begin
                o_tx_serial <= 1'b0;

                if (r_clk_cnt == (CLKS_PER_BIT - 1)) begin
                    r_clk_cnt <= 16'd0;
                    r_state   <= ST_DATA;
                end
                else begin
                    r_clk_cnt <= r_clk_cnt + 16'd1;
                end
            end

            ST_DATA: begin
                o_tx_serial <= r_tx_shift[r_bit_idx];

                if (r_clk_cnt == (CLKS_PER_BIT - 1)) begin
                    r_clk_cnt <= 16'd0;

                    if (r_bit_idx == 3'd7) begin
                        r_bit_idx <= 3'd0;
                        r_state   <= ST_STOP;
                    end
                    else begin
                        r_bit_idx <= r_bit_idx + 3'd1;
                    end
                end
                else begin
                    r_clk_cnt <= r_clk_cnt + 16'd1;
                end
            end

            ST_STOP: begin
                o_tx_serial <= 1'b1;

                if (r_clk_cnt == (CLKS_PER_BIT - 1)) begin
                    r_clk_cnt <= 16'd0;
                    r_state   <= ST_CLEANUP;
                end
                else begin
                    r_clk_cnt <= r_clk_cnt + 16'd1;
                end
            end

            ST_CLEANUP: begin
                o_tx_serial <= 1'b1;
                o_tx_active <= 1'b0;
                r_state       <= ST_IDLE;
            end

            default: begin
                r_state <= ST_IDLE;
            end
        endcase
    end
end

endmodule

`default_nettype wire