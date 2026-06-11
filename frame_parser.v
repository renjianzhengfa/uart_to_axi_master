`timescale 1ns / 1ps
`default_nettype none

module frame_parser #(
    parameter integer MAX_FRAME_DATA_BYTES = 1024
) (
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_rx_dv,
    input  wire [7:0]  i_rx_byte,
    output reg         o_req_valid,
    input  wire        i_req_ready,
    output reg [15:0]  o_req_len,
    output reg [7:0]   o_req_cmd,
    output reg [47:0]  o_req_addr,
    output reg         o_req_data_wr_en,
    output reg [9:0]   o_req_data_wr_addr,
    output reg [7:0]   o_req_data_wr_byte,
    output reg         o_frame_drop_pulse
);

//repleas only hot?
localparam [3:0] ST_IDLE   = 4'd0;
localparam [3:0] ST_LEN_H  = 4'd1;
localparam [3:0] ST_LEN_L  = 4'd2;
localparam [3:0] ST_CMD    = 4'd3;

localparam [3:0] ST_ADDR_0 = 4'd4;
localparam [3:0] ST_ADDR_1 = 4'd5;
localparam [3:0] ST_ADDR_2 = 4'd6;
localparam [3:0] ST_ADDR_3 = 4'd7;
localparam [3:0] ST_ADDR_4 = 4'd8;
localparam [3:0] ST_ADDR_5 = 4'd9;
// localparam [3:0] ST_ADDR_6 = 4'd10;
// localparam [3:0] ST_ADDR_7 = 4'd11;

localparam [3:0] ST_DATA   = 4'd10;
localparam [3:0] ST_CHK    = 4'd11;
localparam [3:0] ST_EOF    = 4'd12;

localparam [7:0] CMD_LITE_READ = 8'h02;
localparam [7:0] CMD_FULL_READ = 8'h04;

localparam integer LP_MAX_FRAME_DATA_BYTES = (MAX_FRAME_DATA_BYTES < 4) ? 4 : ((MAX_FRAME_DATA_BYTES > 1024) ? 1024 : MAX_FRAME_DATA_BYTES);

reg [3:0] r_state;
reg [15:0] r_data_idx;
reg [7:0] r_chk_accum;
reg [15:0] r_len_latched;
reg [7:0] r_cmd_latched;
reg [47:0] r_addr_latched;
reg [15:0] r_payload_byte_count;

function [15:0] calc_payload_byte_count;
    input [7:0] cmd_value;
    input [15:0] len_value;
    begin
        case (cmd_value)
            CMD_LITE_READ,
            CMD_FULL_READ: begin
                calc_payload_byte_count = 16'd0;
            end
            default: begin
                calc_payload_byte_count = len_value;
            end
        endcase
    end
endfunction

always @(posedge i_clk) begin
    if (i_rst) begin
        r_state              <= ST_IDLE;
        r_data_idx           <= 16'd0;
        r_chk_accum          <= 8'd0;
        r_len_latched        <= 16'd0;
        r_cmd_latched        <= 8'd0;
        r_addr_latched       <= 47'd0;
        r_payload_byte_count <= 16'd0;
        o_req_valid        <= 1'b0;
        o_req_len          <= 16'd0;
        o_req_cmd          <= 8'd0;
        o_req_addr         <= 47'd0;
        o_req_data_wr_en   <= 1'b0;
        o_req_data_wr_addr <= 10'd0;
        o_req_data_wr_byte <= 8'd0;
        o_frame_drop_pulse <= 1'b0;
    end
    else begin
        o_req_data_wr_en   <= 1'b0;
        o_frame_drop_pulse <= 1'b0;

        if (o_req_valid && i_req_ready) begin
            o_req_valid <= 1'b0;
        end

        if (i_rx_dv && !o_req_valid) begin
            case (r_state)
                ST_IDLE: begin
                    if (i_rx_byte == 8'hAA) begin
                        r_chk_accum          <= 8'hAA;
                        r_len_latched        <= 16'd0;
                        r_cmd_latched        <= 8'd0;
                        r_addr_latched       <= 47'd0;
                        r_payload_byte_count <= 16'd0;
                        r_data_idx           <= 16'd0;
                        r_state              <= ST_LEN_H;
                    end
                end
                ST_LEN_H: begin
                    r_len_latched[15:8] <= i_rx_byte;
                    r_chk_accum         <= r_chk_accum ^ i_rx_byte;
                    r_state             <= ST_LEN_L;
                end
                ST_LEN_L: begin
                    r_len_latched[7:0] <= i_rx_byte;
                    r_chk_accum        <= r_chk_accum ^ i_rx_byte;
                    r_state            <= ST_CMD;
                end
                ST_CMD: begin
                    r_cmd_latched        <= i_rx_byte;
                    r_payload_byte_count <= calc_payload_byte_count(i_rx_byte, r_len_latched);
                    r_chk_accum          <= r_chk_accum ^ i_rx_byte;
                    r_state              <= ST_ADDR_0;
                end
                ST_ADDR_0: begin
                    r_addr_latched[47:40] <= i_rx_byte;
                    r_chk_accum           <= r_chk_accum ^ i_rx_byte;
                    r_state               <= ST_ADDR_1;
                end
                ST_ADDR_1: begin
                    r_addr_latched[39:32] <= i_rx_byte;
                    r_chk_accum          <= r_chk_accum ^ i_rx_byte;
                    r_state              <= ST_ADDR_2;
                end
                ST_ADDR_2: begin
                    r_addr_latched[31:24] <= i_rx_byte;
                    r_chk_accum          <= r_chk_accum ^ i_rx_byte;
                    r_state              <= ST_ADDR_3;
                end
                ST_ADDR_3: begin
                    r_addr_latched[23:16] <= i_rx_byte;
                    r_chk_accum          <= r_chk_accum ^ i_rx_byte;
                    r_state              <= ST_ADDR_4;
                end
                ST_ADDR_4: begin
                    r_addr_latched[15:8] <= i_rx_byte;
                    r_chk_accum          <= r_chk_accum ^ i_rx_byte;
                    r_state              <= ST_ADDR_5;
                end
                ST_ADDR_5: begin
                    r_addr_latched[7:0] <= i_rx_byte;
                    r_chk_accum         <= r_chk_accum ^ i_rx_byte;
                    r_data_idx          <= 16'd0;
                    if (r_payload_byte_count == 16'd0) begin
                        r_state <= ST_CHK;
                    end
                    else begin
                        r_state <= ST_DATA;
                    end
                end

                ST_DATA: begin
                    if (r_data_idx < LP_MAX_FRAME_DATA_BYTES) begin
                        o_req_data_wr_en   <= 1'b1;
                        o_req_data_wr_addr <= r_data_idx[9:0];
                        o_req_data_wr_byte <= i_rx_byte;
                    end
                    r_chk_accum <= r_chk_accum ^ i_rx_byte;
                    if (r_data_idx == (r_payload_byte_count - 16'd1)) begin
                        r_state <= ST_CHK;
                    end
                    r_data_idx <= r_data_idx + 16'd1;
                end
                ST_CHK: begin
                    if (i_rx_byte == r_chk_accum) begin
                        r_state <= ST_EOF;
                    end
                    else begin
                        o_frame_drop_pulse <= 1'b1;
                        r_state            <= ST_IDLE;
                    end
                end
                ST_EOF: begin
                    if (i_rx_byte == 8'h55) begin
                        o_req_len   <= r_len_latched;
                        o_req_cmd   <= r_cmd_latched;
                        o_req_addr  <= r_addr_latched;
                        o_req_valid <= 1'b1;
                    end
                    else begin
                        o_frame_drop_pulse <= 1'b1;
                    end
                    r_state <= ST_IDLE;
                end
                default: begin
                    r_state <= ST_IDLE;
                end
            endcase
        end
    end
end

endmodule

`default_nettype wire
