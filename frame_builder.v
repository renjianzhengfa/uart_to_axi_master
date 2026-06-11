`timescale 1ns / 1ps
`default_nettype none

module frame_builder #(
    parameter integer MAX_FRAME_DATA_BYTES = 1024
) (
    input  wire        i_clk,
    input  wire        i_rst,
    
    input  wire        i_resp_valid,
    output wire        o_resp_ready,
    input  wire [15:0] i_resp_len_field,
    input  wire [10:0] i_resp_data_count,
    input  wire [7:0]  i_resp_cmd,
    input  wire [47:0] i_resp_addr,
    input  wire [7:0]  i_resp_status,
    input  wire        i_resp_data_wr_en,
    input  wire [9:0]  i_resp_data_wr_addr,
    input  wire [7:0]  i_resp_data_wr_byte,

    output wire        o_tx_valid,
    input  wire        i_tx_ready,
    output wire [7:0]  o_tx_byte,
    output reg         o_busy
);

localparam integer LP_MAX_FRAME_DATA_BYTES = (MAX_FRAME_DATA_BYTES < 4) ? 4 : ((MAX_FRAME_DATA_BYTES > 1024) ? 1024 : MAX_FRAME_DATA_BYTES);

reg [7:0] r_resp_data_mem [0:LP_MAX_FRAME_DATA_BYTES-1];
reg [15:0] r_resp_len_field_latched;
reg [10:0] r_resp_data_count_latched;
reg [7:0] r_resp_cmd_latched;
reg [47:0] r_resp_addr_latched;
reg [7:0] r_resp_status_latched;
reg [10:0] r_send_idx;
reg [7:0] r_chk_accum;
reg [7:0] r_tx_byte_reg;

wire [10:0] w_chk_idx;
wire [10:0] w_eof_idx;
wire w_send_handshake;

assign o_resp_ready    = !o_busy;
assign o_tx_valid      = o_busy;
assign o_tx_byte       = r_tx_byte_reg;
assign w_chk_idx       = 11'd11 + r_resp_data_count_latched;
assign w_eof_idx       = 11'd12 + r_resp_data_count_latched;
assign w_send_handshake = o_busy && i_tx_ready;

integer r_idx;

always @(*) begin
    r_tx_byte_reg = 8'h55;
    case (r_send_idx)
        11'd0: r_tx_byte_reg = 8'hAA;
        11'd1: r_tx_byte_reg = r_resp_len_field_latched[15:8];
        11'd2: r_tx_byte_reg = r_resp_len_field_latched[7:0];
        11'd3: r_tx_byte_reg = r_resp_cmd_latched;
        11'd4: r_tx_byte_reg = r_resp_addr_latched[47:40];
        11'd5: r_tx_byte_reg = r_resp_addr_latched[39:32];
        11'd6: r_tx_byte_reg = r_resp_addr_latched[31:24];
        11'd7: r_tx_byte_reg = r_resp_addr_latched[23:16];
        11'd8: r_tx_byte_reg = r_resp_addr_latched[15:8];
        11'd9: r_tx_byte_reg = r_resp_addr_latched[7:0];
        11'd10: r_tx_byte_reg = r_resp_status_latched;

        default: begin
            if ((r_send_idx >= 11'd11) && (r_send_idx < w_chk_idx)) begin
                r_tx_byte_reg = r_resp_data_mem[r_send_idx - 11'd11];
            end
            else if (r_send_idx == w_chk_idx) begin
                r_tx_byte_reg = r_chk_accum;
            end
            else begin
                r_tx_byte_reg = 8'h55;
            end
        end
    endcase
end

always @(posedge i_clk) begin
    if (i_rst) begin
        r_resp_len_field_latched  <= 16'd0;
        r_resp_data_count_latched <= 11'd0;
        r_resp_cmd_latched        <= 8'd0;
        r_resp_addr_latched       <= 48'd0;
        r_resp_status_latched     <= 8'd0;
        r_send_idx                <= 11'd0;
        r_chk_accum               <= 8'd0;
        o_busy                    <= 1'b0;
        for (r_idx = 0; r_idx < LP_MAX_FRAME_DATA_BYTES; r_idx = r_idx + 1) begin
            r_resp_data_mem[r_idx] <= 8'd0;
        end
    end
    else begin
        if (i_resp_data_wr_en) begin
            r_resp_data_mem[i_resp_data_wr_addr] <= i_resp_data_wr_byte;
        end
        if (!o_busy) begin
            if (i_resp_valid) begin
                r_resp_len_field_latched  <= i_resp_len_field;
                r_resp_data_count_latched <= i_resp_data_count;
                r_resp_cmd_latched        <= i_resp_cmd;
                r_resp_addr_latched       <= i_resp_addr;
                r_resp_status_latched     <= i_resp_status;
                r_send_idx                <= 11'd0;
                r_chk_accum               <= 8'd0;
                o_busy                    <= 1'b1;
            end
        end
        else if (w_send_handshake) begin
            if (r_send_idx < w_chk_idx) begin
                r_chk_accum <= r_chk_accum ^ r_tx_byte_reg;
            end
            if (r_send_idx == w_eof_idx) begin
                o_busy <= 1'b0;
            end
            else begin
                r_send_idx <= r_send_idx + 11'd1;
            end
        end
    end
end

endmodule

`default_nettype wire
