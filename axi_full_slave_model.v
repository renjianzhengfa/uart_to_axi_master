`timescale 1ns / 1ps
`default_nettype none

module axi_full_slave_model #(
    parameter integer AXI_ADDR_WIDTH = 40,
    parameter integer MEM_WORDS = 512
) (
    input  wire                      i_clk,
    input  wire                      i_rst,
    input  wire [7:0]                i_aw_wait_cycles,
    input  wire [7:0]                i_w_wait_cycles,
    input  wire [7:0]                i_b_wait_cycles,
    input  wire [7:0]                i_ar_wait_cycles,
    input  wire [7:0]                i_r_wait_cycles,
    input  wire                      i_block_aw,
    input  wire                      i_block_w,
    input  wire                      i_block_b,
    input  wire                      i_block_ar,
    input  wire                      i_block_r,
    input  wire                      i_force_bresp_err,
    input  wire                      i_force_rresp_err,
    input  wire [AXI_ADDR_WIDTH-1:0] i_s_axi_awaddr,
    input  wire [7:0]                i_s_axi_awlen,
    input  wire [2:0]                i_s_axi_awsize,
    input  wire [1:0]                i_s_axi_awburst,
    input  wire                      i_s_axi_awlock,
    input  wire [3:0]                i_s_axi_awcache,
    input  wire [2:0]                i_s_axi_awprot,
    input  wire [3:0]                i_s_axi_awqos,
    input  wire                      i_s_axi_awvalid,
    output reg                       o_s_axi_awready,
    input  wire [31:0]               i_s_axi_wdata,
    input  wire [3:0]                i_s_axi_wstrb,
    input  wire                      i_s_axi_wlast,
    input  wire                      i_s_axi_wvalid,
    output reg                       o_s_axi_wready,
    output reg [1:0]                 o_s_axi_bresp,
    output reg                       o_s_axi_bvalid,
    input  wire                      i_s_axi_bready,
    input  wire [AXI_ADDR_WIDTH-1:0] i_s_axi_araddr,
    input  wire [7:0]                i_s_axi_arlen,
    input  wire [2:0]                i_s_axi_arsize,
    input  wire [1:0]                i_s_axi_arburst,
    input  wire                      i_s_axi_arlock,
    input  wire [3:0]                i_s_axi_arcache,
    input  wire [2:0]                i_s_axi_arprot,
    input  wire [3:0]                i_s_axi_arqos,
    input  wire                      i_s_axi_arvalid,
    output reg                       o_s_axi_arready,
    output reg [31:0]                o_s_axi_rdata,
    output reg [1:0]                 o_s_axi_rresp,
    output reg                       o_s_axi_rlast,
    output reg                       o_s_axi_rvalid,
    input  wire                      i_s_axi_rready
);

localparam [2:0] ST_IDLE  = 3'd0;
localparam [2:0] ST_WR_AW = 3'd1;
localparam [2:0] ST_WR_W  = 3'd2;
localparam [2:0] ST_WR_B  = 3'd3;
localparam [2:0] ST_RD_AR = 3'd4;
localparam [2:0] ST_RD_R  = 3'd5;

reg [2:0] r_state;
reg [7:0] r_wait_cnt;
reg [AXI_ADDR_WIDTH-1:0] r_addr_latched;
reg [7:0] r_beats_latched;
reg [7:0] r_beat_idx;
reg [31:0] r_mem_word [0:MEM_WORDS-1];
integer r_idx;

always @(posedge i_clk) begin
    if (i_rst) begin
        r_state          <= ST_IDLE;
        r_wait_cnt       <= 8'd0;
        r_addr_latched   <= {AXI_ADDR_WIDTH{1'b0}};
        r_beats_latched  <= 8'd0;
        r_beat_idx       <= 8'd0;
        o_s_axi_awready <= 1'b0;
        o_s_axi_wready  <= 1'b0;
        o_s_axi_bresp   <= 2'b00;
        o_s_axi_bvalid  <= 1'b0;
        o_s_axi_arready <= 1'b0;
        o_s_axi_rdata   <= 32'd0;
        o_s_axi_rresp   <= 2'b00;
        o_s_axi_rlast   <= 1'b0;
        o_s_axi_rvalid  <= 1'b0;

        for (r_idx = 0; r_idx < MEM_WORDS; r_idx = r_idx + 1) begin
            r_mem_word[r_idx] <= 32'd0;
        end
    end
    else begin
        case (r_state)
            ST_IDLE: begin
                r_wait_cnt       <= 8'd0;
                r_beat_idx       <= 8'd0;
                o_s_axi_awready <= 1'b0;
                o_s_axi_wready  <= 1'b0;
                o_s_axi_bvalid  <= 1'b0;
                o_s_axi_arready <= 1'b0;
                o_s_axi_rvalid  <= 1'b0;
                o_s_axi_rlast   <= 1'b0;

                if (i_s_axi_awvalid) begin
                    r_state <= ST_WR_AW;
                end
                else if (i_s_axi_arvalid) begin
                    r_state <= ST_RD_AR;
                end
            end

            ST_WR_AW: begin
                if (o_s_axi_awready && i_s_axi_awvalid) begin
                    o_s_axi_awready <= 1'b0;
                    r_addr_latched    <= i_s_axi_awaddr;
                    r_beats_latched   <= i_s_axi_awlen + 8'd1;
                    r_beat_idx        <= 8'd0;
                    r_wait_cnt        <= 8'd0;
                    r_state           <= ST_WR_W;
                end
                else if (!i_block_aw) begin
                    if (r_wait_cnt >= i_aw_wait_cycles) begin
                        o_s_axi_awready <= 1'b1;
                    end
                    else begin
                        r_wait_cnt <= r_wait_cnt + 8'd1;
                    end
                end
            end

            ST_WR_W: begin
                if (o_s_axi_wready && i_s_axi_wvalid) begin
                    o_s_axi_wready <= 1'b0;
                    r_mem_word[r_addr_latched[10:2] + r_beat_idx] <= i_s_axi_wdata;
                    r_wait_cnt <= 8'd0;

                    if (r_beat_idx == (r_beats_latched - 8'd1)) begin
                        o_s_axi_bresp <= i_force_bresp_err ? 2'b10 : 2'b00;
                        r_state         <= ST_WR_B;
                    end

                    r_beat_idx <= r_beat_idx + 8'd1;
                end
                else if (!i_block_w) begin
                    if (r_wait_cnt >= i_w_wait_cycles) begin
                        o_s_axi_wready <= 1'b1;
                    end
                    else begin
                        r_wait_cnt <= r_wait_cnt + 8'd1;
                    end
                end
            end

            ST_WR_B: begin
                if (o_s_axi_bvalid && i_s_axi_bready) begin
                    o_s_axi_bvalid <= 1'b0;
                    r_wait_cnt       <= 8'd0;
                    r_state          <= ST_IDLE;
                end
                else if (!i_block_b) begin
                    if (r_wait_cnt >= i_b_wait_cycles) begin
                        o_s_axi_bvalid <= 1'b1;
                    end
                    else begin
                        r_wait_cnt <= r_wait_cnt + 8'd1;
                    end
                end
            end

            ST_RD_AR: begin
                if (o_s_axi_arready && i_s_axi_arvalid) begin
                    o_s_axi_arready <= 1'b0;
                    r_addr_latched    <= i_s_axi_araddr;
                    r_beats_latched   <= i_s_axi_arlen + 8'd1;
                    r_beat_idx        <= 8'd0;
                    r_wait_cnt        <= 8'd0;
                    r_state           <= ST_RD_R;
                end
                else if (!i_block_ar) begin
                    if (r_wait_cnt >= i_ar_wait_cycles) begin
                        o_s_axi_arready <= 1'b1;
                    end
                    else begin
                        r_wait_cnt <= r_wait_cnt + 8'd1;
                    end
                end
            end

            ST_RD_R: begin
                if (o_s_axi_rvalid && i_s_axi_rready) begin
                    o_s_axi_rvalid <= 1'b0;
                    o_s_axi_rlast  <= 1'b0;
                    r_wait_cnt       <= 8'd0;

                    if (r_beat_idx == (r_beats_latched - 8'd1)) begin
                        r_state <= ST_IDLE;
                    end

                    r_beat_idx <= r_beat_idx + 8'd1;
                end
                else if (!i_block_r) begin
                    if (r_wait_cnt >= i_r_wait_cycles) begin
                        o_s_axi_rvalid <= 1'b1;
                        o_s_axi_rdata  <= r_mem_word[r_addr_latched[10:2] + r_beat_idx];
                        o_s_axi_rresp  <= i_force_rresp_err ? 2'b10 : 2'b00;
                        o_s_axi_rlast  <= (r_beat_idx == (r_beats_latched - 8'd1));
                    end
                    else begin
                        r_wait_cnt <= r_wait_cnt + 8'd1;
                    end
                end
            end

            default: begin
                r_state <= ST_IDLE;
            end
        endcase
    end
end

endmodule

`default_nettype wire