`timescale 1ns / 1ps
`default_nettype none

module axi_full_master #(
    parameter integer AXI_ADDR_WIDTH     = 48,
    parameter         AXI_DATA_WIDTH     = 64,
    parameter integer AXI_TIMEOUT_CYCLES = 1024,
    parameter [3:0]   AXI_AWCACHE        = 4'b0011,
    parameter [3:0]   AXI_ARCACHE        = 4'b0011,
    parameter [2:0]   AXI_AWPROT         = 3'b000,
    parameter [2:0]   AXI_ARPROT         = 3'b000,
    parameter [3:0]   AXI_AWQOS          = 4'd0,
    parameter [3:0]   AXI_ARQOS          = 4'd0,
    localparam        AXI_STRB_WIDTH     = AXI_DATA_WIDTH/8
) (
    input  wire                         i_clk,
    input  wire                         i_rst,
    input  wire                         i_cmd_valid,
    output wire                         o_cmd_ready,
    input  wire                         i_cmd_write,
    input  wire [AXI_ADDR_WIDTH-1:0]    i_cmd_addr,
    input  wire [8:0]                   i_cmd_beats,
    input  wire [2:0]                   i_cmd_size,
    output reg                          o_cmd_done,
    output reg                          o_cmd_error,
    output reg                          o_cmd_timeout,

    input  wire [AXI_DATA_WIDTH-1:0]    i_wr_data,
    input  wire                         i_wr_valid,
    output wire                         o_wr_ready,
    output reg  [AXI_DATA_WIDTH-1:0]    o_rd_data,
    output reg                          o_rd_valid,
    input  wire                         i_rd_ready,

    output reg  [AXI_ADDR_WIDTH-1:0]    o_m_axi_awaddr,
    output reg  [7:0]                   o_m_axi_awlen,
    output reg  [2:0]                   o_m_axi_awsize,
    output reg  [1:0]                   o_m_axi_awburst,
    output reg                          o_m_axi_awlock,
    output reg  [3:0]                   o_m_axi_awcache,
    output reg  [2:0]                   o_m_axi_awprot,
    output reg  [3:0]                   o_m_axi_awqos,
    output reg                          o_m_axi_awvalid,
    input  wire                         i_m_axi_awready,

    output wire [AXI_DATA_WIDTH-1:0]    o_m_axi_wdata,
    output wire [AXI_STRB_WIDTH-1:0]    o_m_axi_wstrb,
    output wire                         o_m_axi_wlast,
    output wire                         o_m_axi_wvalid,
    input  wire                         i_m_axi_wready,

    input  wire [1:0]                   i_m_axi_bresp,
    input  wire                         i_m_axi_bvalid,
    output reg                          o_m_axi_bready,

    output reg  [AXI_ADDR_WIDTH-1:0]    o_m_axi_araddr,
    output reg  [7:0]                   o_m_axi_arlen,
    output reg  [2:0]                   o_m_axi_arsize,
    output reg  [1:0]                   o_m_axi_arburst,
    output reg                          o_m_axi_arlock,
    output reg  [3:0]                   o_m_axi_arcache,
    output reg  [2:0]                   o_m_axi_arprot,
    output reg  [3:0]                   o_m_axi_arqos,
    output reg                          o_m_axi_arvalid,
    input  wire                         i_m_axi_arready,

    input  wire [AXI_DATA_WIDTH-1:0]    i_m_axi_rdata,
    input  wire [1:0]                   i_m_axi_rresp,
    input  wire                         i_m_axi_rlast,
    input  wire                         i_m_axi_rvalid,
    output wire                         o_m_axi_rready,

    input  wire [AXI_STRB_WIDTH-1:0]    i_wr_strb
);

localparam [2:0] ST_IDLE  = 3'd0;
localparam [2:0] ST_WR_AW = 3'd1;
localparam [2:0] ST_WR_W  = 3'd2;
localparam [2:0] ST_WR_B  = 3'd3;
localparam [2:0] ST_RD_AR = 3'd4;
localparam [2:0] ST_RD_R  = 3'd5;

localparam [2:0] AXI_SIZE = $clog2(AXI_STRB_WIDTH);

reg  [2:0] r_state;
reg  [8:0] r_cmd_beats_latched;
reg  [8:0] r_wr_beat_idx;
reg  [8:0] r_rd_beat_idx;
reg  [31:0] r_timeout_cnt;
reg         r_error_seen;

wire w_wr_last_beat;
wire w_wr_handshake;
wire w_rd_handshake;

assign o_cmd_ready    = (r_state == ST_IDLE);
assign w_wr_last_beat = (r_wr_beat_idx == (r_cmd_beats_latched - 9'd1));
assign o_wr_ready     = (r_state == ST_WR_W) && (r_wr_beat_idx < r_cmd_beats_latched) && i_m_axi_wready;

assign o_m_axi_wdata  = i_wr_data;
assign o_m_axi_wstrb  = i_wr_strb;
assign o_m_axi_wlast  = (r_state == ST_WR_W) && (r_wr_beat_idx < r_cmd_beats_latched) && w_wr_last_beat;
assign o_m_axi_wvalid = (r_state == ST_WR_W) && (r_wr_beat_idx < r_cmd_beats_latched) && i_wr_valid;

assign o_m_axi_rready = (r_state == ST_RD_R) && i_rd_ready;

assign w_wr_handshake = o_m_axi_wvalid && i_m_axi_wready;
assign w_rd_handshake = i_m_axi_rvalid && o_m_axi_rready;

always @(posedge i_clk) begin
    if (i_rst) begin
        r_state             <= ST_IDLE;
        r_cmd_beats_latched <= 9'd0;
        r_wr_beat_idx       <= 9'd0;
        r_rd_beat_idx       <= 9'd0;
        r_timeout_cnt       <= 32'd0;
        r_error_seen        <= 1'b0;

        o_cmd_done          <= 1'b0;
        o_cmd_error         <= 1'b0;
        o_cmd_timeout       <= 1'b0;

        o_rd_data           <= {AXI_DATA_WIDTH{1'b0}};
        o_rd_valid          <= 1'b0;

        o_m_axi_awaddr      <= {AXI_ADDR_WIDTH{1'b0}};
        o_m_axi_awlen       <= 8'd0;
        o_m_axi_awsize      <= AXI_SIZE[2:0];
        o_m_axi_awburst     <= 2'b01;
        o_m_axi_awlock      <= 1'b0;
        o_m_axi_awcache     <= AXI_AWCACHE;
        o_m_axi_awprot      <= AXI_AWPROT;
        o_m_axi_awqos       <= AXI_AWQOS;
        o_m_axi_awvalid     <= 1'b0;

        o_m_axi_bready      <= 1'b0;

        o_m_axi_araddr      <= {AXI_ADDR_WIDTH{1'b0}};
        o_m_axi_arlen       <= 8'd0;
        o_m_axi_arsize      <= AXI_SIZE[2:0];
        o_m_axi_arburst     <= 2'b01;
        o_m_axi_arlock      <= 1'b0;
        o_m_axi_arcache     <= AXI_ARCACHE;
        o_m_axi_arprot      <= AXI_ARPROT;
        o_m_axi_arqos       <= AXI_ARQOS;
        o_m_axi_arvalid     <= 1'b0;
    end
    else begin
        o_cmd_done    <= 1'b0;
        o_cmd_error   <= 1'b0;
        o_cmd_timeout <= 1'b0;
        o_rd_valid    <= 1'b0;

        case (r_state)
            ST_IDLE: begin
                r_timeout_cnt <= 32'd0;
                r_wr_beat_idx <= 9'd0;
                r_rd_beat_idx <= 9'd0;
                r_error_seen  <= 1'b0;

                if (i_cmd_valid) begin
                    if (i_cmd_beats == 9'd0) begin
                        o_cmd_done  <= 1'b1;
                        o_cmd_error <= 1'b1;
                    end
                    else begin
                        r_cmd_beats_latched <= i_cmd_beats;

                        if (i_cmd_write) begin
                            o_m_axi_awaddr  <= i_cmd_addr;
                            o_m_axi_awlen   <= i_cmd_beats - 9'd1;
                            o_m_axi_awsize  <= i_cmd_size;
                            o_m_axi_awburst <= 2'b01;
                            o_m_axi_awlock  <= 1'b0;
                            o_m_axi_awcache <= AXI_AWCACHE;
                            o_m_axi_awprot  <= AXI_AWPROT;
                            o_m_axi_awqos   <= AXI_AWQOS;
                            o_m_axi_awvalid <= 1'b1;
                            r_state         <= ST_WR_AW;
                        end
                        else begin
                            o_m_axi_araddr  <= i_cmd_addr;
                            o_m_axi_arlen   <= i_cmd_beats - 9'd1;
                            o_m_axi_arsize  <= i_cmd_size;
                            o_m_axi_arburst <= 2'b01;
                            o_m_axi_arlock  <= 1'b0;
                            o_m_axi_arcache <= AXI_ARCACHE;
                            o_m_axi_arprot  <= AXI_ARPROT;
                            o_m_axi_arqos   <= AXI_ARQOS;
                            o_m_axi_arvalid <= 1'b1;
                            r_state         <= ST_RD_AR;
                        end
                    end
                end
            end

            ST_WR_AW: begin
                if (o_m_axi_awvalid && i_m_axi_awready) begin
                    o_m_axi_awvalid <= 1'b0;
                    r_timeout_cnt   <= 32'd0;
                    r_state         <= ST_WR_W;
                end
                else if (r_timeout_cnt >= (AXI_TIMEOUT_CYCLES - 1)) begin
                    o_m_axi_awvalid <= 1'b0;
                    o_cmd_done      <= 1'b1;
                    o_cmd_error     <= 1'b1;
                    o_cmd_timeout   <= 1'b1;
                    r_timeout_cnt   <= 32'd0;
                    r_state         <= ST_IDLE;
                end
                else begin
                    r_timeout_cnt <= r_timeout_cnt + 32'd1;
                end
            end

            ST_WR_W: begin
                if (w_wr_handshake) begin
                    r_timeout_cnt <= 32'd0;

                    if (w_wr_last_beat) begin
                        o_m_axi_bready <= 1'b1;
                        r_state        <= ST_WR_B;
                    end

                    r_wr_beat_idx <= r_wr_beat_idx + 9'd1;
                end
                else if (r_timeout_cnt >= (AXI_TIMEOUT_CYCLES - 1)) begin
                    o_m_axi_bready <= 1'b0;
                    o_cmd_done     <= 1'b1;
                    o_cmd_error    <= 1'b1;
                    o_cmd_timeout  <= 1'b1;
                    r_timeout_cnt  <= 32'd0;
                    r_state        <= ST_IDLE;
                end
                else begin
                    r_timeout_cnt <= r_timeout_cnt + 32'd1;
                end
            end

            ST_WR_B: begin
                if (i_m_axi_bvalid && o_m_axi_bready) begin
                    o_m_axi_bready <= 1'b0;
                    o_cmd_done     <= 1'b1;
                    o_cmd_error    <= (i_m_axi_bresp != 2'b00);
                    r_timeout_cnt  <= 32'd0;
                    r_state        <= ST_IDLE;
                end
                else if (r_timeout_cnt >= (AXI_TIMEOUT_CYCLES - 1)) begin
                    o_m_axi_bready <= 1'b0;
                    o_cmd_done     <= 1'b1;
                    o_cmd_error    <= 1'b1;
                    o_cmd_timeout  <= 1'b1;
                    r_timeout_cnt  <= 32'd0;
                    r_state        <= ST_IDLE;
                end
                else begin
                    r_timeout_cnt <= r_timeout_cnt + 32'd1;
                end
            end

            ST_RD_AR: begin
                if (o_m_axi_arvalid && i_m_axi_arready) begin
                    o_m_axi_arvalid <= 1'b0;
                    r_timeout_cnt   <= 32'd0;
                    r_state         <= ST_RD_R;
                end
                else if (r_timeout_cnt >= (AXI_TIMEOUT_CYCLES - 1)) begin
                    o_m_axi_arvalid <= 1'b0;
                    o_cmd_done      <= 1'b1;
                    o_cmd_error     <= 1'b1;
                    o_cmd_timeout   <= 1'b1;
                    r_timeout_cnt   <= 32'd0;
                    r_state         <= ST_IDLE;
                end
                else begin
                    r_timeout_cnt <= r_timeout_cnt + 32'd1;
                end
            end

            ST_RD_R: begin
                if (w_rd_handshake) begin
                    o_rd_data     <= i_m_axi_rdata;
                    o_rd_valid    <= 1'b1;
                    r_timeout_cnt <= 32'd0;

                    if (i_m_axi_rresp != 2'b00)
                        r_error_seen <= 1'b1;

                    if (i_m_axi_rlast != (r_rd_beat_idx == (r_cmd_beats_latched - 9'd1)))
                        r_error_seen <= 1'b1;

                    if (r_rd_beat_idx == (r_cmd_beats_latched - 9'd1)) begin
                        o_cmd_done  <= 1'b1;
                        o_cmd_error <= r_error_seen |
                                       (i_m_axi_rresp != 2'b00) |
                                       (i_m_axi_rlast != 1'b1);
                        r_state     <= ST_IDLE;
                    end

                    r_rd_beat_idx <= r_rd_beat_idx + 9'd1;
                end
                else if (r_timeout_cnt >= (AXI_TIMEOUT_CYCLES - 1)) begin
                    o_cmd_done    <= 1'b1;
                    o_cmd_error   <= 1'b1;
                    o_cmd_timeout <= 1'b1;
                    r_timeout_cnt <= 32'd0;
                    r_state       <= ST_IDLE;
                end
                else begin
                    r_timeout_cnt <= r_timeout_cnt + 32'd1;
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