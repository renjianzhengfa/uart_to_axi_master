`timescale 1ns / 1ps
`default_nettype none

module bridge_ctrl_chs #(
    parameter  AXI_ADDR_WIDTH       = 48,
    parameter  AXI_DATA_BYTES       = 8,

    parameter  MAX_FRAME_DATA_BYTES = 1024,

    parameter [AXI_ADDR_WIDTH-1:0] REG_MIN_ADDR   = 48'h0300_0000,
    parameter [AXI_ADDR_WIDTH-1:0] REG_MAX_ADDR   = 48'h08FF_FFFF,
    parameter [AXI_ADDR_WIDTH-1:0] SPM_C_MIN_ADDR = 48'h1000_0000,
    parameter [AXI_ADDR_WIDTH-1:0] SPM_C_MAX_ADDR = 48'h13FF_FFFF,
    parameter [AXI_ADDR_WIDTH-1:0] SPM_U_MIN_ADDR = 48'h1400_0000,
    parameter [AXI_ADDR_WIDTH-1:0] SPM_U_MAX_ADDR = 48'h17FF_FFFF,

    // 
    parameter [AXI_ADDR_WIDTH-1:0] EXEC_ENTRY_LO_ADDR = 48'h0300_0000,
    parameter [AXI_ADDR_WIDTH-1:0] EXEC_ENTRY_HI_ADDR = 48'h0300_0004,
    parameter [AXI_ADDR_WIDTH-1:0] EXEC_GO_ADDR       = 48'h0300_0008,
    parameter [31:0]               EXEC_GO_DATA       = 32'h0000_0002
) (
    input  wire                           i_clk,
    input  wire                           i_rst,

    // Parsed request from frame parser
    input  wire                           i_req_valid,
    output reg                            o_req_ready,
    input  wire [15:0]                    i_req_len,
    input  wire [7:0]                     i_req_cmd,
    input  wire [AXI_ADDR_WIDTH-1:0]      i_req_addr,
    input  wire                           i_req_data_wr_en,
    input  wire [9:0]                     i_req_data_wr_addr,
    input  wire [7:0]                     i_req_data_wr_byte,

    // Response payload RAM write-out toward frame builder
    output reg                            o_resp_valid,
    input  wire                           i_resp_ready,
    output reg [15:0]                     o_resp_len_field,
    output reg [10:0]                     o_resp_data_count,
    output reg [7:0]                      o_resp_cmd,
    output reg [AXI_ADDR_WIDTH-1:0]       o_resp_addr,
    output reg [7:0]                      o_resp_status,
    output reg                            o_resp_data_wr_en,
    output reg [9:0]                      o_resp_data_wr_addr,
    output reg [7:0]                      o_resp_data_wr_byte,

    // Unified AXI4 burst command path
    output reg                            o_axi_cmd_valid,
    input  wire                           i_axi_cmd_ready,
    output reg                            o_axi_cmd_write,
    output reg [AXI_ADDR_WIDTH-1:0]       o_axi_cmd_addr,
    output reg [8:0]                      o_axi_cmd_beats,
    output reg [2:0]                      o_axi_cmd_size,
    input  wire                           i_axi_cmd_done,
    input  wire                           i_axi_cmd_error,
    input  wire                           i_axi_cmd_timeout,

    // Write data stream toward AXI master
    output wire [AXI_DATA_BYTES*8-1:0]    o_axi_wr_data,
    output wire [AXI_DATA_BYTES-1:0]      o_axi_wr_strb,
    output wire                           o_axi_wr_valid,
    input  wire                           i_axi_wr_ready,

    // Read data stream from AXI master
    input  wire [AXI_DATA_BYTES*8-1:0]    i_axi_rd_data,
    input  wire                           i_axi_rd_valid,
    output reg                            o_axi_rd_ready,

    output wire                           o_busy
);


localparam [AXI_ADDR_WIDTH-1:0] NPU_MIN_ADDR   = 48'h4000_0000;
localparam [AXI_ADDR_WIDTH-1:0] NPU_MAX_ADDR   = 48'h4001_0000;

localparam REG_ACCESS_BYTES     = 4;
localparam integer LP_MAX_FRAME_DATA_BYTES = (MAX_FRAME_DATA_BYTES < 16) ? 16 : MAX_FRAME_DATA_BYTES;
localparam integer LP_DATA_WIDTH           = AXI_DATA_BYTES * 8;
localparam integer LP_MAX_BURST_BEATS      = 256;
localparam integer LP_MAX_BURST_BYTES      = LP_MAX_BURST_BEATS * AXI_DATA_BYTES;
localparam integer LP_AXI_BYTE_LSB       = $clog2(AXI_DATA_BYTES);
localparam [2:0] LP_AXI_SIZE_MEM         = $clog2(AXI_DATA_BYTES);
localparam [2:0] LP_AXI_SIZE_REG         = $clog2(REG_ACCESS_BYTES);


localparam [3:0] ST_IDLE       = 4'd0;
localparam [3:0] ST_CHECK      = 4'd1;
localparam [3:0] ST_PREP       = 4'd2;
localparam [3:0] ST_CMD        = 4'd3;
localparam [3:0] ST_WRITE      = 4'd4;
localparam [3:0] ST_READ       = 4'd5;
localparam [3:0] ST_RESP_COPY  = 4'd6;
localparam [3:0] ST_RESP_SEND  = 4'd7;
localparam [3:0] ST_EXEC_PREP  = 4'd8;
localparam [3:0] ST_EXEC_CMD   = 4'd9;
localparam [3:0] ST_EXEC_WAIT  = 4'd10;

localparam [7:0] CMD_REG_WRITE = 8'h01;
localparam [7:0] CMD_REG_READ  = 8'h02;
localparam [7:0] CMD_MEM_WRITE = 8'h03;
localparam [7:0] CMD_MEM_READ  = 8'h04;
localparam [7:0] CMD_PING      = 8'hF0;
localparam [7:0] CMD_EXEC      = 8'hF1;

localparam [7:0] STATUS_SUCCESS   = 8'h00;
localparam [7:0] STATUS_ALIGN_ERR = 8'h01;
localparam [7:0] STATUS_AXI_ERR   = 8'h02;
localparam [7:0] STATUS_LEN_ERR   = 8'h03;
localparam [7:0] STATUS_CMD_ERR   = 8'h04;
localparam [7:0] STATUS_RANGE_ERR = 8'h05;

reg [3:0] r_state;

reg [7:0] r_req_data_mem  [0:LP_MAX_FRAME_DATA_BYTES-1];
reg [7:0] r_resp_data_mem [0:LP_MAX_FRAME_DATA_BYTES-1];

reg [15:0]                 r_req_len_latched;
reg [15:0]                 r_req_len_original;
reg [7:0]                  r_req_cmd_latched;
reg [AXI_ADDR_WIDTH-1:0]   r_req_addr_latched;

reg [7:0]                  r_resp_status_latched;
reg [15:0]                 r_resp_len_field_latched;
reg [10:0]                 r_resp_data_count_latched;
reg [9:0]                  r_copy_idx;

reg [AXI_ADDR_WIDTH-1:0]   r_cur_addr;
reg [15:0]                 r_bytes_left;
reg [15:0]                 r_data_ptr;
reg [15:0]                 r_burst_bytes;
reg [8:0]                  r_burst_beats;
reg [8:0]                  r_wr_beat_idx;
reg [8:0]                  r_rd_beat_idx;
reg                        r_is_write;
reg                        r_is_reg_cmd;

reg [LP_DATA_WIDTH-1:0]    axi_wr_data_comb;
integer                    wr_pack_idx;

// EXEC 
reg                        r_exec_active;
reg [1:0]                  r_exec_step;
reg [AXI_ADDR_WIDTH-1:0]   r_exec_cmd_addr;
reg [LP_DATA_WIDTH-1:0]    r_exec_wr_data;
reg [AXI_DATA_BYTES-1:0]   r_exec_wr_strb;

wire        w_cmd_supported;
wire        w_is_reg_cmd_req;
wire        w_is_mem_cmd_req;
wire        w_exec_cfg_valid;

wire        w_mem_addr_aligned;
wire        w_reg_addr_aligned;
wire        w_reg_len_aligned;
wire        w_cmd_wr;
wire        w_cmd_pe;

wire        w_addr_in_reg;
wire        w_addr_in_spm;
wire [15:0] w_next_burst_bytes;
wire [8:0]  w_next_burst_beats;
wire        w_last_burst;
wire [15:0] w_cur_beat_base_byte;
wire [15:0] w_reg_byte_lane;
wire [2:0]  w_cur_cmd_size;
wire [AXI_DATA_BYTES-1:0] w_reg_wr_strb;

wire w_addr_in_npu;
assign w_addr_in_npu =
    (r_req_addr_latched >= NPU_MIN_ADDR) &&
    (r_req_addr_latched <= NPU_MAX_ADDR);

assign o_busy = (r_state != ST_IDLE);

assign w_cmd_wr = (r_req_cmd_latched == CMD_REG_WRITE)
                || (r_req_cmd_latched == CMD_REG_READ )
                || (r_req_cmd_latched == CMD_MEM_WRITE)
                || (r_req_cmd_latched == CMD_MEM_READ );

assign w_cmd_pe = (r_req_cmd_latched == CMD_PING ) || (r_req_cmd_latched == CMD_EXEC     );

assign w_cmd_supported = w_cmd_wr || w_cmd_pe;

assign w_is_reg_cmd_req = (r_req_cmd_latched == CMD_REG_WRITE) || (r_req_cmd_latched == CMD_REG_READ);
assign w_is_mem_cmd_req = (r_req_cmd_latched == CMD_MEM_WRITE) || (r_req_cmd_latched == CMD_MEM_READ);

assign w_exec_cfg_valid =
    (EXEC_ENTRY_LO_ADDR != {AXI_ADDR_WIDTH{1'b0}}) &&
    (EXEC_ENTRY_HI_ADDR != {AXI_ADDR_WIDTH{1'b0}}) &&
    (EXEC_GO_ADDR       != {AXI_ADDR_WIDTH{1'b0}});

assign w_mem_addr_aligned =
    (r_req_addr_latched[$clog2(AXI_DATA_BYTES)-1:0] == {($clog2(AXI_DATA_BYTES)){1'b0}});

assign w_reg_addr_aligned =
    (r_req_addr_latched[$clog2(REG_ACCESS_BYTES)-1:0] == {($clog2(REG_ACCESS_BYTES)){1'b0}});

assign w_reg_len_aligned =
    (r_req_len_latched[$clog2(REG_ACCESS_BYTES)-1:0] == {($clog2(REG_ACCESS_BYTES)){1'b0}});

assign w_addr_in_reg =
    ((r_req_addr_latched >= REG_MIN_ADDR) && (r_req_addr_latched <= REG_MAX_ADDR)) || (w_addr_in_npu);

assign w_addr_in_spm =
    ((r_req_addr_latched >= SPM_C_MIN_ADDR) && (r_req_addr_latched <= SPM_C_MAX_ADDR)) ||
    ((r_req_addr_latched >= SPM_U_MIN_ADDR) && (r_req_addr_latched <= SPM_U_MAX_ADDR));

assign w_last_burst = (r_bytes_left == r_burst_bytes);
assign w_cur_beat_base_byte = r_data_ptr + (r_wr_beat_idx * AXI_DATA_BYTES);
assign w_reg_byte_lane      = {{(16-LP_AXI_BYTE_LSB){1'b0}}, r_cur_addr[LP_AXI_BYTE_LSB-1:0]};
assign w_cur_cmd_size       = r_is_reg_cmd ? LP_AXI_SIZE_REG : LP_AXI_SIZE_MEM;

function automatic [15:0] f_min16;
    input [15:0] a;
    input [15:0] b;
    begin
        f_min16 = (a < b) ? a : b;
    end
endfunction

function automatic [15:0] f_bytes_to_4kb;
    input [AXI_ADDR_WIDTH-1:0] addr;
    reg [11:0] low12;
    begin
        low12 = addr[11:0];
        f_bytes_to_4kb = 16'd4096 - {4'd0, low12};
    end
endfunction

function automatic [15:0] f_calc_burst_bytes;
    input [AXI_ADDR_WIDTH-1:0] addr;
    input [15:0] remaining_bytes;
    reg [15:0] candidate;
    reg [15:0] bound_4kb;
    begin
        candidate = f_min16(remaining_bytes, LP_MAX_BURST_BYTES[15:0]);
        bound_4kb = f_bytes_to_4kb(addr);
        f_calc_burst_bytes = f_min16(candidate, bound_4kb);
    end
endfunction

function automatic [8:0] f_calc_beats;
    input [15:0] burst_bytes;
    reg [15:0] tmp;
    begin
        tmp = burst_bytes + AXI_DATA_BYTES - 1;
        f_calc_beats = tmp / AXI_DATA_BYTES;
    end
endfunction

function automatic [AXI_DATA_BYTES-1:0] f_pack_wr_strb;
    input [15:0] burst_bytes;
    input [8:0]  beat_idx;
    integer j;
    reg [15:0] beat_base;
    reg [AXI_DATA_BYTES-1:0] tmp;
    begin
        beat_base = beat_idx * AXI_DATA_BYTES;
        tmp = {AXI_DATA_BYTES{1'b0}};
        for (j = 0; j < AXI_DATA_BYTES; j = j + 1) begin
            if ((beat_base + j) < burst_bytes)
                tmp[j] = 1'b1;
        end
        f_pack_wr_strb = tmp;
    end
endfunction

function automatic [AXI_DATA_BYTES-1:0] f_reg_wr_strb;
    input [AXI_ADDR_WIDTH-1:0] addr;
    integer j;
    reg [15:0] lane_base;
    reg [AXI_DATA_BYTES-1:0] tmp;
    begin
        lane_base = {{(16-LP_AXI_BYTE_LSB){1'b0}}, addr[LP_AXI_BYTE_LSB-1:0]};
        tmp = {AXI_DATA_BYTES{1'b0}};
        for (j = 0; j < AXI_DATA_BYTES; j = j + 1) begin
            if ((j >= lane_base) && (j < (lane_base + REG_ACCESS_BYTES)))
                tmp[j] = 1'b1;
        end
        f_reg_wr_strb = tmp;
    end
endfunction

assign w_reg_wr_strb = f_reg_wr_strb(r_cur_addr);

assign w_next_burst_bytes = r_is_reg_cmd ? REG_ACCESS_BYTES
                                         : f_calc_burst_bytes(r_cur_addr, r_bytes_left);
assign w_next_burst_beats = r_is_reg_cmd ? 9'd1
                                         : f_calc_beats(w_next_burst_bytes);


assign o_axi_wr_valid =
    ((r_state == ST_WRITE) && (r_wr_beat_idx < r_burst_beats)) ||
    ((r_state == ST_EXEC_WAIT) && r_exec_active);


always @(*) begin
    axi_wr_data_comb = {LP_DATA_WIDTH{1'b0}};

    if (r_is_reg_cmd) begin
        // REG path is 32-bit/4B on a 64-bit AXI data bus.
        // Place payload bytes into the byte lane selected by r_cur_addr[2:0].
        for (wr_pack_idx = 0; wr_pack_idx < REG_ACCESS_BYTES; wr_pack_idx = wr_pack_idx + 1) begin
            if ((r_data_ptr + wr_pack_idx) < LP_MAX_FRAME_DATA_BYTES) begin
                axi_wr_data_comb[(w_reg_byte_lane + wr_pack_idx)*8 +: 8] =
                    r_req_data_mem[r_data_ptr + wr_pack_idx];
            end
        end
    end
    else begin
        // MEM path keeps the existing 64-bit/8B packing and burst behavior.
        for (wr_pack_idx = 0; wr_pack_idx < AXI_DATA_BYTES; wr_pack_idx = wr_pack_idx + 1) begin
            if ((w_cur_beat_base_byte + wr_pack_idx) < LP_MAX_FRAME_DATA_BYTES)
                axi_wr_data_comb[wr_pack_idx*8 +: 8] = r_req_data_mem[w_cur_beat_base_byte + wr_pack_idx];
        end
    end
end

assign o_axi_wr_data = r_exec_active ? r_exec_wr_data : axi_wr_data_comb;
assign o_axi_wr_strb = r_exec_active ? r_exec_wr_strb :
                       (r_is_reg_cmd ? w_reg_wr_strb : f_pack_wr_strb(r_burst_bytes, r_wr_beat_idx));

integer r_idx;
integer k;

always @(posedge i_clk) begin
    if (i_rst) begin
        r_state                  <= ST_IDLE;
        r_req_len_latched        <= 16'd0;
        r_req_len_original       <= 16'd0;
        r_req_cmd_latched        <= 8'd0;
        r_req_addr_latched       <= {AXI_ADDR_WIDTH{1'b0}};
        r_resp_status_latched    <= 8'd0;
        r_resp_len_field_latched <= 16'd0;
        r_resp_data_count_latched<= 11'd0;
        r_copy_idx               <= 10'd0;
        r_cur_addr               <= {AXI_ADDR_WIDTH{1'b0}};
        r_bytes_left             <= 16'd0;
        r_data_ptr               <= 16'd0;
        r_burst_bytes            <= 16'd0;
        r_burst_beats            <= 9'd0;
        r_wr_beat_idx            <= 9'd0;
        r_rd_beat_idx            <= 9'd0;
        r_is_write               <= 1'b0;
        r_is_reg_cmd             <= 1'b0;

        r_exec_active            <= 1'b0;
        r_exec_step              <= 2'd0;
        r_exec_cmd_addr          <= {AXI_ADDR_WIDTH{1'b0}};
        r_exec_wr_data           <= {LP_DATA_WIDTH{1'b0}};
        r_exec_wr_strb           <= {AXI_DATA_BYTES{1'b0}};

        o_req_ready              <= 1'b0;
        o_resp_valid             <= 1'b0;
        o_resp_len_field         <= 16'd0;
        o_resp_data_count        <= 11'd0;
        o_resp_cmd               <= 8'd0;
        o_resp_addr              <= {AXI_ADDR_WIDTH{1'b0}};
        o_resp_status            <= 8'd0;
        o_resp_data_wr_en        <= 1'b0;
        o_resp_data_wr_addr      <= 10'd0;
        o_resp_data_wr_byte      <= 8'd0;
        o_axi_cmd_valid          <= 1'b0;
        o_axi_cmd_write          <= 1'b0;
        o_axi_cmd_addr           <= {AXI_ADDR_WIDTH{1'b0}};
        o_axi_cmd_beats          <= 9'd0;
        o_axi_cmd_size           <= LP_AXI_SIZE_MEM;
        o_axi_rd_ready           <= 1'b0;

        for (r_idx = 0; r_idx < LP_MAX_FRAME_DATA_BYTES; r_idx = r_idx + 1) begin
            r_req_data_mem[r_idx]  <= 8'd0;
            r_resp_data_mem[r_idx] <= 8'd0;
        end
    end else begin
        o_req_ready       <= 1'b0;
        o_resp_valid      <= 1'b0;
        o_resp_data_wr_en <= 1'b0;
        o_axi_cmd_valid   <= 1'b0;
        o_axi_rd_ready    <= 1'b0;

        if (i_req_data_wr_en && (i_req_data_wr_addr < LP_MAX_FRAME_DATA_BYTES)) begin
            r_req_data_mem[i_req_data_wr_addr] <= i_req_data_wr_byte;
        end

        if ((r_state == ST_READ) && i_axi_rd_valid) begin
            if (r_is_reg_cmd) begin
                // REG read returns only REG_ACCESS_BYTES bytes. Extract from the
                // AXI byte lane selected by r_cur_addr[2:0].
                for (k = 0; k < REG_ACCESS_BYTES; k = k + 1) begin
                    if ((r_data_ptr + k) < LP_MAX_FRAME_DATA_BYTES) begin
                        r_resp_data_mem[r_data_ptr + k]
                            <= i_axi_rd_data[(w_reg_byte_lane + k)*8 +: 8];
                    end
                end
            end
            else begin
                for (k = 0; k < AXI_DATA_BYTES; k = k + 1) begin
                    if (((r_rd_beat_idx * AXI_DATA_BYTES) + k) < r_burst_bytes) begin
                        if ((r_data_ptr + (r_rd_beat_idx * AXI_DATA_BYTES) + k) < LP_MAX_FRAME_DATA_BYTES) begin
                            r_resp_data_mem[r_data_ptr + (r_rd_beat_idx * AXI_DATA_BYTES) + k]
                                <= i_axi_rd_data[k*8 +: 8];
                        end
                    end
                end
            end
        end

        case (r_state)
            ST_IDLE: begin
                if (i_req_valid && i_resp_ready) begin
                    o_req_ready        <= 1'b1;
                    r_req_len_latched  <= i_req_len;
                    r_req_len_original <= i_req_len;
                    r_req_cmd_latched  <= i_req_cmd;
                    r_req_addr_latched <= i_req_addr;
                    o_resp_cmd         <= i_req_cmd;
                    o_resp_addr        <= i_req_addr;
                    r_state            <= ST_CHECK;
                end
            end

            ST_CHECK: begin
                if (!w_cmd_supported) begin
                    r_resp_status_latched     <= STATUS_CMD_ERR;
                    r_resp_len_field_latched  <= 16'd0;
                    r_resp_data_count_latched <= 11'd0;
                    r_state                   <= ST_RESP_SEND;
                end
                else if (r_req_cmd_latched == CMD_PING) begin
                    r_resp_status_latched     <= STATUS_SUCCESS;
                    r_resp_len_field_latched  <= 16'd8;
                    r_resp_data_count_latched <= 11'd8;
                    r_resp_data_mem[0]        <= 8'h55;
                    r_resp_data_mem[1]        <= 8'h01;
                    r_resp_data_mem[2]        <= AXI_ADDR_WIDTH;
                    r_resp_data_mem[3]        <= AXI_DATA_BYTES;
                    r_resp_data_mem[4]        <= REG_ACCESS_BYTES;
                    r_resp_data_mem[5]        <= {6'd0, w_exec_cfg_valid, 1'b1};
                    r_resp_data_mem[6]        <= 8'h00;
                    r_resp_data_mem[7]        <= 8'h00;
                    r_copy_idx                <= 10'd0;
                    r_state                   <= ST_RESP_COPY;
                end
                else if (r_req_cmd_latched == CMD_EXEC) begin
                    if (!w_exec_cfg_valid) begin
                        r_resp_status_latched     <= STATUS_RANGE_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if (r_req_len_latched != 16'd0) begin
                        r_resp_status_latched     <= STATUS_LEN_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else begin
                        r_exec_active <= 1'b1;
                        r_exec_step   <= 2'd0;
                        r_state       <= ST_EXEC_PREP;
                    end
                end
                else if (w_is_reg_cmd_req) begin
                    if (!w_addr_in_reg) begin
                        r_resp_status_latched     <= STATUS_RANGE_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if (!w_reg_addr_aligned) begin
                        r_resp_status_latched     <= STATUS_ALIGN_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if ((r_req_len_latched == 16'd0) ||
                             (r_req_len_latched > LP_MAX_FRAME_DATA_BYTES) ||
                             !w_reg_len_aligned) begin
                        r_resp_status_latched     <= STATUS_LEN_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else begin
                        r_is_write   <= (r_req_cmd_latched == CMD_REG_WRITE);
                        r_is_reg_cmd <= 1'b1;
                        r_cur_addr   <= r_req_addr_latched;
                        r_bytes_left <= r_req_len_latched;
                        r_data_ptr   <= 16'd0;
                        r_state      <= ST_PREP;
                    end
                end
                else begin
                    if (!w_addr_in_spm) begin
                        r_resp_status_latched     <= STATUS_RANGE_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if (!w_mem_addr_aligned) begin
                        r_resp_status_latched     <= STATUS_ALIGN_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if ((r_req_len_latched == 16'd0) || (r_req_len_latched > LP_MAX_FRAME_DATA_BYTES)) begin
                        r_resp_status_latched     <= STATUS_LEN_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else begin
                        r_is_write   <= (r_req_cmd_latched == CMD_MEM_WRITE);
                        r_is_reg_cmd <= 1'b0;
                        r_cur_addr   <= r_req_addr_latched;
                        r_bytes_left <= r_req_len_latched;
                        r_data_ptr   <= 16'd0;
                        r_state      <= ST_PREP;
                    end
                end
            end

            ST_PREP: begin
                r_burst_bytes <= w_next_burst_bytes;
                r_burst_beats <= w_next_burst_beats;
                r_wr_beat_idx <= 9'd0;
                r_rd_beat_idx <= 9'd0;
                o_axi_cmd_write <= r_is_write;
                o_axi_cmd_addr  <= r_cur_addr;
                o_axi_cmd_beats <= w_next_burst_beats;
                o_axi_cmd_size  <= w_cur_cmd_size;
                r_state         <= ST_CMD;
            end

            ST_CMD: begin
                o_axi_cmd_valid <= 1'b1;
                o_axi_cmd_write <= r_is_write;
                o_axi_cmd_addr  <= r_cur_addr;
                o_axi_cmd_beats <= r_burst_beats;
                o_axi_cmd_size  <= w_cur_cmd_size;
                if (i_axi_cmd_ready) begin
                    if (r_is_write)
                        r_state <= ST_WRITE;
                    else
                        r_state <= ST_READ;
                end
            end

            ST_WRITE: begin
                if (o_axi_wr_valid && i_axi_wr_ready) begin
                    r_wr_beat_idx <= r_wr_beat_idx + 9'd1;
                end
                if (i_axi_cmd_done) begin
                    if (i_axi_cmd_error || i_axi_cmd_timeout) begin
                        r_resp_status_latched     <= STATUS_AXI_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if (w_last_burst) begin
                        r_resp_status_latched     <= STATUS_SUCCESS;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else begin
                        r_cur_addr   <= r_cur_addr + r_burst_bytes;
                        r_bytes_left <= r_bytes_left - r_burst_bytes;
                        r_data_ptr   <= r_data_ptr + r_burst_bytes;
                        r_state      <= ST_PREP;
                    end
                end
            end

            ST_READ: begin
                o_axi_rd_ready <= 1'b1;
                if (i_axi_rd_valid) begin
                    r_rd_beat_idx <= r_rd_beat_idx + 9'd1;
                end
                if (i_axi_cmd_done) begin
                    if (i_axi_cmd_error || i_axi_cmd_timeout) begin
                        r_resp_status_latched     <= STATUS_AXI_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if (w_last_burst) begin
                        r_resp_status_latched     <= STATUS_SUCCESS;
                        r_resp_len_field_latched  <= r_req_len_original;
                        r_resp_data_count_latched <= r_req_len_original[10:0];
                        r_copy_idx                <= 10'd0;
                        r_state                   <= ST_RESP_COPY;
                    end
                    else begin
                        r_cur_addr   <= r_cur_addr + r_burst_bytes;
                        r_bytes_left <= r_bytes_left - r_burst_bytes;
                        r_data_ptr   <= r_data_ptr + r_burst_bytes;
                        r_state      <= ST_PREP;
                    end
                end
            end

            ST_EXEC_PREP: begin
                o_axi_cmd_write <= 1'b1;
                o_axi_cmd_beats <= 9'd1;
                o_axi_cmd_size  <= LP_AXI_SIZE_REG;

                r_exec_wr_data  <= {LP_DATA_WIDTH{1'b0}};
                r_exec_wr_strb  <= f_reg_wr_strb(r_exec_cmd_addr);

                if (r_exec_step == 2'd0) begin
                    r_exec_cmd_addr <= EXEC_ENTRY_LO_ADDR;
                    r_exec_wr_data[31:0] <= r_req_addr_latched[31:0];
                    r_exec_wr_strb <= f_reg_wr_strb(EXEC_ENTRY_LO_ADDR);
                    o_axi_cmd_addr <= EXEC_ENTRY_LO_ADDR;
                end
                else if (r_exec_step == 2'd1) begin
                    r_exec_cmd_addr <= EXEC_ENTRY_HI_ADDR;
                    r_exec_wr_data[63:32] <= r_req_addr_latched[AXI_ADDR_WIDTH-1:32];
                    r_exec_wr_strb <= f_reg_wr_strb(EXEC_ENTRY_HI_ADDR);
                    o_axi_cmd_addr <= EXEC_ENTRY_HI_ADDR;
                end
                else begin
                    r_exec_cmd_addr <= EXEC_GO_ADDR;
                    r_exec_wr_data[31:0] <= EXEC_GO_DATA;
                    r_exec_wr_strb <= f_reg_wr_strb(EXEC_GO_ADDR);
                    o_axi_cmd_addr <= EXEC_GO_ADDR;
                end

                r_state <= ST_EXEC_CMD;
            end

            ST_EXEC_CMD: begin
                o_axi_cmd_valid <= 1'b1;
                o_axi_cmd_write <= 1'b1;
                o_axi_cmd_addr  <= r_exec_cmd_addr;
                o_axi_cmd_beats <= 9'd1;
                o_axi_cmd_size  <= LP_AXI_SIZE_REG;

                if (i_axi_cmd_ready) begin
                    r_state <= ST_EXEC_WAIT;
                end
            end

            ST_EXEC_WAIT: begin
                if (i_axi_cmd_done) begin
                    if (i_axi_cmd_error || i_axi_cmd_timeout) begin
                        r_exec_active            <= 1'b0;
                        r_resp_status_latched     <= STATUS_AXI_ERR;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                    else if (r_exec_step == 2'd0) begin
                        r_exec_step <= 2'd1;
                        r_state     <= ST_EXEC_PREP;
                    end
                    else if (r_exec_step == 2'd1) begin
                        r_exec_step <= 2'd2;
                        r_state     <= ST_EXEC_PREP;
                    end
                    else begin
                        r_exec_active            <= 1'b0;
                        r_resp_status_latched     <= STATUS_SUCCESS;
                        r_resp_len_field_latched  <= 16'd0;
                        r_resp_data_count_latched <= 11'd0;
                        r_state                   <= ST_RESP_SEND;
                    end
                end
            end

            ST_RESP_COPY: begin
                o_resp_data_wr_en   <= 1'b1;
                o_resp_data_wr_addr <= r_copy_idx;
                o_resp_data_wr_byte <= r_resp_data_mem[r_copy_idx];

                if ({1'b0, r_copy_idx} == (r_resp_data_count_latched - 11'd1)) begin
                    r_state <= ST_RESP_SEND;
                end
                r_copy_idx <= r_copy_idx + 10'd1;
            end

            ST_RESP_SEND: begin
                if (i_resp_ready) begin
                    o_resp_valid      <= 1'b1;
                    o_resp_len_field  <= r_resp_len_field_latched;
                    o_resp_data_count <= r_resp_data_count_latched;
                    o_resp_status     <= r_resp_status_latched;
                    r_state           <= ST_IDLE;
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