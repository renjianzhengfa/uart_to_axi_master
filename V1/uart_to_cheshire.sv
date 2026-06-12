`timescale 1ns / 1ps
`default_nettype none

module uart_to_cheshire #(
    parameter integer SYS_CLK_FREQ_HZ      = 50000000,
    parameter integer UART_BAUD_RATE       = 115200,
    parameter integer AXI_ADDR_WIDTH       = 48,
    parameter integer AXI_TIMEOUT_CYCLES   = 1024,
    parameter integer MAX_FRAME_DATA_BYTES = 1024,
    parameter [3:0]   AXI_FULL_AWCACHE     = 4'b0011,
    parameter [3:0]   AXI_FULL_ARCACHE     = 4'b0011,
    parameter [2:0]   AXI_FULL_AWPROT      = 3'b000,
    parameter [2:0]   AXI_FULL_ARPROT      = 3'b000,
    parameter [3:0]   AXI_FULL_AWQOS       = 4'd0,
    parameter [3:0]   AXI_FULL_ARQOS       = 4'd0,
    parameter  AXI_DATA_WIDTH       = 64,
    parameter  AXI_DATA_BYTES       = AXI_DATA_WIDTH / 8,
    parameter [AXI_ADDR_WIDTH-1:0]  REG_MIN_ADDR         = 63'h0300_0000,
    parameter [AXI_ADDR_WIDTH-1:0]  REG_MAX_ADDR         = 63'h08FF_FFFF,
    parameter [AXI_ADDR_WIDTH-1:0]  SPM_C_MIN_ADDR       = 63'h1000_0000,
    parameter [AXI_ADDR_WIDTH-1:0]  SPM_C_MAX_ADDR       = 63'h13FF_FFFF,
    parameter [AXI_ADDR_WIDTH-1:0]  SPM_U_MIN_ADDR       = 63'h1400_0000,
    parameter [AXI_ADDR_WIDTH-1:0]  SPM_U_MAX_ADDR       = 63'h17FF_FFFF,
    parameter [AXI_ADDR_WIDTH-1:0] DDR_MIN_ADDR             = 48'h8000_0000,
    parameter [AXI_ADDR_WIDTH-1:0] DDR_MAX_ADDR             = 48'hFFFF_FFFF
) (
    i_clk,
    i_rst_n,
    i_uart_rx,
    o_uart_tx,
    o_axi_ext_mst_req,
    i_axi_ext_mst_rsp
);

    typedef logic [AXI_ADDR_WIDTH-1:0] addr_t;
    typedef logic [1:0]                id_t  ;
    typedef logic [1:0]                user_t;
    typedef logic [AXI_DATA_WIDTH-1:0] data_t;
    typedef logic [AXI_DATA_BYTES-1:0] strb_t;

    typedef logic [7:0] axi_len_t;
    typedef logic [2:0] axi_size_t;
    typedef logic [1:0] axi_burst_t;
    typedef logic [3:0] axi_cache_t;
    typedef logic [2:0] axi_prot_t;
    typedef logic [3:0] axi_qos_t;
    typedef logic [3:0] axi_region_t;
    typedef logic [5:0] axi_atop_t;
    typedef logic [1:0] axi_resp_t;

    typedef struct packed {
        id_t        id;
        addr_t      addr;
        axi_len_t   len;
        axi_size_t  size;
        axi_burst_t burst;
        logic       lock;
        axi_cache_t cache;
        axi_prot_t  prot;
        axi_qos_t   qos;
        axi_region_t region;
        axi_atop_t  atop;
        user_t      user;
    } aw_chan_t;

    typedef struct packed {
        data_t data;
        strb_t strb;
        logic  last;
        user_t user;
    } w_chan_t;

    typedef struct packed {
        id_t       id;
        axi_resp_t resp;
        user_t     user;
    } b_chan_t;

    typedef struct packed {
        id_t        id;
        addr_t      addr;
        axi_len_t   len;
        axi_size_t  size;
        axi_burst_t burst;
        logic       lock;
        axi_cache_t cache;
        axi_prot_t  prot;
        axi_qos_t   qos;
        axi_region_t region;
        user_t      user;
    } ar_chan_t;

    typedef struct packed {
        id_t       id;
        data_t     data;
        axi_resp_t resp;
        logic      last;
        user_t     user;
    } r_chan_t;

    typedef struct packed {
        aw_chan_t aw;
        logic     aw_valid;
        w_chan_t  w;
        logic     w_valid;
        logic     b_ready;
        ar_chan_t ar;
        logic     ar_valid;
        logic     r_ready;
    } req_t;

    typedef struct packed {
        logic    aw_ready;
        logic    ar_ready;
        logic    w_ready;
        logic    b_valid;
        b_chan_t b;
        logic    r_valid;
        r_chan_t r;
    } resp_t;

    input  wire  i_clk;
    input  wire  i_rst_n;
    input  wire  i_uart_rx;
    output wire  o_uart_tx;

    output req_t  o_axi_ext_mst_req;
    input  resp_t i_axi_ext_mst_rsp;

    // ---------------------------------------------------------------------
    // Flat AXI wires between uart_axi_bridge and local req/rsp pack.
    // ---------------------------------------------------------------------
    wire [AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]                m_axi_awlen;
    wire [2:0]                m_axi_awsize;
    wire [1:0]                m_axi_awburst;
    wire                      m_axi_awlock;
    wire [3:0]                m_axi_awcache;
    wire [2:0]                m_axi_awprot;
    wire [3:0]                m_axi_awqos;
    wire                      m_axi_awvalid;
    wire                      m_axi_awready;

    wire [AXI_DATA_WIDTH-1:0] m_axi_wdata;
    wire [AXI_DATA_BYTES-1:0] m_axi_wstrb;
    wire                      m_axi_wlast;
    wire                      m_axi_wvalid;
    wire                      m_axi_wready;

    wire [1:0]                m_axi_bresp;
    wire                      m_axi_bvalid;
    wire                      m_axi_bready;

    wire [AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]                m_axi_arlen;
    wire [2:0]                m_axi_arsize;
    wire [1:0]                m_axi_arburst;
    wire                      m_axi_arlock;
    wire [3:0]                m_axi_arcache;
    wire [2:0]                m_axi_arprot;
    wire [3:0]                m_axi_arqos;
    wire                      m_axi_arvalid;
    wire                      m_axi_arready;

    wire [AXI_DATA_WIDTH-1:0] m_axi_rdata;
    wire [1:0]                m_axi_rresp;
    wire                      m_axi_rlast;
    wire                      m_axi_rvalid;
    wire                      m_axi_rready;

    // ---------------------------------------------------------------------
    // UART protocol + bridge + AXI master core
    // ---------------------------------------------------------------------
    uart_axi_bridge #(
        .SYS_CLK_FREQ_HZ      (SYS_CLK_FREQ_HZ),
        .UART_BAUD_RATE       (UART_BAUD_RATE),
        .AXI_ADDR_WIDTH       (AXI_ADDR_WIDTH),
        .AXI_TIMEOUT_CYCLES   (AXI_TIMEOUT_CYCLES),
        .MAX_FRAME_DATA_BYTES (MAX_FRAME_DATA_BYTES),
        .AXI_FULL_AWCACHE     (AXI_FULL_AWCACHE),
        .AXI_FULL_ARCACHE     (AXI_FULL_ARCACHE),
        .AXI_FULL_AWPROT      (AXI_FULL_AWPROT),
        .AXI_FULL_ARPROT      (AXI_FULL_ARPROT),
        .AXI_FULL_AWQOS       (AXI_FULL_AWQOS),
        .AXI_FULL_ARQOS       (AXI_FULL_ARQOS),
        .AXI_DATA_BYTES      (AXI_DATA_BYTES),
        .REG_MIN_ADDR        (REG_MIN_ADDR  ),
        .REG_MAX_ADDR        (REG_MAX_ADDR  ),
        .SPM_C_MIN_ADDR      (SPM_C_MIN_ADDR),
        .SPM_C_MAX_ADDR      (SPM_C_MAX_ADDR),
        .SPM_U_MIN_ADDR      (SPM_U_MIN_ADDR),
        .SPM_U_MAX_ADDR      (SPM_U_MAX_ADDR),
        .DDR_MIN_ADDR       (DDR_MIN_ADDR),
        .DDR_MAX_ADDR       (DDR_MAX_ADDR)
    ) u_uart_axi_bridge (
        .i_clk           (i_clk),
        .i_rst_n         (i_rst_n),
        .i_uart_rx       (i_uart_rx),
        .o_uart_tx       (o_uart_tx),

        .o_m_axi_awaddr  (m_axi_awaddr),
        .o_m_axi_awlen   (m_axi_awlen),
        .o_m_axi_awsize  (m_axi_awsize),
        .o_m_axi_awburst (m_axi_awburst),
        .o_m_axi_awlock  (m_axi_awlock),
        .o_m_axi_awcache (m_axi_awcache),
        .o_m_axi_awprot  (m_axi_awprot),
        .o_m_axi_awqos   (m_axi_awqos),
        .o_m_axi_awvalid (m_axi_awvalid),
        .i_m_axi_awready (m_axi_awready),

        .o_m_axi_wdata   (m_axi_wdata),
        .o_m_axi_wstrb   (m_axi_wstrb),
        .o_m_axi_wlast   (m_axi_wlast),
        .o_m_axi_wvalid  (m_axi_wvalid),
        .i_m_axi_wready  (m_axi_wready),

        .i_m_axi_bresp   (m_axi_bresp),
        .i_m_axi_bvalid  (m_axi_bvalid),
        .o_m_axi_bready  (m_axi_bready),

        .o_m_axi_araddr  (m_axi_araddr),
        .o_m_axi_arlen   (m_axi_arlen),
        .o_m_axi_arsize  (m_axi_arsize),
        .o_m_axi_arburst (m_axi_arburst),
        .o_m_axi_arlock  (m_axi_arlock),
        .o_m_axi_arcache (m_axi_arcache),
        .o_m_axi_arprot  (m_axi_arprot),
        .o_m_axi_arqos   (m_axi_arqos),
        .o_m_axi_arvalid (m_axi_arvalid),
        .i_m_axi_arready (m_axi_arready),

        .i_m_axi_rdata   (m_axi_rdata),
        .i_m_axi_rresp   (m_axi_rresp),
        .i_m_axi_rlast   (m_axi_rlast),
        .i_m_axi_rvalid  (m_axi_rvalid),
        .o_m_axi_rready  (m_axi_rready)
    );

    // ---------------------------------------------------------------------
    // Pack flat AXI -> local req_t
    // ---------------------------------------------------------------------
    always @* begin
        o_axi_ext_mst_req = '0;

        // AW
        o_axi_ext_mst_req.aw_valid   = m_axi_awvalid;
        o_axi_ext_mst_req.aw.id      = '0;
        o_axi_ext_mst_req.aw.addr    = m_axi_awaddr;
        o_axi_ext_mst_req.aw.len     = m_axi_awlen;
        o_axi_ext_mst_req.aw.size    = m_axi_awsize;
        o_axi_ext_mst_req.aw.burst   = m_axi_awburst;
        o_axi_ext_mst_req.aw.lock    = m_axi_awlock;
        o_axi_ext_mst_req.aw.cache   = m_axi_awcache;
        o_axi_ext_mst_req.aw.prot    = m_axi_awprot;
        o_axi_ext_mst_req.aw.qos     = m_axi_awqos;
        o_axi_ext_mst_req.aw.region  = '0;
        o_axi_ext_mst_req.aw.atop    = '0;
        o_axi_ext_mst_req.aw.user    = '0;

        // W
        o_axi_ext_mst_req.w_valid    = m_axi_wvalid;
        o_axi_ext_mst_req.w.data     = m_axi_wdata;
        o_axi_ext_mst_req.w.strb     = m_axi_wstrb;
        o_axi_ext_mst_req.w.last     = m_axi_wlast;
        o_axi_ext_mst_req.w.user     = '0;

        // B
        o_axi_ext_mst_req.b_ready    = m_axi_bready;

        // AR
        o_axi_ext_mst_req.ar_valid   = m_axi_arvalid;
        o_axi_ext_mst_req.ar.id      = '0;
        o_axi_ext_mst_req.ar.addr    = m_axi_araddr;
        o_axi_ext_mst_req.ar.len     = m_axi_arlen;
        o_axi_ext_mst_req.ar.size    = m_axi_arsize;
        o_axi_ext_mst_req.ar.burst   = m_axi_arburst;
        o_axi_ext_mst_req.ar.lock    = m_axi_arlock;
        o_axi_ext_mst_req.ar.cache   = m_axi_arcache;
        o_axi_ext_mst_req.ar.prot    = m_axi_arprot;
        o_axi_ext_mst_req.ar.qos     = m_axi_arqos;
        o_axi_ext_mst_req.ar.region  = '0;
        o_axi_ext_mst_req.ar.user    = '0;

        // R
        o_axi_ext_mst_req.r_ready    = m_axi_rready;
    end

    // ---------------------------------------------------------------------
    // Unpack local resp_t -> flat AXI
    // ---------------------------------------------------------------------
    assign m_axi_awready = i_axi_ext_mst_rsp.aw_ready;
    assign m_axi_wready  = i_axi_ext_mst_rsp.w_ready;
    assign m_axi_bvalid  = i_axi_ext_mst_rsp.b_valid;
    assign m_axi_bresp   = i_axi_ext_mst_rsp.b.resp;

    assign m_axi_arready = i_axi_ext_mst_rsp.ar_ready;
    assign m_axi_rvalid  = i_axi_ext_mst_rsp.r_valid;
    assign m_axi_rdata   = i_axi_ext_mst_rsp.r.data;
    assign m_axi_rresp   = i_axi_ext_mst_rsp.r.resp;
    assign m_axi_rlast   = i_axi_ext_mst_rsp.r.last;

endmodule

`default_nettype wire
