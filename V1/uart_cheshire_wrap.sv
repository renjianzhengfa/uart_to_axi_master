`include "cheshire/typedef.svh"

module uart_cheshire_wrap (
    input  logic clk_i,
    input  logic rst_ni,
    input  logic uart_rx_i,
    output logic uart_tx_o
);

  import cheshire_pkg::*;

  function automatic cheshire_cfg_t gen_cfg();
    cheshire_cfg_t cfg = DefaultCfg;
    cfg.AxiExtNumMst = 1;
    return cfg;
  endfunction

  localparam cheshire_cfg_t CheshireCfg = gen_cfg();

  `CHESHIRE_TYPEDEF_ALL(csh_, CheshireCfg)

  csh_axi_mst_req_t axi_ext_mst_req [0:CheshireCfg.AxiExtNumMst-1];
  csh_axi_mst_rsp_t axi_ext_mst_rsp [0:CheshireCfg.AxiExtNumMst-1];

  csh_axi_llc_req_t axi_llc_mst_req;

  cheshire_soc #(
    .Cfg               (CheshireCfg),
    .ExtHartinfo       ('0),
    .axi_ext_llc_req_t (csh_axi_llc_req_t),
    .axi_ext_llc_rsp_t (csh_axi_llc_rsp_t),
    .axi_ext_mst_req_t (csh_axi_mst_req_t),
    .axi_ext_mst_rsp_t (csh_axi_mst_rsp_t),
    .axi_ext_slv_req_t (csh_axi_slv_req_t),
    .axi_ext_slv_rsp_t (csh_axi_slv_rsp_t),
    .reg_ext_req_t     (csh_reg_req_t),
    .reg_ext_rsp_t     (csh_reg_rsp_t)
  ) dut (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .test_mode_i       (1'b0),
    .boot_mode_i       (2'b00),
    .rtc_i             (1'b0),

    .axi_llc_mst_req_o (axi_llc_mst_req),
    .axi_llc_mst_rsp_i ('0),

    .axi_ext_mst_req_i (axi_ext_mst_req),
    .axi_ext_mst_rsp_o (axi_ext_mst_rsp),

    .axi_ext_slv_req_o (),
    .axi_ext_slv_rsp_i ('0),
    .reg_ext_slv_req_o (),
    .reg_ext_slv_rsp_i ('0),

    .intr_ext_i        ('0),
    .intr_ext_o        (),
    .xeip_ext_o        (),
    .mtip_ext_o        (),
    .msip_ext_o        (),
    .dbg_active_o      (),
    .dbg_ext_req_o     (),
    .dbg_ext_unavail_i ('0),

    .jtag_tck_i        (1'b0),
    .jtag_trst_ni      (1'b1),
    .jtag_tms_i        (1'b0),
    .jtag_tdi_i        (1'b0),
    .jtag_tdo_o        (),
    .jtag_tdo_oe_o     (),

    .uart_tx_o         (),
    .uart_rx_i         (1'b1),
    .uart_rts_no       (),
    .uart_dtr_no       (),
    .uart_cts_ni       (1'b0),
    .uart_dsr_ni       (1'b0),
    .uart_dcd_ni       (1'b0),
    .uart_rin_ni       (1'b0),

    .i2c_sda_o         (),
    .i2c_sda_i         (1'b1),
    .i2c_sda_en_o      (),
    .i2c_scl_o         (),
    .i2c_scl_i         (1'b1),
    .i2c_scl_en_o      (),

    .spih_sck_o        (),
    .spih_sck_en_o     (),
    .spih_csb_o        (),
    .spih_csb_en_o     (),
    .spih_sd_o         (),
    .spih_sd_en_o      (),
    .spih_sd_i         ('0),

    .gpio_i            ('0),
    .gpio_o            (),
    .gpio_en_o         (),
    .slink_rcv_clk_i   ('0),
    .slink_rcv_clk_o   (),
    .slink_i           ('0),
    .slink_o           (),

    .vga_hsync_o       (),
    .vga_vsync_o       (),
    .vga_red_o         (),
    .vga_green_o       (),
    .vga_blue_o        (),

    .usb_clk_i         (1'b0),
    .usb_rst_ni        (1'b1),
    .usb_dm_i          (1'b0),
    .usb_dm_o          (),
    .usb_dm_oe_o       (),
    .usb_dp_i          (1'b0),
    .usb_dp_o          (),
    .usb_dp_oe_o       ()
  );

  uart_to_cheshire #(
    .SYS_CLK_FREQ_HZ      (50000000),
    .UART_BAUD_RATE       (115200),
    .AXI_ADDR_WIDTH       (48),
    .AXI_TIMEOUT_CYCLES   (1024),
    .MAX_FRAME_DATA_BYTES (1024),
    .AXI_FULL_AWCACHE     (4'b0011),
    .AXI_FULL_ARCACHE     (4'b0011),
    .AXI_FULL_AWPROT      (3'b000),
    .AXI_FULL_ARPROT      (3'b000),
    .AXI_FULL_AWQOS       (4'd0),
    .AXI_FULL_ARQOS       (4'd0),
    .AXI_DATA_WIDTH       (64),
    .AXI_DATA_BYTES       (8),
    .REG_MIN_ADDR         (48'h0000_0300_0000),
    .REG_MAX_ADDR         (48'h0000_08FF_FFFF),
    .SPM_C_MIN_ADDR       (48'h0000_1000_0000),
    .SPM_C_MAX_ADDR       (48'h0000_13FF_FFFF),
    .SPM_U_MIN_ADDR       (48'h0000_1400_0000),
    .SPM_U_MAX_ADDR       (48'h0000_17FF_FFFF)
  ) i_uart_bridge (
    .i_clk             (clk_i),
    .i_rst_n           (rst_ni),
    .i_uart_rx         (uart_rx_i),
    .o_uart_tx         (uart_tx_o),
    .o_axi_ext_mst_req (axi_ext_mst_req[0]),
    .i_axi_ext_mst_rsp (axi_ext_mst_rsp[0])
  );

endmodule
