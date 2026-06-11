`timescale 1ns / 1ps
`default_nettype none

module tb_cheshire_chs;

initial begin
    $fsdbDumpfile("uart_master.fsdb");
    $fsdbDumpvars(0, tb_cheshire_chs, "+all");
    $fsdbDumpMDA();
    $fsdbDumpflush;
end

logic r_clk;
logic r_rst_n;
logic r_uart_rx;
logic w_uart_tx;

logic [7:0] tx_buf  [0:4095];
logic [7:0] rx_buf  [0:4095];
logic [7:0] exp_buf [0:4095];
logic [7:0] data_buf[0:2047];

integer tx_cnt;
integer rx_cnt;
integer exp_cnt;
integer case_cnt;
integer fail_cnt;
integer i;

localparam integer CLK_PERIOD_NS      = 20;
localparam integer TB_SYS_CLK_FREQ_HZ = 50_000_000;
// Keep this aligned with uart_cheshire_wrap -> uart_to_cheshire current setting.
localparam integer TB_UART_BAUD_RATE  = 115200;
localparam integer TB_CLKS_PER_BIT    = (TB_SYS_CLK_FREQ_HZ + (TB_UART_BAUD_RATE/2)) / TB_UART_BAUD_RATE;
localparam integer RX_WAIT_CYCLES     = 2_000_000;

localparam [63:0] REG_ADDR_BASE       = 64'h0000_0000_0200_0010;
localparam [63:0] MEM_ADDR_BASE       = 64'h0000_0000_1400_0100;
localparam [63:0] BAD_ADDR_BASE       = 64'h0000_0001_8000_0000;

localparam [7:0] CMD_REG_WRITE = 8'h01;
localparam [7:0] CMD_REG_READ  = 8'h02;
localparam [7:0] CMD_MEM_WRITE = 8'h03;
localparam [7:0] CMD_MEM_READ  = 8'h04;

localparam [7:0] STATUS_SUCCESS   = 8'h00;
localparam [7:0] STATUS_ALIGN_ERR = 8'h01;
localparam [7:0] STATUS_AXI_ERR   = 8'h02;
localparam [7:0] STATUS_LEN_ERR   = 8'h03;
localparam [7:0] STATUS_CMD_ERR   = 8'h04;
localparam [7:0] STATUS_RANGE_ERR = 8'h05;

uart_cheshire_wrap dut (
    .clk_i     (r_clk),
    .rst_ni    (r_rst_n),
    .uart_rx_i (r_uart_rx),
    .uart_tx_o (w_uart_tx)
);

logic       rx_dv;
logic [7:0] rx_byte;
uart_rx #(
    .CLKS_PER_BIT(TB_CLKS_PER_BIT)
) u_tb_uart_rx (
    .i_clk      (r_clk),
    .i_rst      (!r_rst_n),
    .i_rx_serial(w_uart_tx),
    .o_rx_dv    (rx_dv),
    .o_rx_byte  (rx_byte)
);

always #(CLK_PERIOD_NS/2) r_clk = ~r_clk;

always_ff @(posedge r_clk) begin
    if (!r_rst_n) begin
        rx_cnt <= 0;
    end else if (rx_dv) begin
        rx_buf[rx_cnt] <= rx_byte;
        rx_cnt <= rx_cnt + 1;
    end
end

task automatic wait_clks(input integer n);
    integer j;
    begin
        for (j = 0; j < n; j = j + 1) @(posedge r_clk);
    end
endtask

task automatic uart_send_byte(input [7:0] data_byte);
    integer bit_idx;
    begin
        r_uart_rx = 1'b0;
        wait_clks(TB_CLKS_PER_BIT);
        for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
            r_uart_rx = data_byte[bit_idx];
            wait_clks(TB_CLKS_PER_BIT);
        end
        r_uart_rx = 1'b1;
        wait_clks(TB_CLKS_PER_BIT);
    end
endtask

task automatic send_tx_buf;
    integer j;
    begin
        for (j = 0; j < tx_cnt; j = j + 1)
            uart_send_byte(tx_buf[j]);
        wait_clks(TB_CLKS_PER_BIT * 2);
    end
endtask

task automatic clear_rx_buf;
    integer j;
    begin
        rx_cnt = 0;
        for (j = 0; j < 4096; j = j + 1)
            rx_buf[j] = 8'h00;
    end
endtask

task automatic clear_data_buf;
    integer j;
    begin
        for (j = 0; j < 2048; j = j + 1)
            data_buf[j] = 8'h00;
    end
endtask

task automatic clear_exp_buf;
    integer j;
    begin
        exp_cnt = 0;
        for (j = 0; j < 4096; j = j + 1)
            exp_buf[j] = 8'h00;
    end
endtask

// Assumes your CURRENT parser protocol is already 64-bit address:
// AA | len_h | len_l | cmd | addr[63:56] ... addr[7:0] | payload | chk | 55
// If your parser is still 40-bit, change this task back to 5 address bytes.
task automatic build_request(
    input [15:0] req_len,
    input [7:0]  req_cmd,
    input [63:0] req_addr,
    input integer payload_count
);
    integer j;
    reg [7:0] chk;
    begin
        tx_cnt = 0;
        tx_buf[tx_cnt] = 8'hAA; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_len[15:8]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_len[7:0];  tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_cmd;       tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[63:56]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[55:48]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[47:40]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[39:32]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[31:24]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[23:16]; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[15:8];  tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = req_addr[7:0];   tx_cnt = tx_cnt + 1;
        for (j = 0; j < payload_count; j = j + 1) begin
            tx_buf[tx_cnt] = data_buf[j];
            tx_cnt = tx_cnt + 1;
        end
        chk = 8'h00;
        for (j = 0; j < tx_cnt; j = j + 1)
            chk = chk ^ tx_buf[j];
        tx_buf[tx_cnt] = chk; tx_cnt = tx_cnt + 1;
        tx_buf[tx_cnt] = 8'h55; tx_cnt = tx_cnt + 1;
    end
endtask

// Assumes current builder response is:
// AA | len_h | len_l | cmd | addr[63:56] ... addr[7:0] | status | payload | chk | 55
// If your builder is still 40-bit, change this task back to 5 address bytes.
task automatic build_expected_response(
    input [15:0] resp_len,
    input [7:0]  resp_cmd,
    input [63:0] resp_addr,
    input [7:0]  resp_status,
    input integer payload_count
);
    integer j;
    reg [7:0] chk;
    begin
        clear_exp_buf();
        exp_buf[exp_cnt] = 8'hAA; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_len[15:8]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_len[7:0];  exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_cmd;       exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[63:56]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[55:48]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[47:40]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[39:32]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[31:24]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[23:16]; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[15:8];  exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_addr[7:0];   exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = resp_status;      exp_cnt = exp_cnt + 1;
        for (j = 0; j < payload_count; j = j + 1) begin
            exp_buf[exp_cnt] = data_buf[j];
            exp_cnt = exp_cnt + 1;
        end
        chk = 8'h00;
        for (j = 0; j < exp_cnt; j = j + 1)
            chk = chk ^ exp_buf[j];
        exp_buf[exp_cnt] = chk; exp_cnt = exp_cnt + 1;
        exp_buf[exp_cnt] = 8'h55; exp_cnt = exp_cnt + 1;
    end
endtask

task automatic wait_rx_count(input integer target_cnt, input [8*48-1:0] tag);
    integer cyc;
    begin
        cyc = 0;
        while ((rx_cnt < target_cnt) && (cyc < RX_WAIT_CYCLES)) begin
            @(posedge r_clk);
            cyc = cyc + 1;
        end
        if (rx_cnt < target_cnt) begin
            $display("[FAIL] %0s timeout waiting response, got %0d expected %0d", tag, rx_cnt, target_cnt);
            fail_cnt = fail_cnt + 1;
        end
    end
endtask

task automatic compare_response(input [8*48-1:0] tag);
    integer j;
    reg mismatch;
    begin
        mismatch = 1'b0;
        if (rx_cnt != exp_cnt) begin
            mismatch = 1'b1;
        end else begin
            for (j = 0; j < exp_cnt; j = j + 1) begin
                if (rx_buf[j] !== exp_buf[j]) mismatch = 1'b1;
            end
        end

        if (mismatch) begin
            $display("[FAIL] %0s response mismatch", tag);
            $write("  expected:");
            for (j = 0; j < exp_cnt; j = j + 1) $write(" %02x", exp_buf[j]);
            $write("\n  actual  :");
            for (j = 0; j < rx_cnt; j = j + 1) $write(" %02x", rx_buf[j]);
            $write("\n");
            fail_cnt = fail_cnt + 1;
        end else begin
            $display("[PASS] %0s", tag);
        end
    end
endtask

task automatic expect_no_response(input [8*48-1:0] tag);
    begin
        wait_clks(RX_WAIT_CYCLES/8);
        if (rx_cnt != 0) begin
            $display("[FAIL] %0s expected no response, got %0d bytes", tag, rx_cnt);
            fail_cnt = fail_cnt + 1;
        end else begin
            $display("[PASS] %0s", tag);
        end
    end
endtask

initial begin
    r_clk    = 1'b0;
    r_rst_n  = 1'b0;
    r_uart_rx= 1'b1;
    tx_cnt   = 0;
    rx_cnt   = 0;
    exp_cnt  = 0;
    case_cnt = 0;
    fail_cnt = 0;

    clear_data_buf();
    clear_rx_buf();
    clear_exp_buf();

    wait_clks(20);
    r_rst_n = 1'b1;
    wait_clks(200);

    // ---------------------------------------------------------
    // 1) mem write 16B
    // ---------------------------------------------------------
    case_cnt = case_cnt + 1;
    clear_rx_buf();
    clear_data_buf();
    data_buf[0]=8'hA3; data_buf[1]=8'hA2; data_buf[2]=8'hA1; data_buf[3]=8'hA0;
    data_buf[4]=8'hB3; data_buf[5]=8'hB2; data_buf[6]=8'hB1; data_buf[7]=8'hB0;
    data_buf[8]=8'hC3; data_buf[9]=8'hC2; data_buf[10]=8'hC1; data_buf[11]=8'hC0;
    data_buf[12]=8'hD3; data_buf[13]=8'hD2; data_buf[14]=8'hD1; data_buf[15]=8'hD0;
    build_request(16'd16, CMD_MEM_WRITE, MEM_ADDR_BASE, 16);
    build_expected_response(16'd0, CMD_MEM_WRITE, MEM_ADDR_BASE, STATUS_SUCCESS, 0);
    send_tx_buf();
    wait_rx_count(exp_cnt, "mem_write_16_ok");
    compare_response("mem_write_16_ok");
    wait_clks(2000);

    // ---------------------------------------------------------
    // 2) mem read 16B back
    // ---------------------------------------------------------
    case_cnt = case_cnt + 1;
    clear_rx_buf();
    clear_data_buf();
    data_buf[0]=8'hA3; data_buf[1]=8'hA2; data_buf[2]=8'hA1; data_buf[3]=8'hA0;
    data_buf[4]=8'hB3; data_buf[5]=8'hB2; data_buf[6]=8'hB1; data_buf[7]=8'hB0;
    data_buf[8]=8'hC3; data_buf[9]=8'hC2; data_buf[10]=8'hC1; data_buf[11]=8'hC0;
    data_buf[12]=8'hD3; data_buf[13]=8'hD2; data_buf[14]=8'hD1; data_buf[15]=8'hD0;
    build_request(16'd16, CMD_MEM_READ, MEM_ADDR_BASE, 0);
    build_expected_response(16'd16, CMD_MEM_READ, MEM_ADDR_BASE, STATUS_SUCCESS, 16);
    send_tx_buf();
    wait_rx_count(exp_cnt, "mem_read_16_ok");
    compare_response("mem_read_16_ok");
    wait_clks(2000);

    // ---------------------------------------------------------
    // 3) mem write 256B
    // ---------------------------------------------------------
    case_cnt = case_cnt + 1;
    clear_rx_buf();
    clear_data_buf();
    for (i = 0; i < 256; i = i + 1) data_buf[i] = i[7:0];
    build_request(16'd256, CMD_MEM_WRITE, 64'h0000_0000_1400_0000, 256);
    build_expected_response(16'd0, CMD_MEM_WRITE, 64'h0000_0000_1400_0000, STATUS_SUCCESS, 0);
    send_tx_buf();
    wait_rx_count(exp_cnt, "mem_write_256_ok");
    compare_response("mem_write_256_ok");
    wait_clks(4000);

    // ---------------------------------------------------------
    // 4) mem read 256B back
    // ---------------------------------------------------------
    case_cnt = case_cnt + 1;
    clear_rx_buf();
    clear_data_buf();
    for (i = 0; i < 256; i = i + 1) data_buf[i] = i[7:0];
    build_request(16'd256, CMD_MEM_READ, 64'h0000_0000_1400_0000, 0);
    build_expected_response(16'd256, CMD_MEM_READ, 64'h0000_0000_1400_0000, STATUS_SUCCESS, 256);
    send_tx_buf();
    wait_rx_count(exp_cnt, "mem_read_256_ok");
    compare_response("mem_read_256_ok");
    wait_clks(4000);

    // ---------------------------------------------------------
    // 5) bad checksum should be dropped
    // ---------------------------------------------------------
    case_cnt = case_cnt + 1;
    clear_rx_buf();
    clear_data_buf();
    build_request(16'd16, CMD_MEM_READ, MEM_ADDR_BASE, 0);
    tx_buf[tx_cnt-2] = tx_buf[tx_cnt-2] ^ 8'hFF;
    send_tx_buf();
    expect_no_response("checksum_drop");
    wait_clks(1000);

    // ---------------------------------------------------------
    // 6) range error case
    // ---------------------------------------------------------
    case_cnt = case_cnt + 1;
    clear_rx_buf();
    clear_data_buf();
    build_request(16'd16, CMD_MEM_READ, BAD_ADDR_BASE, 0);
    build_expected_response(16'd0, CMD_MEM_READ, BAD_ADDR_BASE, STATUS_RANGE_ERR, 0);
    send_tx_buf();
    wait_rx_count(exp_cnt, "range_err");
    compare_response("range_err");

    $display("Cases: %0d Failures: %0d", case_cnt, fail_cnt);
    if (fail_cnt == 0) begin
        $display("TB PASS");
    end else begin
        $fatal(1, "TB FAIL");
    end

    $finish;
end

endmodule

`default_nettype wire