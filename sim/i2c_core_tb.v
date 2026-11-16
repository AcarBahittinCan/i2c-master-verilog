`timescale 1ns / 1ps

module i2c_core_tb;

    // ================================================================
    //  Parameters
    // ================================================================
    localparam CLK_PERIOD   = 10;
    localparam HALF_PERIOD  = 125;
    localparam BIT_PERIOD   = 2 * HALF_PERIOD;

    // ================================================================
    //  DUT signals
    // ================================================================
    reg         clk_i;
    reg         rst_i;
    reg  [1:0]  nby_i;
    reg  [6:0]  adr_i;
    wire [31:0] rdr_o;
    reg  [31:0] tdr_i;
    reg         tx_en;
    reg         rx_en;
    wire        tx_done;
    wire        rx_done;

    wire sda, scl;
    reg  sda_slave;
    reg  scl_slave;

    assign sda = (sda_slave === 1'b0) ? 1'b0 : 1'bz;
    assign scl = (scl_slave === 1'b0) ? 1'b0 : 1'bz;

    pullup(sda);
    pullup(scl);

    // ================================================================
    //  DUT
    // ================================================================
    i2c_core dut (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .nby_i(nby_i),
        .adr_i(adr_i),
        .rdr_o(rdr_o),
        .tdr_i(tdr_i),
        .tx_en(tx_en),
        .rx_en(rx_en),
        .tx_done(tx_done),
        .rx_done(rx_done),
        .sda(sda),
        .scl(scl)
    );

    // ================================================================
    //  Counters
    // ================================================================
    integer test_num;
    integer pass_count;
    integer fail_count;
    integer total_tests;

    // ================================================================
    //  Clock
    // ================================================================
    initial clk_i = 0;
    always #(CLK_PERIOD/2) clk_i = ~clk_i;

    // ================================================================
    //  Helper tasks
    // ================================================================

    task reset_dut;
    begin
        rst_i     = 1;
        tx_en     = 0;
        rx_en     = 0;
        nby_i     = 0;
        adr_i     = 0;
        tdr_i     = 0;
        sda_slave = 1'bz;
        scl_slave = 1'bz;
        repeat(10) @(posedge clk_i);
        rst_i = 0;
        repeat(5) @(posedge clk_i);
    end
    endtask

    task wait_for_start;
        integer timeout;
    begin
        timeout = 0;
        while (timeout < 50000) begin
            @(negedge sda);
            if (scl === 1'b1) begin
                timeout = 50000;
            end
            timeout = timeout + 1;
        end
    end
    endtask

    task wait_scl_rise;
        integer timeout;
    begin
        timeout = 0;
        while (scl !== 1'b0 && timeout < 50000) begin
            @(posedge clk_i);
            timeout = timeout + 1;
        end
        timeout = 0;
        while (scl !== 1'b1 && timeout < 50000) begin
            @(posedge clk_i);
            timeout = timeout + 1;
        end
    end
    endtask

    task slave_receive_byte;
        output [7:0] received;
        integer i;
    begin
        received = 0;
        for (i = 7; i >= 0; i = i - 1) begin
            wait_scl_rise;
            repeat(5) @(posedge clk_i);
            received[i] = sda;
        end
    end
    endtask

    task slave_send_ack;
    begin
        while (scl !== 1'b0) @(posedge clk_i);
        sda_slave = 1'b0;
        wait_scl_rise;
        while (scl !== 1'b0) @(posedge clk_i);
        sda_slave = 1'bz;
    end
    endtask

    task slave_send_nack;
    begin
        while (scl !== 1'b0) @(posedge clk_i);
        sda_slave = 1'bz;
        wait_scl_rise;
        while (scl !== 1'b0) @(posedge clk_i);
    end
    endtask

    task slave_send_byte;
        input [7:0] data;
        integer i;
    begin
        for (i = 7; i >= 0; i = i - 1) begin
            while (scl !== 1'b0) @(posedge clk_i);
            if (data[i])
                sda_slave = 1'bz;
            else
                sda_slave = 1'b0;
            wait_scl_rise;
        end
        while (scl !== 1'b0) @(posedge clk_i);
        sda_slave = 1'bz;
    end
    endtask

    task wait_done;
        input integer max_cycles;
        integer cnt;
    begin
        cnt = 0;
        while (tx_done !== 1'b1 && rx_done !== 1'b1 && cnt < max_cycles) begin
            @(posedge clk_i);
            cnt = cnt + 1;
        end
        if (cnt >= max_cycles)
            $display("  [TIMEOUT] No done signal");
    end
    endtask

    task wait_idle;
        input integer max_cycles;
        integer cnt;
    begin
        cnt = 0;
        while (dut.state !== 4'd0 && cnt < max_cycles) begin
            @(posedge clk_i);
            cnt = cnt + 1;
        end
    end
    endtask

    task check_pass;
        input integer cond;
        input [512*8-1:0] msg;
    begin
        if (cond) begin
            $display("  [PASS] %0s", msg);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %0s", msg);
            fail_count = fail_count + 1;
        end
        total_tests = total_tests + 1;
    end
    endtask

    // ================================================================
    //  Main
    // ================================================================
    reg [7:0] cap_addr;
    reg [7:0] cap_data;
    reg [7:0] cap_data2;

    initial begin
        $display("================================================================");
        $display("  I2C Core Testbench");
        $display("================================================================");

        pass_count  = 0;
        fail_count  = 0;
        total_tests = 0;
        test_num    = 0;

        // ==========================================================
        // TEST 1: Reset
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Reset behavior ---", test_num);
        reset_dut;

        check_pass(tx_done === 1'b0, "tx_done = 0 after reset");
        check_pass(rx_done === 1'b0, "rx_done = 0 after reset");
        check_pass(rdr_o === 32'b0,  "rdr_o = 0 after reset");
        check_pass(sda === 1'b1,     "SDA high after reset");
        check_pass(scl === 1'b1,     "SCL high after reset");

        // ==========================================================
        // TEST 2: Single byte write (nby=0)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Single byte write (nby=0) ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'hAB; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr);
        check_pass(cap_addr === 8'hA0, "Addr byte = 0xA0");
        slave_send_ack;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hAB, "Data byte = 0xAB");
        slave_send_ack;

        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "tx_done asserted");
        check_pass(rx_done === 1'b0, "rx_done stays 0");

        // ==========================================================
        // TEST 3: Multi-byte write (nby=1, 2 bytes)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: 2-byte write (nby=1) ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h0000CDAB; nby_i = 2'd1;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hAB, "Byte 0 = 0xAB");
        slave_send_ack;

        slave_receive_byte(cap_data2);
        check_pass(cap_data2 === 8'hCD, "Byte 1 = 0xCD");
        slave_send_ack;

        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "tx_done after 2-byte write");

        // ==========================================================
        // TEST 4: Single byte read (nby=0)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Single byte read (nby=0) ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd0;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr);
        check_pass(cap_addr === 8'hA1, "Addr byte (read) = 0xA1");
        slave_send_ack;

        slave_send_byte(8'h37);
        wait_scl_rise;
        while (scl !== 1'b0) @(posedge clk_i);

        wait_done(100000); wait_idle(10000);
        check_pass(rx_done === 1'b1, "rx_done asserted");
        check_pass(tx_done === 1'b0, "tx_done stays 0");
        check_pass(rdr_o[7:0] === 8'h37, "rdr_o[7:0] = 0x37");

        // ==========================================================
        // TEST 5: Multi-byte read (nby=1, 2 bytes)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: 2-byte read (nby=1) ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd1;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        slave_send_byte(8'hAA);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        slave_send_byte(8'hBB);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        wait_done(100000); wait_idle(10000);
        check_pass(rx_done === 1'b1, "rx_done after 2-byte read");
        check_pass(rdr_o[7:0] === 8'hAA, "rdr_o[7:0] = 0xAA");
        check_pass(rdr_o[15:8] === 8'hBB, "rdr_o[15:8] = 0xBB");

        // ==========================================================
        // TEST 6: NACK on address
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: NACK on address ---", test_num);
        reset_dut;

        adr_i = 7'h70; tdr_i = 32'h55; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr);
        slave_send_nack;

        wait_done(100000); wait_idle(10000);
        check_pass(dut.state === 4'd0, "Returns to IDLE after addr NACK");

        // ==========================================================
        // TEST 7: NACK on data
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: NACK on data ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h42; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data);
        slave_send_nack;

        wait_done(100000); wait_idle(10000);
        check_pass(dut.state === 4'd0, "Returns to IDLE after data NACK");

        // ==========================================================
        // TEST 8: Back-to-back transfers
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Back-to-back transfers ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h11; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data); slave_send_ack;
        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "First transfer done");

        tdr_i = 32'h22;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        check_pass(tx_done === 1'b0, "tx_done cleared at START");

        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'h22, "Second transfer data = 0x22");
        slave_send_ack;
        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "Second transfer done");

        // ==========================================================
        // TEST 9: Clock stretching during write
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Clock stretching (write) ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'hFF; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;

        begin : cs_wr_first
            integer i;
            for (i = 0; i < 4; i = i + 1) begin
                wait_scl_rise;
                repeat(5) @(posedge clk_i);
            end
        end

        while (scl !== 1'b0) @(posedge clk_i);
        scl_slave = 1'b0;
        $display("  [INFO] Stretching clock 500 cycles");
        repeat(500) @(posedge clk_i);
        check_pass(scl === 1'b0, "SCL held low during stretch");
        scl_slave = 1'bz;

        begin : cs_wr_rest
            integer i;
            for (i = 0; i < 4; i = i + 1) begin
                wait_scl_rise;
                repeat(5) @(posedge clk_i);
            end
        end

        slave_send_ack;
        slave_receive_byte(cap_data);
        slave_send_ack;

        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "Transfer ok after clock stretch");

        // ==========================================================
        // TEST 10: Arbitration loss (ADDR)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Arbitration loss (ADDR) ---", test_num);
        reset_dut;

        adr_i = 7'h7F; tdr_i = 32'hFF; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        wait_scl_rise;
        repeat(3) @(posedge clk_i);
        sda_slave = 1'b0;
        repeat(20) @(posedge clk_i);

        wait_idle(10000);
        sda_slave = 1'bz;
        repeat(5) @(posedge clk_i);

        check_pass(dut.arb_lost === 1'b1, "arb_lost flag set");
        check_pass(dut.state === 4'd0, "Returns to IDLE");
        check_pass(sda === 1'b1, "SDA released");
        check_pass(scl === 1'b1, "SCL released");

        // ==========================================================
        // TEST 11: tx_done at STOP end
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: tx_done at STOP completion ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h99; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data); slave_send_ack;

        repeat(10) @(posedge clk_i);
        check_pass(dut.state === 4'd8, "In STOP state");
        check_pass(tx_done === 1'b0, "tx_done NOT set during STOP");

        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "tx_done set after STOP");

        // ==========================================================
        // TEST 12: rx_done at STOP end
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: rx_done at STOP completion ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd0;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_send_byte(8'hEE);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        repeat(10) @(posedge clk_i);
        check_pass(rx_done === 1'b0, "rx_done NOT set during STOP");

        wait_done(100000); wait_idle(10000);
        check_pass(rx_done === 1'b1, "rx_done set after STOP");
        check_pass(rdr_o[7:0] === 8'hEE, "rdr_o = 0xEE");

        // ==========================================================
        // TEST 13: Done flags cleared in START
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Done flags cleared in START ---", test_num);

        adr_i = 7'h50; tdr_i = 32'hAA; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        // Wait until core enters START state
        begin : wait_start_13
            integer wt;
            wt = 0;
            while (dut.state !== 4'd1 && wt < 10000) begin
                @(posedge clk_i);
                wt = wt + 1;
            end
        end
        repeat(5) @(posedge clk_i);

        check_pass(tx_done === 1'b0, "tx_done cleared in START");
        check_pass(rx_done === 1'b0, "rx_done cleared in START");

        // Wait until ADDR state starts, then handle slave side normally
        begin : wait_addr_13
            integer wt;
            wt = 0;
            while (dut.state !== 4'd2 && wt < 50000) begin
                @(posedge clk_i);
                wt = wt + 1;
            end
        end
        // Wait for first SCL rise in ADDR
        wait_scl_rise;
        // Now we're synced - but we missed bit 7. Receive remaining 7 bits
        // Just skip address check and complete the transfer
        begin : drain_addr_13
            integer i;
            for (i = 6; i >= 0; i = i - 1) begin
                wait_scl_rise;
                repeat(5) @(posedge clk_i);
            end
        end
        slave_send_ack;
        slave_receive_byte(cap_data); slave_send_ack;
        wait_done(100000); wait_idle(10000);

        // ==========================================================
        // TEST 14: TDR latch protection
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: TDR latch protection ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h0000BEEF; nby_i = 2'd1;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        tdr_i = 32'h0000DEAD;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hEF, "Byte 0 = 0xEF (not corrupted)");
        slave_send_ack;

        slave_receive_byte(cap_data2);
        check_pass(cap_data2 === 8'hBE, "Byte 1 = 0xBE (not corrupted)");
        slave_send_ack;

        wait_done(100000); wait_idle(10000);

        // ==========================================================
        // TEST 15: 4-byte write (nby=3)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: 4-byte write (nby=3) ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'hDEADBEEF; nby_i = 2'd3;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hEF, "Byte 0 = 0xEF");
        slave_send_ack;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hBE, "Byte 1 = 0xBE");
        slave_send_ack;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hAD, "Byte 2 = 0xAD");
        slave_send_ack;

        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hDE, "Byte 3 = 0xDE");
        slave_send_ack;

        wait_done(100000); wait_idle(10000);
        check_pass(tx_done === 1'b1, "tx_done after 4-byte write");

        // ==========================================================
        // TEST 16: 4-byte read (nby=3)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: 4-byte read (nby=3) ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd3;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        slave_send_byte(8'h11);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        slave_send_byte(8'h22);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        slave_send_byte(8'h33);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        slave_send_byte(8'h44);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);

        wait_done(100000); wait_idle(10000);
        check_pass(rx_done === 1'b1, "rx_done after 4-byte read");
        check_pass(rdr_o[7:0]   === 8'h11, "rdr_o[7:0] = 0x11");
        check_pass(rdr_o[15:8]  === 8'h22, "rdr_o[15:8] = 0x22");
        check_pass(rdr_o[23:16] === 8'h33, "rdr_o[23:16] = 0x33");
        check_pass(rdr_o[31:24] === 8'h44, "rdr_o[31:24] = 0x44");

        // ==========================================================
        // TEST 17: Bus idle state
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Bus idle after transfer ---", test_num);
        repeat(100) @(posedge clk_i);
        check_pass(sda === 1'b1, "SDA high in IDLE");
        check_pass(scl === 1'b1, "SCL high in IDLE");

        // ==========================================================
        // TEST 18: tx_en priority over rx_en
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: tx_en priority over rx_en ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h77; nby_i = 2'd0;
        tx_en = 1; rx_en = 1;
        @(posedge clk_i); @(posedge clk_i);
        tx_en = 0; rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr);
        check_pass(cap_addr[0] === 1'b0, "R/W bit = 0 (write, tx_en priority)");
        slave_send_ack;
        slave_receive_byte(cap_data); slave_send_ack;
        wait_done(100000); wait_idle(10000);

        // ==========================================================
        // TEST 19: rdr preserved during write
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: rdr preserved during write ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd0;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_send_byte(8'hCC);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);
        wait_done(100000); wait_idle(10000);
        check_pass(rdr_o[7:0] === 8'hCC, "rdr populated with 0xCC");

        adr_i = 7'h50; tdr_i = 32'h55; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data); slave_send_ack;
        wait_done(100000); wait_idle(10000);

        check_pass(rdr_o[7:0] === 8'hCC, "rdr preserved after write");

        // ==========================================================
        // TEST 20: Clock stretching during read
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Clock stretching (read) ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd0;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        begin : cs_rd_hi
            integer i;
            for (i = 7; i >= 4; i = i - 1) begin
                while (scl !== 1'b0) @(posedge clk_i);
                sda_slave = 1'bz;
                wait_scl_rise;
            end
        end

        while (scl !== 1'b0) @(posedge clk_i);
        scl_slave = 1'b0;
        $display("  [INFO] Stretching clock during read 300 cycles");
        repeat(300) @(posedge clk_i);
        scl_slave = 1'bz;

        begin : cs_rd_lo
            integer i;
            reg [3:0] nibble;
            nibble = 4'hA;
            for (i = 3; i >= 0; i = i - 1) begin
                while (scl !== 1'b0) @(posedge clk_i);
                if (nibble[i])
                    sda_slave = 1'bz;
                else
                    sda_slave = 1'b0;
                wait_scl_rise;
            end
        end

        while (scl !== 1'b0) @(posedge clk_i);
        sda_slave = 1'bz;

        wait_scl_rise;
        while (scl !== 1'b0) @(posedge clk_i);

        wait_done(100000); wait_idle(10000);
        check_pass(rx_done === 1'b1, "Read ok after clock stretch");
        check_pass(rdr_o[7:0] === 8'hFA, "rdr_o = 0xFA");

        // ==========================================================
        // TEST 21: Arbitration loss (DATA_WR)
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Arbitration loss (DATA_WR) ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'hFF; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;

        wait_scl_rise;
        repeat(3) @(posedge clk_i);
        sda_slave = 1'b0;

        repeat(50) @(posedge clk_i);
        wait_idle(10000);
        sda_slave = 1'bz;

        check_pass(dut.arb_lost === 1'b1, "arb_lost during DATA_WR");
        check_pass(dut.state === 4'd0, "Returns to IDLE");

        // ==========================================================
        // TEST 22: Address 0x00
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Address 0x00 ---", test_num);
        reset_dut;

        adr_i = 7'h00; tdr_i = 32'hAA; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr);
        check_pass(cap_addr === 8'h00, "Address 0x00 correct");
        slave_send_ack;
        slave_receive_byte(cap_data); slave_send_ack;
        wait_done(100000); wait_idle(10000);

        // ==========================================================
        // TEST 23: Data 0xFF
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Data 0xFF ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'hFF; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'hFF, "Data 0xFF correct");
        slave_send_ack;
        wait_done(100000); wait_idle(10000);

        // ==========================================================
        // TEST 24: Data 0x00
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Data 0x00 ---", test_num);
        reset_dut;

        adr_i = 7'h50; tdr_i = 32'h00; nby_i = 2'd0;
        tx_en = 1; @(posedge clk_i); @(posedge clk_i); tx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_receive_byte(cap_data);
        check_pass(cap_data === 8'h00, "Data 0x00 correct");
        slave_send_ack;
        wait_done(100000); wait_idle(10000);

        // ==========================================================
        // TEST 25: Read 0xFF
        // ==========================================================
        test_num = test_num + 1;
        $display("\n--- Test %0d: Read 0xFF ---", test_num);
        reset_dut;

        adr_i = 7'h50; nby_i = 2'd0;
        rx_en = 1; @(posedge clk_i); @(posedge clk_i); rx_en = 0;

        wait_for_start;
        slave_receive_byte(cap_addr); slave_send_ack;
        slave_send_byte(8'hFF);
        wait_scl_rise; while (scl !== 1'b0) @(posedge clk_i);
        wait_done(100000); wait_idle(10000);

        check_pass(rdr_o[7:0] === 8'hFF, "Read 0xFF correct");

        // ==========================================================
        // FINAL REPORT
        // ==========================================================
        $display("\n================================================================");
        $display("  TESTBENCH COMPLETE");
        $display("  Total : %0d", total_tests);
        $display("  PASS  : %0d", pass_count);
        $display("  FAIL  : %0d", fail_count);
        if (fail_count == 0)
            $display("  RESULT: *** ALL TESTS PASSED ***");
        else
            $display("  RESULT: *** %0d TEST(S) FAILED ***", fail_count);
        $display("================================================================");
        $finish;
    end

    // Watchdog
    initial begin
        #100000000;
        $display("\n[WATCHDOG] Timeout!");
        $display("  PASS: %0d  FAIL: %0d", pass_count, fail_count);
        $finish;
    end

endmodule