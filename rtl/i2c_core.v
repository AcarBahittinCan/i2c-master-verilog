`timescale 1ns / 1ps

module i2c_core(

    input  wire        clk_i,
    input  wire        rst_i,

    input  wire [1:0]  nby_i,
    input  wire [6:0]  adr_i,
    output wire [31:0] rdr_o,
    input  wire [31:0] tdr_i,
    input  wire        tx_en,
    input  wire        rx_en,

    output reg         tx_done,
    output reg         rx_done,

    inout  wire        sda,
    inout  wire        scl

);

    localparam HALF_PERIOD = 125;

    localparam IDLE     = 4'd0;
    localparam START    = 4'd1;
    localparam ADDR     = 4'd2;
    localparam ACK_ADDR = 4'd3;
    localparam DATA_WR  = 4'd4;
    localparam ACK_WR   = 4'd5;
    localparam DATA_RD  = 4'd6;
    localparam ACK_RD   = 4'd7;
    localparam STOP     = 4'd8;

    reg [3:0]  state, state_nxt;
    reg [8:0]  counter, counter_nxt;       
    reg [2:0]  bit_cnt, bit_cnt_nxt;
    reg        scl_reg, scl_nxt;
    reg        sda_reg, sda_nxt;
    reg [31:0] shift_reg, shift_reg_nxt;
    reg [31:0] rdr, rdr_nxt;
    reg [1:0]  byte_cnt, byte_cnt_nxt;
    reg        tx_done_nxt, rx_done_nxt;
    reg        ack_reg, ack_reg_nxt;
    reg        arb_lost ,arb_lost_nxt;
    reg        rw_mode, rw_mode_nxt;
    reg [31:0] tdr_latched, tdr_latched_nxt;

    assign sda = sda_reg ? 1'bz : 1'b0;
    assign scl = scl_reg ? 1'bz : 1'b0;

    wire sda_i = sda;
    wire scl_i = scl;

    assign rdr_o = rdr;

    always @(posedge clk_i) begin
        if (rst_i) begin
            state     <= IDLE;
            counter   <= 0;
            bit_cnt   <= 7;
            scl_reg   <= 1;
            sda_reg   <= 1;
            shift_reg <= 0;
            rdr       <= 0;
            byte_cnt  <= 0;
            tx_done   <= 0;
            rx_done   <= 0;
            ack_reg   <= 0;
            arb_lost  <= 0;
            rw_mode   <= 0;
            tdr_latched <= 0;
        end else begin
            state     <= state_nxt;
            counter   <= counter_nxt;
            bit_cnt   <= bit_cnt_nxt;
            scl_reg   <= scl_nxt;
            sda_reg   <= sda_nxt;
            shift_reg <= shift_reg_nxt;
            rdr       <= rdr_nxt;
            byte_cnt  <= byte_cnt_nxt;
            tx_done   <= tx_done_nxt;
            rx_done   <= rx_done_nxt;
            ack_reg   <= ack_reg_nxt;
            arb_lost  <= arb_lost_nxt;
            rw_mode   <= rw_mode_nxt;
            tdr_latched <= tdr_latched_nxt;
        end
    end


    always @(*) begin

        state_nxt     = state;
        counter_nxt   = counter + 1;
        bit_cnt_nxt   = bit_cnt;
        scl_nxt       = scl_reg;
        sda_nxt       = sda_reg;
        shift_reg_nxt = shift_reg;
        rdr_nxt       = rdr;
        byte_cnt_nxt  = byte_cnt;
        tx_done_nxt   = tx_done;
        rx_done_nxt   = rx_done;
        ack_reg_nxt   = ack_reg;
        arb_lost_nxt  = arb_lost;
        rw_mode_nxt   = rw_mode;
        tdr_latched_nxt = tdr_latched;

        
        if(scl_reg == 1 && scl_i == 0)
            counter_nxt = counter;

       
        if(state != IDLE && state != START && state != STOP) begin
            if(counter < HALF_PERIOD)
                scl_nxt = 0;
            else
                scl_nxt = 1;
        end

        case(state)

        IDLE: begin
            scl_nxt      = 1;
            sda_nxt      = 1;
            counter_nxt  = 0;
            bit_cnt_nxt  = 7;
            byte_cnt_nxt = 0;

            if(tx_en) begin
                shift_reg_nxt   = tdr_i;
                tdr_latched_nxt = tdr_i;
                state_nxt       = START;
                rw_mode_nxt     = 0;
                arb_lost_nxt    = 0;           
            end
            else if(rx_en) begin
                 state_nxt     = START;
                rw_mode_nxt   = 1;
                rdr_nxt       = 0;
                arb_lost_nxt  = 0;
            end
        end


        START: begin
         
            scl_nxt     = 1;
            tx_done_nxt = 0;
            rx_done_nxt = 0;

            if(counter < HALF_PERIOD)
                sda_nxt = 1;
            else
                sda_nxt = 0;

            if(counter == 2*HALF_PERIOD-1) begin
                counter_nxt   = 0;
                scl_nxt       = 0;
                shift_reg_nxt = {24'b0, adr_i, rw_mode};
                state_nxt     = ADDR;
            end
        end


        ADDR: begin

            sda_nxt = shift_reg[7];
            
            if(scl_i == 1 && sda_reg == 1 && sda_i == 0) begin
                arb_lost_nxt = 1;
                state_nxt    = IDLE;
                sda_nxt      = 1;
                scl_nxt      = 1;
            end

            if(counter == 2*HALF_PERIOD-1) begin
            
                counter_nxt   = 0;
                scl_nxt       = 0;
                shift_reg_nxt = {shift_reg[30:0], 1'b0};

                if(bit_cnt == 0) begin
                    bit_cnt_nxt = 7;
                    state_nxt   = ACK_ADDR;
                end
                else
                    bit_cnt_nxt = bit_cnt - 1;
            end
        end


        ACK_ADDR: begin

            sda_nxt = 1;   

            if(counter == (HALF_PERIOD + HALF_PERIOD/2)&& scl_i == 1)
                ack_reg_nxt = sda_i;

            if(counter == 2*HALF_PERIOD-1) begin

                counter_nxt = 0;
                scl_nxt     = 0;

                if(ack_reg == 0) begin

                    if(rw_mode)
                        state_nxt = DATA_RD;
                    else begin
                        shift_reg_nxt = tdr_latched[7:0];
                        state_nxt     = DATA_WR;
                    end
                end
                else
                    state_nxt = STOP;
            end
        end


        DATA_WR: begin

            sda_nxt = shift_reg[7];

            if(scl_i == 1 && sda_reg == 1 && sda_i == 0) begin
                arb_lost_nxt = 1;
                state_nxt    = IDLE;
                sda_nxt      = 1;
                scl_nxt      = 1;
            end

            if(counter == 2*HALF_PERIOD-1) begin

                counter_nxt   = 0;
                scl_nxt       = 0;
                shift_reg_nxt = {shift_reg[30:0], 1'b0};

                if(bit_cnt == 0) begin
                    bit_cnt_nxt = 7;
                    state_nxt   = ACK_WR;
                end
                else
                    bit_cnt_nxt = bit_cnt - 1;
            end
        end


        ACK_WR: begin

            sda_nxt = 1;

            if(counter == (HALF_PERIOD + HALF_PERIOD/2) && scl_i == 1)
                ack_reg_nxt = sda_i;

            if(counter == 2*HALF_PERIOD-1) begin

                counter_nxt = 0;
                scl_nxt     = 0;

                if(ack_reg == 0) begin

                    if(byte_cnt == nby_i) begin
                        state_nxt   = STOP;
                    end
                    else begin
                        case(byte_cnt)
                            2'd0: shift_reg_nxt = {24'b0, tdr_latched[15:8]};
                            2'd1: shift_reg_nxt = {24'b0, tdr_latched[23:16]};
                            2'd2: shift_reg_nxt = {24'b0, tdr_latched[31:24]};
                            default: shift_reg_nxt = 0;
                        endcase
                        byte_cnt_nxt  = byte_cnt + 1;
                        state_nxt     = DATA_WR;
                    end
                end
                else
                    state_nxt = STOP;
            end
        end


        DATA_RD: begin

            sda_nxt = 1;   

            if(counter == (HALF_PERIOD + HALF_PERIOD/2) && scl_i == 1)
                shift_reg_nxt = {shift_reg[30:0], sda_i};

            if(counter == 2*HALF_PERIOD-1) begin

                counter_nxt = 0;
                scl_nxt     = 0;

                if(bit_cnt == 0) begin

                    bit_cnt_nxt = 7;
                    rdr_nxt     = rdr;
                    rdr_nxt[byte_cnt*8 +: 8] = shift_reg_nxt[7:0];
                    state_nxt   = ACK_RD;
                end
                else
                    bit_cnt_nxt = bit_cnt - 1;
            end
        end


        ACK_RD: begin

            sda_nxt = (byte_cnt == nby_i) ? 1 : 0;

            if(counter == 2*HALF_PERIOD-1) begin

                counter_nxt = 0;
                scl_nxt     = 0;

                if(byte_cnt == nby_i) begin
                    state_nxt   = STOP;
                end
                else begin
                    byte_cnt_nxt = byte_cnt + 1;
                    state_nxt    = DATA_RD;
                end
            end
        end


        STOP: begin

            if(counter < HALF_PERIOD) begin
                scl_nxt = 0;
                sda_nxt = 0;
            end
            else if(counter < 2*HALF_PERIOD) begin
                scl_nxt = 1;
                sda_nxt = 0;
            end
            else begin
                scl_nxt = 1;
                sda_nxt = 1;
            end

            if(counter == 3*HALF_PERIOD-1) begin
                counter_nxt = 0;
                state_nxt   = IDLE;
                if(rw_mode == 0)
                    tx_done_nxt = 1;
                else
                    rx_done_nxt = 1;
            end
        end

        default: state_nxt = IDLE;

        endcase
    end

endmodule