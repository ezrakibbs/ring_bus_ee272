// ============================================================================
// dd.sv  --  EE226 Fall 26 ring-bus timer/counter device
//
// baby.sv does:   `include <dd.sv>
//                 dd n0(cr.clkresi, ringt.rin, ringd.rout, 7'b0,
//                       pps.ppdevo, pps.ppdevi);
//
// so this file provides module "dd" (interface wrapper) around
// "dd_timer_core" (plain-port design, easy to synthesise / unit test).
//
// Ring-bus summary (from the project spec)
//   * 2-bit Cmd (0 Idle, 1 Start, 2 Continue, 3 End) + 8-bit Data per cycle
//   * Packet bytes : [Tok|Dest(7)] [0|Src(7)] [Cmd(3)|Reg/Aux(5)] [data...]
//   * Token passing: a device may only start a packet when it owns the token
//   * Data is little endian, registers are 64 bits (8 bytes)
//
// Register map (all 64 bit)
//   0 : [47:0] accumulator, [63] accumulate enable, [62] stop on overflow
//   1 : [47:0] phase increment
//   2 : [48:0] pulse0 "high" limit,  [63] pulse0 output state (read only)
//   3 : [48:0] pulse0 "low"  limit
//   4 : [48:0] pulse1 "high" limit,  [63] pulse1 output state (read only)
//   5 : [48:0] pulse1 "low"  limit
//   6 : [47:0] external counter, [50:48] count-condition select
//   7 : [3:0]  interrupt bits (0 P0 LtoH, 1 P1 LtoH, 2 counter sign, 3 acc ovf)
// ============================================================================
`ifndef DD_SV_INCLUDED
`define DD_SV_INCLUDED
`define DD_DEBUG

// ----------------------------------------------------------------------------
// Core design (no interfaces)
// ----------------------------------------------------------------------------
module dd_timer_core #(
    parameter int RESP_DEPTH = 8        // queued read/write responses (power of 2)
) (
    input  logic       clk,
    input  logic       reset,           // synchronous, active high

    // ring bus
    input  logic [1:0] in_cmd,
    input  logic [7:0] in_data,
    output logic [1:0] out_cmd,
    output logic [7:0] out_data,

    input  logic [6:0] addr,            // this device's ring address

    // pins
    input  logic       ce,
    output logic       p0_out,
    output logic       p1_out
);

    // ------------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------------
    localparam logic [1:0] C_IDLE  = 2'd0;
    localparam logic [1:0] C_START = 2'd1;
    localparam logic [1:0] C_CONT  = 2'd2;
    localparam logic [1:0] C_END   = 2'd3;

    localparam logic [2:0] CMD_RREQ = 3'd0;   // ReadRequest
    localparam logic [2:0] CMD_RRSP = 3'd1;   // ReadResponse
    localparam logic [2:0] CMD_WREQ = 3'd2;   // WriteRequest
    localparam logic [2:0] CMD_WRSP = 3'd3;   // WriteResponse
    localparam logic [2:0] CMD_IREQ = 3'd4;   // InterruptRequest
    localparam logic [2:0] CMD_IRSP = 3'd5;   // InterruptResponse

    localparam logic [6:0] INTC_ADDR   = 7'h3F;  // interrupt controller
    localparam logic [4:0] ERR_BAD_REG = 5'd1;   // aux value for invalid register

    localparam int PW = (RESP_DEPTH > 1) ? $clog2(RESP_DEPTH) : 1;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // timer
    logic [47:0] acc, inc;
    logic        run, oneshot;
    logic [48:0] c2, c3, c4, c5;
    logic        p0_st, p1_st;
    logic [47:0] cnt;
    logic [2:0]  cnt_ctl;
    logic [3:0]  int_bits, int_rep;     // int_rep = bits already reported
    logic        irq_out;               // interrupt sent, response not seen yet
    logic        ce_q;

    // receive tracking
    logic        in_pkt;                // between Start and End
    logic        rx_mine;               // current packet addressed to me
    logic [4:0]  rx_idx;                // index of next byte (saturating)
    logic [6:0]  rx_src;
    logic [2:0]  rx_cmd;
    logic [4:0]  rx_reg;
    logic [63:0] rx_wbuf;
    logic [3:0]  rx_nd;                 // data bytes received (max 8)

    // write commit (one clock, whole register)
    logic        wr_req;
    logic [2:0]  wr_reg;
    logic [63:0] wr_mask, wr_data;

    // token / transmit
    logic        boot;                  // first cycle after reset
    logic        hold_tok;              // I own the token while waiting for current pkt to end
    logic        idle_tok;              // token is available on my idle output
    logic        tx_active;
    logic        tx_rel;                // send token-carrying idle after End
    logic [3:0]  tx_idx;                // index of next byte to send
    logic [2:0]  tx_cmd;
    logic [4:0]  tx_aux;
    logic        tx_has_data;
    logic [63:0] tx_data;

    // response FIFO
    logic [6:0]  rq_dest [RESP_DEPTH];
    logic        rq_wr   [RESP_DEPTH];  // 1 = WriteResponse, 0 = ReadResponse
    logic        rq_err  [RESP_DEPTH];
    logic        rq_hasd [RESP_DEPTH];
    logic [63:0] rq_data [RESP_DEPTH];
    logic [PW-1:0] rq_wp, rq_rp;
    logic [PW:0]   rq_cnt;

    // ------------------------------------------------------------------
    // Helper functions
    // ------------------------------------------------------------------
    function automatic logic [63:0] bmask(input logic [3:0] n);
        logic [63:0] m;
        m = 64'd0;
        for (int i = 0; i < 8; i++)
            if (i < int'(n)) m[8*i +: 8] = 8'hFF;
        return m;
    endfunction

    function automatic logic [63:0] mrg(input logic [63:0] a,
                                        input logic [63:0] w,
                                        input logic [63:0] m);
        return (a & ~m) | (w & m);
    endfunction

    // current value of register r as seen on the bus
    function automatic logic [63:0] rd_reg(input logic [2:0] r);
        case (r)
            3'd0:    return {run, oneshot, 14'd0, acc};
            3'd1:    return {16'd0, inc};
            3'd2:    return {p0_st, 14'd0, c2};
            3'd3:    return {15'd0, c3};
            3'd4:    return {p1_st, 14'd0, c4};
            3'd5:    return {15'd0, c5};
            3'd6:    return {13'd0, cnt_ctl, cnt};
            default: return {60'd0, int_bits};
        endcase
    endfunction

    // ------------------------------------------------------------------
    // Receive decode
    // ------------------------------------------------------------------
    logic is_idle, is_start, is_cont, is_end;
    logic start_mine, body_byte, body_mine, end_mine;
    logic tok_in;

    always_comb begin
        is_idle    = (in_cmd == C_IDLE);
        is_start   = (in_cmd == C_START);
        is_cont    = (in_cmd == C_CONT);
        is_end     = (in_cmd == C_END);
        tok_in     = in_data[7];
        start_mine = is_start && (in_data[6:0] == addr);
        body_byte  = (is_cont || is_end) && in_pkt;
        body_mine  = body_byte && rx_mine;
        end_mine   = is_end && in_pkt && rx_mine;
    end

    // fields of the packet being received (include byte arriving this cycle)
    logic [2:0]  cmd_v;
    logic [4:0]  reg_v;
    logic [6:0]  src_v;
    logic [63:0] wbuf_v;
    logic [3:0]  nd_v;
    logic [2:0]  didx;

    always_comb begin
        cmd_v  = (rx_idx == 5'd2) ? in_data[7:5] : rx_cmd;
        reg_v  = (rx_idx == 5'd2) ? in_data[4:0] : rx_reg;
        src_v  = (rx_idx == 5'd1) ? in_data[6:0] : rx_src;
        wbuf_v = rx_wbuf;
        nd_v   = rx_nd;
        didx   = rx_idx[2:0] - 3'd3;
        if (body_mine && rx_idx >= 5'd3 && rx_idx <= 5'd10) begin
            wbuf_v[{didx, 3'b000} +: 8] = in_data;
            nd_v = 4'(rx_idx - 5'd2);
        end
    end

    // request decode (acts on the End byte)
    logic proc, rd_ev, wr_ev, ir_ev, reg_ok, push;

    always_comb begin
        proc   = end_mine && (rx_idx >= 5'd2);
        rd_ev  = proc && (cmd_v == CMD_RREQ);
        wr_ev  = proc && (cmd_v == CMD_WREQ);
        ir_ev  = proc && (cmd_v == CMD_IRSP);
        reg_ok = (reg_v <= 5'd7);
        push   = (rd_ev || wr_ev) && (rq_cnt < (PW+1)'(RESP_DEPTH));
    end

    // ------------------------------------------------------------------
    // Timer datapath
    // ------------------------------------------------------------------
    logic [48:0] sum;
    logic        ovf, run_n;
    logic [47:0] acc_n;
    logic        cmp0, cmp1, rise0, rise1;
    logic        cnt_inc, sign_chg;
    logic [47:0] cnt_n;
    logic [3:0]  int_set, int_ack, int_base, int_f, int_rep_n;
    logic [63:0] r0_n, r1_n, r2_n, r3_n, r4_n, r5_n, r6_n, r7_n;
    logic [63:0] r0_f, r1_f, r2_f, r3_f, r4_f, r5_f, r6_f, r7_f;

    always_comb begin
        // prescaler
        sum   = {1'b0, acc} + {1'b0, inc};
        ovf   = run && sum[48];
        acc_n = run ? sum[47:0] : acc;
        run_n = run && !(ovf && oneshot);

        // pulses: set at reg_hi, cleared at reg_lo (handles wrap-around when c2 > c3)
        if (c2 <= c3)
            cmp0 = ({1'b0, acc} >= c2) && ({1'b0, acc} < c3);
        else
            cmp0 = ({1'b0, acc} >= c2) || ({1'b0, acc} < c3);

        if (c4 <= c5)
            cmp1 = ({1'b0, acc} >= c4) && ({1'b0, acc} < c5);
        else
            cmp1 = ({1'b0, acc} >= c4) || ({1'b0, acc} < c5);

        rise0 = cmp0 && !p0_st;
        rise1 = cmp1 && !p1_st;

        // external counter
        case (cnt_ctl)
            3'd1:    cnt_inc = ovf &&  ce;
            3'd2:    cnt_inc = ovf && !ce;
            3'd3:    cnt_inc = ce  && !ce_q;
            3'd4:    cnt_inc = !ce &&  ce_q;
            default: cnt_inc = 1'b0;
        endcase
        cnt_n    = cnt_inc ? (cnt + 48'd1) : cnt;
        sign_chg = cnt_inc && (cnt[47] ^ cnt_n[47]);

        // interrupts
        int_set  = {ovf, sign_chg, rise1, rise0};
        int_ack  = ir_ev ? reg_v[3:0] : 4'd0;
        int_base = (int_bits & ~int_ack) | int_set;

        // hardware next value of each register (64 bit view) ...
        r0_n = {run_n, oneshot, 14'd0, acc_n};
        r1_n = {16'd0, inc};
        r2_n = {cmp0, 14'd0, c2};
        r3_n = {15'd0, c3};
        r4_n = {cmp1, 14'd0, c4};
        r5_n = {15'd0, c5};
        r6_n = {13'd0, cnt_ctl, cnt_n};
        r7_n = {60'd0, int_base};

        // ... merged with a pending bus write (only the bytes that were sent)
        r0_f = (wr_req && wr_reg == 3'd0) ? mrg(r0_n, wr_data, wr_mask) : r0_n;
        r1_f = (wr_req && wr_reg == 3'd1) ? mrg(r1_n, wr_data, wr_mask) : r1_n;
        r2_f = (wr_req && wr_reg == 3'd2) ? mrg(r2_n, wr_data, wr_mask) : r2_n;
        r3_f = (wr_req && wr_reg == 3'd3) ? mrg(r3_n, wr_data, wr_mask) : r3_n;
        r4_f = (wr_req && wr_reg == 3'd4) ? mrg(r4_n, wr_data, wr_mask) : r4_n;
        r5_f = (wr_req && wr_reg == 3'd5) ? mrg(r5_n, wr_data, wr_mask) : r5_n;
        r6_f = (wr_req && wr_reg == 3'd6) ? mrg(r6_n, wr_data, wr_mask) : r6_n;
        r7_f = (wr_req && wr_reg == 3'd7) ? mrg(r7_n, wr_data, wr_mask) : r7_n;

        int_f = r7_f[3:0];
    end

    // ------------------------------------------------------------------
    // Token / transmit control
    // ------------------------------------------------------------------
    logic want_resp, want_irq, want_tx;
    logic cap_mine, cap_fwd, tok_idle, launch, launch_irq, release_tok;
    logic hold_tok_n, idle_tok_n;

    // launch fields
    logic [6:0]  l_dest;
    logic [2:0]  l_cmd;
    logic [4:0]  l_aux;
    logic        l_hasd;
    logic [63:0] l_data;

    always_comb begin
        want_resp = (rq_cnt != '0);
        want_irq  = ((int_bits & ~int_rep) != 4'd0) && !irq_out;
        want_tx   = want_resp || want_irq;

        tok_idle  = is_idle && tok_in;

        // capture the token of a packet addressed to me (consume it) ...
        cap_mine  = start_mine && tok_in;
        // ... or of a passing packet if I have something to send afterwards
        cap_fwd   = is_start && !start_mine && tok_in && want_tx &&
                    !tx_active && !hold_tok && !boot;

        // start sending: token seen on an idle, already held, or currently offered on idle_tok
        launch      = !tx_active && !tx_rel && !boot && !in_pkt && !is_start && want_tx &&
                      (tok_idle || hold_tok || idle_tok);
        launch_irq  = launch && !want_resp;
        // nothing to send but I hold the token: offer it on idle
        release_tok = !tx_active && !tx_rel && !boot && !in_pkt && !is_start &&
                      hold_tok && !want_tx;

        hold_tok_n = hold_tok;
        if (launch || release_tok) hold_tok_n = 1'b0;
        if (cap_mine || cap_fwd)   hold_tok_n = 1'b1;

        // Keep advertising the token on C_IDLE until either we launch a packet
        // or the upstream device starts a packet (taking ownership of the token).
        idle_tok_n = idle_tok;
        if (boot && (addr == 7'd0))
            idle_tok_n = 1'b1;
        else if (launch || is_start)
            idle_tok_n = 1'b0;
        else if (tx_rel || release_tok || (!tx_active && !in_pkt && tok_idle))
            idle_tok_n = 1'b1;

        if (want_resp) begin
            l_dest = rq_dest[rq_rp];
            l_cmd  = rq_wr[rq_rp] ? CMD_WRSP : CMD_RRSP;
            l_aux  = rq_err[rq_rp] ? ERR_BAD_REG : 5'd0;
            l_hasd = rq_hasd[rq_rp];
            l_data = rq_data[rq_rp];
        end else begin
            l_dest = INTC_ADDR;
            l_cmd  = CMD_IREQ;
            l_aux  = {1'b0, int_bits};
            l_hasd = 1'b0;
            l_data = 64'd0;
        end

        int_rep_n = (int_rep & ~int_ack & int_f) |
                    (launch_irq ? (int_bits & int_f) : 4'd0);
    end

    // ------------------------------------------------------------------
    // Output byte selection
    // ------------------------------------------------------------------
    logic [1:0] out_cmd_n;
    logic [7:0] out_data_n;
    logic       tx_last;

    always_comb begin
        tx_last    = ((tx_idx == 4'd2) && !tx_has_data) || (tx_idx == 4'd10);
        out_cmd_n  = C_IDLE;
        out_data_n = 8'h00;

        if (boot) begin
            // device 0 starts the token after reset
            out_data_n = {(addr == 7'd0), 7'd0};
        end else if (tx_active) begin
            case (tx_idx)
                4'd1: begin
                    out_cmd_n  = C_CONT;
                    out_data_n = {1'b0, addr};
                end
                4'd2: begin
                    out_cmd_n  = tx_has_data ? C_CONT : C_END;
                    out_data_n = {tx_cmd, tx_aux};
                end
                default: begin
                    out_cmd_n  = tx_last ? C_END : C_CONT;
                    out_data_n = tx_data[{(tx_idx - 4'd3), 3'b000} +: 8];
                end
            endcase
        end else if (tx_rel) begin
            out_cmd_n  = C_IDLE;
            out_data_n = 8'h80;                   // release token right after End
        end else if (launch) begin
            out_cmd_n  = C_START;
            out_data_n = {1'b0, l_dest};          // token bit cleared (we own it)
        end else if (release_tok) begin
            out_cmd_n  = C_IDLE;
            out_data_n = 8'h80;                   // idle carrying the token
        end else if (start_mine || body_mine) begin
            out_cmd_n  = C_IDLE;                  // packet consumed -> idle, no token
            out_data_n = 8'h00;
        end else if (is_start) begin
            out_cmd_n  = C_START;                 // forward (token cleared if captured)
            out_data_n = {(tok_in && !cap_fwd), in_data[6:0]};
        end else if (is_idle) begin
            out_cmd_n  = C_IDLE;
            out_data_n = {(tok_in || idle_tok), 7'd0};
        end else begin
            out_cmd_n  = in_cmd;                  // Continue / End of a forwarded packet
            out_data_n = in_data;
        end
    end

    // ------------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (reset) begin
            acc <= '0;  inc <= '0;  run <= 1'b0;  oneshot <= 1'b0;
            c2 <= '0;  c3 <= '0;  c4 <= '0;  c5 <= '0;
            p0_st <= 1'b0;  p1_st <= 1'b0;
            cnt <= '0;  cnt_ctl <= '0;
            int_bits <= '0;  int_rep <= '0;  irq_out <= 1'b0;
            ce_q <= 1'b0;

            in_pkt <= 1'b0;  rx_mine <= 1'b0;  rx_idx <= '0;
            rx_src <= '0;  rx_cmd <= '0;  rx_reg <= '0;
            rx_wbuf <= '0;  rx_nd <= '0;
            wr_req <= 1'b0;  wr_reg <= '0;  wr_mask <= '0;  wr_data <= '0;

            boot <= 1'b1;  hold_tok <= 1'b0;  idle_tok <= 1'b0;
            tx_active <= 1'b0;  tx_rel <= 1'b0;  tx_idx <= '0;  tx_cmd <= '0;  tx_aux <= '0;
            tx_has_data <= 1'b0;  tx_data <= '0;

            rq_wp <= '0;  rq_rp <= '0;  rq_cnt <= '0;

            out_cmd  <= C_IDLE;
            out_data <= 8'h00;
        end else begin
            boot     <= 1'b0;
            out_cmd  <= out_cmd_n;
            out_data <= out_data_n;
            ce_q     <= ce;

            // ---- timer ----
            acc     <= r0_f[47:0];
            run     <= r0_f[63];
            oneshot <= r0_f[62];
            inc     <= r1_f[47:0];
            c2      <= r2_f[48:0];
            c3      <= r3_f[48:0];
            c4      <= r4_f[48:0];
            c5      <= r5_f[48:0];
            p0_st   <= cmp0;
            p1_st   <= cmp1;
            cnt     <= r6_f[47:0];
            cnt_ctl <= r6_f[50:48];

            // ---- interrupts ----
            int_bits <= int_f;
            int_rep  <= int_rep_n;
            if (ir_ev)           irq_out <= 1'b0;
            else if (launch_irq) irq_out <= 1'b1;

            // ---- write commit ----
            wr_req <= 1'b0;
            if (wr_ev && reg_ok) begin
                wr_req  <= 1'b1;
                wr_reg  <= reg_v[2:0];
                wr_mask <= bmask(nd_v);
                wr_data <= wbuf_v;
            end

            // ---- receive tracking ----
            if (is_start) begin
                in_pkt  <= 1'b1;
                rx_mine <= start_mine;
                rx_idx  <= 5'd1;
                rx_wbuf <= '0;
                rx_nd   <= '0;
            end else if (body_byte) begin
                if (rx_idx != 5'd31) rx_idx <= rx_idx + 5'd1;
                if (rx_mine) begin
                    if (rx_idx == 5'd1) rx_src <= in_data[6:0];
                    if (rx_idx == 5'd2) begin
                        rx_cmd <= in_data[7:5];
                        rx_reg <= in_data[4:0];
                    end
                    rx_wbuf <= wbuf_v;
                    rx_nd   <= nd_v;
                end
                if (is_end) in_pkt <= 1'b0;
            end

            // ---- token / transmit ----
            hold_tok <= hold_tok_n;
            idle_tok <= idle_tok_n;
            tx_rel   <= tx_active && tx_last;

            if (launch) begin
                tx_active   <= 1'b1;
                tx_idx      <= 4'd1;
                tx_cmd      <= l_cmd;
                tx_aux      <= l_aux;
                tx_has_data <= l_hasd;
                tx_data     <= l_data;
            end else if (tx_active) begin
                tx_idx <= tx_idx + 4'd1;
                if (tx_last) tx_active <= 1'b0;
            end

            // ---- response FIFO ----
            if (push) begin
                rq_dest[rq_wp] <= src_v;
                rq_wr  [rq_wp] <= wr_ev;
                rq_err [rq_wp] <= !reg_ok;
                rq_hasd[rq_wp] <= rd_ev && reg_ok;
                rq_data[rq_wp] <= (rd_ev && reg_ok) ? rd_reg(reg_v[2:0]) : 64'd0;
                rq_wp <= (rq_wp == PW'(RESP_DEPTH-1)) ? '0 : rq_wp + 1'b1;
            end
            if (launch && want_resp)
                rq_rp <= (rq_rp == PW'(RESP_DEPTH-1)) ? '0 : rq_rp + 1'b1;
            rq_cnt <= rq_cnt + (push ? 1'b1 : 1'b0) -
                      ((launch && want_resp) ? 1'b1 : 1'b0);
        end
    end


`ifdef DD_DEBUG
    // ---- simulation-only trace: compile with +define+DD_DEBUG ----
    always @(posedge clk) if (!reset) begin
        if (in_cmd != C_IDLE || in_data != 8'h00 || out_cmd != C_IDLE || out_data != 8'h00)
            $display("DD %0t in=%0d/%02h out=%0d/%02h hold=%b txa=%b rel=%b inpkt=%b rq=%0d irq=%b int=%h",
                     $time, in_cmd, in_data, out_cmd, out_data,
                     hold_tok, tx_active, tx_rel, in_pkt, rq_cnt, irq_out, int_bits);
        if (is_idle && tok_in && (tx_active || tx_rel))
            $display("DD %0t WARNING: token arrived while transmitting (dropped)", $time);
        if ((rd_ev || wr_ev) && !push)
            $display("DD %0t WARNING: response queue full, response dropped", $time);
        if (wr_req)
            $display("DD %0t write commit reg=%0d mask=%h data=%h", $time, wr_reg, wr_mask, wr_data);
        if (launch)
            $display("DD %0t launch cmd=%0d dest=%h aux=%h resp=%b", $time, l_cmd, l_dest, l_aux, want_resp);
        if (release_tok)
            $display("DD %0t release token (nothing to send)", $time);
        if (run != r0_f[63])
            $display("DD %0t run bit -> %b", $time, r0_f[63]);
        if (int_bits != int_f)
            $display("DD %0t int_bits %h -> %h", $time, int_bits, int_f);
    end
`endif

    assign p0_out = p0_st;
    assign p1_out = p1_st;

endmodule : dd_timer_core


// ----------------------------------------------------------------------------
// Wrapper matching:  dd n0(cr.clkresi, ringt.rin, ringd.rout, 7'b0,
//                          pps.ppdevo, pps.ppdevi);
// ----------------------------------------------------------------------------
module dd (
    CLKRESET.clkresi cr,
    RBUS.rin         rin,
    RBUS.rout        rout,
    input  logic [6:0] addr,
    PPINS.ppdevo     ppo,
    PPINS.ppdevi     ppi
);

    logic [1:0] oc;
    logic [7:0] od;
    logic       p0, p1;

    dd_timer_core core (
        .clk      (cr.clk),
        .reset    (cr.reset),
        .in_cmd   (rin.Cmd),
        .in_data  (rin.Data),
        .out_cmd  (oc),
        .out_data (od),
        .addr     (addr),
        .ce       (ppi.CE),
        .p0_out   (p0),
        .p1_out   (p1)
    );

    assign rout.Cmd  = oc;
    assign rout.Data = od;
    assign ppo.P1    = p0;
    assign ppo.P2    = p1;

endmodule : dd

`endif
