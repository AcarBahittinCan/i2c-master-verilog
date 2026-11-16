# I2C Master Core (Verilog)

A synthesizable **I2C master** controller in Verilog with multi-byte read/write, clock stretching and arbitration-loss detection. It comes with a self-checking testbench of **25 directed tests**.

> Developed as a peripheral IP for a TEKNOFEST chip design project.

## Features

- 7-bit addressing, **write and read** transactions
- **1–4 byte bursts** per transaction (`nby_i` = byte count − 1), packed into 32-bit TX/RX registers
- **Open-drain** SDA/SCL (`0` or `Z`) for an external pull-up bus
- **Clock stretching:** the SCL counter freezes while a slave holds SCL low
- **Arbitration-loss detection** (`arb_lost`) during address and data phases
- **TDR latching:** transmit data is captured at the start of a transaction, so changes to `tdr_i` mid-transfer have no effect
- `tx_done` / `rx_done` pulse at STOP completion and are cleared at the next START
- Two-process FSM (registered state + combinational next-state), `tx_en` has priority over `rx_en`

**SCL frequency:** `f_SCL = f_clk / 250` (`HALF_PERIOD = 125`), which gives **400 kHz Fast-mode at 100 MHz**.

## Interface

| Port | Dir | Description |
|---|---|---|
| `clk_i`, `rst_i` | in | Clock, synchronous active-high reset |
| `adr_i[6:0]` | in | 7-bit slave address |
| `nby_i[1:0]` | in | Number of bytes − 1 (0 → 1 byte … 3 → 4 bytes) |
| `tdr_i[31:0]` | in | Transmit data, byte 0 = `[7:0]` sent first |
| `rdr_o[31:0]` | out | Received data, byte 0 in `[7:0]` |
| `tx_en` / `rx_en` | in | Start a write / read transaction |
| `tx_done` / `rx_done` | out | Transaction complete |
| `sda`, `scl` | inout | Open-drain I2C bus |

## FSM

```
IDLE ─► START ─► ADDR ─► ACK_ADDR ─┬─► DATA_WR ─► ACK_WR ─┐
                                   │                     ├─(byte_cnt == nby)─► STOP ─► IDLE
                                   └─► DATA_RD ─► ACK_RD ─┘
        NACK ─► STOP          arbitration lost ─► IDLE (bus released)
```

## Verification

`sim/i2c_core_tb.v` models a slave with open-drain drivers and `pullup`s, and checks each result with PASS/FAIL reporting:

| # | Tests |
|---|---|
| 1, 17 | Reset behaviour, bus idle after transfer |
| 2–5, 15–16 | 1, 2 and 4-byte writes and reads |
| 6–7 | NACK on address and on data |
| 8 | Back-to-back transfers |
| 9, 20 | Clock stretching during write and read |
| 10, 21 | Arbitration loss in ADDR and DATA_WR |
| 11–13 | `tx_done`/`rx_done` timing and clearing |
| 14, 19 | TDR latch protection, RDR preserved during write |
| 18 | `tx_en` priority over `rx_en` |
| 22–25 | Corner data/address values (`0x00`, `0xFF`) |

Run in Vivado (add `rtl/` + `sim/`, top = `i2c_core_tb`) or with Icarus Verilog:

```bash
iverilog -o tb rtl/i2c_core.v sim/i2c_core_tb.v && vvp tb
```

## Author

**Bahittin Can Acar**
