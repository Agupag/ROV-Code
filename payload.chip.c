// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
//
// nRF24L01+ payload-side emulator.  Implements the subset of the radio
// register set and SPI command protocol used by the RF24 Arduino library
// in PRX (primary receiver) mode with ACK payloads.
//
// Reference: nRF24L01+ Product Specification v1.0
//   - Section 8: SPI command set
//   - Section 9: Register map
//   - Section 7.5: ACK payload handling

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Set to 1 to enable printf diagnostics in the Wokwi console
#define PAYLOAD_VERBOSE 1
#define RF24_MAX_PAYLOAD 32

// ---- SPI command bytes (nRF24L01+ datasheet Table 20) ----
#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_ACTIVATE 0x50
#define CMD_R_RX_PL_WID 0x60
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_FLUSH_TX 0xE1
#define CMD_FLUSH_RX 0xE2
#define CMD_NOP 0xFF

// ---- Register addresses (nRF24L01+ datasheet Table 28) ----
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_RPD 0x09
#define REG_RX_ADDR_P0 0x0A
#define REG_RX_ADDR_P1 0x0B
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

// ---- CONFIG register bits ----
#define CONFIG_PRIM_RX 0x01
#define CONFIG_PWR_UP 0x02
#define CONFIG_MASK_MAX_RT 0x10
#define CONFIG_MASK_TX_DS 0x20
#define CONFIG_MASK_RX_DR 0x40

// ---- STATUS register bits ----
#define STATUS_RX_DR 0x40
#define STATUS_TX_DS 0x20
#define STATUS_MAX_RT 0x10
#define STATUS_RX_P_NO_EMPTY 0x0E
#define STATUS_RX_P_NO_P1 0x02

// ---- FIFO_STATUS register bits ----
#define FIFO_STATUS_RX_EMPTY 0x01
#define FIFO_STATUS_TX_EMPTY 0x10

#define ACTIVATE_DATA 0x73

// ---- Byte offsets into control_data for thruster commands ----
#define CTRL_S1_OFFSET 4
#define CTRL_M1_OFFSET 8
#define CTRL_M2_OFFSET 10
#define CTRL_M3_OFFSET 12
#define CTRL_M4_OFFSET 14

typedef struct {
  pin_t vcc;
  pin_t gnd;
  pin_t ce;
  pin_t csn;
  pin_t irq;
  pin_t m1_fwd;
  pin_t m1_rev;
  pin_t m2_fwd;
  pin_t m2_rev;
  pin_t m3_fwd;
  pin_t m3_rev;
  pin_t m4_fwd;
  pin_t m4_rev;
  pin_watch_config_t ce_watch;
  pin_watch_config_t csn_watch;
  spi_dev_t spi;
  uint8_t spi_byte;

  uint8_t regs[32];
  uint8_t rx_addr_p0[5];
  uint8_t rx_addr_p1[5];
  uint8_t tx_addr[5];
  uint8_t telemetry_payload[RF24_MAX_PAYLOAD];
  uint8_t control_payload[RF24_MAX_PAYLOAD];
  uint8_t telemetry_len;
  uint8_t control_len;
  bool rx_full;
  bool ce_high;
  bool csn_active;
  bool features_unlocked;

  bool expecting_command;
  uint8_t current_cmd;
  uint8_t stream_index;

  uint16_t telemetry_sequence;
} chip_state_t;

static void write_u16_le(uint8_t *buffer, uint8_t offset, uint16_t value) {
  buffer[offset] = (uint8_t)(value & 0xff);
  buffer[offset + 1] = (uint8_t)(value >> 8);
}

static uint16_t read_u16_le(const uint8_t *buffer, uint8_t offset) {
  return (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
}

static bool prx_active(const chip_state_t *chip) {
  return chip->ce_high &&
         (chip->regs[REG_CONFIG] & CONFIG_PWR_UP) &&
         (chip->regs[REG_CONFIG] & CONFIG_PRIM_RX) &&
         (chip->regs[REG_EN_RXADDR] & 0x02);
}

// Build the STATUS register value per nRF24L01+ datasheet Section 9.
// Bits 6:4 = interrupt flags (stored in regs[STATUS])
// Bits 3:1 = RX_P_NO (pipe with data, or 0x07 if empty)
// Bit 0    = TX_FULL (always 0 in this simplified model)
static uint8_t status_value(const chip_state_t *chip) {
  uint8_t status = chip->regs[REG_STATUS] & (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
  status |= chip->rx_full ? STATUS_RX_P_NO_P1 : STATUS_RX_P_NO_EMPTY;
  return status;
}

// Build the FIFO_STATUS register value per nRF24L01+ datasheet.
// Bit 0 = RX_EMPTY, Bit 4 = TX_EMPTY
static uint8_t fifo_status_value(const chip_state_t *chip) {
  uint8_t fifo = FIFO_STATUS_TX_EMPTY;
  if (!chip->rx_full) {
    fifo |= FIFO_STATUS_RX_EMPTY;
  }
  return fifo;
}

// Drive the IRQ pin based on STATUS flags AND CONFIG mask bits.
// Per the datasheet: IRQ is active-LOW and reflects (flag & ~mask).
static void update_irq(chip_state_t *chip) {
  uint8_t flags = chip->regs[REG_STATUS] & (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
  uint8_t masks = chip->regs[REG_CONFIG] & (CONFIG_MASK_RX_DR | CONFIG_MASK_TX_DS | CONFIG_MASK_MAX_RT);
  // The mask bits in CONFIG suppress the corresponding IRQ source.
  // MASK_RX_DR (bit 6 of CONFIG) suppresses RX_DR (bit 6 of STATUS), etc.
  // Conveniently they are the same bit positions.
  const bool irq_active = (flags & ~masks) != 0;
  pin_write(chip->irq, irq_active ? LOW : HIGH);
}

// Drive a single FWD/REV LED pair for one thruster.
// value=0: both off, value=1: FWD on, value=2: REV on
static void set_thruster_leds(pin_t fwd, pin_t rev, uint16_t value) {
  pin_write(fwd, value == 1 ? HIGH : LOW);
  pin_write(rev, value == 2 ? HIGH : LOW);
}

static void update_thruster_leds(chip_state_t *chip) {
  if (chip->control_len < 16) {
    set_thruster_leds(chip->m1_fwd, chip->m1_rev, 0);
    set_thruster_leds(chip->m2_fwd, chip->m2_rev, 0);
    set_thruster_leds(chip->m3_fwd, chip->m3_rev, 0);
    set_thruster_leds(chip->m4_fwd, chip->m4_rev, 0);
    return;
  }

  uint16_t s1 = read_u16_le(chip->control_payload, CTRL_S1_OFFSET);
  uint16_t m1 = read_u16_le(chip->control_payload, CTRL_M1_OFFSET);
  uint16_t m2 = read_u16_le(chip->control_payload, CTRL_M2_OFFSET);
  uint16_t m3 = read_u16_le(chip->control_payload, CTRL_M3_OFFSET);
  uint16_t m4 = read_u16_le(chip->control_payload, CTRL_M4_OFFSET);

  set_thruster_leds(chip->m1_fwd, chip->m1_rev, m1);
  set_thruster_leds(chip->m2_fwd, chip->m2_rev, m2);
  set_thruster_leds(chip->m3_fwd, chip->m3_rev, m3);
  set_thruster_leds(chip->m4_fwd, chip->m4_rev, m4);

#if PAYLOAD_VERBOSE
  printf("[payload] ACK ctrl received: S1=%u M1=%u M2=%u M3=%u M4=%u\n", s1, m1, m2, m3, m4);
#endif
}

static void store_control_byte(chip_state_t *chip, uint8_t byte) {
  if (chip->stream_index < RF24_MAX_PAYLOAD) {
    chip->control_payload[chip->stream_index] = byte;
  }
  chip->stream_index++;
}

static uint8_t reg_width(uint8_t reg) {
  if (reg == REG_RX_ADDR_P0 || reg == REG_RX_ADDR_P1 || reg == REG_TX_ADDR) {
    return 5;
  }
  return 1;
}

static uint8_t read_register_byte(const chip_state_t *chip, uint8_t reg, uint8_t index) {
  if (reg == REG_STATUS) {
    return status_value(chip);
  }
  if (reg == REG_FIFO_STATUS) {
    return fifo_status_value(chip);
  }
  if (reg == REG_RX_ADDR_P0 && index < 5) return chip->rx_addr_p0[index];
  if (reg == REG_RX_ADDR_P1 && index < 5) return chip->rx_addr_p1[index];
  if (reg == REG_TX_ADDR && index < 5) return chip->tx_addr[index];
  if (index == 0 && reg < 32) return chip->regs[reg];
  return 0;
}

static void queue_telemetry(chip_state_t *chip) {
  memset(chip->telemetry_payload, 0, sizeof(chip->telemetry_payload));

  // Build a synthetic payload_data struct matching the Arduino struct layout.
  // Each field is uint16_t little-endian.
  write_u16_le(chip->telemetry_payload, 0, 0xFFFF);   // startmark
  write_u16_le(chip->telemetry_payload, 2, 0);         // badPktCnt
  write_u16_le(chip->telemetry_payload, 4, 0);         // yaw
  write_u16_le(chip->telemetry_payload, 6, 90);        // pitch (add 90 → 0 real)
  write_u16_le(chip->telemetry_payload, 8, 180);       // roll  (add 180 → 0 real)
  write_u16_le(chip->telemetry_payload, 10, 1200);     // vlt = 12.00V
  write_u16_le(chip->telemetry_payload, 12, 35);       // amp = 0.35A
  write_u16_le(chip->telemetry_payload, 14, 0);        // pres = 0 cm H2O
  write_u16_le(chip->telemetry_payload, 16, 2200);     // temp = 22.00C
  write_u16_le(chip->telemetry_payload, 18, 2026);     // yr
  write_u16_le(chip->telemetry_payload, 20, 4);        // mon
  write_u16_le(chip->telemetry_payload, 22, 7);        // day
  write_u16_le(chip->telemetry_payload, 24, 12);       // hr
  write_u16_le(chip->telemetry_payload, 26, 0);        // min
  write_u16_le(chip->telemetry_payload, 28, chip->telemetry_sequence % 60); // sec
  write_u16_le(chip->telemetry_payload, 30, 0);        // tds

  chip->telemetry_sequence++;
  chip->telemetry_len = RF24_MAX_PAYLOAD;
  chip->rx_full = true;
  chip->regs[REG_STATUS] |= STATUS_RX_DR;
  update_irq(chip);

#if PAYLOAD_VERBOSE
  printf("[payload] Telemetry #%u queued (RX_DR set, IRQ asserted)\n", chip->telemetry_sequence);
#endif
}

static void maybe_queue_telemetry(chip_state_t *chip) {
  if (prx_active(chip) && !chip->rx_full && !(chip->regs[REG_STATUS] & STATUS_RX_DR)) {
    queue_telemetry(chip);
  }
}

static void clear_rx_payload(chip_state_t *chip, bool clear_irq_flag) {
  chip->rx_full = false;
  chip->telemetry_len = 0;
  if (clear_irq_flag) {
    chip->regs[REG_STATUS] &= ~STATUS_RX_DR;
    update_irq(chip);
  }
}

static void write_register_byte(chip_state_t *chip, uint8_t reg, uint8_t index, uint8_t value) {
  if (reg == REG_RX_ADDR_P0 && index < 5) {
    chip->rx_addr_p0[index] = value;
    return;
  }
  if (reg == REG_RX_ADDR_P1 && index < 5) {
    chip->rx_addr_p1[index] = value;
    return;
  }
  if (reg == REG_TX_ADDR && index < 5) {
    chip->tx_addr[index] = value;
    return;
  }
  if (index != 0 || reg >= 32) {
    return;
  }
  if (reg == REG_STATUS) {
    // Per datasheet: writing a 1 to a flag bit CLEARS it.
    chip->regs[REG_STATUS] &= ~(value & (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT));
    update_irq(chip);
    maybe_queue_telemetry(chip);
    return;
  }
  chip->regs[reg] = value;
  if (reg == REG_CONFIG || reg == REG_EN_RXADDR) {
    maybe_queue_telemetry(chip);
  }
}

static void load_power_on_defaults(chip_state_t *chip) {
  memset(chip->regs, 0, sizeof(chip->regs));
  memset(chip->rx_addr_p0, 0xE7, sizeof(chip->rx_addr_p0));
  memset(chip->tx_addr, 0xE7, sizeof(chip->tx_addr));
  memset(chip->rx_addr_p1, 0xC2, sizeof(chip->rx_addr_p1));

  // Power-on reset defaults per nRF24L01+ datasheet Table 28
  chip->regs[REG_CONFIG] = 0x08;        // EN_CRC=1
  chip->regs[REG_EN_AA] = 0x3F;         // auto-ack on all pipes
  chip->regs[REG_EN_RXADDR] = 0x03;     // pipes 0 & 1 enabled
  chip->regs[REG_SETUP_AW] = 0x03;      // 5-byte address
  chip->regs[REG_SETUP_RETR] = 0x03;    // 250us, 3 retries
  chip->regs[REG_RF_CH] = 0x02;         // channel 2
  chip->regs[REG_RF_SETUP] = 0x0F;      // 2Mbps, 0dBm, LNA
  chip->regs[REG_DYNPD] = 0x00;
  chip->regs[REG_FEATURE] = 0x00;
}

static void finish_current_command(chip_state_t *chip) {
  if ((chip->current_cmd == CMD_W_TX_PAYLOAD || (chip->current_cmd & 0xF8) == CMD_W_ACK_PAYLOAD) &&
      chip->stream_index > 0) {
    chip->control_len = chip->stream_index > RF24_MAX_PAYLOAD ? RF24_MAX_PAYLOAD : chip->stream_index;
    update_thruster_leds(chip);
  }

  if (chip->current_cmd == CMD_R_RX_PAYLOAD && chip->rx_full && chip->stream_index >= chip->telemetry_len) {
    clear_rx_payload(chip, false);
#if PAYLOAD_VERBOSE
    printf("[payload] RX payload read complete, cleared rx_full\n");
#endif
  }
}

// ---- SPI command processing ----
// Per the nRF24L01+ datasheet: the first MISO byte of EVERY command is
// the STATUS register.  The return value of handle_command_start is
// used as MISO for the SECOND byte (the first data byte after the command).

static uint8_t handle_command_start(chip_state_t *chip, uint8_t byte) {
  chip->current_cmd = byte;
  chip->stream_index = 0;
  chip->expecting_command = false;

  if (byte == CMD_FLUSH_TX) {
    chip->expecting_command = true;
    return status_value(chip);
  }

  if (byte == CMD_FLUSH_RX) {
    clear_rx_payload(chip, true);
    maybe_queue_telemetry(chip);
    chip->expecting_command = true;
    return status_value(chip);
  }

  if (byte == CMD_NOP) {
    chip->expecting_command = true;
    return status_value(chip);
  }

  // R_RX_PL_WID: Per datasheet, byte 1 = STATUS (handled by CSN-LOW
  // preload), byte 2 = payload width.  The return value here becomes
  // the MISO for byte 2.
  if (byte == CMD_R_RX_PL_WID) {
    // Return the payload width for the next (second) byte.
    return chip->rx_full ? chip->telemetry_len : 0;
  }

  if (byte == CMD_ACTIVATE) {
    return status_value(chip);
  }

  // R_REGISTER: return value here becomes MISO byte 2 (first data byte)
  if ((byte & 0xE0) == CMD_R_REGISTER) {
    const uint8_t reg = byte & 0x1F;
    chip->stream_index = 1;
    return read_register_byte(chip, reg, 0);
  }

  if ((byte & 0xE0) == CMD_W_REGISTER) {
    return status_value(chip);
  }

  if (byte == CMD_R_RX_PAYLOAD) {
    chip->stream_index = 1;
    if (chip->rx_full && chip->telemetry_len > 0) {
      return chip->telemetry_payload[0];
    }
    return 0;
  }

  if (byte == CMD_W_TX_PAYLOAD || (byte & 0xF8) == CMD_W_ACK_PAYLOAD) {
    return status_value(chip);
  }

  chip->expecting_command = true;
  return status_value(chip);
}

static uint8_t handle_command_data(chip_state_t *chip, uint8_t byte) {
  if ((chip->current_cmd & 0xE0) == CMD_R_REGISTER) {
    const uint8_t reg = chip->current_cmd & 0x1F;
    uint8_t out = 0;
    if (chip->stream_index < reg_width(reg)) {
      out = read_register_byte(chip, reg, chip->stream_index);
      chip->stream_index++;
    } else {
      chip->expecting_command = true;
      out = status_value(chip);
    }
    return out;
  }

  if ((chip->current_cmd & 0xE0) == CMD_W_REGISTER) {
    const uint8_t reg = chip->current_cmd & 0x1F;
    write_register_byte(chip, reg, chip->stream_index, byte);
    chip->stream_index++;
    if (chip->stream_index >= reg_width(reg)) {
      chip->expecting_command = true;
    }
    return status_value(chip);
  }

  if (chip->current_cmd == CMD_ACTIVATE) {
    if (byte == ACTIVATE_DATA) {
      chip->features_unlocked = !chip->features_unlocked;
    }
    chip->expecting_command = true;
    return status_value(chip);
  }

  if (chip->current_cmd == CMD_R_RX_PL_WID) {
    // Third+ bytes after R_RX_PL_WID — not expected, return STATUS
    chip->expecting_command = true;
    return status_value(chip);
  }

  if (chip->current_cmd == CMD_R_RX_PAYLOAD) {
    uint8_t out = 0;
    if (chip->rx_full && chip->stream_index < chip->telemetry_len) {
      out = chip->telemetry_payload[chip->stream_index];
      chip->stream_index++;
    } else {
      chip->expecting_command = true;
      out = status_value(chip);
    }
    return out;
  }

  if (chip->current_cmd == CMD_W_TX_PAYLOAD || (chip->current_cmd & 0xF8) == CMD_W_ACK_PAYLOAD) {
    store_control_byte(chip, byte);
    return status_value(chip);
  }

  chip->expecting_command = true;
  return status_value(chip);
}

static uint8_t handle_spi_byte(chip_state_t *chip, uint8_t byte) {
  if (chip->expecting_command) {
    return handle_command_start(chip, byte);
  }
  return handle_command_data(chip, byte);
}

static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (!chip->csn_active || count == 0) {
    return;
  }

  chip->spi_byte = handle_spi_byte(chip, buffer[0]);
  if (chip->csn_active) {
    spi_start(chip->spi, &chip->spi_byte, 1);
  }
}

static void on_ce_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t *)user_data;
  (void)pin;
  chip->ce_high = value == HIGH;
#if PAYLOAD_VERBOSE
  printf("[payload] CE %s\n", chip->ce_high ? "HIGH" : "LOW");
#endif
  maybe_queue_telemetry(chip);
}

static void on_csn_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t *)user_data;
  (void)pin;

  if (value == LOW) {
    chip->csn_active = true;
    chip->expecting_command = true;
    chip->current_cmd = CMD_NOP;
    chip->stream_index = 0;
    // Per nRF24L01+ datasheet: STATUS is always clocked out on MISO
    // while the command byte is clocked in on MOSI.
    chip->spi_byte = status_value(chip);
    spi_start(chip->spi, &chip->spi_byte, 1);
    return;
  }

  finish_current_command(chip);
  chip->csn_active = false;
  spi_stop(chip->spi);
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->vcc = pin_init("VCC", INPUT);
  chip->gnd = pin_init("GND", INPUT);
  chip->ce = pin_init("CE", INPUT);
  chip->csn = pin_init("CSN", INPUT_PULLUP);
  pin_t sck = pin_init("SCK", INPUT);
  pin_t mosi = pin_init("MOSI", INPUT);
  pin_t miso = pin_init("MISO", OUTPUT);
  chip->irq = pin_init("IRQ", OUTPUT);
  chip->m1_fwd = pin_init("M1_FWD", OUTPUT);
  chip->m1_rev = pin_init("M1_REV", OUTPUT);
  chip->m2_fwd = pin_init("M2_FWD", OUTPUT);
  chip->m2_rev = pin_init("M2_REV", OUTPUT);
  chip->m3_fwd = pin_init("M3_FWD", OUTPUT);
  chip->m3_rev = pin_init("M3_REV", OUTPUT);
  chip->m4_fwd = pin_init("M4_FWD", OUTPUT);
  chip->m4_rev = pin_init("M4_REV", OUTPUT);

  load_power_on_defaults(chip);

  pin_write(miso, LOW);
  pin_write(chip->irq, HIGH);
  set_thruster_leds(chip->m1_fwd, chip->m1_rev, 0);
  set_thruster_leds(chip->m2_fwd, chip->m2_rev, 0);
  set_thruster_leds(chip->m3_fwd, chip->m3_rev, 0);
  set_thruster_leds(chip->m4_fwd, chip->m4_rev, 0);

  const spi_config_t spi_config = {
      .sck = sck,
      .mosi = mosi,
      .miso = miso,
      .done = chip_spi_done,
      .mode = 0,
      .user_data = chip,
  };
  chip->spi = spi_init(&spi_config);

  chip->ce_watch = (pin_watch_config_t){
      .edge = BOTH,
      .pin_change = on_ce_change,
      .user_data = chip,
  };
  chip->csn_watch = (pin_watch_config_t){
      .edge = BOTH,
      .pin_change = on_csn_change,
      .user_data = chip,
  };

  pin_watch(chip->ce, &chip->ce_watch);
  pin_watch(chip->csn, &chip->csn_watch);

  printf("[payload] chip initialized — SPI mode 0, waiting for RF24 init\n");
}
