// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RF24_VERBOSE 0
#define RF24_MAX_PAYLOAD 32

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

#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_RX_ADDR_P0 0x0A
#define REG_RX_ADDR_P1 0x0B
#define REG_TX_ADDR 0x10
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define CONFIG_PRIM_RX 0x01
#define CONFIG_PWR_UP 0x02

#define STATUS_RX_DR 0x40
#define STATUS_TX_DS 0x20
#define STATUS_MAX_RT 0x10
#define STATUS_RX_P_NO_EMPTY 0x0E
#define STATUS_RX_P_NO_P1 0x02

#define FIFO_STATUS_RX_EMPTY 0x01
#define FIFO_STATUS_TX_EMPTY 0x10

#define ACTIVATE_DATA 0x73

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
  pin_t m1f;
  pin_t m1r;
  pin_t m2f;
  pin_t m2r;
  pin_t m3f;
  pin_t m3r;
  pin_t m4f;
  pin_t m4r;
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

static uint8_t status_value(const chip_state_t *chip) {
  uint8_t status = chip->regs[REG_STATUS] & (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
  status |= chip->rx_full ? STATUS_RX_P_NO_P1 : STATUS_RX_P_NO_EMPTY;
  return status;
}

static uint8_t fifo_status_value(const chip_state_t *chip) {
  uint8_t fifo = FIFO_STATUS_TX_EMPTY;
  if (!chip->rx_full) {
    fifo |= FIFO_STATUS_RX_EMPTY;
  }
  return fifo;
}

static void update_irq(chip_state_t *chip) {
  const bool irq_active = (chip->regs[REG_STATUS] & (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)) != 0;
  pin_write(chip->irq, irq_active ? LOW : HIGH);
}

static void set_thruster_outputs(pin_t forward_pin, pin_t reverse_pin, uint16_t thruster_value) {
  pin_write(forward_pin, thruster_value == 1 ? HIGH : LOW);
  pin_write(reverse_pin, thruster_value == 2 ? HIGH : LOW);
}

static void update_thruster_outputs(chip_state_t *chip) {
  if (chip->control_len < 16) {
    set_thruster_outputs(chip->m1f, chip->m1r, 0);
    set_thruster_outputs(chip->m2f, chip->m2r, 0);
    set_thruster_outputs(chip->m3f, chip->m3r, 0);
    set_thruster_outputs(chip->m4f, chip->m4r, 0);
    return;
  }

  set_thruster_outputs(chip->m1f, chip->m1r, read_u16_le(chip->control_payload, CTRL_M1_OFFSET));
  set_thruster_outputs(chip->m2f, chip->m2r, read_u16_le(chip->control_payload, CTRL_M2_OFFSET));
  set_thruster_outputs(chip->m3f, chip->m3r, read_u16_le(chip->control_payload, CTRL_M3_OFFSET));
  set_thruster_outputs(chip->m4f, chip->m4r, read_u16_le(chip->control_payload, CTRL_M4_OFFSET));
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

  write_u16_le(chip->telemetry_payload, 0, 0xFFFF);
  write_u16_le(chip->telemetry_payload, 2, 0);
  write_u16_le(chip->telemetry_payload, 4, 0);
  write_u16_le(chip->telemetry_payload, 6, 90);
  write_u16_le(chip->telemetry_payload, 8, 180);
  write_u16_le(chip->telemetry_payload, 10, 1200);
  write_u16_le(chip->telemetry_payload, 12, 35);
  write_u16_le(chip->telemetry_payload, 14, 0);
  write_u16_le(chip->telemetry_payload, 16, 2200);
  write_u16_le(chip->telemetry_payload, 18, 2026);
  write_u16_le(chip->telemetry_payload, 20, 4);
  write_u16_le(chip->telemetry_payload, 22, 3);
  write_u16_le(chip->telemetry_payload, 24, 12);
  write_u16_le(chip->telemetry_payload, 26, 0);
  write_u16_le(chip->telemetry_payload, 28, chip->telemetry_sequence % 60);
  write_u16_le(chip->telemetry_payload, 30, 0);

  chip->telemetry_sequence++;
  chip->telemetry_len = RF24_MAX_PAYLOAD;
  chip->rx_full = true;
  chip->regs[REG_STATUS] |= STATUS_RX_DR;
  update_irq(chip);
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

  chip->regs[REG_CONFIG] = 0x08;
  chip->regs[REG_EN_AA] = 0x3F;
  chip->regs[REG_EN_RXADDR] = 0x03;
  chip->regs[REG_SETUP_AW] = 0x03;
  chip->regs[REG_SETUP_RETR] = 0x03;
  chip->regs[REG_RF_CH] = 0x02;
  chip->regs[REG_RF_SETUP] = 0x0F;
  chip->regs[REG_DYNPD] = 0x00;
  chip->regs[REG_FEATURE] = 0x00;
}

static void finish_current_command(chip_state_t *chip) {
  if ((chip->current_cmd == CMD_W_TX_PAYLOAD || (chip->current_cmd & 0xF8) == CMD_W_ACK_PAYLOAD) &&
      chip->stream_index > 0) {
    chip->control_len = chip->stream_index > RF24_MAX_PAYLOAD ? RF24_MAX_PAYLOAD : chip->stream_index;
    update_thruster_outputs(chip);
  }

  if (chip->current_cmd == CMD_R_RX_PAYLOAD && chip->rx_full && chip->stream_index >= chip->telemetry_len) {
    clear_rx_payload(chip, false);
  }
}

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

  if (byte == CMD_R_RX_PL_WID) {
    return chip->rx_full ? chip->telemetry_len : 0;
  }

  if (byte == CMD_ACTIVATE) {
    return status_value(chip);
  }

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
  chip->m1f = pin_init("M1F", OUTPUT);
  chip->m1r = pin_init("M1R", OUTPUT);
  chip->m2f = pin_init("M2F", OUTPUT);
  chip->m2r = pin_init("M2R", OUTPUT);
  chip->m3f = pin_init("M3F", OUTPUT);
  chip->m3r = pin_init("M3R", OUTPUT);
  chip->m4f = pin_init("M4F", OUTPUT);
  chip->m4r = pin_init("M4R", OUTPUT);

  load_power_on_defaults(chip);

  pin_write(miso, LOW);
  pin_write(chip->irq, HIGH);
  set_thruster_outputs(chip->m1f, chip->m1r, 0);
  set_thruster_outputs(chip->m2f, chip->m2r, 0);
  set_thruster_outputs(chip->m3f, chip->m3r, 0);
  set_thruster_outputs(chip->m4f, chip->m4r, 0);

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

#if RF24_VERBOSE
  printf("[rf24] chip initialized with SPI API\n");
#endif
}
