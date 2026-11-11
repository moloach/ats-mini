/* SI4732 (Silicon Labs radio receiver) simulation for Wokwi.
 *
 * Emulates the PU2CLR SI4735 library command protocol over I2C 0x11:
 * every read starts with a status byte (CTS set, ERR clear), followed by
 * the command specific response. Tuned frequencies and properties are
 * tracked so frequency/RSQ reads return plausible values.
 *
 * Not simulated: actual RF/DSP (audio, RDS groups, SSB patches).
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "wokwi-api.h"

#define SI473X_ADDR 0x11

#define RESP_MAX 13

/* Response bytes follow AN332 layouts used by the PU2CLR SI4735 library. */
#define RSSI_FAKE 0x37
#define SNR_FAKE  0x28

typedef enum {
  CMD_POWER_UP       = 0x01,
  CMD_POWER_DOWN     = 0x11,
  CMD_GET_REV        = 0x10,
  CMD_SET_PROPERTY   = 0x12,
  CMD_GET_PROPERTY   = 0x13,
  CMD_GET_INT_STATUS = 0x14,
  CMD_FM_TUNE_FREQ   = 0x20,
  CMD_FM_TUNE_STATUS = 0x22,
  CMD_FM_RSQ_STATUS  = 0x23,
  CMD_FM_RDS_STATUS  = 0x24,
  CMD_FM_AGC_STATUS  = 0x27,
  CMD_AM_TUNE_FREQ   = 0x40,
  CMD_AM_TUNE_STATUS = 0x42,
  CMD_AM_RSQ_STATUS  = 0x43,
  CMD_AM_AGC_STATUS  = 0x47,
} cmd_t;

#define PROPERTY_SLOTS 32

typedef struct {
  i2c_dev_t i2c;
  pin_t     rst;

  uint8_t resp[RESP_MAX];
  uint8_t resp_len;
  uint8_t resp_pos;

  uint8_t cmd;
  uint8_t arg[8];
  uint8_t arg_len;

  uint16_t freq_hz;

  uint16_t prop_id[PROPERTY_SLOTS];
  uint16_t prop_val[PROPERTY_SLOTS];
} si4732_t;

static si4732_t si4732;

static void resp_clear(void)
{
  memset(si4732.resp, 0, sizeof(si4732.resp));
  si4732.resp[0] = 0x80; /* CTS=1, ERR=0 */
  si4732.resp_len = 1;
  si4732.resp_pos = 0;
}

static void resp_fill(uint8_t len)
{
  resp_clear();
  si4732.resp_len = len;
}

static void resp_set_freq(uint16_t freq)
{
  si4732.resp[2] = (uint8_t)(freq >> 8);
  si4732.resp[3] = (uint8_t)(freq & 0xff);
  si4732.resp[4] = RSSI_FAKE; /* RSSI  */
  si4732.resp[5] = SNR_FAKE;  /* SNR   */
}

static uint16_t property_read(uint16_t id)
{
  for(uint8_t i = 0; i < PROPERTY_SLOTS; i++)
    if(si4732.prop_id[i] == id && si4732.prop_id[i])
      return si4732.prop_val[i];
  return 0;
}

static void property_write(uint16_t id, uint16_t val)
{
  for(uint8_t i = 0; i < PROPERTY_SLOTS; i++) {
    if(si4732.prop_id[i] == id) { si4732.prop_val[i] = val; return; }
    if(!si4732.prop_id[i]) { si4732.prop_id[i] = id; si4732.prop_val[i] = val; return; }
  }
}

static void cmd_run(void)
{
  uint16_t freq;

  si4732.arg_len = 0;
  resp_clear();

  switch(si4732.cmd) {
    case CMD_GET_REV:
      resp_fill(9);
      si4732.resp[1] = 0x42;          /* part number low byte (SI4732/5) */
      si4732.resp[2] = 0x00;          /* firmware major */
      si4732.resp[3] = 0x2a;          /* firmware minor */
      si4732.resp[6] = 0x00;          /* chip revision */
      si4732.resp[7] = 0x00;          /* library revision */
      break;

    case CMD_GET_PROPERTY: {
      uint16_t val = property_read((uint16_t)((si4732.arg[2] << 8) | si4732.arg[3]));
      resp_fill(4);
      si4732.resp[2] = (uint8_t)(val >> 8);
      si4732.resp[3] = (uint8_t)(val & 0xff);
      break;
    }

    case CMD_FM_TUNE_FREQ:
    case CMD_AM_TUNE_FREQ:
      freq = (uint16_t)((si4732.arg[1] << 8) | si4732.arg[2]);
      if(freq) si4732.freq_hz = freq;
      break;

    case CMD_FM_TUNE_STATUS:
    case CMD_AM_TUNE_STATUS:
      resp_fill(8);
      si4732.resp[1] = 0x01;          /* VALID channel */
      resp_set_freq(si4732.freq_hz);
      break;

    case CMD_FM_RSQ_STATUS:
    case CMD_AM_RSQ_STATUS:
      resp_fill(si4732.cmd == CMD_FM_RSQ_STATUS ? 8 : 6);
      si4732.resp[1] = 0x00;          /* interrupt flags */
      si4732.resp[2] = 0x01;          /* VALID channel */
      si4732.resp[3] = 0xe4;          /* stereo pilot + full stereo blend (FM) */
      si4732.resp[4] = RSSI_FAKE;
      si4732.resp[5] = SNR_FAKE;
      break;

    case CMD_FM_RDS_STATUS:
      resp_fill(RESP_MAX);
      break;

    case CMD_FM_AGC_STATUS:
    case CMD_AM_AGC_STATUS:
      resp_fill(3);
      break;

    default:
      break;
  }
}

bool chip_i2c_connect(void *user_data, uint32_t address, bool read)
{
  (void)user_data;
  (void)address;
  if(read) si4732.resp_pos = 0;
  return true; /* ACK */
}

uint8_t chip_i2c_read(void *user_data)
{
  (void)user_data;
  uint8_t value = si4732.resp[si4732.resp_pos];
  if(si4732.resp_pos < RESP_MAX - 1) si4732.resp_pos++;
  return value;
}

bool chip_i2c_write(void *user_data, uint8_t data)
{
  (void)user_data;
  if(si4732.arg_len == 0)
    si4732.cmd = data;
  else if(si4732.arg_len < sizeof(si4732.arg))
    si4732.arg[si4732.arg_len - 1] = data;
  si4732.arg_len++;
  return true; /* ACK */
}

void chip_i2c_disconnect(void *user_data)
{
  (void)user_data;
  cmd_run();
}

static void chip_rst_change(void *user_data, pin_t pin, uint32_t value)
{
  (void)user_data;
  if(value == LOW) {
    memset(si4732.prop_id, 0, sizeof(si4732.prop_id));
    si4732.freq_hz = 0;
  }
}

void chip_init(void)
{
  si4732.rst = pin_init("RST", INPUT);
  si4732.resp_pos = 0;
  resp_clear();

  i2c_config_t i2c_config;
  memset(&i2c_config, 0, sizeof(i2c_config));
  i2c_config.address = SI473X_ADDR;
  i2c_config.scl = pin_init("SCL", INPUT_PULLUP);
  i2c_config.sda = pin_init("SDA", INPUT_PULLUP);
  i2c_config.connect = chip_i2c_connect;
  i2c_config.read = chip_i2c_read;
  i2c_config.write = chip_i2c_write;
  i2c_config.disconnect = chip_i2c_disconnect;
  i2c_config.user_data = &si4732;
  si4732.i2c = i2c_init(&i2c_config);
}
