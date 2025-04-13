/*
 *  hello-1.c - The simplest kernel module.
 */

#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/ieee802154.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <linux/etherdevice.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#include "decadriver/deca_regs.h"
#include "dw3000.h"

#define PARSE_ADDRESS(regFileID, idx)                                          \
  (((0x1F & ((regFileID + idx) >> 16)) << 9) |                                 \
   ((0x7F & (regFileID + idx)) << 2))

static bool dw3000_reg_writeable(struct device *dev, unsigned int reg) {
  return true;
}

static bool dw3000_reg_readable(struct device *dev, unsigned int reg) {
  bool rc;

  /* all writeable are also readable */
  rc = dw3000_reg_writeable(dev, reg);
  if (rc)
    return rc;

  return true;
}

static bool dw3000_reg_volatile(struct device *dev, unsigned int reg) {
  return true;
}

static bool dw3000_reg_precious(struct device *dev, unsigned int reg) {
  return false;
}

struct dw3000_state_change {
  struct dw3000_local *lp;
  int irq;

  struct hrtimer timer;
  struct spi_message msg;
  struct spi_transfer trx;
  u8 buf[256];

  void (*complete)(void *context);
  u8 from_state;
  u8 to_state;
  int trac;

  bool free;
};

struct dw3000_local {
  struct spi_device *spi;
  struct ieee802154_hw *hw;

  int speed_default;

  struct dw3000_config_t config;

  int irq_counter;

  struct {
    struct regmap *fast;
    struct regmap *full32;
    struct regmap *full16;
    struct regmap *full8;
  } regmap;
  struct gpio_desc *slp_tr;

  struct completion state_complete;
  struct dw3000_state_change state;

  struct sk_buff *tx_skb;
  struct dw3000_state_change tx;

  struct work_struct xmit_work;
  struct work_struct xmit_check_tx_work;

  u8 dgc_otp_set;
  u8 sleep_mode;
  u8 dblbuffon;

  u32 last_hw_timestamp;

  ktime_t hw_time;
  struct hrtimer timestamp_sync_timer;
  struct hrtimer tx_check_timer;

  u8 xmit_busy;
};

static struct regmap_config dw3000_regmap_fast_cfg = {
    .name = "DW3000 short address access regmap",
    .reg_bits = 1 + 6,
    .pad_bits = 1,
    .val_bits = 8,
    .write_flag_mask = 0x80,
    .read_flag_mask = 0x00,
    .max_register = 0x1f,
    .writeable_reg = dw3000_reg_writeable,
    .readable_reg = dw3000_reg_readable,
    .volatile_reg = dw3000_reg_volatile,
    .precious_reg = dw3000_reg_precious,

    .reg_format_endian = REGMAP_ENDIAN_BIG,
    .val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static struct regmap_config dw3000_regmap_full_cfg = {
    .name = "DW3000 full address access regmap",
    .reg_bits = 2 + 5 + 7 + 2,
    .val_bits = 32,
    .write_flag_mask = 0x0080 | DW3000_SPI_EAMRW,
    .read_flag_mask = 0x0000 | DW3000_SPI_EAMRW,
    .writeable_reg = dw3000_reg_writeable,
    .readable_reg = dw3000_reg_readable,
    .volatile_reg = dw3000_reg_volatile,
    .precious_reg = dw3000_reg_precious,

    .reg_format_endian = REGMAP_ENDIAN_BIG,
    .val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static struct dw3000_config_t dw3000_default_config = {
    .chan = 5,
    .dataRate = DWT_BR_6M8,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = 1,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .stsMode = DWT_STS_MODE_OFF,
    .sfdTO = (129 + 8 - 8),
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0,
};

static inline void dw3000_async_error(struct dw3000_local *lp,
                                      struct dw3000_state_change *ctx, int rc) {
  dev_err(&lp->spi->dev, "spi_async error %d\n", rc);

  return;
}

static int dw3000_async_write_command(struct dw3000_local *lp, u8 reg,
                                      struct dw3000_state_change *ctx,
                                      void (*complete)(void *context)) {
  int rc;

  ctx->buf[0] = (u8)((DW3000_SPI_WR_BIT >> 8) | (reg << 1) | DW3000_SPI_FAC);

  ctx->trx.len = 1;
  ctx->msg.complete = complete;

  rc = spi_async(lp->spi, &ctx->msg);
  if (rc) {
    dev_err(&lp->spi->dev, "Failed to write command: %d\n", rc);
    dw3000_async_error(lp, ctx, rc);
  }

  return rc;
}

static int dw3000_sync_write_command(struct dw3000_local *lp, u8 reg) {
  int rc;

  u8 buf[1] = {(u8)((DW3000_SPI_WR_BIT >> 8) | (reg << 1) | DW3000_SPI_FAC)};

  rc |= spi_write(lp->spi, buf, sizeof(buf));
  if (rc < 0) {
    dev_err(&lp->spi->dev, "Failed to write command: %d\n", rc);
    return rc;
  }

  return rc;
}

static void dw3000_async_write_completed(void *context) {
  struct dw3000_state_change *ctx = context;
  struct dw3000_local *lp = ctx->lp;

  return;
}

static int dw3000_modify8bit(struct dw3000_local *lp, u16 reg, u8 and, u8 or) {
  int rc;

  u8 buf[2] = {and, or };

  rc = regmap_raw_write(lp->regmap.full8, reg | DW3000_SPI_AND_OR_8, buf,
                        sizeof(buf));
  if (rc < 0) {
    dev_err(&lp->spi->dev, "Failed to modify 8-bit register: %d\n", rc);
    return rc;
  }

  return rc;
}

static int dw3000_modify16bit(struct dw3000_local *lp, u16 reg, u16 and,
                              u16 or) {
  int rc;

  u8 buf[4] = {and, and >> 8, or, or >> 8};

  rc = regmap_raw_write(lp->regmap.full16, reg | DW3000_SPI_AND_OR_16, buf,
                        sizeof(buf));
  if (rc < 0) {
    dev_err(&lp->spi->dev, "Failed to modify 16-bit register: %d\n", rc);
    return rc;
  }

  return rc;
}

static int dw3000_modify32bit(struct dw3000_local *lp, u16 reg, u32 and,
                              u32 or) {
  int rc;

  u8 buf[8] = {and, and >> 8, and >> 16, and >> 24,
               or,  or >> 8,  or >> 16,  or >> 24};

  rc = regmap_raw_write(lp->regmap.full32, reg | DW3000_SPI_AND_OR_32, buf,
                        sizeof(buf));
  if (rc < 0) {
    dev_err(&lp->spi->dev, "Failed to modify 32-bit register: %d\n", rc);
    return rc;
  }

  return rc;
}

#define dw3000_and8bit(lp, reg, and) dw3000_modify8bit(lp, reg, and, 0)
#define dw3000_or8bit(lp, reg, or) dw3000_modify8bit(lp, reg, -1, or)
#define dw3000_and16bit(lp, reg, and) dw3000_modify16bit(lp, reg, and, 0)
#define dw3000_or16bit(lp, reg, or) dw3000_modify16bit(lp, reg, -1, or)
#define dw3000_and32bit(lp, reg, and) dw3000_modify32bit(lp, reg, and, 0)
#define dw3000_or32bit(lp, reg, or) dw3000_modify32bit(lp, reg, -1, or)
#define dw3000_read8bit(lp, reg, val) regmap_read(lp->regmap.full8, reg, val)
#define dw3000_read16bit(lp, reg, val) regmap_read(lp->regmap.full16, reg, val)
#define dw3000_read32bit(lp, reg, val) regmap_read(lp->regmap.full32, reg, val)
#define dw3000_write8bit(lp, reg, val) regmap_write(lp->regmap.full8, reg, val)
#define dw3000_write16bit(lp, reg, val)                                        \
  regmap_write(lp->regmap.full16, reg, val)
#define dw3000_write32bit(lp, reg, val)                                        \
  regmap_write(lp->regmap.full32, reg, val)
#define dw3000_read(lp, reg, val, len)                                         \
  regmap_raw_read(lp->regmap.full8, reg, val, len)
#define dw3000_write(lp, reg, val, len)                                        \
  regmap_raw_write(lp->regmap.full8, reg, val, len)

static int dw3000_clearonconfig(struct dw3000_local *lp) {
  int rc = 0;

  rc |= dw3000_write16bit(lp, PARSE_ADDRESS(AON_DIG_CFG_ID, 0), 0x0000);

  struct reg_sequence seq[3] = {
      {PARSE_ADDRESS(ANA_CFG_ID, 0), 0},
      {PARSE_ADDRESS(AON_CTRL_ID, 0), 0},
      {PARSE_ADDRESS(AON_CTRL_ID, 0), AON_CTRL_ARRAY_SAVE_BIT_MASK},
  };

  rc |= regmap_multi_reg_write(lp->regmap.full8, seq, 3);

  if (rc) {
    dev_err(&lp->spi->dev,
            "Failed to clear AON_DIG_CFG, ANA_CFG and AON_CTRL: %d\n", rc);
    return rc;
  }

  return rc;
}

static int dw3000_checkidlerc(struct dw3000_local *lp) {
  int rc = 0;
  int val;

  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0), &val);

  printk(KERN_INFO "DW3000 SYS_STATUS: %04x\n", val);
  if (rc) {
    dev_err(&lp->spi->dev, "Failed to read SYS_STATUS: %d\n", rc);
    return rc;
  }

  return ((val & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}

static int dw3000_softreset(struct dw3000_local *lp) {
  int rc = 0;

  rc |= dw3000_clearonconfig(lp);

  mdelay(1);

  rc |= dw3000_or8bit(lp, PARSE_ADDRESS(CLK_CTRL_ID, 0), FORCE_SYSCLK_FOSC);

  rc |= dw3000_write8bit(lp, PARSE_ADDRESS(SOFT_RST_ID, 0), DWT_RESET_ALL);

  lp->sleep_mode = 0;
  lp->dblbuffon = DBL_BUFF_OFF;

  if (rc) {
    dev_err(&lp->spi->dev, "Failed to reset DW3000: %d\n", rc);
    return rc;
  }

  mdelay(1);

  return rc;
}

static u32 dw3000_optread(struct dw3000_local *lp, u16 address) {
  u32 ret_data = 0;
  int rc = 0;

  // Set manual access mode

  struct reg_sequence seq[3] = {
      {PARSE_ADDRESS(OTP_CFG_ID, 0), 0x0001},
      {PARSE_ADDRESS(OTP_ADDR_ID, 0), address},
      {PARSE_ADDRESS(OTP_CFG_ID, 0), 0x0002},
  };

  rc |= regmap_multi_reg_write(lp->regmap.full16, seq, 3);

  // Attempt a read from OTP address
  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(OTP_RDATA_ID, 0), &ret_data);
  if (rc) {
    dev_err(&lp->spi->dev, "Failed to read OTP_RDATA: %d\n", rc);
    return 0;
  }

  // Return the 32-bit read data
  return ret_data;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 *
 * @brief This function runs the PGF calibration. This is needed prior to
 * reception.
 *
 * input parameters
 *
 * return result of PGF calibration (DWT_ERROR/-1 = error)
 *
 */
static int dw3000_run_pgfcal(struct dw3000_local *lp) {
  int rc = 0;
  ;
  u32 data;
  u32 val = 0;
  u8 cnt, flag;
  // put into cal mode
  // Turn on delay mode
  data = (((u32)0x02) << RX_CAL_CFG_COMP_DLY_BIT_OFFSET) |
         (RX_CAL_CFG_CAL_MODE_BIT_MASK & 0x1);

  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(RX_CAL_CFG_ID, 0), data);

  // Trigger PGF Cal
  rc |= dw3000_or8bit(lp, PARSE_ADDRESS(RX_CAL_CFG_ID, 0),
                      RX_CAL_CFG_CAL_EN_BIT_MASK);

  for (flag = 1, cnt = 0; cnt < 3; cnt++) {

    rc |= dw3000_read8bit(lp, PARSE_ADDRESS(RX_CAL_STS_ID, 0), &val);

    if (val == 1) { // PGF cal is complete
      flag = 0;
      break;
    }

    udelay(20);
  }
  if (flag) {
    rc = -1;
  }

  // Put into normal mode
  struct reg_sequence seq[] = {
      {PARSE_ADDRESS(RX_CAL_CFG_ID, 0x0), 0},
      {PARSE_ADDRESS(RX_CAL_STS_ID, 0x0), 1} // clear the status
  };

  rc |= regmap_multi_reg_write(lp->regmap.full8, seq, 2);

  rc |= dw3000_or8bit(lp, PARSE_ADDRESS(RX_CAL_CFG_ID, 0x2), 0x1);

  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(RX_CAL_RESI_ID, 0x0), &val);

  if (val == ERR_RX_CAL_FAIL) {
    // PGF I Cal Fail
    dev_err(&lp->spi->dev, "PGF I Cal Fail\n");
    return -1;
  }

  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(RX_CAL_RESQ_ID, 0x0), &val);

  if (val == ERR_RX_CAL_FAIL) {
    // PGF Q Cal Fail
    dev_err(&lp->spi->dev, "PGF Q Cal Fail\n");
    return -1;
  }

  if (rc) {
    dev_err(&lp->spi->dev, "Failed to run PGF calibration: %d\n", rc);
    return rc;
  }

  return rc;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 *
 * @brief This function runs the PGF calibration. This is needed prior to
 * reception. Note: If the RX calibration routine fails the device receiver
 * performance will be severely affected, the application should reset and try
 * again
 *
 * input parameters
 * @param ldoen    -   if set to 1 the function will enable LDOs prior to
 * calibration and disable afterwards.
 *
 * return result of PGF calibration (DWT_ERROR/-1 = error)
 *
 */
int dw3000_pgf_cal(struct dw3000_local *lp, int ldoen) {
  int rc = 0;
  int temp;
  u32 val;

  // PGF needs LDOs turned on - ensure PGF LDOs are enabled
  if (ldoen == 1) {
    rc |= dw3000_read16bit(lp, PARSE_ADDRESS(LDO_CTRL_ID, 0), &val);

    rc |= dw3000_or16bit(lp, PARSE_ADDRESS(LDO_CTRL_ID, 0),
                         (LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK |
                          LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK |
                          LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
  }

  // Run PGF Cal
  rc |= dw3000_run_pgfcal(lp);

  // Turn off RX LDOs if previously off
  if (ldoen == 1) {
    rc |= dw3000_and16bit(lp, PARSE_ADDRESS(LDO_CTRL_ID, 0), val);
    if (rc < 0) {
      dev_err(&lp->spi->dev, "Failed to disable LDOs: %d\n", rc);
      return rc;
    }
  }

  if (rc) {
    dev_err(&lp->spi->dev, "PGF calibration failed: %d\n", rc);
    return rc;
  }

  return rc;
}

static int dw3000_readrxtimestamp(struct dw3000_local *lp, u64 *timestamp) {
  int rc = 0;

  switch (lp->dblbuffon) {
  case DBL_BUFF_ACCESS_BUFFER_1:
    rc |= dw3000_read(
        lp, PARSE_ADDRESS(INDIRECT_POINTER_B_ID, BUF1_RX_TIME - BUF1_RX_FINFO),
        timestamp, RX_TIME_RX_STAMP_LEN);
    break;
  case DBL_BUFF_ACCESS_BUFFER_0:
    rc |= dw3000_read(lp, PARSE_ADDRESS(BUF0_RX_TIME, 0), timestamp,
                      RX_TIME_RX_STAMP_LEN);
    break;
  default:
    rc |= dw3000_read(lp, PARSE_ADDRESS(RX_TIME_0_ID, 0), timestamp,
                      RX_TIME_RX_STAMP_LEN);
    break;
  }

  *timestamp = le64_to_cpu(*timestamp);
  *timestamp &= 0xFFFFFFFFFF;

  return rc;
}

static int dw3000_readtxtimestamp(struct dw3000_local *lp, u64 *timestamp) {
  int rc = 0;

  rc |= dw3000_read(lp, PARSE_ADDRESS(TX_TIME_LO_ID, 0), timestamp,
                    TX_TIME_TX_STAMP_LEN);

  *timestamp = le64_to_cpu(*timestamp);
  *timestamp &= 0xFFFFFFFFFF;

  return rc;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @brief This function sets the default values of the lookup tables depending
 * on the channel selected.
 *
 * input parameters
 * @param[in] channel - Channel that the device will be transmitting/receiving
 * on.
 *
 * no return value
 */
static int dw3000_configmrxlut(struct dw3000_local *lp, int channel) {
  int rc = 0;
  __u32 lut0, lut1, lut2, lut3, lut4, lut5, lut6 = 0;

  if (channel == 5) {
    lut0 = (__u32)CH5_DGC_LUT_0;
    lut1 = (__u32)CH5_DGC_LUT_1;
    lut2 = (__u32)CH5_DGC_LUT_2;
    lut3 = (__u32)CH5_DGC_LUT_3;
    lut4 = (__u32)CH5_DGC_LUT_4;
    lut5 = (__u32)CH5_DGC_LUT_5;
    lut6 = (__u32)CH5_DGC_LUT_6;
  } else {
    lut0 = (__u32)CH9_DGC_LUT_0;
    lut1 = (__u32)CH9_DGC_LUT_1;
    lut2 = (__u32)CH9_DGC_LUT_2;
    lut3 = (__u32)CH9_DGC_LUT_3;
    lut4 = (__u32)CH9_DGC_LUT_4;
    lut5 = (__u32)CH9_DGC_LUT_5;
    lut6 = (__u32)CH9_DGC_LUT_6;
  }
  struct reg_sequence lut_seq[] = {
      {PARSE_ADDRESS(DGC_LUT_0_CFG_ID, 0x0), lut0},
      {PARSE_ADDRESS(DGC_LUT_1_CFG_ID, 0x0), lut1},
      {PARSE_ADDRESS(DGC_LUT_2_CFG_ID, 0x0), lut2},
      {PARSE_ADDRESS(DGC_LUT_3_CFG_ID, 0x0), lut3},
      {PARSE_ADDRESS(DGC_LUT_4_CFG_ID, 0x0), lut4},
      {PARSE_ADDRESS(DGC_LUT_5_CFG_ID, 0x0), lut5},
      {PARSE_ADDRESS(DGC_LUT_6_CFG_ID, 0x0), lut6},
      {PARSE_ADDRESS(DGC_CFG0_ID, 0x0), DWT_DGC_CFG0},
      {PARSE_ADDRESS(DGC_CFG1_ID, 0x0), DWT_DGC_CFG1},
  };

  rc |= regmap_multi_reg_write(lp->regmap.full32, lut_seq, ARRAY_SIZE(lut_seq));
  if (rc) {
    dev_err(&lp->spi->dev, "Failed to write LUT registers: %d\n", rc);
  }

  return rc;
}

static int dw3000_configure(struct dw3000_local *lp,
                            struct dw3000_config_t *config) {
  int rc = 0;

  u8 chan = config->chan;
  u8 cnt, flag;

  u8 scp = ((config->rxCode > 24) || (config->txCode > 24)) ? 1 : 0;

  u8 mode =
      (config->phrMode == DWT_PHRMODE_EXT) ? SYS_CFG_PHR_MODE_BIT_MASK : 0;

  u16 sts_len;

  sts_len = GET_STS_REG_SET_VALUE((__u16)(config->stsLength));
  lp->sleep_mode &= (~(DWT_ALT_OPS | DWT_SEL_OPS3));

  u8 preamble_len;
  switch (config->txPreambLength) {
  case DWT_PLEN_32:
    preamble_len = 32;
    break;
  case DWT_PLEN_64:
    preamble_len = 64;
    break;
  case DWT_PLEN_72:
    preamble_len = 72;
    break;
  case DWT_PLEN_128:
    preamble_len = 128;
    break;
  default:
    preamble_len = 256;
    break;
  }

  rc |= dw3000_modify32bit(
      lp, PARSE_ADDRESS(SYS_CFG_ID, 0),
      ~(SYS_CFG_PHR_MODE_BIT_MASK | SYS_CFG_PHR_6M8_BIT_MASK |
        SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_PDOA_MODE_BIT_MASK |
        SYS_CFG_CP_SDC_BIT_MASK),
      ((__u32)config->pdoaMode) << SYS_CFG_PDOA_MODE_BIT_OFFSET |
          ((__u16)config->stsMode & DWT_STS_CONFIG_MASK)
              << SYS_CFG_CP_SPC_BIT_OFFSET |
          (SYS_CFG_PHR_6M8_BIT_MASK &
           ((__u32)config->phrRate << SYS_CFG_PHR_6M8_BIT_OFFSET)) |
          mode);

  if (scp) {
    // configure OPS tables for SCP mode
    lp->sleep_mode |=
        DWT_ALT_OPS |
        DWT_SEL_OPS1; // configure correct OPS table is kicked on wakeup
    rc |= dw3000_modify32bit(lp, PARSE_ADDRESS(OTP_CFG_ID, 0),
                             ~(OTP_CFG_OPS_ID_BIT_MASK),
                             DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);

    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(IP_CONFIG_LO_ID, 0),
                            IP_CONFIG_LO_SCP);
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(IP_CONFIG_HI_ID, 0),
                            IP_CONFIG_HI_SCP);

    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(STS_CONFIG_LO_ID, 0),
                            STS_CONFIG_LO_SCP);
    rc |= dw3000_write8bit(lp, PARSE_ADDRESS(STS_CONFIG_HI_ID, 0),
                           STS_CONFIG_HI_SCP);
  } else //
  {
    u16 sts_mnth;
    if (config->stsMode != DWT_STS_MODE_OFF) {
      //! Not needed
    }

    // configure OPS tables for non-SCP mode
    if (preamble_len >= 256) {
      lp->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS0;
      dw3000_modify32bit(lp, PARSE_ADDRESS(OTP_CFG_ID, 0),
                         ~(OTP_CFG_OPS_ID_BIT_MASK),
                         DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
    } else {
      dw3000_modify32bit(lp, PARSE_ADDRESS(OTP_CFG_ID, 0),
                         ~(OTP_CFG_OPS_ID_BIT_MASK),
                         DWT_OPSET_SHORT | OTP_CFG_OPS_KICK_BIT_MASK);
    }
  }

  rc |= dw3000_modify8bit(lp, PARSE_ADDRESS(DTUNE0_ID, 0),
                          (u8)~DTUNE0_PRE_PAC_SYM_BIT_MASK, config->rxPAC);

  rc |= dw3000_write8bit(lp, PARSE_ADDRESS(STS_CFG0_ID, 0), sts_len - 1);

  if (config->txPreambLength == DWT_PLEN_72) {
    rc |= dw3000_write8bit(lp, PARSE_ADDRESS(TX_FCTRL_HI_ID, 1), 8);
  } else {
    rc |= dw3000_write8bit(lp, PARSE_ADDRESS(TX_FCTRL_HI_ID, 1), 0);
  }

  if ((config->stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND) {
    // configure lower preamble detection threshold for no data STS mode
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(DTUNE3_ID, 0), PD_THRESH_NO_DATA);
  } else {
    // configure default preamble detection threshold for other modes
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(DTUNE3_ID, 0), PD_THRESH_DEFAULT);
  }

  u32 temp;
  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(CHAN_CTRL_ID, 0), &temp);

  temp &= (~(CHAN_CTRL_RX_PCODE_BIT_MASK | CHAN_CTRL_TX_PCODE_BIT_MASK |
             CHAN_CTRL_SFD_TYPE_BIT_MASK | CHAN_CTRL_RF_CHAN_BIT_MASK));

  if (chan == 9)
    temp |= CHAN_CTRL_RF_CHAN_BIT_MASK;

  temp |= (CHAN_CTRL_RX_PCODE_BIT_MASK &
           ((u32)config->rxCode << CHAN_CTRL_RX_PCODE_BIT_OFFSET));
  temp |= (CHAN_CTRL_TX_PCODE_BIT_MASK &
           ((u32)config->txCode << CHAN_CTRL_TX_PCODE_BIT_OFFSET));
  temp |= (CHAN_CTRL_SFD_TYPE_BIT_MASK &
           ((u32)config->sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(CHAN_CTRL_ID, 0), temp);

  rc |= dw3000_modify32bit(lp, PARSE_ADDRESS(TX_FCTRL_ID, 0),
                           ~(TX_FCTRL_TXBR_BIT_MASK | TX_FCTRL_TXPSR_BIT_MASK),
                           ((u32)config->dataRate << TX_FCTRL_TXBR_BIT_OFFSET) |
                               ((u32)config->txPreambLength)
                                   << TX_FCTRL_TXPSR_BIT_OFFSET);

  // DTUNE (SFD timeout)
  //  Don't allow 0 - SFD timeout will always be enabled
  if (config->sfdTO == 0) {
    config->sfdTO = DWT_SFDTOC_DEF;
  }

  rc |= dw3000_write16bit(lp, PARSE_ADDRESS(DTUNE0_ID, 2), config->sfdTO);

  // RF
  if (chan == 9) {
    // Setup TX analog for ch9
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(TX_CTRL_HI_ID, 0), RF_TXCTRL_CH9);

    rc |= dw3000_write16bit(lp, PARSE_ADDRESS(PLL_CFG_ID, 0), RF_PLL_CFG_CH9);
    // Setup RX analog for ch9
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(RX_CTRL_HI_ID, 0), RF_RXCTRL_CH9);
  } else {
    // Setup TX analog for ch5
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(TX_CTRL_HI_ID, 0), RF_TXCTRL_CH5);

    rc |= dw3000_write16bit(lp, PARSE_ADDRESS(PLL_CFG_ID, 0), RF_PLL_CFG_CH5);
  }

  struct reg_sequence seq[4] = {
      {PARSE_ADDRESS(LDO_RLOAD_ID, 1), LDO_RLOAD_VAL_B1},
      {PARSE_ADDRESS(TX_CTRL_LO_ID, 2), RF_TXCTRL_LO_B2},
      {PARSE_ADDRESS(PLL_CAL_ID, 0), RF_PLL_CFG_LD},
      {PARSE_ADDRESS(SYS_STATUS_ID, 0), SYS_STATUS_CP_LOCK_BIT_MASK},
  };

  rc |= regmap_multi_reg_write(lp->regmap.full8, seq, 4);

  rc |= dw3000_write16bit(lp, PARSE_ADDRESS(CLK_CTRL_ID, 0), DWT_AUTO_CLKS);

  rc |= dw3000_or8bit(lp, PARSE_ADDRESS(SEQ_CTRL_ID, 1),
                      SEQ_CTRL_AINIT2IDLE_BIT_MASK >> 8);

  for (flag = 1, cnt = 0; cnt < 10; cnt++) {
    udelay(20);

    u32 sys_status;
    rc |= dw3000_read8bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0), &sys_status);

    if (sys_status & SYS_STATUS_CP_LOCK_BIT_MASK) { // PLL is locked
      flag = 0;
      break;
    }
  }

  // if (flag) {
  //   dev_err(&lp->spi->dev, "PLL is not locked\n");
  //   return -EIO;
  // }

  if ((config->rxCode >= 9) &&
      (config->rxCode <= 24)) // only enable DGC for PRF 64
  {
    // load RX LUTs
    /* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
    if (lp->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP) {
      if (chan == 5) {
        rc |= dw3000_modify32bit(
            lp, PARSE_ADDRESS(OTP_CFG_ID, 0), ~(OTP_CFG_DGC_SEL_BIT_MASK),
            (DWT_DGC_SEL_CH5 << OTP_CFG_DGC_SEL_BIT_OFFSET) |
                OTP_CFG_DGC_KICK_BIT_MASK);
      } else if (chan == 9) {
        rc |= dw3000_modify32bit(
            lp, PARSE_ADDRESS(OTP_CFG_ID, 0), ~(OTP_CFG_DGC_SEL_BIT_MASK),
            (DWT_DGC_SEL_CH9 << OTP_CFG_DGC_SEL_BIT_OFFSET) |
                OTP_CFG_DGC_KICK_BIT_MASK);
      }
    }
    /* Else we manually program hard-coded values into the DGC registers. */
    else {
      rc |= dw3000_configmrxlut(lp, chan);
    }
    rc |= dw3000_modify16bit(lp, PARSE_ADDRESS(DGC_CFG_ID, 0),
                             (__u16)~DGC_CFG_THR_64_BIT_MASK,
                             DWT_DGC_CFG << DGC_CFG_THR_64_BIT_OFFSET);
  } else {
    rc |= dw3000_and8bit(lp, PARSE_ADDRESS(DGC_CFG_ID, 0),
                         ~DGC_CFG_RX_TUNE_EN_BIT_MASK);
  }

  rc |= dw3000_pgf_cal(lp, 1);

  rc |= dw3000_write16bit(lp, PARSE_ADDRESS(CIA_CONF_ID, 0), 32750);
  rc |= dw3000_write16bit(lp, PARSE_ADDRESS(TX_ANTD_ID, 0), 32750);

  if (rc) {
    dev_err(&lp->spi->dev, "Failed to configure DW3000: %d\n", rc);
    return rc;
  }

  return rc;
}

static int dw3000_hw_init(struct dw3000_local *lp, int mode) {
  int rc = 0;

  lp->sleep_mode = DWT_RUNSAR; // Configure RUN_SAR on wake by default as it is
                               // needed when running PGF_CAL

  u32 ldo_tune[2] = {0x00000000, 0x00000000};

  ldo_tune[0] = dw3000_optread(lp, LDOTUNELO_ADDRESS);
  ldo_tune[1] = dw3000_optread(lp, LDOTUNEHI_ADDRESS);

  u16 biastune =
      (dw3000_optread(lp, BIAS_TUNE_ADDRESS) >> 16) & BIAS_CTRL_BIAS_MASK;

  printk(KERN_INFO "Bias Tune: %x\n", biastune);

  if (ldo_tune[0] != 0 && ldo_tune[1] != 0 && biastune != 0) {
    rc |= dw3000_or16bit(lp, PARSE_ADDRESS(OTP_CFG_ID, 0), LDO_BIAS_KICK);

    rc |= dw3000_modify16bit(lp, PARSE_ADDRESS(BIAS_CTRL_ID, 0),
                             ~BIAS_CTRL_BIAS_MASK, biastune);
  }

  // Read DGC_CFG from OTP
  if (dw3000_optread(lp, DGC_TUNE_ADDRESS) == DWT_DGC_CFG0) {
    lp->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
  } else {
    lp->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
  }

  u32 init_trim = dw3000_optread(lp, XTRIM_ADDRESS) & 0x7f;
  if (init_trim == 0)
    init_trim = 0x2e;

  rc |= dw3000_write8bit(lp, PARSE_ADDRESS(XTAL_ID, 0), init_trim);
  if (rc) {
    dev_err(&lp->spi->dev, "Failed to set XTAL: %d\n", rc);
    return rc;
  }

  return rc;
}

static void dw3000_setup_spi_messages(struct dw3000_local *lp,
                                      struct dw3000_state_change *state) {
  state->lp = lp;
  state->irq = lp->spi->irq;
  spi_message_init(&state->msg);
  state->msg.context = state;
  state->trx.len = 2;
  state->trx.tx_buf = state->buf;
  state->trx.rx_buf = state->buf;
  spi_message_add_tail(&state->trx, &state->msg);
}

static irqreturn_t dw3000_isr_thr(int irq, void *data) {
  int rc = 0;

  struct dw3000_local *lp = (struct dw3000_local *)data;

  u32 sys_status;
  u32 fint;

  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0), &sys_status);

  if (!(sys_status & SYS_STATUS_IRQS_BIT_MASK))
    return IRQ_HANDLED;

  rc |= dw3000_read8bit(lp, PARSE_ADDRESS(FINT_STAT_ID, 0), &fint);

  printk(KERN_INFO "DW3000 FINT_STAT: 0x%X SYS_STATUS: 0x%04X\n", fint,
         sys_status);

  u8 irq_handled = 0;

  if (fint & FINT_STAT_RXOK_BIT_MASK &&
      sys_status & SYS_STATUS_RXFCG_BIT_MASK) {
    irq_handled = 1;

    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0),
                            SYS_STATUS_ALL_RX_GOOD);

    u32 rx_info;
    rc |= dw3000_read32bit(lp, PARSE_ADDRESS(RX_FINFO_ID, 0), &rx_info);

    u16 rx_len = rx_info & RX_FINFO_RXFLEN_BIT_MASK;

    if (rx_len > 0) {
      __u32 rx_buff_addr;
      u8 lqi = 127;
      rx_buff_addr = RX_BUFFER_0_ID;
      struct sk_buff *skb;
      skb = dev_alloc_skb(IEEE802154_MTU);

      if (!skb) {
        dev_vdbg(&lp->spi->dev, "failed to allocate sk_buff\n");
      }

      void *data = skb_put(skb, rx_len - 2);

      rc |= dw3000_read(lp, PARSE_ADDRESS(rx_buff_addr, 0), data, rx_len - 2);

      if (rc) {
        dev_err(&lp->spi->dev, "Failed to read RX buffer: %d\n", rc);
        goto complete_irq;
      }

      if (rx_info & RX_FINFO_RNG_BIT_MASK) {
        u64 ts = 0;
        rc |= dw3000_readrxtimestamp(lp, &ts);

        struct skb_shared_hwtstamps *hwts;

        hwts = skb_hwtstamps(skb);
        hwts->hwtstamp = ns_to_ktime(ts);
        printk(KERN_INFO "DW3000 RX Timestamp: %llu\n", ts);
      }

      ieee802154_rx_irqsafe(lp->hw, skb, lqi);

      print_hex_dump_bytes("DW3000 RX buffer: ", DUMP_PREFIX_OFFSET, data,
                           rx_len - 2);
    }
  }

  if (fint & FINT_STAT_TXOK_BIT_MASK &&
      sys_status & SYS_STATUS_TXFRS_BIT_MASK && lp->xmit_busy) {
    irq_handled = 1;
    lp->xmit_busy = 0;

    rc |= dw3000_write8bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0),
                           SYS_STATUS_ALL_TX);

    struct sk_buff *skb = lp->tx_skb;
    struct skb_shared_info *shinfo = skb_shinfo(skb);

    u8 request_ts = shinfo->tx_flags & SKBTX_HW_TSTAMP &&
                    shinfo->tx_flags & SKBTX_IN_PROGRESS &&
                    !(shinfo->tx_flags & SKBTX_SCHED_TSTAMP);

    if (request_ts) {
      struct skb_shared_hwtstamps hwtstamps;
      u64 ts = 0;

      rc |= dw3000_readtxtimestamp(lp, &ts);

      memset(&hwtstamps, 0, sizeof(hwtstamps));
      hwtstamps.hwtstamp = ns_to_ktime(ts);

      skb_tstamp_tx(skb, &hwtstamps);

      printk(KERN_INFO "DW3000: requested TX timestamp: %llu\n", ts);
    } else {
      printk(KERN_INFO "DW3000: TX timestamp not requested\n");
    }

    ieee802154_xmit_complete(lp->hw, lp->tx_skb, 0);
  }

  if (fint & FINT_STAT_RXERR_BIT_MASK) {
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0),
                            SYS_STATUS_ALL_RX_ERR);
  }

  if (fint & FINT_STAT_SYS_PANIC_BIT_MASK || !fint && !sys_status) {
    irq_handled = 1;

    printk(KERN_INFO "DW3000: System Panic\n");

    u32 sys_status_hi;

    rc |= dw3000_read16bit(lp, PARSE_ADDRESS(SYS_STATUS_HI_ID, 0),
                           &sys_status_hi);

    printk(KERN_INFO "DW3000: SYS_STATUS_HI: 0x%04X\n", sys_status_hi);

    if (sys_status_hi & SYS_STATUS_HI_CMD_ERR_BIT_MASK) {
      printk(KERN_INFO "DW3000: Command Error\n");

      rc |= dw3000_write16bit(lp, PARSE_ADDRESS(SYS_STATUS_HI_ID, 0),
                              SYS_STATUS_HI_CMD_ERR_BIT_MASK);
    }

    rc |= dw3000_sync_write_command(lp, CMD_CLR_IRQS);
  }

  if (!irq_handled) {
    rc |= dw3000_sync_write_command(lp, CMD_CLR_IRQS);
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0), 0xffffffff);
    rc |= dw3000_write16bit(lp, PARSE_ADDRESS(SYS_STATUS_HI_ID, 0), 0xffff);
    rc |= dw3000_sync_write_command(lp, CMD_TXRXOFF);
  }

complete_irq:
  rc |= dw3000_sync_write_command(lp, CMD_RX);
  rc |= dw3000_sync_write_command(lp, CMD_CLR_IRQS);

  if (rc) {
    dev_err(&lp->spi->dev, "Failed to handle interrupt: %d\n", rc);
  }

  return IRQ_HANDLED;
}

static irqreturn_t dw3000_isr(int irq, void *data) {
  struct dw3000_local *lp = (struct dw3000_local *)data;

  // disable_irq_nosync(irq);

  return IRQ_WAKE_THREAD;
}

static void dw3000_async_timestamp_completed(void *context) {
  struct dw3000_state_change *ctx = context;
  struct dw3000_local *lp = ctx->lp;

  u32 sys_time =
      ctx->buf[5] << 24 | ctx->buf[4] << 16 | ctx->buf[3] << 8 | ctx->buf[2];

  long diff = (sys_time - lp->last_hw_timestamp + (1UL << 32)) % (1UL << 32);
  lp->last_hw_timestamp = sys_time;

  lp->hw_time = ktime_add_ns(lp->hw_time, 4 * diff);

  printk(KERN_INFO "DW3000 HW Time: %lld.%lld s\n",
         ktime_to_ns(lp->hw_time) / 1000000000,
         ktime_to_ns(lp->hw_time) / 1000000 % 1000);

  kfree(ctx);
}

// Timer Callback function. This will be called when timer expires
enum hrtimer_restart dw3000_timestamp_sync_timer(struct hrtimer *timer) {
  /* do your timer stuff here */
  int rc = 0;

  struct dw3000_local *lp =
      container_of(timer, struct dw3000_local, timestamp_sync_timer);
  hrtimer_forward_now(timer, ktime_set(0, 1000 * 1000 * 1000)); // 100 ms

  // printk(KERN_INFO "DW3000: HW timer synchronization\n");

  struct dw3000_state_change *ctx;

  ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);

  if (!ctx) {
    return HRTIMER_RESTART;
  }

  dw3000_setup_spi_messages(lp, ctx);

  u16 address =
      PARSE_ADDRESS(SYS_TIME_ID, 0) | DW3000_SPI_EAMRW << 8 | DW3000_SPI_RD_BIT;

  ctx->buf[0] = (address >> 8) & 0xff;
  ctx->buf[1] = address & 0xff;

  ctx->trx.len = 6;
  ctx->msg.complete = dw3000_async_timestamp_completed;

  rc |= spi_async(lp->spi, &ctx->msg);
  if (rc) {
    dev_err(&lp->spi->dev, "Failed to write command: %d\n", rc);
  }

  return HRTIMER_RESTART;
}

static void dw3000_xmit_completed(void *context) {
  struct dw3000_state_change *ctx = context;
  struct dw3000_local *lp = ctx->lp;

  kfree(ctx);
}

static void dw3000_tx_check_work(struct work_struct *work) {
  int rc = 0;

  struct dw3000_local *lp =
      container_of(work, struct dw3000_local, xmit_check_tx_work);

  printk(KERN_INFO "DW3000: TX timeout reached, stopping TX\n");

  rc |= dw3000_sync_write_command(lp, CMD_TXRXOFF);
  rc |= dw3000_sync_write_command(lp, CMD_RX);
  lp->xmit_busy = 0;

  struct sk_buff *skb = lp->tx_skb;

  ieee802154_xmit_error(lp->hw, skb, IEEE802154_SYSTEM_ERROR);

  return;
}

enum hrtimer_restart dw3000_tx_check_timer(struct hrtimer *timer) {
  int rc = 0;

  struct dw3000_local *lp =
      container_of(timer, struct dw3000_local, tx_check_timer);

  if (!lp->xmit_busy)
    return HRTIMER_NORESTART;

  schedule_work(&lp->xmit_check_tx_work);

  return HRTIMER_NORESTART;
}

static void dw3000_xmit_work(struct work_struct *work) {
  int rc = 0;

  struct dw3000_local *lp = container_of(work, struct dw3000_local, xmit_work);
  struct dw3000_state_change *ctx = &lp->tx;

  rc |= dw3000_sync_write_command(lp, CMD_TXRXOFF);

  struct sk_buff *skb = lp->tx_skb;
  struct skb_shared_info *shinfo = skb_shinfo(skb);

  u8 delayed_tx = shinfo->tx_flags & SKBTX_SCHED_TSTAMP;
  u8 request_ts = !delayed_tx && shinfo->tx_flags & SKBTX_HW_TSTAMP;

  // write TX cntrl
  u32 ranging = delayed_tx | request_ts ? 1 : 0;
  u32 txFrameLength = skb->len + 2;
  u32 txBufferOffset = 0;

  u32 val = txFrameLength |
            ((__u32)(txBufferOffset + DWT_TX_BUFF_OFFSET_ADJUST)
             << TX_FCTRL_TXB_OFFSET_BIT_OFFSET) |
            ((!!ranging) << TX_FCTRL_TR_BIT_OFFSET);

  // write TX

  rc |= dw3000_modify32bit(lp, PARSE_ADDRESS(TX_FCTRL_ID, 0),
                           ~(TX_FCTRL_TXB_OFFSET_BIT_MASK |
                             TX_FCTRL_TR_BIT_MASK | TX_FCTRL_TXFLEN_BIT_MASK),
                           val);

  if (delayed_tx && (skb->skb_mstamp_ns & (1ULL << 63))) {
    rc |= dw3000_write(lp, PARSE_ADDRESS(TX_BUFFER_ID, 0), skb->data, skb->len - 4);

    dw3000_write8bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 3),
                     (SYS_STATUS_HPDWARN_BIT_MASK >> 24));

    u32 sys_time;
    u64 rx_ts = skb->skb_mstamp_ns;
                     
    dw3000_write8bit(lp, PARSE_ADDRESS(SYS_TIME_ID, 0), 0x00);
    dw3000_read32bit(lp, PARSE_ADDRESS(SYS_TIME_ID, 0), &sys_time);

    u32 tx_time = sys_time + 125000;
    tx_time &= 0xFFFFFFFE;
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(DX_TIME_ID, 0), cpu_to_le32(tx_time));

    u32 delay = ((u64)tx_time << 8) - rx_ts;
    delay = cpu_to_le32(delay);

    rc |= dw3000_write(lp, PARSE_ADDRESS(TX_BUFFER_ID, skb->len - 4), &delay, 4);
  }
  else if(delayed_tx) {
    u32 ts = skb->skb_mstamp_ns;
    ts = cpu_to_le32(ts);
    rc |= dw3000_write32bit(lp, PARSE_ADDRESS(DX_TIME_ID, 0), ts);
    rc |= dw3000_write(lp, PARSE_ADDRESS(TX_BUFFER_ID, 0), skb->data, skb->len);

    dw3000_write8bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 3),
                     (SYS_STATUS_HPDWARN_BIT_MASK >> 24));
  }
  else {
    rc |= dw3000_write(lp, PARSE_ADDRESS(TX_BUFFER_ID, 0), skb->data, skb->len);
  }

  // write TX start

  enable_irq(lp->spi->irq);
  rc |= dw3000_sync_write_command(lp, delayed_tx ? CMD_DTX_W4R : CMD_TX_W4R);

  if (delayed_tx) {
    u32 checkTxOk;
    rc |= dw3000_read8bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 3), &checkTxOk);

    if (checkTxOk & (SYS_STATUS_HPDWARN_BIT_MASK >>
                     24)) // Transmit Delayed Send set over Half a Period away.
    {
      printk(KERN_INFO "DW3000: Transmit Delayed Send set over Half a Period "
                       "away. TX will be sent in the next half period\n");
      goto tx_error;
    } else {
      u32 sys_state;
      rc |= dw3000_read8bit(lp, PARSE_ADDRESS(SYS_STATE_LO_ID, 2), &sys_state);
      if (sys_state == (DW_SYS_STATE_TXERR >> 16))
        goto tx_error;
    }
  }
  lp->xmit_busy = 1;

  hrtimer_start(&lp->tx_check_timer, ktime_set(0, 100 * 1000 * 1000),
                HRTIMER_MODE_REL);


  if (request_ts) {
    shinfo->tx_flags |= SKBTX_IN_PROGRESS;
    skb_tx_timestamp(skb);
    printk(KERN_INFO "DW3000: TX timestamp is being requested\n");
  }

  if (delayed_tx) {
    u64 ts = skb->skb_mstamp_ns;
    ts = cpu_to_le64(ts);
    printk(KERN_INFO "DW3000: Delayed TX timestamp %llu\n", ts << 8);
  }

  // setup timer for tx check
  return;

tx_error:
  printk(KERN_INFO "DW3000: TX error\n");

  rc |= dw3000_sync_write_command(lp, CMD_TXRXOFF);
  rc |= dw3000_sync_write_command(lp, CMD_RX);

  lp->xmit_busy = 0;

  if(skb)
    ieee802154_xmit_error(lp->hw, skb, IEEE802154_SYSTEM_ERROR);

  return;
}

static int dw3000_xmit(struct ieee802154_hw *hw, struct sk_buff *skb) {
  struct dw3000_local *lp = hw->priv;
  struct dw3000_state_change *ctx = &lp->tx;

  disable_irq_nosync(lp->spi->irq);

  if (lp->xmit_busy) {
    printk(KERN_INFO "DW3000: Xmit busy\n");
    return -EBUSY;
  }

  if (skb->len > IEEE802154_MTU) {
    printk(KERN_INFO "DW3000: Frame too long\n");
    return -EINVAL;
  }

  if (skb->data == NULL) {
    printk(KERN_INFO "DW3000: Invalid frame\n");
    return -EINVAL;
  }

  lp->tx_skb = skb;
  // run xmit_work
  schedule_work(&lp->xmit_work);

  return 0;
}

static int dw3000_ed(struct ieee802154_hw *hw, u8 *level) {
  WARN_ON(!level);

  // TODO! Implement ED

  return 0;
}

static void dw3000_set_channel_cb(void *context) {
  struct dw3000_state_change *ctx = context;
  struct dw3000_local *lp = ctx->lp;

  int rc = 0;

  rc |= dw3000_configure(lp, &lp->config);

  rc |= dw3000_sync_write_command(lp, CMD_RX);

  return;
}

static int dw3000_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel) {
  struct dw3000_local *lp = hw->priv;
  int rc = 0;

  printk(KERN_INFO "DW3000: Setting channel %d\n", channel);

  if (channel != 5 && channel != 9) {
    dev_err(&lp->spi->dev, "Invalid channel: %d\n", channel);
    return -EINVAL;
  }

  lp->config.chan = channel;

  rc |= dw3000_sync_write_command(lp, CMD_TXRXOFF);

  rc |= dw3000_configure(lp, &lp->config);

  rc |= dw3000_sync_write_command(lp, CMD_RX);

  return rc;
}

static int dw3000_set_hw_addr_filt(struct ieee802154_hw *hw,
                                   struct ieee802154_hw_addr_filt *filt,
                                   unsigned long changed) {
  int rc = 0;
  struct dw3000_local *lp = hw->priv;

  if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
    u16 short_addr = filt->short_addr;
    printk(KERN_INFO "DW3000: Short address changed to 0x%04X\n", short_addr);

    rc |= dw3000_write16bit(
        lp, PARSE_ADDRESS(PANADR_ID, PANADR_SHORTADDR_BIT_OFFSET), short_addr);
  }
  if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
    u64 ieee_addr = filt->ieee_addr;
    printk(KERN_INFO "DW3000: Extended address changed to 0x%016llX\n",
           ieee_addr);

    rc |=
        dw3000_write(lp, PARSE_ADDRESS(EUI_64_LO_ID, 0), (void *)&ieee_addr, 8);
  }
  if (changed & IEEE802154_AFILT_PANID_CHANGED) {
    u16 pan_id = filt->pan_id;
    printk(KERN_INFO "DW3000: PAN ID changed to 0x%04X\n", pan_id);

    rc |= dw3000_write16bit(
        lp, PARSE_ADDRESS(PANADR_ID, PANADR_PAN_ID_BYTE_OFFSET), pan_id);
  }
  if (changed & IEEE802154_AFILT_PANC_CHANGED) {
    u8 pan_coordinator = filt->pan_coord;
    printk(KERN_INFO
           "DW3000: Frame address filtering as a pan coordinator %s\n",
           pan_coordinator ? "enabled" : "disabled");

    if (pan_coordinator) {
      rc |= dw3000_or16bit(lp, PARSE_ADDRESS(ADR_FILT_CFG_ID, 0),
                           ADR_FILT_CFG_FFBC_BIT_MASK);
    } else {
      rc |= dw3000_and16bit(lp, PARSE_ADDRESS(ADR_FILT_CFG_ID, 0),
                            ~ADR_FILT_CFG_FFBC_BIT_MASK);
    }
  }

  return rc;
}

static int dw3000_set_promiscuous_mode(struct ieee802154_hw *hw, bool on) {
  int rc = 0;
  struct dw3000_local *lp = hw->priv;

  if (on) {
    printk(KERN_INFO "DW3000: Promiscuous mode enabled\n");

    rc |= dw3000_and32bit(lp, PARSE_ADDRESS(SYS_CFG_ID, 0),
                          ~SYS_CFG_FFEN_BIT_MASK);
  } else {
    printk(KERN_INFO "DW3000: Promiscuous mode disabled\n");

    rc |=
        dw3000_or32bit(lp, PARSE_ADDRESS(SYS_CFG_ID, 0), SYS_CFG_FFEN_BIT_MASK);
  }

  return rc;
}

static int dw3000_start(struct ieee802154_hw *hw) {
  int rc = 0;
  struct dw3000_local *lp = hw->priv;

  // print message
  printk(KERN_INFO "DW3000: starting sequence\n");

  // clear interrupts
  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_STATUS_ID, 0), -1);
  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_STATUS_HI_ID, 0), -1);

  // enable interrupts
  rc |=
      dw3000_or32bit(lp, PARSE_ADDRESS(SYS_CFG_ID, 0), SYS_CFG_RXAUTR_BIT_MASK);

  // Set RX timeout
  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(RX_FWTO_ID, 0), 0xfffff);

  // Set frame filtering
  rc |= dw3000_or16bit(lp, PARSE_ADDRESS(ADR_FILT_CFG_ID, 0),
                       ADR_FILT_CFG_FFAB_BIT_MASK | ADR_FILT_CFG_FFAD_BIT_MASK |
                           ADR_FILT_CFG_FFAA_BIT_MASK |
                           ADR_FILT_CFG_FFAM_BIT_MASK);
  // enable frame filtering
  rc |= dw3000_or32bit(lp, PARSE_ADDRESS(SYS_CFG_ID, 0), SYS_CFG_FFEN_BIT_MASK);

  enable_irq(lp->spi->irq);

  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_ENABLE_LO_ID, 0),
                          (SYS_STATUS_RXOK | SYS_STATUS_TXFRS_BIT_MASK |
                           SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR) &
                              ~(SYS_STATUS_RXSTO_BIT_MASK));
  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_ENABLE_HI_ID, 0),
                          SYS_ENABLE_HI_CMD_ERR_ENABLE_BIT_MASK);

  rc |= dw3000_sync_write_command(lp, CMD_RX);

  if (rc) {
    dev_err(&lp->spi->dev, "Failed to start dw3000: %d\n", rc);
    return rc;
  }

  return 0;
}

static void dw3000_stop(struct ieee802154_hw *hw) {
  int rc = 0;
  struct dw3000_local *lp = hw->priv;

  printk(KERN_INFO "DW3000: stopping sequence\n");

  rc |= dw3000_sync_write_command(lp, CMD_TXRXOFF);

  rc |= dw3000_write32bit(lp, PARSE_ADDRESS(SYS_ENABLE_LO_ID, 0), 0x00);

  if (rc) {
    dev_err(&lp->spi->dev, "Failed to stop dw3000: %d\n", rc);
    return;
  }

  disable_irq(lp->spi->irq);

  return;
}

static const struct ieee802154_ops dw3000_ops = {
    .owner = THIS_MODULE,
    .xmit_async = dw3000_xmit,
    .ed = dw3000_ed,
    .set_channel = dw3000_set_channel,
    .start = dw3000_start,
    .stop = dw3000_stop,
    .set_hw_addr_filt = dw3000_set_hw_addr_filt,
    .set_promiscuous_mode = dw3000_set_promiscuous_mode};

static int dw3000_probe(struct spi_device *spi) {
  printk(KERN_INFO "DW3000: driver probing\n");

  struct ieee802154_hw *hw;
  struct dw3000_local *lp;
  struct gpio_desc *slp_tr;
  unsigned int status;
  int rc, irq_type;
  u8 xtal_trim;

  if (!spi->irq) {
    dev_err(&spi->dev, "no IRQ specified\n");
    return -EINVAL;
  }

  hw = ieee802154_alloc_hw(sizeof(*lp), &dw3000_ops);
  if (!hw)
    return -ENOMEM;

  hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_RX_OMIT_CKSUM |
              IEEE802154_HW_AFILT | IEEE802154_HW_PROMISCUOUS;

  hw->phy->supported.channels[4] = 1 << 5 | 1 << 9;
  hw->phy->current_page = 4;
  hw->phy->current_channel = 5;

  lp = hw->priv;
  lp->hw = hw;
  lp->spi = spi;
  lp->slp_tr = slp_tr;
  lp->last_hw_timestamp = 0;
  lp->hw_time = ktime_set(0, 0);
  hw->parent = &spi->dev;
  lp->config = dw3000_default_config;

  INIT_WORK(&lp->xmit_work, dw3000_xmit_work);
  INIT_WORK(&lp->xmit_check_tx_work, dw3000_tx_check_work);

  lp->speed_default = spi->max_speed_hz;
  printk(KERN_INFO "DW3000: Max SPI speed: %d Hz\n", lp->speed_default);
  spi->max_speed_hz = lp->speed_default < 1000000 ? lp->speed_default : 1000000;

  spi_set_drvdata(spi, lp);

  lp->regmap.fast = devm_regmap_init_spi(spi, &dw3000_regmap_fast_cfg);
  if (IS_ERR(lp->regmap.fast)) {
    rc = PTR_ERR(lp->regmap.fast);
    dev_err(&spi->dev, "Failed to allocate short address register map: %d\n",
            rc);
    return rc;
  }

  dw3000_regmap_full_cfg.val_bits = 32;
  lp->regmap.full32 = devm_regmap_init_spi(spi, &dw3000_regmap_full_cfg);
  if (IS_ERR(lp->regmap.full32)) {
    rc = PTR_ERR(lp->regmap.full32);
    dev_err(&spi->dev, "Failed to allocate full address register map: %d\n",
            rc);
    return rc;
  }

  dw3000_regmap_full_cfg.val_bits = 16;
  lp->regmap.full16 = devm_regmap_init_spi(spi, &dw3000_regmap_full_cfg);
  if (IS_ERR(lp->regmap.full16)) {
    rc = PTR_ERR(lp->regmap.full16);
    dev_err(&spi->dev, "Failed to allocate full address register map: %d\n",
            rc);
    return rc;
  }

  dw3000_regmap_full_cfg.val_bits = 8;
  lp->regmap.full8 = devm_regmap_init_spi(spi, &dw3000_regmap_full_cfg);
  if (IS_ERR(lp->regmap.full8)) {
    rc = PTR_ERR(lp->regmap.full8);
    dev_err(&spi->dev, "Failed to allocate full address register map: %d\n",
            rc);
    return rc;
  }

  dw3000_setup_spi_messages(lp, &lp->state);
  dw3000_setup_spi_messages(lp, &lp->tx);

  irq_type = irq_get_trigger_type(spi->irq);
  if (!irq_type)
    irq_type = IRQF_TRIGGER_RISING;

  rc =
      devm_request_threaded_irq(&spi->dev, spi->irq, dw3000_isr, dw3000_isr_thr,
                                irq_type, dev_name(&spi->dev), lp);
  if (rc) {
    dev_err(&spi->dev, "Failed to request IRQ %d: %d\n", spi->irq, rc);
    return rc;
  }

  disable_irq(spi->irq);

  if (rc) {
    printk(KERN_ERR "Failed to request IRQ %d: %d\n", spi->irq, rc);
  }

  rc = 0;

  rc |= dw3000_softreset(lp);

  int attempts = 10;
  while (!dw3000_checkidlerc(lp)) {
    mdelay(100);
    printk(KERN_INFO "IDLE_RC not set\n");
    if (attempts-- == 0) {
      dev_err(&spi->dev, "Failed to set IDLE_RC\n");
      rc = -ENODEV;
      goto free_dev;
    }
  }

  spi->max_speed_hz = lp->speed_default;

  u32 dev_id;
  rc |= dw3000_read32bit(lp, PARSE_ADDRESS(DEV_ID_ID, 0), &dev_id);

  printk(KERN_INFO "Device ID: 0x%X\n", dev_id);

  if (dev_id != 0xDECA0302) {
    dev_err(&spi->dev, "Invalid device ID: 0x%X\n", dev_id);
    dev_err(&spi->dev, "Expected: 0xDECA0302\n");
    dev_err(&spi->dev, "Please check your connections\n");
    rc = -ENODEV;
    goto free_dev;
  }

  u32 partID = dw3000_optread(lp, PARTID_ADDRESS);
  u32 lotID = dw3000_optread(lp, LOTID_ADDRESS);

  printk(KERN_INFO "DW3000 Part ID: %x\n", partID);
  printk(KERN_INFO "DW3000 Lot ID: %x\n", lotID);

  rc |= dw3000_hw_init(lp, DWT_DW_INIT);

  rc |= dw3000_configure(lp, &lp->config);

  // hrtimer_init(&lp->timestamp_sync_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  // lp->timestamp_sync_timer.function = dw3000_timestamp_sync_timer;
  // hrtimer_start(&lp->timestamp_sync_timer, ktime_set(0, 1000 * 1000 * 1000),
  //               HRTIMER_MODE_REL); // 100 ms

  hrtimer_init(&lp->tx_check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  lp->tx_check_timer.function = dw3000_tx_check_timer;

  rc = ieee802154_register_hw(lp->hw);
  if (rc)
    goto free_dev;

  return rc;

free_dev:
  disable_irq(spi->irq);
  devm_free_irq(&spi->dev, spi->irq, lp);
  if (lp->hw)
  {
    spi->max_speed_hz = lp->speed_default;
    ieee802154_free_hw(lp->hw);
  }

  return rc;
}

static void dw3000_remove(struct spi_device *spi) {
  struct dw3000_local *lp = spi_get_drvdata(spi);
  printk(KERN_INFO "DW3000: remove\n");

  disable_irq(spi->irq);
  // hrtimer_cancel(&lp->timestamp_sync_timer);
  hrtimer_cancel(&lp->tx_check_timer);
  devm_free_irq(&spi->dev, spi->irq, lp);

  ieee802154_unregister_hw(lp->hw);
  ieee802154_free_hw(lp->hw);
  dev_dbg(&spi->dev, "unregistered dw3000\n");
}

static const struct of_device_id dw3000_of_match[] = {
    {
        .compatible = "qorvo,dw3000",
    },
    {},
};
MODULE_DEVICE_TABLE(of, dw3000_of_match);

static const struct spi_device_id dw3000_device_id[] = {
    {
        .name = "dw3000",
    },
    {},
};
MODULE_DEVICE_TABLE(spi, dw3000_device_id);

static struct spi_driver dw3000_driver = {
    .id_table = dw3000_device_id,
    .driver =
        {
            .of_match_table = dw3000_of_match,
            .name = "dw3000",
        },
    .probe = dw3000_probe,
    .remove = dw3000_remove,
};

module_spi_driver(dw3000_driver);

MODULE_DESCRIPTION("Linux driver for Qorvo DW3000");
MODULE_LICENSE("GPL v2");