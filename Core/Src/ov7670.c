/*
 * ov7670.c - OV7670 (no FIFO) driver for STM32F767ZI
 *
 * SCCB is driven with the hardware I2C peripheral (I2C2 on PF0/PF1).
 * SCCB reads do not allow a repeated START, so a read is done as
 * "write register address + STOP" followed by a separate "read 1 byte".
 *
 * The register table configures QVGA (320x240) RGB565 output.
 * Values are based on the Linux ov7670 driver defaults plus the commonly
 * used QVGA scaling settings for microcontroller projects.
 */
#include "ov7670.h"

static I2C_HandleTypeDef *ov_i2c;
uint8_t ov7670_last_reg;   /* register of the last SCCB access (diagnostics) */

typedef struct {
  uint8_t reg;
  uint8_t val;
} ov_reg_t;

#define REG_END 0xFF

/* ---- 1. General defaults (gamma, AGC/AEC, AWB, misc reserved regs) ---- */
static const ov_reg_t ov7670_default_regs[] = {
  {0x3A, 0x04},              /* TSLB */
  {0x12, 0x00},              /* COM7: VGA */
  {0x17, 0x13}, {0x18, 0x01}, {0x32, 0xB6},
  {0x19, 0x02}, {0x1A, 0x7A}, {0x03, 0x0A},
  {0x0C, 0x00}, {0x3E, 0x00},
  {0x70, 0x3A}, {0x71, 0x35}, {0x72, 0x11}, {0x73, 0xF0}, {0xA2, 0x02},
  {0x15, 0x00},              /* COM10: VSYNC/HREF/PCLK normal polarity */
  /* Gamma curve */
  {0x7A, 0x20}, {0x7B, 0x10}, {0x7C, 0x1E}, {0x7D, 0x35}, {0x7E, 0x5A},
  {0x7F, 0x69}, {0x80, 0x76}, {0x81, 0x80}, {0x82, 0x88}, {0x83, 0x8F},
  {0x84, 0x96}, {0x85, 0xA3}, {0x86, 0xAF}, {0x87, 0xC4}, {0x88, 0xD7},
  {0x89, 0xE8},
  /* AGC / AEC */
  {0x13, 0xE0},              /* COM8: AGC/AEC off while configuring */
  {0x00, 0x00}, {0x10, 0x00}, {0x0D, 0x40}, {0x14, 0x18},
  {0xA5, 0x05}, {0xAB, 0x07}, {0x24, 0x95}, {0x25, 0x33}, {0x26, 0xE3},
  {0x9F, 0x78}, {0xA0, 0x68}, {0xA1, 0x03}, {0xA6, 0xD8}, {0xA7, 0xD8},
  {0xA8, 0xF0}, {0xA9, 0x90}, {0xAA, 0x94},
  {0x13, 0xE5},              /* COM8: AGC + AEC on */
  /* Reserved / magic values */
  {0x0E, 0x61}, {0x0F, 0x4B}, {0x16, 0x02}, {0x1E, 0x07}, {0x21, 0x02},
  {0x22, 0x91}, {0x29, 0x07}, {0x33, 0x0B}, {0x35, 0x0B}, {0x37, 0x1D},
  {0x38, 0x71}, {0x39, 0x2A}, {0x3C, 0x78}, {0x4D, 0x40}, {0x4E, 0x20},
  {0x69, 0x00}, {0x6B, 0x4A}, {0x74, 0x10}, {0x8D, 0x4F}, {0x8E, 0x00},
  {0x8F, 0x00}, {0x90, 0x00}, {0x91, 0x00}, {0x96, 0x00}, {0x9A, 0x00},
  {0xB0, 0x84}, {0xB1, 0x0C}, {0xB2, 0x0E}, {0xB3, 0x82}, {0xB8, 0x0A},
  /* AWB */
  {0x43, 0x0A}, {0x44, 0xF0}, {0x45, 0x34}, {0x46, 0x58}, {0x47, 0x28},
  {0x48, 0x3A}, {0x59, 0x88}, {0x5A, 0x88}, {0x5B, 0x44}, {0x5C, 0x67},
  {0x5D, 0x49}, {0x5E, 0x0E}, {0x6C, 0x0A}, {0x6D, 0x55}, {0x6E, 0x11},
  {0x6F, 0x9F}, {0x6A, 0x40}, {0x01, 0x40}, {0x02, 0x60},
  {0x13, 0xE7},              /* COM8: AGC + AEC + AWB on */
  /* Misc */
  {0x41, 0x08}, {0x3F, 0x00}, {0x75, 0x05}, {0x76, 0xE1}, {0x4C, 0x00},
  {0x77, 0x01}, {0x4B, 0x09}, {0xC9, 0x60}, {0x41, 0x38}, {0x56, 0x40},
  {0x34, 0x11}, {0x3B, 0x12}, {0xA4, 0x88}, {0x96, 0x00}, {0x97, 0x30},
  {0x98, 0x20}, {0x99, 0x30}, {0x9A, 0x84}, {0x9B, 0x29}, {0x9C, 0x03},
  {0x9D, 0x4C}, {0x9E, 0x3F}, {0x78, 0x04},
  {0x79, 0x01}, {0xC8, 0xF0}, {0x79, 0x0F}, {0xC8, 0x00},
  {0x79, 0x10}, {0xC8, 0x7E}, {0x79, 0x0A}, {0xC8, 0x80},
  {0x79, 0x0B}, {0xC8, 0x01}, {0x79, 0x0C}, {0xC8, 0x0F},
  {0x79, 0x0D}, {0xC8, 0x20}, {0x79, 0x09}, {0xC8, 0x80},
  {0x79, 0x02}, {0xC8, 0xC0}, {0x79, 0x03}, {0xC8, 0x40},
  {0x79, 0x05}, {0xC8, 0x30}, {0x79, 0x26},
  {REG_END, REG_END}
};

/* ---- 2. RGB565 output format ---- */
static const ov_reg_t ov7670_rgb565_regs[] = {
  {0x12, 0x04},              /* COM7: RGB */
  {0x8C, 0x00},              /* RGB444 off */
  {0x04, 0x00},              /* COM1 */
  {0x40, 0xD0},              /* COM15: RGB565, full 00-FF range */
  {0x14, 0x38},              /* COM9: max AGC gain x16 */
  {0x4F, 0xB3}, {0x50, 0xB3}, {0x51, 0x00},   /* colour matrix */
  {0x52, 0x3D}, {0x53, 0xA7}, {0x54, 0xE4}, {0x58, 0x9E},
  {0x3D, 0xC0},              /* COM13: gamma + UV saturation auto */
  {REG_END, REG_END}
};

/* ---- 3. QVGA 320x240 (VGA scaled down by 2 with DCW) ---- */
static const ov_reg_t ov7670_qvga_regs[] = {
  {0x12, 0x14},              /* COM7: QVGA + RGB */
  {0x0C, 0x04},              /* COM3: DCW (downsample) enable */
  {0x3E, 0x19},              /* COM14: manual scaling, PCLK /2 */
  {0x70, 0x3A},              /* SCALING_XSC */
  {0x71, 0x35},              /* SCALING_YSC */
  {0x72, 0x11},              /* SCALING_DCWCTR: /2 H and V */
  {0x73, 0xF1},              /* SCALING_PCLK_DIV: /2 */
  {0xA2, 0x02},              /* SCALING_PCLK_DELAY */
  /* Window. If the picture is shifted or has a coloured bar on one
     side, adjust HSTART (0x17) / HSTOP (0x18) by +-1. */
  {0x17, 0x16}, {0x18, 0x04}, {0x32, 0x24},
  {0x19, 0x02}, {0x1A, 0x7A}, {0x03, 0x0A},
  /* Clock: XCLK 13.5 MHz, DBLV 0x4A (PLL x4), CLKRC 0x01. PCLK is half the
     internal clock (COM14). Slow it down with OV7670_SetPclkDiv(). */
  {0x6B, 0x4A},              /* DBLV */
  {0x11, 0x01},              /* CLKRC */
  {0x1E, 0x07},              /* MVFP: set 0x27 to mirror, 0x17 to flip */
  {REG_END, REG_END}
};

/* The OV7670 occasionally drops an ACK, so every access is retried a few times */
#define SCCB_TRIES 5
uint32_t ov7670_retry_count;   /* how many retries were needed (diagnostics) */

HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  ov7670_last_reg = reg;
  for (int i = 0; i < SCCB_TRIES; i++) {
    if (HAL_I2C_Master_Transmit(ov_i2c, OV7670_SCCB_ADDR, buf, 2, 100) == HAL_OK)
      return HAL_OK;
    ov7670_retry_count++;
    HAL_Delay(1);
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *val)
{
  /* SCCB: no repeated START -> two separate transactions */
  ov7670_last_reg = reg;
  for (int i = 0; i < SCCB_TRIES; i++) {
    if (HAL_I2C_Master_Transmit(ov_i2c, OV7670_SCCB_ADDR, &reg, 1, 100) == HAL_OK &&
        HAL_I2C_Master_Receive(ov_i2c, OV7670_SCCB_ADDR | 0x01, val, 1, 100) == HAL_OK)
      return HAL_OK;
    ov7670_retry_count++;
    HAL_Delay(1);
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef OV7670_WriteTable(const ov_reg_t *t)
{
  for (; t->reg != REG_END; t++) {
    if (OV7670_WriteReg(t->reg, t->val) != HAL_OK)
      return HAL_ERROR;
    HAL_Delay(1);
  }
  return HAL_OK;
}

HAL_StatusTypeDef OV7670_SetYUV(void)
{
  if (OV7670_WriteReg(0x12, 0x10) != HAL_OK) return HAL_ERROR;   /* COM7: QVGA + YUV */
  /* A COM7 write can disturb other settings: re-apply the QVGA
     window/scaling table, skipping its first entry (COM7 = RGB) */
  if (OV7670_WriteTable(ov7670_qvga_regs + 1) != HAL_OK) return HAL_ERROR;
  if (OV7670_WriteReg(0x40, 0xC0) != HAL_OK) return HAL_ERROR;   /* COM15: full 00-FF range, no RGB565 */
  /* byte order: COM13 bit0 = 0 and TSLB bit3 = 0 -> Y U Y V (Y first) */
  if (OV7670_WriteReg(0x3D, 0xC0) != HAL_OK) return HAL_ERROR;   /* COM13: gamma + UV auto */
  return OV7670_WriteReg(0x3A, 0x04);                             /* TSLB */
}

HAL_StatusTypeDef OV7670_SetTestPattern(uint8_t on)
{
  /* test_pattern[1:0] = SCALING_YSC bit7 : SCALING_XSC bit7, 10 = 8-bar colour bar */
  uint8_t xsc, ysc;
  if (OV7670_ReadReg(0x70, &xsc) != HAL_OK || OV7670_ReadReg(0x71, &ysc) != HAL_OK) return HAL_ERROR;
  if (OV7670_WriteReg(0x70, xsc & 0x7F) != HAL_OK) return HAL_ERROR;
  return OV7670_WriteReg(0x71, on ? (ysc | 0x80) : (ysc & 0x7F));
}

HAL_StatusTypeDef OV7670_SetPclkDiv(uint8_t div)
{
  /* CLKRC 0x01 is the normal speed; the internal clock divides by CLKRC+1 */
  if (div != 1 && div != 2 && div != 4 && div != 8) return HAL_ERROR;
  return OV7670_WriteReg(0x11, (uint8_t)(2 * div - 1));
}

HAL_StatusTypeDef OV7670_SetBanding50Hz(uint32_t frame_us)
{
  if (frame_us == 0) return HAL_ERROR;
  /* banding step = sensor rows in one half period of the mains (10 ms at 50 Hz) */
  uint32_t row_ns = frame_us * 1000u / OV7670_FRAME_ROWS;
  uint32_t bd50 = (10000000u + row_ns / 2) / row_ns;
  uint32_t bd60 = (8333333u + row_ns / 2) / row_ns;
  if (bd50 < 1) bd50 = 1;
  if (bd50 > 255) bd50 = 255;
  if (bd60 < 1) bd60 = 1;
  if (bd60 > 255) bd60 = 255;
  uint32_t max50 = OV7670_FRAME_ROWS / bd50, max60 = OV7670_FRAME_ROWS / bd60;
  if (max50 < 1) max50 = 1;
  if (max50 > 0x3F) max50 = 0x3F;
  if (max60 < 1) max60 = 1;
  if (max60 > 0x3F) max60 = 0x3F;

  uint8_t com8;
  if (OV7670_WriteReg(0x9D, (uint8_t)bd50) != HAL_OK) return HAL_ERROR;   /* BD50ST */
  if (OV7670_WriteReg(0x9E, (uint8_t)bd60) != HAL_OK) return HAL_ERROR;   /* BD60ST */
  if (OV7670_WriteReg(0xA5, (uint8_t)max50) != HAL_OK) return HAL_ERROR;  /* BD50MAX */
  if (OV7670_WriteReg(0xAB, (uint8_t)max60) != HAL_OK) return HAL_ERROR;  /* BD60MAX */
  /* COM11: 50/60 Hz auto-detect off, use BD50ST, exposure may go below
     one step in very bright light */
  if (OV7670_WriteReg(0x3B, 0x0A) != HAL_OK) return HAL_ERROR;
  if (OV7670_ReadReg(0x13, &com8) != HAL_OK) return HAL_ERROR;
  return OV7670_WriteReg(0x13, com8 | 0x20);                              /* COM8: banding filter on */
}

uint8_t OV7670_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t pid = 0, ver = 0;
  ov_i2c = hi2c;

  /* XCLK must already be running (TIM2_CH1 on PA5), otherwise SCCB does not answer */
  HAL_Delay(10);

  if (OV7670_ReadReg(0x0A, &pid) != HAL_OK) return OV7670_ERR_SCCB;
  if (OV7670_ReadReg(0x0B, &ver) != HAL_OK) return OV7670_ERR_SCCB;
  if (pid != 0x76 || ver != 0x73) return OV7670_ERR_ID;

  OV7670_WriteReg(0x12, 0x80);      /* COM7: software reset */
  HAL_Delay(30);

  if (OV7670_WriteTable(ov7670_default_regs) != HAL_OK) return OV7670_ERR_SCCB;
  if (OV7670_WriteTable(ov7670_rgb565_regs)  != HAL_OK) return OV7670_ERR_SCCB;
  if (OV7670_WriteTable(ov7670_qvga_regs)    != HAL_OK) return OV7670_ERR_SCCB;

  HAL_Delay(300);                   /* let AEC/AWB settle */
  return OV7670_OK;
}
