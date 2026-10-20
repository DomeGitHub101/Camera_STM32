/*
 * ov7670.h - OV7670 (no FIFO) driver for STM32F767ZI using DCMI + SCCB (I2C2)
 *
 * Output format: QVGA 320x240, RGB565 (2 bytes/pixel, high byte first)
 */
#ifndef OV7670_H
#define OV7670_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define OV7670_WIDTH        320
#define OV7670_HEIGHT       240
#define OV7670_FRAME_BYTES  (OV7670_WIDTH * OV7670_HEIGHT * 2)   /* 153600 */

#define OV7670_SCCB_ADDR    0x42    /* 8-bit write address (7-bit 0x21) */

/* Return codes */
#define OV7670_OK           0
#define OV7670_ERR_SCCB     1   /* no answer on SIOC/SIOD */
#define OV7670_ERR_ID       2   /* answered, but PID/VER not 0x76/0x73 */

extern uint8_t ov7670_last_reg;   /* register of the last SCCB access */
extern uint32_t ov7670_retry_count; /* SCCB retries needed so far */

uint8_t OV7670_Init(I2C_HandleTypeDef *hi2c);
/* Call after OV7670_Init: output YUV422 (Y U Y V, 2 bytes/pixel) instead of
   RGB565. The Y bytes alone form a grayscale image. */
HAL_StatusTypeDef OV7670_SetYUV(void);
HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t val);
HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *val);

#endif /* OV7670_H */
