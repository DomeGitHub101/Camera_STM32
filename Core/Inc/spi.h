/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI5 for the ILI9341 LCD (written by hand in the CubeMX layout:
  *          this project is not regenerated with CubeMX)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi5;

/* USER CODE BEGIN Private defines */
/* SPI5 runs from APB2 (108 MHz): /2 = 54 MHz. Garbled picture? use SPI_BAUDRATEPRESCALER_4 */
#define LCD_SPI_PRESCALER   SPI_BAUDRATEPRESCALER_2
/* USER CODE END Private defines */

void MX_SPI5_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

