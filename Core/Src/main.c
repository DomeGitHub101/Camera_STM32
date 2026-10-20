/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ov7670.h"
#include <stdio.h>
#include <string.h>
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BAUD    2000000  /* ST-LINK VCP; set the PC side (tools/ov7670_viewer.py) to match.
                                 2 Mbaud = 54 MHz / 27 exactly. Garbage on the PC? use 921600 */
#define SEND_FRAMES  1        /* 1 = send every captured frame to the PC over USART3 */
#define SEND_GRAY    1        /* 1 = camera in YUV, send only Y = grayscale, 1 byte/pixel
                                 0 = RGB565 colour, 2 bytes/pixel */
#define GRAY_Y_OFFSET 0       /* which byte of each pixel pair is Y; image looks like noise? use 1 */
#define SEND_SCALE   2        /* 1 = send 320x240, 2 = send 160x120 (4x less data, ~4x faster) */
#define SEND_W       (OV7670_WIDTH / SEND_SCALE)
#define SEND_H       (OV7670_HEIGHT / SEND_SCALE)
#define SEND_BPP     (SEND_GRAY ? 1 : 2)
#define SEND_BYTES   (SEND_W * SEND_H * SEND_BPP)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__attribute__((aligned(32))) static uint8_t frame_buf[OV7670_FRAME_BYTES];
static volatile uint8_t frame_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

#define BB_SCL_PIN  GPIO_PIN_1   /* PF1 */
#define BB_SDA_PIN  GPIO_PIN_0   /* PF0 */
#define SCL_H()  HAL_GPIO_WritePin(GPIOF, BB_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()  HAL_GPIO_WritePin(GPIOF, BB_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()  HAL_GPIO_WritePin(GPIOF, BB_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()  HAL_GPIO_WritePin(GPIOF, BB_SDA_PIN, GPIO_PIN_RESET)
#define SDA_IN() (HAL_GPIO_ReadPin(GPIOF, BB_SDA_PIN) == GPIO_PIN_SET)
#define SCL_IN() (HAL_GPIO_ReadPin(GPIOF, BB_SCL_PIN) == GPIO_PIN_SET)

static void bb_delay(void)
{
  for (volatile uint32_t i = 0; i < 1000; i++) {}   /* ~5 us at 216 MHz */
}

static void bb_init(void)
{
  GPIO_InitTypeDef g = {0};
  HAL_I2C_DeInit(&hi2c2);
  __HAL_RCC_GPIOF_CLK_ENABLE();
  g.Pin = BB_SCL_PIN | BB_SDA_PIN;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  SCL_H(); SDA_H();
  HAL_GPIO_Init(GPIOF, &g);
  bb_delay();
}

/* Bus recovery: if the camera still holds SDA low (MCU was reset in the
   middle of a transfer), clock SCL until it lets go, then send a STOP */
static void bb_recover(void)
{
  for (int i = 0; i < 9 && !SDA_IN(); i++) {
    SCL_L(); bb_delay(); SCL_H(); bb_delay();
  }
  SDA_L(); bb_delay(); SCL_H(); bb_delay(); SDA_H(); bb_delay();
}

static void bb_start(void) { SDA_H(); SCL_H(); bb_delay(); SDA_L(); bb_delay(); SCL_L(); bb_delay(); }
static void bb_stop(void)  { SDA_L(); bb_delay(); SCL_H(); bb_delay(); SDA_H(); bb_delay(); }

static int bb_write(uint8_t b)
{
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) SDA_H(); else SDA_L();
    bb_delay(); SCL_H(); bb_delay(); SCL_L(); bb_delay();
  }
  SDA_H(); bb_delay();
  SCL_H(); bb_delay();
  int ack = !SDA_IN();
  SCL_L(); bb_delay();
  return ack;
}

static uint8_t bb_read(void)
{
  uint8_t b = 0;
  SDA_H();
  for (int i = 0; i < 8; i++) {
    bb_delay(); SCL_H(); bb_delay();
    b = (b << 1) | (SDA_IN() ? 1 : 0);
    SCL_L();
  }
  SDA_H(); bb_delay(); SCL_H(); bb_delay(); SCL_L(); bb_delay();
  return b;
}

static void bb_read_reg(uint8_t reg, char *msg, size_t n)
{
  bb_start();
  int a1 = bb_write(0x42);
  int a2 = bb_write(reg);
  bb_stop();
  bb_start();
  int a3 = bb_write(0x43);
  uint8_t v = bb_read();
  bb_stop();
  snprintf(msg, n, "  reg 0x%02X: ACK addrW=%d reg=%d addrR=%d  value=0x%02X\r\n",
           reg, a1, a2, a3, v);
}

/* Send one frame to the PC:
   AA 55 'F' type | width u16 LE | height u16 LE | pixels | byte sum u32 LE
   type 'R' = RGB565 (high byte first), 'G' = 8-bit gray */
#if SEND_GRAY || SEND_SCALE > 1
static uint8_t send_buf[SEND_BYTES];
#endif

static void send_frame(void)
{
  uint8_t hdr[8] = {0xAA, 0x55, 'F', SEND_GRAY ? 'G' : 'R',
                    SEND_W & 0xFF, SEND_W >> 8,
                    SEND_H & 0xFF, SEND_H >> 8};
#if SEND_GRAY || SEND_SCALE > 1
  /* keep every SEND_SCALE-th pixel of every SEND_SCALE-th line;
     in gray mode keep only the Y byte of each pixel */
  uint32_t k = 0;
  for (uint32_t y = 0; y < OV7670_HEIGHT; y += SEND_SCALE)
    for (uint32_t x = 0; x < OV7670_WIDTH; x += SEND_SCALE) {
      uint32_t i = (y * OV7670_WIDTH + x) * 2;
#if SEND_GRAY
      send_buf[k++] = frame_buf[i + GRAY_Y_OFFSET];
#else
      send_buf[k++] = frame_buf[i];
      send_buf[k++] = frame_buf[i + 1];
#endif
    }
  uint8_t *src = send_buf;
#else
  uint8_t *src = frame_buf;
#endif
  uint32_t sum = 0;
  for (uint32_t i = 0; i < SEND_BYTES; i++) sum += src[i];

  HAL_UART_Transmit(&huart3, hdr, sizeof hdr, 100);
  for (uint32_t off = 0; off < SEND_BYTES; off += 32768) {
    uint32_t n = SEND_BYTES - off;
    if (n > 32768) n = 32768;
    HAL_UART_Transmit(&huart3, src + off, n, 2000);
  }
  HAL_UART_Transmit(&huart3, (uint8_t *)&sum, 4, 100);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
  frame_ready = 1;
}

/* DCMI diagnostics: counted per second in the main loop */
static volatile uint32_t vsync_cnt = 0, line_cnt = 0, dcmi_err_cnt = 0, dcmi_err_code = 0;
static volatile uint8_t dcmi_err_pending = 0;

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *h) { vsync_cnt++; }
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *h)  { line_cnt++; }
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h)
{
  dcmi_err_cnt++;
  dcmi_err_code = h->ErrorCode;   /* 0x01 = overrun, 0x02 = sync error, 0x40 = DMA */
  dcmi_err_pending = 1;
}

/* Count level changes on an input pin for 'ms' milliseconds (works in AF mode too) */
static uint32_t count_edges(GPIO_TypeDef *port, uint16_t pin, uint32_t ms)
{
  uint32_t n = 0, t0 = HAL_GetTick();
  uint32_t last = port->IDR & pin;
  while (HAL_GetTick() - t0 < ms) {
    uint32_t now = port->IDR & pin;
    if (now != last) { n++; last = now; }
  }
  return n;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM2_Init();
  huart3.Init.BaudRate = UART_BAUD;   /* 115200 is too slow for images (13 s per frame) */
  HAL_UART_Init(&huart3);
  char msg[96];
  uart_print("\r\n=== OV7670 camera test ===\r\n");

  /* ---- Step 1: XCLK = TIM2_CH1 PWM 13.5 MHz on PA5 (D13) ---- */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  uart_print("Step 1: XCLK on PA5 started (13.5 MHz)\r\n");
  HAL_Delay(50);

  uart_print("Bit-bang SCCB test (PF1=SCL, PF0=SDA):\r\n");
  bb_init();
  snprintf(msg, sizeof msg, "  idle levels: SCL=%d SDA=%d (expect 1 1)\r\n", SCL_IN(), SDA_IN());
  uart_print(msg);
  if (!SDA_IN() || !SCL_IN()) {
    bb_recover();
    snprintf(msg, sizeof msg, "  after bus recovery: SCL=%d SDA=%d\r\n", SCL_IN(), SDA_IN());
    uart_print(msg);
  }
  bb_read_reg(0x0A, msg, sizeof msg); uart_print(msg);
  bb_read_reg(0x0B, msg, sizeof msg); uart_print(msg);
  MX_I2C2_Init();

  /* Slow I2C2 down to ~10 kHz (PRESC=11, SCLL=0xC7, SCLH=0xC3 at 54 MHz):
     the internal pull-ups (~40k) are too weak for 100 kHz.
     Remove this once external 4.7k pull-ups are fitted on SIOD/SIOC. */
  hi2c2.Init.Timing = 0xB042C3C7;
  __HAL_I2C_DISABLE(&hi2c2);
  hi2c2.Instance->TIMINGR = hi2c2.Init.Timing;
  __HAL_I2C_ENABLE(&hi2c2);

  /* ---- Step 1b: I2C2 bus scan (OV7670 should show up at 0x21) ---- */
  uint8_t found = 0;
  uart_print("Step 1b: I2C2 bus scan...\r\n");
  for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 2, 10) == HAL_OK) {
      snprintf(msg, sizeof msg, "  found device at 0x%02X (8-bit 0x%02X)\r\n", addr, addr << 1);
      uart_print(msg);
      found++;
    }
  }
  snprintf(msg, sizeof msg, "Step 1b: scan done, %u device(s) found\r\n", found);
  uart_print(msg);

  /* ---- Step 2: SCCB ---- */
  uint8_t r = OV7670_Init(&hi2c2);   /* reads PID/VER, then writes all registers */
  if (r == OV7670_ERR_SCCB) {
    uart_print("Step 2: FAIL - no answer on SCCB (check SIOC=PF1, SIOD=PF0, pull-ups, 3.3V, XCLK)\r\n");
    snprintf(msg, sizeof msg, "  failed at reg 0x%02X, HAL I2C error 0x%08lX\r\n",
             ov7670_last_reg, (unsigned long)HAL_I2C_GetError(&hi2c2));
    uart_print(msg);
    uint8_t v = 0;
    HAL_StatusTypeDef st = OV7670_ReadReg(0x0A, &v);
    snprintf(msg, sizeof msg, "  retry read 0x0A: status=%d value=0x%02X error=0x%08lX\r\n",
             st, v, (unsigned long)HAL_I2C_GetError(&hi2c2));
    uart_print(msg);
    while (1) {}
  }
  uint8_t pid = 0, ver = 0;
  OV7670_ReadReg(0x0A, &pid);
  OV7670_ReadReg(0x0B, &ver);
  snprintf(msg, sizeof msg, "Step 2: PID=0x%02X VER=0x%02X %s (SCCB retries=%lu)\r\n",
           pid, ver, (r == OV7670_OK) ? "OK" : "FAIL (expect 0x76/0x73)",
           (unsigned long)ov7670_retry_count);
  uart_print(msg);
  if (r != OV7670_OK) while (1) {}
#if SEND_GRAY
  uart_print(OV7670_SetYUV() == HAL_OK ? "Step 2b: YUV mode (grayscale) OK\r\n"
                                       : "Step 2b: FAIL - could not switch to YUV\r\n");
#endif

  /* ---- Step 3a: are the camera sync signals toggling at all? ---- */
  uint32_t e_pclk  = count_edges(GPIOA, GPIO_PIN_6, 200);
  uint32_t e_href  = count_edges(GPIOA, GPIO_PIN_4, 200);
  uint32_t e_vsync = count_edges(GPIOG, GPIO_PIN_9, 200);
  snprintf(msg, sizeof msg, "Step 3a: edges in 200 ms: PCLK=%lu HREF=%lu VSYNC=%lu\r\n",
           (unsigned long)e_pclk, (unsigned long)e_href, (unsigned long)e_vsync);
  uart_print(msg);

  /* ---- Step 3: DCMI snapshot capture ---- */
  uart_print("Step 3: DCMI started, printing frames per second...\r\n");
  uint32_t frames = 0, total = 0, t_print = HAL_GetTick();
  uint32_t t_cap = HAL_GetTick(), restarts = 0;
  frame_ready = 0;
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                     (uint32_t)frame_buf, OV7670_FRAME_BYTES / 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (frame_ready) {
	        HAL_DCMI_Stop(&hdcmi);
	        frames++;
	        total++;
	        frame_ready = 0;
#if SEND_FRAMES
	        SCB_InvalidateDCache_by_Addr((uint32_t *)frame_buf, OV7670_FRAME_BYTES);
	        send_frame();   /* capture is stopped, so the buffer is not overwritten while sending */
#endif
	        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
	                           (uint32_t)frame_buf, OV7670_FRAME_BYTES / 4);
	        t_cap = HAL_GetTick();
	      }
	      /* An error aborts the capture, and a VSYNC glitch ends a snapshot early
	         (the HAL frame callback only fires once the DMA buffer is full):
	         restart the capture in both cases */
	      if (dcmi_err_pending || HAL_GetTick() - t_cap > 500) {
	        if (!dcmi_err_pending) restarts++;
	        dcmi_err_pending = 0;
	        HAL_DCMI_Stop(&hdcmi);
	        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
	                           (uint32_t)frame_buf, OV7670_FRAME_BYTES / 4);
	        t_cap = HAL_GetTick();
	      }
	      if (HAL_GetTick() - t_print >= 1000) {
	        SCB_InvalidateDCache_by_Addr((uint32_t *)frame_buf, OV7670_FRAME_BYTES);
	        uint32_t c = ((OV7670_HEIGHT / 2) * OV7670_WIDTH + OV7670_WIDTH / 2) * 2;
#if SEND_GRAY
	        /* centre pixel brightness (Y byte) */
	        snprintf(msg, sizeof msg, "fps=%lu total=%lu centre Y=%u\r\n",
	                 (unsigned long)frames, (unsigned long)total, frame_buf[c + GRAY_Y_OFFSET]);
#else
	        /* centre pixel (RGB565, high byte first) */
	        uint16_t px = (frame_buf[c] << 8) | frame_buf[c + 1];
	        snprintf(msg, sizeof msg, "fps=%lu total=%lu centre=0x%04X R=%u G=%u B=%u\r\n",
	                 (unsigned long)frames, (unsigned long)total, px,
	                 (px >> 11) & 0x1F, (px >> 5) & 0x3F, px & 0x1F);
#endif
	        uart_print(msg);
	        snprintf(msg, sizeof msg, "  dcmi: vsync=%lu lines=%lu dma_left=%lu/%u err=%lu code=0x%02lX restarts=%lu\r\n",
	                 (unsigned long)vsync_cnt, (unsigned long)line_cnt,
	                 (unsigned long)__HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle), OV7670_FRAME_BYTES / 4,
	                 (unsigned long)dcmi_err_cnt, (unsigned long)dcmi_err_code, (unsigned long)restarts);
	        uart_print(msg);
	        vsync_cnt = 0; line_cnt = 0; dcmi_err_cnt = 0; restarts = 0;
	        frames = 0;
	        t_print = HAL_GetTick();
	      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
