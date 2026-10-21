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
#include "apriltag_lite.h"
#include <stdio.h>
#include <string.h>
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- serial port (ST-LINK USB) ---- */
#define UART_BAUD          2000000  /* Tera Term and tools/ov7670_viewer.py must use this speed.
                                       2 Mbaud = 54 MHz / 27 exactly. Garbage? use 921600 */
/* ---- AprilTag (tag36h11) detection on the STM32 ---- */
#define DET_SCALE          1        /* 1 = detect on 320x240 (longer range), 2 = on 160x120 (faster) */
#define DET_W              (OV7670_WIDTH / DET_SCALE)
#define DET_H              (OV7670_HEIGHT / DET_SCALE)
#define CONFIRM_FRAMES     2        /* a tag must be seen this many frames in a row to become the command */
#define TAG_LOST_MS        1000     /* no command tag for this long -> STOP */
/* ---- result lines in Tera Term ---- */
#define OUTPUT_DELAY_MS    500      /* delay between result lines; change at run time with + and - */
#define OUTPUT_DELAY_STEP  100
#define OUTPUT_DELAY_MIN   100
#define OUTPUT_DELAY_MAX   10000
/* ---- live video for tools/ov7670_viewer.py (sent only after it asks with 'v') ---- */
#define VIDEO_SCALE        2        /* 1 = 320x240, 2 = 160x120; must be a multiple of DET_SCALE */
#define VIDEO_W            (OV7670_WIDTH / VIDEO_SCALE)
#define VIDEO_H            (OV7670_HEIGHT / VIDEO_SCALE)
/* ---- misc ---- */
#define BOOT_DIAG          0        /* 1 = print the SCCB / I2C scan / sync-signal tests at start-up */
#define GRAY_Y_OFFSET      0        /* which byte of each YUYV pixel pair is Y */

#if (VIDEO_SCALE % DET_SCALE) != 0
#error "VIDEO_SCALE must be a multiple of DET_SCALE"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__attribute__((aligned(32))) static uint8_t frame_buf[OV7670_FRAME_BYTES];   /* YUYV from the DCMI */
static volatile uint8_t frame_ready = 0;
static uint8_t gray[DET_W * DET_H];            /* Y plane = input of the tag detector */
static uint8_t video_buf[VIDEO_W * VIDEO_H];   /* frame sent to the PC viewer */
static at_config_t    at_cfg;
static at_detection_t dets[AT_MAX_DETECT];

/* Tag ID -> robot command (other IDs are reported but give no command) */
static const char *const tag_cmd[] = {
  "STOP",         /* ID 0 */
  "FORWARD",      /* ID 1 */
  "TURN_LEFT",    /* ID 2 */
  "TURN_RIGHT",   /* ID 3 */
  "BACKWARD",     /* ID 4 */
};
#define NUM_TAG_CMDS ((int)(sizeof tag_cmd / sizeof tag_cmd[0]))

static int      active_id = -1;       /* tag ID currently driving the robot, -1 = none */
static int      cand_id = -1;         /* tag waiting for CONFIRM_FRAMES */
static int      cand_count = 0;
static uint32_t active_seen = 0;      /* tick when the active tag was last seen */
static uint8_t  video_mode = 0;       /* 1 = viewer connected: also send frames + @ lines */
static uint32_t output_delay = OUTPUT_DELAY_MS;

/* statistics for the result lines */
static uint32_t stat_frames = 0, stat_det_us = 0, stat_t0 = 0;
static int            last_n = 0;     /* tags found in the latest frame */
static at_detection_t last_best;      /* biggest of them */
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

/* Non-blocking read of one byte typed in Tera Term / sent by the viewer, -1 if none */
static int uart_getc(void)
{
  USART_TypeDef *u = huart3.Instance;
  if (u->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
  if (u->ISR & USART_ISR_RXNE) return (int)(u->RDR & 0xFF);
  return -1;
}

/* ---- SCCB bit-bang: bus recovery at start-up (+ register test with BOOT_DIAG) ---- */
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

#if BOOT_DIAG
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
#endif /* BOOT_DIAG */

/* ---- camera frames ---- */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
  frame_ready = 1;
}

static volatile uint8_t dcmi_err_pending = 0;

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h)
{
  dcmi_err_pending = 1;
}

/* Copy the Y bytes of the YUYV frame into gray[] (every DET_SCALE-th pixel and line) */
static void extract_gray(void)
{
  uint8_t *dst = gray;
  for (int y = 0; y < OV7670_HEIGHT; y += DET_SCALE) {
    const uint8_t *src = frame_buf + y * OV7670_WIDTH * 2 + GRAY_Y_OFFSET;
    for (int x = 0; x < OV7670_WIDTH; x += DET_SCALE) *dst++ = src[x * 2];
  }
}

/* ---- tag -> command ---- */
static const char *cmd_text(int id)
{
  if (id < 0) return "STOP (no tag)";
  return tag_cmd[id];
}

/* id = biggest tag with a command in this frame, -1 = none */
static void update_command(int id, uint32_t now)
{
  if (id < 0) {
    cand_id = -1;
    cand_count = 0;
    return;
  }
  if (id == cand_id) cand_count++;
  else { cand_id = id; cand_count = 1; }
  if (cand_count >= CONFIRM_FRAMES) active_id = id;
  if (id == active_id) active_seen = now;
}

/* One result line for Tera Term, every output_delay ms */
static void print_status(uint32_t now)
{
  char line[160];
  uint32_t dt = now - stat_t0;
  uint32_t fps10 = dt ? stat_frames * 10000u / dt : 0;                  /* fps x10 */
  uint32_t det10 = stat_frames ? stat_det_us / stat_frames / 100u : 0;  /* ms x10 */
  int len = snprintf(line, sizeof line, "[%6lu.%02lu s] CMD: %-13s | ",
                     (unsigned long)(now / 1000), (unsigned long)(now % 1000 / 10), cmd_text(active_id));
  if (last_n > 0)
    len += snprintf(line + len, sizeof line - len, "tag ID %d at (%d,%d) %dpx rot %d err %d%s | ",
                    last_best.id, (int)last_best.cx, (int)last_best.cy, (int)(last_best.side + 0.5f),
                    last_best.rotation, last_best.hamming, last_n > 1 ? " +more" : "");
  else
    len += snprintf(line + len, sizeof line - len, "no tag | ");
  snprintf(line + len, sizeof line - len, "detect %lu.%lu ms, %lu.%lu fps\r\n",
           (unsigned long)(det10 / 10), (unsigned long)(det10 % 10),
           (unsigned long)(fps10 / 10), (unsigned long)(fps10 % 10));
  uart_print(line);
  stat_frames = 0;
  stat_det_us = 0;
  stat_t0 = now;
}

static void print_help(void)
{
  char line[100];
  uart_print("tag ID -> command: 0 STOP, 1 FORWARD, 2 TURN_LEFT, 3 TURN_RIGHT, 4 BACKWARD\r\n");
  snprintf(line, sizeof line, "output delay %lu ms  (keys: + slower, - faster, h help)\r\n",
           (unsigned long)output_delay);
  uart_print(line);
}

/* Keys from Tera Term (+ - h) and from the viewer (v = video on, t = text only) */
static void handle_key(int c)
{
  char line[60];
  switch (c) {
  case 'v':
    video_mode = 1;
    break;
  case 't':
    video_mode = 0;
    uart_print("\r\nvideo off\r\n");
    break;
  case '+':
  case '-':
    if (c == '+' && output_delay + OUTPUT_DELAY_STEP <= OUTPUT_DELAY_MAX) output_delay += OUTPUT_DELAY_STEP;
    if (c == '-' && output_delay >= OUTPUT_DELAY_MIN + OUTPUT_DELAY_STEP) output_delay -= OUTPUT_DELAY_STEP;
    snprintf(line, sizeof line, "output delay = %lu ms\r\n", (unsigned long)output_delay);
    uart_print(line);
    break;
  case 'h':
  case '?':
    print_help();
    break;
  default:
    break;
  }
}

/* ---- video for tools/ov7670_viewer.py ---- */
/* Tags found by the STM32, so the viewer can draw them (coordinates in DET_W x DET_H) */
static void send_tag_lines(int n)
{
  char line[100];
  for (int i = 0; i < n; i++) {
    const at_detection_t *d = &dets[i];
    snprintf(line, sizeof line, "@TAG %d %d %d %d %d %d %d %d %d %d %d\r\n", DET_W, DET_H, d->id,
             (int)d->corners[0][0], (int)d->corners[0][1], (int)d->corners[1][0], (int)d->corners[1][1],
             (int)d->corners[2][0], (int)d->corners[2][1], (int)d->corners[3][0], (int)d->corners[3][1]);
    uart_print(line);
  }
  snprintf(line, sizeof line, "@CMD %s\r\n", cmd_text(active_id));
  uart_print(line);
}

/* AA 55 'F' 'G' | width u16 LE | height u16 LE | grey pixels | byte sum u32 LE */
static void send_video(void)
{
  const int step = VIDEO_SCALE / DET_SCALE;
  uint32_t k = 0, sum = 0;
  for (int y = 0; y < DET_H; y += step)
    for (int x = 0; x < DET_W; x += step) {
      video_buf[k] = gray[y * DET_W + x];
      sum += video_buf[k++];
    }
  uint8_t hdr[8] = {0xAA, 0x55, 'F', 'G',
                    VIDEO_W & 0xFF, VIDEO_W >> 8, VIDEO_H & 0xFF, VIDEO_H >> 8};
  HAL_UART_Transmit(&huart3, hdr, sizeof hdr, 100);
  for (uint32_t off = 0; off < sizeof video_buf; off += 32768) {
    uint32_t n = sizeof video_buf - off;
    if (n > 32768) n = 32768;
    HAL_UART_Transmit(&huart3, video_buf + off, n, 2000);
  }
  HAL_UART_Transmit(&huart3, (uint8_t *)&sum, 4, 100);
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
  /* caches: the tag detector runs several times faster.
     frame_buf is written by DMA, so it is invalidated before every read */
  SCB_EnableICache();
  SCB_EnableDCache();
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
  huart3.Init.BaudRate = UART_BAUD;   /* 115200 is too slow for the video to the viewer */
  HAL_UART_Init(&huart3);
  char msg[96];
  uart_print("\r\n=== OV7670 AprilTag reader (tag36h11, detection on the STM32) ===\r\n");

  /* XCLK = TIM2_CH1 PWM 13.5 MHz on PA5 (D13) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_Delay(50);

  /* free the SCCB bus if the camera still holds SDA low from before a reset */
  bb_init();
  if (!SDA_IN() || !SCL_IN()) bb_recover();
#if BOOT_DIAG
  snprintf(msg, sizeof msg, "  SCCB idle levels: SCL=%d SDA=%d (expect 1 1)\r\n", SCL_IN(), SDA_IN());
  uart_print(msg);
  bb_read_reg(0x0A, msg, sizeof msg); uart_print(msg);
  bb_read_reg(0x0B, msg, sizeof msg); uart_print(msg);
#endif
  MX_I2C2_Init();

  /* SCCB at ~10 kHz (PRESC=11, SCLL=0xC7, SCLH=0xC3 at 54 MHz): it is only used
     at start-up, and the slow speed also works with weak pull-ups */
  hi2c2.Init.Timing = 0xB042C3C7;
  __HAL_I2C_DISABLE(&hi2c2);
  hi2c2.Instance->TIMINGR = hi2c2.Init.Timing;
  __HAL_I2C_ENABLE(&hi2c2);

#if BOOT_DIAG
  uart_print("  I2C2 bus scan:");
  for (uint8_t addr = 0x01; addr <= 0x7F; addr++)
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 2, 10) == HAL_OK) {
      snprintf(msg, sizeof msg, " 0x%02X", addr);
      uart_print(msg);
    }
  uart_print("\r\n");
#endif

  /* camera registers: QVGA, then YUV (the Y bytes are the grey image) */
  uint8_t r = OV7670_Init(&hi2c2);
  if (r == OV7670_ERR_SCCB) {
    uart_print("camera: FAIL - no answer on SCCB (check SIOC=PF1, SIOD=PF0, pull-ups, 3.3V, XCLK)\r\n");
    snprintf(msg, sizeof msg, "  failed at reg 0x%02X, HAL I2C error 0x%08lX\r\n",
             ov7670_last_reg, (unsigned long)HAL_I2C_GetError(&hi2c2));
    uart_print(msg);
    while (1) {}
  }
  uint8_t pid = 0, ver = 0;
  OV7670_ReadReg(0x0A, &pid);
  OV7670_ReadReg(0x0B, &ver);
  if (r != OV7670_OK || OV7670_SetYUV() != HAL_OK) {
    snprintf(msg, sizeof msg, "camera: FAIL - PID=0x%02X VER=0x%02X (expect 0x76/0x73)\r\n", pid, ver);
    uart_print(msg);
    while (1) {}
  }
  snprintf(msg, sizeof msg, "camera: OK (PID 0x%02X VER 0x%02X), grey image, detection on %dx%d\r\n",
           pid, ver, DET_W, DET_H);
  uart_print(msg);

#if BOOT_DIAG
  snprintf(msg, sizeof msg, "  edges in 200 ms: PCLK=%lu HREF=%lu VSYNC=%lu\r\n",
           (unsigned long)count_edges(GPIOA, GPIO_PIN_6, 200),
           (unsigned long)count_edges(GPIOA, GPIO_PIN_4, 200),
           (unsigned long)count_edges(GPIOG, GPIO_PIN_9, 200));
  uart_print(msg);
#endif

  at_default_config(&at_cfg);
  print_help();

  /* cycle counter for timing the detector */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  frame_ready = 0;
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)frame_buf, OV7670_FRAME_BYTES / 4);
  uint32_t t_cap = HAL_GetTick();
  uint32_t t_out = HAL_GetTick();
  stat_t0 = t_out;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int key = uart_getc();
    if (key >= 0) handle_key(key);

    if (frame_ready) {
      HAL_DCMI_Stop(&hdcmi);
      frame_ready = 0;
      SCB_InvalidateDCache_by_Addr((uint32_t *)frame_buf, OV7670_FRAME_BYTES);
      extract_gray();
      /* the next frame is captured while this one is processed */
      HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)frame_buf, OV7670_FRAME_BYTES / 4);
      t_cap = HAL_GetTick();

      uint32_t c0 = DWT->CYCCNT;
      int n = at_detect(&at_cfg, gray, DET_W, DET_H, dets, AT_MAX_DETECT);
      stat_det_us += (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000u);
      stat_frames++;

      /* biggest (= closest) tag overall, and biggest tag that has a command */
      int best = -1, best_cmd = -1;
      for (int i = 0; i < n; i++) {
        if (best < 0 || dets[i].side > dets[best].side) best = i;
        if (dets[i].id < NUM_TAG_CMDS && (best_cmd < 0 || dets[i].side > dets[best_cmd].side)) best_cmd = i;
      }
      last_n = n;
      if (best >= 0) last_best = dets[best];
      update_command(best_cmd >= 0 ? dets[best_cmd].id : -1, HAL_GetTick());

      if (video_mode) {
        send_tag_lines(n);
        send_video();
      }
    }

    /* command tag gone for too long -> STOP */
    if (active_id >= 0 && HAL_GetTick() - active_seen > TAG_LOST_MS) active_id = -1;

    /* An error aborts the capture, and a VSYNC glitch ends a snapshot early
       (the HAL frame callback only fires once the DMA buffer is full):
       restart the capture in both cases */
    if (dcmi_err_pending || HAL_GetTick() - t_cap > 500) {
      dcmi_err_pending = 0;
      HAL_DCMI_Stop(&hdcmi);
      HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)frame_buf, OV7670_FRAME_BYTES / 4);
      t_cap = HAL_GetTick();
    }

    if (HAL_GetTick() - t_out >= output_delay) {
      t_out = HAL_GetTick();
      print_status(t_out);
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
