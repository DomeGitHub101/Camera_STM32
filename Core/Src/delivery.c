/*
 * delivery.c - table delivery mission on the STM32.
 *
 * Page 1: touch "โต๊ะ N" -> target = tag ID N, the mission starts.
 * Page 2: stop and scan: the wheels stand still while the pan servo
 * (PD14, TIM4_CH3) sweeps 180 deg; nothing found -> straight ahead
 * (WHEEL_SEARCH) for MOVE_STEP_MS, then the next scan; no time limit, until the target tag is
 * seen in CONFIRM_TAG_FRAMES frames in a row, then follows it. Only CANCEL
 * on the screen ends the search. Only the
 * target tag drives the servo, so it never locks on a wrong tag. Wrong tags
 * are reported once per tag per mission.
 *
 * Driving: decide_drive() turns the target's bearing and distance into
 * left / right wheel percentages plus a readable hint (MOVE CLOSER / MOVE
 * BACK / TURN LEFT / TURN RIGHT / HOLD). In position (centred and at
 * TARGET_DIST_MM) for HOLD_TIME_MS = arrived.
 *
 * Lower board (motors + ToF) on USART2, 115200 (README section 2.7):
 *   -> $M,left,right,mode*XX   every LINK_PERIOD_MS (its motor watchdog stops
 *                              the wheels after 500 ms without one)
 *   <- $O,level,dist_mm*XX     ToF obstacle level 0..3; level >= 2 pauses the
 *                              mission and shows "กรุณาหลบหน่อยครับ"
 * Tera Term shows the same frames ("-> $M...") every output_delay.
 *
 * Going back: at the table the customer touches "รับอาหารแล้ว" (or
 * RECEIVE_TIMEOUT_MS passes); the robot then looks for the start-point tag
 * HOME_TAG_ID and drives to it with the same search / drive logic.
 */
#include "delivery.h"
#include "cat_ui.h"
#include "main.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- tag geometry: distance = CAM_FOCAL_PX * TAG_SIZE_MM / side in pixels ---- */
#define TAG_SIZE_MM        142.0f   /* edge of the BLACK square of the printed tag (measured 14.2 cm) */
#define CAM_FOCAL_PX       418.0f   /* at 320 px image width; calibrated: 30 cm read 172 mm with 100 mm / 340 px.
                                       re-calibrate: new = old * real / printed distance */
#define CONFIRM_TAG_FRAMES 3        /* same tag in this many frames in a row before it counts */
#define HOME_TAG_ID        0        /* tag at the start point (kitchen): the robot drives back to it */
/* going back: a slow full turn on the spot first, in steps: spin TURN_STEP_MS,
   stand still TURN_LOOK_MS, ... until TURN_360_MS of spinning in all */
#define TURN_360_MS        8000     /* spinning time for ~360 deg at WHEEL_SPIN (time it on the floor); a bit too long is harmless: the turn stops when the home tag shows up */
#define TURN_STEP_MS       500      /* long enough to get going past the ramp on the lower board */
#define TURN_LOOK_MS       700
#define RECEIVE_TIMEOUT_MS 10000    /* at the table: go back after this long without "รับอาหารแล้ว" (0 = wait) */
#define HOME_SHOW_MS       3000     /* "ถึงจุดเริ่มต้นแล้ว" shown this long, then the table page */
#define NOT_FOUND_SHOW_MS  2500
#define WRONG_SHOW_MS      1600     /* red eyes + text after a wrong tag */
#define LOST_MS            700      /* target out of view this long -> stop and wait (orange) */
#define WAIT_MS            1500     /* ... then sweep again */
/* ---- pan servo: PD14 = TIM4_CH3, 50 Hz, 500..2500 us = -90..+90 deg ---- */
#define SERVO_CENTER_US    1500
#define SERVO_US_PER_90    1000
#define PAN_SERVO_ENABLED  1        /* 1 = servo fitted (PD14); 0 = the camera looks straight ahead and never sweeps */
#define PAN_SERVO_DIR      -1       /* -1 if the camera turns left when it should turn right */
#define PAN_LIMIT_DEG      90
/* scan = step-and-look: a slow continuous sweep makes the servo shake (it
   moves in ticks at its own deadband), so it turns SCAN_STEP_DEG at full
   speed, stands still SCAN_LOOK_MS, and so on across 180 deg */
#define SCAN_STEP_DEG      20
#define SCAN_LOOK_MS       600
#define SERVO_MAX_DEG_PER_S 40      /* top servo speed: going back to centre or jumping onto a tag is no faster */
#define PAN_DEADBAND_DEG   2        /* tag within +- this of the image centre: servo stays still */
#define SETTLE_FRAMES      2        /* frames after a scan step that were (partly) taken while it moved */
/* following the target: each frame the servo turns TRACK_GAIN of the way to
   the tag, at most TRACK_STEP_MAX_DEG, then skips TRACK_SETTLE_FRAMES frames */
#define TRACK_GAIN         0.5f
#define TRACK_STEP_MAX_DEG 4.0f
#define TRACK_SETTLE_FRAMES 1
/* ---- drive commands: get the tag centred at TARGET_DIST_MM ---- */
#define TARGET_DIST_MM     800      /* stop this far from the tag */
#define DIST_TOL_MM        100      /* +- this is close enough */
#define CENTER_TOL_DEG     5        /* tag within +- this of straight ahead is centred */
#define ALIGN_DEG          15       /* tag further off than this: turn on the spot first */
#define HOLD_TIME_MS       1000     /* in position this long = arrived */
/* ---- wheel commands, % of full PWM (lower board: motor_set(left, right)) ---- */
#define WHEEL_SEARCH       25       /* straight ahead between two scans */
#define MOVE_STEP_MS       1500     /* drive this long after a 180 deg scan found nothing, then scan again */
#define WHEEL_FWD_MAX      40       /* forward when far away (start low, raise after floor tests) */
#define WHEEL_FWD_MIN      20       /* forward just outside the distance tolerance */
#define SLOW_DOWN_MM       600      /* forward ramps from MIN to MAX over this much extra distance */
#define WHEEL_REVERSE      25       /* backing off when too close */
#define WHEEL_SPIN         50       /* turning on the spot: one side +, the other - (50 = the lower board's speed cap) */
#define WHEEL_PCT_PER_DEG  1        /* steering while driving: wheel difference per degree off centre */
#define WHEEL_STEER_MAX    15
#define DOCK_ZONE_MM       300      /* closer than TARGET_DIST_MM + this: mode 2 (docking, ToF stays quiet) */
/* ---- link to the lower board (USART2) ---- */
#define LINK_PERIOD_MS     100      /* $M rate (TIM6 interrupt); the lower board stops after 500 ms without one */
#define LINK_STALE_MS      400      /* main loop silent this long: the interrupt sends stop */
#define LINK_RX_TIMEOUT_MS 1000     /* no $O this long (after the first one) = lower board lost */
#define LINK_DEBUG         0        /* 1 = print a LINK TX/RX line every output_delay (counts + last HAL status) */
#define LINK_MODE_STOP     0
#define LINK_MODE_DRIVE    1
#define LINK_MODE_DOCKING  2
#define TOUCH_DEBUG        0        /* 1 = print every touch (for checking the touch mapping) */

#define NUM_TAG_IDS        587      /* tag36h11 */
/* ms from t to now; 0 if t is (slightly) in the future */
#define ELAPSED(now, t)    ((int32_t)((now) - (t)) > 0 ? (int32_t)((now) - (t)) : 0)

typedef enum { M_SELECT, M_SEARCH, M_WAIT, M_FOUND, M_ARRIVED, M_NOT_FOUND, M_TURN } mission_t;

static mission_t mode = M_SELECT;
static int target = 0, selected = 0;
static float pan = 0;                 /* deg, + = camera looks right */
static int sweep_dir = 1;
typedef enum { S_TO_EDGE, S_SCAN, S_MOVE } search_phase_t;
static search_phase_t phase = S_TO_EDGE;   /* M_SEARCH: wheels stopped while S_TO_EDGE / S_SCAN */
static uint32_t t_phase;
static uint8_t search_drive = 0;      /* 1 = M_SEARCH wants the wheels straight ahead now */
static uint8_t turn_spinning = 0, turn_spin = 0;   /* M_TURN: in a spin step / wheels turning now */
static uint32_t turn_spun_ms = 0;     /* M_TURN: spinning time so far */
static uint32_t t_poll, t_seen, t_state, t_wrong_until, t_beat;
static uint8_t beat_now = 0;
static int wrong_id = -1;
static uint8_t reported[(NUM_TAG_IDS + 7) / 8];
static float bearing = 0, dist_mm = 0;
static uint32_t t_in_pos = 0;         /* since when the robot has been in position, 0 = not */
static int settle = 0;                /* frames still to skip after the servo moved */
static uint8_t returning = 0;         /* 1 = on the way back to HOME_TAG_ID */

typedef struct {
  int left, right;                    /* wheel %, -100..100, + = forward */
  int link_mode;                      /* LINK_MODE_* */
  char text[64];                      /* readable hint */
} drive_t;

/* lower board link */
static uint8_t link_now = 0;          /* send $M at once (state changed) */

static char last_frame[40] = "";      /* last $M sent, echoed to Tera Term */
static volatile uint8_t rx_ring[128];
static volatile uint16_t rx_head = 0;
static uint16_t rx_tail = 0;
static uint8_t rx_byte;
static char rx_line[48];
static int rx_len = 0;
static uint8_t obst_seen = 0;         /* a $O has arrived at least once */
static uint8_t link_lost = 0;
static int obst_level = 0, obst_dist = 0;
static uint32_t t_obst_rx = 0;
static int prev_id[AT_MAX_DETECT], prev_streak[AT_MAX_DETECT], prev_n = 0;

static void print(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

/* ---- link to the lower board (USART2) ---- */
static uint8_t xor_sum(const char *p, const char *end)
{
  uint8_t sum = 0;
  for (; p < end && *p; p++) sum ^= (uint8_t)*p;          /* everything between $ and * */
  return sum;
}

static int clamp_pct(int v)
{
  return v > 100 ? 100 : (v < -100 ? -100 : v);
}

/* The $M frames go out from the TIM6 interrupt every LINK_PERIOD_MS, from
   boot on, whatever the main loop is doing (camera set-up, LCD redraws).
   The main loop only sets the newest command. If it stops doing so for
   LINK_STALE_MS (stuck somewhere), the interrupt sends "stop" instead, so a
   hung main loop never keeps the wheels turning. */
static char frame_cur[44] = "$M,0,0,0*51\r\n";   /* newest frame, set by link_send() */
static char frame_tx[44];                         /* the copy being sent */
static volatile uint32_t t_cmd = 0;               /* when the main loop last set a command */

void link_send(int left, int right, int mode)
{
  char body[32], line[44];
  snprintf(body, sizeof body, "M,%d,%d,%d", clamp_pct(left), clamp_pct(right), mode);
  snprintf(last_frame, sizeof last_frame, "$%s*%02X", body, xor_sum(body, body + strlen(body)));
  snprintf(line, sizeof line, "%s\r\n", last_frame);
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  strcpy(frame_cur, line);
  t_cmd = HAL_GetTick();
  __set_PRIMASK(primask);
}

/* link diagnostics: what the TIM6 sender did, what came back */
static volatile uint32_t dbg_ticks, dbg_ok, dbg_busy, dbg_err, dbg_done, dbg_rx_bytes, dbg_rx_ok, dbg_rx_bad;
static volatile HAL_StatusTypeDef dbg_last = HAL_OK;
static volatile uint32_t dbg_gstate;

void TIM6_DAC_IRQHandler(void)
{
  if (!(TIM6->SR & TIM_SR_UIF)) return;
  TIM6->SR = ~TIM_SR_UIF;
  dbg_ticks++;
  dbg_gstate = huart2.gState;
  if (huart2.gState != HAL_UART_STATE_READY) { dbg_busy++; return; }   /* last frame still going out */
  const char *f = HAL_GetTick() - t_cmd > LINK_STALE_MS ? "$M,0,0,0*51\r\n" : frame_cur;
  strcpy(frame_tx, f);
  dbg_last = HAL_UART_Transmit_IT(&huart2, (uint8_t *)frame_tx, strlen(frame_tx));
  if (dbg_last == HAL_OK) dbg_ok++;
  else dbg_err++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
  if (h->Instance == USART2) dbg_done++;                          /* the whole frame left the TX pin */
}

#if LINK_DEBUG
static void link_debug_print(void)
{
  static const char *const st[] = {"HAL_OK", "HAL_ERROR", "HAL_BUSY", "HAL_TIMEOUT"};
  char line[200];
  snprintf(line, sizeof line,
           "LINK TX: tim6 %lu, Transmit_IT %s x%lu, busy %lu, fail %lu, sent %lu, gState 0x%02lX, BRR %lu"
           " | RX: %lu bytes, $O ok %lu bad %lu | last %s\r\n",
           (unsigned long)dbg_ticks, st[dbg_last & 3], (unsigned long)dbg_ok, (unsigned long)dbg_busy,
           (unsigned long)dbg_err, (unsigned long)dbg_done, (unsigned long)dbg_gstate,
           (unsigned long)USART2->BRR, (unsigned long)dbg_rx_bytes, (unsigned long)dbg_rx_ok,
           (unsigned long)dbg_rx_bad, last_frame);
  print(line);
}
#endif

/* $M,0,0,0 every LINK_PERIOD_MS from now on (TIM6, 10 kHz count) */
void link_start(void)
{
  link_send(0, 0, LINK_MODE_STOP);
  __HAL_RCC_TIM6_CLK_ENABLE();
  uint32_t clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) clk *= 2;   /* timer clock = 2 x PCLK1 */
  TIM6->CR1 = 0;
  TIM6->PSC = clk / 10000u - 1;
  TIM6->ARR = LINK_PERIOD_MS * 10u - 1;
  TIM6->EGR = TIM_EGR_UG;
  TIM6->SR = 0;
  TIM6->DIER = TIM_DIER_UIE;
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 6, 0);       /* below the DCMI / USART2 interrupts */
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  TIM6->CR1 = TIM_CR1_CEN;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
  if (h->Instance != USART2) return;
  dbg_rx_bytes++;
  rx_ring[rx_head] = rx_byte;
  rx_head = (uint16_t)((rx_head + 1) % sizeof rx_ring);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
  if (h->Instance == USART2) HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* overrun / noise: listen again */
}

/* one complete line from the lower board */
static void link_line(char *s, uint32_t now)
{
  char *star = strchr(s, '*');
  if (s[0] != '$' || !star) { dbg_rx_bad++; return; }
  if (xor_sum(s + 1, star) != (uint8_t)strtol(star + 1, NULL, 16)) { dbg_rx_bad++; return; }   /* bad checksum: drop */
  int level, dist;
  if (sscanf(s + 1, "O,%d,%d", &level, &dist) != 2) { dbg_rx_bad++; return; }
  dbg_rx_ok++;
  char line[64];
  t_obst_rx = now;
  obst_seen = 1;
  if (link_lost) {
    link_lost = 0;
    print("LINK OK (lower board answers again)\r\n");
  }
  if (level != obst_level) {
    if (level > 0) snprintf(line, sizeof line, "OBSTACLE level %d (~%d mm)\r\n", level, dist);
    else snprintf(line, sizeof line, "CLEAR\r\n");
    print(line);
    link_now = 1;
  }
  obst_level = level;
  obst_dist = dist;
}

static void link_rx_poll(uint32_t now)
{
  while (rx_tail != rx_head) {
    char c = (char)rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % sizeof rx_ring);
    if (c == '\r') continue;
    if (c == '\n' || rx_len >= (int)sizeof rx_line - 1) {
      rx_line[rx_len] = 0;
      link_line(rx_line, now);
      rx_len = 0;
    } else {
      rx_line[rx_len++] = c;
    }
  }
  /* the lower board has talked before and went quiet: stop and say so */
  if (obst_seen && !link_lost && ELAPSED(now, t_obst_rx) > LINK_RX_TIMEOUT_MS) {
    link_lost = 1;
    obst_level = 0;
    print("LINK LOST: no $O from the lower board\r\n");
    link_now = 1;
  }
}

/* ---- pan servo ---- */
/* deg: where it is going, where it is, how fast (deg/s). servo_now is moved by
   the TIM4 interrupt every 20 ms (one PWM period), so the servo turns in even
   small steps however long a camera frame takes in the main loop. */
static volatile float servo_target = 0, servo_now = 0;

static void servo_out(float deg)
{
  TIM4->CCR3 = (uint32_t)(SERVO_CENTER_US + PAN_SERVO_DIR * deg * SERVO_US_PER_90 / 90.0f + 0.5f);
}

/* only sets the target: the interrupt turns towards it at SERVO_MAX_DEG_PER_S */
static void servo_write(float deg)
{
  if (deg > PAN_LIMIT_DEG) deg = PAN_LIMIT_DEG;
  if (deg < -PAN_LIMIT_DEG) deg = -PAN_LIMIT_DEG;
  servo_target = deg;
}

/* every 20 ms, just before the next PWM pulse */
void TIM4_IRQHandler(void)
{
  TIM4->SR = ~TIM_SR_UIF;
  float step = SERVO_MAX_DEG_PER_S * 0.020f;
  float d = servo_target - servo_now;
  if (d > step) d = step;
  if (d < -step) d = -step;
  servo_now += d;
  servo_out(servo_now);
}

static int servo_moving(void)
{
  return fabsf(servo_target - servo_now) > 0.5f;
}

static void servo_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  g.Pin = GPIO_PIN_14;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &g);

  /* timer clock = 2 x PCLK1 when APB1 is divided: 1 tick = 1 us, 20 ms period */
  uint32_t clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) clk *= 2;
  TIM4->CR1 = 0;
  TIM4->PSC = clk / 1000000u - 1;
  TIM4->ARR = 20000 - 1;
  TIM4->CCMR2 = (TIM4->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S)) |
                TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;   /* PWM mode 1 */
  TIM4->CCER |= TIM_CCER_CC3E;
  servo_out(0);
  TIM4->CR1 = TIM_CR1_ARPE;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR = 0;
  TIM4->DIER |= TIM_DIER_UIE;
  HAL_NVIC_SetPriority(TIM4_IRQn, 7, 0);           /* lowest here: a late step is harmless */
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  TIM4->CR1 |= TIM_CR1_CEN;
}

/* Only the target tag comes here.
   bearing = servo angle + the tag's angle in the image, which is only right
   when the image was taken at that servo angle. So the servo moves in small
   steps that keep the tag in the middle of the picture, and the frames taken
   while it moves are not used for the bearing. */
static void pan_update(const at_detection_t *d, int img_w, int img_h, int rotation)
{
  float f = CAM_FOCAL_PX * img_w / 320.0f;
  /* pixels right of centre in the upright picture (turned 'rotation' deg clockwise) */
  float off;
  switch (rotation) {
  case 90:  off = img_h / 2.0f - d->cy; break;
  case 180: off = img_w / 2.0f - d->cx; break;
  case 270: off = d->cy - img_h / 2.0f; break;
  default:  off = d->cx - img_w / 2.0f; break;
  }
  float err = atanf(off / f) * 57.2958f;                      /* deg, + = right of centre */
  dist_mm = d->side > 0 ? f * TAG_SIZE_MM / d->side : 99999.0f;
  if (servo_moving()) return;         /* still turning to 'pan': the angle is not known yet */
  if (settle > 0) {                   /* image taken while the servo was moving */
    settle--;
    return;
  }
  bearing = pan + err;
  /* keep the tag in the middle of the picture: a small step towards it every frame */
  if (PAN_SERVO_ENABLED && fabsf(err) > PAN_DEADBAND_DEG) {
    float step = err * TRACK_GAIN;
    if (step > TRACK_STEP_MAX_DEG) step = TRACK_STEP_MAX_DEG;
    if (step < -TRACK_STEP_MAX_DEG) step = -TRACK_STEP_MAX_DEG;
    pan += step;
    if (pan > PAN_LIMIT_DEG) pan = PAN_LIMIT_DEG;
    if (pan < -PAN_LIMIT_DEG) pan = -PAN_LIMIT_DEG;
    servo_write(pan);
    settle = TRACK_SETTLE_FRAMES;
  }
}

/* ---- mission ---- */
static void print_found(void)
{
  char line[80];
  snprintf(line, sizeof line, "FOUND tag %d bearing %+d deg ~%d mm\r\n",
           target, (int)lroundf(bearing), (int)lroundf(dist_mm));
  print(line);
}

static int in_position(void)
{
  return fabsf(dist_mm - TARGET_DIST_MM) <= DIST_TOL_MM;   /* distance only: the angle need not be centred */
}

/* What the wheels should do now: straight ahead while searching, steered
   onto the tag once it is in view (M_FOUND), stopped otherwise. */
static drive_t decide_drive(void)
{
  drive_t d = {0, 0, LINK_MODE_STOP, ""};
  if (link_lost) {
    snprintf(d.text, sizeof d.text, "STOP (lower board not answering)");
    return d;
  }
  if (mode == M_TURN) {
    if (obst_level >= 2) {
      snprintf(d.text, sizeof d.text, "WAIT (obstacle ~%d mm)", obst_dist);
      return d;
    }
    if (!turn_spin) {
      snprintf(d.text, sizeof d.text, "TURN 360: look (%lu/%d ms spun)", (unsigned long)turn_spun_ms, TURN_360_MS);
      return d;
    }
    d.link_mode = LINK_MODE_DRIVE;
    d.left = WHEEL_SPIN;                                 /* spin right on the spot */
    d.right = -WHEEL_SPIN;
    snprintf(d.text, sizeof d.text, "TURN 360: spin %d%%", WHEEL_SPIN);
    return d;
  }
  if (mode == M_SEARCH) {
    if (obst_level >= 2) {
      snprintf(d.text, sizeof d.text, "WAIT (obstacle ~%d mm)", obst_dist);
      return d;
    }
    if (!search_drive) {
      snprintf(d.text, sizeof d.text, "STOP (scanning, pan %+d)", (int)lroundf(pan));
      return d;
    }
    d.link_mode = LINK_MODE_DRIVE;                       /* ToF guards the way ahead */
    d.left = d.right = WHEEL_SEARCH;
    snprintf(d.text, sizeof d.text, "FORWARD %d%% (searching)", WHEEL_SEARCH);
    return d;
  }
  if (mode != M_FOUND) {
    snprintf(d.text, sizeof d.text, "STOP (no tag)");
    return d;
  }
  int off = (int)lroundf(bearing);                       /* + = tag is to the right */
  int err = (int)lroundf(dist_mm) - TARGET_DIST_MM;      /* + = too far */
  int len = 0, steer = 0;
  d.link_mode = err <= DOCK_ZONE_MM ? LINK_MODE_DOCKING : LINK_MODE_DRIVE;
  if (obst_level >= 2) {                                 /* the lower board blocks forward anyway */
    snprintf(d.text, sizeof d.text, "WAIT (obstacle ~%d mm)", obst_dist);
    return d;
  }

  if (abs(err) <= DIST_TOL_MM) {                         /* at the distance: stop, whatever the angle */
    snprintf(d.text, sizeof d.text, "HOLD (at %d mm)", (int)lroundf(dist_mm));
    return d;
  }
  if (abs(off) > CENTER_TOL_DEG) {
    steer = off * WHEEL_PCT_PER_DEG;
    if (steer > WHEEL_STEER_MAX) steer = WHEEL_STEER_MAX;
    if (steer < -WHEEL_STEER_MAX) steer = -WHEEL_STEER_MAX;
    len += snprintf(d.text + len, sizeof d.text - len, "TURN %s %d deg", off > 0 ? "RIGHT" : "LEFT", abs(off));
  }
  if (abs(off) > ALIGN_DEG) {                            /* far off to the side: turn on the spot */
    d.left = off > 0 ? WHEEL_SPIN : -WHEEL_SPIN;
    d.right = -d.left;
    snprintf(d.text + len, sizeof d.text - len, " first");
    return d;
  }
  int v = 0;
  if (err > DIST_TOL_MM) {
    float k = (float)(err - DIST_TOL_MM) / SLOW_DOWN_MM;
    if (k > 1) k = 1;
    v = WHEEL_FWD_MIN + (int)lroundf(k * (WHEEL_FWD_MAX - WHEEL_FWD_MIN));
    snprintf(d.text + len, sizeof d.text - len, "%sMOVE CLOSER %d mm", len ? ", " : "", err);
  } else if (err < -DIST_TOL_MM) {
    v = -WHEEL_REVERSE;
    snprintf(d.text + len, sizeof d.text - len, "%sMOVE BACK %d mm", len ? ", " : "", -err);
  } else if (!len) {
    snprintf(d.text, sizeof d.text, "HOLD (in position)");
  }
  d.left = clamp_pct(v + steer);                         /* turning right = left wheels faster */
  d.right = clamp_pct(v - steer);
  return d;
}

/* Newest command for the TIM6 sender, every main-loop pass; after a state
   change the frame goes out at once instead of at the next 100 ms tick */
static void link_tick(uint32_t now)
{
  (void)now;
  drive_t d = decide_drive();
  link_send(d.left, d.right, d.link_mode);
  if (link_now) {
    link_now = 0;
    TIM6->EGR = TIM_EGR_UG;           /* update event: the interrupt sends now and restarts the 100 ms */
  }
}

static void back_to_select(void)
{
  mode = M_SELECT;
  pan = 0;
  servo_write(0);
  link_send(0, 0, LINK_MODE_STOP);
  ui_show_select(selected);
}

/* what the robot is heading for, for the result lines */
static const char *goal(void)
{
  static char buf[24];
  if (returning) snprintf(buf, sizeof buf, "home (tag %d)", HOME_TAG_ID);
  else snprintf(buf, sizeof buf, "table %d", target);
  return buf;
}

/* wheels stop, the servo goes to the start edge, then sweeps across 180 deg */
static void start_scan(void)
{
  phase = S_TO_EDGE;
  search_drive = 0;
  pan = -sweep_dir * PAN_LIMIT_DEG;
  servo_write(pan);
  link_now = 1;
}

static void reset_search(void)
{
  memset(reported, 0, sizeof reported);
  prev_n = 0;
  sweep_dir = 1;
  start_scan();
  wrong_id = -1;
  t_wrong_until = 0;
  t_seen = 0;
  t_in_pos = 0;
  mode = M_SEARCH;
  t_state = HAL_GetTick();
  beat_now = 1;
  link_now = 1;
}

/* food taken (or timeout): drive back to the start-point tag */
static void start_return(void)
{
  char line[48];
  returning = 1;
  target = HOME_TAG_ID;
  snprintf(line, sizeof line, "RETURN to home (tag %d)\r\n", HOME_TAG_ID);
  print(line);
  ui_show_mission(HOME_TAG_ID);
  reset_search();
  /* the start point is behind: a slow full turn first (stops when tag HOME_TAG_ID shows up) */
  mode = M_TURN;
  pan = 0;
  servo_write(0);
  turn_spinning = 0;
  turn_spin = 0;
  turn_spun_ms = 0;
  t_phase = HAL_GetTick();
  print("TURN 360\r\n");
}

static void start_mission(int table)
{
  char line[40];
  selected = target = table;
  snprintf(line, sizeof line, "TABLE %d selected\r\n", table);
  print(line);
  ui_show_select(table);              /* the touched button turns orange */
  HAL_Delay(150);
  ui_show_mission(table);
  returning = 0;
  reset_search();
}

static void enter(mission_t m, uint32_t now)
{
  mode = m;
  t_state = now;
  beat_now = 1;
  link_now = 1;
}

void delivery_frame(const at_detection_t *dets, int n, int img_w, int img_h, int rotation, uint32_t now)
{
  if (mode == M_SELECT) return;

  /* frames in a row for each tag in view */
  int streak[AT_MAX_DETECT];
  for (int i = 0; i < n; i++) {
    streak[i] = 1;
    for (int j = 0; j < prev_n; j++)
      if (prev_id[j] == dets[i].id) streak[i] = prev_streak[j] + 1;
  }
  for (int i = 0; i < n; i++) { prev_id[i] = dets[i].id; prev_streak[i] = streak[i]; }
  prev_n = n;

  if (mode == M_ARRIVED || mode == M_NOT_FOUND) return;

  /* the target (the biggest one if it is seen twice) */
  int ti = -1;
  for (int i = 0; i < n; i++)
    if (dets[i].id == target && (ti < 0 || dets[i].side > dets[ti].side)) ti = i;
  if (ti >= 0) {
    pan_update(&dets[ti], img_w, img_h, rotation);
    t_seen = now;
    if (streak[ti] >= CONFIRM_TAG_FRAMES) {
      if (mode != M_FOUND) {
        enter(M_FOUND, now);
        t_wrong_until = 0;
        print_found();
      }
      /* arrived = centred at the target distance for HOLD_TIME_MS */
      if (!in_position()) {
        t_in_pos = 0;
      } else if (!t_in_pos) {
        t_in_pos = now;
      } else if (ELAPSED(now, t_in_pos) >= HOLD_TIME_MS) {
        char line[64];
        snprintf(line, sizeof line, "ARRIVED %s (~%d mm, %+d deg)\r\n",
                 goal(), (int)lroundf(dist_mm), (int)lroundf(bearing));
        print(line);
        enter(M_ARRIVED, now);
        link_send(0, 0, LINK_MODE_STOP);
        if (!returning) ui_show_received(1);   /* the customer confirms with this button */
        return;
      }
    }
  }

  /* wrong tags: once per tag per mission, not while following the target,
     and not on the way back (the tables are expected to be seen then) */
  if (mode == M_FOUND || returning) return;
  for (int i = 0; i < n; i++) {
    int id = dets[i].id;
    if (id == target || streak[i] < CONFIRM_TAG_FRAMES || id < 0 || id >= NUM_TAG_IDS) continue;
    if (reported[id / 8] & (1 << (id % 8))) continue;
    reported[id / 8] |= 1 << (id % 8);
    char line[48];
    snprintf(line, sizeof line, "INVALID tag %d (target %d)\r\n", id, target);
    print(line);
    wrong_id = id;
    t_wrong_until = now + WRONG_SHOW_MS;
  }
}

void delivery_poll(uint32_t now, uint32_t output_delay)
{
  uint32_t dt = now - t_poll;
  t_poll = now;
  if (dt > 200) dt = 200;
  int lcd = ui_lcd_check(now);
  if (lcd > 0) print("LCD had reset itself (white): set up and redrawn\r\n");
  if (lcd < 0) print("LCD check off: no reply on MISO (PF8)\r\n");
  link_rx_poll(now);
#if LINK_DEBUG
  static uint32_t t_dbg = 0;
  if (now - t_dbg >= output_delay) {
    t_dbg = now;
    link_debug_print();
  }
#endif

  /* touch */
  char line[64];
  int x, y;
  if (ui_poll_touch(&x, &y)) {
#if TOUCH_DEBUG
    extern uint16_t ui_touch_raw[2];
    snprintf(line, sizeof line, "TOUCH x=%d y=%d (adc %u,%u)\r\n", x, y, ui_touch_raw[0], ui_touch_raw[1]);
    print(line);
#endif
    if (mode == M_SELECT) {
      int t = ui_hit_table(x, y);
      if (t) start_mission(t);
    } else if (mode == M_ARRIVED && !returning && ui_hit_received(x, y)) {
      snprintf(line, sizeof line, "RECEIVED table %d\r\n", target);
      print(line);
      start_return();
    } else if (ui_hit_cancel(x, y)) {
      print("CANCEL\r\n");
      back_to_select();
    }
  }
  if (mode == M_SELECT) {
    link_tick(now);                   /* $M,0,0,0 keeps the lower board's watchdog fed */
    return;
  }
  /* a touch may have started a mission (redraw + delay): read the clock again */
  now = HAL_GetTick();
  t_poll = now;

  /* someone in the way (lower board's ToF): the mission's clocks stand still */
  int blocked = obst_level >= 2 && !link_lost;
  if (blocked) {
    t_state += dt;
    if (t_seen) t_seen += dt;
    t_in_pos = 0;
  }

  if (blocked) t_phase += dt;

  switch (mode) {
  case M_SEARCH:
    /* stop - scan 180 deg - move MOVE_STEP_MS - scan ... with no time limit,
       until the tag is found or CANCEL is touched */
    search_drive = 0;
    /* target seen but not confirmed yet: pan_update() holds the camera on it */
    if (t_seen && ELAPSED(now, t_seen) < LOST_MS) break;
    if (blocked) break;
    if (!PAN_SERVO_ENABLED) {         /* no servo: just straight ahead, camera fixed */
      search_drive = 1;
      break;
    }
    if (phase == S_MOVE) {
      search_drive = 1;
      if (ELAPSED(now, t_phase) >= MOVE_STEP_MS) start_scan();
    } else if (phase == S_TO_EDGE) {
      if (!servo_moving()) {                 /* at the start edge: look, then step from here */
        phase = S_SCAN;
        t_phase = now;
        settle = SETTLE_FRAMES;
      }
    } else {
      /* step-and-look: t_phase restarts while the servo is still turning */
      if (servo_moving()) {
        t_phase = now;
        break;
      }
      if (ELAPSED(now, t_phase) < SCAN_LOOK_MS) break;
      /* looked long enough at the other edge (the one it is heading for, not
         the start edge): 180 deg seen, nothing there */
      if ((sweep_dir > 0 && pan >= PAN_LIMIT_DEG) || (sweep_dir < 0 && pan <= -PAN_LIMIT_DEG)) {
        sweep_dir = -sweep_dir;              /* next scan starts from this edge */
        phase = S_MOVE;
        t_phase = now;
        pan = 0;                             /* look ahead while driving */
        servo_write(pan);
        print("SCAN done, not found: MOVE\r\n");
        break;
      }
      pan += sweep_dir * SCAN_STEP_DEG;
      if (pan > PAN_LIMIT_DEG) pan = PAN_LIMIT_DEG;
      if (pan < -PAN_LIMIT_DEG) pan = -PAN_LIMIT_DEG;
      servo_write(pan);
      t_phase = now;
      settle = SETTLE_FRAMES;         /* frames taken during the step give no bearing */
    }
    break;
  case M_TURN:
    /* slow full turn: spin TURN_STEP_MS, stand still TURN_LOOK_MS (sharp
       frames), ... until TURN_360_MS of spinning; tag HOME_TAG_ID seen -> hold,
       confirmed -> M_FOUND (delivery_frame) */
    turn_spin = 0;
    if (blocked) break;               /* t_phase stands still meanwhile */
    if (t_seen && ELAPSED(now, t_seen) < LOST_MS) break;
    if (turn_spinning) {
      if (ELAPSED(now, t_phase) >= TURN_STEP_MS) {
        turn_spun_ms += ELAPSED(now, t_phase);
        turn_spinning = 0;
        t_phase = now;
      } else {
        turn_spin = 1;
      }
    } else if (ELAPSED(now, t_phase) >= TURN_LOOK_MS) {
      if (turn_spun_ms >= TURN_360_MS) {
        print("TURN 360 done, not found: SEARCH home\r\n");
        enter(M_SEARCH, now);
        start_scan();
      } else {
        turn_spinning = 1;
        turn_spin = 1;
        t_phase = now;
      }
    }
    break;
  case M_FOUND:
    if (ELAPSED(now, t_seen) > LOST_MS) { t_in_pos = 0; enter(M_WAIT, now); }
    break;
  case M_WAIT:
    if (ELAPSED(now, t_state) > WAIT_MS) {
      enter(M_SEARCH, now);
      start_scan();                   /* lost it: look around before driving on */
    }
    break;
  case M_ARRIVED:
    if (returning) {
      if (ELAPSED(now, t_state) > HOME_SHOW_MS) { back_to_select(); return; }
    } else if (RECEIVE_TIMEOUT_MS && ELAPSED(now, t_state) > RECEIVE_TIMEOUT_MS) {
      snprintf(line, sizeof line, "RECEIVE TIMEOUT table %d\r\n", target);
      print(line);
      start_return();
      return;
    }
    break;
  case M_NOT_FOUND:
    if (ELAPSED(now, t_state) > NOT_FOUND_SHOW_MS) { back_to_select(); return; }
    break;
  default:
    break;
  }

  /* face */
  int wrong = (int32_t)(t_wrong_until - now) > 0 && (mode == M_SEARCH || mode == M_WAIT);
  /* pupils look where the camera looks; without the servo, at the tag */
  float look = PAN_SERVO_ENABLED ? pan : (mode == M_FOUND ? bearing * 3 : 0);
  int dx = (int)lroundf(look * 16.0f / PAN_LIMIT_DEG);
  if (mode == M_ARRIVED) {
    ui_set_eyes(EYES_HAPPY, 0);
    ui_set_status(returning ? STATUS_HOME : STATUS_ARRIVED, target);
  } else if (link_lost) {
    ui_set_eyes(EYES_WAIT, dx);
    ui_set_status(STATUS_LINK_LOST, 0);
  } else if (blocked) {
    ui_set_eyes(EYES_WAIT, dx);
    ui_set_status(STATUS_OBSTACLE, 0);
  } else if (returning) {
    ui_set_eyes(mode == M_FOUND ? EYES_FOUND : (mode == M_WAIT ? EYES_WAIT : EYES_SEARCH), dx);
    ui_set_status(STATUS_GOING_HOME, 0);
  } else if (mode == M_NOT_FOUND) {
    ui_set_eyes(EYES_WAIT, dx);
    ui_set_status(STATUS_NOT_FOUND, target);
  } else if (wrong) {
    ui_set_eyes(EYES_WRONG, dx);
    ui_set_status(STATUS_WRONG, wrong_id);
  } else if (mode == M_FOUND) {
    ui_set_eyes(EYES_FOUND, dx);
    ui_set_status(STATUS_FOUND, target);
  } else {
    ui_set_eyes(mode == M_WAIT ? EYES_WAIT : EYES_SEARCH, dx);
    ui_set_status(STATUS_SEARCHING, target);
  }

  link_tick(now);

  /* result lines for Tera Term: at every change and every output_delay */
  if (beat_now || now - t_beat >= output_delay) {
    beat_now = 0;
    t_beat = now;
    drive_t d = decide_drive();
    char line2[80];
    if (mode == M_FOUND) print_found();
    if (mode != M_ARRIVED && mode != M_NOT_FOUND) {
      snprintf(line2, sizeof line2, "DRIVE %s\r\n", d.text);
      print(line2);
    }
    snprintf(line2, sizeof line2, "-> %s\r\n", last_frame);   /* what the lower board gets */
    print(line2);
  }
}

void delivery_init(void)
{
  servo_init();
  ui_init();
  ui_show_select(0);
  t_poll = HAL_GetTick();
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* $O lines from the lower board */
  link_send(0, 0, LINK_MODE_STOP);
  print("touch a table on the screen (TABLE / FOUND / DRIVE / INVALID / ARRIVED / RECEIVED / RETURN lines and -> $M wheel commands follow)\r\n");
}

/* 's' key: step the servo +45 -> +90 -> 0 -> -45 -> -90 -> 0 ... to check wiring and direction */
void delivery_servo_test(void)
{
  static const int8_t steps[] = {45, 90, 0, -45, -90, 0};
  static uint8_t i = 0;
  char line[80];
  if (!(TIM4->CR1 & TIM_CR1_CEN)) servo_init();   /* camera failed: delivery_init() never ran */
  pan = steps[i];
  i = (i + 1) % sizeof steps;
  servo_write(pan);
  snprintf(line, sizeof line, "servo test: %+d deg -> %d us (+ should look right)\r\n",
           (int)pan, (int)(SERVO_CENTER_US + PAN_SERVO_DIR * pan * SERVO_US_PER_90 / 90.0f));
  print(line);
}

const char *delivery_status(void)
{
  static char buf[112];
  uint32_t now = HAL_GetTick();
  if (link_lost) return "LOWER BOARD NOT ANSWERING (no $O) - wheels stopped";
  if (obst_level >= 2 && mode != M_SELECT) {
    snprintf(buf, sizeof buf, "OBSTACLE level %d ~%d mm - waiting (%s)", obst_level, obst_dist, goal());
    return buf;
  }
  switch (mode) {
  case M_SEARCH: {
    snprintf(buf, sizeof buf, "SEARCH %s  %lu s  %s, pan %+d", goal(),
             (unsigned long)(ELAPSED(now, t_state) / 1000), search_drive ? "forward" : "scanning",
             (int)lroundf(pan));
    break;
  }
  case M_WAIT:      snprintf(buf, sizeof buf, "WAIT %s (lost it)", goal()); break;
  case M_TURN:      snprintf(buf, sizeof buf, "TURN 360 looking for %s  %lu/%d ms", goal(),
                             (unsigned long)turn_spun_ms, TURN_360_MS); break;
  case M_FOUND:     snprintf(buf, sizeof buf, "FOUND %s %+d deg ~%d mm | %s", goal(),
                             (int)lroundf(bearing), (int)lroundf(dist_mm), decide_drive().text); break;
  case M_ARRIVED:
    if (returning) snprintf(buf, sizeof buf, "ARRIVED home");
    else snprintf(buf, sizeof buf, "ARRIVED table %d  touch the button (or wait %d s)", target,
                  RECEIVE_TIMEOUT_MS / 1000);
    break;
  case M_NOT_FOUND: snprintf(buf, sizeof buf, "NOT FOUND %s", goal()); break;
  default:          snprintf(buf, sizeof buf, "SELECT a table on the LCD"); break;
  }
  return buf;
}
