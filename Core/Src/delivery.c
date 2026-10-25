/*
 * delivery.c - table delivery mission on the STM32.
 *
 * Page 1: touch "โต๊ะ N" -> target = tag ID N, the mission starts.
 * Page 2: the pan servo (PD14, TIM4_CH3) sweeps until the target tag is
 * seen in CONFIRM_TAG_FRAMES frames in a row, then follows it. Only the
 * target tag drives the servo, so it never locks on a wrong tag. Wrong tags
 * are reported once per tag per mission.
 *
 * Driving: decide_drive() turns the target's bearing and distance into one
 * command {speed, turn} plus a readable hint (MOVE CLOSER / MOVE BACK /
 * TURN LEFT / TURN RIGHT / HOLD). Without the wheel board the hints are only
 * printed; the same {speed, turn} goes to link_send() for the lower board.
 * In position (centred and at TARGET_DIST_MM) for HOLD_TIME_MS = arrived.
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
#define RECEIVE_TIMEOUT_MS 10000    /* at the table: go back after this long without "รับอาหารแล้ว" (0 = wait) */
#define HOME_SHOW_MS       3000     /* "ถึงจุดเริ่มต้นแล้ว" shown this long, then the table page */
#define NOT_FOUND_SHOW_MS  2500
#define WRONG_SHOW_MS      1600     /* red eyes + text after a wrong tag */
#define LOST_MS            700      /* target out of view this long -> stop and wait (orange) */
#define WAIT_MS            1500     /* ... then sweep again */
/* ---- pan servo: PD14 = TIM4_CH3, 50 Hz, 500..2500 us = -90..+90 deg ---- */
#define SERVO_CENTER_US    1500
#define SERVO_US_PER_90    1000
#define PAN_SERVO_ENABLED  0        /* 0 = no servo yet: the camera looks straight ahead and never sweeps */
#define PAN_SERVO_DIR      1        /* -1 if the camera turns left when it should turn right */
#define PAN_LIMIT_DEG      90
#define SWEEP_DEG_PER_S    45
#define SEARCH_TIMEOUT_MS  15000    /* keep searching this long after picking a table (or after losing it) */
#define PAN_DEADBAND_DEG   8        /* tag within +- this of the image centre: servo stays still */
#define SETTLE_FRAMES      2        /* frames after a servo move that were (partly) taken while it moved */
/* ---- drive commands: get the tag centred at TARGET_DIST_MM ---- */
#define TARGET_DIST_MM     800      /* stop this far from the tag */
#define DIST_TOL_MM        100      /* +- this is close enough */
#define CENTER_TOL_DEG     5        /* tag within +- this of straight ahead is centred */
#define ALIGN_DEG          15       /* tag further off than this: turn on the spot first */
#define DRIVE_SPEED_MAX    200      /* forward speed when far away */
#define DRIVE_SPEED_MIN    60       /* forward speed just outside the tolerance */
#define SLOW_DOWN_MM       600      /* speed ramps from MIN to MAX over this much extra distance */
#define REVERSE_SPEED      80       /* backing off when too close */
#define TURN_MAX_DEG       45
#define HOLD_TIME_MS       1000     /* in position this long = arrived */
#define LINK_ON_USART2     0        /* 1 = also send the raw frames on USART2 to the lower board */
#define TOUCH_DEBUG        1        /* 1 = print every touch (for checking the touch mapping) */

#define NUM_TAG_IDS        587      /* tag36h11 */
/* ms from t to now; 0 if t is (slightly) in the future */
#define ELAPSED(now, t)    ((int32_t)((now) - (t)) > 0 ? (int32_t)((now) - (t)) : 0)

typedef enum { M_SELECT, M_SEARCH, M_WAIT, M_FOUND, M_ARRIVED, M_NOT_FOUND } mission_t;

static mission_t mode = M_SELECT;
static int target = 0, selected = 0;
static float pan = 0;                 /* deg, + = camera looks right */
static int sweep_dir = 1;
static uint32_t t_poll, t_seen, t_state, t_wrong_until, t_beat;
static uint8_t beat_now = 0;
static int wrong_id = -1;
static uint8_t reported[(NUM_TAG_IDS + 7) / 8];
static float bearing = 0, dist_mm = 0;
static uint32_t t_in_pos = 0;         /* since when the robot has been in position, 0 = not */
static int settle = 0;                /* frames still to skip after the servo moved */
static uint8_t returning = 0;         /* 1 = on the way back to HOME_TAG_ID */

typedef struct {
  int speed;                          /* + forward, - backward */
  int turn;                           /* deg, + = right */
  char text[64];                      /* readable hint */
} drive_t;
static int prev_id[AT_MAX_DETECT], prev_streak[AT_MAX_DETECT], prev_n = 0;

static void print(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

/* ---- link to the lower board ---- */
void link_send(int speed, int turn)
{
  char body[32], line[48];
  snprintf(body, sizeof body, "D,%d,%d", speed, turn);
  uint8_t sum = 0;
  for (const char *p = body; *p; p++) sum ^= (uint8_t)*p;   /* XOR of everything between $ and * */
  snprintf(line, sizeof line, "-> $%s*%02X\r\n", body, sum);
  print(line);
#if LINK_ON_USART2
  extern UART_HandleTypeDef huart2;
  HAL_UART_Transmit(&huart2, (uint8_t *)line + 3, strlen(line) - 3, 100);
#endif
}

/* ---- pan servo ---- */
static void servo_write(float deg)
{
  if (deg > PAN_LIMIT_DEG) deg = PAN_LIMIT_DEG;
  if (deg < -PAN_LIMIT_DEG) deg = -PAN_LIMIT_DEG;
  TIM4->CCR3 = (uint32_t)(SERVO_CENTER_US + PAN_SERVO_DIR * deg * SERVO_US_PER_90 / 90.0f + 0.5f);
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
  servo_write(0);
  TIM4->CR1 = TIM_CR1_ARPE;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->CR1 |= TIM_CR1_CEN;
}

/* Only the target tag comes here.
   bearing = servo angle + the tag's angle in the image, which is only right
   when the image was taken at that servo angle. So the servo stays still
   while the tag is near the centre, jumps straight onto the tag when it
   drifts out, and the frames taken while it moves are not used for the bearing. */
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
  if (settle > 0) {                   /* image taken while the servo was moving */
    settle--;
    return;
  }
  bearing = pan + err;
  if (PAN_SERVO_ENABLED && fabsf(err) > PAN_DEADBAND_DEG) {
    pan = bearing;
    if (pan > PAN_LIMIT_DEG) pan = PAN_LIMIT_DEG;
    if (pan < -PAN_LIMIT_DEG) pan = -PAN_LIMIT_DEG;
    servo_write(pan);
    settle = SETTLE_FRAMES;
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
  return fabsf(bearing) <= CENTER_TOL_DEG && fabsf(dist_mm - TARGET_DIST_MM) <= DIST_TOL_MM;
}

/* What the robot should do now. Only meaningful while the target is in view (M_FOUND). */
static drive_t decide_drive(void)
{
  drive_t d = {0, 0, ""};
  if (mode != M_FOUND) {
    snprintf(d.text, sizeof d.text, mode == M_SEARCH ? "STOP (searching)" : "STOP (no tag)");
    return d;
  }
  int off = (int)lroundf(bearing);                       /* + = tag is to the right */
  int err = (int)lroundf(dist_mm) - TARGET_DIST_MM;      /* + = too far */
  int len = 0;

  if (abs(off) > CENTER_TOL_DEG) {
    d.turn = off > TURN_MAX_DEG ? TURN_MAX_DEG : (off < -TURN_MAX_DEG ? -TURN_MAX_DEG : off);
    len += snprintf(d.text + len, sizeof d.text - len, "TURN %s %d deg", off > 0 ? "RIGHT" : "LEFT", abs(off));
  }
  if (abs(off) > ALIGN_DEG) {
    snprintf(d.text + len, sizeof d.text - len, " first");   /* far off to the side: turn on the spot */
    return d;
  }
  if (err > DIST_TOL_MM) {
    float k = (float)(err - DIST_TOL_MM) / SLOW_DOWN_MM;
    if (k > 1) k = 1;
    d.speed = DRIVE_SPEED_MIN + (int)lroundf(k * (DRIVE_SPEED_MAX - DRIVE_SPEED_MIN));
    snprintf(d.text + len, sizeof d.text - len, "%sMOVE CLOSER %d mm", len ? ", " : "", err);
  } else if (err < -DIST_TOL_MM) {
    d.speed = -REVERSE_SPEED;
    snprintf(d.text + len, sizeof d.text - len, "%sMOVE BACK %d mm", len ? ", " : "", -err);
  } else if (!len) {
    snprintf(d.text, sizeof d.text, "HOLD (in position)");
  }
  return d;
}

static void back_to_select(void)
{
  mode = M_SELECT;
  pan = 0;
  servo_write(0);
  link_send(0, 0);
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

static void reset_search(void)
{
  memset(reported, 0, sizeof reported);
  prev_n = 0;
  sweep_dir = 1;
  wrong_id = -1;
  t_wrong_until = 0;
  t_seen = 0;
  t_in_pos = 0;
  mode = M_SEARCH;
  t_state = HAL_GetTick();
  beat_now = 1;
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
        link_send(0, 0);
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
  if (mode == M_SELECT) return;
  /* a touch may have started a mission (redraw + delay): read the clock again */
  now = HAL_GetTick();
  t_poll = now;

  switch (mode) {
  case M_SEARCH:
    /* target seen but not confirmed yet: pan_update() holds the camera on it */
    if (t_seen && ELAPSED(now, t_seen) < LOST_MS) break;
    if (PAN_SERVO_ENABLED) {
      pan += sweep_dir * (float)SWEEP_DEG_PER_S * dt / 1000.0f;
      if (pan >= PAN_LIMIT_DEG) { pan = PAN_LIMIT_DEG; sweep_dir = -1; }
      if (pan <= -PAN_LIMIT_DEG) { pan = -PAN_LIMIT_DEG; sweep_dir = 1; }
      servo_write(pan);
      settle = SETTLE_FRAMES;         /* frames taken during the sweep give no bearing */
    }
    if (ELAPSED(now, t_state) > SEARCH_TIMEOUT_MS) {
      snprintf(line, sizeof line, "NOT FOUND %s%s\r\n", goal(), returning ? ", searching again" : "");
      print(line);
      if (returning) t_state = now;   /* never give up on the way back (cancel stops it) */
      else enter(M_NOT_FOUND, now);
    }
    break;
  case M_FOUND:
    if (ELAPSED(now, t_seen) > LOST_MS) { t_in_pos = 0; enter(M_WAIT, now); }
    break;
  case M_WAIT:
    if (ELAPSED(now, t_state) > WAIT_MS) enter(M_SEARCH, now);
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

  /* drive command: at every change and every output_delay */
  if (beat_now || now - t_beat >= output_delay) {
    beat_now = 0;
    t_beat = now;
    drive_t d = decide_drive();
    if (mode == M_FOUND) print_found();
    if (mode != M_ARRIVED && mode != M_NOT_FOUND) {
      char hint[80];
      snprintf(hint, sizeof hint, "DRIVE %s\r\n", d.text);
      print(hint);
    }
    link_send(d.speed, d.turn);
  }
}

void delivery_init(void)
{
  servo_init();
  ui_init();
  ui_show_select(0);
  t_poll = HAL_GetTick();
  print("touch a table on the screen (TABLE / FOUND / DRIVE / INVALID / ARRIVED / RECEIVED / RETURN lines and -> $D commands follow)\r\n");
}

const char *delivery_status(void)
{
  static char buf[112];
  uint32_t now = HAL_GetTick();
  switch (mode) {
  case M_SEARCH: {
    int left = (SEARCH_TIMEOUT_MS - ELAPSED(now, t_state) + 999) / 1000;
    snprintf(buf, sizeof buf, "SEARCH %s  %d s left  pan %+d", goal(), left < 0 ? 0 : left, (int)lroundf(pan));
    break;
  }
  case M_WAIT:      snprintf(buf, sizeof buf, "WAIT %s (lost it)", goal()); break;
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
