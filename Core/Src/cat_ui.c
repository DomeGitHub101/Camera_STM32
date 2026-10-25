/*
 * cat_ui.c - "cat face" UI on the ILI9341 (320x240 landscape) + XPT2046 touch.
 *
 * Every element (button, eye, text line) is drawn into a small RAM canvas
 * first and sent to the LCD in one window, so nothing flickers and only the
 * parts that change are sent. Thai text comes as 1-bit bitmaps from
 * thai_text.c (tools/gen_thai_text.py).
 */
#include "cat_ui.h"
#include "main.h"
#include "spi.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_Touchscreen.h"
#include "thai_text.h"
#include <stdlib.h>

/* ---- colours (RGB565 from #RRGGBB) ---- */
#define RGB(c)        ((uint16_t)((((c) >> 8) & 0xF800) | (((c) >> 5) & 0x07E0) | (((c) >> 3) & 0x001F)))
#define C_BG          RGB(0x0F1A24)
#define C_BUTTON      RGB(0x223445)
#define C_SELECTED    RGB(0xE8650F)
#define C_TEXT        RGB(0xE9EEF3)
#define C_YELLOW      RGB(0xF5B301)   /* searching */
#define C_GREEN       RGB(0x35D07F)   /* target found / arrived */
#define C_RED         RGB(0xFF5B4F)   /* wrong tag */
#define C_ORANGE      RGB(0xFF9A3C)   /* stopped, waiting */

/* ---- layout ---- */
#define SCR_W         320
#define SCR_H         240
#define TITLE_Y       12
#define BTN_W         92
#define BTN_H         70
#define BTN_R         10
#define BTN_GAP       14
#define BTN_X0        ((SCR_W - (3 * BTN_W + 2 * BTN_GAP)) / 2)
#define BTN_Y0        48
#define EYE_W         88
#define EYE_H         94
#define EYE_R         38
#define EYE_TOP       44
#define EYE_L_CX      94
#define EYE_R_CX      226
#define PUPIL_W       28
#define PUPIL_H       66
#define PUPIL_R       14
#define PUPIL_MAX_DX  16
#define SMILE_R_OUT   38              /* closed "smiling" eye: upper half ring, 10 px thick */
#define SMILE_R_IN    28
#define STATUS_Y      160
#define CANCEL_W      78
#define CANCEL_H      26
#define CANCEL_X      (SCR_W - 6 - CANCEL_W)
#define CANCEL_Y      (SCR_H - 6 - CANCEL_H)
#define RECV_W        120             /* "รับอาหารแล้ว" button, bottom middle, at the table */
#define RECV_H        32
#define RECV_X        106
#define RECV_Y        (SCR_H - 6 - RECV_H)

/* ---- touch: raw XPT2046 readings mapped to the landscape screen ----
   Not TP_Read_Coordinates(): its fixed portrait formula wraps around near the
   panel edges (on the board the top row read as y = 0 or y = 250). */
#define TP_CMD_X         0x90   /* XPT2046 channel that follows the long side (screen x) */
#define TP_CMD_Y         0xD0   /* channel that follows the short side (screen y) */
/* 12-bit ADC values at the screen edges (calibration: the TOUCH lines print
   "adc a,b"; touch the corners and put the readings here). Edges may be given
   in either order: a reversed axis just has LEFT > RIGHT or TOP > BOTTOM. */
#define TP_ADC_X_LEFT    3904
#define TP_ADC_X_RIGHT   387
#define TP_ADC_Y_TOP     3740   /* y axis runs the other way on this panel: bottom (cancel button, y~221) read ~500 */
#define TP_ADC_Y_BOTTOM  240
#define TP_SAMPLES       16
#define TP_MAX_SPREAD    150    /* middle half of the samples must be this close */

volatile uint32_t ui_touch_irq_count = 0;
static volatile uint8_t touch_flag = 0;
uint16_t ui_touch_raw[2];             /* last library coordinates, for calibration prints */

/* ---- canvas: one element at a time, big-endian RGB565 like the LCD wants ---- */
#define CANVAS_BYTES  (SCR_W * 26 * 2)
static uint8_t canvas[CANVAS_BYTES];
static int cv_w, cv_h;

static void cv_begin(int w, int h, uint16_t colour)
{
  if (w * h * 2 > CANVAS_BYTES) h = CANVAS_BYTES / 2 / w;
  cv_w = w;
  cv_h = h;
  for (int i = 0; i < w * h; i++) {
    canvas[2 * i] = colour >> 8;
    canvas[2 * i + 1] = colour & 0xFF;
  }
}

static inline void cv_pixel(int x, int y, uint16_t colour)
{
  if (x < 0 || y < 0 || x >= cv_w || y >= cv_h) return;
  uint8_t *p = canvas + 2 * (y * cv_w + x);
  p[0] = colour >> 8;
  p[1] = colour & 0xFF;
}

/* 1 if (x, y) is inside a w x h rectangle with corner radius r */
static int in_rrect(int x, int y, int w, int h, int r)
{
  if (x < 0 || y < 0 || x >= w || y >= h) return 0;
  int dx = x < r ? r - 1 - x : (x >= w - r ? x - (w - r) : -1);
  int dy = y < r ? r - 1 - y : (y >= h - r ? y - (h - r) : -1);
  if (dx < 0 || dy < 0) return 1;
  return (2 * dx + 1) * (2 * dx + 1) + (2 * dy + 1) * (2 * dy + 1) <= 4 * r * r;
}

static void cv_rrect(int x0, int y0, int w, int h, int r, uint16_t colour)
{
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      if (in_rrect(x, y, w, h, r)) cv_pixel(x0 + x, y0 + y, colour);
}

static void cv_mono(const mono_bmp_t *b, int x0, int y0, uint16_t colour)
{
  int stride = (b->w + 7) / 8;
  for (int y = 0; y < b->h; y++)
    for (int x = 0; x < b->w; x++)
      if (b->bits[y * stride + x / 8] & (0x80 >> (x % 8))) cv_pixel(x0 + x, y0 + y, colour);
}

/* send the canvas to the LCD at (x, y) */
static void cv_send(int x, int y)
{
  ILI9341_Set_Address(x, y, x + cv_w - 1, y + cv_h - 1);
  HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
  uint8_t *p = canvas;
  uint32_t n = (uint32_t)cv_w * cv_h * 2;
  while (n) {
    uint16_t chunk = n > 65534 ? 65534 : (uint16_t)n;
    HAL_SPI_Transmit(&hspi5, p, chunk, 100);
    p += chunk;
    n -= chunk;
  }
  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* ---- a line of text = Thai bitmaps + ASCII glyphs side by side ---- */
typedef struct {
  const mono_bmp_t *part[24];
  int n;
} line_t;

static void line_bmp(line_t *l, const mono_bmp_t *b)
{
  if (l->n < 24) l->part[l->n++] = b;
}

static void line_str(line_t *l, const mono_font_t *f, const char *s)
{
  for (; *s; s++)
    if ((uint8_t)*s >= f->first && (uint8_t)*s < f->first + f->count) line_bmp(l, &f->glyph[*s - f->first]);
}

static void line_num(line_t *l, const mono_font_t *f, int n)
{
  char buf[8];
  int i = sizeof buf - 1;
  buf[i] = 0;
  if (n < 0) n = 0;
  do { buf[--i] = (char)('0' + n % 10); n /= 10; } while (n && i > 0);
  line_str(l, f, buf + i);
}

static int line_width(const line_t *l)
{
  int w = 0;
  for (int i = 0; i < l->n; i++) w += l->part[i]->w;
  return w;
}

static int line_height(const line_t *l)
{
  int h = 0;
  for (int i = 0; i < l->n; i++) if (l->part[i]->h > h) h = l->part[i]->h;
  return h;
}

static void cv_line(const line_t *l, int x, int y, uint16_t colour)
{
  for (int i = 0; i < l->n; i++) {
    cv_mono(l->part[i], x, y, colour);
    x += l->part[i]->w;
  }
}

/* a full-width text line centred on the screen, background included */
static void draw_centered_line(const line_t *l, int y, uint16_t colour)
{
  cv_begin(SCR_W, line_height(l), C_BG);
  cv_line(l, (SCR_W - line_width(l)) / 2, 0, colour);
  cv_send(0, y);
}

/* ---- page 1 ---- */
static void draw_table_button(int i, int selected)
{
  int col = (i - 1) % 3, row = (i - 1) / 3;
  line_t l = {0};
  line_bmp(&l, &TXT_TABLE);
  line_str(&l, &FONT_BTN, " ");
  line_num(&l, &FONT_BTN, i);
  cv_begin(BTN_W, BTN_H, C_BG);
  cv_rrect(0, 0, BTN_W, BTN_H, BTN_R, selected ? C_SELECTED : C_BUTTON);
  cv_line(&l, (BTN_W - line_width(&l)) / 2, (BTN_H - line_height(&l)) / 2, C_TEXT);
  cv_send(BTN_X0 + col * (BTN_W + BTN_GAP), BTN_Y0 + row * (BTN_H + BTN_GAP));
}

void ui_show_select(int selected)
{
  ILI9341_Fill_Screen(C_BG);
  line_t l = {0};
  line_bmp(&l, &TXT_TITLE);
  draw_centered_line(&l, TITLE_Y, C_TEXT);
  for (int i = 1; i <= UI_TABLES; i++) draw_table_button(i, i == selected);
}

int ui_hit_table(int x, int y)
{
  for (int i = 1; i <= UI_TABLES; i++) {
    int bx = BTN_X0 + (i - 1) % 3 * (BTN_W + BTN_GAP), by = BTN_Y0 + (i - 1) / 3 * (BTN_H + BTN_GAP);
    if (x >= bx && x < bx + BTN_W && y >= by && y < by + BTN_H) return i;
  }
  return 0;
}

/* ---- page 2 ---- */
static int cur_eyes = -1, cur_dx = 0;
static int cur_status = -1, cur_status_n = -1;
static int received_shown = 0;        /* "รับอาหารแล้ว" button on screen */

static void draw_eye(int cx, ui_eyes_t eyes, int dx)
{
  cv_begin(EYE_W, EYE_H, C_BG);
  if (eyes == EYES_HAPPY) {
    /* closed eye: upper half of a ring (an upside-down U) */
    int ccx = EYE_W / 2, ccy = EYE_H / 2 + 14;
    for (int y = 0; y <= ccy; y++)
      for (int x = 0; x < EYE_W; x++) {
        int ddx = 2 * (x - ccx) + 1, ddy = 2 * (y - ccy);
        int d2 = ddx * ddx + ddy * ddy;
        if (d2 <= 4 * SMILE_R_OUT * SMILE_R_OUT && d2 >= 4 * SMILE_R_IN * SMILE_R_IN) cv_pixel(x, y, C_GREEN);
      }
  } else {
    static const uint16_t colour[] = {C_YELLOW, C_GREEN, C_RED, C_ORANGE};
    cv_rrect(0, 0, EYE_W, EYE_H, EYE_R, colour[eyes]);
    cv_rrect((EYE_W - PUPIL_W) / 2 + dx, (EYE_H - PUPIL_H) / 2, PUPIL_W, PUPIL_H, PUPIL_R, C_BG);
  }
  cv_send(cx - EYE_W / 2, EYE_TOP);
}

void ui_set_eyes(ui_eyes_t eyes, int dx)
{
  if (dx > PUPIL_MAX_DX) dx = PUPIL_MAX_DX;
  if (dx < -PUPIL_MAX_DX) dx = -PUPIL_MAX_DX;
  if (eyes == EYES_HAPPY) dx = 0;
  if ((int)eyes == cur_eyes && dx == cur_dx) return;
  cur_eyes = eyes;
  cur_dx = dx;
  draw_eye(EYE_L_CX, eyes, dx);
  draw_eye(EYE_R_CX, eyes, dx);
}

void ui_set_status(ui_status_t status, int n)
{
  if ((int)status == cur_status && n == cur_status_n) return;
  cur_status = status;
  cur_status_n = n;
  line_t l = {0};
  uint16_t colour = C_TEXT;
  switch (status) {
  case STATUS_SEARCHING:
    line_bmp(&l, &TXT_SEARCHING);
    break;
  case STATUS_WRONG:
    line_bmp(&l, &TXT_FOUND);
    colour = C_RED;
    break;
  case STATUS_FOUND:
    line_bmp(&l, &TXT_FOUND);
    colour = C_GREEN;
    break;
  case STATUS_ARRIVED:
    line_bmp(&l, &TXT_ARRIVED);
    colour = C_GREEN;
    break;
  case STATUS_NOT_FOUND:
    line_bmp(&l, &TXT_NOT_FOUND);
    colour = C_ORANGE;
    break;
  case STATUS_GOING_HOME:
    line_bmp(&l, &TXT_GOING_HOME);
    break;
  case STATUS_HOME:
    line_bmp(&l, &TXT_HOME_ARRIVED);
    colour = C_GREEN;
    break;
  default:
    break;
  }
  if (status != STATUS_NONE && status != STATUS_GOING_HOME && status != STATUS_HOME) {
    line_str(&l, &FONT_STATUS, " ");
    line_num(&l, &FONT_STATUS, n);
  }
  if (status == STATUS_WRONG) { line_str(&l, &FONT_STATUS, " "); line_bmp(&l, &TXT_NOT_THIS); }
  if (status == STATUS_FOUND) { line_str(&l, &FONT_STATUS, " "); line_bmp(&l, &TXT_FOUND_DONE); }
  if (status == STATUS_ARRIVED) { line_str(&l, &FONT_STATUS, " "); line_bmp(&l, &TXT_ARRIVED_DONE); }
  if (l.n == 0) line_str(&l, &FONT_STATUS, " ");
  draw_centered_line(&l, STATUS_Y, colour);
}

void ui_show_mission(int table)
{
  ILI9341_Fill_Screen(C_BG);
  cur_eyes = -1;
  cur_status = -1;
  received_shown = 0;

  /* bottom left: ID N · tag36h11 */
  line_t l = {0};
  line_str(&l, &FONT_SMALL, "ID ");
  line_num(&l, &FONT_SMALL, table);
  line_str(&l, &FONT_SMALL, " ");
  line_bmp(&l, &TXT_TAG_FAMILY);
  cv_begin(line_width(&l), line_height(&l), C_BG);
  cv_line(&l, 0, 0, C_TEXT);
  cv_send(6, SCR_H - 6 - line_height(&l) - (CANCEL_H - line_height(&l)) / 2);

  /* bottom right: cancel button */
  cv_begin(CANCEL_W, CANCEL_H, C_BG);
  cv_rrect(0, 0, CANCEL_W, CANCEL_H, 8, C_BUTTON);
  cv_mono(&TXT_CANCEL, (CANCEL_W - TXT_CANCEL.w) / 2, (CANCEL_H - TXT_CANCEL.h) / 2, C_TEXT);
  cv_send(CANCEL_X, CANCEL_Y);
}

int ui_hit_cancel(int x, int y)
{
  /* a little bigger than the button: it is small */
  return x >= CANCEL_X - 8 && y >= CANCEL_Y - 8;
}

void ui_show_received(int show)
{
  cv_begin(RECV_W, RECV_H, C_BG);
  if (show) {
    cv_rrect(0, 0, RECV_W, RECV_H, 10, C_SELECTED);
    cv_mono(&TXT_RECEIVED, (RECV_W - TXT_RECEIVED.w) / 2, (RECV_H - TXT_RECEIVED.h) / 2, C_TEXT);
  }
  cv_send(RECV_X, RECV_Y);
  received_shown = show;
}

int ui_hit_received(int x, int y)
{
  return received_shown && x >= RECV_X && x < RECV_X + RECV_W && y >= RECV_Y - 8;
}

/* ---- touch: EXTI2_IRQHandler (stm32f7xx_it.c, CubeMX) calls this ---- */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == GPIO_PIN_2) {
    ui_touch_irq_count++;
    touch_flag = 1;
  }
}

static int cmp_u16(const void *a, const void *b)
{
  return (int)*(const uint16_t *)a - (int)*(const uint16_t *)b;
}

/* One 12-bit XPT2046 conversion. The library's TP_Read() returns it in the top
   12 of 16 bits; the clock is slowed down in ILI9341_Touchscreen.c (TP_DELAY). */
static uint16_t tp_adc(uint8_t cmd)
{
  TP_Write(cmd);
  return TP_Read() >> 4;
}

/* Median of TP_SAMPLES readings per axis; 0 = released or too noisy */
static int tp_read_adc(uint16_t *ax, uint16_t *ay)
{
  uint16_t xs[TP_SAMPLES], ys[TP_SAMPLES];
  HAL_GPIO_WritePin(TP_CS_PORT, TP_CS_PIN, GPIO_PIN_RESET);
  for (int i = 0; i < TP_SAMPLES; i++) {
    xs[i] = tp_adc(TP_CMD_X);
    ys[i] = tp_adc(TP_CMD_Y);
  }
  HAL_GPIO_WritePin(TP_CS_PORT, TP_CS_PIN, GPIO_PIN_SET);
  if (HAL_GPIO_ReadPin(TP_IRQ_PORT, TP_IRQ_PIN) != GPIO_PIN_RESET) return 0;   /* lifted meanwhile */
  qsort(xs, TP_SAMPLES, sizeof xs[0], cmp_u16);
  qsort(ys, TP_SAMPLES, sizeof ys[0], cmp_u16);
  /* the middle half must agree, otherwise the reading is noise */
  if (xs[TP_SAMPLES * 3 / 4] - xs[TP_SAMPLES / 4] > TP_MAX_SPREAD ||
      ys[TP_SAMPLES * 3 / 4] - ys[TP_SAMPLES / 4] > TP_MAX_SPREAD) return 0;
  *ax = xs[TP_SAMPLES / 2];
  *ay = ys[TP_SAMPLES / 2];
  return 1;
}

int ui_poll_touch(int *x, int *y)
{
  static uint8_t down = 0;
  static uint32_t t_pressed = 0;
  int pressed = HAL_GPIO_ReadPin(TP_IRQ_PORT, TP_IRQ_PIN) == GPIO_PIN_RESET;

  if (down) {                         /* wait until released for 60 ms */
    if (pressed) t_pressed = HAL_GetTick();
    else if (HAL_GetTick() - t_pressed > 60) { down = 0; touch_flag = 0; }
    return 0;
  }
  if (!touch_flag) return 0;
  if (!pressed) { touch_flag = 0; return 0; }

  uint16_t ax, ay;
  if (!tp_read_adc(&ax, &ay)) return 0;   /* noisy or released: try again next time */
  touch_flag = 0;
  down = 1;
  t_pressed = HAL_GetTick();
  ui_touch_raw[0] = ax;
  ui_touch_raw[1] = ay;

  int sx = (int)(ax - TP_ADC_X_LEFT) * (SCR_W - 1) / (TP_ADC_X_RIGHT - TP_ADC_X_LEFT);
  int sy = (int)(ay - TP_ADC_Y_TOP) * (SCR_H - 1) / (TP_ADC_Y_BOTTOM - TP_ADC_Y_TOP);
  if (sx < 0) sx = 0;
  if (sx >= SCR_W) sx = SCR_W - 1;
  if (sy < 0) sy = 0;
  if (sy >= SCR_H) sy = SCR_H - 1;
  *x = sx;
  *y = sy;
  return 1;
}

/* ---- start-up ---- */
void ui_init(void)
{
  /* LCD and touch pins, EXTI2 and SPI5 are set up by CubeMX (gpio.c, spi.c) */
  ILI9341_Init();                                 /* also runs MX_SPI5_Init() */
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);      /* landscape 320x240 */
}
