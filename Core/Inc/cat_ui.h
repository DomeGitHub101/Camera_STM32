/*
 * cat_ui.h - "cat face" UI on the ILI9341 2.8" LCD (320x240 landscape) with
 * the XPT2046 touch panel. Only the parts that change are redrawn.
 *
 * Wiring (Lec11): SPI5 PF7 SCK / PF8 MISO / PF9 MOSI, LCD_CS PC8, LCD_RST PC10,
 * LCD_DC PC12, touch bit-banged: T_IRQ PE2 (EXTI2 falling), T_CLK PE3,
 * T_DO PE4, T_DIN PE5, T_CS PE6.
 */
#ifndef CAT_UI_H
#define CAT_UI_H

#include <stdint.h>

#define UI_TABLES  6

typedef enum {
  EYES_SEARCH,     /* yellow: looking for the table */
  EYES_FOUND,      /* green: target tag in view */
  EYES_WRONG,      /* red: a tag that is not the target */
  EYES_WAIT,       /* orange: stopped, waiting */
  EYES_HAPPY,      /* green closed "smiling" eyes: arrived */
} ui_eyes_t;

typedef enum {
  STATUS_NONE,
  STATUS_SEARCHING,   /* กำลังหาโต๊ะ N */
  STATUS_WRONG,       /* เจอโต๊ะ N — ไม่ใช่ (N = the wrong tag) */
  STATUS_FOUND,       /* เจอโต๊ะ N แล้ว! */
  STATUS_ARRIVED,     /* ถึงโต๊ะ N แล้ว · รับอาหารได้เลย */
  STATUS_NOT_FOUND,   /* ไม่พบโต๊ะ N */
  STATUS_GOING_HOME,  /* กำลังกลับจุดเริ่มต้น */
  STATUS_HOME,        /* ถึงจุดเริ่มต้นแล้ว */
} ui_status_t;

extern volatile uint32_t ui_touch_irq_count;   /* T_IRQ falling edges (EXTI2) */

void ui_init(void);
/* page 1: table buttons, 'selected' (1..6, 0 = none) drawn orange */
void ui_show_select(int selected);
/* page 2: cat face for 'table' */
void ui_show_mission(int table);
/* redraws the eyes only when the state or the pupil position changed */
void ui_set_eyes(ui_eyes_t eyes, int pupil_dx);
/* redraws the status line only when it changed */
void ui_set_status(ui_status_t status, int n);

/* 1 = a new press at (x, y) in screen pixels (one per touch, released before the next) */
int ui_poll_touch(int *x, int *y);
int ui_hit_table(int x, int y);    /* page 1: 1..6, 0 = no button */
int ui_hit_cancel(int x, int y);   /* page 2: 1 = cancel button */
/* page 2 at the table: "รับอาหารแล้ว" button (1 = draw, 0 = erase) */
void ui_show_received(int show);
int ui_hit_received(int x, int y); /* 1 = "รับอาหารแล้ว" touched (only while it is shown) */

#endif /* CAT_UI_H */
