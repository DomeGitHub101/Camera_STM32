/*
 * delivery.h - table delivery mission: pick a table on the touch screen, the
 * pan servo looks for that table's AprilTag, the cat face shows what it sees
 * and drive commands for the lower board go out through link_send().
 */
#ifndef DELIVERY_H
#define DELIVERY_H

#include <stdint.h>
#include "apriltag_lite.h"

/* screen + servo; call once after the camera is running */
void delivery_init(void);
/* after every good camera frame: img_w x img_h = image the tags were found in,
   rotation = clockwise turn (0/90/180/270) that makes that image upright */
void delivery_frame(const at_detection_t *dets, int n, int img_w, int img_h, int rotation, uint32_t now);
/* every main-loop pass: touch, servo sweep, timers, link heartbeat every output_delay ms */
void delivery_poll(uint32_t now, uint32_t output_delay);

/* step the pan servo one test angle further and print it ('s' key) */
void delivery_servo_test(void);

/* one-line mission state, e.g. "SEARCH table 3  9 s left", for the PC viewer */
const char *delivery_status(void);

/* start sending $M,0,0,0 to the lower board every 100 ms (TIM6 interrupt);
   call right after the USART2 init, before the slow camera / LCD set-up */
void link_start(void);
/* every command for the lower board goes through here: "$M,left,right,mode*XX" on USART2
   (left / right = wheel %, -100..100; mode 0 = stop, 1 = drive, 2 = docking).
   It sets the newest command; the TIM6 interrupt sends it every 100 ms. */
void link_send(int left, int right, int mode);

#endif /* DELIVERY_H */
