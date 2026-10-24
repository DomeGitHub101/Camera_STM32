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
/* after every good camera frame (img_w = width of the image the tags were found in) */
void delivery_frame(const at_detection_t *dets, int n, int img_w, uint32_t now);
/* every main-loop pass: touch, servo sweep, timers, link heartbeat every output_delay ms */
void delivery_poll(uint32_t now, uint32_t output_delay);

/* one-line mission state, e.g. "SEARCH table 3  9 s left", for the PC viewer */
const char *delivery_status(void);

/* every command for the lower board goes through here: "$D,speed,turn*XX" */
void link_send(int speed, int turn);

#endif /* DELIVERY_H */
