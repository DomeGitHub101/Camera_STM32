/*
 * apriltag_lite.h - small AprilTag (tag36h11) detector for microcontrollers
 *
 * No malloc: all working memory is static, sized for AT_MAX_W x AT_MAX_H.
 * Pipeline: adaptive threshold -> trace dark blob outlines -> fit quads
 *           -> homography -> sample the 8x8 cell grid -> match tag36h11 codes
 */
#ifndef APRILTAG_LITE_H
#define APRILTAG_LITE_H

#include <stdint.h>

#define AT_MAX_W        320     /* largest image the static buffers support */
#define AT_MAX_H        240
#define AT_MAX_DETECT   8       /* tags reported per image */

typedef struct {
  int   min_side;       /* smallest tag (black square side) in pixels, e.g. 10 */
  int   min_contrast;   /* min grey difference between black and white, e.g. 20 */
  int   max_hamming;    /* bit errors corrected, 0..3 (2 is a good default) */
} at_config_t;

typedef struct {
  int   id;             /* tag36h11 ID */
  int   hamming;        /* bit errors corrected */
  int   rotation;       /* tag rotation in the image: 0, 90, 180, 270 (clockwise) */
  float cx, cy;         /* centre in pixels */
  float side;           /* mean side length of the black square in pixels */
  float corners[4][2];  /* black-square corners, clockwise from the tag's top-left */
} at_detection_t;

extern const uint32_t at_tag36h11_count;
extern const uint64_t at_tag36h11_codes[];

/* Default settings */
void at_default_config(at_config_t *cfg);

/* Detect tags in an 8-bit grey image (w <= AT_MAX_W, h <= AT_MAX_H, both multiples of 4).
   Returns the number of detections written to out (at most max_out). Not re-entrant. */
int at_detect(const at_config_t *cfg, const uint8_t *gray, int w, int h,
              at_detection_t *out, int max_out);

#endif /* APRILTAG_LITE_H */
