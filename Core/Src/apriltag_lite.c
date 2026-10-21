/*
 * apriltag_lite.c - small AprilTag (tag36h11) detector for microcontrollers
 *
 * 1. Adaptive threshold: per 4x4 tile min/max, spread over 5x5 tiles,
 *    pixel = dark / light (or unknown where the contrast is too low).
 * 2. Trace the outline of every dark blob (Moore neighbour tracing).
 * 3. Fit a quad to the outline: 4 extreme points -> least-squares edge
 *    lines -> line intersections give sub-pixel corners.
 * 4. Homography tag -> image, sample the 8x8 cell grid (black border +
 *    6x6 data bits) and the white ring around it.
 * 5. Match the 36 data bits (4 rotations) against all tag36h11 codes.
 */
/* The Debug build uses -O0, which makes the detector several times slower */
#pragma GCC optimize ("O2")

#include "apriltag_lite.h"
#include <math.h>
#include <string.h>

#define TILE          4
#define TILES_W       (AT_MAX_W / TILE)
#define TILES_H       (AT_MAX_H / TILE)
#define MAX_PTS       2048      /* longest outline that is fitted */
#define MAX_CANDIDATES 400     /* tag-sized outlines tried per image (bounds the run time) */

/* Rejection hook: the host test build defines AT_FAIL to log the reason */
#ifndef AT_FAIL
#define AT_FAIL(reason) return 0
#endif

enum { PX_DARK = 0, PX_LIGHT = 1, PX_UNKNOWN = 2, PX_MARK = 0x80 };

static uint8_t bin[AT_MAX_W * AT_MAX_H];
static uint8_t tmin[TILES_W * TILES_H], tmax[TILES_W * TILES_H];
static uint8_t smin[TILES_W * TILES_H], smax[TILES_W * TILES_H];
static int16_t pts_x[MAX_PTS], pts_y[MAX_PTS];

/* 8 neighbours, clockwise (image y points down), starting West */
static const int8_t DX[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
static const int8_t DY[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
static const int8_t DIR_OF[3][3] = { {1, 2, 3}, {0, -1, 4}, {7, 6, 5} };   /* [dy+1][dx+1] */

void at_default_config(at_config_t *cfg)
{
  cfg->min_side = 12;
  cfg->min_contrast = 20;
  cfg->max_hamming = 2;
}

/* ---------------------------------------------------------------- threshold */

static void threshold(const at_config_t *cfg, const uint8_t *g, int w, int h)
{
  int tw = w / TILE, th = h / TILE;

  for (int ty = 0; ty < th; ty++)
    for (int tx = 0; tx < tw; tx++) {
      uint8_t mn = 255, mx = 0;
      for (int y = 0; y < TILE; y++) {
        const uint8_t *p = g + (ty * TILE + y) * w + tx * TILE;
        for (int x = 0; x < TILE; x++) {
          if (p[x] < mn) mn = p[x];
          if (p[x] > mx) mx = p[x];
        }
      }
      tmin[ty * tw + tx] = mn;
      tmax[ty * tw + tx] = mx;
    }

  /* spread over the 5x5 neighbouring tiles so each edge sees both sides */
  for (int ty = 0; ty < th; ty++)
    for (int tx = 0; tx < tw; tx++) {
      uint8_t mn = 255, mx = 0;
      for (int dy = -2; dy <= 2; dy++) {
        int yy = ty + dy;
        if (yy < 0 || yy >= th) continue;
        for (int dx = -2; dx <= 2; dx++) {
          int xx = tx + dx;
          if (xx < 0 || xx >= tw) continue;
          if (tmin[yy * tw + xx] < mn) mn = tmin[yy * tw + xx];
          if (tmax[yy * tw + xx] > mx) mx = tmax[yy * tw + xx];
        }
      }
      smin[ty * tw + tx] = mn;
      smax[ty * tw + tx] = mx;
    }

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      int t = (y / TILE) * tw + x / TILE;
      int mn = smin[t], mx = smax[t];
      uint8_t v;
      if (mx - mn < cfg->min_contrast) v = PX_UNKNOWN;
      else v = (g[y * w + x] > (mn + mx) / 2) ? PX_LIGHT : PX_DARK;
      bin[y * w + x] = v;
    }

  /* a non-dark frame keeps the tracer inside the image */
  for (int x = 0; x < w; x++) bin[x] = bin[(h - 1) * w + x] = PX_UNKNOWN;
  for (int y = 0; y < h; y++) bin[y * w] = bin[y * w + w - 1] = PX_UNKNOWN;
}

/* ------------------------------------------------------------------ tracing */

/* Trace the outline through (sx, sy), whose west neighbour is not dark.
   Stores up to MAX_PTS points and returns the outline length. */
static int trace(int w, int sx, int sy)
{
  int n = 0, x = sx, y = sy, back = 0, first_return = 0;

  for (;;) {
    if (n < MAX_PTS) { pts_x[n] = (int16_t)x; pts_y[n] = (int16_t)y; }
    n++;
    bin[y * w + x] |= PX_MARK;

    int k, nd = 0;
    for (k = 1; k <= 8; k++) {                 /* clockwise from the background side */
      nd = (back + k) & 7;
      if ((bin[(y + DY[nd]) * w + x + DX[nd]] & 3) == PX_DARK) break;
    }
    if (k > 8) break;                          /* isolated pixel */

    int pd = (back + k - 1) & 7;               /* last background neighbour */
    int bx = x + DX[pd], by = y + DY[pd];
    x += DX[nd];
    y += DY[nd];
    back = DIR_OF[by - y + 1][bx - x + 1];

    if (x == sx && y == sy) {
      if (back == 0) break;                    /* entered the start like the first time */
      if (first_return) { n = first_return; break; }
      first_return = n;
    }
    if (n > 8 * MAX_PTS) break;                /* safety */
  }
  return n;
}

/* ---------------------------------------------------------------- quad fit */

typedef struct { float px, py, dx, dy; } line_t;   /* point + unit direction */

/* Least-squares line through `count` outline points starting at i0 (indices wrap) */
static int fit_line(int n, int i0, int count, float cx, float cy, line_t *L, float *max_dev)
{
  float mx = 0, my = 0;
  for (int k = 0; k < count; k++) {
    int i = (i0 + k) % n;
    mx += pts_x[i];
    my += pts_y[i];
  }
  mx /= count;
  my /= count;

  float sxx = 0, sxy = 0, syy = 0;
  for (int k = 0; k < count; k++) {
    int i = (i0 + k) % n;
    float ex = pts_x[i] - mx, ey = pts_y[i] - my;
    sxx += ex * ex;
    sxy += ex * ey;
    syy += ey * ey;
  }
  float a = 0.5f * atan2f(2.0f * sxy, sxx - syy);
  float dx = cosf(a), dy = sinf(a);

  float dev = 0;
  for (int k = 0; k < count; k++) {
    int i = (i0 + k) % n;
    float d = fabsf((pts_x[i] - mx) * dy - (pts_y[i] - my) * dx);
    if (d > dev) dev = d;
  }
  *max_dev = dev;

  /* the outline runs along the dark pixels: move the line half a pixel outwards */
  float nx = -dy, ny = dx;
  if (nx * (mx - cx) + ny * (my - cy) < 0) { nx = -nx; ny = -ny; }
  L->px = mx + 0.5f * nx;
  L->py = my + 0.5f * ny;
  L->dx = dx;
  L->dy = dy;
  return 1;
}

static int intersect(const line_t *a, const line_t *b, float *x, float *y)
{
  float det = a->dx * b->dy - a->dy * b->dx;
  if (fabsf(det) < 0.25f) return 0;            /* edges nearly parallel */
  float t = ((b->px - a->px) * b->dy - (b->py - a->py) * b->dx) / det;
  *x = a->px + t * a->dx;
  *y = a->py + t * a->dy;
  return 1;
}

static int fit_quad(const at_config_t *cfg, int n, float q[4][2])
{
  float cx = 0, cy = 0;
  for (int i = 0; i < n; i++) { cx += pts_x[i]; cy += pts_y[i]; }
  cx /= n;
  cy /= n;

  /* A = farthest from the centre, C = farthest from A */
  int iA = 0, iC = 0;
  float best = -1;
  for (int i = 0; i < n; i++) {
    float d = (pts_x[i] - cx) * (pts_x[i] - cx) + (pts_y[i] - cy) * (pts_y[i] - cy);
    if (d > best) { best = d; iA = i; }
  }
  best = -1;
  for (int i = 0; i < n; i++) {
    float d = (float)(pts_x[i] - pts_x[iA]) * (pts_x[i] - pts_x[iA]) +
              (float)(pts_y[i] - pts_y[iA]) * (pts_y[i] - pts_y[iA]);
    if (d > best) { best = d; iC = i; }
  }
  float diag = sqrtf(best);
  if (diag < cfg->min_side) AT_FAIL("quad1");

  /* B, D = farthest from the diagonal AC on either side */
  float ax = pts_x[iA], ay = pts_y[iA];
  float ex = (pts_x[iC] - ax) / diag, ey = (pts_y[iC] - ay) / diag;
  int iB = -1, iD = -1;
  float dB = 0, dD = 0;
  for (int i = 0; i < n; i++) {
    float s = ex * (pts_y[i] - ay) - ey * (pts_x[i] - ax);
    if (s > dB) { dB = s; iB = i; }
    if (s < dD) { dD = s; iD = i; }
  }
  if (iB < 0 || iD < 0 || dB < 0.25f * diag || -dD < 0.25f * diag) AT_FAIL("quad2");

  /* corner indices in outline order */
  int idx[4] = {iA, iB, iC, iD};
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 4; j++)
      if (idx[j] < idx[i]) { int t = idx[i]; idx[i] = idx[j]; idx[j] = t; }

  line_t L[4];
  for (int s = 0; s < 4; s++) {
    int a = idx[s], b = idx[(s + 1) & 3];
    int len = (b - a + n) % n;
    if (len < cfg->min_side / 2) AT_FAIL("quad3");
    int skip = len / 8 + 1;                    /* ignore the rounded corners */
    int count = len - 2 * skip;
    if (count < 3) AT_FAIL("quad4");
    float dev;
    fit_line(n, (a + skip) % n, count, cx, cy, &L[s], &dev);
    if (dev > 1.5f + 0.05f * len) AT_FAIL("quad5");    /* edge is not straight */
  }

  for (int c = 0; c < 4; c++) {
    if (!intersect(&L[(c + 3) & 3], &L[c], &q[c][0], &q[c][1])) AT_FAIL("quad6");
    float ddx = q[c][0] - pts_x[idx[c]], ddy = q[c][1] - pts_y[idx[c]];
    if (ddx * ddx + ddy * ddy > (0.15f * diag + 3) * (0.15f * diag + 3)) AT_FAIL("quad7");
  }

  /* make the order clockwise on screen (positive shoelace sum with y down) */
  float area = 0;
  for (int c = 0; c < 4; c++)
    area += q[c][0] * q[(c + 1) & 3][1] - q[(c + 1) & 3][0] * q[c][1];
  if (area < 0) {
    float t0 = q[1][0], t1 = q[1][1];
    q[1][0] = q[3][0]; q[1][1] = q[3][1];
    q[3][0] = t0;      q[3][1] = t1;
    area = -area;
  }
  if (0.5f * area < (float)cfg->min_side * cfg->min_side) AT_FAIL("quad8");

  /* convex, and no side too short */
  for (int c = 0; c < 4; c++) {
    float x0 = q[c][0], y0 = q[c][1];
    float x1 = q[(c + 1) & 3][0], y1 = q[(c + 1) & 3][1];
    float x2 = q[(c + 2) & 3][0], y2 = q[(c + 2) & 3][1];
    if ((x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) <= 0) AT_FAIL("quad9");
    if ((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) < 0.5f * cfg->min_side * cfg->min_side) AT_FAIL("quad10");
  }
  return 1;
}

/* --------------------------------------------------------------- homography */

/* H maps tag cell coordinates (u, v), black square = (0,0)..(8,8), to the image */
static int homography(const float q[4][2], double H[8])
{
  static const double U[4] = {0, 8, 8, 0}, V[4] = {0, 0, 8, 8};
  double A[8][9];
  for (int i = 0; i < 4; i++) {
    double u = U[i], v = V[i], x = q[i][0], y = q[i][1];
    double r0[9] = {u, v, 1, 0, 0, 0, -u * x, -v * x, x};
    double r1[9] = {0, 0, 0, u, v, 1, -u * y, -v * y, y};
    memcpy(A[2 * i], r0, sizeof r0);
    memcpy(A[2 * i + 1], r1, sizeof r1);
  }
  for (int c = 0; c < 8; c++) {                /* Gauss-Jordan, partial pivoting */
    int p = c;
    for (int r = c + 1; r < 8; r++)
      if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
    if (fabs(A[p][c]) < 1e-9) return 0;
    if (p != c)
      for (int k = 0; k < 9; k++) { double t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; }
    for (int r = 0; r < 8; r++) {
      if (r == c) continue;
      double f = A[r][c] / A[c][c];
      for (int k = c; k < 9; k++) A[r][k] -= f * A[c][k];
    }
  }
  for (int i = 0; i < 8; i++) H[i] = A[i][8] / A[i][i];
  return 1;
}

static void project(const double H[8], float u, float v, float *x, float *y)
{
  double z = H[6] * u + H[7] * v + 1.0;
  *x = (float)((H[0] * u + H[1] * v + H[2]) / z);
  *y = (float)((H[3] * u + H[4] * v + H[5]) / z);
}

/* ----------------------------------------------------------------- decoding */

static int bilinear(const uint8_t *g, int w, int h, float x, float y, int *out)
{
  if (x < 0 || y < 0 || x >= w - 1 || y >= h - 1) return 0;
  int ix = (int)x, iy = (int)y;
  float fx = x - ix, fy = y - iy;
  const uint8_t *p = g + iy * w + ix;
  float v = p[0] * (1 - fx) * (1 - fy) + p[1] * fx * (1 - fy) +
            p[w] * (1 - fx) * fy + p[w + 1] * fx * fy;
  *out = (int)(v + 0.5f);
  return 1;
}

/* mean grey of the cell whose centre is (u, v) in cell coordinates */
static int cell(const double H[8], const uint8_t *g, int w, int h, float u, float v, int *out)
{
  static const float OFS[5][2] = {{0, 0}, {-0.2f, -0.2f}, {0.2f, -0.2f}, {0.2f, 0.2f}, {-0.2f, 0.2f}};
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    float x, y;
    int s;
    project(H, u + OFS[i][0], v + OFS[i][1], &x, &y);
    if (!bilinear(g, w, h, x, y, &s)) return 0;
    sum += s;
  }
  *out = sum / 5;
  return 1;
}

static uint64_t rot90(uint64_t code)                  /* rotate the 6x6 grid clockwise */
{
  uint64_t out = 0;
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++) {
      int src = (5 - c) * 6 + r;
      out = (out << 1) | ((code >> (35 - src)) & 1u);
    }
  return out;
}

static int popcount64(uint64_t x)
{
  int n = 0;
  while (x) { x &= x - 1; n++; }
  return n;
}

static int decode(const at_config_t *cfg, const uint8_t *g, int w, int h,
                  const float q[4][2], at_detection_t *d)
{
  double H[8];
  if (!homography(q, H)) AT_FAIL("decode1");

  /* white ring around the black square -> white level */
  int wsum = 0, wn = 0, v;
  for (int i = -1; i <= 8; i++) {
    float t = i + 0.5f;
    if (cell(H, g, w, h, t, -0.5f, &v)) { wsum += v; wn++; }
    if (cell(H, g, w, h, t,  8.5f, &v)) { wsum += v; wn++; }
    if (i >= 0 && i <= 7) {
      if (cell(H, g, w, h, -0.5f, t, &v)) { wsum += v; wn++; }
      if (cell(H, g, w, h,  8.5f, t, &v)) { wsum += v; wn++; }
    }
  }
  if (wn < 20) AT_FAIL("decode2");

  /* black border cells + data cells */
  int border[28], nb = 0, bsum = 0, data[36];
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++) {
      if (!cell(H, g, w, h, c + 0.5f, r + 0.5f, &v)) AT_FAIL("decode3");
      if (r == 0 || r == 7 || c == 0 || c == 7) { border[nb++] = v; bsum += v; }
      else data[(r - 1) * 6 + (c - 1)] = v;
    }

  int white = wsum / wn, black = bsum / nb;
  if (white - black < cfg->min_contrast) AT_FAIL("decode4");
  int thr = (white + black) / 2;

  int bad = 0;
  for (int i = 0; i < nb; i++) if (border[i] > thr) bad++;
  if (bad > 3) AT_FAIL("decode5");

  uint64_t code = 0;
  for (int i = 0; i < 36; i++) code = (code << 1) | (data[i] > thr ? 1u : 0u);

  int best_id = -1, best_hd = 99, best_k = 0;
  for (int k = 0; k < 4; k++) {
    for (uint32_t id = 0; id < at_tag36h11_count; id++) {
      int hd = popcount64(code ^ at_tag36h11_codes[id]);
      if (hd < best_hd) { best_hd = hd; best_id = (int)id; best_k = k; }
    }
    code = rot90(code);
  }
  if (best_hd > cfg->max_hamming) AT_FAIL("decode6");

  /* rot90^k(observed) == canonical  ->  the tag is turned (4-k)*90 deg clockwise,
     and its top-left corner is at q[(4-k) & 3] */
  int m = (4 - best_k) & 3;
  d->id = best_id;
  d->hamming = best_hd;
  d->rotation = m * 90;
  for (int c = 0; c < 4; c++) {
    d->corners[c][0] = q[(m + c) & 3][0];
    d->corners[c][1] = q[(m + c) & 3][1];
  }
  project(H, 4.0f, 4.0f, &d->cx, &d->cy);
  float side = 0;
  for (int c = 0; c < 4; c++) {
    float ddx = q[(c + 1) & 3][0] - q[c][0], ddy = q[(c + 1) & 3][1] - q[c][1];
    side += sqrtf(ddx * ddx + ddy * ddy);
  }
  d->side = side / 4;
  return 1;
}

/* --------------------------------------------------------------------- main */

static int add_detection(at_detection_t *out, int n, int max_out, const at_detection_t *d)
{
  for (int i = 0; i < n; i++) {                /* same tag found twice: keep the bigger quad */
    float dx = out[i].cx - d->cx, dy = out[i].cy - d->cy;
    if (out[i].id == d->id && dx * dx + dy * dy < 0.25f * d->side * d->side) {
      if (d->side > out[i].side) out[i] = *d;
      return n;
    }
  }
  if (n < max_out) out[n++] = *d;
  return n;
}

int at_detect(const at_config_t *cfg, const uint8_t *gray, int w, int h,
              at_detection_t *out, int max_out)
{
  if (w > AT_MAX_W || h > AT_MAX_H || w % TILE || h % TILE) return 0;
  threshold(cfg, gray, w, h);

  int ndet = 0, ncand = 0;
  for (int y = 1; y < h - 1; y++)
    for (int x = 1; x < w - 1; x++) {
      /* start of an unvisited outline: dark pixel with a non-dark west neighbour */
      if (bin[y * w + x] != PX_DARK || (bin[y * w + x - 1] & 3) == PX_DARK) continue;
      int n = trace(w, x, y);
      if (n < (cfg->min_side * 5) / 2 || n > MAX_PTS) continue;   /* noise specks are cheap: not counted */
      if (++ncand > MAX_CANDIDATES) return ndet;

      float q[4][2];
      at_detection_t d;
      if (fit_quad(cfg, n, q) && decode(cfg, gray, w, h, q, &d))
        ndet = add_detection(out, ndet, max_out, &d);
    }
  return ndet;
}
