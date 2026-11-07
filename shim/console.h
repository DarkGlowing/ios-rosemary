#pragma once
#include <stdint.h>

#define FB_BASE		0x7bd20000UL	/* reserved-memory cont_splash_region */
#define FB_W		1080
#define FB_H		2400
/* Measured on the device (rosemary, dsi_panel_k7_38_0c_0a_fhdp_video):
 *   /sys/class/graphics/fb0/virtual_size = 1088,4800
 *   /sys/class/graphics/fb0/stride       = 4352
 *   bits_per_pixel = 32, name = mtkfb
 * So the line pitch is 1088 pixels (1080 visible + 8 padding) and the buffer
 * is double-buffered (2 x 2400 rows). Using 1080*4 shears everything by 8 px
 * per row, which is exactly the "text turned into diagonal streaks" symptom. */
#define FB_PITCH_W	1088			/* padded line width in pixels */
#ifndef FB_STRIDE
#define FB_STRIDE	(FB_PITCH_W * 4)	/* 4352 bytes per line */
#endif
#define FB_PITCH_PX	(FB_STRIDE / 4)

#define RED	0xffff0000U
#define BLUE	0xff0000a0U
#define WHITE	0xffffffffU
#define YELLOW	0xffffff00U
#define GREEN	0xff00ff00U

extern volatile uint32_t *con_fb;

void con_clear(uint32_t bg);
/* Four corner squares: proves base address, stride and pixel format at once.
 * Corner markers (not a full fill) so a wrong stride shows up as a smear
 * instead of a uniform-looking screen. */
void con_marks(uint32_t fg);
/* Calibration frame: 8 vertical colour bars over the top half, black over the
 * bottom half, text below. A wrong stride turns the bars into slanted stripes,
 * which is obvious on screen. */
void con_bars(void);
void con_text_lower_half(void);
/* Duplicate buffer 0 into the second 2400-row buffer of the double-buffered
 * mtkfb, so the frame is visible whichever one is scanned out. */
void con_mirror_second(void);
void con_mirror_first(void);
void con_label_pitch(const char *s, int y, int pitch_px, uint32_t fg);
void con_set_dy(int dy);
void con_set_row(int r);
void con_set_pitch(int px);
void con_set_bg(uint32_t bg);
int con_get_pitch(void);
int con_detect_pitch(int buf, int *flat);
void con_paint_buf(int buf, uint32_t c);
/* Geometry test frame: 4px border on the visible edges, centre cross and four
 * 120x120 corner squares. Easy to describe in words without a photo. */
void con_geom_test(void);
void con_geom2(int buf);
void con_skip(void);
void con_str(const char *label, const char *s);
void con_puts(const char *s);
void con_hex(const char *label, uint64_t v);
/* label, then "a=xxxxxxxx b=xxxxxxxx" style pairs of 32-bit values */
void con_hex2(const char *label, const char *a, uint32_t va, const char *b, uint32_t vb);
