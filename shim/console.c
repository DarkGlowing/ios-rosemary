/*
 * Minimal text console on the ABL splash framebuffer. With the MMU off
 * every access is Device memory, so stay aligned (-mstrict-align).
 */
#include "console.h"
#include "font.h"

#define SCALE		3
#define CH_W		(8 * SCALE)
#define CH_H		(16 * SCALE)
#define MARGIN		24
#define LABEL_COLS	8

volatile uint32_t *con_fb = (volatile uint32_t *)FB_BASE;
static uint32_t con_bg;
static int row;
static int con_dy;	/* vertical offset in pixels (buffer id test) */
static int con_pitch = FB_PITCH_W;	/* detected line pitch, pixels */

void con_clear(uint32_t bg)
{
	for (int y = 0; y < FB_H; y++) {
		volatile uint32_t *line = con_fb + y * con_pitch;
		for (int x = 0; x < con_pitch; x++)
			line[x] = bg;
	}
	con_bg = bg;
	row = 0;
}

static void put_char(int col, char ch, uint32_t fg)
{
	if (ch < 0x20 || ch > 0x7e)
		ch = '?';
	const unsigned char *g = font8x16[ch - 0x20];
	int x0 = MARGIN + col * CH_W, y0 = con_dy + MARGIN + row * CH_H;
	/* Bounds are relative to the buffer we are drawing into (con_dy is either
	 * 0 or FB_H), not to the whole 2-buffer area. */
	if (x0 + CH_W > FB_W || y0 + CH_H > con_dy + FB_H)
		return;
	for (int gy = 0; gy < 16; gy++)
		for (int gx = 0; gx < 8; gx++) {
			uint32_t c = (g[gy] & (0x80 >> gx)) ? fg : con_bg;
			for (int sy = 0; sy < SCALE; sy++) {
				volatile uint32_t *p = con_fb + (y0 + gy * SCALE + sy) * con_pitch
						       + x0 + gx * SCALE;
				for (int sx = 0; sx < SCALE; sx++)
					p[sx] = c;
			}
		}
}

static int put_str(int col, const char *s, uint32_t fg)
{
	while (*s)
		put_char(col++, *s++, fg);
	return col;
}

void con_marks(uint32_t fg)
{
	const int S = 160;
	int xs[4] = { 0, FB_W - S, 0, FB_W - S };
	int ys[4] = { 0, 0, FB_H - S, FB_H - S };

	for (int k = 0; k < 4; k++)
		for (int y = ys[k]; y < ys[k] + S; y++) {
			volatile uint32_t *p = con_fb + y * con_pitch + xs[k];
			for (int x = 0; x < S; x++)
				p[x] = fg;
		}
}

static void fmt_hex(char *buf, uint64_t v, int digits)
{
	static const char hex[] = "0123456789ABCDEF";
	for (int i = 0; i < digits; i++)
		buf[i] = hex[(v >> (4 * (digits - 1 - i))) & 0xf];
	buf[digits] = 0;
}

void con_bars(void)
{
	static const uint32_t band[8] = {
		0xffff0000U, 0xff00ff00U, 0xff0000ffU, 0xffffff00U,
		0xff00ffffU, 0xffff00ffU, 0xffffffffU, 0xff404040U
	};
	const int half = FB_H / 2;
	const int bw = FB_W / 8;

	for (int y = 0; y < FB_H; y++) {
		volatile uint32_t *line = con_fb + y * con_pitch;
		for (int x = 0; x < con_pitch; x++) {
			int b = x / bw;
			line[x] = y < half ? band[b > 7 ? 7 : b] : 0;
		}
	}
	con_bg = 0;
	row = 0;
}

static void hline(int y, int x0, int x1, uint32_t c)
{
	volatile uint32_t *p = con_fb + y * con_pitch;
	for (int x = x0; x < x1; x++)
		p[x] = c;
}

static void rect(int x0, int y0, int w, int h, uint32_t c)
{
	for (int y = y0; y < y0 + h; y++) {
		volatile uint32_t *p = con_fb + y * con_pitch + x0;
		for (int x = 0; x < w; x++)
			p[x] = c;
	}
}

void con_geom_test(void)
{
	for (int y = 0; y < FB_H; y++)
		hline(y, 0, con_pitch, 0);
	/* border: exactly on the visible edges */
	for (int y = 0; y < 4; y++) {
		hline(y, 0, FB_W, WHITE);
		hline(FB_H - 1 - y, 0, FB_W, WHITE);
	}
	for (int y = 0; y < FB_H; y++) {
		hline(y, 0, 4, WHITE);
		hline(y, FB_W - 4, FB_W, WHITE);
	}
	/* centre cross */
	for (int y = 0; y < FB_H; y++)
		for (int x = FB_W / 2 - 2; x < FB_W / 2 + 2; x++)
			con_fb[y * con_pitch + x] = WHITE;
	for (int y = FB_H / 2 - 2; y < FB_H / 2 + 2; y++)
		hline(y, 0, FB_W, WHITE);
	/* corner squares: 120x120, 8 px inside the border */
	rect(8, 8, 120, 120, RED);
	rect(FB_W - 128, 8, 120, 120, GREEN);
	rect(8, FB_H - 128, 120, 120, BLUE);
	rect(FB_W - 128, FB_H - 128, 120, 120, YELLOW);
	con_bg = 0;
	row = (FB_H / 2) / CH_H + 1;
}

void con_set_dy(int dy)
{
	con_dy = dy;
}

void con_set_row(int r)
{
	row = r;
}

void con_set_bg(uint32_t bg)
{
	con_bg = bg;
}

void con_set_pitch(int px)
{
	if (px > 0)
		con_pitch = px;
}

int con_get_pitch(void)
{
	return con_pitch;
}

/*
 * Detect the line pitch the display controller is actually using.
 *
 * The framebuffer still holds the boot logo left there by LK. For the correct
 * pitch, consecutive scan lines are highly correlated; for a wrong pitch the
 * same memory looks like noise. So: score several candidates by the summed
 * absolute difference of neighbouring rows and keep the best.
 *
 * Returns the pitch in pixels; *flat is set if the area looks like a solid
 * colour (no logo, nothing to correlate), in which case the caller should keep
 * the default.
 */
int con_detect_pitch(int buf, int *flat)
{
	const int base = buf * FB_H;
	int best = con_pitch;
	unsigned long bestscore = ~0UL;

	for (int pass = 0; pass < 2; pass++) {
		int lo = pass ? best - 20 : 960;
		int hi = pass ? best + 20 : 2560;
		int step = pass ? 1 : 16;
		if (lo < 32) lo = 32;
		for (int p = lo; p <= hi; p += step) {
			unsigned long score = 0;
			for (int y = 40; y < 60; y++) {
				for (int x = 0; x < 320; x += 8) {
					uint32_t a = con_fb[(base + y) * p + x];
					uint32_t b = con_fb[(base + y + 1) * p + x];
					int dr = (int)((a >> 16) & 0xff) - (int)((b >> 16) & 0xff);
					int dg = (int)((a >> 8) & 0xff) - (int)((b >> 8) & 0xff);
					int db = (int)(a & 0xff) - (int)(b & 0xff);
					score += (unsigned long)(dr < 0 ? -dr : dr)
					       + (unsigned long)(dg < 0 ? -dg : dg)
					       + (unsigned long)(db < 0 ? -db : db);
				}
			}
			if (score < bestscore) {
				bestscore = score;
				best = p;
			}
		}
	}
	if (flat)
		*flat = bestscore < 40 * 20 * 40;   /* ~uniform rows */
	return best;
}

/* Paint one whole screen buffer (FB_H rows starting at row `buf * FB_H`). */
void con_paint_buf(int buf, uint32_t c)
{
	for (int y = 0; y < FB_H; y++) {
		volatile uint32_t *line = con_fb + (buf * FB_H + y) * con_pitch;
		for (int x = 0; x < con_pitch; x++)
			line[x] = c;
	}
}

/* Geometry test drawn directly into buffer `buf` (0 or 1):
 * 9 white horizontal rulers every 299 rows, a centre vertical line and the
 * four corner squares. Uneven ruler spacing = wrong pitch; rulers that stop
 * early = wrong height. */
void con_geom2(int buf)
{
	const int base = buf * FB_H;

	for (int y = 0; y < FB_H; y++) {
		volatile uint32_t *line = con_fb + (base + y) * con_pitch;
		for (int x = 0; x < con_pitch; x++)
			line[x] = 0;
	}
	for (int k = 0; k < 9; k++) {
		int y = k * 299;
		if (y > FB_H - 4)
			y = FB_H - 4;
		for (int d = 0; d < 4; d++) {
			volatile uint32_t *line = con_fb + (base + y + d) * con_pitch;
			for (int x = 0; x < con_pitch; x++)
				line[x] = WHITE;
		}
	}
	for (int y = 0; y < FB_H; y++) {
		volatile uint32_t *line = con_fb + (base + y) * con_pitch;
		for (int x = FB_W / 2 - 2; x < FB_W / 2 + 2; x++)
			line[x] = WHITE;
	}
	rect(8, base + 8, 120, 120, RED);
	rect(FB_W - 128, base + 8, 120, 120, GREEN);
	rect(8, base + FB_H - 128, 120, 120, BLUE);
	rect(FB_W - 128, base + FB_H - 128, 120, 120, YELLOW);
}

/* Copy buffer 1 back into buffer 0 (frame was drawn in #1). Keeps both
 * buffers identical in case the controller alternates between them. */
/* Render one ASCII label using an arbitrary line pitch (in pixels). Used to
 * find the display controller's real pitch in a single boot: eight labels,
 * eight pitches, the crisp one identifies itself. */
void con_label_pitch(const char *s, int y, int pitch_px, uint32_t fg)
{
	for (int i = 0; s[i]; i++) {
		unsigned char ch = (unsigned char)s[i];
		if (ch < 0x20 || ch > 0x7e)
			ch = '?';
		const unsigned char *g = font8x16[ch - 0x20];
		int x0 = 24 + i * 24;
		if (x0 + 24 > FB_W)
			break;
		for (int gy = 0; gy < 16; gy++)
			for (int gx = 0; gx < 8; gx++) {
				uint32_t c = (g[gy] & (0x80 >> gx)) ? fg : 0;
				for (int sy = 0; sy < 3; sy++)
					for (int sx = 0; sx < 3; sx++) {
						volatile uint32_t *p = con_fb +
							(y + gy * 3 + sy) * pitch_px + x0 + gx * 3 + sx;
						p[0] = c;
					}
			}
	}
}

void con_mirror_first(void)
{
	volatile uint64_t *d = (volatile uint64_t *)con_fb;
	volatile const uint64_t *s = (volatile const uint64_t *)(con_fb + FB_H * con_pitch);
	for (int i = 0; i < FB_H * con_pitch / 2; i++)
		d[i] = s[i];
}

void con_mirror_second(void)
{
	volatile uint64_t *d = (volatile uint64_t *)(con_fb + FB_H * con_pitch);
	volatile const uint64_t *s = (volatile const uint64_t *)con_fb;
	for (int i = 0; i < FB_H * con_pitch / 2; i++)
		d[i] = s[i];
}

void con_text_lower_half(void)
{
	row = (FB_H / 2) / CH_H + 1;
}

void con_skip(void)
{
	row++;
}

void con_str(const char *label, const char *s)
{
	put_str(0, label, YELLOW);
	put_str(LABEL_COLS, s, WHITE);
	row++;
}

void con_hex(const char *label, uint64_t v)
{
	char buf[17];
	fmt_hex(buf, v, 16);
	con_str(label, buf);
}

void con_hex2(const char *label, const char *a, uint32_t va, const char *b, uint32_t vb)
{
	char buf[9];
	int col = put_str(0, label, YELLOW);
	col = col < LABEL_COLS ? LABEL_COLS : col + 1;
	col = put_str(col, a, YELLOW);
	fmt_hex(buf, va, 8);
	col = put_str(col, buf, WHITE);
	if (b) {
		col = put_str(col + 1, b, YELLOW);
		fmt_hex(buf, vb, 8);
		put_str(col, buf, WHITE);
	}
	row++;
}
