/*
 * st7789.c - ST7789 SPI LCD backend for pico-infonesPlus (PicoGB hardware).
 * See st7789.h for the design overview. Init sequence, MADCTL (0x60), INVOFF
 * AND the per-transaction CS framing are ported verbatim from the proven
 * pico1-gb-320 driver. This panel REQUIRES CS to be pulsed low/high around
 * every command, data and pixel-burst transaction (holding CS low permanently
 * leaves the panel blank/white).
 */
#include "st7789.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

/* ST7789 command set */
#define ST7789_SWRESET 0x01
#define ST7789_SLPOUT  0x11
#define ST7789_NORON   0x13
#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_MADCTL  0x36
#define ST7789_COLMOD  0x3A
#define ST7789_FRCTRL2 0xC6   /* normal-mode frame rate control (RTNA in bits[4:0]) */
#define ST7789_PORCTRL 0xB2   /* porch control (vertical back/front porch line counts) */

/* Slope-match (with 12-bit colour): raise RTNA so each line scans SLOWER (~67.8us at 0x10,
 * matching our ~67us 12-bit write), and trim the porch so the frame still lands at ~60.0Hz
 * (240 visible + ~6 porch lines). 0x0F~58.5 / 0x0E~60.5 / 0x10 slower-line. The pacer
 * (settings.syncSpeedAdj) does the fine vertical-drift match; re-tune it after changing these. */
#ifndef ST7789_FRCTRL2_VAL
#define ST7789_FRCTRL2_VAL 0x10   /* RTNA=16, ~67.8us/line, parks ~60Hz. Baseline. (Swept 0x0F/0x11/0x12 =
                                   * 62/58/56fps: the tear stayed 45deg at ALL of them, so the panel line-scan
                                   * rate is NOT a lever for the tear angle — the angle is invariant to it.) */
#endif
#ifndef ST7789_PORCH_BP
#define ST7789_PORCH_BP    0x0A   /* back porch lines: MODERATE trim from default 0x0C=12 -> 10. ~+1.2% refresh,
                                   * nudges this panel's native 59.3Hz toward 60.0Hz so the pacer parks at true 60fps.
                                   * (0x03 was too aggressive a trim and risks a black panel like the 320 init did.) */
#endif
#ifndef ST7789_PORCH_FP
#define ST7789_PORCH_FP    0x0A   /* front porch lines: same moderate 12->10 trim. Raise toward 0x0C if the panel
                                   * blacks; lower toward 0x08 if still under 60Hz after parking. */
#endif

/* DMA pacing (smooth-write experiment): instead of bursting a line's words at full SPI
 * speed (~52us) then idling, pace the DMA with a DMA-timer so the line streams EVENLY
 * over the panel's per-line scan (~67.5us), matching the read-beam = no diagonal. DREQ
 * word rate = clk_sys * X/Y. clk_sys=292MHz, OUT_WORDS=219, target ~67.5us/line =>
 * 3.244 MHz => 1/90. Larger Y = slower delivery (longer line). 0 = unpaced (off). */
#ifndef ST7789_DMA_PACE_X
#define ST7789_DMA_PACE_X  1u
#endif
#ifndef ST7789_DMA_PACE_Y
#define ST7789_DMA_PACE_Y  0u    /* 0 = UNTHROTTLED (full SPI, ~52us/line). Needed so the per-ROW RASET of the
                                  * 1-row scatter (~14us/line) still fits the 60fps budget. Parked-seam baseline
                                  * is Y=90 (see the 60fps_parkedseam build); restore that if reverting scatter. */
#endif

static int                s_dma_chan = -1;
static int                s_dma_timer = -1;
static dma_channel_config s_dma_cfg;

/* Two ping-pong scanline buffers, full panel width so the emulator can render
 * at the +32 offset; borders stay black (cleared once in st7789_init). */
static uint16_t s_linebuf[2][ST7789_WIDTH];
static int      s_linebuf_idx;

/* ---- pin/bus primitives (match GB driver naming/behaviour) ------------- */

static inline void set_cs(bool s)  { gpio_put(ST7789_PIN_CS, s); }
static inline void set_dc(bool s)  { gpio_put(ST7789_PIN_DC, s); }   /* RS */
static inline void set_rst(bool s) { gpio_put(ST7789_PIN_RST, s); }
static inline void set_bl(bool s)  { gpio_put(ST7789_PIN_BL, s); }

static void spi_w8(const uint8_t *bytes, size_t len)
{
    spi_set_format(ST7789_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_write_blocking(ST7789_SPI_PORT, bytes, len);
}

static void spi_w16(const uint16_t *hw, size_t len)
{
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_write16_blocking(ST7789_SPI_PORT, hw, len);
}

/* Each of these pulses CS low for the transaction then raises it again. */
static void write_cmd(uint8_t cmd)
{
    set_dc(0);
    set_cs(0);
    spi_w8(&cmd, 1);
    set_cs(1);
}

static void write_data8(uint8_t d)
{
    set_dc(1);
    set_cs(0);
    spi_w8(&d, 1);
    set_cs(1);
}

static void write_data8_buf(const uint8_t *buf, size_t len)
{
    set_dc(1);
    set_cs(0);
    spi_w8(buf, len);
    set_cs(1);
}

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t buf[4];
    write_cmd(ST7789_CASET);
    buf[0] = (uint8_t)(x0 >> 8); buf[1] = (uint8_t)x0;
    buf[2] = (uint8_t)(x1 >> 8); buf[3] = (uint8_t)x1;
    write_data8_buf(buf, 4);

    write_cmd(ST7789_RASET);
    buf[0] = (uint8_t)(y0 >> 8); buf[1] = (uint8_t)y0;
    buf[2] = (uint8_t)(y1 >> 8); buf[3] = (uint8_t)y1;
    write_data8_buf(buf, 4);
}

/* Set ONLY the row window (RASET). The 1-row scatter writer re-points the row per line;
 * CASET (columns) is set once per frame since it never changes. */
static void set_raset(uint16_t y0, uint16_t y1)
{
    uint8_t buf[4];
    write_cmd(ST7789_RASET);
    buf[0] = (uint8_t)(y0 >> 8); buf[1] = (uint8_t)y0;
    buf[2] = (uint8_t)(y1 >> 8); buf[3] = (uint8_t)y1;
    write_data8_buf(buf, 4);
}

/* Set ONLY the column window (CASET), once per game frame. */
static void set_caset(uint16_t x0, uint16_t x1)
{
    uint8_t buf[4];
    write_cmd(ST7789_CASET);
    buf[0] = (uint8_t)(x0 >> 8); buf[1] = (uint8_t)x0;
    buf[2] = (uint8_t)(x1 >> 8); buf[3] = (uint8_t)x1;
    write_data8_buf(buf, 4);
}

/* ---- public API -------------------------------------------------------- */

static void st7789_scale_init(void);   /* builds the horizontal interpolation tables */

void st7789_init(void)
{
    /* (clk_peri is configured in main() before initAll, so the SD card on spi1
     * and this display both run at the final peripheral clock.) */

    /* SPI bus + DMA */
    spi_init(ST7789_SPI_PORT, ST7789_SPI_BAUD);
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(ST7789_PIN_CLK, GPIO_FUNC_SPI);
    gpio_set_function(ST7789_PIN_SDA, GPIO_FUNC_SPI);
    gpio_set_slew_rate(ST7789_PIN_CLK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ST7789_PIN_SDA, GPIO_SLEW_RATE_FAST);

    s_dma_chan = dma_claim_unused_channel(true);
    s_dma_cfg  = dma_channel_get_default_config(s_dma_chan);
    channel_config_set_transfer_data_size(&s_dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&s_dma_cfg, true);
    channel_config_set_write_increment(&s_dma_cfg, false);
#if ST7789_DMA_PACE_Y
    /* Timer-paced: the DMA writes one word per DMA-timer tick (slower than the SPI can
     * drain, so the FIFO never fills) -> the line streams evenly = smooth write. */
    s_dma_timer = dma_claim_unused_timer(true);
    dma_timer_set_fraction(s_dma_timer, ST7789_DMA_PACE_X, ST7789_DMA_PACE_Y);
    channel_config_set_dreq(&s_dma_cfg, dma_get_timer_dreq(s_dma_timer));
#else
    channel_config_set_dreq(&s_dma_cfg, spi_get_dreq(ST7789_SPI_PORT, true));
#endif

    /* Control GPIOs (SIO outputs) */
    gpio_init(ST7789_PIN_CS);  gpio_set_dir(ST7789_PIN_CS,  GPIO_OUT);
    gpio_init(ST7789_PIN_DC);  gpio_set_dir(ST7789_PIN_DC,  GPIO_OUT);
    gpio_init(ST7789_PIN_RST); gpio_set_dir(ST7789_PIN_RST, GPIO_OUT);
    gpio_init(ST7789_PIN_BL);  gpio_set_dir(ST7789_PIN_BL,  GPIO_OUT);

    /* Reset + init sequence (verbatim from pico1-gb-320). */
    set_rst(1);
    set_cs(1);
    set_dc(0);
    sleep_ms(1);

    set_bl(0);                  /* backlight off during init */

    set_rst(0); sleep_ms(10);
    set_rst(1); sleep_ms(120);

    write_cmd(ST7789_SWRESET);  sleep_ms(150);
    write_cmd(ST7789_SLPOUT);   sleep_ms(50);

    write_cmd(ST7789_COLMOD);   write_data8(0x55);   sleep_ms(10);   /* 16-bit; game switches to 12-bit */
    write_cmd(ST7789_MADCTL);   write_data8(0x60);                   /* 90deg CW (verify on the 240 panel) */
    write_cmd(ST7789_INVON);    sleep_ms(10);   /* 240x240 panel: inverted vs the 320 panel (was INVOFF) */
    /* Speed the panel toward 60Hz by trimming the porches MODERATELY (0x0A vs the panel default
     * 0x0C). The 320 init's AGGRESSIVE 0x03 porches blacked this panel; 0x0A is a gentle nudge
     * (~+1.5% => ~60.2Hz) so the parked seam lands near 60fps instead of the native ~59.3Hz.
     * If it BLACKS: raise back toward 0x0C. If still under 60Hz: trim further toward 0x08. */
    {
        const uint8_t porctrl[5] = { ST7789_PORCH_BP, ST7789_PORCH_FP, 0x00, 0x33, 0x33 };
        write_cmd(ST7789_PORCTRL);  write_data8_buf(porctrl, 5);
    }
    write_cmd(ST7789_NORON);    sleep_ms(10);

    set_window(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
    write_cmd(ST7789_DISPON);   sleep_ms(50);

    st7789_fill(0x0000);        /* clear panel (incl. side bars) */
    set_bl(1);                  /* backlight on */

    st7789_scale_init();        /* build the horizontal interpolation tables */
}

/* Switch the panel pixel format. The MENU draws 16-bit RGB565 (st7789_send_line); the
 * GAME streams 12-bit RGB444 (packed, for the SPI bandwidth). core0 calls this with the
 * panel idle (core1 not streaming) at game entry/exit. */
void st7789_set_color_mode(bool twelve_bit)
{
    st7789_wait_idle();
    write_cmd(ST7789_COLMOD);
    write_data8(twelve_bit ? 0x53u : 0x55u);
}


void st7789_fill(uint16_t color)
{
    st7789_wait_idle();
    set_window(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
    write_cmd(ST7789_RAMWR);
    set_dc(1);
    set_cs(0);
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    for (uint32_t i = 0; i < (uint32_t)ST7789_WIDTH * ST7789_HEIGHT; i++)
        spi_write16_blocking(ST7789_SPI_PORT, &color, 1);
    set_cs(1);
}

uint16_t *st7789_next_linebuf(void)
{
    uint16_t *b = s_linebuf[s_linebuf_idx];
    s_linebuf_idx ^= 1;
    return b;
}

void st7789_send_line(int row, uint16_t *buf)
{
    /* Finish the previous line (also raises CS to close that burst). */
    st7789_wait_idle();

    /* Convert the line from RGB555 to the panel's RGB565 in place. Reading only
     * bits 0..14 also strips InfoNES's 0x8000 backdrop flag (bit 15) — needed by
     * the sprite-priority logic but not a real colour bit. */
    for (unsigned i = 0; i < ST7789_WIDTH; i++)
    {
        uint16_t v = buf[i];
        buf[i] = (uint16_t)(((v & 0x7C00) << 1) | ((v & 0x03E0) << 1) | (v & 0x001F));
    }

    /* Full panel width: the caller passes a 320-wide line. The menu fills all
     * 320; the NES game fills the 256 centre (X_OFFSET) and leaves the side
     * columns black. One unified path for both. */
    set_window(0, (uint16_t)row, ST7789_WIDTH - 1u, (uint16_t)row);
    write_cmd(ST7789_RAMWR);

    set_dc(1);
    set_cs(0);                  /* assert CS for the pixel burst (raised in wait_idle) */
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    dma_channel_configure(s_dma_chan, &s_dma_cfg,
                          &spi_get_hw(ST7789_SPI_PORT)->dr,
                          buf,
                          ST7789_WIDTH,
                          true);
}

/* Horizontal scaler — true linear interpolation (the horizontal pass of bilinear).
 * This runs on CORE1, hidden under the per-line SPI DMA wait: while the previous
 * line streams to the panel (~70 us of otherwise-idle CPU per line), core1 scales
 * the next line. So the 256->292 (8:7 pixel-aspect) stretch costs core0 nothing and
 * the picture stays at 60 fps. For each output column we precompute the left source
 * index and a 0..32 blend weight (frac*32); st7789_scale_init() builds the table.
 * Columns that would read past the last source pixel get weight 0 (exact copy), so
 * src[NES_WIDTH] is never *meaningfully* used. */
static uint16_t s_lerp_i[ST7789_OUT_WIDTH];   /* left source index per output column */
static uint8_t  s_lerp_w[ST7789_OUT_WIDTH];   /* blend weight 0..32 (frac * 32) */

/* ST7789_OUT_WORDS (packed 12-bit word count per line) is defined in st7789.h. */

static void st7789_scale_init(void)
{
    for (unsigned d = 0; d < ST7789_OUT_WIDTH; d++)
    {
        /* source position in fixed point (5 fractional bits): d * NES_WIDTH / OUT_WIDTH */
        uint32_t p = (uint32_t)d * (ST7789_NES_WIDTH << 5) / ST7789_OUT_WIDTH;
        unsigned i = p >> 5;
        unsigned w = p & 31u;                    /* fraction in 0..31 */
        if (i + 1u >= ST7789_NES_WIDTH)          /* last column: clamp, exact copy */
        {
            i = ST7789_NES_WIDTH - 1u;
            w = 0u;
        }
        s_lerp_i[d] = (uint16_t)i;
        s_lerp_w[d] = (uint8_t)w;
    }
}

/* RGB555 (InfoNES; bit 15 is the backdrop flag) -> panel RGB565. The masks drop bit 15. */
static inline uint16_t st7789_rgb555to565(uint16_t v)
{
    return (uint16_t)(((v & 0x7C00u) << 1) | ((v & 0x03E0u) << 1) | (v & 0x001Fu));
}

/* index -> RGB565 LUT. The ring stores 1-byte palette INDICES (half the RAM of RGB555 => twice
 * the scatter window); core1 expands index->color through this. Built once by st7789_set_palette()
 * from the real (saturated) RGB555 palette, before NesPalette is switched to identity. */
static uint16_t s_palette565[64];
void st7789_set_palette(const uint16_t *pal, int n)
{
    for (int i = 0; i < 64; i++)
        s_palette565[i] = st7789_rgb555to565((i < n) ? pal[i] : 0u);
}

/* Linear blend of two RGB565 pixels with weight w (0..32) using the packed
 * "spread R/B low, G high" trick so all three channels blend in one 32-bit op. */
static inline uint16_t st7789_blend565(uint16_t a, uint16_t b, uint32_t w)
{
    uint32_t a2 = (a & 0xF81Fu) | ((uint32_t)(a & 0x07E0u) << 16);
    uint32_t b2 = (b & 0xF81Fu) | ((uint32_t)(b & 0x07E0u) << 16);
    uint32_t c2 = a2 + (((b2 - a2) * w) >> 5);
    return (uint16_t)((c2 & 0xF81Fu) | ((c2 >> 16) & 0x07E0u));
}

/* Light horizontal sharpen (1-D unsharp mask) to counter the bilinear softening:
 * out = c + (2c - left - right) >> SHIFT, per RGB565 channel, clamped. Bigger SHIFT =
 * gentler. 1 = off (compiled out). Runs on core1, hidden under the per-line DMA wait. */
#define ST7789_SHARP_SHIFT 1   /* DISABLED: the per-channel sharpen is too heavy for the
                                * M0+ per-line budget (crashed core1 to 32fps -> choppy
                                * audio). >1 enables it; needs a much cheaper kernel first. */
#if ST7789_SHARP_SHIFT > 1
static inline uint16_t st7789_sharpen565(uint16_t l, uint16_t c, uint16_t r)
{
    int rl = (l >> 11) & 0x1F, gl = (l >> 5) & 0x3F, bl = l & 0x1F;
    int rc = (c >> 11) & 0x1F, gc = (c >> 5) & 0x3F, bc = c & 0x1F;
    int rr = (r >> 11) & 0x1F, gr = (r >> 5) & 0x3F, br = r & 0x1F;
    int R = rc + ((2 * rc - rl - rr) >> ST7789_SHARP_SHIFT);
    int G = gc + ((2 * gc - gl - gr) >> ST7789_SHARP_SHIFT);
    int B = bc + ((2 * bc - bl - br) >> ST7789_SHARP_SHIFT);
    if (R < 0) R = 0; else if (R > 31) R = 31;
    if (G < 0) G = 0; else if (G > 63) G = 63;
    if (B < 0) B = 0; else if (B > 31) B = 31;
    return (uint16_t)((R << 11) | (G << 5) | B);
}
#endif

/* Pack 4 RGB565 pixels into 3 words of ST7789 12-bit (RGB444), big-endian byte order so
 * a 16-bit MSB-first SPI DMA emits the byte stream in order. 565 -> 4-bit channels = top
 * bits (R[15:12] G[10:7] B[4:1]). 2 pixels = 3 bytes, so 4 pixels = 6 bytes = 3 words. */
static inline void st7789_pack4(uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t *out)
{
    unsigned r0=(p0>>12)&0xF, g0=(p0>>7)&0xF, b0=(p0>>1)&0xF;
    unsigned r1=(p1>>12)&0xF, g1=(p1>>7)&0xF, b1=(p1>>1)&0xF;
    unsigned r2=(p2>>12)&0xF, g2=(p2>>7)&0xF, b2=(p2>>1)&0xF;
    unsigned r3=(p3>>12)&0xF, g3=(p3>>7)&0xF, b3=(p3>>1)&0xF;
    out[0] = (uint16_t)((((r0<<4)|g0) << 8) | ((b0<<4)|r1));   /* y0 : y1 */
    out[1] = (uint16_t)((((g1<<4)|b1) << 8) | ((r2<<4)|g2));   /* y2 : y3 */
    out[2] = (uint16_t)((((b2<<4)|r3) << 8) | ((g3<<4)|b3));   /* y4 : y5 */
}

/* As st7789_pack4 but every 4-bit channel halved (scanline darken). The halving folds
 * into the extraction shift (one bit further: >>13/>>8/>>2, mask 0x7), so it's the SAME
 * cost as the bright pack — no extra per-pixel work, unlike halving in the scaler loop
 * (which spilled registers on the M0+ and cost ~2.6ms/frame). */
static inline void st7789_pack4_dark(uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t *out)
{
    unsigned r0=(p0>>13)&0x7, g0=(p0>>8)&0x7, b0=(p0>>2)&0x7;
    unsigned r1=(p1>>13)&0x7, g1=(p1>>8)&0x7, b1=(p1>>2)&0x7;
    unsigned r2=(p2>>13)&0x7, g2=(p2>>8)&0x7, b2=(p2>>2)&0x7;
    unsigned r3=(p3>>13)&0x7, g3=(p3>>8)&0x7, b3=(p3>>2)&0x7;
    out[0] = (uint16_t)((((r0<<4)|g0) << 8) | ((b0<<4)|r1));
    out[1] = (uint16_t)((((g1<<4)|b1) << 8) | ((r2<<4)|g2));
    out[2] = (uint16_t)((((b2<<4)|r3) << 8) | ((g3<<4)|b3));
}

/* ---- optional light 2-tap VERTICAL blend (FIT/8:7 letterbox only) — DISABLED, doesn't fit ----
 * The 240->196 vertical downscale drops ~1 in 6.4 source lines (nearest) -> mild shimmer on
 * vertical scroll. This mixes ST7789_VBLEND_W/32 of the PREVIOUS output row into each row.
 * ★ MEASURED 2026-06-29: it does NOT fit 60fps on this panel — c1_disp 11.5ms -> 22.1ms,
 * 46fps, c0block 7.8ms. The 240 weighted st7789_blend565 per line costs ~54us, OVER the
 * ~43us/line DMA ceiling, so it's not hidden (same wall horizontal bilinear hits, ~46fps).
 * Conclusion: NO per-pixel full-line blend (either axis) fits 60fps on the M0+ @292MHz here.
 * Left in behind this flag (=0) as the documented result; flip to 1 only on a faster panel/MCU. */
#ifndef ST7789_VBLEND
#define ST7789_VBLEND   0
#endif
#ifndef ST7789_VBLEND_W
#define ST7789_VBLEND_W 8u   /* weight toward the previous row, out of 32 (8 = 25% prev / 75% current) */
#endif
#if ST7789_VBLEND
static uint16_t s_vprev[ST7789_OUT_WIDTH];   /* previous FIT row's NEAREST output pixels (RGB565) */
static bool     s_vprev_valid = false;
/* core1 calls this at each frame start so the first row never blends with the previous frame. */
static inline void st7789_vblend_reset(void) { s_vprev_valid = false; }
#endif

void __not_in_flash_func(st7789_scale_game_line)(uint16_t *src, uint16_t *dst, bool darken, bool aspect_1_1)
{
    /* Expand the 1-byte palette indices to RGB565 once, up front (this replaces the old
     * in-place RGB555->565 pass — same per-pixel cost, a LUT load instead of a bit-shuffle).
     * +1 of padding so the bilinear src[i+1] read at the final column stays in bounds.
     * static (not on the stack): core1 is the only caller, and it keeps core1's 2KB stack lean. */
    static uint16_t s[ST7789_NES_WIDTH + 1];
    for (unsigned i = 0; i < ST7789_NES_WIDTH; i++)
        s[i] = s_palette565[src[i] & 0x3Fu];
    s[ST7789_NES_WIDTH] = s[ST7789_NES_WIDTH - 1u];

    if (aspect_1_1)
    {
        /* 1:1 CENTRE-CROP: 256 NES px don't fit the 240 screen, so show the middle OUT_WIDTH of
         * them (lose ~8px each side) at native 1:1 — fills the screen, no bars. */
        const unsigned crop = (ST7789_NES_WIDTH - ST7789_OUT_WIDTH) / 2u;  /* = 8 */
        if (darken)
            for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
            {
                st7789_pack4_dark(s[d+0u+crop], s[d+1u+crop], s[d+2u+crop], s[d+3u+crop], dst);
                dst += 3;
            }
        else
            for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
            {
                st7789_pack4(s[d+0u+crop], s[d+1u+crop], s[d+2u+crop], s[d+3u+crop], dst);
                dst += 3;
            }
        return;
    }

    /* FIT downscale (256->240, ~6%): NEAREST horizontal (s_lerp_i[d]); a gather-bilinear here
     * measured 45fps on this panel. The optional light VERTICAL blend (above) mixes the previous
     * output row to soften the 240->196 line-drop. */
#if ST7789_VBLEND
    if (s_vprev_valid)
    {
        for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
        {
            uint16_t c0 = s[s_lerp_i[d+0]], c1 = s[s_lerp_i[d+1]],
                     c2 = s[s_lerp_i[d+2]], c3 = s[s_lerp_i[d+3]];
            uint16_t b0 = st7789_blend565(c0, s_vprev[d+0], ST7789_VBLEND_W);
            uint16_t b1 = st7789_blend565(c1, s_vprev[d+1], ST7789_VBLEND_W);
            uint16_t b2 = st7789_blend565(c2, s_vprev[d+2], ST7789_VBLEND_W);
            uint16_t b3 = st7789_blend565(c3, s_vprev[d+3], ST7789_VBLEND_W);
            s_vprev[d+0] = c0; s_vprev[d+1] = c1; s_vprev[d+2] = c2; s_vprev[d+3] = c3;
            if (darken) st7789_pack4_dark(b0, b1, b2, b3, dst);
            else        st7789_pack4(b0, b1, b2, b3, dst);
            dst += 3;
        }
        return;
    }
    /* first row of the frame: nothing to blend with yet — emit nearest and seed s_vprev. */
    for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
    {
        uint16_t c0 = s[s_lerp_i[d+0]], c1 = s[s_lerp_i[d+1]],
                 c2 = s[s_lerp_i[d+2]], c3 = s[s_lerp_i[d+3]];
        s_vprev[d+0] = c0; s_vprev[d+1] = c1; s_vprev[d+2] = c2; s_vprev[d+3] = c3;
        if (darken) st7789_pack4_dark(c0, c1, c2, c3, dst);
        else        st7789_pack4(c0, c1, c2, c3, dst);
        dst += 3;
    }
    s_vprev_valid = true;
#else
    if (darken)
        for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
        {
            st7789_pack4_dark(s[s_lerp_i[d+0]], s[s_lerp_i[d+1]], s[s_lerp_i[d+2]], s[s_lerp_i[d+3]], dst);
            dst += 3;
        }
    else
        for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
        {
            st7789_pack4(s[s_lerp_i[d+0]], s[s_lerp_i[d+1]], s[s_lerp_i[d+2]], s[s_lerp_i[d+3]], dst);
            dst += 3;
        }
#endif
}

void st7789_wait_idle(void)
{
    if (s_dma_chan >= 0)
        dma_channel_wait_for_finish_blocking(s_dma_chan);
    while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    set_cs(1);                  /* close any open pixel-burst CS frame */
}

/* ---- core1 display offload (single-producer / single-consumer ring) -----
 * core0 (the emulator) renders each RAW scanline into a ring slot and commits it:
 * normal games write 256 RGB555 pixels (slot[0..255]); NSF writes its full-width UI.
 * core1 consumes a slot, SCALES + converts it into a ping-pong output buffer (s_out),
 * and streams that to the panel by DMA. The scale of line N+1 runs while line N's DMA
 * is in flight, so it hides under the ~70 us/line the CPU would otherwise spend just
 * waiting on the SPI (see st7789_scale_game_line). The window/RAMWR burst is opened
 * once per frame and lines stream into auto-incrementing GRAM. Only used for the GAME;
 * the menu draws directly on core0 (core1 idles when the ring is empty).            */
/* SPSC line ring. core0 (InfoNES / the NSF UI) renders each scanline DIRECTLY into a 16-bit slot —
 * via the identity NesPalette so the values are palette INDICES (0x8000 backdrop flag preserved for
 * sprite priority); core1's scaler masks &0x3F and expands index->RGB565 via s_palette565. There is
 * NO scratch+copy step: that per-line scratch->slot copy cost core0 ~0.8ms/frame, which pushed it
 * over the 60fps budget on this 240 unit (the 1-byte-index trick only existed to shrink the
 * scatter's deep ring, which this build doesn't use). 320 wide holds the full NSF UI. Small ring:
 * per-line streaming, core1 easily keeps up, so a handful of slots suffice. */
#define ST7789_RING_N 32
static uint16_t          s_ring[ST7789_RING_N][320]; /* raw source lines (16-bit palette indices) */
static volatile uint16_t s_ring_row[ST7789_RING_N];
static volatile uint32_t s_ring_wr; /* core0 produce counter */
static volatile uint32_t s_ring_rd; /* core1 consume/free counter */

/* core1's scaled-output ping-pong: one is DMA'd to the panel while core1 scales the
 * next line into the other. OUT_WIDTH wide (the post-scale panel line). */
static uint16_t s_out[2][ST7789_OUT_WORDS];   /* packed 12-bit (RGB444) panel lines */

/* One all-black packed (RGB444) panel line, DMA'd repeatedly to paint the FIT-mode 8:7
 * letterbox bars. Constant (zero = black), so a bar row costs core1 zero per-frame work. */
static uint16_t s_black_row[ST7789_OUT_WORDS]; /* zero-initialised => black */

/* ---- 1-row scatter (anti-tearing, fine-grain) ------------------------------------------
 * core1 holds back a GROUP of rows and re-emits them ONE ROW AT A TIME in a bit-reversed
 * order. Each row goes to its CORRECT GRAM row (image unchanged) — only the WRITE ORDER is
 * scrambled, so the moving new/old boundary breaks into 1px-tall noise instead of the 4px
 * chunks of the earlier band version. The panel refresh sweeps perpendicular to our writes
 * (MADCTL transpose), so this is a SHALLOW dither ribbon along the diagonal, not a dissolve
 * (full dissolve needs full-frame depth => single-buffer stall => ~39fps). BAND_H stays as a
 * knob; =1 here means one RASET per row (needs the wide-open DMA to hold 60fps). Game rows
 * are 4..235 (232 = 7 full 32-row groups + an 8-row tail). */
#define ST7789_SC_BAND_H   4u                              /* rows per band (1 RASET each). 4 on the 240 panel:
                                                            * its controller is SLOW at per-line window commands
                                                            * (~57us/RASET vs ~16us on the 320), so 1-row scatter
                                                            * dropped to 44fps. 4-row bands = 1/4 the RASETs. */
#define ST7789_SC_GROUP_B  16u                             /* bands per group (power of 2); GROUP_H stays 64 rows */
#define ST7789_SC_GROUP_H  (ST7789_SC_GROUP_B*ST7789_SC_BAND_H)  /* 64-row scatter window */
#define ST7789_SC_FIRST    4u                              /* first game row InfoNES renders */
#define ST7789_SC_LASTP1   236u                            /* one past the last (renders 4..235) */
/* Bit-reversal scatter order, built at init (st7789_scatter_init) for any power-of-two GROUP_B:
 * successive rows land as far apart as possible, so the transition points spread across the widest
 * column span the group depth allows. Deeper group => the group emits over more time => the panel
 * sweeps more columns during it => the flip-points spread wider horizontally. */
static uint8_t s_band_perm[ST7789_SC_GROUP_B];

/* NSF mode: the slot already holds a full-width UI laid out at panel columns, so core1
 * converts (no horizontal scale). Set by core0 via st7789_set_nsf_mode(). */
static volatile bool s_nsf_mode = false;
void st7789_set_nsf_mode(bool on) { s_nsf_mode = on; }

/* Scanline mode: when on, core1 darkens every odd output row (CRT look). Set by core0
 * from the screen mode at game start / on the SELECT+UP/DOWN toggle. */
static volatile bool s_scanlines = false;
void st7789_set_scanlines(bool on) { s_scanlines = on; }

/* Aspect mode: false = 8:7 (256 bilinear-stretched to OUT_WIDTH), true = 1:1 (256 NES
 * pixels centred in OUT_WIDTH with black bars, no scale). Same window/DMA either way —
 * only the scaler's output content changes, so switching needs no border re-clear. */
static volatile bool s_aspect_1_1 = false;
void st7789_set_aspect(bool one_to_one) { s_aspect_1_1 = one_to_one; }

/* Retune the per-line DMA pace at runtime (live tear-angle knob). Only meaningful when the
 * timer was claimed at init (ST7789_DMA_PACE_Y != 0). Smaller y = faster per-line write. */
void st7789_set_dma_pace(uint16_t y)
{
    /* NOTE: the runtime retune is NOT honored on this RP2040 (the register updates but the live DMA
     * rate doesn't follow; confirmed via calib). Effective pace is the compile-time ST7789_DMA_PACE_Y.
     * Kept for the (working) game-entry application path / future use. */
    if (s_dma_timer >= 0 && y > 0)
        dma_timer_set_fraction(s_dma_timer, ST7789_DMA_PACE_X, y);
}

/* True while core1 has a frame burst open / a line DMA in flight. core1 is the SOLE
 * owner of the SPI+DMA during a game; st7789_ring_flush() waits for this to clear
 * (core1 drains + raises CS when asked) before core0 may touch the panel. */
static volatile bool s_dma_active = false;

/* Set by st7789_ring_flush() to ask core1 to close the open frame burst (raise CS) the
 * next time the ring is empty, so core0 can take the panel. WITHOUT this request core1
 * keeps the burst open across a brief mid-frame ring-empty (CS stays low) and resumes
 * streaming into the same RAMWR window — otherwise closing CS mid-frame would deselect
 * the panel and silently drop every line until the next frame re-opened the window
 * (seen as "only the top rows update" when the fast 1:1 scaler outruns core0). */
static volatile bool s_flush_req = false;

uint16_t *st7789_ring_acquire(void)
{
    /* Block until a slot is free (ring not full). */
#if ST7789_USB_DEVICE
    if ((uint32_t)(s_ring_wr - s_ring_rd) >= ST7789_RING_N)
    {
        uint64_t t = time_us_64();
        while ((uint32_t)(s_ring_wr - s_ring_rd) >= ST7789_RING_N)
            tight_loop_contents();
        st7789_acquire_block_us += (uint32_t)(time_us_64() - t);
    }
#else
    while ((uint32_t)(s_ring_wr - s_ring_rd) >= ST7789_RING_N)
        tight_loop_contents();
#endif
    /* core0 renders 16-bit (index + 0x8000 backdrop flag) DIRECTLY into the slot — no copy. */
    return s_ring[s_ring_wr % ST7789_RING_N];
}

void st7789_ring_commit(int row)
{
    /* The slot was rendered into directly (no scratch->slot copy); just publish it. */
    s_ring_row[s_ring_wr % ST7789_RING_N] = (uint16_t)row;
    __dmb();                    /* publish the pixel writes before the index */
    s_ring_wr = s_ring_wr + 1;
}

void st7789_ring_flush(void)
{
    /* core1 owns the SPI+DMA during a game. Ask it to close the burst (s_flush_req),
     * then wait for it to consume every slot and drain+raise CS. After this returns the
     * bus is idle and core0 may draw the panel directly. */
    s_flush_req = true;
    while (s_ring_rd != s_ring_wr)
        tight_loop_contents();
    while (s_dma_active)
        tight_loop_contents();
    s_flush_req = false;
}

#if ST7789_USB_DEVICE
/* DIAG: microseconds for one gap-free contiguous DMA of ST7789_CALIB_WORDS words.
 * Isolates raw SPI throughput (no per-line setup, no producer lockstep): compare to
 * ST7789_CALIB_WORDS * (16 / SPI_BAUD). If it matches, the per-frame overhead is
 * per-line setup gaps (ping-pong helps); if it's already higher, the SPI itself is
 * the limit (ping-pong won't help). Call once at startup, before core1 launches. */
#define ST7789_CALIB_WORDS (ST7789_OUT_WIDTH * 32u)
volatile uint32_t st7789_calib_us = 0;
volatile uint32_t st7789_calib_clkperi = 0; /* clk_peri Hz */
volatile uint32_t st7789_calib_baud = 0;    /* SPI bit clock Hz (from the PL022 divisors) */

void st7789_calibrate_spi(void)
{
    st7789_calib_clkperi = clock_get_hz(clk_peri);
    {
        uint32_t cpsr = spi_get_hw(ST7789_SPI_PORT)->cpsr & 0xffu;
        uint32_t scr  = (spi_get_hw(ST7789_SPI_PORT)->cr0 >> 8) & 0xffu;
        st7789_calib_baud = cpsr ? (st7789_calib_clkperi / (cpsr * (1u + scr))) : 0;
    }

    /* Panel-SAFE: keep CS de-asserted so the panel ignores the burst entirely. The
     * SPI peripheral still clocks the data out at the same rate (CS is just a GPIO we
     * toggle), so the throughput measurement is identical — but we issue NO window /
     * RAMWR command and leave the panel's GRAM and command state untouched. The data
     * source (s_ring) is irrelevant since nothing receives it. */
    set_cs(1);                                  /* panel deselected */
    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    uint64_t t0 = time_us_64();
    dma_channel_configure(s_dma_chan, &s_dma_cfg, &spi_get_hw(ST7789_SPI_PORT)->dr,
                          s_ring, ST7789_CALIB_WORDS, true);
    dma_channel_wait_for_finish_blocking(s_dma_chan);
    while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    uint64_t t1 = time_us_64();

    st7789_calib_us = (uint32_t)(t1 - t0);
}
#endif

#if ST7789_USB_DEVICE
/* DIAG (serial-diagnostics build only). st7789_last_frame_us = first-line-to-first-
 * line period of the last frame (≈16667 when paced). st7789_core1_idle_us = a free-
 * running counter of the microseconds core1 spent IDLE waiting for core0. core0
 * derives the true display-active time as (1e6 - idle/sec). Measuring idle (one
 * timer read per idle period ~ once/frame) avoids per-scanline time_us_64 overhead.
 * Compiled out of the shipping (host) build entirely. */
volatile uint32_t st7789_last_frame_us = 0;
volatile uint32_t st7789_core1_idle_us = 0;
/* Time core0 spent BLOCKED in st7789_ring_acquire (ring full => core1 can't keep
 * up => core1/display is the bottleneck). Near-zero => core0/emulation is. */
volatile uint32_t st7789_acquire_block_us = 0;
#endif

/* Re-fire the line DMA (constant ST7789_OUT_WORDS count) at a new source buffer, into the
 * already-open RAMWR window. The trans count must be rewritten each time (it reads 0 after a
 * completed burst). Used by the FIT letterbox path to stream successive game/black-bar rows. */
static inline void st7789_dma_row(const uint16_t *buf)
{
    if (s_dma_active)
        dma_channel_wait_for_finish_blocking(s_dma_chan);
    dma_channel_set_trans_count(s_dma_chan, ST7789_OUT_WORDS, false);
    dma_channel_set_read_addr(s_dma_chan, buf, true);
    s_dma_active = true;
}

static void __attribute__((noreturn)) st7789_core1_run(void)
{
    int      prev_G = -1;      /* first row of the previously-issued group (frame delimiter) */
    int      cur    = 0;       /* ping-pong index into s_out[] */
#if ST7789_USB_DEVICE
    uint64_t frame_start = 0;
#endif
    for (;;)
    {
        /* ---- ring empty: idle, honoring a flush request ----------------------------- */
        if (s_ring_rd == s_ring_wr)
        {
            /* Do NOT close an open burst for a brief mid-frame gap — keep CS low so the
             * next line resumes the same window. Close (raise CS) ONLY when core0 asked
             * for the panel via st7789_ring_flush() (s_flush_req). */
#if ST7789_USB_DEVICE
            uint64_t idle_start = time_us_64();
#endif
            while (s_ring_rd == s_ring_wr)
            {
                if (s_dma_active && s_flush_req)
                {
                    dma_channel_wait_for_finish_blocking(s_dma_chan);
                    while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
                        tight_loop_contents();
                    set_cs(1);
                    s_dma_active = false;
                }
                tight_loop_contents();
            }
#if ST7789_USB_DEVICE
            st7789_core1_idle_us += (uint32_t)(time_us_64() - idle_start);
#endif
            continue;
        }

        uint32_t base = s_ring_rd;
        int      G    = (int)s_ring_row[base % ST7789_RING_N];

        /* ---- NSF: plain sequential single-line streaming (no scatter) ---------------- */
        if (s_nsf_mode)
        {
            bool      new_frame = (prev_G < 0) || (G < prev_G);
            prev_G = G;
            uint16_t *dst = s_out[cur];
            uint16_t *sp  = &s_ring[base % ST7789_RING_N][0];
            uint16_t *o   = dst;
            for (unsigned d = 0; d < ST7789_OUT_WIDTH; d += 4)
            {
                st7789_pack4(s_palette565[sp[d+0] & 0x3Fu], s_palette565[sp[d+1] & 0x3Fu],
                             s_palette565[sp[d+2] & 0x3Fu], s_palette565[sp[d+3] & 0x3Fu], o);
                o += 3;
            }
            __dmb();
            s_ring_rd = base + 1;
            if (s_dma_active)
                dma_channel_wait_for_finish_blocking(s_dma_chan);
            if (new_frame)
            {
                while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
                    tight_loop_contents();
                set_cs(1);
                set_window(ST7789_OUT_X_OFFSET, (uint16_t)G,
                           ST7789_OUT_X_OFFSET + ST7789_OUT_WIDTH - 1u, ST7789_HEIGHT - 1u);
                write_cmd(ST7789_RAMWR);
                set_dc(1);
                set_cs(0);
                spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
                dma_channel_configure(s_dma_chan, &s_dma_cfg, &spi_get_hw(ST7789_SPI_PORT)->dr,
                                      dst, ST7789_OUT_WORDS, true);
            }
            else
            {
                dma_channel_set_trans_count(s_dma_chan, ST7789_OUT_WORDS, false);
                dma_channel_set_read_addr(s_dma_chan, dst, true);
            }
            s_dma_active = true;
            cur = 1 - cur;
            continue;
        }

        /* ---- GAME: plain sequential per-line streaming (NO scatter) ------------------ */
        /* This 240 panel is too slow per-line for the scatter to hold 60fps, and the clean
         * single tear "looks fine" on the small screen. So stream rows in order: open ONE
         * RAMWR window at frame start and let every line auto-increment into GRAM. */
        {
            bool      new_frame = (prev_G < 0) || (G < prev_G);
            prev_G = G;
#if ST7789_USB_DEVICE
            if (new_frame)
            {
                uint64_t now = time_us_64();
                if (frame_start != 0)
                    st7789_last_frame_us = (uint32_t)(now - frame_start);
                frame_start = now;
            }
#endif
#if ST7789_VBLEND
            if (new_frame && !s_aspect_1_1)
                st7789_vblend_reset();   /* don't blend row 0 with the previous frame's last row */
#endif
            uint16_t *dst = s_out[cur];
            st7789_scale_game_line(&s_ring[base % ST7789_RING_N][0], dst,
                                   s_scanlines && (G & 1), s_aspect_1_1);
            __dmb();
            s_ring_rd = base + 1;

            /* ---- FIT / 8:7 LETTERBOX: 22 black + 196 game + 22 black rows in ONE full-height
             * window (same 240-row SPI cost as full-screen, so no fps hit). core0 has already
             * vertically downscaled 240->196 by committing only the selected lines, tagged
             * output rows ST7789_LB_BAR..ST7789_LB_BAR+ST7789_LB_ACTIVE-1; we add the bars. */
            if (!s_aspect_1_1)
            {
                if (new_frame)
                {
                    if (s_dma_active)
                        dma_channel_wait_for_finish_blocking(s_dma_chan);
                    while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
                        tight_loop_contents();
                    set_cs(1);
                    set_window(ST7789_OUT_X_OFFSET, 0,
                               ST7789_OUT_X_OFFSET + ST7789_OUT_WIDTH - 1u, ST7789_HEIGHT - 1u);
                    write_cmd(ST7789_RAMWR);
                    set_dc(1);
                    set_cs(0);
                    spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
                    /* top bar: ST7789_LB_BAR black rows (panel rows 0..LB_BAR-1) */
                    dma_channel_configure(s_dma_chan, &s_dma_cfg, &spi_get_hw(ST7789_SPI_PORT)->dr,
                                          s_black_row, ST7789_OUT_WORDS, true);
                    s_dma_active = true;
                    for (unsigned b = 1; b < ST7789_LB_BAR; b++)
                        st7789_dma_row(s_black_row);
                    st7789_dma_row(dst);            /* first active row (panel row LB_BAR) */
                }
                else
                {
                    st7789_dma_row(dst);            /* active rows LB_BAR+1 .. LB_BAR+LB_ACTIVE-1 */
                }
                /* bottom bar, once, right after the last active row */
                if ((unsigned)G == ST7789_LB_BAR + ST7789_LB_ACTIVE - 1u)
                    for (unsigned b = 0; b < ST7789_LB_BAR; b++)
                        st7789_dma_row(s_black_row);
                cur = 1 - cur;
                continue;
            }

            if (s_dma_active)
                dma_channel_wait_for_finish_blocking(s_dma_chan);
            if (new_frame)
            {
                while (spi_get_hw(ST7789_SPI_PORT)->sr & SPI_SSPSR_BSY_BITS)
                    tight_loop_contents();
                set_cs(1);
                set_window(ST7789_OUT_X_OFFSET, (uint16_t)G,
                           ST7789_OUT_X_OFFSET + ST7789_OUT_WIDTH - 1u, ST7789_HEIGHT - 1u);
                write_cmd(ST7789_RAMWR);
                set_dc(1);
                set_cs(0);
                spi_set_format(ST7789_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
                dma_channel_configure(s_dma_chan, &s_dma_cfg, &spi_get_hw(ST7789_SPI_PORT)->dr,
                                      dst, ST7789_OUT_WORDS, true);
            }
            else
            {
                dma_channel_set_trans_count(s_dma_chan, ST7789_OUT_WORDS, false);
                dma_channel_set_read_addr(s_dma_chan, dst, true);
            }
            s_dma_active = true;
            cur = 1 - cur;
        }
    }
}

/* Build the scatter (emit) order for the GROUP_B rows of a group. Done once before core1
 * launches => a FIXED, deterministic order (stable dither texture, no per-frame shimmer).
 *
 * GOLDEN-RATIO fixed-stride low-discrepancy order: perm[i] = (i*STEP) mod GROUP_B, with
 * STEP ~= GROUP_B/phi and coprime to GROUP_B. This is the "best of both" between the two
 * orders that failed:
 *   - bit-reversal: spread evenly but emitted all even rows then all odd rows => a regular
 *     every-other-row brightness COMB.
 *   - random shuffle: broke the periodicity but a random subset CLUMPS => drifting blotches.
 * The phi stride spreads as evenly as bit-reversal (no clumps) yet is maximally aperiodic
 * (no comb, no moire). STEP is odd, so consecutive emits alternate row parity => evens and
 * odds stay interleaved throughout (no even/odd halves). Requires power-of-two GROUP_B. */
static void st7789_scatter_init(void)
{
    unsigned step = (ST7789_SC_GROUP_B * 6180u) / 10000u;   /* ~= GROUP_B / phi */
    if ((step & 1u) == 0u) step += 1u;                      /* odd => coprime to power-of-2 GROUP_B */
    unsigned v = 0u;
    for (unsigned i = 0; i < ST7789_SC_GROUP_B; i++)
    {
        s_band_perm[i] = (uint8_t)v;
        v = (v + step) & (ST7789_SC_GROUP_B - 1u);          /* (i*step) mod GROUP_B, incremental */
    }
}

void st7789_start_display_core1(void)
{
    st7789_scatter_init();   /* fill s_band_perm before core1 reads it */
    multicore_launch_core1(st7789_core1_run);
}
