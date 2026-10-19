/*******************************************************************************
  WS2812B LED strip driver, SPI2 (mikroBUS J402 MOSI = RK4 / SDO2).
  See ws2812_spi.h for wiring and method.
*******************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "ws2812_spi.h"

#ifndef WS2812_HOST_TEST
#include "definitions.h"      /* device registers, SPI2 PLIB */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

/* ---- frame geometry ------------------------------------------------------ */

#define WS2812_BITS_PER_PIXEL       (24U * 3U)                       /* 72 */
#define WS2812_DATA_WORDS           ((WS2812_NUM_PIXELS * WS2812_BITS_PER_PIXEL + 31U) / 32U)

/* Line must stay low for >= 280 us after the data (WS2812B latch).  Each word
 * of zeros is 32 / 2.38 MHz = ~13.4 us, so 24 words = ~320 us. */
#define WS2812_RESET_WORDS          24U
#define WS2812_FRAME_WORDS          (WS2812_DATA_WORDS + WS2812_RESET_WORDS)

/* Animated patterns ("strip run", "strip cycle") */
#define WS2812_FX_MIN_MS            20U
#define WS2812_FX_MAX_MS            60000U
#define WS2812_FX_TASK_STACK        256U        /* words */
#define WS2812_FX_TASK_PRIO         (tskIDLE_PRIORITY + 1U)

/* ---- state --------------------------------------------------------------- */

/* RAM is tight in this project, so there is no encoded frame buffer: the SPI
 * words are generated on the fly while the previous ones are shifting out.
 *
 * All driver state lives in one heap block taken on first use; the only static
 * variable is the pointer to it.  Wi-Fi on this project stops connecting when
 * static RAM grows by ~190 bytes (the heap start address moves; the root cause
 * is outside this driver), so keep it that way. */
typedef struct
{
    uint8_t  rgb[WS2812_NUM_PIXELS][3];     /* r, g, b as the user set them */
    uint8_t  brightness;

    /* running pattern -- all fields protected by lock */
    bool        fx_on;
    ws2812_fx_t fx;
    uint32_t    fx_ms;
    uint32_t    fx_step;
    uint8_t     fx_rgb[3];                  /* pattern colour */
    uint32_t    fx_rand;                    /* PRNG state for twinkle */
#ifndef WS2812_HOST_TEST
    SemaphoreHandle_t lock;
    TaskHandle_t      fx_task;
#else
    bool              fx_task;
#endif
} ws2812_ctx_t;

static ws2812_ctx_t *s_ctx = NULL;

/* Name, default step time and default colour of each pattern.  Kept in flash. */
typedef struct
{
    const char *name;
    uint16_t    ms;
    uint8_t     rgb[3];
} ws2812_fx_info_t;

static const ws2812_fx_info_t k_fx[WS2812_FX_COUNT] = {
    [WS2812_FX_CYCLE]   = { "cycle",   1000U, { 255U,   0U,   0U } },
    [WS2812_FX_CHASE]   = { "chase",     40U, { 255U,   0U,   0U } },
    [WS2812_FX_COMET]   = { "comet",     30U, {   0U, 120U, 255U } },
    [WS2812_FX_SCANNER] = { "scanner",   25U, { 255U,   0U,   0U } },
    [WS2812_FX_RAINBOW] = { "rainbow",   30U, { 255U, 255U, 255U } },
    [WS2812_FX_WIPE]    = { "wipe",      30U, {   0U, 255U,   0U } },
    [WS2812_FX_THEATER] = { "theater",  120U, { 255U, 140U,  40U } },
    [WS2812_FX_BREATHE] = { "breathe",   20U, { 160U,   0U, 255U } },
    [WS2812_FX_TWINKLE] = { "twinkle",   60U, { 255U, 255U, 255U } },
};

/* ---- OS glue ------------------------------------------------------------- */

#ifdef WS2812_HOST_TEST
#define ws2812_alloc(n)     malloc(n)
#define ws2812_free(p)      free(p)
static bool ws2812_os_init(ws2812_ctx_t *c) { (void)c; return true; }
static void ws2812_lock(void)   { }
static void ws2812_unlock(void) { }
static bool ws2812_fx_task_start(void) { s_ctx->fx_task = true; return true; }
#else
#define ws2812_alloc(n)     pvPortMalloc(n)
#define ws2812_free(p)      vPortFree(p)

static bool ws2812_os_init(ws2812_ctx_t *c)
{
    c->lock    = xSemaphoreCreateMutex();
    c->fx_task = NULL;
    return c->lock != NULL;
}

static void ws2812_lock(void)
{
    (void)xSemaphoreTake(s_ctx->lock, portMAX_DELAY);
}

static void ws2812_unlock(void)
{
    (void)xSemaphoreGive(s_ctx->lock);
}

static void ws2812_fx_task(void *arg);

/* Called with the lock held. */
static bool ws2812_fx_task_start(void)
{
    if (s_ctx->fx_task != NULL)
    {
        return true;            /* already running; it picks up the new pattern */
    }
    return xTaskCreate(ws2812_fx_task, "ws2812", WS2812_FX_TASK_STACK,
                       NULL, WS2812_FX_TASK_PRIO, &s_ctx->fx_task) == pdPASS;
}
#endif

/* ---- bit stream generator ------------------------------------------------ */

/* Produces the SPI bit stream (MSB first, G R B per pixel) one 32-bit word at a
 * time.  Every WS2812 bit becomes 3 SPI bits: 0 -> 100, 1 -> 110.  After the last
 * pixel it keeps producing zeros, which is the reset / latch time. */
typedef struct
{
    uint32_t pixel;
    uint32_t chan;          /* 0 = G, 1 = R, 2 = B */
    int32_t  bit;           /* 7..0 within the current byte */
    uint8_t  value;         /* current byte, brightness applied */
    uint64_t acc;
    uint32_t nbits;
} ws2812_gen_t;

static const uint8_t k_chan_to_rgb[3] = { 1U, 0U, 2U };   /* G, R, B -> rgb index */

static void gen_load_byte(ws2812_gen_t *g)
{
    uint32_t v = s_ctx->rgb[g->pixel][k_chan_to_rgb[g->chan]];
    g->value = (uint8_t)((v * s_ctx->brightness + 127U) / 255U);
    g->bit   = 7;
}

static void gen_init(ws2812_gen_t *g)
{
    memset(g, 0, sizeof(*g));
    if (WS2812_NUM_PIXELS > 0U)
    {
        gen_load_byte(g);
    }
}

static uint32_t gen_next_word(ws2812_gen_t *g)
{
    while (g->nbits < 32U)
    {
        uint32_t sym = 0U;                          /* 000 after the data */

        if (g->pixel < WS2812_NUM_PIXELS)
        {
            sym = (((g->value >> g->bit) & 1U) != 0U) ? 0x6U : 0x4U;

            if (--g->bit < 0)
            {
                if (++g->chan >= 3U)
                {
                    g->chan = 0U;
                    g->pixel++;
                }
                if (g->pixel < WS2812_NUM_PIXELS)
                {
                    gen_load_byte(g);
                }
            }
        }

        g->acc = (g->acc << 3) | sym;
        g->nbits += 3U;
    }

    uint32_t word = (uint32_t)(g->acc >> (g->nbits - 32U));
    g->nbits -= 32U;
    return word;
}

/* ---- hardware ------------------------------------------------------------ */

#ifdef WS2812_HOST_TEST

/* Host build: the test harness provides these. */
extern bool ws2812_test_spi_busy(void);
extern void ws2812_test_spi_word(uint32_t word);

static bool ws2812_hw_setup(void)
{
    return !ws2812_test_spi_busy();
}

static void ws2812_hw_send_frame(void)
{
    ws2812_gen_t g;
    gen_init(&g);
    for (uint32_t i = 0U; i < WS2812_FRAME_WORDS; i++)
    {
        ws2812_test_spi_word(gen_next_word(&g));
    }
}

#else

static bool ws2812_hw_setup(void)
{
    if (SPI2_IsBusy())
    {
        return false;           /* another client (DRV_SPI) is mid-transfer */
    }

    /* 32-bit words, idle-low clock (so MOSI idles low).  SPI2_TransferSetup
     * keeps the rest of the PLIB configuration (master, enhanced buffer). */
    SPI_TRANSFER_SETUP setup;
    setup.clockFrequency = WS2812_SPI_BIT_RATE_HZ;
    setup.clockPolarity  = SPI_CLOCK_POLARITY_IDLE_LOW;
    setup.clockPhase     = SPI_CLOCK_PHASE_LEADING_EDGE;
    setup.dataBits       = SPI_DATA_BITS_32;

    /* The PLIB transmit/receive interrupts are idle (SPI2_Initialize leaves them
     * disabled and SPI2_WriteRead re-arms them per transfer), and this driver
     * polls, so they must stay off during the frame. */
    return SPI2_TransferSetup(&setup, WS2812_SPI_SOURCE_CLOCK_HZ);
}

static void ws2812_drain_rx(void)
{
    /* Master mode receives while it transmits; keep the RX FIFO empty. */
    while ((SPI2STAT & _SPI2STAT_SPIRBE_MASK) == 0U)
    {
        (void)SPI2BUF;
    }
    SPI2STATCLR = _SPI2STAT_SPIROV_MASK;
}

static void ws2812_hw_send_frame(void)
{
    ws2812_gen_t g;
    gen_init(&g);

#if WS2812_IRQ_PROTECT
    unsigned int status = __builtin_disable_interrupts();
#endif

    ws2812_drain_rx();

    uint32_t sent    = 0U;
    uint32_t pending = gen_next_word(&g);              /* always one word ahead */

    while (sent < WS2812_FRAME_WORDS)
    {
        if ((SPI2STAT & _SPI2STAT_SPITBF_MASK) == 0U)  /* room in the TX FIFO */
        {
            SPI2BUF = pending;
            sent++;
            pending = gen_next_word(&g);
        }
        ws2812_drain_rx();
    }

    /* Wait for the last word to leave the shift register. */
    uint32_t guard = 2000000U;
    while (((SPI2STAT & _SPI2STAT_SRMT_MASK) == 0U) && (guard-- > 0U))
    {
        ws2812_drain_rx();
    }
    ws2812_drain_rx();

#if WS2812_IRQ_PROTECT
    if ((status & 1U) != 0U)
    {
        __builtin_enable_interrupts();
    }
#endif
}

#endif /* WS2812_HOST_TEST */

/* ---- internals (lock held) ---------------------------------------------- */

static void fill_locked(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint32_t i = 0U; i < WS2812_NUM_PIXELS; i++)
    {
        s_ctx->rgb[i][0] = r;
        s_ctx->rgb[i][1] = g;
        s_ctx->rgb[i][2] = b;
    }
}

static bool show_locked(void)
{
    if (!ws2812_hw_setup())
    {
        return false;
    }
    ws2812_hw_send_frame();
    return true;
}

/* ---- patterns ------------------------------------------------------------ */

/* Pixel i = pattern colour scaled by level/255 (level squared for a smoother
 * fade to the eye). */
static void put_level(uint32_t i, uint32_t level)
{
    uint32_t l = (level * level + 254U) / 255U;
    for (uint32_t c = 0U; c < 3U; c++)
    {
        s_ctx->rgb[i][c] = (uint8_t)((s_ctx->fx_rgb[c] * l + 127U) / 255U);
    }
}

/* Colour wheel: 0..255 -> red -> green -> blue -> red. */
static void wheel(uint32_t pos, uint8_t out[3])
{
    pos &= 0xFFU;
    if (pos < 85U)
    {
        out[0] = (uint8_t)(255U - pos * 3U); out[1] = (uint8_t)(pos * 3U); out[2] = 0U;
    }
    else if (pos < 170U)
    {
        pos -= 85U;
        out[0] = 0U; out[1] = (uint8_t)(255U - pos * 3U); out[2] = (uint8_t)(pos * 3U);
    }
    else
    {
        pos -= 170U;
        out[0] = (uint8_t)(pos * 3U); out[1] = 0U; out[2] = (uint8_t)(255U - pos * 3U);
    }
}

static uint32_t next_rand(void)
{
    /* xorshift32 */
    uint32_t x = s_ctx->fx_rand;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    s_ctx->fx_rand = x;
    return x;
}

#define N   WS2812_NUM_PIXELS

/* Draw frame number s of the current pattern into rgb[]. */
static void fx_render_locked(uint32_t s)
{
    switch (s_ctx->fx)
    {
    case WS2812_FX_CYCLE:               /* whole strip red -> green -> blue */
    {
        static const uint8_t k_rgb[3][3] = {
            { 255U, 0U, 0U }, { 0U, 255U, 0U }, { 0U, 0U, 255U }
        };
        const uint8_t *c = k_rgb[s % 3U];
        fill_locked(c[0], c[1], c[2]);
        break;
    }

    case WS2812_FX_CHASE:               /* one pixel running along the strip */
        fill_locked(0U, 0U, 0U);
        put_level(s % N, 255U);
        break;

    case WS2812_FX_COMET:               /* head with a 10-pixel fading tail */
    {
        const uint32_t tail = (N < 10U) ? N : 10U;
        uint32_t head = s % N;
        fill_locked(0U, 0U, 0U);
        for (uint32_t k = 0U; k < tail; k++)
        {
            put_level((head + N - k) % N, 255U * (tail - k) / tail);
        }
        break;
    }

    case WS2812_FX_SCANNER:             /* bounces end to end, short tail */
    {
        const uint32_t tail = 6U;
        uint32_t period = (N > 1U) ? 2U * (N - 1U) : 1U;
        uint32_t pos    = s % period;
        bool     fwd    = pos < N;
        uint32_t head   = fwd ? pos : period - pos;
        fill_locked(0U, 0U, 0U);
        for (uint32_t k = 0U; k < tail; k++)
        {
            int32_t i = fwd ? (int32_t)head - (int32_t)k : (int32_t)head + (int32_t)k;
            if (i >= 0 && i < (int32_t)N)
            {
                put_level((uint32_t)i, 255U * (tail - k) / tail);
            }
        }
        break;
    }

    case WS2812_FX_RAINBOW:             /* full rainbow across the strip, moving */
        for (uint32_t i = 0U; i < N; i++)
        {
            wheel(i * 256U / N + s * 4U, s_ctx->rgb[i]);
        }
        break;

    case WS2812_FX_WIPE:                /* fill pixel by pixel, then empty */
    {
        uint32_t pos = s % (2U * N);
        for (uint32_t i = 0U; i < N; i++)
        {
            bool lit = (pos < N) ? (i <= pos) : (i > pos - N);
            put_level(i, lit ? 255U : 0U);
        }
        break;
    }

    case WS2812_FX_THEATER:             /* every 3rd pixel lit, marching */
        for (uint32_t i = 0U; i < N; i++)
        {
            put_level(i, ((i + s) % 3U == 0U) ? 255U : 0U);
        }
        break;

    case WS2812_FX_BREATHE:             /* whole strip fades in and out */
    {
        uint32_t t     = s % 100U;
        uint32_t level = (t < 50U) ? t * 255U / 49U : (99U - t) * 255U / 49U;
        for (uint32_t i = 0U; i < N; i++)
        {
            put_level(i, level);
        }
        break;
    }

    case WS2812_FX_TWINKLE:             /* random sparkles that fade out */
        if (s == 0U)
        {
            fill_locked(0U, 0U, 0U);
        }
        for (uint32_t i = 0U; i < N; i++)
        {
            for (uint32_t c = 0U; c < 3U; c++)
            {
                s_ctx->rgb[i][c] = (uint8_t)(s_ctx->rgb[i][c] * 3U / 4U);
            }
        }
        put_level(next_rand() % N, 255U);
        break;

    default:
        break;
    }
}

#undef N

/* One animation frame: draw, send, advance. */
static void fx_step_locked(void)
{
    fx_render_locked(s_ctx->fx_step);
    (void)show_locked();
    s_ctx->fx_step++;
}

#ifndef WS2812_HOST_TEST
static void ws2812_fx_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        ws2812_lock();
        if (!s_ctx->fx_on)
        {
            s_ctx->fx_task = NULL;          /* a later "strip run" starts a new task */
            ws2812_unlock();
            vTaskDelete(NULL);
        }
        fx_step_locked();
        uint32_t ms = s_ctx->fx_ms;
        ws2812_unlock();

        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}
#endif

/* ---- public API ---------------------------------------------------------- */

bool WS2812_Initialize(void)
{
    if (s_ctx == NULL)
    {
        ws2812_ctx_t *c = ws2812_alloc(sizeof(*c));
        if (c == NULL)
        {
            return false;
        }
        memset(c, 0, sizeof(*c));
        c->brightness = 255U;
        c->fx_rand    = 0x2545F491U;
        if (!ws2812_os_init(c))
        {
            ws2812_free(c);
            return false;
        }
        s_ctx = c;
    }
    return true;
}

void WS2812_SetPixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (WS2812_Initialize() && (index < WS2812_NUM_PIXELS))
    {
        ws2812_lock();
        s_ctx->rgb[index][0] = r;
        s_ctx->rgb[index][1] = g;
        s_ctx->rgb[index][2] = b;
        ws2812_unlock();
    }
}

void WS2812_Fill(uint8_t r, uint8_t g, uint8_t b)
{
    if (WS2812_Initialize())
    {
        ws2812_lock();
        fill_locked(r, g, b);
        ws2812_unlock();
    }
}

void WS2812_Clear(void)
{
    WS2812_Fill(0U, 0U, 0U);
}

void WS2812_SetBrightness(uint8_t brightness)
{
    if (WS2812_Initialize())
    {
        ws2812_lock();
        s_ctx->brightness = brightness;
        ws2812_unlock();
    }
}

uint8_t WS2812_GetBrightness(void)
{
    return (s_ctx != NULL) ? s_ctx->brightness : 255U;
}

bool WS2812_Show(void)
{
    if (!WS2812_Initialize())
    {
        return false;
    }
    ws2812_lock();
    bool ok = show_locked();
    ws2812_unlock();
    return ok;
}

const char *WS2812_EffectName(ws2812_fx_t fx)
{
    return ((uint32_t)fx < (uint32_t)WS2812_FX_COUNT) ? k_fx[fx].name : NULL;
}

bool WS2812_StartEffect(ws2812_fx_t fx, uint32_t period_ms, const uint8_t *rgb)
{
    if (!WS2812_Initialize() || (uint32_t)fx >= (uint32_t)WS2812_FX_COUNT)
    {
        return false;
    }
    if (period_ms == 0U)
    {
        period_ms = k_fx[fx].ms;
    }
    if (period_ms < WS2812_FX_MIN_MS || period_ms > WS2812_FX_MAX_MS)
    {
        return false;
    }

    ws2812_lock();
    s_ctx->fx      = fx;
    s_ctx->fx_ms   = period_ms;
    s_ctx->fx_step = 0U;                    /* restart the pattern from frame 0 */
    memcpy(s_ctx->fx_rgb, (rgb != NULL) ? rgb : k_fx[fx].rgb, 3U);
    s_ctx->fx_on   = true;
    bool ok = ws2812_fx_task_start();
    if (!ok)
    {
        s_ctx->fx_on = false;
    }
    ws2812_unlock();
    return ok;
}

void WS2812_StopEffect(void)
{
    if (s_ctx != NULL)
    {
        ws2812_lock();
        s_ctx->fx_on = false;               /* the task exits at its next wake-up */
        ws2812_unlock();
    }
}

bool WS2812_StartCycle(uint32_t period_ms)
{
    return WS2812_StartEffect(WS2812_FX_CYCLE, period_ms, NULL);
}

void WS2812_StopCycle(void)
{
    WS2812_StopEffect();
}

#ifdef WS2812_HOST_TEST
/* Test hooks: run one frame as the task would, and peek at the state. */
void ws2812_test_fx_step(void)
{
    if (s_ctx != NULL && s_ctx->fx_on)
    {
        fx_step_locked();
    }
}
bool     ws2812_test_fx_on(void) { return s_ctx != NULL && s_ctx->fx_on; }
uint32_t ws2812_test_fx_ms(void) { return s_ctx != NULL ? s_ctx->fx_ms : 0U; }
int      ws2812_test_fx(void)    { return s_ctx != NULL ? (int)s_ctx->fx : -1; }
#endif

/* ---- text commands ------------------------------------------------------- */

/* Parse the next unsigned decimal number (<= max) and advance *p.  Leading
 * spaces are skipped.  Returns false if there is none or it is out of range. */
static bool next_number(const char **p, uint32_t max, uint32_t *out)
{
    while (**p == ' ')
    {
        (*p)++;
    }
    if (**p < '0' || **p > '9')
    {
        return false;
    }

    char *end;
    unsigned long v = strtoul(*p, &end, 10);
    if (end == *p || v > max)
    {
        return false;
    }
    *p  = end;
    *out = (uint32_t)v;
    return true;
}

static bool at_end(const char *p)
{
    while (*p == ' ')
    {
        p++;
    }
    return *p == '\0';
}

/* Matches keyword kw at *p (after spaces), followed by a space or end of text. */
static bool keyword(const char **p, const char *kw)
{
    const char *q = *p;
    while (*q == ' ')
    {
        q++;
    }
    size_t n = strlen(kw);
    if (strncmp(q, kw, n) == 0 && (q[n] == ' ' || q[n] == '\0'))
    {
        *p = q + n;
        return true;
    }
    return false;
}

/* "[ms] [r g b]" after a pattern name.  ms = 0 means the pattern's default. */
static bool parse_fx_args(const char *p, uint32_t *ms, uint8_t rgb[3], bool *has_rgb)
{
    uint32_t v[4];
    uint32_t n = 0U;

    while (!at_end(p))
    {
        if (n >= 4U || !next_number(&p, WS2812_FX_MAX_MS, &v[n]))
        {
            return false;
        }
        n++;
    }

    *ms      = 0U;
    *has_rgb = false;

    if (n == 1U || n == 4U)
    {
        *ms = v[0];
        if (*ms == 0U)
        {
            return false;
        }
    }
    if (n == 3U || n == 4U)
    {
        const uint32_t *c = &v[n - 3U];
        if (c[0] > 255U || c[1] > 255U || c[2] > 255U)
        {
            return false;
        }
        rgb[0] = (uint8_t)c[0]; rgb[1] = (uint8_t)c[1]; rgb[2] = (uint8_t)c[2];
        *has_rgb = true;
    }
    return n != 2U;
}

bool WS2812_HandleCommand(const char *command, bool *ok)
{
    static const char prefix[] = "strip ";
    bool result = false;

    if (command == NULL || strncmp(command, prefix, sizeof(prefix) - 1U) != 0)
    {
        return false;
    }

    const char *p = command + sizeof(prefix) - 1U;
    uint32_t a, b, c, d;

    if (keyword(&p, "off"))
    {
        if (at_end(p))
        {
            WS2812_StopEffect();
            WS2812_Clear();
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "stop"))
    {
        /* Freeze the pattern on its current frame. */
        if (at_end(p))
        {
            WS2812_StopEffect();
            result = true;
        }
    }
    else if (keyword(&p, "fill"))
    {
        if (next_number(&p, 255U, &a) && next_number(&p, 255U, &b) &&
            next_number(&p, 255U, &c) && at_end(p))
        {
            WS2812_StopEffect();
            WS2812_Fill((uint8_t)a, (uint8_t)b, (uint8_t)c);
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "pixel"))
    {
        if (next_number(&p, WS2812_NUM_PIXELS - 1U, &a) &&
            next_number(&p, 255U, &b) && next_number(&p, 255U, &c) &&
            next_number(&p, 255U, &d) && at_end(p))
        {
            WS2812_StopEffect();
            WS2812_SetPixel(a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "brightness"))
    {
        /* Does not stop a running pattern; it applies to it. */
        if (next_number(&p, 255U, &a) && at_end(p))
        {
            WS2812_SetBrightness((uint8_t)a);
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "cycle"))
    {
        /* Kept for compatibility: same as "strip run cycle [ms]". */
        if (at_end(p))
        {
            result = WS2812_StartEffect(WS2812_FX_CYCLE, 0U, NULL);
        }
        else if (next_number(&p, WS2812_FX_MAX_MS, &a) && at_end(p) && a > 0U)
        {
            result = WS2812_StartEffect(WS2812_FX_CYCLE, a, NULL);
        }
    }
    else if (keyword(&p, "run"))
    {
        for (uint32_t fx = 0U; fx < (uint32_t)WS2812_FX_COUNT; fx++)
        {
            if (keyword(&p, k_fx[fx].name))
            {
                uint32_t ms;
                uint8_t  rgb[3];
                bool     has_rgb;
                if (parse_fx_args(p, &ms, rgb, &has_rgb))
                {
                    result = WS2812_StartEffect((ws2812_fx_t)fx, ms,
                                                has_rgb ? rgb : NULL);
                }
                break;
            }
        }
    }

    if (ok != NULL)
    {
        *ok = result;
    }
    return true;            /* it was a strip command, valid or not */
}
