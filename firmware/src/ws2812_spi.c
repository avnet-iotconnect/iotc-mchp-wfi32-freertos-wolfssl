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

/* Colour cycle ("strip cycle") */
#define WS2812_CYCLE_DEFAULT_MS     1000U
#define WS2812_CYCLE_MIN_MS         50U
#define WS2812_CYCLE_MAX_MS         60000U
#define WS2812_CYCLE_TASK_STACK     256U        /* words */
#define WS2812_CYCLE_TASK_PRIO      (tskIDLE_PRIORITY + 1U)

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

    /* colour cycle -- all fields protected by lock */
    bool     cycle_on;
    uint8_t  cycle_step;                    /* 0 = red, 1 = green, 2 = blue */
    uint32_t cycle_ms;
#ifndef WS2812_HOST_TEST
    SemaphoreHandle_t lock;
    TaskHandle_t      cycle_task;
#else
    bool              cycle_task;
#endif
} ws2812_ctx_t;

static ws2812_ctx_t *s_ctx = NULL;

/* ---- OS glue ------------------------------------------------------------- */

#ifdef WS2812_HOST_TEST
#define ws2812_alloc(n)     malloc(n)
#define ws2812_free(p)      free(p)
static bool ws2812_os_init(ws2812_ctx_t *c) { (void)c; return true; }
static void ws2812_lock(void)   { }
static void ws2812_unlock(void) { }
static bool ws2812_cycle_task_start(void) { s_ctx->cycle_task = true; return true; }
#else
#define ws2812_alloc(n)     pvPortMalloc(n)
#define ws2812_free(p)      vPortFree(p)

static bool ws2812_os_init(ws2812_ctx_t *c)
{
    c->lock       = xSemaphoreCreateMutex();
    c->cycle_task = NULL;
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

static void ws2812_cycle_task(void *arg);

/* Called with the lock held. */
static bool ws2812_cycle_task_start(void)
{
    if (s_ctx->cycle_task != NULL)
    {
        return true;                    /* already running; picks up new period */
    }
    return xTaskCreate(ws2812_cycle_task, "ws2812", WS2812_CYCLE_TASK_STACK,
                       NULL, WS2812_CYCLE_TASK_PRIO, &s_ctx->cycle_task) == pdPASS;
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

/* Paint the current cycle colour and advance to the next one. */
static void cycle_step_locked(void)
{
    static const uint8_t k_colours[3][3] = {
        { 255U, 0U, 0U }, { 0U, 255U, 0U }, { 0U, 0U, 255U }
    };
    const uint8_t *c = k_colours[s_ctx->cycle_step % 3U];

    fill_locked(c[0], c[1], c[2]);
    (void)show_locked();
    s_ctx->cycle_step = (uint8_t)((s_ctx->cycle_step + 1U) % 3U);
}

#ifndef WS2812_HOST_TEST
static void ws2812_cycle_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        ws2812_lock();
        if (!s_ctx->cycle_on)
        {
            s_ctx->cycle_task = NULL;       /* a later "strip cycle" starts a new task */
            ws2812_unlock();
            vTaskDelete(NULL);
        }
        cycle_step_locked();
        uint32_t ms = s_ctx->cycle_ms;
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
        c->cycle_ms   = WS2812_CYCLE_DEFAULT_MS;
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

bool WS2812_StartCycle(uint32_t period_ms)
{
    if (!WS2812_Initialize() ||
        period_ms < WS2812_CYCLE_MIN_MS || period_ms > WS2812_CYCLE_MAX_MS)
    {
        return false;
    }

    ws2812_lock();
    bool was_on        = s_ctx->cycle_on;
    s_ctx->cycle_ms    = period_ms;
    s_ctx->cycle_on    = true;
    if (!was_on)
    {
        s_ctx->cycle_step = 0U;
    }
    bool ok = ws2812_cycle_task_start();
    if (!ok)
    {
        s_ctx->cycle_on = false;
    }
    ws2812_unlock();
    return ok;
}

void WS2812_StopCycle(void)
{
    if (s_ctx != NULL)
    {
        ws2812_lock();
        s_ctx->cycle_on = false;            /* the task exits at its next wake-up */
        ws2812_unlock();
    }
}

#ifdef WS2812_HOST_TEST
/* Test hook: run one cycle step as the task would. */
void ws2812_test_cycle_step(void)
{
    if (s_ctx != NULL && s_ctx->cycle_on)
    {
        cycle_step_locked();
    }
}
bool ws2812_test_cycle_on(void)  { return s_ctx != NULL && s_ctx->cycle_on; }
uint32_t ws2812_test_cycle_ms(void) { return s_ctx != NULL ? s_ctx->cycle_ms : 0U; }
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

/* Matches keyword kw at *p, followed by a space or end of text. */
static bool keyword(const char **p, const char *kw)
{
    size_t n = strlen(kw);
    if (strncmp(*p, kw, n) == 0 && ((*p)[n] == ' ' || (*p)[n] == '\0'))
    {
        *p += n;
        return true;
    }
    return false;
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
    while (*p == ' ')
    {
        p++;
    }

    uint32_t a, b, c, d;

    if (keyword(&p, "off"))
    {
        if (at_end(p))
        {
            WS2812_StopCycle();
            WS2812_Clear();
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "fill"))
    {
        if (next_number(&p, 255U, &a) && next_number(&p, 255U, &b) &&
            next_number(&p, 255U, &c) && at_end(p))
        {
            WS2812_StopCycle();
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
            WS2812_StopCycle();
            WS2812_SetPixel(a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "brightness"))
    {
        /* Does not stop a running cycle; it applies to it. */
        if (next_number(&p, 255U, &a) && at_end(p))
        {
            WS2812_SetBrightness((uint8_t)a);
            result = WS2812_Show();
        }
    }
    else if (keyword(&p, "cycle"))
    {
        if (at_end(p))
        {
            result = WS2812_StartCycle(WS2812_CYCLE_DEFAULT_MS);
        }
        else if (next_number(&p, WS2812_CYCLE_MAX_MS, &a) && at_end(p))
        {
            result = WS2812_StartCycle(a);
        }
    }

    if (ok != NULL)
    {
        *ok = result;
    }
    return true;            /* it was a strip command, valid or not */
}
