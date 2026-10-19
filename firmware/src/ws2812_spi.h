/*******************************************************************************
  WS2812B LED strip driver for the WFI32-IoT 2.0 board (SPI2 on mikroBUS J402)

  Wiring (all on the mikroBUS socket, J402):
      MOSI  (SDO2, RK4)  --> level shifter (74AHCT125 / 74HCT-type, 5 V) --> strip DIN
      GND                --> strip GND  (also tie to the external 5 V supply GND)
  The strip is powered from its own 5 V supply, never from the board.

  Method: SPI2 is clocked at ~2.4 MHz and every WS2812 bit is sent as three SPI
  bits ("100" = 0, "110" = 1), so one WS2812 bit takes ~1.26 us:
      0 : T0H ~0.42 us / T0L ~0.84 us      1 : T1H ~0.84 us / T1L ~0.42 us
  Colour order on the wire is G, R, B, MSB first.

  Threading: all calls are meant to come from one task (the IoTConnect command
  callback).  Nothing here is re-entrant.
*******************************************************************************/

#ifndef WS2812_SPI_H
#define WS2812_SPI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of pixels driven.  The strip has 300, but the whole frame is sent with
 * interrupts masked (see WS2812_IRQ_PROTECT): ~30 us per pixel, so 60 pixels is
 * ~1.8 ms and 300 pixels would be ~9 ms.  Only the first WS2812_NUM_PIXELS
 * LEDs of the strip are controlled; the rest keep their last colour (power the
 * strip off, or raise this value, to change that). */
#ifndef WS2812_NUM_PIXELS
#define WS2812_NUM_PIXELS       60U
#endif

/* SPI2 peripheral clock feeding the baud generator (PBCLK2).  The Harmony
 * configuration of this project assumes 100 MHz.  If the waveform on a scope
 * is not ~1.26 us per bit, this is the value to fix. */
#ifndef WS2812_SPI_SOURCE_CLOCK_HZ
#define WS2812_SPI_SOURCE_CLOCK_HZ   100000000UL
#endif

#ifndef WS2812_SPI_BIT_RATE_HZ
#define WS2812_SPI_BIT_RATE_HZ       2400000UL
#endif

/* 1: mask all CPU interrupts while a frame is shifted out so that no ISR
 *    (Wi-Fi, FreeRTOS tick) can open a gap in the stream.
 * 0: leave interrupts on (risk of occasional wrong pixels on busy systems). */
#ifndef WS2812_IRQ_PROTECT
#define WS2812_IRQ_PROTECT      1
#endif

/* Allocate (from the heap, once) and clear the driver state.  Called
 * automatically by the other functions.  Returns false if out of memory. */
bool WS2812_Initialize(void);

/* Change the colour of one pixel in the frame buffer (not sent until Show). */
void WS2812_SetPixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);

/* Set every pixel to one colour (frame buffer only). */
void WS2812_Fill(uint8_t r, uint8_t g, uint8_t b);

/* All pixels off (frame buffer only). */
void WS2812_Clear(void);

/* Global brightness, 0..255 (255 = as written).  Applied when sending. */
void WS2812_SetBrightness(uint8_t brightness);
uint8_t WS2812_GetBrightness(void);

/* Send the frame buffer to the strip.  Blocks for the length of the frame.
 * Returns false if SPI2 is in use by something else; nothing is sent then. */
bool WS2812_Show(void);

/* Animated patterns, run by a small FreeRTOS task (created on first use, its
 * stack comes from the heap).  One pattern runs at a time; starting another
 * replaces it.  The pattern colour is used where a pattern has one colour. */
typedef enum
{
    WS2812_FX_CYCLE = 0,    /* whole strip red -> green -> blue           1000 ms */
    WS2812_FX_CHASE,        /* one pixel running along the strip            40 ms */
    WS2812_FX_COMET,        /* head with a fading 10-pixel tail             30 ms */
    WS2812_FX_SCANNER,      /* "Knight Rider", bounces end to end           25 ms */
    WS2812_FX_RAINBOW,      /* moving rainbow (ignores colour)              30 ms */
    WS2812_FX_WIPE,         /* fills pixel by pixel, then empties           30 ms */
    WS2812_FX_THEATER,      /* every 3rd pixel lit, marching               120 ms */
    WS2812_FX_BREATHE,      /* whole strip fades in and out (2 s @ 20 ms)   20 ms */
    WS2812_FX_TWINKLE,      /* random sparkles that fade                    60 ms */
    WS2812_FX_COUNT
} ws2812_fx_t;

/* period_ms = time per frame, 20..60000, or 0 for the pattern's default.
 * rgb = {r, g, b} pattern colour, or NULL for the pattern's default.
 * Returns false on bad arguments or when out of memory. */
bool WS2812_StartEffect(ws2812_fx_t fx, uint32_t period_ms, const uint8_t *rgb);
void WS2812_StopEffect(void);               /* freezes on the current frame */
const char *WS2812_EffectName(ws2812_fx_t fx);

/* Same as WS2812_StartEffect(WS2812_FX_CYCLE, ...) / WS2812_StopEffect(). */
bool WS2812_StartCycle(uint32_t period_ms);
void WS2812_StopCycle(void);

/* Text commands (cloud/console).  Returns true if the text was a strip command:
 *   "strip fill <r> <g> <b>"           all pixels, 0..255 each
 *   "strip pixel <i> <r> <g> <b>"      one pixel, i = 0..WS2812_NUM_PIXELS-1
 *   "strip brightness <0..255>"
 *   "strip off"                        stop any pattern, all pixels off
 *   "strip run <pattern> [ms] [r g b]" start a pattern (names as in ws2812_fx_t:
 *                                      cycle chase comet scanner rainbow wipe
 *                                      theater breathe twinkle)
 *   "strip stop"                       freeze the pattern on its current frame
 *   "strip cycle [ms]"                 same as "strip run cycle [ms]"
 * fill / pixel / off stop a running pattern; brightness applies to it.
 * The strip is updated before returning.  *ok is set to false when the text is
 * a strip command but its arguments are invalid or the SPI is busy. */
bool WS2812_HandleCommand(const char *command, bool *ok);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_SPI_H */
