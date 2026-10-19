# WS2812B LED Strip Commands

Cloud commands for the WS2812B LED strip driven from the WFI32-IoT 2.0 board.
Send them from the IoTConnect device command page, the same way as `led-red on`.
The command is the whole text string; arguments are separated by spaces.

Implementation: `firmware/src/ws2812_spi.c` / `ws2812_spi.h`, dispatched from
`app_iotconnect.c`.

## Quick reference

| Command | What it does |
|---|---|
| `strip fill <r> <g> <b>` | Set all pixels to one colour (each value 0–255) |
| `strip pixel <i> <r> <g> <b>` | Set one pixel; `i` = 0–59 |
| `strip brightness <0-255>` | Global brightness, applied to everything (default 255) |
| `strip off` | Stop any pattern and turn all pixels off |
| `strip run <pattern> [ms] [r g b]` | Start an animated pattern (see below) |
| `strip stop` | Freeze the running pattern on its current frame |
| `strip cycle [ms]` | Same as `strip run cycle [ms]` |

- `fill`, `pixel` and `off` stop a running pattern.
- `brightness` does **not** stop a pattern; the pattern continues at the new level.
- Only one pattern runs at a time; starting another replaces it.

## Patterns

`strip run <pattern> [ms] [r g b]`

- `ms` — time per animation step, 20–60000. Omit for the pattern default.
- `r g b` — pattern colour, 0–255 each. Omit for the pattern default.
- You can give `ms` only, `r g b` only, both (`ms r g b`), or neither.

| Pattern | Effect | Default step | Default colour |
|---|---|---|---|
| `cycle` | Whole strip red → green → blue | 1000 ms | — (fixed R/G/B) |
| `chase` | A single pixel runs along the strip | 40 ms | red |
| `comet` | Bright head with a 10-pixel fading tail | 30 ms | sky blue |
| `scanner` | "Knight Rider" — bounces end to end with a short tail | 25 ms | red |
| `rainbow` | Moving rainbow across all pixels | 30 ms | — (colour ignored) |
| `wipe` | Fills pixel by pixel, then empties | 30 ms | green |
| `theater` | Every 3rd pixel lit, marching | 120 ms | warm white |
| `breathe` | Whole strip fades in and out (100 steps per breath) | 20 ms | purple |
| `twinkle` | Random sparkles that fade | 60 ms | white |

## Examples

```
strip brightness 20            # start dim for a first test
strip fill 255 0 0             # all red
strip fill 0 0 255             # all blue
strip pixel 0 255 255 255      # pixel 0 white
strip run rainbow              # moving rainbow, default speed
strip run comet 20 255 0 0     # fast red comet
strip run scanner 40           # slower scanner, default red
strip run breathe 0 255 0      # green breathe, default speed (r g b only)
strip run chase 60 255 255 0   # yellow chase, 60 ms per step
strip brightness 80            # brighter, pattern keeps running
strip stop                     # freeze current frame
strip off                      # everything off
```

## Responses

Each command is acknowledged to IoTConnect, and the board's application console
prints `LED STRIP: <command> (ok)` or `(failed)`.

A command fails when:
- an argument is missing, extra or out of range (e.g. `strip fill 300 0 0`,
  `strip pixel 60 1 2 3`, `strip run chase 10`);
- the pattern name is unknown;
- SPI2 is busy with another client, or the board is out of heap memory.

## Notes

- **Pixel count:** 60 pixels are driven (`WS2812_NUM_PIXELS` in `ws2812_spi.h`).
  LEDs beyond 60 on a longer strip keep whatever colour they last had.
- **Wiring:** data on mikroBUS socket J402 MOSI (RK4 / SDO2) through ~330 Ω to
  strip DIN; common ground with the strip's own 5 V supply. A 5 V level shifter
  (74AHCT125 / 74HCT-type) makes the data line reliable if you see flicker.
- **Fast patterns and Wi-Fi:** each frame masks interrupts for about 1.8 ms. At
  the 20 ms minimum step that is ~9% of the time. If the cloud connection gets
  unstable while a fast pattern runs, use a slower step (40 ms or more).
- **Colour order:** values are given as R G B; the driver sends them in the
  WS2812B's G-R-B wire order. If red and green appear swapped on your strip,
  the strip uses a different order.
