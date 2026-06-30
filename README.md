# pico-infonesPlus — Pico-GB ST7789 port (240×240 square)

A port of [**pico-infonesPlus**](https://github.com/fhoedemakers/pico-infonesPlus) (a Raspberry Pi
Pico NES emulator) to run on **Pico-GB handheld hardware** fitted with a **240×240 square ST7789 SPI
LCD**, with I2S audio, an SD card, and GPIO buttons. It flashes onto an existing Pico-GB unit with
**zero wiring changes**.

This is the **240×240 square** variant. There is a landscape sibling:
[**pico-infonesPlus-picogb**](https://github.com/superslor/pico-infonesPlus-picogb) (320×240).

> The original project outputs DVI/HDMI video. This fork replaces the video backend with a
> self-contained ST7789 SPI + DMA driver (`st7789.c`) running on the second core, and adds GPIO input,
> I2S audio routing, and aspect/letterbox handling — all behind a `USE_ST7789` build flag so the
> emulator core stays untouched. For full emulator documentation (ROMs, FDS, NSF, metadata, save
> states, region support), see **[README_upstream.md](README_upstream.md)**.

## Hardware

- **MCU:** Raspberry Pi Pico (RP2040), overclocked to **292 MHz**.
- **Display:** 240×240 **square** ST7789 over **SPI0** at ~73 MHz (12-bit RGB444 packed), colour-inverted
  (`INVON`) with a minimal init sequence.
- **Audio:** I2S DAC (PCM5000A) on the Game Boy audio pins.
- **Storage:** microSD over **SPI1** (a separate bus from the display).
- **Input:** 8 GPIO buttons, plus an optional USB controller as Player 2.

### Pin map (`HW_CONFIG=20`)

| Function | Pins |
|---|---|
| Display (SPI0) | CS 17 · CLK 18 · SDA 19 · DC 20 · RST 21 · BL 22 |
| SD card (SPI1) | CS 13 · SCK 14 · MOSI 15 · MISO 12 |
| Buttons (GPIO) | UP 2 · DOWN 3 · LEFT 4 · RIGHT 5 · A 6 · B 7 · SELECT 8 · START 9 |
| I2S audio | DATA 26 · BCLK 27 · LRCLK 28 |
| Onboard LED | 25 |

## Display, aspect & anti-tearing

The 256×240 NES picture is fit to the 240×240 square in one of two ways, cycled in-game:

- **8:7 (FIT):** a true **8:7 letterbox** — the picture fills the width and is scaled to 196 rows with
  **equal 22 px black bars** top and bottom, so proportions are correct.
- **1:1 (crop):** native pixels, centre-cropped to the middle 240 columns (fills the whole square).

Scaling is nearest-neighbour: per-pixel blending doesn't fit this panel's short per-line DMA budget at
60 fps. The panel's free-run refresh is trimmed (porch control) to ~60 Hz so a frame pacer **parks** the
diagonal tear seam with no framerate cost. The scaler and SPI DMA run on core 1; the emulator on core 0.

## Firmware

Pre-built `.uf2` files are in **[`firmware/`](firmware/)**:

| File | Build | USB | Serial diagnostics |
|---|---|---|---|
| `piconesPlus_240_host.uf2` | Shipping | USB controller = Player 2 | none |
| `piconesPlus_240_serialdiag.uf2` | Diagnostics | none | USB CDC on `/dev/ttyACM0` |

**To flash:** hold **BOOTSEL** while plugging in the Pico, then drag the chosen `.uf2` onto the
`RPI-RP2` drive that appears.

## Controls

| Input | Action |
|---|---|
| D-pad, A, B, SELECT, START | NES controller |
| SELECT + UP / DOWN | Cycle screen mode: 8:7 → 8:7 + scanlines → 1:1 crop → 1:1 crop + scanlines |
| SELECT + LEFT / RIGHT | Volume down / up |
| SELECT + B + LEFT / RIGHT | Fine anti-tear sync nudge (0.1 µs) |
| SELECT + B + UP / DOWN | Coarse anti-tear sync nudge (10 µs) |

## Building from source

Requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk) (`PICO_SDK_PATH` set) and the
`arm-none-eabi` toolchain.

```sh
# Shipping build (USB controller as Player 2):
cmake -B build -S . -DHW_CONFIG=20 -DPICO_PLATFORM=rp2040 -DUSE_ST7789=1 -DST7789_USB_SERIAL=0
cmake --build build -j

# Diagnostics build (USB CDC serial instead of the controller):
cmake -B build -S . -DHW_CONFIG=20 -DPICO_PLATFORM=rp2040 -DUSE_ST7789=1 -DST7789_USB_SERIAL=1
cmake --build build -j
```

The firmware is written to `build/piconesPlus.uf2`.

## Credits

- **NES emulator (InfoNES):** [@jay_kumogata](https://twitter.com/jay_kumogata)
- **Raspberry Pi Pico port:** [@shuichi_takano](https://twitter.com/shuichi_takano)
- **Menu system & SD support / upstream project:** [@frenskefrens](https://github.com/fhoedemakers) —
  [fhoedemakers/pico-infonesPlus](https://github.com/fhoedemakers/pico-infonesPlus)
- **ST7789 Portable Port:** **Slor**

Licensed under the same terms as the upstream project — see [LICENSE](LICENSE).
