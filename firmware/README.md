# PicoNES Portable — ST7789 firmware (RP2040 / Pico-GB hardware, 240×240 square)

Flash by holding **BOOTSEL** while plugging in the Pico, then copy a `.uf2` onto the `RPI-RP2` drive.

| File | Build | USB | Serial diag |
|------|-------|-----|-------------|
| `piconesPlus_240_host.uf2`       | Shipping | USB controller = Player 2 | none |
| `piconesPlus_240_serialdiag.uf2` | Diagnostics | none | USB CDC on `/dev/ttyACM0` |

True 8:7 letterbox (equal 22px bars, 196 active rows), parked 60 fps. ST7789 Portable Port by **Slor**.
