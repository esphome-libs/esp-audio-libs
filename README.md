# esp-audio-libs

Audio resampling library for ESP32 devices, with optimized assembly implementations of the underlying dot product on Xtensa cores. Based on the following:
- [ART-resampler](https://github.com/dbry/audio-resampler) for resampling audio, optimized with assembly dot product functions.
    - Author: David Bryant
    - License: BSD-3-Clause
- [esp-dsp](https://github.com/espressif/esp-dsp) assembly functions for the floating point dot product used internally by the resampler.
    - Author: Espressif
    - License: Apache v2.0

## Migrating from earlier versions

The FLAC, MP3, and WAV decoders, as well as the public DSP functions, were removed in version 3.0.0. Replacements:
- FLAC decoding: [esphome-libs/micro-flac](https://github.com/esphome-libs/micro-flac)
- MP3 decoding: [esphome-libs/micro-mp3](https://github.com/esphome-libs/micro-mp3)
- WAV header parsing: [esphome-libs/micro-wav](https://github.com/esphome-libs/micro-wav)
- DSP functions (esp-dsp's `dsps_biquad_f32`, `dsps_dotprod_f32`, `dsps_add_s16`, `dsps_mulc_s16`): use [esp-dsp](https://github.com/espressif/esp-dsp) directly.

The replacements are not drop-in API-compatible.
