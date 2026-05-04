# esp-audio-libs

Audio resampling library for ESP32 devices, with optimized assembly implementations of the underlying dot product on Xtensa cores. Based on the following:
- [ART-resampler](https://github.com/dbry/audio-resampler) for resampling audio, optimized with assembly dot product functions.
    - Author: David Bryant
    - License: BSD-3-Clause
- [esp-dsp](https://github.com/espressif/esp-dsp) assembly functions for the floating point dot product used internally by the resampler.
    - Author: Espressif
    - License: Apache v2.0
