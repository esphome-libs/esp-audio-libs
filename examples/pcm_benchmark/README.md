# PCM Convert / Mixer / Gain Benchmark

Benchmarks `pcm_convert::copy_frames`, `mixer::mix_frames`, and `gain::apply` across a matrix of formats: different input and output bit depths (1, 2, 3, 4 bytes per sample) and, for convert and mix, different channel counts (mono and stereo, including up-mix and down-mix). The gain pass adds in-place and out-of-place variants per bit depth. The input is synthetic PCM generated at runtime, so this is a pure CPU throughput test of the kernels with no embedded audio asset.

## Building and Flashing

### Prerequisites

- **PlatformIO** (recommended) OR ESP-IDF v5.0 or later
- Any ESP32 development board (the benchmark keeps its buffers in internal RAM, so PSRAM is not required)

### Option 1: PlatformIO (Recommended)

```bash
cd examples/pcm_benchmark

# Build, upload, and monitor (ESP32-S3)
pio run -e esp32s3 -t upload -t monitor

# Or target a plain ESP32
pio run -e esp32 -t upload -t monitor
```

The PlatformIO configuration pulls in the parent esp-audio-libs repository as a component, so no additional setup is required.

### Option 2: Native ESP-IDF

```bash
cd examples/pcm_benchmark
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Configuration Options

The benchmark dimensions are compile-time defines (override them with `build_flags` in `platformio.ini` or `-D` in ESP-IDF):

| Define | Default | Meaning |
| ------ | ------- | ------- |
| `PCM_BENCH_FRAMES` | 1024 | Frames processed per library call |
| `PCM_BENCH_BATCH` | 32 | Calls timed together as one measurement sample |
| `PCM_BENCH_SAMPLES` | 50 | Measurement samples collected per scenario |

To benchmark different formats, edit the `CONVERT_SCENARIOS`, `MIX_SCENARIOS`, and `GAIN_SCENARIOS` tables near the top of `src/pcm_benchmark.cpp`. Convert and mix entries are `{bps, channels}` sets; gain entries are `{bps, in_place}`. Bit depth is in bytes (1, 2, 3, 4).

## Expected Output

Each iteration prints one line per scenario, for both alignment cases:

```text
I (310) PCM_BENCH: --- pcm_convert::copy_frames ---
I (330) PCM_BENCH: CONV 2b2 -> 2b2 (copy)      aligned     10.86 ns/frame     736.7 MB/s      1918x RT  (min 10.83 max 11.05 sd 0.04)
I (350) PCM_BENCH: CONV 2b2 -> 2b2 (copy)      misaligned  10.95 ns/frame     730.5 MB/s      1902x RT  (min 10.93 max 11.14 sd 0.04)
I (400) PCM_BENCH: CONV 2b2 -> 4b2 (widen)     aligned     28.72 ns/frame     417.9 MB/s       725x RT  (min 28.69 max 28.90 sd 0.06)
I (580) PCM_BENCH: CONV 2b2 -> 4b2 (widen)     misaligned 108.82 ns/frame     110.3 MB/s       191x RT  (min 108.73 max 108.95 sd 0.09)
...
I (2310) PCM_BENCH: --- mixer::mix_frames ---
I (2480) PCM_BENCH: MIX  2b2 + 2b2 -> 2b2       aligned    100.72 ns/frame     119.1 MB/s       207x RT  (min 100.65 max 100.86 sd 0.09)
...
I (6760) PCM_BENCH: --- gain::apply ---
I (6810) PCM_BENCH: GAIN 2b out-of-place        aligned     27.32 ns/sample    146.4 MB/s       763x RT  (min 27.28 max 27.50 sd 0.06)
I (6930) PCM_BENCH: GAIN 2b out-of-place        misaligned  75.22 ns/sample     53.2 MB/s       277x RT  (min 75.16 max 75.38 sd 0.08)
```

Columns: kind (`CONV`/`MIX`/`GAIN`), format (`<bps>b<channels>`; e.g., `2b2` is 16-bit stereo, with input(s) → output), alignment, ns/frame (ns/sample for gain, which has no channel concept; lower is better), MB/s of read+write traffic, xRT@48k (how many 48 kHz streams one core could sustain on this op alone), and the per-sample min/max/sd spread.

## How It Works

Three 16-byte-aligned buffers (two inputs, one output) are filled with a deterministic pseudo-random pattern; the sample values do not affect timing. Each scenario calls the kernel `PCM_BENCH_BATCH` times back-to-back, timed as a unit with `esp_timer_get_time()` to stay well above the 1 µs timer resolution, and `PCM_BENCH_SAMPLES` such batches give the min/max/avg/sd spread. The misaligned variant offsets every pointer by one byte to force the byte-wise path.

## Interpreting the Results

- The `aligned` rows reflect the wide-load fast path that the library takes when each pointer is aligned to its sample width (2 bytes for 16-bit, 4 bytes for 32-bit). The `misaligned` rows show the byte-wise fallback. The gap is the value of keeping your audio buffers aligned.
- Bit-depth conversion goes through a Q31 intermediate, so widening (e.g. 16→32) is exact while narrowing rounds the low bits. The identity case (matching bit depth and channel count) degenerates to a `memcpy` and is the fastest convert scenario.
- Mixing sums two streams in Q29, so `mix_frames` does roughly twice the input traffic of a convert and lands slower per frame.
- Gain multiplies each sample by a Q31 scale factor (1, 2, and 3 byte widths round; the 4 byte width truncates). It touches a single sample per unit, so its ns/sample numbers are not directly comparable to the per-frame convert/mix numbers; e.g., a stereo frame is two samples. In-place and out-of-place perform the same arithmetic; any difference is purely memory traffic (one buffer touched versus two).
