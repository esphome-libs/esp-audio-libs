// Copyright 2026 Kevin Ahrendt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* esp-audio-libs PCM convert / mixer benchmark
 *
 * Measures the throughput of pcm_convert::copy_frames and mixer::mix_frames
 * across a matrix of formats: different input/output bit depths (1, 2, 3, 4
 * bytes per sample) and different channel counts (mono <-> stereo up/down mix).
 *
 * The input is synthetic PCM generated at runtime (a cheap pseudo-random fill),
 * so the example needs no embedded audio asset -- this is a pure CPU throughput
 * test of the conversion kernels, not a decode test.
 *
 * Each scenario is run twice: once on buffers aligned to their sample width
 * (the documented wide-load fast path on Xtensa) and once on deliberately
 * misaligned buffers (the byte-wise fallback), so the cost of the alignment
 * dispatch is visible directly.
 *
 * Buffers live in internal RAM. They are small enough to stay resident in the
 * data cache, so running them from PSRAM would mostly measure cache behavior
 * rather than the external bus -- not a meaningful comparison, so it is omitted.
 */

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gain.h"
#include "mixer.h"
#include "pcm_convert.h"

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstring>

static const char *const TAG = "PCM_BENCH";

// Frames processed per library call. Sized like a typical audio chunk.
#ifndef PCM_BENCH_FRAMES
#define PCM_BENCH_FRAMES 1024
#endif

// Library calls timed together as one measurement sample. Batching keeps each
// measured interval well above the 1 us esp_timer resolution.
#ifndef PCM_BENCH_BATCH
#define PCM_BENCH_BATCH 32
#endif

// Number of measurement samples collected per scenario (for min/max/avg/sd).
#ifndef PCM_BENCH_SAMPLES
#define PCM_BENCH_SAMPLES 50
#endif

static const uint32_t FRAMES = PCM_BENCH_FRAMES;
static const int BATCH = PCM_BENCH_BATCH;
static const int SAMPLES = PCM_BENCH_SAMPLES;

// Sample rate used only to express throughput as a real-time factor.
static const double REFERENCE_SAMPLE_RATE = 48000.0;

// Widest single-buffer frame: 2 channels * 4 bytes. The +64 tail leaves room
// for the 1-byte offset used to force the misaligned fallback path.
static const size_t MAX_BYTES_PER_FRAME = 2 * 4;
static const size_t BUFFER_BYTES = FRAMES * MAX_BYTES_PER_FRAME + 64;

namespace pcm_convert = esp_audio_libs::pcm_convert;
namespace mixer = esp_audio_libs::mixer;
namespace gain = esp_audio_libs::gain;

// Three aligned working buffers: two inputs (primary/secondary or convert
// source) and one output. Allocated from internal RAM so PSRAM cache misses
// do not contaminate the kernel timings.
static uint8_t *g_in_a = nullptr;
static uint8_t *g_in_b = nullptr;
static uint8_t *g_out = nullptr;

// ---------------------------------------------------------------------------
// Statistics over per-frame timings (nanoseconds)
// ---------------------------------------------------------------------------

struct Stats {
  double min_ns;
  double max_ns;
  double sum_ns;
  double sum_sq_ns;
  int count;
};

static void init_stats(Stats *s) {
  s->min_ns = 1e30;
  s->max_ns = 0.0;
  s->sum_ns = 0.0;
  s->sum_sq_ns = 0.0;
  s->count = 0;
}

static void update_stats(Stats *s, double ns) {
  if (ns < s->min_ns) {
    s->min_ns = ns;
  }
  if (ns > s->max_ns) {
    s->max_ns = ns;
  }
  s->sum_ns += ns;
  s->sum_sq_ns += ns * ns;
  s->count++;
}

// Log one scenario's result: average ns per unit, throughput, real-time factor,
// and the per-sample min/max/sd spread.
//
// The unit is a frame for convert/mix and a single sample for gain (gain has no
// channel concept). bytes_per_unit counts both reads and writes (input
// footprint + output footprint), so MB/s reflects total memory traffic moved.
static void log_result(const char *kind, const char *name, const char *align, const char *unit,
                       const Stats *s, double bytes_per_unit) {
  if (s->count == 0) {
    ESP_LOGI(TAG, "%s %-22s %-10s no data", kind, name, align);
    return;
  }

  double avg = s->sum_ns / s->count;
  double variance = s->sum_sq_ns / s->count - avg * avg;
  double sd = variance > 0.0 ? sqrt(variance) : 0.0;

  double units_per_sec = avg > 0.0 ? 1e9 / avg : 0.0;
  double mb_per_sec = avg > 0.0 ? bytes_per_unit * 1000.0 / avg : 0.0;
  double realtime_x = units_per_sec / REFERENCE_SAMPLE_RATE;

  ESP_LOGI(TAG, "%s %-22s %-10s %6.2f ns/%-7s %7.1f MB/s  %8.0fx RT  (min %.2f max %.2f sd %.2f)",
           kind, name, align, avg, unit, mb_per_sec, realtime_x, s->min_ns, s->max_ns, sd);
}

// ---------------------------------------------------------------------------
// Scenario tables
// ---------------------------------------------------------------------------

struct ConvertScenario {
  const char *name;
  uint8_t in_bps;
  uint8_t in_ch;
  uint8_t out_bps;
  uint8_t out_ch;
};

// bps in bytes (1/2/3/4), ch in channels. Names use the "<bps>b<ch>" shorthand.
static const ConvertScenario CONVERT_SCENARIOS[] = {
    {"2b2 -> 2b2 (copy)", 2, 2, 2, 2},   {"2b2 -> 4b2 (widen)", 2, 2, 4, 2},
    {"4b2 -> 2b2 (narrow)", 4, 2, 2, 2}, {"3b2 -> 2b2", 3, 2, 2, 2},
    {"2b2 -> 3b2", 2, 2, 3, 2},          {"1b2 -> 2b2", 1, 2, 2, 2},
    {"2b1 -> 2b2 (upmix)", 2, 1, 2, 2},  {"2b2 -> 2b1 (downmix)", 2, 2, 2, 1},
    {"2b1 -> 4b2", 2, 1, 4, 2},
};
static const int NUM_CONVERT_SCENARIOS = sizeof(CONVERT_SCENARIOS) / sizeof(CONVERT_SCENARIOS[0]);

struct MixScenario {
  const char *name;
  uint8_t pri_bps;
  uint8_t pri_ch;
  uint8_t sec_bps;
  uint8_t sec_ch;
  uint8_t out_bps;
  uint8_t out_ch;
};

static const MixScenario MIX_SCENARIOS[] = {
    {"2b2 + 2b2 -> 2b2", 2, 2, 2, 2, 2, 2}, {"2b2 + 2b1 -> 2b2", 2, 2, 2, 1, 2, 2},
    {"4b2 + 2b2 -> 4b2", 4, 2, 2, 2, 4, 2}, {"3b2 + 3b2 -> 3b2", 3, 2, 3, 2, 3, 2},
    {"2b1 + 2b1 -> 2b1", 2, 1, 2, 1, 2, 1}, {"2b2 + 4b2 -> 2b2", 2, 2, 4, 2, 2, 2},
};
static const int NUM_MIX_SCENARIOS = sizeof(MIX_SCENARIOS) / sizeof(MIX_SCENARIOS[0]);

// gain::apply works on a flat run of samples and has no channel concept, so a
// scenario is just a sample width plus whether the call is in-place (output ==
// input) or out-of-place. The aligned/misaligned variants are added by the
// runner, exercising the wide-load fast path versus the byte-wise fallback.
struct GainScenario {
  const char *name;
  uint8_t bps;
  bool in_place;
};

static const GainScenario GAIN_SCENARIOS[] = {
    {"2b out-of-place", 2, false}, {"3b out-of-place", 3, false}, {"4b out-of-place", 4, false},
    {"2b in-place", 2, true},      {"3b in-place", 3, true},      {"4b in-place", 4, true},
};
static const int NUM_GAIN_SCENARIOS = sizeof(GAIN_SCENARIOS) / sizeof(GAIN_SCENARIOS[0]);

// ---------------------------------------------------------------------------
// Benchmark runners
// ---------------------------------------------------------------------------

// One measurement sample = BATCH back-to-back kernel calls, timed together.
// The reported ns/frame is averaged across all calls in all samples.
static void run_convert(const ConvertScenario &s, bool misaligned) {
  size_t off = misaligned ? 1 : 0;
  const uint8_t *in = g_in_a + off;
  uint8_t *out = g_out + off;

  // Warm caches / branch predictors before timing.
  for (int w = 0; w < 4; w++) {
    pcm_convert::copy_frames(in, out, s.in_bps, s.in_ch, s.out_bps, s.out_ch, FRAMES);
  }

  Stats st;
  init_stats(&st);
  for (int sample = 0; sample < SAMPLES; sample++) {
    int64_t t0 = esp_timer_get_time();
    for (int b = 0; b < BATCH; b++) {
      pcm_convert::copy_frames(in, out, s.in_bps, s.in_ch, s.out_bps, s.out_ch, FRAMES);
    }
    int64_t dt_us = esp_timer_get_time() - t0;
    double ns_per_frame = (double)dt_us * 1000.0 / ((double)BATCH * (double)FRAMES);
    update_stats(&st, ns_per_frame);
  }

  double bytes_per_frame = (double)s.in_bps * s.in_ch + (double)s.out_bps * s.out_ch;
  log_result("CONV", s.name, misaligned ? "misaligned" : "aligned", "frame", &st, bytes_per_frame);
}

static void run_mix(const MixScenario &s, bool misaligned) {
  size_t off = misaligned ? 1 : 0;
  const uint8_t *pri = g_in_a + off;
  const uint8_t *sec = g_in_b + off;
  uint8_t *out = g_out + off;

  for (int w = 0; w < 4; w++) {
    mixer::mix_frames(pri, s.pri_bps, s.pri_ch, sec, s.sec_bps, s.sec_ch, out, s.out_bps, s.out_ch,
                      FRAMES);
  }

  Stats st;
  init_stats(&st);
  for (int sample = 0; sample < SAMPLES; sample++) {
    int64_t t0 = esp_timer_get_time();
    for (int b = 0; b < BATCH; b++) {
      mixer::mix_frames(pri, s.pri_bps, s.pri_ch, sec, s.sec_bps, s.sec_ch, out, s.out_bps,
                        s.out_ch, FRAMES);
    }
    int64_t dt_us = esp_timer_get_time() - t0;
    double ns_per_frame = (double)dt_us * 1000.0 / ((double)BATCH * (double)FRAMES);
    update_stats(&st, ns_per_frame);
  }

  double bytes_per_frame = (double)s.pri_bps * s.pri_ch + (double)s.sec_bps * s.sec_ch +
                           (double)s.out_bps * s.out_ch;
  log_result("MIX ", s.name, misaligned ? "misaligned" : "aligned", "frame", &st, bytes_per_frame);
}

// gain::apply scales a flat run of FRAMES samples. A representative non-unity
// scale (-6 dB) is used so the kernel does real multiply/round work; the exact
// value does not affect timing. For the in-place case the buffer is also the
// output, so repeated calls drive the samples toward zero -- harmless here since
// the conversion math has no zero special-casing.
static void run_gain(const GainScenario &s, bool misaligned) {
  static const int32_t q31_scale = gain::db_to_q31(-6.0f);
  size_t off = misaligned ? 1 : 0;
  const uint8_t *in = g_in_a + off;
  uint8_t *out = s.in_place ? (g_in_a + off) : (g_out + off);

  for (int w = 0; w < 4; w++) {
    gain::apply(in, out, q31_scale, FRAMES, s.bps);
  }

  Stats st;
  init_stats(&st);
  for (int sample = 0; sample < SAMPLES; sample++) {
    int64_t t0 = esp_timer_get_time();
    for (int b = 0; b < BATCH; b++) {
      gain::apply(in, out, q31_scale, FRAMES, s.bps);
    }
    int64_t dt_us = esp_timer_get_time() - t0;
    double ns_per_sample = (double)dt_us * 1000.0 / ((double)BATCH * (double)FRAMES);
    update_stats(&st, ns_per_sample);
  }

  // One read plus one write of the sample, regardless of in-place vs not.
  double bytes_per_sample = 2.0 * (double)s.bps;
  log_result("GAIN", s.name, misaligned ? "misaligned" : "aligned", "sample", &st, bytes_per_sample);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

// Fill a buffer with a deterministic pseudo-random pattern. Real data is
// irrelevant to timing; a varied bit pattern just avoids any denormal/zero
// special-casing in the conversion math.
static void fill_pattern(uint8_t *buf, size_t bytes, uint32_t seed) {
  uint32_t state = seed;
  for (size_t i = 0; i < bytes; i++) {
    state = state * 1664525u + 1013904223u;  // Numerical Recipes LCG
    buf[i] = (uint8_t)(state >> 24);
  }
}

static bool alloc_buffers() {
  uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  g_in_a = (uint8_t *)heap_caps_aligned_alloc(16, BUFFER_BYTES, caps);
  g_in_b = (uint8_t *)heap_caps_aligned_alloc(16, BUFFER_BYTES, caps);
  g_out = (uint8_t *)heap_caps_aligned_alloc(16, BUFFER_BYTES, caps);
  if (g_in_a == nullptr || g_in_b == nullptr || g_out == nullptr) {
    return false;
  }
  fill_pattern(g_in_a, BUFFER_BYTES, 0x12345678u);
  fill_pattern(g_in_b, BUFFER_BYTES, 0x9abcdef0u);
  memset(g_out, 0, BUFFER_BYTES);
  return true;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "=== esp-audio-libs PCM convert / mixer benchmark ===");
  ESP_LOGI(TAG, "Frames/call: %lu   Batch: %d   Samples: %d", (unsigned long)FRAMES, BATCH, SAMPLES);
  ESP_LOGI(TAG, "Free internal: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  if (!alloc_buffers()) {
    ESP_LOGE(TAG, "Failed to allocate benchmark buffers (%zu bytes x3)", BUFFER_BYTES);
    return;
  }

  ESP_LOGI(TAG, "Columns: <kind> <format> <alignment> <ns/frame> <MB/s> <xRT@48k> (per-sample spread)");
  ESP_LOGI(TAG, "  ns/frame: lower is better.  MB/s: total read+write traffic.");
  ESP_LOGI(TAG, "  xRT@48k: how many 48 kHz streams one core could sustain on this op alone.");

  uint32_t iteration = 0;
  while (true) {
    iteration++;
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Iteration %lu ===", (unsigned long)iteration);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- pcm_convert::copy_frames ---");
    for (int i = 0; i < NUM_CONVERT_SCENARIOS; i++) {
      run_convert(CONVERT_SCENARIOS[i], false);
      run_convert(CONVERT_SCENARIOS[i], true);
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- mixer::mix_frames ---");
    for (int i = 0; i < NUM_MIX_SCENARIOS; i++) {
      run_mix(MIX_SCENARIOS[i], false);
      run_mix(MIX_SCENARIOS[i], true);
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- gain::apply ---");
    for (int i = 0; i < NUM_GAIN_SCENARIOS; i++) {
      run_gain(GAIN_SCENARIOS[i], false);
      run_gain(GAIN_SCENARIOS[i], true);
    }

    ESP_LOGI(TAG, "---");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
