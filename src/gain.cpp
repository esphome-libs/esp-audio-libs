#include "gain.h"

#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstdint>

#include "compiler.h"
#include "q31_utils.h"

namespace esp_audio_libs {
namespace gain {

int32_t db_to_q31(float db) {
  if (std::isnan(db)) {
    return 0;
  }
  if (db >= 0.0f) {
    return INT32_MAX;
  }
  const float linear = std::pow(10.0f, db / 20.0f);
  const float scaled = std::round(linear * 2147483648.0f);  // 2^31
  if (scaled >= 2147483648.0f) {
    return INT32_MAX;
  }
  if (scaled <= 0.0f) {
    return 0;
  }
  return static_cast<int32_t>(scaled);
}

int32_t db_reduction_to_q31(uint8_t db) {
  int32_t factor = INT32_MAX;  // 0 dB = unity gain
  for (uint8_t i = 0; i < db && factor > 0; ++i) {
    factor = internal::step_down_1db(factor);
  }
  return factor;
}

void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale, size_t samples_to_scale,
           size_t bytes_per_sample) {
  // Each case shifts the input sample into Q31 form, then performs a Q31×Q31 high-half multiply
  // (`(int64_t)a * (int64_t)b >> 32`). This is a single instruction on ESP32/other platforms.
  // This yields a Q30 result in int32. That int32 sample is shifted to restore the original bit
  // width. A rounding term is added for the 8 bit, 16 bit, and 24 bit cases.
  //
  // The 16 and 32 bit cases dispatch on a runtime alignment check: aligned buffers use the
  // fast_unpack_to_q31<> helper (which folds the load into a single l16si/l32i on Xtensa), and
  // misaligned buffers fall back to the byte-wise unpack_to_q31<> path that produces identical
  // output. Xtensa raises an alignment exception on misaligned word/halfword loads, so the
  // runtime gate is required.
  switch (bytes_per_sample) {
    case 1: {
      // 8 bit input shifted left by 24 to reach Q31. The high-half multiply produces s * sf >> 8.
      // Shift right by 23 to recover the 8 bit sample. Rounding term is 1 << 22.
      constexpr int32_t rounding = 1 << 22;
      int8_t *out = reinterpret_cast<int8_t *>(output_buffer);
      size_t i = 0;
      for (; i + 4 <= samples_to_scale; i += 4) {
        const int32_t s0 = internal::unpack_to_q31<1>(audio_samples + i);
        const int32_t s1 = internal::unpack_to_q31<1>(audio_samples + i + 1);
        const int32_t s2 = internal::unpack_to_q31<1>(audio_samples + i + 2);
        const int32_t s3 = internal::unpack_to_q31<1>(audio_samples + i + 3);
        const int32_t high0 = static_cast<int32_t>((static_cast<int64_t>(s0) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t high1 = static_cast<int32_t>((static_cast<int64_t>(s1) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t high2 = static_cast<int32_t>((static_cast<int64_t>(s2) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t high3 = static_cast<int32_t>((static_cast<int64_t>(s3) * static_cast<int64_t>(q31_scale)) >> 32);
        out[i] = static_cast<int8_t>((high0 + rounding) >> 23);
        out[i + 1] = static_cast<int8_t>((high1 + rounding) >> 23);
        out[i + 2] = static_cast<int8_t>((high2 + rounding) >> 23);
        out[i + 3] = static_cast<int8_t>((high3 + rounding) >> 23);
      }
      for (; i < samples_to_scale; ++i) {
        const int32_t s = internal::unpack_to_q31<1>(audio_samples + i);
        const int32_t high = static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
        out[i] = static_cast<int8_t>((high + rounding) >> 23);
      }
      break;
    }
    case 2: {
      // 16 bit input shifted left by 16 to reach Q31. The high-half multiply produces s * sf >> 16.
      // Shift right by 15 to recover the 16 bit sample. Rounding term is 1 << 14.
      constexpr int32_t rounding = 1 << 14;
      const bool aligned =
          ((reinterpret_cast<uintptr_t>(audio_samples) | reinterpret_cast<uintptr_t>(output_buffer)) & 0x1) == 0;
      if (aligned) {
        // Output stores use EAL_MEMCPY (to avoid strict-aliasing UB) with EAL_ASSUME_ALIGNED on
        // `out` (just established by the runtime check above) so each memcpy folds to a single
        // s16i on Xtensa. Aligned input loads are handled inside fast_unpack_to_q31<2>.
        const uint8_t *in = audio_samples;
        uint8_t *out = static_cast<uint8_t *>(EAL_ASSUME_ALIGNED(output_buffer, 2));
        size_t i = 0;
        for (; i + 4 <= samples_to_scale; i += 4) {
          const int32_t s0 = internal::fast_unpack_to_q31<2>(in + (i + 0) * 2);
          const int32_t s1 = internal::fast_unpack_to_q31<2>(in + (i + 1) * 2);
          const int32_t s2 = internal::fast_unpack_to_q31<2>(in + (i + 2) * 2);
          const int32_t s3 = internal::fast_unpack_to_q31<2>(in + (i + 3) * 2);
          const int32_t high0 =
              static_cast<int32_t>((static_cast<int64_t>(s0) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high1 =
              static_cast<int32_t>((static_cast<int64_t>(s1) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high2 =
              static_cast<int32_t>((static_cast<int64_t>(s2) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high3 =
              static_cast<int32_t>((static_cast<int64_t>(s3) * static_cast<int64_t>(q31_scale)) >> 32);
          const int16_t r0 = static_cast<int16_t>((high0 + rounding) >> 15);
          const int16_t r1 = static_cast<int16_t>((high1 + rounding) >> 15);
          const int16_t r2 = static_cast<int16_t>((high2 + rounding) >> 15);
          const int16_t r3 = static_cast<int16_t>((high3 + rounding) >> 15);
          EAL_MEMCPY(out + (i + 0) * 2, &r0, sizeof(int16_t));
          EAL_MEMCPY(out + (i + 1) * 2, &r1, sizeof(int16_t));
          EAL_MEMCPY(out + (i + 2) * 2, &r2, sizeof(int16_t));
          EAL_MEMCPY(out + (i + 3) * 2, &r3, sizeof(int16_t));
        }
        for (; i < samples_to_scale; ++i) {
          const int32_t s = internal::fast_unpack_to_q31<2>(in + i * 2);
          const int32_t high = static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          const int16_t r = static_cast<int16_t>((high + rounding) >> 15);
          EAL_MEMCPY(out + i * 2, &r, sizeof(int16_t));
        }
      } else {
        for (size_t i = 0; i < samples_to_scale; ++i) {
          const uint8_t *p_in = audio_samples + (i * 2);
          uint8_t *p_out = output_buffer + (i * 2);
          const int32_t s = internal::unpack_to_q31<2>(p_in);
          const int32_t high = static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          // Round and convert Q30 to Q31 form so pack keeps the right bytes.
          const int32_t scaled_q31 = static_cast<int32_t>(static_cast<uint32_t>(high + rounding) << 1);
          internal::pack_q31<2>(scaled_q31, p_out);
        }
      }
      break;
    }
    case 3: {
      // 24 bit input shifted left by 8 to reach Q31. The high-half multiply produces s * sf >> 24.
      // Shift right by 7 to recover the 24 bit sample. Rounding term is 1 << 6.
      constexpr int32_t rounding = 1 << 6;
      for (size_t i = 0; i < samples_to_scale; ++i) {
        const uint8_t *p_in = audio_samples + (i * 3);
        uint8_t *p_out = output_buffer + (i * 3);
        const int32_t s = internal::unpack_to_q31<3>(p_in);
        const int32_t high = static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t scaled = (high + rounding) >> 7;
        p_out[0] = static_cast<uint8_t>(scaled);
        p_out[1] = static_cast<uint8_t>(scaled >> 8);
        p_out[2] = static_cast<uint8_t>(scaled >> 16);
      }
      break;
    }
    case 4: {
      // 32 bit input is already Q31. The high-half multiply produces s * sf >> 32, a Q30 result.
      // Shift left by 1 to restore Q31. With sf in [0, INT32_MAX] the magnitude of the high half
      // is at most 2^30 (reached at s=INT32_MIN, sf=INT32_MAX), so the shift never overflows.
      // Cast through uint32_t so the shift is well-defined for negative values.
      const bool aligned =
          ((reinterpret_cast<uintptr_t>(audio_samples) | reinterpret_cast<uintptr_t>(output_buffer)) & 0x3) == 0;
      if (aligned) {
        // Output stores use EAL_MEMCPY (to avoid strict-aliasing UB) with EAL_ASSUME_ALIGNED on
        // `out` (just established by the runtime check above) so each memcpy folds to a single
        // s32i on Xtensa. Aligned input loads are handled inside fast_unpack_to_q31<4>.
        const uint8_t *in = audio_samples;
        uint8_t *out = static_cast<uint8_t *>(EAL_ASSUME_ALIGNED(output_buffer, 4));
        size_t i = 0;
        for (; i + 4 <= samples_to_scale; i += 4) {
          const int32_t v0 = internal::fast_unpack_to_q31<4>(in + (i + 0) * 4);
          const int32_t v1 = internal::fast_unpack_to_q31<4>(in + (i + 1) * 4);
          const int32_t v2 = internal::fast_unpack_to_q31<4>(in + (i + 2) * 4);
          const int32_t v3 = internal::fast_unpack_to_q31<4>(in + (i + 3) * 4);
          const int32_t high0 =
              static_cast<int32_t>((static_cast<int64_t>(v0) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high1 =
              static_cast<int32_t>((static_cast<int64_t>(v1) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high2 =
              static_cast<int32_t>((static_cast<int64_t>(v2) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high3 =
              static_cast<int32_t>((static_cast<int64_t>(v3) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t r0 = static_cast<int32_t>(static_cast<uint32_t>(high0) << 1);
          const int32_t r1 = static_cast<int32_t>(static_cast<uint32_t>(high1) << 1);
          const int32_t r2 = static_cast<int32_t>(static_cast<uint32_t>(high2) << 1);
          const int32_t r3 = static_cast<int32_t>(static_cast<uint32_t>(high3) << 1);
          EAL_MEMCPY(out + (i + 0) * 4, &r0, sizeof(int32_t));
          EAL_MEMCPY(out + (i + 1) * 4, &r1, sizeof(int32_t));
          EAL_MEMCPY(out + (i + 2) * 4, &r2, sizeof(int32_t));
          EAL_MEMCPY(out + (i + 3) * 4, &r3, sizeof(int32_t));
        }
        for (; i < samples_to_scale; ++i) {
          const int32_t v = internal::fast_unpack_to_q31<4>(in + i * 4);
          const int32_t high = static_cast<int32_t>((static_cast<int64_t>(v) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t r = static_cast<int32_t>(static_cast<uint32_t>(high) << 1);
          EAL_MEMCPY(out + i * 4, &r, sizeof(int32_t));
        }
      } else {
        for (size_t i = 0; i < samples_to_scale; ++i) {
          const uint8_t *p_in = audio_samples + (i * 4);
          uint8_t *p_out = output_buffer + (i * 4);
          const int32_t s = internal::unpack_to_q31<4>(p_in);
          const int32_t high = static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t scaled_q31 = static_cast<int32_t>(static_cast<uint32_t>(high) << 1);
          internal::pack_q31<4>(scaled_q31, p_out);
        }
      }
      break;
    }
  }
}

namespace {

// Applies a gain that ramps linearly from q31_start (exclusive) to q31_end (inclusive), as
// constant-factor sub-blocks of sub_block_samples each, one apply() call per sub-block. q31_start is
// the level the previous block ended on, so the first sub-block is already one step past it; the
// last sub-block uses exactly q31_end. sub_block_samples == 0 or >= samples_to_scale applies q31_end
// to the whole block. Both factors must be in [0, INT32_MAX]. May operate in place.
void apply_ramp(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_start, int32_t q31_end,
                size_t samples_to_scale, size_t sub_block_samples, size_t bytes_per_sample) {
  if (samples_to_scale == 0) {
    return;
  }
  // Constant factor, or the whole block fits in one sub-block: apply q31_end to all of it.
  if (q31_start == q31_end || sub_block_samples == 0 || sub_block_samples >= samples_to_scale) {
    apply(audio_samples, output_buffer, q31_end, samples_to_scale, bytes_per_sample);
    return;
  }

  const size_t num_sub_blocks = samples_to_scale / sub_block_samples + (samples_to_scale % sub_block_samples != 0);
  // Both factors are in [0, INT32_MAX], so the difference fits int32 and this is a 32-bit divide.
  // Accumulated rounding drift is erased by landing exactly on q31_end below.
  const int32_t delta = (q31_end - q31_start) / static_cast<int32_t>(num_sub_blocks);

  int32_t factor = q31_start;
  size_t processed = 0;
  for (size_t k = 0; k < num_sub_blocks; ++k) {
    const size_t chunk = std::min(sub_block_samples, samples_to_scale - processed);
    // Each sub-block holds the level reached by its end; the last one lands exactly on q31_end.
    factor = (k + 1 == num_sub_blocks) ? q31_end : (factor + delta);
    apply(audio_samples + processed * bytes_per_sample, output_buffer + processed * bytes_per_sample, factor, chunk,
          bytes_per_sample);
    processed += chunk;
  }
}

// Whole 1 dB grid steps strictly between `from` and `target`. Both must be nonzero: zero is not on
// the geometric grid, so ramp_segments() covers silence with a linear segment at the floor.
uint32_t count_1db_steps(int32_t from, int32_t target) {
  const bool down = target < from;
  uint32_t n = 0;
  int32_t x = from;
  for (;;) {
    const int32_t next = down ? internal::step_down_1db(x) : internal::step_up_1db(x);
    // Stop on reaching the target, or if the step stalls (step_up_1db is flat for x <= 8).
    const bool reached = down ? (next <= target) : (next >= target);
    const bool no_progress = down ? (next >= x) : (next <= x);
    if (reached || no_progress)
      break;
    x = next;
    ++n;
  }
  return n;
}

// Applies step_down_1db n times at compile time.
constexpr int32_t step_down_n(int32_t x, unsigned n) {
  return n == 0 ? x : step_down_n(internal::step_down_1db(x), n - 1);
}

// Grid point where a fade to silence leaves the 1 dB grid: the ramp steps down to here, then one
// more segment (the same length as a 1 dB step) runs linearly to 0. A fade in from below the floor
// mirrors it: one linear segment up to the floor, then 1 dB steps.
constexpr unsigned SILENCE_FLOOR_DB = 100;
constexpr int32_t SILENCE_FLOOR_Q31 = step_down_n(INT32_MAX, SILENCE_FLOOR_DB);
static_assert(SILENCE_FLOOR_Q31 > 8, "silence floor must sit above the range where step_up_1db stalls");

// Number of equal-length segments a ramp from `from` to `target` takes: one per whole 1 dB step,
// one to land exactly on the target, plus the linear segment at the silence floor when the ramp
// starts below it or ends at 0.
uint32_t ramp_segments(int32_t from, int32_t target) {
  if (target < from) {
    if (target != 0)
      return count_1db_steps(from, target) + 1;
    if (from <= SILENCE_FLOOR_Q31)
      return 1;                                           // Already at or below the floor: linear to 0.
    return count_1db_steps(from, SILENCE_FLOOR_Q31) + 2;  // Land on the floor, then linear to 0.
  }
  if (from >= SILENCE_FLOOR_Q31)
    return count_1db_steps(from, target) + 1;
  if (target <= SILENCE_FLOOR_Q31)
    return 1;                                             // Whole ramp is below the floor: linear.
  return count_1db_steps(SILENCE_FLOOR_Q31, target) + 2;  // Linear to the floor, then 1 dB steps.
}

// Largest Q31 change per constant-factor sub-block: one eighth of a 1 dB step from unity. Keeps the
// wide to/from-silence segment as smooth as the 1 dB grid.
constexpr uint32_t MAX_Q31_PER_SUB_BLOCK = static_cast<uint32_t>(INT32_MAX - internal::step_down_1db(INT32_MAX)) / 8;
static_assert(MAX_Q31_PER_SUB_BLOCK == 29192104, "1 dB sub-block granularity changed");

// Sub-block length for a segment: 8 sub-steps, or more when the span exceeds 1 dB. Floored at 8
// samples so apply()'s 4-wide unrolled loop is always reached on very fast ramps.
inline size_t sub_block_for_segment(uint32_t samples, uint32_t span_q31) {
  constexpr uint32_t SUB_STEPS_PER_SEGMENT = 8;
  constexpr uint32_t MIN_SUB_BLOCK_SAMPLES = 8;
  const uint32_t by_span = span_q31 / MAX_Q31_PER_SUB_BLOCK + 1;
  const uint32_t sub_steps = by_span > SUB_STEPS_PER_SEGMENT ? by_span : SUB_STEPS_PER_SEGMENT;
  const uint32_t sub = samples / sub_steps;
  return sub < MIN_SUB_BLOCK_SAMPLES ? MIN_SUB_BLOCK_SAMPLES : sub;
}

}  // namespace

void GainRamp::schedule_(int32_t target_q31, uint32_t samples_per_step, uint32_t steps) {
  this->target_q31_ = target_q31;
  this->samples_per_step_ = samples_per_step;
  if (samples_per_step == 0) {
    // Nothing to ramp, or too short to give each step a sample: jump.
    this->current_q31_ = target_q31;
    this->samples_remaining_ = 0;
    return;
  }
  // Exact multiple of samples_per_step so the final segment ends as samples_remaining hits 0.
  // Also what makes process() start a fresh segment on its next call after a retarget. A rate large
  // enough to overflow is clamped by shrinking the step so the product still fits and stays exact.
  if (samples_per_step > UINT32_MAX / steps) {
    this->samples_per_step_ = UINT32_MAX / steps;
  }
  this->samples_remaining_ = this->samples_per_step_ * steps;
}

void GainRamp::post_request_(int32_t target_q31, uint32_t param, bool at_rate) {
  // Seqlock writer: odd sequence while the fields are in flux, even once they are consistent.
  const uint32_t seq = this->request_seq_.load(std::memory_order_relaxed);
  this->request_seq_.store(seq + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  this->request_target_.store(target_q31, std::memory_order_relaxed);
  this->request_param_.store(param, std::memory_order_relaxed);
  this->request_at_rate_.store(at_rate, std::memory_order_relaxed);
  this->request_seq_.store(seq + 2, std::memory_order_release);
}

void GainRamp::take_request_() {
  // Seqlock reader. Never waits: a request caught mid-write is simply tried again next block, so
  // process() cannot stall on a lower-priority writer it has preempted.
  const uint32_t seq = this->request_seq_.load(std::memory_order_acquire);
  if (seq == this->applied_seq_ || (seq & 1) != 0) {
    return;
  }
  const int32_t target_q31 = this->request_target_.load(std::memory_order_relaxed);
  const uint32_t param = this->request_param_.load(std::memory_order_relaxed);
  const bool at_rate = this->request_at_rate_.load(std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_acquire);
  if (this->request_seq_.load(std::memory_order_relaxed) != seq) {
    return;
  }
  this->applied_seq_ = seq;

  if (target_q31 == this->target_q31_ && this->samples_remaining_ == 0) {
    return;  // Already settled there.
  }
  if (param == 0 || target_q31 == this->current_q31_) {
    this->schedule_(target_q31, 0, 0);
    return;
  }
  const uint32_t steps = ramp_segments(this->current_q31_, target_q31);
  this->schedule_(target_q31, at_rate ? param : param / steps, steps);
}

void GainRamp::set_target_over(int32_t target_q31, uint32_t ramp_samples) {
  this->post_request_(target_q31, ramp_samples, false);
}

void GainRamp::set_target_db_reduction_over(uint8_t db, uint32_t ramp_samples) {
  this->set_target_over(db_reduction_to_q31(db), ramp_samples);
}

void GainRamp::set_target_db_over(float db, uint32_t ramp_samples) {
  this->set_target_over(db_to_q31(db), ramp_samples);
}

void GainRamp::set_target_at_rate(int32_t target_q31, uint32_t samples_per_db) {
  this->post_request_(target_q31, samples_per_db, true);
}

void GainRamp::set_target_db_reduction_at_rate(uint8_t db, uint32_t samples_per_db) {
  this->set_target_at_rate(db_reduction_to_q31(db), samples_per_db);
}

void GainRamp::set_target_db_at_rate(float db, uint32_t samples_per_db) {
  this->set_target_at_rate(db_to_q31(db), samples_per_db);
}

void GainRamp::process(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples) {
  this->take_request_();
  // schedule_() guarantees samples_per_step_ > 0 whenever samples_remaining_ > 0.
  while (samples > 0 && this->samples_remaining_ > 0) {
    uint32_t samples_left_in_step = this->samples_remaining_ % this->samples_per_step_;
    if (samples_left_in_step == 0) {
      // Segment boundary: the last segment ends on the target, earlier ones one 1 dB step closer.
      // A fade to silence stops stepping at the floor (the last segment is linear from there), and
      // a fade in from below the floor first goes linearly to the floor. See ramp_segments().
      samples_left_in_step = this->samples_per_step_;
      if (this->samples_remaining_ <= this->samples_per_step_) {
        this->seg_target_q31_ = this->target_q31_;
      } else if (this->target_q31_ < this->current_q31_) {
        const int32_t next = internal::step_down_1db(this->current_q31_);
        this->seg_target_q31_ = (this->target_q31_ == 0 && next < SILENCE_FLOOR_Q31) ? SILENCE_FLOOR_Q31 : next;
      } else if (this->current_q31_ < SILENCE_FLOOR_Q31) {
        this->seg_target_q31_ = SILENCE_FLOOR_Q31;
      } else {
        this->seg_target_q31_ = internal::step_up_1db(this->current_q31_);
      }
    }

    const uint32_t chunk = std::min(samples, samples_left_in_step);
    // Span and samples left shrink together within a segment, so the sub-block step stays uniform.
    const uint32_t seg_span_abs =
        static_cast<uint32_t>(this->seg_target_q31_ > this->current_q31_ ? this->seg_target_q31_ - this->current_q31_
                                                                         : this->current_q31_ - this->seg_target_q31_);
    const size_t sub_block_samples = sub_block_for_segment(samples_left_in_step, seg_span_abs);
    // Interpolate the remaining span over the remaining samples, exactly, from the live value. No
    // per-sample slope to truncate, so quiet or very long segments never stall, and the last chunk
    // of the segment lands exactly on seg_target_q31_.
    const int32_t chunk_end =
        (chunk >= samples_left_in_step)
            ? this->seg_target_q31_
            : this->current_q31_ +
                  static_cast<int32_t>(static_cast<int64_t>(this->seg_target_q31_ - this->current_q31_) *
                                       static_cast<int64_t>(chunk) / samples_left_in_step);
    apply_ramp(buffer, buffer, this->current_q31_, chunk_end, chunk, sub_block_samples, bytes_per_sample);

    this->current_q31_ = chunk_end;
    buffer += static_cast<size_t>(chunk) * bytes_per_sample;
    this->samples_remaining_ -= chunk;
    samples -= chunk;
  }

  // Settled tail: one constant factor, skipping the unity no-op.
  if (samples > 0 && this->current_q31_ != INT32_MAX) {
    apply(buffer, buffer, this->current_q31_, samples, bytes_per_sample);
  }
  this->current_published_.store(this->current_q31_, std::memory_order_relaxed);
}

}  // namespace gain
}  // namespace esp_audio_libs
