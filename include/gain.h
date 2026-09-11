#pragma once

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
namespace gain {

/// @brief Converts a linear gain in decibels to a Q31 scale factor in [0, INT32_MAX].
/// Q31 cannot represent values >= 1.0. 0 dB and above (including +Inf) clamp to INT32_MAX, which
/// is 1.0 - 2^-31. Very negative dB (including -Inf) rounds to 0. NaN returns 0. Use a different
/// fixed-point format if you need positive dB gain.
/// @param db Gain in dB.
/// @return Q31 scale factor in the range [0, INT32_MAX].
int32_t db_to_q31(float db);

/// @brief Converts an integer dB reduction below unity to a Q31 scale factor. Integer math only.
///
/// Walks the same 1 dB grid GainRamp steps along, so the result is exactly a grid point. 0 is unity
/// (INT32_MAX); 173 and above are silence (0), where the grid bottoms out.
/// @param db Attenuation in dB.
/// @return Q31 scale factor in the range [0, INT32_MAX].
int32_t db_reduction_to_q31(uint8_t db);

/// @brief Multiplies each sample by a Q31 scale factor in [0, INT32_MAX]. Cannot amplify.
///
/// Sample format: signed PCM, little-endian for multi-byte widths, interleaved across channels.
/// 8-bit samples are interpreted as int8 (0x00 is silence), not WAV-style uint8 (0x80 is silence).
/// Callers handling WAV 8-bit must convert before/after calling.
///
/// May operate in-place when output_buffer == audio_samples. Rounding is applied for the 1, 2, and
/// 3 byte widths. The 4 byte width is truncated.
/// At unity (q31_scale == INT32_MAX) the output equals the input within one LSB of rounding loss.
/// Callers in hot paths should skip this call when q31_scale is unity.
/// For best throughput, both audio_samples and output_buffer must be aligned to bytes_per_sample.
/// If either pointer is misaligned, the call falls back to a byte-wise slow path that produces
/// identical output.
/// @param audio_samples Input buffer of interleaved signed samples.
/// @param output_buffer Output buffer (may alias the input).
/// @param q31_scale Q31 scale factor in the range [0, INT32_MAX]. INT32_MAX is unity. Negative
///                  values are not supported.
/// @param samples_to_scale Number of samples (not frames) to scale.
/// @param bytes_per_sample Sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale, size_t samples_to_scale,
           size_t bytes_per_sample);

/// @brief Stateful gain smoother that ramps a Q31 gain toward a target over a fixed sample count.
///
/// The ramp walks a 1 dB grid (steady perceived rate), fills each step linearly in constant-factor sub-blocks,
/// and lands exactly on the target. Silence (Q31 0) is not on the grid: a fade to silence steps down to
/// -100 dB and then runs one more segment, the same length as a 1 dB step, linearly to 0. A fade in from
/// silence (or from anywhere below -100 dB) mirrors that: one linear segment up to -100 dB, then 1 dB steps.
/// Retargeting mid-ramp continues from the live value. Integer math only, so safe in an audio task.
///
/// Default-constructed: settled at unity.
class GainRamp {
 public:
  /// @brief Points the ramp at a new target, starting from the live value.
  ///
  /// Settled at the target already is a no-op. While a ramp is in flight, calling with the same target
  /// reschedules it: the remaining distance is covered in ramp_samples from now. ramp_samples == 0, or
  /// too few samples to give each 1 dB step one sample, changes immediately. Small and large jumps both
  /// take about ramp_samples: the length is rounded down to a whole number of segments, so the ramp
  /// can settle up to (segments - 1) samples early.
  ///
  /// @param target_q31 Target Q31 gain in [0, INT32_MAX].
  /// @param ramp_samples Ramp length in samples (interleaved count, matching process()).
  void set_target(int32_t target_q31, uint32_t ramp_samples);

  /// @brief set_target() with the target given as an integer dB reduction below unity.
  ///
  /// 0 is unity; 173 and above are silence. See db_reduction_to_q31(). Integer math only. The last
  /// conversion is cached, so calling every block with an unchanged level costs one compare.
  void set_target_db_reduction(uint8_t db, uint32_t ramp_samples);

  /// @brief Applies the ramp to a block in place, advancing the live value toward the target.
  ///
  /// Any tail beyond the ramp is scaled by the settled target. Settled at unity is a no-op.
  ///
  /// @param buffer Interleaved samples to scale in place.
  /// @param bytes_per_sample Sample width in bytes: 1, 2, 3, or 4.
  /// @param samples Number of samples (not frames) in the buffer.
  void process(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples);

  /// @brief Live Q31 gain. INT32_MAX is unity.
  int32_t current_q31() const { return this->current_q31_; }
  /// @brief Q31 gain the ramp is heading to or settled at.
  int32_t target_q31() const { return this->target_q31_; }
  /// @brief True while a ramp is in progress.
  bool is_ramping() const { return this->samples_remaining_ > 0; }

 private:
  int32_t current_q31_{INT32_MAX};
  int32_t target_q31_{INT32_MAX};
  int32_t seg_target_q31_{INT32_MAX};  ///< End of the 1 dB segment in progress.
  uint32_t samples_remaining_{0};      ///< 0 means settled.
  uint32_t samples_per_step_{0};       ///< Samples per 1 dB segment.
  uint8_t last_db_{0};                 ///< Last dB passed to set_target_db_reduction().
  int32_t last_db_q31_{INT32_MAX};     ///< db_reduction_to_q31(last_db_).
};

}  // namespace gain
}  // namespace esp_audio_libs
