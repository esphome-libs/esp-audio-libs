#pragma once

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
namespace gain {

/// @brief Converts a gain in dB to a Q31 scale factor in [0, INT32_MAX].
///
/// Q31 cannot exceed unity: 0 dB and above clamp to INT32_MAX. Very negative dB and NaN return 0.
int32_t db_to_q31(float db);

/// @brief Converts an integer dB reduction to a Q31 scale factor using integer math only.
///
/// 0 is unity; 173 and above are silence. Results are exact points on GainRamp's 1 dB grid.
int32_t db_reduction_to_q31(uint8_t db);

/// @brief Scales each sample by a Q31 factor in [0, INT32_MAX]. Cannot amplify.
///
/// Samples are signed PCM, little-endian, interleaved; 8-bit is int8, not WAV-style uint8.
/// May operate in place. Buffers aligned to bytes_per_sample take a faster path with identical
/// output. Callers in hot paths should skip the call when q31_scale is unity.
/// @param audio_samples Input buffer.
/// @param output_buffer Output buffer (may alias the input).
/// @param q31_scale Scale factor; INT32_MAX is unity.
/// @param samples_to_scale Number of samples (not frames).
/// @param bytes_per_sample 1, 2, 3, or 4. Other values are a no-op.
void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale, size_t samples_to_scale,
           size_t bytes_per_sample);

/// @brief Ramps a Q31 gain toward a target over a fixed number of samples. Integer math only.
///
/// The ramp walks a 1 dB grid for a steady perceived rate. Fades to silence step down to -100 dB and
/// then run linearly to 0; fades in from silence mirror that. Default-constructed: settled at unity.
class GainRamp {
 public:
  /// @brief Ramps from the live value to target_q31 (in [0, INT32_MAX]) over ramp_samples.
  ///
  /// Settled at the target already is a no-op; a ramp already in flight is rescheduled. A ramp too
  /// short to give each 1 dB step one sample jumps immediately. ramp_samples counts interleaved
  /// samples, as process() does.
  void set_target(int32_t target_q31, uint32_t ramp_samples);

  /// @brief set_target() with the target as an integer dB reduction; see db_reduction_to_q31().
  void set_target_db_reduction(uint8_t db, uint32_t ramp_samples);

  /// @brief Scales a block in place, advancing the ramp. Settled at unity is a no-op.
  /// @param buffer Interleaved samples, same format as apply().
  /// @param bytes_per_sample 1, 2, 3, or 4.
  /// @param samples Number of samples (not frames).
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
