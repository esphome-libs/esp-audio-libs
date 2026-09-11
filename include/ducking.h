#pragma once

#include <stddef.h>
#include <stdint.h>

#include "gain.h"

namespace esp_audio_libs {
namespace ducking {

/// @brief Largest dB reduction supported by Ducker::set_target(). Larger requests are clamped here.
constexpr uint8_t MAX_DB_REDUCTION = 50;

/// @brief Ramped dB attenuation for one audio stream: a gain::GainRamp that speaks integer dB.
///
/// Create one per stream (default: not ducked). Call set_target() to schedule a level change, then
/// apply() on each block. Integer math only, so safe in an audio task.
class Ducker {
 public:
  /// @brief Schedules a new ducking level, optionally ramped over a number of samples.
  ///
  /// Ramps from the live level toward `decibel_reduction`, 1 dB at a time with each step filled
  /// linearly. `transition_samples == 0` (or too few for one sample per step) changes immediately.
  /// Same level as already in effect is a no-op, so it is safe to call every block.
  ///
  /// @param decibel_reduction Target attenuation in dB; clamped to [0, MAX_DB_REDUCTION].
  /// @param transition_samples Length of the ramp in samples (interleaved count, matching apply()).
  ///                           0 = instant. Rounded down to a whole number of 1 dB steps, see
  ///                           gain::GainRamp::set_target().
  void set_target(uint8_t decibel_reduction, uint32_t transition_samples);

  /// @brief Applies the current attenuation to a block of interleaved PCM samples in place.
  ///
  /// Advances any in-progress transition. Settled at 0 dB is a no-op.
  ///
  /// Sample format matches gain::apply: signed PCM, little-endian, interleaved; 8-bit is int8.
  ///
  /// @param buffer Interleaved samples to attenuate in place.
  /// @param bytes_per_sample Sample width in bytes: 1, 2, 3, or 4.
  /// @param samples Number of samples (not frames) in the buffer.
  void apply(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples) {
    this->ramp_.process(buffer, bytes_per_sample, samples);
  }

  /// @brief Attenuation in dB that set_target() is moving toward or settled at.
  uint8_t target_db_reduction() const { return this->target_db_reduction_; }
  /// @brief Live Q31 gain factor. INT32_MAX is unity (0 dB).
  int32_t current_q31() const { return this->ramp_.current_q31(); }
  /// @brief True while a transition is in progress.
  bool is_ramping() const { return this->ramp_.is_ramping(); }

 private:
  uint8_t target_db_reduction_{0};
  gain::GainRamp ramp_{};
};

}  // namespace ducking
}  // namespace esp_audio_libs
