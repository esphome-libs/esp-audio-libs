#pragma once

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
namespace ducking {

/// @brief Largest dB reduction supported by set_target(). Larger requests are clamped here.
constexpr uint8_t MAX_DB_REDUCTION = 50;

/// @brief Mutable ducking state for one audio stream.
///
/// Create one per stream, zero-initialized (the defaults below mean "not ducked, no transition").
/// Call set_target() to schedule a level change, then apply() on each block of samples.
struct DuckingState {
  // Current attenuation in dB. While a transition is in progress this steps toward the target.
  int8_t current_db_reduction{0};
  // The level set_target() is moving toward. Equal to current_db_reduction when settled.
  uint8_t target_db_reduction{0};
  // Samples left in the active transition. 0 means no transition is in progress.
  uint32_t transition_samples_remaining{0};
  // Samples between successive 1 dB steps during a transition.
  uint32_t samples_per_step{0};
  // Direction of each step: +1 while getting quieter, -1 while getting louder.
  int8_t db_change_per_step{0};
};

/// @brief Schedules a new ducking level, optionally ramped over a number of samples.
///
/// If the requested level differs from the current target, the current target becomes the new
/// starting point and the state ramps 1 dB at a time toward `decibel_reduction`. With
/// `transition_samples == 0` (or no intermediate steps needed) the change is immediate. Calling
/// this with the level already in effect is a no-op, so it is safe to call every block.
///
/// @param state Ducking state to update.
/// @param decibel_reduction Target attenuation in dB; clamped to [0, MAX_DB_REDUCTION].
/// @param transition_samples Length of the ramp in samples (per channel-interleaved sample count
///                           is fine as long as it matches what is passed to apply()). 0 = instant.
void set_target(DuckingState &state, uint8_t decibel_reduction, uint32_t transition_samples);

/// @brief Applies the current ducking attenuation to a block of interleaved PCM samples in place.
///
/// Advances any in-progress transition as it goes, scaling each sub-range by the appropriate
/// fixed-point factor (via gain::apply). When the state is settled at 0 dB this is a no-op.
///
/// Sample format matches gain::apply: signed PCM, little-endian, interleaved; 8-bit is int8.
///
/// @param buffer Interleaved samples to attenuate in place.
/// @param bytes_per_sample Sample width in bytes: 1, 2, 3, or 4.
/// @param samples Number of samples (not frames) in the buffer.
/// @param state Ducking state; updated to reflect any transition progress.
void apply(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples, DuckingState &state);

}  // namespace ducking
}  // namespace esp_audio_libs
