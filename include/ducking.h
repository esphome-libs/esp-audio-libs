#pragma once

#include <stddef.h>
#include <stdint.h>

#include "gain.h"

namespace esp_audio_libs {
namespace ducking {

/// @brief Largest dB reduction supported by set_target(). Larger requests are clamped here.
constexpr uint8_t MAX_DB_REDUCTION = 50;

/// @brief Mutable ducking state for one audio stream.
///
/// Create one per stream (defaults: not ducked). Call set_target() to schedule a level change, then
/// apply() on each block. Only target_db_reduction is meant to be read.
struct DuckingState {
  uint8_t target_db_reduction{0};  ///< Attenuation in dB that set_target() is moving toward.
  gain::GainRamp ramp{};           ///< Carries the live level and the transition.
};

/// @brief Schedules a new ducking level, optionally ramped over a number of samples.
///
/// Ramps from the live level toward `decibel_reduction`, 1 dB at a time with each step filled
/// linearly. `transition_samples == 0` (or too few for one sample per step) changes immediately.
/// Same level as already in effect is a no-op. Integer math only.
///
/// @param state Ducking state to update.
/// @param decibel_reduction Target attenuation in dB; clamped to [0, MAX_DB_REDUCTION].
/// @param transition_samples Length of the ramp in samples (per channel-interleaved sample count
///                           is fine as long as it matches what is passed to apply()). 0 = instant.
void set_target(DuckingState &state, uint8_t decibel_reduction, uint32_t transition_samples);

/// @brief Applies the current ducking attenuation to a block of interleaved PCM samples in place.
///
/// Advances any in-progress transition. Settled at 0 dB is a no-op.
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
