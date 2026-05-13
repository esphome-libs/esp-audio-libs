#include "ducking.h"

#include <algorithm>

#include "gain.h"

namespace esp_audio_libs {
namespace ducking {

namespace {

/// Q31 scale factor for an integer dB reduction in [0, MAX_DB_REDUCTION], computed by iterative
/// multiplication by 10^(-1/20) in Q32 fixed-point (3827893632 = 2 * round(10^(-1/20) * 2^31)).
/// The unsigned 32x32 -> high-32 multiply lowers to a single MULUH (Xtensa) / MULHU (RISC-V).
/// Pure integer math, so it is cheap to recompute per block.
int32_t db_reduction_to_q31(uint8_t db) {
  int32_t factor = INT32_MAX;  // 0 dB = unity gain
  for (uint8_t i = 0; i < db; ++i) {
    factor = static_cast<int32_t>((static_cast<uint64_t>(factor) * 3827893632ULL) >> 32);
  }
  return factor;
}

uint8_t clamp_db(int8_t db) {
  if (db < 0)
    return 0;
  if (db > static_cast<int8_t>(MAX_DB_REDUCTION))
    return MAX_DB_REDUCTION;
  return static_cast<uint8_t>(db);
}

}  // namespace

void set_target(DuckingState &state, uint8_t decibel_reduction, uint32_t transition_samples) {
  if (decibel_reduction > MAX_DB_REDUCTION)
    decibel_reduction = MAX_DB_REDUCTION;

  if (state.target_db_reduction == decibel_reduction)
    return;

  // Start the ramp from the previous target (which becomes the new current level).
  state.current_db_reduction = static_cast<int8_t>(state.target_db_reduction);
  state.target_db_reduction = decibel_reduction;

  // Number of intermediate 1 dB steps. Subtract 1 because the first step is taken immediately.
  uint8_t total_steps = 0;
  if (state.target_db_reduction > state.current_db_reduction) {
    total_steps = static_cast<uint8_t>(state.target_db_reduction - state.current_db_reduction - 1);
    state.db_change_per_step = 1;
  } else {
    total_steps = static_cast<uint8_t>(state.current_db_reduction - state.target_db_reduction - 1);
    state.db_change_per_step = -1;
  }

  if (transition_samples > 0 && total_steps > 0) {
    state.samples_per_step = transition_samples / total_steps;
    // Re-derive remaining samples so it is an exact multiple of samples_per_step.
    state.transition_samples_remaining = state.samples_per_step * total_steps;
    state.current_db_reduction += state.db_change_per_step;
  } else {
    state.transition_samples_remaining = 0;
    state.current_db_reduction = static_cast<int8_t>(state.target_db_reduction);
  }
}

void apply(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples, DuckingState &state) {
  // Safety guard: a transition must have a nonzero step size.
  if (state.transition_samples_remaining > 0 && state.samples_per_step == 0)
    state.transition_samples_remaining = 0;

  if (state.transition_samples_remaining > 0) {
    // Ceiling of samples / samples_per_step.
    uint32_t steps_in_batch = samples / state.samples_per_step + (samples % state.samples_per_step != 0 ? 1 : 0);

    for (uint32_t i = 0; i < steps_in_batch; ++i) {
      uint32_t samples_left_in_step = state.transition_samples_remaining % state.samples_per_step;
      if (samples_left_in_step == 0)
        samples_left_in_step = state.samples_per_step;

      uint32_t chunk = std::min(samples, samples_left_in_step);
      chunk = std::min(chunk, state.transition_samples_remaining);

      const int32_t q31 = db_reduction_to_q31(clamp_db(state.current_db_reduction));
      gain::apply(buffer, buffer, q31, chunk, bytes_per_sample);

      if (samples_left_in_step == chunk)
        state.current_db_reduction += state.db_change_per_step;

      buffer += chunk * bytes_per_sample;
      state.transition_samples_remaining -= chunk;
      samples -= chunk;
    }
  }

  if (state.current_db_reduction > 0 && samples > 0) {
    // Settled (or settled enough): one scale for the rest of the block.
    const int32_t q31 = db_reduction_to_q31(clamp_db(state.current_db_reduction));
    gain::apply(buffer, buffer, q31, samples, bytes_per_sample);
  }
}

}  // namespace ducking
}  // namespace esp_audio_libs
