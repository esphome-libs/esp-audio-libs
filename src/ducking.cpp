#include "ducking.h"

#include "q31_utils.h"

namespace esp_audio_libs {
namespace ducking {

namespace {

/// Q31 factor for an integer dB reduction, by iterated 1 dB steps. Only runs when the target changes.
int32_t db_reduction_to_q31(uint8_t db) {
  int32_t factor = INT32_MAX;  // 0 dB = unity gain
  for (uint8_t i = 0; i < db; ++i) {
    factor = internal::step_down_1db(factor);
  }
  return factor;
}

}  // namespace

void set_target(DuckingState &state, uint8_t decibel_reduction, uint32_t transition_samples) {
  if (decibel_reduction > MAX_DB_REDUCTION)
    decibel_reduction = MAX_DB_REDUCTION;

  if (state.target_db_reduction == decibel_reduction)
    return;

  state.target_db_reduction = decibel_reduction;
  state.ramp.set_target(db_reduction_to_q31(decibel_reduction), transition_samples);
}

void apply(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples, DuckingState &state) {
  state.ramp.process(buffer, bytes_per_sample, samples);
}

}  // namespace ducking
}  // namespace esp_audio_libs
