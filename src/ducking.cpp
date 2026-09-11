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

void Ducker::set_target(uint8_t decibel_reduction, uint32_t transition_samples) {
  if (decibel_reduction > MAX_DB_REDUCTION)
    decibel_reduction = MAX_DB_REDUCTION;

  if (this->target_db_reduction_ == decibel_reduction)
    return;

  this->target_db_reduction_ = decibel_reduction;
  this->ramp_.set_target(db_reduction_to_q31(decibel_reduction), transition_samples);
}

}  // namespace ducking
}  // namespace esp_audio_libs
