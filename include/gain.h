#pragma once

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
namespace gain {

/// @brief Converts a linear gain in decibels to a Q31 scale factor in [0, INT32_MAX].
/// Q31 cannot represent values >= 1.0. 0 dB and above clamp to INT32_MAX, which is 1.0 - 2^-31.
/// Very negative dB rounds to 0. Use a different fixed-point format if you need positive dB gain.
/// @param db Gain in dB.
/// @return Q31 scale factor in the range [0, INT32_MAX].
int32_t db_to_q31(float db);

/// @brief Multiplies each sample by a Q31 scale factor in [0, INT32_MAX]. Cannot amplify.
/// May operate in-place when output_buffer == audio_samples. Rounding is applied for the 1, 2, and
/// 3 byte widths. The 4 byte width is truncated.
/// At unity (q31_scale == INT32_MAX) the output equals the input within one LSB of rounding loss.
/// Callers in hot paths should skip this call when q31_scale is unity.
/// For best throughput, both buffers should be aligned to bytes_per_sample. Misaligned buffers
/// fall back to a byte-wise slow path that produces identical output.
/// @param audio_samples Input buffer of interleaved samples.
/// @param output_buffer Output buffer (may alias the input).
/// @param q31_scale Q31 scale factor in the range [0, INT32_MAX]. INT32_MAX is unity. Negative
///                  values are not supported.
/// @param samples_to_scale Number of samples (not frames) to scale.
/// @param bytes_per_sample Sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale,
           size_t samples_to_scale, size_t bytes_per_sample);

}  // namespace gain
}  // namespace esp_audio_libs
