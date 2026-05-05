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
void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale,
           size_t samples_to_scale, size_t bytes_per_sample);

}  // namespace gain
}  // namespace esp_audio_libs
