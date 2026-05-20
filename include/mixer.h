#pragma once

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
namespace mixer {

/// @brief Sums two interleaved PCM streams into an output buffer.
///
/// The two inputs and the output may each have a different bit depth and channel count. Channel
/// mapping follows the same rule as pcm_convert::copy_frames: extra output channels reuse the
/// last source channel, surplus source channels are dropped.
///
/// Mixing is done in Q30 (each Q31 input is shifted right by one before summing) so the running
/// total stays in a 32-bit register with no 64-bit arithmetic. The sum is clamped to the Q30
/// range, then shifted back to Q31 for output. The cost is one LSB of resolution per input;
/// negligible at every supported bit depth. Chain more than two streams by passing a previous
/// mix result back in as `primary`.
///
/// Sample format: signed PCM, little-endian for multi-byte widths, interleaved across channels.
/// The output buffer must not overlap either input. Misaligned pointers are handled correctly via
/// a runtime dispatch, but aligning each pointer to its sample width (2 bytes for 16-bit, 4 bytes
/// for 32-bit) lets the function take the wide-load fast path on architectures like Xtensa.
///
/// @param primary First source buffer.
/// @param primary_bps First source sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
/// @param primary_channels Number of channels in the first source (must be >= 1).
/// @param secondary Second source buffer.
/// @param secondary_bps Second source sample width in bytes: 1, 2, 3, or 4.
/// @param secondary_channels Number of channels in the second source (must be >= 1).
/// @param output Destination buffer (must not alias either input).
/// @param output_bps Destination sample width in bytes: 1, 2, 3, or 4.
/// @param output_channels Number of destination channels (must be >= 1).
/// @param frames Number of frames to mix.
void mix_frames(const uint8_t *primary, uint8_t primary_bps, uint8_t primary_channels, const uint8_t *secondary,
                uint8_t secondary_bps, uint8_t secondary_channels, uint8_t *output, uint8_t output_bps,
                uint8_t output_channels, uint32_t frames);

}  // namespace mixer
}  // namespace esp_audio_libs
