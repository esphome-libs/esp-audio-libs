#pragma once

/// @file pcm_convert.h
/// @brief Interleaved PCM conversion between bit depths and channel counts

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
namespace gain {
struct LinearGain;  // gain.h
}  // namespace gain

namespace pcm_convert {

/// @brief Converts interleaved PCM frames between bit depths and channel counts
///
/// Each output frame is built channel-by-channel. When the output has more channels than the
/// input, the extra channels reuse the last input channel; when it has fewer, the trailing input
/// channels are dropped. Bit-depth conversion goes through a Q31 intermediate: widening is exact,
/// narrowing rounds to nearest (the dropped low bits are rounded, not truncated; no dithering).
/// When the input and output formats match exactly this degenerates to a plain memcpy.
///
/// Sample format: signed PCM, little-endian for multi-byte widths, interleaved across channels.
/// 8-bit samples are interpreted as int8 (0x00 is silence), not WAV-style uint8.
///
/// The buffers must not overlap, with one supported exception: in-place conversion where `output`
/// exactly aliases `input` (same pointer). That is supported only when the channel count is
/// unchanged and the output is no wider than the input (`output_bps <= input_bps`); i.e.,
/// same-format (a no-op) or narrowing. Such a call is a single forward pass whose write pointer
/// never overtakes the still-unread input. An exactly-aliased call that is not in that supported
/// form (in-place widening or any change in channel count) is detected and returns without touching
/// the buffer, so it is a defined no-op rather than silent corruption. This safety net covers only
/// exact aliasing; partial overlap (an `output` offset into `input`) is undetectable and remains
/// unsupported (undefined behavior).
///
/// Misaligned pointers are handled correctly via a runtime dispatch, but aligning each pointer to
/// its sample width (2 bytes for 16-bit, 4 bytes for 32-bit) lets the function take the wide-load
/// fast path on architectures like Xtensa.
///
/// @note The total byte count `frames * input_channels * input_bps` must fit in `size_t`, and
/// `frames * input_channels` must fit in `uint32_t`. On 32-bit targets the first condition implies
/// the second and caps a single call at ~4 GiB of input, well beyond any realistic audio buffer.
/// On 64-bit hosts `size_t` is wider, so the caller must keep `frames * input_channels` within
/// `uint32_t` directly. The caller is responsible for not exceeding these limits.
///
/// @param input Source buffer of interleaved samples.
/// @param output Destination buffer. Must not overlap the input, except it may exactly alias it
/// (`output == input`) for same-format or narrowing conversions that keep the channel count.
/// @param input_bps Source sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
/// @param input_channels Number of source channels (must be >= 1).
/// @param output_bps Destination sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
/// @param output_channels Number of destination channels (must be >= 1).
/// @param frames Number of frames to convert. Zero is a no-op.
void copy_frames(const uint8_t *input, uint8_t *output, uint8_t input_bps, uint8_t input_channels, uint8_t output_bps,
                 uint8_t output_channels, uint32_t frames);

/// @brief Like copy_frames(), but picks input channels through a map and applies a gain that may amplify
///
/// Output channel `i` of each frame comes from input channel `channel_map[i]`, so channels can be
/// selected, reordered, or duplicated. A null map uses the copy_frames() layout rule instead.
///
/// The gain is applied to the full-precision Q31 sample before it is narrowed to the output width,
/// so amplifying a quiet signal keeps its low bits. Results beyond full scale saturate rather than
/// wrap. Narrowing rounds to nearest, as in copy_frames().
///
/// The multiply keeps only the high 32 bits of the product, so the gained Q31 sample can be low by
/// up to 2^(gain.exponent + 1). That is about 2 bits below a 32-bit input's resolution and far below
/// that of 24-bit or narrower sources. Results stay within 1 LSB of exact rounding for 16-bit output
/// up to an exponent of 15 (about +90 dB) and for 24-bit output up to 7 (about +42 dB).
///
/// A unity gain whose map selects consecutive input channels runs copy_frames() itself, so its
/// output matches copy_frames() exactly. A silent gain writes zeros.
///
/// Sample format, alignment, and size limits follow copy_frames(). The buffers must not overlap,
/// except that `output` may exactly alias `input` when the map is the identity (`channel_map[i] == i`,
/// or a null map), the channel counts match, and `output_bps <= input_bps`. Any other exactly-aliased
/// call returns without touching the buffer.
///
/// @param input Source buffer of interleaved samples.
/// @param output Destination buffer. See above for the one supported form of aliasing.
/// @param input_bps Source sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
/// @param input_channels Number of source channels (must be >= 1).
/// @param output_bps Destination sample width in bytes: 1, 2, 3, or 4. Other values are a no-op.
/// @param channel_map `output_channels` input channel indices, or nullptr. An index of
/// `input_channels` or more makes the call a no-op.
/// @param output_channels Number of destination channels (must be >= 1).
/// @param gain Gain to apply; see gain::db_to_linear_gain(). An exponent above 30 is a no-op.
/// @param frames Number of frames to convert. Zero is a no-op.
void copy_frames_with_gain(const uint8_t *input, uint8_t *output, uint8_t input_bps, uint8_t input_channels,
                           uint8_t output_bps, const uint8_t *channel_map, uint8_t output_channels,
                           const gain::LinearGain &gain, uint32_t frames);

}  // namespace pcm_convert
}  // namespace esp_audio_libs
