#pragma once

/// @file pcm_convert.h
/// @brief Interleaved PCM conversion between bit depths and channel counts

#include <stddef.h>
#include <stdint.h>

namespace esp_audio_libs {
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

}  // namespace pcm_convert
}  // namespace esp_audio_libs
