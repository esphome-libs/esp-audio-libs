#include "pcm_convert.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "compiler.h"
#include "q31_utils.h"

namespace esp_audio_libs {
namespace pcm_convert {

namespace {

using internal::fast_pack_q31;
using internal::fast_unpack_to_q31;
using internal::pack_q31;
using internal::unpack_to_q31;

// Pick the fast (aligned wide-load) or byte-wise variant at compile time. The `Aligned` template
// flag is set by the runtime alignment check in copy_frames(); both branches fold away at -O2.
template<size_t Bps, bool Aligned> EAL_HOT inline int32_t unpack(const uint8_t *data) {
  if (Aligned) {
    return fast_unpack_to_q31<Bps>(data);
  }
  return unpack_to_q31<Bps>(data);
}

template<size_t Bps, bool Aligned> EAL_HOT inline void pack(int32_t sample, uint8_t *data) {
  if (Aligned) {
    fast_pack_q31<Bps>(sample, data);
  } else {
    pack_q31<Bps>(sample, data);
  }
}

// Round a left-justified Q31 sample for storage as OutputBps bytes. pack() drops the low
// (4 - OutputBps) bytes by truncation; adding half the discarded range first rounds to nearest.
// Near-full-scale samples must not wrap when the round term pushes them past +full-scale. Rather
// than a compare-and-branch saturating add (which breaks the unrolled loop's scheduling on the
// in-order pipeline), clamp the input down to (INT32_MAX - term) with a single branchless min so the
// unconditional add that follows can never overflow int32. This is bit-identical to a saturating add
// (a sample already at the cap lands exactly on INT32_MAX) and needs no shifts. Compile-time no-op
// when OutputBps >= InputBps: the dropped bits are already zero (unpack left-justifies), so widening
// and same-width conversions pay nothing and only narrowing rounds.
template<size_t InputBps, size_t OutputBps> EAL_HOT inline int32_t round_for_output(int32_t sample) {
  // TODO(C++17): make this `if constexpr` so the OutputBps >= InputBps branch is discarded rather
  // than relying on -O2 to elide it; the shift > 0 guard below could then drop to plain `shift - 1`.
  if (OutputBps < InputBps) {
    constexpr int shift = (4 - static_cast<int>(OutputBps)) * 8;
    // Clamp so the unused (OutputBps == 4, shift == 0) branch forms no negative shift.
    constexpr int32_t term = INT32_C(1) << (shift > 0 ? shift - 1 : 0);
    return std::min(sample, INT32_MAX - term) + term;
  }
  return sample;
}

// The conversion loop for every channel layout. Each output channel maps to a fixed input channel,
// so the channel loop is outermost and the frame loop innermost.
template<size_t InputBps, size_t OutputBps, bool Aligned>
EAL_HOT void copy_frames_generic(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t input_channels,
                                 uint8_t output_channels, uint32_t frames) {
  const uint8_t max_input_channel_index = input_channels - 1;
  const size_t in_stride = static_cast<size_t>(input_channels) * InputBps;
  const size_t out_stride = static_cast<size_t>(output_channels) * OutputBps;
  for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
    const uint8_t in_ch = std::min<uint8_t>(out_ch, max_input_channel_index);
    const uint8_t *in = in_ptr + in_ch * InputBps;
    uint8_t *out = out_ptr + out_ch * OutputBps;
    for (uint32_t frame = 0; frame < frames; ++frame) {
      pack<OutputBps, Aligned>(round_for_output<InputBps, OutputBps>(unpack<InputBps, Aligned>(in)), out);
      in += in_stride;
      out += out_stride;
    }
  }
}

template<size_t InputBps, bool Aligned>
void copy_frames_dispatch_output(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t output_bps, uint8_t input_channels,
                                 uint8_t output_channels, uint32_t frames) {
  switch (output_bps) {
    case 1:
      copy_frames_generic<InputBps, 1, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
    case 2:
      copy_frames_generic<InputBps, 2, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
    case 3:
      copy_frames_generic<InputBps, 3, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
    case 4:
      copy_frames_generic<InputBps, 4, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
  }
}

template<bool Aligned>
void copy_frames_dispatch_input(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t input_bps, uint8_t output_bps,
                                uint8_t input_channels, uint8_t output_channels, uint32_t frames) {
  switch (input_bps) {
    case 1:
      copy_frames_dispatch_output<1, Aligned>(in_ptr, out_ptr, output_bps, input_channels, output_channels, frames);
      break;
    case 2:
      copy_frames_dispatch_output<2, Aligned>(in_ptr, out_ptr, output_bps, input_channels, output_channels, frames);
      break;
    case 3:
      copy_frames_dispatch_output<3, Aligned>(in_ptr, out_ptr, output_bps, input_channels, output_channels, frames);
      break;
    case 4:
      copy_frames_dispatch_output<4, Aligned>(in_ptr, out_ptr, output_bps, input_channels, output_channels, frames);
      break;
  }
}

template<size_t InputBps, size_t OutputBps>
EAL_HOT void copy_frames_mono_to_stereo(const uint8_t *in_ptr, uint8_t *out_ptr, uint32_t frames) {
  uint32_t frame = 0;
  const uint32_t frames_unrolled = frames & ~7u;  // unroll by 8
  for (; frame < frames_unrolled; frame += 8) {
    const int32_t sample1 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr));
    const int32_t sample2 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + InputBps));
    const int32_t sample3 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 2 * InputBps));
    const int32_t sample4 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 3 * InputBps));
    const int32_t sample5 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 4 * InputBps));
    const int32_t sample6 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 5 * InputBps));
    const int32_t sample7 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 6 * InputBps));
    const int32_t sample8 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 7 * InputBps));
    fast_pack_q31<OutputBps>(sample1, out_ptr);
    fast_pack_q31<OutputBps>(sample1, out_ptr + OutputBps);
    fast_pack_q31<OutputBps>(sample2, out_ptr + 2 * OutputBps);
    fast_pack_q31<OutputBps>(sample2, out_ptr + 3 * OutputBps);
    fast_pack_q31<OutputBps>(sample3, out_ptr + 4 * OutputBps);
    fast_pack_q31<OutputBps>(sample3, out_ptr + 5 * OutputBps);
    fast_pack_q31<OutputBps>(sample4, out_ptr + 6 * OutputBps);
    fast_pack_q31<OutputBps>(sample4, out_ptr + 7 * OutputBps);
    fast_pack_q31<OutputBps>(sample5, out_ptr + 8 * OutputBps);
    fast_pack_q31<OutputBps>(sample5, out_ptr + 9 * OutputBps);
    fast_pack_q31<OutputBps>(sample6, out_ptr + 10 * OutputBps);
    fast_pack_q31<OutputBps>(sample6, out_ptr + 11 * OutputBps);
    fast_pack_q31<OutputBps>(sample7, out_ptr + 12 * OutputBps);
    fast_pack_q31<OutputBps>(sample7, out_ptr + 13 * OutputBps);
    fast_pack_q31<OutputBps>(sample8, out_ptr + 14 * OutputBps);
    fast_pack_q31<OutputBps>(sample8, out_ptr + 15 * OutputBps);
    in_ptr += 8 * InputBps;
    out_ptr += 16 * OutputBps;
  }
  for (; frame < frames; ++frame) {
    const int32_t sample = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr));
    fast_pack_q31<OutputBps>(sample, out_ptr);
    fast_pack_q31<OutputBps>(sample, out_ptr + OutputBps);
    in_ptr += InputBps;
    out_ptr += 2 * OutputBps;
  }
}

template<size_t InputBps>
void copy_frames_mono_to_stereo_dispatch_output(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t output_bps,
                                                uint32_t frames) {
  // 1-byte widths are not specialized here; copy_frames() routes them to the generic loop.
  switch (output_bps) {
    case 2:
      copy_frames_mono_to_stereo<InputBps, 2>(in_ptr, out_ptr, frames);
      break;
    case 3:
      copy_frames_mono_to_stereo<InputBps, 3>(in_ptr, out_ptr, frames);
      break;
    case 4:
      copy_frames_mono_to_stereo<InputBps, 4>(in_ptr, out_ptr, frames);
      break;
  }
}

void copy_frames_mono_to_stereo_dispatch(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t input_bps, uint8_t output_bps,
                                         uint32_t frames) {
  switch (input_bps) {
    case 2:
      copy_frames_mono_to_stereo_dispatch_output<2>(in_ptr, out_ptr, output_bps, frames);
      break;
    case 3:
      copy_frames_mono_to_stereo_dispatch_output<3>(in_ptr, out_ptr, output_bps, frames);
      break;
    case 4:
      copy_frames_mono_to_stereo_dispatch_output<4>(in_ptr, out_ptr, output_bps, frames);
      break;
  }
}

// Whether a 1->1 conversion of these widths uses the byte-wise unpack/pack (any 3-byte side) rather
// than the wide load/store (both sides 2 or 4 byte). Byte-wise sides need many more registers per
// sample, so they take a shallower unroll.
template<size_t InputBps, size_t OutputBps> struct mono_bytewise {
  static constexpr bool value = (InputBps == 3 || OutputBps == 3);
};

// Flat 1->1 bit-depth conversion; carries the bulk of narrowing/widening throughput. The unrolled
// body loads into named locals (not an int32_t[], which spills to the stack) then stores in a
// separate phase, exposing independent chains the in-order pipeline can overlap. Only the wide-load
// 2/4-byte widths take this deep 8-wide unroll; byte-wise 3-byte uses copy_frames_mono_narrow, whose
// shallower unroll avoids spilling the register window.
template<size_t InputBps, size_t OutputBps>
EAL_HOT void copy_frames_mono(const uint8_t *in_ptr, uint8_t *out_ptr, uint32_t frames) {
  uint32_t frame = 0;
  const uint32_t frames_unrolled = frames & ~7u;  // unroll by 8
  for (; frame < frames_unrolled; frame += 8) {
    const int32_t sample1 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr));
    const int32_t sample2 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + InputBps));
    const int32_t sample3 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 2 * InputBps));
    const int32_t sample4 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 3 * InputBps));
    const int32_t sample5 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 4 * InputBps));
    const int32_t sample6 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 5 * InputBps));
    const int32_t sample7 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 6 * InputBps));
    const int32_t sample8 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + 7 * InputBps));
    fast_pack_q31<OutputBps>(sample1, out_ptr);
    fast_pack_q31<OutputBps>(sample2, out_ptr + OutputBps);
    fast_pack_q31<OutputBps>(sample3, out_ptr + 2 * OutputBps);
    fast_pack_q31<OutputBps>(sample4, out_ptr + 3 * OutputBps);
    fast_pack_q31<OutputBps>(sample5, out_ptr + 4 * OutputBps);
    fast_pack_q31<OutputBps>(sample6, out_ptr + 5 * OutputBps);
    fast_pack_q31<OutputBps>(sample7, out_ptr + 6 * OutputBps);
    fast_pack_q31<OutputBps>(sample8, out_ptr + 7 * OutputBps);
    in_ptr += 8 * InputBps;
    out_ptr += 8 * OutputBps;
  }
  for (; frame < frames; ++frame) {
    const int32_t sample = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr));
    fast_pack_q31<OutputBps>(sample, out_ptr);
    in_ptr += InputBps;
    out_ptr += OutputBps;
  }
}

// Shallow (2-wide) unroll for the byte-wise 3-byte widths: the byte-wise unpack/pack needs several
// registers per sample, so an 8-wide unroll spills the window. Two independent chains still give the
// in-order pipeline something to overlap without overflowing the registers.
template<size_t InputBps, size_t OutputBps>
EAL_HOT void copy_frames_mono_narrow(const uint8_t *in_ptr, uint8_t *out_ptr, uint32_t frames) {
  uint32_t frame = 0;
  const uint32_t frames_unrolled = frames & ~1u;  // unroll by 2
  for (; frame < frames_unrolled; frame += 2) {
    const int32_t sample1 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr));
    const int32_t sample2 = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr + InputBps));
    fast_pack_q31<OutputBps>(sample1, out_ptr);
    fast_pack_q31<OutputBps>(sample2, out_ptr + OutputBps);
    in_ptr += 2 * InputBps;
    out_ptr += 2 * OutputBps;
  }
  for (; frame < frames; ++frame) {
    const int32_t sample = round_for_output<InputBps, OutputBps>(fast_unpack_to_q31<InputBps>(in_ptr));
    fast_pack_q31<OutputBps>(sample, out_ptr);
    in_ptr += InputBps;
    out_ptr += OutputBps;
  }
}

template<size_t InputBps, size_t OutputBps>
inline void copy_frames_mono_select(const uint8_t *in_ptr, uint8_t *out_ptr, uint32_t frames) {
  if (mono_bytewise<InputBps, OutputBps>::value) {
    copy_frames_mono_narrow<InputBps, OutputBps>(in_ptr, out_ptr, frames);
  } else {
    copy_frames_mono<InputBps, OutputBps>(in_ptr, out_ptr, frames);
  }
}

template<size_t InputBps>
void copy_frames_mono_dispatch_output(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t output_bps, uint32_t frames) {
  switch (output_bps) {
    case 2:
      copy_frames_mono_select<InputBps, 2>(in_ptr, out_ptr, frames);
      break;
    case 3:
      copy_frames_mono_select<InputBps, 3>(in_ptr, out_ptr, frames);
      break;
    case 4:
      copy_frames_mono_select<InputBps, 4>(in_ptr, out_ptr, frames);
      break;
  }
}

void copy_frames_mono_dispatch(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t input_bps, uint8_t output_bps,
                               uint32_t frames) {
  switch (input_bps) {
    case 2:
      copy_frames_mono_dispatch_output<2>(in_ptr, out_ptr, output_bps, frames);
      break;
    case 3:
      copy_frames_mono_dispatch_output<3>(in_ptr, out_ptr, output_bps, frames);
      break;
    case 4:
      copy_frames_mono_dispatch_output<4>(in_ptr, out_ptr, output_bps, frames);
      break;
  }
}

// Required alignment for a sample width: 2 bytes for 16-bit (mask 0x1), 4 bytes for 32-bit (mask
// 0x3). 1- and 3-byte widths are byte-wise and have no alignment requirement.
inline uintptr_t alignment_mask(uint8_t bps) {
  if (bps == 2) return 0x1;
  if (bps == 4) return 0x3;
  return 0;
}

}  // namespace

void copy_frames(const uint8_t *input, uint8_t *output, uint8_t input_bps, uint8_t input_channels, uint8_t output_bps,
                 uint8_t output_channels, uint32_t frames) {
  if (frames == 0 || input_channels == 0 || output_channels == 0 || input_bps < 1 || input_bps > 4 ||
      output_bps < 1 || output_bps > 4) {
    return;
  }

  // In-place conversion (output exactly aliases input) is supported only when the channel layout is
  // unchanged and the output is no wider than the input: that conversion is a single forward pass
  // whose output stride is no larger than its input stride, so the write pointer can never overtake
  // the still-unread input. Widening or a channel-count change would overwrite input bytes a later
  // read still needs and corrupt the buffer. This only catches exact aliasing; partial overlap (output 
  // offset into input) is undetectable here and remains the caller's responsibility (see the header).
  if (input == output && !(input_channels == output_channels && output_bps <= input_bps)) {
    return;
  }

  if (input_bps == output_bps && input_channels == output_channels) {
    // memcpy with src == dst is undefined behavior even though it is a no-op in practice; skip it
    // for the in-place same-format call, which has nothing to do anyway.
    if (input != output) {
      std::memcpy(output, input, static_cast<size_t>(frames) * input_channels * input_bps);
    }
    return;
  }

  // When the channel counts match, bit-depth conversion is purely per-sample and the interleaved
  // channel layout is irrelevant: treat the buffers as a flat run of frames*channels mono samples
  // and convert each independently. This lets every matching-channel case (including 3+ channels)
  // reuse the unrolled 1->1 path instead of needing its own specialization or the generic loop.
  // frames*channels stays within uint32_t by the documented precondition: on 32-bit targets it
  // follows from the byte-count-fits-size_t rule; on 64-bit hosts the caller guarantees it directly.
  if (input_channels == output_channels) {
    frames *= input_channels;
    input_channels = 1;
    output_channels = 1;
  }

  // Pick the aligned fast path only if each buffer satisfies the alignment its own sample width
  // requires; otherwise fall back to the byte-wise path.
  const bool aligned = (reinterpret_cast<uintptr_t>(input) & alignment_mask(input_bps)) == 0 &&
                       (reinterpret_cast<uintptr_t>(output) & alignment_mask(output_bps)) == 0;

  // Mono to stereo on aligned buffers with 2-, 3-, or 4-byte samples takes a dedicated broadcast
  // that unpacks each input sample once and stores it to both output channels. Every other layout,
  // 1-byte widths, and all misaligned buffers go through the generic loop.
  if (aligned && input_channels == 1 && output_channels == 2 && input_bps >= 2 && output_bps >= 2) {
    copy_frames_mono_to_stereo_dispatch(input, output, input_bps, output_bps, frames);
    return;
  }

  // Flat 1->1 conversion (matching-channel cases were flattened above) takes the unrolled mono path
  // for 2-, 3-, and 4-byte widths. The unroll depth is chosen per width (deep for the wide-load 2-
  // and 4-byte widths, shallow for byte-wise 3-byte) so 3-byte does not spill its register window.
  // 1-byte widths and misaligned buffers fall through to the generic loop.
  if (aligned && input_channels == 1 && output_channels == 1 && input_bps >= 2 && output_bps >= 2) {
    copy_frames_mono_dispatch(input, output, input_bps, output_bps, frames);
    return;
  }

  if (aligned) {
    copy_frames_dispatch_input<true>(input, output, input_bps, output_bps, input_channels, output_channels, frames);
  } else {
    copy_frames_dispatch_input<false>(input, output, input_bps, output_bps, input_channels, output_channels, frames);
  }
}

}  // namespace pcm_convert
}  // namespace esp_audio_libs
