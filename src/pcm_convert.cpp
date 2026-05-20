#include "pcm_convert.h"

#include <algorithm>
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

template<size_t InputBps, size_t OutputBps, bool Aligned>
EAL_HOT void copy_frames_generic(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t input_channels,
                                 uint8_t output_channels, uint32_t frames) {
  const uint8_t max_input_channel_index = input_channels - 1;
  for (uint32_t frame = 0; frame < frames; ++frame) {
    for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
      const uint8_t in_ch = std::min<uint8_t>(out_ch, max_input_channel_index);
      const int32_t q31 = unpack<InputBps, Aligned>(in_ptr + in_ch * InputBps);
      pack<OutputBps, Aligned>(q31, out_ptr + out_ch * OutputBps);
    }
    in_ptr += input_channels * InputBps;
    out_ptr += output_channels * OutputBps;
  }
}

// Channel-count-specialized variant. Hard-coding the channel counts lets the compiler unroll the
// inner channel loop, leaving a single straight-line outer loop that Xtensa can lower to a
// hardware zero-overhead loop (LOOPNEZ/LOOPGTZ).
template<size_t InputBps, size_t OutputBps, uint8_t InputChannels, uint8_t OutputChannels, bool Aligned>
EAL_HOT void copy_frames_fixed_channels(const uint8_t *in_ptr, uint8_t *out_ptr, uint32_t frames) {
  constexpr uint8_t MAX_INPUT_CHANNEL_INDEX = InputChannels - 1;
  for (uint32_t frame = 0; frame < frames; ++frame) {
    for (uint8_t out_ch = 0; out_ch < OutputChannels; ++out_ch) {
      const uint8_t in_ch = std::min<uint8_t>(out_ch, MAX_INPUT_CHANNEL_INDEX);
      const int32_t q31 = unpack<InputBps, Aligned>(in_ptr + in_ch * InputBps);
      pack<OutputBps, Aligned>(q31, out_ptr + out_ch * OutputBps);
    }
    in_ptr += InputChannels * InputBps;
    out_ptr += OutputChannels * OutputBps;
  }
}

template<size_t InputBps, size_t OutputBps, bool Aligned>
EAL_HOT void copy_frames_dispatch_channels(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t input_channels,
                                           uint8_t output_channels, uint32_t frames) {
  // Matching channel counts are rewritten to the 1->1 path by copy_frames(), so only the
  // mismatched mono/stereo remaps need a fixed-channel specialization here. The 1->1 case still
  // arrives via that rewrite (it carries frames*channels samples).
  if (output_channels == 1) {
    if (input_channels == 1)
      return copy_frames_fixed_channels<InputBps, OutputBps, 1, 1, Aligned>(in_ptr, out_ptr, frames);
    if (input_channels == 2)
      return copy_frames_fixed_channels<InputBps, OutputBps, 2, 1, Aligned>(in_ptr, out_ptr, frames);
  } else if (output_channels == 2) {
    if (input_channels == 1)
      return copy_frames_fixed_channels<InputBps, OutputBps, 1, 2, Aligned>(in_ptr, out_ptr, frames);
  }
  // Fallback for unusual channel remaps (3+ channels in or out, with differing counts).
  copy_frames_generic<InputBps, OutputBps, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
}

template<size_t InputBps, bool Aligned>
void copy_frames_dispatch_output(const uint8_t *in_ptr, uint8_t *out_ptr, uint8_t output_bps, uint8_t input_channels,
                                 uint8_t output_channels, uint32_t frames) {
  switch (output_bps) {
    case 1:
      copy_frames_dispatch_channels<InputBps, 1, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
    case 2:
      copy_frames_dispatch_channels<InputBps, 2, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
    case 3:
      copy_frames_dispatch_channels<InputBps, 3, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
      break;
    case 4:
      copy_frames_dispatch_channels<InputBps, 4, Aligned>(in_ptr, out_ptr, input_channels, output_channels, frames);
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
  if (frames == 0 || input_bps < 1 || input_bps > 4 || output_bps < 1 || output_bps > 4) {
    return;
  }

  if (input_bps == output_bps && input_channels == output_channels) {
    std::memcpy(output, input, static_cast<size_t>(frames) * input_channels * input_bps);
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

  if (aligned) {
    copy_frames_dispatch_input<true>(input, output, input_bps, output_bps, input_channels, output_channels, frames);
  } else {
    copy_frames_dispatch_input<false>(input, output, input_bps, output_bps, input_channels, output_channels, frames);
  }
}

}  // namespace pcm_convert
}  // namespace esp_audio_libs
