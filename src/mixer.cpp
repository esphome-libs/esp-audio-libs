#include "mixer.h"

#include <algorithm>

#include "compiler.h"
#include "q31_utils.h"

namespace esp_audio_libs {
namespace mixer {

namespace {

using internal::fast_pack_q31;
using internal::fast_unpack_to_q31;
using internal::pack_q31;
using internal::unpack_to_q31;

// Mix in Q30 so the sum stays in a 32-bit register; the right shift loses 1 LSB per input.
constexpr int32_t Q30_MIN = -(INT32_C(1) << 30);
constexpr int32_t Q30_MAX = (INT32_C(1) << 30) - 1;

// Sum two Q31 inputs into a saturated Q31 output. Each input is halved before adding so the sum
// stays in int32; the result is clamped to the Q30 range and shifted back to Q31. The shift goes
// through uint32_t because left-shifting a negative signed value is undefined before C++20; the
// clamp guarantees the shifted result still fits in int32.
EAL_HOT inline int32_t mix_q31(int32_t primary_q31, int32_t secondary_q31) {
  const int32_t sum_q30 = (primary_q31 >> 1) + (secondary_q31 >> 1);
  const int32_t clamped_q30 = std::min(std::max(sum_q30, Q30_MIN), Q30_MAX);
  return static_cast<int32_t>(static_cast<uint32_t>(clamped_q30) << 1);
}

// Pick the fast (aligned wide-load) or byte-wise variant at compile time. The `Aligned` template
// flag is set by the runtime alignment check in mix_frames(); both branches fold away at -O2.
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

// Hot path: the two inputs and the output all share one sample width. Templating on a single `Bps`
// (instead of the full primary x secondary x output matrix) keeps this fully inlined and a
// zero-overhead-loop candidate while collapsing the instantiation count from 4*4*4 to 4 per
// alignment variant.
template<size_t Bps, bool Aligned>
EAL_HOT void mix_frames_same_bps(const uint8_t *pri_ptr, uint8_t primary_channels, const uint8_t *sec_ptr,
                                 uint8_t secondary_channels, uint8_t *out_ptr, uint8_t output_channels,
                                 uint32_t frames) {
  const uint8_t max_primary_channel_index = primary_channels - 1;
  const uint8_t max_secondary_channel_index = secondary_channels - 1;

  for (uint32_t frame = 0; frame < frames; ++frame) {
    for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
      const uint8_t pri_ch = std::min<uint8_t>(out_ch, max_primary_channel_index);
      const int32_t primary_q31 = unpack<Bps, Aligned>(pri_ptr + pri_ch * Bps);

      const uint8_t sec_ch = std::min<uint8_t>(out_ch, max_secondary_channel_index);
      const int32_t secondary_q31 = unpack<Bps, Aligned>(sec_ptr + sec_ch * Bps);

      pack<Bps, Aligned>(mix_q31(primary_q31, secondary_q31), out_ptr + out_ch * Bps);
    }
    pri_ptr += primary_channels * Bps;
    sec_ptr += secondary_channels * Bps;
    out_ptr += output_channels * Bps;
  }
}

// Runtime-width unpack for the mismatched-width path. A width switch on a loop-invariant `bps`
// (well predicted) replaces the per-input-width template axis, so only the output width and the
// alignment flag stay as template parameters. The `Aligned` flag selects the wide-load variant;
// callers guarantee each buffer is aligned to its own width before picking it.
template<bool Aligned> EAL_HOT inline int32_t unpack_runtime(const uint8_t *data, uint8_t bps) {
  switch (bps) {
    case 1:
      return unpack<1, Aligned>(data);
    case 2:
      return unpack<2, Aligned>(data);
    case 3:
      return unpack<3, Aligned>(data);
    default:
      return unpack<4, Aligned>(data);
  }
}

// Mismatched-width path: the three buffers do not share one sample width. The output width and
// alignment are template parameters (so the per-output-sample pack is the inlined wide store and
// carries no switch), while the two input widths are resolved by the runtime switch above.
template<size_t OutputBps, bool Aligned>
EAL_HOT void mix_frames_generic(const uint8_t *pri_ptr, uint8_t primary_bps, uint8_t primary_channels,
                                const uint8_t *sec_ptr, uint8_t secondary_bps, uint8_t secondary_channels,
                                uint8_t *out_ptr, uint8_t output_channels, uint32_t frames) {
  const uint8_t max_primary_channel_index = primary_channels - 1;
  const uint8_t max_secondary_channel_index = secondary_channels - 1;

  for (uint32_t frame = 0; frame < frames; ++frame) {
    for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
      const uint8_t pri_ch = std::min<uint8_t>(out_ch, max_primary_channel_index);
      const int32_t primary_q31 = unpack_runtime<Aligned>(pri_ptr + pri_ch * primary_bps, primary_bps);

      const uint8_t sec_ch = std::min<uint8_t>(out_ch, max_secondary_channel_index);
      const int32_t secondary_q31 = unpack_runtime<Aligned>(sec_ptr + sec_ch * secondary_bps, secondary_bps);

      pack<OutputBps, Aligned>(mix_q31(primary_q31, secondary_q31), out_ptr + out_ch * OutputBps);
    }
    pri_ptr += primary_channels * primary_bps;
    sec_ptr += secondary_channels * secondary_bps;
    out_ptr += output_channels * OutputBps;
  }
}

template<bool Aligned>
void mix_frames_dispatch_generic(uint8_t output_bps, const uint8_t *pri_ptr, uint8_t primary_bps,
                                 uint8_t primary_channels, const uint8_t *sec_ptr, uint8_t secondary_bps,
                                 uint8_t secondary_channels, uint8_t *out_ptr, uint8_t output_channels,
                                 uint32_t frames) {
  switch (output_bps) {
    case 1:
      mix_frames_generic<1, Aligned>(pri_ptr, primary_bps, primary_channels, sec_ptr, secondary_bps,
                                     secondary_channels, out_ptr, output_channels, frames);
      break;
    case 2:
      mix_frames_generic<2, Aligned>(pri_ptr, primary_bps, primary_channels, sec_ptr, secondary_bps,
                                     secondary_channels, out_ptr, output_channels, frames);
      break;
    case 3:
      mix_frames_generic<3, Aligned>(pri_ptr, primary_bps, primary_channels, sec_ptr, secondary_bps,
                                     secondary_channels, out_ptr, output_channels, frames);
      break;
    case 4:
      mix_frames_generic<4, Aligned>(pri_ptr, primary_bps, primary_channels, sec_ptr, secondary_bps,
                                     secondary_channels, out_ptr, output_channels, frames);
      break;
  }
}

template<bool Aligned>
void mix_frames_dispatch_same_bps(uint8_t bps, const uint8_t *pri_ptr, uint8_t primary_channels,
                                  const uint8_t *sec_ptr, uint8_t secondary_channels, uint8_t *out_ptr,
                                  uint8_t output_channels, uint32_t frames) {
  switch (bps) {
    case 1:
      mix_frames_same_bps<1, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels, out_ptr,
                                      output_channels, frames);
      break;
    case 2:
      mix_frames_same_bps<2, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels, out_ptr,
                                      output_channels, frames);
      break;
    case 3:
      mix_frames_same_bps<3, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels, out_ptr,
                                      output_channels, frames);
      break;
    case 4:
      mix_frames_same_bps<4, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels, out_ptr,
                                      output_channels, frames);
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

void mix_frames(const uint8_t *primary, uint8_t primary_bps, uint8_t primary_channels, const uint8_t *secondary,
                uint8_t secondary_bps, uint8_t secondary_channels, uint8_t *output, uint8_t output_bps,
                uint8_t output_channels, uint32_t frames) {
  if (frames == 0 || primary_channels == 0 || secondary_channels == 0 || output_channels == 0 || primary_bps < 1 ||
      primary_bps > 4 || secondary_bps < 1 || secondary_bps > 4 || output_bps < 1 || output_bps > 4) {
    return;
  }

  // Mismatched widths route to the generic loop, specialized only on the output width and the
  // alignment flag. Each buffer is checked against the alignment its own width requires; only when
  // all three pass is the wide-load variant safe.
  if (primary_bps != secondary_bps || primary_bps != output_bps) {
    const bool generic_aligned = (reinterpret_cast<uintptr_t>(primary) & alignment_mask(primary_bps)) == 0 &&
                                 (reinterpret_cast<uintptr_t>(secondary) & alignment_mask(secondary_bps)) == 0 &&
                                 (reinterpret_cast<uintptr_t>(output) & alignment_mask(output_bps)) == 0;
    if (generic_aligned) {
      mix_frames_dispatch_generic<true>(output_bps, primary, primary_bps, primary_channels, secondary, secondary_bps,
                                        secondary_channels, output, output_channels, frames);
    } else {
      mix_frames_dispatch_generic<false>(output_bps, primary, primary_bps, primary_channels, secondary, secondary_bps,
                                         secondary_channels, output, output_channels, frames);
    }
    return;
  }

  const uint8_t bps = primary_bps;

  // Pick the aligned fast path only if every buffer satisfies the alignment the shared sample width
  // requires; otherwise fall back to the byte-wise variant.
  const uintptr_t mask = alignment_mask(bps);
  const bool aligned = (reinterpret_cast<uintptr_t>(primary) & mask) == 0 &&
                       (reinterpret_cast<uintptr_t>(secondary) & mask) == 0 &&
                       (reinterpret_cast<uintptr_t>(output) & mask) == 0;

  if (aligned) {
    mix_frames_dispatch_same_bps<true>(bps, primary, primary_channels, secondary, secondary_channels, output,
                                       output_channels, frames);
  } else {
    mix_frames_dispatch_same_bps<false>(bps, primary, primary_channels, secondary, secondary_channels, output,
                                        output_channels, frames);
  }
}

}  // namespace mixer
}  // namespace esp_audio_libs
