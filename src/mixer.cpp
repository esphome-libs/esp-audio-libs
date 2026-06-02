#include "mixer.h"

#include <algorithm>
#include <cstdint>

#include "compiler.h"
#include "q31_utils.h"

namespace esp_audio_libs {
namespace mixer {

namespace {

using internal::fast_pack_q31;
using internal::fast_unpack_to_q31;
using internal::pack_q31;
using internal::unpack_to_q31;

// Each input is right-shifted MIX_SHIFT bits before the two are summed. Two bits of headroom (vs.
// the minimum of one) leave enough room that the rounding term can be added without overflowing
// int32, so the whole mix stays in 32-bit math. The cost is 2 discarded LSB of true 31/32-bit-deep
// input content (inaudible); 16- and 24-bit inputs are unaffected since they carry trailing zeros.
// The clamp range is Q(31 - MIX_SHIFT): MIX_MAX maps exactly to full scale at every output depth.
constexpr int MIX_SHIFT = 2;
constexpr int32_t MIX_MIN = -(INT32_C(1) << (31 - MIX_SHIFT));
constexpr int32_t MIX_MAX = (INT32_C(1) << (31 - MIX_SHIFT)) - 1;

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

// Sum two Q31 inputs, round for a Bps-byte output, and store the result. Each input is shifted
// right by MIX_SHIFT before adding so the pre-round sum plus the rounding term stay within int32;
// the rounding term is added before the clamp, so the single [MIX_MIN, MIX_MAX] clamp guards both
// the mix saturation and the rounding overflow: MIX_MAX maps exactly to full scale at the output
// depth (MIX_MAX >> ((4-Bps)*8 - MIX_SHIFT) == output max), so a rounded full-scale sample cannot
// wrap. The final left shift goes through uint32_t because shifting a negative signed value is
// undefined before C++20; the clamp guarantees the shifted result still fits in int32.
template<size_t Bps, bool Aligned>
EAL_HOT inline void mix_and_pack(int32_t primary_q31, int32_t secondary_q31, uint8_t *out) {
  constexpr int net_shift = (4 - static_cast<int>(Bps)) * 8 - MIX_SHIFT;
  // net_shift <= 0 only for Bps == 4 (32-bit output), which does not narrow and must not round, so
  // round_term is 0 there; the `> 0` guard also keeps the shift below from going negative. The term
  // is added unconditionally in the sum, so this 0 is load-bearing, not just a dead-branch value.
  // TODO(C++17): an `if constexpr (net_shift > 0)` here would discard the Bps == 4 case outright and
  // let round_term drop its guard.
  constexpr int32_t round_term = net_shift > 0 ? (INT32_C(1) << (net_shift - 1)) : 0;
  const int32_t sum = (primary_q31 >> MIX_SHIFT) + (secondary_q31 >> MIX_SHIFT) + round_term;
  const int32_t clamped = std::min(std::max(sum, MIX_MIN), MIX_MAX);
  pack<Bps, Aligned>(static_cast<int32_t>(static_cast<uint32_t>(clamped) << MIX_SHIFT), out);
}

// Hot path: the two inputs and the output all share one sample width, so a single `Bps` template
// parameter covers all three and the unpack/pack stay fully inlined.
template<size_t Bps, bool Aligned>
EAL_HOT void mix_frames_same_bps(const uint8_t *pri_ptr, uint8_t primary_channels, const uint8_t *sec_ptr,
                                 uint8_t secondary_channels, uint8_t *out_ptr, uint8_t output_channels,
                                 uint32_t frames) {
  const uint8_t max_primary_channel_index = primary_channels - 1;
  const uint8_t max_secondary_channel_index = secondary_channels - 1;
  const size_t pri_stride = static_cast<size_t>(primary_channels) * Bps;
  const size_t sec_stride = static_cast<size_t>(secondary_channels) * Bps;
  const size_t out_stride = static_cast<size_t>(output_channels) * Bps;

  for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
    const uint8_t pri_ch = std::min<uint8_t>(out_ch, max_primary_channel_index);
    const uint8_t sec_ch = std::min<uint8_t>(out_ch, max_secondary_channel_index);
    const uint8_t *pri = pri_ptr + pri_ch * Bps;
    const uint8_t *sec = sec_ptr + sec_ch * Bps;
    uint8_t *out = out_ptr + out_ch * Bps;

    for (uint32_t frame = 0; frame < frames; ++frame) {
      const int32_t primary_q31 = unpack<Bps, Aligned>(pri);
      const int32_t secondary_q31 = unpack<Bps, Aligned>(sec);
      mix_and_pack<Bps, Aligned>(primary_q31, secondary_q31, out);
      pri += pri_stride;
      sec += sec_stride;
      out += out_stride;
    }
  }
}

// Runtime-width unpack for the mismatched-width path: a switch on `bps` keeps the input width off
// the template axis. The `Aligned` flag selects the wide-load variant; callers guarantee each
// buffer is aligned to its own width before picking it.
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
// alignment stay template parameters (so the pack inlines); the two input widths are resolved by
// the runtime switch above.
template<size_t OutputBps, bool Aligned>
EAL_HOT void mix_frames_generic(const uint8_t *pri_ptr, uint8_t primary_bps, uint8_t primary_channels,
                                const uint8_t *sec_ptr, uint8_t secondary_bps, uint8_t secondary_channels,
                                uint8_t *out_ptr, uint8_t output_channels, uint32_t frames) {
  const uint8_t max_primary_channel_index = primary_channels - 1;
  const uint8_t max_secondary_channel_index = secondary_channels - 1;
  const size_t pri_stride = static_cast<size_t>(primary_channels) * primary_bps;
  const size_t sec_stride = static_cast<size_t>(secondary_channels) * secondary_bps;
  const size_t out_stride = static_cast<size_t>(output_channels) * OutputBps;

  for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
    const uint8_t pri_ch = std::min<uint8_t>(out_ch, max_primary_channel_index);
    const uint8_t sec_ch = std::min<uint8_t>(out_ch, max_secondary_channel_index);
    const uint8_t *pri = pri_ptr + pri_ch * primary_bps;
    const uint8_t *sec = sec_ptr + sec_ch * secondary_bps;
    uint8_t *out = out_ptr + out_ch * OutputBps;

    for (uint32_t frame = 0; frame < frames; ++frame) {
      const int32_t primary_q31 = unpack_runtime<Aligned>(pri, primary_bps);
      const int32_t secondary_q31 = unpack_runtime<Aligned>(sec, secondary_bps);
      mix_and_pack<OutputBps, Aligned>(primary_q31, secondary_q31, out);
      pri += pri_stride;
      sec += sec_stride;
      out += out_stride;
    }
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
