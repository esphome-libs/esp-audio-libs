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

template<size_t PrimaryBps, size_t SecondaryBps, size_t OutputBps, bool Aligned>
EAL_HOT void mix_frames_impl(const uint8_t *pri_ptr, uint8_t primary_channels, const uint8_t *sec_ptr,
                             uint8_t secondary_channels, uint8_t *out_ptr, uint8_t output_channels, uint32_t frames) {
  const uint8_t max_primary_channel_index = primary_channels - 1;
  const uint8_t max_secondary_channel_index = secondary_channels - 1;

  // Mix in Q30 so the sum stays in a 32-bit register; the right shift loses 1 LSB per input.
  constexpr int32_t Q30_MIN = -(INT32_C(1) << 30);
  constexpr int32_t Q30_MAX = (INT32_C(1) << 30) - 1;

  for (uint32_t frame = 0; frame < frames; ++frame) {
    for (uint8_t out_ch = 0; out_ch < output_channels; ++out_ch) {
      const uint8_t pri_ch = std::min<uint8_t>(out_ch, max_primary_channel_index);
      const int32_t primary_q31 = unpack<PrimaryBps, Aligned>(pri_ptr + pri_ch * PrimaryBps);

      const uint8_t sec_ch = std::min<uint8_t>(out_ch, max_secondary_channel_index);
      const int32_t secondary_q31 = unpack<SecondaryBps, Aligned>(sec_ptr + sec_ch * SecondaryBps);

      const int32_t sum_q30 = (primary_q31 >> 1) + (secondary_q31 >> 1);
      const int32_t clamped_q31 = std::min(std::max(sum_q30, Q30_MIN), Q30_MAX) << 1;

      pack<OutputBps, Aligned>(clamped_q31, out_ptr + out_ch * OutputBps);
    }
    pri_ptr += primary_channels * PrimaryBps;
    sec_ptr += secondary_channels * SecondaryBps;
    out_ptr += output_channels * OutputBps;
  }
}

template<size_t PrimaryBps, size_t SecondaryBps, bool Aligned>
void mix_frames_dispatch_output(uint8_t output_bps, const uint8_t *pri_ptr, uint8_t primary_channels,
                                const uint8_t *sec_ptr, uint8_t secondary_channels, uint8_t *out_ptr,
                                uint8_t output_channels, uint32_t frames) {
  switch (output_bps) {
    case 1:
      mix_frames_impl<PrimaryBps, SecondaryBps, 1, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels,
                                                            out_ptr, output_channels, frames);
      break;
    case 2:
      mix_frames_impl<PrimaryBps, SecondaryBps, 2, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels,
                                                            out_ptr, output_channels, frames);
      break;
    case 3:
      mix_frames_impl<PrimaryBps, SecondaryBps, 3, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels,
                                                            out_ptr, output_channels, frames);
      break;
    case 4:
      mix_frames_impl<PrimaryBps, SecondaryBps, 4, Aligned>(pri_ptr, primary_channels, sec_ptr, secondary_channels,
                                                            out_ptr, output_channels, frames);
      break;
  }
}

template<size_t PrimaryBps, bool Aligned>
void mix_frames_dispatch_secondary(uint8_t secondary_bps, uint8_t output_bps, const uint8_t *pri_ptr,
                                   uint8_t primary_channels, const uint8_t *sec_ptr, uint8_t secondary_channels,
                                   uint8_t *out_ptr, uint8_t output_channels, uint32_t frames) {
  switch (secondary_bps) {
    case 1:
      mix_frames_dispatch_output<PrimaryBps, 1, Aligned>(output_bps, pri_ptr, primary_channels, sec_ptr,
                                                         secondary_channels, out_ptr, output_channels, frames);
      break;
    case 2:
      mix_frames_dispatch_output<PrimaryBps, 2, Aligned>(output_bps, pri_ptr, primary_channels, sec_ptr,
                                                         secondary_channels, out_ptr, output_channels, frames);
      break;
    case 3:
      mix_frames_dispatch_output<PrimaryBps, 3, Aligned>(output_bps, pri_ptr, primary_channels, sec_ptr,
                                                         secondary_channels, out_ptr, output_channels, frames);
      break;
    case 4:
      mix_frames_dispatch_output<PrimaryBps, 4, Aligned>(output_bps, pri_ptr, primary_channels, sec_ptr,
                                                         secondary_channels, out_ptr, output_channels, frames);
      break;
  }
}

template<bool Aligned>
void mix_frames_dispatch_primary(uint8_t primary_bps, uint8_t secondary_bps, uint8_t output_bps,
                                 const uint8_t *pri_ptr, uint8_t primary_channels, const uint8_t *sec_ptr,
                                 uint8_t secondary_channels, uint8_t *out_ptr, uint8_t output_channels,
                                 uint32_t frames) {
  switch (primary_bps) {
    case 1:
      mix_frames_dispatch_secondary<1, Aligned>(secondary_bps, output_bps, pri_ptr, primary_channels, sec_ptr,
                                                secondary_channels, out_ptr, output_channels, frames);
      break;
    case 2:
      mix_frames_dispatch_secondary<2, Aligned>(secondary_bps, output_bps, pri_ptr, primary_channels, sec_ptr,
                                                secondary_channels, out_ptr, output_channels, frames);
      break;
    case 3:
      mix_frames_dispatch_secondary<3, Aligned>(secondary_bps, output_bps, pri_ptr, primary_channels, sec_ptr,
                                                secondary_channels, out_ptr, output_channels, frames);
      break;
    case 4:
      mix_frames_dispatch_secondary<4, Aligned>(secondary_bps, output_bps, pri_ptr, primary_channels, sec_ptr,
                                                secondary_channels, out_ptr, output_channels, frames);
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

  // Pick the aligned fast path if every pointer satisfies the strictest sample-width alignment in
  // use across the three buffers; otherwise fall back to the byte-wise path.
  const uintptr_t align_mask =
      alignment_mask(primary_bps) | alignment_mask(secondary_bps) | alignment_mask(output_bps);
  const uintptr_t ptr_or = reinterpret_cast<uintptr_t>(primary) | reinterpret_cast<uintptr_t>(secondary) |
                           reinterpret_cast<uintptr_t>(output);
  const bool aligned = (ptr_or & align_mask) == 0;

  if (aligned) {
    mix_frames_dispatch_primary<true>(primary_bps, secondary_bps, output_bps, primary, primary_channels, secondary,
                                      secondary_channels, output, output_channels, frames);
  } else {
    mix_frames_dispatch_primary<false>(primary_bps, secondary_bps, output_bps, primary, primary_channels, secondary,
                                       secondary_channels, output, output_channels, frames);
  }
}

}  // namespace mixer
}  // namespace esp_audio_libs
