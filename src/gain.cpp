#include "gain.h"

#include <cmath>
#include <cstdint>

#include "compiler.h"
#include "q31_utils.h"

namespace esp_audio_libs {
namespace gain {

int32_t db_to_q31(float db) {
  if (std::isnan(db)) {
    return 0;
  }
  if (db >= 0.0f) {
    return INT32_MAX;
  }
  const float linear = std::pow(10.0f, db / 20.0f);
  const float scaled = std::round(linear * 2147483648.0f);  // 2^31
  if (scaled >= 2147483648.0f) {
    return INT32_MAX;
  }
  if (scaled <= 0.0f) {
    return 0;
  }
  return static_cast<int32_t>(scaled);
}

void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale,
           size_t samples_to_scale, size_t bytes_per_sample) {
  // Each case shifts the input sample into Q31 form, then performs a Q31×Q31 high-half multiply
  // (`(int64_t)a * (int64_t)b >> 32`). This is a single instruction on ESP32/other platforms.
  // This yields a Q30 result in int32. That int32 sample is shifted to restore the original bit
  // width. A rounding term is added for the 8 bit, 16 bit, and 24 bit cases.
  //
  // The 16 and 32 bit cases dispatch on a runtime alignment check: aligned buffers use the
  // fast_unpack_to_q31<> helper (which folds the load into a single l16si/l32i on Xtensa), and
  // misaligned buffers fall back to the byte-wise unpack_to_q31<> path that produces identical
  // output. Xtensa raises an alignment exception on misaligned word/halfword loads, so the
  // runtime gate is required.
  switch (bytes_per_sample) {
    case 1: {
      // 8 bit input shifted left by 24 to reach Q31. The high-half multiply produces s * sf >> 8.
      // Shift right by 23 to recover the 8 bit sample. Rounding term is 1 << 22.
      constexpr int32_t rounding = 1 << 22;
      int8_t *out = reinterpret_cast<int8_t *>(output_buffer);
      size_t i = 0;
      for (; i + 4 <= samples_to_scale; i += 4) {
        const int32_t s0 = internal::unpack_to_q31<1>(audio_samples + i);
        const int32_t s1 = internal::unpack_to_q31<1>(audio_samples + i + 1);
        const int32_t s2 = internal::unpack_to_q31<1>(audio_samples + i + 2);
        const int32_t s3 = internal::unpack_to_q31<1>(audio_samples + i + 3);
        const int32_t high0 =
            static_cast<int32_t>((static_cast<int64_t>(s0) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t high1 =
            static_cast<int32_t>((static_cast<int64_t>(s1) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t high2 =
            static_cast<int32_t>((static_cast<int64_t>(s2) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t high3 =
            static_cast<int32_t>((static_cast<int64_t>(s3) * static_cast<int64_t>(q31_scale)) >> 32);
        out[i] = static_cast<int8_t>((high0 + rounding) >> 23);
        out[i + 1] = static_cast<int8_t>((high1 + rounding) >> 23);
        out[i + 2] = static_cast<int8_t>((high2 + rounding) >> 23);
        out[i + 3] = static_cast<int8_t>((high3 + rounding) >> 23);
      }
      for (; i < samples_to_scale; ++i) {
        const int32_t s = internal::unpack_to_q31<1>(audio_samples + i);
        const int32_t high =
            static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
        out[i] = static_cast<int8_t>((high + rounding) >> 23);
      }
      break;
    }
    case 2: {
      // 16 bit input shifted left by 16 to reach Q31. The high-half multiply produces s * sf >> 16.
      // Shift right by 15 to recover the 16 bit sample. Rounding term is 1 << 14.
      constexpr int32_t rounding = 1 << 14;
      const bool aligned = ((reinterpret_cast<uintptr_t>(audio_samples) |
                             reinterpret_cast<uintptr_t>(output_buffer)) &
                            0x1) == 0;
      if (aligned) {
        // Output stores use EAL_MEMCPY (to avoid strict-aliasing UB) with EAL_ASSUME_ALIGNED on
        // `out` (just established by the runtime check above) so each memcpy folds to a single
        // s16i on Xtensa. Aligned input loads are handled inside fast_unpack_to_q31<2>.
        const uint8_t *in = audio_samples;
        uint8_t *out = static_cast<uint8_t *>(EAL_ASSUME_ALIGNED(output_buffer, 2));
        size_t i = 0;
        for (; i + 4 <= samples_to_scale; i += 4) {
          const int32_t s0 = internal::fast_unpack_to_q31<2>(in + (i + 0) * 2);
          const int32_t s1 = internal::fast_unpack_to_q31<2>(in + (i + 1) * 2);
          const int32_t s2 = internal::fast_unpack_to_q31<2>(in + (i + 2) * 2);
          const int32_t s3 = internal::fast_unpack_to_q31<2>(in + (i + 3) * 2);
          const int32_t high0 =
              static_cast<int32_t>((static_cast<int64_t>(s0) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high1 =
              static_cast<int32_t>((static_cast<int64_t>(s1) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high2 =
              static_cast<int32_t>((static_cast<int64_t>(s2) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high3 =
              static_cast<int32_t>((static_cast<int64_t>(s3) * static_cast<int64_t>(q31_scale)) >> 32);
          const int16_t r0 = static_cast<int16_t>((high0 + rounding) >> 15);
          const int16_t r1 = static_cast<int16_t>((high1 + rounding) >> 15);
          const int16_t r2 = static_cast<int16_t>((high2 + rounding) >> 15);
          const int16_t r3 = static_cast<int16_t>((high3 + rounding) >> 15);
          EAL_MEMCPY(out + (i + 0) * 2, &r0, sizeof(int16_t));
          EAL_MEMCPY(out + (i + 1) * 2, &r1, sizeof(int16_t));
          EAL_MEMCPY(out + (i + 2) * 2, &r2, sizeof(int16_t));
          EAL_MEMCPY(out + (i + 3) * 2, &r3, sizeof(int16_t));
        }
        for (; i < samples_to_scale; ++i) {
          const int32_t s = internal::fast_unpack_to_q31<2>(in + i * 2);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          const int16_t r = static_cast<int16_t>((high + rounding) >> 15);
          EAL_MEMCPY(out + i * 2, &r, sizeof(int16_t));
        }
      } else {
        for (size_t i = 0; i < samples_to_scale; ++i) {
          const uint8_t *p_in = audio_samples + (i * 2);
          uint8_t *p_out = output_buffer + (i * 2);
          const int32_t s = internal::unpack_to_q31<2>(p_in);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          // Round and convert Q30 to Q31 form so pack keeps the right bytes.
          const int32_t scaled_q31 =
              static_cast<int32_t>(static_cast<uint32_t>(high + rounding) << 1);
          internal::pack_q31<2>(scaled_q31, p_out);
        }
      }
      break;
    }
    case 3: {
      // 24 bit input shifted left by 8 to reach Q31. The high-half multiply produces s * sf >> 24.
      // Shift right by 7 to recover the 24 bit sample. Rounding term is 1 << 6.
      constexpr int32_t rounding = 1 << 6;
      for (size_t i = 0; i < samples_to_scale; ++i) {
        const uint8_t *p_in = audio_samples + (i * 3);
        uint8_t *p_out = output_buffer + (i * 3);
        const int32_t s = internal::unpack_to_q31<3>(p_in);
        const int32_t high =
            static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
        const int32_t scaled = (high + rounding) >> 7;
        p_out[0] = static_cast<uint8_t>(scaled);
        p_out[1] = static_cast<uint8_t>(scaled >> 8);
        p_out[2] = static_cast<uint8_t>(scaled >> 16);
      }
      break;
    }
    case 4: {
      // 32 bit input is already Q31. The high-half multiply produces s * sf >> 32, a Q30 result.
      // Shift left by 1 to restore Q31. With sf in [0, INT32_MAX] the magnitude of the high half
      // is at most 2^30 (reached at s=INT32_MIN, sf=INT32_MAX), so the shift never overflows.
      // Cast through uint32_t so the shift is well-defined for negative values.
      const bool aligned = ((reinterpret_cast<uintptr_t>(audio_samples) |
                             reinterpret_cast<uintptr_t>(output_buffer)) &
                            0x3) == 0;
      if (aligned) {
        // Output stores use EAL_MEMCPY (to avoid strict-aliasing UB) with EAL_ASSUME_ALIGNED on
        // `out` (just established by the runtime check above) so each memcpy folds to a single
        // s32i on Xtensa. Aligned input loads are handled inside fast_unpack_to_q31<4>.
        const uint8_t *in = audio_samples;
        uint8_t *out = static_cast<uint8_t *>(EAL_ASSUME_ALIGNED(output_buffer, 4));
        size_t i = 0;
        for (; i + 4 <= samples_to_scale; i += 4) {
          const int32_t v0 = internal::fast_unpack_to_q31<4>(in + (i + 0) * 4);
          const int32_t v1 = internal::fast_unpack_to_q31<4>(in + (i + 1) * 4);
          const int32_t v2 = internal::fast_unpack_to_q31<4>(in + (i + 2) * 4);
          const int32_t v3 = internal::fast_unpack_to_q31<4>(in + (i + 3) * 4);
          const int32_t high0 =
              static_cast<int32_t>((static_cast<int64_t>(v0) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high1 =
              static_cast<int32_t>((static_cast<int64_t>(v1) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high2 =
              static_cast<int32_t>((static_cast<int64_t>(v2) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high3 =
              static_cast<int32_t>((static_cast<int64_t>(v3) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t r0 = static_cast<int32_t>(static_cast<uint32_t>(high0) << 1);
          const int32_t r1 = static_cast<int32_t>(static_cast<uint32_t>(high1) << 1);
          const int32_t r2 = static_cast<int32_t>(static_cast<uint32_t>(high2) << 1);
          const int32_t r3 = static_cast<int32_t>(static_cast<uint32_t>(high3) << 1);
          EAL_MEMCPY(out + (i + 0) * 4, &r0, sizeof(int32_t));
          EAL_MEMCPY(out + (i + 1) * 4, &r1, sizeof(int32_t));
          EAL_MEMCPY(out + (i + 2) * 4, &r2, sizeof(int32_t));
          EAL_MEMCPY(out + (i + 3) * 4, &r3, sizeof(int32_t));
        }
        for (; i < samples_to_scale; ++i) {
          const int32_t v = internal::fast_unpack_to_q31<4>(in + i * 4);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(v) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t r = static_cast<int32_t>(static_cast<uint32_t>(high) << 1);
          EAL_MEMCPY(out + i * 4, &r, sizeof(int32_t));
        }
      } else {
        for (size_t i = 0; i < samples_to_scale; ++i) {
          const uint8_t *p_in = audio_samples + (i * 4);
          uint8_t *p_out = output_buffer + (i * 4);
          const int32_t s = internal::unpack_to_q31<4>(p_in);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t scaled_q31 = static_cast<int32_t>(static_cast<uint32_t>(high) << 1);
          internal::pack_q31<4>(scaled_q31, p_out);
        }
      }
      break;
    }
  }
}

}  // namespace gain
}  // namespace esp_audio_libs
