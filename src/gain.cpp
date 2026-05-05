#include "gain.h"

#include <cmath>
#include <cstdint>

namespace esp_audio_libs {
namespace gain {

namespace {

/// Load a sample of `bytes_per_sample` bytes and place it in Q31 form (sign extended into the
/// upper bits of an int32). Byte-wise access, so it is safe for any pointer alignment.
inline int32_t unpack_audio_sample_to_q31(const uint8_t *data, size_t bytes_per_sample) {
  uint32_t sample = 0;
  if (bytes_per_sample == 1) {
    sample |= static_cast<uint32_t>(data[0]) << 24;
  } else if (bytes_per_sample == 2) {
    sample |= static_cast<uint32_t>(data[0]) << 16;
    sample |= static_cast<uint32_t>(data[1]) << 24;
  } else if (bytes_per_sample == 3) {
    sample |= static_cast<uint32_t>(data[0]) << 8;
    sample |= static_cast<uint32_t>(data[1]) << 16;
    sample |= static_cast<uint32_t>(data[2]) << 24;
  } else if (bytes_per_sample == 4) {
    sample |= static_cast<uint32_t>(data[0]);
    sample |= static_cast<uint32_t>(data[1]) << 8;
    sample |= static_cast<uint32_t>(data[2]) << 16;
    sample |= static_cast<uint32_t>(data[3]) << 24;
  }
  return static_cast<int32_t>(sample);
}

/// Pack a Q31 sample as `bytes_per_sample` bytes, keeping the most significant bytes. Byte-wise
/// access, so it is safe for any pointer alignment. The caller is responsible for adding any
/// rounding term to `sample` before calling.
inline void pack_q31_as_audio_sample(int32_t sample, uint8_t *data, size_t bytes_per_sample) {
  if (bytes_per_sample == 1) {
    data[0] = static_cast<uint8_t>(sample >> 24);
  } else if (bytes_per_sample == 2) {
    data[0] = static_cast<uint8_t>(sample >> 16);
    data[1] = static_cast<uint8_t>(sample >> 24);
  } else if (bytes_per_sample == 3) {
    data[0] = static_cast<uint8_t>(sample >> 8);
    data[1] = static_cast<uint8_t>(sample >> 16);
    data[2] = static_cast<uint8_t>(sample >> 24);
  } else if (bytes_per_sample == 4) {
    data[0] = static_cast<uint8_t>(sample);
    data[1] = static_cast<uint8_t>(sample >> 8);
    data[2] = static_cast<uint8_t>(sample >> 16);
    data[3] = static_cast<uint8_t>(sample >> 24);
  }
}

}  // namespace

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
  // The 16 and 32 bit cases reinterpret_cast the byte buffers to int16_t/int32_t for the inner
  // loop. That requires natural alignment, which Xtensa enforces in hardware. When the buffers
  // are not aligned we fall back to a byte-wise slow path.
  switch (bytes_per_sample) {
    case 1: {
      // 8 bit input shifted left by 24 to reach Q31. The high-half multiply produces s * sf >> 8.
      // Shift right by 23 to recover the 8 bit sample. Rounding term is 1 << 22.
      constexpr int32_t rounding = 1 << 22;
      const int8_t *in = reinterpret_cast<const int8_t *>(audio_samples);
      int8_t *out = reinterpret_cast<int8_t *>(output_buffer);
      size_t i = 0;
      for (; i + 4 <= samples_to_scale; i += 4) {
        const int32_t s0 = static_cast<int32_t>(static_cast<uint32_t>(in[i]) << 24);
        const int32_t s1 = static_cast<int32_t>(static_cast<uint32_t>(in[i + 1]) << 24);
        const int32_t s2 = static_cast<int32_t>(static_cast<uint32_t>(in[i + 2]) << 24);
        const int32_t s3 = static_cast<int32_t>(static_cast<uint32_t>(in[i + 3]) << 24);
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
        const int32_t s = static_cast<int32_t>(static_cast<uint32_t>(in[i]) << 24);
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
        const int16_t *in = reinterpret_cast<const int16_t *>(audio_samples);
        int16_t *out = reinterpret_cast<int16_t *>(output_buffer);
        size_t i = 0;
        for (; i + 4 <= samples_to_scale; i += 4) {
          const int32_t s0 = static_cast<int32_t>(static_cast<uint32_t>(in[i]) << 16);
          const int32_t s1 = static_cast<int32_t>(static_cast<uint32_t>(in[i + 1]) << 16);
          const int32_t s2 = static_cast<int32_t>(static_cast<uint32_t>(in[i + 2]) << 16);
          const int32_t s3 = static_cast<int32_t>(static_cast<uint32_t>(in[i + 3]) << 16);
          const int32_t high0 =
              static_cast<int32_t>((static_cast<int64_t>(s0) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high1 =
              static_cast<int32_t>((static_cast<int64_t>(s1) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high2 =
              static_cast<int32_t>((static_cast<int64_t>(s2) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high3 =
              static_cast<int32_t>((static_cast<int64_t>(s3) * static_cast<int64_t>(q31_scale)) >> 32);
          out[i] = static_cast<int16_t>((high0 + rounding) >> 15);
          out[i + 1] = static_cast<int16_t>((high1 + rounding) >> 15);
          out[i + 2] = static_cast<int16_t>((high2 + rounding) >> 15);
          out[i + 3] = static_cast<int16_t>((high3 + rounding) >> 15);
        }
        for (; i < samples_to_scale; ++i) {
          const int32_t s = static_cast<int32_t>(static_cast<uint32_t>(in[i]) << 16);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          out[i] = static_cast<int16_t>((high + rounding) >> 15);
        }
      } else {
        for (size_t i = 0; i < samples_to_scale; ++i) {
          const uint8_t *p_in = audio_samples + (i * 2);
          uint8_t *p_out = output_buffer + (i * 2);
          const int32_t s = unpack_audio_sample_to_q31(p_in, 2);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          // Round and convert Q30 to Q31 form so pack keeps the right bytes.
          const int32_t scaled_q31 =
              static_cast<int32_t>(static_cast<uint32_t>(high + rounding) << 1);
          pack_q31_as_audio_sample(scaled_q31, p_out, 2);
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
        // Little-endian 24 bit load with sign extension from bit 23.
        const int32_t sample =
            static_cast<int32_t>(static_cast<uint32_t>(p_in[0]) | (static_cast<uint32_t>(p_in[1]) << 8) |
                                 (static_cast<uint32_t>(p_in[2]) << 16) | ((p_in[2] & 0x80) ? 0xFF000000u : 0u));
        const int32_t s = static_cast<int32_t>(static_cast<uint32_t>(sample) << 8);
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
      // Shift left by 1 to restore Q31. With sf in [0, INT32_MAX] the high half never reaches
      // 1 << 30, so the shift is always safe.
      const bool aligned = ((reinterpret_cast<uintptr_t>(audio_samples) |
                             reinterpret_cast<uintptr_t>(output_buffer)) &
                            0x3) == 0;
      if (aligned) {
        const int32_t *in = reinterpret_cast<const int32_t *>(audio_samples);
        int32_t *out = reinterpret_cast<int32_t *>(output_buffer);
        size_t i = 0;
        for (; i + 4 <= samples_to_scale; i += 4) {
          const int32_t high0 =
              static_cast<int32_t>((static_cast<int64_t>(in[i]) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high1 =
              static_cast<int32_t>((static_cast<int64_t>(in[i + 1]) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high2 =
              static_cast<int32_t>((static_cast<int64_t>(in[i + 2]) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t high3 =
              static_cast<int32_t>((static_cast<int64_t>(in[i + 3]) * static_cast<int64_t>(q31_scale)) >> 32);
          out[i] = static_cast<int32_t>(static_cast<uint32_t>(high0) << 1);
          out[i + 1] = static_cast<int32_t>(static_cast<uint32_t>(high1) << 1);
          out[i + 2] = static_cast<int32_t>(static_cast<uint32_t>(high2) << 1);
          out[i + 3] = static_cast<int32_t>(static_cast<uint32_t>(high3) << 1);
        }
        for (; i < samples_to_scale; ++i) {
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(in[i]) * static_cast<int64_t>(q31_scale)) >> 32);
          out[i] = static_cast<int32_t>(static_cast<uint32_t>(high) << 1);
        }
      } else {
        for (size_t i = 0; i < samples_to_scale; ++i) {
          const uint8_t *p_in = audio_samples + (i * 4);
          uint8_t *p_out = output_buffer + (i * 4);
          const int32_t s = unpack_audio_sample_to_q31(p_in, 4);
          const int32_t high =
              static_cast<int32_t>((static_cast<int64_t>(s) * static_cast<int64_t>(q31_scale)) >> 32);
          const int32_t scaled_q31 = static_cast<int32_t>(static_cast<uint32_t>(high) << 1);
          pack_q31_as_audio_sample(scaled_q31, p_out, 4);
        }
      }
      break;
    }
  }
}

}  // namespace gain
}  // namespace esp_audio_libs
