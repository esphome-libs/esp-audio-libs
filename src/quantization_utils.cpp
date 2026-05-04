#include "quantization_utils.h"

#include <cstddef>

namespace esp_audio_libs {
namespace quantization_utils {

void quantized_to_float(const uint8_t *input_buffer, float *output_buffer, uint32_t num_samples, uint8_t input_bits,
                        float gain_db) {
  float gain = powf(10.0f, gain_db / 20.0f);

  if (input_bits <= 8) {
    float gain_factor = gain / 128.0f;

    for (unsigned int i = 0; i < num_samples; ++i) {
      output_buffer[i] = ((int) input_buffer[i] - 128) * gain_factor;
    }

  } else if (input_bits <= 16) {
    float gain_factor = gain / 32768.0f;
    unsigned int i, j;

    for (i = j = 0; i < num_samples; ++i) {
      int16_t value = input_buffer[j++];
      value += input_buffer[j++] << 8;
      output_buffer[i] = value * gain_factor;
    }
  } else if (input_bits <= 24) {
    float gain_factor = gain / 8388608.0f;
    unsigned int i, j;

    for (i = j = 0; i < num_samples; ++i) {
      int32_t value = input_buffer[j++];
      value += input_buffer[j++] << 8;
      value += (int32_t) (signed char) input_buffer[j++] << 16;
      output_buffer[i] = value * gain_factor;
    }
  } else if (input_bits <= 32) {
    float gain_factor = gain / 2147483648.0f;
    unsigned int i, j;

    for (i = j = 0; i < num_samples; ++i) {
      int32_t value = input_buffer[j++];
      value += input_buffer[j++] << 8;
      value += (int32_t) (signed char) input_buffer[j++] << 16;
      value += (int32_t) (signed char) input_buffer[j++] << 24;
      output_buffer[i] = value * gain_factor;
    }
  }
}

uint32_t float_to_quantized(const float *input_buffer, uint8_t *output_buffer, uint32_t num_samples,
                            uint8_t output_bits) {
  float scalar = (static_cast<uint64_t>(1) << output_bits) / 2.0f;
  int32_t offset = (output_bits <= 8) * 128;
  int32_t high_clip = (1 << (output_bits - 1)) - 1;
  int32_t low_clip = ~high_clip;
  int left_shift = (32 - output_bits) % 8;
  size_t i, j;
  uint32_t clipped_samples = 0;

  for (i = j = 0; i < num_samples; ++i) {
    int32_t output = floorf((input_buffer[i] * scalar) + 0.5f);
    if (output_bits < 32) {
      if (output > high_clip) {
        ++clipped_samples;
        output = high_clip;
      } else if (output < low_clip) {
        ++clipped_samples;
        output = low_clip;
      }
    } else {
      if (input_buffer[i] >= 1.0f) {
        ++clipped_samples;
        output = high_clip;
      } else if (input_buffer[i] < -1.0f) {
        ++clipped_samples;
        output = low_clip;
      }
    }

    output_buffer[j++] = output = (output << left_shift) + offset;
    if (output_bits > 8) {
      output_buffer[j++] = output >> 8;

      if (output_bits > 16) {
        output_buffer[j++] = output >> 16;
      }
      if (output_bits > 24) {
        output_buffer[j++] = output >> 24;
      }
    }
  }

  return clipped_samples;
}

}  // namespace quantization_utils
}  // namespace esp_audio_libs
