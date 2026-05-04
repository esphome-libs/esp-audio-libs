#pragma once

#include <cmath>
#include <stdint.h>

namespace esp_audio_libs {
namespace quantization_utils {

/// @brief Converts an array of quantized samples with the specified number of bits into floating point samples.
/// @param input_buffer Pointer to the input quantized samples aligned to the byte
/// @param output_buffer Poitner to the output floating point samples
/// @param num_samples Number of samples to convert
/// @param input_bits Number of bits per sample for the quantized samples
/// @param gain_db Optional amount of gain (in dB) to apply when converting. There is no verification for clipping.
void quantized_to_float(const uint8_t *input_buffer, float *output_buffer, uint32_t num_samples, uint8_t input_bits,
                        float gain_db);

/// @brief Converts an array of floating point samples into quantized samples with the specified number of bits.
/// @param input_buffer Pointer to the input floating point samples
/// @param output_buffer Pointer to the output quantized samples. Samples will be aligned to the byte.
/// @param num_samples Number of samples to convert
/// @param output_bits Number of bits per sample for the quantized samples
/// @return Number of clipped samples
uint32_t float_to_quantized(const float *input_buffer, uint8_t *output_buffer, uint32_t num_samples,
                            uint8_t output_bits);

}  // namespace quantization_utils
}  // namespace esp_audio_libs
