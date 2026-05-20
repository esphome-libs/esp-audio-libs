#pragma once

/// @file q31_utils.h
/// @brief Templated PCM sample <-> Q31 pack/unpack helpers (private to esp-audio-libs).
///
/// A sample of `Bps` bytes is represented as an int32 with the sample value left-justified into
/// the most significant bits (Q31 form), sign extended. This is the common intermediate format
/// used by the mixer and PCM format-conversion code. The `Bps` template parameter is always a
/// compile-time constant here, so the per-width branches below fold away at -O2.
///
/// All pack/unpack functions assume little-endian byte order in the audio buffer.

#include <cstddef>
#include <cstdint>

#include "compiler.h"

namespace esp_audio_libs {
namespace internal {

/// @brief Loads a sample of `Bps` bytes (little-endian, signed) into Q31 form.
///
/// The sample value is left-justified into the most significant bits of an int32, with sign
/// extension. Byte-wise access, so safe for any pointer alignment.
/// @tparam Bps Sample width in bytes: 1, 2, 3, or 4.
/// @param data Pointer to the first byte of the sample.
/// @return Q31 sample value.
template<size_t Bps> inline int32_t unpack_to_q31(const uint8_t *data) {
  static_assert(Bps >= 1 && Bps <= 4, "Bps must be 1, 2, 3, or 4");
  if (Bps == 1) {
    return static_cast<int32_t>(static_cast<uint32_t>(data[0]) << 24);
  } else if (Bps == 2) {
    return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 24));
  } else if (Bps == 3) {
    return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 8) | (static_cast<uint32_t>(data[1]) << 16) |
                                (static_cast<uint32_t>(data[2]) << 24));
  } else {
    return static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
  }
}

/// @brief Stores the most significant `Bps` bytes of a Q31 sample as little-endian PCM.
///
/// Byte-wise access, so safe for any pointer alignment. The caller is responsible for adding any
/// rounding term to `sample` before calling.
/// @tparam Bps Sample width in bytes: 1, 2, 3, or 4.
/// @param sample Q31 sample to store.
/// @param data Pointer to the first byte to write.
template<size_t Bps> inline void pack_q31(int32_t sample, uint8_t *data) {
  static_assert(Bps >= 1 && Bps <= 4, "Bps must be 1, 2, 3, or 4");
  if (Bps == 1) {
    data[0] = static_cast<uint8_t>(sample >> 24);
  } else if (Bps == 2) {
    data[0] = static_cast<uint8_t>(sample >> 16);
    data[1] = static_cast<uint8_t>(sample >> 24);
  } else if (Bps == 3) {
    data[0] = static_cast<uint8_t>(sample >> 8);
    data[1] = static_cast<uint8_t>(sample >> 16);
    data[2] = static_cast<uint8_t>(sample >> 24);
  } else {
    data[0] = static_cast<uint8_t>(sample);
    data[1] = static_cast<uint8_t>(sample >> 8);
    data[2] = static_cast<uint8_t>(sample >> 16);
    data[3] = static_cast<uint8_t>(sample >> 24);
  }
}

/// @brief Like unpack_to_q31, but issues a single wide load for the 16- and 32-bit cases.
///
/// Requires `data` to be aligned to `Bps` for the 2- and 4-byte cases. The 1- and 3-byte cases
/// have no alignment requirement and fall through to the byte-wise path.
/// @tparam Bps Sample width in bytes: 1, 2, 3, or 4.
/// @param data Pointer to the first byte of the sample. Must be aligned to `Bps` for Bps == 2 or 4.
/// @return Q31 sample value.
template<size_t Bps> inline int32_t fast_unpack_to_q31(const uint8_t *data) {
  static_assert(Bps >= 1 && Bps <= 4, "Bps must be 1, 2, 3, or 4");
  if (Bps == 2) {
    int16_t v;
    EAL_MEMCPY(&v, EAL_ASSUME_ALIGNED(data, 2), sizeof(int16_t));
    // Cast to uint16_t first to suppress sign-extension before shifting into Q31.
    return static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint16_t>(v)) << 16);
  } else if (Bps == 4) {
    int32_t v;
    EAL_MEMCPY(&v, EAL_ASSUME_ALIGNED(data, 4), sizeof(int32_t));
    return v;
  } else {
    return unpack_to_q31<Bps>(data);
  }
}

/// @brief Like pack_q31, but issues a single wide store for the 16- and 32-bit cases.
///
/// Requires `data` to be aligned to `Bps` for the 2- and 4-byte cases. The 1- and 3-byte cases
/// have no alignment requirement and fall through to the byte-wise path.
/// @tparam Bps Sample width in bytes: 1, 2, 3, or 4.
/// @param sample Q31 sample to store.
/// @param data Pointer to the first byte to write. Must be aligned to `Bps` for Bps == 2 or 4.
template<size_t Bps> inline void fast_pack_q31(int32_t sample, uint8_t *data) {
  static_assert(Bps >= 1 && Bps <= 4, "Bps must be 1, 2, 3, or 4");
  if (Bps == 2) {
    int16_t v = static_cast<int16_t>(sample >> 16);
    EAL_MEMCPY(EAL_ASSUME_ALIGNED(data, 2), &v, sizeof(int16_t));
  } else if (Bps == 4) {
    EAL_MEMCPY(EAL_ASSUME_ALIGNED(data, 4), &sample, sizeof(int32_t));
  } else {
    pack_q31<Bps>(sample, data);
  }
}

}  // namespace internal
}  // namespace esp_audio_libs
