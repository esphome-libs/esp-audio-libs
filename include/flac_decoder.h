// FLAC decoder optimized for ESP32
// Based on: https://www.nayuki.io/res/simple-flac-implementation/
// Spec: https://xiph.org/flac/format.html

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

// Include necessary headers based on platform
#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#else
#include <cstdlib>
#endif

namespace esp_audio_libs {
namespace flac {

/// @brief Result codes returned by FLACDecoder methods
enum FLACDecoderResult {
  // Success codes
  FLAC_DECODER_SUCCESS = 0,             // Operation completed successfully
  FLAC_DECODER_NO_MORE_FRAMES = 1,      // Reached end of stream (not an error)
  FLAC_DECODER_HEADER_OUT_OF_DATA = 2,  // Need more data to complete header (streaming)

  // Error codes
  FLAC_DECODER_ERROR_OUT_OF_DATA = 3,                       // Unexpected end of data during frame decode
  FLAC_DECODER_ERROR_BAD_MAGIC_NUMBER = 4,                  // File doesn't start with 'fLaC'
  FLAC_DECODER_ERROR_SYNC_NOT_FOUND = 5,                    // Could not find frame sync code
  FLAC_DECODER_ERROR_BAD_BLOCK_SIZE_CODE = 6,               // Invalid block size in frame header
  FLAC_DECODER_ERROR_BAD_HEADER = 7,                        // Malformed frame header
  FLAC_DECODER_ERROR_RESERVED_CHANNEL_ASSIGNMENT = 8,       // Reserved channel assignment value
  FLAC_DECODER_ERROR_BAD_SAMPLE_DEPTH = 16,                 // Unsupported sample bit depth
  FLAC_DECODER_ERROR_RESERVED_SUBFRAME_TYPE = 9,            // Reserved subframe type encountered
  FLAC_DECODER_ERROR_BAD_FIXED_PREDICTION_ORDER = 10,       // Invalid fixed prediction order
  FLAC_DECODER_ERROR_RESERVED_RESIDUAL_CODING_METHOD = 11,  // Reserved residual coding method
  FLAC_DECODER_ERROR_BLOCK_SIZE_NOT_DIVISIBLE_RICE = 12,    // Rice partition error
  FLAC_DECODER_ERROR_MEMORY_ALLOCATION_ERROR = 13,          // Failed to allocate memory
  FLAC_DECODER_ERROR_BLOCK_SIZE_OUT_OF_RANGE = 14,          // Block size exceeds limits
  FLAC_DECODER_ERROR_CRC_MISMATCH = 15,                     // Frame CRC check failed
  FLAC_DECODER_ERROR_METADATA_TOO_LARGE = 16                // Metadata block exceeds size limit
};

/// @brief FLAC metadata block types as defined in the FLAC specification
/// These identify the different types of metadata that can appear in a FLAC file's header.
enum FLACMetadataType {
  FLAC_METADATA_TYPE_STREAMINFO = 0,      // Required stream information (sample rate, channels, etc.)
  FLAC_METADATA_TYPE_PADDING = 1,         // Empty space for future metadata
  FLAC_METADATA_TYPE_APPLICATION = 2,     // Application-specific data
  FLAC_METADATA_TYPE_SEEKTABLE = 3,       // Seek points for fast random access
  FLAC_METADATA_TYPE_VORBIS_COMMENT = 4,  // Vorbis-style comments (tags)
  FLAC_METADATA_TYPE_CUESHEET = 5,        // CD cuesheet information
  FLAC_METADATA_TYPE_PICTURE = 6,         // Embedded album art or pictures
  FLAC_METADATA_TYPE_INVALID = 127        // Invalid metadata type
};

/// @brief Container for a decoded FLAC metadata block
struct FLACMetadataBlock {
  FLACMetadataType type;      // Type of metadata block
  uint32_t length;            // Length of data in bytes
  std::vector<uint8_t> data;  // Raw metadata block data
};

/// Default maximum album art size (0 = disabled, saves memory on constrained devices)
static const uint32_t DEFAULT_MAX_ALBUM_ART_SIZE = 0;

/// Default maximum sizes for metadata blocks (0 = skip/don't store)
static const uint32_t DEFAULT_MAX_PADDING_SIZE = 0;      // Padding: usually not needed
static const uint32_t DEFAULT_MAX_APPLICATION_SIZE = 0;  // Application data: usually not needed
static const uint32_t DEFAULT_MAX_SEEKTABLE_SIZE = 0;    // Seektable: disable by default (no seeking support yet)
static const uint32_t DEFAULT_MAX_VORBIS_COMMENT_SIZE = 2 * 1024;  // Vorbis comments/tags: 2KB (typical 1-2KB)
static const uint32_t DEFAULT_MAX_CUESHEET_SIZE = 0;               // Cuesheet: usually not needed
static const uint32_t DEFAULT_MAX_UNKNOWN_SIZE = 0;                // Unknown types: skip by default

/**
 * @brief FLAC audio decoder optimized for ESP32
 *
 * This decoder supports streaming decoding of FLAC audio files, allowing headers and frames
 * to be processed in chunks. It handles metadata extraction, CRC validation, and outputs
 * interleaved PCM samples.
 *
 * Usage:
 * 1. (Optional) Configure metadata size limits using set_max_metadata_size()
 * 2. Call read_header() with the file header data (may be called multiple times for streaming)
 * 3. Allocate output buffer using get_output_buffer_size_bytes()
 * 4. Call decode_frame() repeatedly to decode audio frames
 *
 * Features:
 * - Streaming support for both header and frame decoding
 * - Configurable metadata size limits per type to manage memory usage
 * - Optional CRC checking for data integrity
 * - Optimized assembly implementations for ESP32 (Xtensa)
 *
 * @section architecture Architecture and Design
 *
 * The decoder is designed as a self-contained, non-inheritable class. All implementation
 * details are private to ensure a stable API and prevent misuse. The class uses a clean
 * separation between public API methods and internal implementation helpers.
 *
 *
 * @section metadata_configuration Metadata Handling
 *
 * Metadata blocks can be individually configured with size limits to control memory usage:
 *
 * @code
 * FLACDecoder decoder;
 *
 * // Enable seektable storage (up to 1KB)
 * decoder.set_max_metadata_size(FLAC_METADATA_TYPE_SEEKTABLE, 1024);
 *
 * // Increase Vorbis comment storage for files with extensive tags
 * decoder.set_max_metadata_size(FLAC_METADATA_TYPE_VORBIS_COMMENT, 4 * 1024);
 *
 * // Enable album art (up to 50KB)
 * decoder.set_max_metadata_size(FLAC_METADATA_TYPE_PICTURE, 50 * 1024);
 * @endcode
 *
 * Default limits are conservative for memory-constrained devices:
 * - STREAMINFO: Always stored (required)
 * - PADDING: 0 bytes (skipped)
 * - APPLICATION: 0 bytes (skipped)
 * - SEEKTABLE: 0 bytes (skipped, seeking is unsupported)
 * - VORBIS_COMMENT: 2KB (typical tags)
 * - CUESHEET: 0 bytes (skipped)
 * - PICTURE: 0 bytes (skipped)
 * - Unknown types: 0 bytes (skipped)
 *
 * @section memory_customization Memory Allocation Customization
 *
 * The decoder uses FLAC_MALLOC and FLAC_FREE macros for internal memory allocation.
 * These macros are defined in this header file and can be overridden at compile time.
 *
 * Default behavior:
 * - ESP32 builds: Allocates from PSRAM first, falls back to internal memory if unavailable
 * - Other platforms: Uses standard malloc/free
 *
 * To override with custom allocation functions, define these macros in your build configuration:
 *
 * @code
 * # Example: platformio.ini
 * build_flags =
 *   -DFLAC_MALLOC=my_custom_malloc
 *   -DFLAC_FREE=my_custom_free
 *
 * # Example: CMakeLists.txt
 * target_compile_definitions(my_target PRIVATE
 *   FLAC_MALLOC=my_custom_malloc
 *   FLAC_FREE=my_custom_free
 * )
 * @endcode
 *
 * @warning CRITICAL: Both FLAC_MALLOC and FLAC_FREE must be defined together as a pair.
 *          Defining only one will result in mismatched allocation/deallocation and may cause
 *          memory corruption or crashes. If only one macro is defined, the implementation will
 *          use the default for the other, which may not be compatible with your custom allocator.
 *
 * @note For non-ESP32 embedded platforms without standard library support, you must provide
 *       custom allocation functions, as the default implementation uses malloc/free.
 *
 * Your custom functions should have these signatures:
 * @code
 * void* my_custom_malloc(size_t size);
 * void my_custom_free(void* ptr);
 * @endcode
 */

// ============================================================================
// Memory Allocation Customization
// ============================================================================

// Define memory allocation macros - can be overridden via compiler defines
// Example: -DFLAC_MALLOC=my_malloc -DFLAC_FREE=my_free
// Both FLAC_MALLOC and FLAC_FREE should be defined together as a pair.

#ifndef FLAC_MALLOC
#ifdef ESP_PLATFORM
#define FLAC_MALLOC(sz) \
  heap_caps_malloc_prefer(sz, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define FLAC_MALLOC(sz) malloc(sz)
#endif
#endif

#ifndef FLAC_FREE
#ifdef ESP_PLATFORM
#define FLAC_FREE(p) free(p)
#else
#define FLAC_FREE(p) free(p)
#endif
#endif

class FLACDecoder {
 public:
  ~FLACDecoder() { this->free_buffers(); }

  // ========================================
  // Core Decoding API
  // ========================================

  /// @brief Read and parse FLAC file header and metadata blocks
  ///
  /// Must be called before decode_frame(). Supports streaming by allowing multiple calls
  /// with additional data when FLAC_DECODER_HEADER_OUT_OF_DATA is returned.
  ///
  /// @param buffer Pointer to buffer containing header data
  /// @param buffer_length Number of bytes in buffer
  /// @return FLAC_DECODER_SUCCESS when header is complete
  ///         FLAC_DECODER_HEADER_OUT_OF_DATA when more data is needed (streaming)
  ///         Error code on failure
  FLACDecoderResult read_header(const uint8_t *buffer, size_t buffer_length);

  /// @brief Decode a single FLAC frame into PCM samples
  ///
  /// Outputs interleaved PCM samples. The output buffer must be large enough to hold
  /// the decoded samples (use get_output_buffer_size_bytes() to determine size).
  ///
  /// @param buffer Pointer to buffer containing frame data
  /// @param buffer_length Number of bytes in buffer
  /// @param output_buffer Pointer to output buffer for PCM samples (must be pre-allocated)
  /// @param num_samples Pointer to receive the number of samples decoded per channel
  /// @return FLAC_DECODER_SUCCESS on success
  ///         FLAC_DECODER_NO_MORE_FRAMES when end of stream reached
  ///         Error code on failure
  FLACDecoderResult decode_frame(const uint8_t *buffer, size_t buffer_length, uint8_t *output_buffer,
                                 uint32_t *num_samples);

  // ========================================
  // Stream Information Getters
  // ========================================

  /// Get number of audio channels (1=mono, 2=stereo, etc.)
  uint32_t get_num_channels() const { return this->num_channels_; }

  /// Get total number of samples in the stream (0 if unknown)
  uint64_t get_num_samples() const { return this->num_samples_; }

  /// Get number of bytes per sample in output (e.g., 2 for 16-bit, 3 for 24-bit)
  /// Returns 4 when 32-bit output mode is enabled
  uint32_t get_output_bytes_per_sample() const {
    if (this->output_32bit_samples_) {
      return 4;
    }
    return (this->sample_depth_ + 7) / 8;
  }

  /// Get sample rate in Hz (e.g., 44100, 48000)
  uint32_t get_sample_rate() const { return this->sample_rate_; }

  /// Get bit depth per sample (e.g., 16, 24)
  uint32_t get_sample_depth() const { return this->sample_depth_; }

  /// Get minimum block size from STREAMINFO
  uint32_t get_min_block_size() const { return this->min_block_size_; }

  /// Get maximum block size from STREAMINFO
  uint32_t get_max_block_size() const { return this->max_block_size_; }

  /// Get MD5 signature from STREAMINFO (128-bit signature as 16-byte array)
  const uint8_t *get_md5_signature() const { return this->md5_signature_; }

  /// Get required output buffer size in samples (max_block_size * num_channels)
  uint32_t get_output_buffer_size() const { return this->max_block_size_ * this->num_channels_; }

  /// Get required output buffer size in bytes
  uint32_t get_output_buffer_size_bytes() const {
    return this->max_block_size_ * this->num_channels_ * this->get_output_bytes_per_sample();
  }

  // ========================================
  // Buffer State (for streaming)
  // ========================================

  /// Get current position in the input buffer (number of bytes consumed)
  std::size_t get_bytes_index() const { return this->buffer_index_; }

  // ========================================
  // Metadata Access
  // ========================================

  /// @brief Get all decoded metadata blocks
  /// @return Vector of all metadata blocks parsed from the file
  const std::vector<FLACMetadataBlock> &get_metadata_blocks() const { return this->metadata_blocks_; }

  /// @brief Get a specific metadata block by type
  /// @param type The metadata type to find (e.g., FLAC_METADATA_TYPE_PICTURE for album art)
  /// @return Pointer to the metadata block, or nullptr if not found
  const FLACMetadataBlock *get_metadata_block(FLACMetadataType type) const {
    for (const auto &block : this->metadata_blocks_) {
      if (block.type == type) {
        return &block;
      }
    }
    return nullptr;
  }

  // ========================================
  // Configuration
  // ========================================

  /// @brief Set maximum album art size to store
  ///
  /// PICTURE metadata blocks larger than this size will be discarded to save memory.
  /// Set to 0 to disable album art storage (default on memory-constrained devices).
  ///
  /// @param max_size Maximum size in bytes (0 = disabled)
  void set_max_album_art_size(uint32_t max_size) { this->max_album_art_size_ = max_size; }

  /// Get current album art size limit
  uint32_t get_max_album_art_size() const { return this->max_album_art_size_; }

  /// @brief Set maximum metadata block size for a specific type
  ///
  /// Controls which metadata blocks are stored during header parsing. Blocks larger
  /// than the specified size will be skipped to save memory. Set to 0 to skip that
  /// metadata type entirely.
  ///
  /// @param type The metadata type (e.g., FLAC_METADATA_TYPE_VORBIS_COMMENT)
  /// @param max_size Maximum size in bytes (0 = skip this type)
  void set_max_metadata_size(FLACMetadataType type, uint32_t max_size);

  /// @brief Get current maximum metadata block size for a specific type
  /// @param type The metadata type
  /// @return Maximum size in bytes (0 = this type is skipped)
  uint32_t get_max_metadata_size(FLACMetadataType type) const;

  /// @brief Enable or disable CRC checking
  ///
  /// When enabled, frame CRC16 values are validated. Disabling can improve performance
  /// but may allow corrupted data to go undetected.
  ///
  /// @param enabled true to enable CRC checks (default), false to disable
  void set_crc_check_enabled(bool enabled) { this->enable_crc_check_ = enabled; }

  /// Get current CRC checking state
  bool get_crc_check_enabled() const { return this->enable_crc_check_; }

  /// @brief Enable or disable 32-bit sample output mode
  ///
  /// When enabled, all samples are output as 32-bit values regardless of the
  /// original bit depth. Samples are left-justified (MSB-aligned), so 24-bit
  /// audio is shifted left by 8, 16-bit by 16, etc. This simplifies downstream
  /// processing on embedded devices by avoiding 3-byte packed samples.
  ///
  /// @param enabled true to enable 32-bit output, false for native packing (default)
  void set_output_32bit_samples(bool enabled) { this->output_32bit_samples_ = enabled; }

  /// Get current 32-bit sample output state
  bool get_output_32bit_samples() const { return this->output_32bit_samples_; }

 private:
  // ========================================
  // Frame Decoding
  // ========================================

  /// @brief Search for frame sync code (0xFFF8) in the buffer
  FLACDecoderResult find_frame_sync(uint8_t &sync_byte_0, uint8_t &sync_byte_1);

  /// @brief Parse frame header and validate CRC8
  FLACDecoderResult decode_frame_header();

  /// @brief Decode all subframes for the current frame (handles channel decorrelation)
  FLACDecoderResult decode_subframes(uint32_t block_size, uint32_t sample_depth, uint32_t channel_assignment);

  /// @brief Decode a single subframe (dispatches to constant/verbatim/fixed/LPC)
  FLACDecoderResult decode_subframe(uint32_t block_size, uint32_t sample_depth, std::size_t block_samples_offset);

  /// @brief Decode fixed prediction subframe (orders 0-4)
  FLACDecoderResult decode_fixed_subframe(uint32_t block_size, std::size_t block_samples_offset, uint32_t pre_order,
                                          uint32_t sample_depth);

  /// @brief Decode linear predictive coding subframe
  FLACDecoderResult decode_lpc_subframe(uint32_t block_size, std::size_t block_samples_offset, uint32_t lpc_order,
                                        uint32_t sample_depth);

  /// @brief Decode Rice-coded residuals
  FLACDecoderResult decode_residuals(int32_t *buffer, size_t warm_up_samples, uint32_t block_size);

  // ========================================
  // Bit Stream Reading
  // ========================================

  /// @brief Read one byte from bit buffer (must be byte-aligned)
  uint32_t read_aligned_byte();

  /// @brief Refill bit buffer from input stream (reads 4 bytes if available)
  inline bool refill_bit_buffer();

  /// @brief Read unsigned integer of specified bit width
  inline uint32_t read_uint(std::size_t num_bits);

  /// @brief Read signed integer of specified bit width (two's complement)
  inline int32_t read_sint(std::size_t num_bits);

  /// @brief Read Rice-coded signed integer
  inline int32_t read_rice_sint(uint8_t param);

  /// @brief Discard bits to align to next byte boundary
  void align_to_byte();

  /// @brief Reset bit buffer state and adjust buffer pointers (must be byte-aligned)
  void reset_bit_buffer();

  // ========================================
  // Internal Utilities
  // ========================================

  /// @brief Free allocated buffers (called by destructor)
  void free_buffers();

  // ========================================
  // Sample Output Helpers
  // ========================================

  /// @brief Write decoded samples to output buffer using 16-bit stereo fast path
  void write_samples_16bit_stereo(uint8_t *output_buffer, uint32_t block_size);

  /// @brief Write decoded samples to output buffer using 16-bit mono fast path
  void write_samples_16bit_mono(uint8_t *output_buffer, uint32_t block_size);

  /// @brief Write decoded samples to output buffer using 24-bit stereo fast path
  void write_samples_24bit_stereo(uint8_t *output_buffer, uint32_t block_size);

  /// @brief Write decoded samples to output buffer using general path
  void write_samples_general(uint8_t *output_buffer, uint32_t block_size, uint32_t bytes_per_sample,
                             uint32_t shift_amount, uint32_t sample_depth);

  /// @brief Write decoded samples to output buffer using 32-bit stereo fast path
  void write_samples_32bit_stereo(uint8_t *output_buffer, uint32_t block_size, uint32_t shift_amount);

  /// @brief Write decoded samples to output buffer using 32-bit mono fast path
  void write_samples_32bit_mono(uint8_t *output_buffer, uint32_t block_size, uint32_t shift_amount);

  /// @brief Write decoded samples to output buffer using 32-bit general path (>2 channels)
  void write_samples_32bit_general(uint8_t *output_buffer, uint32_t block_size, uint32_t shift_amount);

  // ========================================
  // Input Buffer State
  // ========================================
  const uint8_t *buffer_ = nullptr;    // Pointer to current input buffer
  std::size_t buffer_index_ = 0;       // Current position in input buffer
  std::size_t bytes_left_ = 0;         // Bytes remaining in input buffer
  std::size_t frame_start_index_ = 0;  // Start position of current frame (for CRC)

  // ========================================
  // Bit Buffer State
  // ========================================
  uint32_t bit_buffer_ = 0;            // 32-bit buffer for bit-level reading
  std::size_t bit_buffer_length_ = 0;  // Number of valid bits in bit_buffer_

  // ========================================
  // Stream Properties (from STREAMINFO)
  // ========================================
  uint32_t min_block_size_ = 0;  // Minimum block size in samples
  uint32_t max_block_size_ = 0;  // Maximum block size in samples
  uint32_t sample_rate_ = 0;     // Sample rate in Hz
  uint32_t num_channels_ = 0;    // Number of audio channels
  uint32_t sample_depth_ = 0;    // Bits per sample
  uint64_t num_samples_ = 0;     // Total samples in stream (0 if unknown, 36-bit value)
  uint8_t md5_signature_[16]{};  // MD5 signature of unencoded audio data

  // ========================================
  // Current Frame State
  // ========================================
  uint32_t curr_frame_block_size_ = 0;      // Block size of current frame
  uint32_t curr_frame_channel_assign_ = 0;  // Channel assignment of current frame
  uint32_t curr_frame_sample_depth_ = 0;    // Sample depth of current frame

  // ========================================
  // Decode Buffers
  // ========================================
  int32_t *block_samples_ = nullptr;  // Working buffer for decoded samples (all channels)

  // ========================================
  // Decoder State Flags
  // ========================================
  bool out_of_data_ = false;           // Flag indicating end of input data reached
  bool enable_crc_check_ = true;       // Flag to enable/disable CRC validation
  bool output_32bit_samples_ = false;  // Output all samples as 32-bit

  // ========================================
  // Header Parsing State (for streaming)
  // ========================================
  bool partial_header_read_{false};           // In middle of reading header
  bool partial_header_last_{false};           // Current metadata block is the last one
  uint32_t partial_header_type_{0};           // Type of current metadata block being read
  uint32_t partial_header_length_{0};         // Total length of current metadata block
  uint32_t partial_header_bytes_read_{0};     // Bytes read so far for current block
  std::vector<uint8_t> partial_header_data_;  // Accumulated data for current block

  // ========================================
  // Metadata Storage
  // ========================================
  std::vector<FLACMetadataBlock> metadata_blocks_;            // All decoded metadata blocks
  uint32_t max_album_art_size_ = DEFAULT_MAX_ALBUM_ART_SIZE;  // Max album art size to store
  uint32_t max_padding_size_ = DEFAULT_MAX_PADDING_SIZE;
  uint32_t max_application_size_ = DEFAULT_MAX_APPLICATION_SIZE;
  uint32_t max_seektable_size_ = DEFAULT_MAX_SEEKTABLE_SIZE;
  uint32_t max_vorbis_comment_size_ = DEFAULT_MAX_VORBIS_COMMENT_SIZE;
  uint32_t max_cuesheet_size_ = DEFAULT_MAX_CUESHEET_SIZE;
  uint32_t max_unknown_size_ = DEFAULT_MAX_UNKNOWN_SIZE;
};

}  // namespace flac
}  // namespace esp_audio_libs
