#include "flac_decoder.h"

#include "flac_crc.h"
#include "flac_lpc.h"

#include <algorithm>
#include <cassert>
#include <cstring>

// Define optimization attribute for GCC/ESP32 builds
// This ensures PlatformIO builds get O3 optimization for critical functions
#if defined(__GNUC__) && !defined(__clang__)
#define FLAC_OPTIMIZE_O3 __attribute__((optimize("O3")))
#else
#define FLAC_OPTIMIZE_O3
#endif

namespace esp_audio_libs {
namespace flac {

static const uint32_t MAGIC_NUMBER = 0x664C6143;  // 'fLaC'

static const uint32_t UINT_MASK[] = {0x00000000, 0x00000001, 0x00000003, 0x00000007, 0x0000000f, 0x0000001f, 0x0000003f,
                                     0x0000007f, 0x000000ff, 0x000001ff, 0x000003ff, 0x000007ff, 0x00000fff, 0x00001fff,
                                     0x00003fff, 0x00007fff, 0x0000ffff, 0x0001ffff, 0x0003ffff, 0x0007ffff, 0x000fffff,
                                     0x001fffff, 0x003fffff, 0x007fffff, 0x00ffffff, 0x01ffffff, 0x03ffffff, 0x07ffffff,
                                     0x0fffffff, 0x1fffffff, 0x3fffffff, 0x7fffffff, 0xffffffff};

static const std::vector<int32_t> FIXED_COEFFICIENTS[] = {{}, {1}, {-1, 2}, {1, -3, 3}, {-1, 4, -6, 4}};

// ============================================================================
// Header Parsing
// ============================================================================

FLACDecoderResult FLACDecoder::read_header(const uint8_t *buffer, size_t buffer_length) {
  this->buffer_ = buffer;
  this->buffer_index_ = 0;
  this->bytes_left_ = buffer_length;
  this->bit_buffer_ = 0;
  this->bit_buffer_length_ = 0;

  this->out_of_data_ = (buffer_length == 0);

  if (!this->partial_header_read_) {
    this->metadata_blocks_.clear();
    this->partial_header_data_.clear();

    // File must start with 'fLaC'
    if (this->read_uint(32) != MAGIC_NUMBER) {
      return FLAC_DECODER_ERROR_BAD_MAGIC_NUMBER;
    }
  }

  while (!this->partial_header_last_ || (this->partial_header_length_ > 0)) {
    if (this->bytes_left_ == 0) {
      this->partial_header_read_ = true;
      this->reset_bit_buffer();
      return FLAC_DECODER_HEADER_OUT_OF_DATA;
    }

    if (this->partial_header_length_ == 0) {
      this->partial_header_last_ = this->read_uint(1) != 0;
      this->partial_header_type_ = this->read_uint(7);
      this->partial_header_length_ = this->read_uint(24);
      this->partial_header_bytes_read_ = 0;
      this->partial_header_data_.clear();
    }

    // Determine if we should skip this metadata block due to size limits
    bool should_skip = false;
    if (this->partial_header_type_ != FLAC_METADATA_TYPE_STREAMINFO) {
      uint32_t max_size = 0;
      switch (this->partial_header_type_) {
        case FLAC_METADATA_TYPE_PADDING:
          max_size = this->max_padding_size_;
          break;
        case FLAC_METADATA_TYPE_APPLICATION:
          max_size = this->max_application_size_;
          break;
        case FLAC_METADATA_TYPE_SEEKTABLE:
          max_size = this->max_seektable_size_;
          break;
        case FLAC_METADATA_TYPE_VORBIS_COMMENT:
          max_size = this->max_vorbis_comment_size_;
          break;
        case FLAC_METADATA_TYPE_CUESHEET:
          max_size = this->max_cuesheet_size_;
          break;
        case FLAC_METADATA_TYPE_PICTURE:
          max_size = this->max_album_art_size_;
          break;
        default:
          max_size = this->max_unknown_size_;
          break;
      }

      if (this->partial_header_length_ > max_size) {
        should_skip = true;
      }
    }

    if (this->partial_header_type_ == FLAC_METADATA_TYPE_STREAMINFO) {
      this->min_block_size_ = this->read_uint(16);
      this->max_block_size_ = this->read_uint(16);
      this->read_uint(24);  // min frame size
      this->read_uint(24);  // max frame size

      this->sample_rate_ = this->read_uint(20);
      this->num_channels_ = this->read_uint(3) + 1;
      this->sample_depth_ = this->read_uint(5) + 1;
      // Total samples is 36 bits - read high 4 bits and low 32 bits separately
      uint64_t num_samples_high = this->read_uint(4);
      uint64_t num_samples_low = this->read_uint(32);
      this->num_samples_ = (num_samples_high << 32) | num_samples_low;
      // Read MD5 signature (128 bits = 16 bytes)
      for (int i = 0; i < 16; i++) {
        this->md5_signature_[i] = this->read_uint(8);
      }

      this->partial_header_length_ = 0;
      this->partial_header_bytes_read_ = 0;
    } else if (should_skip) {
      uint32_t bytes_to_skip = std::min(this->partial_header_length_ - this->partial_header_bytes_read_,
                                        static_cast<uint32_t>(this->bytes_left_));

      for (uint32_t i = 0; i < bytes_to_skip; i++) {
        this->read_uint(8);
        this->partial_header_bytes_read_++;
      }

      if (this->partial_header_bytes_read_ == this->partial_header_length_) {
        this->partial_header_length_ = 0;
        this->partial_header_bytes_read_ = 0;
        this->partial_header_data_.clear();
      }
    } else if (!should_skip) {
      if (this->partial_header_data_.capacity() < this->partial_header_length_) {
        this->partial_header_data_.reserve(this->partial_header_length_);
      }

      uint32_t bytes_to_read = std::min(this->partial_header_length_ - this->partial_header_bytes_read_,
                                        static_cast<uint32_t>(this->bytes_left_));

      for (uint32_t i = 0; i < bytes_to_read; i++) {
        this->partial_header_data_.push_back(this->read_uint(8));
        this->partial_header_bytes_read_++;
      }

      if (this->partial_header_bytes_read_ == this->partial_header_length_) {
        FLACMetadataBlock block;
        block.type = static_cast<FLACMetadataType>(this->partial_header_type_);
        block.length = this->partial_header_length_;
        block.data = std::move(this->partial_header_data_);
        this->metadata_blocks_.push_back(std::move(block));

        this->partial_header_length_ = 0;
        this->partial_header_bytes_read_ = 0;
        this->partial_header_data_.clear();
      }
    }
  }

  if ((this->sample_rate_ == 0) || (this->num_channels_ == 0) || (this->sample_depth_ == 0) ||
      (this->max_block_size_ == 0)) {
    return FLAC_DECODER_ERROR_BAD_HEADER;
  }

  if ((this->min_block_size_ < 16) || (this->min_block_size_ > this->max_block_size_) ||
      (this->max_block_size_ > 65535)) {
    return FLAC_DECODER_ERROR_BAD_HEADER;
  }

  this->reset_bit_buffer();

  return FLAC_DECODER_SUCCESS;
}

// ============================================================================
// Frame Decoding
// ============================================================================

// Force O3 optimization for decode_frame (performance critical)
// This ensures PlatformIO builds also get optimal performance
FLAC_OPTIMIZE_O3
FLACDecoderResult FLACDecoder::decode_frame(const uint8_t *buffer, size_t buffer_length, uint8_t *output_buffer,
                                            uint32_t *num_samples) {
  this->buffer_ = buffer;
  this->buffer_index_ = 0;
  this->bytes_left_ = buffer_length;
  this->out_of_data_ = false;

  FLACDecoderResult ret = FLAC_DECODER_SUCCESS;

  *num_samples = 0;

  if (!this->block_samples_) {
    this->block_samples_ = (int32_t *) FLAC_MALLOC(this->max_block_size_ * this->num_channels_ * sizeof(int32_t));
  }
  if (!this->block_samples_) {
    return FLAC_DECODER_ERROR_MEMORY_ALLOCATION_ERROR;
  }

  if (this->bytes_left_ == 0) {
    return FLAC_DECODER_NO_MORE_FRAMES;
  }

  ret = this->decode_frame_header();
  if (ret != FLAC_DECODER_SUCCESS) {
    this->reset_bit_buffer();
    return ret;
  }

  // Memory is allocated based on the maximum block size.
  // Ensure that no out-of-bounds access occurs, particularly in case of parsing errors.
  if (this->curr_frame_block_size_ > this->max_block_size_) {
    return FLAC_DECODER_ERROR_BLOCK_SIZE_OUT_OF_RANGE;
  }

  this->decode_subframes(this->curr_frame_block_size_, this->curr_frame_sample_depth_,
                         this->curr_frame_channel_assign_);
  *num_samples = this->curr_frame_block_size_ * this->num_channels_;

  this->align_to_byte();

  if (this->bit_buffer_length_ / 8 + this->bytes_left_ < 2) {
    // Unable to read the last two CRC bytes
    this->reset_bit_buffer();
    return FLAC_DECODER_ERROR_OUT_OF_DATA;
  }

  size_t frame_end_index = this->buffer_index_ - this->bit_buffer_length_ / 8;

  uint16_t crc_read = this->read_uint(16);

  if (this->enable_crc_check_ && frame_end_index > this->frame_start_index_) {
    size_t frame_length = frame_end_index - this->frame_start_index_;
    uint16_t calculated_crc = calculate_crc16(buffer + this->frame_start_index_, frame_length);

    if (calculated_crc != crc_read) {
      return FLAC_DECODER_ERROR_CRC_MISMATCH;
    }
  }

  // Write decoded samples to output buffer using optimized fast paths
  if (this->output_32bit_samples_) {
    // 32-bit output mode: all samples output as 4 bytes, left-justified (MSB-aligned)
    uint32_t shift_amount = 32 - this->curr_frame_sample_depth_;

    if (this->num_channels_ == 2) {
      this->write_samples_32bit_stereo(output_buffer, this->curr_frame_block_size_, shift_amount);
    } else if (this->num_channels_ == 1) {
      this->write_samples_32bit_mono(output_buffer, this->curr_frame_block_size_, shift_amount);
    } else {
      this->write_samples_32bit_general(output_buffer, this->curr_frame_block_size_, shift_amount);
    }
  } else {
    // Native output mode: pack to nearest byte boundary
    uint32_t bytes_per_sample = (this->curr_frame_sample_depth_ + 7) / 8;
    uint32_t shift_amount = 0;
    if (this->curr_frame_sample_depth_ % 8 != 0) {
      shift_amount = 8 - (this->curr_frame_sample_depth_ % 8);
    }

    if (this->curr_frame_sample_depth_ == 16 && shift_amount == 0 && this->num_channels_ == 2) {
      this->write_samples_16bit_stereo(output_buffer, this->curr_frame_block_size_);
    } else if (this->curr_frame_sample_depth_ == 16 && shift_amount == 0 && this->num_channels_ == 1) {
      this->write_samples_16bit_mono(output_buffer, this->curr_frame_block_size_);
    } else if (this->curr_frame_sample_depth_ == 24 && shift_amount == 0 && this->num_channels_ == 2) {
      this->write_samples_24bit_stereo(output_buffer, this->curr_frame_block_size_);
    } else {
      this->write_samples_general(output_buffer, this->curr_frame_block_size_, bytes_per_sample, shift_amount,
                                  this->curr_frame_sample_depth_);
    }
  }

  this->reset_bit_buffer();
  return FLAC_DECODER_SUCCESS;
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_16bit_stereo(uint8_t *output_buffer, uint32_t block_size) {
  // 16-bit stereo fast path
  int16_t *output_samples = reinterpret_cast<int16_t *>(output_buffer);

  for (uint32_t i = 0; i < block_size; ++i) {
    output_samples[2 * i] = this->block_samples_[i];
    output_samples[2 * i + 1] = this->block_samples_[block_size + i];
  }
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_16bit_mono(uint8_t *output_buffer, uint32_t block_size) {
  // 16-bit mono fast path
  int16_t *output_samples = reinterpret_cast<int16_t *>(output_buffer);

  for (uint32_t i = 0; i < block_size; ++i) {
    output_samples[i] = this->block_samples_[i];
  }
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_24bit_stereo(uint8_t *output_buffer, uint32_t block_size) {
  // 24-bit stereo fast path with 2-sample unrolling (due to larger sample size)
  std::size_t output_index = 0;
  uint32_t i = 0;
  const uint32_t unroll_limit = block_size & ~1U;  // Round down to multiple of 2

  // Process 2 samples at a time
  for (; i < unroll_limit; i += 2) {
    // Sample 0 - Left and Right channels
    int32_t sample_0_l = this->block_samples_[i];
    int32_t sample_0_r = this->block_samples_[block_size + i];
    // Sample 1 - Left and Right channels
    int32_t sample_1_l = this->block_samples_[i + 1];
    int32_t sample_1_r = this->block_samples_[block_size + i + 1];

    // Direct 24-bit writes (little-endian)
    output_buffer[output_index++] = sample_0_l & 0xFF;
    output_buffer[output_index++] = (sample_0_l >> 8) & 0xFF;
    output_buffer[output_index++] = (sample_0_l >> 16) & 0xFF;
    output_buffer[output_index++] = sample_0_r & 0xFF;
    output_buffer[output_index++] = (sample_0_r >> 8) & 0xFF;
    output_buffer[output_index++] = (sample_0_r >> 16) & 0xFF;

    output_buffer[output_index++] = sample_1_l & 0xFF;
    output_buffer[output_index++] = (sample_1_l >> 8) & 0xFF;
    output_buffer[output_index++] = (sample_1_l >> 16) & 0xFF;
    output_buffer[output_index++] = sample_1_r & 0xFF;
    output_buffer[output_index++] = (sample_1_r >> 8) & 0xFF;
    output_buffer[output_index++] = (sample_1_r >> 16) & 0xFF;
  }

  // Handle remaining samples
  for (; i < block_size; i++) {
    int32_t sample_l = this->block_samples_[i];
    int32_t sample_r = this->block_samples_[block_size + i];

    output_buffer[output_index++] = sample_l & 0xFF;
    output_buffer[output_index++] = (sample_l >> 8) & 0xFF;
    output_buffer[output_index++] = (sample_l >> 16) & 0xFF;
    output_buffer[output_index++] = sample_r & 0xFF;
    output_buffer[output_index++] = (sample_r >> 8) & 0xFF;
    output_buffer[output_index++] = (sample_r >> 16) & 0xFF;
  }
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_general(uint8_t *output_buffer, uint32_t block_size, uint32_t bytes_per_sample,
                                        uint32_t shift_amount, uint32_t sample_depth) {
  // General case with 4-sample unrolling where possible
  std::size_t output_index = 0;
  uint32_t i = 0;
  const uint32_t unroll_limit = block_size & ~3U;  // Round down to multiple of 4

  // Process 4 samples at a time
  for (; i < unroll_limit; i += 4) {
    // Unroll 4 samples
    for (uint32_t sample_offset = 0; sample_offset < 4; sample_offset++) {
      for (uint32_t j = 0; j < this->num_channels_; j++) {
        int32_t sample = this->block_samples_[(j * block_size) + i + sample_offset];

        if (sample_depth == 8) {
          sample += 128;
        }

        if (shift_amount > 0) {
          sample <<= shift_amount;
        }

        for (uint32_t byte = 0; byte < bytes_per_sample; byte++) {
          output_buffer[output_index++] = (sample >> (byte * 8)) & 0xFF;
        }
      }
    }
  }

  // Handle remaining samples
  for (; i < block_size; i++) {
    for (uint32_t j = 0; j < this->num_channels_; j++) {
      int32_t sample = this->block_samples_[(j * block_size) + i];

      if (sample_depth == 8) {
        sample += 128;
      }

      if (shift_amount > 0) {
        sample <<= shift_amount;
      }

      for (uint32_t byte = 0; byte < bytes_per_sample; byte++) {
        output_buffer[output_index++] = (sample >> (byte * 8)) & 0xFF;
      }
    }
  }
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_32bit_stereo(uint8_t *output_buffer, uint32_t block_size, uint32_t shift_amount) {
  int32_t *output_samples = reinterpret_cast<int32_t *>(output_buffer);
  const int32_t *left = this->block_samples_;
  const int32_t *right = this->block_samples_ + block_size;

  for (uint32_t i = 0; i < block_size; ++i) {
    output_samples[i * 2] = left[i] << shift_amount;
    output_samples[i * 2 + 1] = right[i] << shift_amount;
  }
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_32bit_mono(uint8_t *output_buffer, uint32_t block_size, uint32_t shift_amount) {
  int32_t *output_samples = reinterpret_cast<int32_t *>(output_buffer);
  const int32_t *samples = this->block_samples_;

  for (uint32_t i = 0; i < block_size; ++i) {
    output_samples[i] = samples[i] << shift_amount;
  }
}

FLAC_OPTIMIZE_O3
void FLACDecoder::write_samples_32bit_general(uint8_t *output_buffer, uint32_t block_size, uint32_t shift_amount) {
  int32_t *output_samples = reinterpret_cast<int32_t *>(output_buffer);
  uint32_t output_index = 0;

  for (uint32_t i = 0; i < block_size; ++i) {
    for (uint32_t ch = 0; ch < this->num_channels_; ++ch) {
      output_samples[output_index++] = this->block_samples_[ch * block_size + i] << shift_amount;
    }
  }
}

FLACDecoderResult FLACDecoder::find_frame_sync(uint8_t &sync_byte_0, uint8_t &sync_byte_1) {
  this->frame_start_index_ = 0;

  sync_byte_0 = 0;
  sync_byte_1 = 0;

  bool second_ff_byte_found = false;
  uint32_t byte;

  this->align_to_byte();

  while (true) {
    if (second_ff_byte_found) {
      // try if the prev found 0xff is first of the MAGIC NUMBER
      byte = 0xff;
      second_ff_byte_found = false;
    } else {
      byte = this->read_aligned_byte();
      ++this->frame_start_index_;
    }
    if (byte == 0xff) {
      byte = this->read_aligned_byte();
      ++this->frame_start_index_;
      if (byte == 0xff) {
        // found a second 0xff, could be the first byte of the MAGIC NUMBER
        second_ff_byte_found = true;
      } else if (byte >> 1 == 0x7c) { /* MAGIC NUMBER for the last 6 sync bits and reserved 7th bit */
        sync_byte_0 = 0xff;
        sync_byte_1 = byte;
        this->frame_start_index_ -= 2;  // Include the frame sync code in the start index
        return FLAC_DECODER_SUCCESS;
      }
    } else if (this->out_of_data_) {
      return FLAC_DECODER_ERROR_SYNC_NOT_FOUND;
    }
  }
  return FLAC_DECODER_ERROR_SYNC_NOT_FOUND;
}

FLACDecoderResult FLACDecoder::decode_frame_header() {
  uint8_t raw_header[16];  // Max frame header: 4 fixed + 7 UTF-8 + 2 block size + 2 sample rate = 15 bytes
  uint32_t raw_header_len = 0;
  uint32_t new_byte;
  uint8_t sync_byte_0, sync_byte_1;

  if (this->find_frame_sync(sync_byte_0, sync_byte_1) != FLAC_DECODER_SUCCESS) {
    return FLAC_DECODER_ERROR_SYNC_NOT_FOUND;
  }

  raw_header[raw_header_len++] = sync_byte_0;
  raw_header[raw_header_len++] = sync_byte_1;

  /* make sure that reserved bit is 0 */
  if (raw_header[1] & 0x02) { /* MAGIC NUMBER */
    return FLAC_DECODER_ERROR_BAD_MAGIC_NUMBER;
  }

  new_byte = this->read_aligned_byte();
  if (new_byte == 0xff) { /* MAGIC NUMBER for the first 8 frame sync bits */
    /* if we get here it means our original sync was erroneous since the sync code cannot appear in the header */
    // needs to search for sync code again
    return FLAC_DECODER_ERROR_SYNC_NOT_FOUND;
  }
  raw_header[raw_header_len++] = new_byte;

  // 9.1.1 Block size bits
  uint8_t block_size_code = raw_header[2] >> 4;
  if (block_size_code == 0) {
    return FLAC_DECODER_ERROR_BAD_BLOCK_SIZE_CODE;
  } else if (block_size_code == 1) {
    this->curr_frame_block_size_ = 192;
  } else if ((2 <= block_size_code) && (block_size_code <= 5)) {
    this->curr_frame_block_size_ = 576 << (block_size_code - 2);
  } else if (block_size_code == 6) {
    // gets parsed later
  } else if (block_size_code == 7) {
    // gets parsed later
  } else if (block_size_code <= 15) {
    this->curr_frame_block_size_ = 256 << (block_size_code - 8);
  } else {
    return FLAC_DECODER_ERROR_BAD_BLOCK_SIZE_CODE;
  }

  // 9.1.2 Sample rate bits
  uint8_t sample_rate_code = raw_header[2] & 0x0f;

  // 9.1.3 Channel bits
  new_byte = this->read_aligned_byte();
  if (new_byte == 0xff) { /* MAGIC NUMBER for the first 8 frame sync bits */
    /* if we get here it means our original sync was erroneous since the sync code cannot appear in the header */
    // needs to search for sync code again
    return FLAC_DECODER_ERROR_SYNC_NOT_FOUND;
  }
  raw_header[raw_header_len++] = new_byte;
  this->curr_frame_channel_assign_ = raw_header[3] >> 4;

  // 9.1.4 Bit depth bits
  uint8_t bits_per_sample_code = (raw_header[3] & 0x0e) >> 1;
  switch (bits_per_sample_code) {
    case 0:
      // take bit depth from streaminfo header
      this->curr_frame_sample_depth_ = this->sample_depth_;
      break;
    case 1:
      this->curr_frame_sample_depth_ = 8;
      break;
    case 2:
      this->curr_frame_sample_depth_ = 12;
      break;
    case 3:
      // reserved
      return FLAC_DECODER_ERROR_BAD_SAMPLE_DEPTH;
    case 4:
      this->curr_frame_sample_depth_ = 16;
      break;
    case 5:
      this->curr_frame_sample_depth_ = 20;
      break;
    case 6:
      this->curr_frame_sample_depth_ = 24;
      break;
    case 7:
      this->curr_frame_sample_depth_ = 32;
      break;
    default:
      // Should never reach here but keep for safety
      return FLAC_DECODER_ERROR_BAD_SAMPLE_DEPTH;
  }

  // Reserved bit (raw_header[3] & 0x01) not checked - some encoders don't respect it

  // 9.1.5. Coded number
  // UTF-8 like variable length code - we don't support seeking so we skip validation
  uint32_t next_int = this->read_aligned_byte();
  raw_header[raw_header_len++] = next_int;
  while (next_int >= 0b11000000) {
    uint8_t next_byte = this->read_aligned_byte();
    raw_header[raw_header_len++] = next_byte;
    next_int = (next_int << 1) & 0xFF;
  }

  // 9.1.6 Uncommon block size
  if (block_size_code == 6) {
    uint8_t size_byte = this->read_aligned_byte();
    raw_header[raw_header_len++] = size_byte;
    this->curr_frame_block_size_ = size_byte + 1;
  } else if (block_size_code == 7) {
    uint8_t size_byte1 = this->read_aligned_byte();
    raw_header[raw_header_len++] = size_byte1;
    this->curr_frame_block_size_ = size_byte1 << 8;
    uint8_t size_byte2 = this->read_aligned_byte();
    raw_header[raw_header_len++] = size_byte2;
    this->curr_frame_block_size_ |= size_byte2;
    this->curr_frame_block_size_ += 1;
  }

  // 9.1.7 Uncommon sample rate
  uint32_t frame_sample_rate = 0;
  if (sample_rate_code == 12) {
    uint8_t rate_byte = this->read_aligned_byte();
    raw_header[raw_header_len++] = rate_byte;
    frame_sample_rate = rate_byte * 1000;
  } else if (sample_rate_code == 13) {
    uint8_t rate_byte1 = this->read_aligned_byte();
    raw_header[raw_header_len++] = rate_byte1;
    uint8_t rate_byte2 = this->read_aligned_byte();
    raw_header[raw_header_len++] = rate_byte2;
    frame_sample_rate = (rate_byte1 << 8) | rate_byte2;
  } else if (sample_rate_code == 14) {
    uint8_t rate_byte1 = this->read_aligned_byte();
    raw_header[raw_header_len++] = rate_byte1;
    uint8_t rate_byte2 = this->read_aligned_byte();
    raw_header[raw_header_len++] = rate_byte2;
    frame_sample_rate = ((rate_byte1 << 8) | rate_byte2) * 10;
  } else if (sample_rate_code == 0) {
    // Get from STREAMINFO
    frame_sample_rate = this->sample_rate_;
  } else {
    // Standard sample rate codes (1-11)
    static const uint32_t sample_rate_table[] = {88200, 176400, 192000, 8000,  16000, 22050,
                                                 24000, 32000,  44100,  48000, 96000};
    if (sample_rate_code >= 1 && sample_rate_code <= 11) {
      frame_sample_rate = sample_rate_table[sample_rate_code - 1];
    } else {
      // sample_rate_code == 15 is invalid/reserved
      return FLAC_DECODER_ERROR_BAD_HEADER;
    }
  }

  if (this->out_of_data_) {
    return FLAC_DECODER_ERROR_OUT_OF_DATA;
  }

  // 9.1.8 Frame header CRC
  uint8_t crc_read = this->read_aligned_byte();

  if (this->enable_crc_check_) {
    uint8_t crc_calculated = calculate_crc8(raw_header, raw_header_len);
    if (crc_calculated != crc_read) {
      return FLAC_DECODER_ERROR_CRC_MISMATCH;
    }
  }

  // Validate that frame channel count matches STREAMINFO
  uint32_t frame_channels;
  if (this->curr_frame_channel_assign_ <= 7) {
    frame_channels = this->curr_frame_channel_assign_ + 1;
  } else if (this->curr_frame_channel_assign_ <= 10) {
    frame_channels = 2;  // Stereo decorrelation modes
  } else {
    // Already validated as reserved in decode_subframes
    frame_channels = this->num_channels_;
  }

  if (frame_channels != this->num_channels_) {
    return FLAC_DECODER_ERROR_BAD_HEADER;
  }

  // Validate that frame bit depth matches STREAMINFO (when frame specifies bit depth)
  // bits_per_sample_code == 0 means use STREAMINFO value, which is always valid
  if (bits_per_sample_code != 0 && this->curr_frame_sample_depth_ != this->sample_depth_) {
    return FLAC_DECODER_ERROR_BAD_HEADER;
  }

  // Validate that frame sample rate matches STREAMINFO
  // The decoder does not support mid-stream sample rate changes
  if (frame_sample_rate != this->sample_rate_) {
    return FLAC_DECODER_ERROR_BAD_HEADER;
  }

  return FLAC_DECODER_SUCCESS;
}

// ============================================================================
// Subframe Decoding
// ============================================================================

FLAC_OPTIMIZE_O3
FLACDecoderResult FLACDecoder::decode_subframes(uint32_t block_size, uint32_t sample_depth,
                                                uint32_t channel_assignment) {
  FLACDecoderResult result = FLAC_DECODER_SUCCESS;
  if (channel_assignment <= 7) {
    std::size_t block_samples_offset = 0;
    for (std::size_t i = 0; i < channel_assignment + 1; i++) {
      result = this->decode_subframe(block_size, sample_depth, block_samples_offset);
      if (result != FLAC_DECODER_SUCCESS) {
        return result;
      }
      block_samples_offset += block_size;
    }
  } else if ((8 <= channel_assignment) && (channel_assignment <= 10)) {
    result = this->decode_subframe(block_size, sample_depth + ((channel_assignment == 9) ? 1 : 0), 0);
    if (result != FLAC_DECODER_SUCCESS) {
      return result;
    }
    result = this->decode_subframe(block_size, sample_depth + ((channel_assignment == 9) ? 0 : 1), block_size);
    if (result != FLAC_DECODER_SUCCESS) {
      return result;
    }

    if (channel_assignment == 8) {
      for (std::size_t i = 0; i < block_size; i++) {
        this->block_samples_[block_size + i] = this->block_samples_[i] - this->block_samples_[block_size + i];
      }
    } else if (channel_assignment == 9) {
      for (std::size_t i = 0; i < block_size; i++) {
        this->block_samples_[i] += this->block_samples_[block_size + i];
      }
    } else if (channel_assignment == 10) {
      for (std::size_t i = 0; i < block_size; i++) {
        int32_t side = this->block_samples_[block_size + i];
        int32_t right = this->block_samples_[i] - (side >> 1);
        this->block_samples_[block_size + i] = right;
        this->block_samples_[i] = right + side;
      }
    }
  } else {
    result = FLAC_DECODER_ERROR_RESERVED_CHANNEL_ASSIGNMENT;
  }

  return result;
}

FLAC_OPTIMIZE_O3
FLACDecoderResult FLACDecoder::decode_subframe(uint32_t block_size, uint32_t sample_depth,
                                               std::size_t block_samples_offset) {
  this->read_uint(1);

  uint32_t type = this->read_uint(6);
  uint32_t shift = this->read_uint(1);
  if (shift == 1) {
    while (this->read_uint(1) == 0) {
      shift += 1;

      if (this->out_of_data_) {
        return FLAC_DECODER_ERROR_OUT_OF_DATA;
      }
    }
  }

  sample_depth -= shift;

  FLACDecoderResult result = FLAC_DECODER_SUCCESS;
  if (type == 0) {
    // Constant
    int32_t value = this->read_sint(sample_depth) << shift;
    std::fill(this->block_samples_ + block_samples_offset, this->block_samples_ + block_samples_offset + block_size,
              value);
  } else if (type == 1) {
    // Verbatim
    for (std::size_t i = 0; i < block_size; i++) {
      this->block_samples_[block_samples_offset + i] = (this->read_sint(sample_depth) << shift);
    }
  } else if ((8 <= type) && (type <= 12)) {
    // Fixed prediction
    result = this->decode_fixed_subframe(block_size, block_samples_offset, type - 8, sample_depth);
    if (result != FLAC_DECODER_SUCCESS) {
      return result;
    }
    // Apply wasted bits shift after decoding fixed subframe
    if (shift > 0) {
      for (std::size_t i = 0; i < block_size; i++) {
        this->block_samples_[block_samples_offset + i] <<= shift;
      }
    }
  } else if ((32 <= type) && (type <= 63)) {
    // LPC (linear predictive coding)
    result = this->decode_lpc_subframe(block_size, block_samples_offset, type - 31, sample_depth);
    if (result != FLAC_DECODER_SUCCESS) {
      return result;
    }
    if (shift > 0) {
      for (std::size_t i = 0; i < block_size; i++) {
        this->block_samples_[block_samples_offset + i] <<= shift;
      }
    }
  } else {
    result = FLAC_DECODER_ERROR_RESERVED_SUBFRAME_TYPE;
  }

  return result;
}

FLAC_OPTIMIZE_O3
FLACDecoderResult FLACDecoder::decode_fixed_subframe(uint32_t block_size, std::size_t block_samples_offset,
                                                     uint32_t pre_order, uint32_t sample_depth) {
  if (pre_order > 4) {
    return FLAC_DECODER_ERROR_BAD_FIXED_PREDICTION_ORDER;
  }

  FLACDecoderResult result = FLAC_DECODER_SUCCESS;

  int32_t *const sub_frame_buffer = this->block_samples_ + block_samples_offset;
  int32_t *out_ptr = sub_frame_buffer;

  // warm-up samples
  for (std::size_t i = 0; i < pre_order; i++) {
    *(out_ptr++) = this->read_sint(sample_depth);
  }
  result = decode_residuals(sub_frame_buffer, pre_order, block_size);
  if (result != FLAC_DECODER_SUCCESS) {
    return result;
  }

  // For fixed prediction, quantization level is always 0
  // Check if we can use 32-bit arithmetic safely
  if (can_use_32bit_lpc(sample_depth, FIXED_COEFFICIENTS[pre_order].data(), pre_order, 0)) {
    restore_linear_prediction_32bit(sub_frame_buffer, block_size, FIXED_COEFFICIENTS[pre_order], 0);
  } else {
    restore_linear_prediction_64bit(sub_frame_buffer, block_size, FIXED_COEFFICIENTS[pre_order], 0);
  }

  return result;
}

FLAC_OPTIMIZE_O3
FLACDecoderResult FLACDecoder::decode_lpc_subframe(uint32_t block_size, std::size_t block_samples_offset,
                                                   uint32_t lpc_order, uint32_t sample_depth) {
  FLACDecoderResult result = FLAC_DECODER_SUCCESS;

  int32_t *const sub_frame_buffer = this->block_samples_ + block_samples_offset;
  int32_t *out_ptr = sub_frame_buffer;

  for (std::size_t i = 0; i < lpc_order; i++) {
    *(out_ptr++) = this->read_sint(sample_depth);
  }

  uint32_t precision = this->read_uint(4) + 1;
  int32_t shift = this->read_sint(5);

  std::vector<int32_t> coefs;
  coefs.resize(lpc_order);
  for (std::size_t i = 0; i < lpc_order; i++) {
    coefs[lpc_order - i - 1] = this->read_sint(precision);
  }

  result = decode_residuals(sub_frame_buffer, lpc_order, block_size);
  if (result != FLAC_DECODER_SUCCESS) {
    return result;
  }

  // Check if we can use 32-bit arithmetic safely
  if (can_use_32bit_lpc(sample_depth, coefs.data(), lpc_order, shift)) {
    restore_linear_prediction_32bit(sub_frame_buffer, block_size, coefs, shift);
  } else {
    restore_linear_prediction_64bit(sub_frame_buffer, block_size, coefs, shift);
  }

  return result;
}

FLAC_OPTIMIZE_O3
FLACDecoderResult FLACDecoder::decode_residuals(int32_t *sub_frame_buffer, size_t warm_up_samples,
                                                uint32_t block_size) {
  uint32_t method = this->read_uint(2);
  if (method >= 2) {
    return FLAC_DECODER_ERROR_RESERVED_RESIDUAL_CODING_METHOD;
  }

  uint32_t param_bits = 4;
  uint32_t escape_param = 0xF;
  if (method == 1) {
    param_bits = 5;
    escape_param = 0x1F;
  }

  uint32_t partition_order = this->read_uint(4);
  uint32_t num_partitions = 1 << partition_order;
  if ((block_size % num_partitions) != 0) {
    return FLAC_DECODER_ERROR_BLOCK_SIZE_NOT_DIVISIBLE_RICE;
  }

  int32_t *out_ptr = sub_frame_buffer + warm_up_samples;
  {
    uint32_t count = (block_size >> partition_order) - warm_up_samples;
    uint32_t param = this->read_uint(param_bits);
    if (param < escape_param) {
      for (std::size_t j = 0; j < count; j++) {
        *(out_ptr++) = this->read_rice_sint(param);
      }
    } else {
      std::size_t num_bits = this->read_uint(5);
      if (num_bits == 0) {
        std::memset(out_ptr, 0, count * sizeof(int32_t));
        out_ptr += count;
      } else {
        for (std::size_t j = 0; j < count; j++) {
          *(out_ptr++) = this->read_sint(num_bits);
        }
      }
    }
  }

  uint32_t count = block_size >> partition_order;
  for (std::size_t i = 1; i < num_partitions; i++) {
    uint32_t param = this->read_uint(param_bits);
    if (param < escape_param) {
      for (std::size_t j = 0; j < count; j++) {
        *(out_ptr++) = this->read_rice_sint(param);
      }
    } else {
      std::size_t num_bits = this->read_uint(5);
      if (num_bits == 0) {
        std::memset(out_ptr, 0, count * sizeof(int32_t));
        out_ptr += count;
      } else {
        for (std::size_t j = 0; j < count; j++) {
          *(out_ptr++) = this->read_sint(num_bits);
        }
      }
    }
  }

  return FLAC_DECODER_SUCCESS;
}

// ============================================================================
// Bit Stream Reading
// ============================================================================

void FLACDecoder::reset_bit_buffer() {
  assert(this->bit_buffer_length_ % 8 == 0);

  this->buffer_index_ -= (this->bit_buffer_length_ / 8);
  this->bytes_left_ += (this->bit_buffer_length_ / 8);
  this->bit_buffer_length_ = 0;
  this->bit_buffer_ = 0;
}

uint32_t FLACDecoder::read_aligned_byte() {
  // assumes byte alignment
  assert(this->bit_buffer_length_ % 8 == 0);

  return this->read_uint(8);
}

void FLACDecoder::align_to_byte() {
  if (this->bit_buffer_length_ >= 8) {
    this->bit_buffer_length_ -= (this->bit_buffer_length_ % 8);
  } else {
    this->bit_buffer_length_ = 0;
  }
}

inline bool FLACDecoder::refill_bit_buffer() {
  if (this->bytes_left_ >= 4) {
    uint32_t new_word;
    std::memcpy(&new_word, &this->buffer_[this->buffer_index_], sizeof(uint32_t));
    this->bit_buffer_ = __builtin_bswap32(new_word);
    this->bit_buffer_length_ = 32;
    this->buffer_index_ += 4;
    this->bytes_left_ -= 4;
    return false;
  } else if (this->bytes_left_ > 0) {
    for (int32_t i = 0; i < this->bytes_left_; ++i) {
      uint8_t next_byte = this->buffer_[this->buffer_index_++];
      this->bit_buffer_ = (this->bit_buffer_ << 8) | next_byte;
    }
    this->bit_buffer_length_ = 8 * this->bytes_left_;
    this->bytes_left_ = 0;
    return false;
  }
  return true;
}

inline uint32_t FLACDecoder::read_uint(std::size_t num_bits) {
  uint32_t result = 0;

  const int32_t new_bits_needed = num_bits - this->bit_buffer_length_;

  if (new_bits_needed > 0) {
    int32_t bytes_needed = (new_bits_needed + 7) / 8;

    if (this->bytes_left_ < bytes_needed) {
      this->out_of_data_ = true;
      return 0;
    }

    if (new_bits_needed < 32) {
      // Some of the current bits will be used in the result
      result = this->bit_buffer_ << new_bits_needed;
    }

    this->refill_bit_buffer();
    this->bit_buffer_length_ -= new_bits_needed;
  } else {
    this->bit_buffer_length_ -= num_bits;
  }

  result |= (this->bit_buffer_ >> this->bit_buffer_length_);

  result &= UINT_MASK[num_bits];

  return result;
}

inline int32_t FLACDecoder::read_sint(std::size_t num_bits) {
  // Handle 33-bit reads for side channel in 32-bit MID_SIDE stereo
  if (num_bits > 32) {
    // For 33-bit values, we need special handling
    // Read the upper bits first
    uint32_t upper_bits = this->read_uint(num_bits - 32);
    uint32_t lower_bits = this->read_uint(32);

    // Combine into a 64-bit value
    int64_t value = (static_cast<int64_t>(upper_bits) << 32) | lower_bits;

    // Sign extend from num_bits
    int64_t sign_bit = static_cast<int64_t>(1) << (num_bits - 1);
    if (value & sign_bit) {
      // Negative - sign extend
      int64_t mask = ~((static_cast<int64_t>(1) << num_bits) - 1);
      value |= mask;
    }

    // Truncate to 32 bits (may lose precision for 33-bit values)
    return static_cast<int32_t>(value);
  }

  uint32_t next_int = this->read_uint(num_bits);
  // Special case for 32-bit to avoid undefined behavior (shift by 32)
  if (num_bits == 32) {
    return static_cast<int32_t>(next_int);
  }
  return (int32_t) next_int - (((int32_t) next_int >> (num_bits - 1)) << num_bits);
}

inline int32_t FLACDecoder::read_rice_sint(uint8_t param) {
  uint32_t unary_count = 0;

  // Inline unary bit reading to avoid function call overhead
  while (true) {
    // Check if we have bits available
    if (this->bit_buffer_length_ == 0) {
      // Need to refill buffer
      if (this->refill_bit_buffer()) {
        this->out_of_data_ = true;
        return 0;
      }
    }

    // Extract next bit from MSB position
    uint32_t bit = (this->bit_buffer_ >> (this->bit_buffer_length_ - 1)) & 1;
    this->bit_buffer_length_--;

    if (bit == 1) {
      // Found stop bit
      break;
    }
    unary_count++;
  }

  // Read parameter bits using existing function
  uint32_t binary = this->read_uint(param);
  uint32_t value = (unary_count << param) | binary;
  return static_cast<int32_t>((value >> 1) ^ -(value & 1));
}

// ============================================================================
// Memory Management
// ============================================================================

void FLACDecoder::free_buffers() {
  if (this->block_samples_) {
    FLAC_FREE(this->block_samples_);
    this->block_samples_ = nullptr;
  }

  // Clear metadata blocks
  this->metadata_blocks_.clear();
  this->partial_header_data_.clear();
}

// ============================================================================
// Configuration Methods
// ============================================================================

void FLACDecoder::set_max_metadata_size(FLACMetadataType type, uint32_t max_size) {
  switch (type) {
    case FLAC_METADATA_TYPE_PADDING:
      this->max_padding_size_ = max_size;
      break;
    case FLAC_METADATA_TYPE_APPLICATION:
      this->max_application_size_ = max_size;
      break;
    case FLAC_METADATA_TYPE_SEEKTABLE:
      this->max_seektable_size_ = max_size;
      break;
    case FLAC_METADATA_TYPE_VORBIS_COMMENT:
      this->max_vorbis_comment_size_ = max_size;
      break;
    case FLAC_METADATA_TYPE_CUESHEET:
      this->max_cuesheet_size_ = max_size;
      break;
    case FLAC_METADATA_TYPE_PICTURE:
      this->max_album_art_size_ = max_size;
      break;
    default:
      this->max_unknown_size_ = max_size;
      break;
  }
}

uint32_t FLACDecoder::get_max_metadata_size(FLACMetadataType type) const {
  switch (type) {
    case FLAC_METADATA_TYPE_PADDING:
      return this->max_padding_size_;
    case FLAC_METADATA_TYPE_APPLICATION:
      return this->max_application_size_;
    case FLAC_METADATA_TYPE_SEEKTABLE:
      return this->max_seektable_size_;
    case FLAC_METADATA_TYPE_VORBIS_COMMENT:
      return this->max_vorbis_comment_size_;
    case FLAC_METADATA_TYPE_CUESHEET:
      return this->max_cuesheet_size_;
    case FLAC_METADATA_TYPE_PICTURE:
      return this->max_album_art_size_;
    default:
      return this->max_unknown_size_;
  }
}

}  // namespace flac
}  // namespace esp_audio_libs