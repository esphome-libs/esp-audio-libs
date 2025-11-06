#include "wav_decoder.h"
#include <cstdint>
#include <cstring>

namespace esp_audio_libs {
namespace wav_decoder {

WAVDecoderResult WAVDecoder::decode_header(const uint8_t *buffer, size_t bytes_available) {
  size_t bytes_to_skip = this->bytes_to_skip();
  size_t bytes_to_read = this->bytes_needed();
  this->bytes_processed_ = 0;

  while ((bytes_to_skip + bytes_to_read) > 0) {
    if ((bytes_to_skip > bytes_available) || (bytes_to_read > bytes_available)) {
      return WAV_DECODER_WARNING_INCOMPLETE_DATA;
    }

    if (bytes_to_skip > 0) {
      // Adjust pointer to skip the appropriate bytes
      buffer += bytes_to_skip;
      this->bytes_processed_ += bytes_to_skip;

      bytes_available -= bytes_to_skip;
      bytes_to_skip = 0;
    } else if (bytes_to_read > 0) {
      WAVDecoderResult result = this->next(buffer);
      buffer += bytes_to_read;
      this->bytes_processed_ += bytes_to_read;

      bytes_available -= bytes_to_read;

      if (result == WAV_DECODER_SUCCESS_IN_DATA) {
        return result;
      } else if (result == WAV_DECODER_SUCCESS_NEXT) {
        // Continue parsing header
        bytes_to_skip = this->bytes_to_skip();
        bytes_to_read = this->bytes_needed();
      } else {
        // Unexpected error parsing the wav header
        return result;
      }
    }
  }

  return WAV_DECODER_ERROR_FAILED;
}

WAVDecoderResult WAVDecoder::next(const uint8_t *buffer) {
  this->bytes_to_skip_ = 0;

  switch (this->state_) {
    case WAV_DECODER_BEFORE_RIFF: {
      this->chunk_name_ = std::string((const char *) buffer, 4);
      if (this->chunk_name_ != "RIFF") {
        return WAV_DECODER_ERROR_NO_RIFF;
      }

      std::memcpy(&this->chunk_bytes_left_, buffer + 4, sizeof(uint32_t));
      if ((this->chunk_bytes_left_ % 2) != 0) {
        // Pad byte
        this->chunk_bytes_left_++;
      }

      // WAVE sub-chunk header should follow
      this->state_ = WAV_DECODER_BEFORE_WAVE;
      this->bytes_needed_ = 4;  // WAVE
      break;
    }

    case WAV_DECODER_BEFORE_WAVE: {
      this->chunk_name_ = std::string((const char *) buffer, 4);
      if (this->chunk_name_ != "WAVE") {
        return WAV_DECODER_ERROR_NO_WAVE;
      }

      // Next chunk header
      this->state_ = WAV_DECODER_BEFORE_FMT;
      this->bytes_needed_ = 8;  // chunk name + size
      break;
    }

    case WAV_DECODER_BEFORE_FMT: {
      this->chunk_name_ = std::string((const char *) buffer, 4);
      std::memcpy(&this->chunk_bytes_left_, buffer + 4, sizeof(uint32_t));
      if ((this->chunk_bytes_left_ % 2) != 0) {
        // Pad byte
        this->chunk_bytes_left_++;
      }

      if (this->chunk_name_ == "fmt ") {
        // Read rest of fmt chunk
        this->state_ = WAV_DECODER_IN_FMT;
        this->bytes_needed_ = this->chunk_bytes_left_;
      } else {
        // Skip over chunk
        this->bytes_to_skip_ = this->chunk_bytes_left_;
        this->bytes_needed_ = 8;
      }
      break;
    }

    case WAV_DECODER_IN_FMT: {
      /**
       * audio format (uint16_t)
       * number of channels (uint16_t)
       * sample rate (uint32_t)
       * bytes per second (uint32_t)
       * block align (uint16_t)
       * bits per sample (uint16_t)
       * [rest of format chunk]
       */
      std::memcpy(&this->num_channels_, buffer + 2, sizeof(uint16_t));
      std::memcpy(&this->sample_rate_, buffer + 4, sizeof(uint32_t));
      std::memcpy(&this->bits_per_sample_, buffer + 14, sizeof(uint16_t));

      // Next chunk
      this->state_ = WAV_DECODER_BEFORE_DATA;
      this->bytes_needed_ = 8;  // chunk name + size
      break;
    }

    case WAV_DECODER_BEFORE_DATA: {
      this->chunk_name_ = std::string((const char *) buffer, 4);
      std::memcpy(&this->chunk_bytes_left_, buffer + 4, sizeof(uint32_t));
      if ((this->chunk_bytes_left_ % 2) != 0) {
        // Pad byte
        this->chunk_bytes_left_++;
      }

      if (this->chunk_name_ == "data") {
        // Complete
        this->state_ = WAV_DECODER_IN_DATA;
        this->bytes_needed_ = 0;
        return WAV_DECODER_SUCCESS_IN_DATA;
      }

      // Skip over chunk
      this->bytes_to_skip_ = this->chunk_bytes_left_;
      this->bytes_needed_ = 8;
      break;
    }

    case WAV_DECODER_IN_DATA: {
      return WAV_DECODER_SUCCESS_IN_DATA;
      break;
    }
  }

  return WAV_DECODER_SUCCESS_NEXT;
}

void WAVDecoder::reset() {
  this->state_ = WAV_DECODER_BEFORE_RIFF;
  this->bytes_to_skip_ = 0;
  this->chunk_name_ = "";
  this->chunk_bytes_left_ = 0;

  this->sample_rate_ = 0;
  this->num_channels_ = 0;
  this->bits_per_sample_ = 0;
}

}  // namespace wav_decoder
}  // namespace esp_audio_libs
