#include "resampler.h"
#include "../quantization_utils.h"
#include "../memory_utils.h"

namespace esp_audio_libs {
namespace resampler {

Resampler::~Resampler() {
  if (this->resampler_ != nullptr) {
    art_resampler::resampleFree(this->resampler_);
  }

  if (this->float_input_buffer_ != nullptr) {
    internal::free_psram_fallback(this->float_input_buffer_);
  }
  if (this->float_output_buffer_ != nullptr) {
    internal::free_psram_fallback(this->float_output_buffer_);
  }
};

bool Resampler::initialize(ResamplerConfiguration &config) {
  this->input_bits_ = config.source_bits_per_sample;
  this->output_bits_ = config.target_bits_per_sample;
  this->channels_ = config.channels;
  this->number_of_taps_ = config.number_of_taps;
  this->number_of_filters_ = config.number_of_filters;

  this->float_input_buffer_ =
      (float *) internal::alloc_psram_fallback(this->input_buffer_samples_ * sizeof(float));

  this->float_output_buffer_ =
      (float *) internal::alloc_psram_fallback(this->output_buffer_samples_ * sizeof(float));

  if ((this->float_input_buffer_ == nullptr) || (this->float_output_buffer_ == nullptr)) {
    return false;
  }

  if (config.source_sample_rate != config.target_sample_rate) {
    this->requires_resampling_ = true;
    int flags = 0;

    if (config.subsample_interpolate) {
      flags |= SUBSAMPLE_INTERPOLATE;
    }

    this->sample_ratio_ = config.target_sample_rate / config.source_sample_rate;

    if (this->sample_ratio_ < 1.0f) {
      this->lowpass_ratio_ -= (10.24f / this->number_of_taps_);

      if (this->lowpass_ratio_ < 0.84f) {
        this->lowpass_ratio_ = 0.84f;
      }

      if (this->lowpass_ratio_ < this->sample_ratio_) {
        // avoid discontinuities near unity sample ratios
        this->lowpass_ratio_ = this->sample_ratio_;
      }
    }
    if (this->lowpass_ratio_ * this->sample_ratio_ < 0.98f && config.use_pre_or_post_filter) {
      float cutoff = this->lowpass_ratio_ * this->sample_ratio_ / 2.0f;
      art_resampler::biquad_lowpass(&this->lowpass_coeff_, cutoff);
      this->pre_filter_ = true;
    }

    if (this->lowpass_ratio_ / this->sample_ratio_ < 0.98f && config.use_pre_or_post_filter && !this->pre_filter_) {
      float cutoff = this->lowpass_ratio_ / this->sample_ratio_ / 2.0f;
      art_resampler::biquad_lowpass(&this->lowpass_coeff_, cutoff);
      this->post_filter_ = true;
    }

    if (this->pre_filter_ || this->post_filter_) {
      for (int i = 0; i < this->channels_; ++i) {
        art_resampler::biquad_init(&this->lowpass_[i][0], &this->lowpass_coeff_, 1.0f);
        art_resampler::biquad_init(&this->lowpass_[i][1], &this->lowpass_coeff_, 1.0f);
      }
    }

    if (this->sample_ratio_ < 1.0f) {
      this->resampler_ =
          art_resampler::resampleInit(this->channels_, this->number_of_taps_, this->number_of_filters_,
                                      this->sample_ratio_ * this->lowpass_ratio_, flags | INCLUDE_LOWPASS);
    } else if (this->lowpass_ratio_ < 1.0f) {
      this->resampler_ = art_resampler::resampleInit(this->channels_, this->number_of_taps_, this->number_of_filters_,
                                                     this->lowpass_ratio_, flags | INCLUDE_LOWPASS);
    } else {
      this->resampler_ =
          art_resampler::resampleInit(this->channels_, this->number_of_taps_, this->number_of_filters_, 1.0f, flags);
    }

    if (this->resampler_ == nullptr) {
      return false;
    }
    art_resampler::resampleAdvancePosition(this->resampler_, this->number_of_taps_ / 2.0f);
  }

  return true;
}

ResamplerResults Resampler::resample(const uint8_t *input_buffer, uint8_t *output_buffer, size_t input_frames_available,
                                     size_t output_frames_free, float gain_db) {
  size_t frames_to_process = input_frames_available;

  if (this->requires_resampling_) {
    const size_t necessary_frames =
        art_resampler::resampleGetRequiredSamples(this->resampler_, output_frames_free, this->sample_ratio_);
    frames_to_process = std::min(frames_to_process, necessary_frames);
  } else {
    frames_to_process = std::min(frames_to_process, output_frames_free);
  }
  uint32_t conversion_time = 0;
  if (this->requires_resampling_) {
    quantization_utils::quantized_to_float(input_buffer, this->float_input_buffer_, frames_to_process * this->channels_,
                                           this->input_bits_, gain_db);
  } else {
    // Just converting the bits per sample
    quantization_utils::quantized_to_float(input_buffer, this->float_output_buffer_,
                                           frames_to_process * this->channels_, this->input_bits_, gain_db);
  }

  size_t frames_used = frames_to_process;
  size_t frames_generated = frames_to_process;
  size_t predicted_frames_used = frames_to_process;

  if (this->requires_resampling_) {
    if (this->pre_filter_) {
      for (int i = 0; i < this->channels_; ++i) {
        art_resampler::biquad_apply_buffer(&this->lowpass_[i][0], this->float_input_buffer_ + i, frames_to_process,
                                           this->channels_);
        art_resampler::biquad_apply_buffer(&this->lowpass_[i][1], this->float_input_buffer_ + i, frames_to_process,
                                           this->channels_);
      }
    }

    art_resampler::ResampleResult res =
        art_resampler::resampleProcessInterleaved(this->resampler_, this->float_input_buffer_, frames_to_process,
                                                  this->float_output_buffer_, output_frames_free, this->sample_ratio_);

    frames_used = res.input_used;
    frames_generated = res.output_generated;

    if (this->post_filter_) {
      for (int i = 0; i < this->channels_; ++i) {
        art_resampler::biquad_apply_buffer(&this->lowpass_[i][0], this->float_output_buffer_ + i, frames_generated,
                                           this->channels_);
        art_resampler::biquad_apply_buffer(&this->lowpass_[i][1], this->float_output_buffer_ + i, frames_generated,
                                           this->channels_);
      }
    }
  }

  uint32_t clipped_samples = quantization_utils::float_to_quantized(
      this->float_output_buffer_, output_buffer, frames_generated * this->channels_, this->output_bits_);

  ResamplerResults results = {.frames_used = frames_used,
                              .frames_generated = frames_generated,
                              .predicted_frames_used = frames_to_process,
                              .clipped_samples = clipped_samples};
  return results;
}

}  // namespace resampler
}  // namespace esp_audio_libs