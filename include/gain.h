#pragma once

#include <stddef.h>
#include <stdint.h>

#include <atomic>

namespace esp_audio_libs {
namespace gain {

/// @brief Converts a gain in dB to a Q31 scale factor in [0, INT32_MAX].
///
/// Q31 cannot exceed unity: 0 dB and above clamp to INT32_MAX. Very negative dB and NaN return 0.
int32_t db_to_q31(float db);

/// @brief Converts an integer dB reduction to a Q31 scale factor using integer math only.
///
/// 0 is unity; 173 and above are silence. Results are exact points on GainRamp's 1 dB grid.
int32_t db_reduction_to_q31(uint8_t db);

/// @brief Scales each sample by a Q31 factor in [0, INT32_MAX]. Cannot amplify.
///
/// Samples are signed PCM, little-endian, interleaved; 8-bit is int8, not WAV-style uint8.
/// May operate in place. Buffers aligned to bytes_per_sample take a faster path with identical
/// output. Callers in hot paths should skip the call when q31_scale is unity.
/// @param audio_samples Input buffer.
/// @param output_buffer Output buffer (may alias the input).
/// @param q31_scale Scale factor; INT32_MAX is unity.
/// @param samples_to_scale Number of samples (not frames).
/// @param bytes_per_sample 1, 2, 3, or 4. Other values are a no-op.
void apply(const uint8_t *audio_samples, uint8_t *output_buffer, int32_t q31_scale, size_t samples_to_scale,
           size_t bytes_per_sample);

/// @brief Ramps a Q31 gain toward a target over a fixed length or at a fixed rate.
///
/// The ramp itself is integer math only; the float dB entry points convert once per call. Set the
/// target when the level changes, not every block: re-asserting it restarts a ramp in flight.
///
/// Thread safety: one thread may call the set_target_*() methods while another calls process(). The
/// setters only post a request; process() picks it up at the start of its next call, so a change
/// lands on a block boundary. Two threads must not call setters concurrently, and no method should
/// be called from an interrupt. The getters are safe from any thread.
class GainRamp {
 public:
  /// @brief Ramps from the live value to target_q31 (in [0, INT32_MAX]) in at most ramp_samples.
  ///
  /// For ducking: the duration is fixed and the rate follows from the distance. Settled at the target
  /// already is a no-op; a ramp already in flight is rescheduled. A ramp too short to give each 1 dB
  /// step one sample jumps instead. ramp_samples counts interleaved samples, as process() does.
  void set_target_over(int32_t target_q31, uint32_t ramp_samples);

  /// @brief set_target_over() with the target as an integer dB reduction; see db_reduction_to_q31().
  void set_target_db_reduction_over(uint8_t db, uint32_t ramp_samples);

  /// @brief set_target_over() with the target as a signed dB gain (0 is unity); see db_to_q31().
  void set_target_db_over(float db, uint32_t ramp_samples);

  /// @brief Ramps from the live value to target_q31 (in [0, INT32_MAX]) at samples_per_db per 1 dB step.
  ///
  /// For volume changes: the rate is fixed, so every change moves at the same speed regardless of
  /// distance. Settled at the target already is a no-op; a ramp already in flight is rescheduled.
  /// 0 jumps instead. The linear segment to or from silence is also one step long.
  void set_target_at_rate(int32_t target_q31, uint32_t samples_per_db);

  /// @brief set_target_at_rate() with the target as an integer dB reduction; see db_reduction_to_q31().
  void set_target_db_reduction_at_rate(uint8_t db, uint32_t samples_per_db);

  /// @brief set_target_at_rate() with the target as a signed dB gain (0 is unity); see db_to_q31().
  void set_target_db_at_rate(float db, uint32_t samples_per_db);

  /// @brief Applies any pending request, then scales a block in place, advancing the ramp. Settled
  /// at unity is a no-op.
  /// @param buffer Interleaved samples, same format as apply().
  /// @param bytes_per_sample 1, 2, 3, or 4.
  /// @param samples Number of samples (not frames).
  void process(uint8_t *buffer, uint8_t bytes_per_sample, uint32_t samples);

  /// @brief Live Q31 gain as of the last process() call. INT32_MAX is unity.
  int32_t current_q31() const { return this->current_published_.load(std::memory_order_relaxed); }
  /// @brief Q31 gain most recently requested.
  int32_t target_q31() const { return this->request_target_.load(std::memory_order_relaxed); }
  /// @brief True while the live gain differs from the requested target.
  bool is_ramping() const { return this->current_q31() != this->target_q31(); }

 private:
  /// @brief Posts a request for process() to apply; at_rate selects the meaning of param.
  void post_request_(int32_t target_q31, uint32_t param, bool at_rate);
  /// @brief Applies a newly posted request, if any. Called only by process().
  void take_request_();
  /// @brief Schedules `steps` segments of samples_per_step each toward target_q31; 0 jumps.
  void schedule_(int32_t target_q31, uint32_t samples_per_step, uint32_t steps);

  // Ramp state, owned by process().
  int32_t current_q31_{INT32_MAX};
  int32_t target_q31_{INT32_MAX};
  int32_t seg_target_q31_{INT32_MAX};  ///< End of the 1 dB segment in progress.
  uint32_t samples_remaining_{0};      ///< 0 means settled.
  uint32_t samples_per_step_{0};       ///< Samples per 1 dB segment.
  uint32_t applied_seq_{0};            ///< request_seq_ of the last request applied.

  std::atomic<int32_t> current_published_{INT32_MAX};  ///< current_q31_ for the getters.

  // Request mailbox: a single-writer seqlock. request_seq_ is odd while a write is in progress.
  std::atomic<uint32_t> request_seq_{0};
  std::atomic<int32_t> request_target_{INT32_MAX};
  std::atomic<uint32_t> request_param_{0};
  std::atomic<bool> request_at_rate_{false};
};

}  // namespace gain
}  // namespace esp_audio_libs
