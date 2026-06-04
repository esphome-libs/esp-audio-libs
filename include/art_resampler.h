////////////////////////////////////////////////////////////////////////////
//                         **** RESAMPLER ****                            //
//                     Sinc-based Audio Resampling                        //
//                Copyright (c) 2006 - 2023 David Bryant.                 //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

namespace esp_audio_libs {
namespace art_resampler {

// Flags for the `flags` parameter of resampleInit(). An enum rather than #defines so the
// names stay scoped to this namespace instead of leaking into the global preprocessor
// namespace, where common identifiers (e.g. BLACKMAN_HARRIS) collide with other libraries.
// Left as an unscoped enum so the values keep implicitly converting to the int `flags` field.
enum ResampleFlags {
  SUBSAMPLE_INTERPOLATE = 0x1,
  BLACKMAN_HARRIS = 0x2,
  INCLUDE_LOWPASS = 0x4,
};

typedef struct {
  int numChannels, numSamples, numFilters, numTaps, inputIndex, flags;
  float *tempFilter, outputOffset;
  float **buffers, **filters;
} Resample;

typedef struct {
  unsigned int input_used, output_generated;
} ResampleResult;

Resample *resampleInit(int numChannels, int numTaps, int numFilters, float lowpassRatio, int flags);
ResampleResult resampleProcess(Resample *cxt, const float *const *input, int numInputFrames, float *const *output,
                               int numOutputFrames, float ratio);
ResampleResult resampleProcessInterleaved(Resample *cxt, const float *input, int numInputFrames, float *output,
                                          int numOutputFrames, float ratio);
unsigned int resampleGetRequiredSamples(Resample *cxt, int numOutputFrames, float ratio);
unsigned int resampleGetExpectedOutput(Resample *cxt, int numInputFrames, float ratio);
void resampleAdvancePosition(Resample *cxt, float delta);
float resampleGetPosition(Resample *cxt);
void resampleReset(Resample *cxt);
void resampleFree(Resample *cxt);

}  // namespace art_resampler
}  // namespace esp_audio_libs
