// Copyright 2018-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _DSP_H_
#define _DSP_H_

#include "dsp_platform.h"
#include <stdint.h>

#ifdef ESP_PLATFORM
#include <esp_err.h>
#else
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t dsps_dotprod_f32_ansi(const float *src1, const float *src2, float *dest, int len);
esp_err_t dsps_dotprod_f32_ae32(const float *src1, const float *src2, float *dest, int len);
esp_err_t dsps_dotprod_f32_aes3(const float *src1, const float *src2, float *dest, int len);

#if (dsps_dotprod_f32_aes3_enabled == 1)
#define dsps_dotprod_f32 dsps_dotprod_f32_aes3
#elif (dotprod_f32_ae32_enabled == 1)
#define dsps_dotprod_f32 dsps_dotprod_f32_ae32
#else
#define dsps_dotprod_f32 dsps_dotprod_f32_ansi
#endif

#ifdef __cplusplus
}
#endif

#endif  // _DSP_H_
