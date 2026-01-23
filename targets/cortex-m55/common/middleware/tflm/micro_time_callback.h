/* Copyright 2020 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#ifndef TENSORFLOW_LITE_MICRO_NM1801XX_MICRO_TIME_CALLBACK_H
#define TENSORFLOW_LITE_MICRO_NM1801XX_MICRO_TIME_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

typedef uint32_t (*GetCurrentTimeTicksCallback)(void);

void SetTicksPerSecond(uint32_t ticks_per_second);
void RegisterGetCurrentTimeTicksCallback(GetCurrentTimeTicksCallback callback);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // TENSORFLOW_LITE_MICRO_CORTEX_M_GENERIC_DEBUG_LOG_CALLBACK_H_
