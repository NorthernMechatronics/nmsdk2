/* Copyright 2024 The TensorFlow Authors. All Rights Reserved.

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

#include "tensorflow/lite/micro/micro_time.h"
#include "micro_time_callback.h"

static uint32_t g_ticks_per_second = 0;
static GetCurrentTimeTicksCallback g_get_current_time_ticks_callback = nullptr;

void SetTicksPerSecond(uint32_t ticks_per_second)
{
  g_ticks_per_second = ticks_per_second;
}

void RegisterGetCurrentTimeTicksCallback(GetCurrentTimeTicksCallback callback)
{
  g_get_current_time_ticks_callback = callback;
}

uint32_t InvokeGetCurrentTimeTicksCallback()
{
  if (g_get_current_time_ticks_callback != nullptr) {
    return g_get_current_time_ticks_callback();
  }

  return 0;
}

namespace tflite {

uint32_t ticks_per_second() {
  return g_ticks_per_second;
}

uint32_t GetCurrentTimeTicks() {
  return InvokeGetCurrentTimeTicksCallback();
}

}  // namespace tflite
