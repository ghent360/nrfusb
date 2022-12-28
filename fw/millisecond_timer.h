// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#pragma once

#include "mbed.h"

namespace fw {

class MillisecondTimer {
public:
  MillisecondTimer() {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    handle3_.Instance = TIM3;
    handle3_.Init.Period = 0xFFFF;
    handle3_.Init.Prescaler =
        (uint32_t)(2 * HAL_RCC_GetPCLK1Freq() / 1000000U) - 1;  // 1 us tick
    handle3_.Init.ClockDivision = 0;
    handle3_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle3_.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&handle3_);
    HAL_TIM_Base_Start(&handle3_);

    handle4_.Instance = TIM4;
    handle4_.Init.Period = 0xFFFF;
    handle4_.Init.Prescaler =
        (uint32_t)(2 * HAL_RCC_GetPCLK1Freq() / 1000U) - 1;  // 1 ms tick
    handle4_.Init.ClockDivision = 0;
    handle4_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle4_.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&handle4_);
    HAL_TIM_Base_Start(&handle4_);
  }

  uint32_t read_ms() {
    return TIM4->CNT;
  }

  void wait_ms(uint32_t delay_ms) {
    uint32_t current = TIM4->CNT;
    uint32_t elapsed = 0;
    while (true) {
      const uint32_t next = TIM4->CNT;
      elapsed += next - current;
      // We check delay_ms + 1 since we don't know where in the
      // current microsecond we started.
      if (elapsed >= (delay_ms + 1)) { return; }
      current = next;
    }
  }

  void wait_us(uint32_t delay_us) {
    uint32_t current = TIM3->CNT;
    uint32_t elapsed = 0;
    while (true) {
      const uint32_t next = TIM3->CNT;
      elapsed += next - current;
      // We check delay_ms + 1 since we don't know where in the
      // current microsecond we started.
      if (elapsed >= (delay_us + 1)) { return; }
      current = next;
    }
  }
private:
  TIM_HandleTypeDef handle3_ = {};
  TIM_HandleTypeDef handle4_ = {};
};

}
