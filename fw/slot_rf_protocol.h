// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/static_ptr.h"

#include "fw/millisecond_timer.h"
#include "fw/nrf24l01.h"

namespace fw {

class SlotRfProtocol {
 public:
  struct Options {
    bool ptx = true;
    uint64_t id = 0x3045;
    int32_t data_rate = 1000000;
    int32_t output_power = 0;
    int32_t auto_retransmit_count = 0;

    Nrf24l01::Pins pins;
  };

  SlotRfProtocol(MillisecondTimer*,
                 const Options& options);
  ~SlotRfProtocol();

  void Poll();
  void PollMillisecond();
  void Start();

  struct Slot {
    uint32_t priority = 0;
    uint8_t size = 0;
    uint32_t age = 0;
    uint8_t data[16] = {};
  };

  /// Only valid in writing mode.
  void Set(int slot_idx, const Slot&);

 private:
  class Impl;
  mjlib::micro::StaticPtr<Impl, 2048> impl_;
};

}
