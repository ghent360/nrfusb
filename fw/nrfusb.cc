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

#include "mbed.h"

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_flash.h"
#include "fw/stm32g4_async_usb_cdc.h"

namespace {
namespace base = mjlib::base;
namespace micro = mjlib::micro;
}

int main(void) {
  DigitalOut power_led{PB_15, 1};

  fw::MillisecondTimer timer;

  micro::SizedPool<12288> pool;

  fw::Stm32G4AsyncUsbCdc usb(&pool, {});

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&usb);
  micro::CommandManager command_manager(
      &pool, &usb, &write_stream,
      []() {
        micro::CommandManager::Options options;
        options.max_line_length = 300;
        return options;
      }());

  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);

  fw::Stm32G4Flash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface);

  fw::FirmwareInfo firmware_info(pool, telemetry_manager);

  fw::GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();

  command_manager.AsyncStart();

  while (true) {
    const uint32_t start = timer.read_ms();

    while (true) {
      const uint32_t now = timer.read_ms();
      if (now - start > 10) { break; }

      usb.Poll();
    }

    usb.Poll10Ms();
  }
}

extern "C" {
void SysTick_Handler(void) {
  HAL_IncTick();
}

void abort() {
  mbed_die();
}
}
