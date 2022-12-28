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
#include "fw/nrf_manager.h"
#include "fw/slot_rf_manager.h"
#include "fw/stm32g4_flash.h"
#include "fw/stm32g4_async_usb_cdc.h"
#include "usb.h"

namespace {
namespace base = mjlib::base;
namespace micro = mjlib::micro;

// TODO: Make this dynamically selectable.
#ifdef NRFUSB_RAW
using Manager = fw::NrfManager;
#else
using Manager = fw::SlotRfManager;
#endif
}

int main(void) {
  usb_init_rcc();
  //DigitalOut power_led{PB_15, 1};

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

  char micro_output_buffer[2048] = {};

  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream, micro_output_buffer);

  fw::Stm32G4Flash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface, micro_output_buffer);

  fw::FirmwareInfo firmware_info(pool, telemetry_manager);

  Manager manager(
      pool, persistent_config, command_manager,
      write_stream, &timer,
      [&]() {
        Manager::Options options;

        auto& pins = options.pins;
        pins.mosi = PA_7;
        pins.miso = PA_6;
        pins.sck = PA_5;
        pins.cs = PA_4;
        pins.irq = PB_1;
        pins.ce = PB_0;

        return options;
      }());

  fw::GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();

  command_manager.AsyncStart();
  manager.Start();

  uint32_t old = timer.read_ms();;
  while (true) {
    const uint32_t now = timer.read_ms();

    usb.Poll();
    manager.Poll();

    if (now != old) {
      manager.PollMillisecond();
      old = now;
    }
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
