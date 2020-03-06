// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "fw/firmware_info.h"

namespace fw {

namespace {
struct Info {
  std::array<uint32_t, 3> serial_number = {};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(serial_number));
  }
};
}

class FirmwareInfo::Impl {
 public:
  Impl(mjlib::micro::TelemetryManager& telemetry) {
    const int32_t* const device_signature =
        reinterpret_cast<const int32_t*>(
#if defined(TARGET_STM32F4)
            0x1fff7a10
#elif defined(TARGET_STM32G4)
            0x1fff7590
#else
#error "Unknown target"
#endif
                                         );
    std::memcpy(&info_.serial_number[0], device_signature,
                sizeof(uint32_t) * 3);

    telemetry.Register("firmware", &info_);
  }

  Info info_;
};

FirmwareInfo::FirmwareInfo(mjlib::micro::Pool& pool,
                           mjlib::micro::TelemetryManager& telemetry)
    : impl_(&pool, telemetry) {}

FirmwareInfo::~FirmwareInfo() {}

FirmwareInfo::SerialNumber FirmwareInfo::serial_number() const {
  SerialNumber result;
  for (int i = 0; i < 3; i++) {
    result.number[i] = impl_->info_.serial_number[i];
  }
  return result;
}

}
