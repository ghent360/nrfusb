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

#include "fw/nrf24l01.h"

#include "mjlib/base/string_span.h"

namespace fw {

Nrf24l01::SpiMaster::SpiMaster(SPI* spi, PinName cs, MillisecondTimer* timer)
    : spi_(spi), cs_(cs, 1), timer_(timer) {
}

uint8_t Nrf24l01::SpiMaster::Command(
    uint8_t command,
    std::string_view data_in,
    mjlib::base::string_span data_out) {
  cs_.write(0);

  // The nrf24l01 has a 38ns CS setup time.  8 nops should get us
  // that for any stm32 frequency.
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");

  const uint8_t status = spi_->write(command);

  const auto to_transfer =
      std::max<std::size_t>(data_in.size(), data_out.size());
  for (size_t i = 0; i < to_transfer; i++) {
    const uint8_t out_byte = spi_->write(
        (i < data_in.size()) ? data_in[i] : 0);
    if (i < static_cast<std::size_t>(data_out.size())) {
      data_out[i] = out_byte;
    }
  }

  cs_.write(1);

  return status;
}

uint8_t Nrf24l01::SpiMaster::WriteRegister(uint8_t address, std::string_view data) {
  return Command(0x20 + address, data, {});
}

uint8_t Nrf24l01::SpiMaster::WriteRegister(uint8_t address, uint8_t data) {
  return WriteRegister(address, {reinterpret_cast<const char*>(&data), 1});
}

uint8_t Nrf24l01::SpiMaster::ReadRegister(
    uint8_t address, mjlib::base::string_span data) {
  return Command(0x00 + address, {}, data);
}

void Nrf24l01::SpiMaster::VerifyRegister(uint8_t address, std::string_view data) {
  WriteRegister(address, data);
  ReadRegister(address, {buf_, static_cast<ssize_t>(data.size())});
  if (data != std::string_view{buf_, data.size()}) {
    mbed_die();
  }
}

void Nrf24l01::SpiMaster::VerifyRegister(uint8_t address, uint8_t value) {
  VerifyRegister(address, {reinterpret_cast<const char*>(&value), 1});
}

Nrf24l01::Nrf24l01(MillisecondTimer* timer, const Options& options)
    : timer_(timer),
      options_(options),
      spi_(options.mosi, options.miso, options.sck),
      nrf_(&spi_, options.cs, timer),
      irq_(options.irq),
      ce_(options.ce, 0) {}

Nrf24l01::~Nrf24l01() {}

void Nrf24l01::Poll() {
}

void Nrf24l01::PollMillisecond() {
  const auto now = timer_->read_ms();
  // The NRF isn't turned on for 100ms after power up.
  switch (configure_state_) {
    case kPowerOnReset: {
      // This check can be absolute, because the device only has to
      // do power on reset once.
      if (now < 150) { return; }

      WriteConfig();
      configure_state_ = kEnteringStandby;
      start_entering_standby_ = now;
      return;
    }
    case kEnteringStandby: {
      if ((now - start_entering_standby_) < 2) { return; }

      Configure();
      configure_state_ = kStandby;
      return;
    }
    case kStandby: {
      break;
    }
  }
}

bool Nrf24l01::ready() const {
  return configure_state_ == kStandby;
}

void Nrf24l01::SelectRfChannel(uint8_t channel) {
  nrf_.VerifyRegister(0x05, channel & 0x7f);  // RF_CH
}

bool Nrf24l01::is_data_ready() {
  return irq_.read() == 0;
}

bool Nrf24l01::Read(Packet* packet)  {
  if (irq_.read() != 0) { return false; }

  uint8_t payload_width = 0;
  nrf_.Command(0x60,  // R_RX_PL_WID
               {},
               {reinterpret_cast<char*>(&payload_width), 1});

  if (payload_width == 0) { return false; }
  packet->size = payload_width;
  nrf_.Command(0x61, {}, {&packet->data[0],
          static_cast<ssize_t>(packet->size)});

  return true;
}

void Nrf24l01::Transmit(const Packet* packet) {
  nrf_.Command(0xa0, {&packet->data[0], packet->size}, {});
  // Now pulse the CE pin high for at least 10us.
  ce_.write(1);
  const auto start = timer_->read_us();
  while (true) {
    const auto now = timer_->read_us();
    if ((now - start) > 10)  { break; }
  }
  ce_.write(0);
}

void Nrf24l01::QueueAck(const Packet* packet) {
  // We always use PPP == 0
  nrf_.Command(0xa8, {&packet->data[0], packet->size}, {});
}

void Nrf24l01::WriteConfig() {
  nrf_.WriteRegister(0x00, GetConfig());  // CONFIG
  // Now we need to wait another 1.5ms to enter standby mode for this
  // to take effect.
}

void Nrf24l01::Configure() {
  nrf_.VerifyRegister(0x00, GetConfig());

  nrf_.VerifyRegister(0x01, 0x3f);  // EN_AA enable 0-5
  nrf_.VerifyRegister(0x02, 0x01);  // EN_RXADDR enable 0
  nrf_.VerifyRegister(
      0x03,  // SETUP_AW
      [&]() {
        if (options_.address_length == 3) { return 1; }
        if (options_.address_length == 4) { return 2; }
        if (options_.address_length == 5) { return 3; }
        mbed_die();
      }());
  MJ_ASSERT(options_.automatic_retransmission == false);
  // Thus we don't need to set up SETUP_RETR
  SelectRfChannel(options_.initial_channel);

  nrf_.VerifyRegister(
      0x06,  // RF_SETUP
      [&]() {
        if (options_.data_rate == 250000) {
          return (1 << 5);
        } else if (options_.data_rate == 1000000) {
          return (0 << 5) | (0 << 3);
        } else if (options_.data_rate == 2000000) {
          return (0 << 5) | (1 << 3);
        }
        mbed_die();
      }() |
      [&]() {
        if (options_.output_power == -18) {
          return 0;
        } else if (options_.output_power == -12) {
          return 2;
        } else if (options_.output_power == -6) {
          return 4;
        } else if (options_.output_power == 0) {
          return 6;
        } else if (options_.output_power == 7) {
          return 1;
        }
        mbed_die();
      }());

  std::string_view id_view{
    reinterpret_cast<const char*>(&options_.id),
        static_cast<size_t>(options_.address_length)};
  nrf_.VerifyRegister(0x0a,  id_view); // RX_ADDR_P0
  nrf_.VerifyRegister(0x10,  id_view); // TX_ADDR
  nrf_.VerifyRegister(0x1c, options_.dynamic_payload_length ? 1 : 0);  // DYNPD
  nrf_.VerifyRegister(
      0x1d,  0 // FEATURE
      | ((options_.dynamic_payload_length ? 1 : 0) << 2)
      | ((options_.automatic_acknowledgment ? 1 : 0) << 1)
                      );
}

uint8_t Nrf24l01::GetConfig() const {
  return 0
      | (0 << 6) // MASK_RX_DR - enable RX_DR interrupt
      | (1 << 5) // MASK_TX_DS - disable TX_DS interrupt
      | (1 << 4) // MASK_MAX_RT - disable MAX_RT interrupt
      | ((options_.enable_crc ? 1 : 0) << 3) // EN_CRC
      | (((options_.crc_length == 2) ? 1 : 0) << 2) // CRCO (0=1 byte, 1=2 bytes)
      | (1 << 1) // PWR_UP
      | ((options_.ptx ? 0 : 1) << 0) // PRIM_RX
      ;
}

}  // namespace fw
