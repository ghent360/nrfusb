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

#include "fw/nrf_manager.h"

#include <optional>

#include "mjlib/base/tokenizer.h"
#include "mjlib/base/visitor.h"

#include "fw/nrf24l01.h"

namespace fw {
namespace micro = mjlib::micro;

namespace {

struct Config {
  bool ptx = true;
  int32_t address_length = 5;
  uint64_t id = 0;
  bool dynamic_payload_length = true;
  bool enable_crc = true;
  int32_t crc_length = 2;
  bool automatic_retransmission = false;
  int32_t auto_retransmit_count = 0;
  bool automatic_acknowledgment = false;
  int32_t initial_channel = 2;
  int32_t data_rate = 1000000;
  int32_t output_power = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(ptx));
    a->Visit(MJ_NVP(address_length));
    a->Visit(MJ_NVP(id));
    a->Visit(MJ_NVP(dynamic_payload_length));
    a->Visit(MJ_NVP(enable_crc));
    a->Visit(MJ_NVP(crc_length));
    a->Visit(MJ_NVP(automatic_retransmission));
    a->Visit(MJ_NVP(auto_retransmit_count));
    a->Visit(MJ_NVP(automatic_acknowledgment));
    a->Visit(MJ_NVP(initial_channel));
    a->Visit(MJ_NVP(data_rate));
    a->Visit(MJ_NVP(output_power));
  }
};

int ParseHexNybble(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return -1;
}

int ParseHexByte(const char* value) {
  int high = ParseHexNybble(value[0]);
  if (high < 0) { return high; }
  int low = ParseHexNybble(value[1]);
  if (low < 0) { return low; }
  return (high << 4) | low;
}
}

class NrfManager::Impl {
 public:
  Impl(mjlib::micro::PersistentConfig& persistent_config,
       mjlib::micro::CommandManager& command_manager,
       mjlib::micro::AsyncExclusive<mjlib::micro::AsyncWriteStream>& stream,
       fw::MillisecondTimer* timer,
       const Options& options)
      : options_(options),
        timer_(timer),
        stream_(stream) {
    persistent_config.Register(
        "nrf", &config_, [this]() { this->UpdateConfig(); });
    command_manager.Register(
        "nrf", [this](auto&& command, auto&& response) {
          this->Command(command, response);
        });
  }

  void Start() {
    Restart();
  }

  void UpdateConfig() {
    Restart();
  }

  void Poll() {
    MJ_ASSERT(!!nrf_);
    nrf_->Poll();

    if (nrf_->is_data_ready()) {
      ReadData();
    }
  }

  void PollMillisecond() {
    MJ_ASSERT(!!nrf_);
    nrf_->PollMillisecond();
  }

 private:
  void Restart() {
    nrf_.emplace(
        timer_,
        [&]() {
          Nrf24l01::Options options;
          options.mosi = options_.mosi;
          options.miso = options_.miso;
          options.sck = options_.sck;
          options.cs = options_.cs;
          options.irq = options_.irq;
          options.ce = options_.ce;

          options.ptx = config_.ptx;
          options.address_length = config_.address_length;
          options.id = config_.id;
          options.dynamic_payload_length = config_.dynamic_payload_length;
          options.enable_crc = config_.enable_crc;
          options.crc_length = config_.crc_length;
          options.automatic_retransmission = config_.automatic_retransmission;
          options.auto_retransmit_count = config_.auto_retransmit_count;
          options.automatic_acknowledgment = config_.automatic_acknowledgment;
          options.initial_channel = config_.initial_channel;
          options.data_rate = config_.data_rate;
          options.output_power = config_.output_power;

          return options;
        }());
  }

  void ReadData() {
    Nrf24l01::Packet packet;
    nrf_->Read(&packet);

    if (write_outstanding_) { return; }

    write_outstanding_ = true;
    size_t pos = 0;
    auto fmt = [&](auto ...args) {
      pos += snprintf(&emit_line_[pos], sizeof(emit_line_) - pos, args...);
    };

    fmt("rcv ");
    for (size_t i = 0; i < packet.size; i++) {
      fmt("%02X", packet.data[i]);
    }
    fmt("\r\n");

    stream_.AsyncStart(
        [this](micro::AsyncWriteStream* write_stream,
               micro::VoidCallback done_callback) {
          done_callback_ = done_callback;
          micro::AsyncWrite(*write_stream, emit_line_, [this](auto ec) {
              auto done = this->done_callback_;
              this->done_callback_ = {};
              this->write_outstanding_ = false;
              done();
            });
        });
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(command, " ");

    auto cmd = tokenizer.next();
    if (cmd == "tx") {
      Command_Tx(tokenizer.remaining(), response);
    } else if (cmd == "stat") {
      Command_Stat(response);
    } else if (cmd == "r") {
      Command_Read(tokenizer.remaining(), response);
    } else {
      WriteMessage("ERR unknown command\r\n", response);
    }
  }

  void WriteOK(const micro::CommandManager::Response& response) {
    WriteMessage("OK\r\n", response);
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

  void Command_Tx(std::string_view hexdata,
                  const micro::CommandManager::Response& response) {
    if ((hexdata.size() % 2) != 0) {
      WriteMessage("ERR data invalid length\r\n", response);
      return;
    }

    Nrf24l01::Packet packet;
    size_t bytes = 0;
    for (size_t i = 0; i < hexdata.size(); i += 2) {
      int value = ParseHexByte(&hexdata[i]);
      if (value < 0) {
        WriteMessage("ERR invalid data\r\n", response);
        return;
      }
      packet.data[bytes++] = value;
    }
    packet.size = bytes;

    nrf_->Transmit(&packet);

    WriteOK(response);
  }

  void Command_Stat(const micro::CommandManager::Response& response) {
    uint8_t value = nrf_->status();
    snprintf(emit_line_, sizeof(emit_line_), "OK %02X\r\n", value);
    WriteMessage(emit_line_, response);
  }

  void Command_Read(std::string_view reg_str,
                    const micro::CommandManager::Response& response) {
    const int reg = std::strtol(reg_str.data(), nullptr, 0);
    const uint8_t result = nrf_->ReadRegister(reg);
    snprintf(emit_line_, sizeof(emit_line_), "OK %02X\r\n", result);
    WriteMessage(emit_line_, response);
  }

  const Options options_;
  fw::MillisecondTimer* const timer_;
  mjlib::micro::AsyncExclusive<mjlib::micro::AsyncWriteStream>& stream_;
  Config config_;
  std::optional<Nrf24l01> nrf_;

  bool write_outstanding_ = false;
  char emit_line_[256] = {};
  micro::VoidCallback done_callback_;
};

NrfManager::NrfManager(
    mjlib::micro::Pool& pool,
    mjlib::micro::PersistentConfig& persistent_config,
    mjlib::micro::CommandManager& command_manager,
    mjlib::micro::AsyncExclusive<
    mjlib::micro::AsyncWriteStream>& stream,
    MillisecondTimer* timer,
    const Options& options)
    : impl_(&pool, persistent_config, command_manager, stream, timer, options) {}

NrfManager::~NrfManager() {}

void NrfManager::Poll() {
  impl_->Poll();
}

void NrfManager::PollMillisecond() {
  impl_->PollMillisecond();
}

void NrfManager::Start() {
  impl_->Start();
}

}
