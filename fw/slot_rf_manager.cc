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

#include "fw/slot_rf_manager.h"

#include <optional>

#include "fw/slot_rf_protocol.h"

namespace micro = mjlib::micro;

namespace fw {

namespace {
struct Config {
  bool ptx = true;
  uint32_t id = 0x30251023;
  int32_t data_rate = 1000000;
  int32_t output_power = 0;
  int32_t auto_retransmit_count = 0;
  bool print_channels = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(ptx));
    a->Visit(MJ_NVP(id));
    a->Visit(MJ_NVP(data_rate));
    a->Visit(MJ_NVP(output_power));
    a->Visit(MJ_NVP(auto_retransmit_count));
    a->Visit(MJ_NVP(print_channels));
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
}  // namespace

class SlotRfManager::Impl {
 public:
  Impl(mjlib::micro::PersistentConfig& persistent_config,
       mjlib::micro::CommandManager& command_manager,
       mjlib::micro::AsyncExclusive<mjlib::micro::AsyncWriteStream>& stream,
       fw::MillisecondTimer* timer,
       const Options& options)
      : options_(options),
        timer_(timer),
        stream_(stream) {
    // Default all slots to sending all the time.
    for (auto& priority : priorities_) { priority = 0xffffffff; }

    persistent_config.Register(
        "slot", &config_, [this]() { this->UpdateConfig(); });
    command_manager.Register(
        "slot", [this](auto&& command, auto&& response) {
          this->Command(command, response);
        });
  }

  void Start() {
    Restart();
  }

  void Poll() {
    slot_->Poll();

    const auto current = slot_->slot_bitfield();
    if (current != last_bitfield_) {
      EmitSlots(current ^ last_bitfield_);
    }
    last_bitfield_ = current;

    const auto channel = slot_->channel();
    if (config_.print_channels && channel != last_channel_) {
      EmitChannel(channel);
    }
    last_channel_ = channel;
  }

  void PollMillisecond() {
    slot_->PollMillisecond();
  }

 private:
  void EmitChannel(uint8_t channel) {
    if (write_outstanding_) { return; }

    snprintf(emit_line_, sizeof(emit_line_), "chan %d\r\n", channel);

    EmitLine();
  }

  void EmitSlots(uint32_t slots) {
    if (write_outstanding_) { return; }

    ssize_t pos = 0;
    auto fmt = [&](auto ...args) {
      pos += snprintf(&emit_line_[pos], sizeof(emit_line_) - pos, args...);
    };

    fmt("rcv");
    for (int slot_index = 0; slot_index < 16; slot_index++) {
      const uint32_t mask = 0x3 << (slot_index * 2);
      if ((slots & mask) == 0) { continue; }

      fmt(" %d:", slot_index);

      const auto& slot = slot_->rx_slot(slot_index);
      for (int i = 0; i < slot.size; i++) {
        fmt("%02X", slot.data[i]);
      }
    }

    if (slot_->error()) {
      fmt(" E%X", slot_->error());
    }
    fmt("\r\n");

    EmitLine();
  }

  void EmitLine() {
    write_outstanding_ = true;
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

  void UpdateConfig() {
    Restart();
  }

  void Restart() {
    slot_.emplace(
        timer_,
        [&]() {
          SlotRfProtocol::Options options;
          options.pins = options_.pins;

          options.ptx = config_.ptx;
          options.id = config_.id;
          options.data_rate = config_.data_rate;
          options.output_power = config_.output_power;
          options.auto_retransmit_count = config_.auto_retransmit_count;

          return options;
        }());
    slot_->Start();
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(command, " ");

    auto cmd = tokenizer.next();
    if (cmd == "tx") {
      Command_Tx(tokenizer.remaining(), response);
    } else if (cmd == "pri") {
      Command_Pri(tokenizer.remaining(), response);
    } else {
      WriteMessage("ERR unknown command\r\n", response);
    }
  }

  void Command_Tx(std::string_view remaining,
                  const micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(remaining, " ");

    auto slot_str = tokenizer.next();
    auto hexdata = tokenizer.next();

    if ((hexdata.size() % 2) != 0) {
      WriteMessage("ERR data invalid length\r\n", response);
      return;
    }

    const int slot_index =
        std::max<int>(
            0, std::min<int>(
                SlotRfProtocol::kNumSlots,
                std::strtol(slot_str.data(), nullptr, 0)));

    SlotRfProtocol::Slot slot;
    slot.size = hexdata.size() / 2;
    slot.priority = priorities_[slot_index];

    for (size_t i = 0; i < hexdata.size(); i += 2) {
      const int value = ParseHexByte(&hexdata[i]);
      if (value < 0) {
        WriteMessage("ERR invalid data\r\n", response);
        return;
      }
      slot.data[i / 2] = value;
    }

    slot_->tx_slot(slot_index, slot);

    WriteOK(response);
  }

  void Command_Pri(std::string_view remaining,
                   const micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(remaining, " ");

    auto slot_str = tokenizer.next();
    auto pri_str = tokenizer.next();

    if (slot_str.empty() || pri_str.empty()) {
      WriteMessage("ERR invalid priority\r\n", response);
      return;
    }

    const int slot_index =
        std::max<int>(
            0, std::min<int>(
                SlotRfProtocol::kNumSlots,
                std::strtol(slot_str.data(), nullptr, 0)));
    const uint32_t priority =
        std::strtoul(pri_str.data(), nullptr, 16);

    priorities_[slot_index] = priority;

    auto slot = slot_->tx_slot(slot_index);
    slot.priority = priority;
    slot_->tx_slot(slot_index, slot);

    WriteOK(response);
  }

  void WriteOK(const micro::CommandManager::Response& response) {
    WriteMessage("OK\r\n", response);
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

  const Options options_;
  MillisecondTimer* const timer_;
  micro::AsyncExclusive<micro::AsyncWriteStream>& stream_;

  Config config_;

  std::optional<SlotRfProtocol> slot_;
  uint32_t last_bitfield_ = 0;
  uint8_t last_channel_ = 0;

  uint32_t priorities_[16] = {};

  bool write_outstanding_ = false;
  char emit_line_[256] = {};
  micro::VoidCallback done_callback_;
};

SlotRfManager::SlotRfManager(
    micro::Pool& pool,
    micro::PersistentConfig& persistent_config,
    micro::CommandManager& command_manager,
    micro::AsyncExclusive<micro::AsyncWriteStream>& stream,
    MillisecondTimer* timer,
    const Options& options)
    : impl_(&pool, persistent_config, command_manager, stream, timer, options) {}

SlotRfManager::~SlotRfManager() {}

void SlotRfManager::Poll() {
  impl_->Poll();
}

void SlotRfManager::PollMillisecond() {
  impl_->PollMillisecond();
}

void SlotRfManager::Start() {
  impl_->Start();
}

}  // namespace fw
