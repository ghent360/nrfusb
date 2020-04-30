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
  std::array<uint32_t, SlotRfProtocol::kNumRemotes> ids = {
    0x30251023,
    0,
  };
  int32_t data_rate = 1000000;
  int32_t output_power = 0;
  int32_t auto_retransmit_count = 0;
  bool print_channels = false;
  int32_t transmit_timeout_ms = 1000;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(ptx));
    a->Visit(MJ_NVP(ids));
    a->Visit(MJ_NVP(data_rate));
    a->Visit(MJ_NVP(output_power));
    a->Visit(MJ_NVP(auto_retransmit_count));
    a->Visit(MJ_NVP(print_channels));
    a->Visit(MJ_NVP(transmit_timeout_ms));
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
    for (auto& priority_remote : priorities_) {
      for (auto& priority : priority_remote.priorities) {
        priority = 0xffffffff;
      }
    }

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

    for (size_t remote_index = 0;
         remote_index < SlotRfProtocol::kNumRemotes;
         remote_index++) {
      auto* remote = slot_->remote(remote_index);
      auto& last_bitfield = last_bitfields_[remote_index];
      const auto current = remote->slot_bitfield();
      if (current != last_bitfield) {
        EmitSlots(remote, remote_index, current ^ last_bitfield);
      }
      last_bitfield = current;
    }

    const auto channel = slot_->channel();
    if (config_.print_channels && channel != last_channel_) {
      EmitChannel(channel);
    }
    last_channel_ = channel;
  }

  void PollMillisecond() {
    timeout_remaining_ = std::max<int32_t>(0, timeout_remaining_ - 1);
    if (timeout_remaining_ == 0 && config_.transmit_timeout_ms) {
      DisableTransmit();
    }
    slot_->PollMillisecond();
  }

 private:
  void EmitChannel(uint8_t channel) {
    if (write_outstanding_) { return; }

    snprintf(emit_line_, sizeof(emit_line_), "chan %d\r\n", channel);

    EmitLine();
  }

  void EmitSlots(SlotRfProtocol::Remote* remote, int remote_index, uint32_t slots) {
    if (write_outstanding_) { return; }

    ssize_t pos = 0;
    auto fmt = [&](auto ...args) {
      pos += snprintf(&emit_line_[pos], sizeof(emit_line_) - pos, args...);
    };

    fmt("rcv");
    if (remote_index > 0) {
      fmt("2 %d", remote_index);
    }

    for (int slot_index = 0;
         slot_index < SlotRfProtocol::kNumSlots;
         slot_index++) {
      const uint32_t mask = 0x3 << (slot_index * 2);
      if ((slots & mask) == 0) { continue; }

      fmt(" %d:", slot_index);

      const auto& slot = remote->rx_slot(slot_index);
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
          options.ids = config_.ids;
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
      Command_Tx(0, tokenizer.remaining(), response);
    } else if (cmd == "tx2") {
      Command_Tx2(tokenizer.remaining(), response);
    } else if (cmd == "pri") {
      Command_Pri(0, tokenizer.remaining(), response);
    } else if (cmd == "pri2") {
      Command_Pri2(tokenizer.remaining(), response);
    } else {
      WriteMessage("ERR unknown command\r\n", response);
    }
  }

  void Command_Tx2(std::string_view command,
                   const micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(command, " ");
    auto remote_index_str = tokenizer.next();

    const int remote_index =
        std::max<int>(
            0, std::min<int>(
                SlotRfProtocol::kNumRemotes - 1,
                std::strtol(remote_index_str.data(), nullptr, 0)));

    Command_Tx(remote_index, tokenizer.remaining(), response);
  }

  void Command_Tx(int remote_index,
                  std::string_view remaining,
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
                SlotRfProtocol::kNumSlots - 1,
                std::strtol(slot_str.data(), nullptr, 0)));

    SlotRfProtocol::Slot slot;
    slot.size = hexdata.size() / 2;
    slot.priority = priorities_[remote_index].priorities[slot_index];

    for (size_t i = 0; i < hexdata.size(); i += 2) {
      const int value = ParseHexByte(&hexdata[i]);
      if (value < 0) {
        WriteMessage("ERR invalid data\r\n", response);
        return;
      }
      slot.data[i / 2] = value;
    }

    slot_->remote(remote_index)->tx_slot(slot_index, slot);

    timeout_remaining_ = config_.transmit_timeout_ms;

    WriteOK(response);
  }

  void Command_Pri2(std::string_view command,
                    const micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(command, " ");
    auto remote_index_str = tokenizer.next();

    const int remote_index =
        std::max<int>(
            0, std::min<int>(
                SlotRfProtocol::kNumRemotes - 1,
                std::strtol(remote_index_str.data(), nullptr, 0)));

    Command_Pri(remote_index, tokenizer.remaining(), response);
  }

  void Command_Pri(int remote_index,
                   std::string_view remaining,
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
                SlotRfProtocol::kNumSlots - 1,
                std::strtol(slot_str.data(), nullptr, 0)));
    const uint32_t priority =
        std::strtoul(pri_str.data(), nullptr, 16);

    priorities_[remote_index].priorities[slot_index] = priority;

    auto* const remote = slot_->remote(remote_index);
    auto slot = remote->tx_slot(slot_index);
    slot.priority = priority;
    remote->tx_slot(slot_index, slot);

    WriteOK(response);
  }

  void DisableTransmit() {
    // Set all the priorities at the lower level to 0, so we stop
    // sending slots.
    for (int remote_index = 0;
         remote_index < SlotRfProtocol::kNumRemotes;
         remote_index++) {
      auto* const remote = slot_->remote(remote_index);
      for (int slot_index = 0;
           slot_index < SlotRfProtocol::kNumSlots;
           slot_index++) {
        auto slot = remote->tx_slot(slot_index);
        slot.priority = 0;
        remote->tx_slot(slot_index, slot);
      }
    }
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
  std::array<uint32_t, SlotRfProtocol::kNumRemotes> last_bitfields_ = {};
  uint8_t last_channel_ = 0;

  struct Priorities {
    uint32_t priorities[16] = {};
  };

  std::array<Priorities, SlotRfProtocol::kNumRemotes> priorities_;

  bool write_outstanding_ = false;
  char emit_line_[256] = {};
  micro::VoidCallback done_callback_;

  int32_t timeout_remaining_ = 0;
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
