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

#include "fw/slot_rf_protocol.h"

#include "mjlib/base/visitor.h"
#include "mjlib/micro/static_vector.h"

namespace micro = mjlib::micro;

namespace fw {

namespace {
constexpr int kSlotPeriodMs = 20;
constexpr int kNumSlots = 16;
constexpr int kNumChannels = 23;

}  // namespace

class SlotRfProtocol::Impl {
 public:
  Impl(fw::MillisecondTimer* timer,
       const Options& options)
      : options_(options),
        timer_(timer) {
  }

  void Start() {
    Restart();
  }

  void Poll() {
    MJ_ASSERT(!!nrf_);
    nrf_->Poll();

    if (nrf_->is_data_ready()) {
      nrf_->Read(&rx_packet_);

      // If we are a receiver, we need to mark ourselves as now locked
      // and update slot_timer_ to be ready for the next reception
      // cycle.
      receive_mode_ = kLocked;
      slot_timer_ = kSlotPeriodMs;
    }
    // TODO: Look for data.
  }

  void PollMillisecond() {
    MJ_ASSERT(!!nrf_);
    nrf_->PollMillisecond();

    slot_timer_--;

    if (options_.ptx) {
      if (slot_timer_ == 0) {
        TransmitCycle();
        slot_timer_ = kSlotPeriodMs;
      } else if (slot_timer_ == 2) {
        // Switch to the next channel.
        channel_ = (channel_ + 1) % kNumChannels;
        nrf_->SelectRfChannel(channels_[channel_]);
      }
    } else {
      if (slot_timer_ == 0) {
        slot_timer_ = kSlotPeriodMs;
      }
      // When receiving, we switch to the next channel halfway through
      // our time window.
      if (slot_timer_ == (kSlotPeriodMs / 2) && receive_mode_ == kLocked) {
        // Get ready for the next thing coming our way.
        channel_ = (channel_ + 1) % kNumChannels;
        nrf_->SelectRfChannel(channels_[channel_]);
      }
    }
  }

 private:
  void TransmitCycle() {
    // Increment the ages for all slots.
    for (auto& slot : slots_) {
      slot.age++;
    }

    // Pick a set of slots to send out.

    // For all slots which are enabled for this priority window, fill
    // our transmission buffer with those with the oldest age.
    auto enabled_slots = FindEnabledSlots(priority_count_);
    std::sort(enabled_slots.begin(), enabled_slots.end(),
              [&](auto lhs, auto rhs) {
                return slots_[lhs].age > slots_[rhs].age;
              });

    // Now loop through by age filling up whatever we can.
    for (auto slot_idx : enabled_slots) {
      const int remaining_size = 32 - tx_packet_.size;
      if ((slots_[slot_idx].size + 1) < remaining_size) {
        EmitSlot(slot_idx);
      }
    }

    priority_count_ = (priority_count_ + 1) % 16;

    // Now we send out our frame, whether or not it has anything in it
    // (that gives the receiver a chance to reply).
    nrf_->Transmit(&tx_packet_);
  }

  micro::StaticVector<uint8_t, kNumSlots>
  FindEnabledSlots(uint8_t current_priority) const {
    micro::StaticVector<uint8_t, kNumSlots> result;
    uint32_t mask = 1 << current_priority;
    for (int i = 0; i < kNumSlots; i++) {
      if (slots_[i].priority & mask) { result.push_back(i); }
    }
    return result;
  }

  void EmitSlot(int slot_index) {
    auto& size = tx_packet_.size;

    const int remaining = 32 - size;
    MJ_ASSERT((slots_[slot_index].size + 1) < remaining);

    tx_packet_.data[size] = (slot_index << 4) | slots_[slot_index].size;
    size++;
    std::memcpy(&tx_packet_.data[size], slots_[slot_index].data,
                slots_[slot_index].size);
    size += slots_[slot_index].size;
    slots_[slot_index].age = 0;
  }

  void Restart() {
    nrf_.emplace(
        timer_,
        [&]() {
          Nrf24l01::Options options;
          options.pins = options_.pins;

          options.ptx = options_.ptx;
          options.address_length = 5;
          options.id = SelectShockburstId();
          options.dynamic_payload_length = true;
          options.enable_crc = true;
          options.crc_length = 2;
          options.auto_retransmit_count = options_.auto_retransmit_count;
          options.auto_retransmit_delay_us = 1000;
          options.automatic_acknowledgment = true;
          options.initial_channel = 0;
          options.data_rate = options_.data_rate;
          options.output_power = options_.output_power;

          return options;
        }());

    // Generate our channel switching table.
    GenerateChannelTable();
  }

  uint64_t SelectShockburstId() const {
    const auto slot_id = options_.id;

    const auto byte_lsb = 0xc0 | (slot_id & 0x0f);

    const auto make_byte = [&](int shift) {
      const auto shifted = slot_id >> shift;
      return (shifted & 0xfe) | (((shifted >> 1) & 0x01) ^ 0x01);
    };

    const auto byte1 = make_byte(4);
    const auto byte2 = make_byte(11);
    const auto byte3 = make_byte(18);
    const auto byte4 = make_byte(25);

    return (static_cast<uint64_t>(0)
            | byte_lsb
            | (byte1 << 8)
            | (byte2 << 16)
            | (byte3 << 24)
            | (byte4 << 32));
  }

  void GenerateChannelTable() {
    uint32_t prn = options_.id;
    int channel_count = 0;

    while (channel_count < kNumChannels) {
      prn = (prn * 0x0019660D) + 0x3c6ef35f;

      const uint8_t possible_channel = prn % 125;

      // See if this channel is usable.
      if (!EvaluatePossibleChannel(possible_channel, channel_count)) {
        continue;
      }

      // It is, add it to our list.
      channels_[channel_count] = possible_channel;
      channel_count++;
    }
  }

  bool EvaluatePossibleChannel(uint8_t possible_channel, int channel_count) {
    // If this channel has already been selected, then we discard it.
    for (int i = 0; i < channel_count; i++) {
      if (channels_[i] == possible_channel) { return false; }
    }

    // Evaluate our band limits.
    int band_count[4] = {};
    constexpr int band_channels[4] = { 31, 63, 95, 125 };
    constexpr int band_max[4] = {6, 6, 6, 5};

    const auto get_band = [&](int channel) {
      for (int band = 0; band < 4; band++) {
        if (channel > band_channels[band]) { continue; }
        return band;
      }
      return 0;
    };

    for (int i = 0; i < channel_count; i++) {
      const auto this_channel = channels_[i];
      const int band = get_band(this_channel);
      band_count[band]++;
    }

    const int this_band = get_band(possible_channel);
    if (band_channels[this_band] >= band_max[this_band]) { return false; }

    return true;
  }

  const Options options_;
  fw::MillisecondTimer* const timer_;

  std::optional<Nrf24l01> nrf_;

  bool write_outstanding_ = false;
  char emit_line_[256] = {};
  micro::VoidCallback done_callback_;

  uint8_t channels_[kNumChannels] = {};
  uint8_t channel_ = 0;

  /// For transmitters, this is the canonical source of the system
  /// time.  For receivers, we attempt to synchronize this to
  /// transmitters.
  int slot_timer_ = kSlotPeriodMs;
  int priority_count_ = 0;

  Slot slots_[kNumSlots] = {};

  Nrf24l01::Packet rx_packet_;
  Nrf24l01::Packet tx_packet_;

  enum ReceiveMode {
    kSynchronizing,
    kLocked,
  };

  ReceiveMode receive_mode_ = kSynchronizing;
};

SlotRfProtocol::SlotRfProtocol(MillisecondTimer* timer,
                               const Options& options)
    : impl_(timer, options) {}

SlotRfProtocol::~SlotRfProtocol() {}

void SlotRfProtocol::Poll() {
  impl_->Poll();
}

void SlotRfProtocol::PollMillisecond() {
  impl_->PollMillisecond();
}

void SlotRfProtocol::Start() {
  impl_->Start();
}

}
