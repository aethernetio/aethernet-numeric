/*
 * Copyright 2026 Aethernet Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AE_NUMERIC_PACKED_RING_H_
#define AE_NUMERIC_PACKED_RING_H_

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <span>
#include <type_traits>

#include "ae-numeric/wire_io.h"

namespace ae {

// Smallest unsigned integer type that can represent every value in [0, N].
// Note the inclusive upper bound: a ring over N bytes needs to count up to N
// used bytes, so N == 256 already requires a 16-bit index.
template <std::size_t N>
using SmallestUIntForSize = std::conditional_t<
    (N <= 0xFFULL), std::uint8_t,
    std::conditional_t<(N <= 0xFFFFULL), std::uint16_t,
                       std::conditional_t<(N <= 0xFFFFFFFFULL), std::uint32_t,
                                          std::uint64_t>>>;

// A byte ring buffer that stores serialized T values back to back, with no
// per-record length headers:
//
//   [serialized T][serialized T][serialized T]
//
// This is only possible because T is self-delimiting on the wire: given the
// start of a record, wire_traits<T>::Deserialize reports how many bytes it
// consumed, which is enough to skip to the next record. Records may straddle
// the physical end of the storage; reads reassemble them through a small
// stack buffer before deserializing.
//
// When a push does not fit, the oldest records are evicted (popped from the
// front) until it does, or the push fails if the value alone exceeds capacity.
template <typename T, std::size_t StorageBytes>
  requires WireSerializable<T>
class PackedRing {
 public:
  using value_type = T;
  using Index = SmallestUIntForSize<StorageBytes>;

  static constexpr std::size_t kCapacity = StorageBytes;
  static constexpr std::size_t kMaxRecordBytes = wire_traits<T>::kMaxWireBytes;
  static constexpr Index kEndPos = static_cast<Index>(kCapacity);

  static_assert(kCapacity > 0, "PackedRing capacity must be positive");

  // Forward/input iterator yielding decoded T values by value. It never exposes
  // a reference into the byte storage, since records do not live there as T.
  class iterator {
   public:
    using value_type = T;
    using reference = T;
    using pointer = void;
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::input_iterator_tag;

    constexpr iterator() = default;
    constexpr iterator(PackedRing const* ring, Index pos, Index remaining)
        : ring_(ring), pos_(pos), remaining_(remaining) {}

    T operator*() const {
      assert(ring_ != nullptr);
      assert(pos_ != kEndPos);
      return ring_->ReadValueAt(pos_);
    }

    iterator& operator++() {
      if (remaining_ <= Index{1}) {
        pos_ = kEndPos;
        remaining_ = 0;
        return *this;
      }
      pos_ = ring_->NextPos(pos_);
      --remaining_;
      return *this;
    }
    iterator operator++(int) {
      iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    constexpr bool operator==(iterator const& other) const {
      return ring_ == other.ring_ && pos_ == other.pos_;
    }
    constexpr bool operator!=(iterator const& other) const {
      return !(*this == other);
    }

   private:
    PackedRing const* ring_ = nullptr;
    Index pos_ = kEndPos;
    Index remaining_ = 0;
  };

  PackedRing() = default;

  constexpr bool empty() const {
    return count_ == 0;
  }
  constexpr Index size() const {
    return count_;
  }
  constexpr Index CapacityBytes() const {
    return static_cast<Index>(kCapacity);
  }
  constexpr Index UsedBytes() const {
    return used_;
  }
  constexpr Index FreeBytes() const {
    return static_cast<Index>(kCapacity - static_cast<std::size_t>(used_));
  }

  constexpr void clear() {
    head_ = 0;
    tail_ = 0;
    used_ = 0;
    count_ = 0;
  }

  T front() const {
    return ReadValueAt(head_);
  }

  bool push(T value) {
    std::array<std::uint8_t, kMaxRecordBytes> temp{};
    std::span<std::uint8_t> temp_bytes(temp);
    std::size_t const n = Serialize<T>(value, temp_bytes.data());
    if (n > kCapacity) {
      return false;
    }
    while (static_cast<std::size_t>(FreeBytes()) < n && count_ > 0) {
      PopFront();
    }
    if (static_cast<std::size_t>(FreeBytes()) < n) {
      return false;
    }
    WriteAt(tail_, std::span<std::uint8_t const>(temp_bytes.data(), n));
    tail_ = Wrap(static_cast<std::size_t>(tail_) + n);
    used_ = static_cast<Index>(static_cast<std::size_t>(used_) + n);
    ++count_;
    return true;
  }

  bool PopFront() {
    if (empty()) {
      return false;
    }
    const std::size_t n = SizeAt(head_);
    head_ = Wrap(static_cast<std::size_t>(head_) + n);
    used_ = static_cast<Index>(static_cast<std::size_t>(used_) - n);
    --count_;
    return true;
  }

  constexpr iterator begin() const {
    return iterator(this, empty() ? kEndPos : head_, count_);
  }
  constexpr iterator end() const {
    return iterator(this, kEndPos, 0);
  }

 private:
  static constexpr Index Wrap(std::size_t value) {
    return static_cast<Index>(value % kCapacity);
  }

  // Number of valid bytes from pos forward, in logical (front-to-back) order.
  constexpr std::size_t AvailFrom(Index pos) const {
    const std::size_t offset = (static_cast<std::size_t>(pos) + kCapacity -
                                static_cast<std::size_t>(head_)) %
                               kCapacity;
    return static_cast<std::size_t>(used_) - offset;
  }

  void CopyFrom(Index pos, std::span<std::uint8_t> out) const {
    std::size_t const n = out.size();
    if (n == 0) {
      return;
    }

    std::size_t const offset = static_cast<std::size_t>(pos);
    std::size_t const first = std::min(n, kCapacity - offset);
    std::memcpy(out.data(), storage_ + offset, first);
    if (first < n) {
      std::memcpy(out.data() + first, storage_, n - first);
    }
  }

  void WriteAt(Index pos, std::span<std::uint8_t const> bytes) {
    std::size_t const n = bytes.size();
    if (n == 0) {
      return;
    }

    std::size_t const offset = static_cast<std::size_t>(pos);
    std::size_t const first = std::min(n, kCapacity - offset);
    std::memcpy(storage_ + offset, bytes.data(), first);
    if (first < n) {
      std::memcpy(storage_, bytes.data() + first, n - first);
    }
  }

  DeserializeResult<T> ReadAt(Index pos) const {
    std::array<std::uint8_t, kMaxRecordBytes> temp{};
    std::size_t const avail = AvailFrom(pos);
    std::size_t const to_copy = std::min(avail, kMaxRecordBytes);
    std::span<std::uint8_t> bytes(temp.data(), to_copy);
    CopyFrom(pos, bytes);
    return Deserialize<T>(bytes.data(), bytes.size());
  }

  T ReadValueAt(Index pos) const {
    return ReadAt(pos).value;
  }
  std::size_t SizeAt(Index pos) const {
    return ReadAt(pos).bytes_read;
  }
  Index NextPos(Index pos) const {
    return Wrap(static_cast<std::size_t>(pos) + SizeAt(pos));
  }

  std::uint8_t storage_[StorageBytes] = {};
  Index head_ = 0;
  Index tail_ = 0;
  Index used_ = 0;
  Index count_ = 0;
};

}  // namespace ae

#endif  // AE_NUMERIC_PACKED_RING_H_
