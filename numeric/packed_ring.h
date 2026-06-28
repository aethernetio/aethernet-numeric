/*
 * Copyright 2025 Aethernet Inc.
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

#ifndef NUMERIC_PACKED_RING_H_
#define NUMERIC_PACKED_RING_H_

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <type_traits>

#include "numeric/wire_io.h"

namespace ae {

// Smallest unsigned integer type that can represent every value in [0, N].
// Note the inclusive upper bound: a ring over N bytes needs to count up to N
// used bytes, so N == 256 already requires a 16-bit index.
template <std::size_t N>
using SmallestUIntForSize = std::conditional_t<
    (N <= 0xFFULL), std::uint8_t,
    std::conditional_t<
        (N <= 0xFFFFULL), std::uint16_t,
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

  // Forward/input iterator yielding decoded T values by value. It never exposes
  // a reference into the byte storage, since records do not live there as T.
  class iterator {
   public:
    using value_type = T;
    using reference = T;
    using pointer = void;
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::input_iterator_tag;

    iterator() = default;
    iterator(const PackedRing* ring, Index pos, Index ordinal)
        : ring_(ring), pos_(pos), ordinal_(ordinal) {}

    T operator*() const { return ring_->ReadValueAt(pos_); }

    iterator& operator++() {
      pos_ = ring_->NextPos(pos_);
      ++ordinal_;
      return *this;
    }
    iterator operator++(int) {
      iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const iterator& other) const {
      return ordinal_ == other.ordinal_;
    }
    bool operator!=(const iterator& other) const { return !(*this == other); }

   private:
    const PackedRing* ring_ = nullptr;
    Index pos_ = 0;
    Index ordinal_ = 0;
  };

  PackedRing() = default;

  bool empty() const { return count_ == 0; }
  Index size() const { return count_; }
  Index capacity_bytes() const { return static_cast<Index>(kCapacity); }
  Index used_bytes() const { return used_; }
  Index free_bytes() const {
    return static_cast<Index>(kCapacity - static_cast<std::size_t>(used_));
  }

  void clear() {
    head_ = 0;
    tail_ = 0;
    used_ = 0;
    count_ = 0;
  }

  T front() const { return ReadValueAt(head_); }

  bool push(T value) {
    std::uint8_t temp[kMaxRecordBytes];
    const std::size_t n = Serialize<T>(value, temp);
    if (n > kCapacity) {
      return false;
    }
    while (static_cast<std::size_t>(free_bytes()) < n && count_ > 0) {
      pop_front();
    }
    if (static_cast<std::size_t>(free_bytes()) < n) {
      return false;
    }
    for (std::size_t i = 0; i < n; ++i) {
      storage_[Wrap(static_cast<std::size_t>(tail_) + i)] = temp[i];
    }
    tail_ = Wrap(static_cast<std::size_t>(tail_) + n);
    used_ = static_cast<Index>(static_cast<std::size_t>(used_) + n);
    ++count_;
    return true;
  }

  bool pop_front() {
    if (empty()) {
      return false;
    }
    const std::size_t n = SizeAt(head_);
    head_ = Wrap(static_cast<std::size_t>(head_) + n);
    used_ = static_cast<Index>(static_cast<std::size_t>(used_) - n);
    --count_;
    return true;
  }

  iterator begin() const { return iterator(this, head_, 0); }
  iterator end() const { return iterator(this, tail_, count_); }

 private:
  static constexpr Index Wrap(std::size_t value) {
    return static_cast<Index>(value % kCapacity);
  }

  // Number of valid bytes from pos forward, in logical (front-to-back) order.
  std::size_t AvailFrom(Index pos) const {
    const std::size_t offset =
        (static_cast<std::size_t>(pos) + kCapacity -
         static_cast<std::size_t>(head_)) %
        kCapacity;
    return static_cast<std::size_t>(used_) - offset;
  }

  DeserializeResult<T> ReadAt(Index pos) const {
    std::uint8_t temp[kMaxRecordBytes];
    const std::size_t avail = AvailFrom(pos);
    const std::size_t to_copy = avail < kMaxRecordBytes ? avail : kMaxRecordBytes;
    for (std::size_t i = 0; i < to_copy; ++i) {
      temp[i] = storage_[Wrap(static_cast<std::size_t>(pos) + i)];
    }
    return Deserialize<T>(temp, to_copy);
  }

  T ReadValueAt(Index pos) const { return ReadAt(pos).value; }
  std::size_t SizeAt(Index pos) const { return ReadAt(pos).bytes_read; }
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

#endif  // NUMERIC_PACKED_RING_H_
