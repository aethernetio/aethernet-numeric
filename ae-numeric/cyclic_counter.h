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

#ifndef AE_NUMERIC_CYCLIC_COUNTER_H_
#define AE_NUMERIC_CYCLIC_COUNTER_H_

// Purpose: Store a full-width counter locally while sending only its
// low WireType bits over the wire.
//
// Motivation: Protocol sequence numbers need a long local lifetime,
// but relevant samples differ by far less than 2^N. Truncating the wire
// representation saves bandwidth without storing a separate epoch.
//
// Analogous concepts: serial-number arithmetic, modular counters, and
// truncated sequence numbers. This is not a total order: exactly half
// of the wire space is inherently ambiguous.
//
// Usage: send WireValue(); receive with TryRestore() for a non-mutating
// reconstruction or TryAdvance() to move the stored base only forward.
//
// How it works: the object stores only ValueType value_. It restores
// the nearest full value whose low bits match the sample. No epoch,
// previous sample, or decoder state is stored. Stateless deserialization
// is intentionally unavailable because the wire value lacks high bits.


#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

namespace ae {

// Outcome of restoring a truncated wire sample relative to a full counter.
enum class CyclicRestoreStatus : std::uint8_t {
  Ok = 0,
  Ambiguous = 1,
  OutOfRange = 2,
};

// Outcome of reading bytes then restoring/advancing relative to a live counter.
enum class CyclicDecodeStatus : std::uint8_t {
  Ok = 0,
  TruncatedInput = 1,
  Ambiguous = 2,
  OutOfRange = 3,
};

template <typename ValueType>
struct CyclicDecodeResult {
  CyclicDecodeStatus status = CyclicDecodeStatus::TruncatedInput;
  ValueType value{};
  std::size_t bytes_read = 0;

  constexpr bool ok() const noexcept {
    return status == CyclicDecodeStatus::Ok;
  }
  constexpr explicit operator bool() const noexcept { return ok(); }
};

// Compact cyclic counter: full ValueType locally, truncated WireType on the
// wire. Restoration is relative to the current full value (nearest unambiguous
// neighbor). Half wire-range is ambiguous and rejected by TryRestore.
//
// There is no wire_traits specialization: a truncated wire value cannot form a
// full CyclicCounter without a live base. Serialize WireValue() via
// wire_traits<WireType>; restore with TryRestore / TryAdvance /
// TryDeserializeAndRestore / TryDeserializeAndAdvance.
//
// No epoch / previous / decoder state is stored — sizeof equals
// sizeof(ValueType).
template <typename WireType, typename ValueType>
  requires(std::is_integral_v<WireType> && std::is_unsigned_v<WireType> &&
           !std::is_same_v<WireType, bool> && std::is_integral_v<ValueType> &&
           std::is_unsigned_v<ValueType> && !std::is_same_v<ValueType, bool> &&
           (sizeof(ValueType) > sizeof(WireType)))
class CyclicCounter {
 public:
  using wire_type = WireType;
  using value_type = ValueType;
  using decode_result = CyclicDecodeResult<value_type>;

  static constexpr value_type kWireMask =
      static_cast<value_type>(std::numeric_limits<wire_type>::max());
  // Wire modular space: 2^(8*sizeof(WireType)). Fits in ValueType because
  // sizeof(ValueType) > sizeof(WireType).
  static constexpr value_type kWireSpace = kWireMask + value_type{1};
  static constexpr value_type kHalfRange = kWireSpace / value_type{2};

  static_assert(kWireSpace > kWireMask, "ValueType must hold wire space");
  static_assert(kHalfRange * value_type {2} == kWireSpace,
                "wire space must be even");

  constexpr CyclicCounter() noexcept = default;

  constexpr explicit CyclicCounter(value_type value) noexcept : value_(value) {}

  constexpr value_type Value() const noexcept { return value_; }

  constexpr explicit operator value_type() const noexcept { return value_; }

  constexpr wire_type WireValue() const noexcept {
    return static_cast<wire_type>(value_ & kWireMask);
  }

  // Nearest full value whose low bits equal `wire`, relative to value_.
  // Distinguishes half-range ambiguity from ValueType overflow/underflow.
  constexpr CyclicRestoreStatus TryRestoreStatus(
      wire_type wire, value_type& out) const noexcept {
    wire_type const current = WireValue();
    wire_type const forward = static_cast<wire_type>(wire - current);

    if (forward == wire_type{0}) {
      out = value_;
      return CyclicRestoreStatus::Ok;
    }

    value_type const forward_v = static_cast<value_type>(forward);
    if (forward_v == kHalfRange) {
      return CyclicRestoreStatus::Ambiguous;
    }

    if (forward_v < kHalfRange) {
      if (value_ > (std::numeric_limits<value_type>::max() - forward_v)) {
        return CyclicRestoreStatus::OutOfRange;
      }
      out = value_ + forward_v;
      return CyclicRestoreStatus::Ok;
    }

    value_type const backward = kWireSpace - forward_v;
    if (value_ < backward) {
      return CyclicRestoreStatus::OutOfRange;
    }
    out = value_ - backward;
    return CyclicRestoreStatus::Ok;
  }

  // Returns nullopt on Ambiguous or OutOfRange.
  constexpr std::optional<value_type> TryRestore(
      wire_type wire) const noexcept {
    value_type out{};
    if (TryRestoreStatus(wire, out) != CyclicRestoreStatus::Ok) {
      return std::nullopt;
    }
    return out;
  }

  // Contract: TryRestore must succeed. Debug builds assert on failure.
  constexpr value_type Restore(wire_type wire) const noexcept {
    auto const restored = TryRestore(wire);
    assert(restored.has_value());
    return *restored;
  }

  constexpr void Set(value_type value) noexcept { value_ = value; }

  // Restore relative to value_. On success: if restored > value_, update
  // value_; otherwise leave value_ unchanged. Always returns restored when
  // unambiguous (including older values).
  constexpr std::optional<value_type> TryAdvance(wire_type wire) noexcept {
    value_type restored{};
    if (TryRestoreStatus(wire, restored) != CyclicRestoreStatus::Ok) {
      return std::nullopt;
    }
    if (restored > value_) {
      value_ = restored;
    }
    return restored;
  }

  // bytes -> WireType -> TryRestore. Does not modify value_.
  constexpr decode_result TryDeserializeAndRestore(std::uint8_t const* in,
                                                    std::size_t len) const
      noexcept {
    decode_result result{};
    wire_type wire{};
    if (!ReadWireLittleEndian(in, len, wire)) {
      result.status = CyclicDecodeStatus::TruncatedInput;
      result.bytes_read = 0;
      return result;
    }
    result.bytes_read = sizeof(wire_type);

    value_type restored{};
    CyclicRestoreStatus const st = TryRestoreStatus(wire, restored);
    if (st == CyclicRestoreStatus::Ambiguous) {
      result.status = CyclicDecodeStatus::Ambiguous;
      return result;
    }
    if (st == CyclicRestoreStatus::OutOfRange) {
      result.status = CyclicDecodeStatus::OutOfRange;
      return result;
    }
    result.status = CyclicDecodeStatus::Ok;
    result.value = restored;
    return result;
  }

  // bytes -> WireType -> TryAdvance. May raise value_ when restored is newer.
  constexpr decode_result TryDeserializeAndAdvance(std::uint8_t const* in,
                                                   std::size_t len) noexcept {
    decode_result result = TryDeserializeAndRestore(in, len);
    if (result.ok() && result.value > value_) {
      value_ = result.value;
    }
    return result;
  }

  constexpr CyclicCounter& operator++() noexcept {
    assert(value_ < std::numeric_limits<value_type>::max());
    ++value_;
    return *this;
  }

  constexpr CyclicCounter operator++(int) noexcept {
    CyclicCounter tmp = *this;
    ++(*this);
    return tmp;
  }

  constexpr CyclicCounter& operator--() noexcept {
    assert(value_ > 0);
    --value_;
    return *this;
  }

  constexpr CyclicCounter operator--(int) noexcept {
    CyclicCounter tmp = *this;
    --(*this);
    return tmp;
  }

  constexpr CyclicCounter& operator+=(value_type delta) noexcept {
    assert(delta == 0 ||
           value_ <= std::numeric_limits<value_type>::max() - delta);
    value_ += delta;
    return *this;
  }

  constexpr CyclicCounter& operator-=(value_type delta) noexcept {
    assert(value_ >= delta);
    value_ -= delta;
    return *this;
  }

  friend constexpr CyclicCounter operator+(CyclicCounter value,
                                           value_type delta) noexcept {
    value += delta;
    return value;
  }

  friend constexpr CyclicCounter operator+(value_type delta,
                                           CyclicCounter value) noexcept {
    value += delta;
    return value;
  }

  friend constexpr CyclicCounter operator-(CyclicCounter value,
                                           value_type delta) noexcept {
    value -= delta;
    return value;
  }

  friend constexpr bool operator==(CyclicCounter const& a,
                                   CyclicCounter const& b) noexcept {
    return a.value_ == b.value_;
  }
  friend constexpr bool operator!=(CyclicCounter const& a,
                                   CyclicCounter const& b) noexcept {
    return !(a == b);
  }
  friend constexpr bool operator<(CyclicCounter const& a,
                                  CyclicCounter const& b) noexcept {
    return a.value_ < b.value_;
  }
  friend constexpr bool operator<=(CyclicCounter const& a,
                                   CyclicCounter const& b) noexcept {
    return a.value_ <= b.value_;
  }
  friend constexpr bool operator>(CyclicCounter const& a,
                                  CyclicCounter const& b) noexcept {
    return b < a;
  }
  friend constexpr bool operator>=(CyclicCounter const& a,
                                   CyclicCounter const& b) noexcept {
    return b <= a;
  }

 private:
  static constexpr bool ReadWireLittleEndian(std::uint8_t const* in,
                                             std::size_t len,
                                             wire_type& out) noexcept {
    if (in == nullptr || len < sizeof(wire_type)) {
      return false;
    }
    wire_type bits = 0;
    for (std::size_t i = 0; i < sizeof(wire_type); ++i) {
      bits |= static_cast<wire_type>(static_cast<wire_type>(in[i]) << (8 * i));
    }
    out = bits;
    return true;
  }

  value_type value_{};
};

enum class WireOrder : std::uint8_t {
  Same = 0,
  Newer = 1,
  Older = 2,
  Ambiguous = 3,
};

// Compare two wire samples relative to modular half-range (no full counter).
template <typename WireType>
  requires(std::is_integral_v<WireType> && std::is_unsigned_v<WireType> &&
           !std::is_same_v<WireType, bool>)
constexpr WireOrder CompareWire(WireType a, WireType b) noexcept {
  if (a == b) {
    return WireOrder::Same;
  }
  using Widen = std::conditional_t<
      (sizeof(WireType) < 2), std::uint16_t,
      std::conditional_t<(sizeof(WireType) < 4), std::uint32_t, std::uint32_t>>;
  static_assert(sizeof(Widen) > sizeof(WireType));
  Widen const space =
      static_cast<Widen>(std::numeric_limits<WireType>::max()) + Widen{1};
  Widen const half = space / Widen{2};
  WireType const forward = static_cast<WireType>(b - a);
  Widen const f = static_cast<Widen>(forward);
  if (f == half) {
    return WireOrder::Ambiguous;
  }
  if (f < half) {
    return WireOrder::Newer;
  }
  return WireOrder::Older;
}

}  // namespace ae

#endif  // AE_NUMERIC_CYCLIC_COUNTER_H_
