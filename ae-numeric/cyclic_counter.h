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

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

namespace ae {

// Compact cyclic counter: full ValueType locally, truncated WireType on the
// wire. Restoration is relative to the current full value (nearest unambiguous
// neighbor). Half wire-range is ambiguous and rejected by TryRestore.
//
// No epoch / previous / decoder state is stored — sizeof equals sizeof(ValueType).
template <typename WireType, typename ValueType>
  requires(std::is_integral_v<WireType> && std::is_unsigned_v<WireType> &&
           !std::is_same_v<WireType, bool> && std::is_integral_v<ValueType> &&
           std::is_unsigned_v<ValueType> && !std::is_same_v<ValueType, bool> &&
           (sizeof(ValueType) > sizeof(WireType)))
class CyclicCounter {
 public:
  using wire_type = WireType;
  using value_type = ValueType;

  static constexpr value_type kWireMask =
      static_cast<value_type>(std::numeric_limits<wire_type>::max());
  // Wire modular space: 2^(8*sizeof(WireType)). Fits in ValueType because
  // sizeof(ValueType) > sizeof(WireType).
  static constexpr value_type kWireSpace = kWireMask + value_type{1};
  static constexpr value_type kHalfRange = kWireSpace / value_type{2};

  static_assert(kWireSpace > kWireMask, "ValueType must hold wire space");
  static_assert(kHalfRange * value_type{2} == kWireSpace,
                "wire space must be even");

  constexpr CyclicCounter() noexcept = default;

  constexpr explicit CyclicCounter(value_type value) noexcept : value_(value) {}

  constexpr value_type Value() const noexcept { return value_; }

  constexpr wire_type WireValue() const noexcept {
    return static_cast<wire_type>(value_ & kWireMask);
  }

  // Nearest full value whose low bits equal `wire`, relative to value_.
  // Returns nullopt on half-range ambiguity or ValueType overflow/underflow.
  constexpr std::optional<value_type> TryRestore(wire_type wire) const noexcept {
    wire_type const current = WireValue();
    // Modular forward distance in wire space (unsigned wrap).
    wire_type const forward = static_cast<wire_type>(wire - current);

    if (forward == wire_type{0}) {
      return value_;
    }

    value_type const forward_v = static_cast<value_type>(forward);
    if (forward_v == kHalfRange) {
      return std::nullopt;
    }

    if (forward_v < kHalfRange) {
      if (value_ > (std::numeric_limits<value_type>::max() - forward_v)) {
        return std::nullopt;
      }
      return value_ + forward_v;
    }

    value_type const backward = kWireSpace - forward_v;
    if (value_ < backward) {
      return std::nullopt;
    }
    return value_ - backward;
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
    auto const restored = TryRestore(wire);
    if (!restored.has_value()) {
      return std::nullopt;
    }
    if (*restored > value_) {
      value_ = *restored;
    }
    return restored;
  }

  // Read a little-endian WireType from the buffer and TryAdvance.
  constexpr std::optional<value_type> TryDeserializeAndAdvance(
      std::uint8_t const* in, std::size_t len) noexcept {
    if (in == nullptr || len < sizeof(wire_type)) {
      return std::nullopt;
    }
    wire_type bits = 0;
    for (std::size_t i = 0; i < sizeof(wire_type); ++i) {
      bits |= static_cast<wire_type>(static_cast<wire_type>(in[i])
                                     << (8 * i));
    }
    return TryAdvance(bits);
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
  using Widen =
      std::conditional_t<sizeof(WireType) <= 1, std::uint16_t,
                         std::conditional_t<sizeof(WireType) <= 2, std::uint32_t,
                                            std::uint32_t>>;
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

#include "ae-numeric/wire_io.h"

namespace ae {

// Wire IO carries only the truncated WireType projection. Stateless
// Deserialize builds CyclicCounter(wire) (high bits zero) — not a restored
// full counter. Reconstruct with an existing base via TryRestore / TryAdvance
// or TryDeserializeAndAdvance.
template <typename WireType, typename ValueType>
struct wire_traits<CyclicCounter<WireType, ValueType>> {
  using T = CyclicCounter<WireType, ValueType>;
  using WireTraits = wire_traits<WireType>;

  static constexpr std::size_t kMaxWireBytes = WireTraits::kMaxWireBytes;

  static std::size_t Serialize(T const& value, std::uint8_t* out) noexcept {
    assert(out != nullptr);
    return WireTraits::Serialize(value.WireValue(), out);
  }

  static DeserializeResult<T> Deserialize(std::uint8_t const* in,
                                          std::size_t len) noexcept {
    auto const wire_result = WireTraits::Deserialize(in, len);
    return {T{static_cast<ValueType>(wire_result.value)},
            wire_result.bytes_read};
  }
};

}  // namespace ae

#endif  // AE_NUMERIC_CYCLIC_COUNTER_H_
