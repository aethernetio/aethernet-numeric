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

#ifndef AE_NUMERIC_WIRE_IO_H_
#define AE_NUMERIC_WIRE_IO_H_

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ae-numeric/fixed_point.h"

namespace ae {

template <typename T>
struct DeserializeResult {
  T value;
  std::size_t bytes_read;
};

namespace wire_io_internal {

template <typename T>
  requires(std::is_integral_v<T> && !std::is_same_v<T, bool>)
constexpr std::size_t SerializeLittleEndian(T value, std::uint8_t* out) {
  assert(out != nullptr);
  using Unsigned = std::make_unsigned_t<T>;
  Unsigned bits = static_cast<Unsigned>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    out[i] = static_cast<std::uint8_t>((bits >> (8 * i)) & Unsigned{0xFF});
  }
  return sizeof(T);
}

template <typename T>
  requires(std::is_integral_v<T> && !std::is_same_v<T, bool>)
constexpr T DeserializeLittleEndian(std::uint8_t const* in, std::size_t len) {
  if (len < sizeof(T)) {
    assert(len >= sizeof(T));
    return T{};
  }
  using Unsigned = std::make_unsigned_t<T>;
  Unsigned bits = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    bits |= static_cast<Unsigned>(static_cast<Unsigned>(in[i]) << (8 * i));
  }
  return static_cast<T>(bits);
}

}  // namespace wire_io_internal

template <typename T>
struct wire_traits;

template <typename T>
concept WireSerializable = requires(T value, std::uint8_t* out,
                                    std::uint8_t const* in, std::size_t len) {
  { wire_traits<T>::kMaxWireBytes } -> std::convertible_to<std::size_t>;
  { wire_traits<T>::Serialize(value, out) } -> std::same_as<std::size_t>;
  {
    wire_traits<T>::Deserialize(in, len)
  } -> std::same_as<DeserializeResult<T>>;
};

template <typename T>
  requires(std::is_integral_v<T> && !std::is_same_v<T, bool>)
struct wire_traits<T> {
  static constexpr std::size_t kMaxWireBytes = sizeof(T);

  static std::size_t Serialize(T value, std::uint8_t* out) noexcept {
    assert(out != nullptr);
    return wire_io_internal::SerializeLittleEndian(value, out);
  }

  static DeserializeResult<T> Deserialize(std::uint8_t const* in,
                                          std::size_t len) noexcept {
    return {wire_io_internal::DeserializeLittleEndian<T>(in, len), sizeof(T)};
  }
};

template <typename WireCell, std::uint32_t... TierMaxVals>
struct wire_traits<TieredInt<WireCell, TierMaxVals...>> {
  using T = TieredInt<WireCell, TierMaxVals...>;

  static constexpr std::size_t kMaxWireBytes = T::kMaxWireBytes;

  static std::size_t Serialize(T const& value, std::uint8_t* out) noexcept {
    assert(out != nullptr);
    return value.Serialize(out);
  }

  static DeserializeResult<T> Deserialize(std::uint8_t const* in,
                                          std::size_t len) {
    T value{};
    const std::size_t bytes_read = value.Deserialize(in, len);
    return {value, bytes_read};
  }
};

template <typename Rep, auto Max>
  requires IntegralStorage<Rep>
struct wire_traits<FixedPoint<Rep, Max>> {
  using T = FixedPoint<Rep, Max>;
  using RepTraits = wire_traits<Rep>;

  static constexpr std::size_t kMaxWireBytes = RepTraits::kMaxWireBytes;

  static std::size_t Serialize(T const& value, std::uint8_t* out) noexcept {
    assert(out != nullptr);
    return RepTraits::Serialize(value.Raw(), out);
  }

  static DeserializeResult<T> Deserialize(std::uint8_t const* in,
                                          std::size_t len) {
    auto const rep_result = RepTraits::Deserialize(in, len);
    return {T::FromRaw(rep_result.value), rep_result.bytes_read};
  }
};

template <WireSerializable T>
constexpr std::size_t MaxWireBytes() {
  return wire_traits<T>::kMaxWireBytes;
}

template <WireSerializable T>
std::size_t Serialize(T value, std::uint8_t* out) noexcept {
  assert(out != nullptr);
  return wire_traits<T>::Serialize(value, out);
}

template <WireSerializable T>
DeserializeResult<T> Deserialize(std::uint8_t const* in, std::size_t len) {
  return wire_traits<T>::Deserialize(in, len);
}

// Returns the number of bytes the next serialized T occupies at the start of
// the buffer, without requiring an external length. This relies on T being
// self-delimiting on the wire (the same property that lets it be skipped).
template <WireSerializable T>
std::size_t SerializedSizeAt(std::uint8_t const* in, std::size_t len) {
  return wire_traits<T>::Deserialize(in, len).bytes_read;
}

}  // namespace ae

#endif  // AE_NUMERIC_WIRE_IO_H_
