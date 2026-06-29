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

#ifndef NUMERIC_TIERED_INT_H_
#define NUMERIC_TIERED_INT_H_

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace ae {

namespace tiered_int_internal {

inline constexpr std::size_t kAbsoluteMaxWireBytes = 8;

template <typename WireCell>
struct WireCellTraits {
  static_assert(std::is_integral_v<WireCell> && !std::is_same_v<WireCell, bool>);
  static_assert(sizeof(WireCell) == 1 || sizeof(WireCell) == 2 ||
                sizeof(WireCell) == 4);

  static constexpr bool kIsSigned = std::is_signed_v<WireCell>;
  static constexpr int kBytes = static_cast<int>(sizeof(WireCell));
  static constexpr std::uint64_t kWord =
      (static_cast<std::uint64_t>(1) << (8 * kBytes));
  static constexpr std::uint64_t kHeaderMax = kWord - 1;
};

template <typename WireCell>
constexpr std::uint64_t kWordPow(int tier_idx) {
  const std::uint64_t w = WireCellTraits<WireCell>::kWord;
  if (tier_idx <= 0) {
    return w;
  }
  const std::uint64_t half = kWordPow<WireCell>(tier_idx - 1);
  return half * half;
}

constexpr std::uint64_t zigzag_encode64(std::int64_t n) {
  return (static_cast<std::uint64_t>(n) << 1) ^
         static_cast<std::uint64_t>(n >> 63);
}

constexpr std::int64_t zigzag_decode64(std::uint64_t u) {
  return static_cast<std::int64_t>((u >> 1) ^
                                   (-static_cast<std::int64_t>(u & 1)));
}

template <typename WireCell, std::uint32_t V>
constexpr std::uint32_t WireTierThreshold() {
  if constexpr (std::is_signed_v<WireCell>) {
    return static_cast<std::uint32_t>(
        zigzag_encode64(static_cast<std::int64_t>(V)));
  } else {
    return V;
  }
}

template <std::uint64_t MaxZigZag>
constexpr std::int64_t SignedUpperFromWireMax() {
  return zigzag_decode64(MaxZigZag);
}

template <std::uint64_t MaxZigZag>
constexpr std::int64_t SignedLowerFromWireMax() {
  if constexpr (MaxZigZag == 0) {
    return 0;
  }
  return -static_cast<std::int64_t>((MaxZigZag + 1) / 2);
}

template <typename WireCell, std::uint32_t... TierMaxVals>
struct MaxEncodableValue {
  static constexpr int NumTiers = sizeof...(TierMaxVals) + 1;
  static constexpr std::uint64_t kWord = WireCellTraits<WireCell>::kWord;
  static constexpr std::uint64_t kHeaderMax =
      WireCellTraits<WireCell>::kHeaderMax;

  static constexpr std::uint64_t value = []() constexpr {
    constexpr std::uint32_t tiers[] = {
        WireTierThreshold<WireCell, TierMaxVals>()...};
    if constexpr (NumTiers == 2) {
      return (kHeaderMax - tiers[0] - 1) * kWord + tiers[0] + 1 + kHeaderMax;
    } else if constexpr (NumTiers == 3) {
      constexpr auto two_tier_v =
          (kHeaderMax - tiers[0] - 1) * kWord + tiers[0] + 1 + kHeaderMax;
      constexpr auto word2 = kWordPow<WireCell>(1);
      return (two_tier_v - tiers[1] - 1) * word2 + tiers[1] + 1 + (word2 - 1);
    } else {
      constexpr auto two_tier_v =
          (kHeaderMax - tiers[0] - 1) * kWord + tiers[0] + 1 + kHeaderMax;
      constexpr auto word2 = kWordPow<WireCell>(1);
      constexpr auto three_tier_v =
          (two_tier_v - tiers[1] - 1) * word2 + tiers[1] + 1 + (word2 - 1);
      constexpr auto word4 = kWordPow<WireCell>(2);
      return (three_tier_v - tiers[2] - 1) * word4 + tiers[2] + 1 + (word4 - 1);
    }
  }();
};

template <std::uint64_t Max>
struct MinimalUIntFor {
  using type = std::conditional_t<
      Max <= static_cast<std::uint64_t>(std::numeric_limits<std::uint8_t>::max()),
      std::uint8_t,
      std::conditional_t<
          Max <= static_cast<std::uint64_t>(
                     std::numeric_limits<std::uint16_t>::max()),
          std::uint16_t,
          std::conditional_t<
              Max <= static_cast<std::uint64_t>(
                         std::numeric_limits<std::uint32_t>::max()),
              std::uint32_t, std::uint64_t>>>;
};

template <std::int64_t Min, std::int64_t Max>
struct MinimalIntFor {
  using type = std::conditional_t<
      Min >= static_cast<std::int64_t>(std::numeric_limits<std::int8_t>::min()) &&
          Max <= static_cast<std::int64_t>(std::numeric_limits<std::int8_t>::max()),
      std::int8_t,
      std::conditional_t<
          Min >= static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::min()) &&
              Max <= static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::max()),
          std::int16_t,
          std::conditional_t<
              Min >= static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()) &&
                  Max <= static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()),
              std::int32_t, std::int64_t>>>;
};

inline std::uint64_t read_little_endian_u64(const std::uint8_t* p, int bytes) {
  std::uint64_t out = 0;
  for (int i = 0; i < bytes; ++i) {
    out |= static_cast<std::uint64_t>(p[i]) << (i * 8);
  }
  return out;
}

inline void write_little_endian_u64(std::uint8_t* p, std::uint64_t value,
                                    int bytes) {
  for (int i = 0; i < bytes; ++i) {
    p[i] = static_cast<std::uint8_t>((value >> (i * 8)) & 0xFF);
  }
}

template <std::uint32_t First, std::uint32_t... Rest>
struct FirstOf {
  static constexpr std::uint32_t value = First;
};

template <std::uint32_t... Vals>
struct IsStrictlyIncreasing;

template <>
struct IsStrictlyIncreasing<> {
  static constexpr bool value = true;
};

template <std::uint32_t V>
struct IsStrictlyIncreasing<V> {
  static constexpr bool value = true;
};

template <std::uint32_t A, std::uint32_t B, std::uint32_t... Rest>
struct IsStrictlyIncreasing<A, B, Rest...> {
  static constexpr bool value =
      (A < B) && IsStrictlyIncreasing<B, Rest...>::value;
};

template <typename WireCell, std::uint32_t... Prefix>
struct TierConfigValidator {
  template <std::uint32_t Next>
  static consteval bool ValidateNext() {
    constexpr std::uint64_t max_enc =
        MaxEncodableValue<WireCell, Prefix...>::value;
    constexpr std::uint32_t wire_next =
        WireTierThreshold<WireCell, Next>();
    return wire_next <= max_enc;
  }
};

template <typename WireCell, std::uint32_t... TierMaxVals>
struct TierConfigValidationImpl;

template <typename WireCell, std::uint32_t First>
struct TierConfigValidationImpl<WireCell, First> {
  static constexpr bool kIsValid =
      WireTierThreshold<WireCell, First>() <
      WireCellTraits<WireCell>::kHeaderMax;
};

template <typename WireCell, std::uint32_t First, std::uint32_t Second>
struct TierConfigValidationImpl<WireCell, First, Second> {
  static constexpr bool kIsValid =
      TierConfigValidationImpl<WireCell, First>::kIsValid &&
      TierConfigValidator<WireCell, First>::template ValidateNext<Second>();
};

template <typename WireCell, std::uint32_t First, std::uint32_t Second,
          std::uint32_t Third>
struct TierConfigValidationImpl<WireCell, First, Second, Third> {
  static constexpr bool kIsValid =
      TierConfigValidationImpl<WireCell, First, Second>::kIsValid &&
      TierConfigValidator<WireCell, First, Second>::template ValidateNext<
          Third>();
};

template <typename WireCell, std::uint32_t First, std::uint32_t Second,
          std::uint32_t Third, std::uint32_t Fourth>
struct TierConfigValidationImpl<WireCell, First, Second, Third, Fourth> {
  static constexpr bool kIsValid =
      TierConfigValidationImpl<WireCell, First, Second, Third>::kIsValid &&
      TierConfigValidator<WireCell, First, Second, Third>::template ValidateNext<
          Fourth>();
};

template <typename WireCell, std::uint32_t... TierMaxVals>
struct TierConfigValidation {
  static constexpr bool kIsValid =
      TierConfigValidationImpl<WireCell, TierMaxVals...>::kIsValid;
};

template <typename WireCell, std::uint32_t... TierMaxVals>
inline constexpr bool kIsValidTierConfig =
    TierConfigValidation<WireCell, TierMaxVals...>::kIsValid;

}  // namespace tiered_int_internal

template <typename WireCell, std::uint32_t... TierMaxVals>
struct TieredInt {
  static_assert(tiered_int_internal::WireCellTraits<WireCell>::kBytes >= 1);

  static constexpr bool kIsSigned =
      tiered_int_internal::WireCellTraits<WireCell>::kIsSigned;
  static constexpr int NumTiers = sizeof...(TierMaxVals) + 1;
  static constexpr int kBaseBytes =
      tiered_int_internal::WireCellTraits<WireCell>::kBytes;
  static constexpr std::uint64_t kWord =
      tiered_int_internal::WireCellTraits<WireCell>::kWord;
  static constexpr std::uint64_t kHeaderMax =
      tiered_int_internal::WireCellTraits<WireCell>::kHeaderMax;
  static constexpr std::uint32_t kWireTier0 =
      tiered_int_internal::WireTierThreshold<
          WireCell, tiered_int_internal::FirstOf<TierMaxVals...>::value>();
  static constexpr std::size_t kMaxWireBytes =
      static_cast<std::size_t>(kBaseBytes) * (1u << (NumTiers - 1));

  static_assert(NumTiers <= 4, "At most 4 tiers are supported");
  static_assert(sizeof...(TierMaxVals) >= 1,
                "At least one tier maximum is required");
  static_assert(
      kMaxWireBytes <= tiered_int_internal::kAbsoluteMaxWireBytes,
      "Tier configuration exceeds the 8-byte wire limit");
  static_assert(
      tiered_int_internal::IsStrictlyIncreasing<TierMaxVals...>::value,
      "Tier max values must be strictly increasing");
  static_assert(
      tiered_int_internal::kIsValidTierConfig<WireCell, TierMaxVals...>,
      "Invalid TieredInt tier configuration: each tier boundary must fit "
      "in the previous wire size, and the first tier must leave extension "
      "header space.");

  static constexpr std::uint64_t kMaxEncodable =
      tiered_int_internal::MaxEncodableValue<WireCell,
                                             TierMaxVals...>::value;

  static_assert(
      kMaxEncodable <= std::numeric_limits<std::uint64_t>::max(),
      "TieredInt configuration exceeds uint64_t encodable range");

  static_assert(
      !kIsSigned ||
          (kMaxEncodable <=
           static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())),
      "Signed TieredInt configuration exceeds int64_t encodable range");

  using ValueType = std::conditional_t<
      kIsSigned,
      typename tiered_int_internal::MinimalIntFor<
          tiered_int_internal::SignedLowerFromWireMax<kMaxEncodable>(),
          tiered_int_internal::SignedUpperFromWireMax<kMaxEncodable>()>::type,
      typename tiered_int_internal::MinimalUIntFor<kMaxEncodable>::type>;

  static_assert(
      kIsSigned ||
          (kMaxEncodable <= std::numeric_limits<ValueType>::max()),
      "ValueType must hold kMaxEncodable");
  static_assert(
      !kIsSigned ||
          (tiered_int_internal::SignedLowerFromWireMax<kMaxEncodable>() >=
               static_cast<std::int64_t>(
                   std::numeric_limits<ValueType>::min()) &&
           tiered_int_internal::SignedUpperFromWireMax<kMaxEncodable>() <=
               static_cast<std::int64_t>(
                   std::numeric_limits<ValueType>::max())),
      "ValueType must hold the signed encodable range");

  static constexpr ValueType kUpper =
      kIsSigned ? static_cast<ValueType>(
                      tiered_int_internal::SignedUpperFromWireMax<kMaxEncodable>())
                : static_cast<ValueType>(kMaxEncodable);
  static constexpr ValueType kLower =
      kIsSigned ? static_cast<ValueType>(
                      tiered_int_internal::SignedLowerFromWireMax<kMaxEncodable>())
                : static_cast<ValueType>(0);

  ValueType value_ = 0;

  constexpr TieredInt() = default;

  template <typename T,
            std::enable_if_t<std::is_integral_v<T>, int> = 0>
  constexpr TieredInt(T v) : value_(check_value(v)) {}

  template <typename T,
            std::enable_if_t<std::is_integral_v<T>, int> = 0>
  constexpr TieredInt& operator=(T v) {
    value_ = check_value(v);
    return *this;
  }

  constexpr operator ValueType() const noexcept { return value_; }

 private:
  template <typename T>
  static constexpr ValueType check_signed_value(T v) {
    if constexpr (std::is_signed_v<T>) {
      if (v < kLower || v > kUpper) {
        if consteval {
          throw std::out_of_range(
              "TieredInt value is outside the encodable range");
        } else {
          assert(v >= kLower && v <= kUpper);
        }
      }
    } else {
      const std::uint64_t u = static_cast<std::uint64_t>(v);
      if (u > static_cast<std::uint64_t>(kUpper)) {
        if consteval {
          throw std::out_of_range(
              "TieredInt value is outside the encodable range");
        } else {
          assert(u <= static_cast<std::uint64_t>(kUpper));
        }
      }
      if constexpr (kLower > 0) {
        if (u < static_cast<std::uint64_t>(kLower)) {
          if consteval {
            throw std::out_of_range(
                "TieredInt value is outside the encodable range");
          } else {
            assert(u >= static_cast<std::uint64_t>(kLower));
          }
        }
      }
    }
    return static_cast<ValueType>(v);
  }

  template <typename T>
  static constexpr ValueType check_unsigned_value(T v) {
    if constexpr (std::is_signed_v<T>) {
      if (v < 0) {
        if consteval {
          throw std::invalid_argument(
              "TieredInt value must be non-negative");
        } else {
          assert(v >= 0);
        }
      }
    }
    const std::uint64_t u = static_cast<std::uint64_t>(v);
    if (u > static_cast<std::uint64_t>(kUpper)) {
      if consteval {
        throw std::out_of_range(
            "TieredInt value exceeds the maximum encodable value");
      } else {
        assert(u <= static_cast<std::uint64_t>(kUpper));
      }
    }
    return static_cast<ValueType>(v);
  }

  template <typename T>
  static constexpr ValueType check_value(T v) {
    if constexpr (kIsSigned) {
      return check_signed_value(v);
    } else {
      return check_unsigned_value(v);
    }
  }

  static constexpr std::uint32_t kWireTiers[] = {
      tiered_int_internal::WireTierThreshold<WireCell, TierMaxVals>()...};

  static std::size_t wire_bytes_needed(const std::uint8_t* in,
                                       std::size_t available) {
    if (available < static_cast<std::size_t>(kBaseBytes)) {
      return 0;
    }

    std::uint64_t v =
        tiered_int_internal::read_little_endian_u64(in, kBaseBytes);
    if (v <= kWireTiers[0]) {
      return static_cast<std::size_t>(kBaseBytes);
    }

    if (available < static_cast<std::size_t>(kBaseBytes * 2)) {
      return 0;
    }
    if constexpr (NumTiers < 3) {
      return static_cast<std::size_t>(kBaseBytes * 2);
    } else {
      std::uint64_t low = tiered_int_internal::read_little_endian_u64(
          in + kBaseBytes, kBaseBytes);
      v = (v - kWireTiers[0] - 1) * kWord + kWireTiers[0] + 1 + low;
      if (v <= kWireTiers[1]) {
        return static_cast<std::size_t>(kBaseBytes * 2);
      }

      if (available < static_cast<std::size_t>(kBaseBytes * 4)) {
        return 0;
      }
      if constexpr (NumTiers < 4) {
        return static_cast<std::size_t>(kBaseBytes * 4);
      } else {
        low = tiered_int_internal::read_little_endian_u64(in + kBaseBytes * 2,
                                                          kBaseBytes * 2);
        constexpr auto word2 = tiered_int_internal::kWordPow<WireCell>(1);
        v = (v - kWireTiers[1] - 1) * word2 + kWireTiers[1] + 1 + low;
        if (v <= kWireTiers[2]) {
          return static_cast<std::size_t>(kBaseBytes * 4);
        }

        if (available < static_cast<std::size_t>(kBaseBytes * 8)) {
          return 0;
        }
        return static_cast<std::size_t>(kBaseBytes * 8);
      }
    }
  }

  std::uint64_t deserialize_unsigned_magnitude(const std::uint8_t* in,
                                               std::size_t wire_bytes) const {
    constexpr std::uint32_t tiers[] = {
        tiered_int_internal::WireTierThreshold<WireCell, TierMaxVals>()...};

    std::uint64_t v =
        tiered_int_internal::read_little_endian_u64(in, kBaseBytes);
    if (wire_bytes == static_cast<std::size_t>(kBaseBytes)) {
      return v;
    }

    std::uint64_t low =
        tiered_int_internal::read_little_endian_u64(in + kBaseBytes, kBaseBytes);
    v = (v - tiers[0] - 1) * kWord + tiers[0] + 1 + low;
    if (wire_bytes == static_cast<std::size_t>(kBaseBytes * 2)) {
      return v;
    }
    if constexpr (NumTiers < 3) {
      return v;
    }

    if constexpr (NumTiers >= 3) {
      low = tiered_int_internal::read_little_endian_u64(in + kBaseBytes * 2,
                                                        kBaseBytes * 2);
      constexpr auto word2 = tiered_int_internal::kWordPow<WireCell>(1);
      v = (v - tiers[1] - 1) * word2 + tiers[1] + 1 + low;
    }
    if (wire_bytes == static_cast<std::size_t>(kBaseBytes * 4)) {
      return v;
    }
    if constexpr (NumTiers < 4) {
      return v;
    }

    if constexpr (NumTiers == 4) {
      low = tiered_int_internal::read_little_endian_u64(in + kBaseBytes * 4,
                                                        kBaseBytes * 4);
      constexpr auto word4 = tiered_int_internal::kWordPow<WireCell>(2);
      return (v - tiers[2] - 1) * word4 + tiers[2] + 1 + low;
    }
    return v;
  }

  std::size_t serialize_unsigned_magnitude(std::uint64_t v,
                                           std::uint8_t* out) const {
    constexpr std::uint32_t tiers[] = {
        tiered_int_internal::WireTierThreshold<WireCell, TierMaxVals>()...};
    std::size_t ret = 0;

    if constexpr (NumTiers == 4) {
      if (v > tiers[2]) {
        constexpr auto mod = tiered_int_internal::kWordPow<WireCell>(2);
        auto b2 = (v - tiers[2] - 1) % mod;
        v = ((v - tiers[2] - 1 - b2) / mod) + tiers[2] + 1;
        tiered_int_internal::write_little_endian_u64(
            out + kBaseBytes * 4, b2, kBaseBytes * 4);
        ret = kBaseBytes * 8;
      }
    }
    if constexpr (NumTiers >= 3) {
      if (v > tiers[1]) {
        constexpr auto mod = tiered_int_internal::kWordPow<WireCell>(1);
        auto b2 = (v - tiers[1] - 1) % mod;
        v = ((v - tiers[1] - 1 - b2) / mod) + tiers[1] + 1;
        tiered_int_internal::write_little_endian_u64(
            out + kBaseBytes * 2, b2, kBaseBytes * 2);
        ret = std::max(ret, static_cast<std::size_t>(kBaseBytes * 4));
      }
    }
    if (v > tiers[0]) {
      auto b2 = (v - tiers[0] - 1) % kWord;
      v = ((v - tiers[0] - 1 - b2) / kWord) + tiers[0] + 1;
      tiered_int_internal::write_little_endian_u64(out + kBaseBytes, b2,
                                                   kBaseBytes);
      ret = std::max(ret, static_cast<std::size_t>(kBaseBytes * 2));
    }
    tiered_int_internal::write_little_endian_u64(out, v, kBaseBytes);
    return std::max(ret, static_cast<std::size_t>(kBaseBytes));
  }

  static void throw_or_assert_buffer_too_short() {
    throw std::out_of_range("TieredInt deserialize buffer is too short");
  }

 public:
  std::size_t Serialize(std::uint8_t* out) const {
    if constexpr (kIsSigned) {
      const std::uint64_t u = tiered_int_internal::zigzag_encode64(
          static_cast<std::int64_t>(value_));
      return serialize_unsigned_magnitude(u, out);
    }
    return serialize_unsigned_magnitude(static_cast<std::uint64_t>(value_), out);
  }

  std::size_t Deserialize(const std::uint8_t* in, std::size_t len) {
    const std::size_t wire_bytes = wire_bytes_needed(in, len);
    if (wire_bytes == 0) {
      throw_or_assert_buffer_too_short();
    }
    const std::uint64_t u =
        deserialize_unsigned_magnitude(in, wire_bytes);
    if constexpr (kIsSigned) {
      value_ = static_cast<ValueType>(tiered_int_internal::zigzag_decode64(u));
    } else {
      value_ = static_cast<ValueType>(u);
    }
    return wire_bytes;
  }

  std::size_t Deserialize(const std::uint8_t* in) {
    return Deserialize(in, kMaxWireBytes);
  }

  template <typename TStream,
            std::enable_if_t<
                !std::is_pointer_v<std::decay_t<TStream>> &&
                    !std::is_array_v<std::remove_reference_t<TStream>>,
                int> = 0>
  void SerializeTo(TStream& os) const {
    std::uint8_t buf[kMaxWireBytes];
    auto n = Serialize(buf);
    for (std::size_t i = 0; i < n; ++i) {
      std::uint8_t b = buf[i];
      os << b;
    }
  }

  template <typename TStream,
            std::enable_if_t<
                !std::is_pointer_v<std::decay_t<TStream>> &&
                    !std::is_array_v<std::remove_reference_t<TStream>>,
                int> = 0>
  void DeserializeFrom(TStream& is) {
    std::uint8_t buf[kMaxWireBytes] = {};
    std::size_t len = 0;

    for (int i = 0; i < kBaseBytes; ++i) {
      is >> buf[len++];
    }

    std::size_t wire_bytes = wire_bytes_needed(buf, len);
    while (wire_bytes == 0 && len < kMaxWireBytes) {
      is >> buf[len++];
      wire_bytes = wire_bytes_needed(buf, len);
    }

    Deserialize(buf, len);
  }
};

template <typename WireCell1, std::uint32_t... TierMaxVals1,
          typename WireCell2, std::uint32_t... TierMaxVals2>
int TieredIntCompare(
    TieredInt<WireCell1, TierMaxVals1...> const& left,
    TieredInt<WireCell2, TierMaxVals2...> const& right) {
  using LeftValue =
      typename TieredInt<WireCell1, TierMaxVals1...>::ValueType;
  using RightValue =
      typename TieredInt<WireCell2, TierMaxVals2...>::ValueType;
  const LeftValue& l = left.value_;
  const RightValue& r = right.value_;
  if (std::cmp_less(l, r)) {
    return -1;
  }
  if (std::cmp_greater(l, r)) {
    return 1;
  }
  return 0;
}

template <typename WireCell1, std::uint32_t... TierMaxVals1,
          typename WireCell2, std::uint32_t... TierMaxVals2>
bool operator==(TieredInt<WireCell1, TierMaxVals1...> const& left,
                TieredInt<WireCell2, TierMaxVals2...> const& right) {
  return TieredIntCompare(left, right) == 0;
}

template <typename WireCell1, std::uint32_t... TierMaxVals1,
          typename WireCell2, std::uint32_t... TierMaxVals2>
bool operator<(TieredInt<WireCell1, TierMaxVals1...> const& left,
               TieredInt<WireCell2, TierMaxVals2...> const& right) {
  return TieredIntCompare(left, right) < 0;
}

template <typename WireCell1, std::uint32_t... TierMaxVals1,
          typename WireCell2, std::uint32_t... TierMaxVals2>
bool operator>(TieredInt<WireCell1, TierMaxVals1...> const& left,
               TieredInt<WireCell2, TierMaxVals2...> const& right) {
  return TieredIntCompare(left, right) > 0;
}

}  // namespace ae

namespace std {

template <typename WireCell, std::uint32_t... TierMaxVals>
class numeric_limits<ae::TieredInt<WireCell, TierMaxVals...>> {
 public:
  using T = ae::TieredInt<WireCell, TierMaxVals...>;
  static constexpr bool is_specialized = true;
  static constexpr bool is_signed = T::kIsSigned;
  static constexpr int digits =
      std::numeric_limits<typename T::ValueType>::digits;
  static constexpr int digits10 =
      std::numeric_limits<typename T::ValueType>::digits10;
  static constexpr bool is_integer = true;
  static constexpr bool is_exact = true;
  static constexpr bool is_bounded = true;
  static constexpr bool is_modulo = false;

  static constexpr typename T::ValueType lowest() { return T::kLower; }
  static constexpr typename T::ValueType min() { return T::kLower; }
  static constexpr typename T::ValueType max() { return T::kUpper; }
};

template <typename WireCell, std::uint32_t... TierMaxVals>
struct hash<ae::TieredInt<WireCell, TierMaxVals...>> {
  std::size_t operator()(
      ae::TieredInt<WireCell, TierMaxVals...> const& packed) const {
    using ValueType = typename ae::TieredInt<WireCell, TierMaxVals...>::ValueType;
    using HashType = std::make_unsigned_t<ValueType>;
    return static_cast<std::size_t>(static_cast<HashType>(packed.value_));
  }
};

}  // namespace std

#endif  // NUMERIC_TIERED_INT_H_
