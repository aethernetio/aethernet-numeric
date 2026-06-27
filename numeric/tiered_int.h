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
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <type_traits>

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

template <typename WireCell, std::uint32_t... TierMaxVals>
struct MaxEncodableValue {
  static constexpr int NumTiers = sizeof...(TierMaxVals) + 1;
  static constexpr std::uint64_t kWord = WireCellTraits<WireCell>::kWord;
  static constexpr std::uint64_t kHeaderMax =
      WireCellTraits<WireCell>::kHeaderMax;

  static constexpr std::uint64_t value = []() constexpr {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};
    if constexpr (NumTiers == 1) {
      return tiers[0];
    } else if constexpr (NumTiers == 2) {
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

template <typename WireCell>
WireCell read_le(const std::uint8_t* p, int bytes) {
  using Unsigned = std::make_unsigned_t<WireCell>;
  const auto bits = static_cast<Unsigned>(read_little_endian_u64(p, bytes));
  return static_cast<WireCell>(bits);
}

template <typename WireCell>
void write_le(std::uint8_t* p, WireCell value, int bytes) {
  using Unsigned = std::make_unsigned_t<WireCell>;
  write_little_endian_u64(p, static_cast<std::uint64_t>(static_cast<Unsigned>(value)),
                          bytes);
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

template <int StartSize>
struct WireCellFromStartSize;

template <>
struct WireCellFromStartSize<1> {
  using type = std::uint8_t;
};

template <>
struct WireCellFromStartSize<2> {
  using type = std::uint16_t;
};

template <>
struct WireCellFromStartSize<4> {
  using type = std::uint32_t;
};

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
  static_assert(tiered_int_internal::FirstOf<TierMaxVals...>::value <= kHeaderMax,
                "First tier max must fit in the base wire word");

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
          -static_cast<std::int64_t>(kMaxEncodable),
          static_cast<std::int64_t>(kMaxEncodable)>::type,
      typename tiered_int_internal::MinimalUIntFor<kMaxEncodable>::type>;

  static_assert(
      kIsSigned ||
          (kMaxEncodable <= std::numeric_limits<ValueType>::max()),
      "ValueType must hold kMaxEncodable");
  static_assert(
      !kIsSigned ||
          (static_cast<std::int64_t>(kMaxEncodable) <=
           static_cast<std::int64_t>(std::numeric_limits<ValueType>::max()) &&
           -static_cast<std::int64_t>(kMaxEncodable) >=
               static_cast<std::int64_t>(std::numeric_limits<ValueType>::min())),
      "ValueType must hold the signed encodable range");

  static constexpr ValueType kUpper =
      kIsSigned ? static_cast<ValueType>(kMaxEncodable) : static_cast<ValueType>(kMaxEncodable);
  static constexpr ValueType kLower =
      kIsSigned ? static_cast<ValueType>(-static_cast<std::int64_t>(kMaxEncodable))
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
  static constexpr ValueType check_value(T v) {
    if constexpr (kIsSigned) {
      const std::int64_t w = static_cast<std::int64_t>(v);
      if (w < static_cast<std::int64_t>(kLower) ||
          w > static_cast<std::int64_t>(kUpper)) {
        if consteval {
          throw std::out_of_range(
              "TieredInt value is outside the encodable range");
        } else {
          assert(w >= static_cast<std::int64_t>(kLower) &&
                 w <= static_cast<std::int64_t>(kUpper));
        }
      }
    } else {
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
    }
    return static_cast<ValueType>(v);
  }

  static constexpr std::uint64_t kSignBit =
      static_cast<std::uint64_t>(1) << (8 * kBaseBytes - 1);

  std::size_t serialize_unsigned_magnitude(std::uint64_t v,
                                           std::uint8_t* out) const {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};
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

  std::uint64_t deserialize_unsigned_magnitude(const std::uint8_t* in) const {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};
    const std::uint8_t* p = in;

    std::uint64_t v = tiered_int_internal::read_little_endian_u64(p, kBaseBytes);
    p += kBaseBytes;
    if (v <= tiers[0]) {
      return v;
    }

    std::uint64_t low =
        tiered_int_internal::read_little_endian_u64(p, kBaseBytes);
    p += kBaseBytes;
    v = (v - tiers[0] - 1) * kWord + tiers[0] + 1 + low;
    if constexpr (NumTiers < 3) {
      return v;
    }

    if constexpr (NumTiers >= 3) {
      if (v <= tiers[1]) {
        return v;
      }

      low = tiered_int_internal::read_little_endian_u64(p, kBaseBytes * 2);
      p += kBaseBytes * 2;
      constexpr auto word2 = tiered_int_internal::kWordPow<WireCell>(1);
      v = (v - tiers[1] - 1) * word2 + tiers[1] + 1 + low;
    }
    if constexpr (NumTiers < 4) {
      return v;
    }

    if constexpr (NumTiers == 4) {
      if (v <= tiers[2]) {
        return v;
      }

      low = tiered_int_internal::read_little_endian_u64(p, kBaseBytes * 4);
      constexpr auto word4 = tiered_int_internal::kWordPow<WireCell>(2);
      return (v - tiers[2] - 1) * word4 + tiers[2] + 1 + low;
    }
    return v;
  }

  std::size_t wire_bytes_for_magnitude(std::uint64_t v) const {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};
    if (v <= tiers[0]) {
      return kBaseBytes;
    }
    if constexpr (NumTiers < 3) {
      return kBaseBytes * 2;
    }
    if constexpr (NumTiers >= 3) {
      if (v <= tiers[1]) {
        return kBaseBytes * 2;
      }
    }
    if constexpr (NumTiers < 4) {
      return kBaseBytes * 4;
    }
    if constexpr (NumTiers == 4) {
      if (v <= tiers[2]) {
        return kBaseBytes * 4;
      }
    }
    return kBaseBytes * 8;
  }

 public:
  std::size_t Serialize(std::uint8_t* out) const {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};

    if constexpr (kIsSigned) {
      const auto v = static_cast<std::int64_t>(value_);
      const auto tier0 = static_cast<std::int64_t>(tiers[0]);
      if (v >= -tier0 && v <= tier0) {
        tiered_int_internal::write_le(out, static_cast<WireCell>(value_),
                                      kBaseBytes);
        return kBaseBytes;
      }
      if (v < 0) {
        std::uint8_t tmp[kMaxWireBytes];
        const auto mag = static_cast<std::uint64_t>(-v);
        const auto n = serialize_unsigned_magnitude(mag, tmp);
        using Unsigned = std::make_unsigned_t<WireCell>;
        const auto first_u = static_cast<std::uint64_t>(
            static_cast<Unsigned>(
                tiered_int_internal::read_le<WireCell>(tmp, kBaseBytes)));
        tiered_int_internal::write_little_endian_u64(
            out, first_u | kSignBit, kBaseBytes);
        for (std::size_t i = static_cast<std::size_t>(kBaseBytes); i < n; ++i) {
          out[i] = tmp[i];
        }
        return n;
      }
    }

    return serialize_unsigned_magnitude(static_cast<std::uint64_t>(value_), out);
  }

  std::size_t Deserialize(const std::uint8_t* in) {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};

    if constexpr (kIsSigned) {
      using Unsigned = std::make_unsigned_t<WireCell>;
      const WireCell first = tiered_int_internal::read_le<WireCell>(in, kBaseBytes);
      const auto first_u = static_cast<std::uint64_t>(static_cast<Unsigned>(first));
      const auto tier0 = static_cast<std::int64_t>(tiers[0]);
      const auto tier0_u = static_cast<std::uint64_t>(tiers[0]);
      const auto first_v = static_cast<std::int64_t>(first);

      const bool in_overflow_zone =
          first_u > tier0_u && first_u < (kWord - tier0_u);

      if (!in_overflow_zone && first_v >= -tier0 && first_v <= tier0) {
        value_ = static_cast<ValueType>(first);
        return kBaseBytes;
      }

      if (in_overflow_zone && (first_u & kSignBit) != 0) {
        std::uint8_t tmp[kMaxWireBytes];
        std::memcpy(tmp, in, kMaxWireBytes);
        tiered_int_internal::write_little_endian_u64(
            tmp, first_u & ~kSignBit, kBaseBytes);
        const auto mag = deserialize_unsigned_magnitude(tmp);
        value_ = static_cast<ValueType>(-static_cast<std::int64_t>(mag));
        return wire_bytes_for_magnitude(mag);
      }
    }

    value_ = static_cast<ValueType>(deserialize_unsigned_magnitude(in));
    return wire_bytes_for_magnitude(static_cast<std::uint64_t>(value_));
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
    constexpr std::uint32_t tiers[] = {TierMaxVals...};

    for (int i = 0; i < kBaseBytes; ++i) {
      is >> buf[i];
    }

    const std::uint64_t first_u =
        tiered_int_internal::read_little_endian_u64(buf, kBaseBytes);

    if constexpr (kIsSigned) {
      const auto tier0_u = static_cast<std::uint64_t>(tiers[0]);
      const WireCell first = tiered_int_internal::read_le<WireCell>(buf, kBaseBytes);
      const auto first_v = static_cast<std::int64_t>(first);
      const auto tier0 = static_cast<std::int64_t>(tiers[0]);
      const bool in_overflow_zone =
          first_u > tier0_u && first_u < (kWord - tier0_u);

      if (!in_overflow_zone && first_v >= -tier0 && first_v <= tier0) {
        Deserialize(buf);
        return;
      }
    }

    if (first_u > tiers[0]) {
      for (int i = 0; i < kBaseBytes; ++i) {
        is >> buf[kBaseBytes + i];
      }
      if constexpr (NumTiers >= 3) {
        const std::uint64_t low =
            tiered_int_internal::read_little_endian_u64(buf + kBaseBytes,
                                                        kBaseBytes);
        const std::uint64_t v =
            (first_u - tiers[0] - 1) * kWord + tiers[0] + 1 + low;
        if (v > tiers[1]) {
          for (std::size_t i = 0; i < static_cast<std::size_t>(kBaseBytes * 2);
               ++i) {
            is >> buf[kBaseBytes * 2 + i];
          }
          if constexpr (NumTiers == 4) {
            const std::uint64_t low2 =
                tiered_int_internal::read_little_endian_u64(
                    buf + kBaseBytes * 2, kBaseBytes * 2);
            constexpr auto word2 = tiered_int_internal::kWordPow<WireCell>(1);
            const std::uint64_t v2 =
                (v - tiers[1] - 1) * word2 + tiers[1] + 1 + low2;
            if (v2 > tiers[2]) {
              for (std::size_t i = 0; i < static_cast<std::size_t>(kBaseBytes * 4);
                   ++i) {
                is >> buf[kBaseBytes * 4 + i];
              }
            }
          }
        }
      }
    }

    Deserialize(buf);
  }
};

template <int StartSize, std::uint32_t... TierMaxVals>
using TieredIntFromStartSize =
    TieredInt<typename tiered_int_internal::WireCellFromStartSize<StartSize>::type,
              TierMaxVals...>;

template <typename WireCell1, std::uint32_t... TierMaxVals1,
          typename WireCell2, std::uint32_t... TierMaxVals2>
int TieredIntCompare(
    TieredInt<WireCell1, TierMaxVals1...> const& left,
    TieredInt<WireCell2, TierMaxVals2...> const& right) {
  if (left.value_ < right.value_) {
    return -1;
  }
  if (left.value_ > right.value_) {
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

template <typename WireCell, std::uint32_t... TierMaxVals, typename TStream>
TStream& operator<<(TStream& os,
                    TieredInt<WireCell, TierMaxVals...> const& v) {
  v.SerializeTo(os);
  return os;
}

template <typename WireCell, std::uint32_t... TierMaxVals, typename TStream>
TStream& operator>>(TStream& is, TieredInt<WireCell, TierMaxVals...>& v) {
  v.DeserializeFrom(is);
  return is;
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
