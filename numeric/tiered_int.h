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

namespace ae {

namespace tiered_int_internal {

inline constexpr std::size_t kAbsoluteMaxWireBytes = 8;

template <int StartSize>
struct StartSizeTraits {
  static_assert((StartSize == 1 || StartSize == 2 || StartSize == 4),
                "Unsupported base size");
  static constexpr int kBytes = StartSize;
  static constexpr std::uint64_t kWord =
      StartSize == 1   ? 256ull
      : StartSize == 2 ? 65536ull
                       : 4294967296ull;
  static constexpr std::uint64_t kHeaderMax = kWord - 1;
};

template <int StartSize>
constexpr std::uint64_t kWordPow(int tier_idx) {
  const std::uint64_t w = StartSizeTraits<StartSize>::kWord;
  if (tier_idx <= 0) {
    return w;
  }
  const std::uint64_t half = kWordPow<StartSize>(tier_idx - 1);
  return half * half;
}

template <int StartSize, std::uint32_t... TierMaxVals>
struct MaxEncodableValue {
  static constexpr int NumTiers = sizeof...(TierMaxVals) + 1;
  static constexpr std::uint64_t kWord = StartSizeTraits<StartSize>::kWord;
  static constexpr std::uint64_t kHeaderMax =
      StartSizeTraits<StartSize>::kHeaderMax;

  static constexpr std::uint64_t value = []() constexpr {
    constexpr std::uint32_t tiers[] = {TierMaxVals...};
    if constexpr (NumTiers == 1) {
      return tiers[0];
    } else if constexpr (NumTiers == 2) {
      return (kHeaderMax - tiers[0] - 1) * kWord + tiers[0] + 1 + kHeaderMax;
    } else if constexpr (NumTiers == 3) {
      constexpr auto two_tier_v =
          (kHeaderMax - tiers[0] - 1) * kWord + tiers[0] + 1 + kHeaderMax;
      constexpr auto word2 = kWordPow<StartSize>(1);
      return (two_tier_v - tiers[1] - 1) * word2 + tiers[1] + 1 + (word2 - 1);
    } else {
      constexpr auto two_tier_v =
          (kHeaderMax - tiers[0] - 1) * kWord + tiers[0] + 1 + kHeaderMax;
      constexpr auto word2 = kWordPow<StartSize>(1);
      constexpr auto three_tier_v =
          (two_tier_v - tiers[1] - 1) * word2 + tiers[1] + 1 + (word2 - 1);
      constexpr auto word4 = kWordPow<StartSize>(2);
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

}  // namespace tiered_int_internal

template <int StartSize, std::uint32_t... TierMaxVals>
struct TieredInt {
  static constexpr int NumTiers = sizeof...(TierMaxVals) + 1;
  static constexpr int kBaseBytes = StartSize;
  static constexpr std::uint64_t kWord =
      tiered_int_internal::StartSizeTraits<StartSize>::kWord;
  static constexpr std::uint64_t kHeaderMax =
      tiered_int_internal::StartSizeTraits<StartSize>::kHeaderMax;
  static constexpr std::size_t kMaxWireBytes =
      static_cast<std::size_t>(StartSize) * (1u << (NumTiers - 1));

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
      tiered_int_internal::MaxEncodableValue<StartSize,
                                             TierMaxVals...>::value;

  static_assert(
      kMaxEncodable <= std::numeric_limits<std::uint64_t>::max(),
      "TieredInt configuration exceeds uint64_t encodable range");

  using ValueType =
      typename tiered_int_internal::MinimalUIntFor<kMaxEncodable>::type;

  static_assert(kMaxEncodable <= std::numeric_limits<ValueType>::max(),
                "ValueType must hold kMaxEncodable");

  static constexpr ValueType kUpper = static_cast<ValueType>(kMaxEncodable);

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
    using Wide = std::uint64_t;
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
    if (static_cast<Wide>(v) > static_cast<Wide>(kUpper)) {
      if consteval {
        throw std::out_of_range(
            "TieredInt value exceeds the maximum encodable value");
      } else {
        assert(static_cast<Wide>(v) <= static_cast<Wide>(kUpper));
      }
    }
    return static_cast<ValueType>(v);
  }

 public:
  std::size_t Serialize(std::uint8_t* out) const {
    constexpr ValueType tiers[] = {TierMaxVals...};
    auto v = static_cast<std::uint64_t>(value_);
    std::size_t ret = 0;

    if constexpr (NumTiers == 4) {
      if (v > tiers[2]) {
        constexpr auto mod = tiered_int_internal::kWordPow<StartSize>(2);
        auto b2 = (v - tiers[2] - 1) % mod;
        v = ((v - tiers[2] - 1 - b2) / mod) + tiers[2] + 1;
        tiered_int_internal::write_little_endian_u64(
            out + kBaseBytes * 4, b2, kBaseBytes * 4);
        ret = kBaseBytes * 8;
      }
    }
    if constexpr (NumTiers >= 3) {
      if (v > tiers[1]) {
        constexpr auto mod = tiered_int_internal::kWordPow<StartSize>(1);
        auto b2 = (v - tiers[1] - 1) % mod;
        v = ((v - tiers[1] - 1 - b2) / mod) + tiers[1] + 1;
        tiered_int_internal::write_little_endian_u64(
            out + kBaseBytes * 2, b2, kBaseBytes * 2);
        ret = std::max(ret, kBaseBytes * 4ul);
      }
    }
    if (v > tiers[0]) {
      auto b2 = (v - tiers[0] - 1) % kWord;
      v = ((v - tiers[0] - 1 - b2) / kWord) + tiers[0] + 1;
      tiered_int_internal::write_little_endian_u64(out + kBaseBytes, b2,
                                                   kBaseBytes);
      ret = std::max(ret, kBaseBytes * 2ul);
    }
    tiered_int_internal::write_little_endian_u64(out, v, kBaseBytes);
    return std::max(ret, static_cast<std::size_t>(kBaseBytes));
  }

  std::size_t Deserialize(const std::uint8_t* in) {
    constexpr ValueType tiers[] = {TierMaxVals...};
    const std::uint8_t* p = in;

    std::uint64_t v =
        tiered_int_internal::read_little_endian_u64(p, kBaseBytes);
    p += kBaseBytes;
    if (v <= tiers[0]) {
      value_ = static_cast<ValueType>(v);
      return kBaseBytes;
    }

    std::uint64_t low =
        tiered_int_internal::read_little_endian_u64(p, kBaseBytes);
    p += kBaseBytes;
    v = (v - tiers[0] - 1) * kWord + tiers[0] + 1 + low;
    if constexpr (NumTiers < 3) {
      value_ = static_cast<ValueType>(v);
      return kBaseBytes * 2;
    } else {
      if (v <= tiers[1]) {
        value_ = static_cast<ValueType>(v);
        return kBaseBytes * 2;
      }

      low = tiered_int_internal::read_little_endian_u64(p, kBaseBytes * 2);
      p += kBaseBytes * 2;
      constexpr auto word2 = tiered_int_internal::kWordPow<StartSize>(1);
      v = (v - tiers[1] - 1) * word2 + tiers[1] + 1 + low;
      if constexpr (NumTiers < 4) {
        value_ = static_cast<ValueType>(v);
        return kBaseBytes * 4;
      } else {
        if (v <= tiers[2]) {
          value_ = static_cast<ValueType>(v);
          return kBaseBytes * 4;
        }

        low = tiered_int_internal::read_little_endian_u64(p, kBaseBytes * 4);
        constexpr auto word4 = tiered_int_internal::kWordPow<StartSize>(2);
        value_ = static_cast<ValueType>((v - tiers[2] - 1) * word4 + tiers[2] +
                                        1 + low);
        return kBaseBytes * 8;
      }
    }
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
    constexpr ValueType tiers[] = {TierMaxVals...};

    for (int i = 0; i < kBaseBytes; ++i) {
      is >> buf[i];
    }

    std::size_t total = kBaseBytes;
    std::uint64_t v =
        tiered_int_internal::read_little_endian_u64(buf, kBaseBytes);

    if (v > tiers[0]) {
      for (int i = 0; i < kBaseBytes; ++i) {
        is >> buf[kBaseBytes + i];
      }
      total = kBaseBytes * 2;
      if constexpr (NumTiers >= 3) {
        const std::uint64_t low =
            tiered_int_internal::read_little_endian_u64(buf + kBaseBytes,
                                                        kBaseBytes);
        v = (v - tiers[0] - 1) * kWord + tiers[0] + 1 + low;
        if (v > tiers[1]) {
          for (std::size_t i = 0; i < kBaseBytes * 2; ++i) {
            is >> buf[kBaseBytes * 2 + i];
          }
          total = kBaseBytes * 4;
          if constexpr (NumTiers == 4) {
            const std::uint64_t low2 =
                tiered_int_internal::read_little_endian_u64(
                    buf + kBaseBytes * 2, kBaseBytes * 2);
            constexpr auto word2 =
                tiered_int_internal::kWordPow<StartSize>(1);
            const std::uint64_t v2 =
                (v - tiers[1] - 1) * word2 + tiers[1] + 1 + low2;
            if (v2 > tiers[2]) {
              for (std::size_t i = 0; i < kBaseBytes * 4; ++i) {
                is >> buf[kBaseBytes * 4 + i];
              }
              total = kBaseBytes * 8;
            }
          }
        }
      }
    }

    (void)total;
    Deserialize(buf);
  }
};

template <int StartSize, std::uint32_t... TierMaxVals, typename TStream>
TStream& operator<<(TStream& os,
                    TieredInt<StartSize, TierMaxVals...> const& v) {
  v.SerializeTo(os);
  return os;
}

template <int StartSize, std::uint32_t... TierMaxVals, typename TStream>
TStream& operator>>(TStream& is, TieredInt<StartSize, TierMaxVals...>& v) {
  v.DeserializeFrom(is);
  return is;
}

template <int StartSize1, std::uint32_t... TierMaxVals1, int StartSize2,
          std::uint32_t... TierMaxVals2>
int TieredIntCompare(
    TieredInt<StartSize1, TierMaxVals1...> const& left,
    TieredInt<StartSize2, TierMaxVals2...> const& right) {
  if (left.value_ < right.value_) {
    return -1;
  }
  if (left.value_ > right.value_) {
    return 1;
  }
  return 0;
}

template <int StartSize1, std::uint32_t... TierMaxVals1, int StartSize2,
          std::uint32_t... TierMaxVals2>
bool operator==(TieredInt<StartSize1, TierMaxVals1...> const& left,
                TieredInt<StartSize2, TierMaxVals2...> const& right) {
  return TieredIntCompare(left, right) == 0;
}

template <int StartSize1, std::uint32_t... TierMaxVals1, int StartSize2,
          std::uint32_t... TierMaxVals2>
bool operator<(TieredInt<StartSize1, TierMaxVals1...> const& left,
               TieredInt<StartSize2, TierMaxVals2...> const& right) {
  return TieredIntCompare(left, right) < 0;
}

template <int StartSize1, std::uint32_t... TierMaxVals1, int StartSize2,
          std::uint32_t... TierMaxVals2>
bool operator>(TieredInt<StartSize1, TierMaxVals1...> const& left,
               TieredInt<StartSize2, TierMaxVals2...> const& right) {
  return TieredIntCompare(left, right) > 0;
}

}  // namespace ae

namespace std {

template <int StartSize, std::uint32_t... TierMaxVals>
class numeric_limits<ae::TieredInt<StartSize, TierMaxVals...>> {
 public:
  using T = ae::TieredInt<StartSize, TierMaxVals...>;
  static constexpr bool is_specialized = true;
  static constexpr bool is_signed = false;
  static constexpr int digits =
      std::numeric_limits<typename T::ValueType>::digits;
  static constexpr int digits10 =
      std::numeric_limits<typename T::ValueType>::digits10;
  static constexpr bool is_integer = true;
  static constexpr bool is_exact = true;
  static constexpr bool is_bounded = true;
  static constexpr bool is_modulo = false;

  static constexpr typename T::ValueType lowest() { return 0; }
  static constexpr typename T::ValueType min() { return 0; }
  static constexpr typename T::ValueType max() { return T::kUpper; }
};

template <int StartSize, std::uint32_t... TierMaxVals>
struct hash<ae::TieredInt<StartSize, TierMaxVals...>> {
  std::size_t operator()(
      ae::TieredInt<StartSize, TierMaxVals...> const& packed) const {
    return static_cast<std::size_t>(static_cast<typename ae::TieredInt<
        StartSize, TierMaxVals...>::ValueType>(packed));
  }
};

}  // namespace std

#endif  // NUMERIC_TIERED_INT_H_
