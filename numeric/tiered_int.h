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
#include <cstdint>
#include <functional>
#include <limits>
#include <tuple>
#include <type_traits>

namespace ae {

enum class TierDeserializeRes : std::uint8_t {
  kNo,
  kFinished,
  kNext,
};

namespace tiered_int_internal {

template <int StartSize, int NumTiers>
struct TieredStorageType {
  static_assert((StartSize == 1 || StartSize == 2 || StartSize == 4),
                "Unsupported base size");

  using type = std::conditional_t<
      (StartSize == 1 && NumTiers == 2), std::uint16_t,
      std::conditional_t<
          (StartSize == 1 && NumTiers == 3) ||
              (StartSize == 2 && NumTiers == 2),
          std::uint32_t, std::uint64_t>>;
};

template <typename T>
inline T read_little_endian(const std::uint8_t*& p, int bytes) {
  T out = 0;
  for (int i = 0; i < bytes; ++i) {
    out |= static_cast<T>(*p++) << (i * 8);
  }
  return out;
}

template <typename T>
inline void write_little_endian(std::uint8_t* p, T value, int bytes) {
  for (int i = 0; i < bytes; ++i) {
    *p++ = static_cast<std::uint8_t>((value >> (i * 8)) & 0xFF);
  }
}

}  // namespace tiered_int_internal

template <int StartSize, std::uint32_t... TierMaxVals>
struct TieredInt {
  static constexpr int NumTiers = sizeof...(TierMaxVals) + 1;
  using ValueType =
      typename tiered_int_internal::TieredStorageType<StartSize,
                                                      NumTiers>::type;

  ValueType value_ = 0;

  static constexpr ValueType kUpper = []() constexpr {
    constexpr ValueType tiers[] = {TierMaxVals...};
    if constexpr (NumTiers == 1) {
      return static_cast<ValueType>(tiers[0]);
    } else if constexpr (NumTiers == 2) {
      return static_cast<ValueType>((255 - tiers[0] - 1) * 256 + tiers[0] + 1 +
                                    255);
    } else if constexpr (NumTiers == 3) {
      constexpr auto two_tier_v =
          static_cast<std::uint64_t>((255 - tiers[0] - 1) * 256 + tiers[0] +
                                     1 + 255);
      return static_cast<ValueType>(
          (two_tier_v - tiers[1] - 1) * 65536ull + tiers[1] + 1 + 65535ull);
    } else {
      constexpr auto three_tier_v =
          static_cast<std::uint64_t>((255 - tiers[0] - 1) * 256 + tiers[0] +
                                     1 + 255);
      constexpr auto three_tier_v2 =
          (three_tier_v - tiers[1] - 1) * 65536ull + tiers[1] + 1 + 65535ull;
      return static_cast<ValueType>((three_tier_v2 - tiers[2] - 1) *
                                        65536ull * 65536ull +
                                    tiers[2] + 1 + 4294967295ull);
    }
  }();

  constexpr TieredInt() = default;

  template <typename T>
  constexpr TieredInt(T v) : value_(static_cast<ValueType>(v)) {}

  constexpr operator ValueType() const noexcept { return value_; }

  std::size_t Serialize(std::uint8_t* out) const {
    constexpr ValueType tiers[] = {TierMaxVals...};
    auto v = value_;
    std::size_t ret = 0;

    if constexpr (NumTiers == 4) {
      if (v > tiers[2]) {
        auto b2 = (v - tiers[2] - 1) % (256ul * 256 * 256 * 256);
        v = ((v - tiers[2] - 1 - b2) / (256ul * 256 * 256 * 256)) + tiers[2] +
            1;
        tiered_int_internal::write_little_endian<std::uint32_t>(out + 4, b2, 4);
        ret = 8;
      }
    }
    if constexpr (NumTiers >= 3) {
      if (v > tiers[1]) {
        auto b2 = (v - tiers[1] - 1) % (256 * 256);
        v = ((v - tiers[1] - 1 - b2) / (256 * 256)) + tiers[1] + 1;
        tiered_int_internal::write_little_endian<std::uint16_t>(out + 2, b2, 2);
        ret = std::max(ret, 4ul);
      }
    }
    if (v > tiers[0]) {
      auto b2 = (v - tiers[0] - 1) % 256;
      v = ((v - tiers[0] - 1 - b2) / 256) + tiers[0] + 1;
      out[1] = static_cast<std::uint8_t>(b2);
      ret = std::max(ret, 2ul);
    }
    out[0] = static_cast<std::uint8_t>(v);
    return std::max(ret, 1ul);
  }

  std::size_t Deserialize(const std::uint8_t* in) {
    constexpr ValueType tiers[] = {TierMaxVals...};

    ValueType v = *in++;
    if (v <= tiers[0]) {
      value_ = v;
      return 1;
    }

    std::uint8_t b2 = *in++;
    v = (v - tiers[0] - 1) * 256 + tiers[0] + 1 + b2;
    if constexpr (NumTiers < 3) {
      value_ = v;
      return 2;
    }
    if (v <= tiers[1]) {
      value_ = v;
      return 2;
    }

    std::uint16_t next2 =
        tiered_int_internal::read_little_endian<std::uint16_t>(in, 2);
    v = (v - tiers[1] - 1) * 256 * 256 + tiers[1] + 1 + next2;
    if constexpr (NumTiers < 4) {
      value_ = v;
      return 4;
    }
    if (v <= tiers[2]) {
      value_ = v;
      return 4;
    }

    if constexpr (NumTiers == 4) {
      std::uint32_t next4 =
          tiered_int_internal::read_little_endian<std::uint32_t>(in, 4);
      value_ = (v - tiers[2] - 1) * 256ull * 256 * 256 * 256 + tiers[2] + 1 +
               next4;
    }
    return 8;
  }

  template <typename TStream,
            std::enable_if_t<
                !std::is_pointer_v<std::decay_t<TStream>> &&
                    !std::is_array_v<std::remove_reference_t<TStream>>,
                int> = 0>
  void SerializeTo(TStream& os) const {
    std::uint8_t buf[8];
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
  TierDeserializeRes DeserializeFrom(TStream& is) {
    std::uint8_t buf[8] = {};
    is >> buf[0];

    constexpr ValueType tiers[] = {TierMaxVals...};

    if (buf[0] > tiers[0]) {
      is >> buf[1];
      if constexpr (NumTiers >= 3) {
        auto v = (buf[0] - tiers[0] - 1) * 256 + tiers[0] + 1 + buf[1];
        if (v > tiers[1]) {
          is >> buf[2];
          is >> buf[3];
          if constexpr (NumTiers == 4) {
            auto v2 = (v - tiers[1] - 1) * 256 * 256 + tiers[1] + 1 +
                      (static_cast<std::uint16_t>(buf[2]) |
                       (static_cast<std::uint16_t>(buf[3]) << 8));
            if (v2 > tiers[2]) {
              is >> buf[4];
              is >> buf[5];
              is >> buf[6];
              is >> buf[7];
            }
          }
        }
      }
    }

    Deserialize(buf);
    return TierDeserializeRes::kFinished;
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
