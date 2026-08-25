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

#ifndef AE_NUMERIC_TESTS_LEGACY_AETHER_TIERED_CODEC_H_
#define AE_NUMERIC_TESTS_LEGACY_AETHER_TIERED_CODEC_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

// Reference codec for historical Aether
// TieredInt<std::uint64_t, std::uint8_t, 250>.
// Algorithm matches main:numeric/tiered_int.h, but splits high/low with memcpy
// instead of union type-punning (same LE layout, no UB under UBSan).

namespace ae::legacy {
namespace detail {

using TierTypeList =
    std::tuple<std::uint8_t, std::uint16_t, std::uint32_t, std::uint64_t>;

template <typename T, typename TypeList, std::size_t... Is>
constexpr auto TierIndexImpl(std::index_sequence<Is...> const&) {
  constexpr auto arr =
      std::array{std::is_same_v<T, std::tuple_element_t<Is, TypeList>>...};
  std::size_t res = 0;
  for (; res < arr.size(); ++res) {
    if (arr[res]) {
      break;
    }
  }
  return res;
}

template <typename T, typename TypeList = TierTypeList>
constexpr std::size_t TierIndex() {
  return TierIndexImpl<T, TypeList>(
      std::make_index_sequence<std::tuple_size_v<TypeList>>());
}

template <typename MaxStored, typename MinStored, MinStored MinMaxStoredValue>
struct Limit {
  static constexpr auto kRangeIndex = TierIndex<MaxStored>();
  using PrevTierLimit =
      Limit<std::tuple_element_t<kRangeIndex - 1, TierTypeList>, MinStored,
            MinMaxStoredValue>;
  using StoredType = MaxStored;

  static constexpr auto kAbsoluteMaxValue =
      std::numeric_limits<StoredType>::max();

  static constexpr auto kP = static_cast<StoredType>(1)
                             << (std::numeric_limits<StoredType>::digits >> 1);

  static constexpr auto kMaxStored =
      (static_cast<StoredType>(PrevTierLimit::kMaxStored)) -
      PrevTierLimit::kBorrowCount +
      (PrevTierLimit::kBorrowCount *
       (static_cast<StoredType>(PrevTierLimit::kAbsoluteMaxValue) + 1));

  static constexpr auto kBorrowCount = PrevTierLimit::kP;

  static constexpr auto kUpper =
      kMaxStored - static_cast<StoredType>(PrevTierLimit::kP);
};

template <typename Stored, Stored MaxStoredValue>
struct Limit<Stored, Stored, MaxStoredValue> {
  using StoredType = Stored;
  static constexpr auto kRangeIndex = TierIndex<StoredType>();
  static constexpr auto kAbsoluteMaxValue =
      std::numeric_limits<StoredType>::max();
  static constexpr auto kBorrowCount =
      std::numeric_limits<StoredType>::max() - MaxStoredValue;
  static constexpr std::uint64_t kMaxStored = kAbsoluteMaxValue + 1;
  static constexpr auto kUpper =
      std::numeric_limits<StoredType>::max() - kBorrowCount + 1;
  static constexpr auto kP = static_cast<Stored>(1)
                             << (std::numeric_limits<Stored>::digits >> 1);
};

template <typename T>
void WriteRaw(std::uint8_t* out, std::size_t& n, T const& value) {
  std::memcpy(out + n, &value, sizeof(T));
  n += sizeof(T);
}

template <typename T>
bool ReadRaw(std::uint8_t const* in, std::size_t in_len, std::size_t& pos,
             T& value) {
  if (pos + sizeof(T) > in_len) {
    return false;
  }
  std::memcpy(&value, in + pos, sizeof(T));
  pos += sizeof(T);
  return true;
}

template <typename Value, typename Half>
void SplitLE(Value value, Half& low, Half& high) {
  static_assert(sizeof(Value) == 2 * sizeof(Half));
  std::uint8_t bytes[sizeof(Value)];
  std::memcpy(bytes, &value, sizeof(Value));
  std::memcpy(&low, bytes, sizeof(Half));
  std::memcpy(&high, bytes + sizeof(Half), sizeof(Half));
}

template <typename Value, typename Half>
Value JoinLE(Half low, Half high) {
  static_assert(sizeof(Value) == 2 * sizeof(Half));
  std::uint8_t bytes[sizeof(Value)];
  std::memcpy(bytes, &low, sizeof(Half));
  std::memcpy(bytes + sizeof(Half), &high, sizeof(Half));
  Value value{};
  std::memcpy(&value, bytes, sizeof(Value));
  return value;
}

enum class Res : std::uint8_t { kFinished, kNext };

template <typename MaxStoredType, typename MinStoredType,
          MinStoredType MinMaxStoredValue>
struct Codec;

template <typename StoredType, StoredType MaxValue>
struct Codec<StoredType, StoredType, MaxValue> {
  using ValueType = StoredType;
  static constexpr ValueType kUpper =
      Limit<StoredType, StoredType, MaxValue>::kUpper;

  static void Serialize(ValueType value, std::uint8_t* out, std::size_t& n) {
    WriteRaw(out, n, value);
  }

  static bool Deserialize(std::uint8_t const* in, std::size_t in_len,
                          std::size_t& pos, ValueType& value, Res& res) {
    if (!ReadRaw(in, in_len, pos, value)) {
      return false;
    }
    if (value >= kUpper) {
      value = static_cast<ValueType>(value - kUpper);
      res = Res::kNext;
    } else {
      res = Res::kFinished;
    }
    return true;
  }
};

template <typename MaxStoredType, typename MinStoredType,
          MinStoredType MinMaxStoredValue>
struct Codec {
  using LimitType =
      Limit<MaxStoredType, MinStoredType, MinMaxStoredValue>;
  using ValueType = typename LimitType::StoredType;
  using PrevType = typename LimitType::PrevTierLimit::StoredType;
  using PrevCodec = Codec<PrevType, MinStoredType, MinMaxStoredValue>;

  static constexpr ValueType kUpper = LimitType::kUpper;

  static void Serialize(ValueType value, std::uint8_t* out, std::size_t& n) {
    constexpr auto prev_upper = PrevCodec::kUpper;
    if (value >= prev_upper) {
      ValueType modified =
          static_cast<ValueType>(value - static_cast<ValueType>(prev_upper));
      PrevType low{};
      PrevType high{};
      SplitLE(modified, low, high);
      PrevCodec::Serialize(
          static_cast<PrevType>(high + static_cast<PrevType>(prev_upper)), out,
          n);
      WriteRaw(out, n, low);
      return;
    }
    PrevType low{};
    PrevType high{};
    SplitLE(value, low, high);
    (void)high;
    PrevCodec::Serialize(low, out, n);
  }

  static bool Deserialize(std::uint8_t const* in, std::size_t in_len,
                          std::size_t& pos, ValueType& value, Res& res) {
    constexpr auto prev_upper = PrevCodec::kUpper;
    PrevType high = 0;
    Res prev_res = Res::kFinished;
    if (!PrevCodec::Deserialize(in, in_len, pos, high, prev_res)) {
      return false;
    }
    if (prev_res == Res::kFinished) {
      value = static_cast<ValueType>(high);
      res = Res::kFinished;
      return true;
    }
    PrevType low = 0;
    if (!ReadRaw(in, in_len, pos, low)) {
      return false;
    }
    value = JoinLE<ValueType>(low, high);
    value = static_cast<ValueType>(value + static_cast<ValueType>(prev_upper));
    if (value >= kUpper) {
      value = static_cast<ValueType>(value - kUpper);
      res = Res::kNext;
    } else {
      res = Res::kFinished;
    }
    return true;
  }
};

}  // namespace detail

using LegacyAetherPackedSize =
    detail::Codec<std::uint64_t, std::uint8_t, 250>;

inline constexpr std::uint64_t kLegacyExclusiveUpper =
    static_cast<std::uint64_t>(LegacyAetherPackedSize::kUpper);
inline constexpr std::uint64_t kLegacyInclusiveMax = kLegacyExclusiveUpper - 1U;

inline std::size_t Encode(std::uint64_t value, std::uint8_t* out,
                          std::size_t out_cap) {
  std::size_t n = 0;
  if (out_cap < 8) {
    return out_cap + 1;
  }
  LegacyAetherPackedSize::Serialize(static_cast<std::uint64_t>(value), out, n);
  return n;
}

inline bool Decode(std::uint8_t const* in, std::size_t in_len,
                   std::uint64_t& value_out, std::size_t& consumed) {
  std::size_t pos = 0;
  std::uint64_t value = 0;
  detail::Res res = detail::Res::kFinished;
  if (!LegacyAetherPackedSize::Deserialize(in, in_len, pos, value, res)) {
    return false;
  }
  if (res != detail::Res::kFinished) {
    return false;
  }
  value_out = value;
  consumed = pos;
  return true;
}

}  // namespace ae::legacy

#endif  // AE_NUMERIC_TESTS_LEGACY_AETHER_TIERED_CODEC_H_
