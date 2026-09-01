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

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_RANK_WIRE_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_RANK_WIRE_H_

// Purpose: Map dense packed ranks to SegmentedNumber wire values.
//
// Motivation: wire types include plain unsigned integers and TieredInt
// rank wrappers. Brace initialization from runtime uint32_t is narrowing
// for smaller integral wire types, so checked static_cast is required.
//
// How it works: RankToWire and WireToRank live in ae::seg::detail and
// reject ranks that do not fit the wire domain at compile time or with
// a debug assert at runtime.

#include <cassert>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "ae-numeric/tiered_int.h"

namespace ae::seg::detail {

template <typename T>
inline constexpr bool kIsUnsignedIntegralWire =
    std::is_integral_v<T> && std::is_unsigned_v<T> &&
    !std::is_same_v<T, bool>;

template <typename T>
struct IsTieredIntWire : std::false_type {};

template <typename WireCell, auto... TierMaxVals>
struct IsTieredIntWire<TieredInt<WireCell, TierMaxVals...>>
    : std::true_type {};

template <typename Wire>
inline constexpr bool kIsTieredIntWire =
    IsTieredIntWire<std::remove_cvref_t<Wire>>::value;

template <typename Wire>
inline constexpr bool kIsRankWireType =
    kIsUnsignedIntegralWire<Wire> || kIsTieredIntWire<Wire>;

template <typename Wire>
  requires kIsRankWireType<Wire>
constexpr bool RankFitsWire(std::uint32_t rank) noexcept {
  if constexpr (kIsUnsignedIntegralWire<Wire>) {
    return rank <= static_cast<std::uint32_t>(
                     std::numeric_limits<Wire>::max());
  } else {
    using ValueType = typename Wire::ValueType;
    return rank <= static_cast<std::uint32_t>(
                     std::numeric_limits<ValueType>::max());
  }
}

template <typename Wire>
  requires kIsRankWireType<Wire>
constexpr Wire RankToWire(std::uint32_t rank) {
  assert(RankFitsWire<Wire>(rank));
  if constexpr (kIsUnsignedIntegralWire<Wire>) {
    return static_cast<Wire>(rank);
  } else {
    return Wire{static_cast<typename Wire::ValueType>(rank)};
  }
}

template <typename Wire, std::uint32_t Rank>
  requires kIsRankWireType<Wire>
consteval Wire RankToWireConst() {
  static_assert(RankFitsWire<Wire>(Rank), "rank does not fit wire type");
  if constexpr (kIsUnsignedIntegralWire<Wire>) {
    return static_cast<Wire>(Rank);
  } else {
    return Wire{static_cast<typename Wire::ValueType>(Rank)};
  }
}

template <typename Wire>
  requires kIsRankWireType<Wire>
constexpr std::uint32_t WireToRank(Wire const& w) {
  if constexpr (kIsUnsignedIntegralWire<Wire>) {
    return static_cast<std::uint32_t>(w);
  } else {
    return static_cast<std::uint32_t>(
        static_cast<typename Wire::ValueType>(w));
  }
}

}  // namespace ae::seg::detail

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_RANK_WIRE_H_
