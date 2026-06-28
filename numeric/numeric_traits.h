#pragma once

#include <concepts>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace ae {

template <typename T>
inline constexpr bool kIntegralStorageV = false;

template <typename T>
concept IntegralStorage = kIntegralStorageV<T>;

template <typename T>
struct numeric_traits;

template <std::integral T>
  requires(!std::same_as<T, bool>)
struct numeric_traits<T> {
  using rep_type = T;
  using rep_value_type = T;
  using value_type = T;

  static constexpr bool kIsIntegerLike = true;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = false;
  static constexpr bool kIsSigned = std::is_signed_v<T>;

  static constexpr rep_value_type kRawMax =
      kIsSigned ? static_cast<rep_value_type>(std::numeric_limits<T>::max())
                : std::numeric_limits<T>::max();
  static constexpr rep_value_type kRawMin =
      kIsSigned ? static_cast<rep_value_type>(-kRawMax)
                : rep_value_type{0};
};

template <std::integral T>
  requires(!std::same_as<T, bool>)
inline constexpr bool kIntegralStorageV<T> = true;

}  // namespace ae

#include "numeric/tiered_int.h"

namespace ae {

template <typename WireCell, std::uint32_t... TierMaxVals>
struct numeric_traits<TieredInt<WireCell, TierMaxVals...>> {
  using Rep = TieredInt<WireCell, TierMaxVals...>;
  using rep_type = Rep;
  using rep_value_type = typename Rep::ValueType;
  using value_type = Rep;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = false;
  static constexpr bool kIsSigned = Rep::kIsSigned;

  static constexpr rep_value_type kRawMax = Rep::kUpper;
  static constexpr rep_value_type kRawMin = Rep::kLower;
};

template <typename WireCell, std::uint32_t... TierMaxVals>
inline constexpr bool kIntegralStorageV<TieredInt<WireCell, TierMaxVals...>> =
    true;

}  // namespace ae
