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

#ifndef NUMERIC_RUNTIME_NUMERIC_TRAITS_H_
#define NUMERIC_RUNTIME_NUMERIC_TRAITS_H_

#include <concepts>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace ae {

template <typename T>
struct runtime_numeric_traits {
  static constexpr bool kIsSupported = false;
};

namespace runtime_numeric_internal {

template <std::integral T>
constexpr T SaturateFromInt64(std::int64_t value) {
  if constexpr (std::is_signed_v<T>) {
    if (value > static_cast<std::int64_t>(std::numeric_limits<T>::max())) {
      return std::numeric_limits<T>::max();
    }
    if (value < static_cast<std::int64_t>(std::numeric_limits<T>::min())) {
      return std::numeric_limits<T>::min();
    }
    return static_cast<T>(value);
  } else {
    if (value <= 0) {
      return T{0};
    }
    if (value > static_cast<std::int64_t>(std::numeric_limits<T>::max())) {
      return std::numeric_limits<T>::max();
    }
    return static_cast<T>(value);
  }
}

template <std::integral T>
consteval T RoundSaturationFromDouble(double value) {
  if (value == 0.0) {
    return T{0};
  }

  if constexpr (std::is_signed_v<T>) {
    const double max_v = static_cast<double>(std::numeric_limits<T>::max());
    const double min_v = static_cast<double>(std::numeric_limits<T>::min());
    if (value >= max_v) {
      return std::numeric_limits<T>::max();
    }
    if (value <= min_v) {
      return std::numeric_limits<T>::min();
    }
  } else {
    if (value <= 0.0) {
      return T{0};
    }
    const double max_v = static_cast<double>(std::numeric_limits<T>::max());
    if (value >= max_v) {
      return std::numeric_limits<T>::max();
    }
  }

  const double rounded = (value >= 0.0) ? (value + 0.5) : (value - 0.5);
  return static_cast<T>(rounded);
}

}  // namespace runtime_numeric_internal

template <std::integral T>
  requires(!std::same_as<T, bool>)
struct runtime_numeric_traits<T> {
  using value_type = T;

  static constexpr bool kIsSupported = true;
  static constexpr bool kIsSigned = std::is_signed_v<T>;

  static constexpr T FromInteger(std::int64_t value) {
    return runtime_numeric_internal::SaturateFromInt64<T>(value);
  }

  static consteval T FromDouble(double value) {
    return runtime_numeric_internal::RoundSaturationFromDouble<T>(value);
  }
};

}  // namespace ae

#endif  // NUMERIC_RUNTIME_NUMERIC_TRAITS_H_
