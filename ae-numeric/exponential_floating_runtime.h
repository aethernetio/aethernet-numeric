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

#ifndef AE_NUMERIC_EXPONENTIAL_FLOATING_RUNTIME_H_
#define AE_NUMERIC_EXPONENTIAL_FLOATING_RUNTIME_H_

#include "ae-numeric/runtime_numeric_traits.h"

namespace ae {

template <>
struct runtime_numeric_traits<float> {
  using value_type = float;

  static constexpr bool kIsSupported = true;
  static constexpr bool kIsSigned = true;

  static constexpr float FromInteger(std::int64_t value) {
    return static_cast<float>(value);
  }

  static consteval float FromDouble(double value) {
    return static_cast<float>(value);
  }
};

template <>
struct runtime_numeric_traits<double> {
  using value_type = double;

  static constexpr bool kIsSupported = true;
  static constexpr bool kIsSigned = true;

  static constexpr double FromInteger(std::int64_t value) {
    return static_cast<double>(value);
  }

  static consteval double FromDouble(double value) {
    return value;
  }
};

}  // namespace ae

#endif  // AE_NUMERIC_EXPONENTIAL_FLOATING_RUNTIME_H_
