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

#ifndef AE_NUMERIC_SEGMENTED_NUMBER_FLOATING_RUNTIME_H_
#define AE_NUMERIC_SEGMENTED_NUMBER_FLOATING_RUNTIME_H_

#include "ae-numeric/exponential.h"
#include "ae-numeric/exponential_floating_runtime.h"
#include "ae-numeric/segmented_number.h"

#include <type_traits>

namespace ae::seg {

template <>
inline constexpr bool kFloatingRuntimeEnabled<float> = true;
template <>
inline constexpr bool kFloatingRuntimeEnabled<double> = true;

namespace segmented_number_internal {

template <typename Logical, typename FloatingT>
constexpr typename Logical::rep_value_type FloatingToLogicalRaw(
    FloatingT value) {
  if (value == FloatingT{0}) {
    return static_cast<typename Logical::rep_value_type>(0);
  }
  bool const neg = value < FloatingT{0};
  FloatingT const mag = neg ? -value : value;
  Logical const work =
      exponential_internal::FloatingToWork<Logical, FloatingT>(mag);
  auto const raw = work.RawValue();
  if constexpr (std::is_signed_v<typename Logical::rep_value_type>) {
    return neg ? static_cast<typename Logical::rep_value_type>(-raw) : raw;
  } else {
    (void)neg;
    return raw;
  }
}

template <typename Logical, typename FloatingT>
constexpr FloatingT LogicalRawToFloating(typename Logical::rep_value_type raw) {
  bool const neg =
      std::is_signed_v<typename Logical::rep_value_type> && raw < 0;
  auto ar = neg ? static_cast<typename Logical::rep_value_type>(-raw) : raw;
  if (ar > Logical::kRawMax) {
    ar = Logical::kRawMax;
  }
  Logical const work = Logical::FromRaw(
      fixed_point_internal::RepFromRawValue<typename Logical::rep_type>(ar));
  FloatingT const mag =
      exponential_internal::WorkToFloating<FloatingT, Logical>(work);
  return neg ? -mag : mag;
}

template <typename Logical>
struct RuntimeRawMap<Logical, float> {
  static constexpr typename Logical::rep_value_type ToRaw(float v) {
    return FloatingToLogicalRaw<Logical, float>(v);
  }
  static constexpr float FromRaw(typename Logical::rep_value_type raw) {
    return LogicalRawToFloating<Logical, float>(raw);
  }
};

template <typename Logical>
struct RuntimeRawMap<Logical, double> {
  static constexpr typename Logical::rep_value_type ToRaw(double v) {
    return FloatingToLogicalRaw<Logical, double>(v);
  }
  static constexpr double FromRaw(typename Logical::rep_value_type raw) {
    return LogicalRawToFloating<Logical, double>(raw);
  }
};

}  // namespace segmented_number_internal
}  // namespace ae::seg

#endif  // AE_NUMERIC_SEGMENTED_NUMBER_FLOATING_RUNTIME_H_
