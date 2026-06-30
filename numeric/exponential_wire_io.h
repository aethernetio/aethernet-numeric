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

#ifndef NUMERIC_EXPONENTIAL_WIRE_IO_H_
#define NUMERIC_EXPONENTIAL_WIRE_IO_H_

#include "numeric/exponential.h"
#include "numeric/wire_io.h"

namespace ae {

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode>
struct wire_traits<Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                               BoundaryCode>> {
  using T = Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                        BoundaryCode>;
  using WireTraits = wire_traits<WireT>;

  static constexpr std::size_t kMaxWireBytes = WireTraits::kMaxWireBytes;

  static std::size_t Serialize(const T& value, std::uint8_t* out) {
    return WireTraits::Serialize(value.WireCode(), out);
  }

  static DeserializeResult<T> Deserialize(const std::uint8_t* in,
                                          std::size_t len) {
    const auto wire_result = WireTraits::Deserialize(in, len);
    return {T::FromCode(wire_result.value), wire_result.BytesRead};
  }
};

}  // namespace ae

#endif  // NUMERIC_EXPONENTIAL_WIRE_IO_H_
