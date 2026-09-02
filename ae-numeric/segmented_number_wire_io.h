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

#ifndef AE_NUMERIC_SEGMENTED_NUMBER_WIRE_IO_H_
#define AE_NUMERIC_SEGMENTED_NUMBER_WIRE_IO_H_

// Purpose: Connect SegmentedNumber to the generic wire_io interface.
//
// Motivation: callers should use the same Serialize/Deserialize API for
// built-in integers, TieredInt, FixedPoint, and SegmentedNumber.
//
// Analogous concept: a serialization traits adapter.
//
// Usage: include this header after segmented_number.h and call the
// generic ae::Serialize, ae::Deserialize, and ae::MaxWireBytes APIs.
//
// How it works: SegmentedNumber computes a canonical rank, then its
// compiled wire_type serializes that rank. Deserialization rejects
// truncated encodings and unused ranks. The wire ABI is defined by the
// schema, not by the selected runtime representation.


#include <cassert>

#include "ae-numeric/segmented_number.h"
#include "ae-numeric/wire_io.h"

namespace ae {

template <typename Spec>
struct wire_traits<seg::SegmentedNumber<Spec>> {
  using T = seg::SegmentedNumber<Spec>;
  using WireTraits = wire_traits<typename T::wire_type>;

  static constexpr std::size_t kMaxWireBytes = T::kMaxWireBytes;

  static std::size_t Serialize(T const& value, std::uint8_t* out) {
    assert(out != nullptr);
    return T::Serialize(value, out);
  }

  static DeserializeResult<T> Deserialize(std::uint8_t const* in,
                                          std::size_t len) {
    return T::Deserialize(in, len);
  }
};

}  // namespace ae

#endif  // AE_NUMERIC_SEGMENTED_NUMBER_WIRE_IO_H_
