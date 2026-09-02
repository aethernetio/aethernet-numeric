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

#include "ae_fp_common.h"

#include <type_traits>

#include <ae-numeric/segmented_number.h>

#include "../segmented_test_formats.h"

namespace {

template <typename Wire>
AE_FP_NOINLINE std::uint32_t WireToU32(Wire const& w) {
  if constexpr (std::is_same_v<Wire, std::uint8_t>) {
    return static_cast<std::uint32_t>(w);
  } else {
    return static_cast<std::uint32_t>(static_cast<typename Wire::ValueType>(w));
  }
}

template <typename Num>
AE_FP_NOINLINE std::uint32_t EncodeOne(std::uint32_t raw) {
  using RT = typename Num::runtime_type;
  volatile std::uint32_t in = raw;
  auto const v = RT::FromRaw(
      RT::ClampRaw(static_cast<typename RT::rep_value_type>(
          static_cast<std::int64_t>(in))));
  auto const w = Num::TryEncode(v);
  if (!w.has_value()) {
    return 0xFFFFFFFFu;
  }
  return WireToU32<typename Num::wire_type>(*w);
}

}  // namespace

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestEncodeAll(
    std::uint32_t raw) {
  std::uint32_t s = 0;
  s ^= EncodeOne<ae::test_segmented_formats::Rssi>(raw);
  s ^= EncodeOne<ae::test_segmented_formats::Temperature>(raw);
  s ^= EncodeOne<ae::test_segmented_formats::Humidity>(raw);
  s ^= EncodeOne<ae::test_segmented_formats::Co2>(raw);
  s ^= EncodeOne<ae::test_segmented_formats::RxWindow>(raw);
  s ^= EncodeOne<ae::test_segmented_formats::Battery>(raw);
  s ^= EncodeOne<ae::test_segmented_formats::ConnectDuration>(raw);
  return s;
}
