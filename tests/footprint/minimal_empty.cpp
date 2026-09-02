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

#include <cstdint>

#include <ae-numeric/fixed_point.h>

using RT = ae::FixedPoint<std::int16_t, 125>;

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestEncode(
    std::uint32_t raw) {
  volatile std::uint32_t in = raw;
  auto const v = RT::FromRaw(
      RT::ClampRaw(static_cast<typename RT::rep_value_type>(
          static_cast<std::int64_t>(in))));
  return static_cast<std::uint32_t>(v.RawValue());
}

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestDecode(
    std::uint32_t code) {
  volatile std::uint32_t in = code;
  auto const v = RT::FromRaw(
      RT::ClampRaw(static_cast<typename RT::rep_value_type>(
          static_cast<std::int64_t>(in))));
  return static_cast<std::uint32_t>(v.RawValue());
}
