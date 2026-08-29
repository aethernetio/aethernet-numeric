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

#include <ae-numeric/cyclic_counter.h>

using Counter = ae::CyclicCounter<std::uint16_t, std::uint32_t>;

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestRestore(
    std::uint32_t base, std::uint32_t wire) {
  volatile std::uint32_t b = base;
  volatile std::uint32_t w = wire;
  Counter c{b};
  auto const r = c.TryRestore(static_cast<std::uint16_t>(w));
  return r.has_value() ? *r : 0xFFFFFFFFu;
}

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestAdvance(
    std::uint32_t base, std::uint32_t wire) {
  volatile std::uint32_t b = base;
  volatile std::uint32_t w = wire;
  Counter c{b};
  auto const r = c.TryAdvance(static_cast<std::uint16_t>(w));
  return r.has_value() ? *r : 0xFFFFFFFFu;
}
