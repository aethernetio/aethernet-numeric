/*
 * Copyright 2025 Aethernet Inc.
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

// This translation unit MUST fail to compile: the logical value 100 exceeds the
// declared BoundaryMagnitude of 60, and the ordinary (consteval) constructor
// rejects out-of-range constants.

#include <cstdint>

#include "numeric/exponential.h"
#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"

using Runtime = ae::FixedPoint<std::uint32_t, 60.0>;
using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;
using E = ae::Exponential<Runtime, Wire, 0.001, 60.0, 1529>;

E bad{100};

int main() {
  (void)bad;
  return 0;
}
