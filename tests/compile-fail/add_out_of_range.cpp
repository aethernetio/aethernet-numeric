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

// This translation unit MUST fail to compile: the integer literal 123 is first
// interpreted as a logical F{123}, which exceeds the declared Max of 100.

#include <cstdint>

#include "numeric/fixed_point.h"

using F = ae::FixedPoint<std::uint8_t, 100.0>;

auto bad2 = F{45} + 123;

int main() {
  (void)bad2;
  return 0;
}
