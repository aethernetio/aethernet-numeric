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

// Must fail to compile: constexpr addition exceeds kUpper.

#include <cstdint>

#include <ae-numeric/tiered_int.h>

using T = ae::TieredInt<std::uint8_t, 250, 1514, 1049834>;

constexpr auto kOverflow = T{T::kUpper} + typename T::ValueType{1};

int main() {
  (void)kOverflow;
  return 0;
}
