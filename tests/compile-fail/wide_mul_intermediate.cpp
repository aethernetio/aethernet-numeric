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

#include <ae-numeric/exponential_math_policy.h>

#include <cstdint>

// Wider than 32-bit Raw must fail: no 128-bit intermediate and no silent
// 64x64 overflow path.
using Bad = ae::exponential_internal::MulIntermediate<std::uint64_t>::type;

int main() {
  return static_cast<int>(sizeof(Bad));
}
