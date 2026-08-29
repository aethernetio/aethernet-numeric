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

// Must fail to compile: CyclicCounter has no stateless wire Deserialize.
// A truncated wire value cannot reconstruct epoch/high bits without a live
// full counter base.

#include <cstdint>

#include <ae-numeric/cyclic_counter.h>
#include <ae-numeric/wire_io.h>

using Counter = ae::CyclicCounter<std::uint8_t, std::uint32_t>;

int main() {
  std::uint8_t bytes[1] = {237u};
  auto const restored = ae::Deserialize<Counter>(bytes, sizeof(bytes));
  (void)restored;
  return 0;
}
