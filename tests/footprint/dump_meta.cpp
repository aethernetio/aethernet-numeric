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

#include <cstdint>
#include <cstdio>
#include <type_traits>

#include <ae-numeric/cyclic_counter.h>
#include <ae-numeric/details/segmented_compiler.h>
#include <ae-numeric/segmented_number.h>
#include <ae-numeric/wire_io.h>

#include "../segmented_test_formats.h"

namespace {

template <typename Num>
void PrintSeg(char const* name) {
  using RT = typename Num::runtime_type;
  using WT = typename Num::wire_type;
  std::printf(
      "SEG %s code_count=%zu segments=%zu sizeof_runtime=%zu sizeof_number=%zu "
      "sizeof_wire=%zu max_wire_bytes=%zu formula_coeff_bytes=%zu\n",
      name, Num::kCodeCount, Num::kSegmentCount, sizeof(RT), sizeof(Num),
      sizeof(WT), Num::kMaxWireBytes, Num::kFormulaCoefficientBytes);
}

template <typename Wire, typename Value>
void PrintCyclic(char const* name) {
  using C = ae::CyclicCounter<Wire, Value>;
  std::printf(
      "CYC %s sizeof=%zu wire_bytes=%zu value_bytes=%zu half_range=%u "
      "wire_space=%u\n",
      name, sizeof(C), sizeof(Wire), sizeof(Value),
      static_cast<unsigned>(C::kHalfRange),
      static_cast<unsigned>(C::kWireSpace));
}

}  // namespace

int main() {
  using CS = ae::seg::segmented_compiler_internal::CompiledSegment;
  std::printf("META sizeof_CompiledSegment=%zu alignof_CompiledSegment=%zu\n",
              sizeof(CS), alignof(CS));

  using namespace ae::test_segmented_formats;
  PrintSeg<Rssi>("Rssi");
  PrintSeg<Temperature>("Temperature");
  PrintSeg<Humidity>("Humidity");
  PrintSeg<Co2>("Co2");
  PrintSeg<RxWindow>("RxWindow");
  PrintSeg<Battery>("Battery");
  PrintSeg<ConnectDuration>("ConnectDuration");

  PrintCyclic<std::uint8_t, std::uint16_t>("u8_u16");
  PrintCyclic<std::uint8_t, std::uint32_t>("u8_u32");
  PrintCyclic<std::uint16_t, std::uint32_t>("u16_u32");
  return 0;
}
