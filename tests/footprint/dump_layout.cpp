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

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <stddef.h>

#include <ae-numeric/details/segmented_compiler.h>

#include "../segmented_test_formats.h"

using CS = ae::seg::segmented_compiler_internal::CompiledSegment;

#define AE_FP_FIELD(name)                                                 \
  std::printf("  %-22s  sizeof=%zu  offset=%zu\n", #name, sizeof(CS::name), \
              offsetof(CS, name))

int main() {
  std::printf("sizeof(CompiledSegment)=%zu alignof=%zu\n", sizeof(CS),
              alignof(CS));
  AE_FP_FIELD(physical_begin_raw);
  AE_FP_FIELD(physical_end_raw);
  AE_FP_FIELD(wire_code_begin);
  AE_FP_FIELD(code_count);
  AE_FP_FIELD(curve_kind);
  AE_FP_FIELD(wire_bytes);
  AE_FP_FIELD(intervals);
  AE_FP_FIELD(math_first);
  AE_FP_FIELD(curve_begin_raw);
  AE_FP_FIELD(curve_end_raw);
  AE_FP_FIELD(step0_raw);
  AE_FP_FIELD(last_step_raw);
  AE_FP_FIELD(delta_raw);
  AE_FP_FIELD(log2_r);
  AE_FP_FIELD(log2_q);
  AE_FP_FIELD(log2_begin);
  AE_FP_FIELD(log2_end);
  AE_FP_FIELD(from_upper);

  using namespace ae::test_segmented_formats;
  std::printf("\n");
  std::printf("Rssi sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(Rssi::kSegments), Rssi::kSegmentCount,
              Rssi::kFormulaCoefficientBytes);
  std::printf("Temperature sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(Temperature::kSegments), Temperature::kSegmentCount,
              Temperature::kFormulaCoefficientBytes);
  std::printf("Humidity sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(Humidity::kSegments), Humidity::kSegmentCount,
              Humidity::kFormulaCoefficientBytes);
  std::printf("Co2 sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(Co2::kSegments), Co2::kSegmentCount,
              Co2::kFormulaCoefficientBytes);
  std::printf("RxWindow sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(RxWindow::kSegments), RxWindow::kSegmentCount,
              RxWindow::kFormulaCoefficientBytes);
  std::printf("Battery sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(Battery::kSegments), Battery::kSegmentCount,
              Battery::kFormulaCoefficientBytes);
  std::printf("Connect sizeof(kSegments)=%zu count=%zu formula_bytes=%zu\n",
              sizeof(ConnectDuration::kSegments),
              ConnectDuration::kSegmentCount,
              ConnectDuration::kFormulaCoefficientBytes);
  return 0;
}
