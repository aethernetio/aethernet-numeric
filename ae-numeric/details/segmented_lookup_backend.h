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

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_LOOKUP_BACKEND_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_LOOKUP_BACKEND_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "ae-numeric/details/segmented_compiler.h"
#include "ae-numeric/details/segmented_formula_backend.h"
#include "ae-numeric/integer_math.h"

namespace ae::seg::segmented_lookup_internal {

template <typename Spec, typename RT, std::size_t N>
struct LookupTables {
  std::array<std::int64_t, N> decoded{};
  std::array<std::uint32_t, N> order{};
};

template <typename Spec, typename RT, std::size_t N>
consteval LookupTables<Spec, RT, N> MakeLookupTables() {
  using segmented_compiler_internal::MakeCompiledSegments;
  using segmented_compiler_internal::PlanHolder;
  LookupTables<Spec, RT, N> t{};
  constexpr auto kSegs = MakeCompiledSegments<Spec, RT>();
  constexpr int kNs = PlanHolder<Spec>::kPlan.count;
  for (std::size_t i = 0; i < N; ++i) {
    t.decoded[i] = segmented_formula_internal::DecodeRankRaw<RT>(
        kSegs.data(), kNs, static_cast<std::uint32_t>(i));
    t.order[i] = static_cast<std::uint32_t>(i);
  }
  for (std::size_t i = 1; i < N; ++i) {
    std::uint32_t const key = t.order[i];
    std::int64_t const keyv = t.decoded[key];
    std::size_t j = i;
    while (j > 0 && t.decoded[t.order[j - 1]] > keyv) {
      t.order[j] = t.order[j - 1];
      --j;
    }
    t.order[j] = key;
  }
  return t;
}

template <typename Spec, typename RT, std::size_t N>
constexpr std::uint32_t LookupEncode(LookupTables<Spec, RT, N> const& t,
                                     std::int64_t raw) {
  if (N == 0) {
    return 0;
  }
  std::size_t lo = 0;
  std::size_t hi = N;
  while (lo < hi) {
    std::size_t const mid = lo + (hi - lo) / 2U;
    if (t.decoded[t.order[mid]] < raw) {
      lo = mid + 1U;
    } else {
      hi = mid;
    }
  }
  std::uint32_t best = t.order[lo < N ? lo : N - 1U];
  std::uint64_t best_d = integer_math::AbsI64ToU64(t.decoded[best] - raw);
  auto consider = [&](std::size_t idx) {
    if (idx >= N) {
      return;
    }
    std::uint32_t const rank = t.order[idx];
    std::uint64_t const d =
        integer_math::AbsI64ToU64(t.decoded[rank] - raw);
    if (d < best_d || (d == best_d && rank < best)) {
      best_d = d;
      best = rank;
    }
  };
  if (lo > 0) {
    consider(lo - 1U);
  }
  consider(lo);
  if (lo + 1U < N) {
    consider(lo + 1U);
  }
  return best;
}

}  // namespace ae::seg::segmented_lookup_internal

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_LOOKUP_BACKEND_H_
