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

#ifndef AE_NUMERIC_TESTS_SEGMENTED_REFERENCE_MATH_H_
#define AE_NUMERIC_TESTS_SEGMENTED_REFERENCE_MATH_H_

#include <cmath>
#include <cstdint>

namespace ae::test_segmented_reference {

inline long double RefExpRatio(long double begin, long double end, int n) {
  if (begin <= 0.0L || end <= 0.0L || n <= 0) {
    return 1.0L;
  }
  return std::pow(end / begin, 1.0L / static_cast<long double>(n));
}

inline long double RefGeomSum(long double q, int n) {
  if (n <= 0) {
    return 0.0L;
  }
  if (std::fabsl(q - 1.0L) < 1.0e-18L) {
    return static_cast<long double>(n);
  }
  return (std::pow(q, static_cast<long double>(n)) - 1.0L) / (q - 1.0L);
}

inline long double RefExpValue(long double begin, long double end, int n,
                               int i) {
  if (i <= 0) {
    return begin;
  }
  if (i >= n) {
    return end;
  }
  long double const r = RefExpRatio(begin, end, n);
  return begin * std::pow(r, static_cast<long double>(i));
}

inline long double RefGeomValue(long double begin, long double end,
                                 long double step0, long double q, int n,
                                 int i, bool from_upper, long double last_step) {
  if (i <= 0) {
    return begin;
  }
  if (i >= n) {
    return end;
  }
  if (from_upper) {
    return end - last_step * RefGeomSum(q, n - i);
  }
  return begin + step0 * RefGeomSum(q, i);
}

inline long double RefLinearValue(long double begin, long double step0,
                                  long double delta, int n, int i) {
  if (i <= 0) {
    return begin;
  }
  if (i >= n) {
    return begin + static_cast<long double>(n) * step0 +
           delta * static_cast<long double>(n) *
               static_cast<long double>(n - 1) / 2.0L;
  }
  return begin + static_cast<long double>(i) * step0 +
         delta * static_cast<long double>(i) *
             static_cast<long double>(i - 1) / 2.0L;
}

inline long double RefRelQuantError(long double r) {
  return std::sqrt(r) - 1.0L;
}

inline long double RefSolveGeomQ(int n, long double sum) {
  if (n <= 0 || sum <= static_cast<long double>(n)) {
    return 1.0L;
  }
  long double lo = 1.0L;
  long double hi = 1.5L;
  while (RefGeomSum(hi, n) < sum && hi < 8.0L) {
    hi *= 2.0L;
  }
  for (int i = 0; i < 80; ++i) {
    long double const mid = 0.5L * (lo + hi);
    if (RefGeomSum(mid, n) < sum) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5L * (lo + hi);
}

inline int RefAutoSplitN1(long double begin, long double mid, long double end,
                            int total) {
  int best_n1 = 1;
  long double best_jump = 1.0e300L;
  long double best_err = 1.0e300L;
  for (int n1 = 1; n1 < total; ++n1) {
    int const n2 = total - n1;
    long double const r1 = RefExpRatio(begin, mid, n1);
    long double const r2 = RefExpRatio(mid, end, n2);
    long double const step_before = mid - mid / r1;
    long double const step_after = mid * r2 - mid;
    long double const smaller =
        step_before < step_after ? step_before : step_after;
    if (smaller <= 0.0L) {
      continue;
    }
    long double const jump =
        std::fabsl(step_after - step_before) / smaller;
    long double const err1 = RefRelQuantError(r1);
    long double const err2 = RefRelQuantError(r2);
    long double const err = err1 > err2 ? err1 : err2;
    bool const better = jump < best_jump ||
                        (jump == best_jump && err < best_err) ||
                        (jump == best_jump && err == best_err && n1 < best_n1);
    if (better) {
      best_jump = jump;
      best_err = err;
      best_n1 = n1;
    }
  }
  return best_n1;
}

}  // namespace ae::test_segmented_reference

#endif  // AE_NUMERIC_TESTS_SEGMENTED_REFERENCE_MATH_H_
