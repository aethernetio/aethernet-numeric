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

#ifndef AE_NUMERIC_INTEGER_MATH_H_
#define AE_NUMERIC_INTEGER_MATH_H_

#include <cstdint>
#include <limits>

// GCC gcov cannot instrument constexpr function bodies in headers. Coverage
// builds may define AE_COVERAGE_BUILD to keep the same logic as ordinary
// functions for measurement only; production builds stay constexpr.
#if defined(AE_COVERAGE_BUILD)
#define AE_INTEGER_MATH_CONSTEXPR
#else
#define AE_INTEGER_MATH_CONSTEXPR constexpr
#endif

namespace ae::integer_math {

// Absolute value for int64 without signed negation UB at INT64_MIN.
AE_INTEGER_MATH_CONSTEXPR std::uint64_t AbsI64ToU64(std::int64_t value) noexcept {
  if (value >= 0) {
    return static_cast<std::uint64_t>(value);
  }
  return static_cast<std::uint64_t>(-(value + 1)) + 1U;
}

AE_INTEGER_MATH_CONSTEXPR std::uint64_t GcdU64(std::uint64_t a,
                                               std::uint64_t b) noexcept {
  while (b != 0) {
    std::uint64_t const t = a % b;
    a = b;
    b = t;
  }
  return a;
}

AE_INTEGER_MATH_CONSTEXPR bool MulU64Checked(std::uint64_t a, std::uint64_t b,
                                             std::uint64_t& out) noexcept {
  if (a == 0 || b == 0) {
    out = 0;
    return true;
  }
  if (a > std::numeric_limits<std::uint64_t>::max() / b) {
    return false;
  }
  out = a * b;
  return true;
}

AE_INTEGER_MATH_CONSTEXPR bool AddU64Checked(std::uint64_t a, std::uint64_t b,
                                             std::uint64_t& out) noexcept {
  if (a > std::numeric_limits<std::uint64_t>::max() - b) {
    return false;
  }
  out = a + b;
  return true;
}

// round_nearest(a / b) for a,b > 0 as uint64; false if quotient does not fit.
// Half rounds away from zero for positive values (same as (a + b/2) / b when
// that addition does not overflow).
AE_INTEGER_MATH_CONSTEXPR bool RoundDivU64(std::uint64_t a, std::uint64_t b,
                                           std::uint64_t& out) noexcept {
  if (b == 0) {
    return false;
  }
  std::uint64_t const q = a / b;
  std::uint64_t const r = a % b;
  bool const round_up = (r > (b - 1) / 2);
  if (round_up) {
    if (q == std::numeric_limits<std::uint64_t>::max()) {
      return false;
    }
    out = q + 1U;
  } else {
    out = q;
  }
  return true;
}

// round_nearest((a * f) / d) for non-negative a,f,d with uint64-only math.
AE_INTEGER_MATH_CONSTEXPR bool MulDivU64Nearest(std::uint64_t a, std::uint64_t f,
                                                std::uint64_t d,
                                                std::uint64_t& out) noexcept {
  if (d == 0) {
    return false;
  }
  std::uint64_t const q = a / d;
  std::uint64_t const r = a % d;

  std::uint64_t qf = 0;
  if (!MulU64Checked(q, f, qf)) {
    return false;
  }

  std::uint64_t rf_q = 0;
  std::uint64_t rf = 0;
  if (MulU64Checked(r, f, rf)) {
    rf_q = (rf + d / 2U) / d;
  } else {
    // r * f overflows: split f = f_q * d + f_r.
    std::uint64_t const f_q = f / d;
    std::uint64_t const f_r = f % d;
    std::uint64_t term1 = 0;
    if (!MulU64Checked(r, f_q, term1)) {
      return false;
    }
    std::uint64_t term2_num = 0;
    if (!MulU64Checked(r, f_r, term2_num)) {
      return false;
    }
    rf_q = term1 + (term2_num + d / 2U) / d;
  }

  return AddU64Checked(qf, rf_q, out);
}

// round_nearest(a * 2^shift / b) with every intermediate in uint64.
AE_INTEGER_MATH_CONSTEXPR bool RoundMulPow2DivU64(std::uint64_t a,
                                                  std::uint64_t b,
                                                  unsigned shift,
                                                  std::uint64_t& out) noexcept {
  if (b == 0) {
    return false;
  }
  if (a == 0) {
    out = 0;
    return true;
  }

  std::uint64_t g = GcdU64(a, b);
  a /= g;
  b /= g;

  while (shift > 0 && (b & 1U) == 0U) {
    b >>= 1U;
    --shift;
  }

  std::uint64_t q = a / b;
  std::uint64_t rem = a % b;

  for (unsigned i = 0; i < shift; ++i) {
    if (q > (std::numeric_limits<std::uint64_t>::max() / 2U)) {
      return false;
    }
    q *= 2U;

    if (rem <= (std::numeric_limits<std::uint64_t>::max() / 2U)) {
      rem *= 2U;
      if (rem >= b) {
        rem -= b;
        if (q == std::numeric_limits<std::uint64_t>::max()) {
          return false;
        }
        ++q;
      }
    } else {
      // rem >= 2^63 and rem < b, so b > 2^63. Then 2*rem = 2^64 + lo.
      std::uint64_t const lo =
          (rem - (std::uint64_t{1} << 63U)) << 1U;
      std::uint64_t const rem_hi =
          static_cast<std::uint64_t>(0U - b);  // 2^64 - b
      std::uint64_t carry = 1U;                // floor(2^64 / b) for b > 2^63
      std::uint64_t sum = rem_hi + lo;
      if (sum < rem_hi) {
        ++carry;
      }
      carry += sum / b;
      rem = sum % b;
      if (q > std::numeric_limits<std::uint64_t>::max() - carry) {
        return false;
      }
      q += carry;
    }
  }

  if (rem > (b - 1U) / 2U) {
    if (q == std::numeric_limits<std::uint64_t>::max()) {
      return false;
    }
    ++q;
  }
  out = q;
  return true;
}

// round_nearest(a / (b * 2^shift)) with uint64-only intermediates.
AE_INTEGER_MATH_CONSTEXPR bool RoundDivPow2U64(std::uint64_t a, std::uint64_t b,
                                               unsigned shift,
                                               std::uint64_t& out) noexcept {
  if (b == 0) {
    return false;
  }
  if (a == 0) {
    out = 0;
    return true;
  }

  std::uint64_t g = GcdU64(a, b);
  a /= g;
  b /= g;

  while (shift > 0 && (a & 1U) == 0U) {
    a >>= 1U;
    --shift;
  }

  if (shift >= 64U) {
    unsigned const half_shift = shift - 1U;
    if (half_shift >= 64U) {
      out = 0;
      return true;
    }
    if (b > (std::numeric_limits<std::uint64_t>::max() >> half_shift)) {
      out = 0;
      return true;
    }
    std::uint64_t const threshold = b << half_shift;
    out = (a >= threshold) ? 1U : 0U;
    return true;
  }

  if (b > (std::numeric_limits<std::uint64_t>::max() >> shift)) {
    std::uint64_t const a_hi = a >> shift;
    std::uint64_t const a_lo_mask =
        shift == 0 ? 0 : ((std::uint64_t{1} << shift) - 1U);
    std::uint64_t const a_frac = a & a_lo_mask;
    std::uint64_t q = a_hi / b;
    std::uint64_t r = a_hi % b;
    bool round_up = false;
    if (r > b - r) {
      round_up = true;
    } else if (r == b - r) {
      round_up = true;
    } else {
      std::uint64_t const need = b - 2U * r;
      if (need > (std::numeric_limits<std::uint64_t>::max() >> shift)) {
        round_up = false;
      } else {
        round_up = (a_frac << 1U) >= (need << shift);
      }
    }
    if (round_up) {
      if (q == std::numeric_limits<std::uint64_t>::max()) {
        return false;
      }
      ++q;
    }
    out = q;
    return true;
  }

  std::uint64_t const scaled_den = b << shift;
  return RoundDivU64(a, scaled_den, out);
}

}  // namespace ae::integer_math

#undef AE_INTEGER_MATH_CONSTEXPR

#endif  // AE_NUMERIC_INTEGER_MATH_H_
