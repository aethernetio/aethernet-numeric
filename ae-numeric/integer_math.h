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

// Purpose: Provide checked low-level integer primitives shared by
// FixedPoint and SegmentedNumber numerical code.
//
// Motivation: 32-bit MCUs need wide products, division, rounding, and
// square roots without undefined behavior or unnecessary compiler
// runtime helpers.
//
// Analogous concepts: multiword arithmetic and integer numerical kernels.
//
// Usage: this is a low-level support header; public numeric types call it
// indirectly. Some generic or legacy helpers support 64-bit values, while
// the SegmentedNumber production path selects the explicit 32-bit-safe
// primitives.
//
// How it works: operations use checked bounds, hi/lo word products,
// explicit rounding, and integer root algorithms. Callers select the
// primitive whose documented input range matches their numeric domain.


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
AE_INTEGER_MATH_CONSTEXPR std::uint64_t AbsI64ToU64(
    std::int64_t value) noexcept {
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
AE_INTEGER_MATH_CONSTEXPR bool MulDivU64Nearest(
    std::uint64_t a, std::uint64_t f, std::uint64_t d,
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

// Floor square root. Newton iteration; no overflow of x*x in the check because
// `x > n / x` is used instead of `x * x > n`.
AE_INTEGER_MATH_CONSTEXPR std::uint64_t SqrtU64(std::uint64_t n) noexcept {
  if (n < 2U) {
    return n;
  }
  std::uint64_t x0 = n >> 1U;
  std::uint64_t x1 = (x0 + n / x0) >> 1U;
  while (x1 < x0) {
    x0 = x1;
    x1 = (x0 + n / x0) >> 1U;
  }
  return x0;
}

AE_INTEGER_MATH_CONSTEXPR std::uint32_t AbsI32ToU32(
    std::int32_t value) noexcept {
  if (value >= 0) {
    return static_cast<std::uint32_t>(value);
  }
  return static_cast<std::uint32_t>(-(value + 1)) + 1U;
}

AE_INTEGER_MATH_CONSTEXPR bool MulU32Checked(std::uint32_t a, std::uint32_t b,
                                             std::uint32_t& out) noexcept {
  if (a == 0 || b == 0) {
    out = 0;
    return true;
  }
  if (a > std::numeric_limits<std::uint32_t>::max() / b) {
    return false;
  }
  out = a * b;
  return true;
}

AE_INTEGER_MATH_CONSTEXPR bool AddU32Checked(std::uint32_t a, std::uint32_t b,
                                             std::uint32_t& out) noexcept {
  if (a > std::numeric_limits<std::uint32_t>::max() - b) {
    return false;
  }
  out = a + b;
  return true;
}

// 32×32 → (hi, lo) via 16×16 products. No 64-bit arithmetic type.
AE_INTEGER_MATH_CONSTEXPR void MulU32Wide(std::uint32_t a, std::uint32_t b,
                                          std::uint32_t& hi,
                                          std::uint32_t& lo) noexcept {
  std::uint32_t const a0 = a & 0xFFFFu;
  std::uint32_t const a1 = a >> 16U;
  std::uint32_t const b0 = b & 0xFFFFu;
  std::uint32_t const b1 = b >> 16U;
  std::uint32_t const p00 = a0 * b0;
  std::uint32_t const p01 = a0 * b1;
  std::uint32_t const p10 = a1 * b0;
  std::uint32_t const p11 = a1 * b1;
  lo = p00;
  hi = p11;
  std::uint32_t const p01_lo = p01 << 16U;
  std::uint32_t const p01_hi = p01 >> 16U;
  std::uint32_t s = lo + p01_lo;
  std::uint32_t c = s < lo ? 1U : 0U;
  lo = s;
  hi += p01_hi + c;
  std::uint32_t const p10_lo = p10 << 16U;
  std::uint32_t const p10_hi = p10 >> 16U;
  s = lo + p10_lo;
  c = s < lo ? 1U : 0U;
  lo = s;
  hi += p10_hi + c;
}

// (hi:lo) / d → 32-bit quotient. Requires d != 0 and hi < d.
AE_INTEGER_MATH_CONSTEXPR bool DivU32Wide(std::uint32_t hi, std::uint32_t lo,
                                          std::uint32_t d, std::uint32_t& quot,
                                          std::uint32_t& rem) noexcept {
  if (d == 0 || hi >= d) {
    return false;
  }
  std::uint32_t q = 0;
  for (int i = 0; i < 32; ++i) {
    std::uint32_t const hi_msb = hi >> 31U;
    hi = (hi << 1U) | (lo >> 31U);
    lo <<= 1U;
    q <<= 1U;
    if (hi_msb != 0U || hi >= d) {
      hi -= d;
      q |= 1U;
    }
  }
  quot = q;
  rem = hi;
  return true;
}

AE_INTEGER_MATH_CONSTEXPR bool RoundDivU32(std::uint32_t a, std::uint32_t b,
                                           std::uint32_t& out) noexcept {
  if (b == 0) {
    return false;
  }
  std::uint32_t const q = a / b;
  std::uint32_t const r = a % b;
  if (r >= b - r) {
    if (q == std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
    out = q + 1U;
  } else {
    out = q;
  }
  return true;
}

// round_nearest((a * f) / d) with 32-bit operations only.
AE_INTEGER_MATH_CONSTEXPR bool MulDivU32Nearest(
    std::uint32_t a, std::uint32_t f, std::uint32_t d,
    std::uint32_t& out) noexcept {
  if (d == 0) {
    return false;
  }
  std::uint32_t hi = 0;
  std::uint32_t lo = 0;
  MulU32Wide(a, f, hi, lo);
  std::uint32_t q = 0;
  std::uint32_t r = 0;
  if (!DivU32Wide(hi, lo, d, q, r)) {
    return false;
  }
  if (r >= d - r) {
    if (q == std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
    ++q;
  }
  out = q;
  return true;
}

AE_INTEGER_MATH_CONSTEXPR std::uint32_t SqrtU32(std::uint32_t n) noexcept {
  if (n < 2U) {
    return n;
  }
  std::uint32_t x0 = n >> 1U;
  std::uint32_t x1 = (x0 + n / x0) >> 1U;
  while (x1 < x0) {
    x0 = x1;
    x1 = (x0 + n / x0) >> 1U;
  }
  return x0;
}

// Floor sqrt of a 64-bit magnitude stored as (hi, lo). 32-bit operations only:
// digit-by-digit search of the 32-bit root, comparing squares via MulU32Wide.
AE_INTEGER_MATH_CONSTEXPR std::uint32_t SqrtU32Wide(std::uint32_t hi,
                                                    std::uint32_t lo) noexcept {
  if (hi == 0U) {
    return SqrtU32(lo);
  }
  std::uint32_t res = 0;
  for (unsigned i = 0; i < 32U; ++i) {
    std::uint32_t const cand = res | (1U << (31U - i));
    std::uint32_t phi = 0;
    std::uint32_t plo = 0;
    MulU32Wide(cand, cand, phi, plo);
    if (phi < hi || (phi == hi && plo <= lo)) {
      res = cand;
    }
  }
  return res;
}

// Two's-complement negate of a 64-bit value stored as (hi, lo) uint32 halves.
AE_INTEGER_MATH_CONSTEXPR void NegU32Wide(std::uint32_t& hi,
                                          std::uint32_t& lo) noexcept {
  lo = ~lo + 1U;
  hi = ~hi + (lo == 0U ? 1U : 0U);
}

AE_INTEGER_MATH_CONSTEXPR void MulI32Wide(std::int32_t a, std::int32_t b,
                                          std::uint32_t& hi,
                                          std::uint32_t& lo) noexcept {
  bool const neg = (a < 0) != (b < 0);
  MulU32Wide(AbsI32ToU32(a), AbsI32ToU32(b), hi, lo);
  if (neg) {
    NegU32Wide(hi, lo);
  }
}

// Shift (hi:lo) left. Returns false if bits shift out of hi.
AE_INTEGER_MATH_CONSTEXPR bool ShlU32WideChecked(std::uint32_t& hi,
                                                 std::uint32_t& lo,
                                                 unsigned bits) noexcept {
  if (bits == 0U) {
    return true;
  }
  if (bits >= 64U) {
    if (hi != 0U || lo != 0U) {
      return false;
    }
    return true;
  }
  if (bits >= 32U) {
    unsigned const rest = bits - 32U;
    if (hi != 0U) {
      return false;
    }
    if (rest != 0U && (lo >> (32U - rest)) != 0U) {
      return false;
    }
    hi = lo << rest;
    lo = 0U;
    return true;
  }
  if ((hi >> (32U - bits)) != 0U) {
    return false;
  }
  hi = (hi << bits) | (lo >> (32U - bits));
  lo <<= bits;
  return true;
}

// Arithmetic-free logical right shift of (hi:lo) with optional round-nearest
// (half away from zero on the unsigned magnitude).
AE_INTEGER_MATH_CONSTEXPR void ShrU32Wide(std::uint32_t& hi, std::uint32_t& lo,
                                          unsigned bits,
                                          bool round_nearest) noexcept {
  if (bits == 0U) {
    return;
  }
  if (bits >= 64U) {
    std::uint32_t const round =
        round_nearest && (hi != 0U || lo != 0U) ? 1U : 0U;
    hi = 0U;
    lo = round;
    return;
  }
  std::uint32_t round_bit = 0U;
  if (round_nearest) {
    if (bits <= 32U) {
      round_bit = (lo >> (bits - 1U)) & 1U;
    } else {
      round_bit = (hi >> (bits - 33U)) & 1U;
    }
  }
  if (bits >= 32U) {
    lo = hi >> (bits - 32U);
    hi = 0U;
  } else {
    lo = (lo >> bits) | (hi << (32U - bits));
    hi >>= bits;
  }
  if (round_bit != 0U) {
    lo += 1U;
    if (lo == 0U) {
      hi += 1U;
    }
  }
}

// Compare unsigned products a*b vs c*d using 16×16 decomposition.
AE_INTEGER_MATH_CONSTEXPR int CmpMulU32(std::uint32_t a, std::uint32_t b,
                                        std::uint32_t c,
                                        std::uint32_t d) noexcept {
  std::uint32_t ahi = 0;
  std::uint32_t alo = 0;
  std::uint32_t bhi = 0;
  std::uint32_t blo = 0;
  MulU32Wide(a, b, ahi, alo);
  MulU32Wide(c, d, bhi, blo);
  if (ahi != bhi) {
    return ahi < bhi ? -1 : 1;
  }
  if (alo != blo) {
    return alo < blo ? -1 : 1;
  }
  return 0;
}

}  // namespace ae::integer_math

#undef AE_INTEGER_MATH_CONSTEXPR

#endif  // AE_NUMERIC_INTEGER_MATH_H_
