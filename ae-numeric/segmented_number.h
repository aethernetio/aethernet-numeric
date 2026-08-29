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

#ifndef AE_NUMERIC_SEGMENTED_NUMBER_H_
#define AE_NUMERIC_SEGMENTED_NUMBER_H_

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <type_traits>

#include "ae-numeric/details/segmented_compiler.h"
#include "ae-numeric/details/segmented_curves.h"
#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/details/segmented_formula_backend.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/numeric_traits.h"
#include "ae-numeric/runtime_numeric_traits.h"
#include "ae-numeric/wire_io.h"

namespace ae::seg {

template <typename T>
inline constexpr bool kFloatingRuntimeEnabled = false;

namespace segmented_number_internal {

template <typename Logical, typename RT>
struct RuntimeRawMap {
  using raw_type = typename Logical::rep_value_type;

  static constexpr raw_type ToRaw(RT const& v) {
    static_assert(numeric_traits<RT>::kIsFixedPoint,
                  "include ae-numeric/segmented_number_floating_runtime.h "
                  "to use floating runtime");
    return v.RawValue();
  }

  static constexpr RT FromRaw(raw_type raw) {
    static_assert(numeric_traits<RT>::kIsFixedPoint,
                  "include ae-numeric/segmented_number_floating_runtime.h "
                  "to use floating runtime");
    auto const clamped =
        raw < RT::kRawMin ? RT::kRawMin
                          : (raw > RT::kRawMax ? RT::kRawMax : raw);
    return RT::FromRaw(
        fixed_point_internal::RepFromRawValue<typename RT::rep_type>(
            RT::ClampRaw(clamped)));
  }
};

template <typename Spec>
using RuntimeType = std::conditional_t<
    kIsFloatingRuntimePolicy<typename Spec::runtime_policy>,
    typename Spec::runtime_policy::rep,
    segmented_compiler_internal::LogicalTypeOf<Spec>>;

template <typename Wire>
constexpr Wire RankToWire(std::uint32_t rank) {
  if constexpr (std::is_same_v<Wire, std::uint8_t>) {
    return static_cast<std::uint8_t>(rank);
  } else {
    return Wire{rank};
  }
}

template <typename Wire>
constexpr std::uint32_t WireToRank(Wire const& w) {
  if constexpr (std::is_same_v<Wire, std::uint8_t>) {
    return static_cast<std::uint32_t>(w);
  } else {
    return static_cast<std::uint32_t>(static_cast<typename Wire::ValueType>(w));
  }
}

}  // namespace segmented_number_internal

template <typename Spec>
class SegmentedNumber {
 public:
  using spec_type = Spec;
  using runtime_type = segmented_number_internal::RuntimeType<Spec>;
  using logical_type = segmented_compiler_internal::LogicalTypeOf<Spec>;
  using wire_type = segmented_compiler_internal::WireTypeOf<Spec>;
  using runtime_raw_type = typename logical_type::rep_value_type;
  static_assert(sizeof(runtime_raw_type) <= 4,
                "SegmentedNumber runtime raw must be at most 32 bits");

  static_assert(std::is_same_v<typename Spec::compute_policy, compute::Formula>,
                "SegmentedNumber supports compute::Formula only");

  static_assert(sizeof(runtime_raw_type) <= 4,
                "SegmentedNumber runtime raw must be at most 32 bits");
  static constexpr auto kPlan =
      segmented_compiler_internal::PlanHolder<Spec>::kPlan;
  static constexpr std::size_t kSegmentCount =
      static_cast<std::size_t>(kPlan.count);
  static constexpr std::size_t kCodeCount =
      static_cast<std::size_t>(kPlan.code_count);
  static constexpr std::size_t kMaxWireBytes =
      kPlan.n8 > 0 ? 8U : (kPlan.n4 > 0 ? 4U : (kPlan.n2 > 0 ? 2U : 1U));
  // Compile-time schema identity (FNV-1a). Not used by encode/decode math.
  static constexpr std::uint64_t kSchemaHash = kPlan.schema_hash;
  static constexpr std::uint32_t kOneByteCount = kPlan.n1;
  static constexpr std::uint32_t kTwoByteCount = kPlan.n2;
  static constexpr std::uint32_t kFourByteCount = kPlan.n4;
  static constexpr std::size_t kFormulaCoefficientBytes =
      segmented_compiler_internal::FormulaCoefficientBytes<Spec>();

  static_assert(!kIsFloatingRuntimePolicy<typename Spec::runtime_policy> ||
                    kFloatingRuntimeEnabled<typename Spec::runtime_policy::rep>,
                "include ae-numeric/segmented_number_floating_runtime.h "
                "to use floating runtime");
  static_assert(kCodeCount >= 1U, "format without values");

  static constexpr auto kSegments =
      segmented_compiler_internal::MakeExactCompiledSegments<
          Spec, logical_type, kSegmentCount>();

  constexpr SegmentedNumber() : value_(DecodeUnchecked(wire_type{})) {}

  runtime_type Value() const { return value_; }

  static std::optional<runtime_type> TryDecode(wire_type wire) {
    std::uint32_t const rank =
        segmented_number_internal::WireToRank<wire_type>(wire);
    if (rank >= kCodeCount) {
      return std::nullopt;
    }
    return DecodeUnchecked(wire);
  }

  static constexpr runtime_type Decode(wire_type wire) {
    std::uint32_t const rank =
        segmented_number_internal::WireToRank<wire_type>(wire);
    if (rank >= kCodeCount) {
      assert(rank < kCodeCount);
      if (!std::is_constant_evaluated()) {
        std::abort();
      }
    }
    return DecodeUnchecked(wire);
  }

  static std::optional<wire_type> TryEncode(runtime_type value) {
    runtime_raw_type const raw =
        segmented_number_internal::RuntimeRawMap<logical_type,
                                                 runtime_type>::ToRaw(value);
    if (raw < kDeclaredRawMin || raw > kDeclaredRawMax) {
      return std::nullopt;
    }
    return segmented_number_internal::RankToWire<wire_type>(EncodeRaw(raw));
  }

  static std::optional<SegmentedNumber> TryFromRuntime(runtime_type value) {
    auto const w = TryEncode(value);
    if (!w.has_value()) {
      return std::nullopt;
    }
    return FromWire(*w);
  }

  static SegmentedNumber Saturating(runtime_type value) {
    runtime_raw_type raw =
        segmented_number_internal::RuntimeRawMap<logical_type,
                                                 runtime_type>::ToRaw(value);
    if (raw < kDeclaredRawMin) {
      raw = kDeclaredRawMin;
    }
    if (raw > kDeclaredRawMax) {
      raw = kDeclaredRawMax;
    }
    return FromWire(segmented_number_internal::RankToWire<wire_type>(
        EncodeRaw(raw)));
  }

  static SegmentedNumber FromRuntimeUnchecked(runtime_type value) {
    return FromWire(segmented_number_internal::RankToWire<wire_type>(
        EncodeRaw(segmented_number_internal::RuntimeRawMap<
                  logical_type, runtime_type>::ToRaw(value))));
  }

  static SegmentedNumber FromWire(wire_type wire) {
    SegmentedNumber n;
    n.value_ = DecodeUnchecked(wire);
    return n;
  }

  constexpr bool operator==(SegmentedNumber const& o) const {
    return value_ == o.value_;
  }
  constexpr bool operator!=(SegmentedNumber const& o) const {
    return value_ != o.value_;
  }
  constexpr bool operator<(SegmentedNumber const& o) const {
    return value_ < o.value_;
  }
  constexpr bool operator>(SegmentedNumber const& o) const {
    return value_ > o.value_;
  }
  constexpr bool operator<=(SegmentedNumber const& o) const {
    return value_ <= o.value_;
  }
  constexpr bool operator>=(SegmentedNumber const& o) const {
    return value_ >= o.value_;
  }

  static std::size_t Serialize(SegmentedNumber const& value, std::uint8_t* out) {
    auto const enc = TryEncode(value.Value());
    if (!enc.has_value()) {
      return 0;
    }
    return wire_traits<wire_type>::Serialize(*enc, out);
  }

  static DeserializeResult<SegmentedNumber> Deserialize(std::uint8_t const* in,
                                                        std::size_t len) {
    if (in == nullptr || len == 0) {
      return {SegmentedNumber{}, 0};
    }
    if constexpr (!std::is_same_v<wire_type, std::uint8_t>) {
      if (wire_type::WireBytesNeeded(in, len) == 0) {
        return {SegmentedNumber{}, 0};
      }
    } else if (len < 1) {
      return {SegmentedNumber{}, 0};
    }
    auto const wr = wire_traits<wire_type>::Deserialize(in, len);
    if (wr.bytes_read == 0 || wr.bytes_read > len) {
      return {SegmentedNumber{}, 0};
    }
    std::uint32_t const rank =
        segmented_number_internal::WireToRank<wire_type>(wr.value);
    if (rank >= kCodeCount) {
      return {SegmentedNumber{}, 0};
    }
    return {FromWire(wr.value), wr.bytes_read};
  }

  static constexpr std::size_t MaxWireBytes() { return kMaxWireBytes; }

  static constexpr auto const& Logical() { return kPlan; }

  static constexpr runtime_raw_type kDeclaredRawMin =
      logical_type::FromRatio(kPlan.declared_min.num, kPlan.declared_min.den)
          .RawValue();
  static constexpr runtime_raw_type kDeclaredRawMax =
      logical_type::FromRatio(kPlan.declared_max.num, kPlan.declared_max.den)
          .RawValue();
  static constexpr runtime_raw_type kRepresentableRawMin =
      segmented_formula_internal::ScanRepresentable<Spec, logical_type>(true);
  static constexpr runtime_raw_type kRepresentableRawMax =
      segmented_formula_internal::ScanRepresentable<Spec, logical_type>(
          false);

 private:
  static constexpr runtime_type DecodeUnchecked(wire_type wire) {
    std::uint32_t const rank =
        segmented_number_internal::WireToRank<wire_type>(wire);
    runtime_raw_type const raw =
        segmented_formula_internal::DecodeRankRaw<Spec, logical_type>(rank);
    return segmented_number_internal::RuntimeRawMap<logical_type,
                                                    runtime_type>::FromRaw(raw);
  }

  static constexpr std::uint32_t EncodeRaw(runtime_raw_type raw) {
    return segmented_formula_internal::EncodeRaw<Spec, logical_type>(
        static_cast<std::uint32_t>(kCodeCount), raw);
  }

  runtime_type value_{};
};

template <typename Spec>
using Compile = SegmentedNumber<Spec>;

}  // namespace ae::seg

namespace ae::seg {
namespace segmented_number_size_internal {
template <typename Spec>
constexpr bool SizeMatches() {
  return sizeof(SegmentedNumber<Spec>) ==
         sizeof(typename SegmentedNumber<Spec>::runtime_type);
}
}  // namespace segmented_number_size_internal
}  // namespace ae::seg

namespace ae {

using seg::SegmentedNumber;

template <typename Spec>
struct numeric_traits<seg::SegmentedNumber<Spec>> {
  using value_type = seg::SegmentedNumber<Spec>;
  using rep_type = typename value_type::wire_type;
  using rep_value_type = typename numeric_traits<rep_type>::rep_value_type;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = false;
  static constexpr bool kIsSegmented = true;
  static constexpr bool kIsSigned =
      numeric_traits<typename value_type::runtime_type>::kIsSigned;
};

template <typename Spec>
struct runtime_numeric_traits<seg::SegmentedNumber<Spec>> {
  using value_type = seg::SegmentedNumber<Spec>;
  using RT = typename value_type::runtime_type;

  static constexpr bool kIsSupported = true;
  static constexpr bool kIsSigned = runtime_numeric_traits<RT>::kIsSigned;

  static constexpr value_type FromInteger(std::int64_t value) {
    return value_type::Saturating(runtime_numeric_traits<RT>::FromInteger(value));
  }
};

}  // namespace ae

#endif  // AE_NUMERIC_SEGMENTED_NUMBER_H_
