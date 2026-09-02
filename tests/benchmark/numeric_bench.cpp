/*
 * Copyright 2026 Aethernet Inc.
 *
 * Host micro-benchmark for SegmentedNumber / CyclicCounter (desktop cycles).
 * Prints machine-readable lines; not ESP32-C6 cycle counts.
 */

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <ae-numeric/cyclic_counter.h>
#include <ae-numeric/runtime_numeric_traits.h>
#include <ae-numeric/segmented_number.h>
#include <ae-numeric/segmented_number_wire_io.h>
#include <ae-numeric/wire_io.h>

#include "../segmented_test_formats.h"

namespace {

using Clock = std::chrono::steady_clock;

template <typename F>
double NsPerOp(F&& fn, std::uint64_t iters) {
  for (std::uint64_t i = 0; i < iters / 20 + 1; ++i) {
    fn();
  }
  auto const t0 = Clock::now();
  for (std::uint64_t i = 0; i < iters; ++i) {
    fn();
  }
  auto const t1 = Clock::now();
  double const ns =
      std::chrono::duration<double, std::nano>(t1 - t0).count();
  return ns / static_cast<double>(iters);
}

template <typename Num>
typename Num::runtime_type MakeRuntimeFromInt(std::int64_t logical) {
  using RT = typename Num::runtime_type;
  return ae::runtime_numeric_traits<RT>::FromInteger(logical);
}

template <typename Num>
void BenchSeg(char const* name, std::int64_t logical_sample, std::uint64_t iters) {
  using namespace ae;
  volatile std::uint32_t sink = 0;
  volatile std::int64_t logical_v = logical_sample;
  std::uint8_t buf[16] = {};

  auto const rt0 = MakeRuntimeFromInt<Num>(logical_sample);
  auto const wire_opt = Num::TryEncode(rt0);
  if (!wire_opt) {
    std::printf("BENCH %s SKIP no_encode logical=%lld\n", name,
                static_cast<long long>(logical_sample));
    return;
  }
  auto const wire = *wire_opt;
  auto const n0 = Num::Saturating(rt0);
  auto const packed = wire_traits<Num>::Serialize(n0, buf);

  double const enc = NsPerOp(
      [&] {
        auto const rt = MakeRuntimeFromInt<Num>(logical_v);
        auto w = Num::TryEncode(rt);
        sink ^= w ? 1u : 0u;
      },
      iters);
  double const dec = NsPerOp(
      [&] {
        auto v = Num::TryDecode(wire);
        sink ^= v ? 1u : 0u;
      },
      iters);
  double const ser = NsPerOp(
      [&] {
        auto const rt = MakeRuntimeFromInt<Num>(logical_v);
        Num n = Num::Saturating(rt);
        sink ^= static_cast<std::uint32_t>(
            wire_traits<Num>::Serialize(n, buf));
      },
      iters);
  double const deser = NsPerOp(
      [&] {
        auto r = wire_traits<Num>::Deserialize(buf, sizeof(buf));
        sink ^= static_cast<std::uint32_t>(r.bytes_read);
      },
      iters);
  double const rt_ns = NsPerOp(
      [&] {
        auto const rt = MakeRuntimeFromInt<Num>(logical_v);
        Num n = Num::Saturating(rt);
        auto const nw = wire_traits<Num>::Serialize(n, buf);
        auto r = wire_traits<Num>::Deserialize(buf, nw);
        sink ^= static_cast<std::uint32_t>(r.bytes_read);
      },
      iters);
  std::printf(
      "BENCH_SEG name=%s logical=%lld wire_bytes=%zu encode_ns=%.3f decode_ns=%.3f "
      "serialize_ns=%.3f deserialize_ns=%.3f roundtrip_ns=%.3f sink=%u\n",
      name, static_cast<long long>(logical_sample), packed, enc, dec, ser, deser,
      rt_ns, sink);
}

void BenchCyclic(std::uint64_t iters) {
  using C = ae::CyclicCounter<std::uint8_t, std::uint32_t>;
  C c{1001u};
  volatile std::uint32_t sink = 0;
  std::uint8_t buf[1] = {237u};

  double const wire = NsPerOp(
      [&] { sink ^= c.WireValue(); }, iters);
  double const fwd = NsPerOp(
      [&] {
        auto r = c.TryRestore(237u);
        sink ^= r ? *r : 0u;
      },
      iters);
  C c2{1008u};
  double const back = NsPerOp(
      [&] {
        auto r = c2.TryRestore(235u);
        sink ^= r ? *r : 0u;
      },
      iters);
  C c3{1001u};
  double const adv = NsPerOp(
      [&] {
        C tmp{1001u};
        auto r = tmp.TryAdvance(237u);
        sink ^= r ? *r : 0u;
      },
      iters);
  C c4{1001u};
  double const ctx = NsPerOp(
      [&] {
        C tmp{1001u};
        auto r = tmp.TryDeserializeAndAdvance(buf, 1);
        sink ^= r.ok() ? r.value : 0u;
      },
      iters);
  std::printf(
      "BENCH_CYC wire_ns=%.3f restore_fwd_ns=%.3f restore_back_ns=%.3f "
      "advance_ns=%.3f ctx_deser_ns=%.3f sink=%u\n",
      wire, fwd, back, adv, ctx, sink);
}

}  // namespace

int main() {
  using namespace ae::test_segmented_formats;
  constexpr std::uint64_t kIters = 200000;

  std::printf("BENCH_HOST arch=desktop note=nanoseconds_not_esp32_cycles "
              "iters=%llu\n",
              static_cast<unsigned long long>(kIters));

  // logical samples chosen to exercise central / tail / tier regimes
  BenchSeg<Rssi>("Rssi", -40, kIters);
  BenchSeg<Temperature>("Temperature_center", 25, kIters);
  BenchSeg<Temperature>("Temperature_low", -35, kIters);
  BenchSeg<Temperature>("Temperature_high", 100, kIters);
  BenchSeg<Humidity>("Humidity", 55, kIters);
  BenchSeg<Co2>("Co2_1B", 600, kIters);
  BenchSeg<Co2>("Co2_2B", 2500, kIters);
  BenchSeg<Co2>("Co2_4B", 18000, kIters);
  BenchSeg<RxWindow>("Rx_1B", 1, kIters);
  BenchSeg<RxWindow>("Rx_2B", 120, kIters);
  BenchSeg<RxWindow>("Rx_4B", 7200, kIters);
  BenchSeg<Battery>("Battery", 3, kIters);
  BenchSeg<ConnectDuration>("ConnectDuration", 5, kIters);
  BenchCyclic(kIters);
  return 0;
}
