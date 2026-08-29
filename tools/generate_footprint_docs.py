#!/usr/bin/env python3
# Copyright 2026 Aethernet Inc.
"""Update generated Markdown sections from footprint/benchmark JSON."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


META = {
    "rssi": {
        "label": "RSSI",
        "codes": 128,
        "segments": 1,
        "sizeof_runtime": 1,
        "sizeof_number": 1,
        "sizeof_wire": 1,
        "max_wire_bytes": 1,
    },
    "temperature": {
        "label": "Temperature",
        "codes": 1021,
        "segments": 3,
        "sizeof_runtime": 2,
        "sizeof_number": 2,
        "sizeof_wire": 2,
        "max_wire_bytes": 2,
    },
    "humidity": {
        "label": "Humidity",
        "codes": 256,
        "segments": 3,
        "sizeof_runtime": 2,
        "sizeof_number": 2,
        "sizeof_wire": 1,
        "max_wire_bytes": 1,
    },
    "co2": {
        "label": "CO2",
        "codes": 822,
        "segments": 3,
        "sizeof_runtime": 2,
        "sizeof_number": 2,
        "sizeof_wire": 4,
        "max_wire_bytes": 4,
    },
    "rx": {
        "label": "RX window",
        "codes": 3046,
        "segments": 4,
        "sizeof_runtime": 4,
        "sizeof_number": 4,
        "sizeof_wire": 4,
        "max_wire_bytes": 4,
    },
    "battery": {
        "label": "Battery",
        "codes": 256,
        "segments": 2,
        "sizeof_runtime": 2,
        "sizeof_number": 2,
        "sizeof_wire": 1,
        "max_wire_bytes": 1,
    },
    "connect": {
        "label": "ConnectDuration",
        "codes": 256,
        "segments": 2,
        "sizeof_runtime": 4,
        "sizeof_number": 4,
        "sizeof_wire": 1,
        "max_wire_bytes": 1,
    },
    "thermometer": {
        "label": "Thermometer",
        "codes": "-",
        "segments": "-",
        "sizeof_runtime": "-",
        "sizeof_number": "-",
        "sizeof_wire": "-",
        "max_wire_bytes": "-",
    },
    "all": {
        "label": "All seven",
        "codes": "-",
        "segments": "-",
        "sizeof_runtime": "-",
        "sizeof_number": "-",
        "sizeof_wire": "-",
        "max_wire_bytes": "-",
    },
}

CYCLIC_META = {
    "cyclic_u8_u16": {
        "label": "`uint8_t` -> `uint16_t`",
        "wire": 1,
        "runtime": 2,
        "half": 127,
    },
    "cyclic_u8_u32": {
        "label": "`uint8_t` -> `uint32_t`",
        "wire": 1,
        "runtime": 4,
        "half": 127,
    },
    "cyclic_u16_u32": {
        "label": "`uint16_t` -> `uint32_t`",
        "wire": 2,
        "runtime": 4,
        "half": 32767,
    },
}


def replace_block(text: str, begin: str, end: str, body: str) -> str:
    pattern = re.compile(
        re.escape(begin) + r".*?" + re.escape(end),
        re.DOTALL,
    )
    replacement = f"{begin}\n{body.rstrip()}\n{end}"
    if not pattern.search(text):
        raise SystemExit(f"markers not found: {begin} … {end}")
    return pattern.sub(lambda _m: replacement, text, count=1)


def seg_table(data: dict, opt: str) -> str:
    rows = [
        "| Format | Opt | .text | .rodata | .data | .bss | Flash | RAM | "
        "sizeof(runtime) | sizeof(number) | sizeof(wire) | Codes | Segments |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in [
        "rssi",
        "temperature",
        "humidity",
        "co2",
        "rx",
        "battery",
        "connect",
        "thermometer",
        "all",
    ]:
        m = META[key]
        s = data["segmented"][opt][key]
        rows.append(
            f"| {m['label']} | -{opt} | {s['text']} | {s['rodata']} | {s['data']} | "
            f"{s['bss']} | {s['flash']} | {s['ram']} | {m['sizeof_runtime']} | "
            f"{m['sizeof_number']} | {m['sizeof_wire']} | {m['codes']} | {m['segments']} |"
        )
    return "\n".join(rows)


def cyclic_table(data: dict, opt: str) -> str:
    rows = [
        "| Configuration | Wire B | Runtime B | .text | .rodata | .data | .bss | Flash | RAM | Half-range |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key, m in CYCLIC_META.items():
        s = data["cyclic"][opt][key]
        rows.append(
            f"| {m['label']} | {m['wire']} | {m['runtime']} | {s['text']} | "
            f"{s['rodata']} | {s['data']} | {s['bss']} | {s['flash']} | {s['ram']} | "
            f"{m['half']} |"
        )
    return "\n".join(rows)


def sharing_table(data: dict, opt: str) -> str:
    s = data["segmented"][opt]
    t = s["temperature"]["text"]
    c = s["co2"]["text"]
    h = s["humidity"]["text"]
    b = s["battery"]["text"]
    thermo = s["thermometer"]["text"]
    all7 = s["all"]["text"]
    sum_tc = t + c
    sum_thcb = t + h + c + b
    sum_all = (
        s["rssi"]["text"]
        + t
        + h
        + c
        + s["rx"]["text"]
        + b
        + s["connect"]["text"]
    )
    rows = [
        "| Bundle | Standalone .text sum | Combined .text | Saved |",
        "|---|---:|---:|---:|",
        f"| Temperature + CO2 (standalone sum) | {sum_tc} | (linked separately) | - |",
        f"| Thermometer (T+H+CO2+Bat) | {sum_thcb} | {thermo} | {sum_thcb - thermo} |",
        f"| All seven | {sum_all} | {all7} | {sum_all - all7} |",
    ]
    return "\n".join(rows)


def stack_table(data: dict) -> str:
    su = data.get("stack_usage", {})

    def max_for(name: str, patterns: list[str]) -> int:
        entries = su.get(name, {})
        best = 0
        for fn, nbytes in entries.items():
            for p in patterns:
                if p in fn:
                    best = max(best, int(nbytes))
        return best

    rows = [
        "| Type | Encode | Decode | Serialize | Deserialize | Worst case | Method |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for name, label in [
        ("temperature", "Temperature"),
        ("co2", "CO2"),
        ("rx", "RX"),
        ("battery", "Battery"),
        ("cyclic_u8_u32", "CyclicCounter u8->u32"),
    ]:
        enc = max_for(name, ["TestEncode"])
        dec = max_for(name, ["TestDecode", "TestRestore"])
        ser = max_for(name, ["TestSerialize"])
        deser = max_for(name, ["TestDecode", "TestRestore", "TestAdvance"])
        if name.startswith("cyclic"):
            enc = max_for(name, ["TestRestore", "TestAdvance"])
            ser = max_for(name, ["TestAdvance"])
            deser = max_for(name, ["TestAdvance"])
        worst = max(
            enc,
            dec,
            ser,
            deser,
            max_for(
                name,
                [
                    "LinearRampApproxRuntime",
                    "GeomApproxWork",
                    "EncodeRaw",
                    "Segmented32MathPolicy",
                ],
            ),
        )
        rows.append(
            f"| {label} | {enc} | {dec} | {ser} | {deser} | {worst} | "
            f"GCC `-fstack-usage` ESP32-C6 `-Os` (.su) |"
        )
    return "\n".join(rows)


def parse_bench_lines(path: Path) -> str:
    if not path.exists():
        return (
            "_No `docs/benchmark_results.txt` yet. Build `numeric-bench`, redirect "
            "stdout to that file, then re-run this script._"
        )
    raw = path.read_bytes()
    if raw.startswith(b"\xff\xfe") or raw.startswith(b"\xfe\xff"):
        text = raw.decode("utf-16")
    else:
        text = raw.decode("utf-8-sig")
    rows = [
        "| Name | Encode ns | Decode ns | Serialize ns | Deserialize ns | Round-trip ns | Notes |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    host = ""
    for line in text.splitlines():
        if line.startswith("BENCH_HOST"):
            host = line
            continue
        if line.startswith("BENCH_SEG"):
            kv = dict(
                part.split("=", 1) for part in line.split()[1:] if "=" in part
            )
            rows.append(
                f"| {kv.get('name','?')} | {kv.get('encode_ns','')} | "
                f"{kv.get('decode_ns','')} | {kv.get('serialize_ns','')} | "
                f"{kv.get('deserialize_ns','')} | {kv.get('roundtrip_ns','')} | "
                f"logical={kv.get('logical','')} wire_bytes={kv.get('wire_bytes','')} |"
            )
        elif line.startswith("BENCH_CYC"):
            kv = dict(
                part.split("=", 1) for part in line.split()[1:] if "=" in part
            )
            rows.append(
                f"| CyclicCounter WireValue | {kv.get('wire_ns','')} | - | - | - | - | desktop ns |"
            )
            rows.append(
                f"| CyclicCounter TryRestore forward | {kv.get('restore_fwd_ns','')} | - | - | - | - | desktop ns |"
            )
            rows.append(
                f"| CyclicCounter TryRestore backward | {kv.get('restore_back_ns','')} | - | - | - | - | desktop ns |"
            )
            rows.append(
                f"| CyclicCounter TryAdvance | {kv.get('advance_ns','')} | - | - | - | - | desktop ns |"
            )
            rows.append(
                f"| CyclicCounter contextual deserialize | {kv.get('ctx_deser_ns','')} | - | - | - | - | desktop ns |"
            )
    body = "\n".join(rows)
    if host:
        body = f"`{host}`\n\n" + body
    return body


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument("--results", type=Path, default=None)
    ap.add_argument("--bench", type=Path, default=None)
    args = ap.parse_args()
    repo: Path = args.repo
    results_path = args.results or (repo / "docs" / "footprint_results.json")
    bench_path = args.bench or (repo / "docs" / "benchmark_results.txt")
    data = json.loads(results_path.read_text(encoding="utf-8"))

    fp_path = repo / "docs" / "footprint.md"
    text = fp_path.read_text(encoding="utf-8")
    toolchain = Path(str(data.get("toolchain", ""))).name or "riscv32-esp-elf-g++"
    body_os = (
        f"Toolchain: `{toolchain}` (Espressif riscv32-esp-elf)  \n"
        f"Target: {data['target']}  \n\n"
        "### ESP32-C6 `-Os`\n\n"
        + seg_table(data, "Os")
        + "\n\n### ESP32-C6 `-O2`\n\n"
        + seg_table(data, "O2")
    )
    text = replace_block(
        text,
        "<!-- BEGIN GENERATED FOOTPRINT -->",
        "<!-- END GENERATED FOOTPRINT -->",
        body_os,
    )
    body_cyc = (
        "### ESP32-C6 `-Os`\n\n"
        + cyclic_table(data, "Os")
        + "\n\n### ESP32-C6 `-O2`\n\n"
        + cyclic_table(data, "O2")
        + "\n\nAll three configurations: `.rodata = 0`, `.data = 0`, `.bss = 0`, "
        "heap = 0, tables = 0, 64-bit arithmetic helper undefs = 0."
    )
    text = replace_block(
        text,
        "<!-- BEGIN GENERATED CYCLIC FOOTPRINT -->",
        "<!-- END GENERATED CYCLIC FOOTPRINT -->",
        body_cyc,
    )
    text = replace_block(
        text,
        "<!-- BEGIN GENERATED SHARING -->",
        "<!-- END GENERATED SHARING -->",
        "### ESP32-C6 `-Os`\n\n"
        + sharing_table(data, "Os")
        + "\n\n### ESP32-C6 `-O2`\n\n"
        + sharing_table(data, "O2"),
    )
    text = replace_block(
        text,
        "<!-- BEGIN GENERATED STACK -->",
        "<!-- END GENERATED STACK -->",
        stack_table(data),
    )
    fp_path.write_text(text, encoding="utf-8", newline="\n")
    print(f"updated {fp_path}")

    bench_md = repo / "docs" / "benchmarks.md"
    if bench_md.exists():
        btext = bench_md.read_text(encoding="utf-8")
        btext = replace_block(
            btext,
            "<!-- BEGIN GENERATED BENCHMARKS -->",
            "<!-- END GENERATED BENCHMARKS -->",
            parse_bench_lines(bench_path),
        )
        bench_md.write_text(btext, encoding="utf-8", newline="\n")
        print(f"updated {bench_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
