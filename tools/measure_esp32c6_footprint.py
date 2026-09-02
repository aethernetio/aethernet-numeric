#!/usr/bin/env python3
# Copyright 2026 Aethernet Inc.
"""Measure ESP32-C6 object footprints and emit docs/footprint_results.json."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


SEG_SOURCES = [
    ("empty", "minimal_empty.cpp"),
    ("rssi", "minimal_rssi.cpp"),
    ("temperature", "minimal_temperature.cpp"),
    ("humidity", "minimal_humidity.cpp"),
    ("co2", "minimal_co2.cpp"),
    ("rx", "minimal_rx.cpp"),
    ("battery", "minimal_battery.cpp"),
    ("connect", "minimal_connect.cpp"),
    ("thermometer", "minimal_thermometer.cpp"),
    ("all", "minimal_all.cpp"),
]

CYCLIC_SOURCES = [
    ("cyclic_u8_u16", "minimal_cyclic_u8_u16.cpp"),
    ("cyclic_u8_u32", "minimal_cyclic_u8_u32.cpp"),
    ("cyclic_u16_u32", "minimal_cyclic_u16_u32.cpp"),
]


def find_esp_gxx() -> Path:
    env = os.environ.get("RISCV32_ESP_ELF_GXX")
    if env and Path(env).exists():
        return Path(env)
    roots = [
        Path(r"C:\Espressif\tools\riscv32-esp-elf"),
        Path.home() / ".espressif" / "tools" / "riscv32-esp-elf",
    ]
    matches: list[Path] = []
    for root in roots:
        if not root.exists():
            continue
        matches.extend(root.rglob("riscv32-esp-elf-g++.exe"))
        matches.extend(root.rglob("riscv32-esp-elf-g++"))
    if not matches:
        raise SystemExit("riscv32-esp-elf-g++ not found; set RISCV32_ESP_ELF_GXX")
    # Prefer newer ESP-IDF toolchains (esp-14.* over esp-2022r1).
    def rank(p: Path) -> tuple:
        s = str(p).lower()
        return (
            0 if "esp-14" in s or "esp-13" in s else 1,
            s,
        )

    return sorted(matches, key=rank)[0]


def tool_beside(gxx: Path, name: str) -> Path:
    stem = gxx.name.replace("g++", name).replace("g++.exe", f"{name}.exe")
    # g++.exe -> size.exe / nm.exe / objdump.exe with prefix
    if gxx.name.endswith("g++.exe"):
        cand = gxx.with_name(gxx.name.replace("g++.exe", f"{name}.exe"))
    elif gxx.name.endswith("g++"):
        cand = gxx.with_name(gxx.name.replace("g++", name))
    else:
        cand = gxx.parent / name
    if cand.exists():
        return cand
    raise SystemExit(f"tool not found beside g++: {cand}")


def section_sizes(size_bin: Path, obj: Path) -> dict[str, int]:
    out = subprocess.check_output([str(size_bin), "-A", str(obj)], text=True)
    text = rodata = data = bss = 0
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        name, sz = parts[0], parts[1]
        if not sz.isdigit():
            continue
        n = int(sz)
        if name.startswith(".text"):
            text += n
        elif name.startswith(".rodata"):
            rodata += n
        elif name.startswith(".data"):
            data += n
        elif name.startswith(".bss") or name.startswith(".sbss"):
            bss += n
    return {
        "text": text,
        "rodata": rodata,
        "data": data,
        "bss": bss,
        "flash": text + rodata + data,
        "ram": data + bss,
    }


def undef_helpers(nm_bin: Path, obj: Path) -> list[str]:
    out = subprocess.check_output([str(nm_bin), "-u", str(obj)], text=True)
    pat = re.compile(r"(muldi3|divdi3|udivdi3|moddi3|umoddi3|ashldi3|lshrdi3)")
    found = []
    for line in out.splitlines():
        if pat.search(line):
            found.append(line.strip())
    return found


def compile_one(
    gxx: Path,
    src: Path,
    obj: Path,
    include_root: Path,
    opt: str,
    stack_usage: bool,
) -> None:
    flags = [
        str(gxx),
        "-std=c++20",
        opt,
        "-DNDEBUG",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-exceptions",
        "-fno-rtti",
        "-march=rv32imac",
        "-mabi=ilp32",
        f"-I{include_root}",
        f"-I{include_root / 'tests' / 'footprint'}",
        "-c",
        str(src),
        "-o",
        str(obj),
    ]
    if stack_usage:
        flags.insert(3, "-fstack-usage")
    subprocess.check_call(flags)


def parse_stack_usage(su_path: Path) -> dict[str, int]:
    """Return max static stack bytes per function from GCC .su file."""
    result: dict[str, int] = {}
    if not su_path.exists():
        return result
    for line in su_path.read_text(encoding="utf-8", errors="replace").splitlines():
        # file:line:col:function\tbytes\tstatic
        parts = line.split("\t")
        if len(parts) < 2:
            continue
        head, nbytes = parts[0], parts[1]
        if not nbytes.isdigit():
            continue
        fn = head.split(":")[-1]
        result[fn] = max(result.get(fn, 0), int(nbytes))
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--opts", nargs="+", default=["-Os", "-O2"])
    args = ap.parse_args()
    repo: Path = args.repo
    out_json = args.out or (repo / "docs" / "footprint_results.json")

    gxx = find_esp_gxx()
    size_bin = tool_beside(gxx, "size")
    nm_bin = tool_beside(gxx, "nm")
    out_dir = repo / "build-esp32c6-docs"
    out_dir.mkdir(parents=True, exist_ok=True)
    fp_dir = repo / "tests" / "footprint"

    results: dict = {
        "toolchain": str(gxx),
        "target": "ESP32-C6 / riscv32 ilp32",
        "segmented": {},
        "cyclic": {},
        "stack_usage": {},
    }

    for opt in args.opts:
        opt_key = opt.lstrip("-")
        results["segmented"][opt_key] = {}
        results["cyclic"][opt_key] = {}
        for name, src_name in SEG_SOURCES + CYCLIC_SOURCES:
            src = fp_dir / src_name
            obj = out_dir / f"{name}_{opt_key}.o"
            use_su = name in {
                "temperature",
                "co2",
                "rx",
                "battery",
                "cyclic_u8_u32",
            } and opt == "-Os"
            compile_one(gxx, src, obj, repo, opt, stack_usage=use_su)
            sizes = section_sizes(size_bin, obj)
            helpers = undef_helpers(nm_bin, obj)
            entry = {**sizes, "helpers64": helpers}
            if name.startswith("cyclic_"):
                results["cyclic"][opt_key][name] = entry
            else:
                results["segmented"][opt_key][name] = entry
            if use_su:
                su = parse_stack_usage(obj.with_suffix(".su"))
                results["stack_usage"][name] = su

    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
