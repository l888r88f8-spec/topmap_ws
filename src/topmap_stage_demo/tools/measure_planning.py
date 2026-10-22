#!/usr/bin/env python3
"""Build and run the standalone offline planner timing harness.

It reuses the exact compile flags and link line of the existing
``test_hierarchical_planner`` target (via ``build_data.py``), so the harness is
not built with a different compiler configuration.  The measured planner code
is always the production static library passed with ``--planner-lib`` (default:
the current build's ``libtopmap_planner.a``); archive loading and map setup are
outside every timer inside the harness.

The harness binary name includes a content fingerprint of ``--planner-lib``.
Switching between a saved baseline and the current library therefore always
runs the executable linked against that exact archive, even when both archives
already exist and their mtimes do not change. ``--force`` remains available for
an explicit recompile.

Usage:
    python3 measure_planning.py                       # current build -> timing.json
    python3 measure_planning.py --planner-lib /path/libtopmap_planner.before.a \
        --output ../../../log/surface_query_performance/before_timing.json
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shlex
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
WS_ROOT = os.path.dirname(os.path.dirname(PKG_ROOT))
sys.path.insert(0, HERE)
import build_data  # noqa: E402

SOURCE = os.path.join(HERE, "measure_planning.cpp")
BUILD_DIR = os.path.join(HERE, "build")
BINARY_PREFIX = "measure_planning"
DEFAULT_ARCHIVE = os.path.join(PKG_ROOT, "data", "stages.topmap.bin")
DEFAULT_CONFIG = os.path.join(PKG_ROOT, "data", "demo_config.yaml")
DEFAULT_OUTPUT = os.path.join(WS_ROOT, "log", "surface_query_performance",
                              "timing.json")
DEFAULT_PLANNER_LIB = os.path.join(WS_ROOT, "build", "topmap_planner",
                                   "libtopmap_planner.a")


def planner_fingerprint(planner_lib: str) -> str:
    """Content identity of the archive actually linked into the harness."""
    digest = hashlib.sha256()
    with open(planner_lib, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    digest.update(b"\0size=" + str(os.path.getsize(planner_lib)).encode("ascii"))
    return digest.hexdigest()[:16]


def binary_path(planner_lib: str) -> str:
    return os.path.join(BUILD_DIR,
                        "%s-%s" % (BINARY_PREFIX, planner_fingerprint(planner_lib)))


def compile_command(planner_lib: str, binary: str) -> list[str]:
    defines, includes, cxx_flags = build_data.parse_flags()
    link_tail = build_data.parse_link_tail()
    link_tail = [
        os.path.abspath(planner_lib) if token.endswith("libtopmap_planner.a") else token
        for token in link_tail
    ]
    command = [build_data.CXX, "-std=c++17"]
    command += cxx_flags
    command += defines
    command += includes
    command += [SOURCE, "-o", binary]
    command += link_tail
    return command


def needs_build(force: bool, planner_lib: str, binary: str) -> bool:
    if force or not os.path.isfile(binary):
        return True
    binary_mtime = os.path.getmtime(binary)
    return (binary_mtime < os.path.getmtime(SOURCE) or
            binary_mtime < os.path.getmtime(planner_lib))


def build(force: bool, planner_lib: str) -> str:
    if not os.path.isfile(planner_lib):
        raise SystemExit("planner library not found: %s" % planner_lib)
    binary = binary_path(planner_lib)
    if not needs_build(force, planner_lib, binary):
        print("[measure_planning] up to date: %s" % binary)
        return binary
    os.makedirs(BUILD_DIR, exist_ok=True)
    command = compile_command(planner_lib, binary)
    process = subprocess.run(command, cwd=build_data.PROBE_CWD, text=True,
                             capture_output=True)
    if process.returncode != 0:
        sys.stderr.write(process.stdout)
        sys.stderr.write(process.stderr)
        raise SystemExit("harness compile failed with exit code %d" %
                         process.returncode)
    print("[measure_planning] built %s" % binary)
    return binary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", default=DEFAULT_ARCHIVE)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--planner-lib", default=DEFAULT_PLANNER_LIB,
                        help="production planner archive to link (before/after)")
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--force", action="store_true", help="always recompile")
    parser.add_argument("--print-command", action="store_true")
    args = parser.parse_args(argv)

    planner_lib = os.path.abspath(args.planner_lib)
    if args.print_command:
        binary = (binary_path(planner_lib) if os.path.isfile(planner_lib)
                  else os.path.join(BUILD_DIR, BINARY_PREFIX))
        print(" ".join(shlex.quote(item)
                       for item in compile_command(planner_lib, binary)))
        return 0

    binary = build(args.force, planner_lib)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    command = [binary, os.path.abspath(args.archive), os.path.abspath(args.config),
               os.path.abspath(args.output), str(args.warmup), str(args.runs)]
    process = subprocess.run(command, cwd=HERE, text=True)
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
