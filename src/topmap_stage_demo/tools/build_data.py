#!/usr/bin/env python3
"""Compile export_stages.cpp against the existing static topmap libraries and
run it to regenerate ``data/stages.json`` and ``data/planning.json``.

The planning JSON is produced by the production ``HierarchicalPlanner`` run on
the same mesh/graph as the stages, with the fixed demo endpoints.  It is a real
precomputed path, not a hand-drawn or interpolated line.

The compile recipe mirrors the already-built ``test_hierarchical_planner``
target: the include/define flags are parsed from its ``flags.make`` and the
library flags from ``libtopmap_planner.a`` onward are parsed from its
``link.txt``.  The linker is invoked with the original build directory as its
working directory (that is where the relative ``libtopmap_planner.a`` lives),
but every artefact this script produces is written inside this demo directory.

Usage:
    python3 build_data.py                 # compile if needed, then export
    python3 build_data.py --force         # always recompile
    python3 build_data.py --print-command # show the compile command only
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))          # <pkg>/tools
PKG_ROOT = os.path.dirname(HERE)                            # <pkg>
PROBE_DIR = (
    "/home/robot/topmap_ws/build/topmap_planner/CMakeFiles/"
    "test_hierarchical_planner.dir"
)
# flags.make / link.txt are relative to the target binary dir.
PROBE_CWD = "/home/robot/topmap_ws/build/topmap_planner"
FLAGS_MAKE = os.path.join(PROBE_DIR, "flags.make")
LINK_TXT = os.path.join(PROBE_DIR, "link.txt")

SOURCE = os.path.join(HERE, "export_stages.cpp")
BUILD_DIR = os.path.join(HERE, "build")
BINARY = os.path.join(BUILD_DIR, "export_stages")
DEFAULT_OUT = os.path.join(PKG_ROOT, "data", "stages.json")
DEFAULT_PLANNING_OUT = os.path.join(PKG_ROOT, "data", "planning.json")
DEFAULT_ARCHIVE_OUT = os.path.join(PKG_ROOT, "data", "stages.topmap.bin")
PCD = os.path.join(PKG_ROOT, "data", "scene_h_ground_original_ramp.pcd")
CONFIG = os.path.join(PKG_ROOT, "data", "demo_config.yaml")
METADATA = os.path.join(PKG_ROOT, "data", "scene_metadata.json")

CXX = "/usr/bin/c++"


class BuildError(RuntimeError):
    pass


def _make_var(text: str, name: str) -> list[str]:
    """Return one whitespace-split CXX_* variable from a flags.make file."""
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(name) and "=" in stripped:
            head, _, tail = stripped.partition("=")
            if head.strip() == name:
                return tail.split()
    raise BuildError("variable %s not found in %s" % (name, FLAGS_MAKE))


def parse_flags(path: str = FLAGS_MAKE) -> tuple[list[str], list[str], list[str]]:
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    return (
        _make_var(text, "CXX_DEFINES"),
        _make_var(text, "CXX_INCLUDES"),
        _make_var(text, "CXX_FLAGS"),
    )


def parse_link_tail(path: str = LINK_TXT) -> list[str]:
    with open(path, "r", encoding="utf-8") as handle:
        tokens = shlex.split(handle.read().strip())
    for index, token in enumerate(tokens):
        if token.endswith("libtopmap_planner.a"):
            return tokens[index:]
    raise BuildError("libtopmap_planner.a not found in %s" % path)


def compile_command() -> list[str]:
    defines, includes, cxx_flags = parse_flags()
    # Keep the target's own flags but make sure the standard and optimisation
    # are explicit and deterministic for this standalone translation unit.
    flags = [item for item in cxx_flags if item not in ("-g", "-DNDEBUG")]
    command = [CXX, "-std=c++17", "-O2"]
    command += flags
    command += defines
    command += includes
    command += [SOURCE, "-o", BINARY]
    command += parse_link_tail()
    return command


def needs_rebuild(force: bool) -> bool:
    if force:
        return True
    if not os.path.isfile(BINARY):
        return True
    return os.path.getmtime(BINARY) < os.path.getmtime(SOURCE)


def build(force: bool = False) -> str:
    os.makedirs(BUILD_DIR, exist_ok=True)
    if not needs_rebuild(force):
        print("[build_data] up to date: %s" % BINARY)
        return BINARY
    command = compile_command()
    print("[build_data] compiling export_stages.cpp ...")
    process = subprocess.run(command, cwd=PROBE_CWD, text=True, capture_output=True)
    if process.stdout.strip():
        print(process.stdout.rstrip())
    if process.returncode != 0:
        sys.stderr.write(process.stderr)
        raise BuildError("compilation failed with exit code %d" % process.returncode)
    if process.stderr.strip():
        sys.stderr.write(process.stderr)
    if not os.path.isfile(BINARY):
        raise BuildError("compiler reported success but %s is missing" % BINARY)
    print("[build_data] built %s" % BINARY)
    return BINARY


def export(binary: str, out_path: str,
           planning_out_path: str = DEFAULT_PLANNING_OUT,
           archive_out_path: str = DEFAULT_ARCHIVE_OUT) -> tuple[dict, dict]:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    os.makedirs(os.path.dirname(planning_out_path), exist_ok=True)
    os.makedirs(os.path.dirname(archive_out_path), exist_ok=True)
    command = [binary, PCD, CONFIG, METADATA, out_path, planning_out_path,
               archive_out_path]
    print("[build_data] running actual pipeline and HierarchicalPlanner ...")
    process = subprocess.run(command, cwd=HERE, text=True, capture_output=True)
    if process.stderr.strip():
        sys.stderr.write(process.stderr)
    if process.returncode != 0:
        raise BuildError(
            "export_stages failed with exit code %d" % process.returncode
        )
    if process.stdout.strip():
        print("[build_data] %s" % process.stdout.strip())
    with open(out_path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    with open(planning_out_path, "r", encoding="utf-8") as handle:
        planning = json.load(handle)
    return data, planning


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="always recompile")
    parser.add_argument("--out", default=DEFAULT_OUT, help="stages.json output path")
    parser.add_argument(
        "--planning-out",
        default=DEFAULT_PLANNING_OUT,
        help="planning.json output path (real offline HierarchicalPlanner result)",
    )
    parser.add_argument(
        "--archive-out",
        default=DEFAULT_ARCHIVE_OUT,
        help="loadable MapArchive output for the demo planner_node",
    )
    parser.add_argument(
        "--print-command", action="store_true", help="print the compile command and exit"
    )
    args = parser.parse_args(argv)

    if args.print_command:
        print(" ".join(shlex.quote(item) for item in compile_command()))
        return 0

    try:
        binary = build(force=args.force)
        data, planning = export(
            binary, os.path.abspath(args.out), os.path.abspath(args.planning_out),
            os.path.abspath(args.archive_out),
        )
    except BuildError as error:
        sys.stderr.write("[build_data] error: %s\n" % error)
        return 1

    counts = data["counts"]
    print("[build_data] wrote %s" % os.path.abspath(args.out))
    print("[build_data] wrote %s" % os.path.abspath(args.planning_out))
    print("[build_data] wrote %s" % os.path.abspath(args.archive_out))
    print(
        "[build_data] vertices=%(vertex_count)d faces=%(face_count)d "
        "unique_edges=%(unique_edge_count)d grown=%(grown_region_count)d "
        "split=%(split_region_count)d final_regions=%(final_region_count)d "
        "candidates=%(portal_candidate_count)d valid_portals=%(portal_valid_count)d "
        "final_portals=%(final_portal_count)d" % counts
    )
    print(
        "[build_data] planning status=%s points=%d length=%.6f regions=%d portals=%d "
        "safety_valid=%s"
        % (
            planning["status"],
            planning["counts"]["path_points"],
            planning["path_length_m"],
            len(planning["topology"]["regions"]),
            len(planning["topology"]["portals"]),
            planning["safety"]["valid"],
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
