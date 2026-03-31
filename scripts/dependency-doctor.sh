#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." &>/dev/null && pwd)"
export REPO_ROOT

python3 - <<'PY'
from __future__ import annotations

import os
from pathlib import Path

repo_root = Path(os.environ["REPO_ROOT"])
code_root = repo_root / "code"

requirement_files = sorted(code_root.glob("requirements*.txt"))
if not requirement_files:
    raise SystemExit("No requirements*.txt files found under code/.")

pins: dict[str, set[str]] = {}
pin_files: dict[str, set[str]] = {}
unpinned_entries: list[tuple[Path, str]] = []

flavour_requirement_files = {
    "requirements.cpu.txt",
    "requirements.cuda.txt",
    "requirements.rocm.txt",
}

for req_file in requirement_files:
    for raw_line in req_file.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith(("-r", "--")):
            continue

        if "==" not in line:
            unpinned_entries.append((req_file, line))
            continue

        package, version = line.split("==", maxsplit=1)
        package = package.strip().lower()
        version = version.strip()
        if not package or not version:
            unpinned_entries.append((req_file, line))
            continue
        pins.setdefault(package, set()).add(version)
        pin_files.setdefault(package, set()).add(req_file.name)

if unpinned_entries:
    print("Found unpinned requirement entries:")
    for path, line in unpinned_entries:
        print(f"- {path.relative_to(repo_root)}: {line}")
    raise SystemExit(1)

conflicts: dict[str, set[str]] = {}
for pkg, versions in pins.items():
    if len(versions) <= 1:
        continue

    source_files = pin_files.get(pkg, set())
    if source_files and source_files.issubset(flavour_requirement_files):
        continue

    conflicts[pkg] = versions

if conflicts:
    print("Found conflicting package pins across requirements files:")
    for pkg, versions in sorted(conflicts.items()):
        print(f"- {pkg}: {', '.join(sorted(versions))}")
    raise SystemExit(1)

print("Dependency doctor checks passed:")
print(f"- validated {len(requirement_files)} requirements files")
print(f"- validated {len(pins)} pinned packages")
PY

echo "dependency-doctor: OK"
