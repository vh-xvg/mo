#!/bin/bash
# MIT License
#
# Copyright (c) 2026 Adrian Port
# See ../LICENSE for full license text.

set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
base="$(basename "$repo_dir")"
cd "$(dirname "$repo_dir")"
tar --exclude='./build' --exclude='./*.o' --exclude='./*.a' --exclude='./mo' --exclude='./dl' --exclude='./load' --exclude='./make_jpg' --exclude='./makeimage' -czf "${base}.tar.gz" "$base"
echo "Created ${base}.tar.gz"
