#!/bin/bash
# Build the pyorbslam3 extension module in-place (into pyorbslam3/).
set -euo pipefail
cd "$(dirname "$0")"
cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -Dpybind11_DIR="$(python3 -c 'import pybind11; print(pybind11.get_cmake_dir())')" \
  "$@"
cmake --build build -j"$(nproc)"
echo "Built: $(ls pyorbslam3/_core*.so)"
