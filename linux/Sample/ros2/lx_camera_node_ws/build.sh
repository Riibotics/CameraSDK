#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
rm -rf build/ install/ log/

# Ubuntu 22.04 amd64 + OpenMPI include paths
MPI_INC_BASE="/usr/lib/x86_64-linux-gnu/openmpi/include"
MPI_INC_LIST="${MPI_INC_BASE};${MPI_INC_BASE}/openmpi"

if [ ! -d "$MPI_INC_BASE" ]; then
  echo "OpenMPI include dir not found: $MPI_INC_BASE"
  echo "Install libopenmpi-dev or adjust MPI paths."
  exit 1
fi

colcon build --cmake-args \
  -DMPI_C_COMPILER_INCLUDE_DIRS="$MPI_INC_LIST" \
  -DMPI_C_HEADER_DIR="$MPI_INC_BASE"

source install/setup.bash
