#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
rm -rf build/ install/ log/

# Optional OpenMPI include hints (some environments need them)
MPI_INC_BASE="/usr/lib/x86_64-linux-gnu/openmpi/include"
MPI_INC_LIST="${MPI_INC_BASE};${MPI_INC_BASE}/openmpi"

if [ -d "$MPI_INC_BASE" ]; then
  echo "OpenMPI include dir found: $MPI_INC_BASE"
  colcon build --cmake-args \
    -DMPI_C_COMPILER_INCLUDE_DIRS="$MPI_INC_LIST" \
    -DMPI_C_HEADER_DIR="$MPI_INC_BASE"
else
  echo "OpenMPI include dir not found: $MPI_INC_BASE"
  echo "Building without MPI include hints."
  echo "If build fails with mpi.h errors, install libopenmpi-dev."
  colcon build
fi

source install/setup.bash
