#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
rm -rf build/ install/ log/

AUTO_SET_SOCKET_BUFFER="${LX_AUTO_SET_SOCKET_BUFFER:-1}"
SOCKET_BUFFER_MB="${LX_SOCKET_BUFFER_MB:-10}"
SOCKET_SCRIPT="${SCRIPT_DIR}/../../../set_socket_buffer_size.sh"

if [ "$AUTO_SET_SOCKET_BUFFER" != "0" ]; then
  if [ -f "$SOCKET_SCRIPT" ]; then
    echo "Try setting socket buffer size (${SOCKET_BUFFER_MB}MB)."
    if [ "$(id -u)" = "0" ]; then
      sh "$SOCKET_SCRIPT" "$SOCKET_BUFFER_MB" || echo "Socket buffer setup failed; continue build."
    elif command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
      sudo sh "$SOCKET_SCRIPT" "$SOCKET_BUFFER_MB" || echo "Socket buffer setup failed; continue build."
    else
      echo "Skip socket buffer auto setup (requires root/sudo)."
      echo "Manual command: sudo sh $SOCKET_SCRIPT $SOCKET_BUFFER_MB"
    fi
  else
    echo "Socket buffer script not found: $SOCKET_SCRIPT"
  fi
fi

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
