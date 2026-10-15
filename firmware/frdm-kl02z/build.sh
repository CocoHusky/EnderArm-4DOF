#!/bin/sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)"
DEPS="$ROOT/.deps"
KSDK="$DEPS/ksdk"
APP="$KSDK/examples/frdmkl02z/demo_apps/enderarm_sensor"
PINNED_COMMIT="ac3682bd5fe41b65e4ff5b8bebd102a606adaf14"
KSDK_URL="https://github.com/theatrus/ksdk.git"

command -v cmake >/dev/null || { echo "cmake is required" >&2; exit 1; }

if [ -z "${ARMGCC_DIR:-}" ]; then
  if [ -x "$DEPS/arm-toolchain/Payload/bin/arm-none-eabi-gcc" ]; then
    ARMGCC_DIR="$DEPS/arm-toolchain/Payload"
  elif [ -x "/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc" ]; then
    ARMGCC_DIR="/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi"
  else
    echo "Complete Arm GNU Embedded toolchain not found." >&2
    echo "Run: ./firmware/frdm-kl02z/setup_toolchain_macos.sh" >&2
    exit 1
  fi
fi
if [ ! -x "$ARMGCC_DIR/bin/arm-none-eabi-gcc" ] || [ ! -f "$ARMGCC_DIR/arm-none-eabi/include/stdio.h" ]; then
  echo "ARMGCC_DIR must contain the complete Arm GNU Embedded toolchain including Newlib: $ARMGCC_DIR" >&2
  exit 1
fi
export ARMGCC_DIR

mkdir -p "$DEPS"
if [ ! -d "$KSDK/.git" ]; then
  git clone --filter=blob:none "$KSDK_URL" "$KSDK"
fi
git -C "$KSDK" fetch --quiet origin "$PINNED_COMMIT"
git -C "$KSDK" checkout --quiet --detach "$PINNED_COMMIT"

# Build the KL02 KSDK platform library from this exact pinned checkout.
# Restore the one build recipe we patch so repeated builds always start from
# the exact pinned source, then apply the compatibility change deterministically.
PLATFORM_REL="lib/ksdk_platform_lib/armgcc/KL02Z4"
PLATFORM="$KSDK/$PLATFORM_REL"
git -C "$KSDK" checkout "$PINNED_COMMIT" -- "$PLATFORM_REL/CMakeLists.txt"
python3 - "$PLATFORM/CMakeLists.txt" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
s=p.read_text()
lines=[]
for line in s.splitlines():
    if "CMAKE_C_FLAGS_" in line and "-std=gnu99" in line and "-include sys/types.h" not in line:
        line=line.replace("-std=gnu99", "-include sys/types.h -std=gnu99")
    lines.append(line)
p.write_text("\n".join(lines)+"\n")
PY
rm -rf "$PLATFORM/CMakeFiles" "$PLATFORM/CMakeCache.txt" "$PLATFORM/release" "$PLATFORM/debug"
(
  cd "$PLATFORM"
  cmake -DCMAKE_TOOLCHAIN_FILE="../../../../tools/cmake_toolchain_files/armgcc.cmake" \
        -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
        -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .
  cmake --build . --parallel 4
)

rm -rf "$APP"
cp -R "$KSDK/examples/frdmkl02z/demo_apps/bubble_level_tpm" "$APP"
cp "$ROOT/firmware/frdm-kl02z/main.c" "$APP/main.c"

CMAKE="$APP/armgcc/CMakeLists.txt"
python3 - "$CMAKE" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
s=p.read_text()
s=s.replace("bubble_level_tpm", "enderarm_sensor")
# KSDK 1.3 predates modern Newlib headers. Its fsl_misc_utilities.c uses
# caddr_t without including sys/types.h; inject that standard header at
# compile time rather than modifying the pinned SDK source.
lines=[]
for line in s.splitlines():
    if "CMAKE_C_FLAGS_" in line and "-std=gnu99" in line:
        line=line.replace("-std=gnu99", "-include sys/types.h -std=gnu99")
    lines.append(line)
p.write_text("\n".join(lines)+"\n")
PY

BUILD="$APP/armgcc/build"
rm -rf "$BUILD"
mkdir -p "$BUILD"
cd "$BUILD"
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../../tools/cmake_toolchain_files/armgcc.cmake" \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --parallel 4

OUT="$ROOT/firmware/frdm-kl02z/build"
rm -rf "$OUT"
mkdir -p "$OUT"
cp release/enderarm_sensor.elf release/enderarm_sensor.hex release/enderarm_sensor.bin "$OUT/"
arm-none-eabi-size "$OUT/enderarm_sensor.elf"
echo "Built: $OUT/enderarm_sensor.elf"
