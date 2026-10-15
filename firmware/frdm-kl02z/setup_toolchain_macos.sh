#!/bin/sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)"
OUT="$ROOT/.deps/arm-toolchain"

command -v brew >/dev/null || { echo "Homebrew is required to fetch the official Arm toolchain package" >&2; exit 1; }
command -v pkgutil >/dev/null || { echo "pkgutil is required on macOS" >&2; exit 1; }

mkdir -p "$ROOT/.deps"
echo "Fetching official Arm GNU Embedded toolchain package..."
brew fetch --cask gcc-arm-embedded >/dev/null
PKG="$(brew --cache --cask gcc-arm-embedded)"
[ -f "$PKG" ] || { echo "Could not locate downloaded gcc-arm-embedded package" >&2; exit 1; }

rm -rf "$OUT"
pkgutil --expand-full "$PKG" "$OUT"

TOOLCHAIN="$OUT/Payload"
[ -x "$TOOLCHAIN/bin/arm-none-eabi-gcc" ] || {
  echo "Extracted package does not contain arm-none-eabi-gcc" >&2
  exit 1
}
[ -f "$TOOLCHAIN/arm-none-eabi/include/stdio.h" ] || {
  echo "Extracted package does not contain Newlib headers" >&2
  exit 1
}

"$TOOLCHAIN/bin/arm-none-eabi-gcc" --version | head -1
echo "Toolchain ready: $TOOLCHAIN"
echo "No system install or sudo was used."
