#!/usr/bin/env bash
# tools/render_screens/render.sh: render the head unit's main screens to PNG
# on a desktop, using the real renderer (firmware/opentread/hw_display.cpp).
#
# The renderer is compiled unchanged with LovyanGFX, the test/emu platform
# stubs (clock, heap, Serial), and a host LGFX_CYD that is a 320x240 sprite
# instead of the SPI panel (host_lgfx_cyd.h; the generated hw_display.h swaps
# it in for the real class). No SDL is needed: fake_sdl/ only selects
# LovyanGFX's desktop headers.
#
# Usage: tools/render_screens/render.sh [out_dir (default docs/img)] [scale (default 2)]
# Env:   LGFX_DIR  LovyanGFX checkout (default: the arduino-cli sketchbook's
#                  libraries/LovyanGFX); must be 1.2.26 like the firmware.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
FW="$ROOT/firmware/opentread"
OUT="${1:-$ROOT/docs/img}"
SCALE="${2:-2}"
CXX="${CXX:-clang++}"
CC="${CC:-clang}"

if [ -z "${LGFX_DIR:-}" ]; then
  SKETCHBOOK="$(arduino-cli config get directories.user 2>/dev/null || true)"
  LGFX_DIR="${SKETCHBOOK:-$HOME/Documents/Arduino}/libraries/LovyanGFX"
fi
[ -f "$LGFX_DIR/src/LovyanGFX.hpp" ] || { echo "render: LovyanGFX not found at $LGFX_DIR (set LGFX_DIR)" >&2; exit 1; }
LG="$LGFX_DIR/src"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/opentread_render.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

# The real renderer, plus a capture hook inside its translation unit (the
# panel object lives in an anonymous namespace there).
cp "$FW/hw_display.cpp" "$WORK/hw_display.cpp"
cat >> "$WORK/hw_display.cpp" <<'EOF'

namespace disp {
void hostCapture(uint8_t* rgb) { gLcd.readRectRGB(0, 0, gLcd.width(), gLcd.height(), rgb); }
}  // namespace disp
EOF

# hw_display.h with the hardware LGFX_CYD class replaced by the host one.
python3 - "$FW/hw_display.h" "$HERE/host_lgfx_cyd.h" "$WORK/hw_display.h" <<'PY'
import re, sys
src, host, dst = sys.argv[1:]
text = open(src, encoding="utf-8").read()
m = re.search(r"^class LGFX_CYD : public lgfx::LGFX_Device \{\n.*?^\};\n", text, re.S | re.M)
if not m:
    sys.exit("render: LGFX_CYD class not found in hw_display.h")
open(dst, "w", encoding="utf-8").write(text[:m.start()] + open(host, encoding="utf-8").read() + text[m.end():])
PY

INC=(-I"$WORK" -I"$FW" -I"$FW/test/emu/stubs" -I"$HERE/fake_sdl" -I"$LG")
DEFS=(-DOT_BRINGUP=0)
LG_SRCS=("$LG"/lgfx/v1/LGFXBase.cpp "$LG"/lgfx/v1/LGFX_Sprite.cpp "$LG"/lgfx/v1/lgfx_fonts.cpp
         "$LG"/lgfx/v1/misc/*.cpp "$LG"/lgfx/v1/panel/Panel_Device.cpp)
LG_CSRCS=("$LG"/lgfx/utility/*.c "$LG"/lgfx/Fonts/efont/*.c "$LG"/lgfx/Fonts/IPA/*.c)  # font tables: referenced by lgfx_fonts.cpp

echo "render: building (LovyanGFX from $LGFX_DIR)"
for c in "${LG_CSRCS[@]}"; do
  "$CC" -O1 -w -I"$LG" -c "$c" -o "$WORK/$(basename "$c").o"
done
"$CXX" -std=c++17 -O1 -w "${DEFS[@]}" "${INC[@]}" \
  "$HERE/main.cpp" "$HERE/host_lgfx.cpp" "$WORK/hw_display.cpp" "$FW/debug_serial.cpp" "$FW"/src/core/*.cpp \
  "${LG_SRCS[@]}" "$WORK"/*.c.o -lz -Wl,-dead_strip -o "$WORK/render_screens"

mkdir -p "$OUT"
"$WORK/render_screens" "$OUT" "$SCALE"
