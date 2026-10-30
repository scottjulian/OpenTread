// tools/render_screens: render head-unit screens on a desktop with the REAL
// renderer (firmware/opentread/hw_display.cpp, compiled unchanged against
// LovyanGFX with a 320x240 sprite as the panel), and save them as PNGs.
//
// Each scene feeds the renderer Frames the way app.cpp does (frame() when
// due, service() every pass, setHold() every pass while holding) on the
// emulated clock, long enough for the pace/speed display filters to settle.
//
// Usage: render_screens <out_dir> [scale]
#include <zlib.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "emu_platform.h"  // test/emu stubs: the emulated clock (emu::gNow)
#include "hw_display.h"
#include "src/core/config.h"

namespace disp {
void hostCapture(uint8_t* rgb);  // appended to the copied hw_display.cpp by render.sh
}

namespace {

constexpr int W = 320, H = 240;
constexpr int64_t PASS_US = 5000;  // one loop pass

// Belt distance for a given number of metres (M_PER_PULSE_UM per pulse).
uint32_t pulsesFor(double metres) {
  return static_cast<uint32_t>(metres * 1e6 / static_cast<double>(ot::M_PER_PULSE_UM) + 0.5);
}

int32_t holdPermille = -1;  // < 0: no hold in progress

void run(disp::Frame f, int64_t forUs) {
  const int64_t end = emu::gNow + forUs;
  while (emu::gNow < end) {
    const int64_t now = esp_timer_get_time();
    disp::setHold(now, holdPermille >= 0, holdPermille < 0 ? 0 : holdPermille);
    if (disp::frameDue(now)) {
      f.t = now;
      disp::frame(f);
    }
    disp::service(now);
    emu::gNow += PASS_US;
  }
}

void putU32(std::vector<uint8_t>& v, uint32_t x) {
  for (int s = 24; s >= 0; s -= 8) v.push_back(static_cast<uint8_t>(x >> s));
}

void chunk(std::vector<uint8_t>& png, const char* type, const std::vector<uint8_t>& data) {
  putU32(png, static_cast<uint32_t>(data.size()));
  std::vector<uint8_t> body(type, type + 4);
  body.insert(body.end(), data.begin(), data.end());
  png.insert(png.end(), body.begin(), body.end());
  putU32(png, static_cast<uint32_t>(crc32(0, body.data(), static_cast<uInt>(body.size()))));
}

bool writePng(const std::string& path, const std::vector<uint8_t>& rgb, int scale) {
  const int w = W * scale, h = H * scale;
  std::vector<uint8_t> raw;
  raw.reserve(static_cast<size_t>(h) * (1 + static_cast<size_t>(w) * 3));
  for (int y = 0; y < h; ++y) {
    raw.push_back(0);  // filter: none
    const uint8_t* row = &rgb[static_cast<size_t>(y / scale) * W * 3];
    for (int x = 0; x < w; ++x) raw.insert(raw.end(), row + (x / scale) * 3, row + (x / scale) * 3 + 3);
  }
  uLongf zlen = compressBound(static_cast<uLong>(raw.size()));
  std::vector<uint8_t> z(zlen);
  if (compress2(z.data(), &zlen, raw.data(), static_cast<uLong>(raw.size()), 9) != Z_OK) return false;
  z.resize(zlen);

  std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
  std::vector<uint8_t> ihdr;
  putU32(ihdr, static_cast<uint32_t>(w));
  putU32(ihdr, static_cast<uint32_t>(h));
  ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});  // 8-bit RGB
  chunk(png, "IHDR", ihdr);
  chunk(png, "IDAT", z);
  chunk(png, "IEND", {});
  FILE* fp = std::fopen(path.c_str(), "wb");
  if (!fp) return false;
  const bool ok = std::fwrite(png.data(), 1, png.size(), fp) == png.size();
  return std::fclose(fp) == 0 && ok;
}

bool capture(const std::string& dir, const char* name, int scale) {
  std::vector<uint8_t> rgb(static_cast<size_t>(W) * H * 3);
  disp::hostCapture(rgb.data());
  const std::string path = dir + "/" + name + ".png";
  if (!writePng(path, rgb, scale)) {
    std::fprintf(stderr, "render_screens: cannot write %s\n", path.c_str());
    return false;
  }
  std::printf("render_screens: %s\n", path.c_str());
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <out_dir> [scale]\n", argv[0]);
    return 2;
  }
  const std::string out = argv[1];
  const int scale = argc > 2 ? std::atoi(argv[2]) : 2;
  if (scale < 1 || scale > 8) return 2;

  disp::Frame f;
  f.units = ot::Units::Mi;
  f.linked = true;  // a watch is connected
  f.t = esp_timer_get_time();
  if (!disp::begin(f)) {
    std::fprintf(stderr, "render_screens: disp::begin failed\n%s", emu::serialOut.c_str());
    return 1;
  }
  bool ok = true;

  // 1. Running: 6.0 mph (10:00 /mi), 1.84 mi, 18:22 moving of 19:05.
  f.state = ot::WorkoutState::Running;
  f.speedMmps = 2682;
  f.wkPulses = pulsesFor(2965.0);
  f.movingUs = (18 * 60 + 22) * 1000000LL;
  f.totalUs = (19 * 60 + 5) * 1000000LL;
  run(f, 6000000);
  ok &= capture(out, "screen_running", scale);

  // 2. Hold to stop, 60 % of the 2 s hold (the belt has stopped: PAUSED).
  f.state = ot::WorkoutState::Paused;
  f.speedMmps = 0;
  f.wkPulses = pulsesFor(3300.0);
  f.movingUs = (20 * 60 + 31) * 1000000LL;
  f.totalUs = (21 * 60 + 40) * 1000000LL;
  run(f, 3000000);
  for (holdPermille = 0; holdPermille <= 600; holdPermille += 10) run(f, 20000);
  run(f, 200000);
  ok &= capture(out, "screen_hold_to_stop", scale);
  holdPermille = -1;

  // 3. Summary of that run: 2.05 mi in 20:31 moving (avg 6.0 mph), top 7.5 mph.
  f.state = ot::WorkoutState::Summary;
  f.hasSummary = true;
  f.summary.workoutId = 1;
  f.summary.pulses = f.wkPulses;
  f.summary.movingUs = f.movingUs;
  f.summary.totalUs = (21 * 60 + 42) * 1000000LL;
  f.summary.topMmps = 3353;
  f.summary.avgMmps = static_cast<int32_t>(
      (static_cast<int64_t>(f.summary.pulses) * ot::M_PER_PULSE_UM * 1000 + f.summary.movingUs / 2) /
      f.summary.movingUs);
  f.summary.endReason = ot::EndReason::HoldStop;
  f.summary.summaryStartUs = emu::gNow - 8000000;  // entered 8 s ago
  f.summaryUs = ot::SUMMARY_US;
  run(f, 2000000);
  ok &= capture(out, "screen_summary", scale);

  return ok ? 0 : 1;
}
