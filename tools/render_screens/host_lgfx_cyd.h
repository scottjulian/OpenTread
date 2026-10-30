// Host stand-in for the CYD panel class (tools/render_screens): a 320x240
// RGB565 sprite that the real renderer draws into. Only the calls the
// renderer makes on its panel are provided; rotation is fixed (the sprite is
// already landscape), backlight and touch do nothing.
class LGFX_CYD : public lgfx::LGFX_Sprite {
 public:
  static constexpr const char* PANEL_NAME = "host";
  static constexpr uint32_t DMA_FLIP_BYTES = 2048;

  bool init() {
    setColorDepth(16);
    return createSprite(320, 240) != nullptr;
  }
  bool reserveDmaBuffers() { return true; }
  void setRotation(uint_fast8_t r) { rot_ = static_cast<uint8_t>(r); }
  uint8_t getRotation() const { return rot_; }
  void setBrightness(uint8_t) {}
  uint_fast8_t getTouchRaw(lgfx::touch_point_t*, uint_fast8_t = 1) { return 0; }
  void convertRawXY(lgfx::touch_point_t*, uint_fast8_t = 1) {}

 private:
  uint8_t rot_ = 0;
};
