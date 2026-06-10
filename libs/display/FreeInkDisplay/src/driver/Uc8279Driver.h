#pragma once

// UC8279 panel driver — EEGO A4 (Topwin TWE0398NZ12, 552x768 B/W, ESP32-S3).
//
// UltraChip UC8279 is a UC8xxx dual-RAM controller, the same family as the X3 /
// Murphy UC8253: five 42-byte LUT banks (0x20..0x24), DTM1/DTM2 RAM planes
// (0x10/0x13), display refresh 0x12, active-low BUSY. This driver is modeled on
// Uc8253MurphyDriver; the panel is portrait-native (552x768) so — unlike Murphy —
// no 90° rotation is needed and planes stream straight via EpdBus::sendPlaneFlipped.
//
// The init / TRES / VCOM / LUT values are the UC8279 vendor-reference defaults.
// TODO(bin): verify every value against the EEGO stock firmware command stream,
// and replace the reference LUTs with the recovered stock waveforms.
//
// Selection: linked only when -DFREEINK_DRIVER_UC8279 (the EEGO board env).

#include "PanelDriver.h"

namespace freeink {

// One UC8279 waveform bank: the five LUT registers (0x20 VCOM .. 0x24 BB).
struct Uc8279LutBank {
  const uint8_t* vcom;  // 0x20
  const uint8_t* ww;    // 0x21
  const uint8_t* bw;    // 0x22
  const uint8_t* wb;    // 0x23
  const uint8_t* bb;    // 0x24
};

struct Uc8279Config {
  Uc8279LutBank def;           // full (GC) ghost-clearing waveform
  Uc8279LutBank fast;          // fast (DU) waveform
  uint8_t lutLen;              // bytes per LUT (42)
  uint8_t ghostClearInterval;  // promote FAST -> full every N refreshes
  uint8_t cdiGc;               // VCOM/data interval (cmd 0x50) for the GC waveform
  uint8_t cdiDu;               // ... for the DU waveform
};

const Uc8279Config& uc8279DefaultConfig();

class Uc8279Driver : public PanelDriver {
 public:
  explicit Uc8279Driver(const Uc8279Config& cfg = uc8279DefaultConfig());

  uint32_t spiHz() const override;
  // UC8279 BUSY is active-low and asserted only while working: wait for the
  // HIGH->LOW edge, then back to HIGH (the X3 two-phase wait).
  BusyPolarity busyPolarity() const override { return BusyPolarity::X3TwoPhase; }
  PanelGeometry geometry() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;

 private:
  void initController(EpdBus& bus);
  void loadLut(EpdBus& bus, const Uc8279LutBank& bank, uint8_t cdi);
  void triggerRefresh(EpdBus& bus, bool turnOff);

  const Uc8279Config& _cfg;

  uint16_t _fbW;   // framebuffer width  (552)
  uint16_t _fbH;   // framebuffer height (768)
  uint16_t _fbWb;  // framebuffer width bytes (69)

  bool _isScreenOn = false;
  uint8_t _fastRefreshCount = 0;
};

PanelDriver& uc8279Driver();

}  // namespace freeink
