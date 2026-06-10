#include "Uc8279Driver.h"

#include <BoardConfig.h>

#include "../lut/Uc8279Luts.h"

namespace freeink {
namespace {
// UC8279 command set (UC8xxx family; same base opcodes as the UC8253 panels, with
// a 4-byte resolution and a gate-scan-mode register).
constexpr uint8_t CMD_PANEL_SETTING = 0x00;
constexpr uint8_t CMD_POWER_SETTING = 0x01;
constexpr uint8_t CMD_POWER_OFF = 0x02;
constexpr uint8_t CMD_POWER_ON = 0x04;
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x06;
constexpr uint8_t CMD_DEEP_SLEEP = 0x07;
constexpr uint8_t CMD_DTM1 = 0x10;  // OLD-data RAM
constexpr uint8_t CMD_DISPLAY_REFRESH = 0x12;
constexpr uint8_t CMD_DTM2 = 0x13;  // NEW-data RAM
constexpr uint8_t CMD_LUT_VCOM = 0x20;
constexpr uint8_t CMD_LUT_WW = 0x21;
constexpr uint8_t CMD_LUT_BW = 0x22;
constexpr uint8_t CMD_LUT_WB = 0x23;
constexpr uint8_t CMD_LUT_BB = 0x24;
constexpr uint8_t CMD_PLL_CONTROL = 0x30;
constexpr uint8_t CMD_VCOM_DATA_INTERVAL = 0x50;
constexpr uint8_t CMD_RESOLUTION = 0x61;
constexpr uint8_t CMD_GATE_SCAN_START = 0x65;
constexpr uint8_t CMD_VCOM_DC = 0x82;
constexpr uint8_t CMD_GATE_SCAN_MODE = 0xE1;
}  // namespace

const Uc8279Config& uc8279DefaultConfig() {
  static const Uc8279Config cfg = {
      {UC8279_LUT_20_GC, UC8279_LUT_21_GC, UC8279_LUT_22_GC, UC8279_LUT_23_GC, UC8279_LUT_24_GC},
      {UC8279_LUT_20_DU, UC8279_LUT_21_DU, UC8279_LUT_22_DU, UC8279_LUT_23_DU, UC8279_LUT_24_DU},
      UC8279_LUT_LEN,  // 42
      8,               // promote FAST -> full every 8 refreshes
      0x97,            // CDI for the GC waveform
      0xD7,            // CDI for the DU waveform
  };
  return cfg;
}

Uc8279Driver::Uc8279Driver(const Uc8279Config& cfg)
    : _cfg(cfg),
      _fbW(BoardConfig::ACTIVE.displayWidth),
      _fbH(BoardConfig::ACTIVE.displayHeight),
      _fbWb(BoardConfig::ACTIVE.displayWidth / 8) {}

uint32_t Uc8279Driver::spiHz() const {
  return BoardConfig::ACTIVE.displaySpiHz != 0 ? BoardConfig::ACTIVE.displaySpiHz : 10000000;
}

PanelGeometry Uc8279Driver::geometry() const {
  return {_fbW, _fbH, _fbWb, static_cast<uint32_t>(_fbWb) * _fbH};
}

void Uc8279Driver::loadLut(EpdBus& bus, const Uc8279LutBank& bank, uint8_t cdi) {
  bus.cmd(CMD_VCOM_DATA_INTERVAL);
  bus.data(cdi);
  bus.cmdData(CMD_LUT_VCOM, bank.vcom, _cfg.lutLen);
  bus.cmdData(CMD_LUT_WW, bank.ww, _cfg.lutLen);
  bus.cmdData(CMD_LUT_BW, bank.bw, _cfg.lutLen);
  bus.cmdData(CMD_LUT_WB, bank.wb, _cfg.lutLen);
  bus.cmdData(CMD_LUT_BB, bank.bb, _cfg.lutLen);
}

void Uc8279Driver::initController(EpdBus& bus) {
  // Vendor-reference UC8279 init (EEGO scaffold). TODO(bin): verify against the
  // stock firmware; in particular the TRES/VCOM/booster values.
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(0x3F);
  bus.data(0x4A);
  bus.cmd(CMD_POWER_SETTING);
  bus.data(0x03);
  bus.data(0x00);
  bus.data(0x78);
  bus.data(0x78);
  bus.data(0x17);
  bus.cmd(CMD_BOOSTER_SOFT_START);
  bus.data(0x25);
  bus.data(0x25);
  bus.data(0x3C);
  bus.cmd(CMD_VCOM_DC);
  bus.data(0x24);
  bus.cmd(CMD_PLL_CONTROL);
  bus.data(0x0F);  // ~80 Hz
  bus.cmd(CMD_RESOLUTION);
  bus.data(0x02);  // HRES = 0x0228 = 552
  bus.data(0x28);
  bus.data(0x03);  // VRES = 0x0300 = 768
  bus.data(0x00);
  bus.cmd(CMD_GATE_SCAN_START);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);
  bus.data(0x00);
  bus.cmd(CMD_GATE_SCAN_MODE);
  bus.data(0x02);

  bus.cmd(CMD_POWER_ON);
  bus.waitBusy(" EEGO_PON");
  _isScreenOn = true;

  // Seed both planes white so the first refresh diffs against white, not garbage.
  bus.fillPlane(CMD_DTM1, 0xFF, _fbH, _fbWb);
  bus.fillPlane(CMD_DTM2, 0xFF, _fbH, _fbWb);
}

void Uc8279Driver::begin(EpdBus& bus) {
  bus.reset(100);  // UC8279 wants a post-reset settle before init
  _isScreenOn = false;
  _fastRefreshCount = 0;
  initController(bus);
}

void Uc8279Driver::triggerRefresh(EpdBus& bus, bool turnOff) {
  if (!_isScreenOn) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" EEGO_PON");
    _isScreenOn = true;
  }
  bus.cmd(CMD_DISPLAY_REFRESH);
  bus.waitBusy(" EEGO_DRF");
  if (turnOff) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" EEGO_POF");
    _isScreenOn = false;
  }
}

void Uc8279Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  (void)prev;  // B/W path writes the current frame to both planes; no software prev buffer

  // FAST keeps only the destination-drive phase; ghosting accumulates, so promote
  // to a full (ghost-clearing) refresh every ghostClearInterval refreshes.
  bool useFast = (mode == RefreshMode::Fast);
  if (useFast) {
    if (_cfg.ghostClearInterval != 0 && _fastRefreshCount >= _cfg.ghostClearInterval) {
      useFast = false;
      _fastRefreshCount = 0;
    } else {
      _fastRefreshCount++;
    }
  } else {
    _fastRefreshCount = 0;
  }

  loadLut(bus, useFast ? _cfg.fast : _cfg.def, useFast ? _cfg.cdiDu : _cfg.cdiGc);

  // Old==new on both planes -> the WW/BB waveforms drive every pixel to its target.
  // UC8xxx scans gates bottom-to-top, so each plane is streamed Y-flipped.
  bus.sendPlaneFlipped(CMD_DTM1, fb, _fbH, _fbWb);
  bus.sendPlaneFlipped(CMD_DTM2, fb, _fbH, _fbWb);

  triggerRefresh(bus, turnOff);
}

void Uc8279Driver::deepSleep(EpdBus& bus) {
  if (_isScreenOn) {
    bus.cmd(CMD_POWER_OFF);
    bus.waitBusy(" EEGO power-down");
    _isScreenOn = false;
  }
  bus.cmd(CMD_DEEP_SLEEP);
  bus.data(0xA5);
}

// Per-board waveform injection mirrors the X3 / Murphy drivers: a board driving a
// different UC8279 panel supplies its own LUT banks via -DFREEINK_UC8279_CONFIG=
// yourConfig (e.g. the recovered EEGO stock LUTs) without editing this driver.
#ifdef FREEINK_UC8279_CONFIG
const Uc8279Config& FREEINK_UC8279_CONFIG();
static const Uc8279Config& uc8279ActiveConfig() { return FREEINK_UC8279_CONFIG(); }
#else
static const Uc8279Config& uc8279ActiveConfig() { return uc8279DefaultConfig(); }
#endif

PanelDriver& uc8279Driver() {
  static Uc8279Driver instance(uc8279ActiveConfig());
  return instance;
}

}  // namespace freeink
