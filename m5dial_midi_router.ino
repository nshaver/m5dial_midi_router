#include <M5Dial.h>
#include <NimBLEDevice.h>
#include <lvgl.h>
#include <vector>

// ============================================================
// BLE MIDI & HARDWARE CONFIG
// ============================================================

#define MIDI_SERVICE_UUID "03b80e5a-ede8-4b33-a751-6ce34ec4c700"

#define MIDI_PORTB_RX 1
#define MIDI_PORTB_TX 2
#define MIDI_BAUD 31250

// ============================================================
// General MIDI Instrument Names
// ============================================================

struct GmInstrument {
  uint8_t programNumber;
  const char* name;
};

const GmInstrument gmInstruments[] = {
  {0, "Acoustic Grand"}, {1, "Bright Acoustic"}, {2, "Electric Grand"}, {3, "Honky-tonk Piano"},
  {4, "Electric Piano 1"}, {5, "Electric Piano 2"}, {6, "Harpsichord"}, {7, "Clavinet"},
  {8, "Celesta"}, {9, "Glockenspiel"}, {10, "Music Box"}, {11, "Vibraphone"},
  {12, "Marimba"}, {13, "Xylophone"}, {14, "Tubular Bells"}, {15, "Dulcimer"},
  {16, "Drawbar Organ"}, {17, "Percussive Organ"}, {18, "Rock Organ"}, {19, "Church Organ"},
  {20, "Reed Organ"}, {21, "Accordion"}, {22, "Harmonica"}, {23, "Tango Accordion"},
  {24, "Acoustic Guitar (nylon)"}, {25, "Acoustic Guitar (steel)"}, {26, "Electric Guitar (jazz)"}, {27, "Electric Guitar (clean)"},
  {28, "Electric Guitar (muted)"}, {29, "Overdriven Guitar"}, {30, "Distortion Guitar"}, {31, "Guitar Harmonics"},
  {32, "Acoustic Bass"}, {33, "Electric Bass (finger)"}, {34, "Electric Bass (pick)"}, {35, "Fretless Bass"},
  {36, "Slap Bass 1"}, {37, "Slap Bass 2"}, {38, "Synth Bass 1"}, {39, "Synth Bass 2"},
  {40, "Violin"}, {41, "Viola"}, {42, "Cello"}, {43, "Contrabass"},
  {44, "Tremolo Strings"}, {45, "Pizzicato Strings"}, {46, "Orchestral Harp"}, {47, "Timpani"},
  {48, "String Ensemble 1"}, {49, "String Ensemble 2"}, {50, "SynthStrings 1"}, {51, "SynthStrings 2"},
  {52, "Choir Aahs"}, {53, "Voice Oohs"}, {54, "Synth Voice"}, {55, "Orchestra Hit"},
  {56, "Trumpet"}, {57, "Trombone"}, {58, "Tuba"}, {59, "Muted Trumpet"},
  {60, "French Horn"}, {61, "Brass Section"}, {62, "SynthBrass 1"}, {63, "SynthBrass 2"},
  {64, "Soprano Sax"}, {65, "Alto Sax"}, {66, "Tenor Sax"}, {67, "Baritone Sax"},
  {68, "Oboe"}, {69, "English Horn"}, {70, "Bassoon"}, {71, "Clarinet"},
  {72, "Piccolo"}, {73, "Flute"}, {74, "Recorder"}, {75, "Pan Flute"},
  {76, "Blown Bottle"}, {77, "Shakuhachi"}, {78, "Whistle"}, {79, "Ocarina"},
  {80, "Lead 1 (square)"}, {81, "Lead 2 (sawtooth)"}, {82, "Lead 3 (calliope)"}, {83, "Lead 4 (chiff)"},
  {84, "Lead 5 (charang)"}, {85, "Lead 6 (voice)"}, {86, "Lead 7 (fifths)"}, {87, "Lead 8 (bass+lead)"},
  {88, "Pad 1 (new age)"}, {89, "Pad 2 (warm)"}, {90, "Pad 3 (polysynth)"}, {91, "Pad 4 (choir)"},
  {92, "Pad 5 (bowed)"}, {93, "Pad 6 (metallic)"}, {94, "Pad 7 (halo)"}, {95, "Pad 8 (sweep)"},
  {96, "FX 1 (rain)"}, {97, "FX 2 (soundtrack)"}, {98, "FX 3 (crystal)"}, {99, "FX 4 (atmosphere)"},
  {100, "FX 5 (brightness)"}, {101, "FX 6 (goblins)"}, {102, "FX 7 (echoes)"}, {103, "FX 8 (sci-fi)"},
  {104, "Sitar"}, {105, "Banjo"}, {106, "Shamisen"}, {107, "Koto"},
  {108, "Kalimba"}, {109, "Bagpipe"}, {110, "Fiddle"}, {111, "Shanai"},
  {112, "Tinkle Bell"}, {113, "Agogo"}, {114, "Steel Drums"}, {115, "Woodblock"},
  {116, "Taiko Drum"}, {117, "Melodic Tom"}, {118, "Synth Drum"}, {119, "Reverse Cymbal"},
  {120, "Guitar Fret Noise"}, {121, "Breath Noise"}, {122, "Seashore"}, {123, "Bird Tweet"},
  {124, "Telephone Ring"}, {125, "Helicopter"}, {126, "Applause"}, {127, "Gunshot"}
};
const int totalGmInstruments = sizeof(gmInstruments) / sizeof(GmInstrument);

// ============================================================
// SYSTEM & ROUTING STATE
// ============================================================

uint8_t targetMidiChannel = 1; // 1 to 16
int selectedSynthPatchIndex = 0;

enum AppState {
  STATE_IDLE,          
  STATE_SCANNING,
  STATE_CONNECTING,
  STATE_CONNECTED
};

AppState currentState = STATE_IDLE;
volatile bool requestUiUpdate = false;

struct MidiDevice {
  String name;
  NimBLEAddress address;
  int rssi;
};

std::vector<MidiDevice> midiDeviceList;
int selectedDeviceIndex = 0;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pMidiChar = nullptr;
bool isConnected = false;
bool bleMidiInSysEx = false;

// ============================================================
// MENU SYSTEM STATE
// ============================================================

enum MenuMode {
  MENU_MAIN,
  MENU_EDIT_CHANNEL,
  MENU_EDIT_PATCH,
  MENU_SELECT_DEVICE
};

MenuMode currentMenuMode = MENU_MAIN;
int currentMenuItem = 0; // 0: BLE Control, 1: MIDI Out Ch, 2: Synth Patch
const int totalMenuItems = 3;

long oldEncoderPosition = 0;

// ============================================================
// MIDI TX RING BUFFER
// ============================================================

#define MIDI_TX_BUFFER_SIZE 2048
uint8_t midiTxBuffer[MIDI_TX_BUFFER_SIZE];
volatile size_t txHead = 0;
volatile size_t txTail = 0;
volatile uint32_t midiTxDropped = 0;
portMUX_TYPE midiBufferMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastMidiActivityTime = 0;
const unsigned long midiLedDurationMs = 40;
bool midiLedState = false;

// ============================================================
// LVGL WIDGET REFERENCES
// ============================================================

// Top Status Half
lv_obj_t* statusBleLabel = nullptr;
lv_obj_t* statusChanLabel = nullptr;
lv_obj_t* statusPatchLabel = nullptr;
lv_obj_t* midiIndicatorObj = nullptr;

// Bottom Menu Half
lv_obj_t* menuItem0Obj = nullptr;
lv_obj_t* menuItem0Label = nullptr;
lv_obj_t* menuItem1Obj = nullptr;
lv_obj_t* menuItem1Label = nullptr;
lv_obj_t* menuItem2Obj = nullptr;
lv_obj_t* menuItem2Label = nullptr;

// Function Prototypes
void connectToDevice(NimBLEAddress addr);
void disconnectDevice();
void updateUI();
void startScan();
void clearMidiTxBuffer();
void processMidiTxBuffer();
bool queueMidiByte(uint8_t b);
void parseBleMidiPacket(uint8_t* pData, size_t length);
void queueMidiMessage(uint8_t status, const uint8_t* data, int dataLen);
void updateMidiActivityIndicator();
void sendLocalSynthProgramChange(uint8_t program);
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);

// ============================================================
// LVGL FLUSH & HARDWARE MIDI
// ============================================================

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  M5.Display.startWrite();
  M5.Display.setAddrWindow(area->x1, area->y1, w, h);
  M5.Display.writePixels((uint16_t*)color_p, w * h, true);
  M5.Display.endWrite();

  lv_disp_flush_ready(disp);
}

void sendLocalSynthProgramChange(uint8_t program) {
  uint8_t status = 0xC0 | ((targetMidiChannel - 1) & 0x0F);
  Serial2.write(status);
  Serial2.write(program);
  Serial.printf("[Synth] Program Change sent on Ch %u: PC %u (%s)\n", 
                targetMidiChannel, program, gmInstruments[selectedSynthPatchIndex].name);
}

bool queueMidiByte(uint8_t b) {
  bool queued = false;
  portENTER_CRITICAL(&midiBufferMux);
  size_t nextHead = (txHead + 1) % MIDI_TX_BUFFER_SIZE;
  if (nextHead != txTail) {
    midiTxBuffer[txHead] = b;
    txHead = nextHead;
    queued = true;
  } else {
    midiTxDropped++;
  }
  portEXIT_CRITICAL(&midiBufferMux);
  return queued;
}

void clearMidiTxBuffer() {
  portENTER_CRITICAL(&midiBufferMux);
  txHead = 0;
  txTail = 0;
  portEXIT_CRITICAL(&midiBufferMux);
}

void processMidiTxBuffer() {
  while (true) {
    size_t tail, head;
    portENTER_CRITICAL(&midiBufferMux);
    tail = txTail;
    head = txHead;
    portEXIT_CRITICAL(&midiBufferMux);

    if (tail == head || Serial2.availableForWrite() <= 0) break;

    uint8_t b;
    portENTER_CRITICAL(&midiBufferMux);
    if (txTail == txHead) {
      portEXIT_CRITICAL(&midiBufferMux);
      break;
    }
    b = midiTxBuffer[txTail];
    txTail = (txTail + 1) % MIDI_TX_BUFFER_SIZE;
    portEXIT_CRITICAL(&midiBufferMux);

    Serial2.write(b);
  }
}

void queueMidiMessage(uint8_t status, const uint8_t* data, int dataLen) {
  // Remap Channel Messages (0x80 - 0xEF) to targetMidiChannel
  if (status >= 0x80 && status < 0xF0) {
    status = (status & 0xF0) | ((targetMidiChannel - 1) & 0x0F);
  }

  queueMidiByte(status);
  for (int i = 0; i < dataLen; i++) {
    queueMidiByte(data[i]);
  }

  lastMidiActivityTime = millis();
  if (!midiLedState && isConnected) {
    midiLedState = true;
    updateMidiActivityIndicator();
  }
}

void updateMidiActivityIndicator() {
  if (!midiIndicatorObj) return;
  if (midiLedState && isConnected) {
    lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x00FF00), 0);
  } else {
    lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x222222), 0);
  }
}

// ============================================================
// BLE MIDI PARSER & CALLBACKS
// ============================================================

void parseBleMidiPacket(uint8_t* pData, size_t length) {
  if (length < 2) return;
  if ((pData[0] & 0x80) == 0) return;

  uint8_t runningStatus = 0;
  size_t i = 1;

  if (bleMidiInSysEx) {
    while (i < length) {
      uint8_t b = pData[i++];
      if (b >= 0xF8) { queueMidiByte(b); continue; }
      if (b == 0xF7) { queueMidiByte(b); bleMidiInSysEx = false; continue; }
      if ((b & 0x80) == 0) { queueMidiByte(b); continue; }
      bleMidiInSysEx = false;
      break;
    }
    return;
  }

  while (i < length) {
    if ((pData[i] & 0x80) != 0) { i++; }
    if (i >= length) break;

    while (i < length && pData[i] >= 0xF8) queueMidiByte(pData[i++]);
    if (i >= length) break;

    uint8_t b = pData[i];
    if (b & 0x80) {
      runningStatus = b;
      i++;
    } else if (runningStatus == 0) {
      i++; continue;
    }

    if (runningStatus >= 0xF8) {
      queueMidiByte(runningStatus);
      runningStatus = 0;
      continue;
    }

    uint8_t statusType = runningStatus & 0xF0;
    int dataLen = 0;

    if (statusType == 0x80 || statusType == 0x90 || statusType == 0xA0 || statusType == 0xB0 || statusType == 0xE0) dataLen = 2;
    else if (statusType == 0xC0 || statusType == 0xD0 || runningStatus == 0xF1 || runningStatus == 0xF3) dataLen = 1;
    else if (runningStatus == 0xF2) dataLen = 2;
    else if (runningStatus == 0xF6) dataLen = 0;
    else { runningStatus = 0; continue; }

    if (i + dataLen > length) break;

    bool validData = true;
    for (int d = 0; d < dataLen; d++) {
      if (pData[i + d] & 0x80) { validData = false; break; }
    }
    if (!validData) break;

    queueMidiMessage(runningStatus, &pData[i], dataLen);
    i += dataLen;
    if (runningStatus >= 0xF0) runningStatus = 0;
  }
}

void midiNotifyCallback(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (isNotify && pData && length > 0) parseBleMidiPacket(pData, length);
}

class MidiClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    Serial.println("[BLE] Status: Connected to MIDI device.");
  }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    Serial.printf("[BLE] Status: Disconnected (Reason: %d)\n", reason);
    isConnected = false;
    pMidiChar = nullptr;
    bleMidiInSysEx = false;
    clearMidiTxBuffer();
    currentState = STATE_IDLE;
    currentMenuMode = MENU_MAIN;
    requestUiUpdate = true;
  }
  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {}
};

class ScanCallbacks : public NimBLEScanCallbacks {
  void processDiscoveredDevice(const NimBLEAdvertisedDevice* advertisedDevice) {
    if (currentState != STATE_SCANNING) return;
    NimBLEAddress addr = advertisedDevice->getAddress();
    String devName = advertisedDevice->getName().c_str();

    bool isMidiDevice = advertisedDevice->isAdvertisingService(NimBLEUUID(MIDI_SERVICE_UUID)) ||
                        devName.indexOf("microKEY") >= 0 || devName.indexOf("MIDI") >= 0;
    if (!isMidiDevice) return;

    for (auto& dev : midiDeviceList) {
      if (dev.address == addr) {
        if (devName.length() > 0 && (dev.name.startsWith("<Pending") || dev.name != devName)) {
          dev.name = devName;
          requestUiUpdate = true;
        }
        return;
      }
    }

    MidiDevice newDev;
    newDev.name = devName.length() > 0 ? devName : "BLE MIDI Device";
    newDev.address = addr;
    newDev.rssi = advertisedDevice->getRSSI();
    midiDeviceList.push_back(newDev);
    requestUiUpdate = true;
  }
  void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override { processDiscoveredDevice(advertisedDevice); }
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override { processDiscoveredDevice(advertisedDevice); }
  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    if (currentState == STATE_SCANNING) NimBLEDevice::getScan()->start(5, false, false);
  }
} scanCallbacks;

void startScan() {
  NimBLEScan* pScan = NimBLEDevice::getScan();
  if (pScan->isScanning()) pScan->stop();
  pScan->clearResults();
  midiDeviceList.clear();
  selectedDeviceIndex = 0;
  pScan->start(5, false, false);
  currentState = STATE_SCANNING;
  currentMenuMode = MENU_SELECT_DEVICE;
  updateUI();
}

void connectToDevice(NimBLEAddress addr) {
  NimBLEScan* pScan = NimBLEDevice::getScan();
  if (pScan->isScanning()) pScan->stop();

  if (!pClient) {
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MidiClientCallbacks(), false);
  }

  currentState = STATE_CONNECTING;
  updateUI();
  lv_timer_handler();

  if (!pClient->connect(addr, true)) {
    currentState = STATE_SCANNING;
    startScan();
    return;
  }

  pClient->updateConnParams(6, 12, 0, 500);
  pClient->exchangeMTU();
  pClient->secureConnection();
  delay(1500);

  if (!pClient->discoverAttributes()) {
    pClient->disconnect();
    currentState = STATE_SCANNING;
    startScan();
    return;
  }

  NimBLERemoteService* pService = pClient->getService(NimBLEUUID(MIDI_SERVICE_UUID));
  if (!pService) {
    delay(500);
    pService = pClient->getService(NimBLEUUID(MIDI_SERVICE_UUID));
  }

  if (!pService) {
    pClient->disconnect();
    currentState = STATE_SCANNING;
    startScan();
    return;
  }

  pMidiChar = nullptr;
  for (auto pCh : pService->getCharacteristics(true)) {
    if (pCh->canNotify()) {
      pMidiChar = pCh;
      break;
    }
  }

  if (!pMidiChar || !pMidiChar->subscribe(true, midiNotifyCallback)) {
    pClient->disconnect();
    currentState = STATE_SCANNING;
    startScan();
    return;
  }

  bleMidiInSysEx = false;
  clearMidiTxBuffer();
  isConnected = true;
  currentState = STATE_CONNECTED;
  currentMenuMode = MENU_MAIN;
  updateUI();
}

void disconnectDevice() {
  NimBLEScan* pScan = NimBLEDevice::getScan();
  if (pScan->isScanning()) pScan->stop();
  if (pClient && pClient->isConnected()) pClient->disconnect();

  isConnected = false;
  pMidiChar = nullptr;
  bleMidiInSysEx = false;
  clearMidiTxBuffer();
  currentState = STATE_IDLE;
  currentMenuMode = MENU_MAIN;
  updateUI();
}

// ============================================================
// UI LAYOUT & STATUS / MENU SYSTEM (LVGL)
// ============================================================

void setupUI() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  // ----------------------------------------------------------
  // Top 1/2: Status Display Panel (Y: 0 to 119)
  // ----------------------------------------------------------
  statusBleLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(statusBleLabel, &lv_font_montserrat_14, 0);
  lv_obj_align(statusBleLabel, LV_ALIGN_TOP_MID, 0, 15);

  statusChanLabel = lv_label_create(scr);
  lv_obj_set_style_text_color(statusChanLabel, lv_color_hex(0x00FFFF), 0);
  lv_obj_set_style_text_font(statusChanLabel, &lv_font_montserrat_14, 0);
  lv_obj_align(statusChanLabel, LV_ALIGN_TOP_MID, 0, 40);

  statusPatchLabel = lv_label_create(scr);
  lv_obj_set_style_text_color(statusPatchLabel, lv_color_hex(0xFF00FF), 0);
  lv_obj_set_style_text_font(statusPatchLabel, &lv_font_montserrat_14, 0);
  lv_obj_align(statusPatchLabel, LV_ALIGN_TOP_MID, 0, 65);

  // MIDI activity indicator bar
  midiIndicatorObj = lv_obj_create(scr);
  lv_obj_set_size(midiIndicatorObj, 140, 6);
  lv_obj_align(midiIndicatorObj, LV_ALIGN_TOP_MID, 0, 95);
  lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x222222), 0);
  lv_obj_set_style_border_width(midiIndicatorObj, 0, 0);
  lv_obj_set_style_radius(midiIndicatorObj, 2, 0);

  // Center Divider Line
  lv_obj_t* line = lv_line_create(scr);
  static lv_point_t line_points[] = {{10, 108}, {230, 108}};
  lv_obj_set_style_line_color(line, lv_color_hex(0x444444), 0);
  lv_obj_set_style_line_width(line, 2, 0);
  lv_line_set_points(line, line_points, 2);

  // ----------------------------------------------------------
  // Bottom 1/2: Menu System Panel (Y: 110 to 240)
  // ----------------------------------------------------------
  menuItem0Obj = lv_obj_create(scr);
  lv_obj_set_size(menuItem0Obj, 210, 32);
  lv_obj_align(menuItem0Obj, LV_ALIGN_TOP_MID, 0, 115);
  lv_obj_set_style_radius(menuItem0Obj, 4, 0);
  menuItem0Label = lv_label_create(menuItem0Obj);
  lv_obj_set_style_text_font(menuItem0Label, &lv_font_montserrat_14, 0);
  lv_obj_center(menuItem0Label);

  menuItem1Obj = lv_obj_create(scr);
  lv_obj_set_size(menuItem1Obj, 210, 32);
  lv_obj_align(menuItem1Obj, LV_ALIGN_TOP_MID, 0, 152);
  lv_obj_set_style_radius(menuItem1Obj, 4, 0);
  menuItem1Label = lv_label_create(menuItem1Obj);
  lv_obj_set_style_text_font(menuItem1Label, &lv_font_montserrat_14, 0);
  lv_obj_center(menuItem1Label);

  menuItem2Obj = lv_obj_create(scr);
  lv_obj_set_size(menuItem2Obj, 210, 32);
  lv_obj_align(menuItem2Obj, LV_ALIGN_TOP_MID, 0, 189);
  lv_obj_set_style_radius(menuItem2Obj, 4, 0);
  menuItem2Label = lv_label_create(menuItem2Obj);
  lv_obj_set_style_text_font(menuItem2Label, &lv_font_montserrat_14, 0);
  lv_obj_center(menuItem2Label);

  updateUI();
}

void setMenuItemStyle(lv_obj_t* container, lv_obj_t* label, bool isSelected, bool isEditing, const char* text) {
  lv_label_set_text(label, text);
  lv_obj_center(label);

  if (isEditing) {
    lv_obj_set_style_bg_color(container, lv_color_hex(0xFF9900), 0); // Orange highlight when active editing
    lv_obj_set_style_text_color(label, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(container, 2, 0);
  } else if (isSelected) {
    lv_obj_set_style_bg_color(container, lv_color_hex(0x0066CC), 0); // Blue focus highlight
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(container, 0, 0);
  } else {
    lv_obj_set_style_bg_color(container, lv_color_hex(0x181818), 0); // Inactive dark item background
    lv_obj_set_style_text_color(label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_border_width(container, 0, 0);
  }
}

void updateUI() {
  if (!statusBleLabel) return;

  // 1. Refresh Top Status
  if (currentState == STATE_IDLE) {
    lv_obj_set_style_text_color(statusBleLabel, lv_color_hex(0xFF4444), 0);
    lv_label_set_text(statusBleLabel, "BLE: Disconnected");
  } else if (currentState == STATE_SCANNING) {
    lv_obj_set_style_text_color(statusBleLabel, lv_color_hex(0xFFFF00), 0);
    lv_label_set_text(statusBleLabel, "BLE: Scanning...");
  } else if (currentState == STATE_CONNECTING) {
    lv_obj_set_style_text_color(statusBleLabel, lv_color_hex(0x00FFFF), 0);
    lv_label_set_text(statusBleLabel, "BLE: Connecting...");
  } else if (currentState == STATE_CONNECTED) {
    String devName = midiDeviceList.empty() ? "Device" : midiDeviceList[selectedDeviceIndex].name;
    if (devName.length() > 14) devName = devName.substring(0, 12) + "..";
    lv_obj_set_style_text_color(statusBleLabel, lv_color_hex(0x00FF00), 0);
    lv_label_set_text(statusBleLabel, ("BLE: " + devName).c_str());
  }
  lv_obj_align(statusBleLabel, LV_ALIGN_TOP_MID, 0, 15);

  lv_label_set_text(statusChanLabel, ("MIDI Out Ch: " + String(targetMidiChannel)).c_str());
  lv_obj_align(statusChanLabel, LV_ALIGN_TOP_MID, 0, 40);

  String patchText = String(gmInstruments[selectedSynthPatchIndex].programNumber) + ": " + gmInstruments[selectedSynthPatchIndex].name;
  if (patchText.length() > 22) patchText = patchText.substring(0, 20) + "..";
  lv_label_set_text(statusPatchLabel, patchText.c_str());
  lv_obj_align(statusPatchLabel, LV_ALIGN_TOP_MID, 0, 65);

  // 2. Refresh Bottom Menu
  if (currentMenuMode == MENU_SELECT_DEVICE) {
    // Menu item 0 becomes device navigator during scanning
    if (midiDeviceList.empty()) {
      setMenuItemStyle(menuItem0Obj, menuItem0Label, true, false, "Searching...");
    } else {
      String dName = midiDeviceList[selectedDeviceIndex].name;
      if (dName.length() > 16) dName = dName.substring(0, 14) + "..";
      setMenuItemStyle(menuItem0Obj, menuItem0Label, true, false, ("> " + dName).c_str());
    }
    setMenuItemStyle(menuItem1Obj, menuItem1Label, false, false, "Cancel Scan");
    setMenuItemStyle(menuItem2Obj, menuItem2Label, false, false, "");
    return;
  }

  // General Main Menu Render
  const char* bleActionStr = isConnected ? "Disconnect BLE" : "Scan BLE MIDI";
  setMenuItemStyle(menuItem0Obj, menuItem0Label, (currentMenuItem == 0), false, bleActionStr);

  String chMenuStr = "MIDI Out Ch: < " + String(targetMidiChannel) + " >";
  setMenuItemStyle(menuItem1Obj, menuItem1Label, (currentMenuItem == 1), (currentMenuMode == MENU_EDIT_CHANNEL), chMenuStr.c_str());

  String pMenuStr = "Patch: < " + String(gmInstruments[selectedSynthPatchIndex].programNumber) + " >";
  setMenuItemStyle(menuItem2Obj, menuItem2Label, (currentMenuItem == 2), (currentMenuMode == MENU_EDIT_PATCH), pMenuStr.c_str());
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);

  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5.Display.setRotation(1);

  Serial.begin(115200);
  delay(1000);

  Serial2.begin(MIDI_BAUD, SERIAL_8N1, MIDI_PORTB_RX, MIDI_PORTB_TX);
  Serial.println("[MIDI] SAM2695 synth port initialized at 31250 baud.");

  lv_init();

  static lv_color_t buf[240 * 40];
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, 240 * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 240;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  setupUI();

  NimBLEDevice::init("M5Dial-Router");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->setMaxResults(0);

  currentState = STATE_IDLE;
  updateUI();
}

// ============================================================
// MAIN LOOP & NAVIGATION
// ============================================================

void triggerMenuAction(int item) {
  if (item == 0) { // BLE Action
    if (isConnected) {
      disconnectDevice();
    } else {
      startScan();
    }
  } else if (item == 1) { // Edit Out Channel
    currentMenuMode = (currentMenuMode == MENU_EDIT_CHANNEL) ? MENU_MAIN : MENU_EDIT_CHANNEL;
    updateUI();
  } else if (item == 2) { // Edit Program Change
    currentMenuMode = (currentMenuMode == MENU_EDIT_PATCH) ? MENU_MAIN : MENU_EDIT_PATCH;
    updateUI();
  }
}

void loop() {
  M5Dial.update();

  static uint32_t last_tick = 0;
  uint32_t current_tick = millis();
  lv_tick_inc(current_tick - last_tick);
  last_tick = current_tick;

  lv_timer_handler();
  processMidiTxBuffer();

  if (requestUiUpdate) {
    requestUiUpdate = false;
    updateUI();
  }

  if (midiLedState && (millis() - lastMidiActivityTime >= midiLedDurationMs)) {
    midiLedState = false;
    updateMidiActivityIndicator();
  }

  // Long press button to turn off power
  static unsigned long btnPressStartTime = 0;
  static bool isHolding = false;

  if (M5Dial.BtnA.isPressed()) {
    if (!isHolding) {
      isHolding = true;
      btnPressStartTime = millis();
    } else if (millis() - btnPressStartTime >= 5000) {
      Serial.println("[Power] 5-second button hold detected. Shutting down...");
      M5.Power.powerOff();
    }
  } else {
    isHolding = false;
  }

  // ----------------------------------------------------------
  // Encoder Input Processing
  // ----------------------------------------------------------
  long newPos = M5Dial.Encoder.read() / 4;

  if (newPos != oldEncoderPosition) {
    bool increment = (newPos > oldEncoderPosition);

    if (currentMenuMode == MENU_MAIN) {
      // Navigate Menu Items
      if (increment) currentMenuItem = (currentMenuItem + 1) % totalMenuItems;
      else currentMenuItem = (currentMenuItem - 1 + totalMenuItems) % totalMenuItems;
      updateUI();
    } 
    else if (currentMenuMode == MENU_EDIT_CHANNEL) {
      // Modify Target Channel
      if (increment) targetMidiChannel = (targetMidiChannel % 16) + 1;
      else targetMidiChannel = (targetMidiChannel == 1) ? 16 : targetMidiChannel - 1;
      sendLocalSynthProgramChange(gmInstruments[selectedSynthPatchIndex].programNumber);
      updateUI();
    } 
    else if (currentMenuMode == MENU_EDIT_PATCH) {
      // Modify Synth Patch
      if (increment) selectedSynthPatchIndex = (selectedSynthPatchIndex + 1) % totalGmInstruments;
      else selectedSynthPatchIndex = (selectedSynthPatchIndex - 1 + totalGmInstruments) % totalGmInstruments;
      sendLocalSynthProgramChange(gmInstruments[selectedSynthPatchIndex].programNumber);
      updateUI();
    } 
    else if (currentMenuMode == MENU_SELECT_DEVICE && !midiDeviceList.empty()) {
      // Navigate Scanned Devices
      if (increment) selectedDeviceIndex = (selectedDeviceIndex + 1) % midiDeviceList.size();
      else selectedDeviceIndex = (selectedDeviceIndex - 1 + midiDeviceList.size()) % midiDeviceList.size();
      updateUI();
    }

    oldEncoderPosition = newPos;
  }

  // ----------------------------------------------------------
  // Touch & Center Button Input Processing
  // ----------------------------------------------------------
  auto touch = M5.Touch.getDetail();
  bool touchMenuRegion = touch.wasPressed() && (touch.y >= 108);
  bool btnClicked = M5Dial.BtnA.wasClicked();

  if (btnClicked) {
    if (currentMenuMode == MENU_SELECT_DEVICE) {
      if (!midiDeviceList.empty()) {
        connectToDevice(midiDeviceList[selectedDeviceIndex].address);
      } else {
        disconnectDevice();
      }
    } else {
      triggerMenuAction(currentMenuItem);
    }
  } 
  else if (touchMenuRegion) {
    if (currentMenuMode == MENU_SELECT_DEVICE) {
      if (touch.y < 150 && !midiDeviceList.empty()) {
        connectToDevice(midiDeviceList[selectedDeviceIndex].address);
      } else if (touch.y >= 150) {
        disconnectDevice();
      }
    } else {
      // Touch mapping for menu items in lower half
      if (touch.y < 148) {
        currentMenuItem = 0;
        triggerMenuAction(0);
      } else if (touch.y >= 148 && touch.y < 185) {
        currentMenuItem = 1;
        triggerMenuAction(1);
      } else if (touch.y >= 185) {
        currentMenuItem = 2;
        triggerMenuAction(2);
      }
    }
  }

  delay(1);
}
