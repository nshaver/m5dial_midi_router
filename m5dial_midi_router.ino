#include <M5Dial.h>
#include <NimBLEDevice.h>
#include <lvgl.h>
#include <vector>

// ============================================================
// BLE MIDI
// ============================================================

#define MIDI_SERVICE_UUID "03b80e5a-ede8-4b33-a751-6ce34ec4c700"

// ============================================================
// Hardware MIDI / SAM2695 Synth Port
// ============================================================

#define MIDI_PORTB_RX 1
#define MIDI_PORTB_TX 2
#define MIDI_BAUD 31250


// ============================================================
// Diagnostics
// ============================================================

#define DEBUG_RAW_BLE        0
#define DEBUG_NOTE_MESSAGES  0
#define DEBUG_PARSER_ERRORS  1
#define DEBUG_TX_OVERFLOW    1

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
int selectedSynthPatchIndex = 0;

// ============================================================
// MIDI TX Ring Buffer
// ============================================================

#define MIDI_TX_BUFFER_SIZE 2048

uint8_t midiTxBuffer[MIDI_TX_BUFFER_SIZE];

volatile size_t txHead = 0;
volatile size_t txTail = 0;
volatile uint32_t midiTxDropped = 0;

portMUX_TYPE midiBufferMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// BLE / Application State
// ============================================================

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
long oldEncoderPosition = 0;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pMidiChar = nullptr;

bool isConnected = false;

// ============================================================
// BLE MIDI Parser State
// ============================================================

bool bleMidiInSysEx = false;

// ============================================================
// MIDI Activity Indicator State
// ============================================================

unsigned long lastMidiActivityTime = 0;
const unsigned long midiLedDurationMs = 40;
bool midiLedState = false;

// ============================================================
// LVGL UI Widgets
// ============================================================

lv_obj_t* patchLabel = nullptr;
lv_obj_t* bleLine1Label = nullptr;
lv_obj_t* bleLine2Label = nullptr;
lv_obj_t* midiIndicatorObj = nullptr;
lv_obj_t* headerLabel = nullptr;

// ============================================================
// Function Prototypes
// ============================================================

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
// LVGL Display Flush Callback
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

// ============================================================
// Local SAM2695 Direct Control
// ============================================================

void sendLocalSynthProgramChange(uint8_t program) {
  Serial2.write(0xC0); // Program Change on Channel 1
  Serial2.write(program);
  Serial.printf("[Synth] Local Program Change sent: PC %u (%s)\n", 
                program, gmInstruments[selectedSynthPatchIndex].name);
}

// ============================================================
// MIDI TX BUFFER
// ============================================================

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
    size_t tail;
    size_t head;

    portENTER_CRITICAL(&midiBufferMux);
    tail = txTail;
    head = txHead;
    portEXIT_CRITICAL(&midiBufferMux);

    if (tail == head) break;
    if (Serial2.availableForWrite() <= 0) break;

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
  if (!isConnected || !midiIndicatorObj) return;

  if (midiLedState) {
    lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x00FF00), 0);
  } else {
    lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x000000), 0);
  }
}

// ============================================================
// BLE PARSER & CALLBACKS
// ============================================================

void parseBleMidiPacket(uint8_t* pData, size_t length) {
  if (length < 2) return;
  uint8_t header = pData[0];
  if ((header & 0x80) == 0) return;

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

    while (i < length && pData[i] >= 0xF8) {
      queueMidiByte(pData[i++]);
    }
    if (i >= length) break;

    uint8_t b = pData[i];
    if (b & 0x80) {
      runningStatus = b;
      i++;
    } else {
      if (runningStatus == 0) { i++; continue; }
    }

    if (runningStatus >= 0xF8) {
      queueMidiByte(runningStatus);
      runningStatus = 0;
      continue;
    }

    uint8_t statusType = runningStatus & 0xF0;
    int dataLen = 0;

    if (statusType == 0x80 || statusType == 0x90 || statusType == 0xA0 || statusType == 0xB0 || statusType == 0xE0) {
      dataLen = 2;
    } else if (statusType == 0xC0 || statusType == 0xD0) {
      dataLen = 1;
    } else if (runningStatus == 0xF1 || runningStatus == 0xF3) {
      dataLen = 1;
    } else if (runningStatus == 0xF2) {
      dataLen = 2;
    } else if (runningStatus == 0xF6) {
      dataLen = 0;
    } else {
      runningStatus = 0;
      continue;
    }

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
  if (!isNotify || !pData || length == 0) return;
  parseBleMidiPacket(pData, length);
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

  // Force LVGL to render the "Connecting" frame immediately before blocking operations
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
  const std::vector<NimBLERemoteCharacteristic*>& pChars = pService->getCharacteristics(true);
  for (auto pCh : pChars) {
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
  updateUI();
}

// ============================================================
// UI Layout & Updates (LVGL)
// ============================================================

void setupUI() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  // Top Title Label
  headerLabel = lv_label_create(scr);
  lv_label_set_text(headerLabel, "MIDI Router");
  lv_obj_set_style_text_color(headerLabel, lv_color_hex(0x00FFFF), 0);
  lv_obj_set_style_text_font(headerLabel, &lv_font_montserrat_14, 0);
  lv_obj_align(headerLabel, LV_ALIGN_TOP_MID, 0, 15);

  // Center Patch Label
  patchLabel = lv_label_create(scr);
  lv_obj_set_style_text_color(patchLabel, lv_color_hex(0xFF00FF), 0);
  lv_obj_set_style_text_font(patchLabel, &lv_font_montserrat_14, 0);
  lv_obj_align(patchLabel, LV_ALIGN_CENTER, 0, -5);

  // Divider line separating upper patch area and bottom 1/3 BLE zone
  lv_obj_t* line = lv_line_create(scr);
  static lv_point_t line_points[] = {{40, 150}, {200, 150}};
  lv_obj_set_style_line_color(line, lv_color_hex(0x555555), 0);
  lv_obj_set_style_line_width(line, 2, 0);
  lv_line_set_points(line, line_points, 2);

  // Bottom 1/3 BLE Line 1
  bleLine1Label = lv_label_create(scr);
  lv_obj_set_style_text_font(bleLine1Label, &lv_font_montserrat_14, 0);
  lv_obj_align(bleLine1Label, LV_ALIGN_TOP_MID, 0, 162);

  // Bottom 1/3 BLE Line 2
  bleLine2Label = lv_label_create(scr);
  lv_obj_set_style_text_font(bleLine2Label, &lv_font_montserrat_14, 0);
  lv_obj_align(bleLine2Label, LV_ALIGN_TOP_MID, 0, 188);

  // MIDI activity indicator dot/bar
  midiIndicatorObj = lv_obj_create(scr);
  lv_obj_set_size(midiIndicatorObj, 120, 10);
  lv_obj_align(midiIndicatorObj, LV_ALIGN_TOP_MID, 0, 216);
  lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x000000), 0);
  lv_obj_set_style_border_width(midiIndicatorObj, 0, 0);
  lv_obj_set_style_radius(midiIndicatorObj, 3, 0);

  updateUI();
}

void updateUI() {
  if (!patchLabel) return;

  // Update Patch Info
  String patchInfo = String(gmInstruments[selectedSynthPatchIndex].programNumber) + ": " + gmInstruments[selectedSynthPatchIndex].name;
  lv_label_set_text(patchLabel, patchInfo.c_str());
  lv_obj_align(patchLabel, LV_ALIGN_CENTER, 0, -5);

  // Update Bottom BLE Section
  if (currentState == STATE_IDLE) {
    lv_obj_set_style_text_color(bleLine1Label, lv_color_hex(0xFF0000), 0);
    lv_label_set_text(bleLine1Label, "BLE: Unconnected");
    lv_obj_align(bleLine1Label, LV_ALIGN_TOP_MID, 0, 162);

    lv_obj_set_style_text_color(bleLine2Label, lv_color_hex(0xFFFF00), 0);
    lv_label_set_text(bleLine2Label, "Touch to Scan");
    lv_obj_align(bleLine2Label, LV_ALIGN_TOP_MID, 0, 188);

    lv_obj_set_style_bg_color(midiIndicatorObj, lv_color_hex(0x000000), 0);
  }
  else if (currentState == STATE_SCANNING) {
    if (midiDeviceList.empty()) {
      lv_obj_set_style_text_color(bleLine1Label, lv_color_hex(0xFFFF00), 0);
      lv_label_set_text(bleLine1Label, "SCANNING...");
      lv_obj_align(bleLine1Label, LV_ALIGN_TOP_MID, 0, 162);

      lv_obj_set_style_text_color(bleLine2Label, lv_color_hex(0xFFFFFF), 0);
      lv_label_set_text(bleLine2Label, "Searching...");
      lv_obj_align(bleLine2Label, LV_ALIGN_TOP_MID, 0, 188);
    } else {
      lv_obj_set_style_text_color(bleLine1Label, lv_color_hex(0xFFFFFF), 0);
      lv_label_set_text(bleLine1Label, ("Found: " + String((int)midiDeviceList.size()) + " devices").c_str());
      lv_obj_align(bleLine1Label, LV_ALIGN_TOP_MID, 0, 158);

      String displayName = midiDeviceList[selectedDeviceIndex].name;
      if (displayName.length() > 14) displayName = displayName.substring(0, 12) + "..";
      lv_obj_set_style_text_color(bleLine2Label, lv_color_hex(0x00FF00), 0);
      lv_label_set_text(bleLine2Label, displayName.c_str());
      lv_obj_align(bleLine2Label, LV_ALIGN_TOP_MID, 0, 182);
    }
  }
  else if (currentState == STATE_CONNECTING) {
    lv_obj_set_style_text_color(bleLine1Label, lv_color_hex(0x00FFFF), 0);
    lv_label_set_text(bleLine1Label, "Connecting:");
    lv_obj_align(bleLine1Label, LV_ALIGN_TOP_MID, 0, 162);

    String targetName = midiDeviceList[selectedDeviceIndex].name;
    if (targetName.length() > 14) targetName = targetName.substring(0, 12) + "..";
    lv_obj_set_style_text_color(bleLine2Label, lv_color_hex(0x00FF00), 0);
    lv_label_set_text(bleLine2Label, targetName.c_str());
    lv_obj_align(bleLine2Label, LV_ALIGN_TOP_MID, 0, 188);
  }
  else if (currentState == STATE_CONNECTED) {
    lv_obj_set_style_text_color(bleLine1Label, lv_color_hex(0x00FF00), 0);
    lv_label_set_text(bleLine1Label, "Connected");
    lv_obj_align(bleLine1Label, LV_ALIGN_TOP_MID, 0, 162);

    String activeName = midiDeviceList[selectedDeviceIndex].name;
    if (activeName.length() > 16) activeName = activeName.substring(0, 14) + "..";
    lv_obj_set_style_text_color(bleLine2Label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(bleLine2Label, activeName.c_str());
    lv_obj_align(bleLine2Label, LV_ALIGN_TOP_MID, 0, 188);

    updateMidiActivityIndicator();
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  // Latch power ON immediately via GPIO 46 to hold power when waking from button press
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);

  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5.Display.setRotation(1);

  Serial.begin(115200);
  delay(1000);

  Serial2.begin(MIDI_BAUD, SERIAL_8N1, MIDI_PORTB_RX, MIDI_PORTB_TX);
  Serial.println("[MIDI] SAM2695 synth port initialized at 31250 baud.");

  // Initialize LVGL
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
// LOOP
// ============================================================

void loop() {
  M5Dial.update();

  // Feed elapsed time to LVGL so it can process rendering/timers
  static uint32_t last_tick = 0;
  uint32_t current_tick = millis();
  lv_tick_inc(current_tick - last_tick);
  last_tick = current_tick;

  lv_timer_handler(); // Let LVGL render UI frames

  processMidiTxBuffer();

  if (requestUiUpdate) {
    requestUiUpdate = false;
    updateUI();
  }

  if (midiLedState && (millis() - lastMidiActivityTime >= midiLedDurationMs)) {
    midiLedState = false;
    updateMidiActivityIndicator();
  }

  // ==========================================================
  // Long Press Button Handler for Power Off
  // ==========================================================
  static unsigned long btnPressStartTime = 0;
  static bool isHolding = false;

  if (M5Dial.BtnA.isPressed()) {
    if (!isHolding) {
      isHolding = true;
      btnPressStartTime = millis();
    } else {
      // Check if held for x seconds
      if (millis() - btnPressStartTime >= 5000) {
        Serial.println("[Power] 5-second button hold detected. Shutting down...");
        
        // Visual cue on screen before dying
        lv_label_set_text(patchLabel, "SHUTTING DOWN");
        lv_obj_set_style_text_color(patchLabel, lv_color_hex(0xFF0000), 0);
        lv_timer_handler();
        delay(500);

        // Cut power via M5 Power management API
        M5.Power.powerOff();
      }
    }
  } else {
    if (isHolding) {
      isHolding = false; // Reset if released before x seconds
    }
  }

  // Encoder Handling: Always controls SAM2695 Patch Selection unless scanning
  long newPos = M5Dial.Encoder.read() / 4;

  if (newPos != oldEncoderPosition) {
    if (currentState == STATE_SCANNING && !midiDeviceList.empty()) {
      if (newPos > oldEncoderPosition) {
        selectedDeviceIndex = (selectedDeviceIndex + 1) % midiDeviceList.size();
      } else {
        selectedDeviceIndex = (selectedDeviceIndex - 1 + midiDeviceList.size()) % midiDeviceList.size();
      }
      updateUI();
    } else {
      if (newPos > oldEncoderPosition) {
        selectedSynthPatchIndex = (selectedSynthPatchIndex + 1) % totalGmInstruments;
      } else {
        selectedSynthPatchIndex = (selectedSynthPatchIndex - 1 + totalGmInstruments) % totalGmInstruments;
      }
      sendLocalSynthProgramChange(gmInstruments[selectedSynthPatchIndex].programNumber);
      updateUI();
    }
    oldEncoderPosition = newPos;
  }

  // Native Touch Actions (Bottom 1/3 Region = BLE Control) & Button Fallback
  auto touch = M5.Touch.getDetail();
  bool touchBottom = touch.wasPressed() && (touch.y >= 150);
  bool btnTriggered = M5Dial.BtnA.wasClicked();

  if (touchBottom || btnTriggered) {
    if (currentState == STATE_IDLE) {
      currentState = STATE_SCANNING;
      startScan();
    }
    else if (currentState == STATE_SCANNING && !midiDeviceList.empty()) {
      currentState = STATE_CONNECTING;
      updateUI();
      connectToDevice(midiDeviceList[selectedDeviceIndex].address);
    }
    else if (currentState == STATE_CONNECTED) {
      disconnectDevice();
    }
  }

  delay(1);
}
