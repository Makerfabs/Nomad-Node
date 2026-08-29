#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <FastLED.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <bluefruit.h>
#include <SPI.h>
#include <Wire.h>

using namespace Adafruit_LittleFS_Namespace;

#define pw_on_off 30
#define v33_on_off 16
#define V_RFSW 13

#define BUTTON_PIN 5
#define BUZZER_PIN 22

#define STATUS_LED_PIN 25
#define STATUS_LED_COUNT 1

#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27

#define GPS_RX_PIN 8
#define GPS_TX_PIN 6
#define GPS_BAUD_RATE 9600

#define LORA_CS_PIN 44
#define LORA_IRQ_PIN 14
#define LORA_BUSY_PIN 43
#define LORA_RESET_PIN 42
#define LORA_SPI_SCK_PIN 47
#define LORA_SPI_MISO_PIN 45
#define LORA_SPI_MOSI_PIN 46

static const float LORA_FREQUENCY_MHZ = 868.0F;
static const float LORA_BANDWIDTH_KHZ = 125.0F;
static const uint8_t LORA_SPREADING_FACTOR = 7;
static const uint8_t LORA_CODING_RATE = 5;
static const uint8_t LORA_SYNC_WORD = 0x12;
static const int8_t LORA_OUTPUT_POWER_DBM = 22;
static const uint16_t LORA_PREAMBLE_LENGTH = 8;
static const float LORA_TCXO_VOLTAGE = 1.8F;

static const uint32_t REPORT_INTERVAL_MS = 60000;
static const uint32_t LONG_PRESS_MS = 2000UL;
static const uint32_t DOUBLE_CLICK_WINDOW_MS = 450UL;
static const uint32_t BUTTON_DEBOUNCE_MS = 30UL;
static const uint8_t CW2015_ADDRESS = 0x62;

static const char MODE_FILE_PATH[] = "/tracker_mode.bin";
static const uint8_t MODE_FILE_MAGIC = 0xA6;

static const uint8_t PACKET_HEADER_HIGH = 0x5A;
static const uint8_t PACKET_HEADER_LOW = 0xA5;

static const size_t TRACKER_DATA_SIZE = 24U;
static const size_t TRACKER_PACKET_SIZE = 30U;

static const size_t PACKET_LENGTH_OFFSET = 2U;
static const size_t PACKET_ID_OFFSET = 4U;
static const size_t PACKET_SEQUENCE_OFFSET = 12U;
static const size_t PACKET_TIME_OFFSET = 16U;
static const size_t PACKET_LATITUDE_OFFSET = 20U;
static const size_t PACKET_LONGITUDE_OFFSET = 24U;
static const size_t PACKET_CRC_OFFSET = 28U;

enum DeviceMode : uint8_t {
  MODE_RECEIVER = 0,
  MODE_TRANSMITTER = 1
};

CRGB statusLeds[STATUS_LED_COUNT];
BLEUart bleUart;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
LR1121 radio = new Module(LORA_CS_PIN, LORA_IRQ_PIN, LORA_RESET_PIN, LORA_BUSY_PIN);

static const uint32_t rfSwitchPins[] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC,
  RADIOLIB_NC,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfSwitchTable[] = {
  {LR11x0::MODE_STBY, {LOW, LOW}},
  {LR11x0::MODE_RX, {HIGH, LOW}},
  {LR11x0::MODE_TX, {HIGH, HIGH}},
  {LR11x0::MODE_TX_HP, {LOW, HIGH}},
  {LR11x0::MODE_TX_HF, {LOW, LOW}},
  END_OF_MODE_TABLE
};

DeviceMode currentMode = MODE_RECEIVER;
bool fileSystemReady = false;
bool radioReady = false;
bool receiverListening = false;  // 仅在接收模式且手机已连接时为 true。
bool immediateReportPending = false;
uint32_t lastReportMs = 0;
bool lastRawButtonState = HIGH;
bool stableButtonState = HIGH;
uint32_t buttonDebounceStartMs = 0;
uint32_t buttonPressStartMs = 0;
bool longPressHandled = false;
uint8_t pendingClickCount = 0;
uint32_t firstClickReleaseMs = 0;
char deviceId[17] = {0};
uint64_t deviceIdValue = 0;
uint32_t reportSequence = 0;

void setupAdvertising();
void connectCallback(uint16_t connectionHandle);
void disconnectCallback(uint16_t connectionHandle, uint8_t reason);

void processGpsInput();
void processButton();
void registerShortClick(uint32_t nowMs);
void switchMode();
void applyCurrentMode(bool sendImmediately);
void setStatusLed(CRGB color);
void beep(uint16_t durationMs);
void shutdownDevice();
void loadSavedMode();
void saveCurrentMode();
void buildDeviceId();
bool readCw2015Register(uint8_t registerAddress, uint8_t &value);
void wakeBatteryGauge();
float readBatteryVoltage();
float readBatteryPercentage();
uint32_t buildGpsUnixTime();
int32_t daysFromCivil(int32_t year, uint32_t month, uint32_t day);
void writeUint32BigEndian(uint8_t *destination, uint32_t value);
void writeUint16BigEndian(uint8_t *destination, uint16_t value);
void writeUint64BigEndian(uint8_t *destination, uint64_t value);
uint16_t readUint16BigEndian(const uint8_t *source);
uint16_t calculateCrc16(const uint8_t *data, size_t length);
void buildDevicePacket(uint8_t *packet);
bool validateDevicePacket(const uint8_t *packet, size_t length);
void printPacketHex(const uint8_t *packet, size_t length);
void processTransmitterMode();
void transmitDeviceReport();
void processReceiverMode();
void restartLoRaReceiver();
void stopLoRaReceiver();
void forwardPacketToBle(const uint8_t *packet, size_t length);

void setup() {
  pinMode(pw_on_off, OUTPUT);
  digitalWrite(pw_on_off, HIGH);
  pinMode(v33_on_off, OUTPUT);
  digitalWrite(v33_on_off, HIGH);
  pinMode(V_RFSW, OUTPUT);
  digitalWrite(V_RFSW, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(115200);
  FastLED.addLeds<WS2812, STATUS_LED_PIN, GRB>(statusLeds, STATUS_LED_COUNT);
  setStatusLed(CRGB::Blue);
  beep(120);

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();
  wakeBatteryGauge();

  gpsSerial.begin(GPS_BAUD_RATE);
  buildDeviceId();
  fileSystemReady = InternalFS.begin();
  loadSavedMode();

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Nomad Node Tracker");
  bleUart.begin();
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);
  
  SPI.setPins(LORA_SPI_SCK_PIN, LORA_SPI_MISO_PIN, LORA_SPI_MOSI_PIN);
  SPI.begin();
  int radioState = radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE, LORA_SYNC_WORD, LORA_OUTPUT_POWER_DBM, LORA_PREAMBLE_LENGTH, LORA_TCXO_VOLTAGE);
  radioReady = radioState == RADIOLIB_ERR_NONE;
  if (radioReady) {
    radio.setRfSwitchTable(rfSwitchPins, rfSwitchTable);
    Serial.println("[LORA] LR1121 initialized");
  } else {
    Serial.print("[LORA] Initialization failed, code: ");
    Serial.println(radioState);
  }
  applyCurrentMode(true);
  Serial.print("[SYSTEM] Device ID: ");
  Serial.println(deviceId);
}

void loop() {
  processGpsInput();
  processButton();
  if (currentMode == MODE_TRANSMITTER) {
    processTransmitterMode();
  } else {
    processReceiverMode();
  }
  delay(2);
}

void processGpsInput() {
  while (gpsSerial.available() > 0) {
    gps.encode(static_cast<char>(gpsSerial.read()));
  }
}

void processButton() {
  uint32_t nowMs = millis();
  bool rawButtonState = digitalRead(BUTTON_PIN);
  if (rawButtonState != lastRawButtonState) {
    lastRawButtonState = rawButtonState;
    buttonDebounceStartMs = nowMs;
  }
  if ((uint32_t)(nowMs - buttonDebounceStartMs) >= BUTTON_DEBOUNCE_MS && stableButtonState != rawButtonState) {
    stableButtonState = rawButtonState;
    if (stableButtonState == LOW) {
      buttonPressStartMs = nowMs;
      longPressHandled = false;
    } else {
      uint32_t pressDurationMs = nowMs - buttonPressStartMs;
      if (!longPressHandled && pressDurationMs < LONG_PRESS_MS) {
        registerShortClick(nowMs);
      }
    }
  }
  if (stableButtonState == LOW && !longPressHandled && (uint32_t)(nowMs - buttonPressStartMs) >= LONG_PRESS_MS) {
    longPressHandled = true;
    pendingClickCount = 0;
    shutdownDevice();
  }
  if (pendingClickCount == 1 && (uint32_t)(nowMs - firstClickReleaseMs) > DOUBLE_CLICK_WINDOW_MS) {
    pendingClickCount = 0;
  }
}

void registerShortClick(uint32_t nowMs) {
  if (pendingClickCount == 0) {
    pendingClickCount = 1;
    firstClickReleaseMs = nowMs;
    return;
  }
  if ((uint32_t)(nowMs - firstClickReleaseMs) <= DOUBLE_CLICK_WINDOW_MS) {
    pendingClickCount = 0;
    switchMode();
  } else {
    pendingClickCount = 1;
    firstClickReleaseMs = nowMs;
  }
}

void switchMode() {
  currentMode = currentMode == MODE_RECEIVER ? MODE_TRANSMITTER : MODE_RECEIVER;
  saveCurrentMode();
  applyCurrentMode(true);
  beep(currentMode == MODE_RECEIVER ? 100 : 180);
}

void applyCurrentMode(bool sendImmediately) {
  if (currentMode == MODE_RECEIVER) {
    setStatusLed(CRGB::Blue);
    immediateReportPending = false;
    stopLoRaReceiver();
    setupAdvertising();
    Serial.println("[MODE] Receiver, blue LED, waiting for BLE connection");
  } else {
    setStatusLed(CRGB::Red);
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    uint16_t connectionHandles[4] = {0, 0, 0, 0};
    uint8_t connectionCount = Bluefruit.getConnectedHandles(connectionHandles, 4);
    for (uint8_t index = 0; index < connectionCount; index++) {
      Bluefruit.disconnect(connectionHandles[index]);
    }
    stopLoRaReceiver();
    lastReportMs = millis();
    immediateReportPending = sendImmediately;
    Serial.println("[MODE] Transmitter, red LED, BLE disabled, interval 5 minutes");
  }
}

void setStatusLed(CRGB color) {
  statusLeds[0] = color;
  FastLED.show();
}

void beep(uint16_t durationMs) {
  tone(BUZZER_PIN, 3000, durationMs);
}

void shutdownDevice() {
  Serial.println("[POWER] Long press detected, shutting down");
  beep(200);
  setStatusLed(CRGB::Black);
  Bluefruit.Advertising.stop();
  if (radioReady) {
    radio.standby();
  }
  digitalWrite(V_RFSW, LOW);
  digitalWrite(v33_on_off, LOW);
  delay(250);
  digitalWrite(pw_on_off, LOW);
  // while (true) {
  //   delay(1000);
  // }
}

void loadSavedMode() {
  currentMode = MODE_RECEIVER;
  if (!fileSystemReady) {
    Serial.println("[MODE] File system unavailable, using receiver default");
    return;
  }
  File modeFile(InternalFS);
  if (!modeFile.open(MODE_FILE_PATH, FILE_O_READ)) {
    Serial.println("[MODE] No saved mode, using receiver default");
    return;
  }
  uint8_t storedData[2] = {0, 0};
  size_t bytesRead = modeFile.read(storedData, sizeof(storedData));
  modeFile.close();
  if (bytesRead == sizeof(storedData) && storedData[0] == MODE_FILE_MAGIC && storedData[1] <= MODE_TRANSMITTER) {
    currentMode = static_cast<DeviceMode>(storedData[1]);
    Serial.print("[MODE] Loaded saved mode: ");
    Serial.println(currentMode == MODE_RECEIVER ? "Receiver" : "Transmitter");
  } else {
    Serial.println("[MODE] Invalid saved mode, using receiver default");
  }
}

void saveCurrentMode() {
  if (!fileSystemReady) {
    return;
  }
  File modeFile(InternalFS);
  if (InternalFS.exists(MODE_FILE_PATH)) {
    InternalFS.remove(MODE_FILE_PATH);
  }
  if (!modeFile.open(MODE_FILE_PATH, FILE_O_WRITE)) {
    Serial.println("[MODE] Failed to save mode");
    return;
  }
  uint8_t storedData[2] = {MODE_FILE_MAGIC, static_cast<uint8_t>(currentMode)};
  size_t bytesWritten = modeFile.write(storedData, sizeof(storedData));
  modeFile.close();
  if (bytesWritten != sizeof(storedData)) {
    Serial.println("[MODE] Failed to write complete mode record");
    return;
  }
}

void buildDeviceId() {
  deviceIdValue = static_cast<uint64_t>(NRF_FICR->DEVICEID[1]) << 32 | static_cast<uint64_t>(NRF_FICR->DEVICEID[0]);
  snprintf(deviceId, sizeof(deviceId), "%08lX%08lX", static_cast<unsigned long>(NRF_FICR->DEVICEID[1]), static_cast<unsigned long>(NRF_FICR->DEVICEID[0]));
}

bool readCw2015Register(uint8_t registerAddress, uint8_t &value) {
  Wire.beginTransmission(CW2015_ADDRESS);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(CW2015_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

void wakeBatteryGauge() {
  Wire.beginTransmission(CW2015_ADDRESS);
  Wire.write(static_cast<uint8_t>(0x0A));
  Wire.write(static_cast<uint8_t>(0x00));
  Wire.endTransmission();
}

float readBatteryVoltage() {
  uint8_t mostSignificantByte = 0;
  uint8_t leastSignificantByte = 0;
  if (!readCw2015Register(0x02, mostSignificantByte) || !readCw2015Register(0x03, leastSignificantByte)) {
    return -1.0F;
  }
  uint16_t rawVoltage = static_cast<uint16_t>(mostSignificantByte) << 8 | leastSignificantByte;
  return static_cast<float>(rawVoltage) * 305.0F / 1000000.0F;
}

float readBatteryPercentage() {
  uint8_t integerPercentage = 0;
  uint8_t fractionalPercentage = 0;
  if (!readCw2015Register(0x04, integerPercentage) || !readCw2015Register(0x05, fractionalPercentage)) {
    return -1.0F;
  }
  return static_cast<float>(integerPercentage) + static_cast<float>(fractionalPercentage) / 256.0F;
}

int32_t daysFromCivil(int32_t year, uint32_t month, uint32_t day) {
  year -= month <= 2U ? 1 : 0;
  int32_t era = (year >= 0 ? year : year - 399) / 400;
  uint32_t yearOfEra = static_cast<uint32_t>(year - era * 400);
  int32_t shiftedMonth = static_cast<int32_t>(month) + (month > 2U ? -3 : 9);
  uint32_t dayOfYear = (153U * static_cast<uint32_t>(shiftedMonth) + 2U) / 5U + day - 1U;
  uint32_t dayOfEra = yearOfEra * 365U + yearOfEra / 4U - yearOfEra / 100U + dayOfYear;
  return era * 146097 + static_cast<int32_t>(dayOfEra) - 719468;
}

uint32_t buildGpsUnixTime() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return 0U;
  }
  int32_t days = daysFromCivil(static_cast<int32_t>(gps.date.year()), gps.date.month(), gps.date.day());
  if (days < 0) {
    return 0U;
  }
  uint64_t seconds = static_cast<uint64_t>(days) * 86400ULL;
  seconds += static_cast<uint64_t>(gps.time.hour()) * 3600ULL;
  seconds += static_cast<uint64_t>(gps.time.minute()) * 60ULL;
  seconds += static_cast<uint64_t>(gps.time.second());
  return seconds > 0xFFFFFFFFULL ? 0U : static_cast<uint32_t>(seconds);
}

void writeUint32BigEndian(uint8_t *destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>((value >> 24) & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  destination[2] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  destination[3] = static_cast<uint8_t>(value & 0xFFU);
}

void writeUint16BigEndian(uint8_t *destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  destination[1] = static_cast<uint8_t>(value & 0xFFU);
}

uint16_t readUint16BigEndian(const uint8_t *source) {
  return static_cast<uint16_t>(static_cast<uint16_t>(source[0]) << 8U | source[1]);
}

void writeUint64BigEndian(uint8_t *destination, uint64_t value) {
  for (uint8_t index = 0; index < 8U; index++) {
    uint8_t shift = static_cast<uint8_t>((7U - index) * 8U);
    destination[index] = static_cast<uint8_t>((value >> shift) & 0xFFULL);
  }
}

uint16_t calculateCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t index = 0; index < length; index++) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8U; bit++) {
      crc = (crc & 0x8000U) != 0U ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void buildDevicePacket(uint8_t *packet) {
  memset(packet, 0, TRACKER_PACKET_SIZE);
  packet[0] = PACKET_HEADER_HIGH;
  packet[1] = PACKET_HEADER_LOW;
  writeUint16BigEndian(packet + PACKET_LENGTH_OFFSET, TRACKER_DATA_SIZE);
  writeUint64BigEndian(packet + PACKET_ID_OFFSET, deviceIdValue);
  writeUint32BigEndian(packet + PACKET_SEQUENCE_OFFSET, reportSequence++);
  writeUint32BigEndian(packet + PACKET_TIME_OFFSET, buildGpsUnixTime());
  bool gpsFixValid = gps.location.isValid() && gps.location.age() < 30000UL;
  int32_t latitudeMicrodegrees = gpsFixValid ? static_cast<int32_t>(gps.location.lat() * 1000000.0) : 0;
  int32_t longitudeMicrodegrees = gpsFixValid ? static_cast<int32_t>(gps.location.lng() * 1000000.0) : 0;
  writeUint32BigEndian(packet + PACKET_LATITUDE_OFFSET, static_cast<uint32_t>(latitudeMicrodegrees));
  writeUint32BigEndian(packet + PACKET_LONGITUDE_OFFSET, static_cast<uint32_t>(longitudeMicrodegrees));
  uint16_t packetCrc = calculateCrc16(packet + PACKET_LENGTH_OFFSET, PACKET_CRC_OFFSET - PACKET_LENGTH_OFFSET);
  packet[PACKET_CRC_OFFSET] = static_cast<uint8_t>((packetCrc >> 8) & 0xFFU);
  packet[PACKET_CRC_OFFSET + 1U] = static_cast<uint8_t>(packetCrc & 0xFFU);
}

bool validateDevicePacket(const uint8_t *packet, size_t length) {
  if (length != TRACKER_PACKET_SIZE) {
    return false;
  }
  if (packet[0] != PACKET_HEADER_HIGH || packet[1] != PACKET_HEADER_LOW) {
    return false;
  }
  if (readUint16BigEndian(packet + PACKET_LENGTH_OFFSET) != TRACKER_DATA_SIZE) {
    return false;
  }
  uint16_t receivedCrc = static_cast<uint16_t>(packet[PACKET_CRC_OFFSET]) << 8 | packet[PACKET_CRC_OFFSET + 1U];
  uint16_t calculatedCrc = calculateCrc16(packet + PACKET_LENGTH_OFFSET, PACKET_CRC_OFFSET - PACKET_LENGTH_OFFSET);
  return receivedCrc == calculatedCrc;
}

void printPacketHex(const uint8_t *packet, size_t length) {
  for (size_t index = 0; index < length; index++) {
    if (packet[index] < 0x10U) {
      Serial.print('0');
    }
    Serial.print(packet[index], HEX);
    Serial.print(index + 1U == length ? '\n' : ' ');
  }
}

void processTransmitterMode() {
  uint32_t nowMs = millis();
  if (!immediateReportPending && (uint32_t)(nowMs - lastReportMs) < REPORT_INTERVAL_MS) {
    return;
  }
  immediateReportPending = false;
  lastReportMs = nowMs;
  transmitDeviceReport();
}

void transmitDeviceReport() {
  if (!radioReady) {
    Serial.println("[TX] Radio unavailable");
    return;
  }
  uint8_t packet[TRACKER_PACKET_SIZE] = {0};
  buildDevicePacket(packet);
  radio.standby();
  radio.clearIrqFlags(RADIOLIB_LR11X0_IRQ_ALL);
  int transmitState = radio.transmit(packet, TRACKER_PACKET_SIZE);
  radio.standby();
  if (transmitState == RADIOLIB_ERR_NONE) {
    Serial.print("[TX] Binary packet: ");
    printPacketHex(packet, TRACKER_PACKET_SIZE);
  } else {
    Serial.print("[TX] Failed, code: ");
    Serial.println(transmitState);
  }
}

void processReceiverMode() {
  if (!radioReady || currentMode != MODE_RECEIVER) {
    return;
  }
  if (!Bluefruit.connected()) {
    if (receiverListening) {
      stopLoRaReceiver();
      setStatusLed(CRGB::Blue);
    }
    return;
  }
  if (!receiverListening) {
    return;
  }
  uint32_t irqStatus = radio.getIrqStatus();
  uint32_t terminalEvents = RADIOLIB_LR11X0_IRQ_RX_DONE | RADIOLIB_LR11X0_IRQ_CRC_ERR | RADIOLIB_LR11X0_IRQ_TIMEOUT | RADIOLIB_LR11X0_IRQ_HEADER_ERR;
  if ((irqStatus & terminalEvents) == 0) {
    return;
  }
  if ((irqStatus & RADIOLIB_LR11X0_IRQ_RX_DONE) != 0) {
    uint8_t packet[TRACKER_PACKET_SIZE] = {0};
    int receiveState = radio.readData(packet, TRACKER_PACKET_SIZE);
    if (receiveState == RADIOLIB_ERR_NONE && validateDevicePacket(packet, TRACKER_PACKET_SIZE)) {
      float receivedRssi = radio.getRSSI();
      float receivedSnr = radio.getSNR();
      Serial.print("[RX] RSSI ");
      Serial.print(receivedRssi, 1);
      Serial.print(" dBm, SNR ");
      Serial.print(receivedSnr, 1);
      Serial.print(" dB, binary packet: ");
      printPacketHex(packet, TRACKER_PACKET_SIZE);
      forwardPacketToBle(packet, TRACKER_PACKET_SIZE);
    } else if (receiveState != RADIOLIB_ERR_NONE) {
      Serial.print("[RX] Read failed, code: ");
      Serial.println(receiveState);
    } else {
      Serial.println("[RX] Tracker frame header, length, or CRC16 invalid");
    }
  } else {
    Serial.print("[RX] Radio event: 0x");
    Serial.println(irqStatus, HEX);
  }
  restartLoRaReceiver();
}

void restartLoRaReceiver() {
  if (!radioReady || currentMode != MODE_RECEIVER || !Bluefruit.connected()) {
    receiverListening = false;
    return;
  }
  radio.standby();
  radio.clearIrqFlags(RADIOLIB_LR11X0_IRQ_ALL);
  int receiveState = radio.startReceive();
  if (receiveState != RADIOLIB_ERR_NONE) {
    receiverListening = false;
    Serial.print("[RX] Restart failed, code: ");
    Serial.println(receiveState);
    return;
  }
  receiverListening = true;
}

void stopLoRaReceiver() {
  receiverListening = false;
  if (!radioReady) {
    return;
  }
  radio.standby();
  radio.clearIrqFlags(RADIOLIB_LR11X0_IRQ_ALL);
}

void forwardPacketToBle(const uint8_t *packet, size_t length) {
  if (!Bluefruit.connected()) {
    Serial.println("[BLE] No phone connected, live packet dropped");
    return;
  }
  uint16_t connectionHandle = Bluefruit.connHandle();
  BLEConnection *connection = Bluefruit.Connection(connectionHandle);
  if (connection == nullptr) {
    Serial.println("[BLE] Connection object unavailable");
    return;
  }
  uint16_t payloadSize = connection->getMtu() > 3 ? connection->getMtu() - 3 : 20;
  size_t offset = 0;
  while (offset < length) {
    size_t remainingBytes = length - offset;
    size_t chunkLength = remainingBytes < payloadSize ? remainingBytes : payloadSize;
    size_t writtenBytes = bleUart.write(connectionHandle, packet + offset, chunkLength);
    if (writtenBytes != chunkLength) {
      Serial.print("[BLE] Fragment send failed, bytes: ");
      Serial.println(writtenBytes);
      return;
    }
    offset += chunkLength;
    delay(10);
  }
  Serial.println("[BLE] Packet forwarded");
}

void connectCallback(uint16_t connectionHandle) {
  BLEConnection *connection = Bluefruit.Connection(connectionHandle);
  char peerName[32] = {0};
  if (connection != nullptr) {
    connection->getPeerName(peerName, sizeof(peerName));
  }
  Serial.print("[BLE] Connected: ");
  Serial.println(peerName);
  if (currentMode == MODE_RECEIVER) {
    restartLoRaReceiver();
    setStatusLed(receiverListening ? CRGB::Green : CRGB::Blue);
  }
}

void disconnectCallback(uint16_t connectionHandle, uint8_t reason) {
  (void)connectionHandle;
  Serial.print("[BLE] Disconnected, reason: 0x");
  Serial.println(reason, HEX);
  if (currentMode == MODE_RECEIVER && !Bluefruit.connected()) {
    stopLoRaReceiver();
    setStatusLed(CRGB::Blue);
  }
}

void setupAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}
