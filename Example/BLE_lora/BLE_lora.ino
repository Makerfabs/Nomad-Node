/*
Arduino IDE V2.3.6
Additional boards manager URLs:https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
Board: nrf52 V1.6.1
RadioLib V7.5.0

Tools:
Board: "Nordic nRF52840 DK"


Download the “Serial Bluetooth Terminal” app on your phone to communicate with the device.
*/

#include <bluefruit.h>
#include <RadioLib.h>
#include <FastLED.h>

#define NUM_LEDS 1
#define LED_PIN 25

CRGB leds[NUM_LEDS];

BLEUart bleuart;

// ====================== LoRa Config ======================

#define FREQUENCY          868.0
#define BANDWIDTH          125.0
#define SPREADING_FACTOR   7
#define CODING_RATE        5
#define SYNCWORD           0x12
#define OUTPUT_POWER       22
#define PREAMBLE_LEN       8
#define TCXOVOLTAGE        1.8

#define LR1121_CS    44
#define LR1121_IRQ   14
#define LR1121_BUSY  43
#define LR1121_RST   42

LR1121 radio = new Module(
  LR1121_CS,
  LR1121_IRQ,
  LR1121_RST,
  LR1121_BUSY
);

#define pw_on_off     NRF_GPIO_PIN_MAP(0, 30)
#define v33_on_off    NRF_GPIO_PIN_MAP(0, 16)
#define V_RFSW        NRF_GPIO_PIN_MAP(0, 13)

#define BUZZER_PIN    22

static const uint32_t rfswitch_dio_pins[] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC,
  RADIOLIB_NC,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  { LR11x0::MODE_STBY,   { LOW,  LOW  } },
  { LR11x0::MODE_RX,     { HIGH, LOW  } },
  { LR11x0::MODE_TX,     { HIGH, HIGH } },
  { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
  { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
  END_OF_MODE_TABLE,
};

volatile bool radioBusy = false;
String bleRxBuffer = "";

void setup() {

  Serial.begin(115200);

  pinMode(pw_on_off, OUTPUT);
  digitalWrite(pw_on_off, HIGH);

  pinMode(v33_on_off, OUTPUT);
  digitalWrite(v33_on_off, HIGH);

  pinMode(V_RFSW, OUTPUT);
  digitalWrite(V_RFSW, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);

  tone(BUZZER_PIN, 3000, 200);

  delay(300);

  FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);
  leds[0] = CRGB::Green;
  FastLED.show();

  Bluefruit.begin();

  Bluefruit.setTxPower(4);

  Bluefruit.setName("LoRa Bridge");

  bleuart.begin();

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  setupAdvertising();

  Serial.println("BLE Started");

  SPI.setPins(47, 45, 46);
  SPI.begin();

  Serial.print("[LR1121] Initializing ... ");

  int state = radio.begin(
                FREQUENCY,
                BANDWIDTH,
                SPREADING_FACTOR,
                CODING_RATE,
                SYNCWORD,
                OUTPUT_POWER,
                PREAMBLE_LEN,
                TCXOVOLTAGE
              );

  if (state != RADIOLIB_ERR_NONE) {

    Serial.print("FAILED: ");
    Serial.println(state);

    while (1);

  }

  Serial.println("SUCCESS");

  radio.setRfSwitchTable(
    rfswitch_dio_pins,
    rfswitch_table
  );

  digitalWrite(BUZZER_PIN, LOW);

  radio.startReceive();

  Serial.println("LoRa RX Started");

  Serial.println("System Ready");
}

void loop() {

  BLE_To_LoRa();

  LoRa_To_BLE();
}

// ========================================================
// BLE -> LoRa
// ========================================================
void BLE_To_LoRa() {

  if (!bleuart.available()) {
    return;
  }

  if (radioBusy) {
    return;
  }

  String msg = "";

  while (bleuart.available()) {

    char c = bleuart.read();

    msg += c;
  }

  msg.trim();

  if (msg.length() == 0) {
    return;
  }

  Serial.print("[BLE RX] ");
  Serial.println(msg);

  radioBusy = true;

  // stop receive
  radio.standby();

  int state = radio.transmit(msg);

  if (state == RADIOLIB_ERR_NONE) {

    Serial.println("[LoRa TX] Success");

  } else {

    Serial.print("[LoRa TX] Failed: ");
    Serial.println(state);
  }

  delay(20);

  // VERY IMPORTANT
  radio.startReceive();

  radioBusy = false;
}

// ========================================================
// LoRa -> BLE
// ========================================================
void LoRa_To_BLE() {

  if (radioBusy) {
    return;
  }

  if (radio.getIrqStatus() & RADIOLIB_LR11X0_IRQ_RX_DONE) {

    radioBusy = true;

    String str;

    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {

      Serial.print("[LoRa RX] ");
      Serial.println(str);

      bleuart.print(str);
      bleuart.print("\r\n");

      Serial.println("[BLE TX] Forwarded");

    } else {

      Serial.print("[LoRa RX ERROR] ");
      Serial.println(state);
    }

    delay(10);

    radio.startReceive();

    radioBusy = false;
  }
}

void connect_callback(uint16_t conn_handle) {

  BLEConnection* connection =
    Bluefruit.Connection(conn_handle);

  char peer_name[32] = {0};

  connection->getPeerName(
    peer_name,
    sizeof(peer_name)
  );

  Serial.print("Connected: ");
  Serial.println(peer_name);

  bleuart.println("BLE Connected");
}

void disconnect_callback(
  uint16_t conn_handle,
  uint8_t reason
) {

  Serial.print("Disconnected: 0x");
  Serial.println(reason, HEX);
}

void setupAdvertising() {

  Bluefruit.Advertising.stop();

  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
  );

  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);

  Bluefruit.Advertising.setInterval(32, 244);

  Bluefruit.Advertising.setFastTimeout(30);

  Bluefruit.Advertising.start(0);

  Serial.println("Advertising...");
}