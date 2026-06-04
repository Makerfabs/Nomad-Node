/*
Arduino IDE V2.3.6
Additional boards manager URLs:https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
Board: nrf52 V1.6.1

Tools:
Board: "Nordic nRF52840 DK"

QMA6100P Step Counter — Fixed-baseline peak detection

Strategy:
1. First 2 seconds: measure resting magnitude → lock as baseline
2. Walking: magnitude oscillates above/below baseline
3. Step = rising edge crosses baseline + 0.4g, with debounce & min interval
*/

#include <Wire.h>
#include "QMA6100P.h"

// ----- Pins -----
#define I2C_SDA_PIN   26
#define I2C_SCL_PIN   27

// ----- Step Detection -----
#define STEP_THRESHOLD       0.45   // g — mag must exceed baseline+this to arm
#define STEP_REARM           0.15   // g — mag must drop below baseline+this to re-arm
#define STEP_MIN_INTERVAL    180    // ms — minimum between steps
#define DEBOUNCE_SAMPLES     3      // consecutive confirmations

QMA6100P qmaAccel;
outputData myData;

// ----- Fixed Baseline -----
float   fixedBaseline   = 1.0;
bool    baselineLocked  = false;
float   baselineSum     = 0;
int     baselineSamples = 0;

// ----- Step Detector -----
uint32_t steps           = 0;
uint32_t lastStepTime    = 0;
bool     armed           = false;   // true after mag crossed above threshold
uint8_t  armCount        = 0;       // consecutive above-threshold samples
uint8_t  rearmCount      = 0;       // consecutive below-rearm samples

// ----- Debug -----
uint32_t lastPrintTime   = 0;

// ============================================================================
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=================================");
  Serial.println("  QMA6100P Step Counter");
  Serial.println("=================================");

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();

  if (!qmaAccel.begin(QMA6100P_ADDRESS_LOW, &Wire))
  {
    Serial.println("ERROR: QMA6100P not found!");
    while (1);
  }
  Serial.print("Chip ID: 0x"); Serial.println(qmaAccel.getUniqueID(), HEX);

  qmaAccel.softwareReset();
  delay(10);
  qmaAccel.setRange(SFE_QMA6100P_RANGE2G);
  qmaAccel.enableAccel();
  qmaAccel.setOffset(0, 0, 0);

  Serial.println("Hold still — calibrating baseline (2s)...");
}

// ============================================================================
void loop()
{
  uint32_t now = millis();

  // ----- Read sensor -----
  qmaAccel.getAccelData(&myData);
  float mag = sqrt(myData.xData * myData.xData +
                   myData.yData * myData.yData +
                   myData.zData * myData.zData);

  // ----- Phase 1: Lock baseline (first 2 seconds, ~100 samples) -----
  if (!baselineLocked)
  {
    baselineSum += mag;
    baselineSamples++;
    if (baselineSamples >= 100)  // 100 samples @ 50Hz = 2 seconds
    {
      fixedBaseline = baselineSum / baselineSamples;
      baselineLocked = true;
      Serial.print("Baseline locked: ");
      Serial.print(fixedBaseline, 3);
      Serial.println(" g  —  Start walking!\n");
    }
    delay(20);
    return;
  }

  // ----- Phase 2: Step Detection -----
  float hi = fixedBaseline + STEP_THRESHOLD;   // e.g. 1.4 + 0.45 = 1.85
  float lo = fixedBaseline + STEP_REARM;        // e.g. 1.4 + 0.15 = 1.55

  // Rising edge: mag crosses above hi → arm the detector
  if (mag > hi)
  {
    armCount++;
    rearmCount = 0;
    if (armCount >= DEBOUNCE_SAMPLES && !armed)
    {
      armed = true;
    }
  }
  // Falling edge: mag drops below lo → if armed, count a step
  else if (mag < lo)
  {
    rearmCount++;
    armCount = 0;
    if (rearmCount >= DEBOUNCE_SAMPLES && armed)
    {
      if (now - lastStepTime >= STEP_MIN_INTERVAL)
      {
        steps++;
        lastStepTime = now;
      }
      armed = false;
      rearmCount = 0;
    }
  }
  else
  {
    // In the middle zone: decay counters slowly
    if (armCount > 0) armCount--;
    if (rearmCount > 0) rearmCount--;
  }

  // ----- Print every second -----
  if (now - lastPrintTime >= 1000)
  {
    Serial.println("-----------------------------------");
    Serial.print("X:"); Serial.print(myData.xData, 2);
    Serial.print(" Y:"); Serial.print(myData.yData, 2);
    Serial.print(" Z:"); Serial.print(myData.zData, 2);
    Serial.print(" | Mag:"); Serial.print(mag, 2);
    Serial.print(" Base:"); Serial.print(fixedBaseline, 2);
    Serial.print(" Hi:"); Serial.print(hi, 2);
    Serial.print(" Lo:"); Serial.print(lo, 2);
    Serial.print(" Armed:"); Serial.print(armed ? "Y" : "N");
    Serial.print(" | Steps:"); Serial.println(steps);
    lastPrintTime = now;
  }

  delay(20); // 50Hz
}
