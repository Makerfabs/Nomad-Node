# nRF52840+LR1121

## 1.Introduction



Product Link: [nRF52840+LR1121]()

Wiki Link:  [nRF52840+LR1121](https://wiki.makerfabs.com/MaTouch%20Lite%20ESP32_S3%20SPI%20IPS%20with%20Touch%202.4%20ST7789.html)

## 2.Feature

- Controller: nRF52840, 64 MHz Cortex-M4 with FPU, 1 MB Flash, 256 KB RAM
- Wireless: Bluetooth Low Energy, Bluetooth mesh, Thread, Zigbee
- LoRa Communication: LR1121, support in the range 150 - 960MHz (sub-GHz) and 2.4GHz
- GNSS Positioning: ATGM336H, support GPS, BDS, GLONASS, Galileo
- 3-Axis-Accelerometer: QMA6100P
- Power Management: CW2015
- WS2812, Buzzer, User button(One-Key Power On/Off Support).
- Charger: Yes


## 3.Arduino IDE

- Install the Arduino IDE V2.3.6.

- Install Adafruit nRF52 V1.6.1.

Click File --> Preferences, enter the following into the “Additional Board Manager URLs” field:

```

https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

```

![52840.png](https://wikiadmin.makerfabs.com/api/uploads/images/ef41c94ee9894fcc81547c3a236561ee.png)

- Search for nrf52, select v1.6.1 version and press install.

![52840 board.png](https://wikiadmin.makerfabs.com/api/uploads/images/d62f67269f5848fdb8530d4014a6c0b9.png)

- Install RadioLib V7.5.0.

![RadioLib.png](https://wikiadmin.makerfabs.com/api/uploads/images/7a83b7f682234210b9a437d971d5177c.png)

- Install TinyGPSPlus V1.0.3.

![TinyGPSPlus.png](https://wikiadmin.makerfabs.com/api/uploads/images/95424ed273d3426ea3b4b045a52f0eca.png)

## 4. Usage

### 4.1 BLE to LoRa Bridge

This example demonstrates a BLE ↔ LoRa bridge based on the nRF52840 and LR1121.

- The phone connects to the device through BLE.

- Text data sent from the phone is forwarded through LoRa.

- LoRa packets received from another device are forwarded back to the phone through BLE.

![ble_lora.png](https://wikiadmin.makerfabs.com/api/uploads/images/d732f5a4f86449dea786b204dba1c480.png)


- Open the [BLE_LoRa](https://github.com/Makerfabs/nRF52840-LR1121/tree/main/Example/BLE_lora) by Arduino.

Use Type-C USB cable to connect the board and PC, and select the development board "Nordic nRF52840 DK" and the port.

- Click the Upload button in the Arduino IDE and wait a few times while the code compiles and uploads to your board.

- Take out another new board and repeat the above steps to program it (If you want to distinguish between two devices, you can set them to different Bluetooth names).

![BLE name.png](https://wikiadmin.makerfabs.com/api/uploads/images/e0c050b5a3a849219960665003d0ca13.png)

- Download the “Serial Bluetooth Terminal” app on your phone.

![app.jpg](https://wikiadmin.makerfabs.com/api/uploads/images/ac6ed058ada245149db344b89c8acc16.jpg)

Open the app, click Devices ---> Bluetooth ---> SCAN.

![APP1.jpg](https://wikiadmin.makerfabs.com/api/uploads/images/d07547f0c9d14720a83fc979675ceb19.jpg)

![APP2.png](https://wikiadmin.makerfabs.com/api/uploads/images/565c084b68f84a63b85f0032bb948a7b.png)

Once the device is found, tap it to connect it to your phone.

![APP3.png](https://wikiadmin.makerfabs.com/api/uploads/images/13ae3b7d830c4b2ab63fd9f964a61d24.png)

Once another device is connected in the same way, they can send data to each other.

- Text in blue indicates data sent;

-  Text in blue indicates data received.

![APP4.jpg](https://wikiadmin.makerfabs.com/api/uploads/images/d51394b1b74b4fb388185207200e4f0b.jpg)


### 4.2 QMA6100P Step Counter

This example demonstrates a simple software-based step counter using the QMA6100P 3-axis accelerometer. The algorithm calculates the acceleration magnitude from the X, Y, and Z axes and detects walking steps using a fixed-baseline peak detection method.

During startup, the device remains still for 2 seconds to establish a baseline acceleration value. As the user walks, the acceleration magnitude oscillates around this baseline. A step is counted when the signal rises above a predefined threshold and then falls back below a rearm threshold. Debounce filtering and a minimum step interval are used to reduce false detections.

- Open the [QMA6100P_Step_Count](https://github.com/Makerfabs/nRF52840-LR1121/tree/main/Example/QMA6100P_Step_Count) by Arduino.

Use Type-C USB cable to connect the board and PC, and select the development board "Nordic nRF52840 DK" and the port.

- Click the Upload button in the Arduino IDE and wait a few times while the code compiles and uploads to your board.

**Result**

![step.png](https://wikiadmin.makerfabs.com/api/uploads/images/f1f95e6659864384a4033b82c5d6b56c.png)





### 4.3 GPS Demo

This example demonstrates how to obtain real-time GPS positioning information using the ATGM336H GPS module.

- Open the [ATGM336H](https://github.com/Makerfabs/nRF52840-LR1121/tree/main/Example/ATGM336H) by Arduino.

Use Type-C USB cable to connect the board and PC, and select the development board "Nordic nRF52840 DK" and the port.

- Click the Upload button in the Arduino IDE and wait a few times while the code compiles and uploads to your board.

**Result**

![gps.png](https://wikiadmin.makerfabs.com/api/uploads/images/8b5ced944fb349829ad2550d7187a4a6.png)

**Note:** The location data can only be obtained outdoors. 
The first time you obtain this information, you will need to wait approximately 33 seconds (the cold start time for this module is 33 seconds).








### 4.4 Get device battery level

This device is equipped with a CW2015 module, allowing users to monitor the battery voltage and charge level.

- Open the [CW2015](https://github.com/Makerfabs/nRF52840-LR1121/tree/main/Example/CW2015) by Arduino.

Use Type-C USB cable to connect the board and PC, and select the development board "Nordic nRF52840 DK" and the port.

- Click the Upload button in the Arduino IDE and wait a few times while the code compiles and uploads to your board.

**Result**
Open the Serial to check the battery level. Since the battery can be charged via USB, the charge level is slowly increasing.

![CW2015.png](https://wikiadmin.makerfabs.com/api/uploads/images/fb19f10946be45178640bbdd27fbd4bf.png)



## 5. FAQ
You can list your questions here or contact techsupport@makerfabs.com for technology support. Detailed descriptions of your question will help to solve your question.

Q1: During flashing, the message “Target is not in DFU mode. Ground the DFU pin and RESET, then release both to enter DFU mode” appears. Flashing was not successful.

![DFU.png](https://wikiadmin.makerfabs.com/api/uploads/images/2624de143a4f4a349c1d69bd05b37352.png)

A1: Double-click the REST button to enter DFU mode, and the flashing will be successful.
