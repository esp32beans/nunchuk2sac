/*
MIT License

Copyright (c) 2024 esp32beans@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Set to false to turn off debug output.
const bool DEBUG_ON = false;

#define DBG_print(...)    if (DEBUG_ON) Serial.print(__VA_ARGS__)
#define DBG_println(...)  if (DEBUG_ON) Serial.println(__VA_ARGS__)
#define DBG_printf(...)   if (DEBUG_ON) Serial.printf(__VA_ARGS__)

#include <NintendoExtensionCtrl.h>
#include <Adafruit_MCP4728.h>

#if defined(ARDUINO_ARCH_RP2040) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) \
    || defined(ARDUINO_SAM_DUE) \
    || defined(ARDUINO_ARCH_RENESAS_UNO)
  #if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST) \
      || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
    #define STEMMA_I2C &Wire
    Nunchuk nunchuk;
  #else
    #define STEMMA_I2C &Wire1
    Nunchuk nunchuk(Wire1);
  #endif
#else
  #define DEFAULT_I2C &Wire
  Nunchuk nunchuk;
#endif

// Use MCP4728 DAC to create an analog joystick.
// Connect the the stick to any SAC expanstion port.

Adafruit_MCP4728 dac4;

// SAC = Sony Access Controller
// Ref: PS5 Access Controller Expansion Ports Specifiction 1.00
// Table 7
//
// Item                                              |Specification                  |Note
// --------------------------------------------------|-------------------------------|-----------------------------------------------------
// Potentiometer resistance value                    |10k ± 3k Ω                     |-
// Vdd maximum input voltage                         |≦ 1.8 V                        |-
// Minimum output voltage threshold (negative axis)  |≦ 0.6 V                        |When the stick shaft is inclined to the negative side
// Maximum output voltage threshold (positive axis)  |≧ 1.2 V                        |When the stick shaft is inclined to the positive side
// Midpoint output voltage range (*9)                |0.8 ≦ midpoint voltage ≦ 1.0 V |When no stick operation has occurred
// (*9) The voltage range at the center position when no stick operation
// has occurred
//

#define VOLTS_TO_DAC(v) ((v / 2.048f) * DAC_MAX_RES)
const float SAC_MAX_V = 1.80f;
const uint16_t DAC_MAX_RES = 4095;
const uint16_t DAC_ABS_MAX = VOLTS_TO_DAC(SAC_MAX_V);
const uint16_t DAC_MIN = VOLTS_TO_DAC(0.6f);
const uint16_t DAC_MID = DAC_ABS_MAX / 2;
const uint16_t DAC_MAX = VOLTS_TO_DAC(1.2f);
const uint16_t DAC_BUTTON = DAC_MID;

const uint16_t MCP4728_VREF_INTERNAL_MASK = MCP4728_VREF_INTERNAL << 15;
const uint16_t DAC_EEPROM_POWERON_AXIS = MCP4728_VREF_INTERNAL_MASK | DAC_MID;
const uint16_t DAC_EEPROM_POWERON_BUTTON = MCP4728_VREF_INTERNAL_MASK;

//
// |MCP4728 BB  |TRRS BB 1  |TRRS SAC 1 |TRRS BB 2  |TRRS SAC 2 |TRRS BB 3  |TRRS SAC 3 |Description
// |------------|-----------|-----------|-----------|-----------|-----------|-----------|---------
// |GND         |Ring       |Ring2      |           |           |           |           |
// |VA          |Left       |Tip        |           |           |           |           |Stick Y axis
// |VB          |Right      |Ring1      |           |           |           |           |Stick X axis
// |VC          |           |           |Left       |Tip        |           |           |Button 1
// |VD          |           |           |           |           |Left       |Tip        |Button 2
// |VCC         |n/c        |           |           |           |           |           |
// |            |           |           |Ring<1>    |Ring2<1>   |Ring<1>    |Ring2<1>   |
// |            |           |           |Right      |Ring1      |Right      |Ring1      |
// |            |           |           |Sleeve     |Sleeve     |Sleeve     |Sleeve     |
//
// The TRRS BB columns have the labels that appear on the TRRS break out board. This useful for connecting wires.
//
// The TRRS SAC columns have the corresponding names on the TRRS jack. This terminology is used in the Sony Access Controller Input Specification.
//
// Note <1>: Ring1, Ring2, and Sleeve contacts are connected to each
// other on the TRRS SAC jack. The button open/close signal appears on the
// TRRS Tip contact.

//                 Y axis,     X axis,     button 1,   button 2
bool dac_write_all(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    /*
     * @param channel The channel to update
     * @param new_value The new value to assign
     * @param new_vref Optional vref setting - Defaults to `MCP4728_VREF_VDD`
     * @param new_gain Optional gain setting - Defaults to `MCP4728_GAIN_1X`
     * @param new_pd_mode Optional power down mode setting - Defaults to
     * `MCP4728_PD_MOOE_NORMAL`
     * @param udac Optional UDAC setting - Defaults to `false`, latching (nearly).
     * Set to `true` to latch when the UDAC pin is pulled low
     *
     */

  bool a_ok, b_ok, c_ok, d_ok;
  a_ok = dac4.setChannelValue(MCP4728_CHANNEL_A, a, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  b_ok = dac4.setChannelValue(MCP4728_CHANNEL_B, b, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  c_ok = dac4.setChannelValue(MCP4728_CHANNEL_C, c, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  d_ok = dac4.setChannelValue(MCP4728_CHANNEL_D, d, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  return a_ok && b_ok && c_ok && d_ok;
}

bool dac_center_all(void) {
  // Center stick and release buttons.
  //                   Y axis , X axis , 2 buttons
  return dac_write_all(DAC_MID, DAC_MID, 0, 0);
}

void setup_mcp4728(void) {
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || \
  defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || \
  defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || \
  defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
  // ESP32 is kinda odd in that secondary ports must be manually
  // assigned their pins with setPins()!
  Wire1.setPins(SDA1, SCL1);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

#if defined(ADAFRUIT_FEATHER_ESP32_V2)
  // Turn on the I2C power by pulling pin HIGH.
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

#ifdef DEFAULT_I2C
  if (!dac4.begin()) {
    DBG_println("Couldn't find MCP4728 quad DAC");
    while (1) delay(10);
  }
#endif
#ifdef STEMMA_I2C
  if (!dac4.begin(MCP4728_I2CADDR_DEFAULT, STEMMA_I2C)) {
    DBG_println("Couldn't find MCP4728 quad DAC");
    while (1) delay(10);
  }
#endif
  DBG_println("Found MCP4728 quad DAC");
  DBG_printf("DAC_MAX=%u, DAC_MID=%u, DAC_MIN=%u\n",
      DAC_MAX, DAC_MID, DAC_MIN);

  uint16_t channel_a, channel_b, channel_c, channel_d;
  dac4.readEEPROM(&channel_a, &channel_b, &channel_c, &channel_d);
  DBG_printf("readEEPROM, %04x, %04x, %04x, %04x\n", channel_a, channel_b,
      channel_c, channel_d);
  if ((channel_a != DAC_EEPROM_POWERON_AXIS) ||
      (channel_b != DAC_EEPROM_POWERON_AXIS) ||
      (channel_c != DAC_EEPROM_POWERON_BUTTON) ||
      (channel_d != DAC_EEPROM_POWERON_BUTTON)) {
    DBG_println("Store power up default DAC values to EEPROM");
    dac_center_all();

    dac4.saveToEEPROM();
  }
}

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup() {
  // wait until device mounted
  if (DEBUG_ON) {
    Serial.begin(115200);
    while (!Serial && (millis() < 4000)) delay(10);   // wait for native usb
    Serial.println("Wii nunchuk to SAC analog joystick");
  }

  setup_mcp4728();
  while (!nunchuk.connect()) {
    DBG_println("Nunchuk not detected!");
    delay(1000);
  }
}

void loop() {
  if (nunchuk.update()) {
    uint16_t dac_x = map(nunchuk.joyX(), 0, 255, DAC_MIN, DAC_MAX);
    uint16_t dac_y = map(nunchuk.joyY(), 0, 255, DAC_MIN, DAC_MAX);
    uint16_t dac_button_c = (nunchuk.buttonC()) ? DAC_BUTTON : 0;
    uint16_t dac_button_z = (nunchuk.buttonZ()) ? DAC_BUTTON : 0;
    dac_write_all(dac_x, dac_y, dac_button_c, dac_button_z);
    DBG_printf("Nunchuk: dac_x: %d dac_y: %d\n", dac_x, dac_y);
  }
  else {
    DBG_println("Nunchuk disconnected");
  }
}
