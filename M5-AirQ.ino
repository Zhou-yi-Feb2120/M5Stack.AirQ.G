
/*
 * I2C-Generator: 0.3.0
 * Yaml Version: 2.1.3
 * Template Version: 0.7.0-112-g190ecaa
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#define USE_HSPI_FOR_EPD

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
//#include <GFX.h>
// Note: if you use this with ENABLE_GxEPD2_GFX 1:
//       uncomment it in GxEPD2_GFX.h too, or add #include <GFX.h> before any #include <GxEPD2_GFX.h>
#include <Arduino.h>
#include <SensirionI2CSfa3x.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>
#include <GxEPD2_BW.h>
//#include <GxEPD2_3C.h>
//#include <GxEPD2_4C.h>
//#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <FastLED.h>

#define PIN_LED    21
#define NUM_LEDS   1

#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_154_GDEY0154D67 // GDEY0154D67 200x200, SSD1681, (FPC-B001 20.05.21)

#define GxEPD2_BW_IS_GxEPD2_BW true
//#define GxEPD2_3C_IS_GxEPD2_3C true
//#define GxEPD2_7C_IS_GxEPD2_7C true
//#define GxEPD2_1248_IS_GxEPD2_1248 true
#define IS_GxEPD(c, x) (c##x)
#define IS_GxEPD2_BW(x) IS_GxEPD(GxEPD2_BW_IS_, x)
//#define IS_GxEPD2_3C(x) IS_GxEPD(GxEPD2_3C_IS_, x)
//#define IS_GxEPD2_7C(x) IS_GxEPD(GxEPD2_7C_IS_, x)
//#define IS_GxEPD2_1248(x) IS_GxEPD(GxEPD2_1248_IS_, x)

#if defined(ESP32)
#define MAX_DISPLAY_BUFFER_SIZE 65536ul // e.g.
#if IS_GxEPD2_BW(GxEPD2_DISPLAY_CLASS)
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
#elif IS_GxEPD2_3C(GxEPD2_DISPLAY_CLASS)
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))
#elif IS_GxEPD2_7C(GxEPD2_DISPLAY_CLASS)
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2))
#endif
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(/*CS=*/ 4, /*DC=*/ 3, /*RST=*/ 2, /*BUSY=*/ 1));
#endif

#include "bitmaps/Bitmaps200x200.h" // 1.54" b/w

#if defined(ESP32) && defined(USE_HSPI_FOR_EPD)
SPIClass hspi(HSPI);
#endif

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

int sensorCh = 0;
CRGB leds[NUM_LEDS];

SensirionI2CSfa3x sfa3x;
SensirionI2CSen5x sen5x;
SensirionI2CScd4x scd4x;
SensirionI2cSht4x sensor;

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        //Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    } else {
        //Serial.print("ProductName:");
        //Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        //Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    } else {
        //Serial.print("Firmware: ");
        //Serial.print(firmwareMajor);
        //Serial.print(".");
        //Serial.print(firmwareMinor);
        //Serial.print(", ");

        //Serial.print("Hardware: ");
        //Serial.print(hardwareMajor);
        //Serial.print(".");
        //Serial.println(hardwareMinor);
    }
}

void printUint16Hex(uint16_t value) {
    //Serial.print(value < 4096 ? "0" : "");
    //Serial.print(value < 256 ? "0" : "");
    //Serial.print(value < 16 ? "0" : "");
    //Serial.print(value, HEX);
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        //Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    } else {
        //Serial.print("SerialNumber:");
        //Serial.println((char*)serialNumber);
    }
}

void setup() {

    Serial.begin(115200);
    //while (!Serial) {
    //    delay(100);
    //}

    // 将 G10 配置为输出模式
    pinMode(10, OUTPUT);
    pinMode(46, OUTPUT);
    // 将 G10 设置为低电平
    digitalWrite(10, LOW);
    digitalWrite(46, HIGH);

    analogReadResolution(12);

    #if defined(ESP32) && defined(USE_HSPI_FOR_EPD)
      hspi.begin(5, -1, 6, 4); // remap hspi for EPD (swap pins), SCK, MISO, MOSI, SS
      display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    #endif

    display.init(115200);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    
    Wire.begin();
    Wire1.begin(11,12);    

    sfa3x.begin(Wire);
    sensor.begin(Wire, SHT40_I2C_ADDR_44);
    sen5x.begin(Wire1);
    scd4x.begin(Wire1);

    sensor.softReset();
    delay(10);

    uint16_t error;
    char errorMessage[256];
    float aTemperature = 0.0;
    float aHumidity = 0.0;
    error = sen5x.deviceReset();
    if (error) {
        //Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    }

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
#endif

    // set a temperature offset in degrees celsius
    // Note: supported by SEN54 and SEN55 sensors
    // By default, the temperature and humidity outputs from the sensor
    // are compensated for the modules self-heating. If the module is
    // designed into a device, the temperature compensation might need
    // to be adapted to incorporate the change in thermal coupling and
    // self-heating of other device components.
    //
    // A guide to achieve optimal performance, including references
    // to mechanical design-in examples can be found in the app note
    // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
    // Please refer to those application notes for further information
    // on the advanced compensation settings used
    // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and
    // `setRhtAccelerationMode`.
    //
    // Adjust tempOffset to account for additional temperature offsets
    // exceeding the SEN module's self heating.
    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        //Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    } else {
        //Serial.print("Temperature Offset set to ");
        //Serial.print(tempOffset);
        //Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    error = scd4x.startPeriodicMeasurement();
    error = sfa3x.startContinuousMeasurement();
    if (error == 0) {
      sensorCh = 1;
    }
    else {
      error = sensor.measureLowestPrecision(aTemperature, aHumidity);
      if (error == 0) {
        sensorCh = 2;
      }
    }

    if (sensorCh == 0) {
      showSensorName("SEN.---.SCD.\nPM1.0(ug/m3):\nPM2.5(ug/m3):\nPM4.0(ug/m3):\nPM10 (ug/m3):\nR.H. (%):\nTEMP (^C):\n    (--NC--)\nVOC (Index):\nNOX (Index):\nCO2 (ppm):");
    }
    else if (sensorCh == 1) {
      showSensorName("SEN.SFA.SCD.\nPM1.0(ug/m3):\nPM2.5(ug/m3):\nPM4.0(ug/m3):\nPM10 (ug/m3):\nR.H. (%):\nTEMP (^C):\nHCHO (ppb):\nVOC (Index):\nNOX (Index):\nCO2 (ppm):");
    }
    else if (sensorCh == 2) {
      showSensorName("SEN.SHT.SCD.\nPM1.0(ug/m3):\nPM2.5(ug/m3):\nPM4.0(ug/m3):\nPM10 (ug/m3):\nR.H. (%):\nTEMP (^C):\n    (--NC--)\nVOC (Index):\nNOX (Index):\nCO2 (ppm):");
    }

    FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);
}

void loop() {
    uint16_t error;
    char errorMessage[256];
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;
    uint16_t co2;
    float temperature = 0.0f;
    float humidity = 0.0f;
    int16_t hcho;
    int16_t SFA_humidity;
    int16_t SFA_temperature;
    bool isDataReady = false;
    float aTemperature = 0.0;
    float aHumidity = 0.0;

    delay(1000);

    int adcValue = analogRead(14); // 读取ADC值（0 - 4095）
    float voltage = (adcValue / 4095.0) * 3.3 * 2.0 + 0.29; // 将ADC值转换为实际电压值
    
    // Read Measurement
    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    error = scd4x.readMeasurement(co2, temperature, humidity);
    error = sfa3x.readMeasuredValues(hcho, SFA_humidity, SFA_temperature);
    error = sensor.measureHighPrecision(aTemperature, aHumidity);

    if (sensorCh == 0) {      //SEN
        showSensorData(
        voltage, massConcentrationPm1p0, massConcentrationPm2p5, 
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity + 3, 
        ambientTemperature - 3, 0, vocIndex, noxIndex, co2);
    }
    else if (sensorCh == 1) {        //SFA
        showSensorData(
        voltage, massConcentrationPm1p0, massConcentrationPm2p5, 
        massConcentrationPm4p0, massConcentrationPm10p0, SFA_humidity / 100.0, 
        SFA_temperature / 200.0, hcho / 5.00, vocIndex, noxIndex, co2);
    }
    else if (sensorCh == 2) {       //SHT
        showSensorData(
        voltage, massConcentrationPm1p0, massConcentrationPm2p5, 
        massConcentrationPm4p0, massConcentrationPm10p0, aHumidity, 
        aTemperature, 0, vocIndex, noxIndex, co2);
    }
    
    setLED( massConcentrationPm1p0, massConcentrationPm2p5, 
            massConcentrationPm4p0, massConcentrationPm10p0,
            vocIndex, noxIndex, co2);
    FastLED.show();
}

void showSensorName(const char* sensorName) {
    display.setFullWindow();            // 设置全屏模式显示一次传感器名称
    display.firstPage();
    do {
          display.fillScreen(GxEPD_WHITE);  // 清屏
          display.setCursor(0, 13);        // 设置光标位置
          display.print(sensorName);        // 显示传感器名称
    } while (display.nextPage());
}

// 显示传感器数据的函数
void showSensorData(float data0, float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10) {
      // 定义部分更新区域的坐标和大小
      uint16_t box_x = 144; // 部分更新区域左上角的X坐标
      uint16_t box_y = 13; // 部分更新区域左上角的Y坐标
      uint16_t box_w = 198 - box_x; // 部分更新区域的宽度
      uint16_t box_h = 198; // 部分更新区域的高度
      uint16_t a_text_h = 18;

      display.setPartialWindow(box_x, 1, box_w, box_h); // 设置部分更新区域
      display.firstPage();
      do {
          display.fillRect(box_x, 1, box_w, box_h, GxEPD_WHITE); // 清空显示区域

          display.setCursor(box_x, box_y ); // 设置光标位置
          display.print(data0, 2); // BAT
          display.print("V");

          display.setCursor(box_x, box_y + a_text_h * 1); // 设置光标位置
          display.print(data1, 1); // PM1.0

          display.setCursor(box_x, box_y + a_text_h * 2); // 设置光标位置
          display.print(data2, 1); // PM2.5

          display.setCursor(box_x, box_y + a_text_h * 3); // 设置光标位置
          display.print(data3, 1); // PM4.0

          display.setCursor(box_x, box_y + a_text_h * 4); // 设置光标位置
          display.print(data4, 1); // PM10

          display.setCursor(box_x, box_y + a_text_h * 5 + 1); // 设置光标位置
          display.print(data5, 2); // R.H.

          display.setCursor(box_x, box_y + a_text_h * 6 + 1); // 设置光标位置
          display.print(data6, 2); // TEMP

          display.setCursor(box_x, box_y + a_text_h * 7 + 1); // 设置光标位置
          display.print(data7, 1); // HCHO

          display.setCursor(box_x, box_y + a_text_h * 8 + 1); // 设置光标位置
          display.print(data8, 0); // VOC

          display.setCursor(box_x, box_y + a_text_h * 9 + 1); // 设置光标位置
          display.print(data9, 0); // NOX

          display.setCursor(box_x, box_y + a_text_h * 10 + 1); // 设置光标位置
          display.print(data10, 0); // CO2
      } while (display.nextPage());
}

void setLED(float data0, float data1, float data2, float data3, float data4, float data5, float data6) {
    if((data0 > 75) || (data1 > 75) || (data2 > 150) || (data3 > 150) || (data4 > 400) || (data5 > 300) || (data6 > 2000)) {
        leds[0] = ColorFromPalette(RainbowColors_p, 200, 128, LINEARBLEND); //紫
    }
    else if ((data0 > 50) || (data1 > 50) || (data2 > 100) || (data3 > 100) || (data4 > 250) || (data5 > 150) || (data6 > 1500)) { 
        leds[0] = ColorFromPalette(RainbowColors_p, 3, 128, LINEARBLEND);   //红
    }
    else if ((data0 > 37.5) || (data1 > 37.5) || (data2 > 75) || (data3 > 75) || (data4 > 150) || (data5 > 20) || (data6 > 1000)) {
        leds[0] = ColorFromPalette(RainbowColors_p, 60, 128, LINEARBLEND);  //黄
    }
    else if ((data0 > 25) || (data1 > 25) || (data2 > 50) || (data3 > 50) || (data4 > 100) || (data5 > 3) || (data6 > 700)) {
        leds[0] = ColorFromPalette(RainbowColors_p, 133, 128, LINEARBLEND); //青
    }
    else {
        leds[0] = ColorFromPalette(RainbowColors_p, 90, 128, LINEARBLEND);  //绿
    }
}
