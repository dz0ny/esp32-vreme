#include "cert.h"
#include "esp_adc_cal.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>

RTC_DATA_ATTR bool fastWIFI = false;
RTC_DATA_ATTR int channel = 0;
RTC_DATA_ATTR IPAddress ip;
RTC_DATA_ATTR IPAddress gw;
RTC_DATA_ATTR IPAddress mask;
RTC_DATA_ATTR IPAddress dns;

#define I2C_SDA 14
#define I2C_SCL 12

#define BATTERY_PIN 34
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 60 * 1 /* Time ESP32 will go to sleep (in minutes) */

typedef struct Meritev {
  float temperatura;
  float vlaga;
  float pritisk;
  float baterija;
  int rssi;
} Meritev;
Meritev meritev;

Adafruit_BME280 *sensor;
Adafruit_BME280 *initSensor(uint8_t addr) {
  Adafruit_BME280 *vc = new Adafruit_BME280();
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  bool status = vc->begin(addr, &Wire);
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    return vc;
  }
  vc->setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X4, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X2, Adafruit_BME280::STANDBY_MS_250);
  return vc;
}

float getBatteryVoltage() {
  // we've set 12-bit ADC resolution 2^12=4096 and voltage divider makes it half
  // (2.0) of maximum readable value (which is 3.3V)
  return analogRead(BATTERY_PIN) * 2.0 * (3.3 / 4096.0);
}

void BME280_Sleep(int device_address) {
  const uint8_t CTRL_MEAS_REG = 0xF4;
  // BME280 Register 0xF4 (control measurement register) sets the device mode,
  // specifically bits 1,0 The bit positions are called 'mode[1:0]'. See
  // datasheet Table 25 and Paragraph 3.3 for more detail. Mode[1:0]  Mode
  //    00      'Sleep'  mode
  //  01 / 10   'Forced' mode, use either '01' or '10'
  //    11      'Normal' mode
  Serial.println(F("BME280 to Sleep mode..."));
  Wire.beginTransmission(device_address);
  Wire.requestFrom(device_address, 1);
  uint8_t value = Wire.read();
  value = (value & 0xFC) + 0x00;      // Clear bits 1 and 0
  Wire.write((uint8_t)CTRL_MEAS_REG); // Select Control Measurement Register
  Wire.write((uint8_t)value);         // Send 'XXXXXX00' for Sleep mode
  Wire.endTransmission();
}

String WebString(float val) {
  String a = String(val);
  a.replace(".", "%2C");
  return a;
}

void appendDataToSheet() {
  HTTPClient http;

  String data = "Temperatura=" + WebString(meritev.temperatura) +
                "&Vlaga=" + WebString(meritev.vlaga) +
                "&Pritisk=" + WebString(meritev.pritisk) +
                "&Baterija=" + WebString(meritev.baterija) +
                "&RSSI=" + String(meritev.rssi);
  Serial.println(data);
  http.begin(URL + data, root_ca.c_str());
  int httpCode = http.GET();
  if (httpCode > 0) { // Check for the returning code
    Serial.println(httpCode);
  }

  else {
    Serial.println("Error on HTTP request");
  }
  http.end();
}

bool initWIFI() {
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
    int tries = 0;
    if (fastWIFI) {
      Serial.println("FastWiFi loaded");
      WiFi.config(ip, gw, mask, dns);
      WiFi.begin(WIFI_SSID, WIFI_PASS, channel);
    } else {
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

    while (WiFi.status() != WL_CONNECTED) {
      tries++;
      Serial.println("|");
      if (tries >= 15)
        return false;
      delay(1000);
    }
    Serial.print("Connected ");
    Serial.println(WiFi.localIP());
    // cache last connection results in RTC memory so that we can boot faster
    Serial.println("FastWiFi saved");
    fastWIFI = true;
    channel = WiFi.channel();
    ip = WiFi.localIP();
    gw = WiFi.gatewayIP();
    mask = WiFi.subnetMask();
    dns = WiFi.dnsIP();
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  if (!initWIFI()) {
    return;
  };

  sensor = initSensor(BME280_ADDRESS_ALTERNATE);

  // set battery measurement pin
  adcAttachPin(BATTERY_PIN);
  adcStart(BATTERY_PIN);
  analogReadResolution(12); // Default of 12 is not very linear. Recommended
                            // to use 10 or 11 depending on needed resolution.

  meritev.pritisk = sensor->readPressure() / 100.0F; // hpa
  meritev.vlaga = sensor->readHumidity();
  meritev.temperatura = sensor->readTemperature();
  meritev.baterija = getBatteryVoltage();
  meritev.rssi = WiFi.RSSI();

  appendDataToSheet();
}

void loop() {
  BME280_Sleep(BME280_ADDRESS_ALTERNATE);
  Serial.println(F("ESP32 to Sleep mode...")); // Say going to sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000L);
  esp_deep_sleep_start(); // Now place ESP32 into sleep mode for duration set
}
