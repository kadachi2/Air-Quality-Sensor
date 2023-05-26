/*
  @author: Kenta Adachi, Rong Mu
  @date: 2023/5/20
  @project_detail: An ESP32 centered air quality sensor, which measures CO2 concentration, particulate matter counts,
                 toxic gas detection, and temperature and humidity
  @components: ESP32, MH-Z19B CO2 sensor, PMS5003 particulate matter sensor, MP503 toxic gas sensor, and DHT20 temperature and humidity sensor
  @reference: https://how2electronics.com/interfacing-pms5003-air-quality-sensor-arduino/ 
*/

#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include "MHZ19.h"

#define CO2_RX 26
#define CO2_TX 27
#define PM_RX 13
#define PM_TX 12
#define BAUDRATE 9600

unsigned long start_time = 0;

MHZ19 my_MHZ19; // CO2 sensor
Adafruit_AHTX0 aht; // temperature/humidity sensor
HardwareSerial *MHZ19_port = &Serial1;
HardwareSerial *PMS_port = &Serial2;

struct PMS{
  uint16_t frame_len;
  uint16_t pm10_unit, pm25_unit, pm100_unit;
  uint16_t pm10_atm, pm25_atm, pm100_atm;
  uint16_t particle_count_3, particle_count_5, particle_count_10, particle_count_25, paricle_count_50, particle_count_100; 
  uint16_t reserved;
  uint16_t checksum;
};

struct PMS pms; // pm sensor

// handles uart communication with particulate matter sensor
int read_pms(){
  uint8_t start1, start2;
  uint8_t read_buffer[30];
  uint16_t checksum = 0;
  uint16_t struct_buffer[15];

  if (!PMS_port->available()) return 0;
  if ((start1 = PMS_port->read()) != 0x42) return 0;
  if ((start2 = PMS_port->read()) != 0x4d) return 0;

  // read byte 2 to byte 31
  for (int i = 0; i < 30; ++i){
    read_buffer[i] = PMS_port->read();
  }
  
  // sum byte 0 to byte 29 to get checksum
  checksum += start1+start2;
  for (int i = 0; i < 28; ++i){
    checksum += read_buffer[i];
  }

  if (checksum != ((read_buffer[28] << 8) + read_buffer[29])) return 0;

  for (int i = 0; i < 15; ++i){
    struct_buffer[i] = (read_buffer[i*2] << 8) + read_buffer[i*2+1];
  }

  memcpy(&pms, (void *)struct_buffer, 30);

  return 1;
}

void setup() {
  // configure UART communcation with CO2 sensor
  Serial.begin(BAUDRATE);

 // setting up CO2 sensor
  MHZ19_port->begin(BAUDRATE, SERIAL_8N1, CO2_RX, CO2_TX);
  my_MHZ19.begin(Serial1);
  my_MHZ19.autoCalibration();

  // setting up particulate matter sensor
  PMS_port->begin(BAUDRATE, SERIAL_8N1, PM_RX, PM_TX);

  // setting up temperature/humidity sensor
  if (!aht.begin()){
    while (1) delay(10);
  }
}

void loop() {
  if (millis() >= start_time + 2000){
    int CO2 = my_MHZ19.getCO2();
    start_time = millis();

    Serial.print("CO2 concentration: ");
    Serial.print(CO2);
    Serial.println(" ppm");

    if (read_pms()){
      Serial.print("PM 1.0 (ug/m3): ");
      Serial.println(pms.pm10_atm);
      Serial.print("PM 2.5 (ug/m3): ");
      Serial.println(pms.pm25_atm);
      Serial.print("PM 10.0 (ug/m3): ");
      Serial.println(pms.pm100_atm);
    }

    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" C");
    Serial.print("Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println(" % rh");

    Serial.println();
  }
}