#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#define SensorPin A0         // the pH meter Analog output is connected with the Arduino’s Analog
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;

namespace pin {
const byte tds_sensor = A1;
const byte one_wire_bus = 7;  // Dallas Temperature Sensor
}

namespace device {
float aref = 4.3;
}

namespace sensor {
float ec = 0;
unsigned int tds = 0;
float waterTempC = 0;
float waterTempF = 0;
float ecCalibration = 1;
}

OneWire oneWire(pin::one_wire_bus);
DallasTemperature dallasTemperature(&oneWire);


void setup() {
  Serial.begin(9600);

  dallasTemperature.begin();
}


void loop() {
  dallasTemperature.requestTemperatures();
  sensor::waterTempC = dallasTemperature.getTempCByIndex(0);
  sensor::waterTempF = dallasTemperature.getTempCByIndex(0);
  float rawEc = analogRead(pin::tds_sensor) * device::aref / 1024.0;                                           // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float temperatureCoefficient = 1.0 + 0.02 * (sensor::waterTempC - 25.0);                                     // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration;                                       // temperature and calibration compensation
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5;  //convert voltage value to tds value
  // Serial.print(F("TDS:"));
  // Serial.println(sensor::tds);
  // Serial.print(F("EC:"));
  // Serial.println(sensor::ec, 2);
  // Serial.print(F("Temperature:"));
  // Serial.println(sensor::waterTemp, 2);
  delay(1000);
  for (int i = 0; i < 10; i++)  //Get 10 sample value from the sensor for smooth the value
  {
    buf[i] = analogRead(SensorPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++)  //sort the analog from small to large
  {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++) avgValue += buf[i];  //take the average value of 6 center sample, from 2  until 7f

  float phValue = (float)avgValue * 5.0 / 1024 / 6;  //convert the analog into millivolt
  phValue = 3.5 * phValue;
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["phValue"] = serialized(String(phValue, 2));  //conver to 2 decimal
  jsonDoc["tds"] = serialized(String(sensor::tds));
  jsonDoc["ec"] = serialized(String(sensor::ec, 2));                  //conver to 2 decimal
  jsonDoc["waterTempC"] = serialized(String(sensor::waterTempC, 2));  //conver to 2 decimal
  jsonDoc["waterTempF"] = serialized(String(sensor::waterTempF, 2));  //conver to 2 decimal
  char buffer[200];
  serializeJson(jsonDoc, buffer);
  // String jsonString;
  // serializeJson(jsonDoc, jsonString);
  serializeJson(jsonDoc, buffer);
  Serial.println(buffer);
}

// void readTdsQuick() {
//   dallasTemperature.requestTemperatures();
//   sensor::waterTemp = dallasTemperature.getTempCByIndex(0);
//   float rawEc = analogRead(pin::tds_sensor) * device::aref / 1024.0;                                           // read the analog value more stable by the median filtering algorithm, and convert to voltage value
//   float temperatureCoefficient = 1.0 + 0.02 * (sensor::waterTemp - 25.0);                                      // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
//   sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration;                                       // temperature and calibration compensation
//   sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5;  //convert voltage value to tds value
//   Serial.print(F("TDS:"));
//   Serial.println(sensor::tds);
//   Serial.print(F("EC:"));
//   Serial.println(sensor::ec, 2);
//   Serial.print(F("Temperature:"));
//   Serial.println(sensor::waterTemp, 2);
// }