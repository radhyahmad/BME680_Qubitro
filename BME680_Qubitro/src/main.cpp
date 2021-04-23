#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include <ArduinoJson.h>
#include <QubitroMqttClient.h>
#include <WiFi.h>

void wifi_init();
void qubitro_init();

WiFiClient wifiClient;
QubitroMqttClient mqttClient(wifiClient);

char deviceID[] = "";
char deviceToken[] = "";
const char* ssid = "";
const char* password = "";

struct dataSensor
{
  float temperature = 0.0F;
  float humidity = 0.0F;
  float pressure = 0.0F;
  float voc = 0.0F;
};

static char payload[256];
static dataSensor data;
StaticJsonDocument<256>doc;

Adafruit_BME680 bme;
unsigned long lastData = 0;

void setup() {

  Serial.begin(9600);
  delay(200);
  Serial.println(F("BME680 test"));  

  if (!bme.begin())
  {
    Serial.println(F("Could not find a valid BME680"));
    while(1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  wifi_init();
  qubitro_init();

}

void loop() {

  mqttClient.poll();

  unsigned long endTime = bme.beginReading();

  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }

  delay(50);

  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }

  float t = bme.temperature;
  float h = bme.humidity;
  float p = bme.pressure / 100.0;
  float v = bme.gas_resistance / 1000.0;

  data.temperature = t;
  data.humidity = h;
  data.pressure = p;
  data.voc = v;

  doc["Temperature"] = data.temperature;
  doc["Humidity"] = data.humidity;
  doc["Pressure"] = data.pressure;
  doc["VOC"] = data.voc;

  serializeJsonPretty(doc, payload);

  unsigned long now = millis();

  if (now - lastData > 2000)
  {
    lastData = now;
    mqttClient.beginMessage(deviceID);
    mqttClient.print(payload);
    mqttClient.endMessage();
  }
  
}

void wifi_init(){

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();
  delay(100);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (true)
  {
    delay(1000);
    Serial.print(".");
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("");
      Serial.println("WiFi Connected.");
      Serial.print("Local IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());
      break;
    }
    
  }
  
}

void qubitro_init(){

  char host[] = "broker.qubitro.com";
  int port = 1883;
  mqttClient.setId(deviceID);
  mqttClient.setDeviceIdToken(deviceID, deviceToken);
  Serial.println("Connecting to Qubitro...");

  if (!mqttClient.connect(host, port))
  {
    Serial.print("Connection failed. Error code");
    Serial.println(mqttClient.connectError());
    Serial.println("Visit docs.qubitro.com");
  }

  Serial.println("Connected to Qubitro.");
  mqttClient.subscribe(deviceID);
  
}