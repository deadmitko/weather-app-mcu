// Pin definitions
#define MODEM_PWKEY 4
#define MODEM_RST 5
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22
#define SDS011_TX 32
#define SDS011_RX 33

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#define SerialAT Serial1

#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SoftwareSerial.h>

// Create objects
TwoWire I2CPower = TwoWire(0);
TwoWire I2CBME = TwoWire(1);
Adafruit_BME680 bme;

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);
SoftwareSerial sdsSerial(SDS011_TX, SDS011_RX);

// GPRS credentials
const char apn[] = "internet.vivacom.bg";
const char gprsUser[] = "vivacom";
const char gprsPass[] = "vivacom";
const char simPIN[] = "0000";

// MQTT settings
const char* broker = "ssh.r007.top";
const int mqtt_port = 1883;

// Deep sleep settings
#define uS_TO_S_FACTOR 1000000UL
#define TIME_TO_SLEEP 180

// Retry settings
const unsigned long RETRY_DELAY = 5000; // 5 seconds between retries
const int MAX_RETRIES = 10; // Maximum number of retries before giving up

// SDS011 data structure
struct sds011_data {
  float pm25;
  float pm10;
};

// Function prototypes
bool setPowerBoostKeepOn(int en);
bool setup_modem();
bool connectMQTT();
void setup_sensors();
void read_and_publish_data();
sds011_data read_sds011();

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C buses
  I2CPower.begin(I2C_SDA, I2C_SCL, 100000);  // 100kHz for lower power
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);    // 100kHz for lower power

  // Keep power when running from battery
  setPowerBoostKeepOn(1);

  // Setup modem pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Initialize sensors
  setup_sensors();

  // Configure deep sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  // Setup modem and connect to network
  if (!setup_modem()) {
    Serial.println("Failed to setup modem after multiple attempts. Going to sleep.");
    goto sleep;
  }

  // Connect to MQTT
  if (!connectMQTT()) {
    Serial.println("Failed to connect to MQTT after multiple attempts. Going to sleep.");
    goto sleep;
  }

  // Read sensor data and publish to MQTT
  read_and_publish_data();

  // Disconnect
  mqtt.disconnect();
  modem.gprsDisconnect();

sleep:
  // Prepare for sleep
  esp_deep_sleep_start();
}

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(0x75);
  I2CPower.write(0x00);
  I2CPower.write(en ? 0x37 : 0x35);
  return I2CPower.endTransmission() == 0;
}

bool setup_modem() {
  int retries = 0;
  while (retries < MAX_RETRIES) {
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);

    Serial.println("Initializing modem...");
    if (!modem.init()) {
      Serial.println("Failed to initialize modem. Retrying...");
      retries++;
      delay(RETRY_DELAY);
      continue;
    }

    if (strlen(simPIN) && modem.getSimStatus() != 3) {
      modem.simUnlock(simPIN);
    }

    Serial.print("Connecting to APN: ");
    Serial.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      Serial.println(" fail");
      retries++;
      delay(RETRY_DELAY);
      continue;
    }
    Serial.println(" OK");
    return true;
  }
  return false;
}

bool connectMQTT() {
  int retries = 0;
  mqtt.setServer(broker, mqtt_port);
  
  while (retries < MAX_RETRIES) {
    Serial.print("Connecting to MQTT broker...");
    if (mqtt.connect("ESP32Client")) {
      Serial.println(" OK");
      return true;
    }
    Serial.println(" fail");
    retries++;
    delay(RETRY_DELAY);
  }
  return false;
}

void setup_sensors() {
  if (!bme.begin()) {  // Use the I2C address of your BME680
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }

  bme.setTemperatureOversampling(BME680_OS_2X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(320, 150);

  // Initialize SDS011
  sdsSerial.begin(9600);
}

void read_and_publish_data() {
  if (bme.performReading()) {
    float temperature = bme.temperature;
    float pressure = bme.pressure / 100.0;
    float humidity = bme.humidity;
    float voc = bme.gas_resistance / 1000.0;

    // Read SDS011 data
    sds011_data sds_data = read_sds011();

    char payload[150];
    snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
             temperature, pressure, humidity, sds_data.pm25, sds_data.pm10);

    if (mqtt.publish("temp", payload)) {
      Serial.println("Data published to MQTT");
    } else {
      Serial.println("MQTT publish failed");
    }
  } else {
    Serial.println("Failed to read BME680");
  }
}

sds011_data read_sds011() {
  sds011_data result = {0, 0};
  byte buffer[10];
  int index = 0;
  unsigned long timeout = millis() + 3000;  // 3 second timeout

  while (millis() < timeout) {
    if (sdsSerial.available()) {
      byte b = sdsSerial.read();
      if (b == 0xAA && index == 0) {
        buffer[index++] = b;
      } else if (index > 0) {
        buffer[index++] = b;
        if (index == 10) {
          if (buffer[1] == 0xC0 && buffer[9] == 0xAB) {
            result.pm25 = (buffer[3] * 256 + buffer[2]) / 10.0;
            result.pm10 = (buffer[5] * 256 + buffer[4]) / 10.0;
            return result;
          }
          index = 0;
        }
      }
    }
  }

  Serial.println("SDS011 read timeout");
  return result;
}