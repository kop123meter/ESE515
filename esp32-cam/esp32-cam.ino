#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include <base64.h>
#include "time.h"
#include <Wire.h>
#include <SPI.h> // Only To Satisfy Compiler, Otherwise Adafruit Libraries Shows a Error
//#include <Adafruit_Sensor.h>
#include <SPIFFS.h>
#include <FS.h>
#include <ArduinoJson.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

#define ESP32CAM_LED_INBUILT 33 // BUILTIN LED Internally Connected to GPIO 33 (Reverse Logic)
#define ESP32CAM_LED_FLASH 4    // BUILTIN FLASH Internally Connected to GPIO 4

// Replace the next variables with your SSID/Password combination
const char *ssid = "AirPennNet-Device";
const char *password = "penn1740wifi";

// MQTT
// Add your MQTT Broker IP address, example:
// const char* mqtt_server = "192.168.1.144";
// const char* mqtt_server = "mqtt.eclipse.org";
const char *mqtt_server = "io.adafruit.com";

const char *mqtt_clientid = "kop";
const char *mqtt_username = "kop"; // Adafruit Username
const char *mqtt_password = "aio_Lerd351MxVqlDRCW7DM86PkczpZk"; // Adafruit AIO Key

// MQTT Publish Topics (Replace "username" with adafruit username)
const char *mqtt_publish_topic_camera = "kop/feeds/camera";
// MQTT Subscribe Topics (Replace "username" with adafruit username)
// const char *mqtt_subscribe_topic_fanstate = "username/feeds/fanstate";
// const char *mqtt_subscribe_topic_dehumidifier = "username/feeds/dehumidifier";

// -----------------I2C-----------------
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);

// External H/W
#define LEDPin 2           // LED Pin Connected to GPIO 2
#define DeHumidifierPin 12 // DeHumidifier Connected to GPIO 12
#define FanPin 13          // Fan Connected to GPIO 13
// #define RelayPin 16

// ---------------Sensors---------------

// BME 280 (Using I2C)
//Adafruit_BME280 bme;

// BME 280 (Using SPI)
// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10
// Adafruit_BME280 bme(BME_CS);                              // hardware SPI
// Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// Sensor Variable (BME280)
float temperature, humidity, pressure, altitude;

#define SEALEVELPRESSURE_HPA (1013.25)
//Adafruit_SGP30 sgp;

// Sensor Variable (SGP30)
uint16_t eCO2_base, TVOC_base;
int co2content; // CO2 Content from SGP30

// Device States
bool LEDState = false, prevLEDState = false;
bool DeHumidifierState = false, FanState = false;

// Device Run Time
int DeHumidifierRunTime = 0, FanRunTime = 0;
bool InitialLoop = true; // Bool To Avoid Initial Delay (First Run)
int RunTime = 5 * 60;    // 5 minutes

// Trigger Limits
float HumidityLimit = 90.00; // Relative Humidity = 90%
int CO2ContentLimit = 900;   // CO2 Content Limit = 900PPM

// NTPServer
// India Requires an Offset of (+5:30 = 330m)
const char *ntpServer = "asia.pool.ntp.org"; // Asia Time
const long gmtOffset_sec = 270 * 60;         // +4:30 hr (270m)
const int daylightOffset_sec = 60 * 60;      // +1:00 hr (60m)
int hour;                                    // Stores Hour (Time)
// int minute, second;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
// char msg[50];
// int value = 0;

void callback(char *topic, byte *message, unsigned int length);
void reconnect();
void setup_wifi();
void publishSensors(const char *publishtopic, const char *data);
void printLocalTime();
void activateLED(int hour);
void readConfigFile();
void writeConfigFile();

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("MYCO CAMERA V1");
  Serial.println();

  // ---------------I2C-----------------

  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  // -------------OUTPUT----------------

  pinMode(LEDPin, OUTPUT);
  pinMode(ESP32CAM_LED_INBUILT, OUTPUT);
  pinMode(DeHumidifierPin, OUTPUT);
  pinMode(FanPin, OUTPUT);
  // pinMode(RelayPin, OUTPUT);

  digitalWrite(LEDPin, LOW);
  digitalWrite(ESP32CAM_LED_INBUILT, HIGH); // LED OFF (Inverse Logic)
  digitalWrite(DeHumidifierPin, LOW);
  digitalWrite(FanPin, LOW);
  // digitalWrite(RelayPin, LOW);

  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());   // 串口输出当前ESP32/ESP8266的MAC地址

  uint8_t macAddr[6]; 
  WiFi.macAddress(macAddr);  
  Serial.printf("用转存到数组的方式获取MAC地址: %02x:%02x:%02x:%02x:%02x:%02x\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

  //------------Camera----------------

  // FLASH LED
  pinMode(ESP32CAM_LED_FLASH, OUTPUT);
  digitalWrite(ESP32CAM_LED_FLASH, LOW);

  // buffer.reserve(32000);
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; // was at 20
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; //800 x 600 necessary for Adafruit IO
  config.jpeg_quality = 30;
  config.fb_count = 1;

  // Camera Init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  

  setup_wifi();

  // Connect To NTP Server For Time & Date
  Serial.println("---Configuring Time From NTP Server---");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // Set MQTT Server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop()
{
  // --------------MQTT Connection-----------------

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // ----------------------------------------------

  //long now = millis();

  //activateLED(hour);


    // -------------Capture Picture-----------------

    //digitalWrite(ESP32CAM_LED_FLASH, HIGH); // Enable FlashLED

    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();

    if (!fb)
    {
      Serial.println("Camera Capture Failed");
      return;
    }
    else
    {
      Serial.println("Camera Captured");
    }

    digitalWrite(ESP32CAM_LED_FLASH, LOW); // Disable FlashLED
    // delay(1000);

    // size_t size = fb->len;
    String buffer = base64::encode((uint8_t *)fb->buf, fb->len);
    // String buffer = base64::encode(fb->buf, fb->len);
    //Serial.println(buffer);

    unsigned int length();
    Serial.println("Buffer Length: ");
    Serial.print(buffer.length());
    Serial.println("");

    if (buffer.length() > 102400)
    {
      Serial.println("Image size too big");
      return;
    }

    Serial.print("Publishing...");

    if (client.publish(mqtt_publish_topic_camera, buffer.c_str()))
    {
      Serial.print("Published Image to ");
      Serial.print(mqtt_publish_topic_camera);
    }
    else
    {
      Serial.println(client.publish(mqtt_publish_topic_camera, buffer.c_str()));
      Serial.println("Error Publishing Image");
    }
    Serial.println("");
    Serial.println("------------------------");

    //printLocalTime();
    //lastMsg = now;
  //if (InitialLoop)
  //{
    //InitialLoop = false;
  //}
  delay(30000);
}

// Checks For Hour and Drives LEDs
void activateLED(int hour)
{
  prevLEDState = LEDState;
  if (hour >= 6 && hour < 18)
  {
    digitalWrite(LEDPin, HIGH);
    digitalWrite(ESP32CAM_LED_INBUILT, LOW);
    LEDState = true;
  }
  else
  {
    digitalWrite(LEDPin, LOW);
    digitalWrite(ESP32CAM_LED_INBUILT, HIGH);
    LEDState = false;
  }
}

// Publishes Data to MQTT Topic
void publishSensors(const char *publishtopic, const char *data)
{
  Serial.println("------------------------");
  Serial.print("Publishing...");
  if (client.publish(publishtopic, data))
  {
    Serial.print("Published To ");
  }
  else
  {
    Serial.print("Error Publishing To ");
  }
  Serial.print(publishtopic);
  Serial.print(" | ");
  Serial.print(data);
  Serial.println("");
  Serial.println("------------------------");
}

// Connects to WiFi Network
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Subscribes to MQTT Topic
void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
}

// Reconnects Till Successful MQTT Connection
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_clientid, mqtt_username, mqtt_password))
    {
      Serial.println("connected");
      // Subscribe Topic
      // client.subscribe(mqtt_subscribe_topic);

      InitialLoop = true; // To Publish the States Instantly after Losing Connection
      Serial.println("-------Sending States Instantly-------");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed To Obtain Time");
    // Try To Obtain Time From NTPServer Once Again
    Serial.println("---Configuring Time From NTP Server---");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // Serial.print("Day of week: ");
  // Serial.println(&timeinfo, "%A");
  // Serial.print("Month: ");
  // Serial.println(&timeinfo, "%B");
  // Serial.print("Day of Month: ");
  // Serial.println(&timeinfo, "%d");
  // Serial.print("Year: ");
  // Serial.println(&timeinfo, "%Y");
  // Serial.print("Hour: ");
  // Serial.println(&timeinfo, "%H");
  // Serial.print("Hour (12 hour format): ");
  // Serial.println(&timeinfo, "%I");
  // Serial.print("Minute: ");
  // Serial.println(&timeinfo, "%M");
  // Serial.print("Second: ");
  // Serial.println(&timeinfo, "%S");

  // Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour, 3, "%H", &timeinfo);
  // Serial.println(timeHour);

  // char timeMinute[3];
  // strftime(timeMinute, 3, "%M", &timeinfo);
  // // Serial.println(timeMinute);

  // char timeSecond[3];
  // strftime(timeSecond, 3, "%S", &timeinfo);
  // // Serial.println(timeSecond);

  // char timeWeekDay[10];
  // strftime(timeWeekDay, 10, "%A", &timeinfo);
  // Serial.println(timeWeekDay);
  // Serial.println();

  // Global Variables
  hour = atoi(timeHour);
  // minute = atoi(timeMinute);
  // second = atoi(timeSecond);

  // Print Global Variables
  Serial.print("Hour : ");
  Serial.println(hour);
  // Serial.print("Minute : ");
  // Serial.println(minute);
  // Serial.print("Second : ");
  // Serial.println(second);
  // Serial.println("");
  Serial.println("------------------------");
}

// Read Baseline Values for SGP30 Sensor from SPIFFS
void readConfigFile()
{
  Serial.println("Reading Config File");
  File configFile = SPIFFS.open("/config.json", "r");
  if (configFile)
  {
    Serial.println("Opened Config File");
    size_t size = configFile.size();

    // Allocate a buffer to store contents of the file.
    std::unique_ptr<char[]> buf(new char[size]);

    configFile.readBytes(buf.get(), size);
    DynamicJsonDocument jsonBuffer(32);

    deserializeJson(jsonBuffer, buf.get());
    serializeJson(jsonBuffer, Serial);

    if (!jsonBuffer.isNull())
    {
      Serial.println("\nparsed json");
      eCO2_base = jsonBuffer["CO2Base"];
      TVOC_base = jsonBuffer["TVOCBase"];
    }
    else
    {
      Serial.println("Failed To Load JSON Config");
      ESP.restart();
    }
  }
}

// Write Baseline Values for SGP30 Sensor from SPIFFS
void writeConfigFile()
{
  Serial.println("Saving Config File");
  DynamicJsonDocument jsonBuffer(32);

  jsonBuffer["CO2Base"] = eCO2_base;
  jsonBuffer["TVOCBase"] = TVOC_base;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed To Open Config File For Writing");
  }

  serializeJson(jsonBuffer, Serial);
  serializeJson(jsonBuffer, configFile);
  configFile.close();
}