#include <Arduino.h>
#include <ESP32WifiCLI.hpp>
#include <PubSubClient.h>
#include <String.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <arduino-timer.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>

#define HOSTNAME_DEFAULT "lumen"

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to I2C SCL Clock
   Connect SDA to I2C SDA Data
   Connect VCC/VDD to 3.3V or 5V (depends on sensor's logic level, check the datasheet)
   Connect GROUND to common ground

   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc.
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.

   History
   =======
   2013/JAN/31  - First version (KTOWN)
*/
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

const String systemHostname(HOSTNAME_DEFAULT);

auto timer = timer_create_default(); // create a timer with default settings

#pragma region "Error Visibility"
bool toggleErrorLed(void *)
{
  static bool state = false;

  state = !state;
  digitalWrite(LED_BUILTIN, state);

  return true;
}

void setErrorStatus(bool isErrored = true)
{
  static uintptr_t error_task = 0;

  if (isErrored)
  {
    if (error_task == 0)
    {
      error_task = timer.every(500, toggleErrorLed);
    }
  }
  else
  {
    if (error_task != 0)
    {
      timer.cancel(error_task);
      error_task = 0;
    }
  }
}
#pragma endregion

#pragma region "MQTT Management"
bool checkClimateSensor(void *)
{
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity_event.relative_humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.println();
  // SensorEvent event;
  // auto interrupted = sensor.getSensorEvent(&event);

  // if (interrupted == 1)
  // {
  //   if (mqtt.connect(systemHostname.c_str()))
  //   {
  //     StaticJsonDocument<256> doc;

  //     doc["type"] = statusToString(event.type);
  //     doc["distance"] = distanceToString(event.distance);
  //     doc["energy"] = event.energy;

  //     mqtt.beginPublish("lumen/event", measureJson(doc), false);
  //     BufferingPrint bufferedClient(mqtt, 32);
  //     serializeJson(doc, bufferedClient);
  //     bufferedClient.flush();
  //     mqtt.endPublish();
  //     setErrorStatus(false);
  //   }
  //   else
  //   {
  //     setErrorStatus(true);
  //   }
  // }

  return true; // repeat? true
}

bool checkLightSensor(void *)
{
  /* Get a new sensor event */
  sensors_event_t light_event;
  tsl.getEvent(&light_event);

  /* Display the results (light is measured in lux) */
  if (light_event.light)
  {
    Serial.print(light_event.light);
    Serial.println(" lux");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }

  // SensorEvent event;
  // auto interrupted = sensor.getSensorEvent(&event);

  // if (interrupted == 1)
  // {
  //   if (mqtt.connect(systemHostname.c_str()))
  //   {
  //     StaticJsonDocument<256> doc;

  //     doc["type"] = statusToString(event.type);
  //     doc["distance"] = distanceToString(event.distance);
  //     doc["energy"] = event.energy;

  //     mqtt.beginPublish("lumen/event", measureJson(doc), false);
  //     BufferingPrint bufferedClient(mqtt, 32);
  //     serializeJson(doc, bufferedClient);
  //     bufferedClient.flush();
  //     mqtt.endPublish();
  //     setErrorStatus(false);
  //   }
  //   else
  //   {
  //     setErrorStatus(true);
  //   }
  // }

  return true; // repeat? true
}

void onPubSubCallback(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, "lumen/ping") == 0)
  {
    esp_chip_info_t info;
    esp_chip_info(&info);
    StringPrint stream;
    stream.print(F("Cores: "));
    stream.println(info.cores);
    stream.print(F("Model: "));
    switch (info.model)
    {
    case CHIP_ESP32:
      stream.println(F("ESP32"));
      break;
    case CHIP_ESP32S2:
      stream.println(F("ESP32-S2"));
      break;
    case CHIP_ESP32S3:
      stream.println(F("ESP32-S3"));
      break;
    case CHIP_ESP32C3:
      stream.println(F("ESP32-C3"));
      break;
    case CHIP_ESP32H2:
      stream.println(F("ESP32-H2"));
      break;
    default:
      stream.println(F("Unknown"));
      break;
    }

    mqtt.publish("lumen/pong", stream.str().c_str(), false);
  }
}
#pragma endregion

#pragma region "Wifi CLI Callbacks"
class mESP32WifiCLICallbacks : public ESP32WifiCLICallbacks
{
  void onWifiStatus(bool isConnected)
  {
    if (isConnected)
    {
      if (!mqtt.connected())
      {

        String broker = wcli.getString("BROKER_HOST", "");
        if (!broker.isEmpty())
        {
          mqtt.setServer(broker.c_str(), 1883);
          mqtt.setCallback(onPubSubCallback);
        }

        if (mqtt.connect(systemHostname.c_str()) && mqtt.subscribe("lumen/ping"))
        {
          setErrorStatus(false);
        }
        else
        {
          setErrorStatus(true);
        }
      }

      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  void onHelpShow()
  {
    // Enter your custom help here:
    Serial.println("\r\nCommands:\r\n");
    Serial.println("broker <hostname> \tset the MQTT broker hostname");
    Serial.println("reboot\t\t\tperform a soft ESP32 reboot");
  }
};

void reboot(String opts)
{
  ESP.restart();
}

void setBroker(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  String broker = operands.first();

  wcli.setString("BROKER_HOST", broker);
  Serial.println("\r\nMQTT broker set to " + broker);
  Serial.println("Please reboot to apply the change.");
}
#pragma endregion

void setup()
{
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  WiFi.hostname(systemHostname);

  Serial.flush(); // Only for showing the message on serial
  delay(1000);
  // wcli.setCallback(new mESP32WifiCLICallbacks());

  // // Connect to WPA/WPA2 network
  // wcli.begin();

  // // attempt to connect to Wifi network:
  // if (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(5000);
  //   wcli.connect();
  // }

  // // Enter your custom commands:
  // wcli.term->add("broker", &setBroker, "\t<hostname> set the MQTT broker hostname");
  // wcli.term->add("reboot", &reboot, "\tperform a ESP32 reboot");

  /*
  https://learn.adafruit.com/adafruit-esp32-feather-v2/pinouts#stemma-qt-connector-3112257

  At the top-left corner of the ESP32 module, is a STEMMA QT connector, labeled QT I2C on the silk.
  This connector allows you to connect a variety of sensors and breakouts with STEMMA QT connectors using various associated cables.

  You must enable the NEOPIXEL_I2C_POWER pin (GPIO 2) for the STEMMA QT connector power to work. Set it to be an output and HIGH in your code.

  There is a NEOPIXEL_I2C_POWER (GPIO 2) pin that must be set to an output and HIGH for the STEMMA QT connector power to work.
  For running in low power mode, you can disable (set output and LOW) the NEOPIXEL_I2C_POWER pin,
  this will turn off the separate 3.3V regulator that powers the QT connector's red wire
  */
  // pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  // digitalWrite(NEOPIXEL_I2C_POWER, HIGH);

  /*
      // weather monitoring
      Serial.println("-- Weather Station Scenario --");
      Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
      Serial.println("filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1, // temperature
                      Adafruit_BME280::SAMPLING_X1, // pressure
                      Adafruit_BME280::SAMPLING_X1, // humidity
                      Adafruit_BME280::FILTER_OFF   );

      // suggested rate is 1/60Hz (1m)
      delayTime = 60000; // in milliseconds
  */

  // call the checkSensor function every 500 millis (0.5 second)

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    setErrorStatus(true);
    while (1)
      delay(10);
  }

  /*
      // weather monitoring
      Serial.println("-- Weather Station Scenario --");
      Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
      Serial.println("filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1, // temperature
                      Adafruit_BME280::SAMPLING_X1, // pressure
                      Adafruit_BME280::SAMPLING_X1, // humidity
                      Adafruit_BME280::FILTER_OFF   );

      // suggested rate is 1/60Hz (1m)
      delayTime = 60000; // in milliseconds
  */

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);

  /* Initialise the sensor */
  // use tsl.begin() to default to Wire,
  // tsl.begin(&Wire2) directs api to use Wire2, etc.
  if (!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.println("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    setErrorStatus(true);
    while (1)
      delay(10);
  }

  /* Setup the sensor gain and integration time */
  /* You can also manually set the gain or enable auto-gain support */
  tsl.setGain(TSL2561_GAIN_1X); /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  // tsl.enableAutoRange(true); /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS); /* 16-bit data but slowest conversions */

  timer.every(60000, checkClimateSensor);
  timer.every(5000, checkLightSensor);
}

void loop()
{
  // mqtt.loop();
  timer.tick();
  // wcli.loop();
}