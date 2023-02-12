#ifndef _CLIMATE_SENSOR_SETTINGS_H_
#define _CLIMATE_SENSOR_SETTINGS_H_

#include <cstdint>
#include <Adafruit_BME280.h>

// Values for modifying the IC's settings. All of these values are set to their
// default values.
struct ClimateSensorSettings
{
    // For more details on the following scenarious, see chapter
    // 3.5 "Recommended modes of operation" in the datasheet

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

    /*
        // humidity sensing
        Serial.println("-- Humidity Sensing Scenario --");
        Serial.println("forced mode, 1x temperature / 1x humidity / 0x pressure oversampling");
        Serial.println("= pressure off, filter off");
        bme.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1,   // temperature
                        Adafruit_BME280::SAMPLING_NONE, // pressure
                        Adafruit_BME280::SAMPLING_X1,   // humidity
                        Adafruit_BME280::FILTER_OFF );

        // suggested rate is 1Hz (1s)
        delayTime = 1000;  // in milliseconds
    */

    /*
        // indoor navigation
        Serial.println("-- Indoor Navigation Scenario --");
        Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
        Serial.println("0.5ms standby period, filter 16x");
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,  // temperature
                        Adafruit_BME280::SAMPLING_X16, // pressure
                        Adafruit_BME280::SAMPLING_X1,  // humidity
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5 );

        // suggested rate is 25Hz
        // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
        // T_ovs = 2
        // P_ovs = 16
        // H_ovs = 1
        // = 40ms (25Hz)
        // with standby time that should really be 24.16913... Hz
        delayTime = 41;
        */

    /*
    // gaming
    Serial.println("-- Gaming Scenario --");
    Serial.println("normal mode, 4x pressure / 1x temperature / 0x humidity oversampling,");
    Serial.println("= humidity off, 0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X4,   // pressure
                    Adafruit_BME280::SAMPLING_NONE, // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );

    // Suggested rate is 83Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5)
    // T_ovs = 1
    // P_ovs = 4
    // = 11.5ms + 0.5ms standby
    delayTime = 12;
    */
    Adafruit_BME280::sensor_mode mode = Adafruit_BME280::MODE_NORMAL;
    Adafruit_BME280::sensor_sampling tempSampling = Adafruit_BME280::SAMPLING_X16;
    Adafruit_BME280::sensor_sampling pressSampling = Adafruit_BME280::SAMPLING_X16;
    Adafruit_BME280::sensor_sampling humSampling = Adafruit_BME280::SAMPLING_X16;
    Adafruit_BME280::sensor_filter filter = Adafruit_BME280::FILTER_OFF;
    Adafruit_BME280::standby_duration duration = Adafruit_BME280::STANDBY_MS_0_5;
};

#endif
