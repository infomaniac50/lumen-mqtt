#ifndef _LIGHT_SENSOR_SETTINGS_H_
#define _LIGHT_SENSOR_SETTINGS_H_

#include <cstdint>
#include <Adafruit_TSL2561_U.h>

// Values for modifying the IC's settings. All of these values are set to their
// default values.
struct LightSensorSettings
{
    /* Setup the sensor gain and integration time */
    /* You can also manually set the gain or enable auto-gain support */
    // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
    // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
    // tsl.enableAutoRange(true);         /* Auto-gain ... switches automatically between 1x and 16x */

    /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);   /* fast but low resolution */
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

    bool enableAutoRange = true;
    tsl2561IntegrationTime_t integrationTime = TSL2561_INTEGRATIONTIME_13MS;
    tsl2561Gain_t gain = TSL2561_GAIN_1X;
};

#endif
