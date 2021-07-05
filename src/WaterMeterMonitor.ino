/*
 * Project WaterMeterMonitor
 * Description: Read water meter using a magnetometer.
 * Author: Benjamin Kraus <ben@benkraus.com>
 * Date: 2021-07-04
 */

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LIS3MDL lis3mdl;
#define LIS3MDL_CS A2

bool foundLIS3MDL = false;
int16_t lastReading[] = {0, 0, 0};
int16_t minReading[] = { 32767,  32767,  32767};
int16_t maxReading[] = {-32768, -32768, -32768};

void updateReadings(int channel, int16_t reading) {
  lastReading[channel] = reading;
  minReading[channel] = min(minReading[channel], reading);
  maxReading[channel] = max(maxReading[channel], reading);
}

int resetReadings(String) {
  for (int c = 0; c < 3; ++c) {
    lastReading[c] = 0;
    minReading[c] = 32767;
    maxReading[c] = -32768;
  }
  return 0;
}

String getStatusOneChannel(int channel) {
  return String::format("{ \"last\": \"%i\", \"min\": %i, \"max\": %i }",
    lastReading[channel], minReading[channel], maxReading[channel]);
}

String getStatusJSON() {
  String x = getStatusOneChannel(0);
  String y = getStatusOneChannel(1);
  String z = getStatusOneChannel(2);
  return String::format("{ \"x\": \"%s\", \"y\": %s, \"z\": %s }",
    x.c_str(), y.c_str(), z.c_str());
}

void setup() {
  Particle.variable("status", getStatusJSON);
  Particle.function("resetReadings", resetReadings);

  // Attempt to connect to the LIS3MDL board.
  foundLIS3MDL = lis3mdl.begin_SPI(LIS3MDL_CS);
  if (foundLIS3MDL) {
    lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.configInterrupt(false, // disbale interrupt generation on X-axis
                            false, // disable interrupt generation on Y-axis
                            false, // disable interrupt generation on Z-axis
                            false, // polarity (false = low, true = high)
                            false, // disable interrupt latching
                            false); // disable interrupt pin
  }
}

void loop() {
  // Read new values
  if (foundLIS3MDL) {
    lis3mdl.read();
    updateReadings(0, lis3mdl.x);
    updateReadings(1, lis3mdl.y);
    updateReadings(2, lis3mdl.z);
    delay(8); // 125 Hz
  }
}