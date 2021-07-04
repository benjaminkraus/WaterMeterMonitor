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


String getStatusJSON(bool foundLIS3MDL, int16_t* reading) {
  char buf[256];
  JSONBufferWriter writer(buf, sizeof(buf)-1);
  writer.beginObject();
    writer.name("LIS3MDL").value(foundLIS3MDL);
    writer.name("x").value(reading[0]);
    writer.name("y").value(reading[1]);
    writer.name("z").value(reading[2]);
  writer.endObject();
  writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;
  return buf;
}

String getCurrentStatusJSON() {
  return getStatusJSON(foundLIS3MDL, lastReading);
}

void setup() {
  Particle.variable("status", getCurrentStatusJSON);

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
    lastReading[0] = lis3mdl.x;
    lastReading[1] = lis3mdl.y;
    lastReading[2] = lis3mdl.z;
    delay(500);
  }
}