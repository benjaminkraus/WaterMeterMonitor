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

/**********************
 * Pulse Counters
 **********************/

// Counters for different length intervals.
time_t intervalStart = 0;
time_t intervalLength = 5; // 5 second intervals
unsigned short sampleCount = 0;
float sampleRate = 0;
unsigned short intervalCount[] = {0, 0, 0};
unsigned short minutelyCount[] = {0, 0, 0};
unsigned long hourlyCount[] = {0, 0, 0};
unsigned long dailyCount[] = {0, 0, 0};

unsigned short lastIntervalCount[] = {0, 0, 0};
unsigned short lastMinutelyCount[] = {0, 0, 0};
unsigned long lastHourlyCount[] = {0, 0, 0};
unsigned long lastDailyCount[] = {0, 0, 0};

unsigned short maxIntervalCount[] = {0, 0, 0};
unsigned short maxMinutelyCount[] = {0, 0, 0};

// Functions to handle counters.
void resetCounter(unsigned short* counter, unsigned short* last) {
  last[0] = counter[0];
  last[1] = counter[1];
  last[2] = counter[2];
  counter[0] = 0;
  counter[1] = 0;
  counter[2] = 0;
}

void resetCounter(unsigned long* counter, unsigned long* last) {
  last[0] = counter[0];
  last[1] = counter[1];
  last[2] = counter[2];
  counter[0] = 0;
  counter[1] = 0;
  counter[2] = 0;
}

void updateMaxCounts() {
  for (int c = 0; c < 3; ++c) {
    maxIntervalCount[c] = max(maxIntervalCount[c], intervalCount[c]);
    maxMinutelyCount[c] = max(maxMinutelyCount[c], minutelyCount[c]);
  }
}

void updateIntervalCounts(time_t now) {
  if (Time.day(now) != Time.day(intervalStart)) {
    resetCounter(dailyCount, lastDailyCount);
  }

  if (Time.hour(now) != Time.hour(intervalStart)) {
    updateMaxCounts();
    resetCounter(hourlyCount, lastHourlyCount);
  }

  if (Time.minute(now) != Time.minute(intervalStart)) {
    resetCounter(minutelyCount, lastMinutelyCount);
    sampleRate = (float)sampleCount / 60;
    sampleCount = 0;
  }

  if (now - intervalStart > intervalLength) {
    resetCounter(intervalCount, lastIntervalCount);
    intervalStart = now;
  }
}

/**********************
 * Magnetometer Readings
 **********************/

// Variables to store readings.
int16_t lastReading[] = {0, 0, 0};
int16_t minReading[] = { 32767,  32767,  32767};
int16_t maxReading[] = {-32768, -32768, -32768};
uint64_t samplingDelay = 8; // 8 milliseconds = 125 Hz
uint64_t lastReadingTimestamp = 0;
bool lastReadingHigh[] = {false, false, false};

// Hard-coded thresholds.
int16_t lowThreshold[] = {-1199, -2886, -21409};
int16_t highThreshold[] = {-411, -1970,  -20586};

// Functions to handle readings.
void incrementCounts(int c) {
  ++intervalCount[c];
  ++minutelyCount[c];
  ++hourlyCount[c];
  ++dailyCount[c];
}

void updateChannelReadings(int c, int16_t reading) {
  lastReading[c] = reading;
  minReading[c] = min(minReading[c], reading);
  maxReading[c] = max(maxReading[c], reading);

  if (reading > highThreshold[c]) {
    if (!lastReadingHigh[c]) {
      incrementCounts(c);
    }
    lastReadingHigh[c] = true;
  } else if (reading < lowThreshold[c]) {
    if (lastReadingHigh[c]) {
      incrementCounts(c);
    }
    lastReadingHigh[c] = false;
  }
}

void updateReadings() {
  lis3mdl.read();
  ++sampleCount;
  lastReadingTimestamp = System.millis();
  updateChannelReadings(0, lis3mdl.x);
  updateChannelReadings(1, lis3mdl.y);
  updateChannelReadings(2, lis3mdl.z);
}

int resetReadings(String) {
  for (int c = 0; c < 3; ++c) {
    lastReading[c] = 0;
    minReading[c] = 32767;
    maxReading[c] = -32768;
  }
  return 0;
}

String getReadingStatusOneChannel(int c) {
  return String::format("{ \"last\": %i, \"min\": %i, \"max\": %i }",
    lastReading[c], minReading[c], maxReading[c]);
}

String getReadingStatusJSON() {
  String x = getReadingStatusOneChannel(0);
  String y = getReadingStatusOneChannel(1);
  String z = getReadingStatusOneChannel(2);
  return String::format("{ \"x\": %s, \"y\": %s, \"z\": %s }",
    x.c_str(), y.c_str(), z.c_str());
}

String getCounterStatusOneChannel(int c,
  unsigned short* interval, unsigned short* minutely,
  unsigned long* hourly, unsigned long* daily) {
  return String::format("{ \"interval\": %i, \"minute\": %i, \"hour\": %i, \"day\": %i }",
    interval[c], minutely[c], hourly[c], daily[c]);
}

String getMaxCountChannel(int c) {
  return String::format("{ \"interval\": %i, \"minute\": %i }",
    maxIntervalCount[c], maxMinutelyCount[c]);
}

String getCounterStatusOneChannel(int c) {
  return String::format("{ \"current\": %s, \"last\": %s, \"max\": %s }",
    getCounterStatusOneChannel(c, intervalCount, minutelyCount, hourlyCount, dailyCount).c_str(),
    getCounterStatusOneChannel(c, lastIntervalCount, lastMinutelyCount, lastHourlyCount, lastDailyCount).c_str(),
    getMaxCountChannel(c).c_str());
}

String getCounterStatusJSON() {
  String x = getCounterStatusOneChannel(0);
  String y = getCounterStatusOneChannel(1);
  String z = getCounterStatusOneChannel(2);
  return String::format("{ \"x\": %s, \"y\": %s, \"z\": %s, \"sample_rate\": %f }",
    x.c_str(), y.c_str(), z.c_str(), sampleRate);
}

String getStatusJSON() {
  String counterStatus = getCounterStatusJSON();
  String readingsStatus = getReadingStatusJSON();
  return String::format("{ \"counters\": %s, \"readings\": %s }",
    counterStatus.c_str(), readingsStatus.c_str());
}

void setup() {
  Particle.variable("status", getStatusJSON);
  Particle.function("resetReadings", resetReadings);

  // Hard-coded Eastern Standard Time
  Time.zone(-5);

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

  if (foundLIS3MDL) {
    updateReadings();
    updateIntervalCounts(Time.now());
  }
}

void loop() {
  // Update counters based on the current time.
  time_t currentTime = Time.now();
  if (currentTime - intervalStart > intervalLength) {
    updateIntervalCounts(currentTime);
  }

  // Read new values.
  if (foundLIS3MDL) {
    uint64_t now = System.millis();
    if (foundLIS3MDL && now - lastReadingTimestamp >= samplingDelay) {
      updateReadings();
    }
  }
}