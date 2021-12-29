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

/********************************************
 * Variables for Magnetometer Readings
 ********************************************/

int16_t lastReading[] = {0, 0, 0};
int16_t minReading[] = { 32767,  32767,  32767};
int16_t maxReading[] = {-32768, -32768, -32768};
uint64_t samplingDelay = 8; // 8 milliseconds = 125 Hz
uint64_t lastReadingTimestamp = 0;
bool lastReadingHigh[] = {false, false, false};

int16_t lowThreshold[] = {-1176, -2520, -21694};
int16_t highThreshold[] = {-369, -1816, -20953};

/********************************************
 * Variables for Pulse Counters
 ********************************************/

// Interval pulse counters and time stamps.
String channels[] = {"x", "y", "z"};
time_t intervalLength = 5; // 5 second intervals
time_t intervalStartTime = 0;
time_t lastIntervalStartTime = 0;
time_t lastPulseTimestamp = 0;
time_t lastMessageSent = 0;
unsigned short intervalCount[] = {0, 0, 0};
unsigned short lastIntervalCount[] = {0, 0, 0};
bool waterRunning = false;

// Daily pulse counter.
unsigned long dailyCount[] = {0, 0, 0};
unsigned long lastDailyCount[] = {0, 0, 0};
time_t lastCounterReset = 0;

// Variables to calculate sampling rate.
unsigned short sampleCount = 0;
time_t lastSampleCountReset = 0;
float sampleRate = 0;

/********************************************
 * Functions for Magnetometer Readings
 ********************************************/
void incrementCounts(int c) {
  ++intervalCount[c];
  ++dailyCount[c];
  lastPulseTimestamp = Time.now();
  if (intervalStartTime == 0) {
    intervalStartTime = lastPulseTimestamp;
  }
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

/********************************************
 * Functions for Pulse Counters
 ********************************************/

void updateSampleRate() {
  time_t now = Time.now();
  if (now - lastSampleCountReset >= 60) {
    sampleRate = (float)sampleCount / ((float)(now - lastSampleCountReset));
    sampleCount = 0;
    lastSampleCountReset = now;
  }
}

float getFlowRate(time_t from, time_t to, unsigned short* counter) {
  if (to > from) {
    unsigned short maxFlow = max(counter[0], counter[1]);
    maxFlow = max(maxFlow, counter[2]);
    return (float)maxFlow/((float)(to-from));
  }
  return 0.0;
}

String getPulseCountsJSON(unsigned long* counter) {
  return String::format("{ \"x\": %u, \"y\": %u, \"z\": %u }",
    counter[0], counter[1], counter[2]);
}

String getDiagnosticJSON(String statusMsg) {
  return String::format("\"{ \\\"statusMessage\\\": \\\"%s\\\" }\"",
    statusMsg.c_str());
}

void publishWaterOn(time_t timestamp) {
  String statusMsg = "water started";
  String timeStr = Time.format(timestamp, TIME_FORMAT_ISO8601_FULL);
  String status = String::format(
    "{ \"waterRunning\": %d, \"time\": \"%s\", \"statusMessage\": \"%s\", \"diagnostics\": %s }",
        waterRunning, timeStr.c_str(), statusMsg.c_str(), getDiagnosticJSON(statusMsg).c_str());
  Particle.publish("waterMeter/waterOn", status);
  lastMessageSent = timestamp;
}

void publishWaterRunning(time_t from, time_t to, unsigned short* counter) {
  String statusMsg = waterRunning ? "water running" : "water stopped";
  String timeStr = Time.format(to, TIME_FORMAT_ISO8601_FULL);
  String pulseCountStr = getPulseCountsJSON(lastDailyCount);
  float flowRate = getFlowRate(from, to, counter);
  String status = String::format(
    "{ \"waterRunning\": %d, \"time\": \"%s\", \"pulseCount\": %s, \"flowRate\": %f, \"statusMessage\": \"%s\", \"diagnostics\": %s }",
        waterRunning, timeStr.c_str(), pulseCountStr.c_str(), flowRate, statusMsg.c_str(), getDiagnosticJSON(statusMsg).c_str());
  Particle.publish("waterMeter/waterRunning", status);
  lastMessageSent = to;
}

void publishStatus(time_t timestamp, String statusMsg) {
  String timeStr = Time.format(timestamp, TIME_FORMAT_ISO8601_FULL);
  String pulseCountStr = getPulseCountsJSON(dailyCount);
  float flowRate = 0.0;
  String status = String::format(
    "{ \"waterRunning\": %d, \"time\": \"%s\", \"pulseCount\": %s, \"flowRate\": %f, \"statusMessage\": \"%s\", \"diagnostics\": %s }",
        waterRunning, timeStr.c_str(), pulseCountStr.c_str(), flowRate, statusMsg.c_str(), getDiagnosticJSON(statusMsg).c_str());
  Particle.publish("waterMeter/status", status);
  lastMessageSent = timestamp;
}

void rolloverCounter(unsigned short* counter, unsigned short* last) {
  last[0] = counter[0];
  last[1] = counter[1];
  last[2] = counter[2];
  counter[0] = 0;
  counter[1] = 0;
  counter[2] = 0;
}

bool checkNonZeroCounter(unsigned short* counter) {
  return counter[0] != 0 || counter[1] != 0 || counter[2] != 0;
}

void copyCounter(unsigned long* counter, unsigned long* last) {
  last[0] = counter[0];
  last[1] = counter[1];
  last[2] = counter[2];
}

void resetCounter(unsigned long* counter) {
  counter[0] = 0;
  counter[1] = 0;
  counter[2] = 0;
}

bool nextUpdateDue(time_t now, time_t lastUpdate, time_t minInterval) {
  // Publish updates at least once every four hours.
  int currentHour = Time.hour(now);
  bool divisibleByFour = ((currentHour / 4) * 4 == currentHour);
  return currentHour != Time.hour(lastUpdate) && divisibleByFour
      && now - lastUpdate >= minInterval;
}

void intervalUpdates() {
  time_t now = Time.now();
  if (!waterRunning && intervalStartTime > 0) {
    waterRunning = true;
    publishWaterOn(intervalStartTime);
  } else if (waterRunning && now - intervalStartTime >= intervalLength) {
    // Check if the water is still running.
    waterRunning = checkNonZeroCounter(intervalCount);

    // Publish the previous interval's pulse counts and flow rate.
    // This is delayed by one interval so that we know whether the water has
    // stopped flowing before sending each update.
    if (checkNonZeroCounter(lastIntervalCount)) {
      time_t timestamp = waterRunning ? intervalStartTime : lastPulseTimestamp + 1;
      publishWaterRunning(lastIntervalStartTime, timestamp, lastIntervalCount);
    }

    // Copy the current interval and daily count.
    // If the water has stopped running, intervalCount will be all zeros and
    // will cause lastIntervalCount to be zeroed out as well.
    copyCounter(dailyCount, lastDailyCount);
    rolloverCounter(intervalCount, lastIntervalCount);
    lastIntervalStartTime = intervalStartTime;

    // If the water is still running, increment the intervalStartTime.
    // If the water has stopped running, set the intervalStartTime to 0.
    intervalStartTime = waterRunning ? now : 0;
  } else if (Time.day(now) != Time.day(lastCounterReset)
      && now - lastMessageSent >= intervalLength) {
    // Reset the daily counters.
    resetCounter(dailyCount);
    lastCounterReset = now;

    // Publish the status after resetting the counters to record the zero values.
    publishStatus(now, "reset counters");
  } else if (nextUpdateDue(now, lastMessageSent, intervalLength)) {
    // Publish at least one message every hour.
    publishStatus(now, "hourly update");
  }
}

void setup() {
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

  time_t now = Time.now();
  if (foundLIS3MDL) {
    // Get initial readings from the magnetometer.
    updateReadings();

    // Getting the initial readings could have triggered erroneous pulse counts.
    // Reset the pulse counts back to zero. Call rolloverCounter twice to make
    // sure that lastIntervalCount is zeroed out.
    rolloverCounter(intervalCount, lastIntervalCount);
    rolloverCounter(intervalCount, lastIntervalCount);
    resetCounter(dailyCount);
    intervalStartTime = 0;
    lastCounterReset = now;
    publishStatus(now, "restart successful");
  } else {
    publishStatus(now, "sensor not found");
  }
}
time_t lastKeepAlive = 0;


void loop() {
  // Read new values.
  if (foundLIS3MDL) {
    uint64_t now = System.millis();
    if (foundLIS3MDL && now - lastReadingTimestamp >= samplingDelay) {
      updateReadings();
    }
  }

  updateSampleRate();
  intervalUpdates();
}
