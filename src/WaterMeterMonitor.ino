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

// Pulse counters
String channels[] = {"x", "y", "z"};
time_t intervalLength = 5; // 5 second intervals
time_t lastInterval = 0;
unsigned short intervalCount[] = {0, 0, 0};
unsigned long dailyCount[] = {0, 0, 0};
time_t firstPulseTimestamp = 0;
time_t lastPulseTimestamp = 0;
bool waterRunning = false;
time_t lastCounterReset = 0;

// Variables to calculate sampling rate.
unsigned short sampleCount = 0;
unsigned short lastMinuteSampleCount = 0;

float getFlowRate(time_t from, time_t to) {
  if (to > from) {
    unsigned short maxFlow = max(intervalCount[0], intervalCount[1]);
    maxFlow = max(maxFlow, intervalCount[2]);
    return (float)maxFlow/((float)(to-from));
  }
  return 0.0;
}

String getPulseCountsJSON() {
  return String::format("{ \"x\": %u, \"y\": %u, \"z\": %u }",
    dailyCount[0], dailyCount[1], dailyCount[2]);
}

String getDiagnosticJSON(String statusMsg) {
  float sampleRate = (float)lastMinuteSampleCount/60;
  return String::format("\"{ \\\"statusMessage\\\": \\\"%s\\\", \\\"sampleRate\\\": %f }\"", 
    statusMsg.c_str(), sampleRate);
}

void publishWaterStateChange(time_t timestamp) {
  String statusMsg = waterRunning ? "water started" : "water stopped";
  String timestr = Time.format(timestamp, TIME_FORMAT_ISO8601_FULL);
  String status = String::format(
    "{ \"waterRunning\": %d, \"time\": \"%s\", \"statusMessage\": \"%s\" }",
        waterRunning, timestr.c_str(), statusMsg.c_str());
  Particle.publish("waterMeter/waterOn", status);
}

void publishWaterRunning(time_t from, time_t to, String statusMsg) {
  String timestr = Time.format(to, TIME_FORMAT_ISO8601_FULL);
  String status = String::format(
    "{ \"waterRunning\": %d, \"time\": \"%s\", \"pulseCount\": %s, \"flowRate\": %f, \"statusMessage\": \"%s\", \"diagnostics\": %s }",
        waterRunning, timestr.c_str(), getPulseCountsJSON().c_str(), getFlowRate(from, to), statusMsg.c_str(), getDiagnosticJSON(statusMsg).c_str());
  Particle.publish("waterMeter/waterRunning", status);
}

void publishStatus(time_t from, time_t to, String statusMsg) {
  String timestr = Time.format(to, TIME_FORMAT_ISO8601_FULL);
  String status = String::format(
    "{ \"waterRunning\": %d, \"time\": \"%s\", \"pulseCount\": %s, \"flowRate\": %f, \"statusMessage\": \"%s\", \"diagnostics\": %s }",
        waterRunning, timestr.c_str(), getPulseCountsJSON().c_str(), getFlowRate(from, to), statusMsg.c_str(), getDiagnosticJSON(statusMsg).c_str());
  Particle.publish("waterMeter/status", status);
}

void resetCounter(unsigned short* counter) {
  counter[0] = 0;
  counter[1] = 0;
  counter[2] = 0;
}

void resetCounter(unsigned long* counter) {
  counter[0] = 0;
  counter[1] = 0;
  counter[2] = 0;
}

bool updateWaterRunning(time_t from, time_t now) {
  bool messageSent = false;
  bool wasWaterRunning = waterRunning;
  waterRunning = intervalCount[0] != 0 || 
    intervalCount[1] != 0 || intervalCount[2] != 0;
  if (wasWaterRunning != waterRunning) {
    // If the water just started running, report the first pulse timestamp.
    // If the water just stopped running, report the last pulse timestamp.
    time_t timestamp = waterRunning ? firstPulseTimestamp : lastPulseTimestamp + 1;
    publishWaterStateChange(timestamp);

    // If the first pulse was just detected don't send a second message with the
    // same timestamp. 
    messageSent = (firstPulseTimestamp == now) || !waterRunning;
  } else if (waterRunning) {
    // If the water is still running, update the first pulse timestamp to the
    // start of the current interval so the flow rate estimate is accurate.
    firstPulseTimestamp = from-1;
  }

  // If the water is not running, reset the firstPulseTimestamp.
  if (!waterRunning) {
    firstPulseTimestamp = 0;
  }

  return messageSent;
}

void intervalUpdates(time_t now) {
  time_t currentInterval = (now / intervalLength);
  time_t intervalStartTime = lastInterval * intervalLength;

  // Update sample rate counter.
  if (Time.minute(now) != Time.minute(intervalStartTime)) {
    lastMinuteSampleCount = sampleCount;
    sampleCount = 0;
  }

  // Determine whether the water is running or not.
  bool messageSent = updateWaterRunning(intervalStartTime, now);

  // If the water is running, publish the current readings.
  if (!messageSent && waterRunning) {
    publishWaterRunning(firstPulseTimestamp, lastPulseTimestamp, "water running");
    messageSent = true;
  }

  // Reset the daily counters as long as the water is not running.
  if (!messageSent && !waterRunning && Time.day(now) != Time.day(lastCounterReset)) {
    resetCounter(dailyCount);
    lastCounterReset = now;
    publishStatus(intervalStartTime, now, "reset counters");
    messageSent = true;
  }

  // Send hourly updates as long as the water is not running.
  if (!messageSent && !waterRunning && Time.hour(now) != Time.hour(intervalStartTime)) {
    publishStatus(intervalStartTime, now, "hourly update");
    messageSent = true;
  }

  // Reset the interval counters.
  resetCounter(intervalCount);
  lastInterval = currentInterval;
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
int16_t lowThreshold[] = {-1176, -2520, -21694};
int16_t highThreshold[] = {-369, -1816, -20953};

// Functions to handle readings.
void incrementCounts(int c) {
  ++intervalCount[c];
  ++dailyCount[c];
  lastPulseTimestamp = Time.now();
  if (firstPulseTimestamp == 0) {
    firstPulseTimestamp = lastPulseTimestamp;
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

  if (foundLIS3MDL) {
    // Get initial readings from the magnetometer.
    updateReadings();

    // Getting the initial readings could have triggered erroneous pulse counts.
    // Reset the pulse counts back to zero.
    resetCounter(intervalCount);
    resetCounter(dailyCount);
    firstPulseTimestamp = 0;
  }
  
  // Set the last interval start as one higher than the current interval, which
  // which will delay any interval counts until a full interval has elapsed.
  time_t now = Time.now();
  lastInterval = (now / intervalLength) + 1;

  // Set the last time the daily counters were reset.
  lastCounterReset = now;

  // Publish a message indicating that startup completed.
  publishStatus(now, now, "restart");
}

void loop() {
  // Read new values.
  if (foundLIS3MDL) {
    uint64_t now = System.millis();
    if (foundLIS3MDL && now - lastReadingTimestamp >= samplingDelay) {
      updateReadings();
    }
  }

  // Update counters based on the current time.
  time_t now = Time.now();
  time_t currentInterval = (now / intervalLength);
  if (currentInterval > lastInterval) {
    intervalUpdates(now);
  }
}