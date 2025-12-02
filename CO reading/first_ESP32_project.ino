// #define SENSOR 4        // Pin connected to MQ-7 analog output
// #define DELAY 500        // Delay in milliseconds

// int sensorValue = 0;     // Variable to store the sensor value

// void setup() {
//   Serial.begin(115200);  // Start serial communication at 115200 baud
//   Serial.println("MQ-7 Sensor start");
// }

// void loop() {
//   // Read the analog value from the MQ-7 sensor
//   sensorValue = analogRead(SENSOR);

//   // Print the sensor value to the serial monitor
//   Serial.print("Sensor Value: ");
//   Serial.println(sensorValue);

//   delay(DELAY);  // Wait for half a second
// }

// MQ-7 approximate ppm reader for ESP32 (auto-calibrates R0 and saves it)
// WARNING: This produces approximate ppm values only. For accurate CO results,
// the MQ-7 requires heater cycling (different heater voltages/times) and
// careful calibration. Use as a rough indicator only.

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

Preferences prefs;

#define SENSOR_PIN        4        // analog pin (GPIO4)
#define RL_VALUE          10000.0  // load resistor in ohms (change if different)
#define SENSOR_VCC        3.3      // sensor supply used by voltage divider (V)
#define ADC_MAX           4095.0   // 12-bit ADC
#define SAMPLES           10       // analog samples to average
#define CALIB_SAMPLES     60       // samples to average for initial calibration
#define CALIB_DELAY_MS    500      // delay between calibration samples in ms
#define PRINT_INTERVAL    1000     // main loop print interval in ms

float R0 = -1.0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);
  Serial.println();
  Serial.println("MQ-7 approximate ppm reader (3V, approximate).");

  // Set ADC attenuation so ADC covers to ~3.3V (use ADC_11db)
  // On many cores the function is analogSetPinAttenuation(pin, attenuation)
  // If not available on your core comment the next line.
  #if defined(analogSetPinAttenuation)
    analogSetPinAttenuation(SENSOR_PIN, ADC_11db); // allow reading up to ~3.3V
  #endif

  // Load saved R0 if exists
  prefs.begin("mq7", false);
  if (prefs.isKey("R0")) {
    R0 = prefs.getFloat("R0", -1.0);
    if (R0 > 0) {
      Serial.print("Loaded saved R0 = ");
      Serial.print(R0, 2);
      Serial.println(" ohm (approx)");
    } else {
      R0 = -1;
    }
  }

  // If R0 not saved, calibrate now (automatic)
  if (R0 <= 0) {
    Serial.println("No saved calibration (R0). Starting auto-calibration.");
    Serial.println(">>> Keep sensor in CLEAN, fresh air during calibration.");
    Serial.println("Auto-calibration will take roughly (CALIB_SAMPLES * CALIB_DELAY_MS) ms.");
    calibrateR0(CALIB_SAMPLES);
  }
}

float readSensorVoltage() {
  long total = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    total += analogRead(SENSOR_PIN);
    delay(10);
  }
  float raw = (float)total / SAMPLES;
  // convert ADC count -> voltage using ADC range. We used 11db attenuation so Vref ~3.3V
  float voltage = raw / ADC_MAX * 3.3;
  return voltage;
}

float calcRs(float vSensor) {
  // Divider: Vout = Vc * RL/(Rs + RL) => Rs = RL*(Vc/Vout - 1)
  if (vSensor <= 0.0001) return 1e9;
  float Rs = RL_VALUE * (SENSOR_VCC / vSensor - 1.0);
  return Rs;
}

float rsro_to_ppm(float rs, float r0) {
  // Approximation from many hobby sources for MQ-7:
  // Rs/R0 = 22.07 * (ppm)^-0.667  => ppm = (22.07/(Rs/R0))^(1/0.667)
  if (r0 <= 0) return -1;
  float ratio = rs / r0;
  if (ratio <= 0) return -1;
  float ppm = pow(22.07f / ratio, 1.0f / 0.667f);
  return ppm;
}

void calibrateR0(int samples_for_cal) {
  double sumRs = 0.0;
  for (int i = 0; i < samples_for_cal; ++i) {
    float v = readSensorVoltage();
    float rs = calcRs(v);
    sumRs += rs;
    Serial.print(".");
    delay(CALIB_DELAY_MS);
  }
  Serial.println();
  R0 = (float)(sumRs / samples_for_cal);
  Serial.print("Calibration done. R0 = ");
  Serial.print(R0, 2);
  Serial.println(" ohm (approx). Saving to flash.");
  prefs.putFloat("R0", R0);
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;
    float v = readSensorVoltage();
    float rs = calcRs(v);

    Serial.print("V_sensor: ");
    Serial.print(v, 3);
    Serial.print(" V, Rs: ");
    Serial.print(rs, 1);
    Serial.print(" ohm, ");

    if (R0 <= 0) {
      Serial.println("R0 not set. No ppm available.");
    } else {
      float ppm = rsro_to_ppm(rs, R0);
      if (ppm < 0 || isnan(ppm) || isinf(ppm)) {
        Serial.println("ppm: ERR");
      } else {
        Serial.print("CO (approx ppm): ");
        Serial.println(ppm, 2);
      }
    }
  }

  // allow Serial input commands:
  // type 'c' in Serial Monitor then Enter to re-run calibration manually
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.equalsIgnoreCase("c")) {
      Serial.println("Manual recalibration requested. Keep sensor in clean air.");
      calibrateR0(CALIB_SAMPLES);
    } else if (s.equalsIgnoreCase("d")) {
      // delete saved R0
      prefs.remove("R0");
      R0 = -1;
      Serial.println("Deleted saved R0. It will auto-calibrate on next boot or run 'c'.");
    } else {
      Serial.println("Commands: 'c' = calibrate (in clean air), 'd' = delete saved R0.");
    }
  }
}
