// MQ-7 ESP32 — fixed timing + clearer logs
#define HEATER_PIN 25 // MOSFET gate (GPIO 25)
#define SENSOR_PIN 34 // AO (GPIO 34, ADC1_CH6)

#include <math.h>

float RL = 10000.0; // Load resistor on your MQ-7 board (10k typical)
float R0 = 10000.0; // Calibrate in clean air and update

const float slope_m = -0.77;
const float constant_b = 1.7;

// Timing constants (exact cycle you requested)
const unsigned long FULL_MS = 60000UL; // 60 seconds full heat
const unsigned long LOW_MS = 90000UL;  // 90 seconds low-power
const unsigned long QUICK_MS = 50UL;   // 50 ms quick full-power read

// Sampling inside phases
const unsigned long SAMPLE_INTERVAL_MS = 10000UL; // sample every 10 seconds
const int ADC_SAMPLES = 8;                        // average N samples

// PWM settings
const int pwmChannel = 0;
const int pwmFreq = 1000;
const int pwmRes = 8; // 0..255
const int dutyFull = 255;
const int dutyLow = 72; // ~28% -> ~1.4V avg (1.4/5 ≈ 0.28)

void setup()
{
  Serial.begin(115200);
  delay(100);

  // ADC attenuation for near 3.3V range (use ADC_11db for max range)
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  ledcSetup(pwmChannel, pwmFreq, pwmRes);
  ledcAttachPin(HEATER_PIN, pwmChannel);

  Serial.println();
  Serial.println("=== MQ-7 ESP32: Corrected timing cycle ===");
  Serial.println("Cycle: 60s @5V -> 90s @1.4V -> 50ms quick read");
  Serial.println();
}

void loop()
{
  unsigned long phaseStart;

  // --- FULL POWER PHASE (5V) ---
  ledcWrite(pwmChannel, dutyFull);
  Serial.println(">>> PHASE START: FULL POWER (5V) — 60s");
  phaseStart = millis();
  while (millis() - phaseStart < FULL_MS)
  {
    doPhaseSample("5V", dutyFull);
    waitMsOrUntil(SAMPLE_INTERVAL_MS, phaseStart + FULL_MS);
  }
  Serial.println("<<< PHASE END: FULL POWER (5V)");
  Serial.println();

  // --- LOW POWER PHASE (~1.4V avg) ---
  ledcWrite(pwmChannel, dutyLow);
  Serial.println(">>> PHASE START: LOW POWER (~1.4V) — 90s");
  phaseStart = millis();
  while (millis() - phaseStart < LOW_MS)
  {
    doPhaseSample("1.4V", dutyLow);
    waitMsOrUntil(SAMPLE_INTERVAL_MS, phaseStart + LOW_MS);
  }
  Serial.println("<<< PHASE END: LOW POWER (~1.4V)");
  Serial.println();

  // --- QUICK FULL POWER READ ---
  ledcWrite(pwmChannel, dutyFull);
  delay(QUICK_MS); // 50 ms as required
  Serial.println(">>> QUICK FULL POWER READ");
  printSingleReading("quick-read");
  Serial.println("<<< CYCLE COMPLETE");
  Serial.println();
}

// Wait up to 'ms' but stop earlier if untilTimeMillis reached
void waitMsOrUntil(unsigned long ms, unsigned long untilTimeMillis)
{
  unsigned long end = millis() + ms;
  if (end > untilTimeMillis)
    end = untilTimeMillis;
  while (millis() < end)
  {
    delay(10);
  }
}

// Single sampling routine for a phase (averages ADC_SAMPLES)
void doPhaseSample(const char *tag, int duty)
{
  int raw = readAverageADC(SENSOR_PIN, ADC_SAMPLES);
  float vout = rawToVoltage(raw);
  float rs = calcRs(vout);
  float ppm = calcPpm(rs);

  Serial.print("[");
  Serial.print(tag);
  Serial.print("] duty=");
  Serial.print(duty);
  Serial.print(" (est Vheater avg=");
  Serial.print((duty / 255.0f) * 5.0f, 2);
  Serial.print("V) ");
  printReading(raw, vout, rs, ppm);
}

void printSingleReading(const char *tag)
{
  int raw = readAverageADC(SENSOR_PIN, ADC_SAMPLES);
  float vout = rawToVoltage(raw);
  float rs = calcRs(vout);
  float ppm = calcPpm(rs);

  Serial.print("[");
  Serial.print(tag);
  Serial.print("] ");
  printReading(raw, vout, rs, ppm);
}

void printReading(int raw, float vout, float rs, float ppm)
{
  if (raw == 0)
  {
    Serial.println("Raw=0 | Vout=0.000V | Rs=inf | PPM=N/A  <-- ADC=0 (AO disconnected or sensor unpowered)");
    return;
  }
  if (raw >= 4095)
  {
    Serial.println("Raw=4095 | Vout~3.300V | Rs~0 | PPM=ovf  <-- ADC saturation (AO > ADC range)");
    return;
  }

  Serial.print("Raw=");
  Serial.print(raw);
  Serial.print(" | Vout=");
  Serial.print(vout, 3);
  Serial.print(" V | Rs=");
  Serial.print(rs, 1);
  Serial.print(" ohm | PPM=");
  if (!isfinite(ppm) || ppm <= 0.0)
    Serial.println("N/A");
  else
    Serial.println(ppm, 2);
}

// --- ADC helpers ---
int readAverageADC(int pin, int samples)
{
  long sum = 0;
  for (int i = 0; i < samples; ++i)
  {
    sum += analogRead(pin);
    delay(5);
  }
  return (int)(sum / samples);
}

float rawToVoltage(int raw)
{
  return (3.3f * raw) / 4095.0f;
}

float calcRs(float vout)
{
  if (vout <= 0.0001f)
    return INFINITY;
  // If your board scales AO down (divider), you must reverse-scale here before using 5.0:
  // e.g. float sensorVout = vout * (5.0f / 3.3f); // uncomment if AO was divided
  float sensorVout = vout;
  if (sensorVout <= 0.0001f)
    return INFINITY;
  float rs = (5.0f - sensorVout) * RL / sensorVout;
  return rs;
}

float calcPpm(float rs)
{
  if (!isfinite(rs) || rs <= 0.0f)
    return NAN;
  float ratio = rs / R0;
  if (ratio <= 0.0f)
    return NAN;
  float val = (log10(ratio) - constant_b) / slope_m;
  if (val > 50.0f)
    return INFINITY;
  if (val < -50.0f)
    return 0.0f;
  float ppm = pow(10.0f, val);
  return ppm;
}
