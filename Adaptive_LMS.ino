/* Augusto, Nadela
 * =====================================================================
 *  ADAPTIVE DSP SIMULATION — LMS / NLMS ADAPTIVE NOISE CANCELLER
 *  Board  : Arduino Uno / Nano / Mega (any AVR or ARM Arduino)
 *  Output : Arduino IDE Serial Plotter (Tools > Serial Plotter, 115200 baud)
 * =====================================================================
 *
 *  WHAT THIS SIMULATES
 *  -------------------
 *  The classic "adaptive noise cancellation" setup (Widrow, 1975):
 *
 *     clean signal s[n] ─────────────┐
 *                                    (+)──► d[n]  (primary / noisy input)
 *     noise v[n] ──► [unknown path H]┘          │
 *         │                                     │ +
 *         │  reference input x[n]               ▼
 *         └──────────► [adaptive filter W] ──► (−) ──► e[n] = d[n] − y[n]
 *                              ▲          y[n]          │
 *                              └──── LMS weight update ◄┘
 *
 *  - s[n] : the signal we actually want (a sine wave).
 *  - v[n] : a noise source. It reaches the primary sensor after passing
 *           through an "unknown" channel H (an FIR filter we pretend we
 *           don't know).
 *  - x[n] : a reference sensor that picks up the raw noise only.
 *  - W    : an adaptive FIR filter. The LMS algorithm tunes its weights so
 *           that y[n] ≈ noise part of d[n]. When that happens the error
 *           e[n] = d[n] − y[n] becomes ≈ s[n]  → the clean signal is recovered.
 *
 *  POTENTIOMETER CONTROLS (wire each pot: outer legs to 5V and GND,
 *  wiper to the analog pin)
 *  ------------------------------------------------------------------
 *    A0 : Step size μ (mu)       — how fast the filter adapts
 *                                  (small = slow but smooth, large = fast but noisy)
 *    A1 : Noise amplitude         — how strongly the signal is corrupted
 *    A2 : Signal frequency        — frequency of the clean sine wave
 *
 *  OPTIONAL PUSH BUTTON
 *    D2 → button → GND : resets the filter weights to zero so you can
 *                        watch the filter converge again from scratch.
 *
 *  SERIAL PLOTTER TRACES
 *    Noisy     : d[n], what the primary sensor sees
 *    Clean     : s[n], the true signal (reference for comparison)
 *    Output    : e[n], the adaptive filter's recovered signal
 *    Top/Bottom: fixed lines that stop the plotter from auto-rescaling
 */

#include <math.h>

// ---------------------------------------------------------------------
// USER CONFIGURATION
// ---------------------------------------------------------------------

// Set to 0 if a potentiometer is NOT connected. An unconnected analog pin
// "floats" and returns random values, so we fall back to a fixed default.
#define USE_POT_MU     1
#define USE_POT_NOISE  1
#define USE_POT_FREQ   1
#define USE_RESET_BTN  1

// Pin assignments
const uint8_t PIN_POT_MU    = A0;
const uint8_t PIN_POT_NOISE = A1;
const uint8_t PIN_POT_FREQ  = A2;
const uint8_t PIN_RESET_BTN = 2;

// Default values used when a pot is disabled above
const float DEFAULT_MU        = 0.05f;
const float DEFAULT_NOISE_AMP = 1.0f;
const float DEFAULT_FREQ_HZ   = 2.0f;

// Ranges the pots are mapped onto
const float MU_MIN    = 0.001f, MU_MAX    = 0.5f;   // NLMS step size
const float NOISE_MIN = 0.0f,   NOISE_MAX = 2.0f;   // noise amplitude
const float FREQ_MIN  = 0.5f,   FREQ_MAX  = 8.0f;   // signal frequency (Hz)

// Sampling rate of the simulation. 100 Hz keeps the Serial Plotter
// readable (its window shows ~500 points = 5 seconds of data).
const float    FS_HZ          = 100.0f;
const uint32_t SAMPLE_PERIOD_US = (uint32_t)(1000000.0f / FS_HZ);

// Amplitude of the clean sine wave
const float SIGNAL_AMP = 1.0f;

// ---------------------------------------------------------------------
// FILTER DEFINITIONS
// ---------------------------------------------------------------------

// Number of taps (weights) in the adaptive filter. It must be at least
// as long as the unknown path H for perfect cancellation.
const uint8_t N_TAPS = 8;

// The "unknown" noise path H between the noise source and the primary
// sensor. The adaptive filter never reads these numbers — it has to
// discover them on its own by minimizing the error.
const uint8_t H_LEN = 5;
const float H[H_LEN] = { 0.6f, -0.45f, 0.30f, 0.15f, -0.10f };

// Adaptive filter state
float w[N_TAPS];       // adaptive weights (what LMS learns)
float xBuf[N_TAPS];    // delay line of reference input x[n], x[n-1], ...
float vBuf[H_LEN];     // delay line of noise for the unknown path H

// Small constant to avoid divide-by-zero in NLMS normalization
const float NLMS_EPS = 1e-4f;

// ---------------------------------------------------------------------
// RUNTIME STATE
// ---------------------------------------------------------------------
float    phase = 0.0f;        // sine phase accumulator (radians)
uint32_t nextSampleUs = 0;    // time of the next sample (micros)

// Smoothed pot readings (0.0 .. 1.0). Smoothing removes ADC jitter so
// the parameters don't flicker.
float potMuNorm    = 0.0f;
float potNoiseNorm = 0.0f;
float potFreqNorm  = 0.0f;

// =====================================================================
// HELPER FUNCTIONS
// =====================================================================

// Reads an analog pin and returns a value in 0.0..1.0, low-pass filtered
// with an exponential moving average: avg = avg + α(new − avg).
float readPotSmoothed(uint8_t pin, float previous) {
  const float ALPHA = 0.05f;                      // smoothing factor
  float raw = analogRead(pin) / 1023.0f;          // 10-bit ADC → 0..1
  return previous + ALPHA * (raw - previous);
}

// Linear interpolation: maps t (0..1) onto the range [lo, hi].
float mapFloat(float t, float lo, float hi) {
  return lo + t * (hi - lo);
}

// Logarithmic mapping: better for μ because useful values span several
// decades (0.001 → 0.5). Equal pot rotation = equal ratio change.
float mapLog(float t, float lo, float hi) {
  return lo * powf(hi / lo, t);
}

// Approximately Gaussian white noise with zero mean and unit variance.
// Sum of 4 uniform randoms (Central Limit Theorem) — cheap on an AVR.
float gaussianNoise() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    sum += random(-1000, 1001) / 1000.0f;   // uniform in [-1, 1]
  }
  // Variance of one uniform[-1,1] is 1/3 → sum of 4 has variance 4/3.
  return sum * 0.8660254f;                  // scale to variance ≈ 1
}

// Shifts a delay line one step and inserts the newest sample at index 0.
void pushDelayLine(float *buf, uint8_t len, float newest) {
  for (int8_t i = len - 1; i > 0; i--) {
    buf[i] = buf[i - 1];
  }
  buf[0] = newest;
}

// Clears the adaptive weights (and delay lines) — filter starts "untrained".
void resetFilter() {
  for (uint8_t i = 0; i < N_TAPS; i++) { w[i] = 0.0f; xBuf[i] = 0.0f; }
  for (uint8_t i = 0; i < H_LEN;  i++) { vBuf[i] = 0.0f; }
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);                 // must match Serial Plotter baud

#if USE_RESET_BTN
  pinMode(PIN_RESET_BTN, INPUT_PULLUP); // button pulls pin LOW when pressed
#endif

  // Seed the random generator from a floating analog pin so the noise
  // sequence differs on every power-up.
  randomSeed(analogRead(A5));

  resetFilter();

  // Prime the smoothed pot values with a direct reading so the filter
  // does not start from 0 and slowly ramp up.
#if USE_POT_MU
  potMuNorm = analogRead(PIN_POT_MU) / 1023.0f;
#endif
#if USE_POT_NOISE
  potNoiseNorm = analogRead(PIN_POT_NOISE) / 1023.0f;
#endif
#if USE_POT_FREQ
  potFreqNorm = analogRead(PIN_POT_FREQ) / 1023.0f;
#endif

  nextSampleUs = micros();
}

// =====================================================================
// MODULE 1 — CONTROL INPUT MODULE
// Reads the three potentiometers and the reset button, and converts
// them into the filter parameters: step size μ, noise level, frequency.
// =====================================================================
void readControls(float &mu, float &noiseAmp, float &freqHz) {
#if USE_POT_MU
  potMuNorm = readPotSmoothed(PIN_POT_MU, potMuNorm);
  mu = mapLog(potMuNorm, MU_MIN, MU_MAX);
#else
  mu = DEFAULT_MU;
#endif

#if USE_POT_NOISE
  potNoiseNorm = readPotSmoothed(PIN_POT_NOISE, potNoiseNorm);
  noiseAmp = mapFloat(potNoiseNorm, NOISE_MIN, NOISE_MAX);
#else
  noiseAmp = DEFAULT_NOISE_AMP;
#endif

#if USE_POT_FREQ
  potFreqNorm = readPotSmoothed(PIN_POT_FREQ, potFreqNorm);
  freqHz = mapFloat(potFreqNorm, FREQ_MIN, FREQ_MAX);
#else
  freqHz = DEFAULT_FREQ_HZ;
#endif

#if USE_RESET_BTN
  if (digitalRead(PIN_RESET_BTN) == LOW) {
    resetFilter();                      // watch the filter re-learn
  }
#endif
}

// =====================================================================
// MODULE 2 — SIGNAL GENERATION MODULE (the INPUT side)
// Creates the clean sine wave s[n], the raw noise v[n], and the noisy
// primary signal d[n] = s[n] + (noise after passing through path H).
// =====================================================================
void generateInputs(float freqHz, float noiseAmp,
                    float &s, float &v, float &d) {
  // Clean signal s[n]. A phase accumulator is used instead of
  // sin(2πfn/Fs) so turning the frequency pot does not cause a jump.
  phase += 2.0f * PI * freqHz / FS_HZ;
  if (phase >= 2.0f * PI) phase -= 2.0f * PI;   // keep phase bounded
  s = SIGNAL_AMP * sinf(phase);

  // Noise source v[n]. The reference sensor measures this directly.
  v = noiseAmp * gaussianNoise();

  // Pass the noise through the "unknown" path H (FIR convolution):
  //   noiseAtPrimary[n] = Σ H[k] · v[n−k]
  pushDelayLine(vBuf, H_LEN, v);
  float noiseAtPrimary = 0.0f;
  for (uint8_t k = 0; k < H_LEN; k++) {
    noiseAtPrimary += H[k] * vBuf[k];
  }

  // Primary (noisy) input — this is what we feed into the filter.
  d = s + noiseAtPrimary;
}

// =====================================================================
// MODULE 3 — ADAPTIVE FILTER MODULE (the core of the project: NLMS)
// Takes the reference noise x[n] and the noisy signal d[n], returns
// the cleaned output e[n], and updates the weights so it gets better
// on every sample.
// =====================================================================
float adaptiveFilter(float x, float d, float mu) {
  // (a) FILTER OUTPUT  y[n] = Σ w[k] · x[n−k]
  //     y[n] is the filter's current guess of the noise inside d[n].
  pushDelayLine(xBuf, N_TAPS, x);
  float y = 0.0f;
  float xPower = 0.0f;                  // ||x||² for NLMS normalization
  for (uint8_t k = 0; k < N_TAPS; k++) {
    y      += w[k] * xBuf[k];
    xPower += xBuf[k] * xBuf[k];
  }

  // (b) ERROR SIGNAL  e[n] = d[n] − y[n]
  //     In noise cancellation, e[n] IS the system output: once the
  //     filter has learned H, the noise cancels and e[n] ≈ s[n].
  float e = d - y;

  // (c) NLMS WEIGHT UPDATE
  //     w[k] ← w[k] + (μ / (ε + ||x||²)) · e[n] · x[n−k]
  //     Gradient descent on e². Dividing by the input power (Normalized
  //     LMS) keeps the filter stable no matter how loud the noise is.
  float stepScale = mu / (NLMS_EPS + xPower);
  for (uint8_t k = 0; k < N_TAPS; k++) {
    w[k] += stepScale * e * xBuf[k];
  }

  return e;
}

// =====================================================================
// MODULE 4 — PLOT MODULE (Serial Plotter output)
// Sends one line per sample in the format "Label:value,Label:value,..."
// The plotter draws one colored line per value:
//   Noisy  = INPUT  d[n] (signal + noise)
//   Clean  = the true signal s[n], for comparison
//   Output = OUTPUT e[n] (signal recovered by the adaptive filter).
//            Drawn last, so it lies on top of Clean once the filter
//            has learned.
//   Top/Bottom = fixed lines at +3 / −3 so the plot doesn't rescale
// =====================================================================
void plotSample(float d, float s, float e) {
  Serial.print(F("Noisy:"));    Serial.print(d, 3);
  Serial.print(F(",Clean:"));   Serial.print(s, 3);
  Serial.print(F(",Output:"));  Serial.print(e, 3);
  Serial.print(F(",Top:"));     Serial.print(3.0);
  Serial.print(F(",Bottom:"));  Serial.println(-3.0);
  // ~55 characters at 115200 baud ≈ 5 ms, under the 10 ms sample
  // period, so the 100 Hz timing is preserved.
}

// =====================================================================
// MAIN LOOP — one iteration = one DSP sample
// Ties the modules together: controls → inputs → filter → plot.
// =====================================================================
void loop() {
  // Fixed sample rate: wait until it is time for the next sample.
  // A running target time (instead of delay()) keeps Fs accurate.
  if ((int32_t)(micros() - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;

  float mu, noiseAmp, freqHz;
  readControls(mu, noiseAmp, freqHz);           // Module 1

  float s, v, d;
  generateInputs(freqHz, noiseAmp, s, v, d);    // Module 2

  float e = adaptiveFilter(v, d, mu);           // Module 3 (x[n] = v[n])

  plotSample(d, s, e);                          // Module 4
}
