#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;
float spo2;

// Kalman filter parameters
float Q = 0.1;
float R_noise = 5;  // Renamed to avoid conflict with the ratio variable
float P = 1, K = 0;
float X_est = 75;
float X_est_spo2 = 97;

// Kalman filter function
float kalmanFilter(float measurement, float &X_est, float &P) {
  P = P + Q;
  K = P / (P + R_noise);
  X_est = X_est + K * (measurement - X_est);
  P = (1 - K) * P;
  return X_est;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize MAX30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not connected, please check connections!");
    while (1);
  }
  
  // Sensor settings for MAX30102
  particleSensor.setup(); // Use default settings
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  // MAX30102 does not have a green LED, so we omit its setting

  // Initialize ECG sensor (AD8232)
  pinMode(14, INPUT);
  pinMode(12, INPUT);
}

void loop() {
  // Read data from MAX30102 sensor
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  bool fingerDetected = irValue > 5000; // Check if finger is placed

  if (fingerDetected) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        
        beatsPerMinute = kalmanFilter(beatsPerMinute, X_est, P);
        beatAvg = kalmanFilter(beatAvg, X_est, P);
      }
    }

    // Calculate SpO₂ based on the ratio of red to IR light absorption
    float ratio = ((float)redValue / irValue);
    spo2 = 110 - (25 * ratio);
    spo2 = kalmanFilter(spo2, X_est_spo2, P);
  } else {
    beatsPerMinute = 0;
    beatAvg = 0;
    spo2 = 0;
  }

  // Read ECG sensor (AD8232)
  int ecgValue = analogRead(A0);
  bool leadOff = (digitalRead(14) == 1) || (digitalRead(12) == 1);

  // Print sensor data via Serial Monitor
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", RED=");
  Serial.print(redValue);

  if (fingerDetected) {
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.print(", SpO2=");
    Serial.print(spo2);
    Serial.print("%, ECG=");
    Serial.print(ecgValue);
  } else {
    Serial.print(", Finger Not Detected!");
  }

  if (leadOff) {
    Serial.print(", Leads Off!");
  }
  
  Serial.println();
  delay(10);
}