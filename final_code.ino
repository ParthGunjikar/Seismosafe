#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <UARTClass.h>

// ADXL335 Accelerometer Pins
const int xPin = A1;
const int yPin = A2;
const int zPin = A3;

// MQ-2 Gas Sensor Pin
const int mq2Pin = A0;
const float gasThreshold = 300; // Adjust based on sensitivity

// Buzzer for alert
const int buzzerPin = 9;

// Sampling parameters
const float Fs = 20.0;
const float dt = 1.0 / Fs;

// STA-LTA Parameters
const int stw = 1 * Fs;
const int ltw = 10 * Fs;
const float threshold = 2.0;

#define bufferSize 2000
float acc_X[bufferSize] = {0};
float acc_Y[bufferSize] = {0};
float acc_Z[bufferSize] = {0};
float sta[bufferSize] = {0};
float lta[bufferSize] = {0};
float sra[bufferSize] = {0};

int index = 0;
bool pWaveDetected = false;
bool gasLeakDetected = false;

// LCD Initialization
TwoWire Wire(1);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// UART for GSM and GPS
UARTClass Gsm(1); // UART1 for GSM
UARTClass Gps(2); // UART2 for GPS

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Initialize GSM & GPS
  Gsm.begin(9600);
  Gps.begin(9600);
  
  Serial.println("Initializing System...");
}
float readADXL335(int pin) {
  int rawValue = analogRead(pin);
  float voltage = (rawValue / 1023.0) * 3.3;
  return (voltage - 1.65) / 0.330;
}

float highPassFilter(float input, float prevInput, float prevOutput, float alpha) {
  return alpha * (prevOutput + input - prevInput);
}

int readMQ2() {
  return analogRead(mq2Pin);
}
String getGPSCoordinates() {
  String gpsData = "";
  while (Gps.available()) {
    char c = Gps.read();
    gpsData += c;
    if (c == '\n') break; // End of NMEA sentence
  }
  
  if (gpsData.indexOf("$GPGGA") != -1) {
    int latIndex = gpsData.indexOf(",") + 1;
    latIndex = gpsData.indexOf(",", latIndex) + 1;
    String latitude = gpsData.substring(latIndex, gpsData.indexOf(",", latIndex));
    
    int lonIndex = gpsData.indexOf(",", latIndex) + 1;
    lonIndex = gpsData.indexOf(",", lonIndex) + 1;
    String longitude = gpsData.substring(lonIndex, gpsData.indexOf(",", lonIndex));

    return "Lat: " + latitude + " Lon: " + longitude + 
           "\nGoogle Maps: https://maps.google.com/?q=" + latitude + "," + longitude;
  }
  
  return "GPS No Fix";
}
void sendSMSAlert(String eventType, String gpsLocation) {
  Gsm.println("AT+CMGF=1"); // Set SMS to text mode
  delay(500);
  Gsm.println("AT+CMGS=\"+916360120205\""); // Replace with your number
  delay(500);
  Gsm.print(eventType + " Detected! Location: ");
  Gsm.print(gpsLocation);
  Gsm.write(26); // End SMS with Ctrl+Z
  delay(5000);
}
void loop() {
  float raw_X = readADXL335(xPin);
  float raw_Y = readADXL335(yPin);
  float raw_Z = readADXL335(zPin);

  static float prev_X = 0, prev_Y = 0, prev_Z = 0;
  static float filtered_X = 0, filtered_Y = 0, filtered_Z = 0;
  float alpha = 0.8;

  filtered_X = highPassFilter(raw_X, prev_X, filtered_X, alpha);
  filtered_Y = highPassFilter(raw_Y, prev_Y, filtered_Y, alpha);
  filtered_Z = highPassFilter(raw_Z, prev_Z, filtered_Z, alpha);

  prev_X = raw_X;
  prev_Y = raw_Y;
  prev_Z = raw_Z;

  for (int i = bufferSize - 1; i > 0; i--) {
    acc_X[i] = acc_X[i - 1];
    acc_Y[i] = acc_Y[i - 1];
    acc_Z[i] = acc_Z[i - 1];
  }

  acc_X[0] = filtered_X;
  acc_Y[0] = filtered_Y;
  acc_Z[0] = filtered_Z;

  int gasLevel = readMQ2();

  if (index >= ltw) {
    float staSum_X = 0, ltaSum_X = 0;
    float staSum_Y = 0, ltaSum_Y = 0;
    float staSum_Z = 0, ltaSum_Z = 0;

    for (int i = 0; i < stw; i++) {
      staSum_X += abs(acc_X[i]);
      staSum_Y += abs(acc_Y[i]);
      staSum_Z += abs(acc_Z[i]);
    }

    int safe_ltw = min(ltw, bufferSize);
    for (int i = 0; i < safe_ltw; i++) {
      ltaSum_X += abs(acc_X[i]);
      ltaSum_Y += abs(acc_Y[i]);
      ltaSum_Z += abs(acc_Z[i]);
    }

    sta[0] = (staSum_X / stw + staSum_Y / stw + staSum_Z / stw) / 3;
    lta[0] = (ltaSum_X / safe_ltw + ltaSum_Y / safe_ltw + ltaSum_Z / safe_ltw) / 3;
    sra[0] = sta[0] / lta[0];

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SRA: ");
    lcd.print(sra[0], 2);

    if (sra[0] > threshold) {
      pWaveDetected = true;
      Serial.println("🚨 Earthquake Detected!");
      digitalWrite(buzzerPin, HIGH);

      lcd.setCursor(0, 1);
      lcd.print("P-Wave Detected!");

      String gpsLocation = getGPSCoordinates();
      Serial.println("Location: " + gpsLocation);
      sendSMSAlert("Earthquake", gpsLocation);

      delay(5);
      digitalWrite(buzzerPin, LOW);
    }

    if (gasLevel > gasThreshold) {
      gasLeakDetected = true;
      Serial.println("🔥 Gas Leak Detected!");
      digitalWrite(buzzerPin, HIGH);

      lcd.setCursor(0, 1);
      lcd.print("Gas Leak Detected!");

      String gpsLocation = getGPSCoordinates();
      Serial.println("Location: " + gpsLocation);
      sendSMSAlert("Gas Leak", gpsLocation);

      delay(5);
      digitalWrite(buzzerPin, LOW);
    }
  }

  index++;
  if (index >= bufferSize) index = ltw;

  Serial.print("X:"); Serial.print(filtered_X);
  Serial.print(" Y:"); Serial.print(filtered_Y);
  Serial.print(" Z:"); Serial.print(filtered_Z);
  Serial.print(" SRA:"); Serial.println(sra[0]);
  Serial.print(" Gas:"); Serial.println(gasLevel);

  delay(100);
}