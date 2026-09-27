\#define BLYNK_TEMPLATE_ID "TMPL35nd-Pqzc"

\#define BLYNK_TEMPLATE_NAME "Health monitoring system"

\#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"



\#include \<WiFi.h>

\#include \<BlynkSimpleEsp32.h>

\#include \<TinyGPS++.h>

\#include \<HardwareSerial.h>

\#include \<Wire.h>

\#include "MAX30105.h"

\#include "heartRate.h"

\#include \<math.h>

\#include "Protocentral_MAX30205.h"





char ssid[] = "YOUR_WIFI_NAME";

char pass[] = "YOUR_WIFI_PASSWORD";



MAX30105 particleSensor;

TinyGPSPlus gps;

HardwareSerial GPS(2);

MAX30205 tempSensor;



float latitude = 0.0;

float longitude = 0.0;



const int LED_PIN = 2;

const int BUZZER_PIN = 33;



float accelMagnitude = 0;

float gyroMagnitude = 0;



bool freeFallDetected = false;

bool impactDetected = false;



unsigned long freeFallTime = 0;

unsigned long impactTime = 0;







byte fallStage = 0;







\#define ECG_PIN 34

\#define ECG_DAC 25

\#define LO_PLUS 26

\#define LO_MINUS 27



// ---------- ECG FILTER ----------

const float FS = 250.0;

const float DT = 1.0 / FS;



// High-pass

float hp_x_prev = 0;

float hp_y_prev = 0;

float hp_alpha;



// Low-pass

float lp_y_prev = 0;

float lp_alpha;



// 50 Hz notch

float notch_x1 = 0;

float notch_x2 = 0;

float notch_y1 = 0;

float notch_y2 = 0;



float notch_b0;

float notch_b1;

float notch_b2;

float notch_a1;

float notch_a2;



unsigned long lastECGSample = 0;

const unsigned long ECG_SAMPLE_INTERVAL = 4000;

int filteredECGValue = 0;

bool leadsOff = true;



// MPU6050

int16_t AcX, AcY, AcZ;

int16_t GyX, GyY, GyZ;



int abnormalCount = 0;





// ---- Heart rate averaging (EMA) ----

long lastBeat = 0;

float beatsPerMinute;

float beatAvgF = 0;   // running average, float for smoothness

byte beatCount = 0;   // how many real beats seen since finger placed

int beatAvg = 0;





// ---- Timing control ----

unsigned long lastBlynkUpdate = 0;

unsigned long lastSlowTaskTime = 0;   // MPU + GPS-forward + status logic

unsigned long lastGPSDebug = 0;

const unsigned long SLOW_TASK_INTERVAL = 200;   // ms: MPU/status/Blynk cadence

const unsigned long BLYNK_UPDATE_INTERVAL = 1000;

unsigned long fingerPlacedAt = 0;

bool wasFingerPresent = false;



String lastStatus = "";

String lastFall = "";

float temperature = 0.0;

float lastValidTemp = 0.0;



unsigned long lastTempRead = 0;

const unsigned long TEMP_READ_INTERVAL = 1000;



// Temporary testing threshold

const float TEMP_ALERT_THRESHOLD = 37.2;



int tempHighCount = 0;

bool temperatureAlert = false;



void resetHeartRate() {

  beatAvg = 0;

  beatAvgF = 0;

  beatCount = 0;

  abnormalCount = 0;

  lastBeat = 0;

}

void runSlowTasks(long irValue, int ecgValue, bool leadsOff);

void setup() {

  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("Initializing...");

  Wire.begin();

  while (!tempSensor.scanAvailableSensors()) {

  Serial.println("MAX30205 not found!");

  delay(1000);

}



tempSensor.begin();



Serial.println("MAX30205 temperature sensor ready.");

  GPS.begin(9600, SERIAL_8N1, 16, 17); // RX=16 (from GPS TX), TX=17 (to GPS RX)



  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {

    Serial.println("MAX30105 was not found. Please check wiring/power.");

    while (1);

  }

  Serial.println("Place your index finger on the sensor with steady pressure.");



  pinMode(LED_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LO_PLUS, INPUT);

  pinMode(LO_MINUS, INPUT);



  // ---------- ECG FILTER SETUP ----------



float fc_hp = 0.5;

hp_alpha = 1.0 / (1.0 + 2.0 \* PI \* fc_hp / FS);



float fc_lp = 40.0;

float rc = 1.0 / (2.0 \* PI \* fc_lp);

lp_alpha = DT / (rc + DT);



float f0 = 50.0;

float Q = 5.0;



float w0 = 2.0 \* PI \* f0 / FS;

float alpha = sin(w0) / (2.0 \* Q);



float a0 = 1.0 + alpha;



notch_b0 = 1.0 / a0;

notch_b1 = -2.0 \* cos(w0) / a0;

notch_b2 = 1.0 / a0;



notch_a1 = -2.0 \* cos(w0) / a0;

notch_a2 = (1.0 - alpha) / a0;



pinMode(ECG_DAC, OUTPUT);



  // Wake up MPU6050

  Wire.beginTransmission(0x68);

  Wire.write(0x6B);

  Wire.write(0);

  Wire.endTransmission(true);



  particleSensor.setup();

  particleSensor.setPulseAmplitudeRed(0x1F);

  particleSensor.setPulseAmplitudeGreen(0);





}



void loop() {

  Blynk.run();

  // ---- MAX30205 temperature ----

if (millis() - lastTempRead >= TEMP_READ_INTERVAL) {



  lastTempRead = millis();



  float newTemp = tempSensor.getTemperature();



  // Accept only valid temperature readings

  if (newTemp >= 20.0 && newTemp <= 45.0) {



  lastValidTemp = newTemp;



  Serial.print("MAX30205 Temperature: ");

  Serial.print(lastValidTemp, 2);

  Serial.println(" °C");

}

else {

  Serial.println("MAX30205: Invalid reading");

}

}

  long irValue = particleSensor.getIR();



 //if (irValue > 50000 && checkForBeat(irValue)) {

    bool fingerPresentNow = (irValue > 50000);

  if (fingerPresentNow && !wasFingerPresent) {

    fingerPlacedAt = millis();   // finger just placed — start settle timer

  }

  wasFingerPresent = fingerPresentNow;

  bool pastSettle = fingerPresentNow && (millis() - fingerPlacedAt > 6000);



  if (fingerPresentNow && checkForBeat(irValue) && pastSettle) {



    long now = millis();





if (lastBeat == 0) {

    lastBeat = now;

} else {

    long delta = now - lastBeat;

    lastBeat = now;



    beatsPerMinute = 60.0 / (delta / 1000.0);





    if (beatsPerMinute >= 50 && beatsPerMinute <= 120)

{

    if (beatCount == 0)

    {

        beatAvgF = beatsPerMinute;

    }

    else

    {

        beatAvgF = 0.85 \* beatAvgF + 0.15 \* beatsPerMinute;

    }



    beatCount++;

    beatAvg = round(beatAvgF);

}

}

  }



  while (GPS.available()) {

    gps.encode(GPS.read());

  }

  if (gps.location.isValid()) {

    latitude = gps.location.lat();

    longitude = gps.location.lng();

  }



  // ---------- ECG READ + FILTER + DAC ----------



if (micros() - lastECGSample >= ECG_SAMPLE_INTERVAL) {



  lastECGSample += ECG_SAMPLE_INTERVAL;



  leadsOff =(digitalRead(LO_PLUS) == HIGH || digitalRead(LO_MINUS) == HIGH);



  if (!leadsOff) {



    int ecgRaw = analogRead(ECG_PIN);



    // HIGH PASS

    float hp =

      hp_alpha \* (hp_y_prev + ecgRaw - hp_x_prev);



    hp_x_prev = ecgRaw;

    hp_y_prev = hp;



    // LOW PASS

    float lp =

      lp_y_prev + lp_alpha \* (hp - lp_y_prev);



    lp_y_prev = lp;



    // 50 Hz NOTCH

    float notch =

      notch_b0 \* lp +

      notch_b1 \* notch_x1 +

      notch_b2 \* notch_x2 -

      notch_a1 \* notch_y1 -

      notch_a2 \* notch_y2;



    notch_x2 = notch_x1;

    notch_x1 = lp;



    notch_y2 = notch_y1;

    notch_y1 = notch;



    filteredECGValue = (int)notch;



    // DAC OUTPUT

    int dacValue = 128 + (int)(notch \* 0.15);



    dacValue = constrain(dacValue, 10, 245);



    dacWrite(ECG_DAC, dacValue);

  }

  else {

    filteredECGValue = 0;

    dacWrite(ECG_DAC, 128);

  }

}



    if (millis() - lastSlowTaskTime >= SLOW_TASK_INTERVAL) {

    lastSlowTaskTime = millis();

    runSlowTasks(irValue, filteredECGValue, leadsOff);

  }



  // Optional GPS debug — once a second, remove once fix is confirmed working

  if (millis() - lastGPSDebug >= 1000) {

    lastGPSDebug = millis();

    Serial.print("Sats: ");

    Serial.print(gps.satellites.value());

    Serial.print(" Valid: ");

    Serial.println(gps.location.isValid());

  }



}



void runSlowTasks(long irValue, int ecgValue, bool leadsOff) {

  // ---- MPU6050 read ----

  Wire.beginTransmission(0x68);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 14, true);



  AcX = Wire.read() << 8 | Wire.read();

  AcY = Wire.read() << 8 | Wire.read();

  AcZ = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // skip temp

  GyX = Wire.read() << 8 | Wire.read();

  GyY = Wire.read() << 8 | Wire.read();

  GyZ = Wire.read() << 8 | Wire.read();



  float Ax = AcX / 16384.0;

float Ay = AcY / 16384.0;

float Az = AcZ / 16384.0;



accelMagnitude = sqrt(Ax \* Ax + Ay \* Ay + Az \* Az);

float Gx = GyX / 131.0;

float Gy = GyY / 131.0;

float Gz = GyZ / 131.0;



gyroMagnitude = sqrt(Gx\*Gx + Gy\*Gy + Gz\*Gz);



Serial.print("Accel = ");

Serial.print(accelMagnitude);



Serial.print("  Gyro = ");

Serial.println(gyroMagnitude);



  // ---- No finger on sensor: reset everything ----

  bool fallDetected = false;

  bool healthAlert = false;



if (!freeFallDetected && accelMagnitude < 0.75)

{

    freeFallDetected = true;

    freeFallTime = millis();



    Serial.println("Free Fall");

}



if (freeFallDetected && !impactDetected &&

    gyroMagnitude > 80 &&

    millis() - freeFallTime < 1000)

{

    impactDetected = true;



    Serial.println("Rotation");

}



if (impactDetected && impactTime ==0 &&

    accelMagnitude > 1.05)

{

    impactTime = millis();



    Serial.println("Impact");

}





    if(impactDetected &&&#x20;

    millis()- impactTime >800 && gyroMagnitude<20)

{

    fallDetected = true;



    Serial.println("\*\*\*\*\*\*\*\* FALL CONFIRMED \*\*\*\*\*\*\*\*");



    freeFallDetected = false;

    impactDetected = false;

    impactTime=0;

}



if (freeFallDetected &&

    millis() - freeFallTime > 1500)

{

    freeFallDetected = false;

    impactDetected = false;

    impactTime=0;

    freeFallTime=0;

}



  if (irValue < 50000) {

    resetHeartRate();

    fallDetected = false;

    healthAlert = false;

    lastStatus = "";

    lastFall = "";



    Blynk.virtualWrite(V0, 0);

    Blynk.virtualWrite(V1, "PLACE FINGER");

    Blynk.virtualWrite(V2, "NO");



    digitalWrite(LED_PIN, LOW);

    digitalWrite(BUZZER_PIN, HIGH);

    Serial.println("PLACE FINGER");

    return;

  }





 // ---- Health alert logic ----



bool bpmAlert = false;



// BPM abnormality

if (beatAvg > 30) {



  if (beatAvg < 55 || beatAvg > 110) {

    abnormalCount++;

  }

  else {

    abnormalCount = 0;

  }



  if (abnormalCount >= 3) {

    bpmAlert = true;

  }



}

else {

  abnormalCount = 0;

}





// Temperature abnormality

if (lastValidTemp >= TEMP_ALERT_THRESHOLD) {

  tempHighCount++;

}

else {

  tempHighCount = 0;

}





// Temperature must remain high for about 30 seconds

if (tempHighCount >= 30) {

  temperatureAlert = true;

}

else {

  temperatureAlert = false;

}



if (temperatureAlert) {

    Serial.println("\*\*\*\*\* TEMPERATURE ALERT \*\*\*\*\*");

}



// Combined health alert

healthAlert = bpmAlert || temperatureAlert;



  // ---- Periodic Blynk BPM update ----

 if (millis() - lastBlynkUpdate > BLYNK_UPDATE_INTERVAL) {



  if (beatAvg > 30) {

    Blynk.virtualWrite(V0, beatAvg);

  }



  if (lastValidTemp > 0) {

    Blynk.virtualWrite(V7, lastValidTemp);

  }



  lastBlynkUpdate = millis();

}



  Serial.println("----------------");

  Serial.print("IR: "); Serial.println(irValue);

  Serial.print("BPM: ");

  Serial.println(beatAvg > 40 ? String(beatAvg) : "Calculating...");



  Serial.print("Temperature: ");



if (lastValidTemp > 0) {

    Serial.print(lastValidTemp, 2);

    Serial.println(" °C");

} else {

    Serial.println("Waiting...");

}



  if (leadsOff) {

    Serial.println("ECG Leads Off");

  } else {

    Serial.print("ECG = "); Serial.println(ecgValue);

    Blynk.virtualWrite(V3, ecgValue);

  }



  // ---- Fall / emergency / normal status ----

  if (fallDetected) {

    if (beatAvg > 30) {



  bool bpmNormal =

    (beatAvg >= 55 && beatAvg <= 110);



  bool healthAbnormal =

    (!bpmNormal || temperatureAlert);



  String status;



  if (healthAbnormal) {

    status = "CRITICAL EMERGENCY";

  }

  else {

    status = "FALL DETECTED";

  }



      Serial.println(status);



      if (lastStatus != status) {

        Blynk.virtualWrite(V1, status);

        lastStatus = status;

        if (!bpmNormal) {

          Serial.println("Sending Notification...");

          Blynk.logEvent("critical_emergency", "Critical Emergency Detected");

        }

      }

      if (lastFall != "YES") {

        Blynk.virtualWrite(V2, "YES");

        lastFall = "YES";

      }



      if (gps.location.isValid()) {

        Blynk.virtualWrite(V4, latitude);

        Blynk.virtualWrite(V5, longitude);

        String mapsLink = "https\://maps.google.com/?q=" +

                           String(latitude, 6) + "," + String(longitude, 6);

        Blynk.virtualWrite(V6, mapsLink);

        Serial.println(mapsLink);

      } else {

        Serial.println("GPS: no fix yet, location not sent");

      }



      digitalWrite(LED_PIN, HIGH);

    } else {

      Serial.println("Checking Heart Rate...");

      if (lastStatus != "CHECKING HEART RATE") {

        Blynk.virtualWrite(V1, "CHECKING HEART RATE");

        lastStatus = "CHECKING HEART RATE";

      }

      if (lastFall != "YES") {

        Blynk.virtualWrite(V2, "YES");

        lastFall = "YES";

      }

      digitalWrite(LED_PIN, HIGH);

    }

  } else if (healthAlert) {

    Serial.println("HEALTH ALERT");

    if (lastStatus != "HEALTH ALERT") {

      Blynk.virtualWrite(V1, "HEALTH ALERT");

      lastStatus = "HEALTH ALERT";

    }

    digitalWrite(LED_PIN, HIGH);

  } else {

    Serial.println("NORMAL");

    if (lastStatus != "NORMAL") {

      Blynk.virtualWrite(V1, "NORMAL");

      lastStatus = "NORMAL";

    }

    if (lastFall != "NO") {

      Blynk.virtualWrite(V2, "NO");

      lastFall = "NO";

    }

    digitalWrite(LED_PIN, LOW);

  }



  // ---- Buzzer: LOW = sounding (per your original active-low wiring) ----

  if (lastStatus == "CRITICAL EMERGENCY" ||

      lastStatus == "HEALTH ALERT" ||

      lastStatus == "FALL DETECTED") {

    digitalWrite(BUZZER_PIN, LOW);

  } else {

    digitalWrite(BUZZER_PIN, HIGH);

  }

}
