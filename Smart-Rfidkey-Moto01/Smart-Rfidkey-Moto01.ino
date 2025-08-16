#include <WiFi.h>
#include <FirebaseESP32.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Adafruit_PN532.h>


#define WIFI_SSID "YOURSSID"
#define WIFI_PASSWORD "YOURPASSWORD"
#define FIREBASE_HOST "YOURFIREBASEHOST"
#define FIREBASE_AUTH "YOURFIREKEY"
#define RELAY_PIN_SYSTEM     26
#define BUZZER_PIN           25
#define VOLTAGE_PIN          34
#define SDA_PIN 21
#define SCL_PIN 22


// ตั้งค่าความถี่ในการอัปเดตข้อมูลให้เป็นค่าเดียว
#define UPDATE_INTERVAL_MS 15000  // ส่งข้อมูลทุก 15 วินาที
// -----------------------------

const float R1 = 4700.0;
const float R2 = 1000.0;
const float CORRECTION_FACTOR = 1.084;
const float VOLTAGE_MAX = 12.7;
const float VOLTAGE_MIN = 11.8;
const float MAX_RANGE_KM = 30.0;
String authorizedUID = "CB E2 44 05";

// --- ตัวแปรและ Object ---
FirebaseData fbdo, stream;
FirebaseAuth auth;
FirebaseConfig config;
TinyGPSPlus gps;
HardwareSerial& gpsSerial = Serial2;
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);


bool isSystemOn = false;
bool isRemotelyLocked = false;
String currentTripId = "";
unsigned long lastSendTime = 0;

// --- (ส่วนฟังก์ชันพื้นฐาน และฟังก์ชัน streamCallback) ---
void beepSuccess() { digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW); }
void beepFail() { digitalWrite(BUZZER_PIN, HIGH); delay(500); digitalWrite(BUZZER_PIN, LOW); }
void beepSystemOff() {
  digitalWrite(BUZZER_PIN, HIGH); delay(80); digitalWrite(BUZZER_PIN, LOW);
  delay(80);
  digitalWrite(BUZZER_PIN, HIGH); delay(80); digitalWrite(BUZZER_PIN, LOW);
}
void streamCallback(StreamData data) {
  if (data.dataType() == "boolean") {
    isRemotelyLocked = data.boolData();
    Serial.printf(">>> Remote Lock is now %s\n", isRemotelyLocked ? "ON" : "OFF");
    beepSuccess();
  }
}
void streamTimeoutCallback(bool timeout) { if (timeout) Serial.println("Stream timeout, resuming..."); }

// --- (ส่วนฟังก์ชัน sendStatusToFirebase) ---
void sendStatusToFirebase() {
  if (!Firebase.ready()) return;

  FirebaseJson json;
  json.set("is_on", isSystemOn);

  float totalVoltage = 0;
  for (int i = 0; i < 5; i++) {
    totalVoltage += analogRead(VOLTAGE_PIN);
    delay(10);
  }
  int adcValue = totalVoltage / 5;

  float voltageOut = adcValue * (3.3 / 4095.0);
  float rawBatteryVoltage = voltageOut * (R1 + R2) / R2;
  float batteryVoltage = rawBatteryVoltage * CORRECTION_FACTOR;
  int soc = constrain(map(batteryVoltage * 100, VOLTAGE_MIN * 100, VOLTAGE_MAX * 100, 0, 100), 0, 100);
  float range = MAX_RANGE_KM * (soc / 100.0);
  json.set("voltage", String(batteryVoltage, 2));
  json.set("soc_percent", soc);
  json.set("estimated_range_km", String(range, 1));

  if (gps.location.isValid()) {
    json.set("latitude", String(gps.location.lat(), 6));
    json.set("longitude", String(gps.location.lng(), 6));
  }
  
  json.set("last_update", ".sv");
  
  String path = "/vehicle_1/realtime_status";
  Serial.printf("Sending Status (System %s)... ", isSystemOn ? "ON" : "OFF");
  if (Firebase.setJSON(fbdo, path, json)) {
    Serial.println("SUCCESS");
  } else {
    Serial.print("FAILED: ");
    Serial.println(fbdo.errorReason());
  }
}

// --- (ส่วน setup) ---
void setup() {
  Serial.begin(115200);
  Serial.println("\nUltimate Firmware (Stable Version) Initializing...");

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(RELAY_PIN_SYSTEM, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN_SYSTEM, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    Serial.println("FATAL ERROR: Didn't find PN532 board. Halting.");
    while (1) { delay(10); }
  }
  nfc.SAMConfig();
  Serial.println("PN532 board found!");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println(" Connected!");

config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  if (!Firebase.beginStream(stream, "/vehicle_1/security/is_locked")) {
    Serial.printf("Stream begin error: %s\n", stream.errorReason().c_str());
  }
  Firebase.setStreamCallback(stream, streamCallback, streamTimeoutCallback);
  
  Serial.println("System Ready. Scan your card to turn ON/OFF.");
}

void loop() {
  // --- งานที่ 1: จัดการ RFID ---
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 200);

  if (success) {
    String scannedUID = "";
    for (uint8_t i = 0; i < uidLength; i++) {
        if (uid[i] < 0x10) scannedUID += '0';
        scannedUID += String(uid[i], HEX);
        if (i < uidLength - 1) scannedUID += ' ';
    }
    scannedUID.toUpperCase();

    if (scannedUID == authorizedUID) {
      // --- (สำคัญ) ตรรกะการล็อก ---
      if (isRemotelyLocked && !isSystemOn) { 
        // กรณีที่ 1: รถถูกล็อกอยู่ และ ผู้ใช้พยายามจะ "เปิด" ระบบ
        Serial.println("ACTION BLOCKED: Vehicle is remotely locked!");
        beepFail();
      } else { 
        // กรณีที่ 2: รถไม่ได้ถูกล็อก หรือ ผู้ใช้พยายามจะ "ปิด" ระบบ
        // ให้ทำงานสลับสถานะได้ตามปกติ
        isSystemOn = !isSystemOn;
        digitalWrite(RELAY_PIN_SYSTEM, isSystemOn ? LOW : HIGH);
        Serial.printf("System state is now %s\n", isSystemOn ? "ON" : "OFF");
        
        Firebase.setBool(fbdo, "/vehicle_1/realtime_status/is_on", isSystemOn);
          // --- (สำคัญ) ตรรกะการบันทึก Trip Log ---
        if (isSystemOn) { // ถ้าเพิ่งเปิดระบบ
          beepSuccess();
          currentTripId = "trip_" + String(millis());
          String tripPath = "/vehicle_1/trip_logs/" + currentTripId;
          
          // ส่งข้อมูลแต่ละส่วนแยกกัน
          Firebase.setTimestamp(fbdo, tripPath + "/start_time");
          Firebase.setString(fbdo, tripPath + "/user_uid", scannedUID);

        } else { // ถ้าเพิ่งปิดระบบ
          beepSystemOff();
          String tripPath = "/vehicle_1/trip_logs/" + currentTripId;
          Firebase.setTimestamp(fbdo, tripPath + "/end_time");
        }
        // ----------------------------------------------------
      }
    } else {
      beepFail();
    }
  }

  // --- งานที่ 2: อ่านข้อมูล GPS ---
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  
  // --- งานที่ 3: ส่งข้อมูลสถานะขึ้น Firebase (ทำงานตลอดเวลาด้วยความถี่เดียว) ---
  if (millis() - lastSendTime > UPDATE_INTERVAL_MS) {
    lastSendTime = millis();
    sendStatusToFirebase();
  }
  // -----------------------------
}