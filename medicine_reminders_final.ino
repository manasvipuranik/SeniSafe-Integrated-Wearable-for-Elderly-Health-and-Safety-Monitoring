#include <WiFi.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// ====== WiFi Credentials ======
const char* ssid = "iPhone";            // <-- Replace with your WiFi SSID
const char* password = "76207610";    // <-- Replace with your WiFi password

WiFiServer server(80);

// ====== LCD Pins ======
const int rs = 25, en = 26, d4 = 27, d5 = 14, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// ====== MAX30102 Sensor Setup ======
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// ====== Patient Info ======
String statusHR = "Low";
String patientName = "Patient1";
String bedNumber = "1111";

void setup() {
  pinMode(2, OUTPUT); // Buzzer or LED
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);

  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Heart Rate Monit");
  lcd.setCursor(0, 1);
  lcd.print("   System");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM=");

  // ====== MAX30102 Initialization ======
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring/power.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // ====== WiFi Setup ======
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  long irValue = particleSensor.getIR();

  // ====== Heartbeat Detection ======
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // ====== LCD Display ======
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("     BPM=");
  lcd.print(beatAvg);

  // ====== Status & Alert ======
  if (beatAvg > 80) {
    digitalWrite(2, HIGH);
    statusHR = "High";
  } else {
    digitalWrite(2, LOW);
    statusHR = "Low";
  }

  if (irValue < 50000) {
    digitalWrite(2, LOW);
    lcd.setCursor(0, 0);
    lcd.print("  No Finger");
    statusHR = "No Finger";
  }

  // ====== Web Server ======
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Send HTML
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.println("<!DOCTYPE html><html lang='en'>");
            client.println("<head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>");
            client.println("<meta http-equiv='refresh' content='2'>");
            client.println("<title>Heart Rate Monitor</title>");
            client.println("<style>");
            client.println("body { font-family: 'Segoe UI', Tahoma, sans-serif; background: #f9f9f9; margin: 0; padding: 0; }");
            client.println(".card { max-width: 400px; background: #fff; margin: 40px auto; padding: 30px; border-radius: 16px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); text-align: center; }");
            client.println("h1 { margin-bottom: 10px; color: #333; }");
            client.println("p { font-size: 1.1em; margin: 8px 0; }");
            client.println(".status { font-weight: bold; font-size: 1.2rem; padding: 10px 20px; border-radius: 10px; display: inline-block; }");
            client.println(".high { background: #ffcccc; color: #c0392b; }");
            client.println(".low { background: #cce5ff; color: #2c7cd1; }");
            client.println(".nofinger { background: #fff3cd; color: #856404; }");
            client.println("</style></head><body>");
            client.println("<div class='card'>");
            client.println("<h1>Heart Rate Monitor</h1>");
            client.println("<p><strong>Patient:</strong> " + patientName + "</p>");
            client.println("<p><strong>Bed No:</strong> " + bedNumber + "</p>");
            client.println("<p><strong>BPM:</strong> " + String(beatAvg) + "</p>");
// Choose status style
            String statusClass = "low";
            if (statusHR == "High") statusClass = "high";
            else if (statusHR == "No Finger") statusClass = "nofinger";

            client.println("<p class='status " + statusClass + "'>Status: " + statusHR + "</p>");
            client.println("</div></body></html>");
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }


    client.stop();
    Serial.println("Client Disconnected");
  }
}