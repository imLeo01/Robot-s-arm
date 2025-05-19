#include <WiFi.h>

const char* ssid = "ESP32-LED";
const char* password = "12345678";

const int ledPin = 8; // GPIO 8 cho ESP32-C3

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // IP tĩnh cho Access Point
  IPAddress local_IP(192, 168, 1, 1);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New Client.");
    String request = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;

        if (c == '\n') {
          if (request.indexOf("GET /ON") >= 0) {
            digitalWrite(ledPin, HIGH);
            Serial.println("LED ON");
          }
          if (request.indexOf("GET /OFF") >= 0) {
            digitalWrite(ledPin, LOW);
            Serial.println("LED OFF");
          }

          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<!DOCTYPE html><html><head><title>ESP32 LED</title></head>");
          client.println("<body><h1>Điều khiển Đèn ESP32</h1>");
          client.println("<p><a href=\"/ON\">Bật đèn</a></p>");
          client.println("<p><a href=\"/OFF\">Tắt đèn</a></p>");
          client.println("</body></html>");
          break;
        }
      }
    }

    client.stop();
    Serial.println("Client Disconnected.");
  }
}