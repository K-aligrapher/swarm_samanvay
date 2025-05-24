#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Nothing Phone (3a)_1568";
const char* password = "alpha11@feb";

// MQTT broker IP address
const char* mqtt_server = "broker.hivemq.com";  // Change this to your broker IP

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer server(80);

// Web handler to serve control panel
void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
    <html>
    <head>
      <title>Master Drone Control</title>
      <style>
        input[type=range] { width: 300px; }
        button { margin: 5px; padding: 10px; }
      </style>
      <script>
        function runMotor(motor, dir) {
          let speed = document.getElementById('speed' + motor).value;
          fetch(/run?motor=${motor}&speed=${speed}&reverse=${dir});
        }
        function stopAll() {
          fetch('/stop');
        }
      </script>
    </head>
    <body>
      <h2>ESP32 Master Drone Control</h2>
      <div>
        Motor 1 Speed: <input type="range" id="speed1" min="0" max="255" value="128">
        <button onclick="runMotor(1, 0)">Forward</button>
        <button onclick="runMotor(1, 1)">Reverse</button><br>

        Motor 2 Speed: <input type="range" id="speed2" min="0" max="255" value="128">
        <button onclick="runMotor(2, 0)">Forward</button>
        <button onclick="runMotor(2, 1)">Reverse</button><br>

        Motor 3 Speed: <input type="range" id="speed3" min="0" max="255" value="128">
        <button onclick="runMotor(3, 0)">Forward</button>
        <button onclick="runMotor(3, 1)">Reverse</button><br>

        Motor 4 Speed: <input type="range" id="speed4" min="0" max="255" value="128">
        <button onclick="runMotor(4, 0)">Forward</button>
        <button onclick="runMotor(4, 1)">Reverse</button><br><br>

        <button onclick="stopAll()">Stop All Motors</button>
      </div>
    </body>
    </html>
  )rawliteral");
}

// Send motor command over MQTT
void handleRun() {
  if (!server.hasArg("motor") || !server.hasArg("speed") || !server.hasArg("reverse")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  int motor = server.arg("motor").toInt();
  int speed = server.arg("speed").toInt();
  bool reverse = server.arg("reverse") == "1";

  String payload = String(motor) + "," + String(speed) + "," + String(reverse ? 1 : 0);
  mqttClient.publish("drone/command", payload.c_str());

  server.send(200, "text/plain", "Command sent: " + payload);
}

// Send stop all command
void stopAllMotors() {
  mqttClient.publish("drone/command", "stop");
  server.send(200, "text/plain", "Stop command sent");
}

// Connect to WiFi
void setupWiFi() {
  delay(100);
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
}

// Connect to MQTT broker
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("MasterDroneClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 2 seconds");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  setupWiFi();

  mqttClient.setServer(mqtt_server, 1883);

  server.on("/", handleRoot);
  server.on("/run", handleRun);
  server.on("/stop", stopAllMotors);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  server.handleClient();
}