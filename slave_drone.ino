#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// Motor A pins (IN1, IN2) for each motor
#define M1_IN1 25
#define M1_IN2 26
#define M2_IN1 27
#define M2_IN2 14
#define M3_IN1 13
#define M3_IN2 12
#define M4_IN1 33
#define M4_IN2 32

// PWM configuration
#define PWM_FREQ 5000
#define PWM_RES 8

// Channels for each motor pin
#define CH_M1_IN1 0
#define CH_M1_IN2 1
#define CH_M2_IN1 2
#define CH_M2_IN2 3
#define CH_M3_IN1 4
#define CH_M3_IN2 5
#define CH_M4_IN1 6
#define CH_M4_IN2 7

// DRV8833 Enable pin
#define DRV_ENABLE_PIN 4  // Connect this GPIO to the EEP pin of DRV8833

const char* ssid = "Winterfell";
const char* password = "Tyrion@1234";
const char* mqtt_server = "broker.hivemq.com";  // Public broker

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer server(80);

void setupMotor(int in1, int in2, int ch1, int ch2) {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  ledcSetup(ch1, PWM_FREQ, PWM_RES);
  ledcSetup(ch2, PWM_FREQ, PWM_RES);
  ledcAttachPin(in1, ch1);
  ledcAttachPin(in2, ch2);
  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
}

void driveMotor(int motor, int speed, bool reverse = false) {
  speed = constrain(speed, 0, 255);
  int ch1, ch2;

  switch (motor) {
    case 1: ch1 = CH_M1_IN1; ch2 = CH_M1_IN2; break;
    case 2: ch1 = CH_M2_IN1; ch2 = CH_M2_IN2; break;
    case 3: ch1 = CH_M3_IN1; ch2 = CH_M3_IN2; break;
    case 4: ch1 = CH_M4_IN1; ch2 = CH_M4_IN2; break;
    default:
      Serial.println("Invalid motor number: " + String(motor));
      return;
  }

  if (reverse) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, speed);
  } else {
    ledcWrite(ch1, speed);
    ledcWrite(ch2, 0);
  }

  Serial.printf("Motor %d running %s at speed %d\n", motor, reverse ? "reverse" : "forward", speed);
}

void stopAllMotors() {
  for (int ch = 0; ch <= 7; ch++) {
    ledcWrite(ch, 0);
  }
  Serial.println("All motors stopped");
}

void publishStatus(const String& message) {
  mqttClient.publish("drone/status", message.c_str());
  Serial.println("Published: " + message);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("MQTT Message received: " + message);

  if (message == "stop") {
    stopAllMotors();
    publishStatus("All motors stopped");
    return;
  }

  int motor, speed, reverse;
  int parts = sscanf(message.c_str(), "%d,%d,%d", &motor, &speed, &reverse);
  if (parts == 3) {
    driveMotor(motor, speed, reverse == 1);
    publishStatus("Motor " + String(motor) + " running " + (reverse ? "reverse" : "forward") + " at speed " + String(speed));
  } else {
    publishStatus("Invalid command format: " + message);
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ChildDroneClient")) {
      Serial.println("connected");
      mqttClient.subscribe("drone/command");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 2 seconds");
      delay(2000);
    }
  }
}

void handleRoot() {
  server.send(200, "text/plain", "Child drone active and listening for MQTT commands.");
}

void setup() {
  Serial.begin(115200);

  // Enable DRV8833
  pinMode(DRV_ENABLE_PIN, OUTPUT);
  digitalWrite(DRV_ENABLE_PIN, HIGH);  // Enable driver

  // Setup motors
  setupMotor(M1_IN1, M1_IN2, CH_M1_IN1, CH_M1_IN2);
  setupMotor(M2_IN1, M2_IN2, CH_M2_IN1, CH_M2_IN2);
  setupMotor(M3_IN1, M3_IN2, CH_M3_IN1, CH_M3_IN2);
  setupMotor(M4_IN1, M4_IN2, CH_M4_IN1, CH_M4_IN2);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  server.handleClient();
}