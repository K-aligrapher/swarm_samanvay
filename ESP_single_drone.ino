
#include <WiFi.h>
#include <WebServer.h>

// Motor A pins (IN1, IN2) for each motor
#define M1_IN1 25
#define M1_IN2 26
#define M2_IN1 27
#define M2_IN2 14
#define M3_IN1 12
#define M3_IN2 13
#define M4_IN1 32
#define M4_IN2 33

// PWM configuration
#define PWM_FREQ 5000
#define PWM_RES 8

// Channels for each motor pin (need 8 total)
#define CH_M1_IN1 0
#define CH_M1_IN2 1
#define CH_M2_IN1 2
#define CH_M2_IN2 3
#define CH_M3_IN1 4
#define CH_M3_IN2 5
#define CH_M4_IN1 6
#define CH_M4_IN2 7

const char* ssid = "Nothing Phone (3a)_1568";
const char* password = "alpha11@feb";

WebServer server(80);

// Utility: Setup one motor's PWM
void setupMotor(int in1, int in2, int ch1, int ch2) {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  ledcSetup(ch1, PWM_FREQ, PWM_RES);
  ledcSetup(ch2, PWM_FREQ, PWM_RES);
  ledcAttachPin(in1, ch1);
  ledcAttachPin(in2, ch2);
  // Initially stopped
  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
}

void driveMotor(int motor, int speed, bool reverse = false) {
  // Clamp speed
  speed = constrain(speed, 0, 255);
  int ch1, ch2;

  switch (motor) {
    case 1: ch1 = CH_M1_IN1; ch2 = CH_M1_IN2; break;
    case 2: ch1 = CH_M2_IN1; ch2 = CH_M2_IN2; break;
    case 3: ch1 = CH_M3_IN1; ch2 = CH_M3_IN2; break;
    case 4: ch1 = CH_M4_IN1; ch2 = CH_M4_IN2; break;
    default: server.send(400, "text/plain", "Invalid motor"); return;
  }

  if (reverse) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, speed);
  } else {
    ledcWrite(ch1, speed);
    ledcWrite(ch2, 0);
  }

  server.send(200, "text/plain", "Motor " + String(motor) + " running " + (reverse ? "reverse" : "forward") + " at speed " + String(speed));
}

void stopAllMotors() {
  for (int ch = 0; ch <= 7; ch++) {
    ledcWrite(ch, 0);
  }
  server.send(200, "text/plain", "All motors stopped");
}

void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
    <html>
    <head>
      <title>DRV8833 Motor Control</title>
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
      <h2>ESP32 DRV8833 Motor Control (4 Motors)</h2>
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

void handleRun() {
  if (!server.hasArg("motor") || !server.hasArg("speed") || !server.hasArg("reverse")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  int motor = server.arg("motor").toInt();
  int speed = server.arg("speed").toInt();
  bool reverse = server.arg("reverse") == "1";
  driveMotor(motor, speed, reverse);
}

void setup() {
  Serial.begin(115200);

  // Setup motors with PWM
  setupMotor(M1_IN1, M1_IN2, CH_M1_IN1, CH_M1_IN2);
  setupMotor(M2_IN1, M2_IN2, CH_M2_IN1, CH_M2_IN2);
  setupMotor(M3_IN1, M3_IN2, CH_M3_IN1, CH_M3_IN2);
  setupMotor(M4_IN1, M4_IN2, CH_M4_IN1, CH_M4_IN2);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/run", handleRun);
  server.on("/stop", stopAllMotors);
  server.begin();
}

void loop() {
  server.handleClient();
}