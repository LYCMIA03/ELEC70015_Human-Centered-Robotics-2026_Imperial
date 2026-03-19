#include <SimpleFOC.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdarg.h>

#define OTAEnable
#ifdef OTAEnable
#include <WebServer.h>
#include <ElegantOTA.h>
#endif
// =========================
// WiFi / MQTT Config
// =========================
const char* ssid = "Xiaomi";
const char* password = "1234567890";
WebServer server(80);
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
char mqttClientId[40] = { 0 };
const char* mqtt_topic = "imperial/yh4222/esp32/test";
const char* mqtt_calib_topic = "imperial/yh4222/esp32/calib";
const char* mqtt_res_topic = "imperial/yh4222/esp32/res";
const char* mqtt_status_topic = "imperial/yh4222/esp32/status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// =========================
// Motor / Driver / Sensor
// =========================


BLDCMotor motor = BLDCMotor(4);
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 16, 17, 5);

Encoder sensor = Encoder(18, 19, 512);

Servo myServo;
const int servoPin = 13;
const int servoPulseMinUs = 500;
const int servoPulseMaxUs = 2500;
const unsigned long servoTiltHoldMs = 1000;
const float motorTargetNearThresholdRad = 0.17f;
const unsigned long motorTargetNearDelayMs = 500;
const float servoReturnSettledThresholdDeg = 1.0f;

float servoZeroAngleDeg = 112.0f;
float servoTiltAngleDeg = 55.0f;
float servoSpeedDegPerSec = 100.0f;
float servoCurrentAngleDeg = 112.0f;
float servoTargetAngleDeg = 112.0f;
float plateZeroAngleDeg = 0.0f;
bool servoReturnPending = false;
bool servoTiltReachedForHold = false;
bool servoTiltOnTargetPending = false;
bool motorNearTargetLatched = false;
bool mqttCommandExecuting = false;
bool mqttExecutionWaitForTiltBack = false;
unsigned long servoReturnAtMs = 0;
unsigned long servoMotionLastMs = 0;
unsigned long motorNearTargetAtMs = 0;
unsigned long statusLastPublishMs = 0;
const unsigned long statusPublishIntervalMs = 2000;
// float servoTiltAngleDeg = 80.0f;
int angleToPulseUs(float angleDegrees) {
  const float clamped = constrain(angleDegrees, 0.0f, 270.0f);
  const float span = static_cast<float>(servoPulseMaxUs - servoPulseMinUs);
  return static_cast<int>(servoPulseMinUs + span * (clamped / 270.0f));
}

void setServoTarget(float angleDegrees) {
  servoTargetAngleDeg = constrain(angleDegrees, 0.0f, 270.0f);
}

void maintainServoMotion() {
  const unsigned long now = millis();
  if (servoMotionLastMs == 0) {
    servoMotionLastMs = now;
    myServo.writeMicroseconds(angleToPulseUs(servoCurrentAngleDeg));
    return;
  }

  const unsigned long dtMs = now - servoMotionLastMs;
  if (dtMs == 0) {
    return;
  }
  servoMotionLastMs = now;

  float delta = servoTargetAngleDeg - servoCurrentAngleDeg;
  if (delta == 0.0f) {
    return;
  }

  const float maxStepDeg = servoSpeedDegPerSec * (static_cast<float>(dtMs) / 1000.0f);
  if (maxStepDeg <= 0.0f) {
    return;
  }

  float absDelta = delta;
  if (absDelta < 0.0f) {
    absDelta = -absDelta;
  }

  if (absDelta <= maxStepDeg) {
    servoCurrentAngleDeg = servoTargetAngleDeg;
  } else {
    servoCurrentAngleDeg += (delta > 0.0f) ? maxStepDeg : -maxStepDeg;
  }

  myServo.writeMicroseconds(angleToPulseUs(servoCurrentAngleDeg));
}

void publishStatus(const char* status) {
  Serial.print("STATUS: ");
  Serial.println(status);
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_status_topic, status);
  }
}

void publishStatusf(const char* format, ...) {
  char status[128];
  va_list args;
  va_start(args, format);
  vsnprintf(status, sizeof(status), format, args);
  va_end(args);
  publishStatus(status);
}

void publishPeriodicStatus() {
  if (!mqttClient.connected()) {
    return;
  }

  const unsigned long now = millis();
  if (now - statusLastPublishMs < statusPublishIntervalMs) {
    return;
  }
  statusLastPublishMs = now;

  char status[128];
  snprintf(
    status,
    sizeof(status),
    "STATE,target=%.3f,shaft=%.3f,return=%d,tilt=%d,scur=%.1f,stgt=%.1f,sspd=%.1f",
    motor.target,
    motor.shaft_angle,
    servoReturnPending ? 1 : 0,
    servoTiltOnTargetPending ? 1 : 0,
    servoCurrentAngleDeg,
    servoTargetAngleDeg,
    servoSpeedDegPerSec);
  mqttClient.publish(mqtt_status_topic, status);
}

#define GEAR_RATIO 5.0f

float plateZeroAngleRad() {
  return plateZeroAngleDeg * PI / 180.0f;
}

void moveServoToZero() {
  setServoTarget(servoZeroAngleDeg);
}

void triggerServoTilt() {
  setServoTarget(servoZeroAngleDeg - servoTiltAngleDeg);
  servoReturnPending = true;
  servoTiltReachedForHold = false;
  publishStatus("SERVO_TILT_TRIGGERED");
}

void maintainServoTilt() {
  if (!servoReturnPending) {
    return;
  }

  if (!servoTiltReachedForHold) {
    float remainingDeg = servoTargetAngleDeg - servoCurrentAngleDeg;
    if (remainingDeg < 0.0f) {
      remainingDeg = -remainingDeg;
    }
    if (remainingDeg <= 1.0f) {
      servoTiltReachedForHold = true;
      servoReturnAtMs = millis() + servoTiltHoldMs;
    }
    return;
  }

  const long remainingMs = static_cast<long>(servoReturnAtMs - millis());
  if (remainingMs <= 0) {
    moveServoToZero();
    servoReturnPending = false;
    servoTiltReachedForHold = false;
    publishStatus("SERVO_RETURNED_ZERO");
  }
}

void maintainTiltOnTarget() {
  if (!servoTiltOnTargetPending) {
    motorNearTargetLatched = false;
    return;
  }

  float errorRad = motor.target - motor.shaft_angle;
  if (errorRad < 0.0f) {
    errorRad = -errorRad;
  }

  if (errorRad <= motorTargetNearThresholdRad) {
    if (!motorNearTargetLatched) {
      motorNearTargetLatched = true;
      motorNearTargetAtMs = millis();
    } else if (millis() - motorNearTargetAtMs >= motorTargetNearDelayMs) {
      triggerServoTilt();
      servoTiltOnTargetPending = false;
      motorNearTargetLatched = false;
      Serial.println("Motor reached target and delay elapsed, servo tilt triggered.");
    }
  } else {
    motorNearTargetLatched = false;
  }
}

void maintainMqttExecutionLock() {
  if (!mqttCommandExecuting) {
    return;
  }

  if (!mqttExecutionWaitForTiltBack) {
    mqttCommandExecuting = false;
    return;
  }

  if (servoTiltOnTargetPending || servoReturnPending) {
    return;
  }

  float remainingDeg = servoZeroAngleDeg - servoCurrentAngleDeg;
  if (remainingDeg < 0.0f) {
    remainingDeg = -remainingDeg;
  }

  if (remainingDeg > servoReturnSettledThresholdDeg) {
    return;
  }

  mqttExecutionWaitForTiltBack = false;
  mqttCommandExecuting = false;
  publishStatus("EXEC_COMPLETED");
}

bool executeRes(int res) {
  float offsetRad = 0.0f;

  switch (res) {
    case 1:
      offsetRad = 0.0f;
      break;
    case 2:
      offsetRad = -PI / 2.0f;
      break;
    case 3:
      offsetRad = -PI;
      break;
    case 4:
      offsetRad = PI / 2.0f;
      break;
    default:
      Serial.print("Unsupported res value: ");
      Serial.println(res);
      publishStatusf("ERR_UNSUPPORTED_RES,res=%d", res);
      return false;
  }

  motor.target = (plateZeroAngleRad() + offsetRad) * GEAR_RATIO;
  servoTiltOnTargetPending = true;
  motorNearTargetLatched = false;
  Serial.print("Updated motor target from res ");
  Serial.print(res);
  Serial.print(": ");
  Serial.println(motor.target);
  publishStatusf("RES_APPLIED,res=%d,target=%.3f", res, motor.target);
  return true;
}

// =========================
// WiFi / MQTT Functions
// =========================
void start_wifi() {
  Serial.println("\n=================================");
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void wait_wifi() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n=================================");
  Serial.println(">>> WIFI SUCCESSFULLY CONNECTED <<<");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("=================================");
#ifdef OTAEnable
  server.on("/", []() {
    server.send(200, "text/plain", "ESP32 Ready");
  });
  ElegantOTA.begin(&server);
  server.begin();
#endif
}

void generateMqttClientId() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");

  const unsigned long randomPart =
    (static_cast<unsigned long>(random(0x10000)) << 16) | static_cast<unsigned long>(random(0x10000));

  snprintf(mqttClientId, sizeof(mqttClientId), "esp32-%s-%08lX", mac.c_str(), randomPart);
}

unsigned long mqttLastAttemptMs = 0;
const unsigned long mqttRetryIntervalMs = 5000;
unsigned long randomResLastMs = 0;
const unsigned long randomResIntervalMs = 5000;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char message[32];
  const unsigned int copyLength = min(length, static_cast<unsigned int>(sizeof(message) - 1));
  memcpy(message, payload, copyLength);
  message[copyLength] = '\0';

  const bool isControlTopic =
    strcmp(topic, mqtt_calib_topic) == 0 || strcmp(topic, mqtt_res_topic) == 0;
  if (isControlTopic && mqttCommandExecuting) {
    publishStatusf("MQTT_IGNORED_BUSY,topic=%s", topic);
    return;
  }

  if (strcmp(topic, mqtt_calib_topic) == 0) {
    const float receivedCalibrationDeg = atof(message);
    const float currentTargetPlateDeg = (motor.target / GEAR_RATIO) * 180.0f / PI;
    plateZeroAngleDeg = currentTargetPlateDeg + receivedCalibrationDeg;
    motor.target = plateZeroAngleRad() * GEAR_RATIO;
    servoTiltOnTargetPending = false;
    motorNearTargetLatched = false;
    Serial.print("Received calibration deg: ");
    Serial.println(receivedCalibrationDeg);
    Serial.print("Current target plate deg: ");
    Serial.println(currentTargetPlateDeg);
    Serial.print("Updated plateZeroAngleDeg: ");
    Serial.println(plateZeroAngleDeg);
    publishStatusf("CALIB_APPLIED,rx=%.2f,targetDeg=%.2f,zeroDeg=%.2f,target=%.3f",
                   receivedCalibrationDeg,
                   currentTargetPlateDeg,
                   plateZeroAngleDeg,
                   motor.target);
  } else if (strcmp(topic, mqtt_res_topic) == 0) {
    if (executeRes(atoi(message))) {
      mqttCommandExecuting = true;
      mqttExecutionWaitForTiltBack = true;
      publishStatus("EXEC_STARTED");
    }
  }
}

void mqttMaintain() {
  if (mqttClient.connected()) return;
  const unsigned long now = millis();
  if (now - mqttLastAttemptMs < mqttRetryIntervalMs) return;
  mqttLastAttemptMs = now;
  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(mqttClientId)) {
    Serial.println("\n=================================");
    Serial.println(">>> MQTT SUCCESSFULLY CONNECTED <<<");
    Serial.println("=================================");
    publishStatusf("MQTT_ONLINE,id=%s", mqttClientId);
    mqttClient.subscribe(mqtt_topic);
    mqttClient.subscribe(mqtt_calib_topic);
    mqttClient.subscribe(mqtt_res_topic);
    Serial.print("Listening on: ");
    Serial.println(mqtt_topic);
    Serial.print("Listening on: ");
    Serial.println(mqtt_calib_topic);
    Serial.print("Listening on: ");
    Serial.println(mqtt_res_topic);
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" retrying in 5s");
  }
}

void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}

// =========================
// Commander
// =========================
Commander command = Commander(Serial);
void doMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void doServo(char* cmd) {
  if (cmd[0] == 'Z') {
    servoZeroAngleDeg = atof(cmd + 1);
    moveServoToZero();
    publishStatusf("SERVO_ZERO_UPDATED,deg=%.2f", servoZeroAngleDeg);
  } else if (cmd[0] == 'T') {
    servoTiltAngleDeg = atof(cmd + 1);
    publishStatusf("SERVO_TILT_UPDATED,deg=%.2f", servoTiltAngleDeg);
  } else if (cmd[0] == 'V') {
    const float requestedSpeed = atof(cmd + 1);
    if (requestedSpeed > 0.0f) {
      servoSpeedDegPerSec = requestedSpeed;
      publishStatusf("SERVO_SPEED_UPDATED,dps=%.2f", servoSpeedDegPerSec);
    } else {
      publishStatus("ERR_SERVO_SPEED_INVALID");
    }
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(static_cast<unsigned long>(micros()));

  // --- Start WiFi in background, then init peripherals in parallel ---
  start_wifi();
  generateMqttClientId();
  Serial.print("MQTT client ID: ");
  Serial.println(mqttClientId);

  ESP32PWM::allocateTimer(1);
  myServo.attach(servoPin, servoPulseMinUs, servoPulseMaxUs);
  servoCurrentAngleDeg = constrain(servoZeroAngleDeg, 0.0f, 270.0f);
  servoTargetAngleDeg = servoCurrentAngleDeg;
  myServo.writeMicroseconds(angleToPulseUs(servoCurrentAngleDeg));
  servoMotionLastMs = millis();
  delay(500);
  triggerServoTilt();
  for (int i = 0; i < 120; i++) {
    maintainServoMotion();
    delay(10);
  }
  moveServoToZero();
  for (int i = 0; i < 120; i++) {
    maintainServoMotion();
    delay(10);
  }
  servoReturnPending = false;
  servoTiltReachedForHold = false;
  delay(500);

  SimpleFOCDebug::enable(&Serial);

  sensor.init();
  sensor.enableInterrupts(doA, doB);

  // Link sensor
  motor.linkSensor(&sensor);

  // =========================
  // Driver config
  // =========================
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 24;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  // =========================
  // Motor config
  // =========================
  motor.voltage_sensor_align = 3.0;
  motor.voltage_limit = 4.0;
  motor.velocity_limit = 22.0;

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  motor.P_angle.P = 10.0;
  motor.PID_velocity.P = 0.17;
  motor.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0;

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL;
  motor.monitor_downsample = 10;  // print every 10th loop

  // motor.PID_velocity.P = 0.12;
  // motor.PID_velocity.I = 0.005;
  motor.LPF_velocity.Tf = 0.04;

  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }

  if (!motor.initFOC()) {
    Serial.println("FOC init failed!");
    return;
  }

  // --- Wait for WiFi to finish, then configure MQTT ---
  wait_wifi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  motor.target = 0.0;

  command.add('M', doMotor, "Motor");
  command.add('S', doServo, "Servo (SZ<deg>=zero, ST<deg>=tilt, SV<deg/s>=speed)");

  Serial.println(F("Motor ready."));
  Serial.println(F("Use command M to set Target Angle (in radians)."));
  Serial.println(F("Use command SZ<deg> to set servo zero, ST<deg> to set tilt, SV<deg/s> to set speed (e.g. SZ100, ST15, SV40)."));
}

void loop() {
  mqttMaintain();
  mqttClient.loop();

  // const unsigned long now = millis();
  // if (now - randomResLastMs >= randomResIntervalMs) {
  //   randomResLastMs = now;
  //   executeRes(random(1, 5));
  // }

  motor.loopFOC();
  motor.move();
  maintainTiltOnTarget();
  maintainServoTilt();
  maintainServoMotion();
  maintainMqttExecutionLock();
  publishPeriodicStatus();
  command.run();
#ifdef OTAEnable
  ElegantOTA.loop();
#endif
  // motor.monitor();
}