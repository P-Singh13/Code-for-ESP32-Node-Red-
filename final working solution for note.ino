#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#define I2C_SLAVE_ADDR 0x04  // 4 in hexadecimal

MPU6050 mpu6050(Wire);

#define TRIG_PIN 23  // Connecting the Ultrasonic Sensor's TRIG pin to ESP32 pin GIOP26
#define ECHO_PIN 19  // Connecting the Ultrasonic Sensor's ECHO pin to ESP32 pin GIOP25
#define ledPin 17

// Replace the next variables with your SSID/Password combination
const char* ssid = "c6singh";
const char* password = "Bench006";

// Add MQTT Broker IP address
const char* mqtt_server = "192.168.137.194";


WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

float duration_us, distance_cm;
float Distance;
float temperature;
float AngleX, AngleY, AngleZ;
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;

int LMS, RMS, Angle;

void setup() {

  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ledPin, OUTPUT);
}


void setup_wifi() {

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* message, unsigned int length) {

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message

  if (String(topic) == "esp32/output") {
  Serial.println("------------");
  Serial.print("Distance reading from car 2 = ");
  Serial.println(messageTemp);
    
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  if (String(topic) == "esp32/output") {
    if (messageTemp == "go straight") {
      Serial.print("Going Forward");
      LMS = 255;
      RMS = 255;
      Angle = 94;
      transmission();
    }
  }
  if (String(topic) == "esp32/output") {
    if (messageTemp == "") {
      Serial.print("go back");
      LMS = -255;
      RMS = -255;
      Angle = 94;
      transmission();
    }
  }
  if (String(topic) == "esp32/output") {
    if (messageTemp == "CW_Turn") {
      Serial.print("turn anti-clockwise");
      LMS = 150;
      RMS = 120;
      Angle = 135;
      transmission();
    }
  }
  if (String(topic) == "esp32/output") {
    if (messageTemp == "ACW_Turn") {
      Serial.print("turn anti-clockwise");
      LMS = 120;
      RMS = 150;
      Angle = 45;
      transmission();
    }
  }
}

void reconnect() {

  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  Distance = HC_Distance();
  temperature = mpu6050.getTemp();
  AngleX = mpu6050.getAngleX();
  AngleY = mpu6050.getAngleY();
  AngleZ = mpu6050.getAngleZ();
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();
  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  // Convert the value to a char array
  char disString[8];
  dtostrf(Distance, 1, 2, disString);
  Serial.print("Distance : ");
  Serial.print(disString);
  Serial.println(" cm ");
  client.publish("esp32/distance", disString);

  // Convert the value to a char array
  char tempString[8];
  dtostrf(temperature, 1, 2, tempString);
  Serial.print("Temperature: ");
  Serial.println(tempString);
  client.publish("esp32/temperature", tempString);

  // Convert the value to a char array
  char angX_String[8];
  dtostrf(AngleX, 1, 2, angX_String);
  Serial.print("AngleX : ");
  Serial.println(angX_String);
  client.publish("esp32/angleX", angX_String);

  // Convert the value to a char array
  char angY_String[8];
  dtostrf(AngleY, 1, 2, angY_String);
  Serial.print("AngleY : ");
  Serial.println(angY_String);
  client.publish("esp32/angleY", angY_String);

  // Convert the value to a char array
  char angZ_String[8];
  dtostrf(AngleZ, 1, 2, angZ_String);
  Serial.print("AngleZ : ");
  Serial.println(angZ_String);
  client.publish("esp32/angleZ", angZ_String);

  // Convert the value to a char array
  char accX_String[8];
  dtostrf(accX, 1, 2, accX_String);
  Serial.print("accX : ");
  Serial.println(angX_String);
  client.publish("esp32/accX", accX_String);

  // Convert the value to a char array
  char accY_String[8];
  dtostrf(accY, 1, 2, accY_String);
  Serial.print("accY: ");
  Serial.println(angY_String);
  client.publish("esp32/accY", accY_String);

  // Convert the value to a char array
  char accZ_String[8];
  dtostrf(accZ, 1, 2, accZ_String);
  Serial.print("accZ : ");
  Serial.println(angZ_String);
  client.publish("esp32/accZ", accZ_String);

  // Convert the value to a char array
  char gyroX_String[8];
  dtostrf(gyroX, 1, 2, gyroX_String);
  Serial.print("gyroX : ");
  Serial.println(gyroX_String);
  client.publish("esp32/gyroX", gyroX_String);

  // Convert the value to a char array
  char gyroY_String[8];
  dtostrf(gyroY, 1, 2, gyroY_String);
  Serial.print("gyroY : ");
  Serial.println(gyroY_String);
  client.publish("esp32/gyroY", gyroY_String);

  // Convert the value to a char array
  char gyroZ_String[8];
  dtostrf(gyroZ, 1, 2, gyroZ_String);
  Serial.print("gyroZ : ");
  Serial.println(gyroZ_String);
  client.publish("esp32/gyroZ", gyroZ_String);
}

int HC_Distance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = 0.017 * duration_us;

  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  return (distance_cm);
}

void transmission() {
  int x = LMS;
  int y = RMS;
  int z = Angle;

  Wire.beginTransmission(I2C_SLAVE_ADDR);

  Wire.write((byte)((x & 0x0000FF00) >> 8));
  Wire.write((byte)(x & 0x000000FF));
  Wire.write((byte)((y & 0x0000FF00) >> 8));
  Wire.write((byte)(y & 0x000000FF));
  Wire.write((byte)((z & 0x0000FF00) >> 8));
  Wire.write((byte)(z & 0x000000FF));

  Wire.endTransmission();
}
