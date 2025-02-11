// Water_Quality_Detection_Using_ardinuo_Mega
//Arduino Mega  Project
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "GravityTDS.h"
#include <SimpleTimer.h>

// LCD Configuration (20x4 I2C LCD)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// GSM Module (SIM800L) Configuration
#define SIM800L_RX 7
#define SIM800L_TX 8
SoftwareSerial sim800l(SIM800L_RX, SIM800L_TX);

// Bluetooth (HC-05) Configuration
#define HC05_TX 10
#define HC05_RX 11
SoftwareSerial bluetooth(HC05_TX, HC05_RX);

// DHT11 Configuration
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// TDS Sensor Configuration
#define TdsSensorPin A2
GravityTDS gravityTds;
float tdsValue = 0;

// Turbidity Sensor Configuration
#define turbidityPin A1

// Gas Sensor Configuration
#define gasPin A6
#define BUZZER_PIN 3

// Flow Sensor Configuration
#define flowPin 2
volatile long pulse = 0;
unsigned long lastTime = 0;
float flowRate = 0;
float totalVolume = 0;

// pH Sensor Configuration
SimpleTimer timer;
float calibration_value = 21.34 - 0.7;
int buffer_arr[10], temp;
float ph_act;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  sim800l.begin(9600);

  // Initialize LCD (20x4)
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Initialize Sensors
  dht.begin();
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(flowPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowPin), increase, RISING);

  timer.setInterval(500L, display_pHValue);
  delay(2000);
  lcd.clear();
}

void loop() {
  timer.run();

  // Read Temperature & Humidity
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Read TDS Sensor
  gravityTds.setTemperature(t);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();

  // Read Turbidity Sensor
  int turbidityValue = analogRead(turbidityPin);
  float turbidityVoltage = turbidityValue * (5.0 / 1024.0);
  float turbidity = -1120.4 * turbidityVoltage * turbidityVoltage + 5742.3 * turbidityVoltage - 4352.9;

  // Read Gas Sensor
  int gasValue = analogRead(gasPin);

  // Read Flow Sensor
  if (millis() - lastTime > 1000) {
    flowRate = 2.663 * pulse;
    totalVolume += flowRate;
    pulse = 0;
    lastTime = millis();
  }

  // Read pH Sensor
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(A0);
    delay(30);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer_arr[i] > buffer_arr[j]) {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }
  unsigned long avgval = 0;
  for (int i = 2; i < 8; i++) {
    avgval += buffer_arr[i];
  }
  float volt = (float)avgval * 5.0 / 1024 / 6;
  ph_act = -5.70 * volt + calibration_value;

  // Display Data Sequentially
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(h);
  lcd.print("%");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("pH: ");
  lcd.print(ph_act, 2);
  lcd.setCursor(0, 1);
  lcd.print("TDS: ");
  lcd.print(tdsValue, 0);
  lcd.print(" ppm");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Turbidity: ");
  lcd.print(turbidity, 1);
  lcd.print(" NTU");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Flow: ");
  lcd.print(flowRate, 1);
  lcd.print(" mL/s");
  lcd.setCursor(0, 1);
  lcd.print("Gas: ");
  lcd.print(gasValue);
  delay(2000);

  // Send Data via Bluetooth (HC-05)
  sendBluetoothData(t, h, ph_act, tdsValue, turbidity, flowRate, gasValue);

  // Send SMS Alert
  String smsMessage = "Temp: " + String(t) + " C, Hum: " + String(h) + " %, pH: " + String(ph_act, 2) +
                      " TDS: " + String(tdsValue, 0) + " ppm, Turb: " + String(turbidity, 1) +
                      " NTU, Flow: " + String(flowRate, 1) + " mL/s, Gas: " + String(gasValue);
  sendSMS("+923009673468", smsMessage);

  if (gasValue > 200) {
    analogWrite(BUZZER_PIN, 50);
    sendSMS("+1234567890", "GAS ALERT! Gas level is high.");
  } else {
    analogWrite(BUZZER_PIN, 0);
  }
}

// Interrupt for Flow Sensor
void increase() {
  pulse++;
}

// Function to Display pH Value in Serial Monitor
void display_pHValue() {
  Serial.print("pH Value: ");
  Serial.println(ph_act, 2);
}

// Function to Send SMS via GSM (SIM800L)
void sendSMS(String phoneNumber, String message) {
  sim800l.println("AT+CMGF=1");
  delay(1000);
  sim800l.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(1000);
  sim800l.print(message);
  delay(1000);
  sim800l.write(26);
  delay(1000);
  Serial.println("SMS Sent: " + message);
}

// Function to Send Data via Bluetooth (HC-05)
void sendBluetoothData(float temp, float hum, float ph, float tds, float turbidity, float flow, int gas) {
  bluetooth.print("Temp: ");
  bluetooth.print(temp);
  bluetooth.print(" C, Hum: ");
  bluetooth.print(hum);
  bluetooth.print("%, pH: ");
  bluetooth.print(ph, 2);
  bluetooth.print(", TDS: ");
  bluetooth.print(tds, 0);
  bluetooth.print(" ppm, Turb: ");
  bluetooth.print(turbidity, 1);
  bluetooth.print(" NTU, Flow: ");
  bluetooth.print(flow, 1);
  bluetooth.print(" mL/s, Gas: ");
  bluetooth.println(gas);

  Serial.println("Data sent via Bluetooth");
}
