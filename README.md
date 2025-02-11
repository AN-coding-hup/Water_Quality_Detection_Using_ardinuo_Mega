# Water_Quality_Detection_Using_ardinuo_Mega
Arduino Mega  Project
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

  // Initialize LCD
  lcd.begin(20, 4);
  lcd.backlight();
  
  // Display Welcome Message
  lcd.setCursor(0, 0);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Department of Zoology");
  lcd.setCursor(0, 2);
  lcd.print("The Islamia University");
  lcd.setCursor(0, 3);
  lcd.print("of Bahawalpur");
  delay(5000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IoT Based:");
  lcd.setCursor(0, 1);
  lcd.print("Determination of");
  lcd.setCursor(0, 2);
  lcd.print("Physicochemical");
  lcd.setCursor(0, 3);
  lcd.print("Parameters of Fish Ponds");
  delay(5000);
  lcd.clear();
  
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
}

void loop() {
  timer.run();

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  gravityTds.setTemperature(t);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  int turbidityValue = analogRead(turbidityPin);
  float turbidityVoltage = turbidityValue * (5.0 / 1024.0);
  float turbidity = -1120.4 * turbidityVoltage * turbidityVoltage + 5742.3 * turbidityVoltage - 4352.9;
  int gasValue = analogRead(gasPin);

  if (millis() - lastTime > 1000) {
    flowRate = 2.663 * pulse;
    totalVolume += flowRate;
    pulse = 0;
    lastTime = millis();
  }

  // pH Sensor Reading
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(A0);
    delay(30);
  }
  unsigned long avgval = 0;
  for (int i = 2; i < 8; i++) {
    avgval += buffer_arr[i];
  }
  float volt = (float)avgval * 5.0 / 1024 / 6;
  ph_act = -5.70 * volt + calibration_value;

  // Display Data in a Pattern
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print(" Temp: "); lcd.print(t); lcd.print(" C ");
  lcd.setCursor(0, 2);
  lcd.print(" Humidity: "); lcd.print(h); lcd.print("% ");
  lcd.setCursor(0, 3);
  lcd.print("********************");
  delay(5000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print(" pH: "); lcd.print(ph_act, 2);
  lcd.setCursor(0, 2);
  lcd.print(" TDS: "); lcd.print(tdsValue, 0); lcd.print(" ppm");
  lcd.setCursor(0, 3);
  lcd.print("********************");
  delay(5000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print(" Turbidity: "); lcd.print(turbidity, 1); lcd.print(" NTU");
  lcd.setCursor(0, 3);
  lcd.print("********************");
  delay(5000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print(" Flow: "); lcd.print(flowRate, 1); lcd.print(" mL/s");
  lcd.setCursor(0, 2);
  lcd.print(" Gas: "); lcd.print(gasValue);
  lcd.setCursor(0, 3);
  lcd.print("********************");
  delay(5000);
}

void increase() {
  pulse++;
}

void display_pHValue() {
  Serial.print("pH Value: ");
  Serial.println(ph_act, 2);
}
