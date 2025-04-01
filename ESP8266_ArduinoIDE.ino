#define BLYNK_TEMPLATE_ID "TMPL6eNQPuqG-"
#define BLYNK_TEMPLATE_NAME "smartplant"
#define BLYNK_AUTH_TOKEN "9Z6rcfqUpynFGgpKIHQA8ux2OnDdt9b2"

// Include the necessary libraries
#include <LiquidCrystal_I2C.h>
#define BLYNK_PRINT Serial  
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <WifiUdp.h>
#include <NTPClient.h>

// Initialize the LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WiFi and Blynk credentials
char auth[] = "9Z6rcfqUpynFGgpKIHQA8ux2OnDdt9b2";  
char ssid[] = "abcd";  // WiFi SSID
char pass[] = "abcd1234";  // WiFi Password

// NTP Client for time sync
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 0, 60000); 

// Initialize DHT11 sensor
DHT dht(D4, DHT11);  // DHT11 Temperature Sensor connected to pin D4

// Initialize Blynk timer
BlynkTimer timer;
WidgetRTC rtc;    // RTC Widget for Blynk

// Define component pins
#define soil A0      // Soil Moisture Sensor connected to pin A0
#define PIR D5       // PIR Motion Sensor connected to pin D5
#define RELAY_PIN_1  D3  // Relay connected to pin D3
#define PUSH_BUTTON_1 D7  // Push button connected to pin D7

// Blynk VPIN 
#define VPIN_START_HOUR V7
#define VPIN_START_MINUTE V8
#define VPIN_STOP_HOUR V9
#define VPIN_STOP_MINUTE V10
#define VPIN_TIMER_SWITCH V13
#define VPIN_BUTTON_1 V12
#define VPIN_AUTO_SWITCH V14

int relay1State = LOW;
int previousRelayState = LOW;
int PIR_ToggleValue;
int startHour = 0;    // Start Hour to water (Default)
int startMinute = 0;  // Start Minute (Default)
int stopHour = 0;     // Stop Hour (Default)
int stopMinute = 0;   // Stop Minute (Default)
bool pumpState = false;  // Water pump state is off (Default)
bool isTimerEnable = false; // Timer is disabled (Default)
bool autoEnable = false; // Set for auto watering with soil moisture sensor

// Functions to initialize and update the LCD
void initializeLCD() {
  lcd.clear();  
}

void updateLCDForRelay() {
  if (relay1State == HIGH) {
    lcd.setCursor(11, 1);
    lcd.print("W:ON ");
  } else if (relay1State == LOW) {
    lcd.setCursor(11, 1);
    lcd.print("W:OFF");
  }
}

// Function to get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t);

  lcd.setCursor(8, 0);
  lcd.print("H:");
  lcd.print(h);
}

BLYNK_WRITE(VPIN_AUTO_SWITCH){
  autoEnable = param.asInt();
  Serial.print("Auto mode: ");
  Serial.println(autoEnable ? "ENABLED" : "DISABLED");
}

// Function to get the soil moisture values
void soilMoistureSensor() {
  int value = analogRead(soil);
  value = map(value, 0, 1024, 0, 100);
  value = (value - 100) * -1;

  Blynk.virtualWrite(V3, value);
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(value);
  lcd.print(" ");
  // Auto mode on
  if (autoEnable){
    if (value < 20 && relay1State == LOW) {
      // If moisture is below 20, turn on the relay (water the plant)
      relay1State = HIGH;
      digitalWrite(RELAY_PIN_1, relay1State);
      lcd.setCursor(11, 1);
      lcd.print("W:ON ");
      Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);  // Sync with Blynk app
    } else if (value >= 60 && relay1State == HIGH) {
      // If moisture reaches 60 or more, turn off the relay (stop watering)
      relay1State = LOW;
      digitalWrite(RELAY_PIN_1, relay1State);
      lcd.setCursor(11, 1);
      lcd.print("W:OFF");
      Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);  // Sync with Blynk app
    }
  }
   
}

// PIR sensor function
void PIRsensor() {
  bool value = digitalRead(PIR);
  if (value) {
    Blynk.logEvent("pirmotion", "WARNING! Motion Detected!");
    WidgetLED LED(V5);
    LED.on();
  } else {
    WidgetLED LED(V5);
    LED.off();
  }  
}

// Button check and control relay manually
void checkPhysicalButton() {
  if (digitalRead(PUSH_BUTTON_1) == LOW) {
    relay1State = !relay1State;
    digitalWrite(RELAY_PIN_1, relay1State);
    Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
  }
}

BLYNK_WRITE(VPIN_BUTTON_1) {
  relay1State = param.asInt();
  digitalWrite(RELAY_PIN_1, relay1State);
}
BLYNK_WRITE(VPIN_TIMER_SWITCH) {
  isTimerEnable = param.asInt();
  Serial.print("Timer mode: ");
  Serial.println(isTimerEnable ? "ENABLED" : "DISABLED");
}
BLYNK_WRITE(V6) {
  PIR_ToggleValue = param.asInt();
}
BLYNK_WRITE(VPIN_START_HOUR) {
  startHour = param.asInt();
}

BLYNK_WRITE(VPIN_START_MINUTE) {
  startMinute = param.asInt();
}

BLYNK_WRITE(VPIN_STOP_HOUR) {
  stopHour = param.asInt();
}

BLYNK_WRITE(VPIN_STOP_MINUTE) {
  stopMinute = param.asInt();
}

// Timer function to control pump based on schedule
void checkSchedule() {
  if (isTimerEnable) {
    int currentHour = hour();
    int currentMinute = minute();

    if ((currentHour > startHour || (currentHour == startHour && currentMinute >= startMinute)) &&
        (currentHour < stopHour || (currentHour == stopHour && currentMinute < stopMinute))) {
      if (!pumpState) {
        pumpState = true;
        digitalWrite(RELAY_PIN_1, HIGH);  // Turn on the pump
        relay1State = HIGH;
        Serial.println("Pump turned ON by schedule");
        Blynk.virtualWrite(VPIN_BUTTON_1, 1);  // Sync pump state with Blynk
        updateLCDForRelay();
      }
    } else {
      if (pumpState) {
        pumpState = false;
        digitalWrite(RELAY_PIN_1, LOW);  // Turn off the pump
        relay1State = LOW;
        Serial.println("Pump turned OFF by schedule");
        Blynk.virtualWrite(VPIN_BUTTON_1, 0);  // Sync pump state with Blynk
        updateLCDForRelay();
      }
    }
  }
}

// Main setup function
void setup() {
  Serial.begin(9600);
  Wire.begin(D2, D1);
  lcd.init();
  lcd.backlight();
  
  pinMode(PIR, INPUT);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
  
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  rtc.begin();
  
  initializeLCD();

  Blynk.syncVirtual(VPIN_START_HOUR, VPIN_START_MINUTE, VPIN_STOP_HOUR, VPIN_STOP_MINUTE, VPIN_TIMER_SWITCH, VPIN_AUTO_SWITCH);

  timer.setInterval(100L, soilMoistureSensor);
  timer.setInterval(100L, DHT11sensor);
  timer.setInterval(500L, checkPhysicalButton);
  timer.setInterval(1000L, checkSchedule); // Check the schedule every 1 second
}

void loop() {
  if (relay1State != previousRelayState) {
    previousRelayState = relay1State;
    initializeLCD();
    updateLCDForRelay();
  }
  Blynk.run();
  timer.run();
}