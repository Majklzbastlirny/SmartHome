#include <Adafruit_BMP085.h>
#include <Adafruit_PCF8574.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <InfluxDbClient.h>
#include <OpenTherm.h>
#include <Simpletimer.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"



//TIME
#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"
short YEar = 0;
short Month = 0;
short Day = 0;
short WeekDay = 0;
short Hour = 0;
short Minute = 0;
short Second = 0;
char Seconds[16];
char Minutes[16];
char Hours[16];
char daysOfTheWeek[7][12] = { "Ne", "Po", "Ut", "St", "Ct", "Pa", "So" };
char months[12][12] = { "Prosinec", "Leden", "Unor", "Brezen", "Duben", "Kveten", "Cerven", "Cervenec", "Srpen", "Zari", "Rijen", "Listopad" };


//NEOPIXELS
#define PIN 2        // Neopixel GPIO
#define NUMPIXELS 6  // Number of pixels
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


LiquidCrystal_I2C lcd(0x27, 20, 4);

//BUTTONS
Adafruit_PCF8574 pcf;
bool UpButton = 0;
bool DownButton = 0;
bool LeftButton = 0;
bool RightButton = 0;
bool OKButton = 0;
bool MenuButton = 0;
bool ExitButton = 0;

//OpenTherm
const int OTinPin = 12;
const int OToutPin = 14;
OpenTherm ot(OTinPin, OToutPin);
int OTStatusNeopixel = 0;
bool DWHstatus = 0;
bool CHstatus = 0;
float sp = 23,                     //set point
  pv = 0,                          //current temperature
  pv_last = 0,                     //prior temperature
  ierr = 0,                        //integral error
  dt = 0,                          //time between measurements
  op = 0;                          //PID controller output
unsigned long ts = 0, new_ts = 0;  //timestamp


//BMP180
Adafruit_BMP085 bmp;
float LocalTemp = 0;
float voltage = 0;

//WIFI
const char* ssid = "Edma_Patro";
const char* password = "pes_Fido";
const char* hostname = "OpenThermGW";
WiFiClient client;

byte MaxStrengh[] = {
  B00011,
  B00011,
  B00011,
  B01111,
  B01111,
  B01111,
  B11111,
  B11111
};
byte MediumStrengh[] = {
  B00000,
  B00000,
  B00000,
  B00110,
  B00110,
  B00110,
  B11110,
  B11110
};
byte LowStrengh[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11000,
  B11000
};
byte NoStrengh[] = {
  B10001,
  B01010,
  B01010,
  B00100,
  B00100,
  B01010,
  B01010,
  B10001
};

//MQTT
#define MQTT_BROKER_IP "192.168.0.8"
#define MQTT_BROKER_PORT 1883  //default port is 1883
#define MQTT_USERNAME "admin"
#define MQTT_PASSWORD "123456780"

bool MQTTstate = 0;
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_USERNAME, MQTT_PASSWORD);
Adafruit_MQTT_Subscribe TempRequ = Adafruit_MQTT_Subscribe(&mqtt, "Chalupa/TempReq");
Adafruit_MQTT_Subscribe TempCurr = Adafruit_MQTT_Subscribe(&mqtt, "Chalupa/TempCur");


//timers
Simpletimer OTtimer{};
Simpletimer RefreshTimer{};
Simpletimer Datetimer{};

void IRAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pixels.begin();
  ot.begin(handleInterrupt);


  if (!pcf.begin(0x38, &Wire)) {
    while (1)
      ;
  }

  if (!bmp.begin(0x77)) {
    while (1)
      ;
  }



  //LCD init routine
  lcd.begin();
  lcd.clear();
  lcd.backlight();

  //Neopixel init routine
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));

  for (int i = 1; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
    delay(10);
  }

  WiFi.hostname(hostname);
  WiFi.begin(ssid, password);

  unsigned long startMillis = millis();
  unsigned long currentMillis = 0;
  const unsigned long timeout = 5000;  // 5 seconds

  int dotCount = 0;
  int dotMax = 10;

  lcd.setCursor(1, 1);
  lcd.print("Connecting to WiFi");
  MQTT_connect();

  while (WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - startMillis >= timeout) {
      break;
    }
    pixels.setPixelColor(1, pixels.Color(50, 0, 0));
    pixels.show();
    lcd.setCursor(dotCount + 5, 2);
    lcd.print(".");
    delay(350);
    lcd.setCursor(dotCount + 5, 2);
    lcd.print(" ");
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.show();
    dotCount++;
    if (dotCount >= dotMax) {
      dotCount = 0;
    }
  }
  lcd.setCursor(0, 3);
  timeSync(TZ_INFO, "time.cloudflare.com");  //192.168.0.1

  lcd.print(WiFi.localIP());
  delay(25);
  delay(100);




  Adafruit_MQTT_Subscribe* subscription;
  mqtt.subscribe(&TempRequ);
  delay(10);
  mqtt.subscribe(&TempCurr);
  delay(20);
  Get_Data(10000);

  lcd.clear();
  ShowDateTime();
}



void loop() {
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));











  delay(100);
  UpdateButtons();

  if (Datetimer.timer(30000)) {
    ShowDateTime();
  }
  if (RefreshTimer.timer(5000)) {
    ReadBatVolt();
    WifiStrengh();
    UpdatePixels();
    MQTT_connect();
    Get_Data(1000);
    PrintTemps();
  }

  if (OTtimer.timer(750)) {
    OT();
  }
}

void UpdateButtons() {
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
  UpButton = !pcf.digitalRead(0);     //gpio 0
  DownButton = !pcf.digitalRead(1);   //gpio 1
  LeftButton = !pcf.digitalRead(2);   //gpio 2
  RightButton = !pcf.digitalRead(3);  //gpio 3
  OKButton = !pcf.digitalRead(4);     //gpio 4
  MenuButton = !pcf.digitalRead(5);   //gpio 5 - vpravo nahoře
  ExitButton = !pcf.digitalRead(6);   //gpio 6 - vlevo nahoře

  if (UpButton || DownButton || LeftButton || RightButton || OKButton || MenuButton || ExitButton) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 75));
    pixels.show();
  }

  //Serial.println("update");
}

void ReadBatVolt() {
  int adcValue = analogRead(A0);  // read the analog input value
  //float voltage = adcValue / 1023.0 * 1.0;  // translate the 10-bit value to a voltage value between 0 and 1

  float voltage;
  if (adcValue >= 936) {
    voltage = 5.0 + (adcValue - 936) * (1.0 - 5.0) / (1023 - 936);
  } else if (adcValue >= 557) {
    voltage = 3.0 + (adcValue - 557) * (5.0 - 3.0) / (933 - 557);
  } else {
    voltage = 2.0 + (adcValue - 366) * (3.0 - 2.0) / (557 - 366);
  }

  float percentage = (voltage - 3.0) * 100.0 / (4.2 - 3.0);
  percentage = round(percentage / 2) * 2;  //round to nearest 2%


  lcd.setCursor(15, 0);


  if (percentage > 100) {
    lcd.print("CHR");
  } else {
    lcd.print(String(percentage, 0) + "%");
  }
}


void ShowDateTime() {
  lcd.setCursor(0, 0);
  // time_t tnow = time(nullptr);
  //Serial.print("Čas nyní: ");
  //Serial.println(ctime(&tnow));

  time_t now;
  struct tm* timeinfo;
  time(&now);
  timeinfo = localtime(&now);
  YEar = ((timeinfo->tm_year) + 1900);
  Month = ((timeinfo->tm_mon) + 1);
  Day = (timeinfo->tm_mday);
  WeekDay = (timeinfo->tm_wday);
  Hour = (timeinfo->tm_hour);
  Minute = (timeinfo->tm_min);
  Second = (timeinfo->tm_sec);


  char bufferdate[32];
  sprintf(bufferdate, "%d.%d.", Day, Month);
  char buffertime[32];
  sprintf(Hours, "%02i", Hour);
  sprintf(Minutes, "%02i", Minute);
  sprintf(Seconds, "%02i", Second);
  sprintf(buffertime, "%s:%s", Hours, Minutes);

  lcd.print(String(daysOfTheWeek[WeekDay]) + " " + String(buffertime) + " " + String(bufferdate));
}

void PrintTemps() {
  lcd.setCursor(10, 1);
  lcd.print("Loc:");
  lcd.print(bmp.readTemperature());
  lcd.print("C");

  lcd.setCursor(10, 2);
  lcd.print("Cur:");
  lcd.print((char*)TempCurr.lastread);
  lcd.print("C");

  lcd.setCursor(10, 3);

  lcd.print("Req:");
  lcd.print((char*)TempRequ.lastread);
  lcd.print("C");
}


void WifiStrengh() {
  lcd.setCursor(19, 0);

  int wifiStrength = WiFi.RSSI();
  //Serial.println(wifiStrength);
  if (WiFi.status() == WL_CONNECTED) {
    //Serial.println("con");

    if (wifiStrength > -50) {
      lcd.createChar(0, MaxStrengh);

      lcd.setCursor(19, 0);

      lcd.write((byte)0);
    } else if (wifiStrength > -70) {
      lcd.createChar(0, MediumStrengh);
      lcd.setCursor(19, 0);

      lcd.write((byte)0);
    } else {
      lcd.createChar(0, LowStrengh);
      lcd.setCursor(19, 0);

      lcd.write((byte)0);
    }
  } else {
    //Serial.println("noncon");

    lcd.createChar(0, NoStrengh);
    lcd.setCursor(19, 0);

    lcd.write((byte)0);
  }
}
//PLACE FOR NEOPIXEL CONTROL
void UpdatePixels() {
  WifiStatus();
  OTStatus();
  MQTTStatus();
  HeatState();

  pixels.show();
}
//NEOPIXEL 1 = WIFI state
void WifiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    pixels.setPixelColor(1, pixels.Color(0, 0, 50));
  } else {
    pixels.setPixelColor(1, pixels.Color(50, 0, 0));
  }
  pixels.show();
}

//NEOPIXEL 2 = Opentherm state
void OTStatus() {
  if (OTStatusNeopixel == 1) {
    pixels.setPixelColor(2, pixels.Color(0, 0, 50));

  } else if (OTStatusNeopixel == 2) {
    pixels.setPixelColor(2, pixels.Color(0, 50, 50));
  }

  else {
    pixels.setPixelColor(2, pixels.Color(50, 0, 0));
  }
  pixels.show();
}

//NEOPIXEL 3 = comm state (mqtt/db)
void MQTTStatus() {
  if (MQTTstate == 1) {
    pixels.setPixelColor(3, pixels.Color(0, 0, 50));

  } else {
    pixels.setPixelColor(3, pixels.Color(50, 0, 0));
  }
  pixels.show();
}
//NEOPIXEL 4 = mqtt data receive


//NEOPIXEL 5 = kotel state (Hotwater, DWH, both or non)
void HeatState() {
  if (CHstatus == 0 && DWHstatus == 0) {  //cold, blue
    pixels.setPixelColor(5, pixels.Color(0, 0, 50));
  } else if (CHstatus == 1 && DWHstatus == 1) {  //all, combination of red and green
    pixels.setPixelColor(5, pixels.Color(50, 50, 0));
  } else if (CHstatus == 0 && DWHstatus == 1) {  //DWH, green
    pixels.setPixelColor(5, pixels.Color(0, 50, 0));
  } else if (CHstatus == 1 && DWHstatus == 0) {  //central heating, red
    pixels.setPixelColor(5, pixels.Color(50, 0, 0));
  }
  pixels.show();
}

//OpenTherm
void OT() {
    new_ts = millis();

  bool enableCentralHeating = true;
  bool enableHotWater = true;
  bool enableCooling = false;
  unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
  OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
  if (responseStatus == OpenThermResponseStatus::SUCCESS) {
    Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
    CHstatus = ot.isCentralHeatingActive(response) ? 1 : 0;
    Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
    DWHstatus = ot.isHotWaterActive(response) ? 1 : 0;
    Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));
    OTStatusNeopixel = 1;
  }
  if (responseStatus == OpenThermResponseStatus::NONE) {
    Serial.println("Error: OpenTherm is not initialized");
    OTStatusNeopixel = 2;
  } else if (responseStatus == OpenThermResponseStatus::INVALID) {
    Serial.println("Error: Invalid response " + String(response, HEX));
    OTStatusNeopixel = 3;
  } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
    Serial.println("Error: Response timeout");
    OTStatusNeopixel = 4;
  }

  //Set Boiler Temperature to 64 degrees C
      // pv1 = bmp.readTemperature();
    sp = atoi((char *)TempRequ.lastread);
    pv = atoi((char *)TempCurr.lastread);
    dt = (new_ts - ts) / 1000.0;
    ts = new_ts;
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      op = pid(sp, pv, pv_last, ierr, dt);
      //Set Boiler Temperature
      ot.setBoilerTemperature(op);
    }
    pv_last = pv;

}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    MQTTstate = 1;
    MQTTStatus();
    return;
  }
  MQTTstate = 0;
  MQTTStatus();
  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 1;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 2 seconds...");
    mqtt.disconnect();
    delay(2000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      MQTTstate = 0;
      MQTTStatus();
      break;
    }
  }
  Serial.println("MQTT Connected!");
  MQTTstate = 1;
  MQTTStatus();
}


void Get_Data(int scantime) {
  //
  Adafruit_MQTT_Subscribe* subscription;
  MQTTping();
  pixels.setPixelColor(4, pixels.Color(0, 25, 25));
  pixels.show();
  mqtt.subscribe(&TempRequ);
  mqtt.subscribe(&TempCurr);
  while ((subscription = mqtt.readSubscription(scantime))) {
    if (subscription == &TempRequ) {
      Serial.print(F("Got: "));
      pixels.setPixelColor(4, pixels.Color(0, 0, 50));
      pixels.show();
      Serial.println((char*)TempRequ.lastread);

    } else if (subscription == &TempCurr) {
      Serial.print(F("Got: "));
      pixels.setPixelColor(4, pixels.Color(0, 0, 50));
      pixels.show();
      Serial.println((char*)TempCurr.lastread);
    }
    Serial.println("Getting data");
  }
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));

  pixels.show();
}
void MQTTping() {
  if (!mqtt.ping()) {
    mqtt.disconnect();
  }
}


void SetHeater() {
  new_ts = millis();
  if (new_ts - ts > 1000) {
    //Set/Get Boiler Status
    bool enableCentralHeating = true;
    bool enableHotWater = true;
    bool enableCooling = false;
    unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
    if (responseStatus != OpenThermResponseStatus::SUCCESS) {
      Serial.println("Error: Invalid boiler response " + String(response, HEX));
    }

    // pv1 = bmp.readTemperature();
    sp = atoi((char *)TempRequ.lastread);
    pv = atoi((char *)TempCurr.lastread);
    dt = (new_ts - ts) / 1000.0;
    ts = new_ts;
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      op = pid(sp, pv, pv_last, ierr, dt);
      //Set Boiler Temperature
      ot.setBoilerTemperature(op);
    }
    pv_last = pv;

    //bmp.readTemperature();  //async temperature request

    // publish_temperature();
  }
}

float pid(float sp, float pv, float pv_last, float &ierr, float dt) {
  //Get_Data(0);
  float Kc = 10.0;    // K / %Heater
  float tauI = 50.0;  // sec
  float tauD = 1.0;   // sec
  // PID coefficients
  float KP = Kc;
  float KI = Kc / tauI;
  float KD = Kc * tauD;
  // upper and lower bounds on heater level
  float ophi = 100;
  float oplo = 0;
  // calculate the error
  float error = sp - pv;
  // calculate the integral error
  ierr = ierr + KI * error * dt;
  // calculate the measurement derivative
  float dpv = (pv - pv_last) / dt;
  // calculate the PID output
  float P = KP * error;  //proportional contribution
  float I = ierr;        //integral contribution
  float D = -KD * dpv;   //derivative contribution
  float op = P + I + D;
  // implement anti-reset windup
  if ((op < oplo) || (op > ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(ophi, op));
  }
  ierr = I;
  Serial.println("sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
  return op;
}

//Interrupts
