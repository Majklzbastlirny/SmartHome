#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_AHT10.h>
#include <RotaryEncoder.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <InfluxDbClient.h>
#include <ESP_EEPROM.h>
#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"

const long EditInterval = 2000;           // interval at which to blink (milliseconds)
bool EditMode;


#define ENCa 14
#define ENCb 13
#define SW2 12
bool SW2en = 0;
int SW2time = 0;
#define SW1 2
bool SW1en = 0;
int SW1time = 0;
double ActionIdle = 0;
static const unsigned long idlelimit = 7500; // ms
static unsigned long lastRefreshTime = 0;

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHT10 aht;
WiFiManager wifiManager;

short Year = 0;
short Month = 0;
short Day = 0;
short WeekDay = 0;
short Hour = 0;
short Minute = 0;
short Second = 0;
char Seconds[16];
char Minutes[16];
char Hours[16];
char daysOfTheWeek[7][12] = {"Ne", "Po", "Ut", "St", "Ct", "Pa", "So"};
char months[12][12] = {"Prosinec", "Leden", "Unor", "Brezen", "Duben", "Kveten", "Cerven", "Cervenec", "Srpen", "Zari", "Rijen", "Listopad"};

double requestedTEMP = 20;
double MINtemp = 10;
double MAXtemp = 26;
int posmem;
int DirE = 0;

float count;
float Batcount = 80021;
double Voltage;
int POSITION = 0;
static int pos = 0;
int FirstRun = 0;
double memory;
bool defaultrequest = 0;
bool wifiless = 0;
double EditStartTime = 0;

RotaryEncoder *encoder = nullptr;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(16);
  encoder = new RotaryEncoder(ENCa, ENCb, RotaryEncoder::LatchMode::TWO03);
  if (digitalRead(SW2) == 0) {
    wifiless = 1;
  }
  else wifiless = 0;
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ENCa), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCb), checkPosition, CHANGE);


  delay(500);
  Serial.println(" ");
  if (! aht.begin()) {
    Serial.println("Could not find AHT10? Check wiring");
    while (1) delay(10);
  }
  delay(150); // wait for the OLED to power up
  display.begin(i2c_Address, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(100, 0);
  display.println("Booting up....");
  ReviveValues(0);
  display.display();
  if (wifiless == 0) {
    wifiManager.autoConnect("Thermostat", "123456780");
    timeSync(TZ_INFO, "time.cloudflare.com");
  }
}

void loop() {



  count++;
  attachInterrupt(digitalPinToInterrupt(SW1), BUT1, FALLING);

  encoder->tick();

  if (EditMode == 1) {
    //Serial.println("EDITMODE");
    if ((millis() - ActionIdle) > idlelimit) {
      EditMode = 0;
      FirstRun = 0;
      encoder->setPosition(posmem);
      defaultrequest = 1;
      //SaveValues(0);
    }

  }


  else if (EditMode == 0) {
    int newPos = encoder->getPosition();
    if (pos != newPos) {
      pos = newPos;
    }
    POSITION = pos / 2;
    //Serial.println(POSITION);
  }

  if (count > 80) {
    count = 0;
    //Serial.println( (millis() / 1000) % 3600);
    // Serial.println("pos " + String(pos) + ", newPos " + String(newPos) + ", POSITION " + String(POSITION));
    display.clearDisplay();
    ShowBat();
    ShowDateTime();





    //SWITCHES

    if (digitalRead(SW2) == 0) {
      ActionIdle = millis();
      //Serial.println(String(SW2en)+" "+String(SW2)+" "+String(SW2en)+" "+String(SW2time)+" "+String(SW2time + EditInterval));
      if (SW2en == 0) {
        SW2en = 1;
        SW2time = millis();
      }
      if ((SW2time + EditInterval) < millis()) {
        posmem = pos;
        EditMode = 1;
        SW2en = 0;
        EditStartTime = millis();
      }

      if (EditMode == 1 && (EditStartTime + 1000) < millis() ) { 
        EditMode = 0;
        FirstRun = 0;
        encoder->setPosition(posmem);
        SaveValues(0);
      }
    }
    else {
      SW2en = 0;
    }







    display.drawLine(0, 9, 127, 9, SH110X_WHITE);

    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    if (EditMode == 1) {
      display.setTextSize(1);
      display.setCursor(90, 0);
      display.println("E");
      // display.println((int)(encoder->getDirection()));

    }
    else {

    }


    if (POSITION == 0) {

      //Serial.println("Temp: " + String(temp.temperature) + "°C, Relative humidity: " + String(humidity.relative_humidity) + " %");
      display.setTextSize(2);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 12);
      display.print("Tem: ");
      display.println(temp.temperature);
      display.print("Hum: ");
      display.println(humidity.relative_humidity);
      display.setTextSize(1);
      display.print("SW1 " + String(digitalRead(SW1)));
      display.println(", SW2 " + String(digitalRead(SW2)));

    }

    else if (POSITION == 1) { //
      // (int)encoder->getDirection();
      display.setTextSize(1);
      display.setCursor(0, 12);

      if (EditMode == 1) {
        DirE = 0;
        if (FirstRun == 0) {
          (int)encoder->getDirection();
          memory = requestedTEMP;
        }
        FirstRun++;
        DirE = (int)encoder->getDirection();
        //delay(25);
        //temporary fix
        Serial.println("Dir " + String(DirE));
        if (DirE == 1) {
          requestedTEMP += 0.25;
        }
        else if (DirE == -1) {
          requestedTEMP += -0.25;
        }
        if (defaultrequest == 1) {
          requestedTEMP = memory;
          defaultrequest = 0;
        }
      }
      
      if (EditMode == 0 && defaultrequest == 1){
        defaultrequest = 0;
        requestedTEMP = memory;
      }

      requestedTEMP = constrain(requestedTEMP,MINtemp, MAXtemp);
      display.println("Temperature:\n");
      display.setTextSize(2);
      display.println("Cur: " + String(temp.temperature));
      display.println("Req: " + String(requestedTEMP));
      display.setTextSize(1);
    }

    else if (POSITION == -1) {
      display.setCursor(0, 12);
      String ssid;
      int32_t rssi;
      uint8_t encryptionType;
      uint8_t* bssid;
      int32_t channel;
      bool hidden;
      int scanResult;

      Serial.println(F("Starting WiFi scan..."));

      scanResult = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);

      if (scanResult == 0) {
        Serial.println(F("No networks found"));
      } else if (scanResult > 0) {
        Serial.printf(PSTR("%d networks found:\n"), scanResult);

        // Print unsorted scan results
        for (int8_t i = 0; i < scanResult; i++) {
          WiFi.getNetworkInfo(i, ssid, encryptionType, rssi, bssid, channel, hidden);

          display.printf(PSTR("%ddBm %s\n"), rssi, ssid.c_str());
          delay(0);
        }
      } else {
        Serial.printf(PSTR("WiFi scan error %d"), scanResult);
      }
    }
    else {

      display.setCursor(0, 12);
      display.setTextSize(1);
      display.println("AMOGUS " + String(POSITION));
      display.println("edistatus" + String(EditMode));
      display.setTextSize(1);
      display.println("time" + String(ActionIdle));
      display.println(posmem);
    }

    display.display();
    attachInterrupt(digitalPinToInterrupt(SW1), BUT1, FALLING);

  }
}

void ShowBat() {

  Batcount++;
  if (Batcount > 25) {
    Batcount = 0;
    Voltage = (((4.03 * analogRead(A0)) / 762) - 2.75);
    Serial.println("NOW: " + String(Voltage));
  }


  display.setCursor(100, 0);

  //Serial.println("Voltage: " + String(Voltage) + " " +  String(analogRead(A0)) + String(Voltage * 77,0) + "%");
  //0,96 = 3,75
  //4,02v = 1,26

  if (Voltage > 1.5) {
    display.println("Chr");
  }
  else {
    display.println(String((Voltage) * 76, 0) + "%");
  }

}




void ShowDateTime() {
  display.setCursor(0, 0);
  time_t tnow = time(nullptr);
  //Serial.print("Čas nyní: ");
  //Serial.println(ctime(&tnow));

  time_t now;
  struct tm * timeinfo;
  time(&now);
  timeinfo = localtime(&now);
  Year  = ((timeinfo->tm_year) + 1900);
  Month = ((timeinfo->tm_mon) + 1);
  Day = (timeinfo->tm_mday);
  WeekDay = (timeinfo->tm_wday);
  Hour = (timeinfo->tm_hour);
  Minute = (timeinfo->tm_min);
  Second = (timeinfo->tm_sec);
  // Serial.println(Year);
  // Serial.println(Month);
  // Serial.println(Day);
  // Serial.println(WeekDay);
  // Serial.println(Hour);
  // Serial.println(Minute);
  // Serial.println(Second);

  char bufferdate[32];
  sprintf(bufferdate, "%d.%d.", Day, Month);
  char buffertime[32];
  sprintf(Hours, "%02i", Hour);
  sprintf(Minutes, "%02i", Minute);
  sprintf(Seconds, "%02i", Second);
  sprintf(buffertime, "%s:%s", Hours, Minutes);

  display.print(String( daysOfTheWeek[WeekDay]) + " " + String(buffertime) + " " + String(bufferdate));

}

void ReviveValues(bool tREV) {
  //requestedTEMP = EEPROM.read(0);
  EEPROM.get(0, requestedTEMP);
  Serial.println("Value was:" + String(requestedTEMP));
}
void SaveValues(bool tSAVE) {
  // EEPROM.update(0, requestedTEMP);
  EEPROM.put(0, requestedTEMP);//next has to be (4, ....) because requestedtemp == float (4bytes)

  EEPROM.commit();
  Serial.println("Saving value:" + String(requestedTEMP));
}


//ISR realm

IRAM_ATTR void checkPosition()
{
  ActionIdle = millis();
  encoder->tick(); // just call tick() to check the state.
  //DirE = (int)encoder->getDirection();
}

IRAM_ATTR void BUT1() {
  // if (digitalRead(SW1) == 0) {
  ActionIdle = millis();
  if (EditMode == 1) {
    defaultrequest = 1;
  }
  if (EditMode == 0) {
    encoder->setPosition(0);
  }
  detachInterrupt(digitalPinToInterrupt(SW1));
}
