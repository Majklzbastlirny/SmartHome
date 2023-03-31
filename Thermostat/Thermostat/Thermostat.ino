#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_AHT10.h>
#include <RotaryEncoder.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncTCP.h>

//#include <ESPAsyncWebServer.h>
#include <WiFiManager.h>
#include <InfluxDbClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <ESP_EEPROM.h>
#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"

const long EditInterval = 2000;           // interval at which to blink (milliseconds)
bool EditMode;

//mqtt
WiFiClient client;
//AsyncWebServer server(80);

#define MQTT_BROKER_IP    "192.168.0.3"
#define MQTT_BROKER_PORT  1883    //default port is 1883
#define MQTT_USERNAME     "Thermostat"
#define MQTT_PASSWORD     "123456780"

Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_USERNAME, MQTT_PASSWORD);
Adafruit_MQTT_Publish RequestedTemp = Adafruit_MQTT_Publish(&mqtt, "Chalupa/TempReq");
Adafruit_MQTT_Publish RealTemp = Adafruit_MQTT_Publish(&mqtt, "Chalupa/TempCur");
Adafruit_MQTT_Subscribe OverrideTemp = Adafruit_MQTT_Subscribe(&mqtt, "Chalupa/TempOverride");



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
bool TempIncreaseFlag = 0;
int TempIncrease = 1; //po dosažení teploty sepnutí, zvyž požadovanou o TempIncrease a sepni flag tempincreaes. až dosáhne TempReq + TempIncrease, flag nastav na 0 a vrať se na TempReq

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
int MQTTcount = 0;
double requestedTEMP = 20;
double MINtemp = 5;
double MAXtemp = 35;
int posmem;
int DirE = 0;
float mqtttemp = 0;
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

//Modeselect
int TherMode[] = {0, 0, 0, 0}; //0 = auto, 1 = Man, 2 = Away, 3 = OFF)

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
  MQTT_Connect();
  delay(50);
  Adafruit_MQTT_Subscribe *subscription;
  mqtt.subscribe(&OverrideTemp);





  
}


  



void loop() {

    MQTT_Connect();


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
    Adafruit_MQTT_Subscribe *subscription;



  if(requestedTEMP >= mqtttemp && TempIncreaseFlag == 0){
    TempIncreaseFlag = 1;
    requestedTEMP += TempIncrease;
    
  }
if(requestedTEMP <= mqtttemp && TempIncreaseFlag == 1){
  TempIncreaseFlag = 0;
  requestedTEMP -= TempIncrease;
}


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
        TempIncreaseFlag == 0;
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
    mqtttemp = temp.temperature;
    if (EditMode == 1) {
      display.setTextSize(1);
      display.setCursor(90, 0);
      display.println("E");
      // display.println((int)(encoder->getDirection()));

    }
    else {

    }
    //4 Módy. Auto (nastavení po-ne až  4 změny/den), manual (nastavit 1 a jede to nonstop), off (nastavit nejnižší teplotu), away (nastavuje se na určitý počet dnů a na určitou tepolotu. Udržuje danou teplotu po x dní dle nastavení. funguje pouze v modu MAN a AUTO)
    //future: Strana 0: Teplota teď, requested a v menším vlhkos), Strana -1: settings (zde se nastavuje mód (auto, man, away a off), Strana -2: away nastavení (délka a teplota), Strana 2 manual nastavení, Strana 3 nastavení automatiky (TBA)
    //později menu systém. ne "strany". otočí se doprava, zde se kliknutím SW2 zamkne na této straně. encoderem se vybere nastavení. dalším kliknutím SW2 se přejde do nastavení inviduální věci. pro editaci EDITMODE (5sek hold SW2)(nastavování funguje stejně jako nyní (bez interakce po dobu 5sekund se revertne nastavení, stisknutím SW2 se uloží,stisknutím SW1 se vrátí na předchozí value), po uložení pomocí SW1 se vrátí zpět do menu)
    /*
      Modeselect(-1)-Home(0)-Treepage(1)
      Modeselect: (e)
      Auto [█]
      Manu [░]
      Away [░]
      Off  [░]


      Home:
      Tem:xx,xx (size2)
      Req:xx,xx (size2)
      hum: xx,xx% (size1)

      Hodnoty NOW asi neintegrovat. zbytečnost

      Treepage: ---MANUAL settings---Now: xx,xxC
                |                 |-Req: xx,xxC (e)
                |
                |-Away settings---Temp---Now: xx,xxC
                |               |      |-Req: xx,xxC (e)
                |               |
                |               |-Time--AwayDays: xx (e)
                |
                |
                |-AUTO settings---Po(Ne)---S1(SX)---Enabled: [█] (e)
                |                                 |-TriggerTime: xx:xx
                |                                 |-Now: xx,xxC
                |                                 |-Req: xx,xxC (e)
                |
                |-OFF settings---Now: xx,xxC
                               |-Req: xx,xxC (e)




      (e) = editable
    */

    if (POSITION == 0) {

      //Serial.println("Temp: " + String(temp.temperature) + "°C, Relative humidity: " + String(humidity.relative_humidity) + " %");
      display.setTextSize(2);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 12);
      display.print("Tem: ");
      display.println(temp.temperature);
      display.print("Hum: ");
      display.println(humidity.relative_humidity);
      display.print("Req: ");
      display.println(requestedTEMP);
      display.setTextSize(1);



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

      if (EditMode == 0 && defaultrequest == 1) {
        defaultrequest = 0;
        requestedTEMP = memory;
      }

      requestedTEMP = constrain(requestedTEMP, MINtemp, MAXtemp);
      display.println("Temperature:\n");
      display.setTextSize(2);
      display.println("Cur: " + String(temp.temperature));
      display.println("Req: " + String(requestedTEMP));
      display.setTextSize(1);
    }
    else if ( POSITION == -1) { //modeselect
      display.setCursor(0, 12);
      display.println("AUTO: [" + String(TherMode[0]) + "]");
      display.println("MAN:  [" + String(TherMode[1]) + "]");
      display.println("AWAY: [" + String(TherMode[2]) + "]");
      display.println("OFF:  [" + String(TherMode[3]) + "]");



    }


    else if (POSITION == -2) {
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
      display.setTextSize(1);
      display.print("SW1 " + String(digitalRead(SW1)));
      display.println(", SW2 " + String(digitalRead(SW2)));
    }

    display.display();
    attachInterrupt(digitalPinToInterrupt(SW1), BUT1, FALLING);
    PublishMQTT(0);

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

void MQTT_Connect() {
  int8_t ret;
  //digitalWrite(LEDDIAG, HIGH );

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.println("Připojuji se k MQTT serveru ");

  uint8_t retries = 4;
  mqtt.connect();
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Nepodařilo se mi připojit k MQTT serveru. Zkusím to znovu za 5 sekund.");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for user to reset me
      // digitalWrite(SensorPWR, LOW);
      // digitalWrite(HeatPin, LOW);

      ESP.restart();
    }
  }
  Serial.println("Úspěšně připojeno k MQTT serveru!");
}

void PublishMQTT(int vari) {

  MQTTcount++;
  //Serial.println(MQTTcount);
  if (MQTTcount > 25) {
    MQTTcount = 0;
    Serial.println(F("Probíhá odesílání dat na server"));
    Serial.print(F("Probíhá odesílání teploty požadované:"));
    if (! RequestedTemp.publish(requestedTEMP)) {
      Serial.println(F(" Failed"));
    } else {
      Serial.println(F(" OK!"));
    }
    delay(10);

    Serial.print(F("Probíhá odesílání teploty reálné:"));
    if (! RealTemp.publish(mqtttemp)) {
      Serial.println(F(" Failed"));
    } else {
      Serial.println(F(" OK!"));
    }
    

  }
}
