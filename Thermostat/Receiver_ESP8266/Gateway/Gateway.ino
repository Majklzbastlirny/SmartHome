#include <Adafruit_BMP085.h>
#include <Adafruit_PCF8574.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PIN 2        // Neopixel GPIO
#define NUMPIXELS 6  // Number of pixels
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


LiquidCrystal_I2C lcd(0x27, 20, 4);

Adafruit_PCF8574 pcf;
bool UpButton = 0;
bool DownButton = 0;
bool LeftButton = 0;
bool RightButton = 0;
bool OKButton = 0;
bool MenuButton = 0;
bool ExitButton = 0;

Adafruit_BMP085 bmp;
float LocalTemp = 0;
float voltage = 0;


void setup() {
  Wire.begin();
  Serial.begin(115200);
  pixels.begin();


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
}

void loop() {
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();

  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Pres: ");
  lcd.print(pressure);
  lcd.print(" Pa");

  delay(100);
  UpdateButtons();
  ReadBatVolt();
}

void UpdateButtons() {
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
  delay(250);
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
}

void ReadBatVolt() {
  int adcValue = analogRead(A0);            // read the analog input value
  float voltage = adcValue / 1023.0 * 1.0;  // translate the 10-bit value to a voltage value between 0 and 1
  lcd.setCursor(0, 2);

  lcd.print(String(adcValue) +" " + String(voltage) + " V");
}