

//Originally meant to run on the RP2040
//That board's fucked and I don't know why
//Changing to run on the Arduino Mega, will also dev to deploy on Uno

#include <Adafruit_NeoPixel.h>
//#include <Wire.h>
//#include <SoftwareSerial.h>
//#include <Adafruit_TinyUSB.h>
//#include <Adafruit_NeoPXL8.h>
#include <Adafruit_BMP085.h>
#include <math.h>
//#include <Arduino_AVRSTL.h>



#define NUM_LEDS 1            // NeoPixels PER STRAND, total number is 8X this!
#define COLOR_ORDER NEO_RGBW  // NeoPixel color format (see Adafruit_NeoPixel)

int8_t megaPin1 = 9;
int8_t megaPin2 = 8;
int8_t RP2040ScorpioPins[8] = { 16, 17, 18, 19, 20, 21, 22, 23 };  //Feather RP2040 Scorpio LED Pins

//Adafruit_NeoPXL8 leds(NUM_LEDS, RP2040ScorpioPins, COLOR_ORDER);
Adafruit_NeoPixel megaAltitude(NUM_LEDS, megaPin1, COLOR_ORDER);
Adafruit_NeoPixel megaStatus(NUM_LEDS, megaPin2, COLOR_ORDER);

Adafruit_BMP085 bmp;

float calibrateBMP(bool& cs);
void readoutPressure();
void readoutBMP();
float startingAlt{};

static uint32_t cRed = Adafruit_NeoPixel::Color(255, 0, 0, 0);
static uint32_t cOrange = Adafruit_NeoPixel::Color(255, 160, 0, 0);
static uint32_t cCyan = Adafruit_NeoPixel::Color(0, 255, 255, 0);
static uint32_t cGreen = Adafruit_NeoPixel::Color(0, 255, 0, 0);
static uint32_t cYellow = Adafruit_NeoPixel::Color(255, 255, 0, 0);
static uint32_t cBlue = Adafruit_NeoPixel::Color(0, 0, 255, 0);
static uint32_t cPurple = Adafruit_NeoPixel::Color(192, 0, 255, 0);
static uint32_t cMagenta = Adafruit_NeoPixel::Color(255, 0, 255, 0);
static uint32_t cWhite = Adafruit_NeoPixel::Color(0, 0, 0, 255);
static uint32_t cOff = Adafruit_NeoPixel::Color(0, 0, 0, 0);


static uint8_t conditionColours[8][3] = {
  255, 0, 0,    // Row 0: Red           - Waiting for launch
  255, 160, 0,  // Row 1: Orange        - Launched
  255, 255, 0,  // Row 2: Yellow        - Separation
  0, 255, 0,    // Row 3: Green         - Apogee
  0, 255, 255,  // Row 4: Cyan          - 10,000 feet hit
  0, 0, 255,    // Row 5: Blue          - Drogue Deployed
  192, 0, 255,  // Row 6: Purple        - Main deployed
  255, 0, 255   // Row 7: Magenta       - Fault State
};

void colourState(int s, Adafruit_NeoPixel pxl);

void setAltColour(float alt, float sAlt, Adafruit_NeoPixel& megaAltitude);
void setStateColour(float alt, float sAlt, Adafruit_NeoPixel& megaStatus);


void setup() {
  // put your setup code here, to run once:
  bool calStatus = false;
  Serial.begin(9600);
  megaAltitude.begin();
  megaAltitude.setBrightness(50);
  for (int i = 0; i < 10; i++) {
    megaAltitude.setPixelColor(0, cRed);
    megaAltitude.show();
    delay(100);
    megaAltitude.setPixelColor(0, cOff);
    megaAltitude.show();
    delay(100);
  }
  megaAltitude.setPixelColor(0, cRed);
  megaAltitude.show();


  megaStatus.begin();
  megaStatus.setBrightness(50);
  for (int i = 0; i < 10; i++) {
    megaStatus.setPixelColor(0, cRed);
    megaStatus.show();
    delay(100);
    megaStatus.setPixelColor(0, cOff);
    megaStatus.show();

    delay(100);
  }
  megaStatus.setPixelColor(0, cOff);
  megaStatus.show();


  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  delay(150);
  //float startingAlt = calibrateBMP(calStatus);
  startingAlt = bmp.readPressure();
  Serial.println(startingAlt);
}

void loop() {
  // put your main code here, to run repeatedly:
  //readoutBMP();
  delay(1);
  // megaAltitude.fill(cOff);
  // megaAltitude.show();
  // megaStatus.fill(cOff);
  // megaStatus.show();
  // delay(1);
  //readoutPressure();
  float readingCur = bmp.readPressure();
  setAltColour(readingCur, startingAlt, megaAltitude);
  megaAltitude.show();
  setStateColour(readingCur, startingAlt, megaStatus);
  megaStatus.show();
}

/*~~This Function may soon be depricated, avoid using~~*/
void colourState(int s, Adafruit_NeoPixel pxl) {
  switch (s) {
    case 0:
      pxl.clear();
      pxl.fill(cRed);
      pxl.show();
      return;
    case 1:
      pxl.clear();
      pxl.fill(cOrange);
      pxl.show();
      return;
    case 2:
      pxl.clear();
      pxl.fill(cYellow);
      pxl.show();
      return;
    case 3:
      pxl.clear();
      pxl.fill(cGreen);
      pxl.show();
      return;
    case 4:
      pxl.clear();
      pxl.fill(cCyan);
      pxl.show();
      return;
    case 5:
      pxl.clear();
      pxl.fill(cBlue);
      pxl.show();
      return;
    case 6:
      pxl.clear();
      pxl.fill(cPurple);
      pxl.show();
      return;
    case 7:
      pxl.clear();
      pxl.fill(cMagenta);
      pxl.show();
      return;
    case 8:
      pxl.clear();
      pxl.fill(cWhite);
      pxl.show();
      return;
    case 9:
      pxl.clear();
      pxl.fill(cOff);
      pxl.show();
      return;
  }
}

void readoutBMP() {
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(101500));
  Serial.println(" meters");

  Serial.println();
}

void readoutPressure() {
  Serial.print("Pressure: ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
}

void setStateColour(float alt, float sAlt, Adafruit_NeoPixel& mStatus) {
  int deltaD = sAlt - alt;
  Serial.print("Pressure Difference: ");
  Serial.println(deltaD);
  if (deltaD > sAlt - 69681) {
    Serial.println("10,000 feet!");
    mStatus.fill(cWhite);
    mStatus.show();
  }
  if (deltaD > sAlt - 76712) {
    Serial.println("7,500 feet!");
    mStatus.fill(cPurple);
    mStatus.show();
  }
  if (deltaD > sAlt - 84307) {
    Serial.println("5,000 feet!");
    mStatus.fill(cCyan);
    mStatus.show();
  }
  if (deltaD > sAlt - 89148) {
    Serial.println("3,500 feet!");
    mStatus.fill(cGreen);
    mStatus.show();
  }
  if (deltaD > sAlt - 94212) {
    Serial.println("2,000 feet!");
    mStatus.fill(cYellow);
    mStatus.show();
  }
  if (deltaD > sAlt - 97716) {
    Serial.println("1,000 feet!");
    mStatus.fill(cOrange);
    mStatus.show();
  }
  if (deltaD > sAlt - 99507) {
    Serial.println("500 feet!");
    mStatus.fill(cRed);
    mStatus.show();
  }
  if (deltaD < sAlt - 99507) {
    Serial.println("Waiting for major change");
    mStatus.fill(cOff);
    mStatus.show();
  }
}


void setAltColour(float alt, float sAlt, Adafruit_NeoPixel& mAlt) {
  uint8_t r = 255, g = 0, b = 0, w = 0;
  //r = constrain(r, 0, 255), g = constrain(g, 0, 255), b = constrain(b, 0, 255), w = constrain(w, 0, 255);

  int offset = (alt - sAlt) /250;
  Serial.print("Offset from Start: ");
  Serial.println(offset);
  // r = r + offset;
  // g = g - offset;
  //mAlt.clear();
  //mAlt.setPixelColor(0, Adafruit_NeoPixel::Color(0, 0, 0, 0));
  if (r + offset > 255 && g - offset < 0) {
    mAlt.setPixelColor(0, Adafruit_NeoPixel::Color(r, g, b, w));
  }
  uint8_t cR = r + offset, cG = g - offset, cB = b, cW = w;
  if (r + offset < 255 && g - offset > 0) {
    mAlt.setPixelColor(0, Adafruit_NeoPixel::Color(cR, cG, cB, cW));
  }
  //mAlt.show();
}

float calibrateBMP(bool& calStatus) {

  float calibratedPressure{}, pressureSum{};

  for (int i = 0; i < 50; i++) {
    delay(10);
    pressureSum = +bmp.readPressure();
    if (i != 49) {
      continue;
    }
    if (i == 49) {
      calibratedPressure = pressureSum / 50;
      calStatus = true;
      return calibratedPressure;
    } else {
      calStatus = false;
      continue;
    }
  }
}