#include <Arduino.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <time.h>
#include <ezButton.h>
#include <OneWire.h>
#include <DallasTemperature.h>


TFT_eSPI tft = TFT_eSPI();

//Viikude definitsioonid
#define CS_PIN  21
#define TIRQ_PIN  2
#define LCDBL_PIN 32
#define TDS_PIN 36
#define PH_PIN 34
#define TEMP_PIN 14
#define WTRLVL_PIN 13
#define RLY10_PIN 05
#define RLY9_PIN 16                                         
#define RLY8_PIN 17
#define RLY7_PIN 22
#define RLY6_PIN 33
#define RLY5_PIN 25
#define RLY4_PIN 26
#define RLY3_PIN 12
//#define RLY2_PIN 12
//#define RLY1_PIN 13

// globaalsed muutujad arvutusteks
#define VREF 3.3 // osund pinge
#define SCOUNT 30 // mõõtmiste arv
#define PUMPTIME 5000U // pumba tööaeg
#define PUMPINTERVAL 30000U // pumba puhke aeg

XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);
OneWire tempSensor(TEMP_PIN); // temperatuuri anduri defineerimine
DallasTemperature sensors(&tempSensor); // temperatuuri anduri edastamine teegile

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define FONT_SIZE 1
#define FSB24 &FreeSansBold24pt7b

// pH+ nupu tsooni suurus
#define PHPBUTTON_X 170
#define PHPBUTTON_Y 260
#define PHPBUTTON_W 60
#define PHPBUTTON_H 60

// pH- nupu tsooni suurus
#define PHMBUTTON_X 0
#define PHMBUTTON_Y 260
#define PHMBUTTON_W 60
#define PHMBUTTON_H 60

// EC- nupu tsooni suurus
#define ECMBUTTON_X 250
#define ECMBUTTON_Y 260
#define ECMBUTTON_W 60
#define ECMBUTTON_H 60

// EC+ nupu tsooni suurus
#define ECPBUTTON_X 420
#define ECPBUTTON_Y 260
#define ECPBUTTON_W 60
#define ECPBUTTON_H 60

// LED nupu tsooni suurus
#define LEDBUTTON_X 370
#define LEDBUTTON_Y 145
#define LEDBUTTON_W 95
#define LEDBUTTON_H 95

// Veetaseme hoiatuse tsooni suurus
#define WTRBUTTON_X 265
#define WTRBUTTON_Y 145
#define WTRBUTTON_W 95
#define WTRBUTTON_H 95

const int ledOnHour = 7;   // Kasvulampide sisse lülitamisaeg 24HR
const int ledOnMinute = 59; // Kasvulampide sisse lülitamisaeg Minute
const int ledOffHour = 23;   // Kasvulampide välja lülitamisaeg 24HR
const int ledOffMinute = 59; // Kasvulampide välja lülitamisaeg Minute

// NTP serveri ja wifi andmed
const char* ssid     = "Karjatse_terass_Wi-Fi";
const char* password = "";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;

// ec väärtuse arvutamiseks muutujad
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

// andurite muutujad
float averageVoltage = 0;
float ecValue = 0;

int ecRead;

ezButton mySwitch(WTRLVL_PIN); // Veetaseme anduri sisend

// puutetundliku ekraani kordinaadid: (x, y) ja surve (z)
int x, y, z;
float pH = 7;
float EC = 1;

// Süsteemi olekud
bool ledState = true;
bool wPumpState = true;
bool ppPumpstate = false;
bool pmPumpstate = false;
bool ntPumpstate = false;

// pH sensori muutujad
float calibration_value = 20.24 + 0.9;
int phval = 0;
int phBuffer = 0;
unsigned long int avgval; 
int phBufferArr[SCOUNT], phBufferTempArr[SCOUNT], temp; 
float phValue = 7;
 

// Funktsioonide prototüübid
void drawPhpButton(void);
void drawPhmButton(void);
void drawEcpButton(void);
void drawEcmButton(void);
void drawLedButton(void);
void drawWtrButton(void);
void printTouchToSerial(int touchX, int touchY, int touchZ);
void printPh(float savedPh);
void printEc(float savedEc);
int getMedianNum(int bArray[], int iFilterLen);
void printSensorEc(float readEc);
void printSensorpH(float readpH);
void printSensorTemp(float readTemp);
void printLocalTime(void);


void setup() 
{
  sensors.begin();
  ts.begin();
  ts.setRotation(1);

  // viikude režiim
  pinMode(LCDBL_PIN, OUTPUT); // ekraani taustavalgus

  pinMode(TDS_PIN, INPUT); // EC anduri sisend
  pinMode(TEMP_PIN, INPUT); // Temperatuuri anduri sisend
  pinMode(PH_PIN, INPUT); // pH anduri sisend
  //pinMode(WTRLVL_PIN, INPUT);
  pinMode(RLY10_PIN, OUTPUT); // LED lambi relee
  pinMode(RLY9_PIN, OUTPUT); // LED lambi relee
  pinMode(RLY8_PIN, OUTPUT); // LED lambi relee
  pinMode(RLY7_PIN, OUTPUT); // LED lambi relee
  pinMode(RLY6_PIN, OUTPUT); // Toitainete pumba relee
  pinMode(RLY5_PIN, OUTPUT); // pH+ pumba relee
  pinMode(RLY4_PIN, OUTPUT); // pH- pumba relee
  pinMode(RLY3_PIN, OUTPUT); // Veepumba relee
  //pinMode(RLY2_PIN, OUTPUT); // Reserv
  //pinMode(RLY1_PIN, OUTPUT); // Reserv

  Serial.begin(115200); // Serial Monitoris side kuvamiseks
  mySwitch.setDebounceTime(50);

  // Ekraani käivitamine ja Käivitussõnum
  tft.init();
  digitalWrite(32, HIGH);
  tft.setRotation(3);
  
  // Ühendamine WiFi-ga
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // WiFi-st lahti ühendamine
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  //tervitus sõnum
  tft.fillRectVGradient(0,0,480,320,TFT_BLACK,TFT_DARKGREEN);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setFreeFont(FSB24);
  tft.drawString("RIPPAED", 240, 160);
  delay(4000);

  // Avakuva elementide joonistamine
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  drawPhpButton();
  drawPhmButton();
  drawEcpButton();
  drawEcmButton();
  drawLedButton();
  tft.setTextColor(TFT_WHITE);
  printPh(pH);
  printEc(EC);
  printLocalTime();

  //lampide sisselülitamine
  digitalWrite(RLY7_PIN, HIGH);
  digitalWrite(RLY8_PIN, HIGH);
  digitalWrite(RLY9_PIN, HIGH);
  digitalWrite(RLY10_PIN, HIGH);
  Serial.println("LAMPS ON");

  // pumba sisselülitamine
  digitalWrite(WTRLVL_PIN, HIGH);
  Serial.println("WATER PUMP ON");
}

void loop() 
{ 
  static unsigned long elapsedTime = millis();
  if(millis()-elapsedTime > 10000U)
  {
    elapsedTime = millis();
    printLocalTime();
  }
  
  tft.drawLine(0, 260, 480, 260, TFT_WHITE);
  /***********************************************************************************************************
   ***************************************** Kasutaja tegevus ************************************************
   ***********************************************************************************************************/
  if (ts.tirqTouched() && ts.touched())
  {
    TS_Point p = ts.getPoint();
    // Puutetundliku ekraani kalibreerimine.
    x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    z = p.z;
    printTouchToSerial(x, y, z);

    if ((x > PHPBUTTON_X) && (x < (PHPBUTTON_X + PHPBUTTON_W))) 
    {
        if ((y > (PHPBUTTON_Y)) && (y <= (PHPBUTTON_Y + PHPBUTTON_H))) 
        {
          Serial.println("pH+ button pressed");
          pH = pH + 0.1;
          printPh(pH);
        }
    }
    if ((x > PHMBUTTON_X) && (x < (PHMBUTTON_X + PHMBUTTON_W))) 
    {
        if ((y > (PHMBUTTON_Y)) && (y <= (PHMBUTTON_Y + PHMBUTTON_H))) 
        {
          Serial.println("pH- button pressed");
          pH = pH - 0.1;
          printPh(pH);
        }
    }
    if ((x > ECPBUTTON_X) && (x < (ECPBUTTON_X + ECPBUTTON_W))) 
    {
        if ((y > (ECPBUTTON_Y)) && (y <= (ECPBUTTON_Y + ECPBUTTON_H))) 
        {
          Serial.println("EC+ button pressed");
          EC = EC + 0.1;
          printEc(EC);
        }
    }
    if ((x > ECMBUTTON_X) && (x < (ECMBUTTON_X + ECMBUTTON_W))) 
    {
        if ((y > (ECMBUTTON_Y)) && (y <= (ECMBUTTON_Y + ECMBUTTON_H))) 
        {
          Serial.println("EC- button pressed");
          EC = EC - 0.1;
          printEc(EC);
        }
    }
    if ((x > LEDBUTTON_X) && (x < (LEDBUTTON_X + LEDBUTTON_W))) 
    {
        if ((y > (LEDBUTTON_Y)) && (y <= (LEDBUTTON_Y + LEDBUTTON_H))) 
        {
          
          if (ledState == true)
          {
            Serial.println("LED OFF");
            digitalWrite(RLY7_PIN, LOW);
            digitalWrite(RLY8_PIN, LOW); 
            digitalWrite(RLY9_PIN, LOW); 
            digitalWrite(RLY10_PIN, LOW);
            ledState = false;
          }
          else
          {
            Serial.println("LED ON");
            digitalWrite(RLY7_PIN, HIGH);
            digitalWrite(RLY8_PIN, HIGH);
            digitalWrite(RLY9_PIN, HIGH);
            digitalWrite(RLY10_PIN, HIGH);
            ledState = true;
          }
          
          
        }
    }
    delay(150);
  }

  /***********************************************************************************************************
   ***************************************** Temp sensor *****************************************************
   ***********************************************************************************************************/
  static unsigned long tempSampleTimepoint = millis();
  if (millis()-tempSampleTimepoint > 10500U)
  {
    sensors.requestTemperatures();
    tempSampleTimepoint = millis();
    printSensorTemp(sensors.getTempCByIndex(0));
  }
  /***********************************************************************************************************
   ***************************************** TDS sensor ******************************************************
   ***********************************************************************************************************/

  static unsigned long ecSampleTimepoint = millis();
  if (millis()-ecSampleTimepoint > 40U)
  {     //every 40 milliseconds,read the analog value from the ADC
    ecSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
    { 
      analogBufferIndex = 0;
    }
  }

  static unsigned long ecPrintTimepoint = millis();
  if (millis()-ecPrintTimepoint > 10000U)
  {
    ecPrintTimepoint = millis();
    for (copyIndex=0; copyIndex<SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(20^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(sensors.getTempCByIndex(0)-25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;
      
      //convert voltage value to EC value
      ecValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.002; 
    }
      printSensorEc(ecValue);
      Serial.print("EC voltage:");
      Serial.print(averageVoltage,2);
      Serial.println("V   ");     
  }

  /***********************************************************************************************************
   ***************************************** pH sensor *******************************************************
   ***********************************************************************************************************/
  // Loetakse 30 väärtust 40ms tagant
  static unsigned long pHSampleTimepoint = millis();
  if (millis()-pHSampleTimepoint > 40U)
  {
    pHSampleTimepoint = millis();
    phBufferArr[phBuffer] = analogRead(PH_PIN);
    phBuffer++;
    if (phBuffer == SCOUNT)
    { 
      phBuffer = 0;
    }
  }

  // mõõdetud väärtused järjestatakse suuruse järjekorras
  static unsigned long phPrintTimepoint = millis();
  if (millis()-phPrintTimepoint > 10250U)
  {
    phPrintTimepoint = millis();
    for (copyIndex=0; copyIndex<SCOUNT; copyIndex++)
    {
      phBufferTempArr[copyIndex] = phBufferArr[copyIndex];

      averageVoltage = getMedianNum(phBufferTempArr,SCOUNT) * (float)VREF / 4096.0;

      phValue = -5.70 * averageVoltage + calibration_value;  //  
    }
    printSensorpH(phValue);
    Serial.print("pH voltage: ");
    Serial.print(averageVoltage, 2);
    Serial.println("V   ");
  }

  /***********************************************************************************************************
   ***************************************** Peristaltilised Pumbad ******************************************
   ***********************************************************************************************************/

  static unsigned long elapsedTime2 = millis();
  if (millis()-elapsedTime2 > PUMPINTERVAL)
  {
    if (ecValue < EC)
    {
      if (ntPumpstate == false)
      {
        digitalWrite(RLY6_PIN, HIGH);
        Serial.println("NUTRIENT PUMP ON");
        ntPumpstate = true;
      }
      if (millis()-elapsedTime2 > PUMPINTERVAL + PUMPTIME)
      {
        digitalWrite(RLY6_PIN, LOW);
        Serial.println("NUTRIENT PUMP OFF");
        elapsedTime2 = millis();
        ntPumpstate = false;
      }
    }
  }
  
  static unsigned long elapsedTime3 = millis();
  if (millis()-elapsedTime3 > PUMPINTERVAL)
  {
    if (phValue < pH)
    {
      if (ppPumpstate == false)
      {
        digitalWrite(RLY5_PIN, HIGH);
        Serial.println("pH+ PUMP ON");
        ppPumpstate = true;
      }
      if (millis()-elapsedTime3 > PUMPINTERVAL + PUMPTIME)
      {
        digitalWrite(RLY5_PIN, LOW);
        Serial.println("pH+ PUMP OFF");
        elapsedTime3 = millis();
        ppPumpstate = false;
      }
    }
  }

  static unsigned long elapsedTime4 = millis();
  if (millis()-elapsedTime4 > PUMPINTERVAL)
  {
    if (phValue > pH)
    {
      if (pmPumpstate == false)
      {
        digitalWrite(RLY4_PIN, HIGH);
        Serial.println("pH- PUMP ON");
        pmPumpstate = true;
      }
      if (millis()-elapsedTime4 > PUMPINTERVAL + PUMPTIME)
      {
        digitalWrite(RLY4_PIN, LOW);
        Serial.println("pH- PUMP OFF");
        elapsedTime4 = millis();
        pmPumpstate = false;
      }
    }
  }

  /***********************************************************************************************************
   ***************************************** Veepump *********************************************************
   ***********************************************************************************************************/
  
  mySwitch.loop();

  int state = mySwitch.getState();
  if (state == LOW && wPumpState == false)
  {
    digitalWrite(14, HIGH);
    wPumpState = true;
    Serial.println("WATER PUMP ON");
    tft.fillRect(265, 145, 95, 95, TFT_BLACK);
  }
  else if (state == HIGH && wPumpState == true)
  {
    delay(5000);
    digitalWrite(14, LOW);
    wPumpState = false;
    Serial.println("WATER PUMP OFF");
    drawWtrButton();
  }
  
  /***********************************************************************************************************
   ***************************************** Lambid **********************************************************
   ***********************************************************************************************************/

  // Leiab hetke aja
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime(&rawtime);
  
    // Kontrollib, kas aeg on relee sisse lülitada
  if (timeinfo->tm_hour == ledOnHour && timeinfo->tm_min == ledOnMinute && ledState == false) 
  {
      // Relee lülitatakse sisse
    digitalWrite(RLY7_PIN, HIGH);
    digitalWrite(RLY8_PIN, HIGH);
    digitalWrite(RLY9_PIN, HIGH);
    digitalWrite(RLY10_PIN, HIGH);
    Serial.println("LAMPS ON");
    ledState = true;
    
  } 
  // Kontrollib, kas aeg on relee välja lülitada
  else if (timeinfo->tm_hour == ledOffHour && timeinfo->tm_min == ledOffMinute && ledState == true)
  {
      // Relee lülitatakse sisse
    digitalWrite(RLY7_PIN, LOW);
    digitalWrite(RLY8_PIN, LOW); 
    digitalWrite(RLY9_PIN, LOW); 
    digitalWrite(RLY10_PIN, LOW); 
    Serial.println("LAMPS OFF");
    ledState = false; 
  }

  //**************************************Kellaaja kalibreerimine***************************************
  static unsigned long timeCalPoint = millis();
  if(millis()-timeCalPoint > 86400000U)
  {
    timeCalPoint = millis();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    
    // Serverist küsitakse aeg
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    // WiFi-st lahti ühendamine
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } 
}

// ---Funktsioonid---

// joonistab pH+ nupu
void drawPhpButton() 
{
  tft.drawRoundRect(PHPBUTTON_X, PHPBUTTON_Y, PHPBUTTON_W, PHPBUTTON_H,5, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(4);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("+", (PHPBUTTON_X + PHPBUTTON_W/2), (PHPBUTTON_Y + PHPBUTTON_H/2 + 5));
}

// joonistab pH- nupu
void drawPhmButton() 
{
  tft.drawRoundRect(PHMBUTTON_X, PHMBUTTON_Y, PHMBUTTON_W, PHMBUTTON_H,5, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(4);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("-", (PHMBUTTON_X + PHMBUTTON_W/2), (PHMBUTTON_Y + PHMBUTTON_H/2 + 5));
}

// joonistab EC+ nupu
void drawEcpButton() 
{
  tft.drawRoundRect(ECPBUTTON_X, ECPBUTTON_Y, ECPBUTTON_W, ECPBUTTON_H,5, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(4);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("+", (ECPBUTTON_X + ECPBUTTON_W/2), (ECPBUTTON_Y + ECPBUTTON_H/2 + 5));
}

// joonistab EC- nupu
void drawEcmButton() 
{
  tft.drawRoundRect(ECMBUTTON_X, ECMBUTTON_Y, ECMBUTTON_W, ECMBUTTON_H,5, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(4);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("-", (ECMBUTTON_X + ECMBUTTON_W/2), (ECMBUTTON_Y + ECMBUTTON_H/2 + 5));
}

// joonistab LED nupu
void drawLedButton() 
{
  tft.drawRoundRect(LEDBUTTON_X, LEDBUTTON_Y, LEDBUTTON_W, LEDBUTTON_H,5, TFT_WHITE);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextFont(4);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("LED", (LEDBUTTON_X + LEDBUTTON_W/2), (LEDBUTTON_Y + LEDBUTTON_H/2 + 5));
}

// joonistab Veetaseme hoiatuse
void drawWtrButton() 
{
  tft.drawRoundRect(WTRBUTTON_X, WTRBUTTON_Y, WTRBUTTON_W, WTRBUTTON_H,5, TFT_WHITE);
  tft.setTextColor(TFT_RED);
  tft.setTextFont(4);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Veetase", (WTRBUTTON_X + WTRBUTTON_W/2), (WTRBUTTON_Y + WTRBUTTON_H/2 - 15));
  tft.drawString("madal", (WTRBUTTON_X + WTRBUTTON_W/2), (WTRBUTTON_Y + WTRBUTTON_H/2 + 15));
}

// Prindi puute kordinaadid ja tugevus Serial Monitori
void printTouchToSerial(int touchX, int touchY, int touchZ) 
{
  Serial.print("X = ");
  Serial.print(touchX);
  Serial.print(" | Y = ");
  Serial.print(touchY);
  Serial.print(" | Pressure = ");
  Serial.print(touchZ);
  Serial.println();
}

void printPh(float savedPh)
{
  tft.fillRect(115, 275, 50, 45, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("pH ", 95, 310);
  tft.drawFloat(savedPh, 1, 135, 310);
}

void printEc(float savedEc)
{
  tft.fillRect(360, 275, 50, 45, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("EC ", 345, 310);
  tft.drawFloat(savedEc, 1, 385, 310);
}

// mediaan väärtuse filtreerimis algorütm
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) 
  {
    for (i = 0; i < iFilterLen - j - 1; i++) 
    {
      if (bTab[i] > bTab[i + 1]) 
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else 
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void printSensorEc(float readEc)
{
  Serial.print("EC Value:");
  Serial.print(readEc,1);
  Serial.println("mS/cm");
  tft.fillRect(90, 0, 75, 86, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("EC: ", 0, 0);
  tft.drawFloat(readEc, 1, 90, 0);
  tft.setTextSize(1);
  tft.setTextDatum(BL_DATUM);
  tft.drawString("mS/cm", 165, 44);
}

void printSensorpH(float readpH)
{
  Serial.print("pH Value:");
  Serial.println(readpH,1);
  tft.fillRect(90, 86, 105, 86, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("pH: ", 0, 86);
  tft.drawFloat(readpH, 1, 90, 86);
}

void printSensorTemp(float readTemp)
{
  Serial.print("Water Temperature:");
  Serial.print(readTemp,1);
  Serial.println("C");
  tft.fillRect(90, 172, 160, 86, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("T: ", 0, 172);
  tft.drawFloat(readTemp, 1, 90, 172);
  tft.setTextSize(1);
  tft.setTextDatum(BL_DATUM);
  tft.drawCircle(197, 190, 3, TFT_WHITE);
  tft.drawString("C", 200, 216);
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  tft.fillRect(400, 0, 80, 40, TFT_BLACK);
  tft.setCursor(400, 0, 4);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.println(&timeinfo, "%H:%M");
}