#define REV 20200818

#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
LiquidCrystal lcd(12, 13, 4, 5, 6, 7);
byte lcdNumCols = 16; // -- number of columns in the LCD
LcdBarGraph lbg(&lcd, lcdNumCols);  // -- creating bargraph instance, format is (&lcd, lcdNumCols, start X, start Y). So (&lcd, 16, 0, 1) would set the bargraph length to 16 columns and start the bargraph at column 0 on row 1.
bool LcdNeedRefresh = false;
long LcdRefreshTimer[2]={0,100};

int pulseCounter = 0;
int sensorValue;
int sensorValuePrev;
float voltage;
int encoderValue=0;
long encoderDebouncingTimer[2]={0,20};

// // inputs
const int EncA = A6; // input only
const int EncB = A2;
const int AZ = A7;   // input only
const int CCWsw = A4;
const int CWsw = A5;
const int STARTsw = A3;
const int CCWpulse = 2;
const int CWpulse = 3;
// outputs
const int CWrel = 8;
const int CCWrel = 9;
const int BRAKErel = 10;
const int PWM = 11;

//----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.setCursor(1, 0);
  lcd.print("Rotator 4 TEST");
  lcd.setCursor(2, 1);
  lcd.print("rev ");
  lcd.print(REV);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("pwm ");
  lcd.setCursor(15, 1);
  lcd.print("V");
  analogReference(EXTERNAL);

  pinMode(EncA, INPUT);
  pinMode(EncB, INPUT);
  pinMode(AZ, INPUT);
  pinMode(CCWsw, INPUT);
  pinMode(CWsw, INPUT);
  pinMode(STARTsw, INPUT);
  pinMode(CCWpulse, INPUT);
  pinMode(CWpulse, INPUT);

  pinMode(CWrel, OUTPUT);
    digitalWrite(CWrel,0);
  pinMode(CCWrel, OUTPUT);
    digitalWrite(CCWrel,0);
  pinMode(BRAKErel, OUTPUT);
    digitalWrite(BRAKErel,0);
  pinMode(PWM, OUTPUT);
    analogWrite(PWM,0);

  attachInterrupt(digitalPinToInterrupt(CWpulse), CWpulseCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(CCWpulse), CCWpulseCounter, FALLING);
}

//----------------------------------------------------------------------------
void loop() {
  sensorValue = analogRead(AZ);
  if(sensorValue != sensorValuePrev && millis()-LcdRefreshTimer[0]>LcdRefreshTimer[1]){
    sensorValuePrev = sensorValue;
    voltage = sensorValue * (5.0 / 1023.0);
    LcdNeedRefresh = true;
    LcdRefreshTimer[0]=millis();
  }

int EncBuf = analogRead(EncB);
  if(EncBuf<150 && millis()-encoderDebouncingTimer[0]>encoderDebouncingTimer[1]){
    encoderDebouncingTimer[0]=millis();
    if(analogRead(EncA)<150){
      encoderValue++;
    }else{
      encoderValue--;
    }
    if(encoderValue>254){
      encoderValue=254;
    }
    if(encoderValue<0){
      encoderValue=0;
    }
    LcdNeedRefresh=true;
  }

// WRITE pouze pri zmene
  if(analogRead(CCWsw)<150){
    digitalWrite(CCWrel,1);
  }else{
    digitalWrite(CCWrel,0);
  }
  if(analogRead(CWsw)<150){
    digitalWrite(CWrel,1);
  }else{
    digitalWrite(CWrel,0);
  }
  if(analogRead(STARTsw)<150){
    digitalWrite(BRAKErel,1);
  }else{
    digitalWrite(BRAKErel,0);
  }

  LCD();
}

void LCD(){
  if(LcdNeedRefresh==true){
    lbg.drawValue( sensorValue, 1024);
    // lbg.drawValue( analogRead(sensorPin), 1024);
    Serial.println(voltage);

    lcd.setCursor(3, 1);
    lcd.print(encoderValue);
    lcd.print("|");
    lcd.print(pulseCounter);
    lcd.print(" ");
    lcd.setCursor(10, 1);
    lcd.print("|");
    lcd.print(voltage);

    LcdNeedRefresh=false;
  }
}

void CWpulseCounter(){
  pulseCounter++;
  analogWrite(PWM,pulseCounter);
  LcdNeedRefresh = true;
}
void CCWpulseCounter(){
  pulseCounter--;
  analogWrite(PWM,pulseCounter);
  LcdNeedRefresh = true;
}
