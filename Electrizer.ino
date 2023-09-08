#include <AntaresESP32HTTP.h>
#include "time.h"
#include <Wire.h>
#include "ThingSpeak.h"
#include "DFRobot_INA219.h"
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4
#define ACCESSKEY "53fb37b2ec3b771c:3adf9fbc4957d021"
#define WIFISSID "Android"
#define PASSWORD "lalalalala"
#define projectName "IonizerDevice0"
#define deviceName "Sensor"
#define deviceNames "Relay"
#define relayPin 5
#define relayPin0 14
#define ledPin 17
#define TdsSensorPin 35
#define VREF 3       
#define SCOUNT  32   
#define PH_OFFSET -1.00 
#define SensorPin 33        
unsigned long myChannelNumber = 2193524;
const char * myWriteAPIKey = "TIUG2O65914ZP4WZ";
const int pwmPin = 18;  
const int pwmGnd = 19;
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 0;
unsigned long int avgValue;  
float b;
float phValue;
float vPower;
int Power;
float vDev;
float cDev;
float pDev;
int wDev;
int buf[10],temp;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600*6;
const int   daylightOffset_sec = 3600;
String ayeuna = "";
AntaresESP32HTTP antares(ACCESSKEY);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
LiquidCrystal_I2C lcd(0x27, 20, 4);
DFRobot_INA219_IIC     ina219(&Wire,INA219_I2C_ADDRESS1);
WiFiClient  client;
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;
float v_temp;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
int tdsValue = 0;
int ADCpH;
int ADCtds;
float temperatu = 25;
int curr_Value = 0;
bool stateDev = true;

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  char timeStringBuff[50]; 
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S %d-%b-%Y", &timeinfo);
  String asString(timeStringBuff);
  ayeuna=asString;
  Serial.print("Ayeuna ");
  Serial.println(ayeuna);
}

void setup() {
  Serial.begin(115200);     
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(pwmPin, pwmChannel);
  pinMode(pwmGnd,OUTPUT);
  digitalWrite(pwmGnd,LOW);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.print("Starting......");
      while(!Serial);
    
    Serial.println();
    //Initialize the sensor
    while(ina219.begin() != true) {
        Serial.println("INA219 begin faild");
        delay(2000);
    }
    //Linear calibration
    ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
    Serial.println();
  pinMode(relayPin,OUTPUT);
  digitalWrite(relayPin,HIGH);
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);
  pinMode(relayPin0,OUTPUT);
  digitalWrite(relayPin0,HIGH);
  pinMode(TdsSensorPin,INPUT);
  pinMode(SensorPin,INPUT);
  antares.setDebug(true);
  antares.wifiConnection(WIFISSID,PASSWORD);  
  ThingSpeak.begin(client);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  if (sensors.isParasitePowerMode()) Serial.println("Parasite power is: ON");
  else Serial.println("Parasite power is: OFF");
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();
  sensors.setResolution(insideThermometer, 9);
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

}

void loop() {
  read_ds();
  if(stateDev==true){
  read_tds();
  read_pH();
  }
  read_current();
  view_lcd();
  sendAntares();
  sendthinkspeak();

}

void sendthinkspeak(){
 
  ThingSpeak.setField(1, v_temp);
  ThingSpeak.setField(2, phValue);
  ThingSpeak.setField(3, ADCtds);
  ThingSpeak.setField(4, vDev);
  ThingSpeak.setField(5, cDev);
  ThingSpeak.setField(6, wDev);
  

  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}


void sendAntares(){
  printLocalTime();
  int temp = v_temp*100;
  int ph = phValue*100;
  int tds = tdsValue*100;
  int st=1;
  antares.add("temp", temp);
  antares.add("pH", ph);
  antares.add("tds", tds);
  antares.add("volt", vDev);
  antares.add("curr", cDev);
  antares.add("waktu", ayeuna);
  antares.add("pwm", wDev);
  antares.send(projectName, deviceName);
  antares.get(projectName, deviceNames);
  if(antares.getSuccess()) {
    int relay = antares.getInt("relay");
    Serial.println("Relay   : " + String(relay));
    if(relay==1){
      int pwmDev = antares.getInt("pwm");
      wDev = pwmDev;
      stateDev=false;
      digitalWrite(relayPin,LOW);
      digitalWrite(relayPin0,LOW);
      ledcWrite(pwmChannel, pwmDev);
    }
    if(relay==0){
      if(stateDev==false){
    digitalWrite(relayPin,HIGH);
    digitalWrite(relayPin0,HIGH);
    delay(180000);    
      }
      stateDev=true;
    digitalWrite(relayPin,HIGH);
    digitalWrite(relayPin0,HIGH);
    }
  }
  //delay(20000);
}


void view_lcd(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp. : ");
  lcd.print(v_temp);
  lcd.print(" *C");
  lcd.setCursor(0,1);
  lcd.print("TDS   : ");
  lcd.print(tdsValue);
  lcd.print(" ppm");
  lcd.setCursor(0,2);
  lcd.print("pH    : ");
  lcd.print(phValue);
  lcd.setCursor(0,3);
  lcd.print("Curr. : ");
  float curr=curr_Value;
  lcd.print(vPower);
  lcd.print(" mA");
  
}
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  v_temp = tempC;
}

void read_ds(void)
{ 
  sensors.requestTemperatures(); 
  printTemperature(insideThermometer); 
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void read_tds(){
int b=0;
for(int c=0; c<30;c++){
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    
    int a=analogRead(TdsSensorPin);
    Serial.println(a);
    b=b+a;
    analogBufferIndex++;
    delay(40);
    }  
    ADCtds=b/30;
    Serial.println("adc tds = " +String(ADCtds)) ;
   for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      float compensationCoefficient = 1.0+0.02*(temperatu-25.0);
      float compensationVoltage=averageVoltage/compensationCoefficient;
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;   
     }
     tdsValue = tdsValue*4.3;
     Serial.print("TDS Value:");
     tdsValue=tdsValue;
     Serial.print(tdsValue,0);
     Serial.println("ppm");
     analogBufferIndex = 0;
  }

  void read_pH()
{
  for(int i=0;i<10;i++)       
  { 
    buf[i]=analogRead(SensorPin);
    //Serial.println(buf[i]);
    delay(10);
  }
  for(int i=0;i<9;i++)        
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      
    avgValue+=buf[i];
  phValue=(float)avgValue*5/4095/6; 
  ADCpH=(float)avgValue/6;
  phValue=(-0.0095*ADCpH) + 20.60;                  
  Serial.print("    pH:");  
  Serial.print(phValue,2);
  Serial.println(" ");
  digitalWrite(ledPin, HIGH);       
  delay(800);
  digitalWrite(ledPin, LOW); 
}

void read_current(void)
{
    Serial.print("BusVoltage:   ");
    Serial.print(ina219.getBusVoltage_V(), 2);
    Serial.println("V");
    Serial.print("ShuntVoltage: ");
    Serial.print(ina219.getShuntVoltage_mV(), 3);
    Serial.println("mV");
    Serial.print("Current:      ");
    Serial.print(ina219.getCurrent_mA(), 1);
    Serial.println("mA");
    Serial.print("Power:        ");
    Serial.print(ina219.getPower_mW(), 1);
    Serial.println("mW");
    Serial.println("");
    delay(1000);
    vDev = ina219.getBusVoltage_V()-0.1;
    if(vDev<0){
      vDev=0;
    }
    cDev = (1.85952*ina219.getCurrent_mA())-12.97141;
    pDev = ina219.getPower_mW();
    if(cDev<0){
      cDev=0;
    }
    
    vPower=cDev;
    curr_Value = cDev;
    Serial.print("current  : ");
    Serial.println(cDev);
}
