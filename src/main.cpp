#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <INA226.h> //https://github.com/jarzebski/Arduino-INA226

#include <Adafruit_NeoPixel.h>
#include <U8g2lib.h>//fonts: https://github.com/olikraus/u8g2/wiki/fntlistall#18-pixel-height
// #include <U8x8lib.h>//https://github.com/olikraus/u8g2/wiki/u8x8setupc

// #include <MsTimer2.h>//https://github.com/PaulStoffregen/MsTimer2?utm_source=platformio&utm_medium=piohome

// #include <TimerOne.h>//https://www.pjrc.com/teensy/td_libs_TimerOne.html

#include "Ticker.h"

//D8 =>pin 11 - buzzer
// led  => 13
#define PIN_BUZZER 11//15

//maximum tempereture then beep
#define MAX_TEMP 80

// Data wire is plugged into port 2 on the Arduino
#define TEMP_RESOLUTION 9//temp resolution
#define ONE_WIRE_BUS 2//pin
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

INA226 ina;

//neopixels
#define PIN 3
#define NUMPIXELS 8
#define DELAYVAL 100
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//display
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0);

//align test left/center/right on display
#define LCDWidth                        u8g2.getDisplayWidth()
#define ALIGN_CENTER(t)                 ((LCDWidth - (u8g2.getUTF8Width(t))) / 2)
#define ALIGN_RIGHT(t)                  (LCDWidth -  u8g2.getUTF8Width(t))
#define ALIGN_LEFT                      0


int i=0;
void drawDisplay2();
void getTemp();
void setTemp();
void displayLed();
void beep_stop();

//https://github.com/sstaub/Ticker?utm_source=platformio&utm_medium=piohome
Ticker timerDisplay(drawDisplay2, 300); // draw data to display
Ticker timerRequestTempereture(getTemp, 200,1); // req temp from sensors
Ticker timerSetTemp(setTemp,750/ (1 << (12-TEMP_RESOLUTION)),1,MILLIS);//set temp
Ticker timerLed(displayLed, 50); // led neopixel
Ticker timerBepp(beep_stop,1,1); // beep stop

void beep_stop(){
  digitalWrite(PIN_BUZZER,LOW);
}//beep_stop

void beep(int ms=1000){
    digitalWrite(PIN_BUZZER,HIGH);
    timerBepp.interval((uint32_t)ms);
    timerBepp.start();
}//beep


// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
   Serial.begin(115200);

// Default INA226 address is 0x40
  ina.begin();
  ina.configure(INA226_AVERAGES_256, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.022/2, 10);  // Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A

   //tempereture sensors
  sensors.setResolution(TEMP_RESOLUTION);
  sensors.begin();
  sensors.setResolution(TEMP_RESOLUTION);

  //neopixels
    pinMode(LED_BUILTIN, OUTPUT);
    strip.begin();
    strip.setBrightness(100);

//display
    u8g2.setBusClock(400000);//more fps https://www.seeedstudio.com/blog/2019/07/05/u8g2-for-seeeduino-boards/
    u8g2.begin();

    timerDisplay.start();
    timerRequestTempereture.resume();
    timerLed.start();

    beep(5000);
}//setup

//neopixels Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}//colorWipe

int counter=0;

//temperatures
float temp1=0;
float temp2=0;
float temp3=0;

void setTemp(){
  temp1=sensors.getTempCByIndex(0);
  temp2=sensors.getTempCByIndex(1);
  temp3=sensors.getTempCByIndex(2);

  if(temp1>MAX_TEMP||temp2>MAX_TEMP||temp3>MAX_TEMP){
    digitalWrite(PIN_BUZZER,HIGH);
  }else{
    digitalWrite(PIN_BUZZER,LOW);
  }
  timerRequestTempereture.start();
}//setTemp

void getTemp(){
  sensors.setWaitForConversion(false);// makes it async
  sensors.requestTemperatures();
  sensors.setWaitForConversion(true);

  timerSetTemp.start();
}//getTempm

void drawDisplay2(){
    String str_voltage=String(String(ina.readBusVoltage(),3)+"V");
    String str_ampere=String(String(ina.readShuntCurrent(),3)+"A");
    String str_watts=String(String(ina.readBusPower(),3)+"W");
    String str_temp=String(String(temp1,0)+"C "+String(temp2,0)+"C "+String(temp3,0)+"C");

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_fur20_tf);//18px  u8g2_font_ncenR18_tf
    u8g2.drawStr(ALIGN_RIGHT(str_voltage.c_str()),20,str_voltage.c_str());
    u8g2.drawStr(ALIGN_RIGHT(str_ampere.c_str()),43,str_ampere.c_str());
    u8g2.setFont(u8g2_font_9x18B_tf);//10px  u8g2_font_7x14_tf
    u8g2.drawStr(ALIGN_RIGHT(str_watts.c_str()),55,str_watts.c_str());
    
    u8g2.setFont(u8g2_font_5x7_tf);//6px font  u8g2_font_5x7_tf
    u8g2.drawStr(ALIGN_RIGHT(str_temp.c_str()),64,str_temp.c_str());
   } while ( u8g2.nextPage() );
}//drawDisplay

uint16_t led=0;
int color_r=255;
int color_g=0;
int color_b=0;

void displayLed(){
  strip.setPixelColor(led,color_r,color_g,color_b);
  if(led<strip.numPixels()-1){led++;}else{
    led=0;
    if(color_r==255&&color_g==255&&color_b==255){color_r=255;color_g=0;color_b=0;}
    else{
      if(color_r==255){color_r=0;color_g=255;}
      else if(color_g==255){color_g=0;color_b=255;}
      else if(color_b==255){color_r=255;color_g=255;color_b=255;}
      }//else led
    }
// Serial.println(">>LED, pixels:"+String(strip.numPixels())+"led:"+String(led)+", R: "+String(color_r)+", G: "+String(color_g)+", B:"+String(color_b));
  strip.show();
}//displayled

// the loop function runs over and over again forever
void loop() {
  // delay(1000);// wait

//set timers
  timerDisplay.update();
  timerRequestTempereture.update();
  timerSetTemp.update();
  timerLed.update();
  timerBepp.update();
}//loop

