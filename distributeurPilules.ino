
// original code from   https://www.instructables.com/Automatic-Pill-or-Medication-Dispenser/

/*
       Daniel Perron version 1.0   26 octobre 2025
     -  Use rtc build-in Alarm1 And Alarm2 instead of fix code for alarm
     -  Add three buttons Mode,+ and -  to change real time and alarms
     -  Use Adafruit debounce for buttons.
     -  LCD display wasn't compatible I did have to change the library
       https://github.com/fmalpartida/New-LiquidCrystal 
       abbra: https://abra-electronics.com/opto-illumination/lcds-and-displays/character/lcd-mod-13-16x2-lcd-display-with-i2c-backpack-white-on-blue.html

*/


/*
   https://github.com/NormanDunbar/ArduinoClock/blob/master/Sources/ArduinoClock/ArduinoClock.ino

   This works well good way to set the RTC via serial input Enter u to start

  This is working WELL with 28byj stepper
  https://www.makerguides.com/28byj-48-stepper-motor-arduino-tutorial/
  https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/

  button wiring
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button

*/

#include <Arduino.h>
#include <Wire.h>                   // For I2C communication
#include <LiquidCrystal_I2C.h>      // For LCD
#include <RTClib.h>                 // For RTC
#include "Adafruit_Debounce.h"


// ORIGINAL: My LCD uses address 0x3F and is 20 by 4.
//Had to change to the following for my LCD

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

RTC_DS3231 rtc;   // Create RTC for the DS3231 RTC module, address is 0x68 fixed.

// Include the AccelStepper library:
#include <AccelStepper.h>

// Arduino UNO pin usage
/*
0  Serial RX
1  Serial TX
2  OUT LED 1 green
3  OUT LED 2 red
4. IN. SWITCH mom. Heure Programme (heure,alarme 1 , alarme 2)
5  IN  SWITCH mom. Pilule Prise
6. IN. SWITCH mom. +
7. IN. SWICTH mom. -
8   Stepper Pin 1
9   Stepper Pin 1
10  Stepper Pin 1
11  Stepper Pin 1
...
18  SDA I2C
19  SCL I2C
*/



// Motor pin definitions: Flip Pin1 and Pin3 if it turns the wrong way
#define motorPin1  10      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  8     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode MotorInterfaceType 8: 4096 steps per rotation
// For full step mode change MotorInterfaceType to 4
#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

int buttonState = 0;  


// constants won't change. They're used here to set pin numbers:
const int led1 = 2;
const int led2 = 3;

// button
const int buttonPilulePin = 5;
const int buttonModePin = 4;
const int buttonPlusPin = 6;
const int buttonMoinsPin = 7;

// adafruit buttons Debounce
Adafruit_Debounce buttonPilule(buttonPilulePin, LOW);
Adafruit_Debounce buttonMode(buttonModePin, LOW);
Adafruit_Debounce buttonPlus(buttonPlusPin, LOW);
Adafruit_Debounce buttonMoins(buttonMoinsPin, LOW);

// state cycle   normal  or edit one of the clock of the rtc
enum CycleEnum {
   Normal,
   TimeHour,
   TimeMin,
   Alarm1Hour,
   Alarm1Min,
   Alarm2Hour,
   Alarm2Min
};

const int cycleMax = (int) ((CycleEnum)Alarm2Min);

enum PiluleState{
   NoPilule=0,
   AM_PrendrePilule,
   AM_PilulePrise,
   PM_PrendrePilule,
   PM_PilulePrise
};

const char piluleMsg[5][17]= {
                              //0123456789012345
                               "                ",
                               "AM Prends Pilule",
                               "AM Pilule Prise ",
                               "PM Prends Pilule",
                               "PM Pilule Prise "};

const char *ChangeMin=         "Changer Minute  ";
const char *ChangeHeure=       "Changer Heure   ";

DateTime TimeModif;

CycleEnum cycle = Normal;
PiluleState pilule = NoPilule;


// delay to enable mode mode. (2 seconds)

unsigned long ModeTimer;
unsigned long RotateTimer;
bool PrendrePiluleFlag = false;

/* WIRING DIAGRAM:

    ARDUINO     RTC     LCD DISPLAY
    -------------------------------
        A4      SDA         SDA
        A5      SCL         SCL
        5V      VCC         VCC
       GND      GND         GNG


    The code below is a modified version of the code at:
    https://www.circuitbasics.com/how-to-use-a-real-time-clock-module-with-the-arduino
    and was modified by:

    Norman Dunbar [norman (at) dunbar-it (dot) co (dot) uk] to:

    Use built in RTCLib functions to
    format the date and time for display on the LCD.

    carry out validation checks on the user input and
    the DateTime created from same, before updating the RTC.


    USAGE:

    The clock runs happily without problems, as long as it has 5V down the USB!

    Open the serial monitor, or similar, and type u (and enter). The LCD will display "Edit mode"
    and you will be prompted to enter the years, months etc to set the clock.

    While entering details, if you need to abort, type -1 for the value and all changes so
    far will be aborted. The LCD will return to showing the date and time as before.

    All inputs are validated for a certain range of values. Months are 1 to 12, but what about
    leap years? Those are not validated (yet) and neither are "30 days hath September ...".

    After all inputs are done, a temporary DateTime is created with the entered values. If
    this is a valid date time - leap year ok, days in month ok etc - then the RTC will be
    set. If the DateTime is invalid, then we go around all the inputs again.

*/

// Minimum and maximum values structure for each input.
typedef struct minMax_t {
  int minimum;
  int maximum;
};


/*
   Function to validate user input.
   Returns TRUE if value in range, FALSE otherwise.
*/
bool checkInput(const int value, const minMax_t minMax) {
  if ((value >= minMax.minimum) &&
      (value <= minMax.maximum))
    return true;

  Serial.print(value);
  Serial.print(" is out of range ");
  Serial.print(minMax.minimum);
  Serial.print(" - ");
  Serial.println(minMax.maximum);
  return false;
}

/*
   Function to update RTC time using user input.
*////////////////////////////////////////////////////////////////////////////////////////////////////////

void SerialShowTime(DateTime theTime)
{
    char timeStr[32];
  sprintf(timeStr, "%02d/%02d/%04d %02d:%02d:%02d", 
              theTime.month(), theTime.day(), theTime.year(), 
              theTime.hour(), theTime.minute(), theTime.second()); 
  Serial.println(timeStr);
}

void serialUpdateRTC()
{

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Edit Mode...");

  // Prompts for user input.
  const char txt[6][15] = { "year [4-digit]", "month [1~12]", "day [1~31]",
                            "hours [0~23]", "minutes [0~59]", "seconds [0~59]"
                          };


  // Define limits for the 6 user inputs.
  const minMax_t minMax[] = {
    {2000, 9999},   // Year
    {1, 12},        // Month
    {1, 31},        // Day
    {0, 23},        // Hours
    {0, 59},        // Minutes
    {0, 59},        // Seconds
  };

  String str = "";

  uint16_t newDate[6];
  DateTime newDateTime= rtc.now();
  Serial.print("Current Time:");
  SerialShowTime(newDateTime);

  // The outer loop. Goes around and around until we get a valid
  // date and time. Thats all 6 inputs valid. It does not validate
  // February and/or leap years - it doesn't have to DateTime.isValid()
  // does that for us.
  while (1) {
    while (Serial.available()) {
      Serial.read();  // Clear serial buffer
    }

    // We have 6 different user inputs to capture.
    for (int i = 0; i < 6; i++) {

      // This loop exits when one user input is valid in
      // as far as being numeric and in range, sort of.
      // Leap years and month end dates are validated later
      // when we have the complete date and time.
      while (1) {
        Serial.print("Enter ");
        Serial.print(txt[i]);
        Serial.print(" (or -1 to abort) : ");

        while (!Serial.available()) {
          ; // Wait for user input
        }

        str = Serial.readString();  // Read user input

        // The actual value depends on the line ending configured in
        // the Serial Monitor. The configured line end character(s)
        // are part of the input string!
        // If the value is -1, then we abort the clock change
        // completely and bale out of this function.
        if ((str == "-1")   || (str == "-1\n") ||
            (str == "-1\r") || (str == "-1\r\n")) {
          Serial.println("\nABORTED");
          return;
        }

        newDate[i] = str.toInt();   // Convert user input to number and save to array

        // Validate input is in range, exit this inner while() loop
        // if so, otherwise, lets go round again.
        if (checkInput(newDate[i], minMax[i]))
          break;

      }

      Serial.println(newDate[i]); // Show user their input
    }

    // We have all the user input, was it valid - leap years,
    // days in months etc. If the DateTime is valid, we are done
    // in the outer while() loop and will exit to set the RTC.
    newDateTime = DateTime(newDate[0],(uint8_t) newDate[1], (uint8_t)newDate[2],(uint8_t) newDate[3],(uint8_t) newDate[4], (uint8_t)newDate[5]);
    if (newDateTime.isValid())
      break;

    // Otherwise, we have to do it all again.
    Serial.println("Date/time entered was invalid, please try again.");
  }

  // Update RTC as we have a valid date & time.
  
  Serial.print("Update Time:");
  SerialShowTime(newDateTime);
  Serial.println("RTC Updated!");
  rtc.adjust(newDateTime);
   Serial.print("after Update Time:");
  DateTime now= rtc.now();
  SerialShowTime(now);
}

/*
   Function to update RTC time using user input.
*////////////////////////////////////////////////////////////////////////////////////////////////////////
void serialUpdateAlarm(int alarmeNo)
{
  char Tbuffer[64];
  lcd.clear();
  lcd.setCursor(0, 0);

  Serial.print("Alarm ");
  Serial.print(alarmeNo);
  Serial.print(" :");

  DateTime nTime = rtc.getAlarm1();

  if(alarmeNo == 2)
      nTime = rtc.getAlarm2(); 
  sprintf(Tbuffer, "%02d:%02d:%02d", 
              nTime.hour(), nTime.minute(), nTime.second());
  Serial.println(Tbuffer);

  lcd.print("Edit Alarm...");

  // Prompts for user input.
  const char txt[3][15] = { "hours [0~23]", "minutes [0~59]", "seconds [0~59]"
                          };


  // Define limits for the 3 user inputs.
  const minMax_t minMax[] = {
    {0, 23},        // Hours
    {0, 59},        // Minutes
    {0, 59},        // Seconds
  };

  String str = "";

  uint8_t newDate[3];
 
  // The outer loop. Goes around and around until we get a valid
  // date and time. Thats all 6 inputs valid. It does not validate
  // February and/or leap years - it doesn't have to DateTime.isValid()
  // does that for us.
  while (1) {
    while (Serial.available()) {
      Serial.read();  // Clear serial buffer
    }

    // We have 6 different user inputs to capture.
    for (int i = 0; i < 3; i++) {

      // This loop exits when one user input is valid in
      // as far as being numeric and in range, sort of.
      // Leap years and month end dates are validated later
      // when we have the complete date and time.
      while (1) {
        Serial.print("Enter ");
        Serial.print(txt[i]);
        Serial.print(" (or -1 to abort) : ");

        while (!Serial.available()) {
          ; // Wait for user input
        }

        str = Serial.readString();  // Read user input

        // The actual value depends on the line ending configured in
        // the Serial Monitor. The configured line end character(s)
        // are part of the input string!
        // If the value is -1, then we abort the clock change
        // completely and bale out of this function.
        if ((str == "-1")   || (str == "-1\n") ||
            (str == "-1\r") || (str == "-1\r\n")) {
          Serial.println("\nABORTED");
          return;
        }

        newDate[i] = str.toInt();   // Convert user input to number and save to array

        // Validate input is in range, exit this inner while() loop
        // if so, otherwise, lets go round again.
        if (checkInput(newDate[i], minMax[i]))
          break;

      }

      Serial.println(newDate[i]); // Show user their input
    }

    // We have all the user input, was it valid - leap years,
    // days in months etc. If the DateTime is valid, we are done
    // in the outer while() loop and will exit to set the RTC.
    DateTime now = rtc.now();
    nTime = DateTime(now.year(), now.month(),now.day(), newDate[0], newDate[1], newDate[2]);
    if (nTime.isValid())
      break;

    // Otherwise, we have to do it all again.
    Serial.println("Date/time entered was invalid, please try again.");
  }

  // Update RTC as we have a valid date & time.
   Serial.print("New alarm time :");
   SerialShowTime(nTime);

   if(alarmeNo==2)
    {
     if(!rtc.setAlarm2(nTime,DS3231_A2_Hour))
        Serial.println("Set Alarme 2 failed");
    }
    else
    {
     if(!rtc.setAlarm1(nTime,DS3231_A1_Hour))
        Serial.println("Set Alarme 1 failed");
    }

   DateTime newAlarm = alarmeNo ==2 ? rtc.getAlarm2() : rtc.getAlarm1();

    Serial.print("get Alarme :");
    SerialShowTime(newAlarm);

}


/*
   Function to update LCD text
*///////////////////////////////////////////////////////////////////////////////////////////////////////

void ShowTime(DateTime theTime, char * timeType,bool AvecSeconde)
{

  char timeBuffer[] = "hh:mm "; //24 hour
  char timeBufferSec[] = "hh:mm:ss";
  // Move LCD cursor to second line, far left position.
  lcd.setCursor(0, 0);
  lcd.print(timeType);
  lcd.setCursor(AvecSeconde ? 8 : 10, 0);
  lcd.print(theTime.toString(AvecSeconde ? timeBufferSec : timeBuffer));
}

void updateLCD_Normal()
{
  // Get time and date from RTC.
  ShowTime(rtc.now(),"Heure   ",true);
  lcd.setCursor(0,1);
  if(pilule > PM_PilulePrise)
        pilule = NoPilule;
  lcd.print(piluleMsg[pilule]);
}


void updateLCD_Time()
{
  ShowTime(rtc.now(),"Heure     ",false);
  lcd.setCursor(0,1);
  if(cycle == TimeMin)
     lcd.print(ChangeMin);
   else
      lcd.print(ChangeHeure);
}

void updateLCD_Alarm1()
{
   ShowTime(rtc.getAlarm1(),"Alarme AM ",false);
   lcd.setCursor(0,1);
  if(cycle == Alarm1Min)
     lcd.print(ChangeMin);
   else
      lcd.print(ChangeHeure);

}
 
void updateLCD_Alarm2()
{
   ShowTime(rtc.getAlarm2(),"Alarme PM ",false);
   lcd.setCursor(0,1);
  if(cycle == Alarm2Min)
     lcd.print(ChangeMin);
   else
      lcd.print(ChangeHeure);
}

void updateLCD()
{

  switch(cycle)
  {
    case TimeHour:
    case TimeMin:
                    updateLCD_Time();
                    break;
    case Alarm1Hour:
    case Alarm1Min:
                    updateLCD_Alarm1();
                    break;
    case Alarm2Hour:
    case Alarm2Min:
                    updateLCD_Alarm2();
                    break;
    default:
                    updateLCD_Normal();
  }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate()
{
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);

  // stepper in half step mode .  5.625 degree with gearbox 64:1 ratio. Total step per turn = 4096
  // Run the motor forward at 500 steps/second until the motor reaches 4096 steps (1 revolution):
  // Gear ratio is 60 to 8 teeth ie 7.5:1 therefore  1/15th of 360 degrees  is ( 4096 * 60 / 8 / 15)= 2048 steps
  while (stepper.currentPosition() != 2048) {
    stepper.setSpeed(500);
    stepper.runSpeed();
  }
  stepper.disableOutputs();
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Wire.setClock(50000L);

  // Set stepper maximum steps per second:
  stepper.setMaxSpeed(1000);

  Serial.begin(9600);

  Serial.print(__FILE__);


  // ORIGINAL: My LiquidCrystal_I2C library is a different one and is
  //           built in to the Arduino. It uses begin() instead of init()
  //           to initialise the LCD.
  //    lcd.init();       // Initialize lcd

  lcd.begin(16,2);        // Initialize lcd
  lcd.backlight();    // Switch-on lcd backlight
  //lcd.noBacklight();    // Switch-off lcd backlight
  rtc.begin();        // Initialise RTC module.

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  // start button debounce
  buttonPilule.begin();
  buttonMode.begin();
  buttonPlus.begin();
  buttonMoins.begin();

  // need to kill square wave otherwise we are unable to set the alarm
  //  this is an error because we should be able to set the alarm time no matter what
  // but the setAlarm function prevent it. it holds  if (!(ctrl & 0x04)) { return false;} which is annoying.
  rtc.writeSqwPinMode(DS3231_OFF);

  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
}

void AdjustSpecificClock(bool AddFlag)
{
  DateTime tmodif;
  // get correct datetime
  switch(cycle)
  {
    case TimeHour:
    case TimeMin:
                     tmodif = rtc.now();
                     break;
    case Alarm1Hour:
    case Alarm1Min:
                      tmodif = rtc.getAlarm1();
                      break;
    case Alarm2Hour:
    case Alarm2Min:
                      tmodif = rtc.getAlarm2();
                      break;
    default:          return;
  }
  //adjust new datetim
  switch(cycle)
  {
     case TimeHour:
     case Alarm1Hour:
     case Alarm2Hour:
                      if(AddFlag)
                          tmodif = tmodif + TimeSpan(0,1,0,0);
                        else
                          tmodif = tmodif - TimeSpan(0,1,0,0);
                      break;
     case TimeMin:
     case Alarm1Min:
     case Alarm2Min:
                      if(AddFlag)
                          tmodif = tmodif + TimeSpan(0,0,1,0);
                        else
                          tmodif = tmodif - TimeSpan(0,0,1,0);
                      break;
  }
  // set Correct datetime   
  switch(cycle)
  {
    case TimeHour:
    case TimeMin:
                     rtc.adjust(tmodif);
                     break;
    case Alarm1Hour:
    case Alarm1Min:
                    rtc.setAlarm1(tmodif,DS3231_A1_Hour);        
                    break;
    case Alarm2Hour:
    case Alarm2Min:
                    rtc.setAlarm2(tmodif,DS3231_A2_Hour);        
                    break;
  }
   updateLCD();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // Get time and date from RTC.
  DateTime now = rtc.now();
  // update buttons
  buttonPilule.update();
  buttonMode.update();
  buttonPlus.update();
  buttonMoins.update();

  //delay(100);
  updateLCD();
  if(rtc.alarmFired(1))
  {
    rtc.clearAlarm(1);
    lcd.clear();
    rotate(); //start stepper
    pilule = AM_PrendrePilule;
  }
 

  if(rtc.alarmFired(2))
  {
     rtc.clearAlarm(2);
    lcd.clear();
    rotate(); //start stepper
    pilule = PM_PrendrePilule;

  }
 

  // Press button when pills taken. Check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if(buttonPilule.justPressed())
  {
    //digitalWrite(led1, LOW);
    if(pilule == AM_PrendrePilule)
    {
       pilule = AM_PilulePrise;
    }
    if(pilule == PM_PrendrePilule)
    {
       pilule = PM_PilulePrise;
    }
  }

if(buttonMode.justPressed())
{
   if(cycle != Normal)
   {
     int newcycle = ((int) cycle) +1;
     if(newcycle >  cycleMax)
     {
      newcycle = Normal;
     }
     cycle = (CycleEnum) newcycle;
   }
}

if(buttonMode.isPressed())
 {
    if(cycle == Normal)
      {
         if( (millis() - ModeTimer) > 2000)
        {
          cycle =  TimeHour;
        }
      }
 }
 else  
   ModeTimer = millis();



if( buttonPlus.isPressed() && buttonMoins.isPressed() && (cycle == Normal))
{
  if((millis() - RotateTimer) >2000)
     rotate();
}
else
  RotateTimer=millis();
 
if(buttonPlus.justPressed())
{
 
  AdjustSpecificClock(true);
}

if(buttonMoins.justPressed())
{

  AdjustSpecificClock(false);
}

  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'u') serialUpdateRTC();
    if (input == '1') serialUpdateAlarm(1);
    if (input == '2') serialUpdateAlarm(2);
  }
// pilule led flash use millis() to strobe the light

if((pilule == AM_PrendrePilule) || (pilule == PM_PrendrePilule))
      digitalWrite(led1, millis() & 512 ? HIGH : LOW);
  else
      digitalWrite(led1,LOW);

}

