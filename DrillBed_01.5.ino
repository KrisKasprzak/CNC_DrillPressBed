/*

  This program is a CNC x-y table accessory for a drill press. It's purpose is to automatically drill
  holes in PCB boards. It requires a holes.txt file with x-y hole definition. The first hole must be 0,0
  and all other holes in the positive x an y directions.

  This code is free for anyone and may be used or modified as desired.

*/

#include <Arduino.h>
#include <SPI.h>                  // lib to talk to the SPI bus, not really needed
#include <SdFat.h>                // lib to talk to the SD card
#include <ILI9341_t3.h>           // fast display driver lib
#include <XPT2046_Touchscreen.h>
#include <EEPROM.h>               // lib to talk to teensy EEPROM
#include <Stepper.h>
#include "TeensyThreads.h"
#include <font_ArialBold.h>
#include <font_Arial.h>             // custom fonts that ships with ILI9341_t3.h
#include <Arrow.h>
#include <font_ArialBlack.h>      // custom fonts that ships with ILI9341_t3.h
#include "Colors.h"    // custom utilities definition


////////////////////////////////////////////////////////////////////////////////
// #define debug
////////////////////////////////////////////////////////////////////////////////

#define ACTION_LIMIT -1
#define ACTION_OK 0
#define ACTION_PAUSE 1
#define ACTION_RESET 2
#define ACTION_STOP 3
#define FONT_SPLASH Arial_32_Bold
#define FONT_TITLE Arial_18_Bold
#define FONT_COORD Arial_16

#define FONT_BODY Arial_16_Bold
#define FONT_SMALL Arial_12
#define FONT_SMALLBUTTON Arial_12
#define FONT_BUTTON Arial_18
#define FONT_ARROW Arrow
#define FONT_SSMALL Arial_8

#define COLOR_X C_BLUE
#define COLOR_Y C_MAGENTA
#define COLOR_Z C_ORANGE

#define MARGIN 5
#define TFT_DC 9       // DC pin on LCD
#define TFT_CS 10      // chip select pin on LCD
#define LED_PIN A9     // lcd pin to control brightness
#define SD_CS A1        // chop select  pin on sd card
#define T_CS  A7
#define T_IRQ  A8

#define X_STOP  A4
#define Y_STOP  A3

#define STEP_DELAY  1100
#define PULSE_DELAY  1

//////////////////////////////////////////////////////////////////////////////////
// production board changes
/*
  #define ZMOTOR_PIN1 8
  #define ZMOTOR_PIN2 A3
  #define ZMOTOR_PIN3 A2
  #define ZMOTOR_PIN4 A0
*/
#define ZMOTOR_PIN1 4
#define ZMOTOR_PIN2 5
#define ZMOTOR_PIN3 6
#define ZMOTOR_PIN4 7

#define XMOTOR_STEP A5
#define XMOTOR_DIR A6
#define YMOTOR_STEP 1
#define YMOTOR_DIR 0
#define YMOTOR_ENABLE 2

#define MAX_SPEED 1500
#define MIN_SPEED 4000

// convert to variables as input adjustment allowed later
int BtnX, BtnY;  // holders for screen coordinate drawing
long i;
long Steps = 0;
bool IsSD;
float HoleID[500];
float HoleX[500];
float HoleY[500];
int HoleCount = 0;
int ZDepth = 100;
int MaxZDepth = 400;
bool DrillStart = false;

uint16_t ScreenBuf[125 * 125];

long TimePerInch, EndTime, StartTime;
float TestMove = 0.0;
float MaxX = 0;
float MaxY = 0;
byte Action = 0;

long JogTime = 0;
int h, m, s;
char str[12], ostr[12];
long OXSteps = 0;
long OYSteps = 0;
long OZSteps = 0;

int StepperSpeed = 0;
long StepsMoved = 0;
bool Status = false;
bool Pause = false;

int j, CursorX, OCursorX;
Adafruit_GFX_Button DrillHolesBtn;
Adafruit_GFX_Button ZeroZBtn;

Adafruit_GFX_Button   PauseBtn;
Adafruit_GFX_Button   SetBtn;
Adafruit_GFX_Button   DoneBtn;
Adafruit_GFX_Button   StopBtn;
Adafruit_GFX_Button   NextBtn;

Adafruit_GFX_Button   TestXHBtn;
Adafruit_GFX_Button   TestYHBtn;

Adafruit_GFX_Button   XPBtn;
Adafruit_GFX_Button   XNBtn;
Adafruit_GFX_Button   YPBtn;
Adafruit_GFX_Button   YNBtn;
Adafruit_GFX_Button   ZPBtn;
Adafruit_GFX_Button   ZNBtn;

Adafruit_GFX_Button   ZTBtn;
Adafruit_GFX_Button   ZBBtn;
Adafruit_GFX_Button   SetZTopBtn;
Adafruit_GFX_Button   SetZBtmBtn;

Adafruit_GFX_Button   ThBtn;
Adafruit_GFX_Button   HuBtn;
Adafruit_GFX_Button   TeBtn;
Adafruit_GFX_Button   OnBtn;

// create display and DS objects
ILI9341_t3 Display = ILI9341_t3(TFT_CS, TFT_DC, 240, 320);

// create the touch screen object
XPT2046_Touchscreen Touch(T_CS, T_IRQ);

TS_Point p;

SdFat sd;
SdFile dataFile;

unsigned long curtime;

Stepper ZStepper(513, ZMOTOR_PIN1, ZMOTOR_PIN3, ZMOTOR_PIN2, ZMOTOR_PIN4);

void setup() {

  Serial.begin(38400);

  threads.addThread(ProcessTime);

  // setup pinmodes
  pinMode(  ZMOTOR_PIN1 , OUTPUT);
  pinMode(  ZMOTOR_PIN2 , OUTPUT);
  pinMode(  ZMOTOR_PIN3 , OUTPUT);
  pinMode(  ZMOTOR_PIN4 , OUTPUT);

  pinMode(SD_CS , OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TFT_CS, OUTPUT);

  pinMode(XMOTOR_STEP, OUTPUT);
  pinMode(XMOTOR_DIR, OUTPUT);
  pinMode(YMOTOR_STEP, OUTPUT);
  pinMode(YMOTOR_DIR, OUTPUT);
  pinMode(YMOTOR_ENABLE, OUTPUT);

  pinMode(X_STOP, INPUT_PULLUP);
  pinMode(Y_STOP, INPUT_PULLUP);

  digitalWrite(SD_CS, LOW);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LED_PIN, LOW);

  digitalWrite(XMOTOR_DIR, LOW);
  digitalWrite(YMOTOR_DIR, LOW);
  digitalWrite(XMOTOR_STEP, LOW);
  digitalWrite(YMOTOR_STEP, LOW);
  digitalWrite(YMOTOR_ENABLE, HIGH);

  // fire up the display
  Display.begin();
  delay(100);

  DrillHolesBtn.initButton (&Display, 160, 80, 200, 50, C_DKGREY, C_GREY, C_BLACK, "Drill", 2, FONT_BUTTON);
  ZeroZBtn.initButton(&Display, 160, 140, 200, 50, C_DKGREY, C_GREY, C_BLACK, "Zero Z", 2, FONT_BUTTON);

  DoneBtn.initButton(&Display, 270, 225, 93, 30, C_GREY, C_GREY, C_BLACK, "Done", 2, FONT_BUTTON);

  PauseBtn.initButton( &Display, 50, 225, 93, 30, C_GREY, C_GREEN, C_BLACK, "Pause", 2, FONT_BUTTON);
  SetBtn.initButton(   &Display, 160, 225, 93, 30, C_GREY, C_YELLOW, C_BLACK, "Reset", 2, FONT_BUTTON);
  StopBtn.initButton(  &Display, 270, 225, 93, 30, C_GREY, C_RED, C_BLACK, "Stop", 2, FONT_BUTTON);

  TestXHBtn.initButton(&Display, 280, 90, 73, 30, C_GREY, C_GREEN, C_BLACK, "Test", 2, FONT_BUTTON);
  TestYHBtn.initButton(&Display, 280, 170, 73, 30, C_GREY, C_GREEN, C_BLACK, "Test", 2, FONT_BUTTON);

  NextBtn.initButton(&Display, 270, 225, 93, 30, C_WHITE, C_GREEN, C_BLACK, "Next", 2, FONT_BUTTON);

  XNBtn.initButton(&Display, 220 - 2, 120, 40, 40, C_GREY, COLOR_X, C_BLACK, "L", 3, FONT_ARROW);
  XPBtn.initButton(&Display, 300 - 2, 120, 40, 40, C_GREY, COLOR_X, C_BLACK, "R", 3, FONT_ARROW);
  YPBtn.initButton(&Display, 260 - 2, 80, 40, 40,  C_GREY, COLOR_Y, C_BLACK, "U", 3, FONT_ARROW);
  YNBtn.initButton(&Display, 260 - 2, 160, 40, 40, C_GREY, COLOR_Y, C_BLACK, "D", 3, FONT_ARROW);

  ZPBtn.initButton(&Display, 220 - 2, 80, 40, 40,  C_GREY, COLOR_Z, C_BLACK, "U", 3, FONT_ARROW);
  ZNBtn.initButton(&Display, 220 - 2, 160, 40, 40, C_GREY, COLOR_Z, C_BLACK, "D", 3, FONT_ARROW);

  ZTBtn.initButton(&Display,      260, 80, 40, 40,  C_GREY, C_TEAL, C_BLACK, "Top", 1, FONT_SMALLBUTTON);
  SetZTopBtn.initButton(&Display, 300, 80, 40, 40,  C_GREY, C_TEAL, C_BLACK, "Set", 1, FONT_SMALLBUTTON);
  SetZBtmBtn.initButton(&Display, 300, 160, 40, 40,  C_GREY, C_TEAL, C_BLACK, "Set", 1, FONT_SMALLBUTTON);
  ZBBtn.initButton(&Display,      260, 160, 40, 40, C_GREY, C_TEAL, C_BLACK, " Btm", 1, FONT_SMALLBUTTON);

  ThBtn.initButton(&Display, 25, 140, 50, 25, C_GREY, C_LTGREEN, C_BLACK, ".001", 2, FONT_SMALLBUTTON);
  HuBtn.initButton(&Display, 25, 165, 50, 25, C_GREY, C_LTGREEN, C_BLACK, ".010", 2, FONT_SMALLBUTTON);
  TeBtn.initButton(&Display, 25, 190, 50, 25, C_GREY, C_LTGREEN, C_BLACK, ".100", 2, FONT_SMALLBUTTON);
  OnBtn.initButton(&Display, 25, 215, 50, 25, C_GREY, C_LTGREEN, C_BLACK, "1.00", 2, FONT_SMALLBUTTON);

  Display.fillScreen(C_BLACK);
  Display.setRotation(3);

  Touch.begin();
  Touch.setRotation(3);

  // draw cute splash screen
  SplashScreen();

  analogWrite(LED_PIN, 255);

  Display.setTextColor(C_WHITE, C_BLACK);

  StopBtn.drawButton();

  DrawHeader("PCB Drill Mate");

  Display.setFont(FONT_BODY);
  Display.setCursor(0, 50);
  Display.print(F("Loading parameters "));

  GetParameters();

  ZStepper.setSpeed(40);

  delay(50);
  Display.print(F(" . "));
  delay(50);
  Display.print(F(" . "));
  delay(50);
  Display.print(F(" . "));

  Display.setFont(FONT_BODY);
  Display.setCursor(0, 80);
  Display.print(F("Reading: "));

  IsSD = false;

  IsSD = sd.begin(SD_CS);

  Display.setFont(FONT_COORD);
  if (IsSD) {
    ReadHoleData();
    Display.setCursor(150, 80);
    Display.print(HoleCount);
    Display.print(" holes");
  }
  else {
    while (!IsSD) {
      Display.setCursor(200, 80);
      Display.setTextColor(C_RED, C_BLACK);
      Display.print("Searching");
      delay(500) ;

      IsSD = sd.begin(SD_CS);

      Display.setCursor(200, 80);
      Display.setTextColor(C_WHITE, C_BLACK);
      Display.print("Searching");
      delay(500) ;

    }
  }

  if (HoleCount == 0) {
    Display.fillScreen(C_BLACK);
    // draw cute splash screen
    Display.setTextColor(C_RED, C_BLACK);

    Display.setFont(FONT_BODY);
    Display.setCursor(50, 20);
    Display.print(F("Need a valid"));
    Display.setCursor(50, 40);
    Display.print(F("holes.txt file"));
    while (1) {}
  }

  Display.setFont(FONT_BODY);
  Display.setCursor(0, 110);
  Display.print(F("Testing Steppers"));

  Display.setCursor(0, 150);
  Display.print(F("X"));
  Display.setCursor(0, 180);
  Display.print(F("Y"));
  Display.setCursor(0, 210);
  Display.print(F("Z"));

  // test servos and drive each to limit switch
  // this sounds odd, but drive to max distance + say 1 inch
  // if call back from Motor move is -1 it hit limit...motor OK
  // if callback is distance requested, motor is slipping and never hit limit (fail)
  Display.setFont(FONT_COORD);
  StartTime = millis();

  StepsMoved = MoveX(GetXSteps(-9.0));

  Display.setCursor(40, 150);

  if (StepsMoved == -GetXSteps(-9.0)) {
    Display.print(F("Stepper FAIL"));
  }
  else if  ( StepsMoved == -1) {
    OXSteps = 0;
    // move back to get off of stop so we have room to fine tune (0,0) later
    MoveX(GetXSteps(0.2));
    Display.print(F("Stepper OK: "));
    Display.print(OXSteps);
  }

  delay(500);

  StepsMoved = MoveY(GetYSteps(-5.0));

  Display.setCursor(40, 180);

  if (StepsMoved == -GetYSteps(-5.0)) {
    Display.print(F("Stepper FAIL"));
  }
  else if  ( StepsMoved == -1) {

    // move back to get off of stop so we have room to fine tune (0,0) later
    OYSteps = 0;
    MoveY(GetYSteps(0.2));

    Display.print(F("Stepper OK: "));
    Display.print(OYSteps);
  }
  delay(500);
  // just hope Z is moving
  StepsMoved = MoveZ(200);
  StepsMoved = MoveZ(-200);
  Display.setCursor(40, 210);
  if (StepsMoved == 0) {
    Display.print(F("Stepper OK"));
  }
  else {
    Display.print(F("Stepper FAIL "));
    Display.print(StepsMoved);
  }
  EndTime = millis();

  TestMove = (2 * GetXInches(500)) + (2 * GetYInches(500)) + (2 * GetXInches(200));
  TimePerInch = (EndTime - StartTime) / (TestMove);
  TimePerInch = TimePerInch / 1000;
  TimePerInch = TimePerInch / 4000;

  delay(1000);

  DisplayOff();

  MainMenu();

  OptimizeData();

  ZeroOut();

}

void loop() {

  int i = 0;
  int TempX1, TempY1, TempX2, TempY2;
  int Top = 40;
  int Left = 5;
  int Height = 165;
  int Width = 314;
  int Delta = 0;
  bool Pause = false;
  bool KeepIn = false;

  Display.fillScreen(C_BLACK);

  DrawHeader("PCB Drill Mate");

  Display.setFont(FONT_BUTTON);
  StopBtn.drawButton();

  PauseBtn.drawButton();
  StopBtn.drawButton();
  SetBtn.drawButton();
  StopBtn.show();

  // now draw the points
  JogTime = -1,
  PlotBoard(Left, Top, Width, Height, C_YELLOW, C_WHITE, 2, JogTime);
  DisplayOn();

  // now drill and draw completed point and line during each pass

  curtime = millis();
  DrillStart = true;

  for (i = 0; i < HoleCount; i++) {

    TempX1 = MapFloat(HoleX[i], MaxX , 0, Left + MARGIN, Width - MARGIN );
    TempY1 = MapFloat(HoleY[i], 0, MaxY, Top + Height - MARGIN , Top + MARGIN);

    TempX2 = MapFloat(HoleX[i + 1], MaxX, 0 , Left + MARGIN, Width - MARGIN );
    TempY2 = MapFloat(HoleY[i + 1], 0, MaxY, Top + Height - MARGIN   , Top + MARGIN);

    ProcessButtons();

    if (i == 0) {
      Display.fillCircle(TempX1, TempY1, 2, COLOR_X);
      Status = MoveX(HoleX[i]);
      Display.fillCircle(TempX1, TempY1, 2, COLOR_Y);
      Status = MoveY(HoleY[i]);
    }

    else {

      // move X
      Display.fillCircle(TempX1, TempY1, 2, COLOR_X);
      Delta = HoleX[i] - HoleX[i - 1];
      Status = MoveX(Delta);
      ProcessButtons();
      // move Y
      Display.fillCircle(TempX1, TempY1, 2, COLOR_Y);
      Delta = HoleY[i] - HoleY[i - 1];
      Status = MoveY(Delta);
    }
    SmartDelay(200);
    Status = MoveZ(MaxZDepth);
    SmartDelay(20);
    Status = MoveZ(-MaxZDepth);
    SmartDelay(200);

    ProcessButtons();

    // assume hole drilled if they hit next
    Display.fillCircle(TempX1, TempY1, 2, COLOR_Z);

    if (i == 0 ) {
      Display.drawLine(TempX1, TempY1, TempX2, TempY2, C_DKGREY);
      Display.fillCircle(TempX1, TempY1, 2, C_DKRED);
    }
    else if (i == (HoleCount - 1)) {
      Display.fillCircle(TempX1, TempY1, 2, C_DKGREEN);
    }
    else {
      Display.drawLine(TempX1, TempY1, TempX2, TempY2, C_DKGREY);
      Display.fillCircle(TempX1, TempY1, 2, C_DKYELLOW);
    }

    if (Action == ACTION_PAUSE) {
      Action = ACTION_OK;
      Pause = false;
      PauseBtn.drawButton(true);
      while (!Pause) {
        if (Touch.touched()) {
          ProcessTouch();
          if (PressIt(PauseBtn) == true) {
            Pause = true;
          }
        }
      }
      PauseBtn.drawButton();
    }

    if (Action == ACTION_STOP) {
      StopBtn.drawButton(true);
      while (1) { }
    }

    if (Action == ACTION_RESET) {
      Action = ACTION_OK;
      threads.suspend(1);
      SmartDelay(500);
      Display.readRect(194, 58 , 125, 125, ScreenBuf);
      threads.restart(1);
      Display.fillRect(194, 58 , 125, 125, C_WHITE);
      Display.drawRect(194, 58 , 125, 125, C_BLUE);

      XPBtn.drawButton();
      XNBtn.drawButton();
      YPBtn.drawButton();
      YNBtn.drawButton();
      SetBtn.drawButton(true);

      XPBtn.show();
      XNBtn.show();
      YPBtn.show();
      YNBtn.show();

      KeepIn = false;

      while (!KeepIn) {
        if (Touch.touched()) {
          ProcessTouch();

          if (PressIt(SetBtn) == true) {
            KeepIn = true;
          }
          if (PressIt(XPBtn) == true) {
            XPBtn.drawButton(true);
            Status = MoveX(GetXSteps(-0.01));
            XPBtn.drawButton();
          }
          if (PressIt(XNBtn) == true) {
            XNBtn.drawButton(true);
            Status = MoveX(GetXSteps(0.01));
            XNBtn.drawButton();
          }
          if (PressIt(YPBtn) == true) {
            YPBtn.drawButton(true);
            Status = MoveY(GetYSteps(-0.01));
            YPBtn.drawButton();
          }
          if (PressIt(YNBtn) == true) {
            YNBtn.drawButton(true);
            Status = MoveY(GetYSteps(0.01));
            YNBtn.drawButton();
          }
        }
      }

      SetBtn.drawButton();
      XPBtn.hide();
      XNBtn.hide();
      YPBtn.hide();
      YNBtn.hide();
      threads.suspend(1);
      SmartDelay(500);
      Display.writeRect(194, 58 , 125, 125, ScreenBuf);
      threads.restart(1);

    }
  }
  Display.setFont(FONT_BODY);
  threads.suspend(1);
  while (1) {
    Display.setTextColor(C_RED, C_BLACK);
    Display.fillRect(0, 210, 220, 30, C_BLACK);
    Display.setCursor(5, 215);
    Display.print(F("Drilling Complete"));
    delay(500);

    Display.setTextColor(C_WHITE, C_BLACK);
    Display.fillRect(0, 210, 220, 30, C_BLACK);
    Display.setCursor(5, 215);
    Display.print(F("Drilling Complete"));
    delay(500);

  }
}

void ProcessTime() {

  while (1) {

    threads.delay(1000);
    CursorX = 200;
    if (DrillStart) {
      JogTime = (millis() - curtime) / 1000;
      h = (JogTime / 3600);
      m = (JogTime - (3600 * h)) / 60;
      s = (JogTime - (3600 * h) - (m * 60));
      sprintf(str, "%02d:%02d:%02d", h, m, s);
      Display.setFont(FONT_BODY);
      Display.setTextColor(C_WHITE, C_DKBLUE);
      for (j = 0; j < 8; j++) {
        if (str[j] != ostr[j]) {
          Display.fillRect (CursorX, 5, OCursorX - CursorX, 20, C_DKBLUE);
          ostr[j] = str[j];
        }
        //blank out from start to end
        Display.setCursor(CursorX, 5);
        Display.print(str[j]);
        CursorX = Display.getCursorX();
        if (j == 7) {
          OCursorX = CursorX;
        }
      }

    }

    threads.yield();
  }
}

void SmartDelay(unsigned long t) {

  unsigned long temp = millis();
  // hopefully this program will not run for more that 27 days or this will freak out
  while ((millis() - temp) < t) {}

}

void ProcessButtons() {

  if (Touch.touched()) {

    if (PressIt(PauseBtn) == true) {
      PauseBtn.drawButton(true);
      Action = ACTION_PAUSE;
    }

    else if (PressIt(StopBtn) == true) {
      StopBtn.drawButton(true);
      Action = ACTION_STOP;
    }

    else if (PressIt(SetBtn) == true) {
      SetBtn.drawButton(true);
      Action = ACTION_RESET;
    }
  }
}

void OptimizeData() {

  int i = 0;
  int j = 0;
  float MinDistance = 0.0;
  float Temp = 0.0;
  float TempID = 0.0;
  float TempX = 0.0;
  float TempY = 0.0;
  int MinIndex = 0;
  int Footer = 35;

  int Height = Display.height();
  int Width = Display.width();
  bool NeedToSwap = false;
  int NextHole;

  bool KeepIn = true;

  DisplayOff();

  Display.fillScreen(C_BLACK);
  NextBtn.drawButton();
  // now optimize the paths

  for (i = 0; i < HoleCount; i++) {
    MinDistance = 99999.9;
    NeedToSwap = false;
    NextHole = i + 1;
    for (j = i + 1; j < HoleCount; j++) {
      Temp = GetDistance( HoleX[i], HoleY[i], HoleX[j], HoleY[j] );
      if (Temp < MinDistance) {
        NeedToSwap = true;
        MinDistance = Temp;
        MinIndex = j;
      }
    }
    if (NeedToSwap) {
      // now that min found, swap them
      TempX = HoleX[NextHole];
      TempY = HoleY[NextHole];
      TempID = HoleID[NextHole];
      HoleX[NextHole] = HoleX[MinIndex];
      HoleY[NextHole] = HoleY[MinIndex];
      HoleID[NextHole] = HoleID[MinIndex];
      HoleX[MinIndex] = TempX;
      HoleY[MinIndex] = TempY;
      HoleID[MinIndex] = TempID;
    }
  }

  // now draw the points
  JogTime = 0;

  // because we are drilling from the bottom
  // we need to reverse x direction

  PlotBoard(0, 0, 320, 200, C_YELLOW, C_LTGREY, 2, JogTime);

  h = (JogTime / 3600);
  m = (JogTime - (3600 * h)) / 60;
  s = (JogTime - (3600 * h) - (m * 60));
  sprintf(str, "%02d:%02d:%02d", h, m, s);
  Display.fillRect(0, Height - Footer, Width , Footer, C_BLACK);
  Display.setFont(FONT_BUTTON);
  NextBtn.drawButton();
  Display.setFont(FONT_BODY);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setCursor(0, 220);
  Display.print(F("Approx time: "));
  Display.print(str);

  DisplayOn();
  KeepIn = true;
  while (KeepIn) {
    if (Touch.touched()) {
      ProcessTouch();
      if (PressIt(NextBtn) == true) {
        KeepIn = false;
      }
    }
  }
}

void ReadHoleData() {

  char line[180];
  int totalValues;
  int n;
  float floatArr[4]; // specify size here

  dataFile.open("holes.txt", O_READ);

  HoleCount = 0;
  while ((n = dataFile.fgets(line, sizeof(line))) > 0) {

    if (line[n - 1] == '\n') {
      // remove '\n'
      line[n - 1] = 0;
      totalValues = strCSV2Float(&floatArr[0] , &line[0]);

      if (totalValues > 0) {
        HoleID[HoleCount] = floatArr[0];
        HoleX[HoleCount] = GetXSteps(floatArr[1] - HoleX[0]);
        HoleY[HoleCount] = GetYSteps(floatArr[2] - HoleY[HoleCount]);

#ifdef debug
        Serial.print("File Read ");
        Serial.print(floatArr[0], 3);
        Serial.print(", ");
        Serial.print(floatArr[1], 3);
        Serial.print(", ");
        Serial.print(floatArr[2], 3);
        Serial.print(", ");
        Serial.print(floatArr[3], 3);
        Serial.print(": ");
        Serial.print(totalValues);
        Serial.print(" - ");
        Serial.print(HoleCount);
        Serial.print(" - ");
        Serial.print(HoleX[HoleCount]);
        Serial.print(", ");
        Serial.println(HoleY[HoleCount]);
#endif

        if (HoleX[HoleCount] > MaxX) {
          MaxX = HoleX[HoleCount];
        }

        if (HoleY[HoleCount] > MaxY) {
          MaxY = HoleY[HoleCount];
        }

        HoleCount++;

      }

    } else {
      // no '\n' - line too long or missing '\n' at EOF
      // handle error
    }
  }
  dataFile.close();

}


int strCSV2Float(float * strFloatArray , char *myCSVStringing) {

  int strLen = 0;
  int commaCount = 0; // count the number of commas
  int commaCountOld = 0; // count the number of commas
  int wordEndChar = 0;
  int wordStartChar = -1;
  int wordLength = 0;

  for (strLen = 0; myCSVStringing[strLen] != '\0'; strLen++) // first get the string length
  {

    if ( (myCSVStringing[strLen] == ',')  || ( myCSVStringing[strLen + 1] == '\0' ))
    {
      commaCount++;
      wordEndChar = strLen;
    }
    if ( (commaCount - commaCountOld) > 0 )
    {
      int aIter = 0;
      wordLength = (wordEndChar - wordStartChar);
      char word[55] = "";
      for (aIter = 0;  aIter < wordLength; aIter++)
      {
        word[aIter] = myCSVStringing[strLen - wordLength + aIter + 1];
      }

      if (word[aIter - 1] == ',')
        word[aIter - 1] = '\0';

      //  printf("\n");
      word[wordLength] = '\0';
      strFloatArray[commaCount - 1] = atof(&word[0]);

      wordLength = 0;
      wordStartChar = wordEndChar;
      commaCountOld = commaCount;

    }
  }

  return commaCount;
}

void SplashScreen() {

  Display.fillScreen(C_BLACK);
  Display.fillScreenVGradient(0x000E, 0x0000);
  DisplayOn();

  for (int i = 0; i <= 320; i += 2) {
    Display.setTextColor(C_WHITE, C_BLACK);
    Display.setFont(FONT_SPLASH );
    Display.setCursor(30, 40);
    Display.print(F("PCB"));
    Display.setCursor(30, 100);
    Display.print(F("Drill Mate"));

    Display.setFont(FONT_BODY );
    Display.setCursor(40, 180);
    Display.print(F("v 1.5 Kasprzak (c)"));
    Display.setScroll(i);
  }

  DisplayOff();
  Display.fillScreen(C_BLACK);

}

void GetParameters() {

  byte Status = 0;
  int i = 0;

  EEPROM.get(0, Status);

  if (Status == 255) {
    // new unit zero everything out
    for (i = 0; i < 1000; i++) {
#ifdef debug
      Serial.print(F("Clearing EEPROM"));
#endif
      EEPROM.put(i, 0);
    }
    // now set defaults
    Status = 1;
    EEPROM.put(0, Status);
    ZDepth = 100;
    EEPROM.put(100, MaxZDepth);

  }
  else {
    EEPROM.get(100, MaxZDepth);
  }

#ifdef debug
  Serial.print(F("MaxZDepth: "));
  Serial.println(MaxZDepth);

#endif

}

void MainMenu() {

  // UI to draw screen and capture input
  bool KeepIn = true;

  DrawMainMenu();

  while (KeepIn) {

    // if touch screen pressed handle it
    if (Touch.touched()) {

      ProcessTouch();

      if (PressIt(DrillHolesBtn) == true) {
        KeepIn = false;
      }

      if (PressIt(ZeroZBtn) == true) {
        ZeroZ();
        DrawMainMenu();
      }

    }
  }
}

/*

  service function UI screen

*/
void DrawMainMenu() {

  //nothing fancy, just a header and some buttons
  Display.fillScreen(C_BLACK);

  DrawHeader("Main Menu");

  Display.setFont(FONT_BODY );
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(FONT_BUTTON);
  DrillHolesBtn.drawButton();
  ZeroZBtn.drawButton();

  DisplayOn();

}

void DisplayOff() {

  int i = 0;
  for (i = 255; i >= 0; i--) {
    analogWrite(LED_PIN, i);
    delay(2);
  }

}

void DisplayOn() {

  int i = 0;

  for (i = 0; i <= 255; i++) {
    analogWrite(LED_PIN, i);
    delay(2);
  }
}

void ZeroOut() {

  bool KeepIn = true;

  DisplayOff();
  Display.fillScreen(C_BLACK);

  DrawHeader("Set 0,0,0");

  NextBtn.drawButton();
  NextBtn.show();
  StopBtn.hide();

  XPBtn.drawButton();
  XNBtn.drawButton();
  YPBtn.drawButton();
  YNBtn.drawButton();
  ZPBtn.drawButton();
  ZNBtn.drawButton();

  ThBtn.drawButton(true);
  HuBtn.drawButton();
  TeBtn.drawButton();
  OnBtn.drawButton();
  ThBtn.press(true);


  // now draw the points
  JogTime = -1;
  PlotBoard(0, 35, 170, 88, C_DKYELLOW, C_DKGREY, 1, JogTime) ;

  DisplayOn();

  while (KeepIn) {

    // if touch screen pressed handle it
    if (Touch.touched()) {

      ProcessTouch();

      if (PressIt(NextBtn) == true) {
        KeepIn = false;
      }
      else if (PressIt(StopBtn) == true) {
        // KeepIn = false;
      }

      else if (PressIt(ThBtn) == true) {

        ThBtn.drawButton(true);
        HuBtn.drawButton();
        TeBtn.drawButton();
        OnBtn.drawButton();
      }
      else if (PressIt(HuBtn) == true) {

        ThBtn.drawButton();
        HuBtn.drawButton(true);
        TeBtn.drawButton();
        OnBtn.drawButton();
      }
      else if (PressIt(TeBtn) == true) {

        ThBtn.drawButton();
        HuBtn.drawButton();
        TeBtn.drawButton(true);
        OnBtn.drawButton();
      }
      else if (PressIt(OnBtn) == true) {

        ThBtn.drawButton();
        HuBtn.drawButton();
        TeBtn.drawButton();
        OnBtn.drawButton(true);
      }

      else if (PressIt(XPBtn) == true) {

        StopBtn.drawButton();
        XPBtn.drawButton(true);
        if (ThBtn.isPressed()) {
          Status = MoveX(GetXSteps(-0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveX(GetXSteps(-0.01));
        }
        if (TeBtn.isPressed()) {

          Status = MoveX(GetXSteps(-0.1));
        }
        if (OnBtn.isPressed()) {
          Status = MoveX(GetXSteps(-1.0));
        }
        XPBtn.drawButton();
        StopBtn.drawButton(true);
        NextBtn.drawButton();
        NextBtn.show();
        StopBtn.hide();
      }
      else if (PressIt(XNBtn) == true) {

        StopBtn.drawButton();
        XNBtn.drawButton(true);
        if (ThBtn.isPressed()) {
          Status = MoveX(GetXSteps(0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveX(GetXSteps(0.01));
        }
        if (TeBtn.isPressed()) {
          Status = MoveX(GetXSteps(0.1));
        }
        if (OnBtn.isPressed()) {
          Status = MoveX(GetXSteps(1.0));
        }
        XNBtn.drawButton();
        StopBtn.drawButton(true);
        NextBtn.drawButton();
        NextBtn.show();
        StopBtn.hide();
      }
      else if (PressIt(YPBtn) == true) {

        StopBtn.drawButton();
        YPBtn.drawButton(true);
        if (ThBtn.isPressed()) {
          Status = MoveY(GetYSteps(-0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveY(GetYSteps(-0.01));
        }
        if (TeBtn.isPressed()) {
          Status = MoveY(GetYSteps(-0.1));
        }
        if (OnBtn.isPressed()) {
          Status = MoveY(GetYSteps(-1.0));
        }

        YPBtn.drawButton();
        StopBtn.drawButton(true);
        NextBtn.drawButton();
        NextBtn.show();
        StopBtn.hide();
      }
      else if (PressIt(YNBtn) == true) {

        StopBtn.drawButton();
        YNBtn.drawButton(true);
        if (ThBtn.isPressed()) {
          Status = MoveY(GetYSteps(0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveY(GetYSteps(0.01));
        }
        if (TeBtn.isPressed()) {
          Status = MoveY(GetYSteps(0.1));
        }
        if (OnBtn.isPressed()) {
          Status = MoveY(GetYSteps(1.0));
        }

        YNBtn.drawButton();
        StopBtn.drawButton(true);
        NextBtn.drawButton();
        NextBtn.show();
        StopBtn.hide();
      }

      else if (PressIt(ZPBtn) == true) {

        ZPBtn.drawButton(true);
        StopBtn.drawButton();

        if (ThBtn.isPressed()) {
          Status = MoveZ(GetZSteps(-0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveZ(GetZSteps(-0.01));
        }
        if (TeBtn.isPressed()) {
          Status = MoveZ(GetZSteps(-0.1));
        }

        ZPBtn.drawButton();
        StopBtn.drawButton(true);
        NextBtn.drawButton();
        NextBtn.show();
        StopBtn.hide();
      }
      else if (PressIt(ZNBtn) == true) {

        ZNBtn.drawButton(true);
        StopBtn.drawButton();
        StopBtn.show();
        DoneBtn.hide();

        if (ThBtn.isPressed()) {
          Status = MoveZ(GetZSteps(0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveZ(GetZSteps(0.01));
        }
        if (TeBtn.isPressed()) {
          Status = MoveZ(GetZSteps(0.1));
        }
        ZNBtn.drawButton();
        StopBtn.drawButton(true);
        NextBtn.drawButton();
        NextBtn.show();
        StopBtn.hide();

      }
    }
  }

  DisplayOff();

}

void ProcessTouch() {

  if (Touch.touched()) {
    p = Touch.getPoint();
    BtnX = p.x;
    BtnY = p.y;

#ifdef debug
    Serial.print("real coordinates: ");
    Serial.print(BtnX);
    Serial.print(", ");
    Serial.print (BtnY);
#endif

    BtnX = map(p.x, 3975, 169, 0, 320);
    BtnY = map(p.y, 3850, 304, 0, 240);

#ifdef debug
    Serial.print(", Mapped coordinates: ");
    Serial.print(BtnX);
    Serial.print(", ");
    Serial.println(BtnY);
    Display.drawPixel(BtnX, BtnY, C_RED);
#endif

  }
}

bool PressIt(Adafruit_GFX_Button TheButton) {

  if (TheButton.contains(BtnX, BtnY)) {

    TheButton.drawButton(true);
    TheButton.press(true);

    while (Touch.touched()) {

      ProcessTouch();

      if (TheButton.contains(BtnX, BtnY)) {

        if (TheButton.isPressed() == false) {
          TheButton.drawButton(true);
          TheButton.press(true);
        }
      }
      else {
        if (TheButton.isPressed() == true) {
          TheButton.drawButton(false);
          TheButton.press(false);
        }
      }

      delay(50);

    }

    if (TheButton.contains(BtnX, BtnY))  {
      // button released while on button
      TheButton.press(false);
      TheButton.drawButton(false);
      return true;
    }
    else  {
      // button released off of button
      TheButton.press(false);
      TheButton.drawButton(false);
      return false;
    }
  }

  return false;
}

void ZeroZ() {

  int Top = 40;
  int Left = 5;
  int Height = 80;
  int Width = 145;
  bool KeepIn = true;

  Display.fillScreen(C_BLACK);
  DrawHeader("Set Z Depth");

  DoneBtn.drawButton();

  ZPBtn.drawButton();
  ZNBtn.drawButton();
  ZTBtn.drawButton();
  ZBBtn.drawButton();
  SetZTopBtn.drawButton();
  SetZBtmBtn.drawButton();

  ThBtn.drawButton(true);
  HuBtn.drawButton();
  TeBtn.drawButton();

  ThBtn.press(true);

  DrawZInstructions( Left,  Top,  Width,  Height);

  Display.setFont(FONT_BODY );
  Display.setTextColor(C_WHITE, C_BLACK);

  Display.fillRect(150, 180, 60, 25, C_BLACK);
  Display.setCursor(70, 180);
  Display.print("Z steps: ");
  Display.print(MaxZDepth);

  Display.fillRect(150, 210, 60, 25, C_BLACK);
  Display.setCursor(70, 215);
  Display.print("Z depth: ");
  Display.print(GetZInches(MaxZDepth), 2);

  DisplayOn();

  while (KeepIn) {

    // if touch screen pressed handle it
    if (Touch.touched()) {

      ProcessTouch();

      if (PressIt(DoneBtn) == true) {
        KeepIn = false;
      }

      if (PressIt(StopBtn) == true) {
        // KeepIn = false;
      }

      if (PressIt(ZTBtn) == true) {
        MoveZ(-MaxZDepth);
      }

      if (PressIt(ZBBtn) == true) {
        MoveZ(MaxZDepth);
      }

      if (PressIt(SetZTopBtn) == true) {
        ZDepth = 0.0;
      }

      if (PressIt(SetZBtmBtn) == true) {
        MaxZDepth = ZDepth;
      }

      if (PressIt(ThBtn) == true) {
        ThBtn.drawButton(true);
        HuBtn.drawButton();
        TeBtn.drawButton();

      }
      if (PressIt(HuBtn) == true) {
        ThBtn.drawButton();
        HuBtn.drawButton(true);
        TeBtn.drawButton();

      }
      if (PressIt(TeBtn) == true) {
        ThBtn.drawButton();
        HuBtn.drawButton();
        TeBtn.drawButton(true);

      }

      if (PressIt(DoneBtn) == true) {
        KeepIn = false;
      }

      //////////////////////////////////////////////////////////////////////
      if (PressIt(ZPBtn) == true) {

        ZPBtn.drawButton(true);
        StopBtn.show();
        DoneBtn.hide();
        StopBtn.drawButton();

        if (ThBtn.isPressed()) {
          Status = MoveZ(GetZSteps(-0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveZ(GetZSteps(-0.01));
        }
        if (TeBtn.isPressed()) {


          Status = MoveZ(GetZSteps(-0.1));
        }

        ZPBtn.drawButton();


        StopBtn.hide();
        DoneBtn.show();
        DoneBtn.drawButton();

      }
      if (PressIt(ZNBtn) == true) {

        ZNBtn.drawButton(true);
        StopBtn.drawButton();
        StopBtn.show();
        DoneBtn.hide();

        if (ThBtn.isPressed()) {
          Status = MoveZ(GetZSteps(0.001));
        }
        if (HuBtn.isPressed()) {
          Status = MoveZ(GetZSteps(0.01));
        }
        if (TeBtn.isPressed()) {
          Status = MoveZ(GetZSteps(0.1));
        }

        ZNBtn.drawButton();
        StopBtn.hide();
        DoneBtn.show();
        DoneBtn.drawButton();
      }
      Display.setFont(FONT_BODY );
      Display.setTextColor(C_WHITE, C_BLACK);

      Display.fillRect(150, 180, 60, 25, C_BLACK);
      Display.setCursor(70, 180);
      Display.print("Z steps: ");
      Display.print(MaxZDepth);

      Display.fillRect(150, 210, 60, 25, C_BLACK);
      Display.setCursor(70, 215);
      Display.print("Z depth: ");
      Display.print(GetZInches(MaxZDepth), 2);

    }
  }

  EEPROM.put(100, MaxZDepth);

  DisplayOff();
  Display.fillScreen(C_BLACK);

}

void DrawZInstructions(int Left, int Top, int Width, int Height) {

  Display.setFont(FONT_SSMALL);
  Display.setTextColor(C_WHITE, C_BLACK);

  Display.fillRect(Left - 2 , Top - 2 , Width + 4  , Height + 4 , C_BLACK);
  Display.drawRect(Left - 4, Top - 4, Width + 8 , Height + 8, C_WHITE);

  Display.setCursor(Left + 5 , Top + 1 );
  Display.print(F("1. Jog Z up untill stall"));
  Display.setCursor(Left + 5 , Top + 15 );
  Display.print(F("2. Press Z Top button"));
  Display.setCursor(Left + 5 , Top + 29 );
  Display.print(F("3. Raise bed to clear drill bit"));
  Display.setCursor(Left + 5 , Top + 43 );
  Display.print(F("4. Jog Z to drill through PCB"));
  Display.setCursor(Left + 5 , Top + 57 );
  Display.print(F("5. Press Zero Z button"));
  Display.setCursor(Left + 5 , Top + 71 );
  Display.print(F("5. Test with Top/Bottom"));

}

float GetDistance( float X1, float Y1, float X2, float Y2 ) {
  return sqrt( ((X2 - X1) * (X2 - X1)) + ((Y2 - Y1) * (Y2 - Y1)));
}

unsigned long GetJogTime( float X1, float Y1, float X2, float Y2 ) {
  return ( 1 + (TimePerInch * abs(X2 - X1)) + (TimePerInch * abs(Y2 - Y1)));
}

float MapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DrawHeader(const char text[]) {

  Display.setFont(FONT_TITLE);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 30, C_DKBLUE);
  Display.setCursor(10 , 5 );
  Display.print(text);

}

void PlotBoard(int Left, int Top, int Width, int Height, long PointColor, long LineColor, byte PointSize, long & JogTime) {

  int TempX1, TempY1, TempX2, TempY2, i;

  // first draw the points and drill path in native file read



  Display.drawRect(Left, Top, Width , Height, C_WHITE);
  Display.fillRect(Left + 1, Top + 1, Width - 2 , Height - 2, C_BLACK);

  for (i = 0; i < HoleCount; i++) {

    // now that min hole found, display
    // we must reverse x
    TempX1 = MapFloat(HoleX[i], MaxX, 0, Left + MARGIN, Width - MARGIN );
    TempY1 = MapFloat(HoleY[i], 0, MaxY, Top + Height - MARGIN , Top + MARGIN);

    TempX2 = MapFloat(HoleX[i + 1], MaxX , 0 , Left + MARGIN, Width - MARGIN );
    TempY2 = MapFloat(HoleY[i + 1], 0, MaxY, Top + Height - MARGIN   , Top + MARGIN);

    if (i == 0 ) {
      Display.drawLine(TempX1, TempY1, TempX2, TempY2, LineColor);
      Display.fillCircle(TempX1, TempY1, 2, C_RED);
      if (JogTime >= 0) {
        JogTime = JogTime + GetJogTime(HoleX[i], HoleY[i], HoleX[i + 1], HoleY[i + 1]);
      }
    }
    else if (i == (HoleCount - 1)) {
      Display.fillCircle(TempX1, TempY1, 2, C_GREEN);

    }
    else {
      Display.drawLine(TempX1, TempY1, TempX2, TempY2, LineColor);
      Display.fillCircle(TempX1, TempY1, PointSize, PointColor);
      if (JogTime >= 0) {
        JogTime = JogTime + GetJogTime(HoleX[i], HoleY[i], HoleX[i + 1], HoleY[i + 1]);
      }
    }
    // add some time for manual drilling
    JogTime = JogTime + 5;

  }
}



long MoveX (long Steps) {

  //  Serial.print("OXSteps ");
  //  Serial.println(OXSteps);

  long i = 0;

  OXSteps = OXSteps + Steps;

  if (Steps > 0) {

    digitalWrite(XMOTOR_DIR, LOW);

    for (i = 0; i < Steps; i++) {

      if (OXSteps > 36000) {
        // limit hit
        //  return ACTION_LIMIT;
      }
      StepperSpeed = GetStepperSpeed(i, Steps);
      ProcessButtons();
      digitalWrite(XMOTOR_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWrite(XMOTOR_STEP, LOW);
      delayMicroseconds(StepperSpeed);


    }
  }

  else if (Steps < 0) {

    digitalWrite(XMOTOR_DIR, HIGH);
    Steps = -Steps;
    for (i = 0; i < Steps; i++) {
      if (digitalRead(X_STOP) == LOW) {
        // limit hit
        return ACTION_LIMIT;
      }

      StepperSpeed = GetStepperSpeed(i, Steps);
      ProcessButtons();
      digitalWrite(XMOTOR_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWrite(XMOTOR_STEP, LOW);
      delayMicroseconds(StepperSpeed);

    }
  }
  return i;
}

long MoveY (long Steps) {

  long i = 0;

  OYSteps = OYSteps + Steps;

  //  Serial.print("OYSteps ");
  //  Serial.println(OYSteps);

  digitalWrite(YMOTOR_ENABLE, LOW);

  if (Steps > 0) {

    digitalWrite(YMOTOR_DIR, HIGH);

    for (i = 0; i < Steps; i++) {

      if (OYSteps > 16400) {
        // limit hit
        //   return ACTION_LIMIT;
      }
      StepperSpeed = GetStepperSpeed(i, Steps);
      ProcessButtons();
      digitalWrite(YMOTOR_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWrite(YMOTOR_STEP, LOW);
      delayMicroseconds(StepperSpeed);
    }
  }

  else if (Steps < 0) {

    digitalWrite(YMOTOR_DIR, LOW);
    Steps = -Steps;
    for (i = 0; i < Steps; i++) {
      if (digitalRead(Y_STOP) == LOW) {
        // limit hit
        return ACTION_LIMIT;
      }
      StepperSpeed = GetStepperSpeed(i, Steps);
      ProcessButtons();
      digitalWrite(YMOTOR_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWrite(YMOTOR_STEP, LOW);
      delayMicroseconds(StepperSpeed);
    }
  }
  digitalWrite(YMOTOR_ENABLE, LOW);

  return i;
}

long MoveZ (long Steps) {
  long i = 0;

  ZDepth = ZDepth + Steps;

  Display.setTextColor(C_WHITE, C_BLACK);

  if (Steps > 0) {
    for (i = 0; i < Steps; i++) {
      ProcessButtons();
      ZStepper.step(1);
    }
  }
  else if (Steps < 0) {
    Steps = -Steps;
    for (i = 0; i < Steps; i++) {
      // if touch screen pressed handle it
      ProcessButtons();
      ZStepper.step(-1);
    }
  }

  return i;

}

int GetStepperSpeed(long i, long Steps) {

  int Speed = MIN_SPEED;

  if (Steps < 300) {
    Speed = MIN_SPEED;
  }

  else if (i < 100) {
    Speed = MIN_SPEED - (((MIN_SPEED - MAX_SPEED) / 100) * i);
  }

  else if ( i > (Steps - 100)) {
    Speed = MIN_SPEED - ((Steps - i) * ((MIN_SPEED - MAX_SPEED) / 100));
  }
  else {
    Speed = MAX_SPEED;

  }

  return Speed;

}

long GetXSteps(float Distance) {
  return (200 *  Distance * 20 );
}
float GetXInches(long Steps) {
  return ((float) Steps / ( 200. * 20.));
}

long GetYSteps(float Distance) {
  return (200 *  Distance * 20 );
}

float GetYInches(long Steps) {
  return ((float) Steps / ( 200. * 20.));
}

long GetZSteps(float Distance) {
  return ( 2048.0 *  Distance);
}
float GetZInches(long Steps) {
  return ( Steps / 2048.0 );
}
