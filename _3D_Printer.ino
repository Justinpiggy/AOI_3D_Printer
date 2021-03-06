#include <Scheduler.h>
#include <PID.h>
#include <SPI.h>
#include <SD.h>
#include <MAX6675.h>
#include <LiquidCrystal.h>

//Menu item counter
#define PAGE_0_MAX 5
#define PAGE_1_MAX 10

//User settings
#define LCD_INTERVAL 500   //LCD refresh interval
#define BUFFER_SIZE 100   //RAM buffer size
#define PREHEATING_TIMEOUT 120000   //Preheating time limit
#define SERIAL_TIMEOUT 5000
#define MAX_LCDTIME 100

//Debug switches
#define CODE_INFO false
#define POS_INFO false
#define CMD_INFO false
#define BUF_INFO false
#define EN_SERIAL_CLI true
#define EN_SERIAL_REPORT true

//X-axis stepper settings
#define X_STEPS_PER_INCH 5080
#define X_STEPS_PER_MM   200 //Subdivistion set to 8 25*Sub

//Y-axis stepper settings
#define Y_STEPS_PER_INCH 5080
#define Y_STEPS_PER_MM   200 //Subdivistion set to 8 25*Sub

//Z-axis stepper settings
#define Z_STEPS_PER_INCH 5080
#define Z_STEPS_PER_MM   200 //Subdivistion set to 8  25*Sub

//Extruder stepper settings
#define E_STEPS_PER_INCH 4700
#define E_STEPS_PER_MM   185 //Subdivistion set to 32  5.8*Sub


#define CURVE_SECTION_MM 0.019685
#define CURVE_SECTION_INCH 0.5

//Max feedrate
#define MAX_FEEDRATE 2400

//X-axis stepper connection
#define X_STEP_PIN 25
#define X_DIR_PIN 30
#define X_MIN_PIN 38
#define X_MAX_PIN 39
#define X_ENABLE_PIN 31

//Y-axis stepper connection
#define Y_STEP_PIN 26
#define Y_DIR_PIN 32
#define Y_MIN_PIN 40
#define Y_MAX_PIN 41
#define Y_ENABLE_PIN 33

//Z-axis stepper connection
#define Z_STEP_PIN 27
#define Z_DIR_PIN 34
#define Z_MIN_PIN 42
#define Z_MAX_PIN 43
#define Z_ENABLE_PIN 35

//Extruder stepper connection
#define E_STEP_PIN 28
#define E_DIR_PIN 36
#define E_ENABLE_PIN 37

//Accessory connection
#define EXTRUDER_PIN 6
#define BED_PIN 7
#define FAN_PIN 11
#define SD_CS_PIN 8

//MAX6675 connection
#define SCK_E_PIN 2
#define SO_E_PIN 3
#define SCK_B_PIN 4
#define SO_B_PIN 5
#define CS_E_PIN 22
#define CS_B_PIN 23

//LCD connection
#define LCD_LED_PIN 9
#define LCD_RS 48
#define LCD_EN 49
#define LCD_D4 50
#define LCD_D5 51
#define LCD_D6 52
#define LCD_D7 53

//Keyboard connection
#define KeyU_PIN A5
#define KeyD_PIN A3
#define KeyR_PIN A7
#define KeyL_PIN A2
#define KeyOK_PIN A4
#define KeyB_PIN A1
#define KeyEM_PIN A6
#define KeyGND A0

struct LongPt {
  long x;
  long y;
  long z;
  long e;
};

struct FloatPt {
  double x;
  double y;
  double z;
  double e;
};

struct datastr {
  char st[200];
  int leng;
  long start;
};

//Float point selector
int Add(int d);
void Sub(int d);
int MoveL(int d);
int MoveR(int d);

//Long integer selector
int LAdd(int d);
void LSub(int d);
int LMoveL(int d);
int LMoveR(int d);


void ShowProgress(int x, int y, int bar_len, double progress);
int ListSD();
void ClearKey();
void Decode(char instruction[], int length);
double FindData(char keyword, char instruction[], int strlength);
boolean FindCommand(char keyword, char instruction[], int strlength);
long CalFeedrateDelay(double feedrate);
long CalMaxSpeed();
void Move(long micro_delay);
boolean TestMove(byte MinPin, byte MaxPin, long currentP, long targetP, byte directionP);
void MoveStep(byte pinA, byte pinB, byte dir);
void StopSteppers();
void CalDelta();
void SETtarget(double x, double y, double z, double e);
void SETposition(double x, double y, double z, double e);
boolean TestKey(int KeyPin);
void EM();
void Initialize();

FloatPt current;
FloatPt target;
FloatPt delta;

LongPt current_steps;
LongPt target_steps;
LongPt delta_steps;

FloatPt resume;

//Main buffer
datastr membuffer[2][BUFFER_SIZE];

uint8_t ArrowR[8] = {
  B01000,
  B01100,
  B01110,
  B01111,
  B01110,
  B01100,
  B01000,
};

uint8_t ArrowL[8] = {
  B00010,
  B00110,
  B01110,
  B11110,
  B01110,
  B00110,
  B00010,
};

uint8_t ArrowU[8] = {
  B00100,
  B01110,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
};

uint8_t Black[5][8] = {
  {
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
  }, {
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
  },
  {
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
  },
  {
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
  },
  {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
  }
};
uint8_t bar[8] = {
  B00000,
  B10000,
  B01000,
  B00100,
  B00010,
  B00001,
  B00000,
};
long timer;
//Stepper direction
boolean x_direction = 1;
boolean y_direction = 1;
boolean z_direction = 1;
boolean e_direction = 1;
//Axes units
double x_unit = X_STEPS_PER_MM;
double y_unit = Y_STEPS_PER_MM;
double z_unit = Z_STEPS_PER_MM;
double e_unit = E_STEPS_PER_MM;
double curve_section = CURVE_SECTION_MM;
//Test Register
volatile boolean TestXMAXPos, TestYMAXPos, TestZMAXPos;
volatile boolean TestXMINPos, TestYMINPos, TestZMINPos;
//Feedrate calculation
double feedrate = 0.0;
long feedrate_micros = 0;
//G-code absolute mode
boolean abs_mode = true;
//Accessory arguments
byte extruder_pwr = 0;
byte bed_pwr = 0;
int fan_speed = 0;
int extruder_temp = 0;
int bed_temp = 0;
//counters
int num = 0;
int i = 0;
int j = 0;
int n = 0;
int flag = 0;
//SD driver arguments
boolean IsSD = false;
int filemax = 0;
char list[100][20];
//File system arguments
File dataFile;
char data[200];
int datalength;
int datanum = 0;
char filename[100];
long filesize = 1;
long fileposition = 0;
//Serial report arguments
int report_delay = 0;
//Service switches
int print_switch = 0;
int command_switch = 0;
int buffer_switch = 0;
//Ping-pong buffer arguments
int bufferlength[2] = {0, 0};
int bufferstartposition[2] = {0, 0};
int buffernum = 0;
int printi = 0;
long stopposition, stopline;
//Status register
boolean decoding = false;
boolean extruder_ok = false;
boolean bed_ok = false;
boolean timeok = true;
boolean moving = false;
//Number selector
long Dec[10] = {
  1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};
int digit = 0;
int P[10] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
//LCD menu stuff
int page = 0;
int firstrow = 0;
int CursorR = 0;
boolean update = false;
int refresh = -1;
int brightness = 200;
boolean EMS = false;
byte pic = 0;
//Keyboard status
boolean KeyU, KeyD, KeyR, KeyL, KeyOK, KeyB;
//Menu content
char menu0[PAGE_0_MAX][20] = {"Print", "Resume Printing", "SD Info", "SD Refresh", "About"};
char menu1[PAGE_1_MAX][20] = {"Select File", "Set File Offset", "Set X-axis", "Set Y-axis", "Set Z-axis", "Set E-pos", "Set Feedrate", "Set Bed Temp", "Set Extruder Temp", "Set Fan Speed"};
//Heating bed PID settings
double bed_input, bed_output, bed_set;
double bed_aggKp = 4, bed_aggKi = 0.2, bed_aggKd = 1.0;
double bed_consKp = 1, bed_consKi = 0.05, bed_consKd = 0.25;
//Extruder PID settings
double extruder_input, extruder_output, extruder_set;
double extruder_aggKp = 50, extruder_aggKi = 0.2, extruder_aggKd = 1.0;
double extruder_consKp = 15, extruder_consKi = 10, extruder_consKd = 0.2;
//Device instance
MAX6675 get_extruder_temp(CS_E_PIN, SO_E_PIN, SCK_E_PIN, 1);
MAX6675 get_bed_temp(CS_B_PIN, SO_B_PIN, SCK_B_PIN, 1);
LiquidCrystal LCD(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
PID bed_ctrl(&bed_input, &bed_output, &bed_set, bed_consKp, bed_consKi, bed_consKd, DIRECT);
PID extruder_ctrl(&extruder_input, &extruder_output, &extruder_set, extruder_consKp, extruder_consKi, extruder_consKd, DIRECT);


//setup
void setup() {
  pinMode(KeyGND, OUTPUT);
  digitalWrite(KeyGND, LOW);
  SerialUSB.begin(115200);
  long ti = millis();
  //Waiting for Arduino Due SerialUSB
  while ((!SerialUSB) && ((millis() - ti) < SERIAL_TIMEOUT))
  {
  }
  //Initialize filename
  for (n = 0; n < 100; n++)
  {
    filename[n] = 0;
  }
  //Initialize data
  for (n = 0; n < 200; n++)
  {
    data[n] = 0;
  }
  //Initialize LCD
  LCD.begin(20, 4);
  LCD.createChar(0, ArrowR);
  LCD.createChar(1, ArrowU);
  for (n = 0; n < 5; n++)
    LCD.createChar(n + 2, Black[n]);
  LCD.createChar(7, bar);
  LCD.setCursor(0, 0);
  analogWrite(LCD_LED_PIN, brightness);
  LCD.print("AOI 3D Printer");
  LCD.setCursor(0, 1);
  LCD.print("Initializing...");
  //Initialize accessory pins
  pinMode(FAN_PIN, OUTPUT);
  pinMode(BED_PIN, OUTPUT);
  pinMode(EXTRUDER_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(BED_PIN, LOW);
  digitalWrite(EXTRUDER_PIN, LOW);
  //Initialize PID arguments for heating bed
  bed_set = 100;
  bed_ctrl.SetOutputLimits(0, 255);
  bed_ctrl.SetMode(AUTOMATIC);
  //Initialize PID arguments for extruder
  extruder_set = 100;
  extruder_ctrl.SetOutputLimits(0, 255);
  extruder_ctrl.SetMode(AUTOMATIC);
  //Initialize Steppers
  StopSteppers();
  current.x = 0.0;
  current.y = 0.0;
  current.z = 0.0;
  target.x = 0.0;
  target.y = 0.0;
  target.z = 0.0;
  x_unit = X_STEPS_PER_MM;
  y_unit = Y_STEPS_PER_MM;
  z_unit = Z_STEPS_PER_MM;
  e_unit = E_STEPS_PER_MM;
  CalDelta();
  //Pin direction declaration
  pinMode(13, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);

  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  pinMode(E_ENABLE_PIN, OUTPUT);

  pinMode(KeyU_PIN, INPUT_PULLUP);
  pinMode(KeyD_PIN, INPUT_PULLUP);
  pinMode(KeyR_PIN, INPUT_PULLUP);
  pinMode(KeyL_PIN, INPUT_PULLUP);
  pinMode(KeyOK_PIN, INPUT_PULLUP);
  pinMode(KeyB_PIN, INPUT_PULLUP);
  pinMode(KeyEM_PIN, INPUT_PULLUP);
  //Initialize feedrate
  feedrate = MAX_FEEDRATE;
  //Attaching interrupts
  attachInterrupt(X_MAX_PIN, StopXMAX, FALLING);
  attachInterrupt(X_MIN_PIN, StopXMIN, FALLING);
  attachInterrupt(Y_MAX_PIN, StopYMAX, FALLING);
  attachInterrupt(Y_MIN_PIN, StopYMIN, FALLING);
  attachInterrupt(Z_MAX_PIN, StopZMAX, FALLING);
  attachInterrupt(Z_MIN_PIN, StopZMIN, FALLING);

  attachInterrupt(KeyU_PIN, KeyUPressed, FALLING);
  attachInterrupt(KeyD_PIN, KeyDPressed, FALLING);
  attachInterrupt(KeyR_PIN, KeyRPressed, FALLING);
  attachInterrupt(KeyL_PIN, KeyLPressed, FALLING);
  attachInterrupt(KeyOK_PIN, KeyOKPressed, FALLING);
  attachInterrupt(KeyB_PIN, KeyBPressed, FALLING);
  attachInterrupt(KeyEM_PIN, KeyEMPressed, FALLING);

  TestXMAXPos = true;
  TestYMAXPos = true;
  TestZMAXPos = true;
  TestXMINPos = true;
  TestYMINPos = true;
  TestZMINPos = true;
  //Starting tasks
  Scheduler.startLoop(SerialCLI);
  Scheduler.startLoop(TempControl);
  Scheduler.startLoop(SerialUSBReport);
  Scheduler.startLoop(SDtoMEM);
  Scheduler.startLoop(Print);
  Scheduler.startLoop(LCDTimer);
  Scheduler.startLoop(LCDUpdate);
  //Serial report & LCD UI
  SerialUSB.println("SYSTEM INITIALIZED");
  LCD.setCursor(0, 2);
  LCD.print("System Initialized");
  delay(1000);
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Waiting for SD card");
  SerialUSB.println("WAITING FOR SD CARD");
  if (!SD.begin(SD_CS_PIN)) {
    SerialUSB.println("SD CARD NOT FOUND");
    LCD.setCursor(0, 1);
    LCD.print("SD card not found");
    IsSD = false;
  }
  else {
    SerialUSB.println("SD CARD CONNECTED");
    LCD.setCursor(0, 1);
    LCD.print("SD card connected");
    IsSD = true;
  }
  delay(1000);
  SerialUSB.print("SERIAL REPORT INTERVAL SET TO ");
  SerialUSB.print(report_delay);
  SerialUSB.println("s");
  SerialUSB.println("'$'--COMMAND\n'P'--START PRINTING\n'R'--RESUME\n'S'--STOP\n'F'--FILE\n'I'--REPORT INTERVAL\n'O'--FILE OFFSET\n'L'--LIST SD\n'W'--WRITE TO FILE\n");
  update = true;
  refresh = -1;
  page = 0;
}

void loop() {
  switch (page) {
    case 0:  // Main Menu
      {
        if (KeyU)
        {

          if (CursorR == 0)
          {
            if (firstrow > 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              if (PAGE_0_MAX >= 4)
              {
                firstrow = PAGE_0_MAX - 4;
                CursorR = 3;
              }
              else
              {
                firstrow = 0;
                CursorR = PAGE_0_MAX - 1;
              }
              update = true;
            }
          }
          else
          {
            CursorR--;
            update = true;
          }
          ClearKey();
        }
        if (KeyD)
        {

          if (CursorR == 3)
          {
            if (firstrow < PAGE_0_MAX - 4)
            {
              firstrow++;
              update = true;
            }
            else
            {
              firstrow = 0;
              CursorR = 0;
              update = true;
            }
          }
          else
          {
            CursorR++;
            update = true;
          }

          ClearKey();
        }
        if (KeyR)
        {
          filemax = ListSD();
          int select = firstrow + CursorR;
          page = select + 1;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyL)
        {
          ClearKey();
        }
        if (KeyOK)
        {
          filemax = ListSD();
          int select = firstrow + CursorR;
          page = select + 1;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyB)
        {
          ClearKey();
        }
      }
      break;


    case 1: //Print Select File
      {
        if (KeyU)
        {
          if (CursorR == 0)
          {
            if (firstrow > 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              if (filemax >= 4)
              {
                firstrow = filemax - 4;
                CursorR = 3;
              }
              else
              {
                firstrow = 0;
                CursorR = filemax - 1;
              }
              update = true;
            }

          }
          else
          {
            CursorR--;
            update = true;
          }

          ClearKey();
        }
        if (KeyD)
        {

          if (CursorR == 3)
          {
            if (firstrow < filemax - 4)
            {
              firstrow++;
              update = true;
            }
            else
            {
              firstrow = 0;
              CursorR = 0;
              update = true;
            }
          }
          else
          {
            CursorR++;
            update = true;
          }
          ClearKey();
        }
        if (KeyR)
        {
          int select = firstrow + CursorR;
          page = select + 1;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyL)
        {
          page--;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyOK)
        {
          LCD.clear();
          LCD.setCursor(0, 0);
          LCD.print("Initializing");
          LCD.setCursor(0, 1);
          LCD.print("Calibrate Extruder");
          LCD.setCursor(0, 3);
          LCD.print("Please wait");
          Initialize();
          int select = firstrow + CursorR;
          page = 9;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          SerialUSB.print("Printing File: ");
          for (n = 0; (list[select][n] != '\0') && (list[select][n] != '\n'); n++)
          {
            filename[n] = list[select][n];
            SerialUSB.print(filename[n]);
          }
          filename[n] = '\0';
          SerialUSB.println();
          SerialUSB.print("Start");
          buffer_switch = 1;
          moving = false;
          timeok = true;
          timer = 0;
          refresh = 8;
          update = true;
        }
        if (KeyB)
        {
          firstrow = 0;
          CursorR = 0;
          page = 0;
          update = true;
          ClearKey();
        }
      }
      break;

    case 2:  //Resume Print Menu menu1
      {
        if (KeyU)
        {

          if (CursorR == 0)
          {
            if (firstrow > 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              if (PAGE_1_MAX >= 4)
              {
                firstrow = PAGE_1_MAX - 4;
                CursorR = 3;
              }
              else
              {
                firstrow = 0;
                CursorR = PAGE_1_MAX - 1;
              }
              update = true;
            }
          }
          else
          {
            CursorR--;
            update = true;
          }
          ClearKey();
        }
        if (KeyD)
        {

          if (CursorR == 3)
          {
            if (firstrow < PAGE_1_MAX - 4)
            {
              firstrow++;
              update = true;
            }
            else
            {
              firstrow = 0;
              CursorR = 0;
              update = true;
            }
          }
          else
          {
            CursorR++;
            update = true;
          }
          ClearKey();
        }
        if (KeyR)
        {
          int select = firstrow + CursorR;
          page = select + 20;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyL)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyOK)
        {
          int select = firstrow + CursorR;
          page = select + 20;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyB)
        {
          firstrow = 0;
          CursorR = 0;
          page = 0;
          update = true;
          ClearKey();
        }
      }
      break;


    case 3: //SD card Info
      {
        if (KeyU)
        {
          ClearKey();
        }
        if (KeyD)
        {
          ClearKey();
        }
        if (KeyR)
        {
          ClearKey();
        }
        if (KeyL)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyOK)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyB)
        {
          firstrow = 0;
          CursorR = 0;
          page = 0;
          update = true;
          ClearKey();
        }
      }
      break;

    case 4: //Refresh SD
      {
        if (KeyU)
        {
          ClearKey();
        }
        if (KeyD)
        {
          ClearKey();
        }
        if (KeyR)
        {
          ClearKey();
        }
        if (KeyL)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyOK)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyB)
        {
          firstrow = 0;
          CursorR = 0;
          page = 0;
          update = true;
          ClearKey();
        }
      }
      break;

    case 5: //About Page
      {
        if (KeyU)
        {
          ClearKey();
        }
        if (KeyD)
        {
          ClearKey();
        }
        if (KeyR)
        {
          ClearKey();
        }
        if (KeyL)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyOK)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyB)
        {
          firstrow = 0;
          CursorR = 0;
          page = 0;
          update = true;
          ClearKey();
        }
      }
      break;

    case 8://Now printing
      {
        if (KeyU)
        {
          if (brightness < 240)
            brightness += 10;
          update = true;
          ClearKey();
        }
        if (KeyD)
        {
          if (brightness > 10)
            brightness -= 10;
          update = true;
          ClearKey();
        }
        if (KeyR)
        {
          page = 9;
          refresh = 9;
          update = true;
          ClearKey();
        }
        if (KeyL)
        {
          ClearKey();
        }
        if (KeyOK)
        {
          page = 9;
          refresh = 9;
          update = true;
          ClearKey();
        }
        if (KeyB)
        {
          ClearKey();
        }
      }
      break;

    case 9://Sensor Value
      {
        if (KeyU)
        {
          if (brightness < 240)
            brightness += 10;
          update = true;
          ClearKey();
        }
        if (KeyD)
        {
          if (brightness > 10)
            brightness -= 10;
          update = true;
          ClearKey();
        }
        if (KeyR)
        {
          ClearKey();
        }
        if (KeyL)
        {
          page = 8;
          update = true;
          refresh = 8;
          ClearKey();
        }
        if (KeyOK)
        {
          page = 8;
          update = true;
          refresh = 8;
          ClearKey();
        }
        if (KeyB)
        {
          page = 8;
          update = true;
          refresh = 8;
          ClearKey();
        }
      }
      break;

    case 10://Stop Page
      {
        if (KeyU)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyD)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyR)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyL)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyOK)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyB)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
      }
      break;

    case 11://Finish Page
      {
        if (KeyU)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyD)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyR)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyL)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyOK)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
        if (KeyB)
        {
          page = 0;
          firstrow = 0;
          CursorR = 0;
          update = true;
          ClearKey();
        }
      }
      break;


    case 20://Resume Print select file
      {
        if (KeyU)
        {

          if (CursorR == 0)
          {
            if (firstrow > 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              if (filemax >= 4)
              {
                firstrow = filemax - 4;
                CursorR = 3;
              }
              else
              {
                firstrow = 0;
                CursorR = filemax - 1;
              }
              update = true;
            }
          }
          else
          {
            CursorR--;
            update = true;
          }

          ClearKey();
        }
        if (KeyD)
        {

          if (CursorR == 3)
          {
            if (firstrow < filemax - 4)
            {
              firstrow++;
              update = true;
            }
            else
            {
              firstrow = 0;
              CursorR = 0;
              update = true;
            }
          }
          else
          {
            CursorR++;
            update = true;
          }

          ClearKey();
        }
        if (KeyR)
        {
          int select = firstrow + CursorR;
          page = select + 1;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyL)
        {
          firstrow = 0;
          CursorR = 0;
          page = 2;
          update = true;
          ClearKey();
        }
        if (KeyOK)
        {
          int select = firstrow + CursorR;
          page++;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          SerialUSB.print("Set file path to: ");
          for (n = 0; list[select][n] != '\0'; n++)
          {
            filename[n] = list[select][n];
            SerialUSB.print(filename[n]);
          }
          filename[n] = '\0';
          SerialUSB.println();
          update = true;

        }
      }
      if (KeyB)
      {
        firstrow = 0;
        CursorR = 0;
        page = 2;
        update = true;
        ClearKey();
      }
      break;

    case 21://Resume Print select point

      {
        if (KeyU)
        {
          LAdd(digit);
          ClearKey();
        }
        if (KeyD)
        {
          LSub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = LMoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = LMoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          //set position
          SerialUSB.println("OK");
          fileposition = 0;
          for (n = 0; n < 10; n++)
          {
            fileposition += P[n] * Dec[n];
          }
          SerialUSB.println(fileposition);
          dataFile = SD.open(filename);
          filesize = dataFile.size();
          dataFile.close();
          if (fileposition <= filesize)
          {
            page++;
            for (n = 0; n < 10; n++)
              P[n] = 0;
            update = true;
          }
          else {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds file size");
          }
          ClearKey();
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 22://Resume Print select X

      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          resume.x = 0;
          for (n = 0; n < 5; n++)
          {
            resume.x += P[n] * Dec[n];
          }
          SerialUSB.println(resume.x);
          resume.x /= 100000;
          for (n = 0; n < 5; n++)
          {
            resume.x += P[n + 5] * Dec[n];
          }
          SerialUSB.println(resume.x);
          if (resume.x < 300)
          {
            for (n = 0; n < 10; n++)
              P[n] = 0;
            page++;
            update = true;
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds X range");
          }
          ClearKey();
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }


      break;

    case 23://Resume Print select Y
      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          resume.y = 0;
          for (n = 0; n < 5; n++)
          {
            resume.y += P[n] * Dec[n];
          }
          resume.y /= 100000;
          for (n = 0; n < 5; n++)
          {
            resume.y += P[n + 5] * Dec[n];
          }
          if (resume.y < 300)
          {
            for (n = 0; n < 10; n++)
              P[n] = 0;
            page++;
            update = true;
            ClearKey();
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds Y range");
          }
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 24://Resume Print select Z
      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          resume.z = 0;
          for (n = 0; n < 5; n++)
          {
            resume.z += P[n] * Dec[n];
          }
          resume.z /= 100000;
          for (n = 0; n < 5; n++)
          {
            resume.z += P[n + 5] * Dec[n];
          }
          if (resume.z < 300)
          {
            for (n = 0; n < 10; n++)
              P[n] = 0;
            page++;
            update = true;
            ClearKey();
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds Z range");
          }
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;



    case 25://Resume Print select E
      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          resume.e = 0;
          for (n = 0; n < 5; n++)
          {
            resume.e += P[n] * Dec[n];
          }
          resume.e /= 100000;
          for (n = 0; n < 5; n++)
          {
            resume.e += P[n + 5] * Dec[n];
          }
          current.e = resume.e;
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page++;
          update = true;
          ClearKey();
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 26://Resume Print select F
      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          feedrate = 0;
          for (n = 0; n < 5; n++)
          {
            feedrate += P[n] * Dec[n];
          }
          feedrate /= 100000;
          for (n = 0; n < 5; n++)
          {
            feedrate += P[n + 5] * Dec[n];
          }
          if (feedrate < 3200)
          {
            for (n = 0; n < 10; n++)
              P[n] = 0;
            page++;
            update = true;
            ClearKey();
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds F limit");
          }
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 27://Resume Print select B bed
      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          bed_temp = 0;
          for (n = 0; n < 5; n++)
          {
            bed_temp += P[n] * Dec[n];
          }
          bed_temp /= 100000;
          for (n = 0; n < 5; n++)
          {
            bed_temp += P[n + 5] * Dec[n];
          }
          if (bed_temp < 200)
          {
            for (n = 0; n < 10; n++)
              P[n] = 0;
            page++;
            update = true;
            ClearKey();
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds temp limit");
          }
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 28://Resume Print select H extruder
      {
        if (KeyU)
        {
          Add(digit);
          ClearKey();
        }
        if (KeyD)
        {
          Sub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = MoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = MoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          extruder_temp = 0;
          for (n = 0; n < 5; n++)
          {
            extruder_temp += P[n] * Dec[n];
          }
          extruder_temp /= 100000;
          for (n = 0; n < 5; n++)
          {
            extruder_temp += P[n + 5] * Dec[n];
          }
          if (extruder_temp < 300)
          {
            for (n = 0; n < 10; n++)
              P[n] = 0;
            page++;
            update = true;
            ClearKey();
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds temp range");
          }
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 29://Resume Print select C fan_speed
      {
        if (KeyU)
        {
          LAdd(digit);
          ClearKey();
        }
        if (KeyD)
        {
          LSub(digit);
          ClearKey();
        }
        if (KeyR)
        {
          digit = LMoveR(digit);
          ClearKey();
        }
        if (KeyL)
        {
          digit = LMoveL(digit);
          ClearKey();
        }
        if (KeyOK)
        {
          fan_speed = 0;
          for (n = 0; n < 10; n++)
          {
            fan_speed += P[n] * Dec[n];
          }
          if (fan_speed <= 255)
          {
            int select = firstrow + CursorR;
            page = 9;
            firstrow = 0;
            CursorR = 0;
            ClearKey();
            SerialUSB.print("Printing File: ");
            LCD.clear();
            LCD.setCursor(0, 0);
            LCD.print("Preparing to Resume");
            LCD.setCursor(0, 1);
            LCD.print("File: ");
            LCD.print(filename);
            for (n = 0; (filename[n] != '\0') && (filename[n] != '\n'); n++)
            {
              SerialUSB.print(filename[n]);
            }
            SerialUSB.println();
            for (n = 0; n < 10; n++)
              P[n] = 0;
            SerialUSB.print("Start");
            LCD.setCursor(0, 3);
            LCD.print("Calibrate Extruder");
            command_switch = 1;
            Decode("G0 X-100 Y-100 Z300", 18);
            command_switch = 1;
            Decode("G0 Z-100", 9);
            SETtarget(0, 0, 0, 0);
            SETposition(0, 0, 0, 0);
            command_switch = 1;
            Decode("G0 Z300", 18);
            target.x = resume.x;
            target.y = resume.y;
            target.z = resume.z;

            target.e = resume.e;
            current.e = target.e;

            if (bed_temp == 0)
              bed_pwr = 0;
            else
            {
              bed_pwr = 1;
              SerialUSB.println("Preheating Bed");
              LCD.setCursor(0, 3);
              LCD.print("Preheating Bed      ");
              long timeout = millis();
              while ((!bed_ok) && ((millis() - timeout) < PREHEATING_TIMEOUT))
              {
                yield();
              }
            }
            if (extruder_temp == 0)
              extruder_pwr = 0;
            else {
              extruder_pwr = 1;
              SerialUSB.println("Preheating Extruder");
              LCD.setCursor(0, 3);
              LCD.print("Preheating Extruder ");
              long timeout = millis();
              while ((!extruder_ok) && ((millis() - timeout) < PREHEATING_TIMEOUT))
              {
                yield();
              }
            }
            analogWrite(FAN_PIN, fan_speed);
            CalDelta();
            LCD.setCursor(0, 3);
            LCD.print("Extruder Approching");
            Move(60000000 / x_unit / MAX_FEEDRATE);
            buffer_switch = 1;
            moving = false;
            timeok = true;
            timer = 0;

            refresh = 8;
            update = true;
          }
          else
          {
            LCD.setCursor(0, 3);
            LCD.print("!Exceeds speed limit");
          }

          ClearKey();
        }
        if (KeyB)
        {
          for (n = 0; n < 10; n++)
            P[n] = 0;
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    default:
      {
      }
      break;
  }
  yield();
}

void LCDUpdate()
{
  if (update)
  {
    analogWrite(LCD_LED_PIN, brightness);
    LCD.clear();
    switch (page) {
      case 0 :
        {
          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(byte(0));
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(1, n - firstrow);
            if (n == (CursorR + firstrow))
              SerialUSB.print(">");
            SerialUSB.println(menu0[n]);
            LCD.print(menu0[n]);
          }
        }
        break;
      case 1:
        {

          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(byte(0));
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(1, n - firstrow);
            if (n == (CursorR + firstrow))
              SerialUSB.print(">");
            SerialUSB.println(list[n]);
            LCD.print(list[n]);
          }
        }
        break;

      case 2:
        {

          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(byte(0));
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(1, n - firstrow);
            LCD.print(menu1[n]);
            if (n == (CursorR + firstrow))
              SerialUSB.print(">");
            SerialUSB.println(menu1[n]);
          }
        }
        break;
      case 3:
        {
          refresh = -1;
          Sd2Card card;
          SdVolume volume;
          SdFile root;
          if (card.init(SPI_HALF_SPEED, SD_CS_PIN))
          {
            LCD.setCursor(0, 0);
            LCD.print("Card Type: ");
            switch (card.type()) {
              case SD_CARD_TYPE_SD1:
                LCD.print("SD1");
                break;
              case SD_CARD_TYPE_SD2:
                LCD.print("SD2");
                break;
              case SD_CARD_TYPE_SDHC:
                LCD.print("SDHC");
                break;
              default:
                Serial.print("Unknown");
            }
            if (volume.init(card))
            {
              LCD.setCursor(0, 1);
              LCD.print("Format: FAT");
              LCD.print(volume.fatType(), DEC);
              LCD.setCursor(0, 2);
              LCD.print("Size: ");
              uint32_t volumesize;
              volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
              volumesize *= volume.clusterCount();       // we'll have a lot of clusters
              volumesize /= 2048;
              LCD.print(volumesize);
              LCD.print(" MB");
            }
            else
            {
              LCD.setCursor(0, 1);
              LCD.print("No valid volume");
            }
            SD.begin(SD_CS_PIN);
          }
          else
          {
            LCD.setCursor(0, 0);
            LCD.print("SD card not found");
          }
          LCD.setCursor(0, 3);
          LCD.print("Press LEFT key");
        }
        break;

      case 4:
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Waiting for SD card");
          Sd2Card card;
          if (!card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
            SerialUSB.println("SD card not found");
            LCD.setCursor(0, 1);
            LCD.print("SD card not found");
            IsSD = false;
          }
          else {
            SD.begin(SD_CS_PIN);
            SerialUSB.println("SD card connected");
            LCD.setCursor(0, 1);
            LCD.print("SD card connected");
            IsSD = true;
          }
          LCD.setCursor(0, 3);
          LCD.print("Press LEFT key");
        }
        break;
      case 5: //About Page
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Elec: Yicheng Bao");
          LCD.setCursor(0, 1);
          LCD.print("Mech: Yiping Zheng");
          LCD.setCursor(0, 2);
          LCD.print("Prog: Yicheng Bao");
          LCD.setCursor(0, 3);
          LCD.print("Material: Lile Xie");
        }
        break;


      case 8://Now printing
        {
          refresh = 8;
          LCD.setCursor(0, 0);
          LCD.print("Now Printing ");
          LCD.setCursor(0, 1);
          LCD.print("File= ");
          LCD.print(filename);
          double report = (double)(fileposition / filesize);
          ShowProgress(0, 3, 15, report);
          LCD.setCursor(16, 3);
          LCD.print((int)(report * 100));
          LCD.print("%");
        }
        break;

      case 9://Sensor Value
        {
          refresh = 9;
          LCD.setCursor(0, 0);
          char stemp[20];
          sprintf(stemp, "Ext % 3.0lf C -> % 3.0lf C", extruder_input, extruder_temp);
          LCD.print(stemp);
          LCD.setCursor(0, 1);
          sprintf(stemp, "Bed % 3.0lf C -> % 3.0lf C", bed_input, bed_temp);
          LCD.print(stemp);
          LCD.setCursor(0, 2);
          sprintf(stemp, "Fan %d", fan_speed);
          LCD.print(stemp);
          LCD.setCursor(0, 3);
          int xtemp, ytemp, ztemp;
          sprintf(stemp, "X=% 3d Y=% 3d Z=% 3d", xtemp, ytemp, ztemp);
          LCD.print(stemp);
        }
        break;

      case 10://Stop Page
        {
          bed_pwr = 0;
          extruder_pwr = 0;
          fan_speed = 0;
          digitalWrite(BED_PIN, LOW);
          digitalWrite(EXTRUDER_PIN, LOW);
          digitalWrite(FAN_PIN, LOW);
          refresh = -1;
          yield();
          LCD.setCursor(0, 0);
          EMS = false;
          LCD.print("STOP Pos= ");
          LCD.print(stopposition);
          LCD.setCursor(0, 1);
          LCD.print("No=");
          LCD.print(stopline);
          char stemp[30];
          LCD.setCursor(9, 1);
          sprintf(stemp, "T=B%03d E%03d", bed_temp, extruder_temp);
          LCD.print(stemp);
          LCD.setCursor(0, 2);
          sprintf(stemp, "X%06.2lf Y%06.2lf F%3d", current.x, current.y, int(feedrate) / 10);
          LCD.print(stemp);
          LCD.setCursor(0, 3);
          sprintf(stemp, "Z%06.2lf E%06.1lf C%03d", current.z, current.e, int(fan_speed));
          LCD.print(stemp);
          SerialUSB.println("Stopped");
        }
        break;

      case 11://Finish Page
        {
          bed_pwr = 0;
          extruder_pwr = 0;
          fan_speed = 0;
          digitalWrite(BED_PIN, LOW);
          digitalWrite(EXTRUDER_PIN, LOW);
          digitalWrite(FAN_PIN, LOW);
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Congratulation!");
          LCD.setCursor(0, 1);
          LCD.print("Printing finished.");

          char stemp[22];
          long ltemp = timer / 1000;
          sprintf(stemp, "Time= %3dh %2dm %2ds", int(ltemp / 3600), int((ltemp % 3600) / 60), int(ltemp % 60));
          LCD.setCursor(0, 2);
          LCD.print(stemp);
          LCD.setCursor(0, 3);
          LCD.print("Press any key");
        }
        break;

      case 20://Resume Print select file
        {

          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(byte(0));
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(1, n - firstrow);
            if (n == (CursorR + firstrow))
              SerialUSB.print(">");
            SerialUSB.println(list[n]);
            LCD.print(list[n]);
          }
        }
        break;

      case 21://Resume Print select point
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Position (Pos)");
          LCD.setCursor(5, 1);
          long tmp = fileposition;
          for (n = 0; n < 10; n++)
          {
            P[n] = tmp % 10;
            tmp /= 10;
          }
          for (n = 0; n < 10; n++)
          {
            LCD.print(P[9 - n]);
            SerialUSB.print(P[9 - n]);
          }
          SerialUSB.println();
          LCD.setCursor(14, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 22://Resume Print select X
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set X Position (X)");
          LCD.setCursor(5, 1);
          long ltemp = resume.x * 100000 ;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 23://Resume Print select Y
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Y Position (Y)");
          LCD.setCursor(5, 1);
          long ltemp = resume.y * 100000;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 24://Resume Print select Z
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Z Position (Z)");
          LCD.setCursor(5, 1);
          long ltemp = resume.z * 100000;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 25://Resume Print select E
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set E Position (E)");
          LCD.setCursor(5, 1);
          long ltemp = resume.e * 100000;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;
      case 26://Resume Print select F
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Feedrate (F)");
          LCD.setCursor(5, 1);
          long ltemp = feedrate * 100000;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 27://Resume Print select B
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Bed Temp (T=B)");
          LCD.setCursor(5, 1);
          long ltemp = bed_temp * 100000;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 28://Resume Print select H
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Extrdr Temp (T=E)");
          LCD.setCursor(5, 1);
          long ltemp = extruder_temp * 100000;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 5; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.print('.');
          for (n = 4; n < 9; n++)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(15, 2);
          digit = 0;
          LCD.write(1);
        }
        break;

      case 29://Resume Print select C
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Fan Speed (C)");
          LCD.setCursor(5, 1);
          long ltemp = fan_speed;
          SerialUSB.println(ltemp);
          for (n = 0; n < 10; n++)
          {
            P[n] = ltemp % 10;
            ltemp = ltemp / 10;
          }
          for (n = 0; n < 10; n++)
          {
            LCD.print(P[9 - n]);
            SerialUSB.print(P[9 - n]);
          }
          LCD.setCursor(14, 2);
          digit = 0;
          LCD.write(1);
        }
        break;


      default:
        {
        }
        break;
    }

  }
  update = false;
  yield();
}



void LCDTimer() {

  if ((refresh > 0) && (!((!timeok) && moving)))
  {
    switch (refresh)
    {
      case 8:
        {
          LCD.setCursor(13, 0);
          switch (pic)
          {
            case 0:
              {
                LCD.print("-   ");
                pic++;
              }
              break;
            case 1:
              {
                LCD.write(7);
                LCD.print(".  ");
                pic++;
              }
              break;
            case 2:
              {
                LCD.print("|.. ");
                pic++;
              }
              break;
            case 3:
              {
                LCD.print("/...");
                pic = 0;
              }
              break;
          }
          LCD.setCursor(0, 2);
          char stemp[22];
          long ltemp = (millis() - timer) / 1000;
          sprintf(stemp, "Up Time=%3dh %2dm %2ds", int(ltemp / 3600), int((ltemp % 3600) / 60), int(ltemp % 60));
          LCD.print(stemp);
          stopposition = bufferstartposition[buffernum] + printi;
          double report;
          if (printi == BUFFER_SIZE - 1)
            report = membuffer[1 - buffernum][printi].start;
          else
            report = membuffer[buffernum][printi].start;

          report = membuffer[buffernum][printi].start;
          report /= filesize;
          ShowProgress(0, 3, 15, report);
          LCD.setCursor(16, 3);
          LCD.print((int)(report * 100));
          LCD.print("%");
        }
        break;

      case 9:
        {
          LCD.clear();
          LCD.setCursor(0, 0);
          char stemp[22];
          char degree[2] = {0xdf, 0};
          sprintf(stemp, "Ext= % 3.0lf%sC -> %3d", extruder_input, degree, extruder_temp);
          LCD.print(stemp);
          LCD.write(0xdf);
          LCD.print("C");
          LCD.setCursor(0, 1);
          sprintf(stemp, "Bed= % 3.0lf%sC -> %3d", bed_input, degree, bed_temp);
          LCD.print(stemp);
          LCD.write(0xdf);
          LCD.print("C");
          LCD.setCursor(0, 2);
          sprintf(stemp, "Fan= % 3d E=%8.2f", fan_speed, current.e);
          LCD.print(stemp);
          LCD.setCursor(0, 3);
          int xtemp = current.x, ytemp = current.y, ztemp = current.z;
          sprintf(stemp, "X=% 3d Y=% 3d Z= % 3d", xtemp, ytemp, ztemp);
          LCD.print(stemp);
        }
        break;
      default:
        {
        }
        break;
    }
    long LCDs = millis();
    while (millis() - LCDs < 500)
    {
      yield();
    }
  }
  yield();
}


void SerialCLI() {
  if (EN_SERIAL_CLI)
  {
    char c;
    while (SerialUSB.available()) {
      if (i == 0)
      {
        c = SerialUSB.read();
        i = 1;
      }
      else
      {
        data[i - 1] = SerialUSB.read();
        if (data[i - 1] == '\r' || data[i - 1] == '\n')
          flag = 1;
        i++;
      }
    }
    if (flag > 0)
    {
      switch (c) {
        case '0' :
          break;

        case 'P' :
          SerialUSB.print("Printing File: ");
          for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
          {
            SerialUSB.print(filename[n]);
          }
          SerialUSB.println();
          fileposition = 0;
          for (int cn = 0; cn < BUFFER_SIZE; cn++)
          {
            membuffer[0][cn].start = 0;
            membuffer[0][cn].leng = 0;
            for (int cst = 0; cst < 50; cst++)
              membuffer[0][cn].st[cst] = 0;
          }
          for (int cn = 0; n < BUFFER_SIZE; n++)
          {
            membuffer[1][cn].start = 0;
            membuffer[1][cn].leng = 0;
            for (int cst = 0; cst < 50; cst++)
              membuffer[1][cn].st[cst] = 0;
          }

          buffer_switch = 1;
          moving = false;
          timeok = true;
          break;

        case 'F' :
          SerialUSB.print("File path set to: ");
          for (n = 0; ((n < i - 1) && (data[n] != '\r') && (data[n] != '\n')); n++)
          {
            filename[n] = data[n];
          }
          filename[n] = 0;
          for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
          {
            SerialUSB.print(filename[n]);
          }

          break;

        case 'W' :
          dataFile = SD.open(filename, FILE_WRITE);
          if (dataFile) {
            dataFile.seek(fileposition);
            for (n = 0; n < i - 1; n++)
            {
              dataFile.write(data[n]);
            }
            dataFile.close();
          }
          else {
            SerialUSB.println("ERROR: Cannot open the file");
          }
          break;
        case 'R' :
          {
            double resume_e = strtod(data, NULL);
            command_switch = 1;
            Decode("G0 X-100 Y-100 Z300", 18);
            command_switch = 1;
            Decode("G0 Z-100", 9);
            SETtarget(0, 0, 0, 0);
            SETposition(0, 0, 0, 0);
            command_switch = 1;
            Decode("G0 X280 Y280 Z280", 18);
            if (FindData('X', data, i - 1) != 32767)
            {
              target.x = FindData('X', data, i - 1);
            }
            if (FindData('Y', data, i - 1) != 32767)
            {
              target.y = FindData('Y', data, i - 1);
            }
            if (FindData('Z', data, i - 1) != 32767)
            {
              target.z = FindData('Z', data, i - 1);
            }
            if (FindData('E', data, i - 1) != 32767)
            {
              target.e = FindData('E', data, i - 1);
              current.e = target.e;
            }
            if (FindData('F', data, i - 1) != 32767)
            {
              feedrate = FindData('F', data, i - 1);
            }
            if (FindData('B', data, i - 1) != 32767)
            {
              bed_temp = FindData('B', data, i - 1);
              if (bed_temp == 0)
                bed_pwr = 0;
              else
              {
                bed_pwr = 1;
                SerialUSB.println("Preheating Bed");
                long timeout = millis();
                while ((!bed_ok) && ((millis() - timeout) < PREHEATING_TIMEOUT))
                {
                  yield();
                }
              }
            }
            if (FindData('H', data, i - 1) != 32767)
            {
              extruder_temp = FindData('H', data, i - 1);
              if (extruder_temp == 0)
                extruder_pwr = 0;
              else {
                extruder_pwr = 1;
                SerialUSB.println("Preheating Extruder");
                long timeout = millis();
                while ((!extruder_ok) && ((millis() - timeout) < PREHEATING_TIMEOUT))
                {
                  yield();
                }
              }
            }
            if (FindData('C', data, i - 1) != 32767)
            {
              fan_speed = FindData('C', data, i - 1);
              analogWrite(FAN_PIN, fan_speed);
            }
            CalDelta();
            Move(60000000 / x_unit / MAX_FEEDRATE);
            dataFile = SD.open(filename);
            if (dataFile)
            {
              if (fileposition < dataFile.size())
              {
                SerialUSB.print("Resume printing process from ");
                SerialUSB.print(fileposition);
                SerialUSB.print(" at the file ");
                for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
                {
                  SerialUSB.print(filename[n]);
                }
                SerialUSB.println();

                dataFile.close();
                buffer_switch = 1;
                moving = false;
                timeok = true;
              }
              else {
                SerialUSB.println("ERROR: Resume position exceeds file size");
              }

            }
            else {
              SerialUSB.println("ERROR: Cannot open the file");
            }
          }
          break;
        case 'O' :
          {
            char strtemp[100];
            for (n = 0; n < i - 1; n++)
            {
              strtemp[n] = data[n];
            }
            fileposition = atoi(strtemp);
            SerialUSB.print("Set file offset to ");
            SerialUSB.println(fileposition);
          }
          break;
        case '$' :
          datalength = i - 2;
          SerialUSB.print("$");
          for (i = 0; i < datalength; i++)
          {
            SerialUSB.print(data[i]);
          }
          SerialUSB.println();
          command_switch = 1;
          Decode(data, datalength);
          command_switch = 0;
          SerialUSB.println("DONE");
          break;
        case 'I' :
          SerialUSB.print("Report Interval= ");
          report_delay = strtod(data, NULL);
          SerialUSB.print(report_delay);
          SerialUSB.println(" s");
          break;
        case 'L' :
          {
            SerialUSB.println("SD Card root directory:");
            filemax = ListSD();
            for (int fc = 0; fc < filemax; fc++)
              SerialUSB.println(list[fc]);
          }
          break;
        case 'H' :
          {
            SerialUSB.println("HELP MENU");
            SerialUSB.println("'$'--COMMAND\n'P'--START PRINTING\n'R'--RESUME\n'S'--STOP\n'F'--FILE\n'I'--REPORT INTERVAL\n'O'--FILE OFFSET\n'L'--LIST SD\n'W'--WRITE TO FILE\n");
          }
          break;
        case 'S' :
          {
            stopposition = membuffer[buffernum][printi].start;
            stopline = bufferstartposition[buffernum] + printi;
            buffer_switch = 0;
            print_switch = 0;
            buffer_switch = 0;
            command_switch = 0;
            moving = false;
            yield();
            dataFile.close();

            SerialUSB.print("Printing process is interrupted at ");
            SerialUSB.print(stopposition);
            SerialUSB.print("(Instruction No. ");
            SerialUSB.print(stopline);
            SerialUSB.print(") of the file '");
            for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
            {
              SerialUSB.print(filename[n]);
            }
            SerialUSB.println("'");

            SerialUSB.print("Resume Argument: ");
            SerialUSB.print("'X");
            SerialUSB.print(current.x);
            SerialUSB.print(" Y");
            SerialUSB.print(current.y);
            SerialUSB.print(" Z");
            SerialUSB.print(current.z);
            SerialUSB.print(" E");
            SerialUSB.print(current.e);
            SerialUSB.print(" F");
            SerialUSB.print(feedrate);
            SerialUSB.print(" B");
            SerialUSB.print(bed_temp);
            SerialUSB.print(" H");
            SerialUSB.print(extruder_temp);
            SerialUSB.print(" C");
            SerialUSB.print(fan_speed);
            SerialUSB.println("'");
            bed_pwr = 0;
            digitalWrite(BED_PIN, LOW);
            extruder_pwr = 0;
            digitalWrite(EXTRUDER_PIN, LOW);
            fan_speed = 0;
            digitalWrite(FAN_PIN, LOW);
            page = 10;
            firstrow = 0;
            CursorR = 0;
            update = true;

          }
          break;
      }
      flag = 0;
      i = 0;
      c = 0;
    }
  }
  yield();
}


void TempControl()
{
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  bed_input = (double) get_bed_temp.read_temp();
  if (bed_pwr == 1)
  {
    bed_set = bed_temp;
    double delta_temp = abs(bed_input - bed_set);
    if (delta_temp < 10)
    {
      bed_ok = true;
      bed_ctrl.SetTunings(bed_consKp, bed_consKi, bed_consKd);
    }
    else
    {
      bed_ok = false;
      bed_ctrl.SetTunings(bed_aggKp, bed_aggKi, bed_aggKd);
    }
    bed_ctrl.Compute();
    analogWrite(BED_PIN, bed_output);
  }
  extruder_input = (double) get_extruder_temp.read_temp();
  if (extruder_pwr == 1)
  {
    extruder_set = extruder_temp;
    double delta_temp = abs(extruder_input - extruder_set);
    if (delta_temp < 10)
    {
      extruder_ok = true;
      extruder_ctrl.SetTunings(extruder_consKp, extruder_consKi, extruder_consKd);
    }
    else
    {
      extruder_ok = false;
      extruder_ctrl.SetTunings(extruder_aggKp, extruder_aggKi, extruder_aggKd);
    }
    extruder_ctrl.Compute();
    analogWrite(EXTRUDER_PIN, extruder_output);
  }
  yield();
}

void Print()
{
  if (print_switch == 1)
  {

    SerialUSB.println("Printing start");
    for (printi = 0; (printi < bufferlength[buffernum]) && (print_switch == 1); printi++)
    {
      //SerialUSB.println(printi);
      Decode(membuffer[buffernum][printi].st, membuffer[buffernum][printi].leng);
      if (print_switch == 0)
        break;
    }
    printi--;
    print_switch = 0;
    buffernum = 1 - buffernum;
    if (buffer_switch == 0)
    {
      SerialUSB.println("Printing finished");
      buffernum = 1 - buffernum;
      membuffer[buffernum][printi].start = filesize;
    }

  }
  yield();
}

void SerialUSBReport()
{
  if (EN_SERIAL_REPORT)
  {
    if (report_delay)
    {
      double report;
      if (printi == BUFFER_SIZE - 1)
        report = membuffer[1 - buffernum][printi].start;
      else
        report = membuffer[buffernum][printi].start;
      report /= filesize;
      SerialUSB.print("[");
      for (n = 0; n < 60; n++)
      {
        if (n < (60 * report))
        {
          SerialUSB.print("=");
        }
        else
        {
          SerialUSB.print(" ");
        }
      }
      SerialUSB.print("] ");
      SerialUSB.print(report * 100);
      SerialUSB.print("%\nFile position= ");
      if (printi == BUFFER_SIZE - 1)
        SerialUSB.print(membuffer[1 - buffernum][printi].start);
      else
        SerialUSB.print(membuffer[buffernum][printi].start);

      SerialUSB.print(" | Instruction No.= ");
      SerialUSB.println(bufferstartposition[buffernum] + printi + 1);
      SerialUSB.println(printi);
      SerialUSB.println(buffernum);
      SerialUSB.println(membuffer[buffernum][printi].start);
      SerialUSB.print("Extruder Temp= ");
      SerialUSB.print(extruder_input);
      SerialUSB.print(" C / ");
      SerialUSB.print(extruder_temp);
      SerialUSB.print(" C | Bed Temp= ");
      SerialUSB.print(bed_input);
      SerialUSB.print(" C / ");
      SerialUSB.print(bed_temp);
      SerialUSB.print(" C | Fan Speed= ");
      SerialUSB.println(fan_speed);
      SerialUSB.println(timer);
      delay(report_delay * 1000);
    }
  }
  yield();
}

void SDtoMEM()
{
  if ((buffer_switch == 1) && (!decoding))
  {
    dataFile = SD.open(filename);
    if (dataFile)
    {
      dataFile.seek(fileposition);
      SerialUSB.print("Start Time:");
      timer = millis();
      SerialUSB.println(timer);
      SerialUSB.println("Buffering Data");
      buffernum = 1;
      char ch;
      filesize = dataFile.size();
      SerialUSB.println(filesize);
      int bufferposition = 0;
      bufferstartposition[1 - buffernum] = 0;
      ch = dataFile.read();
      if (BUF_INFO)
        SerialUSB.print(ch);
      fileposition++;

      int valid;

      while ((buffer_switch == 1) && (dataFile.available() > 0) && (bufferposition < BUFFER_SIZE) && (fileposition < (filesize)))
      {
        j = 0;
        valid = 0;
        while ((buffer_switch == 1) && (dataFile.available() > 0) && (ch != '\n') && (ch != ';') && (fileposition < (filesize)))
        {
          membuffer[1 - buffernum][bufferposition].st[j] = ch;
          j++;
          fileposition++;
          ch = dataFile.read();
          if (BUF_INFO)
            SerialUSB.print(ch);
        }
        membuffer[1 - buffernum][bufferposition].st[j] = 0;
        valid = j;
        if ((ch == ';'))
          while ((ch != '\n') && (fileposition < (filesize)))
          {
            ch = dataFile.read();
            fileposition++;
            j++;

          }
        membuffer[1 - buffernum][bufferposition].leng = valid + 1;
        //SerialUSB.println(membuffer[1 - buffernum][bufferposition].leng );
        membuffer[1 - buffernum][bufferposition].start = fileposition - j - 1;
        //SerialUSB.println(membuffer[1 - buffernum][bufferposition].start);
        bufferposition++;
        if ((fileposition < (filesize)))
          ch = dataFile.read();
        if (BUF_INFO)
          SerialUSB.print(ch);
        fileposition++;
      }
      bufferlength[1 - buffernum] = bufferposition;
      if (BUF_INFO)
        SerialUSB.println(fileposition);
      buffernum = 0;
      if (buffer_switch == 1)
        print_switch = 1;
      while ((buffer_switch == 1) && fileposition < filesize) {
        int bufferposition = 0;
        bufferstartposition[1 - buffernum] = bufferstartposition[buffernum] + BUFFER_SIZE;
        while ((buffer_switch == 1) && (dataFile.available() > 0)  && (bufferposition < BUFFER_SIZE) && (fileposition < (filesize)))
        {
          j = 0;
          valid = 0;
          while ((buffer_switch == 1) && (dataFile.available() > 0) && (ch != '\n') && (ch != ';') && (fileposition < (filesize)) && (j < 180))
          {
            membuffer[1 - buffernum][bufferposition].st[j] = ch;
            j++;
            fileposition++;
            ch = dataFile.read();
            if (BUF_INFO)
              SerialUSB.print(ch);
          }
          membuffer[1 - buffernum][bufferposition].st[j] = 0;
          valid = j;
          if (ch == ';')
            while ((ch != '\n') && (fileposition < (filesize)) && (j < 180))
            {
              ch = dataFile.read();
              fileposition++;
              j++;
            }
          membuffer[1 - buffernum][bufferposition].leng = valid + 1;
          //SerialUSB.println(membuffer[1 - buffernum][bufferposition].leng);
          membuffer[1 - buffernum][bufferposition].start = fileposition - j - 1;
          //SerialUSB.println(membuffer[1 - buffernum][bufferposition].start);
          bufferposition++;
          ch = dataFile.read();
          if (BUF_INFO)
            SerialUSB.print(ch);
          fileposition++;
        }
        bufferlength[1 - buffernum] = bufferposition;
        while ((print_switch == 1) && (buffer_switch == 1))
        {
          yield();
        }
        SerialUSB.println("Switch Buffer");
        if (buffer_switch == 1)
          print_switch = 1;
      }
      dataFile.close();
      SerialUSB.println("All Buffered");
      while ((print_switch == 1) && (buffer_switch == 1))
      {
        yield();
      }
      timer = millis() - timer;
      SerialUSB.print("Time use:");
      SerialUSB.println(timer);
      if (!EMS)
      {
        SerialUSB.println("Finish work");
        page = 11;
        update = true;
      }
    }
    else {
      SerialUSB.println("ERROR: Cannot open the file");
    }
    buffer_switch = 0;
  }
  yield();
}

int Add(int d)
{
  if (P[d] < 9) {
    if (d < 5)
      LCD.setCursor(15 - d, 1);
    else
      LCD.setCursor(14 - d, 1);
    if ((d == 9) && P[9] == 9) {
      return -1;
    }
    P[d]++;
    LCD.print(P[d]);
    return 1;
  }
  else {
    if (Add(d + 1) > 0)
    {
      P[d] = 0;
      if (d < 5)
        LCD.setCursor(15 - d, 1);
      else
        LCD.setCursor(14 - d, 1);
    }
    LCD.print(P[d]);
    return 1;
  }
}

void Sub(int d)
{
  if (P[d] > 0) {
    if (d < 5)
      LCD.setCursor(15 - d, 1);
    else
      LCD.setCursor(14 - d, 1);
    P[d]--;
    LCD.print(P[d]);
  }
}

int MoveR(int d)
{
  if (d != 0) {
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d--;
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  else {
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d = 9;
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  return d;
}

int MoveL(int d)
{
  if (d != 9) {
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d++;
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  else {
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d = 0;
    if (d < 5)
      LCD.setCursor(15 - d, 2);
    else
      LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  return d;
}




int LAdd(int d)
{
  if (P[d] < 9) {
    LCD.setCursor(14 - d, 1);
    if ((d == 9) && P[9] == 9) {
      return -1;
    }
    P[d]++;
    LCD.print(P[d]);
    return 1;
  }
  else {
    LCD.setCursor(14 - d, 1);
    if (LAdd(d + 1) > 0)
      P[d] = 0;
    LCD.print(P[d]);
    return 1;
  }
}

void LSub(int d)
{
  if (P[d] > 0) {
    LCD.setCursor(14 - d, 1);
    P[d]--;
    LCD.print(P[d]);
  }
}

int LMoveR(int d)
{
  if (d != 0) {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d--;
    LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  else {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d = 9;
    LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  return d;
}

int LMoveL(int d)
{
  if (d != 9) {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d++;
    LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  else {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d = 0;
    LCD.setCursor(14 - d, 2);
    LCD.write(1);
  }
  return d;
}


void ShowProgress(int x, int y, int barlen, double progress)
{
  double progress_temp = progress * barlen;
  int block_counter = progress_temp;
  progress_temp = (progress_temp - (double)block_counter) * 5;
  int block_divider = progress_temp;
  LCD.setCursor(x, y);
  for (int progress_c = 0; progress_c < block_counter; progress_c++)
    LCD.write(6);
  if (block_divider != 0)
    LCD.write(1 + block_divider);
  else LCD.print(" ");
}

int ListSD()
{
  SerialUSB.println("ListSD");
  dataFile.close();
  File root;
  int counter = 0;
  root = SD.open("/");
  root.seek(0);
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      return counter;
    }
    if (entry.isDirectory()) {
    } else {
      n = 0;
      SerialUSB.print(counter);
      SerialUSB.print(" ");
      while ((entry.name()[n]) != '\0')
      {
        list[counter][n] = entry.name()[n];
        SerialUSB.print(list[counter][n]);
        n++;
      }
      SerialUSB.println();
      counter++;
    }
    entry.close();
  }
  root.close();
  SerialUSB.println("SD closed");
}

void ClearKey()
{
  KeyU = false;
  KeyD = false;
  KeyR = false;
  KeyL = false;
  KeyOK = false;
  KeyB = false;
}

void Decode(char instruction[], int length)
{
  decoding = true;
  instruction[length] = 0;
  if (CMD_INFO)
  {
    SerialUSB.print("'");
    SerialUSB.print(instruction);
    SerialUSB.println("'");
  }
  if (instruction[0] == ';')
    return;
  FloatPt fp;
  fp.x = 0.0;
  fp.y = 0.0;
  fp.z = 0.0;
  fp.e = 0.0;
  byte code = 0;
  if (FindCommand('G', instruction, length))
  {
    code = (int)FindData('G', instruction, length);
    if (CODE_INFO)
      SerialUSB.println("G CODE FOUND");
    switch (code)
    {
      case 0:
        {
          if (!abs_mode) {
            if (FindData('X', instruction, length) != 32767)
              target.x += FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y += FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z += FindData('Z', instruction, length);
          }
          else
          {
            if (FindData('X', instruction, length) != 32767)
              target.x = FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y = FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z = FindData('Z', instruction, length);
          }
          if (FindData('F', instruction, length) != 32767)
            feedrate = FindData('F', instruction, length);
          CalDelta();
          if ((feedrate > 0) && (feedrate <= 3000))
          {
            long max_step = max(delta_steps.x, delta_steps.y);
            max_step = max(max_step, delta_steps.z);
            double temp = delta_steps.x;
            temp *= delta_steps.x;
            double space_step = temp;
            temp = delta_steps.y;
            temp *= delta_steps.y;
            space_step += temp;
            temp = delta_steps.z;
            temp *= delta_steps.z;
            space_step += temp;
            space_step = sqrt(space_step);
            feedrate_micros = 60000000 / x_unit / feedrate * space_step / max_step;
          }
          else
            feedrate_micros = 60000000 / x_unit / MAX_FEEDRATE;
          Move(feedrate_micros);
          if (((buffer_switch == 1) || (command_switch == 1)) & (POS_INFO))
          {
            SerialUSB.print("X ");
            SerialUSB.print(current.x);
            SerialUSB.print(" |Y ");
            SerialUSB.print(current.y);
            SerialUSB.print(" |Z ");
            SerialUSB.print(current.z);
            SerialUSB.print(" |E ");
            SerialUSB.print(current.e);
            SerialUSB.print(" |F ");
            SerialUSB.println(feedrate);
          }

        }
        break;


      case 1:
        {
          if (!abs_mode) {
            if (FindData('X', instruction, length) != 32767)
              target.x += FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y += FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z += FindData('Z', instruction, length);
            if (FindData('E', instruction, length) != 32767)
              target.e += FindData('E', instruction, length);
          }
          else
          {
            if (FindData('X', instruction, length) != 32767)
              target.x = FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y = FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z = FindData('Z', instruction, length);
            if (FindData('E', instruction, length) != 32767)
              target.e = FindData('E', instruction, length);
          }
          if (FindData('F', instruction, length) != 32767)
            feedrate = FindData('F', instruction, length);
          CalDelta();
          if ((feedrate > 0) && (feedrate <= 3000))
          {
            long max_step = max(delta_steps.x, delta_steps.y);
            max_step = max(max_step, delta_steps.z);
            double temp = delta_steps.x;
            temp *= delta_steps.x;
            double space_step = temp;
            temp = delta_steps.y;
            temp *= delta_steps.y;
            space_step += temp;
            temp = delta_steps.z;
            temp *= delta_steps.z;
            space_step += temp;
            space_step = sqrt(space_step);
            feedrate_micros = 60000000 / x_unit / feedrate * space_step / max_step;
          }
          else
            feedrate_micros = 60000000 / x_unit / MAX_FEEDRATE;

          Move(feedrate_micros);
          if (((buffer_switch == 1) || (command_switch == 1)) && (POS_INFO))
          {
            SerialUSB.print("X ");
            SerialUSB.print(current.x);
            SerialUSB.print(" |Y ");
            SerialUSB.print(current.y);
            SerialUSB.print(" |Z ");
            SerialUSB.print(current.z);
            SerialUSB.print(" |E ");
            SerialUSB.print(current.e);
            SerialUSB.print(" |F ");
            SerialUSB.println(feedrate);
          }

        }
        break;


      case 2:
        {
          if (!abs_mode) {
            if (FindData('X', instruction, length) != 32767)
              target.x += FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y += FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z += FindData('Z', instruction, length);
            if (FindData('E', instruction, length) != 32767)
              target.e += FindData('E', instruction, length);
          }
          else
          {
            if (FindData('X', instruction, length) != 32767)
              target.x = FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y = FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z = FindData('Z', instruction, length);
            if (FindData('E', instruction, length) != 32767)
              target.e = FindData('E', instruction, length);
          }
          FloatPt center;
          float angleA, angleB, angle, radius, length, aX, aY, bX, bY;

          if (FindData('I', instruction, length) != 32767)
            center.x = current.x + FindData('I', instruction, length);
          if (FindData('J', instruction, length) != 32767)
            center.y = current.y + FindData('J', instruction, length);

          aX = (current.x - center.x);
          aY = (current.y - center.y);
          bX = (fp.x - center.x);
          bY = (fp.y - center.y);

          if (code == 2) { // Clockwise
            angleA = atan2(bY, bX);
            angleB = atan2(aY, aX);
          }
          else { // Counterclockwise
            angleA = atan2(aY, aX);
            angleB = atan2(bY, bX);
          }

          if (angleB <= angleA) angleB += 2 * M_PI;
          angle = angleB - angleA;

          radius = sqrt(aX * aX + aY * aY);
          length = radius * angle;
          int steps, s, step;
          steps = (int) ceil(length / curve_section);

          for (s = 1; s <= steps; s++) {
            step = (code == 3) ? s : steps - s; // Work backwards for CW
            target.x = center.x + radius * cos(angleA + angle * ((float) step / steps));
            target.y = center.y + radius * sin(angleA + angle * ((float) step / steps));
            target.z = current.z;
            // Need to calculate rate for each section of curve
            if (FindData('F', instruction, length) != 32767)
              feedrate = FindData('F', instruction, length);
            CalDelta();
            if ((feedrate > 0) && (feedrate <= 3000))
            {
              long max_step = max(delta_steps.x, delta_steps.y);
              max_step = max(max_step, delta_steps.z);
              double temp = delta_steps.x;
              temp *= delta_steps.x;
              double space_step = temp;
              temp = delta_steps.y;
              temp *= delta_steps.y;
              space_step += temp;
              temp = delta_steps.z;
              temp *= delta_steps.z;
              space_step += temp;
              space_step = sqrt(space_step);
              feedrate_micros = 60000000 / x_unit / feedrate * space_step / max_step;
            }
            else
              feedrate_micros = 60000000 / x_unit / MAX_FEEDRATE;

            Move(feedrate_micros);
          }
        }
        break;



      case 3:
        {
          if (!abs_mode) {
            if (FindData('X', instruction, length) != 32767)
              target.x += FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y += FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z += FindData('Z', instruction, length);
            if (FindData('E', instruction, length) != 32767)
              target.e += FindData('E', instruction, length);
          }
          else
          {
            if (FindData('X', instruction, length) != 32767)
              target.x = FindData('X', instruction, length);
            if (FindData('Y', instruction, length) != 32767)
              target.y = FindData('Y', instruction, length);
            if (FindData('Z', instruction, length) != 32767)
              target.z = FindData('Z', instruction, length);
            if (FindData('E', instruction, length) != 32767)
              target.e = FindData('E', instruction, length);
          }
          FloatPt center;
          float angleA, angleB, angle, radius, length, aX, aY, bX, bY;

          if (FindData('I', instruction, length) != 32767)
            center.x = current.x + FindData('I', instruction, length);
          if (FindData('J', instruction, length) != 32767)
            center.y = current.y + FindData('J', instruction, length);

          aX = (current.x - center.x);
          aY = (current.y - center.y);
          bX = (fp.x - center.x);
          bY = (fp.y - center.y);

          if (code == 2) { // Clockwise
            angleA = atan2(bY, bX);
            angleB = atan2(aY, aX);
          }
          else { // Counterclockwise
            angleA = atan2(aY, aX);
            angleB = atan2(bY, bX);
          }

          if (angleB <= angleA) angleB += 2 * M_PI;
          angle = angleB - angleA;

          radius = sqrt(aX * aX + aY * aY);
          length = radius * angle;
          int steps, s, step;
          steps = (int) ceil(length / curve_section);

          for (s = 1; s <= steps; s++) {
            step = (code == 3) ? s : steps - s; // Work backwards for CW
            target.x = center.x + radius * cos(angleA + angle * ((float) step / steps));
            target.y = center.y + radius * sin(angleA + angle * ((float) step / steps));
            target.z = current.z;
            // Need to calculate rate for each section of curve
            if (FindData('F', instruction, length) != 32767)
              feedrate = FindData('F', instruction, length);
            CalDelta();
            if ((feedrate > 0) && (feedrate <= 3000))
            {
              long max_step = max(delta_steps.x, delta_steps.y);
              max_step = max(max_step, delta_steps.z);
              double temp = delta_steps.x;
              temp *= delta_steps.x;
              double space_step = temp;
              temp = delta_steps.y;
              temp *= delta_steps.y;
              space_step += temp;
              temp = delta_steps.z;
              temp *= delta_steps.z;
              space_step += temp;
              space_step = sqrt(space_step);
              feedrate_micros = 60000000 / x_unit / feedrate * space_step / max_step;
            }
            else
              feedrate_micros = 60000000 / x_unit / MAX_FEEDRATE;

            Move(feedrate_micros);
          }
        }
        break;



      case 4:
        delay((int)FindData('P', instruction, length));

        break;


      case 20:
        x_unit = X_STEPS_PER_INCH;
        y_unit = Y_STEPS_PER_INCH;
        z_unit = Z_STEPS_PER_INCH;
        e_unit = E_STEPS_PER_INCH;
        curve_section = CURVE_SECTION_INCH;
        CalDelta();
        SerialUSB.println("UNIT SET TO INCH");

        break;


      case 21:
        x_unit = X_STEPS_PER_MM;
        y_unit = Y_STEPS_PER_MM;
        z_unit = Z_STEPS_PER_MM;
        e_unit = E_STEPS_PER_MM;
        curve_section = CURVE_SECTION_MM;
        CalDelta();
        SerialUSB.println("UNIT SET TO MM");

        break;


      case 28:
        SerialUSB.println("GO BACK");
        SETtarget(0.0, 0.0, 0.0, target.e);
        CalDelta();
        Move(CalMaxSpeed());

        break;


      case 90:
        abs_mode = true;
        SerialUSB.println("ABS ON");

        break;


      case 91:
        abs_mode = false;
        SerialUSB.println("ABS OFF");

        break;


      case 92:
        {
          if (FindData('X', instruction, length) != 32767)
            current.x = FindData('X', instruction, length);
          if (FindData('Y', instruction, length) != 32767)
            current.y = FindData('Y', instruction, length);
          if (FindData('Z', instruction, length) != 32767)
            current.z = FindData('Z', instruction, length);
          if (FindData('E', instruction, length) != 32767)
            current.e = FindData('F', instruction, length);
        }
        break;


      default:
        break;
    }
  }


  if (FindCommand('M', instruction, length))
  {
    code = FindData('M', instruction, length);
    if (CODE_INFO)
      SerialUSB.println("M CODE FOUND");
    switch (code)
    {
      case 0:
        true;
        break;


      case 84:
        StopSteppers();
        break;

      case 104:
        if ((FindData('S', instruction, length) == 0) || (FindData('S', instruction, length) == 32767))
        {
          SerialUSB.println("Extruder heating OFF");
          digitalWrite(EXTRUDER_PIN, LOW);
          extruder_pwr = 0;
        }
        else {
          SerialUSB.print("Extruder temperature set to ");
          extruder_temp = FindData('S', instruction, length);
          SerialUSB.println(extruder_temp);
          extruder_pwr = 1;
        }
        break;

      case 106:
        if (FindCommand('S', instruction, length))
        {
          fan_speed = FindData('S', instruction, length);
          analogWrite(FAN_PIN, fan_speed);
          SerialUSB.print("Fan Speed set to ");
          SerialUSB.println(fan_speed);
        }
        break;

      case 107:
        fan_speed = 0;
        analogWrite(FAN_PIN, fan_speed);
        break;

      case 109:
        if ((FindData('S', instruction, length) == 0) || (FindData('S', instruction, length) == 32767))
        {
          SerialUSB.println("Extruder heating OFF");
          digitalWrite(EXTRUDER_PIN, LOW);
          extruder_pwr = 0;
        }
        else {
          SerialUSB.print("Extruder temperature set to ");
          extruder_temp = FindData('S', instruction, length);
          SerialUSB.println(extruder_temp);
          extruder_pwr = 1;
          long timeout = millis();
          while ((!extruder_ok) && (print_switch == 1) && ((millis() - timeout) < PREHEATING_TIMEOUT))
          {
            yield();
          }
        }
        break;

      case 140:
        if (FindCommand('S', instruction, length))
        {
          if ((FindData('S', instruction, length) == 0) || (FindData('S', instruction, length) == 32767))
          {
            SerialUSB.println("Bed heating OFF");
            digitalWrite(BED_PIN, LOW);
            bed_pwr = 0;
          }
          else {
            SerialUSB.print("Bed heating temperature set to ");
            bed_temp = FindData('S', instruction, length);
            SerialUSB.println(bed_temp);
            bed_pwr = 1;
            long timeout = millis();
            while ((!bed_ok) && (print_switch == 1) && ((millis() - timeout) < PREHEATING_TIMEOUT))
            {
              yield();
            }
          }
        }
        break;

      case 190:
        if (FindCommand('S', instruction, length))
        {
          bed_temp = FindData('S', instruction, length);
          bed_pwr = 1;
        }
        break;


      default:
        break;
    }
  }
  command_switch = 0;
  decoding = false;
}

double FindData(char keyword, char instruction[], int strlength)
{
  char temp[50] = "";
  for (int i = 0; i < strlength; i++)
  {
    if (instruction[i] == keyword)
    {
      i++;
      int k = 0;
      while ((i < strlength) && (k < 50))
      {
        if ((instruction[i] == 0) || (instruction[i] == ' '))
          break;
        temp[k] = instruction[i];
        i++;
        k++;
      }
      return strtod(temp, NULL);
    }
  }
  return 32767;
}


boolean FindCommand(char keyword, char instruction[], int strlength)
{
  for (int i = 0; i < strlength; i++)
  {
    if (instruction[i] == keyword)
      return true;
  }
  return false;
}


long CalMaxSpeed()
{
  return 60000000 / x_unit / MAX_FEEDRATE;
}


void StopXMAX()
{
  if (digitalRead(X_MAX_PIN) == 0)
  {
    TestXMAXPos = false;
    detachInterrupt(X_MAX_PIN);
    attachInterrupt(X_MAX_PIN, StartXMAX, RISING);
  }

}
void StopYMAX()
{
  if (digitalRead(Y_MAX_PIN) == 0)
  {
    TestYMAXPos = false;
    detachInterrupt(Y_MAX_PIN);
    attachInterrupt(Y_MAX_PIN, StartYMAX, RISING);
  }
}

void StopZMAX()
{
  if (digitalRead(Z_MAX_PIN) == 0)
  {
    TestZMAXPos = false;
    detachInterrupt(Z_MAX_PIN);
    attachInterrupt(Z_MAX_PIN, StartZMAX, RISING);
  }
}

void StopXMIN()
{
  if (digitalRead(X_MIN_PIN) == 0)
  {
    TestXMINPos = false;
    detachInterrupt(X_MIN_PIN);
    attachInterrupt(X_MIN_PIN, StartXMIN, RISING);
  }
}
void StopYMIN()
{
  if (digitalRead(Y_MIN_PIN) == 0)
  {
    TestYMINPos = false;
    detachInterrupt(Y_MIN_PIN);
    attachInterrupt(Y_MIN_PIN, StartYMIN, RISING);
  }
}

void StopZMIN()
{
  if (digitalRead(Z_MIN_PIN) == 0)
  {
    TestZMINPos = false;
    detachInterrupt(Z_MIN_PIN);
    attachInterrupt(Z_MIN_PIN, StartZMIN, RISING);
  }
}

void StartXMAX()
{
  if (digitalRead(X_MAX_PIN) == 1)
  {
    TestXMAXPos = true;
    detachInterrupt(X_MAX_PIN);
    attachInterrupt(X_MAX_PIN, StopXMAX, FALLING);
  }
}
void StartYMAX()
{
  if (digitalRead(Y_MAX_PIN) == 1)
  {
    TestYMAXPos = true;
    detachInterrupt(Y_MAX_PIN);
    attachInterrupt(Y_MAX_PIN, StopYMAX, FALLING);
  }
}

void StartZMAX()
{
  if (digitalRead(Z_MAX_PIN) == 1)
  {
    TestZMAXPos = true;
    detachInterrupt(Z_MAX_PIN);
    attachInterrupt(Z_MAX_PIN, StopZMAX, FALLING);
  }
}

void StartXMIN()
{
  if (digitalRead(X_MIN_PIN) == 1)
  {
    TestXMINPos = true;
    detachInterrupt(X_MIN_PIN);
    attachInterrupt(X_MIN_PIN, StopXMIN, FALLING);
  }
}
void StartYMIN()
{
  if (digitalRead(Y_MIN_PIN) == 1)
  {
    TestYMINPos = true;
    detachInterrupt(Y_MIN_PIN);
    attachInterrupt(Y_MIN_PIN, StopYMIN, FALLING);
  }
}

void StartZMIN()
{
  if (digitalRead(Z_MIN_PIN) == 1)
  {
    TestZMINPos = true;
    detachInterrupt(Z_MIN_PIN);
    attachInterrupt(Z_MIN_PIN, StopZMIN, FALLING);
  }
}

boolean TestKey(int KeyPin)
{
  long dc = 0;
  for (int dcc = 0; dcc < 10000; dcc++)
  {
    dc = 0;
    while (dc < 2000000)
      dc++;
    if (digitalRead(KeyPin) == 1)
    {
      return false;
    }
  }
  return true;
}

void KeyUPressed()
{
  if (TestKey(KeyU_PIN))
  {
    SerialUSB.println("KeyU");
    KeyU = true;
  }
}

void KeyDPressed()
{
  if (TestKey(KeyD_PIN))
  {
    SerialUSB.println("KeyD");
    KeyD = true;
  }
}

void KeyRPressed()
{
  if (TestKey(KeyR_PIN))
  {
    SerialUSB.println("KeyR");
    KeyR = true;
  }
}

void KeyLPressed()
{
  if (TestKey(KeyL_PIN))
  {
    SerialUSB.println("KeyL");
    KeyL = true;
  }
}

void KeyOKPressed()
{
  if (TestKey(KeyOK_PIN))
  {
    SerialUSB.println("KeyOK");
    KeyOK = true;
  }
}

void KeyBPressed()
{
  if (TestKey(KeyB_PIN))
  {
    SerialUSB.println("KeyB");
    KeyB = true;
  }
}

void KeyEMPressed()
{
  if (TestKey(KeyEM_PIN))
  {
    if (TestKey(KeyEM_PIN))
    {
      SerialUSB.println("KeyEM");
      EM();
    }
  }
}


void Initialize()
{
  printi = 0;
  bufferstartposition[0] = 0;
  bufferstartposition[1] = 0;
  bufferlength[0] = 0;
  bufferlength[1] = 0;
  printi = 0;
  for (n = 0; n < BUFFER_SIZE; n++)
  {
    membuffer[0][n].start = 0;
    membuffer[0][n].st[0] = 0;
    membuffer[1][n].start = 0;
    membuffer[1][n].st[0] = 0;
  }
  decoding = false;
  extruder_ok = false;
  fileposition = 0;
  extruder_temp = 0;
  bed_temp = 0;
  fan_speed = 0;
  command_switch = 1;
  Decode("G0 X-100 Y-100 Z300", 18);
  command_switch = 1;
  Decode("G0 Z-100", 9);
  SETtarget(0, 0, 0, 0);
  SETposition(0, 0, 0, 0);
}
void EM()
{

  EMS = true;
  stopposition = membuffer[buffernum][printi].start;
  stopline = bufferstartposition[buffernum] + printi;
  buffer_switch = 0;
  print_switch = 0;
  command_switch = 0;
  moving = false;
  digitalWrite(EXTRUDER_PIN, LOW);
  digitalWrite(BED_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  dataFile.close();
  SerialUSB.print("Printing process is interrupted at ");
  SerialUSB.print(stopposition);
  SerialUSB.print("(Instruction No. ");
  SerialUSB.print(stopline);
  SerialUSB.print(") of the file '");
  for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
  {
    SerialUSB.print(filename[n]);
  }
  SerialUSB.println("'");

  SerialUSB.print("Resume Argument: ");
  SerialUSB.print("'X");
  SerialUSB.print(current.x);
  SerialUSB.print(" Y");
  SerialUSB.print(current.y);
  SerialUSB.print(" Z");
  SerialUSB.print(current.z);
  SerialUSB.print(" E");
  SerialUSB.print(current.e);
  SerialUSB.print(" F");
  SerialUSB.print(feedrate);
  SerialUSB.print(" B");
  SerialUSB.print(bed_temp);
  SerialUSB.print(" H");
  SerialUSB.print(extruder_temp);
  SerialUSB.print(" C");
  SerialUSB.print(fan_speed);
  SerialUSB.println("'");
  bed_pwr = 0;
  digitalWrite(BED_PIN, LOW);
  analogWrite(BED_PIN, 0);
  extruder_pwr = 0;
  digitalWrite(EXTRUDER_PIN, LOW);
  analogWrite(EXTRUDER_PIN, 0);
  fan_speed = 0;
  digitalWrite(FAN_PIN, LOW);
  analogWrite(FAN_PIN, 0);
  page = 10;
  EMS = true;
  update = true;
  refresh = -1;
  ClearKey();
  SerialUSB.println(EMS);
}





void Move(long micro_delay)
{
  TestXMAXPos=digitalRead(X_MAX_PIN);
  TestXMINPos=digitalRead(X_MIN_PIN);
  TestYMAXPos=digitalRead(Y_MAX_PIN);
  TestYMINPos=digitalRead(Y_MIN_PIN);
  TestZMAXPos=digitalRead(Z_MAX_PIN);
  TestZMINPos=digitalRead(Z_MIN_PIN);
  int acc_steps=40;
  long delay_counter = 10;
  long delaytime = micro_delay + acc_steps*delay_counter;
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(E_ENABLE_PIN, LOW);
  long max_delta = 0;
  max_delta = max(delta_steps.x, delta_steps.y);
  max_delta = max(delta_steps.z, max_delta);
  max_delta = max(delta_steps.e, max_delta);

  long x_counter = -max_delta / 2;
  long y_counter = -max_delta / 2;
  long z_counter = -max_delta / 2;
  long e_counter = -max_delta / 2;

  boolean TestX, TestY, TestZ, TestE;
  TestX = true;
  TestY = true;
  TestZ = true;
  TestE = true;
  boolean StopX = false;
  boolean StopY = false;
  boolean StopZ = false;
  boolean StopE = false;
  unsigned int movecmd = 0;
  long time = micros();
  
  long steps_sum;
  
  steps_sum = -((long)current_steps.x - (long)target_steps.x) * (x_direction ? 1 : -1);
  steps_sum += -((long)current_steps.y - (long)target_steps.y) * (y_direction ? 1 : -1);
  steps_sum += -((long)current_steps.z - (long)target_steps.z) * (z_direction ? 1 : -1);
  long all_steps = steps_sum;
  moving = true;
  do
  {
    long start = micros();
    steps_sum = -((long)current_steps.x - (long)target_steps.x) * (x_direction ? 1 : -1);
    steps_sum += -((long)current_steps.y - (long)target_steps.y) * (y_direction ? 1 : -1);
    steps_sum += -((long)current_steps.z - (long)target_steps.z) * (z_direction ? 1 : -1);
    if (steps_sum > (all_steps - acc_steps))
    {
      delaytime -= delay_counter;
    }
    if (steps_sum < acc_steps)
    {
      delaytime += delay_counter;
    }
    PIOD->PIO_OWER = 0x0F;
    PIOD->PIO_OWDR = 0xFFFFFFF0;
    PIOD->PIO_ODSR = 0;
    movecmd = 0;
    TestX = (current_steps.x != target_steps.x) && (TestXMAXPos || (!x_direction)) && (TestXMINPos || (x_direction));
    TestY = (current_steps.y != target_steps.y) && (TestYMAXPos || (!y_direction)) && (TestYMINPos || (y_direction));
    TestZ = (current_steps.z != target_steps.z) && (TestZMAXPos || (!z_direction)) && (TestZMINPos || (z_direction));
    TestE = (current_steps.e != target_steps.e);
    if (TestX)
    {
      x_counter += delta_steps.x;
      if (x_counter > 0)
      {
        movecmd += 1;
        x_counter -= max_delta;
        current_steps.x += x_direction ? 1 : -1;
      }
    }
    else StopX = true;

    if (TestY)
    {
      y_counter += delta_steps.y;
      if (y_counter > 0)
      {
        movecmd += 2;
        y_counter -= max_delta;
        current_steps.y += y_direction ? 1 : -1;
      }
    }
    else StopY = true;

    if (TestZ)
    {
      z_counter += delta_steps.z;
      if (z_counter > 0)
      {
        movecmd += 4;
        z_counter -= max_delta;
        current_steps.z += z_direction ? 1 : -1;
      }
    }
    else StopZ = true;

    if (TestE)
    {
      e_counter += delta_steps.e;
      if (e_counter > 0)
      {
        movecmd += 8;
        e_counter -= max_delta;
        current_steps.e += e_direction ? 1 : -1;
      }
    }
    else StopE = true;

    PIOD->PIO_OWER = 0x0F;
    PIOD->PIO_OWDR = 0xFFFFFFF0;
    PIOD->PIO_ODSR = movecmd;
    long us = delaytime + start;
    while (micros() < us) {
      if ((us - micros()) > MAX_LCDTIME)
      {
        timeok = true;
      }
      else
      {
        timeok = false;
      }
      yield();
    }
  }
  while ((!StopX || !StopY || !StopZ || !StopE) && ((print_switch == 1) || (command_switch == 1)));
  current.x = current_steps.x / x_unit;
  current.y = current_steps.y / y_unit;
  current.z = current_steps.z / z_unit;
  current.e = current_steps.e / e_unit;
  CalDelta();
  moving = false;
}

void StopSteppers()
{
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);
  digitalWrite(E_ENABLE_PIN, HIGH);
}

void CalDelta()
{
  delta.x = abs(target.x - current.x);
  delta.y = abs(target.y - current.y);
  delta.z = abs(target.z - current.z);
  delta.e = abs(target.e - current.e);

  current_steps.x = x_unit * current.x;
  current_steps.y = y_unit * current.y;
  current_steps.z = z_unit * current.z;
  current_steps.e = e_unit * current.e;

  target_steps.x = x_unit * target.x;
  target_steps.y = y_unit * target.y;
  target_steps.z = z_unit * target.z;
  target_steps.e = e_unit * target.e;

  delta_steps.x = abs(target_steps.x - current_steps.x);
  delta_steps.y = abs(target_steps.y - current_steps.y);
  delta_steps.z = abs(target_steps.z - current_steps.z);
  delta_steps.e = abs(target_steps.e - current_steps.e);
  if (delta_steps.x <= 1)
  {
    current.x = target.x;
    current_steps.x = target_steps.x;
  }
  if (delta_steps.y <= 1)
  {
    current.y = target.y;
    current_steps.y = target_steps.y;
  }
  if (delta_steps.z <= 1)
  {
    current.z = target.z;
    current_steps.z = target_steps.z;
  }
  if (delta_steps.e <= 1)
  {
    current.e = target.e;
    current_steps.e = target_steps.e;
  }

  x_direction = (target.x >= current.x);
  y_direction = (target.y >= current.y);
  z_direction = (target.z >= current.z);
  e_direction = (target.e >= current.e);

  digitalWrite(X_DIR_PIN, !x_direction);
  digitalWrite(Y_DIR_PIN, y_direction);
  digitalWrite(Z_DIR_PIN, !z_direction);
  digitalWrite(E_DIR_PIN, e_direction);
}

void SETtarget(double x, double y, double z, double e)
{
  target.x = x;
  target.y = y;
  target.z = z;
  target.e = e;
  CalDelta();
}

void SETposition(double x, double y, double z, double e)
{
  current.x = x;
  current.y = y;
  current.z = z;
  current.e = e;
  CalDelta();
}

