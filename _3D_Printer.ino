#include <Scheduler.h>
#include <PID.h>
#include <SPI.h>
#include <SD.h>
#include <MAX6675.h>
#include <LiquidCrystal.h>


#define PAGE_0_MAX 5
#define PAGE_1_MAX 2
#define LCD_INTERVAL 500
#define BUFFER_SIZE 200

#define X_STEPS_PER_INCH 5080
#define X_STEPS_PER_MM   200

#define Y_STEPS_PER_INCH 5080
#define Y_STEPS_PER_MM   200

#define Z_STEPS_PER_INCH 5080
#define Z_STEPS_PER_MM   200

#define E_STEPS_PER_INCH 19050
#define E_STEPS_PER_MM   750

#define MAX_FEEDRATE 2400

#define X_STEP_PIN 25
#define X_DIR_PIN 30
#define X_MIN_PIN 38
#define X_MAX_PIN 39
#define X_ENABLE_PIN 31

#define Y_STEP_PIN 26
#define Y_DIR_PIN 32
#define Y_MIN_PIN 40
#define Y_MAX_PIN 41
#define Y_ENABLE_PIN 33

#define Z_STEP_PIN 27
#define Z_DIR_PIN 34
#define Z_MIN_PIN 42
#define Z_MAX_PIN 43
#define Z_ENABLE_PIN 35

#define E_STEP_PIN 28
#define E_DIR_PIN 36
#define E_ENABLE_PIN 37

#define EXTRUDER_PIN 5
#define BED_PIN 6
#define FAN_PIN 11
#define SD_CS_PIN 8

#define SCK_PIN 76
#define SO_PIN 74
#define CS_PIN 2

#define LCD_LED_PIN 9

#define LCD_RS 52
#define LCD_EN 53
#define LCD_D0 51
#define LCD_D1 50
#define LCD_D2 49
#define LCD_D3 48
#define LCD_D4 47
#define LCD_D5 46
#define LCD_D6 45
#define LCD_D7 44

#define KeyU_PIN A0
#define KeyD_PIN A1
#define KeyR_PIN A2
#define KeyL_PIN A3
#define KeyOK_PIN A4
#define KeyB_PIN A5
#define KeyEM_PIN A6

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
  char st[50];
  int leng;
  long start;
};

void Add(int d);
void Sub(int d);
int MoveL(int d);
int MoveR(int d);


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

FloatPt current;
FloatPt target;
FloatPt delta;

LongPt current_steps;
LongPt target_steps;
LongPt delta_steps;

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

uint8_t Black[6][8] = {{
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
  },
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

boolean x_direction = 1;
boolean y_direction = 1;
boolean z_direction = 1;
boolean e_direction = 1;

double x_unit = X_STEPS_PER_MM;
double y_unit = Y_STEPS_PER_MM;
double z_unit = Z_STEPS_PER_MM;
double e_unit = E_STEPS_PER_MM;

volatile boolean TestXMAXPos, TestYMAXPos, TestZMAXPos;
volatile boolean TestXMINPos, TestYMINPos, TestZMINPos;

double feedrate = 0.0;
long feedrate_micros = 0;
boolean abs_mode = true;
byte extruder_pwr = 0;
byte bed_pwr = 0;
int fan_speed = 0;
int extruder_temp = 0;
int bed_temp = 0;
int num = 0;
char c;
int i = 0;
int j = 0;
int flag = 0;
File dataFile;
char data[200];
int datalength;
int datanum = 0;
char filename[100];
long filesize = 1;
long fileposition = 0;
int sta = 0;
int report_delay = 10;
int n = 0;
int print_switch = 0;
int bufferlength[2] = {0, 0};
int bufferstartposition[2] = {0, 0};
int buffernum = 0;
int printi = 0;
int buffer_switch = 0;
int brightness = 200;
boolean decoding=false;

long Dec[10] = {
  1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};
int digit = 0;
int P[10] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

boolean IsSD = false;
int filemax = 0;
char list[100][20];

int page = 0;
int firstrow = 0;
int CursorR = 0;
boolean update = false;
int refresh = -1;

boolean KeyU, KeyD, KeyR, KeyL, KeyOK, KeyB;

char menu0[PAGE_0_MAX][20] = {"ABC", "DEF", "GHI", "JKL", "MNO"};
char menu1[PAGE_1_MAX][20] = {"PQR", "STU"};



MAX6675 get_extruder_temp(CS_PIN, SO_PIN, SCK_PIN, 1);
LiquidCrystal LCD(LCD_RS, LCD_EN, LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

double bed_input, bed_output, bed_set;
double bed_aggKp = 4, bed_aggKi = 0.2, bed_aggKd = 1.0;
double bed_consKp = 1, bed_consKi = 0.05, bed_consKd = 0.25;

double extruder_input, extruder_output, extruder_set;
double extruder_aggKp = 4, extruder_aggKi = 0.2, extruder_aggKd = 1.0;
double extruder_consKp = 1, extruder_consKi = 0.05, extruder_consKd = 0.25;

PID bed_ctrl(&bed_input, &bed_output, &bed_set, bed_consKp, bed_consKi, bed_consKd, DIRECT);
PID extruder_ctrl(&extruder_input, &extruder_output, &extruder_set, extruder_consKp, extruder_consKi, extruder_consKd, DIRECT);


void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB)
  {
  }
  for (n=0;n<100;n++)
  {
    filename[n]=0;
  }
  for (n=0;n<200;n++)
  {
    data[n]=0;
  }
  LCD.begin(20, 4);
  LCD.createChar(1, ArrowR);
  LCD.createChar(2, ArrowL);
  for (n = 0; n < 6; n++)
    LCD.createChar(n + 3, Black[n]);
  LCD.setCursor(0, 0);
  LCD.print("AOI 3D Printer");
  LCD.setCursor(0, 1);
  LCD.print("Initializing...");

  pinMode(FAN_PIN, OUTPUT);
  pinMode(BED_PIN, OUTPUT);
  pinMode(EXTRUDER_PIN, OUTPUT);

  bed_set = 100;
  bed_ctrl.SetOutputLimits(0, 255);
  bed_ctrl.SetMode(AUTOMATIC);

  extruder_set = 100;
  extruder_ctrl.SetOutputLimits(0, 255);
  extruder_ctrl.SetMode(AUTOMATIC);

  filename[0] = '/';
  filename[1] = '1';
  filename[2]='.';
  filename[3]='t';
  filename[4]='x';
  filename[5]='t';
  filename[6]=0;
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

  CalDelta();
  feedrate = MAX_FEEDRATE;
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



  SerialUSB.println("SYSTEM INITIALIZED");
  LCD.setCursor(0, 2);
  LCD.print("System Initialized");
  delay(500);
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Waiting for SD card");
  SerialUSB.println("Waiting for SD card");
  if (!SD.begin(SD_CS_PIN)) {
    SerialUSB.println("SD card not found");
    LCD.setCursor(0, 1);
    LCD.print("SD card not found");
    IsSD = false;
  }
  else {
    SerialUSB.println("SD card connected");
    LCD.setCursor(0, 1);
    LCD.print("SD card connected");
    IsSD = true;
  }
  SerialUSB.println("SERIAL REPORT INTERVAL SET TO 10s");
  SerialUSB.println("$ FOR COMMAND / 1 FOR START / 2 FOR SET FILE PATH / R FOR SET REPORT INTERVAL");



}

void loop() {
  switch (page) {
    case 0:  // Main Menu | menu0
      {
        if (KeyU)
        {
          if (firstrow > 0)
          {
            if (CursorR == 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              CursorR--;
              update = true;
            }
          }
          ClearKey();
        }
        if (KeyD)
        {
          if (firstrow != PAGE_0_MAX - 3)
          {
            if (CursorR == 3)
            {
              firstrow++;
              update = true;
            }
            else
            {
              CursorR++;
              update = true;
            }
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
          ClearKey();
        }
        if (KeyOK)
        {
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
          if (firstrow > 0)
          {
            if (CursorR == 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              CursorR--;
              update = true;
            }
          }
          ClearKey();
        }
        if (KeyD)
        {
          if (firstrow != filemax - 3)
          {
            if (CursorR == 3)
            {
              firstrow++;
              update = true;
            }
            else
            {
              CursorR++;
              update = true;
            }
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
          int select = firstrow + CursorR;
          page = 9;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          SerialUSB.print("Printing File: ");
          for (n = 0; list[select][n] != '\0'; n++)
          {
            filename[n] = list[select][n];
            SerialUSB.print(filename[n]);
          }
          filename[n] = '\0';
          SerialUSB.println();
          buffer_switch = 1;
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
          if (firstrow > 0)
          {
            if (CursorR == 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              CursorR--;
              update = true;
            }
          }
          ClearKey();
        }
        if (KeyD)
        {
          if (firstrow != PAGE_0_MAX - 3)
          {
            if (CursorR == 3)
            {
              firstrow++;
              update = true;
            }
            else
            {
              CursorR++;
              update = true;
            }
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
          page = 0;
          firstrow = 0;
          CursorR = 0;
          ClearKey();
          update = true;
        }
        if (KeyOK)
        {
          int select = firstrow + CursorR;
          page = select + 1;
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


    case 6://Resume Print select file
      {
        if (KeyU)
        {
          if (firstrow > 0)
          {
            if (CursorR == 0)
            {
              firstrow--;
              update = true;
            }
            else
            {
              CursorR--;
              update = true;
            }
          }
          ClearKey();
        }
        if (KeyD)
        {
          if (firstrow != PAGE_0_MAX - 3)
          {
            if (CursorR == 3)
            {
              firstrow++;
              update = true;
            }
            else
            {
              CursorR++;
              update = true;
            }
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
          page = 8;
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
          page = 7;
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

    case 7://Resume Print select point
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
          //set position
          fileposition = 0;
          for (n = 0; n < 9; n++)
          {
            fileposition += P[n] * Dec[n];
          }
          if (fileposition <= filesize)
          {
            buffer_switch = 1;
            update = true;
          }
          else
          {
            fileposition = 0;
          }
          ClearKey();
        }
        if (KeyB)
        {
          page = 2;
          update = true;
          ClearKey();
        }
      }
      break;

    case 8://Now printing
      {
        if (KeyU)
        {
          if (brightness < 255)
            brightness++;
          update = true;
          ClearKey();
        }
        if (KeyD)
        {
          if (brightness > 0)
            brightness--;
          update = true;
          ClearKey();
        }
        if (KeyR)
        {
          page = 9;
          update = true;
          ClearKey();
        }
        if (KeyL)
        {
          KeyEMPressed();
          ClearKey();
        }
        if (KeyOK)
        {
          page = 9;
          update = true;
          ClearKey();
        }
        if (KeyB)
        {
          KeyEMPressed();
          ClearKey();
        }
      }
      break;

    case 9://Sensor Value
      {
        if (KeyU)
        {
          if (brightness < 255)
            brightness++;
          update = true;
          ClearKey();
        }
        if (KeyD)
        {
          if (brightness > 0)
            brightness--;
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
          LCD.write(1);
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(n - firstrow, 1);
            LCD.print(menu0[n - firstrow]);
          }
        }
        break;
      case 1:
        {
          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(1);
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(n - firstrow, 1);
            LCD.print(list[n - firstrow]);
          }
        }
        break;

      case 2:
        {
          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(1);
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(n - firstrow, 1);
            LCD.print(menu1[n - firstrow]);
          }
        }
        break;

      case 4:
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Waiting for SD card");
          if (!SD.begin(SD_CS_PIN)) {
            SerialUSB.println("SD card not found");
            LCD.setCursor(0, 1);
            LCD.print("SD card not found");
            IsSD = false;
          }
          else {
            SerialUSB.println("SD card connected");
            LCD.setCursor(0, 1);
            LCD.print("SD card connected");
            IsSD = true;
          }
          LCD.setCursor(0, 3);
          LCD.print("Press   button");
          LCD.setCursor(6, 3);
          LCD.write(2);
        }
        break;
      case 5: //About Page
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Elec: Yicheng Bao");
          LCD.setCursor(0, 1);
          LCD.print("Mec: Yiping Zheng");
          LCD.setCursor(0, 2);
          LCD.print("Prog: Yicheng Bao");
          LCD.setCursor(0, 3);
          LCD.print("Dirctor: Lile Xie");
        }
        break;

      case 6://Resume Print select file
        {
          refresh = -1;
          LCD.setCursor(0, CursorR);
          LCD.write(1);
          for (n = firstrow; n < firstrow + 4; n++)
          {
            LCD.setCursor(n - firstrow, 1);
            LCD.print(list[n - firstrow]);
          }
        }
        break;

      case 7://Resume Print select point
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Set Resume Position");
          LCD.setCursor(5, 1);
          for (n = 0; n < 10; n++)
          {
            P[n] = fileposition - fileposition / 10;
            fileposition = fileposition / 10;
          }
          for (n = 0; n < 10; n--)
          {
            LCD.print(P[9 - n]);
          }
          LCD.setCursor(14, 2);
          digit = 0;
          LCD.write(3);
        }
        break;

      case 8://Now printing
        {
          refresh = 8;
          LCD.setCursor(0, 0);
          LCD.print("Now Printing File:");
          LCD.setCursor(1, 1);
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
          refresh = -1;
          LCD.setCursor(0, 0);
          long stopposition = bufferstartposition[buffernum] + printi;
          long stopline = membuffer[buffernum][printi].start;
          LCD.print("Interrupted at ");
          LCD.setCursor(0, 1);
          LCD.print("Pos= ");
          LCD.print(stopposition);
          LCD.setCursor(0, 2);
          LCD.print("Cmd No.= ");
          LCD.print(stopline);
          LCD.setCursor(0, 3);
          LCD.print("Press any key to exit");
        }
        break;

      case 11://Finish Page
        {
          refresh = -1;
          LCD.setCursor(0, 0);
          LCD.print("Congratulation!");
          LCD.setCursor(0, 1);
          LCD.print("3D Printing Done!");
          LCD.setCursor(0, 2);
          LCD.setCursor(0, 3);
          LCD.print("Press any key to exit");
        }
        break;
    }
  }
  yield();
}



void LCDTimer() {
  
  if (refresh > 0)
  {
    analogWrite(LCD_LED_PIN, brightness);
    switch (refresh)
    {
      case 8:
        {
          LCD.clear();
          LCD.setCursor(0, 0);
          LCD.print("Now Printing File:");
          LCD.setCursor(1, 1);
          LCD.print(filename);
          long stopposition = bufferstartposition[buffernum] + printi;
          double report = (double)(stopposition / filesize);
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
          char stemp[20];
          sprintf(stemp, "Ext= % 3.0lf C -> % 3.0lf C", extruder_input, extruder_temp);
          LCD.print(stemp);
          LCD.setCursor(0, 1);
          sprintf(stemp, "Bed= % 3.0lf C -> % 3.0lf C", bed_input, bed_temp);
          LCD.print(stemp);
          LCD.setCursor(0, 2);
          sprintf(stemp, "Fan= % 3d  E=%lf", fan_speed,current.e);
          LCD.print(stemp);
          LCD.setCursor(0, 3);
          int xtemp=current.x, ytemp=current.y, ztemp=current.z;
          sprintf(stemp, "X=% 3d Y=% 3d Z=% 3d", xtemp, ytemp, ztemp);
          LCD.print(stemp);
        }
        break;
    }
    long LCDs = millis();
    while (millis() - LCDs < 500)
    {
    }
    yield();
  }
  yield();
}


void SerialCLI() {
  while (SerialUSB.available()) {
    if (i == 0)
    {
      c = SerialUSB.read();
      i = 1;
    }
    else
    {
      data[i - 1] = SerialUSB.read();
      SerialUSB.print(data[i-1],HEX);
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

      case '1' :
        SerialUSB.print("Printing File: ");
        for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
        {
          SerialUSB.print(filename[n]);
        }
        SerialUSB.println();
        buffer_switch = 1;
        break;

      case '2' :
        SerialUSB.print("File path set to: ");
        for (n = 0; n < i - 1; n++)
        {
          filename[n] = data[n];
          SerialUSB.print(filename[n],HEX);
        }
        filename[n-1]=0;
        for (n = 0; n < i; n++)
        {
          SerialUSB.print(filename[n],HEX);
        }

        break;

      case '3' :
        dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile) {
          while ((Serial.available() > 0)) {
            byte inChar = Serial.read();
            dataFile.write(inChar);
          }
        }
        else {
          SerialUSB.println("ERROR: Cannot open the file");
        }
        break;
      case '4' :
        {
          long resume_position = strtod(data, NULL);
          dataFile = SD.open(filename);
          if (dataFile)
          {
            if (resume_position < dataFile.size())
            {
              fileposition = resume_position;
              SerialUSB.print("Resume printing process from ");
              SerialUSB.print(resume_position);
              SerialUSB.print(" at the file ");
              for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
              {
                SerialUSB.print(filename[n]);
              }
              SerialUSB.println();
              Decode("G0 X300 Y300 Z300", 18);
              buffer_switch = 1;
            }
            else {
              SerialUSB.println("ERROR: Resume position exceeds file size");
            }
            dataFile.close();
          }
          else {
            SerialUSB.println("ERROR: Cannot open the file");
          }
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
        Decode(data, datalength);
        SerialUSB.println("DONE");
        break;
      case 'R' :
        SerialUSB.print("Report Interval= ");
        report_delay = strtod(data, NULL);
        SerialUSB.print(report_delay);
        SerialUSB.println(" s");
        break;

      case 'S' :
        {
          bed_pwr = 0;
          digitalWrite(BED_PIN, LOW);
          extruder_pwr = 0;
          digitalWrite(EXTRUDER_PIN, LOW);
          fan_speed = 0;
          digitalWrite(FAN_PIN, LOW);
          buffer_switch = 0;

          long stopposition = membuffer[buffernum][printi].start; 
  long stopline =bufferstartposition[buffernum] + printi;
          SerialUSB.print("Printing process is interrupted at ");
          SerialUSB.print(stopposition);
          SerialUSB.print("(Instruction No. ");
          SerialUSB.print(stopline);
          SerialUSB.print(") of the file '");
          for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
          {
            SerialUSB.print(filename[n]);
          }
          page = 10;
          update = true;
          SerialUSB.println("'");
        }
        break;
    }
    flag = 0;
    i = 0;
    c = 0;
  }
  yield();
}


void TempControl()
{
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
  if (bed_pwr == 1)
  {
    bed_set = bed_temp;
    bed_input = bed_input; //get temp
    double delta_temp = abs(bed_input - bed_set);
    if (delta_temp < 10)
    {
      bed_ctrl.SetTunings(bed_consKp, bed_consKi, bed_consKd);
    }
    else
    {
      bed_ctrl.SetTunings(bed_aggKp, bed_aggKi, bed_aggKd);
    }
    bed_ctrl.Compute();
    analogWrite(BED_PIN, bed_output);
  }
  if (extruder_pwr == 1)
  {
    extruder_set = extruder_temp;
    extruder_input = (double) get_extruder_temp.read_temp(); //get temp
    double delta_temp = abs(extruder_input - extruder_set);
    if (delta_temp < 10)
    {
      extruder_ctrl.SetTunings(extruder_consKp, extruder_consKi, extruder_consKd);
    }
    else
    {
      extruder_ctrl.SetTunings(extruder_aggKp, extruder_aggKi, extruder_aggKd);
    }
    extruder_ctrl.Compute();
    analogWrite(EXTRUDER_PIN, extruder_output);
  }
  yield();
}

void Print()
{
  if ((print_switch == 1)&&(!decoding))
  {
    SerialUSB.print("Print start");
    for (printi = 0; printi < bufferlength[buffernum]; printi++)
    {
      SerialUSB.println("Print->Decode");
      Decode(membuffer[buffernum][printi].st, membuffer[buffernum][printi].leng);
      if(decoding)
      yield();
    }
  
  print_switch = 0;
  buffernum = 1 - buffernum;
  }
  yield();
}

void SerialUSBReport()
{
  if ((report_delay)&&(!decoding))
  {
    double report;
    report = membuffer[buffernum][printi].start / filesize;
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
    SerialUSB.print(report);
    SerialUSB.print("%\nFile position= ");
    SerialUSB.print(membuffer[buffernum][printi].start);
    SerialUSB.print(" | Instruction No.= ");
    SerialUSB.println(bufferstartposition[buffernum] + printi);
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
    delay(report_delay * 1000);
  }
  yield();
}

void SDtoMEM()
{
  if ((buffer_switch == 1)&&(!decoding))
  {
    dataFile = SD.open(filename);
    if (dataFile)
    {
      dataFile.seek(fileposition);
      buffernum=1;
      char ch;
      filesize = dataFile.size();
      SerialUSB.println(filesize);
      int bufferposition = 1;
        bufferstartposition[1-buffernum] = fileposition;
        ch = dataFile.read();
        SerialUSB.print(ch);
        fileposition++;
        
        
        
        while ((buffer_switch == 1) && (dataFile.available() > 0) && (bufferposition < BUFFER_SIZE)&&(fileposition < (filesize)))
        {
          j = 0;
          while ((buffer_switch == 1) && (dataFile.available() > 0) && (ch != '\n') && (ch != ';')&&(fileposition < (filesize)))
          {
            membuffer[1 - buffernum][bufferposition].st[j] = ch;
            j++;
            fileposition++;
            ch = dataFile.read();
            SerialUSB.print(ch);
          }
          if ((ch == ';'))
            while ((ch != '\n')&&(fileposition < (filesize)))
            {
              ch = dataFile.read();
              fileposition++;
              j++;
            }
          membuffer[1 - buffernum][bufferposition].leng = j+1;
          membuffer[1 - buffernum][bufferposition].st[j]=0;
          SerialUSB.println();
          SerialUSB.println(j);
          membuffer[1 - buffernum][bufferposition].start = fileposition - j-1;
          SerialUSB.println(fileposition-j-1);
          SerialUSB.println(fileposition);
          bufferposition++;
          if ((fileposition < (filesize)))
          ch = dataFile.read();
          SerialUSB.print(ch);
          fileposition++;
        }
        bufferlength[1 - buffernum] = bufferposition;
        SerialUSB.println(fileposition);
        buffernum=0;
      print_switch=1;
      
      /*
      
      
      while (fileposition < filesize) {
        int bufferposition = 0;
        bufferstartposition[1 - buffernum] = fileposition;
        ch = dataFile.read();
        SerialUSB.print(ch);
        fileposition++;
        while ((buffer_switch == 1) && (dataFile.available() > 0) && (print_switch == 1) && (bufferposition < BUFFER_SIZE)&&(fileposition < (filesize-1)))
        {
          j = 0;
          while ((buffer_switch == 1) && (dataFile.available() > 0) && (ch != '\n') && (ch != ';')&&(fileposition < (filesize-1)))
          {
            membuffer[1 - buffernum][bufferposition].st[j] = ch;
            j++;
            fileposition++;
            ch = dataFile.read();
            SerialUSB.print(ch);
          }
          if (ch == ';')
            while ((ch != '\n')&&(fileposition < (filesize-1)))
            {
              ch = dataFile.read();
              fileposition++;
              j++;
            }
          membuffer[1 - buffernum][bufferposition].leng = j;
          SerialUSB.println(j);
          membuffer[1 - buffernum][bufferposition].start = fileposition - j-2;
          SerialUSB.println(fileposition-j);
          bufferposition++;
          ch = dataFile.read();
          SerialUSB.print(ch);
          fileposition++;
        }
        bufferlength[1 - buffernum] = bufferposition;
        while (print_switch == 1);
        yield();
        print_switch = 1;
      }*/
      dataFile.close();
      SerialUSB.print("PRINT PROCESS DONE");
      page = 11;
      update = true;

    }
    else {
      SerialUSB.println("ERROR: Cannot open the file");
    }
    buffer_switch = 0;
  }
  yield();
}

void Add(int d)
{
  if (P[d] < 9) {
    LCD.setCursor(14 - d, 1);
    if ((d == 9) && P[9] == 4) {
      return;
    }
    P[d]++;
    LCD.print(P[d]);
  }
  else {
    if ((P[9] == 4) && (((P[d - 1] == 9) && (d != 8)) || ((d == 8) && (P[8] == 9)))) {
      return;
    }
    LCD.setCursor(14 - d, 1);
    P[d] = 0;
    LCD.print(P[d]);
    Add(n - 1);
  }
}

void Sub(int d)
{
  for (n = 9; (n >= 0) && (n >= d); n--) {
    if (P[n] != 0) {
      if (P[d] > 0) {
        LCD.setCursor(14 - d, 1);
        P[d]--;
        LCD.print(P[d]);
      }
      else {
        LCD.setCursor(14 - d, 1);
        P[d] = 9;
        LCD.print(P[d]);
        Sub(d);

      }
      break;
    }
  }
}

int MoveR(int d)
{
  if (d != 0) {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d--;
    LCD.setCursor(14 - d, 2);
    LCD.write(3);
  }
  else {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d = 9;
    LCD.setCursor(14 - d, 2);
    LCD.write(3);
  }
  return d;
}

int MoveL(int d)
{
  if (d != 9) {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d++;
    LCD.setCursor(14 - d, 2);
    LCD.write(3);
  }
  else {
    LCD.setCursor(14 - d, 2);
    LCD.print(" ");
    d = 0;
    LCD.setCursor(14 - d, 2);
    LCD.write(3);
  }
  return d;
}





void ShowProgress(int x, int y, int barlen, double progress)
{
  double progress_temp = progress * barlen;
  int block_counter = floor(progress_temp);;
  progress_temp = (progress - (double)block_counter) * 5;
  int block_divider = progress_temp;
  LCD.setCursor(x, y);
  for (int progress_c = 0; progress_c < block_counter; progress_c++)
    LCD.write(8);
  LCD.write(3 + block_divider);
}

int ListSD()
{
  File root;
  int counter = 0;
  root = SD.open("/");
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      return counter;
    }
    if (entry.isDirectory()) {
    } else {
      n = 0;
      while ((entry.name()[n]) != '\0')
      {
        list[counter][n] = entry.name()[n];
        n++;
      }
      counter++;
    }
    entry.close();
  }
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
  decoding=true;
  SerialUSB.println("Decode");
  SerialUSB.println(instruction);
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
          if ((feedrate > 0)&&(feedrate<=3000))
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
            SerialUSB.print("Move");
            SerialUSB.println(feedrate_micros);
          Move(feedrate_micros);

          SerialUSB.print("X Axis=");
          SerialUSB.println(current.x);
          SerialUSB.print("Y Axis=");
          SerialUSB.println(current.y);
          SerialUSB.print("Z Axis=");
          SerialUSB.println(current.z);
          SerialUSB.print("Extruder=");
          SerialUSB.println(current.e);
          SerialUSB.print("Feedrate=");
          SerialUSB.println(feedrate);

    
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
          if ((feedrate > 0)&&(feedrate<=3000))
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
          SerialUSB.print("X Axis=");
          SerialUSB.println(current.x);
          SerialUSB.print("Y Axis=");
          SerialUSB.println(current.y);
          SerialUSB.print("Z Axis=");
          SerialUSB.println(current.z);
          SerialUSB.print("Extruder=");
          SerialUSB.println(current.e);
          SerialUSB.print("Feedrate=");
          SerialUSB.println(feedrate);

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
        CalDelta();
        SerialUSB.println("UNIT SET TO INCH");
        
        break;


      case 21:
        x_unit = X_STEPS_PER_MM;
        y_unit = Y_STEPS_PER_MM;
        z_unit = Z_STEPS_PER_MM;
        e_unit = E_STEPS_PER_MM;
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
            current.e = FindData('E', instruction, length);
        }
        break;


      default:
        break;
    }
  }


  if (FindCommand('M', instruction, length))
  {
    code = FindData('M', instruction, length);
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
        if (FindCommand('S', instruction, length))
        {
          if (FindData('S', instruction, length) == 0)
            digitalWrite(EXTRUDER_PIN, LOW);
          extruder_pwr = 0;
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
        if (FindCommand('S', instruction, length))
        {
          extruder_temp = FindData('S', instruction, length);
          SerialUSB.print("Extruder temperature set to ");
          SerialUSB.println(extruder_temp);
        }
        break;

      case 140:
        if (FindCommand('S', instruction, length))
        {
          if (FindData('S', instruction, length) == 0)
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
  decoding=false;
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
    SerialUSB.println("X->MAX");
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
    SerialUSB.println("X->MIN");
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
    SerialUSB.println("X<-MAX");
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
    SerialUSB.println("X<-MIN");
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



void KeyUPressed()
{
  KeyU = true;
}

void KeyDPressed()
{
  KeyD = true;
}

void KeyRPressed()
{
  KeyR = true;
}

void KeyLPressed()
{
  KeyL = true;
}

void KeyOKPressed()
{
  KeyOK = true;
}

void KeyBPressed()
{
  KeyB = true;
}

void KeyEMPressed()
{
  bed_pwr = 0;
  refresh = -1;
  digitalWrite(BED_PIN, LOW);
  extruder_pwr = 0;
  digitalWrite(EXTRUDER_PIN, LOW);
  fan_speed = 0;
  digitalWrite(FAN_PIN, LOW);
  buffer_switch = 0;

  long stopposition = membuffer[buffernum][printi].start; 
  long stopline =bufferstartposition[buffernum] + printi;
  SerialUSB.print("Printing process is interrupted at ");
  SerialUSB.print(stopposition);
  SerialUSB.print("(Instruction No. ");
  SerialUSB.print(stopline);
  SerialUSB.print(") of the file '");
  for (n = 0; ((filename[n] != '\n') && (filename[n] != '\0')); n++)
  {
    SerialUSB.print(filename[n]);
  }
  page = 10;
  refresh = -1;
  update = true;
}





void Move(long micro_delay)
{
  long delaytime=micro_delay+100;
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
  unsigned int movecmd = 0;
  long time = micros();
  long delay_counter=1;
  long steps_sum=-(current_steps.x-target_steps.x)*x_direction-(current_steps.y-target_steps.y)*y_direction-(current_steps.z-target_steps.z)*z_direction;
  long all_steps=steps_sum;
  do
  {
    long start = micros();
    
    steps_sum=-(current_steps.x-target_steps.x)*x_direction-(current_steps.y-target_steps.y)*y_direction-(current_steps.z-target_steps.z)*z_direction;
    if (steps_sum>(all_steps-100))
    delaytime-=delay_counter;
    if (steps_sum<100)
    delaytime+=delay_counter;
    
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
    PIOD->PIO_OWER = 0x0F;
    PIOD->PIO_OWDR = 0xFFFFFFF0;
    PIOD->PIO_ODSR = movecmd;
    long startt =micros();
    long us=delaytime-(micros()-start);
   while ((micros() - startt) < us) {
      }
startt =micros();
   yield();
  SerialUSB.println(micros()-startt); 
      
  }
  while (TestX || TestY || TestZ || TestE);
  current.x = current_steps.x / x_unit;
  current.y = current_steps.y / y_unit;
  current.z = current_steps.z / z_unit;
  current.e = current_steps.e / e_unit;
  CalDelta();
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

  digitalWrite(X_DIR_PIN, x_direction);
  digitalWrite(Y_DIR_PIN, y_direction);
  digitalWrite(Z_DIR_PIN, z_direction);
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

