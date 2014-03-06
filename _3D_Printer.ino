#include <Scheduler.h>
#include <PID.h>
#include <SPI.h>
#include <SD.h>
#include <MAX6675.h>

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
#define X_DIR_PIN 33
#define X_MIN_PIN 34
#define X_MAX_PIN 35
#define X_ENABLE_PIN 36

#define Y_STEP_PIN 26
#define Y_DIR_PIN 37
#define Y_MIN_PIN 38
#define Y_MAX_PIN 39
#define Y_ENABLE_PIN 40

#define Z_STEP_PIN 27
#define Z_DIR_PIN 41
#define Z_MIN_PIN 42
#define Z_MAX_PIN 43
#define Z_ENABLE_PIN 44

#define E_STEP_PIN 28
#define E_DIR_PIN 45
#define E_MIN_PIN 46
#define E_MAX_PIN 47
#define E_ENABLE_PIN 48

#define EXTRUDER_PIN 8
#define BED_PIN 9
#define FAN_PIN 10

#define SCK_PIN 76
#define SO_PIN 74
#define CS_PIN 2

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
};

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

FloatPt current_steps;
FloatPt target_steps;
FloatPt delta_steps;

datastr tempdata[1000];
datastr bufferA[200];
datastr bufferB[200];

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
int bufferlength = 0;
int flag = 0;
File dataFile;
char data[100];
int datalength;
int datanum = 0;
char filename[100];
long filesize, fileposition;
int sta = 0;
int report_delay=10;
int n = 0;

MAX6675 get_extruder_temp(CS_PIN,SO_PIN,SCK_PIN,1);

double bed_input, bed_output, bed_set;
double bed_aggKp = 4, bed_aggKi = 0.2, bed_aggKd = 1.0;
double bed_consKp = 1, bed_consKi = 0.05, bed_consKd = 0.25;

double extruder_input, extruder_output, extruder_set;
double extruder_aggKp = 4, extruder_aggKi = 0.2, extruder_aggKd = 1.0;
double extruder_consKp = 1, extruder_consKi = 0.05, extruder_consKd = 0.25;

PID bed_ctrl(&bed_input, &bed_output, &bed_set, bed_consKp, bed_consKi, bed_consKd, DIRECT);
PID extruder_ctrl(&extruder_input, &extruder_output, &extruder_set, extruder_consKp, extruder_consKi, extruder_consKd, DIRECT);

const int chipSelect = 4;

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB)
  {
    ;
  }
  pinMode(FAN_PIN, OUTPUT);
  pinMode(BED_PIN, OUTPUT);
  pinMode(EXTRUDER_PIN, OUTPUT);

  bed_set = 100;
  bed_ctrl.SetOutputLimits(0, 255);
  bed_ctrl.SetMode(AUTOMATIC);

  extruder_set = 100;
  extruder_ctrl.SetOutputLimits(0, 255);
  extruder_ctrl.SetMode(AUTOMATIC);

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
  pinMode(E_MIN_PIN, INPUT_PULLUP);
  pinMode(E_MAX_PIN, INPUT_PULLUP);
  CalDelta();
  feedrate = MAX_FEEDRATE;
  Scheduler.startLoop(TempControl);
  Scheduler.startLoop(SerialUSBReport);
  SerialUSB.println("INIT OK");
  i = 0;

  attachInterrupt(X_MAX_PIN, StopXMAX, FALLING);
  attachInterrupt(X_MIN_PIN, StopXMIN, FALLING);
  attachInterrupt(Y_MAX_PIN, StopYMAX, FALLING);
  attachInterrupt(Y_MIN_PIN, StopYMIN, FALLING);
  attachInterrupt(Z_MAX_PIN, StopZMAX, FALLING);
  attachInterrupt(Z_MIN_PIN, StopZMIN, FALLING);
  TestXMAXPos = true;
  TestYMAXPos = true;
  TestZMAXPos = true;
  TestXMINPos = true;
  TestYMINPos = true;
  TestZMINPos = true;
}

void loop() {
  while (SerialUSB.available()) {
    if (i == 0)
    {
      c = SerialUSB.read();
      i = 1;
    }
    else
    {
      data[i - 1] = SerialUSB.read();
      if (data[i - 1] == ';' || data[i - 1] == '\r' || data[i - 1] == '\n')
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
        dataFile = SD.open(filename);
        dataFile.seek(fileposition);
        if (dataFile) {
          bufferlength = 0;
          char ch;
          ch = dataFile.read();
          fileposition++;
          while ((dataFile.available() > 0))
          {
            j = 0;
            while ((dataFile.available() > 0) && (ch != '\n') && (ch != ';'))
            {
              tempdata[bufferlength].st[j] = ch;
              j++;
              fileposition++;
              ch = dataFile.read();
            }
            if (ch == ';')
              while (ch != '\n')
              {
                ch = dataFile.read();
                fileposition++;
              }
            tempdata[bufferlength].leng = j;
            bufferlength++;
            ch = dataFile.read();
            fileposition++;
          }
        }
        dataFile.close();
        for (j = 0; bufferlength > j; j++)
          Decode(tempdata[j].st, tempdata[j].leng);
        break;

      case '2' :
        for (n = 0; n < i; n++)
          filename[n] = data[n - 1];
        break;

      case '3' :
        dataFile = SD.open(filename);
        if (dataFile) {
          while ((Serial.available() > 0)) {
            byte inChar = Serial.read();
            dataFile.write(inChar);
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
        break;
      case 'R' :
        SerialUSB.print("Report Interval= ");
        report_delay=strtod(data,NULL);
        SerialUSB.print(report_delay);
        SerialUSB.println(" s");
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
  if (sta == 1)
  {
    File dataFile = SD.open(filename);
    filesize = dataFile.size();
    while (dataFile.available()) {
      int i = 0;
      while (data[i - 1] != '\n' || data[i - 1] != '\r')
      {
        data[i] = dataFile.read();
        i++;
      }
      datalength = i - 1;
      Decode(data, datalength);
      fileposition = dataFile.position();
    }
    dataFile.close();
    sta = 0;
  }
  yield();
}

void SerialUSBReport()
{
  if (report_delay)
  {
    double report;
    report = fileposition / filesize;
    SerialUSB.print("Progress= ");
    SerialUSB.print(report);
    SerialUSB.println("%");
    SerialUSB.print("Extruder Temp= ");
    SerialUSB.println(extruder_temp);
    SerialUSB.print("Bed Temp= ");
    SerialUSB.println(bed_temp);
    delay(report_delay*1000);
  }
  yield();
}


void Decode(char instruction[], int length)
{
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
          if (feedrate > 0)
          {
            double max_step = max(delta_steps.x, delta_steps.y);
            max_step = max(max_step, delta_steps.z);
            feedrate_micros = 60000000 / x_unit / feedrate;
          }
          else
            feedrate_micros = CalMaxSpeed();
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

          SerialUSB.println("DONE");
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
          if (feedrate > 0)
            feedrate_micros = CalFeedrateDelay(feedrate);
          else
            feedrate_micros = CalMaxSpeed();
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

          SerialUSB.println("DONE");
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
        SETtarget(0.0, 0.0, 0.0, target.e);
        Move(CalMaxSpeed());
        SerialUSB.println("GO BACK");
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
        SETposition(0.0, 0.0, 0.0, target.e);
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
        }
        break;

      case 140:
        if (FindCommand('S', instruction, length))
        {
          if (FindData('S', instruction, length) == 0)
            digitalWrite(BED_PIN, LOW);
          bed_pwr = 0;
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
  yield();
}

double FindData(char keyword, char instruction[], int strlength)
{
  char temp[10] = "";
  for (int i = 0; i < strlength; i++)
  {
    if (instruction[i] == keyword)
    {
      i++;
      int k = 0;
      while ((i < strlength) && (k < 10))
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


long CalFeedrateDelay(double infeedrate)
{
  return 60000000 / x_unit / infeedrate;
}

long CalMaxSpeed()
{
  return CalFeedrateDelay(MAX_FEEDRATE);
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

void Move(long micro_delay)
{
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
  do
  {
    long start = micros();
    PIOD->PIO_CODR = 15;
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
    PIOD->PIO_ODSR = movecmd;
    yield();
    delayMicroseconds(micro_delay - (micros() - start));
  }
  while (TestX || TestY || TestZ || TestE);
  current.x = current_steps.x / x_unit;
  current.y = current_steps.y / y_unit;
  current.z = current_steps.z / z_unit;
  current.e = current_steps.e / e_unit;
  CalDelta();
}

/*
boolean TestMove(byte MinPin, byte MaxPin, long currentP, long targetP, byte directionP)
{
  if ((targetP == currentP)||((!digitalRead(MinPin)) && !directionP)||((!digitalRead(MaxPin)) && directionP))
    return false;
  return true;
}
*/
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







