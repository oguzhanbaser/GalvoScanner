#include <Arduino.h>
#include <TMC2209.h>
#include <MultiStepper.h>
#include <FastAccelStepper.h>
#include <math.h>

#define endswitchX 34
#define endswitchY 35

#define en 21
#define pdn_rx 23
#define pdn_tx 22

#define step_x 5
#define dir_x 4
// #define diag_x 18
// #define en_x 37
// #define rst_x 47
// #define m3_x 48
// #define m2_x 35
// #define m1_x 36

#define step_y 2
#define dir_y 15
// #define diag_y 19
// #define en_y 21
// #define rst_y 40
// #define m3_y 41
// #define m2_y 1
// #define m1_y 8

#define micro_step 32

#define homePosX 7168 * micro_step / 256 // Tweak this to get a perfect 45 deg angle as 0 position
#define homePosY 6748 * micro_step / 256 // Tweak this to get a perfect 45 deg angle as 0 position

#define D 950 // orthogonal distance of "last" mirror and projection plane
#define E 55  // orthogonal distance of X and Y rotational axes

uint32_t xStepPos = 0, yStepPos = 0;
uint32_t xStepPosOld = 0, yStepPosOld = 0;
double angleX = 0, angleY = 0;

TMC2209 Xaxis, Yaxis;
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper *Xaxis_step;
FastAccelStepper *Yaxis_step;

// MultiStepper steppers;

// Flag variables to indicate end-stop activation
bool endStop1Triggered = false;
bool endStop2Triggered = false;

unsigned long lastTime = 0;
uint16_t maxSpeed = 20000 * micro_step / 256;
uint16_t xSpeed = 20000 * micro_step / 256; // Speed for X-axis
uint16_t ySpeed = 20000 * micro_step / 256; // Speed for Y-axis
uint16_t homeSpeed = 5000 * micro_step / 256;

static constexpr int steps_per_rot = 200 * micro_step;
double degrees_per_step = 360.00 / steps_per_rot;
static constexpr double STEPS_PER_RAD = (double)steps_per_rot / (2.0 * M_PI);
bool homingActive = false;

void homing();
void testMotors();
void move_steps(int xstep, int ystep);
void move_To(double x, double y);

// Interrupt service routines (ISRs)
void handleEndStop1()
{
  endStop1Triggered = true;
  // digitalWrite(en_x, HIGH); // Disable X-axis motor
}

void handleEndStop2()
{
  endStop2Triggered = true;
  // digitalWrite(en_y, HIGH); // Disable Y-axis motor
}

void setup()
{
  delay(2000); // Wait for 2 seconds before starting
  // put your setup code here, to run once:
  pinMode(endswitchX, INPUT_PULLUP); // Set end stop 1 pin as input with pull-up resistor
  pinMode(endswitchY, INPUT_PULLUP); // Set end stop 2 pin as input with pull-up resistor

  pinMode(en, OUTPUT);
  digitalWrite(en, LOW);

  // Set up the pins for the X-axis motor driver
  pinMode(step_x, OUTPUT);
  pinMode(dir_x, OUTPUT);
  // pinMode(diag_x, INPUT);

  digitalWrite(step_x, LOW);
  digitalWrite(dir_x, LOW);
  // Serial1.begin(115200, SERIAL_8N1, rx_x, tx_x);

  // Set up the pins for the Y-axis motor driver

  pinMode(step_y, OUTPUT);
  pinMode(dir_y, OUTPUT);
  // pinMode(diag_y, INPUT);

  digitalWrite(step_y, LOW);
  digitalWrite(dir_y, LOW);
  // Serial2.begin(115200, SERIAL_8N1, rx_y, tx_y);

  // Serial1.begin(115200, SERIAL_8N1, pdn_rx, pdn_tx);
  Serial2.begin(115200, SERIAL_8N1, 25, 26);
  delay(100);
  Xaxis.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0, pdn_rx, pdn_tx);
  Yaxis.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_1, pdn_rx, pdn_tx);

  // Attach interrupts to end-stop pins
  attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);

  //  // Initialize stepper motors
  stepperEngine.init();
  Xaxis_step = stepperEngine.stepperConnectToPin(step_x, FasDriver::RMT);
  if (Xaxis_step)
  {
    Xaxis_step->setDirectionPin(dir_x);
    Xaxis_step->setAutoEnable(false);
    Xaxis_step->setSpeedInUs(100);
    Xaxis_step->setAcceleration(2000);
  }
  else
  {
    Serial.println("Failed to connect X axis stepper to pin!");
    while (1)
      delay(10);
  }

  Yaxis_step = stepperEngine.stepperConnectToPin(step_y, FasDriver::RMT);
  if (Yaxis_step)
  {
    Yaxis_step->setDirectionPin(dir_y);
    Yaxis_step->setAutoEnable(false);
    Yaxis_step->setSpeedInUs(100);
    Yaxis_step->setAcceleration(2000);
  }
  else
  {
    Serial.println("Failed to connect Y axis stepper to pin!");
    while (1)
      delay(10);
  }

  digitalWrite(en, LOW); // Enable X-axis motor

  // steppers.addStepper(Xaxis_step);
  // steppers.addStepper(Yaxis_step);

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  if (Xaxis.isSetupAndCommunicating())
  {
    Serial.println("Xaxis Stepper driver is setup and communicating!");
    Serial2.println("Xaxis Stepper driver is setup and communicating!");
  }
  else if (Xaxis.isCommunicatingButNotSetup())
  {
    Serial.println("Xaxis Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    Serial2.println("Xaxis Stepper driver is communicating but not setup!");
    Serial2.println("Running setup again...");
    // stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Xaxis Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
    Serial2.println("Xaxis Stepper driver is not communicating!");
    Serial2.println("Try turning driver power on to see what happens.");
  }

  if (Yaxis.isSetupAndCommunicating())
  {
    Serial.println("Yaxis Stepper driver is setup and communicating!");
    Serial2.println("Yaxis Stepper driver is setup and communicating!");
  }
  else if (Yaxis.isCommunicatingButNotSetup())
  {
    Serial.println("Yaxis Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    Serial2.println("Yaxis Stepper driver is communicating but not setup!");
    Serial2.println("Running setup again...");
    // stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Yaxis Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
    Serial2.println("Yaxis Stepper driver is not communicating!");
    Serial2.println("Try turning driver power on to see what happens.");
  }

  delay(500);

  // homing();

  // move_To(500, 500); // Move to position (100, 100)

  // digitalWrite(en_x, HIGH); // Disable X-axis motor
  // digitalWrite(en_y, HIGH); // Disable Y-axis motor

  Xaxis.setMicrostepsPerStep(micro_step);
  Yaxis.setMicrostepsPerStep(micro_step); // Set microstepping to 1/256 for both axes

  Xaxis.setRunCurrent(100);
  Xaxis.enable();
  Xaxis.setHoldCurrent(100);

  Yaxis.setRunCurrent(100);
  Yaxis.enable();
  Yaxis.setHoldCurrent(100);

  Xaxis.enableCoolStep();
  Yaxis.enableCoolStep();

  // Xaxis.setStealthChopDurationThreshold(0);
  // Yaxis.setStealthChopDurationThreshold(0);

  // Xaxis.disableCoolStep();
  // Yaxis.disableCoolStep();

  // Xaxis.disableStealthChop();
  // Yaxis.disableStealthChop();

  // testMotors();

  // homing();
}

void sendXCommand(byte pCommand, int32_t pValue)
{
  Serial.print("#");
  Serial.print(",");
  Serial.print((char)pCommand);
  Serial.print(",");
  Serial.println((char)pValue);

  Serial2.print("#");
  Serial2.print(",");
  Serial2.print((char)pCommand);
  Serial2.print(",");
  Serial2.println((char)pValue);
}

#define CMD_ENDSTOP 'E'
#define CMD_ENDSTOP_X 'X'
#define CMD_ENDSTOP_Y 'Y'
#define CMD_HOME 'H'
#define CMD_HOME_X 'X'
#define CMD_HOME_Y 'Y'
#define CMD_MOVE_END 'M'
#define CMD_OK 'O'
#define CMD_ERROR 'F'

void testMotors()
{

  Xaxis_step->moveTo(12800, true);
}

void loop()
{

  if (endStop1Triggered)
  {                          // Endstop tetiklendi (LOW ise)
    Xaxis_step->forceStop(); // Motoru hemen durdur
    delay(10);
    endStop1Triggered = false; // Reset flag for future homing

    // Serial.println("Endstop X'e ulaşıldı.");
    sendXCommand(CMD_ENDSTOP, CMD_ENDSTOP_X);
  }

  if (endStop2Triggered)
  {                          // Endstop tetiklendi (LOW ise)
    Yaxis_step->forceStop(); // Motoru hemen durdur
    delay(10);
    endStop2Triggered = false; // Reset flag for future homing

    // Serial.println("Endstop Y'e ulaşıldı.");
    sendXCommand(CMD_ENDSTOP, CMD_ENDSTOP_Y);
  }

  if (Serial2.available())
  {
    char command = Serial2.read();
    if (command == 'H')
    {
      homingActive = true;
      homing(); // Call the homing function when 'h' is received
      homingActive = false;
    }
    else if (command == 'G')
    {
      int xdist = Serial2.parseInt();
      int ydist = Serial2.parseInt();
      // move_To(xdist, ydist); // Call the move_To function with the received coordinates
      move_steps(xdist, ydist); // Call the move_To function with the received coordinates
    }
    else if (command == 'Z')
    {
      Xaxis_step->setCurrentPosition(0);
      Yaxis_step->setCurrentPosition(0);
      // Serial.println("Position reset to (0,0)");
      sendXCommand(CMD_HOME, CMD_OK);
    }
  }

  if (Serial1.available())
  {
    char command = Serial1.read();
    if (command == 'H')
    {
      homingActive = true;
      homing(); // Call the homing function when 'h' is received
      homingActive = false;
    }
    else if (command == 'G')
    {
      int xdist = Serial1.parseInt();
      int ydist = Serial1.parseInt();
      // move_To(xdist, ydist); // Call the move_To function with the received coordinates
      move_steps(xdist, ydist); // Call the move_To function with the received coordinates
    }
    else if (command == 'Z')
    {
      Xaxis_step->setCurrentPosition(0);
      Yaxis_step->setCurrentPosition(0);
      // Serial.println("Position reset to (0,0)");
      sendXCommand(CMD_HOME, CMD_OK);
    }
  }
}

void homing()
{
  // detachInterrupt(digitalPinToInterrupt(endswitchX));
  // detachInterrupt(digitalPinToInterrupt(endswitchY));

  // Move X-axis towards end-stop
  Xaxis_step->setSpeedInHz(homeSpeed);
  Xaxis_step->moveTo(1000000, false); // Move a large negative distance

  while (endStop1Triggered == false)
    delay(1); // Endstop tetiklendi (LOW ise)
  Xaxis_step->stopMove();
  Xaxis_step->forceStopAndNewPosition(0); // Motoru hemen durdur
  // Xaxis_step->setCurrentPosition(0);  // Sıfırla (opsiyonel)

  // Serial.println("Endstop X'e ulaşıldı, Homeming tamamlandı");
  sendXCommand(CMD_HOME, CMD_HOME_X);

  delay(500);

  endStop1Triggered = false; // Reset flag for future homing

  Xaxis_step->moveTo(-homePosX, true);
  Xaxis_step->setCurrentPosition(0); // Sıfırla (opsiyonel)

  Yaxis_step->setSpeedInHz(homeSpeed);
  Yaxis_step->moveTo(1000000, false); // Move a large negative distance

  while (endStop2Triggered == false)
    delay(1); // Endstop tetiklendi (LOW ise)
  Yaxis_step->stopMove();
  Yaxis_step->forceStopAndNewPosition(0); // Motoru hemen durdur

  // Yaxis_step->setCurrentPosition(0);  // Sıfırla (opsiyonel)
  // Serial.println("Endstop Y'e ulaşıldı, Homeming tamamlandı");
  sendXCommand(CMD_HOME, CMD_HOME_Y);

  delay(500);

  endStop2Triggered = false; // Reset flag for future homing

  Yaxis_step->moveTo(-homePosY, true);
  Yaxis_step->setCurrentPosition(0); // Sıfırla (opsiyonel)

  sendXCommand(CMD_HOME, CMD_OK);

  // Attach interrupts to end-stop pins
  // attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  // attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);
}

void move_steps(int xstep, int ystep)
{
  Xaxis_step->moveTo(xstep);
  Yaxis_step->moveTo(ystep);

  while (Xaxis_step->isRunning() || Yaxis_step->isRunning())
  {
  }

  sendXCommand(CMD_MOVE_END, CMD_OK);
}

void move_To(double x, double y)
{
  // angleY = (atan(y / D)) * 57.2957795131;
  // angleX = (atan(x / (E + sqrt(pow(D, 2) + pow(y, 2))))) * 57.2957795131;
  // long targetX = round(angleX / degrees_per_step);
  // long targetY = round(angleY / degrees_per_step);

  const double root = sqrt(D * D + y * y);
  const double denomX = E + root;

  // 2) Açıları RADYAN cinsinden, atan2 ile hesapla
  const double angleY_rad = atan2(y, D);      // atan(y/D)
  const double angleX_rad = atan2(x, denomX); // atan(x/(E+sqrt(...)))

  // 3) Tek seferde, en yakına yuvarlayarak step’e çevir (truncation yerine lround)
  long targetX = lround(angleX_rad * STEPS_PER_RAD);
  long targetY = lround(angleY_rad * STEPS_PER_RAD);

  // Serial.println("X Pos: " + String(x));
  // Serial.println("Y Pos: " + String(y));
  // Serial.println("X Diff: " + String(targetX - Xaxis_step->getCurrentPosition()));
  // Serial.println("Y Diff: " + String(targetY - Yaxis_step->getCurrentPosition()));
  // Serial.println("X-axis steps: " + String(targetX));
  // Serial.println("Y-axis steps: " + String(targetY));
  // Serial.println("X angle: " + String(angleX));
  // Serial.println("Y angle: " + String(angleY));
  // Serial.println("-------------------------");

  // AccelStepper ile hedef pozisyonlara hareket et

  Xaxis_step->moveTo(targetX);
  Yaxis_step->moveTo(targetY);

  while (Xaxis_step->isRunning() || Yaxis_step->isRunning())
  {
  }

  sendXCommand(CMD_MOVE_END, CMD_OK);

  // İki ekseni aynı anda hareket ettir
  // while (Xaxis_step.distanceToGo() != 0 || Yaxis_step.distanceToGo() != 0)
  // {
  //   // Xaxis_step.run();
  //   // Yaxis_step.run();
  // }
}

double oldX = 0, oldY = 0;
void move_To_Diff(double x, double y)
{
  // angleY = (atan(y / D)) * 57.2957795131;
  // angleX = (atan(x / (E + sqrt(pow(D, 2) + pow(y, 2))))) * 57.2957795131;
  // long targetX = round(angleX / degrees_per_step);
  // long targetY = round(angleY / degrees_per_step);

  double diffX = x - oldX;
  double diffY = y - oldY;

  const double root = sqrt(D * D + diffY * diffY);
  const double denomX = E + root;

  // 2) Açıları RADYAN cinsinden, atan2 ile hesapla
  const double angleY_rad = atan2(diffY, D);      // atan(y/D)
  const double angleX_rad = atan2(diffX, denomX); // atan(x/(E+sqrt(...)))

  // 3) Tek seferde, en yakına yuvarlayarak step’e çevir (truncation yerine lround)
  long targetX = lround(angleX_rad * STEPS_PER_RAD);
  long targetY = lround(angleY_rad * STEPS_PER_RAD);

  // Serial.println("X Pos: " + String(x));
  // Serial.println("Y Pos: " + String(y));
  // Serial.println("X Diff: " + String(targetX - Xaxis_step->getCurrentPosition()));
  // Serial.println("Y Diff: " + String(targetY - Yaxis_step->getCurrentPosition()));
  // Serial.println("X-axis steps: " + String(targetX));
  // Serial.println("Y-axis steps: " + String(targetY));
  // Serial.println("X angle: " + String(angleX));
  // Serial.println("Y angle: " + String(angleY));
  Serial.println("-------------------------");

  long newTargetX = Xaxis_step->getCurrentPosition() + targetX;
  long newTargetY = Yaxis_step->getCurrentPosition() + targetY;

  // AccelStepper ile hedef pozisyonlara hareket et

  Xaxis_step->moveTo(newTargetX);
  Yaxis_step->moveTo(newTargetY);

  while (Xaxis_step->isRunning() || Yaxis_step->isRunning())
  {
  }

  oldX = x;
  oldY = y;

  // İki ekseni aynı anda hareket ettir
  // while (Xaxis_step.distanceToGo() != 0 || Yaxis_step.distanceToGo() != 0)
  // {
  //   // Xaxis_step.run();
  //   // Yaxis_step.run();
  // }
}