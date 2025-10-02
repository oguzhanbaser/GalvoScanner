#include <Arduino.h>
#include <TMC2209.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define endswitchX 12
#define endswitchY 14

#define step_x 21
#define dir_x 20
#define en_x 37
// #define rst_x 47
// #define m3_x 48
#define rx_x 3
#define tx_x 47
#define m2_x 35
#define m1_x 36

#define step_y 39
#define dir_y 38
#define en_y 18
// #define rst_y 40
// #define m3_y 41
#define rx_y 41
#define tx_y 40
#define m2_y 1
#define m1_y 8

#define micro_step 256

#define homePosX 6415 * micro_step / 256 // Tweak this to get a perfect 45 deg angle as 0 position
#define homePosY 6410 * micro_step / 256 // Tweak this to get a perfect 45 deg angle as 0 position

TMC2209 Xaxis, Yaxis;
AccelStepper Xaxis_step(1, step_x, dir_x);
AccelStepper Yaxis_step(1, step_y, dir_y);
MultiStepper steppers;

// Flag variables to indicate end-stop activation
bool endStop1Triggered = false;
bool endStop2Triggered = false;

unsigned long lastTime = 0;
uint16_t maxSpeed = 3000 * micro_step / 256;
uint16_t xSpeed = 300 * micro_step / 256; // Speed for X-axis
uint16_t ySpeed = 300 * micro_step / 256; // Speed for Y-axis
uint16_t homeSpeed = 3000 * micro_step / 256;

int steps_per_rot = 200 * micro_step;
double degrees_per_step = 360.00 / steps_per_rot;

void homing();
void testMotors();
void move_To(double x, double y);

// Şu anki pozisyonları oku (isteğe bağlı olarak global değişkenle de tutabilirsin)
long currX = 0;
long currY = 0;

struct XY
{
  double x;
  double y;
};
struct XY currPos;

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

  // Set up the pins for the X-axis motor driver
  pinMode(en_x, OUTPUT);
  pinMode(m2_x, OUTPUT);
  pinMode(m1_x, OUTPUT);
  pinMode(step_x, OUTPUT);
  pinMode(dir_x, OUTPUT);

  digitalWrite(en_x, LOW);
  digitalWrite(m2_x, LOW);
  digitalWrite(m1_x, LOW);
  digitalWrite(step_x, LOW);
  digitalWrite(dir_x, LOW);
  // Serial1.begin(115200, SERIAL_8N1, rx_x, tx_x);

  // Set up the pins for the Y-axis motor driver
  pinMode(en_y, OUTPUT);
  pinMode(m2_y, OUTPUT);
  pinMode(m1_y, OUTPUT);
  pinMode(step_y, OUTPUT);
  pinMode(dir_y, OUTPUT);

  digitalWrite(en_y, LOW);
  digitalWrite(m2_y, LOW);
  digitalWrite(m1_y, LOW);
  digitalWrite(step_y, LOW);
  digitalWrite(dir_y, LOW);
  // Serial2.begin(115200, SERIAL_8N1, rx_y, tx_y);

  Xaxis.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0, rx_x, tx_x);
  Yaxis.setup(Serial2, 115200, TMC2209::SERIAL_ADDRESS_0, rx_y, tx_y);

  // Attach interrupts to end-stop pins
  attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);

  //  // Initialize stepper motors
  Xaxis_step.setMaxSpeed(maxSpeed);
  Xaxis_step.setSpeed(xSpeed);
  Xaxis_step.setCurrentPosition(0);
  Yaxis_step.setMaxSpeed(maxSpeed);
  Yaxis_step.setSpeed(ySpeed);
  Yaxis_step.setCurrentPosition(0);

  steppers.addStepper(Xaxis_step);
  steppers.addStepper(Yaxis_step);

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  if (Xaxis.isSetupAndCommunicating())
  {
    Serial.println("Xaxis Stepper driver is setup and communicating!");
  }
  else if (Xaxis.isCommunicatingButNotSetup())
  {
    Serial.println("Xaxis Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    // stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Xaxis Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }

  if (Yaxis.isSetupAndCommunicating())
  {
    Serial.println("Yaxis Stepper driver is setup and communicating!");
  }
  else if (Yaxis.isCommunicatingButNotSetup())
  {
    Serial.println("Yaxis Stepper driver is communicating but not setup!");
    Serial.println("Running setup again...");
    // stepper_driver.setup(serial_stream);
  }
  else
  {
    Serial.println("Yaxis Stepper driver is not communicating!");
    Serial.println("Try turning driver power on to see what happens.");
  }

  delay(2000);

  // testMotors(); // Test motors by moving to position 10000
  // homing();

  // move_To(500, 500); // Move to position (100, 100)

  // digitalWrite(en_x, HIGH); // Disable X-axis motor
  // digitalWrite(en_y, HIGH); // Disable Y-axis motor

  Xaxis.setMicrostepsPerStep(micro_step);
  Yaxis.setMicrostepsPerStep(micro_step); // Set microstepping to 1/256 for both axes

  Xaxis.setRunCurrent(50);
  Xaxis.enable();
  Xaxis.setHoldCurrent(50);

  Yaxis.setRunCurrent(50);
  Yaxis.enable();
  Yaxis.setHoldCurrent(50);

  Xaxis.enableCoolStep();
  Yaxis.enableCoolStep();

  Xaxis.setStealthChopDurationThreshold(0);
  Yaxis.setStealthChopDurationThreshold(0);

  // Xaxis.disableCoolStep();
  // Yaxis.disableCoolStep();

  Xaxis.disableStealthChop();
  Yaxis.disableStealthChop();

  // for(uint32_t i = 0; i < 12800 * 2; i++)
  // {
  //   digitalWrite(step_x, !digitalRead(step_x)); // Toggle direction for X-axis
  //   delayMicroseconds(10);
  // }

  // homing();

  Xaxis_step.setSpeed(xSpeed);
  Yaxis_step.setSpeed(ySpeed);
}

void testMotors()
{

  Yaxis.setRunCurrent(100);
  Yaxis.enableCoolStep();
  Yaxis.enable();
  Yaxis.moveAtVelocity(-50000);

  Xaxis.setRunCurrent(100);
  Xaxis.enableCoolStep();
  Xaxis.enable();
  Xaxis.moveAtVelocity(-50000);

  while (true)
  {
    if (digitalRead(endswitchX) == LOW)
    {
      Xaxis.disable();
      Serial.println("X-axis end switch triggered");
    }

    if (digitalRead(endswitchY) == LOW)
    {
      Yaxis.disable();
      Serial.println("Y-axis end switch triggered");
    }
  }
}

void loop()
{

  if (Serial.available())
  {
    char command = Serial.read();
    if (command == 'H')
    {
      homing(); // Call the homing function when 'h' is received
    }
    else if (command == 'G')
    {
      int xdist = Serial.parseInt();
      int ydist = Serial.parseInt();
      move_To(xdist, ydist); // Call the move_To function with the received coordinates
    }
  }

  // move_To(-180, -180);
  // delay(5);

  // for(int i = -180; i <= 0; i++)
  // {
  //   move_To(i, i);
  //   delay(20);
  // }

  // move_To(-100, 0);
  // delay(5);

  // Xaxis.run(); // Run the X-axis motor
  // Yaxis.run(); // Run the Y-axis motor
}

int stepSpeed = 100;
void homing()
{
  detachInterrupt(digitalPinToInterrupt(endswitchX));
  detachInterrupt(digitalPinToInterrupt(endswitchY));

  Xaxis_step.setSpeed(homeSpeed);

  while (digitalRead(endswitchX) == HIGH)
  {
    Xaxis_step.runSpeed();
  }

  Serial.println("X-axis end switch triggered");

  Xaxis_step.setCurrentPosition(0);
  Xaxis_step.moveTo(-homePosX);
  Xaxis_step.setSpeed(-homeSpeed);

  while (Xaxis_step.distanceToGo() != 0)
  {
    Xaxis_step.runSpeed();
  }
  Xaxis_step.setCurrentPosition(0);
  Serial.println("X-axis homing complete");

  Yaxis_step.setSpeed(homeSpeed);
  while (digitalRead(endswitchY) == HIGH)
  {
    Yaxis_step.runSpeed();
  }
  Serial.println("Y-axis end switch triggered");

  Yaxis_step.setCurrentPosition(0);
  Yaxis_step.moveTo(-homePosY);
  Yaxis_step.setSpeed(-homeSpeed);
  while (Yaxis_step.distanceToGo() != 0)
  {
    Yaxis_step.runSpeed();
  }
  Yaxis_step.setCurrentPosition(0);
  Serial.println("Y-axis homing complete");

  currX = 0;
  currY = 0;

  // Attach interrupts to end-stop pins
  attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);

  Xaxis_step.setSpeed(xSpeed);
  Yaxis_step.setSpeed(ySpeed);
}

#define D 95 // orthogonal distance of "last" mirror and projection plane
#define E 19 // orthogonal distance of X and Y rotational axes

uint32_t xStepPos = 0, yStepPos = 0;
uint32_t xStepPosOld = 0, yStepPosOld = 0;

void move_To(double x, double y)
{

  struct XY angle;
  angle.y = (atan(y / D)) * 57.2957795131;
  angle.x = (atan(x / (E + sqrt(pow(D, 2) + pow(y, 2))))) * 57.2957795131;
  long targetX = (long)(angle.x / degrees_per_step);
  long targetY = (long)(angle.y / degrees_per_step);

  Serial.println("X Angle: " + String(x));
  Serial.println("Y Angle: " + String(y));
  Serial.println("X-axis steps: " + String(targetX));
  Serial.println("Y-axis steps: " + String(targetY));
  Serial.println("X angle: " + String(angle.x));
  Serial.println("Y angle: " + String(angle.y));
  Serial.println("-------------------------");

  long dx = abs(targetX - currX);
  long dy = abs(targetY - currY);
  int sx = (targetX > currX) ? 1 : -1;
  int sy = (targetY > currY) ? 1 : -1;

  digitalWrite(dir_x, (sx > 0) ? HIGH : LOW);
  digitalWrite(dir_y, (sy > 0) ? HIGH : LOW);

  long err = dx - dy;
  long e2;

  while (currX != targetX || currY != targetY)
  {
    e2 = 2 * err;
    if (e2 > -dy && currX != targetX)
    {
      err -= dy;
      currX += sx;
      digitalWrite(step_x, HIGH);
      delayMicroseconds(1500); // Hız ayarı
      digitalWrite(step_x, LOW);
      delayMicroseconds(1500);
    }
    if (e2 < dx && currY != targetY)
    {
      err += dx;
      currY += sy;
      digitalWrite(step_y, HIGH);
      delayMicroseconds(1500);
      digitalWrite(step_y, LOW);
      delayMicroseconds(1500);
    }
  }

}
