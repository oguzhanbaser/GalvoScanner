#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define endswitchX 12
#define endswitchY 14

#define step_x 21
#define dir_x 20
#define en_x 37
#define rst_x 47
#define m3_x 48
#define m2_x 35
#define m1_x 36

#define step_y 39
#define dir_y 38
#define en_y 18
#define rst_y 40
#define m3_y 41
#define m2_y 1
#define m1_y 8

#define homePosX 1581 // Tweak this to get a perfect 45 deg angle as 0 position
#define homePosY 1095 // Tweak this to get a perfect 45 deg angle as 0 position

#define micro_step 32
#define D 170 // orthogonal distance of "last" mirror and projection plane
#define E 19  // orthogonal distance of X and Y rotational axes

AccelStepper Xaxis(1, step_x, dir_x);
AccelStepper Yaxis(1, step_y, dir_y);
MultiStepper steppers;

// Flag variables to indicate end-stop activation
bool endStop1Triggered = false;
bool endStop2Triggered = false;

unsigned long lastTime = 0;
uint16_t xSpeed = 300; // Speed for X-axis
uint16_t ySpeed = 300; // Speed for Y-axis
uint16_t homeSpeed = 300;

int steps_per_rot = 200 * micro_step;
double degrees_per_step = 360.00 / steps_per_rot;

void homing();
void testMotors();
void move_To(double x, double y);

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
  // put your setup code here, to run once:
  pinMode(endswitchX, INPUT_PULLUP); // Set end stop 1 pin as input with pull-up resistor
  pinMode(endswitchY, INPUT_PULLUP); // Set end stop 2 pin as input with pull-up resistor

   // Set up the pins for the X-axis motor driver
  pinMode(en_x, OUTPUT);
  pinMode(rst_x, OUTPUT);
  pinMode(m3_x, OUTPUT);
  pinMode(m2_x, OUTPUT);
  pinMode(m1_x, OUTPUT);

  digitalWrite(en_x, LOW);
  digitalWrite(rst_x, HIGH);
  digitalWrite(m3_x, HIGH);
  digitalWrite(m2_x, HIGH);
  digitalWrite(m1_x, HIGH);

  // Set up the pins for the Y-axis motor driver
  pinMode(en_y, OUTPUT);
  pinMode(rst_y, OUTPUT);
  pinMode(m3_y, OUTPUT);
  pinMode(m2_y, OUTPUT);
  pinMode(m1_y, OUTPUT);

  digitalWrite(en_y, LOW);
  digitalWrite(rst_y, HIGH);
  digitalWrite(m3_y, HIGH);
  digitalWrite(m2_y, HIGH);
  digitalWrite(m1_y, HIGH);

  // Attach interrupts to end-stop pins
  attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  //  // Initialize stepper motors
  Xaxis.setMaxSpeed(xSpeed); // Set maximum speed for X-axis
  // Xaxis.setAcceleration(500); // Set acceleration for X-axis
  Xaxis.setSpeed(300);
  Xaxis.setCurrentPosition(0); // Set current position of X-axis to 0

  Yaxis.setMaxSpeed(ySpeed); // Set maximum speed for Y-axis
  // Yaxis.setAcceleration(300); // Set acceleration for Y-axis
  Yaxis.setSpeed(ySpeed);
  Yaxis.setCurrentPosition(0); // Set current position of Y-axis to 0

  steppers.addStepper(Xaxis);
  steppers.addStepper(Yaxis);

  //  // Start moving motors
  // Xaxis.moveTo(10000); // Move X-axis to position 10000
  // Yaxis.moveTo(10000); // Move Y-axis to position 10000

  // testMotors(); // Test motors by moving to position 10000
  // homing();

  // move_To(500, 500); // Move to position (100, 100)
}

void testMotors()
{
  Xaxis.setMaxSpeed(3000); 
  Yaxis.setMaxSpeed(500);

  Xaxis.setSpeed(3000); // Set speed for X-axis
  Yaxis.setSpeed(500);

  while (true)
  {
    // Xaxis.runSpeed();
    Yaxis.runSpeed();
  }
}

void loop()
{
  if(Serial.available())
  {
    char command = Serial.read();
    if(command == 'H')
    {
      homing(); // Call the homing function when 'h' is received
    }
    else if(command == 'G')
    {
      int xdist = Serial.parseInt();
      int ydist = Serial.parseInt();
      move_To(xdist, ydist); // Call the move_To function with the received coordinates
    }
  }
  // if (endStop1Triggered)
  // {
  //   Serial.println("End Stop 1 Triggered!");
  //   Xaxis.stop();              // Stop the X-axis motor
  //   endStop1Triggered = false; // Reset the flag
  // }

  // if (endStop2Triggered)
  // {
  //   Serial.println("End Stop 2 Triggered!");
  //   Yaxis.stop();              // Stop the Y-axis motor
  //   endStop2Triggered = false; // Reset the flag
  // }

  if (Xaxis.distanceToGo() == 0)
  {
    digitalWrite(en_x, HIGH); // Disable X-axis motor
  }

  if (Yaxis.distanceToGo() == 0)
  {
    digitalWrite(en_y, HIGH); // Disable Y-axis motor
  }

  // Xaxis.run(); // Run the X-axis motor
  // Yaxis.run(); // Run the Y-axis motor
}

void homing()
{
  detachInterrupt(digitalPinToInterrupt(endswitchX));
  detachInterrupt(digitalPinToInterrupt(endswitchY));

  digitalWrite(en_x, LOW); // Enable X-axis motor
  digitalWrite(en_y, LOW); // Enable Y-axis motor

  int prevXmaxSpeed = Xaxis.maxSpeed();
  int prevYmaxSpeed = Yaxis.maxSpeed();
  int prevXspeed = Xaxis.speed();
  int prevYspeed = Yaxis.speed();
  Xaxis.setMaxSpeed(homeSpeed);
  Yaxis.setMaxSpeed(homeSpeed);

  Xaxis.setSpeed(-homeSpeed);

  while (digitalRead(endswitchX) == HIGH)
  {
    Xaxis.runSpeed();
  }

  // Serial.println("X-axis end switch triggered");

  Xaxis.setCurrentPosition(0);
  Xaxis.moveTo(homePosX);
  Xaxis.setSpeed(homeSpeed);

  while (Xaxis.distanceToGo() != 0)
  {
    Xaxis.runSpeed();
  }
  Xaxis.setCurrentPosition(0);

  // Serial.println("X-axis homing complete");

  Yaxis.setSpeed(-homeSpeed);

  while (digitalRead(endswitchY) == HIGH)
  {
    Yaxis.runSpeed();
  }
  // Serial.println("Y-axis end switch triggered");

  Yaxis.setCurrentPosition(0);
  Yaxis.moveTo(homePosY);
  Yaxis.setSpeed(homeSpeed);
  while (Yaxis.distanceToGo() != 0)
  {
    Yaxis.runSpeed();
  }
  Yaxis.setCurrentPosition(0);
  // Serial.println("Y-axis homing complete");

  Xaxis.setMaxSpeed(prevXmaxSpeed);
  Yaxis.setMaxSpeed(prevYmaxSpeed);
  Xaxis.setSpeed(prevXspeed);
  Yaxis.setSpeed(prevYspeed);
  currPos.x = 0.0;
  currPos.y = 0.0;

  // Attach interrupts to end-stop pins
  attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);
}

void move_To(double x, double y)
{
  struct XY angle;
  angle.y = (atan(y / D) / 2) * 57.2957795131;
  angle.x = (atan(x / (E + sqrt(pow(D, 2) + pow(y, 2)))) / 2) * 57.2957795131;
  long step_pos[2];
  step_pos[0] = (long)(angle.x / degrees_per_step);
  step_pos[1] = (long)(angle.y / degrees_per_step);

  Serial.println("X Angle: " + String(x));
  Serial.println("Y Angle: " + String(y));
  Serial.println("X-axis steps: " + String(step_pos[0]));
  Serial.println("Y-axis steps: " + String(step_pos[1]));
  Serial.println("X angle: " + String(angle.x));
  Serial.println("Y angle: " + String(angle.y));
  Serial.println("-------------------------");

  digitalWrite(en_x, LOW); // Enable X-axis motor
  digitalWrite(en_y, LOW); // Enable Y-axis motor
  steppers.moveTo(step_pos);
  steppers.runSpeedToPosition();
}
