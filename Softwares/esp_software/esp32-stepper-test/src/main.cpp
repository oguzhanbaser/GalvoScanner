#include <Arduino.h>
#include <MultiStepper.h>
#include <FastAccelStepper.h>
#include <math.h>

#define endswitchX 12
#define endswitchY 14

#define step_x 21
#define dir_x 20
#define en_x 37
// #define rst_x 47
#define m3_x 3
// #define rx_x 3
// #define tx_x 47
#define m2_x 35
#define m1_x 36
#define reset_x 47
// #define sleep_x 48

#define step_y 39
#define dir_y 38
#define en_y 18
// #define rst_y 40
#define m3_y 41
// #define rx_y 41
// #define tx_y 40
#define m2_y 1
#define m1_y 8
#define reset_y 40
// #define sleep_y 41

#define micro_step 32

#define homePosX 8000 * micro_step / 256 // Tweak this to get a perfect 45 deg angle as 0 position
#define homePosY 6000 * micro_step / 256 // Tweak this to get a perfect 45 deg angle as 0 position

#define D 95 // orthogonal distance of "last" mirror and projection plane
#define E 19 // orthogonal distance of X and Y rotational axes

uint32_t xStepPos = 0, yStepPos = 0;
uint32_t xStepPosOld = 0, yStepPosOld = 0;
double angleX = 0, angleY = 0;

// TMC2209 Xaxis, Yaxis; // Removed for DRV8825
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper* Xaxis_step;
FastAccelStepper* Yaxis_step;

// MultiStepper steppers;

// Flag variables to indicate end-stop activation
bool endStop1Triggered = false;
bool endStop2Triggered = false;

unsigned long lastTime = 0;
uint16_t maxSpeed = 5000 * micro_step / 256;
uint16_t xSpeed = 5000 * micro_step / 256; // Speed for X-axis
uint16_t ySpeed = 5000 * micro_step / 256; // Speed for Y-axis
uint16_t homeSpeed = 2000 * micro_step / 256;

static constexpr int steps_per_rot = 200 * micro_step;
double degrees_per_step = 360.00 / steps_per_rot;
static constexpr double STEPS_PER_RAD = (double)steps_per_rot / (2.0 * M_PI);
bool homingActive = false;

void homing();
void testMotors();
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

  // Set up the pins for the X-axis motor driver
  pinMode(en_x, OUTPUT);
  pinMode(m2_x, OUTPUT);
  pinMode(m1_x, OUTPUT);
  pinMode(step_x, OUTPUT);
  pinMode(dir_x, OUTPUT);
  pinMode(reset_x, OUTPUT);
  pinMode(m3_x, OUTPUT);


  digitalWrite(en_x, LOW);
  digitalWrite(m2_x, LOW);
  digitalWrite(m1_x, LOW);
  digitalWrite(step_x, LOW);
  digitalWrite(dir_x, LOW);
  digitalWrite(reset_x, HIGH);
  digitalWrite(m3_x, LOW);
  // Serial1.begin(115200, SERIAL_8N1, rx_x, tx_x); // Not needed for DRV8825

  // Set up the pins for the Y-axis motor driver
  pinMode(en_y, OUTPUT);
  pinMode(m2_y, OUTPUT);
  pinMode(m1_y, OUTPUT);
  pinMode(step_y, OUTPUT);
  pinMode(dir_y, OUTPUT);
  pinMode(reset_y, OUTPUT);
  pinMode(m3_y, OUTPUT);


  digitalWrite(en_y, LOW);
  digitalWrite(m2_y, LOW);
  digitalWrite(m1_y, LOW);
  digitalWrite(step_y, LOW);
  digitalWrite(dir_y, LOW);
  digitalWrite(reset_y, HIGH);
  digitalWrite(m3_y, LOW);
  // Serial2.begin(115200, SERIAL_8N1, rx_y, tx_y); // Not needed for DRV8825

  // Xaxis.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0, rx_x, tx_x); // Removed for DRV8825
  // Yaxis.setup(Serial2, 115200, TMC2209::SERIAL_ADDRESS_0, rx_y, tx_y); // Removed for DRV8825

  // Attach interrupts to end-stop pins
  attachInterrupt(digitalPinToInterrupt(endswitchX), handleEndStop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(endswitchY), handleEndStop2, FALLING);

  //  // Initialize stepper motors
  stepperEngine.init();
  Xaxis_step = stepperEngine.stepperConnectToPin(step_x, FasDriver::RMT); // DRV8825 stepper connection
  if(Xaxis_step)
  {
    Xaxis_step->setDirectionPin(dir_x);
    Xaxis_step->setEnablePin(en_x);
    Xaxis_step->setAutoEnable(false);
    Xaxis_step->setSpeedInUs(100);
    Xaxis_step->setAcceleration(2000);
  }else{
    Serial.println("Failed to connect X axis stepper to pin!");
  }

  Yaxis_step = stepperEngine.stepperConnectToPin(step_y, FasDriver::RMT); // DRV8825 stepper connection
  if(Yaxis_step)
  {
    Yaxis_step->setDirectionPin(dir_y);
    Yaxis_step->setEnablePin(en_y);
    Yaxis_step->setAutoEnable(false);
    Yaxis_step->setSpeedInUs(100);
    Yaxis_step->setAcceleration(2000);
  }else{
    Serial.println("Failed to connect Y axis stepper to pin!");
  }

  digitalWrite(m1_x, HIGH);
  digitalWrite(m2_x, HIGH);
  digitalWrite(m3_x, HIGH);

  digitalWrite(m1_y, HIGH);
  digitalWrite(m2_y, HIGH);
  digitalWrite(m3_y, HIGH);

  digitalWrite(en_x, LOW); // Enable X-axis motor
  digitalWrite(en_y, LOW); // Enable Y-axis motor

  // steppers.addStepper(Xaxis_step);
  // steppers.addStepper(Yaxis_step);

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate


  // DRV8825 does not require driver communication checks

  delay(500);

  // homing();

  // move_To(500, 500); // Move to position (100, 100)

  // digitalWrite(en_x, HIGH); // Disable X-axis motor
  // digitalWrite(en_y, HIGH); // Disable Y-axis motor


  // DRV8825 does not support microstep, current, or coolStep configuration via code

  // testMotors();

  // homing();


}

void testMotors()
{

  Xaxis_step->moveTo(12800, true);
  Yaxis_step->moveTo(12800, true);
}

void loop()
{

  if (endStop1Triggered) { // Endstop tetiklendi (LOW ise)
    Xaxis_step->forceStop();         // Motoru hemen durdur
    endStop1Triggered = false; // Reset flag for future homing

    Serial.println("Endstop X'e ulaşıldı.");
  }

  if (endStop2Triggered) { // Endstop tetiklendi (LOW ise)
    Yaxis_step->forceStop();         // Motoru hemen durdur
    endStop2Triggered = false; // Reset flag for future homing

    Serial.println("Endstop Y'e ulaşıldı.");
  }

  if (Serial.available())
  {
    char command = Serial.read();
    if (command == 'H')
    {
      homingActive = true;
      homing(); // Call the homing function when 'h' is received
      homingActive = false;
    }
    else if (command == 'G')
    {
      int xdist = Serial.parseInt();
      int ydist = Serial.parseInt();
      move_To(xdist, ydist); // Call the move_To function with the received coordinates
    }else if(command == 'T')
    {
      int xval = Serial.parseInt();
      int yval = Serial.parseInt();

    }
  }
}

void homing()
{
  // ...existing code...

  // Move X-axis towards end-stop
  Xaxis_step->setSpeedInHz(homeSpeed);
  Xaxis_step->moveTo(-1000000, false); // Move a large negative distance

  while (endStop1Triggered == false) delay(1); // Endstop tetiklendi (LOW ise)
  Xaxis_step->stopMove();
  Xaxis_step->forceStopAndNewPosition(0);         // Motoru hemen durdur
  // Xaxis_step->setCurrentPosition(0);  // Sıfırla (opsiyonel)

  Serial.println("Endstop X'e ulaşıldı, Homeming tamamlandı");

  delay(500);

  endStop1Triggered = false; // Reset flag for future homing

  Xaxis_step->moveTo(homePosX, true);  
  Xaxis_step->setCurrentPosition(0);  // Sıfırla (opsiyonel)

  Yaxis_step->setSpeedInHz(homeSpeed);
  Yaxis_step->moveTo(-1000000, false); // Move a large negative distance

  while (endStop2Triggered == false) delay(1); // Endstop tetiklendi (LOW ise)
  Yaxis_step->stopMove();
  Yaxis_step->forceStopAndNewPosition(0);         // Motoru hemen durdur

  // Yaxis_step->setCurrentPosition(0);  // Sıfırla (opsiyonel)
  Serial.println("Endstop Y'e ulaşıldı, Homeming tamamlandı");

  delay(500);

  endStop2Triggered = false; // Reset flag for future homing

  Yaxis_step->moveTo(homePosY, true);
  Yaxis_step->setCurrentPosition(0);  // Sıfırla (opsiyonel)

  // ...existing code...

}



void move_To(double x, double y)
{
  // angleY = (atan(y / D)) * 57.2957795131;
  // angleX = (atan(x / (E + sqrt(pow(D, 2) + pow(y, 2))))) * 57.2957795131;
  // long targetX = round(angleX / degrees_per_step);
  // long targetY = round(angleY / degrees_per_step);

  const double root = sqrt(D*D + y*y);
  const double denomX = E + root;

  // 2) Açıları RADYAN cinsinden, atan2 ile hesapla
  const double angleY_rad = atan2(y, D);          // atan(y/D)
  const double angleX_rad = atan2(x, denomX);     // atan(x/(E+sqrt(...)))

  // 3) Tek seferde, en yakına yuvarlayarak step’e çevir (truncation yerine lround)
  long targetX = lround(angleX_rad * STEPS_PER_RAD);
  long targetY = lround(angleY_rad * STEPS_PER_RAD);

  Serial.println("New X pos: " + String(x));
  Serial.println("New Y pos: " + String(y));
  Serial.println("Pos Diff X: " + String(targetX - Xaxis_step->getCurrentPosition()));
  Serial.println("Pos Diff Y: " + String(targetY - Yaxis_step->getCurrentPosition()));
  Serial.println("X-axis steps: " + String(targetX));
  Serial.println("Y-axis steps: " + String(targetY));
  Serial.println("X angle in degree: " + String(angleX_rad * 180 / PI));
  Serial.println("Y angle in degree: " + String(angleY_rad * 180 / PI));
  Serial.println("-------------------------");



  // AccelStepper ile hedef pozisyonlara hareket et

  Xaxis_step->moveTo(targetX);
  Yaxis_step->moveTo(targetY);

  while (Xaxis_step->isRunning() || Yaxis_step->isRunning())
  {

  }

  // İki ekseni aynı anda hareket ettir
  // while (Xaxis_step.distanceToGo() != 0 || Yaxis_step.distanceToGo() != 0)
  // {
  //   // Xaxis_step.run();
  //   // Yaxis_step.run();
  // }
} 
