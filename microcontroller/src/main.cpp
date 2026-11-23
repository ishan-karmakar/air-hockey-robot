#include <Arduino.h>
#include <TMC2209.h>
#include <FastAccelStepper.h>

#define SETUP_MOTOR(X, x) setup_motor(x ## _driver, x ## _stepper, X ## _SERIAL_ADDR, X ## _STEP_PIN, X ## _DIR_PIN)

constexpr int BAUD_RATE = 115200;

constexpr int RX_PIN = 22;
constexpr int TX_PIN = 12;

constexpr int RUN_CURRENT_PERCENT = 100;
constexpr int HOLD_CURRENT_PERCENT = 5;
constexpr int MAX_VELOCITY = 100;
constexpr int MAX_ACCEL = 100;
constexpr int REPLY_DELAY = 4;
constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEPS = 256;
constexpr int STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;

constexpr double TABLE_WIDTH = 0.889;
constexpr double TABLE_HEIGHT = 0.9144;
constexpr double PADDLE_ATTACHMENT_RADIUS = 0.023;
constexpr double SPOOL_RADIUS = 0.0125;
constexpr double CABLE_HEIGHT_DIFFERENCE = 0.004;

// Back Left Motor
constexpr TMC2209::SerialAddress BL_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_1;
constexpr bool BL_INVERSION = false;
constexpr int BL_DIR_PIN = 0;
constexpr int BL_STEP_PIN = 1;

// Back Right Motor
constexpr TMC2209::SerialAddress BR_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_2;
constexpr bool BR_INVERSION = false;
constexpr int BR_DIR_PIN = -1;
constexpr int BR_STEP_PIN = -1;

// Front Left Motor
constexpr TMC2209::SerialAddress FL_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_3;
constexpr bool FL_INVERSION = false;
constexpr int FL_DIR_PIN = -1;
constexpr int FL_STEP_PIN = -1;

// Front Right Motor
constexpr TMC2209::SerialAddress FR_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_0;
constexpr bool FR_INVERSION = false;
constexpr int FR_DIR_PIN = 10;
constexpr int FR_STEP_PIN = 11;

struct CableLengths {
  double bl;
  double br;
  double fl;
  double fr;
};

struct MotorPositions {
  long bl;
  long br;
  long fl;
  long fr;
};

FastAccelStepperEngine engine = FastAccelStepperEngine();

TMC2209 bl_driver;
FastAccelStepper *bl_stepper;

TMC2209 br_driver;
FastAccelStepper *br_stepper;

TMC2209 fl_driver;
FastAccelStepper *fl_stepper;

TMC2209 fr_driver;
FastAccelStepper *fr_stepper;

CableLengths initial_lengths;
MotorPositions initial_motor_positions;

void setup_motor(TMC2209& driver, FastAccelStepper*& stepper, TMC2209::SerialAddress addr, int step, int dir) {
  driver.setup(Serial1, BAUD_RATE, addr, RX_PIN, TX_PIN);
  driver.setReplyDelay(REPLY_DELAY);
  driver.setRunCurrent(RUN_CURRENT_PERCENT);
  // driver.setHoldCurrent(HOLD_CURRENT_PERCENT);
  driver.setMicrostepsPerStep(MICROSTEPS);
  driver.disableStealthChop();
  driver.enable();

  stepper = engine.stepperConnectToPin(step);
  if (stepper) {
    stepper->setDirectionPin(dir);
    stepper->setSpeedInHz(MAX_VELOCITY * MICROSTEPS);
    stepper->setAcceleration(MAX_ACCEL * MICROSTEPS);
  }
}

bool check_motors_communicating() {
  if (!bl_driver.isSetupAndCommunicating()) {
    Serial.println("Back left stepper driver not setup and communicating!");
    // return false;
  }
  // if (!br_driver.isSetupAndCommunicating()) {
  //   Serial.println("Back right stepper driver not setup and communicating!");
  //   return false;
  // }
  // if (!fl_driver.isSetupAndCommunicating()) {
  //   Serial.println("Front left stepper driver not setup and communicating!");
  //   return false;
  // }
  if (!fr_driver.isSetupAndCommunicating()) {
    Serial.println("Front right stepper driver not setup and communicating!");
    // return false;
  }
  return true;
}

void log_motor(const char *name, FastAccelStepper *stepper) {
  Serial.print(name);
  Serial.print(" - Position: ");
  Serial.print(stepper->getCurrentPosition());
  Serial.print(", Velocity: ");
  Serial.print(stepper->getCurrentSpeedInMilliHz(true));
  Serial.print(", Acceleration: ");
  Serial.println(stepper->getCurrentAcceleration());
}

CableLengths calculate_cable_lengths(double x, double y) {
  double diagonal = PADDLE_ATTACHMENT_RADIUS / std::sqrt(2.0);

  double bl_x = x - diagonal;
  double bl_y = y - diagonal;

  double br_x = x + diagonal;
  double br_y = y - diagonal;

  double fl_x = x - diagonal;
  double fl_y = y + diagonal;

  double fr_x = x + diagonal;
  double fr_y = y + diagonal;

  double dz = CABLE_HEIGHT_DIFFERENCE;

  return {
    .bl = std::sqrt(bl_x * bl_x + bl_y * bl_y + dz * dz),
    .br = std::sqrt((TABLE_WIDTH - br_x) * (TABLE_WIDTH - br_x) + br_y * br_y + dz * dz),
    .fl = std::sqrt(fl_x * fl_x + (TABLE_HEIGHT - fl_y) * (TABLE_HEIGHT - fl_y) + dz * dz),
    .fr = std::sqrt((TABLE_WIDTH - fr_x) * (TABLE_WIDTH - fr_x) + (TABLE_HEIGHT - fr_y) * (TABLE_HEIGHT - fr_y) + dz * dz),
  };
}

void initialize_reference(double x, double y) {
  initial_lengths = calculate_cable_lengths(x, y);

  initial_motor_positions = {
    .bl = bl_stepper->getCurrentPosition(),
    .br = 0,
    .fl = 0,
    // .br = br_stepper->getCurrentPosition(),
    // .fl = fl_stepper->getCurrentPosition(),
    .fr = fr_stepper->getCurrentPosition(),
  };

  Serial.println("Cable reference initialized");
}

long cable_length_to_steps(double delta_length, bool inversion) {
  double revolutions = delta_length / (2.0 * PI * SPOOL_RADIUS);
  if (inversion) revolutions *= -1;
  return std::lround(revolutions * STEPS_PER_REV);
}

void set_target(double x, double y) {
  CableLengths target_lengths = calculate_cable_lengths(x, y);
  double bl_delta = target_lengths.bl - initial_lengths.bl;
  double br_delta = target_lengths.br - initial_lengths.br;
  double fl_delta = target_lengths.fl - initial_lengths.fl;
  double fr_delta = target_lengths.fr - initial_lengths.fr;

  long bl_steps = cable_length_to_steps(bl_delta, BL_INVERSION);
  long br_steps = cable_length_to_steps(br_delta, BR_INVERSION);
  long fl_steps = cable_length_to_steps(fl_delta, FL_INVERSION);
  long fr_steps = cable_length_to_steps(fr_delta, FR_INVERSION);

  long bl_target = initial_motor_positions.bl + bl_steps;
  long br_target = initial_motor_positions.br + br_steps;
  long fl_target = initial_motor_positions.fl + fl_steps;
  long fr_target = initial_motor_positions.fr + fr_steps;

  bl_stepper->moveTo(bl_target);
  // br_stepper->moveTo(br_target);
  // fl_stepper->moveTo(fl_target);
  fr_stepper->moveTo(fr_target);
}

void setup() {
  Serial.begin(BAUD_RATE);

  engine.init();
  SETUP_MOTOR(BL, bl);
  // SETUP_MOTOR(BR, br);
  // SETUP_MOTOR(FL, fl);
  SETUP_MOTOR(FR, fr);
  
  delay(1000);

  initialize_reference(0, 0);
}

double read_double() {
  while (!Serial.available());
  double d = Serial.parseFloat();
  while (Serial.available() > 0) Serial.read();
  return d;
}

void loop() {
  if (!check_motors_communicating()) return;

  double x = read_double();
  double y = read_double();

  Serial.printf("\nTracking target position (%f, %f)\n", x, y);
  set_target(x, y);

  delay(20);
}