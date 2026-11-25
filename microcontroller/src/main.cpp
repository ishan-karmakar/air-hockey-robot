#include <Arduino.h>
#include <FastAccelStepper.h>
#include "constants.h"

#define SETUP_MOTOR(X, x) setup_motor(x ## _driver, x ## _stepper, X ## _SERIAL_ADDR, X ## _STEP_PIN, X ## _DIR_PIN)

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
  driver.setHoldCurrent(HOLD_CURRENT_PERCENT);
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

double tangent_length(Point pulley, Point attachment) {
    double dx = attachment.x - pulley.x;
    double dy = attachment.y - pulley.y;

    double horizontal_distance_squared = dx * dx + dy * dy;

    double tangent_squared =
        horizontal_distance_squared -
        SPOOL_RADIUS * SPOOL_RADIUS;

    if (tangent_squared < 0.0) {
        Serial.println("ERROR: Attachment inside pulley radius!");
        return NAN;
    }

    return std::sqrt(
        tangent_squared +
        CABLE_HEIGHT_DIFFERENCE * CABLE_HEIGHT_DIFFERENCE
    );
}

CableLengths calculate_cable_lengths(double x, double y) {
    double diagonal = PADDLE_ATTACHMENT_RADIUS / std::sqrt(2.0);

    Point bl_attachment = {x - diagonal, y - diagonal};
    Point br_attachment = {x + diagonal, y - diagonal};
    Point fl_attachment = {x - diagonal, y + diagonal};
    Point fr_attachment = {x + diagonal, y + diagonal};

    return {
        .bl = tangent_length(BL_PULLEY, bl_attachment),
        .br = tangent_length(BR_PULLEY, br_attachment),
        .fl = tangent_length(FL_PULLEY, fl_attachment),
        .fr = tangent_length(FR_PULLEY, fr_attachment),
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

  initialize_reference(PADDLE_RADIUS, PADDLE_RADIUS);
}

double read_double() {
  while (!Serial.available());
  double d = Serial.parseFloat();
  while (Serial.available() > 0) Serial.read();
  return d;
}

bool check_valid_target(double x, double y) {
  if (x > (TABLE_WIDTH - PADDLE_RADIUS) || x < PADDLE_RADIUS)
    return false;
  if (y > (TABLE_HEIGHT - PADDLE_RADIUS) || y < PADDLE_RADIUS)
    return false;
  return true;
}

void loop() {
  if (!check_motors_communicating()) return;

  double x = read_double();
  double y = read_double();
  if (!check_valid_target(x, y)) {
    Serial.printf("\nTarget position (%f, %f) is outside bounds\n", x, y);
    return;
  }

  Serial.printf("\nTracking target position (%f, %f)\n", x, y);
  set_target(x, y);

  delay(20);
}