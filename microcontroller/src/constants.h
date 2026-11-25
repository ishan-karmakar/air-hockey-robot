#pragma once
#include <TMC2209.h>

struct Point {
    double x;
    double y;
};

constexpr int BAUD_RATE = 115200;

constexpr int RX_PIN = 22;
constexpr int TX_PIN = 12;

constexpr int RUN_CURRENT_PERCENT = 100;
constexpr int HOLD_CURRENT_PERCENT = 50;
constexpr int MAX_VELOCITY = 1000;
constexpr int MAX_ACCEL = 1000;
constexpr int REPLY_DELAY = 4;
constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEPS = 128;
constexpr int STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;

constexpr double TABLE_WIDTH = 0.889;
constexpr double TABLE_HEIGHT = 0.9144;
constexpr double PADDLE_ATTACHMENT_RADIUS = 0.023;
constexpr double SPOOL_RADIUS = 0.04;
constexpr double PADDLE_RADIUS = 0.04683125;
constexpr double CABLE_HEIGHT_DIFFERENCE = 0.005;

// Back Left Motor
constexpr TMC2209::SerialAddress BL_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_1;
constexpr bool BL_INVERSION = false;
constexpr int BL_DIR_PIN = 0;
constexpr int BL_STEP_PIN = 1;
constexpr Point BL_PULLEY = {-0.0552007233014, -0.0552007233014};

// Back Right Motor
constexpr TMC2209::SerialAddress BR_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_2;
constexpr bool BR_INVERSION = false;
constexpr int BR_DIR_PIN = -1;
constexpr int BR_STEP_PIN = -1;
constexpr Point BR_PULLEY = {TABLE_WIDTH + 0.0552007233014, -0.0552007233014};

// Front Left Motor
constexpr TMC2209::SerialAddress FL_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_3;
constexpr bool FL_INVERSION = false;
constexpr int FL_DIR_PIN = -1;
constexpr int FL_STEP_PIN = -1;
constexpr Point FL_PULLEY = {-0.039, 0.928};

// Front Right Motor
constexpr TMC2209::SerialAddress FR_SERIAL_ADDR = TMC2209::SERIAL_ADDRESS_0;
constexpr bool FR_INVERSION = false;
constexpr int FR_DIR_PIN = 10;
constexpr int FR_STEP_PIN = 11;
constexpr Point FR_PULLEY = {TABLE_WIDTH + 0.058524, TABLE_HEIGHT};