/*
Robocup 2025 Code for Autonomous Weight Pickup

Authors: Dominic Mcnulty, Matthew Donnelly, Charlie Horne

Last Updated: 27.09.2025
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_TCS34725.h>
#include <utility/imumaths.h>
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <queue>
#include <string>
#include <map>
#include <unordered_map>
#include <Servo.h>
#include "MotorDriver.h"
#include "Encoder.h"

struct Point {
    int x, y;
    bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point &other) const {
        return !(*this == other);
    }
};

Point ramp_bottom_left = {0, 0};
Point ramp_top_right = {0, 0};

#define WEIGHT_DETECTION 1
#define MAX_WEIGHT_COUNT 1
std::string start = "GREEN";

// #define LEFT_FORWARD 0
// #define RIGHT_FORWARD 0
// #define LEFT_CLOCKWISE 0
// #define RIGHT_CLOCKWISE 0
// #define LEFT_ANTI_CLOCKWISE 0 
// #define RIGHT_ANTI_CLOCKWISE 0

#define LEFT_FORWARD 0.99
#define RIGHT_FORWARD 0.91

#define LEFT_CLOCKWISE 0.99
#define RIGHT_CLOCKWISE -0.95

#define LEFT_ANTI_CLOCKWISE -0.98
#define RIGHT_ANTI_CLOCKWISE 0.85














// -PD Control Gains
float Kp = 0.01;

float lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;


#define MINIMUM_SPEED_SCALE 0.78

//Button Info
#define BUTTON_PIN 20
#define BUTTON_COOLDOWN 400

#define DOOR_CLOSED 2200
#define DOOR_OPEN 1380

#define DUMPING_TIME 2000
#define PRESS_COOLDOWN 400


// WHEN WEIGHTS ARE EXITING
#define ANTI_CLOCKWISE_BASE 1700
#define ANTI_CLOCKWISE_WEIGHT_ADDITION 50


//FOR WHEN WEIGHTS ARE ADDED
#define CLOCKWISE_BASE 1250
#define CLOCKWISE_WEIGHT_ADDITION 75

#define INDUCTIVE_PIN 21

//Adjustable mechanical information
#define WHEEL_DIAMETER 67.2
#define ROBOT_DIAMETER 380
#define L0_SENSOR_COUNT 2
#define L1_SENSOR_COUNT 5

#define BASE_SIZE 10

//Sensor Data
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

#define L0_MAX_RANGE 1500
#define L1_MAX_RANGE 2000
#define WEIGHT_DETECTION_RANGE 800
#define WEIGHT_DETECTION_REQUIREMENT 200
#define WEIGHT_BUFFER_REQUIRED 3

//Matrix information    
#define MATRIX_ROWS 48
#define MATRIX_COLS 98
#define MATRIX_MAX 10
#define WALL_REQUIREMENT 3
#define CELL_SIZE (2400 / MATRIX_ROWS) // Total row length is 2.4m in real life, divided by rows gives total area per cell  
#define DECAY_COOLDOWN 8000 // Amount of Milliseconds before decaying all cells (making them less confident)

//Adjustable code values for varying outcomes
#define MAXIMUM_DISTANCE_ERROR 100
#define MAXIMUM_ANGLE_ERROR 10

#define SEED 452

#define THICKNESS 0



//Referenced information
#define TICKS_PER_REV 2900
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define ROBOT_RADIUS (ROBOT_DIAMETER/2)


// Hash function for Point so it can be used in unordered_map
struct PointHash {
    size_t operator()(const Point &p) const {
        // Large prime numbers for unique values
        return (p.x * 73856093) ^ (p.y * 19349663);
    }
};

// SX1509 Addresses
const byte SX1509_ADDRESS = 0x3F;
const byte SX1509_ADDRESS_2 = 0x3E;

uint32_t start_time = 0;

// VL53LX info
const uint8_t xshutPinsL0[L0_SENSOR_COUNT] = {3, 4};
const uint8_t xshutPinsL1[L1_SENSOR_COUNT] = {0,1,2,5,6};

bool got_to_goal = 0;

// Wheel Rotation Info
int weight_count = 0;
int prev_weight_count = 0;
bool rotating = true;
bool start_of_rotation = true;
bool in_drop_off_mode = false;
bool dumping_plastic = false;
unsigned long startDumpTime = 0;
unsigned long lastSwitchTime = 0;
int inductiveSensorVAL = 800; 
int not_found_wheel_count = 0;
bool rotating_clockwise = false;

bool limitSwitchDoor_VAL = 0;
bool limitSwitchWheel_VAL = 0;
const byte limitSwitchDoor = 0;
const byte limitSwitchWheel = 1;

// COLOR
uint16_t clear, red, green, blue;
uint32_t sum;
float g, b;


//Button
int buttonPressed = 0;
unsigned long lastPressTime = 0;
bool buttonPressedStart = false;

// Objects
SX1509 io;
SX1509 io_2;
VL53L0X sensorsL0[L0_SENSOR_COUNT];
VL53L1X sensorsL1[L1_SENSOR_COUNT];
Adafruit_BNO055 bno;
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


//Various Motors
MotorDriver leftMotor;
MotorDriver rightMotor;
MotorDriver spinnerMotor;
Servo holderMotor;
Servo doorMotor;

//Encoders
Encoder leftEncoder(4, 5);
Encoder rightEncoder(2, 3);
long leftEncoderValue = 0, rightEncoderValue = 0;
long previous_leftEncoderValue = 0, previous_rightEncoderValue = 0;

// Pose Information
double currentX = 300, currentY;
uint8_t currentX_int, currentY_int;
uint8_t previousX_int, previousY_int;
double angle_radians = 0, angle_degrees = 0, angle_pitch = 0;

int left_spotted_count = 0;
int right_spotted_count = 0;
bool failed_sensors = false;
bool ignore_sensors = false;


// Pathing information
std::deque<Point> path = {};
std::deque<Point> full_path = {};
std::deque<Point> weight_location_list = {};

//Map
std::vector<std::vector<int>> grid(MATRIX_ROWS, std::vector<int>(MATRIX_COLS, 0));
std::vector<std::vector<int>> padded_grid(MATRIX_ROWS, std::vector<int>(MATRIX_COLS, 0));
std::vector<std::vector<int>> seen_grid(MATRIX_ROWS, std::vector<int>(MATRIX_COLS, 0));
std::vector<std::vector<char>> random_locations;
long previousDecayTime = 0;

double orderedAngle = 0;



enum Actions {
    WEIGHT_SEEN,
    DROP_OFF_START,
    DRIVING,
    DROP_OFF,
    NEW_PATH_REQUIRED
};
Actions currentAction;

double drop_off_angle = (start == "GREEN") ? 0 : -90;

Point home_base = {6, 6};

// RANDOM 

std::deque<Point> designated_position_list = {{6, 24}, {18, 24}, {50, 42}};

// BLUE FORWARD AND BACK
//std::deque<Point> designated_position_list = {{MATRIX_COLS - 3, MATRIX_ROWS - 3}, {3, MATRIX_ROWS - 3}, {MATRIX_COLS - 3, MATRIX_ROWS - 3}, {3, MATRIX_ROWS - 3},{MATRIX_COLS - 3, MATRIX_ROWS - 3}, {3, MATRIX_ROWS - 3}};

// GREEN FORWARD AND BACK
//std::deque<Point> designated_position_list = {{MATRIX_COLS - 3, 3}, {3, 3}, {MATRIX_COLS - 3, 3}, {3, 3}, {MATRIX_COLS - 3, 3}, {3, 3}};


void setup()
{
    Serial.begin(115200); 

    // Attach Motors
    leftMotor.attach(1, 0);
    rightMotor.attach(0, 1); 
    spinnerMotor.attach(7, 0);
    holderMotor.attach(8);
    doorMotor.attach(29);

    // I2C setup
    Wire.begin(); Wire.setClock(400000);
    Wire1.begin(); Wire1.setClock(400000);

    //Encoders
    leftEncoder.begin();
    rightEncoder.begin();

  
    // SX1509 Setup
    if (!io.begin(SX1509_ADDRESS, Wire)) { Serial.println("Failed to communicate."); while(1); }
    if (!io_2.begin(SX1509_ADDRESS_2, Wire1)) { Serial.println("Failed to communicate."); while(1); }
    io_2.pinMode(limitSwitchDoor, INPUT);
    io_2.pinMode(limitSwitchWheel, INPUT); 
    pinMode(INDUCTIVE_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);

    //Reset TOF sensors
    //Disable/reset all sensors by driving their XSHUT pins low.
    for (uint8_t i = 0; i < L0_SENSOR_COUNT; i++)
    {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
    }
    for (uint8_t i = 0; i < L1_SENSOR_COUNT; i++) 
    {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
    }

    // if (tcs.begin()) {
    //     Serial.println("Found sensor");
    // } else {
    //     Serial.println("No TCS34725 found ... check your connections");
    //     while (1); // halt!
    // }


    //Initialize TOF sensor
    for (uint8_t i = 0; i < L0_SENSOR_COUNT; i++) {
        io.digitalWrite(xshutPinsL0[i], HIGH);
        delay(10);
        sensorsL0[i].setTimeout(500);
        if (!sensorsL0[i].init()) { Serial.println("Failed to init sensor"); while(1); }
        sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
        sensorsL0[i].startContinuous(50);
    }
    for (uint8_t i = 0; i < L1_SENSOR_COUNT; i++) {
        io.digitalWrite(xshutPinsL1[i], HIGH);
        delay(10);
        sensorsL1[i].setTimeout(500);
        if (!sensorsL1[i].init()) { Serial.println("Failed to init sensor"); while(1); }
        sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
        sensorsL1[i].startContinuous(50);
    }

    currentY = (start=="GREEN") ? 300 : ((CELL_SIZE*MATRIX_ROWS)-300);
    currentX_int = round(currentX/CELL_SIZE);
    currentY_int = round(currentY/CELL_SIZE);

    currentAction = NEW_PATH_REQUIRED;

    if (start == "BLUE") {
        home_base = {6, MATRIX_ROWS - 6};
    }

    rotating = true;
    Serial.println("Setting up\n");
    while (!rotate_to_next(true)) {};
    doorMotor.writeMicroseconds(DOOR_CLOSED);
    //doorMotor.writeMicroseconds(DOOR_OPEN);
    while (!checkButtonPressed());
    Serial.println("IMU ON");
    // IMU setup
    bno = Adafruit_BNO055(55, 0x28);
    if (!bno.begin()) { Serial.println("No BNO055 detected"); while(1); }
    while (!checkButtonPressed()) {};
}

Point mirrorPoint(Point location) {
    Point temp = {};
    if (!((location.y <= MATRIX_ROWS/2+1) && (location.y >= MATRIX_ROWS/2-1))) {
        temp = {location.x, MATRIX_ROWS - location.y};
        Serial.printf("ADDED (%d, %d\n", location.x, MATRIX_ROWS-location.y-1);
    }
    return temp;
}

void resetSensors() {

    

    for (uint8_t i = 0; i < L0_SENSOR_COUNT; i++) sensorsL0[i] = VL53L0X();
    for (uint8_t i = 0; i < L1_SENSOR_COUNT; i++) sensorsL1[i] = VL53L1X();

    for (uint8_t i = 0; i < L0_SENSOR_COUNT; i++) {
        io.digitalWrite(xshutPinsL0[i], LOW);
    }
    for (uint8_t i = 0; i < L1_SENSOR_COUNT; i++) {
        io.digitalWrite(xshutPinsL1[i], LOW);
    }
    delay(10);   // allow proper shutdown

    for (uint8_t i = 0; i < L0_SENSOR_COUNT; i++) {
        io.digitalWrite(xshutPinsL0[i], HIGH);
        delay(10);
        sensorsL0[i].setTimeout(500);
        if (!sensorsL0[i].init()) {
            Serial.print("L0 sensor "); Serial.print(i); Serial.println(" failed after reset");
            continue;
        }
        sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
        sensorsL0[i].startContinuous(50);
        delay(10);
    }

    for (uint8_t i = 0; i < L1_SENSOR_COUNT; i++) {
        io.digitalWrite(xshutPinsL1[i], HIGH);
        delay(10);
        sensorsL1[i].setTimeout(500);
        if (!sensorsL1[i].init()) {
            Serial.print("L1 sensor "); Serial.print(i); Serial.println(" failed after reset");
            continue;
        }
        sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
        sensorsL1[i].startContinuous(50);
        delay(10);
    }
}



/*********************

Functions used to Rotate Robot

*********************/
double normalizeAngle(double angle) {
    angle = fmod(angle, 360.0);  // remainder after division
    if (angle < 0) angle += 360; // make negative angles positive
    return angle;
}

double shortestAngleDiff(double target, double current) {
    double diff = normalizeAngle(target) - normalizeAngle(current);
    if (diff > 180) diff -= 360;  // go the shorter way
    if (diff < -180) diff += 360;
    return diff;
}


// Helper to get yaw angle
double getGyro(sensors_event_t* event) {
    return event->orientation.x;
}

double getPitch(sensors_event_t* event) {
    return event->orientation.y;
}

void turnToAnglePID(double error) {

    // --- PID Output ---
    double control = Kp * (error);

    //Serial.printf("[PD] err=%.3f last=%.3f c=%.4f\n", error, lastError, control);

    // Clamp control to [-1, 1]
    if (control > 1.0) control = 1.0;
    if (control < -1.0) control = -1.0;

    // --- Motor Mixing ---
    double leftSpeed = 0;
    double rightSpeed = 0;

    if (control < 0) {
        // CLOCKWISE
        double minLeft  = LEFT_CLOCKWISE * MINIMUM_SPEED_SCALE;
        double minRight = RIGHT_CLOCKWISE * MINIMUM_SPEED_SCALE;

        double factor = fabs(control);
        leftSpeed  = minLeft  + (LEFT_CLOCKWISE  - minLeft)  * factor;
        rightSpeed = minRight + (RIGHT_CLOCKWISE - minRight) * factor;

    } else if (control > 0) {
        // ANTICLOCKWISE
        double minLeft  = LEFT_ANTI_CLOCKWISE * MINIMUM_SPEED_SCALE;
        double minRight = RIGHT_ANTI_CLOCKWISE * MINIMUM_SPEED_SCALE;

        
        leftSpeed  = minLeft  + (LEFT_ANTI_CLOCKWISE  - minLeft)  * control;
        rightSpeed = minRight + (RIGHT_ANTI_CLOCKWISE - minRight) * control;
    }

    //Serial.printf("Left=%.3f, Right=%.3f\n", leftSpeed, rightSpeed);
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
}

bool rotateRobot(double designated_rotation, bool close_target)
{
    double error = shortestAngleDiff(angle_degrees, designated_rotation);
    double error_allowed = close_target ? 4 : MAXIMUM_ANGLE_ERROR;
    
    if (fabs(error) >= error_allowed) {   // e.g. within 2 degrees
        turnToAnglePID(error);
        lastError = error;
        lastTime = millis();
        return 0;
    }
    lastError = error;
    lastTime = millis();
    return 1;
    // if (abs(error) >= MAXIMUM_ANGLE_ERROR) {
    //     float close = (abs(error) <= MAXIMUM_ANGLE_ERROR+10) ? 0.8 : 1;  

    //     if (error < 0) {
    //         leftMotor.setSpeed(LEFT_CLOCKWISE * close);
    //         rightMotor.setSpeed(RIGHT_CLOCKWISE * close);
    //     } else {
    //         leftMotor.setSpeed(LEFT_ANTI_CLOCKWISE * close);
    //         rightMotor.setSpeed(RIGHT_ANTI_CLOCKWISE * close);
    //     }
    //     return 0;
    // }
    // return 1;
}



/***

Checks if within a certain amount of error from goal.
Check if within certain amount of error of angle.
Drive Forward or Rotate based on results

***/
bool driveTowardsGoal() {
    double dx = (path[0].x*CELL_SIZE) - currentX;
    double dy = (path[0].y*CELL_SIZE) - currentY;

    //Serial.printf("Current POS: (%d : %d), Desired POS: (%d : %d)\n", currentX_int, currentY_int, path[0].x, path[0].y);

    // If the robot is outside MAXIMUM_DISTANCE_ERROR total in both x and y then it needs to drive, otherwise it is at location and needs to update path
    if (!((abs(dx) + abs(dy)) <= MAXIMUM_DISTANCE_ERROR)) {
        double radians_to_goal = atan2(dy, dx);
        double degrees_to_goal = (radians_to_goal * 180 / PI);
        // If the robot is aligned then travel forward
        if (rotateRobot(degrees_to_goal, false)) {
            //Serial.println("Driving Forward");
            leftMotor.setSpeed(LEFT_FORWARD);
            rightMotor.setSpeed(RIGHT_FORWARD);
        }
        return 0;
    }
    return 1;
}

//Checks if encoders have changed, if so change the currentX and currentY
void updatePosition() {
    previousX_int = currentX_int;
    previousY_int = currentY_int;
  // Read encoders
    long leftEncoderValue = leftEncoder.getCount();
    long rightEncoderValue = rightEncoder.getCount();
    if ((previous_leftEncoderValue != leftEncoderValue)||(previous_rightEncoderValue != rightEncoderValue)) {

        long difference1 = -1*(previous_leftEncoderValue - leftEncoderValue);
        long difference2 = -1*(previous_rightEncoderValue - rightEncoderValue);
        long difference_average = (difference1 + difference2)/2;
        double distance_average = ((WHEEL_CIRCUMFERENCE*difference_average)/TICKS_PER_REV);

        currentX += distance_average * cos(angle_radians);
        currentY += distance_average * sin(angle_radians);

        // Stop robot from thinking its outside of the arena as this is impossible (must be slipping somewhere)
        if (currentX < ROBOT_RADIUS-50) {currentX = ROBOT_RADIUS-50;}
        if (currentX >= MATRIX_COLS*CELL_SIZE-ROBOT_RADIUS-50) {currentX = MATRIX_COLS*CELL_SIZE-ROBOT_RADIUS-50;}

        if (currentY < ROBOT_RADIUS-50) {currentY = ROBOT_RADIUS-50;}
        if (currentY >= MATRIX_ROWS*CELL_SIZE-ROBOT_RADIUS-50) {currentY_int = MATRIX_ROWS*CELL_SIZE-ROBOT_RADIUS-50;}


        currentX_int = round(currentX/CELL_SIZE);
        currentY_int = round(currentY/CELL_SIZE);


        previous_leftEncoderValue = leftEncoderValue;
        previous_rightEncoderValue = rightEncoderValue;
    }
}

void updateRotation(){
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  angle_degrees = getGyro(&orientationData);
  angle_pitch = getPitch(&orientationData);
  angle_radians = (angle_degrees) * PI / 180;
  
  
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

/* 
Creates Grid Path
*/

bool lineDraw(int x0, int y0, int x1, int y1) {
    auto inBounds = [&](int x, int y) {
        return x >= 0 && x < MATRIX_COLS && y >= 0 && y < MATRIX_ROWS;
    };

    auto blocked = [&](int cx, int cy) {
        if (!inBounds(cx, cy)) return true; // out of bounds = blocked
        if ((cx == x0 && cy == y0) || (cx == x1 && cy == y1)) return false;
        return padded_grid[cy][cx] == 1;
    };

    double x = x0 + 0.5;
    double y = y0 + 0.5;
    double ex = x1 + 0.5;
    double ey = y1 + 0.5;

    double dx = ex - x;
    double dy = ey - y;

    int stepX = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);
    int stepY = (dy > 0) ? 1 : (dy < 0 ? -1 : 0);

    dx = std::abs(dx);
    dy = std::abs(dy);

    const double INF = 1e100;
    double tDeltaX = (dx == 0.0) ? INF : 1.0 / dx;
    double tDeltaY = (dy == 0.0) ? INF : 1.0 / dy;

    double fracX = x - std::floor(x);
    double fracY = y - std::floor(y);
    double tMaxX = (dx == 0.0) ? INF : (stepX > 0 ? (1.0 - fracX) : fracX) * tDeltaX;
    double tMaxY = (dy == 0.0) ? INF : (stepY > 0 ? (1.0 - fracY) : fracY) * tDeltaY;

    int cx = x0;
    int cy = y0;

    auto checkNeighbors = [&](int gx, int gy) -> bool {
        if (THICKNESS == 0) {
            if (blocked(gx, gy)) return true;
        } else {
            for (int oy = -THICKNESS; oy <= THICKNESS; ++oy) {
                for (int ox = -THICKNESS; ox <= THICKNESS; ++ox) {
                    if (blocked(gx + ox, gy + oy)) return true;
                }
            }
        }

        return false;
    };

    // Initial check
    if (checkNeighbors(cx, cy)) return true;

    while (cx != x1 || cy != y1) {
        if (tMaxX < tMaxY) {
            cx += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxX) {
            cy += stepY;
            tMaxY += tDeltaY;
        } else {
            // Corner case
            cx += stepX;
            cy += stepY;
            tMaxX += tDeltaX;
            tMaxY += tDeltaY;
        }

        if (checkNeighbors(cx, cy)) return true;
    }
    return false;
}





/*
Converts my current position and TOF_distances into x, y positions based on grid dimensions
*/

void updateMatrix(uint16_t TOF_distance, double angle_offset) {
    if (TOF_distance > ROBOT_RADIUS+2) {
        grid[previousY_int][previousX_int] = -1 * MATRIX_MAX;
        int x0 = currentX_int;
        int y0 = currentY_int;
        int x1 = round((currentX+TOF_distance * cos(angle_radians+(angle_offset*PI/180)))/CELL_SIZE);
        int y1 = round((currentY+TOF_distance * sin(angle_radians+(angle_offset*PI/180)))/CELL_SIZE);

        //Serial.printf("WALL: (%d, %d), DIS: %d\n", x1, y1, TOF_distance);

        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int x = x0, y = y0;
        int sx = (x0 > x1) ? -1 : 1;
        int sy = (y0 > y1) ? -1 : 1;

        if (dx > dy) {
            int err = dx / 2;
            while (x != x1) {
                if ((x != x0 || y != y0) && (x != x1 || y != y1)) {
                    if (y >= 0 && y < MATRIX_ROWS && x >= 0 && x < MATRIX_COLS) {
                        seen_grid[y][x]++;
                        if (grid[y][x] == WALL_REQUIREMENT) {
                            Serial.println("NEW PATH CLEAR");
                            currentAction = NEW_PATH_REQUIRED;
                        }
                        if (grid[y][x] != -1 * MATRIX_MAX) {
                            grid[y][x]--;
                        }
                    }
                }
                err -= dy;
                if (err < 0) {
                    y += sy;
                    err += dx;
                }
                x += sx;
            }
        } else {
            int err = dy / 2;
            while (y != y1) {
                if ((x != x0 || y != y0) && (x != x1 || y != y1)) {
                    if (y >= 0 && y < MATRIX_ROWS && x >= 0 && x < MATRIX_COLS) {
                        seen_grid[y][x]++;
                        if (grid[y][x] == WALL_REQUIREMENT) {
                            Serial.println("NEW PATH CLEAR");
                            currentAction = NEW_PATH_REQUIRED;
                        }
                        if (grid[y][x] != -1 * MATRIX_MAX) {
                            grid[y][x]--;
                        }
                    }
                }
                err -= dx;
                if (err < 0) {
                    x += sx;
                    err += dy;
                }
                y += sy;
            }
        }

        if (TOF_distance < L1_MAX_RANGE) {
            if (y1 >= 1 && y1 < MATRIX_ROWS-1 && x1 >= 1 && x1 < MATRIX_COLS-1) {
                seen_grid[y][x]++;
                //Serial.printf("TOF within grid VAL: %d\n", grid[y1][x1]);
                if (grid[y1][x1] == WALL_REQUIREMENT) {
                    //Serial.println("TOF equal to wall requirement");
                    //Serial.printf("{%d, %d}\n", x1, y1);
                    for (Point location : full_path) {
                        //Serial.printf("{%d, %d}", location.x, location.y);
                        if (((x1-2) <= location.x && location.x <= (x1+2)) && ((y1-2) <= location.y && location.y <= (y1+2))) {
                            Serial.println("NEW PATH REQUIRED TOF");
                            currentAction = NEW_PATH_REQUIRED;
                        }
                    }
                }
                //Serial.printf("GRID (%d, %d) VAL: %d\n", x0, y0, grid[y1][x1]);
                if (grid[y1][x1] != MATRIX_MAX) {
                    grid[y1][x1]++;
                }
            }
        }
        if (y0 >= 0 && y0 < MATRIX_ROWS && x0 >= 0 && x0 < MATRIX_COLS) {
            grid[y0][x0] = 21;
        }
    }
}

std::deque<Point> simplify_path(const std::deque<Point>& path) {
    if (path.size() <= 2) return path;

    // --- STEP 1: Line-of-sight simplification ---
    std::deque<Point> los_simplified;
    size_t i = 0;
    los_simplified.push_back(path.front()); // keep start

    while (i < path.size() - 1) {
        size_t best = i + 1;
        // find farthest visible point
        for (size_t j = path.size() - 1; j > i; --j) {
            if (!lineDraw(path[i].x, path[i].y, path[j].x, path[j].y)) {
                best = j;
                break;
            }
        }
        i = best;
        los_simplified.push_back(path[i]);
    }

    // --- STEP 2: Collinearity simplification ---
    if (los_simplified.size() < 3) return los_simplified;

    std::deque<Point> final_path;
    final_path.push_back(los_simplified[0]);

    for (size_t k = 1; k < los_simplified.size() - 1; ++k) {
        int dx1 = los_simplified[k].x - los_simplified[k - 1].x;
        int dy1 = los_simplified[k].y - los_simplified[k - 1].y;
        int dx2 = los_simplified[k + 1].x - los_simplified[k].x;
        int dy2 = los_simplified[k + 1].y - los_simplified[k].y;

        // if direction changes, keep this point
        if (dx1 != dx2 || dy1 != dy2) {
            final_path.push_back(los_simplified[k]);
        }
    }

    final_path.push_back(los_simplified.back());
    return final_path;
}


// std::deque<Point> simplify_path(const std::deque<Point> &path) {
//     if (path.size() < 3) return path;

//     std::deque<Point> simplified;
//     simplified.push_back(path[0]);

//     for (size_t i = 1; i < path.size() - 1; i++) {
//         int dx1 = path[i].x - path[i-1].x;
//         int dy1 = path[i].y - path[i-1].y;
//         int dx2 = path[i+1].x - path[i].x;
//         int dy2 = path[i+1].y - path[i].y;

//         if (dx1 != dx2 || dy1 != dy2) {
//             simplified.push_back(path[i]);
//         }
//     }

//     simplified.push_back(path.back());
//     return simplified;
// }

// float heuristic(const Point &a, const Point &b) {
//     float dx = static_cast<float>(a.x - b.x);
//     float dy = static_cast<float>(a.y - b.y);
//     return sqrtf(dx * dx + dy * dy); // Euclidean distance
// }


int heuristic(const Point &a, const Point &b) {
    return abs(a.x - b.x) + abs(a.y - b.y); // Manhattan
}

std::deque<Point> reconstruct_path(std::unordered_map<Point, Point, PointHash> &came_from, Point current, Point start) {
    std::deque<Point> path;
    while (came_from.find(current) != came_from.end()) {
        if (current.x < 0 || current.x > MATRIX_COLS || current.y < 0 || current.y > MATRIX_COLS) {
            return {};
        }
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start);  // ADD THIS FOR START TO BE INSIDE COORDINATES
    reverse(path.begin(), path.end());
    return path;
}


/*
Astar Path finding algorithm to find path towards goal with walls in the way, has turn penalty to reduce turns where possible
*/
std::deque<Point> astar_search(Point start, Point goal) {
    auto cmp = [](const std::pair<float, Point> &a, const std::pair<float, Point> &b) {
        return a.first > b.first;
    };

    std::priority_queue<std::pair<float, Point>, std::vector<std::pair<float, Point>>, decltype(cmp)> open_set(cmp);
    open_set.push({0.0f, start});

    std::unordered_map<Point, Point, PointHash> came_from;
    std::unordered_map<Point, float, PointHash> g_score;
    g_score[start] = 0.0f;

    while (!open_set.empty()) {
        Point current = open_set.top().second;
        open_set.pop();

        if (current == goal) {
            return reconstruct_path(came_from, current, start);
        }

        // 8 directions: 4 cardinal + 4 diagonals
        std::vector<Point> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},   // cardinal
            {1, 1}, {-1, 1}, {1, -1}, {-1, -1}  // diagonals
        };
        // for (auto &d : directions) { 
        //     Point neighbor = {current.x + d.x, current.y + d.y}; 
            
        //     if (neighbor.x >= 0 && neighbor.x < MATRIX_COLS && neighbor.y >= 0 && neighbor.y < MATRIX_ROWS) { 
        //         if (padded_grid[neighbor.y][neighbor.x] == 1) continue; // wall 
        //     // cost is 1.0 for straight, sqrt(2) for diagonal 
        //         float move_cost = (d.x == 0 || d.y == 0) ? 1.0f : 1.414f; 
        //         float tentative_g = g_score[current] + move_cost; 
        //         if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) { 
        //             came_from[neighbor] = current; g_score[neighbor] = tentative_g; 
        //             float f_score = tentative_g + heuristic(neighbor, goal); 
        //             open_set.push({f_score, neighbor}); 
        //         }       
        //     } 
        // }
        for (auto &d : directions) {
            Point neighbor = {current.x + d.x, current.y + d.y};

            if (neighbor.x >= 1 && neighbor.x < MATRIX_COLS-1 &&
                neighbor.y >= 1 && neighbor.y < MATRIX_ROWS-1) {
                if (padded_grid[neighbor.y][neighbor.x] == 1) continue; // wall

                // cost is 1.0 for straight, sqrt(2) for diagonal
                float move_cost = (d.x == 0 || d.y == 0) ? 1.0f : 1.414f;

                // add penalty for turning
                float turn_penalty = 0.0;
                if (came_from.find(current) != came_from.end()) {
                    Point prev = came_from[current];
                    Point prev_dir = {current.x - prev.x, current.y - prev.y};
                    if (!(prev_dir.x == d.x && prev_dir.y == d.y)) {
                        turn_penalty = 0.2; // tweak this value
                    }
                }

                float tentative_g = g_score[current] + move_cost + turn_penalty;
                if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    float f_score = tentative_g + heuristic(neighbor, goal);
                    open_set.push({f_score, neighbor});
                }
            }
        }
    }

    return {}; // no path found
}

/*
Functions for Dropping Weights back at base
*/
bool rotate_to_next(bool setup) {
    int anti_clockwise = setup ? (ANTI_CLOCKWISE_BASE+75) : ANTI_CLOCKWISE_BASE;
  int speed = rotating_clockwise ? (CLOCKWISE_BASE - weight_count * CLOCKWISE_WEIGHT_ADDITION) : (anti_clockwise + weight_count*ANTI_CLOCKWISE_WEIGHT_ADDITION);
  limitSwitchWheel_VAL = io_2.digitalRead(limitSwitchWheel);
  //Serial.printf("BUTTON: %d, ROTATION: (%d %d), WEIGHTS (%d %d)\n", buttonPressed, start_of_rotation, rotating, weight_count, prev_weight_count);
  if (rotating) {
    //Serial.println("Rotating");
    rotating = true;
    if (limitSwitchWheel_VAL) {
      //Serial.println("Limit Switch Wheel");
      start_of_rotation = false;
      //Serial.print("On  \n");
      holderMotor.writeMicroseconds(speed);
    } else {
      if (start_of_rotation) {
        holderMotor.writeMicroseconds(speed);
      } else {
        if (not_found_wheel_count == 1 || !rotating_clockwise) {
          holderMotor.writeMicroseconds(1500);
          lastSwitchTime = millis();
          rotating=false;
          start_of_rotation=true;
          not_found_wheel_count = 0;
          prev_weight_count = weight_count;
          return 1;
        } else {
          not_found_wheel_count++;
        }

      }
    }
  }
  return 0;
}

void drop_weights() {

    // SINGLE WEIGHT    
    // Serial.println("Dropping Weights");
    // doorMotor.writeMicroseconds(DOOR_OPEN);
    // leftMotor.setSpeed(LEFT_FORWARD);
    // rightMotor.setSpeed(RIGHT_FORWARD);
    // delay(400);
    // leftMotor.setSpeed(-1*LEFT_FORWARD);
    // rightMotor.setSpeed(-1*RIGHT_FORWARD);
    // delay(400);
    // leftMotor.setSpeed(0.0);
    // rightMotor.setSpeed(0.0);
    // delay(1000);

    // MULTIPLE WEIGHTS
    doorMotor.writeMicroseconds(DOOR_OPEN);
    delay(200);
    do {
        weight_count--;
        holderMotor.writeMicroseconds(1350);
        delay(20);
        Serial.println("Weight Leaving");
        holderMotor.writeMicroseconds(1500);
        leftMotor.setSpeed(0.99);
        rightMotor.setSpeed(0.91);
        delay(400);
        leftMotor.setSpeed(-1*0.99);
        rightMotor.setSpeed(-1*91);
        delay(400);
        leftMotor.setSpeed(0.0);
        rightMotor.setSpeed(0.0);
        delay(1200);
        limitSwitchDoor_VAL = checkDoorPressed();
        limitSwitchWheel_VAL = io_2.digitalRead(limitSwitchWheel);
        rotating_clockwise = false;
        rotating=true;
        while(!rotate_to_next(true));
    } while (weight_count > 0);
    doorMotor.writeMicroseconds(DOOR_CLOSED);
    weight_count = 0;
    prev_weight_count = 0;
}


bool checkButtonPressed() {
  int raw = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  if (raw && ((now - lastPressTime) > BUTTON_COOLDOWN)) {
      lastPressTime = now;
      return 1;
  }
  return 0;
}

bool checkDoorPressed() {
  int raw = io_2.digitalRead(limitSwitchDoor);
  unsigned long now = millis();
  //Serial.printf("%d, %ld %ld\n", raw, lastSwitchTime, now);
  if (raw && ((now - lastSwitchTime) > PRESS_COOLDOWN)) {
    //Serial.printf("Door Pressed %d\n", raw);
    lastSwitchTime = now;
    return 1;
  }
  return 0;
}

// Showly makes the map less confident overtime, to avoid walls that havent been seen in ages that were wrong block pathing
void decayMap() {
    unsigned long now = millis();
    if ((now - previousDecayTime) > DECAY_COOLDOWN) {
        //Serial.print("Actually Decaying");
        for (int y = 0; y < MATRIX_ROWS; y++) {
            for (int x = 0; x < MATRIX_COLS; x++) {
                if (grid[y][x] < 0) {
                    grid[y][x]++;
                } else if (grid[y][x] > 0) {
                    grid[y][x]--;
                }
            }
        }
        previousDecayTime = now;
    }
}

std::vector<std::vector<int>> makePaddedGrid(const std::vector<std::vector<int>>& grid) 
{
    int rows = grid.size();
    int cols = grid[0].size();

    // Step 1: Threshold grid into 0/1
    std::vector<std::vector<int>> binary(rows, std::vector<int>(cols, 0));
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (grid[r][c] >= WALL_REQUIREMENT && !(currentX_int == c && currentY_int == r)) {
                binary[r][c] = 1;
            }
        }
    }

    // Step 2: Pad 1s into neighbors (radius = 2)
    std::vector<std::vector<int>> padded = binary;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (binary[r][c] == 1) {
                for (int dr = -4; dr <= 4; dr++) {
                    for (int dc = -4; dc <= 4; dc++) {
                        if (dr == 0 && dc == 0) continue; // skip self
                        int nr = r + dr;
                        int nc = c + dc;
                        if (nr >= 0 && nr < rows && nc >= 0 && nc < cols &&
                            !(currentX_int == nc && currentY_int == nr)) {
                            padded[nr][nc] = 1;
                        }
                    }
                }
            }
        }
    }
    // ACCOUNT FOR RAMP
    for (int y = ramp_bottom_left.y; y <= ramp_top_right.y && y < MATRIX_ROWS; ++y) {
        for (int x = ramp_bottom_left.x; x <= ramp_top_right.x && x < MATRIX_COLS; ++x) {
            if (y >= 0 && x >= 0) { // check bounds
                //Serial.printf("(%d, %d)\n", x, y);
                padded[y][x] = 1;
            }
        }
    }

    // ACCOUNT FOR BASES
    for (int r = 0; r < BASE_SIZE; r++) {
        for (int c = 0; c < BASE_SIZE; c++) {
            padded[r][c] = 0;
        }
    }
    for (int r = (MATRIX_ROWS - 1 - BASE_SIZE); r < MATRIX_ROWS-1; r++) {
        for (int c = 0; c < BASE_SIZE; c++) {
            padded[r][c] = 0;
        }
    }

    // Step 3: Pad matrix edges & corners by 4 cells (robot radius)
    int cornerPad = 3;

    // Top and bottom edges
    for (int y = 0; y < cornerPad; y++) {
        for (int x = 0; x < cols; x++) {
            padded[y][x] = 1;
            padded[rows - 1 - y][x] = 1;
        }
    }

    // Left and right edges
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cornerPad; x++) {
            padded[y][x] = 1;
            padded[y][cols - 1 - x] = 1;
        }
    }

    // Corners — explicitly fill 4×4 blocks
    // for (int y = 0; y < cornerPad; y++) {
    //     for (int x = 0; x < cornerPad; x++) {
    //         padded[y][x] = 1; // top-left
    //         padded[y][cols - 1 - x] = 1; // top-right
    //         padded[rows - 1 - y][x] = 1; // bottom-left
    //         padded[rows - 1 - y][cols - 1 - x] = 1; // bottom-right
    //     }
    // }

    return padded;
}



void checkWeights(){
    if (dumping_plastic) { 
        Serial.println("Dumping Plastic");
        doorMotor.writeMicroseconds(DOOR_OPEN);
        if ((millis() - startDumpTime) > DUMPING_TIME) {
            Serial.println("Finished Dumping");
            dumping_plastic = false;
            doorMotor.writeMicroseconds(DOOR_CLOSED);
            //doorMotor.writeMicroseconds(DOOR_OPEN);
        }
    } 
  else {
    //doorMotor.writeMicroseconds(DOOR_OPEN);
    doorMotor.writeMicroseconds(DOOR_CLOSED);
  }
  limitSwitchDoor_VAL = checkDoorPressed();
  if (limitSwitchDoor_VAL && !rotating) {
    delay(10);
    inductiveSensorVAL = analogRead(INDUCTIVE_PIN);
    if (inductiveSensorVAL > 250) {
      Serial.println("Found Plastic");
      doorMotor.writeMicroseconds(DOOR_OPEN);
      dumping_plastic = true;
      startDumpTime = millis();
    } else if (inductiveSensorVAL < 250 && weight_count != MAX_WEIGHT_COUNT) {
      Serial.println("Found Metal");
      weight_count++;
      if (weight_count != MAX_WEIGHT_COUNT) {
        rotating_clockwise = true;
        rotating = true;
      }
      Serial.println("Found weight");
    }
  }
}

Point pickMostUnseenLocation() {   
    const int margin = 3;       // avoid edges
    const int maxSeenCap = 10;  // any value above this is treated as this

    struct WeightedPoint {
        Point p;
        double weight;
    };

    std::vector<WeightedPoint> candidates;
    candidates.reserve((MATRIX_ROWS - 2 * margin) * (MATRIX_COLS - 2 * margin));

    int globalMin = std::numeric_limits<int>::max();
    int globalMax = 0;

    // First pass: find min and max seen values (after capping)
    for (int y = margin; y < MATRIX_ROWS - margin; ++y) {
        for (int x = margin; x < MATRIX_COLS - margin; ++x) {
            if (padded_grid[y][x] == 1) continue;  // skip walls

            int seen = std::min(seen_grid[y][x], maxSeenCap);
            if (seen < globalMin) globalMin = seen;
            if (seen > globalMax) globalMax = seen;
        }
    }

    if (globalMin == std::numeric_limits<int>::max()) {
        // no valid cells
        return {0, 0};
    }

    // Second pass: assign weights (favor lower 'seen' values)
    // Weight function: weight = (globalMax - seen + 1)^2   (quadratic bias)
    for (int y = margin; y < MATRIX_ROWS - margin; ++y) {
        for (int x = margin; x < MATRIX_COLS - margin; ++x) {
            if (padded_grid[y][x] == 1) continue;

            int seen = std::min(seen_grid[y][x], maxSeenCap);
            double weight = std::pow((globalMax - seen + 1), 2.0);
            candidates.push_back({{x, y}, weight});
        }
    }

    if (candidates.empty()) return {0, 0};

    // Weighted random pick
    static std::random_device rd;
    static std::mt19937 gen(rd());

    double totalWeight = 0.0;
    for (const auto& c : candidates) totalWeight += c.weight;

    std::uniform_real_distribution<> dist(0.0, totalWeight);
    double r = dist(gen);

    for (const auto& c : candidates) {
        r -= c.weight;
        if (r <= 0.0) {
            return c.p;
        }
    }

    // Fallback (shouldn't happen)
    return candidates.back().p;
}


void addRandomLocation() 
{
    uint8_t randomAttempts = 0;
    Point randomized_location = {};

    do {
        if (randomAttempts == 10) {
            resetGrids();
            randomAttempts = 0;
        }
        randomAttempts++;
        start_time = millis();
        randomized_location = pickMostUnseenLocation();
        Serial.printf("RAND TIME: %d\n", millis()-start_time);
        Serial.printf("RAND (%d, %d)\n", randomized_location.x, randomized_location.y);
    } while (randomized_location.x == 0 && randomized_location.y == 0);
    designated_position_list.push_back(randomized_location);
    Serial.println("Added random");
    if (designated_position_list.size() != 0) {
        Serial.print("[");
        for (size_t i = 0; i < designated_position_list.size(); i++) {
            Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
            if (i < designated_position_list.size() - 1) Serial.print(", ");
        }
        Serial.println("]");
    }
    Serial.println();
}


void resetGrids() {
    for (int r = 0; r < MATRIX_ROWS; r++) {
        for (int c = 0; c < MATRIX_COLS; c++) {
            grid[r][c] = 0;
            padded_grid[r][c] = 0;
            seen_grid[r][c] = 0;
        }
    }
    currentAction = NEW_PATH_REQUIRED;
}


void updateGridInfo() {
    uint16_t TOF_distance_L0_LeftBottom = sensorsL0[1].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    uint16_t TOF_distance_L0_RightBottom = sensorsL0[0].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    uint16_t TOF_distance_L1_Middle = sensorsL1[0].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    updateMatrix(TOF_distance_L1_Middle, 0.0);
    uint16_t TOF_distance_L1_LeftTop = sensorsL1[2].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    updateMatrix(TOF_distance_L1_LeftTop, -37.25);
    uint16_t TOF_distance_L1_LeftMiddle = sensorsL1[4].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    updateMatrix(TOF_distance_L1_LeftMiddle, -12);
    uint16_t TOF_distance_L1_RightTop = sensorsL1[1].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    updateMatrix(TOF_distance_L1_RightTop, 37.25);
    uint16_t TOF_distance_L1_RightMiddle = sensorsL1[3].readRangeContinuousMillimeters() + ROBOT_RADIUS;
    updateMatrix(TOF_distance_L1_RightMiddle, 12);

    // Serial.printf("LEFT_TOP: %d : LEFT_BOT: %d\n", TOF_distance_L1_LeftTop, TOF_distance_L0_LeftBottom);
    // Serial.printf("RIGHT_TOP: %d : RIGHT_BOT: %d\n", TOF_distance_L1_RightTop, TOF_distance_L0_RightBottom);
    // Serial.printf("LEFT: %d\n", TOF_distance_L1_LeftMiddle);
    // Serial.printf("MIDDLE: %d\n", TOF_distance_L1_Middle);
    // Serial.printf("RIGHT: %d\n", TOF_distance_L1_RightMiddle);

    // CHECK FOR WEIGHTS
    if (WEIGHT_DETECTION && !in_drop_off_mode) {
        // Serial.printf("GYRO: %lf, PITCH: %lf\n", angle_degrees, angle_pitch);
        if ((TOF_distance_L0_LeftBottom < WEIGHT_DETECTION_RANGE) && ((TOF_distance_L1_LeftTop - TOF_distance_L0_LeftBottom) > WEIGHT_DETECTION_REQUIREMENT)) {
            if (left_spotted_count == WEIGHT_BUFFER_REQUIRED) {
                left_spotted_count = 0;
                int x_weight = round((currentX-50*cos(angle_degrees)+(TOF_distance_L0_LeftBottom+50) * cos(angle_radians+(-37.25*PI/180)))/CELL_SIZE);
                int y_weight = round((currentY-50*sin(angle_degrees)+(TOF_distance_L0_LeftBottom+50) * sin(angle_radians+(-37.25*PI/180)))/CELL_SIZE);
                if (y_weight >= 1 && y_weight < MATRIX_ROWS-1 && x_weight >= 1 && x_weight < MATRIX_COLS-1 && !padded_grid[y_weight][x_weight] && (designated_position_list[0].x != x_weight && designated_position_list[0].y != y_weight)) {
                    int tally = 0;
                    int already_travelling = 0;
                    for (Point weight_location : weight_location_list) {
                        for (Point designated_position : designated_position_list) {
                            if (weight_location == designated_position) {
                                tally++;
                            }
                            
                        }
                        //Serial.println("{%d <= %d <= %d}, {%d <= %d <= %d}\n", weight_location.x-2, x_weight, weight_location.x+2, weight_location.y-2, y_weight, weight_location.y+2);
                        if (weight_location.x-2 <= x_weight && x_weight <= weight_location.x+2 && weight_location.y-2 <= y_weight && y_weight <= weight_location.y+2) {
                            already_travelling = 1;
                        }
                    }
                    if (!already_travelling) {
                        Serial.printf("Weight found left at: (%d, %d)\n", x_weight, y_weight);
                        designated_position_list.insert(designated_position_list.begin() + tally, {x_weight, y_weight});
                        if (designated_position_list.size() != 0) {
                            Serial.print("[");
                            for (size_t i = 0; i < designated_position_list.size(); i++) {
                                Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
                                if (i < designated_position_list.size() - 1) Serial.print(", ");
                            }
                            Serial.println("]");
                        }
                        Serial.println();
                        weight_location_list.push_front({x_weight, y_weight});    
                    }
                }
            } else {
                left_spotted_count++;
            }

        } else {
            if (left_spotted_count > 0) {
                left_spotted_count--;
            }
        }
        if ((TOF_distance_L0_RightBottom < WEIGHT_DETECTION_RANGE) && ((TOF_distance_L1_RightTop - TOF_distance_L0_RightBottom) > WEIGHT_DETECTION_REQUIREMENT)) {
            if (right_spotted_count == WEIGHT_BUFFER_REQUIRED) {
                right_spotted_count = 0;
                int x_weight = round((currentX+(TOF_distance_L0_RightBottom+50) * cos(angle_radians+(37.25*PI/180)))/CELL_SIZE);
                int y_weight = round((currentY+(TOF_distance_L0_RightBottom+50) * sin(angle_radians+(37.25*PI/180)))/CELL_SIZE);
                if (y_weight >= 1 && y_weight < MATRIX_ROWS-1 && x_weight >= 1 && x_weight < MATRIX_COLS-1 && !padded_grid[y_weight][x_weight]) {
                    int tally = 0;
                    int already_travelling = 0;
                    for (Point weight_location : weight_location_list) {
                        for (Point designated_position : designated_position_list) {
                            if (weight_location == designated_position) {
                                tally++;
                            }
                            
                        }
                        //Serial.println("{%d <= %d <= %d}, {%d <= %d <= %d}\n", weight_location.x-2, x_weight, weight_location.x+2, weight_location.y-2, y_weight, weight_location.y+2);
                        if (weight_location.x-2 <= x_weight && x_weight <= weight_location.x+2 && weight_location.y-2 <= y_weight && y_weight <= weight_location.y+2) {
                            already_travelling = 1;
                        }
                    }
                    if (!already_travelling) {
                        Serial.printf("Weight found right at: (%d, %d)\n", x_weight, y_weight);
                        designated_position_list.insert(designated_position_list.begin() + tally, {x_weight, y_weight});
                         if (designated_position_list.size() != 0) {
                            Serial.print("[");
                            for (size_t i = 0; i < designated_position_list.size(); i++) {
                                Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
                                if (i < designated_position_list.size() - 1) Serial.print(", ");
                            }
                            Serial.println("]");
                        }
                        Serial.println();
                        weight_location_list.push_front({x_weight, y_weight});    
                    }
                    currentAction = NEW_PATH_REQUIRED;
                }
            } else {
                right_spotted_count++;
            }

        } else {
            if (right_spotted_count > 0) {
                right_spotted_count--;
            }
        }
    }
    
    // Position and Rotation RAN directly after TOF for best results

}

/* 
MAIN LOOP
*/
void loop()
{
    limitSwitchWheel_VAL = io_2.digitalRead(limitSwitchWheel);
    if (limitSwitchWheel_VAL && !rotating) {
        Serial.println("Not aligned");
        holderMotor.writeMicroseconds(1720);
    } else if (!rotating) {
        holderMotor.writeMicroseconds(1500);
    }

    if (weight_count == 2) {
        spinnerMotor.setSpeed(0.0);
    } else {
        spinnerMotor.setSpeed(0.9);
    }
    //Updates Current Rotation
    updateRotation();

    //Updates Current Position
    updatePosition();

    // UPDATING GRID INFO
    // IF SENSORS FAIL COMPLETELY AVOID USING THEM
    if (!failed_sensors) {
        start_time = millis();
        updateGridInfo();
        if (millis() - start_time >= 300) {
            Serial.println("Turning Off");
            leftMotor.setSpeed(0.0);
            rightMotor.setSpeed(0.0);
            failed_sensors = true;
        }
    }
    if (!ignore_sensors && failed_sensors) {
        Serial.println("Sensors failed");
        delay(50);
        resetSensors();
        start_time = millis();
        updateGridInfo();
        if (millis() - start_time < 300) {
            failed_sensors = false;
        } else {
            ignore_sensors = true;
        }
    }

    padded_grid = makePaddedGrid(grid);

    //Check for weight updates
    if (!in_drop_off_mode) {
        checkWeights();
    }
    rotate_to_next(false);
    decayMap();

    if ((checkButtonPressed() || weight_count == MAX_WEIGHT_COUNT) && !in_drop_off_mode) {
        Serial.println("Two weights found");
        currentAction = DROP_OFF_START;
    }

    //PATH PRINT
    // Serial.println();
    // Serial.printf("[(%d, %d)]   ", currentX_int, currentY_int);
    // if (path.size() != 0) {
    //     Serial.print("[");
    //     for (size_t i = 0; i < path.size(); i++) {
    //         Serial.printf("(%d,%d)", path[i].x, path[i].y);
    //         if (i < path.size() - 1) Serial.print(", ");
    //     }
    //     Serial.print("]        ");
    // }

    // if (designated_position_list.size() != 0) {
    //     Serial.print("[");
    //     for (size_t i = 0; i < designated_position_list.size(); i++) {
    //         Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
    //         if (i < designated_position_list.size() - 1) Serial.print(", ");
    //     }
    //     Serial.println("]");
    // }
    // Serial.println();

    // //GRID PRINT
    // padded_grid = makePaddedGrid(grid);
    // for (Point location : path) {
    //     Serial.printf("{%d, %d} ", location.x, location.y);
    // }
    // Serial.println();
    // for (int y = 0; y < MATRIX_ROWS; y++) {
    //     for (int x = 0; x < MATRIX_COLS; x++) {
    //         bool already_drawn = false;
    //         for (Point location : path) {
    //             if (location.x == x && location.y == y) {
    //                 already_drawn = true;
    //                 Serial.print("* ");
    //             }
    //         }
    //         if (!already_drawn) {
    //             if (x == currentX_int && y == currentY_int) {
    //                 Serial.printf("4 ");
    //             } else if (padded_grid[y][x] == 1) {
    //                 Serial.printf("1 ");
    //             } else {
    //                 Serial.printf("0 ");
    //             }
    //         }

    //     }
    //     Serial.println();
    // }
    // Serial.println("\n\n\n\n\n\n");

    switch (currentAction) {
        case DRIVING:
            //Serial.println("Driving");
            got_to_goal = driveTowardsGoal();
            if (got_to_goal) {
                Serial.println("Got to location");
                path.pop_front();
                Serial.printf("[(%d, %d)]   ", currentX_int, currentY_int);
                if (path.size() != 0) {
                    Serial.print("[");
                    for (size_t i = 0; i < path.size(); i++) {
                        Serial.printf("(%d,%d)", path[i].x, path[i].y);
                        if (i < path.size() - 1) Serial.print(", ");
                    }
                    Serial.print("]        ");
                }
                Serial.println();
                
            }
            if (path.size() == 0) {
                if (!in_drop_off_mode) {
                    Serial.println("Path size 0");
                    auto it = find(weight_location_list.begin(), weight_location_list.end(), designated_position_list[0]);
                    if (it != weight_location_list.end()) {
                        weight_location_list.erase(it);
                    }
                    Serial.println("POPPING");
                    designated_position_list.pop_front();
                    if (designated_position_list.size() == 0) {
                        leftMotor.setSpeed(0.0);
                        rightMotor.setSpeed(0.0);
                        Serial.println("No locations, randomizing");
                    }
                    currentAction = NEW_PATH_REQUIRED;
                } else {
                    currentAction = DROP_OFF;
                }

            }
            break;
        case NEW_PATH_REQUIRED:
            //Serial.println("New path required");
            //Serial.printf("Current: {%d, %d}, Target: {%d, %d}\n", currentX_int, currentY_int, designated_position_list[0].x, designated_position_list[0].y);
            if (designated_position_list.size() == 0) {
                uint8_t failed_to_find_path = 0;
                do {
                    if (failed_to_find_path == 5) {
                        resetGrids();
                    }
                    failed_to_find_path++;
                    Serial.println("NEW PATH NO DESIGNATED POSITIONS");
                    leftMotor.setSpeed(0.0);
                    rightMotor.setSpeed(0.0);
                    addRandomLocation();
                    full_path = astar_search({currentX_int, currentY_int}, {designated_position_list[0].x, designated_position_list[0].y});
                    path = simplify_path(full_path);
                } while (path.size() == 0);
            } else {
                if (((designated_position_list[0].x == currentX_int && designated_position_list[0].y == currentY_int) || (padded_grid[designated_position_list[0].y][designated_position_list[0].x])) && !in_drop_off_mode) {
                    Serial.println("Position already at");
                    auto it = find(weight_location_list.begin(), weight_location_list.end(), designated_position_list[0]);
                    if (it != weight_location_list.end()) {
                        weight_location_list.erase(it);
                    }
                    Serial.println("POPPING");
                    designated_position_list.pop_front();
                    currentAction = NEW_PATH_REQUIRED;
                } else {
                    full_path = astar_search({currentX_int, currentY_int}, {designated_position_list[0].x, designated_position_list[0].y});
                    path = simplify_path(full_path);
                    currentAction = DRIVING;
                }
            }
            

            
            // Serial.print("Designated Position: ");
            // if (designated_position_list.size() != 0) {
            //     Serial.print("[");
            //     for (size_t i = 0; i < designated_position_list.size(); i++) {
            //         Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
            //         if (i < designated_position_list.size() - 1) Serial.print(", ");
            //     }
            //     Serial.println("]");
            // }

            // Serial.printf("Current: [(%d, %d)]\n", currentX_int, currentY_int);
            // Serial.print("Path : ");
            // if (path.size() != 0) {
            //     Serial.print("[");
            //     for (size_t i = 0; i < path.size(); i++) {
            //         Serial.printf("(%d,%d)", path[i].x, path[i].y);
            //         if (i < path.size() - 1) Serial.print(", ");
            //     }
            //     Serial.print("]        ");
            // }
            // Serial.println();

            // for (int y = 0; y < MATRIX_ROWS; y++) {
            //     for (int x = 0; x < MATRIX_COLS; x++) {
            //         bool already_placed = 0;
            //         for (Point next_location : path) {
            //             if (next_location.x == x && next_location.y == y) {
            //                 already_placed = 1;
            //                 Serial.printf("* ");
            //             }
            //         }
            //         if (!already_placed) {
            //             if (x == designated_position_list[0].x && y == designated_position_list[0].y) {
            //                 Serial.printf ("* ");
            //             } else if (x == currentX_int && y == currentY_int) {
            //                 Serial.printf("4 ");
            //             } else if (padded_grid[y][x] == 1) {
            //                 Serial.printf("1 ");
            //             } else {
            //                 Serial.printf("0 ");
            //             }
            //         }
            //     }
            //     Serial.println();
            // }
            // Serial.println();

            // for (int y = 0; y < MATRIX_ROWS; y++) {
            //     for (int x = 0; x < MATRIX_COLS; x++) {
            //         int value = seen_grid[y][x];
            //         if (value >= 9) {
            //             Serial.print("9 ");
            //         } else {
            //             Serial.printf("%d ", value);
            //         }
            //     }
            //     Serial.println();
            // }
            
            // Serial.println();
            break;
        case DROP_OFF:
            Serial.printf("Current=(%d, %d), Home=(%d, %d)\n", currentX_int, currentY_int, home_base.x, home_base.y);
            Serial.print("PATH: ");
            
            if (path.size() != 0) {
                Serial.print("[");
                for (size_t i = 0; i < path.size(); i++) {
                    Serial.printf("(%d,%d)", path[i].x, path[i].y);
                    if (i < path.size() - 1) Serial.print(", ");
                }
                Serial.print("]        ");
            }
            Serial.println();
            Serial.printf("DESIGNATED POSITIONS: ");
            if (designated_position_list.size() != 0) {
                Serial.print("[");
                for (size_t i = 0; i < designated_position_list.size(); i++) {
                    Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
                    if (i < designated_position_list.size() - 1) Serial.print(", ");
                }
                Serial.println("]");
            }
            Serial.println();
            Serial.println("In Position");
            while(!rotateRobot(drop_off_angle, true)) {
                updateRotation();
                updatePosition();
            }
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            Serial.println("in drop off");
            Serial.println("POPPING");
            designated_position_list = {};

            // tcs.setInterrupt(false);      // turn on LED
            // delay(60);  // takes 50ms to read 
            // tcs.getRawData(&red, &green, &blue, &clear);
            // tcs.setInterrupt(true);  // turn off LED
            // sum = clear;
            // g = green; g /= sum;
            // b = blue; b /= sum;
            // g *= 256; b *= 256;
            // if (start == "GREEN") {
            //     if (g > 150) {
            //         drop_weights();
            //     } else {
            //         while(1);
            //     }
                
            // } else {
            //     if (b > 150) {
            //         drop_weights();
            //     } else {
            //         while(1);
            //     }
            // }
            drop_weights();
            in_drop_off_mode = false;
            currentAction = NEW_PATH_REQUIRED;
            break;
        case DROP_OFF_START:
            designated_position_list.push_front(home_base);
            if (designated_position_list.size() != 0) {
                Serial.print("[");
                for (size_t i = 0; i < designated_position_list.size(); i++) {
                    Serial.printf("(%d,%d)", designated_position_list[i].x, designated_position_list[i].y);
                    if (i < designated_position_list.size() - 1) Serial.print(", ");
                }
                Serial.println("]");
            }
            Serial.println();
            in_drop_off_mode = true;
            currentAction = NEW_PATH_REQUIRED;
            break;
        default:
            //Serial.println("You bugging");
            break;
    }
    delay(5);
}
