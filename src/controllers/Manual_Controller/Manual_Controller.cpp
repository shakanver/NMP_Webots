// File:          NMP_Controller.cpp
// Date:          23/03/2021
// Description:   A controller used to manually control the e-puck
// Author:        Stephanie Liaw
// Modifications: Added a Description

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Gyro.hpp>
#include <limits>
#include <tuple>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define INPUT_SPEED 9
#define SENSOR_ARRAY_LENGTHS 3
#define MAX_MOTOR_SPEED 9

void redManualMove(const int key, webots::Motor *leftMotor, webots::Motor *rightMotor);
void moveBackward(webots::Motor *leftMotor, webots::Motor *rightMotor);
void turnRight(webots::Motor *leftMotor, webots::Motor *rightMotor);
void turnLeft(webots::Motor *leftMotor, webots::Motor *rightMotor);
void moveForward(webots::Motor *leftMotor, webots::Motor *rightMotor);
void stop(webots::Motor *leftMotor, webots::Motor *rightMotor);    

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  //Lidar *lidar = robot->getLidar("lidar");
  Motor *lmotor = robot->getMotor("left wheel motor");
  Motor *rmotor = robot->getMotor("right wheel motor");
  DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
  DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
  DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
  DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
  DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
  DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
  Camera * cam = robot->getCamera("camera");
  Keyboard *keyboard = robot->getKeyboard();
  //Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
  //Gyro *gyro = robot->getGyro("gyro");
      

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();


  // Enable the sensors, feel free to change the sampling rate
  //lidar->enable(50);
  frontLeftDs->enable(timeStep);
  frontRightDs->enable(timeStep);
  leftDs->enable(timeStep);
  rightDs->enable(timeStep);
  leftFrontLeftDs->enable(timeStep);
  rightFrontRightDs->enable(timeStep);
  //accelerometer->enable(timeStep);
  //gyro->enable(timeStep);
  cam->enable(timeStep);
  keyboard->enable(timeStep);
  
  lmotor->setPosition(std::numeric_limits<double>::infinity());
  rmotor->setPosition(std::numeric_limits<double>::infinity());
  lmotor->setVelocity(0);
  rmotor->setVelocity(0);
  int key {-1};

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
        const int prevKey = key;
        key = keyboard->getKey();
        if(key != prevKey){
            lmotor->setPosition(INFINITY);
            rmotor->setPosition(INFINITY);
            redManualMove(key, lmotor, rmotor);
        }
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

//Settng keys to move the robot manually
void redManualMove(const int key, webots::Motor *leftMotor, webots::Motor *rightMotor){
    switch(key){
        case Keyboard::UP:
            moveForward(leftMotor, rightMotor);
            break;
        case Keyboard::DOWN:
            moveBackward(leftMotor, rightMotor);
            break;
        case Keyboard::LEFT:
            turnLeft(leftMotor, rightMotor);
            break;
        case Keyboard::RIGHT:
            turnRight(leftMotor, rightMotor);
            break;
        case ' ':
            stop(leftMotor, rightMotor);
            break;

    }
}


void moveForward(webots::Motor *leftMotor, webots::Motor *rightMotor){
    leftMotor->setVelocity(INPUT_SPEED);
    rightMotor->setVelocity(INPUT_SPEED);

}

void moveBackward(webots::Motor *leftMotor, webots::Motor *rightMotor){
    leftMotor->setVelocity(-1 *INPUT_SPEED);
    rightMotor->setVelocity(-1 *INPUT_SPEED);
}

void turnRight(webots::Motor *leftMotor, webots::Motor *rightMotor){

    leftMotor->setVelocity(1*INPUT_SPEED);
    rightMotor->setVelocity(0.5*INPUT_SPEED);  
}

void turnLeft(webots::Motor *leftMotor, webots::Motor *rightMotor){

    leftMotor->setVelocity(0.5*INPUT_SPEED);
    rightMotor->setVelocity(1*INPUT_SPEED);
}
void stop(webots::Motor *leftMotor, webots::Motor *rightMotor){

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
}
