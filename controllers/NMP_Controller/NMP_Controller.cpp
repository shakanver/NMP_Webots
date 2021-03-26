// File:          NMP_Controller.cpp
// Date:          23/03/2021
// Description:   A controller used to manually control the e-puck
// Author:        Stephanie Liaw
// Modifications: Added a Description

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <iostream> 
#include <iomanip> 
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Gyro.hpp>
#include <webots/display.hpp>
#include <limits>
#include <tuple>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

#define INPUT_SPEED 10
#define SENSOR_ARRAY_LENGTHS 3
#define MAX_MOTOR_SPEED 5

void redManualMove(const int key1, const int key2, webots::Motor *leftMotor, webots::Motor *rightMotor);


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
   // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Lidar *lidar = robot->getLidar("lidar");
  Motor *lmotor = robot->getMotor("left wheel motor");
  Motor *rmotor = robot->getMotor("right wheel motor");
  // in order of left to right
  DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
  DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
  DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
  DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
  DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
  DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
  Display *disp = robot->getDisplay("display");
   


  Camera * cam = robot->getCamera("camera");
  Keyboard *keyboard = robot->getKeyboard();
 // Camera * LeftCam = robot->getLeftCamera("LeftCamera"); // this isn't added to the robot yet
  //Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
  //Gyro *gyro = robot->getGyro("gyro");
      

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();


  // Enable the sensors, feel free to change the sampling rate
  lidar->enable(2);
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
  lmotor->setPosition(INFINITY);
  rmotor->setPosition(INFINITY);

  lmotor->setVelocity(0);
  rmotor->setVelocity(0);
  


  /*
  //double gain = 0.005;
  //double left_dist = 0.0; //distance value of left sensor
  //double right_dist = 0.0; //distance valur of right sensor
  //double sensor_diff = 0.0; //sensor_diff = left_dist  - right_dist
  double left_speed = INPUT_SPEED;  //left motor speed
  double right_speed = INPUT_SPEED; //right motor speed

  double gain = 0.035;
  double left_distances[3] = {0, 0, 0};    //order: left, leftfrontleft, frontleft
  double right_distances[3] = {0, 0, 0};   //order: right, rightfrontright, frontright

  double left_avg = 0;
  double right_avg = 0;
  double avg_sensor_diff = 0;
  */

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
    
    const int disp_width = disp->getWidth();
    const int disp_height = disp->getHeight();
    const int size = lidar->getHorizontalResolution();
    
  while (robot->step(timeStep) != -1) {
        
        int key1 = keyboard->getKey();
        int key2 = keyboard->getKey();
            redManualMove(key1,key2, lmotor, rmotor);
     

    
    
        const float *arr = lidar->getLayerRangeImage(0); // values range from 0.0 to 1.0
       // std::cout << "time=" << robot->getTime() << "||";
        disp->setColor(0xFFFFFF);
        disp->fillRectangle(0,0,disp_width, disp_height);
        for (int i = 0; i < size; i++) {
            
            float angle = lidar->getFov() * i / size;
            float offset = lidar->getFov()/2 - M_PI;
            float k = 120.0;// display scaling factor
            float x = k * arr[i] * cos(angle + offset);
            float y = k * arr[i] * sin(angle + offset);
            disp->setColor(0x00FF00);
            disp->drawLine(disp_width/2, disp_height/2, disp_width/2 + k * cos(offset), disp_height/2 + k * sin(offset));
            disp->drawLine(disp_width/2, disp_height/2,disp_width/2 + k * cos(offset + lidar->getFov()), disp_height/2 + k * sin(offset + lidar->getFov()));
            disp->setColor(0xFF0000);
            disp->drawPixel(disp_width/2 + x, disp_height/2 + y);
            //std :: cout <<std::setprecision(3)<< arr[i] << ", ";
        }
        //std::cout << '\n';
    

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

//Settng keys to move the robot manually
void redManualMove(const int key1, const int key2, webots::Motor *leftMotor, webots::Motor *rightMotor){
    float leftvel = 0;
    float rightvel = 0;
    switch(key1){
        case Keyboard::UP:
            leftvel += 1;
            rightvel += 1;
            break;
        case Keyboard::DOWN:
            leftvel += -1;
            rightvel += -1;
            break;
        case Keyboard::LEFT:
            leftvel += -0.2;
            rightvel += 0.2;
            break;
        case Keyboard::RIGHT:
            leftvel += 0.2;
            rightvel += -0.2;
            break;
    }
    switch(key2){
        case Keyboard::UP:
            leftvel += 1;
            rightvel += 1;
            break;
        case Keyboard::DOWN:
            leftvel += -1;
            rightvel += -1;
            break;
        case Keyboard::LEFT:
            leftvel += -0.2;
            rightvel += 0.2;
            break;
        case Keyboard::RIGHT:
            leftvel += 0.2;
            rightvel += -0.2;
            break;
    }
    leftMotor->setVelocity(leftvel * MAX_MOTOR_SPEED);
    rightMotor->setVelocity(rightvel * MAX_MOTOR_SPEED);
}

    