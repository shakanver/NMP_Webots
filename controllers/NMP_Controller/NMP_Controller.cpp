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

using namespace webots;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
struct Point {
    float x;
    float y;
};
#define INPUT_SPEED 7
#define SENSOR_ARRAY_LENGTHS 3
#define MAX_MOTOR_SPEED 5

void redManualMove(const int key1, const int key2, webots::Motor *leftMotor, webots::Motor *rightMotor);


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  Lidar *lidar = robot->getLidar("lidar");
  Motor *lMotor = robot->getMotor("left wheel motor");
  Motor *rMotor = robot->getMotor("right wheel motor");
  // in order of left to right
  DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
  DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
  DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
  DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
  DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
  DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
  Display *disp = robot->getDisplay("display");
  //Display *disp2 = robot->getDisplay("display2");
  Camera * cam = robot->getCamera("camera");
  Keyboard *keyboard = robot->getKeyboard();
 // Camera * LeftCam = robot->getLeftCamera("LeftCamera"); // this isn't added to the robot yet
  //Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
  //Gyro *gyro = robot->getGyro("gyro");
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
  lMotor->setPosition(INFINITY);
  rMotor->setPosition(INFINITY);
  lMotor->setVelocity(0);
  rMotor->setVelocity(0);
  
  
    const int size = lidar->getHorizontalResolution();
    const float maxRange = lidar->getMaxRange();
    //const float minRange = lidar->getMinRange();
    int stepcount = 0;
    // main loop

  while (robot->step(timeStep) != -1) {
        
        int key1 = keyboard->getKey();
        int key2 = keyboard->getKey();
        redManualMove(key1,key2, lMotor, rMotor);

        const float *arr = lidar->getLayerRangeImage(0); // values range from 0.0 to 1.0

        
        // display lidar data as a display
        //Point points[size];
        // calculate points
        const float offset = - lidar->getFov()/2 - M_PI/2;
        const int disp_w = disp->getWidth();
        const int disp_h = disp->getHeight();
        disp->setColor(0xFFFFFF);
        disp->fillRectangle(0,0,disp_w, disp_h);
        for (int i = 0; i < size; i++) {
            float angle = lidar->getFov() * i / size;
            
            float k = 120.0;// display scaling factor in pixels
            float tempx = cos(angle + offset); // max value for each angle to show on map
            float tempy  = sin(angle + offset);
            float xmax = tempx * k / maxRange;
            float ymax = tempy * k / maxRange;
            float xdraw = arr[i] * xmax;
            float ydraw = arr[i] * ymax;
          
            
            
            
            disp->setColor(0x00FF00);
            disp->drawLine(disp_w/2, disp_h/2, disp_w/2 + k * cos(offset), disp_h/2 + k * sin(offset));
            disp->drawLine(disp_w/2, disp_h/2,disp_w/2 + k * cos(offset + lidar->getFov()), disp_h/2 + k * sin(offset + lidar->getFov()));
            disp->setColor(0x222222);
            disp->drawLine(disp_w/2 + xdraw, disp_h/2 + ydraw, disp_w/2 + xmax, disp_h/2 + ymax);
            disp->setColor(0xFF0000);
            disp->drawPixel(disp_w/2 + xdraw, disp_h/2 + ydraw);
            disp->setColor(0x0000FF);
            disp->fillOval(disp_w/2, disp_h/2, 7, 7);
            
            


            


        }
        
        //std::cout << '\n';
    // sketchy longest vector algorithm
    float left_avg = 0;
    float right_avg = 0;
    for (int i = 0; i < size/2; i++) {
        left_avg += arr[i];
        right_avg += arr[i + size/2];
    }
    left_avg = 2 * left_avg / size;
    right_avg = 2 * right_avg / size;
        
    float avg_sensor_diff = right_avg - left_avg;
    
    
    float gain = 3;
    float left_speed = INPUT_SPEED + (avg_sensor_diff * gain);
    float right_speed = INPUT_SPEED - (avg_sensor_diff * gain);
    lMotor->setVelocity(left_speed);
    rMotor->setVelocity(right_speed);
    std::cout << "left,right,diff = "<< left_avg << ", " << right_avg << ", " << avg_sensor_diff << "\n";
    std::cout << "speeds: left,right = "<< left_speed << ", " << right_speed  << "\n";
    stepcount++;
  };


  delete robot;
  return 0;
}

//Settng keys to move the robot manually
void redManualMove(const int key1, const int key2, webots::Motor *lMotor, webots::Motor *rMotor){
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
    lMotor->setVelocity(leftvel * MAX_MOTOR_SPEED);
    rMotor->setVelocity(rightvel * MAX_MOTOR_SPEED);
}

    