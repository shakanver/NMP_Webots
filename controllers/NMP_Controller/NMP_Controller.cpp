// File:          NMP_Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
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

#define INPUT_SPEED 8.0
#define SENSOR_ARRAY_LENGTHS 3

//helper functions

//takes in an array, returns the maximum value of the array and its corresponding index
std::tuple<double, int> getmaxdiff(double values[])
{
  double max = values[0];
  int maxindex = 0;

  for (int i = 1; i < SENSOR_ARRAY_LENGTHS; i++)
  {
    if (abs(values[i]) >= abs(max))
    {
      max = values[i];
      maxindex = i;
    }
  }
  
  std::cout <<  "maxdiff: " << max <<  std::endl;

  return std::make_tuple(max, maxindex);
}

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
  DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
  DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
  DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
  DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
  DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
  DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
  Camera * cam = robot->getCamera("camera");
  Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
  Gyro *gyro = robot->getGyro("gyro");

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();


  // Enable the sensors, feel free to change the sampling rate
  lidar->enable(50);
  frontLeftDs->enable(timeStep);
  frontRightDs->enable(timeStep);
  leftDs->enable(timeStep);
  rightDs->enable(timeStep);
  leftFrontLeftDs->enable(timeStep);
  rightFrontRightDs->enable(timeStep);
  accelerometer->enable(timeStep);
  gyro->enable(timeStep);
  cam->enable(timeStep);
  
  lmotor->setPosition(std::numeric_limits<double>::infinity());
  rmotor->setPosition(std::numeric_limits<double>::infinity());
  lmotor->setVelocity(0);
  rmotor->setVelocity(0);

  //double gain = 0.005;
  //double left_dist = 0.0; //distance value of left sensor
  //double right_dist = 0.0; //distance valur of right sensor
  //double sensor_diff = 0.0; //sensor_diff = left_dist  - right_dist
  double left_speed = INPUT_SPEED;  //left motor speed
  double right_speed = INPUT_SPEED; //right motor speed

  double gains[3] = {0.01, 0.0075, 0.005}; //order: l&r, lfl&rfr, fl&fr
  double left_distances[3] = {0, 0, 0};    //order: left, leftfrontleft, frontleft
  double right_distances[3] = {0, 0, 0};   //order: right, rightfrontright, frontright
  double diffs[3] = {0, 0, 0};             //order: (left - right), (leftfrontleft - rightfrontright), (frontleft - frontright)

  double maxdiff;
  int maxdiffindex = 0;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  
    std::cout << "left velocity: " << left_speed << std::endl;
    std::cout << "right velocity: " << right_speed << std::endl;
    std::cout << "diff: " << maxdiff << std::endl;
    
    for (int i = 0; i < SENSOR_ARRAY_LENGTHS;  i++) {
      std::cout << "diffs" << i << ": " << diffs[i] << std::endl;
    }
    /*
    lmotor->setVelocity(left_speed);
    rmotor->setVelocity(right_speed);

    left_dist  = frontLeftDs->getValue();
    right_dist = frontRightDs->getValue();
    
    sensor_diff = left_dist - right_dist;
    
    left_speed  = 8 + (sensor_diff*gain);
    right_speed = 8 - (sensor_diff*gain);
     */

    lmotor->setVelocity(left_speed);
    rmotor->setVelocity(right_speed);

    //initialise left and right sensor arrays with sensor values
    left_distances[0] = leftDs->getValue();
    left_distances[1] = leftFrontLeftDs->getValue();
    left_distances[2] = frontLeftDs->getValue();

    right_distances[0] = rightDs->getValue();
    right_distances[1] = rightFrontRightDs->getValue();
    right_distances[2] = frontRightDs->getValue();

    diffs[0] = left_distances[0] - right_distances[0];
    diffs[1] = left_distances[1] - right_distances[1];
    diffs[2] = left_distances[2] - right_distances[2];

    auto [maxdiff,  maxdiffindex] = getmaxdiff(diffs);

    left_speed = INPUT_SPEED + (maxdiff * gains[maxdiffindex]);
    right_speed = INPUT_SPEED - (maxdiff * gains[maxdiffindex]);
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}