// File:          NMP_Controller.cpp
// Date:          7/04/2021
// Description:   A controller used to manually control the e-puck
// Author:        Liam Garde, 
// Modifications: added a algorithm to detect circular landmarks

#include <iostream> 
#include <iomanip> 
#include <vector>
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

struct Point {
    float x;
    float y;
};
#define INPUT_SPEED 7
#define MAX_MOTOR_SPEED 5

#define CONE_RADIUS 0.1 // radius for detecting a cone
#define CONE_TO_CONE_RADIUS 0.3 // radius for joining cones together
// manual movement option
void redManualMove(const int key1, const int key2, webots::Motor *leftMotor, webots::Motor *rightMotor);
std::vector<Point> findCircles(const std::vector<Point> map, const float , unsigned int point_threshold);

int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();
    Lidar *lidar = robot->getLidar("lidar");
    Motor *lMotor = robot->getMotor("left wheel motor");
    Motor *rMotor = robot->getMotor("right wheel motor");
    Display *disp = robot->getDisplay("display");
    //Display *disp2 = robot->getDisplay("display2");
    //Camera * cam = robot->getCamera("camera");
    Keyboard *keyboard = robot->getKeyboard();
    // Camera * LeftCam = robot->getLeftCamera("LeftCamera"); // this isn't added to the robot yet
    //Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
    //Gyro *gyro = robot->getGyro("gyro");
    int timeStep = (int)robot->getBasicTimeStep();
    lidar->enable(1);
    //accelerometer->enable(timeStep);
    //gyro->enable(timeStep);
    //cam->enable(timeStep);
    keyboard->enable(timeStep);
    lMotor->setPosition(INFINITY);
    rMotor->setPosition(INFINITY);
    lMotor->setVelocity(0);
    rMotor->setVelocity(0);
    // init
    const int size = lidar->getHorizontalResolution();
    const float maxRange = lidar->getMaxRange();
    //const float minRange = lidar->getMinRange();
    int stepcount = 0;
    // main loop
    while (robot->step(timeStep) != -1) {
        
        int key1 = keyboard->getKey();
        int key2 = keyboard->getKey();
        redManualMove(key1,key2, lMotor, rMotor);
        // get lidar data
        const float *arr = lidar->getLayerRangeImage(0); // values range from 0.0 to 1.0
        // display and calculate lidar data as a point cloud
        const float offset = - lidar->getFov()/2 - M_PI/2;
        const int disp_w = disp->getWidth();
        const int disp_h = disp->getHeight();
        disp->setColor(0xFFFFFF);
        disp->fillRectangle(0,0,disp_w, disp_h);
        // vector to store lidar data calculated in the x,y plane
        std::vector<Point> map; 
        float k = 0.4 * disp_w;// display scaling factor in pixels
        // iterate over lidar data
        for (int i = 0; i < size; i++) {
            float angle = lidar->getFov() * i / size; // make the angle go from 0 to fov
            float tempx = cos(angle + offset); // max value for each angle to show on map
            float tempy = sin(angle + offset);
            Point data;
            if (arr[i] != maxRange) {
                data.x = arr[i] * tempx;
                data.y = arr[i] * tempy;
                map.push_back(data);
            }
            float xmax = tempx * k / maxRange;
            float ymax = tempy * k / maxRange;
            float xdraw = arr[i] * xmax;
            float ydraw = arr[i] * ymax;
            // draw bottom lines to show fov
            disp->setColor(0x00FF00);
            disp->drawLine(disp_w/2, disp_h/2, disp_w/2 + k * cos(offset), disp_h/2 + k * sin(offset));
            disp->drawLine(disp_w/2, disp_h/2,disp_w/2 + k * cos(offset + lidar->getFov()), disp_h/2 + k * sin(offset + lidar->getFov()));
            // draw points
            disp->setColor(0xFF0000);
            disp->drawPixel(disp_w/2 + xdraw, disp_h/2 + ydraw);
            disp->setColor(0x0000FF);
            // robot symbol
            disp->fillOval(disp_w/2, disp_h/2, 7, 7);
        }
        
        std::cout << map.size() << ", ";
        const unsigned int point_threshold = 6; // number of points in a close area required to make a new point
        map = findCircles(map,CONE_RADIUS,point_threshold);
        std::cout << map.size() << "\n";

        // draw cones and lines between them 
        disp->setColor(0x0000FF);
        for (auto p: map) {
            disp->drawOval(disp_w/2 + k*p.x, disp_h/2 + k*p.y,10,10);
        }
        for (unsigned int i = 0; i < map.size(); i++) {
            Point p1 = map.at(i);
            for (unsigned int j = i + 1; j < map.size(); j++) {
                Point p2 = map.at(j);
                if ( ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)) < CONE_TO_CONE_RADIUS*CONE_TO_CONE_RADIUS) {
                    disp->drawLine(disp_w/2 + k*p1.x, disp_h/2 + k*p1.y, disp_w/2 + k*p2.x, disp_h/2 + k*p2.y);
                }
            }
        }

        // sketchy longest vector algorithm
        float left_avg = 0;
        float right_avg = 0;
        for (int i = 0; i < size/2; i++) {
            left_avg += arr[i];
            right_avg += arr[i + size/2];
        }
        left_avg = 2 * left_avg / size; // sum / n
        right_avg = 2 * right_avg / size;
            
        float avg_sensor_diff = right_avg - left_avg;
        
        
        float gain = 30;
        float left_speed = INPUT_SPEED + (avg_sensor_diff * gain);
        float right_speed = INPUT_SPEED - (avg_sensor_diff * gain);
        lMotor->setVelocity(left_speed);
        rMotor->setVelocity(right_speed);
        // std::cout << "left,right,diff = "<< left_avg << ", " << right_avg << ", " << avg_sensor_diff << "\n";
        // std::cout << "speeds: left,right = "<< left_speed << ", " << right_speed  << "\n";
        stepcount++;
  };


  delete robot;
  return 0;
}

std::vector<Point> findCircles(std::vector<Point> map, const float r, unsigned int point_threshold) {
        std::vector<Point> newMap;
        // start at the first point
        // iterate through all the following points, until a very distant point is found
        // once this point is found, calculate the average of the region before it. then move on
        for (unsigned int i = 0; i < map.size(); i++) {
            Point p1 = map.at(i);
            Point ave = p1;
            for (unsigned int j = i + 1; j < map.size(); j++) {
                Point p2 = map.at(j);
                if (abs(p1.x - p2.x) > r || abs(p1.y - p2.y) > r || j == map.size()-1) {
                    // calc average of points
                    ave.x = ave.x / (j-i);
                    ave.y = ave.y/ (j-i);
                    if (j-i >= point_threshold) {
                        newMap.push_back(ave);
                    }
                    i = j-1;
                    break;
                } else {
                    ave.x += p2.x;
                    ave.y += p2.y;
                }
            }
        }
        return newMap;
}
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

    