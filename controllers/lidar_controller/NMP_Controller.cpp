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
#include <algorithm>
#include <math.h>

using namespace webots;

struct Point {
    float x;
    float y;
};
struct Landmark {
    Point pos;
    int age;
};
struct PointPair {
    Point p1;
    Point p2;
};
#define BASE_SPEED 7
#define DIF_SPEED 2

#define CLUSTER_RADIUS 0.01 // radius for detecting a cone
#define CONE_RADIUS 0.03
#define CONE_TO_CONE_RADIUS 0.3 // radius for joining cones together
#define MIN_CLUSTER_SIZE 10
std::vector<Point> findCircles(const std::vector<Point> radar_points, const float , unsigned int point_threshold);
float dist2(Point p1, Point p2) {
    return ((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
};
int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();
    Lidar *lidar = robot->getLidar("lidar");
    Motor *lMotor = robot->getMotor("left wheel motor");
    Motor *rMotor = robot->getMotor("right wheel motor");
    Display *disp = robot->getDisplay("display");
    int timeStep = (int)robot->getBasicTimeStep();
    lidar->enable(1);
    lMotor->setPosition(INFINITY);
    rMotor->setPosition(INFINITY);
    lMotor->setVelocity(0);
    rMotor->setVelocity(0);
    // init

    const int size = lidar->getHorizontalResolution();
    const float maxRange = lidar->getMaxRange();
    //const float minRange = lidar->getMinRange();
        
        std::vector<Landmark> landmarks; 
    int stepcount = 0;
    const float offset = - lidar->getFov()/2 - M_PI/2;
    const int disp_w = disp->getWidth();
    const int disp_h = disp->getHeight();
    //
    //
    // main loop
    while (robot->step(timeStep) != -1) {
        // get lidar data
        const float *arr = lidar->getLayerRangeImage(0); // values range from 0.0 to 1.0
        // display and calculate lidar data as a point cloud
        disp->setColor(0xFFFFFF);
        disp->fillRectangle(0,0,disp_w, disp_h);
        // vector to store lidar data calculated in the x,y plane
        std::vector<Point> radar_points; 
        float k = 0.4 * disp_w;// display scaling factor in pixels
        // iterate over lidar data
        for (int i = 0; i < size; i++) {
            float angle = lidar->getFov() * i / size; // make the angle go from 0 to fov
            float tempx = cos(angle + offset); // max value for each angle to show on radar_points
            float tempy = sin(angle + offset);
            Point data;
            if (arr[i] != maxRange) {
                data.x = arr[i] * tempx;
                data.y = arr[i] * tempy;
                radar_points.push_back(data);
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
        
        //std::cout << radar_points.size() << ", ";
        const unsigned int point_threshold = MIN_CLUSTER_SIZE; // number of points in a close area required to make a new point
        radar_points = findCircles(radar_points,CLUSTER_RADIUS,point_threshold);

      //  std::cout << radar_points.size() << "\n";
    
        // draw cones and lines between them 
        disp->setColor(0x0000FF);
        for (auto p: radar_points) {
            disp->drawOval(disp_w/2 + k*p.x, disp_h/2 + k*p.y,10,10);
          //  std::cout << p.x << ":" <<p.y <<"|";
        }
        //std::cout << "\n";
        // match cones together for radar movement
        // in: radar_points, out: pairs
        std::vector<PointPair> pairs;
        
        for (unsigned int i = 0; i < radar_points.size(); i++) {
            Point p1 = radar_points.at(i);
            for (unsigned int j = i + 1; j < radar_points.size(); j++) {
                Point p2 = radar_points.at(j);
                if ( dist2(p1,p2) < CONE_TO_CONE_RADIUS*CONE_TO_CONE_RADIUS) {
                    // line from p1 to p2.
                    pairs.push_back(PointPair{p1,p2});
                    //disp->drawLine(disp_w/2 + k*p1.x, disp_h/2 + k*p1.y, disp_w/2 + k*p2.x, disp_h/2 + k*p2.y);
                }
            }
        }
        // takes sets of pairs that represent lines and computes the closest value(radially)
        float arr2[size];
        float minDist = maxRange;
        int minDir = 0;

        // iterate over lidar data range, and take the closest point, factoring in lines (represented by pairs of points)
        for (int i = 0; i < size; i++) {
            float theta = lidar->getFov() * i / size + offset;
            float minVal = maxRange;
            
            for (auto pair:pairs) {
                disp->setColor(0x0000FF);
                disp->drawLine(disp_w/2 + k*pair.p1.x, disp_h/2 + k*pair.p1.y, disp_w/2 + k*pair.p2.x, disp_h/2 + k*pair.p2.y);
                float angle1 = atan2(pair.p1.y,pair.p1.x) ;
                float angle2 = atan2(pair.p2.y,pair.p2.x) ;
                if (theta > fmin(angle1, angle2) && theta < fmax(angle1, angle2)) {
                    float m = (pair.p2.y-pair.p1.y) / (pair.p2.x-pair.p1.x);
                    float b = pair.p1.y-m*pair.p1.x;
                    float t = b/(tan(theta)-m);
                    float r = sqrt(t*t*(1+m*m)+2*m*t*b+b*b);
                    if (r < minVal) {
                        minVal = r;
                    }
                }
            }
            // create a cone in a direction and see if we can traverse that way

            if (minVal < minDist) {
                minDist = minVal;
                minDir = i;
            }
            float distFromCentre = 2.0*i/size-1;
                if (minVal == maxRange) {
                   arr2[i] = maxRange*0.5*(0.9+0.1*(1.0-fabsf(distFromCentre)));
                   
                } else {
                    arr2[i] = minVal*(0.5+0.5*(1.0-fabsf(distFromCentre)));
                }
                //arr2[i] = minVal;
            
            // if (minVal != maxRange) {
            //     float val = minVal* (0.9 + 0.1 * (1-fabsf(distFromCentre)));
            //     if (val>maxDist) {
            //         maxDist = val;
            //         maxDir = distFromCentre;
            //     }
            // }
             
            float tempx = cos(theta);
            float tempy = sin(theta);
            disp->setColor(0x000000);
            disp->drawOval(disp_w/2 + k*tempx*minVal, disp_h/2 + k*tempy*minVal,2,2);
        }
        int cone_length = 180; 
        int cone_dir = size/2;
        float cone_max = 0;
       // std::cout << arr2[40] << "\n";
        for (int i = 0; i < size - cone_length; i++) {
        float cone_range = 2*maxRange;
        
            for (int j = i; j < cone_length + i; j++) {
                if (arr2[j] < cone_range) {
                    cone_range = arr2[j];
                }
            }
            if (cone_range > cone_max) {
                cone_max = cone_range;
                
                cone_dir = i + cone_length/2;
            }
            
        }

        auto dir_to_angle = [lidar,size,offset](int dir){return lidar->getFov() * dir / size + offset;};
        disp->drawLine(disp_w/2, disp_h/2, disp_w/2 + k*cone_max*cos(dir_to_angle(cone_dir)), disp_h/2 + k*cone_max*sin(dir_to_angle(cone_dir)));
        disp->drawLine(disp_w/2, disp_h/2, disp_w/2 + k*cone_max*cos(dir_to_angle(cone_dir - cone_length/2)), disp_h/2 + k*cone_max*sin(dir_to_angle(cone_dir - cone_length/2)));
        disp->drawLine(disp_w/2, disp_h/2, disp_w/2 + k*cone_max*cos(dir_to_angle(cone_dir + cone_length/2)), disp_h/2 + k*cone_max*sin(dir_to_angle(cone_dir + cone_length/2)));
        float dif = 2.0*cone_dir/size -1; // negative if 0
        
        // float left_avg = 0;
        // float right_avg = 0;
        // for (int i = 0; i < size/2; i++) {
        //     left_avg += arr2[i];
        //     right_avg += arr2[i + size/2];
        // }
        // left_avg = 2 * left_avg / size; // sum / n
        // right_avg = 2 * right_avg / size;
        // float avg = (left_avg + right_avg)/2;
        // float avg_sensor_diff = (right_avg - left_avg)/avg;
        // float dif = 4*(minDir*2/size - 1) * (1-minDist); // negative if close object on left, 
        // float left_speed = BASE_SPEED + DIF_SPEED*(0.5*avg_sensor_diff - dif);
        // float right_speed = BASE_SPEED - DIF_SPEED*(0.5*avg_sensor_diff - dif);
        const float gain = 0.7;
        float left_speed = BASE_SPEED + DIF_SPEED* (dif) * gain;
        float right_speed = BASE_SPEED - DIF_SPEED* (dif) * gain;
        
        lMotor->setVelocity(left_speed);
        rMotor->setVelocity(right_speed);
        stepcount++;
  };


  delete robot;
  return 0;
}




std::vector<Point> findCircles(std::vector<Point> radar_points, const float r, unsigned int point_threshold) {
        std::vector<Point> newradar_points;
        
        // start at the first point
        // iterate through all the following points, until a very distant point is found
        // once this point is found, calculate the average of the region before it. then move on
        for (unsigned int i = 0; i < radar_points.size(); i++) {
            Point p1 = radar_points.at(i);
            // ave = p1;
            std::vector<Point> region_points;
            region_points.push_back(p1);
            Point prev = p1;
            for (unsigned int j = i + 1; j < radar_points.size(); j++) {
                Point p2 = radar_points.at(j);
                if (abs(p2.x-prev.x) > r || abs(p2.y-prev.y) > r|| j == radar_points.size()-1) {
                    // calc average of points
                    if (region_points.size() >= point_threshold) {
                        Point c1 = region_points.at(0);
                        Point c2 = region_points.at(region_points.size() / 2);
                        Point c3 = region_points.at(region_points.size()-1);
                        Point c12 {(c2.x+c1.x)/2,(c2.y+c1.y)/2};
                        Point c23 {(c2.x+c3.x)/2,(c2.y+c3.y)/2};
                        float m12 = -(c1.x-c2.x)/(c1.y-c2.y);
                        float m23 = -(c2.x-c3.x)/(c2.y-c3.y);
                        float b12 = -m12*c12.x + c12.y;
                        float b23 = -m23*c23.x + c23.y;
                        float x = (b23-b12)/(m12-m23);
                        float y = m12*x + b12;
                        if (y < -CONE_RADIUS) {
                            newradar_points.push_back(Point{x,y});
                        }
                    }
                    i = j-1;
                    break;
                } else {
                    prev = p2;
                    region_points.push_back(p2);
                }
            }
        }
        return newradar_points;
}