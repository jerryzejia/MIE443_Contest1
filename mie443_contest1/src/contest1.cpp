#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>

using namespace std;

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

//Define Global Variables
double posX, posY, yaw, yaw_now;
double xLastSpin = 0, yLastSpin = 0;
double lastX, lastY;
//Define Maximum Speed
double LINEAR_MAX = 0.2;
double LINEAR_MAX_NEAR_OBSTACLE = 0.1;
double ANGULAR_MAX = pi / 6;
// Define Boolean for bumper
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
//Define Laser Ranges
double laserRange = 5, laserRangeLeft = 5, laserRangeRight = 5;
int nLasers = 0, desiredNLasers = 0, desiredAngle = 15;
int rightIndex = 0, leftIndex = 0;

void rotate(float degree, char direction)
{
    ROS_INFO("rotation starting");
    //Define rad
    double rad;
    rad = DEG2RAD(degree);
    //Update status
    ros::spinOnce();
    //Record the status
    yaw_now = yaw;
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    //Define the magnitude of linear and angular speed here
    double angular = (direction == 'l') ? ANGULAR_MAX : - ANGULAR_MAX;
    double linear = 0.0;
    //Define the rotation direction according to the input
    //Define 360 degree rotation
    while (abs(yaw - yaw_now) < rad){
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        ros::spinOnce();
    }

}

void rotateToFreedom(char direction){
    ros::spinOnce();
    //Record the status
    yaw_now = yaw;
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    //Define the magnitude of linear and angular speed here
    double angular = (direction == 'l') ? ANGULAR_MAX : - ANGULAR_MAX;
    double linear = 0.0;
    while (laserRange < 0.7){
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        ros::spinOnce();
    }
}

void findFurthestDirection(){
    int directions = 360 / desiredAngle;
    int directionThreshold = directions / 3
    int maxIndexForward = 0, maxIndexSide = 0, maxIndex = 0;
    double maxReadingForward = 0, maxReadingSide = 0;
    for (int i = 0; i < directions; i++)
    {
        ros::spinOnce();
        if (i < directions / directionThreshold && i > directions * (directionThreshold - 1) / directionThreshold){
            if (laserRange > maxReadingForward){
                maxReadingForward = laserRange;
                maxIndexForward = i;
                ROS_INFO("max_index_forward: %f", maxReadingForward);
                ROS_INFO("max_index_forward_index: %d", i);
            }
        }
        else if ((i >= directions / directionThreshold && i <= directions * (directionThreshold - 5) / directionThreshold)
        || (i >= directions * (directionThreshold - 5) / directionThreshold && i <= directions * (directionThreshold -1) / directionThreshold)){
            if (laserRange > maxReadingSide){
                maxReadingSide = laserRange;
                maxIndexSide = i;
                ROS_INFO("maxReadingSide: %f", maxReadingSide);
                ROS_INFO("max_index_side: %d", i);
            }
        }
        rotate(360 / directions, 'r');
    }
    maxIndex = (maxReadingSide > 1.0) ? maxIndexSide : maxIndexForward;
    double rotationRatio = (maxIndex < directions / 2) ? 360 * maxIndex / directions : 360 * (directions - maxIndex) / directions;
    char rotationDirection = (maxIndex < directions / 2) ? 'r' : 'l';
    rotate(rotationRatio, rotationDirection);
}

//Bumper callback function
void bumperCallback(const kobuki_msgs::BumperEvent msg)
{
    if (msg.bumper == 0)
        bumperLeft = !bumperLeft;
    else if (msg.bumper == 1)
        bumperCenter = !bumperCenter;
    else if (msg.bumper == 2)
        bumperRight = !bumperRight;
}

//Laser call back function
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    float minLaserDist = std::numeric_limits<float>::infinity();
    //Determine if the desired angle is lower than the laser angle
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min)
    {
        //Update front range
        for (int i = nLasers / 2 - desiredNLasers; i < nLasers / 2 + desiredNLasers; i++){
            minLaserDist = std::min(minLaserDist, msg->ranges[i]);
        }
        //Update left/right range by getting the minimum reading in the corresponding zone
        for (int i = 0; i < desiredNLasers; i++){
            if (laserRangeRight > msg->ranges[i]) {
                laserRangeRight = msg->ranges[i];
                rightIndex = i;
            }
            if (laserRangeLeft > msg->ranges[nLasers - 1 - i]) {
                laserRangeLeft = msg->ranges[nLasers - 1 - i];
                leftIndex = nLasers - 1 - i;
            }
        }
    }
    else{
        for (int i = 0; i < nLasers; i++){
            minLaserDist = std::min(minLaserDist, msg->ranges[i]);
            laserRangeRight = 0;
            laserRangeLeft = 0;
        }
    }
    laserRange = minLaserDist;
}

//odometry call back function
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //Read Info from odom
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position:(%f,%f) Orientation: %f Rad or %f degrees.",posX,posY,yaw,yaw*180/pi);
}

bool isBumperPressed()
{
    return bumperRight || bumperCenter || bumperLeft;
}

double distFromLastSpin(){
    std::sqrt(std::pow(posX - xLastSpin, 2) + std::pow(posY - yLastSpin, 2));
}

double distFromLastLocation(){
    std::sqrt(std::pow(posX - lastX, 2) + std::pow(posY - lastY, 2));
}

int main(int argc, char **argv){
    //Set Initial Mode
    int mode = 1;

    //Offset Calculation
    int leftIndexOffset = leftIndex - ((nLasers - 1) / 2);
    int rightIndexOffset = ((nLasers - 1) / 2) - rightIndex;

    //ROS setup
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    teleController eStop;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    //Define speed variables
    double angular, linear;
    geometry_msgs::Twist vel;

    //Set timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now(); /* start timer */
    uint64_t secondsElapsed = 0;              // the timer just started, so we know it is less than 480, no need to check.

    findFurthestDirection();
    xLastSpin = posX;
    yLastSpin = posY;

    while (ros::ok() && secondsElapsed <= 480){

        mode = ((secondsElapsed > 20 && secondsElapsed < 60) ||(secondsElapsed > 120 && secondsElapsed < 180) || (secondsElapsed > 240 && secondsElapsed < 300) ||
                (secondsElapsed > 420 && secondsElapsed < 480)) ? 1 : 2;

        double threshold = rand() % 1 +  3;
        if ((distFromLastSpin() > threshold && mode == 2) || (distFromLastSpin() > (threshold + 1) * 2 && mode == 1)){
            xLastSpin = posX;
            yLastSpin = posY;
            findFurthestDirection();
        }
        //Print Robot Info
        ROS_INFO("Mode: %d", mode);

        ros::spinOnce();

        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................

        if (isBumperPressed() || isinf(laserRange)){
            uint64_t secondsElapsedLast = secondsElapsed;
            ROS_INFO("bumping");
            lastX = posX;
            lastY = posY;
            linear = -LINEAR_MAX_NEAR_OBSTACLE;
            angular = 0;
            while (distFromLastLocation() < 0.1 && (secondsElapsed - secondsElapsedLast) < 8)
            {
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ros::spinOnce();
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            }
            linear = 0;
            ROS_INFO("calling rotation");

            if (bumperRight){
                rotate(25, 'l');
            }
            else if (bumperLeft){
                rotate(25, 'r');
            }
            else{
                if(laserRangeLeft > laserRangeRight){
                    rotate(90, 'l');
                } else{
                    rotate(90, 'r');
                }
            }

            linear = LINEAR_MAX_NEAR_OBSTACLE;
            lastX = posX;
            lastY = posY;
            secondsElapsedLast = secondsElapsed;
            while (distFromLastLocation() < 0.15 && (secondsElapsed - secondsElapsedLast) < 8){
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ros::spinOnce();
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            }
            linear = 0;
            if (bumperRight)
                rotate(20, 'r');
            else if (bumperLeft)
                rotate(20, 'l');
        }

        else if (!isBumperPressed() && laserRange > 0.7){
            ROS_INFO("Free Space, %f", laserRange);
            if (mode == 1){
                linear = LINEAR_MAX;
                angular = (laserRangeLeft < 0.3) ? -ANGULAR_MAX  : 0 ;
                angular = (laserRangeRight < 0.3) ? ANGULAR_MAX  : 0 ;

            }
            else if (mode == 2){
                ROS_INFO("Mode 2");
                angular = 0.0;
                linear = LINEAR_MAX;
                if (laserRangeLeft - laserRangeRight > 0.2)
                    angular = (laserRangeLeft - laserRangeRight) / laserRangeLeft * (0.75 * ANGULAR_MAX);
                else if (laserRangeRight - laserRangeLeft > 0.2)
                    angular = (laserRangeRight - laserRangeLeft) / laserRangeRight * (-0.75 * ANGULAR_MAX);
                if (laserRangeLeft < 0.5)
                    angular = -ANGULAR_MAX;
                else if (laserRangeRight < 0.5)
                    angular = ANGULAR_MAX;
            }
        }
        else if (!isBumperPressed() && laserRange < 0.5){
            linear = LINEAR_MAX_NEAR_OBSTACLE;
            ROS_INFO("stuck in else if statement");
            char dir = (laserRangeRight > laserRangeLeft) ? 'r' : 'l';
            rotateToFreedom(dir);
        }

        else
        {
            ROS_INFO("stuck in else statement");
            angular = 0;
            if (laserRangeLeft - laserRangeRight > 0.2){
                ROS_INFO("moving to the left %f", angular);
                angular = (laserRangeLeft - laserRangeRight) / laserRangeLeft * (0.75 * ANGULAR_MAX);
            }
            else if (laserRangeRight - laserRangeLeft > 0.2){
                ROS_INFO("moving to the right %f", angular);
                angular = (laserRangeRight - laserRangeLeft) / laserRangeRight * (-0.75 * ANGULAR_MAX);
            }
            linear = (0.5 <= laserRange <= 0.55) ? LINEAR_MAX_NEAR_OBSTACLE : laserRange * LINEAR_MAX;
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    }

    return 0;
}