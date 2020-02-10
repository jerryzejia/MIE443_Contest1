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
double pi = 3.1416;
//Define Maximum Speed
double LINEAR_MAX = 0.25;
double ANGULAR_MAX = pi / 6;
// Define Boolean for bumper
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
//Define Laser Ranges
double laserRange = 11;
double laserRangeLeft = 11, laserRangeRight = 11;
int nLasers = 0, desiredNLasers = 0, desiredAngle = 15;
int rightIndex = 0, leftIndex = 0;

//Rotation function - with given degree and direction.
//When the degree input is 360, the robot will rotate until all the three laser ranges are greater than 0.5
void rotate(float degree, char direction)
{
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
    double angular = ANGULAR_MAX;
    double linear = 0.0;
    //Define the rotation direction according to the input
    if (direction == 'r')
    {
        angular = angular * -1;
    }
    //Define 360 degree rotation
    if (degree == 360)
    {
        while (laserRange < 0.7 || laserRangeLeft < 0.5 || laserRangeRight < 0.5)
        {
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            ros::spinOnce();
        }
    }
    else //Rotate to certain degree
    {
        while (abs(yaw - yaw_now) < rad)
        {
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            ros::spinOnce();
        }
    }
}
//Correction function - the robot will rotate 360 degrees first. Then it will rotate to the direction that has the most space.
//The robot should choose direction on its left or right side prior to the front side. Also, the robot will not turn back.
void correction()
{
    //Define the number of increments in the 360-degree rotation
    int directions = 360 / desiredAngle;
    //Define variables to store maximum value
    int maxIndexForward = 0;
    int maxIndexSide = 0;
    double maxReadingForward = 0;
    double maxReadingSide = 0;
    int maxIndex = 0;

    //Get the max index
    for (int i = 0; i < directions; i++)
    {
        //Update status
        ros::spinOnce();
        //Edit the size of deadzone
        //Size of forward zone
        if (i < directions / 8 || i > directions * 7 / 8)
        {
            if (laserRange > maxReadingForward)
            {
                maxReadingForward = laserRange;
                maxIndexForward = i;
                ROS_INFO("max_index_forward: %f", maxReadingForward);
                ROS_INFO("max_index_forward_index: %d", i);

            }
        }
            //Size of the left/right zone
        else if ((i >= directions / 8 && i <= directions * 3 / 8) || (i >= directions * 5 / 8 && i <= directions * 7 / 8))
        {
            if (laserRange > maxReadingSide)
            {
                maxReadingSide = laserRange;
                maxIndexSide = i;
                ROS_INFO("maxReadingSide: %f", maxReadingSide);
                ROS_INFO("max_index_side: %d", i);

            }
        }
        rotate(360 / directions, 'r');
    }

    //Determine if it can go left/right. If not, go to the direction in forward zone
    if (maxReadingSide > 1.0)
    {
        maxIndex = maxIndexSide;
    }
    else
    {
        maxIndex = maxIndexForward;
    }

    //Determine the direction and rotate
    if (maxIndex < directions / 2)
    {
        rotate(360 * maxIndex / directions, 'r');
    }
    else
    {
        rotate(360 * (directions - maxIndex) / directions, 'l');
    }
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
    //nLasers is 639 increments
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    float minLaserDist = std::numeric_limits<float>::infinity();
    //Print laser scan info
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers,desiredNLasers);
    //desiredNLasers is offset from center in both directions for laserRange

    //Define maximum laser range
    laserRange = 11;
    //Determine if the desired angle is lower than the laser angle
    if (desiredAngle * pi / 180 < msg->angle_max && -desiredAngle * pi / 180 > msg->angle_min)
    {
        //Update front range
        for (int i = nLasers / 2 - desiredNLasers; i < nLasers / 2 + desiredNLasers; i++)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[i]);
        }
        //Update left/right range by getting the minimum reading in the corresponding zone
        laserRangeLeft = 11;
        laserRangeRight = 11;
        for (int i = 0; i < desiredNLasers; i++)
        {
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
    else
    {
        for (int i = 0; i < nLasers; i++)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[i]);
        }
    }
    laserRange = minLaserDist;
    //Clear Out of Range readings
    if (laserRange == 11)
        laserRange = 0;
    if (laserRangeLeft == 11)
        laserRangeLeft = 0;
    if (laserRangeRight == 11)
        laserRangeRight = 0;
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

int main(int argc, char **argv)
{
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

    int time_last = 0;
    //Initial Mode
    //Distance Counter Setup
    correction();
    xLastSpin = posX;
    yLastSpin = posY;

    while (ros::ok() && secondsElapsed <= 480)
    {
        //Mode switch - 120-240s mode 1, else mode 2
        //Mode 1 - goes straight, stop when front range is too low.
        //Mode 2 - corrects the distance when it is going straight. Run correction function after it has passed a certain distance
        mode = ((secondsElapsed > 0 && secondsElapsed < 60) || (secondsElapsed > 120 && secondsElapsed < 180) ||
                (secondsElapsed > 420 && secondsElapsed < 480)) ? 1 : 2;

        if ((distFromLastSpin() > 2.5 && mode == 2) || (distFromLastSpin() > 7 && mode == 1)){
            xLastSpin = posX;
            yLastSpin = posY;
            correction();
        }

        //Correction counter

        //Print Robot Info
        //ROS_INFO("Position:(%f,%f) Orientation: %f degrees. Range: %f,", posX, posY, yaw * 180 / pi, laserRange);
        //ROS_INFO("Range:%f", laserRange);
        //ROS_INFO("LeftIndex:%f,RightIndex: %f",leftIndex, rightIndex);

        ros::spinOnce();

        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................

        if (isBumperPressed())
        {
            lastX = posX;
            lastY = posY;
            //Moving Back
            linear = -0.2;
            angular = 0;
            while (distFromLastLocation() < 0.15)
            {
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ros::spinOnce();
            }
            linear = 0;
            if (bumperRight)
                rotate(20, 'l');
            else if (bumperLeft)
                rotate(20, 'r');
            //Moving Forward
            linear = 0.2;
            lastX = posX;
            lastY = posY;
            while (distFromLastLocation() < 0.15)
            {
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ros::spinOnce();
            }
            linear = 0;
            //going back to the initial direction
            if (bumperRight)
                rotate(20, 'r');
            else if (bumperLeft)
                rotate(20, 'l');
        }

            //Free Space movement
            //Difference between mode 1 and 2
        else if (!isBumperPressed() && laserRange > 0.7){
            if (mode == 1) //Define mode 1
            {
                //Define mode 1 speed
                angular = 0.0;
                linear = LINEAR_MAX;
                //Correction when it is too close to walls
                angular = (laserRangeLeft < 0.5) ? -ANGULAR_MAX * (0.7 - laserRangeLeft) : ANGULAR_MAX * (0.7 - laserRangeRight);
            }
            else if (mode == 2){
                //Define linear speed
                angular = 0.0;
                linear = LINEAR_MAX;
                //Overwrite the angular speed when the distances from two sides are different more than 0.2
                if (laserRangeLeft - laserRangeRight > 0.2)
                    angular = (laserRangeLeft - laserRangeRight) / laserRangeLeft * (0.5 * ANGULAR_MAX);
                else if (laserRangeRight - laserRangeLeft > 0.2)
                    angular = (laserRangeRight - laserRangeLeft) / laserRangeRight * (-0.5 * ANGULAR_MAX);

                //Determines where distance values on both sides are located, and if a large discrepancy, turns
                if (leftIndexOffset + 100 < rightIndexOffset){
                    ROS_INFO("changing offset to left");
                    angular = ANGULAR_MAX;
                }
                else if (rightIndexOffset + 100 < leftIndexOffset){
                    ROS_INFO("changing offset to right");
                    angular = -ANGULAR_MAX;
                }
                //Overwrite the angular speed when it is too close to wall
                if (laserRangeLeft < 0.5)
                    angular = -ANGULAR_MAX;
                else if (laserRangeRight < 0.5)
                    angular = ANGULAR_MAX;
            }
        }
            //When the front sensor reading is too low

        else if (!isBumperPressed() && laserRange < 0.5)
        {
            ROS_INFO("stuck in else if statement");
            //Determine which side has more space
            if (laserRangeRight > laserRangeLeft)
            {
                rotate(360, 'r');
            }
            else
            {
                rotate(360, 'l');
            }
        }

        else
        {
            ROS_INFO("stuck in else statement");
            //Tries to stabilize robot when close to hitting wall
            if (laserRangeLeft - laserRangeRight > 0.2)
                angular = (laserRangeLeft - laserRangeRight) / laserRangeLeft * (0.75 * ANGULAR_MAX);
            else if (laserRangeRight - laserRangeLeft > 0.2)
                angular = (laserRangeRight - laserRangeLeft) / laserRangeRight * (-0.75 * ANGULAR_MAX);
            else
                angular = 0.0;

            //More gradual deceleration, can be extended for more gradual deceleration, more predictable than just
            //decreasing by constant value until close to wall (previous method)
            if (0.6 < laserRange <= 0.65)
                linear = 0.65 * LINEAR_MAX;
            else if (0.55 < laserRange <= 0.6)
                linear = 0.5 * LINEAR_MAX;
            else if (0.5 <= laserRange <= 0.55)
                linear = 0.2 * LINEAR_MAX;
        }

        //write the defined speed to the robot
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        //The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    }

    return 0;
}
