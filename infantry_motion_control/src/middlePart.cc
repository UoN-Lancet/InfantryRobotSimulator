#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <cmath>


class SwerveControl{

public:

  double x, y;

  std_msgs::Float64 angle, velocity;

};


class SubPuber
{
private:
  ros::NodeHandle n;

  ros::Subscriber sub;

  ros::Publisher pub;

  tf::TransformListener listener;

public:

  SubPuber(){
    //Topic you want to subscribe
    sub = n.subscribe("/cmd_vel", 1, &SubPuber::callback, this);

    //Topic you want to publish
    pub = n.advertise<geometry_msgs::Twist>("/chassis_cmd_vel", 1);

  }

  void callback(const geometry_msgs::Twist &cmdVel)
  {
    // ROS_INFO("cmdvel x y z is %lf %lf %lf", cmdVel.linear.x, cmdVel.linear.y, cmdVel.linear.z);

    tf::StampedTransform transform;
    listener.lookupTransform("/GimbalMiddlePart_1", "/base_link", ros::Time(0), transform);
    
    double rollAngleDiff, pitchAngleDiff, yawAngleDiff;
    tf::Matrix3x3(transform.getRotation()).getRPY(rollAngleDiff, pitchAngleDiff, yawAngleDiff);

    double middleAxisDiff = atan2(cmdVel.linear.y, cmdVel.linear.x);

    double cmdVelModulus = sqrt(pow(cmdVel.linear.x, 2) + pow(cmdVel.linear.y, 2));
    ROS_INFO("yaw angle difference is %lf %lf", yawAngleDiff, cmdVelModulus);


    geometry_msgs::Twist chassisCmdVel;
    chassisCmdVel.linear.x = cmdVelModulus * cos(-(yawAngleDiff + middleAxisDiff));
    chassisCmdVel.linear.y = cmdVelModulus * sin(-(yawAngleDiff + middleAxisDiff));
    chassisCmdVel.linear.z = 0;

    chassisCmdVel.angular.x = 0;
    chassisCmdVel.angular.y = 0;
    chassisCmdVel.angular.z = cmdVel.angular.z;


    ROS_INFO("ChassisCmdVel is %lf %lf %lf", 
             chassisCmdVel.linear.x, chassisCmdVel.linear.y, chassisCmdVel.angular.z);

    pub.publish(chassisCmdVel);
  }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "MiddlePart");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPuber middlePart;

  ros::spin();

  return 0;
}

/*
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 100}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 10, y: 0, z: 0}, angular: {x: 0, y: 0, z: 10}}'


rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 0.7854"

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 0"
*/