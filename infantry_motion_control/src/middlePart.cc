#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <cmath>

#define YAW_OFFSET
// #define DEBUG


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

    tf::StampedTransform transform;
    listener.lookupTransform("/base_link", "/GimbalMiddlePart_1", ros::Time(0), transform);
    

    // 中层与底盘的TF变化，数值正负是以底盘为基准，中层旋转的角度。即yawAngleDiff>0时中层相较底盘逆时针旋转。
    double rollAngleDiff, pitchAngleDiff, yawAngleDiff;
    tf::Matrix3x3(transform.getRotation()).getRPY(rollAngleDiff, pitchAngleDiff, yawAngleDiff);


    // cmd_vel与中层朝向的夹角，数值正负是以中层为基准，速度相对的角度。即middleVelDiff>0时速度相较中层逆时针旋转。
    double middleVelDiff = atan2(cmdVel.linear.y, cmdVel.linear.x);

    double cmdVelModulus = sqrt(pow(cmdVel.linear.x, 2) + pow(cmdVel.linear.y, 2));


    geometry_msgs::Twist chassisCmdVel;

#ifdef YAW_OFFSET

    chassisCmdVel.linear.x = cmdVelModulus * sin((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.y = -cmdVelModulus * cos((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.z = 0;

    chassisCmdVel.angular.x = 0;
    chassisCmdVel.angular.y = 0;
    
#else

    // cmd_vel与底盘的总角度差为底盘到中层的角度变换+中层到cmd_vel的角度变换
    chassisCmdVel.linear.x = cmdVelModulus * cos((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.y = cmdVelModulus * sin((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.z = 0;

    chassisCmdVel.angular.x = 0;
    chassisCmdVel.angular.y = 0;

#endif

    chassisCmdVel.angular.z = cmdVel.angular.z;
    ROS_INFO("yawAngleDiff and middleVelDiff is %lf %lf", yawAngleDiff, middleVelDiff);


#ifdef DEBUG
    chassisCmdVel.angular.x *= 30;
    chassisCmdVel.angular.y *= 30;
    chassisCmdVel.angular.z *= 3;
#endif

    // ROS_INFO("ChassisCmdVel is %lf %lf %lf", 
    //          chassisCmdVel.linear.x, chassisCmdVel.linear.y, chassisCmdVel.angular.z);

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

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 1, z: 0}, angular: {x: 0, y: 0, z: 0}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 100}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'


rostopic pub -r 10 /chassis_cmd_vel geometry_msgs/Twist '{linear: {x: 1, y: 1, z: 0}, angular: {x: 0, y: 0, z: -20}}'

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 0.7854"

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 5.4978"

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 0"
*/