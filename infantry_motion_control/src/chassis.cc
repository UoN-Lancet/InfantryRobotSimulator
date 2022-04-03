#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <cmath>


static const float chassisRadius = 0.22627417; // Radius = 0.16 * sqrt(2)  meters
static const float rtChassisRadius = 0.16; // Radius = Rdius / sqrt(2)  meters
static const float velocityOffset = 10;


class SwerveControl{

public:

  double x, y;

  std_msgs::Float64 angle, velocity;

};


class SubPuber
{
private:
  ros::NodeHandle nh;

  ros::Subscriber sub;

  ros::Publisher pubAngle[4];
  ros::Publisher pubVelocity[4];

public:

  SubPuber(){
    sub = nh.subscribe("/chassis_cmd_vel", 1, &SubPuber::callback, this);


    pubAngle[0] = nh.advertise<std_msgs::Float64>("/infantry/Rev31_position_controller/command", 1);
    pubAngle[1] = nh.advertise<std_msgs::Float64>("/infantry/Rev33_position_controller/command", 1);
    pubAngle[2] = nh.advertise<std_msgs::Float64>("/infantry/Rev29_position_controller/command", 1);
    pubAngle[3] = nh.advertise<std_msgs::Float64>("/infantry/Rev27_position_controller/command", 1);

    pubVelocity[0] = nh.advertise<std_msgs::Float64>("/infantry/Rev32_position_controller/command", 1);
    pubVelocity[1] = nh.advertise<std_msgs::Float64>("/infantry/Rev34_position_controller/command", 1);
    pubVelocity[2] = nh.advertise<std_msgs::Float64>("/infantry/Rev30_position_controller/command", 1);
    pubVelocity[3] = nh.advertise<std_msgs::Float64>("/infantry/Rev28_position_controller/command", 1);
  }

  void callback(const geometry_msgs::Twist &chassisCmdVel)
  {


    ROS_INFO("ChassisCmdVel is %lf %lf %lf", 
             chassisCmdVel.linear.x, chassisCmdVel.linear.y, chassisCmdVel.angular.z);

    SwerveControl swerve[4];

    for (int i = 0; i <= 3; i++) {
      swerve[i].x = chassisCmdVel.linear.x;
      swerve[i].y = chassisCmdVel.linear.y;
    }

    double angularSpeedEffect = chassisCmdVel.angular.z * rtChassisRadius;

    // 左前轮
    swerve[0].x -= angularSpeedEffect;
    swerve[0].y += angularSpeedEffect;

    // 右前轮
    swerve[1].x += angularSpeedEffect;
    swerve[1].y += angularSpeedEffect;
    
    // 左后轮
    swerve[2].x -= angularSpeedEffect;
    swerve[2].y -= angularSpeedEffect;

    // 右后轮
    swerve[3].x += angularSpeedEffect;
    swerve[3].y -= angularSpeedEffect;


    for (int i = 0; i <= 3; i++) {
      float angle = atan2(swerve[i].y, swerve[i].x);
      swerve[i].angle.data = (angle >= 0) ? angle : (angle + 2 * M_PI);
      swerve[i].velocity.data = velocityOffset * (float) sqrt(pow(swerve[i].x, 2) + pow(swerve[i].y, 2));
    }

    for (int i = 0; i <= 3; i++) {
      pubAngle[i].publish(swerve[i].angle);
      pubVelocity[i].publish(swerve[i].velocity);
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Chassis");


  SubPuber Chassis;

  ros::spin();

  return 0;
}
