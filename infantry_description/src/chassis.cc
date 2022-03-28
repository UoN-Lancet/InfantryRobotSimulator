#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <cmath>


static const float chassisRadius = 0.22627417; // Radius = 0.16 * sqrt(2)  meters
static const float rtChassisRadius = 0.16; // Radius = Rdius / sqrt(2)  meters


class SwerveControl{

public:

  double x, y;

  std_msgs::Float64 angle, velocity;

};


class SubPuber
{
private:
  ros::NodeHandle n_;

  ros::Subscriber sub_;

  ros::Publisher pubAngle[5];
  ros::Publisher pubVelocity[5];

public:

  SubPuber(){
    //Topic you want to subscribe
    sub_ = n_.subscribe("/chassis_cmd_vel", 1, &SubPuber::callback, this);

    //Topic you want to publish
    // pub_ = n_.advertise<std_msgs::Float64>("/infantry/Rev35_position_controller/command", 1);

    pubAngle[1] = n_.advertise<std_msgs::Float64>("/infantry/Rev31_position_controller/command", 1);
    pubAngle[2] = n_.advertise<std_msgs::Float64>("/infantry/Rev33_position_controller/command", 1);
    pubAngle[3] = n_.advertise<std_msgs::Float64>("/infantry/Rev29_position_controller/command", 1);
    pubAngle[4] = n_.advertise<std_msgs::Float64>("/infantry/Rev27_position_controller/command", 1);

    pubVelocity[1] = n_.advertise<std_msgs::Float64>("/infantry/Rev32_position_controller/command", 1);
    pubVelocity[2] = n_.advertise<std_msgs::Float64>("/infantry/Rev34_position_controller/command", 1);
    pubVelocity[3] = n_.advertise<std_msgs::Float64>("/infantry/Rev30_position_controller/command", 1);
    pubVelocity[4] = n_.advertise<std_msgs::Float64>("/infantry/Rev28_position_controller/command", 1);
  }

  void callback(const geometry_msgs::Twist &chassisCmdVel)
  {


    ROS_INFO("ChassisCmdVel is %lf %lf %lf", 
             chassisCmdVel.linear.x, chassisCmdVel.linear.y, chassisCmdVel.angular.z);

    SwerveControl swerve[5];

    for (int i = 1; i <= 4; i++) {
      swerve[i].x = chassisCmdVel.linear.x;
      swerve[i].y = chassisCmdVel.linear.y;
    }

    double angularSpeedEffect = chassisCmdVel.angular.z * rtChassisRadius;

    swerve[1].x -= angularSpeedEffect;
    swerve[1].y += angularSpeedEffect;

    swerve[2].x += angularSpeedEffect;
    swerve[2].y += angularSpeedEffect;
    
    swerve[3].x -= angularSpeedEffect;
    swerve[3].y -= angularSpeedEffect;
    
    swerve[4].x += angularSpeedEffect;
    swerve[4].y -= angularSpeedEffect;


    for (int i = 1; i <= 4; i++) {
      float angle = atan2(swerve[i].y, swerve[i].x);
      swerve[i].angle.data = (angle >= 0) ? angle : (angle + 2 * M_PI);
      swerve[i].velocity.data = (float) sqrt(pow(swerve[i].x, 2) + pow(swerve[i].y, 2));
    }

    for (int i = 1; i <= 4; i++) {
      pubAngle[i].publish(swerve[i].angle);
      pubVelocity[i].publish(swerve[i].velocity);
    }
  }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Chassis");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPuber Chassis;

  ros::spin();

  return 0;
}

// rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'