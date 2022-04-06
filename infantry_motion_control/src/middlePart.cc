#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <cmath>

#define YAW_OFFSET
// #define DEBUG

static const double tfOffset = 1;
static const double ratioOffset = 0.1;
static const double gyroSpinSpd = 5;
static const double gyroMovingOffset = 0.8;


class SwerveControl{

public:

  double x, y;

  std_msgs::Float64 angle, velocity;

};


class SubPuber
{
private:
  ros::NodeHandle n;

  // 以中层坐标系为基底的速度
  ros::Subscriber cmdVelSub;

  // 以中层为基底的小陀螺方向 1为逆时针 -1为顺时针 0代表不小陀螺
  ros::Subscriber gyroSpinSub;
  int gyroSpinDirection;

  // 中层朝向
  ros::Subscriber midTowardSub;

  // 以底盘座标系为基底的速度，发布后由chassis节点接收并处理
  ros::Publisher chassisPub;

  // 中层的旋转命令，用于使中层朝向目标方向
  ros::Publisher middleMotorPub;

  // 订阅中层和底层的TF关系，用于处理cmd_vel到chassis_cmd_vel的转换
  tf::TransformListener listener;



public:

  SubPuber(){
    cmdVelSub = n.subscribe("/cmd_vel", 1, &SubPuber::cmdVelCallback, this);
    gyroSpinSub = n.subscribe("/gyro_spin", 1, &SubPuber::gyroSpinCallback, this);
    midTowardSub = n.subscribe("/mid_toward", 1, &SubPuber::midTowardCallback, this);
    chassisPub = n.advertise<geometry_msgs::Twist>("/chassis_cmd_vel", 1);
    middleMotorPub = n.advertise<std_msgs::Float64>("/infantry/Rev35_position_controller/command", 1);

    gyroSpinDirection = 0;
  }

  void cmdVelCallback(const geometry_msgs::Twist &cmdVel)
  {

    tf::StampedTransform transform;
    
    
    try{
      listener.lookupTransform("/base_link", "/GimbalMiddlePart_1", ros::Time(0), transform);
    } catch(...) { 
      // ignored
    }

    // 中层与底盘的TF变化，数值正负是以底盘坐标为基底，中层旋转的角度。即yawAngleDiff>0时中层相较底盘逆时针旋转。
    double rollAngleDiff, pitchAngleDiff, yawAngleDiff;
    tf::Matrix3x3(transform.getRotation()).getRPY(rollAngleDiff, pitchAngleDiff, yawAngleDiff);


    // cmd_vel与中层朝向的夹角，数值正负是以中层坐标为基底，速度相对的角度。即middleVelDiff>0时速度相较中层逆时针旋转。
    double middleVelDiff = atan2(cmdVel.linear.y, cmdVel.linear.x);

    double cmdVelModulus = sqrt(pow(cmdVel.linear.x, 2) + pow(cmdVel.linear.y, 2));


    geometry_msgs::Twist chassisCmdVel;

#ifdef YAW_OFFSET

    chassisCmdVel.linear.x = cmdVelModulus * sin((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.y = -cmdVelModulus * cos((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.z = 0;

    if (gyroSpinDirection == 1) {
      chassisCmdVel.linear.x += chassisCmdVel.linear.y * gyroMovingOffset;
    }
    else if (gyroSpinDirection == -1) {
      chassisCmdVel.linear.x -= chassisCmdVel.linear.y * gyroMovingOffset;
    }

    chassisCmdVel.angular.x = 0;
    chassisCmdVel.angular.y = 0;
    
#else

    // cmd_vel与底盘的总角度差为底盘到中层的角度变换+中层到cmd_vel的角度变换
    chassisCmdVel.linear.x = cmdVelModulus * cos((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.y = cmdVelModulus * sin((yawAngleDiff + middleVelDiff));
    chassisCmdVel.linear.z = 0;

    if (gyroSpinDirection == 1) {
      chassisCmdVel.linear.y -= chassisCmdVel.linear.x * gyroMovingOffset;
    }
    else if (gyroSpinDirection == -1) {
      chassisCmdVel.linear.y += chassisCmdVel.linear.x * gyroMovingOffset;
    }

    chassisCmdVel.angular.x = 0;
    chassisCmdVel.angular.y = 0;

#endif

    // chassisCmdVel.angular.z = cmdVel.angular.z + (gyroSpinDirection * rand() % 10) * gyroSpinSpd;
    chassisCmdVel.angular.z = cmdVel.angular.z + gyroSpinDirection * gyroSpinSpd;

    // ROS_INFO("yawAngleDiff and middleVelDiff is %lf %lf", yawAngleDiff, middleVelDiff);

    chassisPub.publish(chassisCmdVel);
  }


  void gyroSpinCallback(const std_msgs::Int32 &gyroSpin)
  {
    gyroSpinDirection = gyroSpin.data;
  }

  void midTowardCallback(const std_msgs::Float64 &midToward)
  {
    tf::StampedTransform mapChassisTF;

    try{
      listener.lookupTransform("/map", "/base_link", ros::Time(0), mapChassisTF);
    } catch(...) {
      // ignored
    }

    double rollAngleDiff, pitchAngleDiff, mapChassisAngleDiff, chassisMiddleAngleDiff;
    tf::Matrix3x3(mapChassisTF.getRotation()).getRPY(rollAngleDiff, pitchAngleDiff, mapChassisAngleDiff);
    std_msgs::Float64 middlePartDirection;


#ifdef YAW_OFFSET

    // middlePartDirection.data = (yawAngleDiff - midToward.data < 0 ? -1 : 1) *
    //                             log(abs(yawAngleDiff - midToward.data)) * ratioOffset * tfOffset;

    middlePartDirection.data = (mapChassisAngleDiff - midToward.data) * tfOffset;

#else

    middlePartDirection.data = log(yawAngleDiff - midToward.data);

#endif

    ROS_INFO("mapChassisAngleDiff: %lf chassisMiddleAngleDiff: %lf", mapChassisAngleDiff, chassisMiddleAngleDiff);
    middleMotorPub.publish(middlePartDirection);
  }
};



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

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 4.7124"

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 0"


rostopic pub -r 100 /mid_toward std_msgs/Float64 "data: 0"
rostopic pub -r 100 /mid_toward std_msgs/Float64 "data: 3.14"
rostopic pub -r 100 /mid_toward std_msgs/Float64 "data: 0.78"

rostopic pub -r 10 /gyro_spin std_msgs/Int32 "data: 1"

rostopic pub -r 1 /infantry/Rev35_position_controller/command std_msgs/Float64 "data: 0"
*/