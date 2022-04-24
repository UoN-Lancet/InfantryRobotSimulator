#include "SerialCommHandle.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define CMD_VEL 0x01 // command id

struct CmdVel
{
  float angularX, angularY, angularZ;
  float linearX, linearY, linearZ;
} 
__attribute__(( aligned(1) ));


class SubscriberProxy
{
private:

  ros::NodeHandle nh;
  ros::Subscriber sub;

  SerialCommHandle commHandle;
  Ref<SerialCommHandle::Publisher<CMD_VEL, CmdVel>> positionPublisher;

  SubscriberProxy() {
    positionPublisher = std::make_shared<SerialCommHandle::Publisher<CMD_VEL, CmdVel>>(commHandle.publisher<CMD_VEL, CmdVel>());
    sub = nh.subscribe("/cmd_vel", 1, &SubscriberProxy::callback,  this);
  }

public:

  static SubscriberProxy* getInstance() {
    static SubscriberProxy instance;
    return &instance;
  }

  void callback(const geometry_msgs::Twist & cmdVel)
  {
    CmdVel cmdVelPacked;
    cmdVelPacked.angularX = cmdVel.angular.x;
    cmdVelPacked.angularY = cmdVel.angular.y;
    cmdVelPacked.angularZ = cmdVel.angular.z;
    cmdVelPacked.linearX = cmdVel.linear.x;
    cmdVelPacked.linearY = cmdVel.linear.y;
    cmdVelPacked.linearZ = cmdVel.linear.z;
    positionPublisher->publish(cmdVelPacked);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SerialPortCommunication");

  SubscriberProxy* subscriberProxy = SubscriberProxy::getInstance();

  ros::spin();

  return 0;
}
