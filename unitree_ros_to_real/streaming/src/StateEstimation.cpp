#include "../include/StateEstimation.h"

StateEstimation::StateEstimation(std::string object_name)
{
  sub = n.subscribe("/vrpn_client_node/" + object_name + "/pose", 1, &StateEstimation::ObjectCallback, this);
  pub = n.advertise<geometry_msgs::TwistStamped>("/vrpn_client_node/" + object_name + "/twist", 1);

  _dataPrev.time = 0.0;
  for (int i = 0; i < 3; i++)
  {
    _dataPrev.position[i] = 0;
    _dataPrev.quaternion[i] = 0;
    _data.v_world[i] = 0;
  }
}

void StateEstimation::ObjectCallback(const geometry_msgs::PoseStamped &msg)
{
  ROS_INFO("I heard: x =%f, y=%f, z=%f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  _data.time = msg.header.stamp.toSec();
  // _data.time = ros::Time::now().toSec();
  twist_msg.header = msg.header;

  _data.position[0] = msg.pose.position.x;
  _data.position[1] = msg.pose.position.y;
  _data.position[2] = msg.pose.position.z;

  _data.quaternion[0] = msg.pose.orientation.w;
  _data.quaternion[1] = msg.pose.orientation.x;
  _data.quaternion[2] = msg.pose.orientation.y;
  _data.quaternion[3] = msg.pose.orientation.z;

  _data.rotmat = ori::quaternionToRotationMatrix(_data.quaternion);
  _data.rpy = ori::quatToRPY(_data.quaternion);

  LinearVelocityCalc();
  AngularVelocityCalc();
  _dataPrev.time = _data.time;
  pub.publish(twist_msg);
}

void StateEstimation::LinearVelocityCalc()
{
  double dt = _data.time - _dataPrev.time;

  for (int i = 0; i < 3; i++)
  {
    _data.v_world[i] = (_data.position[i] - _dataPrev.position[i]) / dt;
  }
  _data.v_body = _data.rotmat * _data.v_world;

  twist_msg.twist.linear.x = _data.v_body[0];
  twist_msg.twist.linear.y = _data.v_body[1];
  twist_msg.twist.linear.z = _data.v_body[2];

  for (int i = 0; i < 3; i++)
  {
    _dataPrev.position[i] = _data.position[i];
  }
}

void StateEstimation::AngularVelocityCalc()
{
  double dt = _data.time - _dataPrev.time;

  _data.omega_world[0] = (2 / dt) * (_dataPrev.quaternion[0] * _data.quaternion[1] - _dataPrev.quaternion[1] * _data.quaternion[0] - _dataPrev.quaternion[2] * _data.quaternion[3] + _dataPrev.quaternion[3] * _data.quaternion[2]);
  _data.omega_world[1] = (2 / dt) * (_dataPrev.quaternion[0] * _data.quaternion[2] + _dataPrev.quaternion[1] * _data.quaternion[3] - _dataPrev.quaternion[2] * _data.quaternion[0] - _dataPrev.quaternion[3] * _data.quaternion[1]);
  _data.omega_world[2] = (2 / dt) * (_dataPrev.quaternion[0] * _data.quaternion[3] - _dataPrev.quaternion[1] * _data.quaternion[2] + _dataPrev.quaternion[2] * _data.quaternion[1] - _dataPrev.quaternion[3] * _data.quaternion[0]);

  _data.omega_body = _data.rotmat * _data.omega_world;

  twist_msg.twist.angular.x = _data.omega_body[0];
  twist_msg.twist.angular.y = _data.omega_body[1];
  twist_msg.twist.angular.z = _data.omega_body[2];

  for (int i = 0; i < 4; i++)
  {
    _dataPrev.quaternion[i] = _data.quaternion[i];
  }
}