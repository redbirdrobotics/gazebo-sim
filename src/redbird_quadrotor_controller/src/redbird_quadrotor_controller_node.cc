#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

static const double setpoint_publishing_rate = 20.;

void await_fcu_connection() {
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "redbird_quadrotor_controller_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, state_cb);
  ros::Publisher position_publisher = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");

  ros::Rate rate(setpoint_publishing_rate);

  await_fcu_connection();

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;
  
  // send a few setpoints before starting.
  for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offboard_set_mode{};
  offboard_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_command{};
  arm_command.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (ros::ok()) {
    auto five_seconds_elapsed = (ros::Time::now() - last_request > ros::Duration(5.));
    if (!five_seconds_elapsed)
      continue;
    
    last_request = ros::Time::now();
    
    if (current_state.mode != "OFFBOARD") {
      if (set_mode_client.call(offboard_set_mode)
        && offboard_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
    } else {
      if (!current_state.armed) {
        if (arming_client.call(arm_command)
          && arm_command.response.success) {
          ROS_INFO("Vehicle armed")
        }
      }
    }

    position_publisher.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }
}