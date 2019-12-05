//DRDO SASE's UAV Fleet Challenge

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

int update_uav1 = 0;
geometry_msgs::Twist setpoint_vel_uav1, zero_vel_uav1;

mavros_msgs::State current_state_uav1;
//Callback function to get current state
void state_cb_uav1(const mavros_msgs::State::ConstPtr& msg1){
    current_state_uav1 = *msg1;
}

geometry_msgs::TwistStamped current_vel_uav1;
//Callback function to store current velocity
void vel_cb_uav1(const geometry_msgs::TwistStamped::ConstPtr& msg2){
	current_vel_uav1 = *msg2;
}

geometry_msgs::Twist current_vel_control_uav1;
//Callback function to set UAV velocity
void vel_control_cb_uav1(const geometry_msgs::Twist::ConstPtr& msg3){
	current_vel_control_uav1 = *msg3;
	setpoint_vel_uav1.linear.x = ((current_vel_control_uav1.linear.x)/100.0)*1.0;
  setpoint_vel_uav1.linear.y = ((current_vel_control_uav1.linear.y)/100.0)*1.0;
  setpoint_vel_uav1.linear.z = ((current_vel_control_uav1.linear.z)/100.0)*1.0;
  setpoint_vel_uav1.angular.x = ((current_vel_control_uav1.angular.x)/100.0)*1.0;
	setpoint_vel_uav1.angular.y = ((current_vel_control_uav1.angular.y)/100.0)*1.0;
	setpoint_vel_uav1.angular.z = ((current_vel_control_uav1.angular.z)/100.0)*1.0;
	update_uav1 = 1;
}

int main(int argc, char **argv)
{
  //Set node name
  ros::init(argc, argv, "uav1_velocity_node");
  ros::NodeHandle nh;

  //Subscribe to current state
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("/uav1/mavros/state", 10, state_cb_uav1);
  //Subscribe to current velocity
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
  	    ("/uav1/mavros/local_position/velocity", 100, vel_cb_uav1);
  //Subscribe to input velocity control
  ros::Subscriber vel_control_sub = nh.subscribe<geometry_msgs::Twist>
  	    ("/uav1/vel_control_topic", 100, vel_control_cb_uav1);
  //Publish the update_uav1d velocity
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
          ("/uav1/mavros/setpoint_velocity/cmd_vel_unstamped", 100);
  //Set control mode
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("/uav1/mavros/set_mode");
  //Arming
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("/uav1/mavros/cmd/arming");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(40.0);

  // wait for FCU connection
  while(ros::ok() && !current_state_uav1.connected){
      ros::spinOnce();
      rate.sleep();
  }

  //Initialize as 0
  setpoint_vel_uav1.linear.x = 0;
  setpoint_vel_uav1.linear.y = 0;
  setpoint_vel_uav1.linear.z = 0;
  setpoint_vel_uav1.angular.x = 0;
  setpoint_vel_uav1.angular.y = 0;
  setpoint_vel_uav1.angular.z = 0;
  zero_vel_uav1 = setpoint_vel_uav1;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
      vel_pub.publish(setpoint_vel_uav1);
      ros::spinOnce();
      rate.sleep();
  }

  //Set to offboard mode
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  //Arm the vehicle
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  //Main control code
  while(ros::ok()){
    if( current_state_uav1.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("UAV1 offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else {
      if( !current_state_uav1.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
          ROS_INFO("UAV1 armed");
        }
        last_request = ros::Time::now();
      }
    }
    //Publish the current velocity
    if(update_uav1==1){
    	vel_pub.publish(setpoint_vel_uav1);
    	update_uav1 = 0;
    }
    //If not update_uav1d - publish zero
    else{
	    vel_pub.publish(zero_vel_uav1);
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
