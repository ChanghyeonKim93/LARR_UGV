#include <iostream>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <time.h>
#include <string>
#include <sstream>

#include <mutex>
#include <thread>

// keyboard input tool
#include "keyinput.h"
#include "tcp_communicator.h"
#include "tools.h"

// ROS
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std;
void callbackDesiredAngularVelocities(const std_msgs::Float32MultiArray::ConstPtr& msg_recv, TCPCOMM& tcp_comm);

int main(int argc, char **argv) {
    ros::init(argc, argv, "tcp_server");
    ros::NodeHandle nh("~");

    int n_age = -1;
    
    ros::param::get("~n", n_age);
   
    // ROS publish  
    // run multi-threads
    std::mutex* mut  = new std::mutex();
    TCPCOMM tcp_comm = TCPCOMM(mut,nh);
    std::thread t1   = tcp_comm.runThreadROS();

    // ROS subscribers
    ros::Subscriber sub_wd = nh.subscribe<std_msgs::Float32MultiArray>("/larr_ugv/wheel_desired",1,boost::bind(callbackDesiredAngularVelocities,_1,boost::ref(tcp_comm)));
    
    // ROS spin
    if(ros::ok()) ros::spin();

    // delete
    t1.join();
    delete mut;

    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}

void callbackDesiredAngularVelocities(const std_msgs::Float32MultiArray::ConstPtr& msg_recv, TCPCOMM& tcp_comm){
    std::vector<float> data = msg_recv->data;
    printf("CALLBACK desired: %0.3f %0.3f\n", data[0],data[1]);
    tcp_comm.setDesiredAngularVelocityLeft(data[0]); 
    tcp_comm.setDesiredAngularVelocityRight(data[1]); 
};
