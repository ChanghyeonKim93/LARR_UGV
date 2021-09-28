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

void callbackDesiredAngularVelocities(const std_msgs::Float32MultiArray::ConstPtr& msg_recv, TCPCOMM& tcp_comm){
    std::vector<float> data = msg_recv->data;
    printf("CALLBACK desired: %0.3f %0.3f\n", data[0],data[1]);
    float wl_desired = data[0];
    float wr_desired = data[1];

    tcp_comm.setDesiredAngularVelocityLeft(wl_desired); 
    tcp_comm.setDesiredAngularVelocityRight(wr_desired); 
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "tcp_server");
    ros::NodeHandle nh("~");

    int n_age = -1;
    
    ros::param::get("~n", n_age);
   

    // user input manual.
    string user_manual; 
    stringstream ss;

    string cam_configure_manual;

    // ROS publish  
    // run multi-threads
    std::mutex* mut = new std::mutex();

    TCPCOMM tcp_comm = TCPCOMM(mut);

    std::thread t1 = tcp_comm.runThread();

    ros::Subscriber sub_wd = nh.subscribe<std_msgs::Float32MultiArray>("/w_desired",1,boost::bind(callbackDesiredAngularVelocities,_1,boost::ref(tcp_comm)));
    
    float wl, wr;

    while(ros::ok())
    {
        ros::spinOnce();
        
        tcp_comm.getAngularVelocityLeft(wl); 
        tcp_comm.getAngularVelocityRight(wr); 

        //w_msg.data  = wl;
        //wd_msg.data = w_d;
        //we_msg.data = wr;
//
        //w_pub.publish(w_msg);
        //wd_pub.publish(wd_msg);
        //we_pub.publish(we_msg);
    }

    // delete
    t1.join();
    delete mut;

    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}