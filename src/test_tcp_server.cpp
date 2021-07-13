#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

#include <mutex>
#include <thread>

// keyboard input tool
#include "keyinput.h"
#include "tcp_communicator.h"

// ROS
#include <std_msgs/Float32.h>

// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string currentDateTime(){
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about data/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

    return buf;
}

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_tcp_server");
    ros::NodeHandle nh("~");

    int n_age   = -1;
    
    ros::param::get("~n", n_age);
   

    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    w: motor speed up   (+0.1 rad/s)"
    << "\n|    s: motor speed down (-0.1 rad/s)" 
    << "\n| Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    // ROS publish
    std_msgs::Float32 wd_msg;
    std_msgs::Float32 w_msg;
    ros::Publisher wd_pub = nh.advertise<std_msgs::Float32>("w_desired", 1);
    ros::Publisher w_pub  = nh.advertise<std_msgs::Float32>("w", 1);
  
  
    // run multi-threads
    std::mutex* mut = new std::mutex();

    TCPCOMM* tcp_comm = new TCPCOMM(mut);



    std::thread t1 = tcp_comm->runThread();
    //std::thread t2 = velocity_estimator.runThread(&communicator);

    int cnt = 0;
    float w_d = 0.0f;
    float wl, wr;
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function.
        ros::spinOnce();
        
        if(c == 'w')
	    {
            w_d += 0.05;
            if(w_d > 2.4) w_d = 2.4;
            tcp_comm->setWLeftDesired(w_d); 
            cout << user_manual;         
        }
        else if(c == 's')
	    {           
            w_d -= 0.05;
            if(w_d < -2.4) w_d = -2.4;
            tcp_comm->setWLeftDesired(w_d); 
            cout << user_manual;         
        }
        else if(c == 'q') break;

        tcp_comm->getAngularVelocities(wl, wr);

        w_msg.data  = wl;
        wd_msg.data = w_d;
        
        w_pub.publish(w_msg);
        wd_pub.publish(wd_msg);
    }

    // delete
    t1.join();
    delete tcp_comm;
    delete mut;

    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
