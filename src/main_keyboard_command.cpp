#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"
#include "tools.h"

// ROS
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_command");
    ros::NodeHandle nh("~");
    
    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    w: (left) motor speed up   (+0.025 rad/s)"
    << "\n|    s: (left) motor speed down (-0.025 rad/s)" 
    << "\n|    e: (right) motor speed up   (+0.025 rad/s)"
    << "\n|    d: (right) motor speed down (-0.025 rad/s)" 
    << "\n| Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    // ROS publish
    ros::Publisher wd_pub = nh.advertise<std_msgs::Float32MultiArray>("/w_desired", 1);  
  
    float wl_d = 0.0f;
    float wr_d = 0.0f;
    float wl, wr;
    float THRES_W = 2.0;
    
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function.        
        if(c == 'w')
	    {
            wl_d += 0.025;
            if(wl_d > THRES_W) wl_d = THRES_W;
            cout << user_manual;         
        }
        else if(c == 's')
	    {           
            wl_d -= 0.025;
            if(wl_d < -THRES_W) wl_d = -THRES_W;
            cout << user_manual;         
        }
        else if(c == 'e')
	    {
            wr_d += 0.025;
            if(wr_d > THRES_W) wr_d = THRES_W;
            cout << user_manual;         
        }
        else if(c == 'd')
	    {           
            wr_d -= 0.025;
            if(wr_d < -THRES_W) wr_d = -THRES_W;
            cout << user_manual;         
        }
        else if(c == 'q') break;
        else continue;

        std_msgs::Float32MultiArray wd_msg;    
        wd_msg.data.push_back(wl_d);
        wd_msg.data.push_back(wr_d);

        wd_pub.publish(wd_msg);

        ros::spinOnce();
    }

    return -1;
}
