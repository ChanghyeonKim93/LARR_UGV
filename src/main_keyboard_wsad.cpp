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
    ros::init(argc, argv, "keyboard_wsad");
    ros::NodeHandle nh("~");
    
    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    w: + velocity"
    << "\n|    s: -velocity" 
    << "\n|    a: left turn"
    << "\n|    d: right turn" 

    << "\n|    a: left turn"
    << "\n|    d: right turn" 
    << "\n| Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    // ROS publish
    ros::Publisher wd_pub = nh.advertise<std_msgs::Float32MultiArray>("/larr_ugv/wheel_desired", 1);  
    ros::Publisher wr_pub = nh.advertise<std_msgs::Float32>("/larr_ugv/wrd", 1);  
    ros::Publisher wl_pub = nh.advertise<std_msgs::Float32>("/larr_ugv/wld", 1);  
    
    float R = 0.1; // wheel radius
    float B = 0.4; // displacement between two wheel centres
    float invr = 1.0/R;
    float Bhalfinvr = B/(2.0*R);

    float v_d = 0.0f;
    float w_d = 0.0f;
    
    float v_step = 0.0025/1.5;
    float w_step = 0.007/1.5;

    float V_MAX = 0.15;
    float V_MIN = -0.15; // maximum vel. [m/s]
    float W_MAX = 0.16; // maximum rotation rate [rad/s]
    float W_MIN = -0.16;

    // wheel rotational velocity
    float wl_d = 0.0f;
    float wr_d = 0.0f;
    float THRES_W = 1.5;

    ros::Rate r(40);
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function.
        if(c == 'w')
	    {
            v_d += v_step;
            if(v_d >= V_MAX) v_d = V_MAX;
            cout << user_manual;         
        }
        else if(c == 's')
	    {           
            v_d -= v_step;
            if(v_d <= V_MIN) v_d = V_MIN;
            cout << user_manual;         
        }
        else if(c == 'd')
	    {
            w_d += w_step;
            if(w_d >= W_MAX) w_d = W_MAX;
            cout << user_manual;         
        }
        else if(c == 'a')
	    {           
            w_d -= w_step;
            if(w_d <= W_MIN) w_d = W_MIN;
            cout << user_manual;         
        }

        else if(c == 'q')
	    {        
            v_d += v_step;   
            w_d -= w_step;
            if(v_d >= V_MAX) v_d = V_MAX;
            if(w_d <= W_MIN) w_d = W_MIN;
            cout << user_manual;         
        }
        else if(c == 'e')
	    {        
            v_d += v_step;   
            w_d += w_step;
            if(v_d >= V_MAX) v_d = V_MAX;
            if(w_d >= W_MAX) w_d = W_MAX;
            cout << user_manual;         
        }
        else if(c == 'z')
	    {        
            v_d -= v_step;   
            w_d += w_step;
            if(v_d <= V_MIN) v_d = V_MIN;
            if(w_d >= W_MAX) w_d = W_MAX;
            cout << user_manual;         
        }
        else if(c == 'c')
	    {        
            v_d -= v_step;   
            w_d -= w_step;
            if(v_d <= V_MIN) v_d = V_MIN;
            if(w_d <= W_MIN) w_d = W_MIN;

            cout << user_manual;         
        }
        else {
            // deccelerate
            float v_abs = fabs(v_d);
            v_abs -= 0.7*v_step;
            v_d = v_abs*((v_d > 0) - (v_d < 0));

            float w_abs = fabs(w_d);
            w_abs -= 0.7*w_step;
            w_d = w_abs*((w_d > 0) - (w_d < 0));

            if(fabs(v_d) < 1.5*v_step) v_d = 0;
            if(fabs(w_d) < 1.5*w_step) w_d = 0;
        } 

        wl_d = invr*v_d - Bhalfinvr*w_d;
        wr_d = invr*v_d + Bhalfinvr*w_d;
        if(wl_d >  THRES_W) wl_d =  THRES_W;
        if(wl_d < -THRES_W) wl_d = -THRES_W;
        if(wr_d >  THRES_W) wr_d =  THRES_W;
        if(wr_d < -THRES_W) wr_d = -THRES_W;

        //cout << "wl wr : " << wl_d << ", " << wr_d << endl;

        std_msgs::Float32MultiArray wd_msg;    
        wd_msg.data.push_back(wl_d);
        wd_msg.data.push_back(wr_d);

        wd_pub.publish(wd_msg);

        std_msgs::Float32 wl_d_msg;
        std_msgs::Float32 wr_d_msg;
        wl_d_msg.data = wl_d;
        wr_d_msg.data = wr_d;
        wl_pub.publish(wl_d_msg);
        wr_pub.publish(wr_d_msg);

        ros::spinOnce();
        r.sleep();
    }

    return -1;
}
