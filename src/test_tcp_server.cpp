#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"

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

    int cnt = 0;
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function.
        ros::spinOnce();

        if(c == 'w')
	{

            cout << user_manual;         
        }
        else if(c == 's')
	{           

            cout << user_manual;         
        }
    }

    // delete


    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
