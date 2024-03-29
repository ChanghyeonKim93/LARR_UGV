#ifndef _TCP_COMMUNICATOR_H_
#define _TCP_COMMUNICATOR_H_

#include <iostream>
#include <time.h>
#include <string>
#include <cstring>
#include <sstream>

// For socket
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // UNIX based applications
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <fcntl.h> // for non-blocking socket.

// epoll
#include <sys/epoll.h>

// mutex
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include "datatype.h"


using namespace std;

// TCP/IP buffer size
#define BUFF_SIZE   1024
#define PORT_NUMBER 1312

#define STATUS_IMU      0b00000001
#define STATUS_CAMERA   0b00000010
#define STATUS_CONTROL  0b00000100
#define STATUS_ENCODER  0b00001000

#define GEAR_RATIO 49      // TO BE sent from the main node.
#define PULSES_PER_REV 38  // TO BE sent from the main node.

#include <signal.h>
// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

class TCPCOMM{
public:
    TCPCOMM(mutex* m) 
    : m_(m)
    {
        openSocket();
    };

    TCPCOMM(mutex* m, ros::NodeHandle nh) 
    : m_(m), nh_(nh)
    {
        openSocket();
        
        // ROS
        msg_wheel_encoders_.data.push_back(0);
        msg_wheel_encoders_.data.push_back(0);
        msgs_sonars_.push_back(sensor_msgs::Range());
        msgs_sonars_.push_back(sensor_msgs::Range());
        msgs_sonars_.push_back(sensor_msgs::Range());
        this->pub_wheel_encoders_ = nh_.advertise<std_msgs::Float32MultiArray>("/larr_ugv/wheel_current", 1);
        this->pub_imu_            = nh_.advertise<sensor_msgs::Imu>("/larr_ugv/imu", 1);
        this->pubs_sonars_.push_back(nh_.advertise<sensor_msgs::Range>("/larr_ugv/sonar0", 1));
        this->pubs_sonars_.push_back(nh_.advertise<sensor_msgs::Range>("/larr_ugv/sonar1", 1));
        this->pubs_sonars_.push_back(nh_.advertise<sensor_msgs::Range>("/larr_ugv/sonar2", 1));

    };

    std::thread runThread() {
        return std::thread([=] { process(); });
    };
    std::thread runThreadROS() {
        return std::thread([=] { processROS(); });
    };

    void process(){
        datatype::FLOAT_UNION kp,kd,ki;
        w_left_desired_.float_  = 0.0f;
        w_right_desired_.float_ = 0.0f;
        kp.float_ = 1.1;//1.5;
        kd.float_ = 0.07;//10.5;
        ki.float_ = 0.0;//0.8;
        // 20210723 p d i: 1.3, 0.1, 0.0 for 20 Hz control loop.
        // 20210723 p d i: 0.9 0.07 0.0 for 100 Hz control loop.

        while(1){
            // A request is received.
            client_socket_ = accept(server_socket_, (struct sockaddr*)&client_addr_, (socklen_t*)&client_addr_size_); //client_addr__size = sizeof(client_addr_); // IPv4 or IPv6
            if ( -1 == client_socket_) continue;
            printf("  - client_socket: %d\n", client_socket_);

            char client_ip[INET_ADDRSTRLEN+1];
            printf("  - Requsting client IP:  [%s]\n", inet_ntop(AF_INET, (const void*)&client_addr_.sin_addr, client_ip, INET_ADDRSTRLEN+1));

            if(-1 == client_socket_) printf("  - connection is not accepted.\n");
            else 
            {
                printf("  - Connection request of [%s] is accepted!\n", client_ip);
                while(1){ // While loop for a client...
                    // a) Receiving part
                    int len_read = read(client_socket_, buff_rcv_, BUFF_SIZE);
                    if(len_read > 0){ // if there is data,
                        // 1. Receive TCP/IP data (from MCU)
                        // 1-1) IMU (three acc, three gyro)
                        //m_->lock();
                        this->decodeIMUData(buff_rcv_);
                        //m_->unlock();

                        // 1-2) Encoder data (left / right)for(int j = 0; j < 4; ++j) w_left.bytes_[j] = buff_rcv[12+j];
                        for(int j = 0; j < 4; ++j) w_left_.bytes_[j]  = buff_rcv_[12+j];
                        for(int j = 0; j < 4; ++j) w_right_.bytes_[j] = buff_rcv_[16+j];
                        
                        // 1-3) Time (two bytes for second, four bytes for microseconds)
                        this->decodeTime(buff_rcv_[20], buff_rcv_[21], buff_rcv_[22], buff_rcv_[23], buff_rcv_[24], buff_rcv_[25]);

                        // difference of time from the current to the previous.
                        double dtime = time_mcu_-time_mcu_prev_;
                        printf("/ time: %0.6lf [s] / dt: %0.3lf [ms] / freq: %0.3lf [Hz] ", time_mcu_, dtime*1000, 1.0/dtime);
                        
                        // 1-4) State vector (camera trigger, other signals)

                        // 1-5) Sonar distances
                        this->decodeSonarData(buff_rcv_);

                        // Visualize
                        for(int j = 0; j < 6; ++j) printf("%d ", data_imu_[j]);

                        printf(" / wheel: ");
                        printf("wl_d : %0.3lf, wr_d : %0.3lf / wl: %0.3lf , wr: %0.3lf   ", w_left_desired_.float_, w_right_desired_.float_, w_left_.float_, w_right_.float_);
                        
                        printf(" / sonar: " );
                        for(int j = 0; j < 3; ++j) printf("%d ", sonar_dist[j]);

                        printf("\n");

                        // 2. Send TCP/IP data (to MCU)
                        // sprintf(buff_snd_, "%d : %s", len_read, buff_rcv_);
                        for(int j = 0; j < 4; ++j) buff_snd_[j]    = w_left_desired_.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+4]  = w_right_desired_.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+8]  = kp.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+12] = kd.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+16] = ki.bytes_[j];


                        write(client_socket_, buff_snd_, 20 + 1); // +1 means "NULL". 
                        
                        time_mcu_prev_ = time_mcu_; 
                    } 
                    else if(len_read == 0) { // EOF (0) means connection is end.
                        printf("  - EOF is detected.\n");
                        break;
                    }
                    // b) Sending part (from the sending queue)
                }

                // Terminate connection.
                printf("   Connection between [%s] is terminated. The socket is closed.\n",client_ip);
                close(client_socket_);            
            }
        } // end WHILE
    };

    void processROS(){
        datatype::FLOAT_UNION kp,kd,ki;
        w_left_desired_.float_  = 0.0f;
        w_right_desired_.float_ = 0.0f;
        kp.float_ = 1.1;//1.5;
        kd.float_ = 0.07;//10.5;
        ki.float_ = 0.0;//0.8;
        // 20210723 p d i: 1.3, 0.1, 0.0 for 20 Hz control loop.
        // 20210723 p d i: 0.9 0.07 0.0 for 100 Hz control loop.

        while(1){
            // A request is received.
            client_socket_ = accept(server_socket_, (struct sockaddr*)&client_addr_, (socklen_t*)&client_addr_size_); //client_addr__size = sizeof(client_addr_); // IPv4 or IPv6
            if ( -1 == client_socket_) continue;
            printf("  - client_socket: %d\n", client_socket_);

            char client_ip[INET_ADDRSTRLEN+1];
            printf("  - Requsting client IP:  [%s]\n", inet_ntop(AF_INET, (const void*)&client_addr_.sin_addr, client_ip, INET_ADDRSTRLEN+1));

            if(-1 == client_socket_) printf("  - connection is not accepted.\n");
            else 
            {
                printf("  - Connection request of [%s] is accepted!\n", client_ip);
                while(1){ // While loop for a client...
                    // a) Receiving part
                    int len_read = read(client_socket_, buff_rcv_, BUFF_SIZE);
                    if(len_read > 0){ // if there is data,
                        // 1. Receive TCP/IP data (from MCU)
                        // 1-1) IMU (three acc, three gyro)
                        //m_->lock();
                        this->decodeIMUData(buff_rcv_);
                        //m_->unlock();

                        // 1-2) Encoder data (left / right)for(int j = 0; j < 4; ++j) w_left.bytes_[j] = buff_rcv[12+j];
                        for(int j = 0; j < 4; ++j) w_left_.bytes_[j]  = buff_rcv_[12+j];
                        for(int j = 0; j < 4; ++j) w_right_.bytes_[j] = buff_rcv_[16+j];
                        
                        // 1-3) Time (two bytes for second, four bytes for microseconds)
                        this->decodeTime(buff_rcv_[20], buff_rcv_[21], buff_rcv_[22], buff_rcv_[23], buff_rcv_[24], buff_rcv_[25]);

                        // difference of time from the current to the previous.
                        double dtime = time_mcu_-time_mcu_prev_;
                        
                        // 1-4) State vector (camera trigger, other signals)

                        // 1-5) Sonar distances
                        this->decodeSonarData(buff_rcv_);


                        // publishers
                        msg_wheel_encoders_.data[0] = w_left_.float_;
                        msg_wheel_encoders_.data[1] = w_right_.float_;

                        msg_imu_.angular_velocity.x    = data_imu_[0];
                        msg_imu_.angular_velocity.y    = data_imu_[1];
                        msg_imu_.angular_velocity.z    = data_imu_[2];
                        msg_imu_.linear_acceleration.x = data_imu_[3];
                        msg_imu_.linear_acceleration.y = data_imu_[4];
                        msg_imu_.linear_acceleration.z = data_imu_[5];

                        for(int j = 0; j < 3; ++j) {
                            msgs_sonars_[j].range = (float)sonar_dist[j]/100.0;
                            msgs_sonars_[j].min_range = 0.01;
                            msgs_sonars_[j].max_range = 0.60;
                            msgs_sonars_[j].radiation_type = msgs_sonars_[j].ULTRASOUND;
                            msgs_sonars_[j].field_of_view = 15.0*3.141592/180.0;
                        }

                        for(int j = 0; j < 3; ++j)
                            pubs_sonars_[j].publish(msgs_sonars_[j]);

                        pub_wheel_encoders_.publish(msg_wheel_encoders_);
                        pub_imu_.publish(msg_imu_);

                        // 2. Send TCP/IP data (to MCU)
                        // sprintf(buff_snd_, "%d : %s", len_read, buff_rcv_);
                        for(int j = 0; j < 4; ++j) buff_snd_[j]    = w_left_desired_.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+4]  = w_right_desired_.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+8]  = kp.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+12] = kd.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+16] = ki.bytes_[j];


                        write(client_socket_, buff_snd_, 20 + 1); // +1 means "NULL". 
                        
                        time_mcu_prev_ = time_mcu_; 
                    } 
                    else if(len_read == 0) { // EOF (0) means connection is end.
                        printf("  - EOF is detected.\n");
                        break;
                    }
                    // b) Sending part (from the sending queue)
                }

                // Terminate connection.
                printf("   Connection between [%s] is terminated. The socket is closed.\n",client_ip);
                close(client_socket_);            
            }
        } // end WHILE
    };


    void setDesiredAngularVelocityLeft(float w_d) {
        m_->lock();
        w_left_desired_.float_ = w_d;
        m_->unlock();
    };
    void setDesiredAngularVelocityRight(float w_d) {
        m_->lock();
        w_right_desired_.float_ = w_d;
        m_->unlock();
    };

    void setDesiredAngularVelocities(float wl_d,float wr_d) {
        m_->lock();
        w_left_desired_.float_ = wl_d;
        w_right_desired_.float_ = wr_d;
        m_->unlock();
    };

    void getAngularVelocityLeft(float& wl){
        m_->lock();
        wl = w_left_.float_;
        m_->unlock();
    };

    void getAngularVelocityRight(float& wr){
        m_->lock();
        wr = w_right_.float_;
        m_->unlock();
    };

    void getAngularVelocities(float& wl, float& wr){
        m_->lock();
        wl = w_left_.float_;
        wr = w_right_.float_;
        m_->unlock();
    };


private:
    inline short decode2BytesToShort(char h_byte, char l_byte){
        return (short) ( (unsigned char)h_byte << 8 | (unsigned char)l_byte);
    };

    void decodeTime(char sec_l, char sec_h, char usec0, char usec1, char usec2, char usec3){
        unsigned short sec 
            = (unsigned short) ( (unsigned char)sec_h << 8 | (unsigned char)sec_l);
        unsigned int usec  
            = (unsigned int)   ( (unsigned char)usec3 << 24 | (unsigned char)usec2 << 16 | (unsigned char)usec1 << 8 | (unsigned char)usec0 );
        time_mcu_ = (double)sec + (double)usec/1000000.0;
    };

    inline void decodeIMUData(char* buff){
        data_imu_[0] = decode2BytesToShort(buff[0],  buff[1]);
        data_imu_[1] = decode2BytesToShort(buff[2],  buff[3]);
        data_imu_[2] = decode2BytesToShort(buff[4],  buff[5]);

        data_imu_[3] = decode2BytesToShort(buff[6],  buff[7]);
        data_imu_[4] = decode2BytesToShort(buff[8],  buff[9]);
        data_imu_[5] = decode2BytesToShort(buff[10], buff[11]);
    };

    inline void decodeSonarData(char* buff){
        sonar_dist[0] = decode2BytesToShort(buff[27],buff[28])/256;
        sonar_dist[1] = decode2BytesToShort(buff[29],buff[30])/256;
        sonar_dist[2] = decode2BytesToShort(buff[31],buff[32])/256;
    };

    void openSocket(){
        signal(SIGINT, signal_callback_handler);

        // Initialize socket.
        server_socket_ = socket(PF_INET, SOCK_STREAM, 0);  // PF_INET: IPv4, SOCK_STREAM: TCP/IP. //M
        int socket_option = 1; // SO_REUSEADDR == true. TIME-WAIT refusal.
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &socket_option, sizeof(socket_option));

        if( -1 == server_socket_){
            cout << "Generation of a server socket fails...\n";
            exit(1);
        } else cout << "Server socket is generated!\n";
    
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family      = AF_INET;
        server_addr_.sin_port        = htons(PORT_NUMBER); // port number . 
        server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        
        if(-1 == bind(server_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_))){
            printf("   ERROR: Fail to bind!\n");
            exit(1);
        } else printf("BIND:   OK\n");

        if(-1 == listen(server_socket_, 5)){ // What is the meaning of '5' ? 
            // listen is failed. no request.
            printf("   ERROR: Fail to listen!\n");
            exit(1);
        } 
        else printf("LISTEN: OK\n");
        
        // Set non-block server
        int flag = fcntl(server_socket_, F_GETFL, 0);
        fcntl(server_socket_, F_SETFL, flag | O_NONBLOCK);
    };


private:

    // Socket structs
    int server_socket_;
    int client_socket_;
    int client_addr_size_;

    struct sockaddr_in server_addr_;
    struct sockaddr_in client_addr_;

    // TCP/IP buffers
    char buff_rcv_[BUFF_SIZE+5]; // why add 5?
    char buff_snd_[BUFF_SIZE+5];

    // Data from the MCU
    double time_mcu_; 
    double time_mcu_prev_; 
    short  data_imu_[6];

    unsigned short sonar_dist[3];

    datatype::FLOAT_UNION w_left_;
    datatype::FLOAT_UNION w_right_;

    datatype::FLOAT_UNION w_left_desired_;
    datatype::FLOAT_UNION w_right_desired_;

    // Data to the MCU

    // Pointer to the main mutex
    std::mutex* m_; 

    // ROS related
    ros::NodeHandle nh_;

    std_msgs::Float32MultiArray msg_wheel_encoders_;
    sensor_msgs::Imu msg_imu_;
    std::vector<sensor_msgs::Range> msgs_sonars_;

    ros::Publisher pub_wheel_encoders_;
    ros::Publisher pub_imu_;
    std::vector<ros::Publisher> pubs_sonars_;
};

#endif
