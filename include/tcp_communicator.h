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

typedef union FLOAT_UNION_{
    float float_;
    char bytes_[4];   
} FLOAT_UNION;

class TCPCOMM{
public:
    TCPCOMM(mutex* m) 
    : m_(m)
    {
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

    void runThread(){

        while(1){
            // A request is received.
            client_socket_ = accept(server_socket_, (struct sockaddr*)&client_addr_, (socklen_t*)&client_addr__size);        //client_addr__size = sizeof(client_addr_); // IPv4 or IPv6
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
                        printf("tcp rcv:%d / \n", len_read);

                        // 1. Receive TCP/IP data (from MCU)
                        // 1-1) IMU (three acc, three gyro)
                        imu_data[0] = decodeIMUBytes(buff_rcv_[0],  buff_rcv_[1]);
                        imu_data[1] = decodeIMUBytes(buff_rcv_[2],  buff_rcv_[3]);
                        imu_data[2] = decodeIMUBytes(buff_rcv_[4],  buff_rcv_[5]);

                        imu_data[3] = decodeIMUBytes(buff_rcv_[6],  buff_rcv_[7]);
                        imu_data[4] = decodeIMUBytes(buff_rcv_[8],  buff_rcv_[9]);
                        imu_data[5] = decodeIMUBytes(buff_rcv_[10], buff_rcv_[11]);

                        // 1-2) Encoder data (left / right)
                        data_enc_left_  = buff_rcv_[12];
                        data_enc_right_ = buff_rcv_[13];
                        
                        // 1-3) Time (two bytes for second, four bytes for microseconds)
                        time_mcu_ = decodeTime(buff_rcv_[14], buff_rcv_[15], buff_rcv_[16], buff_rcv_[17], buff_rcv_[18], buff_rcv_[19]);

                        // 1-4) State vector (camera trigger, other signals)


                        // difference of time from the current to the previous.
                        dtime = time_curr-time_prev;
                        printf("enc l/r: %d, %d", encoder_left, encoder_right);

                        double w_left = (double)encoder_left * pulses_to_rev / (0.05);
                        printf("/ time: %0.6lf [s] / dt: %0.3lf [ms] / freq: %0.3lf [Hz] ", time_curr, dtime*1000, 1.0/dtime);
                        
                        for(int j = 0; j < 6; ++j) printf("%d ",imu_data[j]);
                        printf("\n");

                        printf("w_d : %0.3lf / w_left: %0.3lf   ", w_desired.float_, w_left);

                        // 2. Send TCP/IP data (to MCU)
                        // sprintf(buff_snd_, "%d : %s", len_read, buff_rcv_);
                        for(int j = 0; j < 4; ++j) buff_snd_[j]    = w_desired.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+4]  = kp.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+8]  = kd.bytes_[j];
                        for(int j = 0; j < 4; ++j) buff_snd_[j+12] = ki.bytes_[j];
                        write(client_socket_, buff_snd_, 16 + 1); // +1 means "NULL". 
                        
                        time_prev = time_curr; 
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


private:
    inline short decodeIMUBytes(char h_byte, char l_byte){
        return (short) ( (unsigned char)h_byte << 8 | (unsigned char)l_byte);
    };

    inline double decodeTime(char sec_l, char sec_h, char usec0, char usec1, char usec2, char usec3){
        unsigned short sec 
            = (unsigned short) ( (unsigned char)sec_h << 8 | (unsigned char)sec_l);
        unsigned int usec  
            = (unsigned int)   ( (unsigned char)usec3 << 24 | (unsigned char)usec2 << 16 | (unsigned char)usec1 << 8 | (unsigned char)usec0 );
        return (double)sec + (double)usec/1000000.0;
    };

private:

    // Socket structs
    int server_socket_;
    int client_socket_;
    int client_addr__size_;

    struct sockaddr_in server_addr_;
    struct sockaddr_in client_addr_;

    // TCP/IP buffers
    char buff_rcv_[BUFF_SIZE+5]; // why add 5?
    char buff_snd_[BUFF_SIZE+5];

    // Data from the MCU
    double time_mcu_; 
    float  data_imu_[6];
    float  data_angle_vel_left_;
    float  data_angle_vel_right_;

    // Data to the MCU

    std::mutex* m_; // pointer to the main mutex

};

#endif