#ifndef CUPDPSOCKET_H
#define CUPDPSOCKET_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

class CUdpSocket
{
public:
    CUdpSocket();
    void OnReceive();
    bool initialUDP();
    void SendData(unsigned char *buf ,unsigned char len,char* raddr);

public:
    int m_listenport;
    int m_remoteport;
    
    int sock;
    int recvLen;
    char sendBuffer[255];//自己发出的消息
    int sendLength;//发出信息的长度
    char recvCharArray[65536]; //msg length
    bool isRecvData = false;
};

#endif // CUPDPSOCKET_H
