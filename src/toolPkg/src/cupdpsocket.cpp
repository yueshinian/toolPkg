#include "cupdpsocket.h"

using namespace std;

CUdpSocket::CUdpSocket(int m_listenport_, int m_remoteport_)
{
    m_listenport = m_listenport_;
    m_remoteport = m_remoteport_;


    sock = -1;
    initialUDP();
}
struct sockaddr_in addrto,clnadr;
bool CUdpSocket::initialUDP()
{
    // 绑定地址
   bzero(&addrto, sizeof(struct sockaddr_in));
   addrto.sin_family = AF_INET;
   addrto.sin_addr.s_addr = htonl(INADDR_ANY); 
   addrto.sin_port = htons(m_listenport);

   bzero(&clnadr, sizeof(struct sockaddr_in));
   clnadr.sin_family = AF_INET;
   clnadr.sin_addr.s_addr = inet_addr("192.168.1.251");//htonl(INADDR_ANY);
   clnadr.sin_port = htons(m_remoteport);

   //创建套接字
   if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
   {
       cout<<"socket error"<<endl;
       return false;
   }
    //绑定本机地址，在不绑定的情况下对方是不知道本机ip和端口号的，不绑定需要先接收消息后获取对方的ip和端口后才能发送
   if(bind(sock,(struct sockaddr *)&(addrto), sizeof(struct sockaddr_in)) == -1)
   {
       cout<<"bind error..."<<endl;
       return false;
   }
   cout<<"初始化成功！"<<endl;
}
struct sockaddr_in from;
void CUdpSocket::OnReceive()
{
    memset(recvCharArray,0,sizeof(recvCharArray));
    int len;
    // printf("aaa\n");
    recvLen=recvfrom(sock, recvCharArray, sizeof(recvCharArray), 0/*MSG_DONTWAIT*/, (struct sockaddr*)&from,(socklen_t*)&len);
    // printf("%d\n",recvLen);
    if(recvLen > 0)
    {
        isRecvData = true;
    }
    else
    {
        isRecvData = false;
    }
}

void CUdpSocket::SendData(unsigned char *buf ,unsigned char len ,char* raddr)
{
    int ret = sendto(sock,(char*)buf,len,0,(struct sockaddr*)&clnadr,sizeof(sockaddr_in));
    if(ret <0)
    {
       printf("send error!");
    }
}
