#ifndef TCPACCEPTOR_H
#define TCPACCEPTOR_H

#include <string>
#include <string.h>
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "tcpstream.h"

class TCPAcceptor
{
    int m_lsd;
    string m_address;
    int m_port;
    bool m_listening;

public:
    TCPAcceptor(int port, const char* address="");
    ~TCPAcceptor();
    int start();
    TCPStream* accept();

private:
    TCPAcceptor() {}
};

#endif // TCPACCEPTOR_H
