#ifndef SERVER_H
#define SERVER_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

#define PORT 7777

using namespace std;

class Server {
public:
    int sockfd;
    int newsockfd;
    int n;
    int pid;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
    pthread_t serverThread;
    char msg[1024];
    static string message;

    Server(int port);
    void acceptConnections();
    void sendMessage(string msg);
    void closeConnection();

private:
    static void *Task(void *arg);
};

#endif // SERVER_H
