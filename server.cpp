#include "server.h"

Server::Server(int port) {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(port);
    if ( bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("bind");
        exit(-1);
    }
    if ( listen(sockfd, 5) == -1) {
        perror("listen");
        exit(-1);
    }
}

void Server::acceptConnections() {
    while (true) {
        socklen_t socksize = sizeof(clientAddr);
        newsockfd = accept(sockfd, (struct sockaddr*)&clientAddr, &socksize);
        string str = inet_ntoa(clientAddr.sin_addr);
        pthread_create(&serverThread, NULL, &Task, (void*) newsockfd);
    }
}

void Server::sendMessage(string msg) {
    send(newsockfd, msg.c_str(), msg.length(), 0);
}

void Server::closeConnection() {
    close(sockfd);
    close(newsockfd);
}

void* Server::Task(void *arg) {
    int n;
    int newsockfd = (long) arg;
    char msg[1024];
    pthread_detach(pthread_self());
    while (true) {
        n = recv(newsockfd, msg, 1024, 0);
        if (n == 0) {
            close (newsockfd);
            break;
        }
        msg[n] = 0;
        message = string(msg);
    }
    return 0;
}
