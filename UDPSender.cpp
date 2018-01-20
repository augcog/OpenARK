#include "stdafx.h"
#include "version.h"
#include "UDPSender.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS

namespace ark {
    UDPSender::UDPSender()
    {
        // initialise winsock
        printf("\nInitialising Winsock...");
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        {
            printf("Failed. Error Code : %d", WSAGetLastError());
            exit(EXIT_FAILURE);
        }
        printf("Initialised.\n");

        // create socket
        if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
        {
            printf("socket() failed with error code : %d", WSAGetLastError());
            exit(EXIT_FAILURE);
        }

        // setup address structure
        memset(reinterpret_cast<char *>(&si_other), 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(PORT);
        si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);

    }

    int UDPSender::send(std::string message)
    {
        if (sendto(s, message.c_str(), (int)strlen(message.c_str()), 0, reinterpret_cast<struct sockaddr *>(&si_other), slen) == SOCKET_ERROR)
        {
            printf("sendto() failed with error code : %d", WSAGetLastError());
            exit(EXIT_FAILURE);
        }
        return 0;
    }

    int UDPSender::close() const
    {
        closesocket(s);
        WSACleanup();
        return 0;
    }
}
